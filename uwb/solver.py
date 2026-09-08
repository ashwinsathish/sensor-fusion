#!/usr/bin/env python3
"""UWB positioning that keeps its time and reports its own uncertainty.

The backend in uwb-visualization currently receives
``(frame_nr, anchor_id, tof, timestamp)`` per ranging measurement, solves a
3D least-squares fix, and emits ``{x, y, FrameNr, type}``. Three things worth
having are computed and then thrown away:

* **when** the ranging round happened,
* **how well** the solve fitted (the residual),
* **how many anchors** contributed and in what geometry.

The first is needed to fuse UWB with anything else. The last two ARE the
per-measurement covariance — so the "weight each sensor by its accuracy"
question has an analytic answer instead of a tuned constant.

Three substantive changes to the maths, beyond bookkeeping:

**Fix z.** All nine LIT anchors sit at ceiling height (3.09-5.05 m) in two rows
12 m apart, and the tag rides at a known constant height on the AGV. Solving
for z anyway costs a parameter against a nearly coplanar anchor set.

How much this matters was measured, not assumed, and the answer is
anchor-count dependent (test_solver.py, 10 cm range noise):

    anchors    free z (current)    fixed z     gain
       4            99.3 cm        10.8 cm    9.2x
       5            44.6 cm         9.0 cm    5.0x
       6            12.6 cm         7.4 cm    1.7x
       9             6.3 cm         5.8 cm    1.1x

So with every anchor reporting it is nearly irrelevant — my first guess that
it would be a large win outright was wrong. It is a rescue for DEGRADED
rounds, which is exactly what blockage by racks, the crane and the AGV's own
body produces. Those are the rounds that currently emit silent garbage.

**Gate on residual.** A fix that fits badly is usually one NLOS anchor ruining
an otherwise good round. Report the residual, optionally drop the worst anchor
and re-solve, and refuse to emit a fix that still does not fit.

**Recover the round time from the frame counter.** Arrival timestamps are
jittery — they include serial/ROS transport and Python scheduling. Ranging
rounds are periodic and frame-numbered, so regressing arrival time on frame
number recovers the true round instant far more precisely than any single
arrival timestamp. This is a software PLL over the frame counter.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

SPEED_OF_LIGHT = 299702547.0
DWT_TIME_UNITS = 1.0 / 499.2e6 / 128.0


# ── frame-number clock recovery ──────────────────────────────────────────────

class FrameClock:
    """Maps a ranging frame number to the instant that round actually happened.

    Fits ``t = a * frame_nr + b`` over a sliding window. The slope is the
    ranging period; the intercept absorbs a constant pipeline delay. Outliers
    (a packet that queued behind something) are rejected by median-absolute-
    deviation before the fit, so one late arrival cannot drag the timeline.
    """

    def __init__(self, window: int = 200, min_samples: int = 12):
        self.window = window
        self.min_samples = min_samples
        self._pts: deque[tuple[int, float]] = deque(maxlen=window)
        self.period_s: float | None = None
        self.residual_ms: float | None = None
        self._fit: tuple[float, float] | None = None

    def add(self, frame_nr: int, t_arrival: float) -> None:
        self._pts.append((int(frame_nr), float(t_arrival)))
        self._refit()

    def _refit(self) -> None:
        if len(self._pts) < self.min_samples:
            return
        f = np.array([p[0] for p in self._pts], dtype=float)
        t = np.array([p[1] for p in self._pts], dtype=float)

        # Frame counters wrap (16-bit on these radios). Unwrap before fitting.
        f = _unwrap_counter(f)

        a, b = np.polyfit(f, t, 1)
        resid = t - (a * f + b)
        mad = np.median(np.abs(resid - np.median(resid)))
        if mad > 0:
            keep = np.abs(resid - np.median(resid)) < 5.0 * mad
            if keep.sum() >= self.min_samples:
                a, b = np.polyfit(f[keep], t[keep], 1)
                resid = t[keep] - (a * f[keep] + b)
        self._fit = (float(a), float(b))
        self.period_s = float(a)
        self.residual_ms = float(np.std(resid) * 1000)

    def time_of(self, frame_nr: int) -> float | None:
        """Instant of that ranging round, or None until the fit converges."""
        if self._fit is None:
            return None
        a, b = self._fit
        f = _unwrap_counter(
            np.array([p[0] for p in self._pts] + [int(frame_nr)], dtype=float))[-1]
        return a * f + b


def _unwrap_counter(f: np.ndarray) -> np.ndarray:
    """Undo wrapping of a monotonically increasing integer counter."""
    out = f.astype(float).copy()
    if len(out) < 2:
        return out
    span = float(np.max(out) - np.min(out))
    if span < 1.0:
        return out
    # Detect the modulus from the largest backwards jump.
    d = np.diff(out)
    big_drop = d.min()
    if big_drop >= 0:
        return out
    modulus = 65536.0 if -big_drop > 32768.0 else 2.0 ** np.ceil(np.log2(-big_drop * 2))
    bump = 0.0
    for i in range(1, len(out)):
        if out[i] + bump < out[i - 1] - modulus / 2:
            bump += modulus
        out[i] += bump
    return out


# ── the solve ────────────────────────────────────────────────────────────────

@dataclass
class Fix:
    x: float
    y: float
    z: float
    n_anchors: int
    residual_m: float                 # RMS range residual
    cov: np.ndarray                   # 2x2 position covariance, m^2
    hdop: float                       # horizontal dilution of precision
    anchors_used: list = field(default_factory=list)
    anchors_rejected: list = field(default_factory=list)
    t_round_s: float | None = None    # when the ranging round happened
    t_solved_s: float | None = None   # when this solve finished
    ok: bool = True
    reason: str = ""

    @property
    def sigma_m(self) -> float:
        """1-sigma position uncertainty — what fusion should weight by."""
        return float(np.sqrt(np.trace(self.cov)))

    def as_dict(self) -> dict:
        return {
            "x": self.x, "y": self.y, "z": self.z,
            "type": "UWB",
            "t_round_s": self.t_round_s,
            "t_solved_s": self.t_solved_s,
            "n_anchors": self.n_anchors,
            "residual_m": self.residual_m,
            "sigma_m": self.sigma_m,
            "hdop": self.hdop,
            "cov_xx": float(self.cov[0, 0]), "cov_xy": float(self.cov[0, 1]),
            "cov_yy": float(self.cov[1, 1]),
            "anchors_used": list(self.anchors_used),
            "anchors_rejected": list(self.anchors_rejected),
            "ok": self.ok, "reason": self.reason,
        }


def tof_to_range(tof) -> np.ndarray:
    return np.asarray(tof, dtype=float) * SPEED_OF_LIGHT * DWT_TIME_UNITS


def solve_2d(anchors: np.ndarray, ranges: np.ndarray, z_tag: float,
             *, iterations: int = 60, tol: float = 1e-7,
             p0: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Gauss-Newton for (x, y) with z fixed. Returns (position_xy, jacobian)."""
    anchors = np.asarray(anchors, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    p = anchors[:, :2].mean(axis=0) if p0 is None else np.asarray(p0, float).copy()

    J = np.zeros((len(anchors), 2))
    for _ in range(iterations):
        diff = np.column_stack([p[0] - anchors[:, 0],
                                p[1] - anchors[:, 1],
                                np.full(len(anchors), z_tag) - anchors[:, 2]])
        d = np.linalg.norm(diff, axis=1)
        d = np.maximum(d, 1e-9)
        r = d - ranges
        J = diff[:, :2] / d[:, None]
        step, *_ = np.linalg.lstsq(J, r, rcond=None)
        p = p - step
        if np.linalg.norm(step) < tol:
            break
    return p, J


def fix_from_ranges(anchor_positions, ranges, *, z_tag: float,
                    min_anchors: int = 3, max_residual_m: float = 1.0,
                    reject_worst: bool = True, range_sigma_m: float = 0.10,
                    anchor_ids=None, t_round_s: float | None = None,
                    p0=None) -> Fix:
    """One position fix, with covariance, gating and NLOS rejection.

    `range_sigma_m` is the assumed per-range noise floor. The reported
    covariance takes the larger of that and what the residuals imply, so a
    geometrically perfect but badly-fitting round is not reported as precise.
    """
    A = np.asarray(anchor_positions, dtype=float)
    R = np.asarray(ranges, dtype=float)
    ids = list(anchor_ids) if anchor_ids is not None else list(range(len(R)))
    rejected: list = []

    def _bad(reason: str) -> Fix:
        return Fix(np.nan, np.nan, z_tag, len(R), np.nan,
                   np.full((2, 2), np.nan), np.nan, ids, rejected,
                   t_round_s, time.time(), ok=False, reason=reason)

    if len(R) < min_anchors:
        return _bad(f"only {len(R)} anchors, need {min_anchors}")
    if not np.all(np.isfinite(R)) or np.any(R <= 0):
        return _bad("non-finite or non-positive range")

    p, J = solve_2d(A, R, z_tag, p0=p0)
    resid = _residuals(A, R, p, z_tag)
    rms = float(np.sqrt(np.mean(resid ** 2)))

    # One NLOS anchor inflates the residual far more than it biases the fix —
    # drop it and re-solve, but only if enough anchors remain to stay solvable.
    if reject_worst and rms > max_residual_m and len(R) > min_anchors:
        worst = int(np.argmax(np.abs(resid)))
        rejected.append(ids[worst])
        keep = np.ones(len(R), bool)
        keep[worst] = False
        A, R = A[keep], R[keep]
        ids = [i for k, i in zip(keep, ids) if k]
        p, J = solve_2d(A, R, z_tag, p0=p)
        resid = _residuals(A, R, p, z_tag)
        rms = float(np.sqrt(np.mean(resid ** 2)))

    n = len(R)
    try:
        JTJ_inv = np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        return _bad("degenerate anchor geometry (singular Jacobian)")

    dof = max(n - 2, 1)
    sigma2 = max(float(resid @ resid) / dof, range_sigma_m ** 2)
    cov = sigma2 * JTJ_inv
    hdop = float(np.sqrt(np.trace(JTJ_inv)))

    fix = Fix(float(p[0]), float(p[1]), z_tag, n, rms, cov, hdop,
              ids, rejected, t_round_s, time.time())
    if rms > max_residual_m:
        fix.ok = False
        fix.reason = f"residual {rms:.2f} m exceeds {max_residual_m} m"
    return fix


def _residuals(A, R, p_xy, z_tag) -> np.ndarray:
    p = np.array([p_xy[0], p_xy[1], z_tag])
    return np.linalg.norm(p[None, :] - A, axis=1) - R


# ── accumulator: measurements in, fixes out ──────────────────────────────────

class UwbLocalizer:
    """Groups per-anchor measurements into rounds and emits timed, gated fixes.

    Mirrors the deferral logic of the existing backend — a frame is only solved
    once a packet from a NEWER frame has arrived, which is how you know the
    round is complete — but records that the deferral happened, so the delay it
    adds is measurable instead of assumed.
    """

    def __init__(self, anchor_dict: dict, *, z_tag: float,
                 min_anchors: int = 3, max_residual_m: float = 1.0,
                 range_sigma_m: float = 0.10, maxlen: int = 200):
        self.anchors = {k: np.asarray(v, dtype=float) for k, v in anchor_dict.items()}
        self.z_tag = float(z_tag)
        self.min_anchors = min_anchors
        self.max_residual_m = max_residual_m
        self.range_sigma_m = range_sigma_m
        self.measurements: deque = deque(maxlen=maxlen)
        self.clock = FrameClock()
        self._last_fix: Fix | None = None
        self.n_solved = 0
        self.n_rejected = 0
        self.unknown_anchors: set = set()

    def add(self, frame_nr: int, anchor_id, tof: float,
            t_arrival: float | None = None) -> Fix | None:
        t_arrival = time.time() if t_arrival is None else float(t_arrival)
        self.measurements.append((int(frame_nr), anchor_id, float(tof), t_arrival))
        self.clock.add(frame_nr, t_arrival)
        return self._try_solve()

    def _try_solve(self) -> Fix | None:
        if not self.measurements:
            return None
        newest = max(m[3] for m in self.measurements)
        latest_by_frame: dict = {}
        for fn, _a, _t, ts in self.measurements:
            latest_by_frame[fn] = max(latest_by_frame.get(fn, ts), ts)

        ready = None
        for fn, _a, _t, _ts in self.measurements:
            if latest_by_frame[fn] < newest:      # a newer frame has started
                ready = fn
                break
        if ready is None:
            return None

        rows = [(a, tof, ts) for fn, a, tof, ts in self.measurements if fn == ready]
        cutoff = latest_by_frame[ready]
        self.measurements = deque(
            [m for m in self.measurements if m[3] > cutoff],
            maxlen=self.measurements.maxlen)

        A, R, ids = [], [], []
        for anchor_id, tof, _ts in rows:
            key = self._anchor_key(anchor_id)
            if key is None:
                self.unknown_anchors.add(anchor_id)
                continue
            A.append(self.anchors[key])
            R.append(tof)
            ids.append(key)
        if len(A) < self.min_anchors:
            self.n_rejected += 1
            return None

        # Frame-derived round time beats the raw arrival stamps; fall back to
        # the earliest arrival of the round while the clock fit is warming up.
        t_round = self.clock.time_of(ready)
        if t_round is None:
            t_round = min(ts for _a, _t, ts in rows)

        p0 = None if self._last_fix is None or not self._last_fix.ok else \
            np.array([self._last_fix.x, self._last_fix.y])

        fix = fix_from_ranges(
            np.array(A), tof_to_range(R), z_tag=self.z_tag,
            min_anchors=self.min_anchors, max_residual_m=self.max_residual_m,
            range_sigma_m=self.range_sigma_m, anchor_ids=ids,
            t_round_s=t_round, p0=p0)
        self.n_solved += 1
        if not fix.ok:
            self.n_rejected += 1
        else:
            self._last_fix = fix
        return fix

    def _anchor_key(self, anchor_id):
        if anchor_id in self.anchors:
            return anchor_id
        for cand in (f"0x{int(anchor_id):04X}" if isinstance(anchor_id, (int, np.integer))
                     else None,
                     str(anchor_id)):
            if cand is not None and cand in self.anchors:
                return cand
        return None


def load_anchors(environment_json: str) -> dict:
    import json
    env = json.load(open(environment_json))
    return {a["id"]: np.array([a["position"]["x"], a["position"]["y"],
                               a["position"]["z"]], dtype=float)
            for a in env["anchors"]}
