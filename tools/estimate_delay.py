#!/usr/bin/env python3
"""Joint spatio-temporal calibration of a sensor against the Omron ground truth.

The two unknowns between any sensor (camera fusion output, UWB tag) and the
Omron are entangled:

    frame alignment   x_gt ≈ s·R(θ)·x_sensor + t      (extrinsics)
    time alignment    x_gt(τ) ≈ x_sensor(t)           (latency / clock offset)

Fitting them together from one moving dataset lets one absorb the other: a
constant lag along a straight run looks exactly like a translation. That is why
this tool has two modes.

    static   Estimate ONLY the extrinsics, from segments where the robot is
             standing still. While the robot is stationary, latency has no
             effect, so the frame alignment comes out uncontaminated. This is
             the principled version of "drive to 6 marked spots and calibrate".

    delay    With the extrinsics fixed, estimate ONLY the delay, by sliding the
             sensor track in time against the ground truth. Two independent
             estimators are reported:
               • speed-profile cross-correlation (needs no extrinsics at all —
                 |v| is invariant to rotation and translation)
               • residual minimisation over lag with the extrinsics applied
             If they disagree, something else is wrong (dropped frames, a
             non-constant delay, or the wrong extrinsics).

    joint    Grid-search the delay and re-solve the extrinsics at every lag.
             Use it as a cross-check, never as the primary calibration.

Input CSVs need a time column and two position columns (defaults t,x,y).
Times must be on the SAME clock — run tools/clock_audit.py first, and pass
--sensor-clock-offset if you have to correct one of them.

Usage:
    python3 estimate_delay.py static  gt.csv uwb.csv
    python3 estimate_delay.py delay   gt.csv uwb.csv --extrinsics uwb_extr.json
    python3 estimate_delay.py joint   gt.csv cam.csv --max-lag 2.0 --plot cam.png
"""

from __future__ import annotations

import argparse
import csv
import json
import sys

import numpy as np


# ── io ───────────────────────────────────────────────────────────────────────

def load_track(path, tcol="t", xcol="x", ycol="y", offset=0.0):
    t, x, y = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                ti, xi, yi = float(row[tcol]), float(row[xcol]), float(row[ycol])
            except (KeyError, ValueError, TypeError):
                continue
            if not all(np.isfinite([ti, xi, yi])):
                continue
            t.append(ti + offset)
            x.append(xi)
            y.append(yi)
    if not t:
        raise SystemExit(f"{path}: no usable rows (columns {tcol},{xcol},{ycol})")
    t = np.asarray(t)
    p = np.column_stack([x, y])
    order = np.argsort(t)
    t, p = t[order], p[order]
    keep = np.concatenate([[True], np.diff(t) > 0])       # strictly increasing
    return t[keep], p[keep]


def interp_at(t_src, p_src, t_query):
    return np.column_stack([np.interp(t_query, t_src, p_src[:, k]) for k in (0, 1)])


# ── geometry ─────────────────────────────────────────────────────────────────

def umeyama_2d(src, dst, with_scale=False):
    """Least-squares similarity taking src -> dst. Returns (s, R, t, rmse)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    C = D.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(C)
    W = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:          # no reflections
        W[1, 1] = -1
    R = U @ W @ Vt
    s = (sig @ np.diag(W).ravel()) / (S ** 2).sum(1).mean() if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    res = dst - (s * (R @ src.T).T + t)
    return s, R, t, float(np.sqrt((res ** 2).sum(1).mean()))


def apply_extr(p, s, R, t):
    return s * (R @ p.T).T + t


def yaw_deg(R):
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


# ── segment detection ────────────────────────────────────────────────────────

def find_dwells(t, p, v_thresh=0.05, min_dur=2.0, win=1.0):
    """Contiguous stretches where the ground truth is essentially stationary.

    Thresholds a windowed DISPLACEMENT rather than a sample-to-sample
    derivative — differentiating a noisy 5 Hz track manufactures spurious
    speed and shreds real dwells into fragments.
    """
    slow = np.empty(len(t), bool)
    for i in range(len(t)):
        m = (t >= t[i] - win / 2) & (t <= t[i] + win / 2)
        seg = p[m]
        slow[i] = (seg.max(0) - seg.min(0)).max() < v_thresh * win
    out, i = [], 0
    while i < len(slow):
        if not slow[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(slow) and slow[j + 1]:
            j += 1
        if t[j] - t[i] >= min_dur:
            out.append((t[i], t[j]))
        i = j + 1
    return out


# ── estimators ───────────────────────────────────────────────────────────────

def estimate_extrinsics_static(t_gt, p_gt, t_s, p_s, v_thresh, min_dur,
                               guard, with_scale):
    dwells = find_dwells(t_gt, p_gt, v_thresh, min_dur)
    pairs_s, pairs_g, kept = [], [], []
    for a, b in dwells:
        a, b = a + guard, b - guard                     # drop settle-in edges
        if b <= a:
            continue
        mg = (t_gt >= a) & (t_gt <= b)
        ms = (t_s >= a) & (t_s <= b)
        if mg.sum() < 3 or ms.sum() < 3:
            continue
        pairs_g.append(np.median(p_gt[mg], axis=0))
        pairs_s.append(np.median(p_s[ms], axis=0))
        kept.append((a, b, int(ms.sum()), float(np.std(p_s[ms], axis=0).max())))
    if len(pairs_s) < 3:
        raise SystemExit(
            f"only {len(pairs_s)} usable dwell(s) — need >=3 well-spread stops.\n"
            "Drive the robot to distinct spots and PAUSE a few seconds at each,\n"
            "or relax --v-thresh / --min-dur.")
    src, dst = np.array(pairs_s), np.array(pairs_g)
    s, R, t, rmse = umeyama_2d(src, dst, with_scale)
    return s, R, t, rmse, src, dst, kept


def speed_windowed(t_src, p_src, grid, w):
    """|v| from displacement over a fixed window w.

    Differentiating an interpolated track sample-by-sample amplifies position
    noise by 1/dt; over a 0.3 s window the same noise is divided by 0.3 s
    instead, which is the difference between a usable speed profile and pure
    noise for a 6 cm-noise sensor.
    """
    a = interp_at(t_src, p_src, grid - w / 2)
    b = interp_at(t_src, p_src, grid + w / 2)
    return np.linalg.norm(b - a, axis=1) / w


def xcorr_delay(t_gt, p_gt, t_s, p_s, max_lag, dt=0.02, win=0.3):
    """Delay from speed-profile correlation. Extrinsics-free.

    Sign convention: a POSITIVE result means the sensor is LATE — the sample it
    timestamps at t actually describes where the robot was at t - tau.
    """
    lo = max(t_gt[0], t_s[0]) + win
    hi = min(t_gt[-1], t_s[-1]) - win
    if hi - lo < 4 * max_lag:
        raise SystemExit("overlap too short for the requested --max-lag")
    grid = np.arange(lo, hi, dt)
    vg = speed_windowed(t_gt, p_gt, grid, win)
    vs = speed_windowed(t_s, p_s, grid, win)
    vg = vg - vg.mean()
    vs = vs - vs.mean()
    n = int(max_lag / dt)
    lags = np.arange(-n, n + 1)
    denom = np.sqrt((vg ** 2).sum() * (vs ** 2).sum()) + 1e-12
    # roll(vs, L) shifts the sensor profile EARLIER by L samples, so the peak
    # sits at L = -tau/dt.
    corr = np.array([np.dot(np.roll(vs, L), vg) for L in lags]) / denom
    i = int(np.argmax(corr))
    tau = -lags[i] * dt
    if 0 < i < len(corr) - 1:                            # parabolic refinement
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        d = (y0 - y2) / (2 * (y0 - 2 * y1 + y2) + 1e-12)
        tau -= d * dt
    return float(tau), float(corr[i]), -lags * dt, corr


def residual_vs_lag(t_gt, p_gt, t_s, p_s, taus, extr=None, with_scale=False):
    """RMS residual as a function of lag; re-solves extrinsics if none given."""
    out = []
    for tau in taus:
        tq = t_s - tau
        m = (tq >= t_gt[0]) & (tq <= t_gt[-1])
        if m.sum() < 20:
            out.append(np.nan)
            continue
        g = interp_at(t_gt, p_gt, tq[m])
        if extr is None:
            _, _, _, r = umeyama_2d(p_s[m], g, with_scale)
        else:
            s, R, t = extr
            r = float(np.sqrt(((g - apply_extr(p_s[m], s, R, t)) ** 2).sum(1).mean()))
        out.append(r)
    return np.array(out)


def refine_min(taus, res):
    i = int(np.nanargmin(res))
    tau = taus[i]
    if 0 < i < len(res) - 1 and np.all(np.isfinite(res[i-1:i+2])):
        y0, y1, y2 = res[i - 1], res[i], res[i + 1]
        d = (y0 - y2) / (2 * (y0 - 2 * y1 + y2) + 1e-12)
        tau += d * (taus[1] - taus[0])
    return float(tau), float(res[i])


def block_bootstrap_tau(t_gt, p_gt, t_s, p_s, max_lag, n_boot, block_s, rng):
    """CI for the delay by resampling contiguous blocks of the run."""
    taus = []
    lo, hi = max(t_gt[0], t_s[0]), min(t_gt[-1], t_s[-1])
    n_blocks = max(3, int((hi - lo) / block_s))
    for _ in range(n_boot):
        starts = rng.uniform(lo, hi - block_s, size=n_blocks)
        keep_s = np.zeros(len(t_s), bool)
        for a in starts:
            keep_s |= (t_s >= a) & (t_s <= a + block_s)
        if keep_s.sum() < 50:
            continue
        try:
            tau, _, _, _ = xcorr_delay(t_gt, p_gt, t_s[keep_s], p_s[keep_s], max_lag)
            taus.append(tau)
        except SystemExit:
            continue
    return np.array(taus)


# ── modes ────────────────────────────────────────────────────────────────────

def cmd_static(args):
    t_gt, p_gt = load_track(args.gt, args.tcol, args.xcol, args.ycol)
    t_s, p_s = load_track(args.sensor, args.tcol, args.xcol, args.ycol,
                          args.sensor_clock_offset)
    s, R, t, rmse, src, dst, kept = estimate_extrinsics_static(
        t_gt, p_gt, t_s, p_s, args.v_thresh, args.min_dur, args.guard, args.scale)

    print(f"\n{len(kept)} stationary dwell(s) used:")
    for (a, b, n, sd) in kept:
        print(f"   {b-a:5.1f} s window, {n:4d} sensor samples, "
              f"sensor spread while parked = {sd:.3f} m")
    print(f"\nextrinsics  sensor → Omron frame")
    print(f"   yaw    {yaw_deg(R):+8.3f} deg")
    print(f"   trans  [{t[0]:+.4f}, {t[1]:+.4f}] m")
    print(f"   scale  {s:.6f}" + ("" if args.scale else "   (fixed at 1)"))
    print(f"   dwell-centroid RMSE {rmse:.4f} m over {len(src)} points")
    print("\n   ← this number is a floor on the sensor's static accuracy;")
    print("     it contains NO latency, because nothing was moving.")

    if args.out:
        json.dump({"scale": s, "R": R.tolist(), "t": t.tolist(),
                   "rmse_static_m": rmse, "n_dwells": len(src)},
                  open(args.out, "w"), indent=2)
        print(f"\nwrote {args.out}")
    return 0


def cmd_delay(args, joint=False):
    t_gt, p_gt = load_track(args.gt, args.tcol, args.xcol, args.ycol)
    t_s, p_s = load_track(args.sensor, args.tcol, args.xcol, args.ycol,
                          args.sensor_clock_offset)
    print(f"gt     {len(t_gt):6d} samples, {t_gt[-1]-t_gt[0]:7.1f} s, "
          f"median rate {1/np.median(np.diff(t_gt)):.2f} Hz")
    print(f"sensor {len(t_s):6d} samples, {t_s[-1]-t_s[0]:7.1f} s, "
          f"median rate {1/np.median(np.diff(t_s)):.2f} Hz")
    print(f"overlap {min(t_gt[-1], t_s[-1]) - max(t_gt[0], t_s[0]):.1f} s")

    extr = None
    if not joint and args.extrinsics:
        e = json.load(open(args.extrinsics))
        extr = (e["scale"], np.array(e["R"]), np.array(e["t"]))
        print(f"\nusing extrinsics from {args.extrinsics}: "
              f"yaw {yaw_deg(extr[1]):+.3f}deg  t={np.round(extr[2],4).tolist()}")

    tau_x, peak, lags, corr = xcorr_delay(t_gt, p_gt, t_s, p_s, args.max_lag)
    print(f"\n[A] speed-profile cross-correlation (extrinsics-free)")
    print(f"    delay = {tau_x*1000:+8.1f} ms   peak correlation {peak:.3f}")
    if peak < 0.5:
        print("    ! weak peak — the run is probably too smooth/too short.")
        print("      Drive with clear stop-go-stop transitions.")

    taus = np.arange(-args.max_lag, args.max_lag + 1e-9, args.lag_step)
    res = residual_vs_lag(t_gt, p_gt, t_s, p_s, taus, extr, args.scale)
    tau_r, rmse_min = refine_min(taus, res)
    label = "re-solving extrinsics at every lag" if extr is None else "extrinsics fixed"
    print(f"\n[B] residual minimisation over lag ({label})")
    print(f"    delay = {tau_r*1000:+8.1f} ms   RMSE at optimum {rmse_min:.4f} m")
    r0 = residual_vs_lag(t_gt, p_gt, t_s, p_s, np.array([0.0]), extr, args.scale)[0]
    print(f"    RMSE with no delay correction {r0:.4f} m "
          f"→ correcting the delay removes {100*(1-rmse_min/max(r0,1e-9)):.1f}%")

    print(f"\n[A] vs [B] disagreement: {abs(tau_x-tau_r)*1000:.1f} ms")
    if abs(tau_x - tau_r) > 0.05:
        print("    ! >50 ms apart. Suspect a non-constant delay, dropped frames,")
        print("      or extrinsics that are absorbing part of the lag.")
    else:
        print("    ✓ consistent — the delay estimate is trustworthy.")

    if args.bootstrap:
        rng = np.random.default_rng(0)
        bs = block_bootstrap_tau(t_gt, p_gt, t_s, p_s, args.max_lag,
                                 args.bootstrap, args.block, rng)
        if len(bs) > 10:
            lo, hi = np.percentile(bs, [2.5, 97.5])
            print(f"\nblock bootstrap (n={len(bs)}): delay 95% CI "
                  f"[{lo*1000:+.1f}, {hi*1000:+.1f}] ms   sd {bs.std()*1000:.1f} ms")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
        ax[0].plot(lags, corr, lw=1)
        ax[0].axvline(tau_x, color="r", ls="--", label=f"{tau_x*1000:+.0f} ms")
        ax[0].set_xlabel("lag [s]"); ax[0].set_ylabel("speed corr")
        ax[0].set_title("[A] speed-profile xcorr"); ax[0].legend(); ax[0].grid(alpha=.3)

        ax[1].plot(taus, res, lw=1)
        ax[1].axvline(tau_r, color="r", ls="--", label=f"{tau_r*1000:+.0f} ms")
        ax[1].set_xlabel("lag [s]"); ax[1].set_ylabel("RMSE [m]")
        ax[1].set_title("[B] residual vs lag"); ax[1].legend(); ax[1].grid(alpha=.3)

        tq = t_s - tau_r
        m = (tq >= t_gt[0]) & (tq <= t_gt[-1])
        g = interp_at(t_gt, p_gt, tq[m])
        q = apply_extr(p_s[m], *extr) if extr else \
            apply_extr(p_s[m], *umeyama_2d(p_s[m], g, args.scale)[:3])
        ax[2].plot(p_gt[:, 0], p_gt[:, 1], "-", lw=1, label="Omron GT")
        ax[2].plot(q[:, 0], q[:, 1], ".", ms=2, label="sensor, aligned")
        ax[2].set_aspect("equal"); ax[2].set_title("aligned tracks")
        ax[2].legend(); ax[2].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(args.plot, dpi=140)
        print(f"\nwrote {args.plot}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def common(p):
        p.add_argument("gt", help="ground-truth CSV (Omron)")
        p.add_argument("sensor", help="sensor CSV (camera fusion / UWB)")
        p.add_argument("--tcol", default="t")
        p.add_argument("--xcol", default="x")
        p.add_argument("--ycol", default="y")
        p.add_argument("--sensor-clock-offset", type=float, default=0.0,
                       help="seconds to ADD to sensor times (from clock_audit)")
        p.add_argument("--scale", action="store_true",
                       help="also solve a scale factor (leave off unless both "
                            "frames are genuinely metric-uncertain)")

    st_p = sub.add_parser("static", help="extrinsics from stationary dwells")
    common(st_p)
    st_p.add_argument("--v-thresh", type=float, default=0.05,
                      help="m/s, applied as windowed displacement")
    st_p.add_argument("--min-dur", type=float, default=2.0, help="s")
    st_p.add_argument("--guard", type=float, default=0.5,
                      help="s trimmed from each end of a dwell")
    st_p.add_argument("--out", help="write extrinsics JSON here")

    dl = sub.add_parser("delay", help="delay with extrinsics fixed")
    common(dl)
    dl.add_argument("--extrinsics", help="JSON from `static`")
    dl.add_argument("--max-lag", type=float, default=1.5)
    dl.add_argument("--lag-step", type=float, default=0.005)
    dl.add_argument("--bootstrap", type=int, default=200)
    dl.add_argument("--block", type=float, default=20.0)
    dl.add_argument("--plot")

    jn = sub.add_parser("joint", help="delay + extrinsics together (cross-check)")
    common(jn)
    jn.add_argument("--max-lag", type=float, default=1.5)
    jn.add_argument("--lag-step", type=float, default=0.005)
    jn.add_argument("--bootstrap", type=int, default=200)
    jn.add_argument("--block", type=float, default=20.0)
    jn.add_argument("--plot")

    args = ap.parse_args()
    if args.mode == "static":
        return cmd_static(args)
    return cmd_delay(args, joint=(args.mode == "joint"))


if __name__ == "__main__":
    sys.exit(main())
