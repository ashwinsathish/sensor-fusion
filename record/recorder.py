#!/usr/bin/env python3
"""Recording machinery: clock, streams, sources, run segmentation, QC.

The operator-facing tool is `collect.py`. This module holds the parts.

Design: the sensor connections are opened ONCE and stay up for the whole
session. Starting and stopping a run only swaps where the rows go. That way
you never wait for a camera to reconnect between runs, and a stream that dies
is visible on the display long before you drive anywhere.
"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics as st
import threading
import time
from collections import deque

SESSIONS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "sessions"))


# ── master clock ─────────────────────────────────────────────────────────────

class PiClock:
    """(local - Omron Pi) offset, minimum filtered.

    Each sample is the true offset plus a positive, variable transport delay,
    so the minimum over a window is the least contaminated estimate. Same
    trick NTP uses. Measured stability on this link: better than 1 ms.
    """

    def __init__(self, window_s: float = 60.0):
        self.window_s = window_s
        self._s: deque = deque()
        self.offset: float | None = None
        self.n = 0

    def add(self, t_local: float, t_pi: float) -> None:
        self._s.append((t_local, t_local - t_pi))
        cut = t_local - self.window_s
        while self._s and self._s[0][0] < cut:
            self._s.popleft()
        if len(self._s) >= 5:
            self.offset = min(v for _, v in self._s)
            self.n = len(self._s)

    def to_pi(self, t_local: float) -> float | None:
        return None if self.offset is None else t_local - self.offset


# ── streams ──────────────────────────────────────────────────────────────────

FIELDS = {
    # Long format, one row per measurement. Never merged across sources at
    # record time — the three streams run at different rates and any alignment
    # baked in here could not be undone later.
    "omron": ["t", "x", "y", "x_native", "y_native", "frame_in", "t_recv", "latency_s", "internal_s", "t_published",
              "clock_synced", "seq", "theta_deg", "status", "loc_score",
              "t_valid_is_arrival"],
    "uwb": ["t", "x", "y", "x_unfiltered", "y_unfiltered",
            "x_native", "y_native", "frame_in", "t_recv", "latency_s", "internal_s", "t_published",
            "clock_synced", "seq", "frame_nr", "n_anchors", "anchor_earliest",
            "anchors_used", "residual_m", "sigma_m", "method",
            "tag_node_id", "rounds_deferred", "t_valid_is_arrival"],
    "camera": ["t", "x", "y", "x_native", "y_native", "frame_in", "t_recv", "latency_s", "internal_s", "t_published",
               "clock_synced", "seq", "class", "conf", "n_cams", "cams",
               "vx", "vy", "t_frame_arrived", "t_detect_done",
               "t_valid_is_arrival"],
    "per_cam": ["t", "x", "y", "t_recv", "cam", "px", "py", "conf", "seq"],
    "agilox": ["t", "x", "y", "x_native", "y_native", "frame_in", "t_recv", "latency_s", "clock_synced", "seq",
               "t_valid_is_arrival"],
}


class Stream:
    def __init__(self, path: str, fields: list[str]):
        self.path = path
        self._fh = open(path, "w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=fields, extrasaction="ignore")
        self._w.writeheader()
        self.n = 0
        self._lock = threading.Lock()

    def write(self, row: dict) -> None:
        with self._lock:
            self._w.writerow(row)
            self.n += 1
            if self.n % 50 == 0:
                self._fh.flush()

    def close(self) -> None:
        with self._lock:
            self._fh.flush()
            self._fh.close()


class Health:
    """Liveness of a source, tracked whether or not a run is recording."""

    def __init__(self):
        self.total = 0
        self.last = None
        self._recent: deque = deque(maxlen=60)

    def beat(self) -> None:
        now = time.time()
        self.total += 1
        self.last = now
        self._recent.append(now)

    @property
    def hz(self) -> float:
        if len(self._recent) < 3:
            return 0.0
        span = self._recent[-1] - self._recent[0]
        return (len(self._recent) - 1) / span if span > 0 else 0.0

    @property
    def age_s(self) -> float:
        return math.inf if self.last is None else time.time() - self.last


# ── dwell detection ──────────────────────────────────────────────────────────

class DwellCounter:
    """Counts stationary stretches in the ground truth, live.

    Thresholds a windowed displacement rather than a derivative — a noisy
    5 Hz track differentiated sample-to-sample manufactures speed that is not
    there and shreds real dwells into fragments.
    """

    def __init__(self, radius_m: float = 0.06, min_dur_s: float = 8.0):
        self.radius_m = radius_m
        self.min_dur_s = min_dur_s
        self._buf: deque = deque(maxlen=400)
        self.dwells: list[dict] = []
        self._open: float | None = None
        self._counted = False

    def reset(self) -> None:
        self._buf.clear()
        self.dwells = []
        self._open = None
        self._counted = False

    def add(self, t: float, x: float, y: float) -> None:
        self._buf.append((t, x, y))
        recent = [p for p in self._buf if p[0] > t - 2.0]
        if len(recent) < 5:
            return
        xs = [p[1] for p in recent]
        ys = [p[2] for p in recent]
        still = (max(xs) - min(xs)) < self.radius_m and (max(ys) - min(ys)) < self.radius_m
        if still:
            if self._open is None:
                self._open, self._counted = recent[0][0], False
            elif not self._counted and (t - self._open) >= self.min_dur_s:
                self.dwells.append({"t_start": self._open, "t_end": t,
                                    "x": st.median(xs), "y": st.median(ys)})
                self._counted = True
        else:
            self._open, self._counted = None, False

    @property
    def held_s(self) -> float:
        return 0.0 if self._open is None else time.time() - self._open


class MotionMonitor:
    """Tracks whether a driving run has the stop-go content the delay
    estimator needs. A smooth constant-speed loop is worthless for it."""

    def __init__(self):
        self._buf: deque = deque(maxlen=600)
        self.stop_go = 0
        self._moving = None

    def reset(self) -> None:
        self._buf.clear()
        self.stop_go = 0
        self._moving = None

    def add(self, t: float, x: float, y: float) -> None:
        self._buf.append((t, x, y))
        w = [p for p in self._buf if p[0] > t - 1.0]
        if len(w) < 4:
            return
        d = math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2])
        dt = w[-1][0] - w[0][0]
        v = d / dt if dt > 0 else 0.0
        moving = v > 0.15
        if self._moving is not None and moving != self._moving:
            self.stop_go += 1
        self._moving = moving

    @property
    def speed_mps(self) -> float:
        if len(self._buf) < 4:
            return 0.0
        w = list(self._buf)[-10:]
        dt = w[-1][0] - w[0][0]
        if dt <= 0:
            return 0.0
        return math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2]) / dt


# ── one recorded run ─────────────────────────────────────────────────────────

class Run:
    def __init__(self, root: str, name: str, mode: str):
        self.name = name
        self.mode = mode
        self.dir = os.path.join(root, name)
        os.makedirs(self.dir, exist_ok=True)
        self.streams = {k: Stream(os.path.join(self.dir, f"{k}.csv"), v)
                        for k, v in FIELDS.items()}
        self.clock_log = Stream(os.path.join(self.dir, "clock.csv"),
                                ["t_local", "offset_s", "n"])
        self.dwell = DwellCounter()
        self.motion = MotionMonitor()
        self.marks: list[dict] = []
        self.latency: dict[str, list] = {k: [] for k in FIELDS}
        self.unsynced: dict[str, int] = {k: 0 for k in FIELDS}
        self.t0 = time.time()

    @property
    def elapsed(self) -> float:
        return time.time() - self.t0

    def write(self, kind: str, row: dict) -> None:
        s = self.streams.get(kind)
        if s is None:
            return
        s.write(row)
        lat = row.get("latency_s")
        if isinstance(lat, (int, float)):
            self.latency[kind].append(float(lat))
        if row.get("clock_synced") is False:
            self.unsynced[kind] += 1

    def mark(self, label: str, t_pi: float | None) -> None:
        self.marks.append({"label": label, "t": t_pi, "t_local": time.time()})

    def finish(self, clock: PiClock, clock_mode: str = "local_ntp") -> dict:
        rows = {k: s.n for k, s in self.streams.items()}
        for s in self.streams.values():
            s.close()
        self.clock_log.close()
        manifest = {
            "name": self.name, "mode": self.mode,
            "started_local": self.t0, "duration_s": self.elapsed,
            "rows": rows,
            "clock": {
                "mode": clock_mode,
                "offset_to_omron_pi_s": clock.offset,
                "note": ("this host was NTP-synced; arrival times are its own clock"
                         if clock_mode == "local_ntp" else
                         "this host was NOT NTP-synced; arrival times were "
                         "corrected by the measured offset to the Omron Pi, "
                         "which is NTP-synced"),
            },
            "dwells": self.dwell.dwells,
            "marks": self.marks,
            "stop_go_transitions": self.motion.stop_go,
            "latency_median_s": {k: (sorted(v)[len(v)//2] if v else None)
                                 for k, v in self.latency.items()},
            "unsynced_messages": {k: v for k, v in self.unsynced.items() if v},
            "time_sources": {
                "omron": "producer timestamp in the MQTT payload (the master)",
                "camera": "shutter instant from RTCP, mapped to Pi time",
                "uwb": "t_round if the solver patch is live, else arrival",
                "agilox": "arrival only — that topic carries no timestamp",
            },
        }
        with open(os.path.join(self.dir, "session.json"), "w") as fh:
            json.dump(manifest, fh, indent=2)
        return manifest

    # -- quality control -------------------------------------------------
    def check(self, clock_mode: str = "local_ntp") -> list[tuple[str, str]]:
        """Returns [(level, message)] — 'ok' | 'warn' | 'bad'."""
        out = []
        if clock_mode != "local_ntp":
            out.append(("warn", "this host was not NTP-synced; latencies were "
                                "corrected via the Omron Pi. Usable, but fix NTP "
                                "for the next run."))
        n_gt = self.streams["omron"].n
        if n_gt < 50:
            out.append(("bad", f"only {n_gt} ground-truth rows — was MQTT up?"))
        else:
            out.append(("ok", f"{n_gt} ground-truth rows over "
                              f"{self.elapsed/60:.1f} min"))
        for k in ("camera", "uwb"):
            n = self.streams[k].n
            if n == 0:
                out.append(("warn", f"no {k} data — that sensor contributes nothing"))
            else:
                out.append(("ok", f"{n} {k} rows"))

        # A publisher whose clock is not disciplined makes its latency column
        # meaningless. Catch it here, not three weeks later.
        for k, bad in self.unsynced.items():
            if bad:
                out.append(("bad", f"{k}: {bad} messages with clock_synced=false — "
                                   "that source's timestamps cannot be trusted"))
        for k, lat in self.latency.items():
            if len(lat) < 20:
                continue
            lat = sorted(lat)
            med = lat[len(lat) // 2]
            if med < 0:
                out.append(("bad", f"{k}: median latency {med*1000:.0f} ms is NEGATIVE "
                                   "— a clock is wrong, this data is unusable"))
            elif med > 1.0:
                out.append(("bad", f"{k}: median latency {med:.1f} s — far too large "
                                   "for a LAN. Check that publisher's clock."))
            else:
                out.append(("ok", f"{k} latency median {med*1000:.0f} ms, "
                                  f"p95 {lat[int(len(lat)*0.95)]*1000:.0f} ms"))

        if self.mode == "parking":
            n = len(self.dwell.dwells)
            if n < 3:
                out.append(("bad", f"only {n} dwells — need 3 minimum, 8 ideally. "
                                   "Park longer and hold still."))
            elif n < 6:
                out.append(("warn", f"{n} dwells — usable, but more is better"))
            else:
                out.append(("ok", f"{n} dwells captured"))
            if n >= 2:
                xs = [d["x"] for d in self.dwell.dwells]
                ys = [d["y"] for d in self.dwell.dwells]
                spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                lvl = "ok" if spread > 8 else "warn"
                out.append((lvl, f"dwells span {spread:.1f} m — "
                                 f"{'good coverage' if spread > 8 else 'clustered; spread them out'}"))
        else:
            sg = self.motion.stop_go
            if sg < 6:
                out.append(("bad", f"only {sg} stop/go transitions — the delay "
                                   "estimate needs sharp starts and stops"))
            elif sg < 15:
                out.append(("warn", f"{sg} stop/go transitions — more would help"))
            else:
                out.append(("ok", f"{sg} stop/go transitions"))
        return out
