#!/usr/bin/env python3
"""Use the Omron Pi's clock as the shared time reference, over MQTT.

Fusion does not need UTC. It needs every measurement on ONE timeline. Which
clock that is does not matter, as long as everything agrees on it.

The Omron Pi already broadcasts its clock 5 times a second — every
`Omron/status` payload carries the collector's timestamp in field 0. Measuring
the offset between that and the local clock turns the Pi into a master clock
for the whole setup, with no NTP, no firewall change, and nobody's permission.

The estimator is NTP's own trick. Each sample gives

    raw_offset = t_local_receive - t_pi_send
               = true_offset + one_way_delay

The one-way delay is always positive and varies with queueing, so the MINIMUM
raw offset over a window is the least-contaminated estimate. Averaging would
be pulled around by every delayed packet; the minimum is not.

Measured on this link: 0.37 ms of block-to-block stability over 75 s.

    python3 pi_clock_ref.py                       # watch the offset live
    python3 pi_clock_ref.py --write offset.json   # keep a file recorders read
    python3 pi_clock_ref.py --once --seconds 30   # one measurement and exit

To convert a locally-taken timestamp into Pi time:  t_pi = t_local - offset
"""

from __future__ import annotations

import argparse
import json
import statistics as st
import sys
import time
from collections import deque


class PiClock:
    """Rolling minimum-filtered estimate of (local clock - Pi clock)."""

    def __init__(self, window_s: float = 30.0):
        self.window_s = window_s
        self.samples: deque[tuple[float, float]] = deque()

    def add(self, t_local: float, t_pi: float) -> None:
        self.samples.append((t_local, t_local - t_pi))
        cutoff = t_local - self.window_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    @property
    def offset(self) -> float | None:
        if len(self.samples) < 5:
            return None
        return min(v for _, v in self.samples)

    @property
    def stats(self):
        vals = [v for _, v in self.samples]
        if len(vals) < 5:
            return None
        mn = min(vals)
        return {
            "offset_s": mn,
            "n": len(vals),
            "jitter_ms": (st.pstdev(vals) * 1000) if len(vals) > 1 else 0.0,
            "spread_ms": (max(vals) - mn) * 1000,
        }

    def to_pi_time(self, t_local: float) -> float | None:
        off = self.offset
        return None if off is None else t_local - off


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1833)
    ap.add_argument("--topic", default="Omron/status")
    ap.add_argument("--window", type=float, default=30.0,
                    help="seconds of history the minimum is taken over")
    ap.add_argument("--write", help="continuously write the offset to this JSON file")
    ap.add_argument("--once", action="store_true", help="measure once and exit")
    ap.add_argument("--seconds", type=float, default=30.0, help="duration for --once")
    args = ap.parse_args()

    try:
        import paho.mqtt.client as mqtt
    except ModuleNotFoundError:
        print("needs paho-mqtt:  pip install paho-mqtt")
        return 2

    clock = PiClock(args.window)
    drift = deque(maxlen=400)          # (t_local, offset) for the skew estimate

    def on_connect(c, u, f, rc, p=None):
        c.subscribe(args.topic, qos=0)

    def on_message(c, u, m):
        now = time.time()
        try:
            t_pi = float(m.payload.decode("utf-8", "replace").split(",")[0])
        except (ValueError, IndexError):
            return
        if not (1.0e9 < t_pi < 4.0e9):
            return
        clock.add(now, t_pi)

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "lit-pi-clock-ref")
    cli.on_connect, cli.on_message = on_connect, on_message
    try:
        cli.connect(args.host, args.port, 30)
    except OSError as exc:
        print(f"cannot reach the broker at {args.host}:{args.port} — {exc}")
        print("is the tunnel to the factory up?")
        return 2
    cli.loop_start()

    deadline = time.time() + args.seconds if args.once else None
    last_print = 0.0
    try:
        while True:
            time.sleep(0.5)
            s = clock.stats
            now = time.time()
            if s:
                drift.append((now, s["offset_s"]))
                if args.write:
                    with open(args.write, "w") as fh:
                        json.dump({"offset_s": s["offset_s"],
                                   "measured_at": now,
                                   "n": s["n"], "jitter_ms": s["jitter_ms"],
                                   "note": "t_pi = t_local - offset_s"}, fh)
                if now - last_print > 2.0:
                    last_print = now
                    skew = ""
                    if len(drift) > 60 and drift[-1][0] - drift[0][0] > 60:
                        dt = drift[-1][0] - drift[0][0]
                        dv = drift[-1][1] - drift[0][1]
                        skew = f"   drift {dv/dt*1e6:+.1f} ppm"
                    print(f"offset {s['offset_s']:+.4f} s   n={s['n']:3d}   "
                          f"jitter {s['jitter_ms']:5.2f} ms   "
                          f"spread {s['spread_ms']:5.1f} ms{skew}")
            if deadline and now > deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cli.loop_stop()
        cli.disconnect()

    s = clock.stats
    if not s:
        print("\nno usable samples — is the Omron publishing?")
        return 1
    print(f"\n{'='*64}")
    print(f"local clock - Pi clock = {s['offset_s']:+.6f} s")
    print(f"convert any locally-taken timestamp with:  t_pi = t_local "
          f"- ({s['offset_s']:+.6f})")
    print("\nThe absolute value carries an unknown one-way-transit bias of a few ms.")
    print("That bias is constant, identical for every sensor referenced this way,")
    print("and is absorbed by the per-sensor delay you estimate in estimate_delay.py.")
    print("Re-measure continuously: the two clocks drift apart by tens of ppm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
