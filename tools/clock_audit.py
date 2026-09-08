#!/usr/bin/env python3
"""Audit the clocks behind the LIT fusion stack.

Every timestamp in this testbed is written by a different, free-running
machine. Before any millisecond-level fusion claim can be made, we have to know
how far apart those clocks are and how fast they drift apart.

Two modes:

  live    subscribe to MQTT and compare each producer-side timestamp in the
          payload against this host's clock. Separates the CONSTANT part
          (clock offset, harmless once known) from the VARIABLE part
          (transport jitter, the real limit on fusion accuracy).

  logs    replay recorded live_sim CSV runs and fit offset + skew (ppm) per
          run, then plot how the offset wandered across runs/days.

Usage:
    python3 clock_audit.py live  --host 127.0.0.1 --port 1833 --seconds 60
    python3 clock_audit.py logs  ~/LIT_fac_ray_tracing/logs/live_sim_runs
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import statistics as st
import sys
import time


# ── live mode ────────────────────────────────────────────────────────────────

def parse_producer_ts(topic: str, payload: bytes):
    """Return the producer-side timestamp a payload carries, or None."""
    try:
        text = payload.decode("utf-8", "replace")
    except Exception:
        return None
    if topic.startswith("Omron"):
        # iws-testbed CSV: field 0 is the collector's unix time.
        try:
            ts = float(text.split(",")[0])
        except (ValueError, IndexError):
            return None
        return ts if 1.0e9 < ts < 4.0e9 else None
    # Agilox/status is "<seq>,<x_mm>,<y_mm>,<theta>" — no producer timestamp.
    return None


def run_live(args) -> int:
    try:
        import paho.mqtt.client as mqtt
    except ModuleNotFoundError:
        print("needs paho-mqtt:  pip install paho-mqtt")
        return 2

    samples: dict[str, list[tuple[float, float]]] = {}
    arrivals: dict[str, list[float]] = {}

    def on_connect(c, u, f, rc, p=None):
        print(f"connected to {args.host}:{args.port} (rc={rc})")
        for t in args.topics:
            c.subscribe(t, qos=0)
            print(f"  subscribed {t}")

    def on_message(c, u, m):
        now = time.time()
        arrivals.setdefault(m.topic, []).append(now)
        ts = parse_producer_ts(m.topic, m.payload)
        if ts is not None:
            samples.setdefault(m.topic, []).append((now, now - ts))

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "lit-clock-audit")
    cli.on_connect, cli.on_message = on_connect, on_message
    cli.connect(args.host, args.port, 30)
    cli.loop_start()
    end = time.time() + args.seconds
    while time.time() < end:
        time.sleep(0.2)
    cli.loop_stop()
    cli.disconnect()

    print(f"\n{'=' * 72}\nlisted for {args.seconds:.0f} s\n")
    if not arrivals:
        print("no messages — is the tunnel to the broker up?")
        return 1

    for topic in sorted(arrivals):
        a = arrivals[topic]
        gaps = [b - x for x, b in zip(a, a[1:])]
        print(f"── {topic}   n={len(a)}")
        if gaps:
            print(f"   publish period: median {1000*st.median(gaps):7.1f} ms   "
                  f"min {1000*min(gaps):.1f}   max {1000*max(gaps):.1f}   "
                  f"→ {1/st.median(gaps):.2f} Hz")
        s = samples.get(topic)
        if not s:
            print("   producer timestamp: ABSENT — arrival time is all you have.")
            print("   → cannot separate clock offset from transport delay. Fix at")
            print("     the publisher: stamp the payload at acquisition.\n")
            continue
        offs = [d for _, d in s]
        med = st.median(offs)
        jit = [o - med for o in offs]
        print(f"   producer timestamp: present")
        print(f"   offset (host - producer): median {med:+.4f} s")
        print(f"   jitter about median: std {1000*st.pstdev(offs):6.2f} ms   "
              f"p95 {1000*sorted(abs(j) for j in jit)[int(0.95*len(jit))]:6.2f} ms   "
              f"max {1000*max(abs(j) for j in jit):6.2f} ms")
        if abs(med) > 0.5:
            print(f"   !! {abs(med):.2f} s of clock offset. This is NOT latency —")
            print( "      it is unsynchronised clocks. Discipline both ends (NTP/PTP)")
            print( "      or estimate and subtract it per session.")
        if st.pstdev(offs) < 0.010:
            print( "   ✓ transport jitter is small; once clocks are disciplined,")
            print( "     these timestamps are good enough for ms-level fusion.")
        print()

    print("── this host ──")
    os.system("timedatectl 2>/dev/null | sed -n '1,8p'")
    return 0


# ── logs mode ────────────────────────────────────────────────────────────────

def run_logs(args) -> int:
    try:
        import numpy as np
    except ModuleNotFoundError:
        print("needs numpy")
        return 2

    files = sorted(glob.glob(os.path.join(args.dir, "*", "csv_*.csv")))
    if not files:
        print(f"no csv_*.csv under {args.dir}/*/")
        return 1
    print(f"{len(files)} run files\n")

    per_run = []
    for f in files:
        try:
            rows = list(csv.DictReader(open(f)))
        except Exception:
            continue
        o, w = [], []
        for r in rows:
            try:
                o.append(float(r["omron_timestamp_s"]))
                w.append(float(r["wall_time_unix_s"]))
            except (KeyError, ValueError, TypeError):
                continue
        if len(o) < args.min_samples:
            continue
        o, w = np.array(o), np.array(w)
        d = w - o
        span = w[-1] - w[0]
        skew = np.polyfit(w - w[0], d, 1)[0] * 1e6 if span > 300 else float("nan")
        resid = (d - np.polyval(np.polyfit(w - w[0], d, 1), w - w[0])
                 if span > 300 else d - np.median(d))
        per_run.append({
            "run": os.path.basename(os.path.dirname(f)),
            "t0": w[0], "n": len(d), "dur_min": span / 60,
            "offset": float(np.median(d)), "skew_ppm": float(skew),
            "resid_ms": float(resid.std() * 1000),
            "resid_p99_ms": float(np.percentile(np.abs(resid), 99) * 1000),
        })

    per_run.sort(key=lambda r: r["t0"])
    print(f"{'run':38s} {'n':>6s} {'dur/min':>8s} {'offset/s':>10s} "
          f"{'skew/ppm':>9s} {'jitter/ms':>10s} {'p99/ms':>8s}")
    step = max(1, len(per_run) // args.rows)
    for r in per_run[::step]:
        print(f"{r['run']:38s} {r['n']:6d} {r['dur_min']:8.1f} "
              f"{r['offset']:+10.3f} {r['skew_ppm']:9.2f} "
              f"{r['resid_ms']:10.1f} {r['resid_p99_ms']:8.1f}")

    offs = [r["offset"] for r in per_run]
    jit = [r["resid_ms"] for r in per_run]
    print(f"\n{'=' * 72}")
    print(f"offset across {len(per_run)} runs: min {min(offs):+.3f} s  "
          f"max {max(offs):+.3f} s  → SWING {max(offs)-min(offs):.1f} s")
    print(f"within-run jitter (after removing offset+skew): "
          f"median {st.median(jit):.1f} ms  worst {max(jit):.1f} ms")
    print("\nreading: the swing is unsynchronised clocks (must be fixed or")
    print("estimated per session); the within-run jitter is the residual timing")
    print("uncertainty that actually limits fusion accuracy.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [(r["t0"] - per_run[0]["t0"]) / 86400 for r in per_run]
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(t, offs, ".-", lw=0.8)
        ax[0].set_ylabel("offset host−Omron [s]")
        ax[0].set_title("Omron collector clock vs fusion host clock")
        ax[0].grid(alpha=.3)
        ax[1].semilogy(t, jit, ".", ms=4)
        ax[1].set_ylabel("within-run jitter [ms]")
        ax[1].set_xlabel("days since first run")
        ax[1].grid(alpha=.3, which="both")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=140)
        print(f"\nwrote {args.plot}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    lv = sub.add_parser("live")
    lv.add_argument("--host", default="127.0.0.1")
    lv.add_argument("--port", type=int, default=1833)
    lv.add_argument("--seconds", type=float, default=60.0)
    lv.add_argument("--topics", nargs="+", default=["Omron/status", "Agilox/status"])
    lv.set_defaults(func=run_live)

    lg = sub.add_parser("logs")
    lg.add_argument("dir")
    lg.add_argument("--min-samples", type=int, default=50)
    lg.add_argument("--rows", type=int, default=30, help="max table rows to print")
    lg.add_argument("--plot", help="write a PNG here")
    lg.set_defaults(func=run_logs)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
