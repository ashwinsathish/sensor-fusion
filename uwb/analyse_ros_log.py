#!/usr/bin/env python3
"""Analyse a raw UWB ROS log (`3d_meas.txt` style) end to end.

Answers three questions the team has been guessing at:

  1. How closely do the anchor Raspberry Pis agree on when a ranging round
     happened? (Their guess: 1-10 ms. Measured: 2.75 ms median — but almost
     all of it is a fixed per-node bias, not jitter.)
  2. Are the anchor DW3000 radio clocks synchronised? (They are not: ~2 ppm
     of free-running drift, which is metres of range error per frame.)
  3. Can the reference-anchor broadcasts correct that in software, and how
     well? (Yes — 4-8 cm.)

Needs `msgpack`; the uwb-visualization venv has it:

    /home/sathishkumara/uwb-visualization/.venv/bin/python analyse_ros_log.py 3d_meas.txt

Findings from the 28 Apr 2026 log are written up in FINDINGS_raw_ros_log.md.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import re
import sys
from collections import defaultdict

import numpy as np

DW_TIME_UNITS = 1.0 / 499.2e6 / 128.0        # 15.65 ps
SPEED_OF_LIGHT = 299702547.0
LINE = re.compile(r"^\[UWBNODE (\d+)\],\[([^\]]+)\],(.*)$")


def parse(path: str):
    """-> list of (log_node, pi_timestamp, decoded_msgpack)."""
    import msgpack
    out, failed = [], 0
    for line in open(path, errors="replace"):
        m = LINE.match(line.rstrip("\n"))
        if not m:
            continue
        node, ts, rest = m.groups()
        if not rest.startswith("MPACK:"):
            continue
        b = rest[6:].strip()
        try:
            # The logger truncates base64 padding; restore it before decoding,
            # and use Unpacker so trailing bytes do not abort the parse.
            raw = base64.b64decode(b + "=" * (-len(b) % 4))
            up = msgpack.Unpacker(raw=False)
            up.feed(raw)
            d = up.unpack()
        except Exception:
            failed += 1
            continue
        out.append((int(node), dt.datetime.fromisoformat(ts).timestamp(), d))
    out.sort(key=lambda r: r[1])
    return out, failed


def wrap40(d: np.ndarray) -> np.ndarray:
    """DW3000 timestamps are 40-bit and wrap every ~17.2 s."""
    d = np.where(d > 2 ** 39, d - 2 ** 40, d)
    return np.where(d < -2 ** 39, d + 2 ** 40, d)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--ref", type=lambda s: int(s, 0), default=0x1111,
                    help="reference-anchor node id")
    ap.add_argument("--tag", type=lambda s: int(s, 0), default=0x3c14,
                    help="tag node id")
    args = ap.parse_args()

    recs, failed = parse(args.log)
    if not recs:
        print("no records decoded — is this the right file?")
        return 1
    span = recs[-1][1] - recs[0][1]
    print(f"{len(recs)} records over {span:.1f} s ({failed} undecodable)\n")

    rx = defaultdict(dict)            # (frame, transmitter) -> {node_id: dw_ts}
    pit = defaultdict(list)           # (frame, transmitter) -> [(log_node, pi_ts)]
    for node, t, d in recs:
        if "dtm_poll_rx" not in d:
            continue
        key = (d["frame_nr"], d["RX_node_id"])
        rx[key][d["node_id"]] = d["dtm_poll_rx"]
        pit[key].append((node, t))

    tx_counts = defaultdict(int)
    for _f, tx in rx:
        tx_counts[tx] += 1
    print("transmitters:", {hex(k): v for k, v in tx_counts.items()})
    sizes = np.array([len(v) for v in rx.values()])
    print(f"rounds {len(rx)}   anchors per round: min {sizes.min()} "
          f"median {int(np.median(sizes))} max {sizes.max()}")

    init = sorted((t, d["frame_nr"]) for _n, t, d in recs if "resp_rx_ts" in d)
    if len(init) > 5:
        tt = np.array([t for t, _ in init]); fr = np.array([f for _, f in init])
        k = np.diff(fr) > 0
        per = np.diff(tt)[k] / np.diff(fr)[k]
        print(f"ranging period {np.median(per)*1000:.2f} ms "
              f"-> {1/np.median(per):.2f} Hz")

    # ── 1. Pi timestamp agreement ────────────────────────────────────────
    print("\n1. Raspberry Pi timestamp spread within a ranging round")
    spread = np.array([(max(t for _n, t in v) - min(t for _n, t in v)) * 1000
                       for v in pit.values() if len(v) > 1])
    print(f"   median {np.median(spread):.2f} ms   p90 {np.percentile(spread,90):.2f}   "
          f"p99 {np.percentile(spread,99):.2f}   max {spread.max():.2f} ms")
    rel = defaultdict(list)
    for v in pit.values():
        if len(v) < 3:
            continue
        tmin = min(t for _n, t in v)
        for n, t in v:
            rel[n].append((t - tmin) * 1000)
    print("   per-node delay relative to the earliest in each round:")
    for n in sorted(rel):
        a = np.array(rel[n])
        print(f"     NODE {n:02d}  n={len(a):5d}  median {np.median(a):6.2f} ms   "
              f"p90 {np.percentile(a,90):6.2f}   max {a.max():7.2f}")
    print("   -> a stable per-node BIAS, not jitter. Removable, if the publisher")
    print("      records which node the earliest timestamp came from.")

    # ── 2. radio clock synchronisation ───────────────────────────────────
    def series(a, b, tx):
        f, v = [], []
        for (fr_, tx_), g in rx.items():
            if tx_ != tx or a not in g or b not in g:
                continue
            f.append(fr_)
            v.append(g[a] - g[b])
        o = np.argsort(f)
        return (np.array(f)[o],
                wrap40(np.array(v, dtype=float)[o]) * DW_TIME_UNITS * SPEED_OF_LIGHT)

    anchors = sorted({i for g in rx.values() for i in g})
    pairs = [(a, b) for i, a in enumerate(anchors) for b in anchors[i + 1:]]

    print("\n2. Raw DW3000 clock differences between anchors (metres)")
    for a, b in pairs:
        f, v = series(a, b, args.ref)
        if len(f) < 20:
            continue
        drift = np.polyfit(f - f[0], v, 1)[0]
        ppm = drift / 1e-1 / SPEED_OF_LIGHT * 1e6      # per 100 ms frame
        print(f"   {hex(a)}-{hex(b)}  median {np.median(v):16.1f} m   "
              f"drift {drift:7.2f} m/frame  ({ppm:+.2f} ppm)")
    print("   -> free-running crystals. Raw TDoA is impossible without correction.")

    # ── 3. can the reference broadcasts fix it? ──────────────────────────
    print("\n3. Leave-one-out clock prediction from the reference broadcasts")
    for a, b in pairs:
        f, v = series(a, b, args.ref)
        if len(f) < 40:
            continue
        for W in (2, 4):
            res = []
            for i in range(W, len(f) - W):
                idx = list(range(i - W, i)) + list(range(i + 1, i + W + 1))
                if f[idx].max() - f[idx].min() > 4 * W:
                    continue
                c = np.polyfit(f[idx] - f[i], v[idx], 1)
                res.append(abs(v[i] - np.polyval(c, 0.0)))
            if len(res) < 20:
                continue
            res = np.array(res)
            print(f"   {hex(a)}-{hex(b)}  window +/-{W}  residual median "
                  f"{np.median(res)*100:6.2f} cm   p95 {np.percentile(res,95)*100:6.1f} cm")
    print("   -> software clock sync works, at the few-centimetre level.")
    print("      A tight window beats a wide one: correct locally and often.")

    # ── 4. corrected tag measurements ────────────────────────────────────
    print("\n4. Tag blinks after clock correction")
    for a, b in pairs:
        fr_r, v_r = series(a, b, args.ref)
        fr_t, v_t = series(a, b, args.tag)
        if len(fr_r) < 40 or len(fr_t) < 40:
            continue
        f_ok, c_ok = [], []
        for ft, vt in zip(fr_t, v_t):
            near = np.abs(fr_r - ft) <= 3
            if near.sum() < 4:
                continue
            c = np.polyfit(fr_r[near] - ft, v_r[near], 1)
            f_ok.append(ft)
            c_ok.append(vt - np.polyval(c, 0.0))
        f_ok, c_ok = np.array(f_ok), np.array(c_ok)
        if len(c_ok) < 20:
            continue
        d1 = np.diff(c_ok)[np.diff(f_ok) == 1]
        sd, lag1 = np.std(c_ok), np.std(d1)
        print(f"   {hex(a)}-{hex(b)}  n={len(c_ok):4d}  median {np.median(c_ok):7.2f} m   "
              f"sd {sd*100:5.1f} cm   frame-to-frame {lag1*100:5.1f} cm   "
              f"ratio {lag1/(np.sqrt(2)*sd):.2f}")
    print("   ratio ~1 = pure noise, << 1 = smooth real motion.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
