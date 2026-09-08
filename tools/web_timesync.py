#!/usr/bin/env python3
"""Stopgap clock discipline for a machine whose NTP is firewalled.

This is NOT a replacement for NTP. It is a holding action: it drags the clock
back to within a few hundred milliseconds of true time on a schedule, so an
undisciplined VM does not silently slide by seconds per day while a firewall
request works its way through IT.

It reads the `Date:` header from several large HTTPS servers. That header has
1-second resolution, so a single reading is useless — but each reply brackets
the true time ("it was in [S, S+1) somewhere between when I sent and when I
received"), and intersecting many brackets pins the offset to ~150 ms.

Safety rails:
  • refuses to step by more than --max-step seconds (a garbage header from one
    server cannot throw the clock into next week)
  • requires agreement between at least two independent servers
  • does nothing at all once real NTP starts working, so it quietly retires
    itself the moment the proper fix lands

Usage:
    python3 web_timesync.py                 # report only, no changes
    sudo python3 web_timesync.py --apply    # step the clock if needed
"""

from __future__ import annotations

import argparse
import email.utils
import os
import subprocess
import sys
import time

URLS = ["https://www.google.com", "https://cloudflare.com", "https://www.microsoft.com"]


def ntp_is_working() -> bool:
    """True once the machine has a real, disciplined time source."""
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized",
                              "--value"], capture_output=True, text=True,
                             timeout=5).stdout.strip()
        return out.lower() == "yes"
    except Exception:
        return False


def bracket(url: str, seconds: float):
    """Intersect Date-header brackets. Returns (n, lo, hi) on (true - local)."""
    lo, hi, n = -1e9, 1e9, 0
    end = time.time() + seconds
    while time.time() < end:
        t0 = time.time()
        try:
            out = subprocess.run(["curl", "-sI", "--max-time", "5", url],
                                 capture_output=True, text=True, timeout=6).stdout
        except Exception:
            continue
        t1 = time.time()
        hdr = [l for l in out.splitlines() if l.lower().startswith("date:")]
        if not hdr:
            continue
        try:
            S = email.utils.parsedate_to_datetime(
                hdr[0].split(":", 1)[1].strip()).timestamp()
        except Exception:
            continue
        n += 1
        lo = max(lo, S - t1)
        hi = min(hi, S + 1 - t0)
        if hi <= lo:                      # server stepped mid-measurement
            lo, hi = -1e9, 1e9
            n = 0
        time.sleep(0.12)
    return n, lo, hi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="actually step the clock")
    ap.add_argument("--seconds", type=float, default=15.0, help="poll time per server")
    ap.add_argument("--threshold", type=float, default=0.05,
                    help="only step when off by more than this many seconds")
    ap.add_argument("--max-step", type=float, default=60.0,
                    help="refuse to step further than this (safety rail)")
    args = ap.parse_args()

    if ntp_is_working():
        print("NTP is synchronised — nothing to do. This stopgap has retired itself.")
        return 0

    results = []
    for url in URLS:
        n, lo, hi = bracket(url, args.seconds)
        if n >= 5:
            results.append((url, (lo + hi) / 2, (hi - lo) / 2, n))
            print(f"  {url:32s} n={n:3d}  offset {(lo+hi)/2:+.3f} s  "
                  f"+/-{(hi-lo)/2*1000:.0f} ms")
        else:
            print(f"  {url:32s} unreachable")

    if len(results) < 2:
        print("fewer than two servers agreed — refusing to touch the clock.")
        return 1

    offs = sorted(r[1] for r in results)
    spread = offs[-1] - offs[0]
    if spread > 1.0:
        print(f"servers disagree by {spread:.2f} s — refusing to touch the clock.")
        return 1

    corr = offs[len(offs) // 2]           # median across servers
    res = max(r[2] for r in results)
    print(f"\nlocal clock is {-corr:+.3f} s off true time (+/-{res*1000:.0f} ms)")

    if abs(corr) < args.threshold:
        print(f"within {args.threshold*1000:.0f} ms — leaving it alone.")
        return 0
    if abs(corr) > args.max_step:
        print(f"correction {corr:+.1f} s exceeds --max-step {args.max_step} s. "
              "Refusing. Investigate by hand.")
        return 1
    if not args.apply:
        print("report only. Re-run with --apply (as root) to step the clock.")
        return 0
    if os.geteuid() != 0:
        print("--apply needs root.")
        return 1

    target = time.time() + corr
    r = subprocess.run(["date", "-u", "-s", "@%.3f" % target],
                       capture_output=True, text=True)
    print(f"stepped {corr:+.3f} s → {(r.stdout or r.stderr).strip()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
