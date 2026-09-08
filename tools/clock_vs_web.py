#!/usr/bin/env python3
"""How wrong is THIS machine's clock, when NTP is blocked?

When UDP/123 is firewalled you cannot ask a time server directly — but every
HTTPS response carries a `Date:` header from a well-disciplined server. The
header only has 1-second resolution, so one sample is useless. Polling it
repeatedly is not: each reply says "the true time was in [S, S+1) somewhere
between when I sent and when I received", and intersecting enough of those
brackets pins the offset to well under 100 ms.

That is enough to answer the only question that matters when two machines
disagree by seconds: WHICH ONE IS WRONG.

Usage:
    python3 clock_vs_web.py                 # 35 s against google.com
    python3 clock_vs_web.py --seconds 60 --url https://cloudflare.com
"""

from __future__ import annotations

import argparse
import email.utils
import subprocess
import sys
import time


def measure(url: str, seconds: float, verbose: bool = False):
    lo, hi = -1e9, 1e9          # bracket on (true_time - local_time)
    n = 0
    end = time.time() + seconds
    while time.time() < end:
        t0 = time.time()
        try:
            out = subprocess.run(
                ["curl", "-sI", "--max-time", "5", url],
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
        # the reply was generated between t0 and t1, and the server's own
        # clock was somewhere in [S, S+1) at that moment.
        lo = max(lo, S - t1)
        hi = min(hi, S + 1 - t0)
        if verbose:
            print(f"  n={n:3d}  bracket {lo:+.3f} .. {hi:+.3f}")
        if hi <= lo:            # inconsistent — server clock stepped
            print("  ! bracket collapsed (server clock stepped?) — restarting")
            lo, hi = -1e9, 1e9
        time.sleep(0.13)
    return n, lo, hi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="https://www.google.com")
    ap.add_argument("--seconds", type=float, default=35.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    print(f"polling {args.url} for {args.seconds:.0f} s …")
    n, lo, hi = measure(args.url, args.seconds, args.verbose)
    if n < 5:
        print("not enough replies — is HTTPS reachable from here?")
        return 1

    err = -(lo + hi) / 2        # local - true
    width = (hi - lo)
    print(f"\nsamples: {n}")
    print(f"this machine reads {err:+.3f} s vs true time  "
          f"(bracket {-hi:+.3f} .. {-lo:+.3f}, width {width*1000:.0f} ms)")
    if abs(err) < 0.5:
        print("→ this clock is fine. If another machine disagrees with it by")
        print("  seconds, THAT machine is the broken one.")
    else:
        print(f"→ this clock is off by {abs(err):.1f} s. Fix it here before")
        print("  blaming any other node in the system.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
