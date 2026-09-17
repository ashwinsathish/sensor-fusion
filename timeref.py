#!/usr/bin/env python3
"""The factory's reference clock, and how far this machine is from it.

Measured on 16 Sep 2026 by the factory-side agent:

    10.0.0.2   "NTP-server-host", stratum-3 NTP server on the factory LAN
    10.0.0.3   Server2, PTP-locked (phc2sys offset ~2 ns); agrees with
               10.0.0.2 to -0.05 ms
    anchor Pi  public pool NTP, +0.15 ms vs 10.0.0.2
    UpBoard    (the Omron collector, i.e. the ground truth) public
               ntp.ubuntu.com over 5G: ~30 ms behind, 90 ms jitter,
               last sample rejected — the worst clock in the system

So **10.0.0.2 is the reference clock** for the dataset. Every machine that
stamps a time should sync to it.

This module queries it directly with SNTP, so the offset is a number rather
than timedatectl's yes/no. Minimum-delay filtering over several queries, as
real NTP does: the sample with the shortest round trip has the least
asymmetric-path error.

    python3 timeref.py                 # how far is this machine from 10.0.0.2
    python3 timeref.py --server x.y.z.w
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

REFERENCE_NTP = "10.0.0.2"
NTP_EPOCH = 2208988800


def _query(server: str, timeout: float = 1.5, port: int = 123):
    """One SNTP exchange -> (offset_s, delay_s). offset = server - local."""
    pkt = bytearray(48)
    pkt[0] = 0x23                        # LI=0, VN=4, mode=3 (client)
    t1 = time.time()
    frac = int((t1 % 1) * 2**32)
    struct.pack_into("!II", pkt, 40, int(t1) + NTP_EPOCH, frac)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        s.sendto(pkt, (server, port))
        data, _ = s.recvfrom(64)
    t4 = time.time()
    if len(data) < 48:
        raise ValueError("short NTP reply")

    def ts(off):
        sec, fr = struct.unpack_from("!II", data, off)
        return sec - NTP_EPOCH + fr / 2**32

    t2, t3 = ts(32), ts(40)              # server receive, server transmit
    offset = ((t2 - t1) + (t3 - t4)) / 2
    delay = (t4 - t1) - (t3 - t2)
    return offset, delay


def measure(server: str = REFERENCE_NTP, samples: int = 8, timeout: float = 1.5,
            port: int = 123):
    """Best offset over several queries. Returns dict or None if unreachable."""
    good = []
    for _ in range(samples):
        try:
            good.append(_query(server, timeout, port))
        except (OSError, ValueError):
            pass
        time.sleep(0.05)
    if not good:
        return None
    good.sort(key=lambda r: r[1])        # least-delayed first
    offset, delay = good[0]
    offs = [o for o, _ in good]
    return {
        "server": server,
        "offset_s": offset,              # add this to local time to get reference time
        "delay_s": delay,
        "n": len(good),
        "spread_ms": (max(offs) - min(offs)) * 1000,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default=REFERENCE_NTP)
    ap.add_argument("--samples", type=int, default=10)
    args = ap.parse_args()
    r = measure(args.server, args.samples)
    if r is None:
        print(f"{args.server}: no NTP reply. Not on the factory network, or UDP/123 "
              "is blocked on this path.")
        return 1
    print(f"reference   {r['server']}")
    print(f"offset      {r['offset_s']*1000:+.2f} ms   (this machine is "
          f"{'behind' if r['offset_s'] > 0 else 'ahead'} by {abs(r['offset_s'])*1000:.2f} ms)")
    print(f"round trip  {r['delay_s']*1000:.2f} ms   spread {r['spread_ms']:.2f} ms   "
          f"n={r['n']}")
    if abs(r["offset_s"]) < 0.002:
        print("good — within 2 ms of the factory reference")
    elif abs(r["offset_s"]) < 0.010:
        print("usable, but point this machine's NTP at 10.0.0.2: tools/set_ntp.sh")
    else:
        print("too far off for millisecond timing — run tools/set_ntp.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
