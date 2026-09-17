#!/usr/bin/env python3
"""Find the broker and the cameras, wherever this machine happens to be.

The same services are reachable at different addresses depending on which
network you are on, and getting it wrong wastes the first twenty minutes of
every session. So: try the known candidates, in the order that is fastest and
most direct, and use the first that answers.

    from endpoints import find_broker, find_rtsp
    host, port = find_broker()

    python3 endpoints.py          # report what this machine can reach
"""

from __future__ import annotations

import socket
import sys

# The factory machines, mapped 9 Sep 2026 by inspecting the router's
# port-forward table and then confirming on the machines themselves:
#
#   10.0.0.3    oic-server2   MQTT broker (1883), UWB code checkout,
#                             orchestrator/node-controller (tmux services:1)
#   10.0.0.2    NTP-server-host  FACTORY REFERENCE CLOCK (stratum-3 NTP), and the
#                             ROS central node the UWB solver reads (port 8000)
#   40.0.0.37   sal-UPN-APL01 the Omron status collector — an UpBoard, NOT a
#                             Raspberry Pi. Runs DataCollector.py from
#                             ~/workspace/repos/iws-testbed (branch ashwin-fix)
#                             in tmux session `omron`, user `sal`.
#   50.0.0.2    Reolink NVR   RTSP 554
#
# Ordered best-first. Direct beats forwarded beats proxied.
BROKERS = [
    ("10.0.0.3", 1883, "factory LAN, direct"),
    ("127.0.0.1", 1833, "local mqtt_tunnel.py (SAL VM)"),
    ("193.171.203.67", 1833, "public, port-forwarded"),
]

REFERENCE_NTP = "10.0.0.2"
ROS_CENTRAL = ("10.0.0.2", 8000)

# localization_gui.py serves its websocket on port 8000 of whatever machine
# runs it (the README's 8001 is wrong, and 8001 on Server2 is the
# orchestrator). NOT 10.0.0.2:8000 — that is the ROS central node's API, which
# the solver reads FROM. The feat/tag_update_rate solver publishes to MQTT
# anyway, so the websocket is only a fallback.
UWB_WS = [
    ("127.0.0.1", 8000, "localization_gui.py on this machine"),
    ("10.0.0.3", 8000, "localization_gui.py on Server2"),
    ("193.171.203.67", 8500, "public, port-forwarded"),
]

RTSP = [
    ("50.0.0.2", 554, "factory LAN, direct"),
    ("193.171.203.67", 8502, "public, port-forwarded (needs SF_RTSP_PROXY from SAL)"),
]


def reachable(host: str, port: int, timeout: float = 2.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def _first(cands, timeout):
    for host, port, why in cands:
        if reachable(host, port, timeout):
            return host, port, why
    return None


def find_broker(timeout: float = 2.5):
    """-> (host, port, description) or None."""
    return _first(BROKERS, timeout)


def find_rtsp(timeout: float = 2.5):
    """-> (host, port, description) or None."""
    return _first(RTSP, timeout)


def find_uwb_ws(timeout: float = 2.5):
    """The UWB localisation server's websocket, if one is running anywhere."""
    return _first(UWB_WS, timeout)


def resolve_broker(arg: str | None, port: int | None = None):
    """Turn a --broker argument into (host, port).

    'auto' (or nothing) searches; anything else is used as given, so an
    explicit address always wins.
    """
    if arg and arg != "auto":
        return arg, (port or 1883)
    found = find_broker()
    if found is None:
        raise SystemExit(
            "no MQTT broker reachable. Tried:\n  " +
            "\n  ".join(f"{h}:{p}  ({w})" for h, p, w in BROKERS) +
            "\nOn the SAL VM, start ~/mqtt_tunnel.py first.")
    host, p, why = found
    print(f"broker: {host}:{p}  ({why})")
    return host, p


def main() -> int:
    print("what this machine can reach\n")
    print("  MQTT broker")
    any_broker = False
    for host, port, why in BROKERS:
        up = reachable(host, port)
        any_broker = any_broker or up
        print(f"    {'✓' if up else '·'} {host}:{port:<6} {why}")
    print("\n  UWB localisation server (websocket)")
    for host, port, why in UWB_WS:
        up = reachable(host, port)
        print(f"    {'✓' if up else '·'} {host}:{port:<6} {why}")
    print("\n  ROS central node (what the UWB solver reads)")
    up = reachable(*ROS_CENTRAL)
    print(f"    {'✓' if up else '·'} {ROS_CENTRAL[0]}:{ROS_CENTRAL[1]}")
    print("\n  cameras (RTSP)")
    for host, port, why in RTSP:
        up = reachable(host, port)
        print(f"    {'✓' if up else '·'} {host}:{port:<6} {why}")

    b = find_broker(); r = find_rtsp(); w = find_uwb_ws()
    print("\n  chosen:")
    print(f"    broker  {b[0]}:{b[1]}" if b else "    broker  NONE REACHABLE")
    print(f"    rtsp    {r[0]}:{r[1]}" if r else "    rtsp    NONE REACHABLE")
    print(f"    uwb ws  {w[0]}:{w[1]}" if w else "    uwb ws  none running")
    if not any_broker:
        print("\n  Without a broker nothing can be recorded. On the SAL VM run"
              "\n  ~/mqtt_tunnel.py first; on the factory network check the"
              "\n  laptop is actually on the OIC wifi.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
