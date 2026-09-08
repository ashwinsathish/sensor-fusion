#!/usr/bin/env python3
"""Publish UWB positions to MQTT as a `lit.fusion.v1` source.

The UWB demo serves its fixes over a websocket and never publishes them. This
bridges that gap without modifying the demo: subscribe to its websocket,
convert UWB coordinates into the factory frame, and publish to
`lit/fusion/v1/uwb`.

    python3 publish_uwb.py --dry-run
    python3 publish_uwb.py --ws ws://127.0.0.1:8001/ws --broker 10.0.0.3

**Timestamps.** If the backend has been patched to emit `t_round_s` (see
`patch_backend.py`) that is used as `t_valid` and the UWB latency becomes
measurable. If not, arrival time is used and `t_valid_is_arrival` is set true,
so the dataset says plainly that UWB timing is unknown rather than implying a
precision it does not have.

**Coordinates.** The UWB frame is mirrored on both axes with its origin at
x = 14.8 m in factory metres. Verified: transforming the nine OIC anchors puts
them at x 0.59-38.65, y 0.32-12.46 inside a 39 x 12.5 m hall. Both the raw and
converted values are published.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import subprocess
import sys
import time
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import frames                                              # noqa: E402


def clock_is_synced() -> bool:
    try:
        return subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized",
                               "--value"], capture_output=True, text=True,
                              timeout=5).stdout.strip().lower() == "yes"
    except Exception:
        return False


class AnchorBias:
    """Removes the per-anchor timestamp bias measured in the raw ROS logs.

    The anchor Raspberry Pis do not agree: on the 28 Apr log NODE 03 was
    always earliest and NODE 04 always ~2.75 ms later, each stable to ~0.2 ms.
    "Take the earliest timestamp" therefore inherits whichever node is
    fastest. Tracking the per-anchor offset relative to the running median
    lets that be subtracted, taking UWB timing from ~3 ms to sub-millisecond.

    Only applied when the message says which anchor the timestamp came from.
    """

    def __init__(self, window: int = 400):
        self._h: dict[str, deque] = {}
        self.window = window

    def observe(self, anchor: str, delta: float) -> None:
        self._h.setdefault(anchor, deque(maxlen=self.window)).append(delta)

    def bias(self, anchor: str) -> float:
        h = self._h.get(anchor)
        if not h or len(h) < 20:
            return 0.0
        s = sorted(h)
        return s[len(s) // 2]

    @property
    def known(self) -> dict:
        return {a: round(self.bias(a) * 1000, 2) for a in self._h}


async def run(args) -> int:
    import websockets

    client = None
    if not args.dry_run:
        import paho.mqtt.client as mqtt
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             f"lit-uwb-pub-{os.getpid()}")
        client.connect(args.broker, args.port, 30)
        client.loop_start()
        print(f"publishing to {args.broker}:{args.port} topic {args.topic}")
    else:
        print("DRY RUN — nothing will be published")

    frames.apply_site_overrides()
    synced = clock_is_synced()
    if not synced:
        print("!! this machine's clock is NOT NTP-synced — every UWB t_valid will\n"
              "   inherit that error. Fix it before collecting.")

    seq = 0
    published = 0
    no_time = 0
    bias = AnchorBias()
    last_report = time.time()

    while True:
        try:
            async with websockets.connect(args.ws, ping_interval=5,
                                          ping_timeout=5) as ws:
                print(f"connected {args.ws}")
                while True:
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
                    except asyncio.TimeoutError:
                        now = time.time()
                        if now - last_report > 10:
                            last_report = now
                            print(f"  published {published}  "
                                  f"({no_time} without a source timestamp)  "
                                  f"anchor bias ms {bias.known}")
                        continue
                    now = time.time()
                    try:
                        d = json.loads(raw)
                    except Exception:
                        continue
                    if not isinstance(d, dict) or d.get("type") != "UWB":
                        continue
                    try:
                        xr, yr = float(d["x"]), float(d["y"])
                    except (KeyError, TypeError, ValueError):
                        continue

                    t_round = d.get("t_round_s")
                    is_arrival = t_round is None
                    if is_arrival:
                        no_time += 1
                        t_valid = now
                    else:
                        t_valid = float(t_round)
                        anchor = d.get("anchor_earliest")
                        if anchor:
                            bias.observe(str(anchor), t_valid - now)
                            t_valid -= bias.bias(str(anchor)) - min(
                                (bias.bias(a) for a in bias.known), default=0.0)

                    x, y = frames.uwb_to_factory(xr, yr)
                    seq += 1
                    msg = {
                        "schema": "lit.fusion.v1", "source": "uwb", "seq": seq,
                        "t_valid": round(t_valid, 6),
                        "t_published": round(time.time(), 6),
                        "clock_synced": synced,
                        "t_valid_is_arrival": is_arrival,
                        "x": round(x, 4), "y": round(y, 4),
                        "frame": "factory_interior",
                        "x_uwb": round(xr, 4), "y_uwb": round(yr, 4),
                        "frame_nr": d.get("FrameNr"),
                        "n_anchors": d.get("n_anchors"),
                        "anchor_earliest": d.get("anchor_earliest"),
                        "anchors_used": d.get("anchors_used") or [],
                        "residual_m": d.get("residual_m"),
                        "sigma_m": d.get("sigma_m"),
                        "method": args.method,
                    }
                    payload = json.dumps(msg, separators=(",", ":"))
                    if client is not None:
                        client.publish(args.topic, payload, qos=0, retain=False)
                    elif published < 3 or published % 100 == 0:
                        print(payload[:300])
                    published += 1
        except KeyboardInterrupt:
            break
        except Exception as exc:                            # noqa: BLE001
            print(f"  websocket {type(exc).__name__}: {exc} — retrying")
            await asyncio.sleep(2.0)

    if client is not None:
        client.loop_stop()
        client.disconnect()
    print(f"\nstopped. published {published}, {no_time} without a source timestamp")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ws", default="ws://127.0.0.1:8001/ws")
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1833)
    ap.add_argument("--topic", default="lit/fusion/v1/uwb")
    ap.add_argument("--method", default="twr", choices=["twr", "tdoa"],
                    help="recorded in the payload so the dataset says which "
                         "algorithm produced it")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
