#!/usr/bin/env python3
"""Publish camera localisation to MQTT as a `lit.fusion.v1` source.

Runs the four RTSP camera workers, fuses their detections, and publishes one
message per fused measurement to `lit/fusion/v1/camera`. The collector on the
factory laptop subscribes, stamps arrival, and the difference is the camera
chain's true end-to-end latency.

    python3 publish_camera.py --dry-run          # print, publish nothing
    python3 publish_camera.py --broker 10.0.0.3  # go live

Two things it does differently from the existing pipeline, both because real
capture timestamps now exist:

**Buckets by capture time, not arrival time.** The old consumer gathered
whatever showed up in a fixed 0.25 s wall-clock window. Four cameras have
different latencies, so detections of the SAME instant landed in different
buckets and detections of different instants got averaged together. Grouping
by shutter time fixes that — the thing being fused is now genuinely
simultaneous.

**Waits for stragglers.** A bucket is held open for `--max-wait` (default
150 ms) so a slower camera's view of the same instant can still join. That
delays publication, but not `t_valid` — the measurement keeps the time it
actually happened, and the added delay shows up honestly in `internal_s`.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue as queuelib
import signal
import subprocess
import sys
import time
from collections import defaultdict, deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_CAM_REPO = os.environ.get("CAM_TRACKING_REPO", "/home/sathishkumara/Cam-tracking-LIT")
if _CAM_REPO not in sys.path:
    sys.path.insert(0, _CAM_REPO)


def clock_is_synced() -> bool:
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized",
                              "--value"], capture_output=True, text=True,
                             timeout=5).stdout.strip()
        return out.lower() == "yes"
    except Exception:
        return False


class Velocity:
    """Least-squares velocity over a short window of fused positions.

    Derived, not measured — a straight line through the last `window_s` of
    positions. Reported so a consumer can extrapolate, and flagged as derived
    so nobody mistakes it for an observation.
    """

    def __init__(self, window_s: float = 0.6, min_pts: int = 4):
        self.window_s = window_s
        self.min_pts = min_pts
        self._h: dict[str, deque] = defaultdict(lambda: deque(maxlen=60))

    def update(self, cls: str, t: float, x: float, y: float):
        h = self._h[cls]
        h.append((t, x, y))
        while h and h[0][0] < t - self.window_s:
            h.popleft()
        if len(h) < self.min_pts:
            return None, None
        a = np.array(h)
        span = a[-1, 0] - a[0, 0]
        if span < 0.1:
            return None, None
        t0 = a[:, 0] - a[0, 0]
        vx = float(np.polyfit(t0, a[:, 1], 1)[0])
        vy = float(np.polyfit(t0, a[:, 2], 1)[0])
        return round(vx, 4), round(vy, 4)


def fuse_bucket(dets: list, singletons: set, method: str):
    """Collapse one capture-time bucket into one measurement per class.

    Singleton classes (exactly one omron, one agilox exist) collapse
    unconditionally, so occlusion-split fragments cannot become two objects.
    Everything else clusters by proximity.
    """
    from src.fusion import _distance_clusters, _pick_position     # noqa: E402
    out = []
    by_class: dict[str, list] = defaultdict(list)
    for d in dets:
        by_class[d["class"]].append(d)
    for cls, group in by_class.items():
        clusters = [group] if cls in singletons else _distance_clusters(group, 0.6)
        for cl in clusters:
            world, _rep = _pick_position(cl, method)
            out.append((cls, world, cl))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera-config", nargs="*",
                    default=[os.path.join(_CAM_REPO, "config", "cameras", f"cam{i}.yaml")
                             for i in (1, 2, 3, 4)])
    ap.add_argument("--factory-config",
                    default=os.path.join(_CAM_REPO, "config", "factory.yaml"))
    ap.add_argument("--broker", default="auto")
    ap.add_argument("--port", type=int, default=1833)
    ap.add_argument("--topic", default="lit/fusion/v1/camera")
    ap.add_argument("--max-wait", type=float, default=0.15,
                    help="seconds a capture-time bucket stays open for stragglers")
    ap.add_argument("--bucket", type=float, default=0.04,
                    help="capture-time bucket width (one frame period at 25 fps)")
    ap.add_argument("--classes", nargs="*", default=["omron", "agilox"],
                    help="only publish these classes")
    ap.add_argument("--dry-run", action="store_true",
                    help="print messages instead of publishing")
    ap.add_argument("--substream", action="store_true")
    ap.add_argument("--rtsp-endpoint",
                    help="host[:port] the cameras answer RTSP on, e.g. "
                         "50.0.0.2:554 on the factory LAN, or "
                         "193.171.203.67:8502 through the SAL proxy")
    args = ap.parse_args()

    from endpoints import find_rtsp, resolve_broker            # noqa: E402
    if not args.dry_run:
        args.broker, args.port = resolve_broker(args.broker, args.port)
    if not args.rtsp_endpoint:
        found = find_rtsp()
        if found is None:
            raise SystemExit("no camera RTSP endpoint reachable — see endpoints.py")
        args.rtsp_endpoint = f"{found[0]}:{found[1]}"
        print(f"cameras: {args.rtsp_endpoint}  ({found[2]})")

    import multiprocessing as mp
    import yaml
    from camera_worker_rtsp import camera_worker                  # noqa: E402

    factory = yaml.safe_load(open(args.factory_config))
    singletons = set(factory.get("fusion", {}).get("singleton_classes", []) or [])
    method = factory.get("fusion", {}).get("singleton_position_method",
                                           "confidence_weighted")

    client = None
    if not args.dry_run:
        import paho.mqtt.client as mqtt
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             f"lit-cam-pub-{os.getpid()}")
        client.connect(args.broker, args.port, 30)
        client.loop_start()
        print(f"publishing to {args.broker}:{args.port} topic {args.topic}")
    else:
        print("DRY RUN — nothing will be published")

    synced = clock_is_synced()
    if not synced:
        print("!! this machine's clock is NOT NTP-synced. t_valid still comes from\n"
              "   the camera's own clock, so the data stays valid — but check the\n"
              "   collector's clock, because that is what the latency is measured\n"
              "   against.")

    q: "mp.Queue" = mp.Queue()
    stop = mp.Event()
    procs = []
    for path in args.camera_config:
        cfg = yaml.safe_load(open(path))
        p = mp.Process(target=camera_worker, args=(cfg, factory, q, stop),
                       kwargs={"substream": args.substream,
                               "rtsp_endpoint": args.rtsp_endpoint}, daemon=True)
        p.start()
        procs.append((cfg.get("id"), p))
    print(f"{len(procs)} camera worker(s) starting…")

    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    buckets: dict[int, list] = defaultdict(list)
    vel = Velocity()
    seq = 0
    published = 0
    dropped_late = 0
    last_report = time.time()
    per_cam_seen: dict = defaultdict(int)

    try:
        while not stop.is_set():
            # -- drain ------------------------------------------------
            try:
                while True:
                    d = q.get_nowait()
                    if d.get("class") not in args.classes:
                        continue
                    tv = d.get("capture_host_s") or d.get("ts")
                    if tv is None:
                        continue
                    key = int(round(float(tv) / args.bucket))
                    buckets[key].append(d)
                    per_cam_seen[d.get("cam")] += 1
            except queuelib.Empty:
                pass

            # -- flush buckets that have waited long enough -----------
            now = time.time()
            ready = [k for k in buckets
                     if now - (k * args.bucket) > args.max_wait]
            for k in sorted(ready):
                dets = buckets.pop(k)
                for cls, world, contributors in fuse_bucket(dets, singletons, method):
                    # t_valid weighted the same way the position was, so the
                    # timestamp describes the same instant as the coordinates.
                    w = np.array([max(0.0, float(c.get("conf", 0.0)))
                                  for c in contributors])
                    # Which clock t_valid is expressed on matters, and the right
                    # answer depends on THIS machine.
                    #
                    #   capture_host_s  - all cameras normalised onto this host's
                    #       clock, so inter-camera clock differences cancel. Best
                    #       for fusion, but only absolutely correct if this host
                    #       is NTP-synced.
                    #   capture_camera_s - the camera's own NTP-disciplined clock.
                    #       Absolutely correct wherever this runs, but the four
                    #       cameras differ from each other by ~15 ms (measured).
                    #
                    # The collector computes latency against ITS clock, so an
                    # undisciplined host here would poison every latency in the
                    # dataset. Prefer the host clock only when it is trustworthy.
                    key = "capture_host_s" if synced else "capture_camera_s"
                    ts = np.array([float(c.get(key) or c.get("capture_host_s")
                                         or c["ts"]) for c in contributors])
                    t_valid = float(np.average(ts, weights=w) if w.sum() > 0
                                    else ts.mean())
                    vx, vy = vel.update(cls, t_valid, world[0], world[1])
                    seq += 1
                    msg = {
                        "schema": "lit.fusion.v1", "source": "camera",
                        "seq": seq,
                        "t_valid": round(t_valid, 6),
                        "t_published": round(time.time(), 6),
                        "clock_synced": synced,
                        "t_valid_clock": "host_ntp" if synced else "camera_ntp",
                        "x": round(float(world[0]), 4),
                        "y": round(float(world[1]), 4),
                        "frame": "factory_interior",
                        "class": cls,
                        "vx": vx, "vy": vy,
                        "n_cams": len({c["cam"] for c in contributors}),
                        "cams": sorted({int(c["cam"]) for c in contributors}),
                        "conf": round(max(float(c.get("conf", 0.0))
                                          for c in contributors), 3),
                        "t_frame_arrived": round(min(float(c["arrival_host_s"])
                                                     for c in contributors), 6),
                        "t_detect_done": round(max(float(c["detect_done_s"])
                                                   for c in contributors), 6),
                        "per_cam": [{
                            "cam": int(c["cam"]),
                            "px": round(float(c["pixel"][0]), 1),
                            "py": round(float(c["pixel"][1]), 1),
                            "x": round(float(c["world"][0]), 4),
                            "y": round(float(c["world"][1]), 4),
                            "conf": round(float(c.get("conf", 0.0)), 3),
                            "t_valid": round(float(c.get(key) or c["ts"]), 6),
                        } for c in contributors],
                    }
                    payload = json.dumps(msg, separators=(",", ":"))
                    if client is not None:
                        client.publish(args.topic, payload, qos=0, retain=False)
                    elif published < 5 or published % 50 == 0:
                        print(payload[:400])
                    published += 1

            # a bucket far in the past means a camera is lagging badly
            cutoff = now - 5.0
            for k in [k for k in buckets if k * args.bucket < cutoff]:
                dropped_late += len(buckets.pop(k))

            if now - last_report > 5.0:
                last_report = now
                rates = " ".join(f"cam{c}:{n}" for c, n in sorted(per_cam_seen.items()))
                print(f"  published {published}  open buckets {len(buckets)}  "
                      f"late-dropped {dropped_late}   [{rates}]")
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for _cid, p in procs:
            p.join(timeout=5)
        if client is not None:
            client.loop_stop()
            client.disconnect()
    print(f"\nstopped. published {published} messages, dropped {dropped_late} late "
          f"detections")
    return 0


if __name__ == "__main__":
    sys.exit(main())
