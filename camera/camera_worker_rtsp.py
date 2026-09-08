#!/usr/bin/env python3
"""Drop-in replacement for Cam-tracking-LIT's camera_worker, with real capture times.

Same signature, same output queue contract, same config files. The difference is
what `ts` means.

    old:  ts = time.time() taken in the consumer loop, after the frame came off
          a drop-latest buffer and after undistort. Unknown, unbounded lag.

    new:  ts = when the shutter actually opened, derived from the camera's own
          RTCP Sender Reports and mapped onto this host's clock.

Every detection also carries the breakdown, so the pipeline stops being a black
box you have to estimate a delay for:

    capture_host_s   shutter, on this host's clock      <- the validity instant
    arrival_host_s   last RTP packet of that frame landed
    detect_done_s    YOLO + homography finished
    transport_s      arrival - capture   (camera encode + network)
    inference_s      detect_done - arrival  (your GPU)

Usage mirrors the original:

    from camera_worker_rtsp import camera_worker
    Process(target=camera_worker, args=(cam_cfg, factory_cfg, queue, stop_event))

Standalone check on one camera:

    python3 camera_worker_rtsp.py ../..//Cam-tracking-LIT/config/cameras/cam1.yaml
"""

from __future__ import annotations

import os
import sys
import time
import urllib.parse

import numpy as np

_CAM_REPO = os.environ.get("CAM_TRACKING_REPO", "/home/sathishkumara/Cam-tracking-LIT")
if _CAM_REPO not in sys.path:
    sys.path.insert(0, _CAM_REPO)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rtsp_source import RtspTimedSource            # noqa: E402


# ── config plumbing ──────────────────────────────────────────────────────────

def rtsp_url_from_source(source: str, *, port: int = 554, substream: bool = False,
                         host_override: str | None = None) -> str:
    """Turn the configured Reolink FLV URL into the equivalent RTSP URL.

    The configs point at
        https://50.0.0.2/flv?port=1935&app=bcs&stream=channelN_main.bcs&user=..&password=..
    which carries no capture timestamps at all. The same feed on RTSP does.
    FLV channels are 0-indexed, RTSP channels are 1-indexed.
    """
    if source.startswith("rtsp://"):
        return source
    p = urllib.parse.urlparse(source)
    q = urllib.parse.parse_qs(p.query)
    user = q.get("user", ["admin"])[0]
    password = q.get("password", [""])[0]
    stream = q.get("stream", ["channel0_main.bcs"])[0]

    channel = 0
    if stream.startswith("channel"):
        digits = "".join(c for c in stream[len("channel"):] if c.isdigit())
        channel = int(digits or 0)

    host = host_override or p.hostname
    kind = "sub" if substream else "main"
    creds = f"{urllib.parse.quote(user)}:{urllib.parse.quote(password)}@" if user else ""
    return f"rtsp://{creds}{host}:{port}/h264Preview_{channel + 1:02d}_{kind}"


# ── the worker ───────────────────────────────────────────────────────────────

def camera_worker(cam_cfg, factory_cfg, out_queue, stop_event, *,
                  substream: bool = False, require_clock_s: float = 20.0,
                  rtsp_endpoint=None):
    """Target for multiprocessing.Process. Runs until stop_event is set."""
    from src import config, geometry as geo          # noqa: F401  (Cam-tracking-LIT)
    from src.detector import Detector

    name = cam_cfg.get("name", f"cam{cam_cfg.get('id')}")

    H = cam_cfg.get("homography")
    if H is None:
        print(f"[{name}] No homography — run the calibration tool first. Exiting.")
        return
    H = np.asarray(H, dtype=np.float64)

    K, dist = geo.load_intrinsics(cam_cfg.get("intrinsics"))
    homography_image_size = cam_cfg.get("homography_image_size")
    polygon = factory_cfg["floor"].get("valid_region")

    det_cfg = dict(factory_cfg.get("detector", {}))
    det_cfg.update({k: cam_cfg[k] for k in ("weights", "conf", "device", "imgsz", "classes")
                    if k in cam_cfg})
    detector = Detector(det_cfg)

    # The repo's `stream.endpoints` list resolves the FLV/HTTPS host, which is
    # NOT where RTSP lives — the local tunnel on 127.0.0.1:9443 forwards port
    # 443, not 554. RTSP therefore takes its own endpoint: pass one explicitly,
    # otherwise keep the host written in the camera config.
    if rtsp_endpoint:
        host, _, port = str(rtsp_endpoint).partition(":")
        url = rtsp_url_from_source(cam_cfg["source"], substream=substream,
                                   host_override=host, port=int(port or 554))
    else:
        url = rtsp_url_from_source(cam_cfg["source"], substream=substream)
    safe = url.split("@")[-1]
    print(f"[{name}] RTSP {safe} on device {detector.device}")

    src = RtspTimedSource(url).start()
    if not src.wait_for_clock(require_clock_s):
        # Without a Sender Report there is no capture time, and this whole
        # exercise is pointless — say so loudly rather than silently falling
        # back to arrival timestamps.
        print(f"[{name}] NO RTCP Sender Report within {require_clock_s:.0f} s "
              f"(last error: {src.stats.last_error}). "
              f"Is RTSP enabled on the NVR? Exiting.")
        src.stop()
        return
    print(f"[{name}] capture clock locked, host-camera offset "
          f"{src.stats.clock_offset_s:+.4f} s")

    cam_id = cam_cfg.get("id")
    emitted = 0

    while not stop_event.is_set():
        tf = src.latest()
        if tf is None:
            time.sleep(0.004)
            continue

        frame = geo.undistort_frame(tf.image, K, dist)
        dets = detector.detect(frame)
        detect_done = time.time()

        capture = tf.capture_host_s if tf.capture_host_s is not None else tf.arrival_host_s

        for det in dets:
            stream_pixel = geo.ground_contact_pixel(det["bbox"])
            homography_pixel = _scale_to_homography_size(
                stream_pixel, frame.shape, homography_image_size)
            world = geo.image_to_world(homography_pixel, H)
            if not geo.in_valid_region(world, polygon):
                continue

            out_queue.put({
                "cam": cam_id,
                "class": det["class"],
                "world": (float(world[0]), float(world[1])),
                "pixel": (float(stream_pixel[0]), float(stream_pixel[1])),
                "homography_pixel": (float(homography_pixel[0]), float(homography_pixel[1])),
                "local_id": det["id"],
                "conf": det["conf"],
                # `ts` keeps its name so nothing downstream breaks — but it is
                # now the shutter instant, not "when Python looked at it".
                "ts": capture,
                "capture_host_s": tf.capture_host_s,
                "capture_camera_s": tf.capture_camera_s,
                "arrival_host_s": tf.arrival_host_s,
                "detect_done_s": detect_done,
                "transport_s": (tf.arrival_host_s - capture) if capture else None,
                "inference_s": detect_done - tf.arrival_host_s,
                "camera_clock_offset_s": src.stats.clock_offset_s,
                "rtp_ts": tf.rtp_ts,
            })
            emitted += 1

    src.stop()
    s = src.stats
    print(f"[{name}] stopped. frames {s.frames}, detections {emitted}, "
          f"SRs {s.sender_reports}, dropped {s.dropped_frames}, "
          f"lost fragments {s.lost_fragments}")


def _scale_to_homography_size(pixel, frame_shape, homography_image_size):
    """Map a stream pixel into the image size the homography was fitted on.

    Unchanged from the original worker, and it matters more now: the RTSP main
    stream may not be the same resolution as the FLV feed the homography was
    calibrated against. Same aspect ratio is still required — if the RTSP
    stream has a different one, recalibrate rather than trusting this.
    """
    if not homography_image_size:
        return pixel
    calib_w, calib_h = homography_image_size
    frame_h, frame_w = frame_shape[:2]
    if frame_w <= 0 or frame_h <= 0:
        return pixel
    return np.array([pixel[0] * float(calib_w) / float(frame_w),
                     pixel[1] * float(calib_h) / float(frame_h)], dtype=np.float32)


# ── standalone check ─────────────────────────────────────────────────────────

def main() -> int:
    import argparse
    import multiprocessing as mp
    import queue as queuelib
    import yaml

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cam_config")
    ap.add_argument("--factory", default=os.path.join(_CAM_REPO, "config", "factory.yaml"))
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--sub", action="store_true", help="use the sub-stream")
    args = ap.parse_args()

    cam_cfg = yaml.safe_load(open(args.cam_config))
    factory_cfg = yaml.safe_load(open(args.factory))

    q: "mp.Queue" = mp.Queue()
    stop = mp.Event()
    proc = mp.Process(target=camera_worker,
                      args=(cam_cfg, factory_cfg, q, stop),
                      kwargs={"substream": args.sub})
    proc.start()

    end = time.time() + args.seconds
    lat, n = [], 0
    while time.time() < end:
        try:
            d = q.get(timeout=0.2)
        except queuelib.Empty:
            continue
        n += 1
        if d.get("transport_s") is not None:
            lat.append((d["transport_s"], d["inference_s"]))
        if n % 10 == 0:
            print(f"  {d['class']:8s} ({d['world'][0]:6.2f}, {d['world'][1]:6.2f}) m   "
                  f"transport {1000*d['transport_s']:6.1f} ms   "
                  f"inference {1000*d['inference_s']:6.1f} ms")
    stop.set()
    proc.join(timeout=5)

    if lat:
        tr = sorted(x[0] for x in lat)
        inf = sorted(x[1] for x in lat)
        m = len(tr)
        print(f"\n{n} detections")
        print(f"  camera+network  p05 {1000*tr[m//20]:6.1f}  median "
              f"{1000*tr[m//2]:6.1f}  p95 {1000*tr[int(m*.95)]:6.1f} ms")
        print(f"  YOLO+homography p05 {1000*inf[m//20]:6.1f}  median "
              f"{1000*inf[m//2]:6.1f}  p95 {1000*inf[int(m*.95)]:6.1f} ms")
        print("\nThose two numbers used to be a single unknown. Feed the total "
              "into estimate_delay.py as a prior and check it lands there.")
    else:
        print(f"\n{n} detections, no timing (no Sender Report?)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
