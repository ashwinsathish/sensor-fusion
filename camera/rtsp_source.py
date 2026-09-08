#!/usr/bin/env python3
"""RTSP video source that knows WHEN each frame was captured.

The existing pipeline reads Reolink's `/flv?...bcs` endpoint through OpenCV and
timestamps a frame whenever the consumer loop happens to pick it up. That number
is not a capture time — it is "when Python got around to it", and it sits behind
an unbounded drop-latest buffer.

RTSP carries the real thing. Every RTCP Sender Report pairs the camera's own
wall clock with an RTP timestamp, so any frame's capture instant is

    t_capture_camera = ntp_sr + (rtp_ts - rtp_sr) / 90000

Measured on all four LIT cameras (4 Sep 2026): Sender Reports every 4-6 s,
camera clocks NTP-disciplined and agreeing with each other to ~15 ms.

This module does the whole path itself — RTSP setup, RTP depacketisation
(RFC 6184), H.264 decode via PyAV — because OpenCV's FFmpeg backend gives you
pixels but throws the RTCP mapping away, and this build of OpenCV has no
GStreamer support to fall back on.

    src = RtspTimedSource("rtsp://admin:pw@50.0.0.2:554/h264Preview_01_main")
    src.start()
    while True:
        f = src.latest()
        if f is None:
            time.sleep(0.005); continue
        f.image            # BGR ndarray
        f.capture_host_s   # capture instant, on THIS machine's clock
        f.age_s            # how stale this frame already is

Frames are dropped when inference falls behind, exactly like the old
LatestFrame — but the timestamp travels *with* the frame, so dropping costs
you throughput instead of silently corrupting your timing.
"""

from __future__ import annotations

import collections
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from fractions import Fraction

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from rtsp_clock_probe import RtspClient, parse_rtcp, parse_rtp  # noqa: E402

H264_CLOCK_HZ = 90000
_START_CODE = b"\x00\x00\x00\x01"


# ── RFC 6184 H.264 depacketisation ───────────────────────────────────────────

class H264Depacketizer:
    """RTP payloads -> Annex-B NAL units.

    Handles the three packetisation modes a camera actually uses: single NAL,
    STAP-A aggregation (typically SPS+PPS glued together), and FU-A
    fragmentation (any NAL bigger than the MTU, i.e. every keyframe).
    """

    def __init__(self):
        self._fu_buf: bytearray | None = None
        self.dropped_fragments = 0

    def push(self, payload: bytes) -> list[bytes]:
        if len(payload) < 1:
            return []
        nal_type = payload[0] & 0x1F

        if 1 <= nal_type <= 23:                        # single NAL unit
            self._fu_buf = None
            return [_START_CODE + payload]

        if nal_type == 24:                             # STAP-A
            self._fu_buf = None
            out, off = [], 1
            while off + 2 <= len(payload):
                size = struct.unpack(">H", payload[off:off + 2])[0]
                off += 2
                if size == 0 or off + size > len(payload):
                    break
                out.append(_START_CODE + payload[off:off + size])
                off += size
            return out

        if nal_type == 28:                             # FU-A
            if len(payload) < 2:
                return []
            indicator, header = payload[0], payload[1]
            start, end = header & 0x80, header & 0x40
            if start:
                reconstructed = bytes([(indicator & 0xE0) | (header & 0x1F)])
                self._fu_buf = bytearray(reconstructed)
                self._fu_buf += payload[2:]
                return []
            if self._fu_buf is None:                   # joined mid-fragment
                self.dropped_fragments += 1
                return []
            self._fu_buf += payload[2:]
            if end:
                nal = _START_CODE + bytes(self._fu_buf)
                self._fu_buf = None
                return [nal]
            return []

        # 25-27 (MTAP/STAP-B) and 29 (FU-B) are not used by these cameras.
        return []


class RtpTimestampExtender:
    """Lift the 32-bit RTP timestamp to a monotonic 64-bit counter.

    A 90 kHz clock wraps every ~13.25 hours. Without this, one frame in every
    13 hours would land 13 hours in the past.
    """

    def __init__(self):
        self._last: int | None = None
        self._epoch = 0

    def extend(self, ts32: int) -> int:
        if self._last is not None:
            diff = (ts32 - self._last) & 0xFFFFFFFF
            if diff > 0x80000000:                      # went backwards
                if ts32 > self._last:
                    self._epoch -= 1 << 32
            elif ts32 < self._last:
                self._epoch += 1 << 32
        self._last = ts32
        return self._epoch + ts32


# ── the source ───────────────────────────────────────────────────────────────

@dataclass
class TimedFrame:
    image: object                    # BGR ndarray
    capture_camera_s: float | None   # capture instant on the CAMERA's clock
    capture_host_s: float | None     # same instant, mapped to this host's clock
    arrival_host_s: float            # when the last packet of the AU landed
    rtp_ts: int
    seq: int
    decode_latency_s: float

    @property
    def age_s(self) -> float:
        base = self.capture_host_s if self.capture_host_s is not None else self.arrival_host_s
        return time.time() - base


@dataclass
class SourceStats:
    frames: int = 0
    sender_reports: int = 0
    dropped_frames: int = 0          # decoded but never consumed
    lost_fragments: int = 0
    first_sr_wait_s: float | None = None
    clock_offset_s: float | None = None      # host - camera, minimum filtered
    offset_samples: int = 0
    last_error: str | None = None
    fps: float = 0.0
    _fps_window: collections.deque = field(default_factory=lambda: collections.deque(maxlen=120))


class RtspTimedSource:
    """Background RTSP reader producing decoded frames with capture timestamps."""

    def __init__(self, url: str, *, offset_window_s: float = 120.0,
                 timeout_s: float = 8.0, reconnect: bool = True):
        self.url = url
        self.timeout_s = timeout_s
        self.reconnect = reconnect
        self.offset_window_s = offset_window_s

        self._frame: TimedFrame | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._offsets: collections.deque = collections.deque()   # (t_host, host-camera)
        self.stats = SourceStats()

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> "RtspTimedSource":
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # -- consumer API ------------------------------------------------------
    def latest(self, *, consume: bool = True) -> TimedFrame | None:
        """Most recent decoded frame, or None. Drops any backlog by design."""
        with self._lock:
            f = self._frame
            if consume:
                self._frame = None
            return f

    def wait_for_clock(self, timeout_s: float = 15.0) -> bool:
        """Block until a Sender Report has established the capture-time mapping."""
        end = time.time() + timeout_s
        while time.time() < end and not self._stop.is_set():
            if self.stats.clock_offset_s is not None:
                return True
            time.sleep(0.05)
        return self.stats.clock_offset_s is not None

    # -- clock -------------------------------------------------------------
    def _record_offset(self, t_host: float, sr_ntp: float) -> None:
        """Minimum-filtered (host - camera) offset.

        Each sample is the true offset plus a positive, variable delivery delay
        (RTCP shares the TCP connection with video, so it queues behind frames).
        The minimum over a window is the least contaminated estimate — the same
        trick NTP uses, and the same one pi_clock_ref.py uses on the MQTT feed.
        """
        self._offsets.append((t_host, t_host - sr_ntp))
        cutoff = t_host - self.offset_window_s
        while self._offsets and self._offsets[0][0] < cutoff:
            self._offsets.popleft()
        self.stats.clock_offset_s = min(v for _, v in self._offsets)
        self.stats.offset_samples = len(self._offsets)

    # -- worker ------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as exc:                    # noqa: BLE001
                self.stats.last_error = f"{type(exc).__name__}: {exc}"
            if not self.reconnect or self._stop.is_set():
                break
            time.sleep(1.0)

    def _session(self) -> None:
        import av

        cli = RtspClient(self.url, self.timeout_s)
        st, _, sdp = cli.request("DESCRIBE", extra={"Accept": "application/sdp"})
        if st != 200:
            raise ConnectionError(f"DESCRIBE returned {st} (credentials? stream path?)")

        track, in_video, fmtp = cli.url, False, None
        for line in sdp.splitlines():
            if line.startswith("m="):
                in_video = line.startswith("m=video")
            elif in_video and line.startswith("a=control:"):
                ctl = line.split(":", 1)[1].strip()
                track = ctl if ctl.startswith("rtsp://") else cli.url.rstrip("/") + "/" + ctl
            elif in_video and line.startswith("a=fmtp:"):
                fmtp = line

        st, hd, _ = cli.request(
            "SETUP", track,
            extra={"Transport": "RTP/AVP/TCP;unicast;interleaved=0-1"})
        if st != 200:
            raise ConnectionError(f"SETUP returned {st}")
        cli.session = hd.get("session", "").split(";")[0]

        st, _, _ = cli.request("PLAY", extra={"Range": "npt=0.000-"})
        if st != 200:
            raise ConnectionError(f"PLAY returned {st}")

        codec = av.CodecContext.create("h264", "r")
        tb = Fraction(1, H264_CLOCK_HZ)
        depack = H264Depacketizer()
        extend = RtpTimestampExtender()

        # SPS/PPS from the SDP let the decoder start on the first frame instead
        # of discarding everything until the next in-band parameter set.
        pending = list(_sprop_nals(fmtp))
        sr_map: tuple[float, int] | None = None      # (ntp_unix, extended rtp_ts)
        cur_rtp: int | None = None
        t_open = time.time()

        for channel, payload in cli.read_interleaved():
            if self._stop.is_set():
                break
            now = time.time()

            if channel == 1:
                for sr in parse_rtcp(payload):
                    if not (1.0e9 < sr["ntp_unix"] < 4.0e9):
                        continue                      # uptime clock, not wall clock
                    sr_map = (sr["ntp_unix"], extend.extend(sr["rtp_ts"]))
                    self.stats.sender_reports += 1
                    if self.stats.first_sr_wait_s is None:
                        self.stats.first_sr_wait_s = now - t_open
                    self._record_offset(now, sr["ntp_unix"])
                continue
            if channel != 0:
                continue

            pkt = parse_rtp(payload)
            if pkt is None:
                continue
            hdr = 12 + 4 * pkt["cc"]
            if len(payload) <= hdr:
                continue

            ext_ts = extend.extend(pkt["rtp_ts"])
            if cur_rtp is not None and ext_ts != cur_rtp and pending:
                self._emit(codec, tb, pending, cur_rtp, sr_map, now, pkt["seq"])
                pending = []
            cur_rtp = ext_ts

            pending.extend(depack.push(payload[hdr:]))
            self.stats.lost_fragments = depack.dropped_fragments

            if pkt["marker"] and pending:
                self._emit(codec, tb, pending, ext_ts, sr_map, now, pkt["seq"])
                pending = []

    def _emit(self, codec, tb, nals, ext_rtp, sr_map, arrival, seq) -> None:
        import av

        t0 = time.time()
        try:
            packet = av.Packet(b"".join(nals))
            packet.pts = ext_rtp                       # survives decoder reordering
            packet.time_base = tb
            frames = codec.decode(packet)
        except Exception as exc:                        # noqa: BLE001
            self.stats.last_error = f"decode: {type(exc).__name__}: {exc}"
            return

        for frame in frames:
            pts = frame.pts if frame.pts is not None else ext_rtp
            cap_cam = cap_host = None
            if sr_map is not None:
                ntp_ref, rtp_ref = sr_map
                cap_cam = ntp_ref + (pts - rtp_ref) / H264_CLOCK_HZ
                if self.stats.clock_offset_s is not None:
                    cap_host = cap_cam + self.stats.clock_offset_s

            tf = TimedFrame(
                image=frame.to_ndarray(format="bgr24"),
                capture_camera_s=cap_cam,
                capture_host_s=cap_host,
                arrival_host_s=arrival,
                rtp_ts=pts,
                seq=seq,
                decode_latency_s=time.time() - t0,
            )
            with self._lock:
                if self._frame is not None:
                    self.stats.dropped_frames += 1
                self._frame = tf
            self.stats.frames += 1
            self.stats._fps_window.append(time.time())
            w = self.stats._fps_window
            if len(w) > 5:
                span = w[-1] - w[0]
                self.stats.fps = (len(w) - 1) / span if span > 0 else 0.0


def _sprop_nals(fmtp_line: str | None):
    """SPS/PPS carried in the SDP's sprop-parameter-sets, as Annex-B NALs."""
    if not fmtp_line or "sprop-parameter-sets=" not in fmtp_line:
        return
    import base64
    blob = fmtp_line.split("sprop-parameter-sets=", 1)[1].split(";")[0].strip()
    for part in blob.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            yield _START_CODE + base64.b64decode(part + "===")
        except Exception:                               # noqa: BLE001
            continue


# ── CLI smoke test ───────────────────────────────────────────────────────────

def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Read an RTSP camera with capture timestamps")
    ap.add_argument("url")
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    src = RtspTimedSource(args.url).start()
    print("waiting for the first Sender Report …")
    if not src.wait_for_clock(20.0):
        print(f"no SR within 20 s (last error: {src.stats.last_error})")
        src.stop()
        return 1
    print(f"clock established after {src.stats.first_sr_wait_s:.1f} s, "
          f"host-camera offset {src.stats.clock_offset_s:+.4f} s\n")

    end = time.time() + args.seconds
    lat = []
    while time.time() < end:
        f = src.latest()
        if f is None:
            time.sleep(0.004)
            continue
        if f.capture_host_s is not None:
            lat.append(f.arrival_host_s - f.capture_host_s)
        if src.stats.frames % 25 == 0:
            h, w = f.image.shape[:2]
            print(f"  {w}x{h}  capture->arrival {1000*lat[-1]:6.1f} ms   "
                  f"age {1000*f.age_s:6.1f} ms   {src.stats.fps:.1f} fps   "
                  f"dropped {src.stats.dropped_frames}")
    src.stop()

    lat.sort()
    s = src.stats
    print(f"\nframes {s.frames}  SRs {s.sender_reports}  dropped {s.dropped_frames}  "
          f"lost fragments {s.lost_fragments}")
    if lat:
        n = len(lat)
        print(f"capture->arrival  p05 {1000*lat[n//20]:.1f}  median "
              f"{1000*lat[n//2]:.1f}  p95 {1000*lat[int(n*.95)]:.1f} ms")
    print(f"host-camera offset {s.clock_offset_s:+.4f} s over {s.offset_samples} SRs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
