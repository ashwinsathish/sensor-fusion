# Camera capture timestamps

Replaces the guesswork in Cam-tracking-LIT's frame timing with the camera's own
capture clock, recovered from RTCP Sender Reports.

| | old worker | this |
|---|---|---|
| source | `/flv?…bcs` via OpenCV | RTSP via own client + PyAV |
| `ts` means | when the consumer loop picked the frame up | **when the shutter opened** |
| lag | unknown, unbounded, unlogged | measured per frame, split into transport vs inference |

## Files

- **`rtsp_source.py`** — RTSP session, RFC 6184 depacketisation, H.264 decode,
  per-frame capture time. Reuses the RTSP client from `../tools/rtsp_clock_probe.py`.
- **`camera_worker_rtsp.py`** — drop-in replacement for `src/camera_worker.py`.
  Same signature and queue contract, so `run.py` needs a one-line import swap.
- **`test_rtsp_source.py`** — encode → RTP → depacketise → decode round trip.

## Where this can run — read this first

**Not on the SAL VM.** Its only route out is the HTTP CONNECT proxy
(`lnzproxy01:3128`), which returns **403 on port 554**, and `50.0.0.2` is not
routable from the SAL side at all — only `193.171.203.67`, and RTSP is not
forwarded there. Verified 4 Sep 2026.

This must run on a machine on the factory network (wired preferred), or
someone must forward the NVR's 554 to a port the proxy allows. `8554` is
*not* blocked by the proxy — it just has nothing behind it — so a forward
`193.171.203.67:8554 → 50.0.0.2:554` would open the path.

Also: **RTSP was switched off on the NVR** until 4 Sep. If port 554 stops
answering, check Reolink → Network → Advanced → Server Settings.

## Use

```bash
pip install av                      # already installed in Cam-tracking-LIT/.venv

# one camera, end to end, with a timing breakdown
python3 camera_worker_rtsp.py /home/sathishkumara/Cam-tracking-LIT/config/cameras/cam1.yaml

# just the video source, no detector
python3 rtsp_source.py "rtsp://admin:PASS@50.0.0.2:554/h264Preview_01_main"

# offline checks (run anywhere)
python3 test_rtsp_source.py
```

To switch the real pipeline over, in `Cam-tracking-LIT/run.py`:

```python
# from src.camera_worker import camera_worker
from camera_worker_rtsp import camera_worker
```

No config changes needed — `rtsp_url_from_source()` derives the RTSP URL from
the existing FLV `source:` field (FLV channels are 0-indexed, RTSP 1-indexed,
so `channel0_main.bcs` → `h264Preview_01_main`). Verified against all four
camera configs.

## What each detection now carries

```
ts                      shutter instant on this host's clock  <- the validity instant
capture_camera_s        same instant on the CAMERA's clock
arrival_host_s          last RTP packet of that frame landed
detect_done_s           YOLO + homography finished
transport_s             camera encode + network
inference_s             your GPU
camera_clock_offset_s   host - camera, minimum-filtered over the last 2 min
```

`ts` keeps its old name deliberately, so fusion, tracker and bridge code needs
no change — the number simply became correct.

## Verified

Round-trip test (`test_rtsp_source.py`, runs without a camera):

```
✓ 32-bit RTP timestamp wrap handled (13.25 h boundary)
✓ lost first fragment discarded cleanly, next NAL still parsed
  packetisation exercised: 54 single-NAL, 19 FU-A fragmented, 2 STAP-A
✓ 24/24 frames recovered, worst mean abs pixel error 1.29 (lossy codec)
✓ frame order preserved (cross-check error 72.2 vs 1.3)
```

The RTSP transport half is proven separately — all four LIT cameras answered
on 4 Sep 2026 with Sender Reports every 4–6 s and NTP-disciplined clocks.

**Not yet verified:** the two halves together against a live camera. That needs
the factory network. Expect `transport_s` around 40 ms and watch whether its
p95 tail is smaller than the ~250 ms measured over Wi-Fi — it should be, on a
wired host with no proxy in the way.

## Caveats

**Resolved 4 Sep 2026:** the FLV and RTSP streams were feature-matched
frame-to-frame and are **pixel identical** — 2560x1440 both, 497/500 inliers,
exact identity transform, zero residual. Same encoder, different transport.
So the existing calibration carries over to RTSP unchanged and
`_scale_to_homography_size` behaves identically on both paths. The odd
1603x852 `homography_image_size` is just the size of the screenshot the
reference points were clicked on; the independent x/y scaling absorbs it.

Separately, the calibrations themselves are weak — three of cam1's reference
points sit outside the captured image, self-consistency is 5-16 cm median and
up to 52 cm, and each camera's points cover only a narrow band of floor. A
parking run fixes that without any clicking: see `../tools/fit_homography.py`.

The decoder attaches the RTP timestamp as the packet PTS, so capture times
survive any frame reordering the decoder does. Frames are still dropped when
inference falls behind — but the timestamp travels with the frame, so dropping
costs throughput instead of corrupting timing.
