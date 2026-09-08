# RTSP capture-timestamp probe — LIT factory, 4 Sep 2026

Measured on the factory Wi-Fi (laptop 40.0.0.24 → Reolink NVR 50.0.0.2:554),
SAL VPN disconnected. RTSP had to be enabled first in the Reolink web UI
(Network → Advanced → Server Settings); it was off, which is why every earlier
attempt timed out on port 554.

## Result: RTCP Sender Reports are present on all four channels

| Cam | SRs | min clock offset | SR spread | frame delay p05 | median | p95 | fps |
|-----|-----|------------------|-----------|-----------------|--------|-----|-----|
| 01  | 12  | +68 ms           | 67.1 ms   | 88.4 ms         | 105.6  | 253.2 | 24.1 |
| 02  |  4  | +80 ms           | 41.4 ms   | 87.1 ms         | 103.0  | 235.0 | 22.8 |
| 03  |  4  | +72 ms           | 69.6 ms   | 88.1 ms         | 105.2  | 246.0 | 22.7 |
| 04  |  4  | +83 ms           | 47.3 ms   | 89.7 ms         | 106.7  | 251.2 | 23.0 |

SR interval 4.2–5.8 s. `camera NTP epoch sane: True` on all four.
Stream: H.264 90 kHz, 25 fps configured, `a=control:trackID=1`, TCP interleaved.

## What is settled

* **The "Reolink supplies no transmit-side timestamp" claim is false.** Per-frame
  capture time is recoverable as `ntp_sr + (rtp_ts - rtp_sr) / 90000`.
  It is absent from the `/flv?…bcs` endpoint the pipeline currently uses, which
  is why it looked absent.
* **The camera clocks are disciplined.** They track pool.ntp.org, so the factory
  network permits outbound NTP — unlike the SAL subnet the VM sits on.
* **The four cameras agree with each other to ~15 ms** (min offsets 68–83 ms),
  so cross-camera fusion is already time-coherent at that level.

## What is NOT settled

The observed "delay" = true latency + camera-vs-laptop clock offset, and this
data cannot separate them. Bounds: clock offset <= ~68 ms, so

    true glass-to-host latency  is in  [~38 ms, ~106 ms]

with ~40 ms (one frame period at 25 fps) the most likely value.

Jitter p05→p95 is ~160 ms, but **measured over Wi-Fi**: fps came out 22.7–24.1
against 25 configured, i.e. 4–9% of frames lost. On the wired fusion host both
numbers should improve. Some of the tail is also the probe's own single-threaded
read loop, since RTCP shares one TCP connection with the video.

Re-run wired, from the machine that will actually do the fusion, before quoting
any latency number in a paper.

## Config note

Camera NTP was set to sync every 1440 min (24 h); changed to 60 min during this
visit. Server is pool.ntp.org, port 123, auto-synchronize on.
