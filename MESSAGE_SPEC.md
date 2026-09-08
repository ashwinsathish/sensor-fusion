# LIT fusion — MQTT message contract (`lit.fusion.v1`)

Every source publishes JSON to its own topic. One collector on the factory
laptop subscribes to all of them, stamps arrival, and writes the dataset.

```
lit/fusion/v1/camera     published by the YOLO pipeline
lit/fusion/v1/uwb        published by the UWB server
lit/fusion/v1/omron      published by the Omron Raspberry Pi   (ground truth)
lit/fusion/v1/agilox     optional
```

Keep the existing `Omron/status` and `Agilox/status` topics running unchanged —
other things depend on them. These are additional.

## The one field that matters

```
t_valid
```

**The instant the measurement was true**, in UTC unix seconds as a float. Not
when it was computed, not when it was sent. Every argument in this project has
come down to this number being absent or wrong.

| source | what `t_valid` must be |
|---|---|
| camera | the **shutter instant**, from the camera's RTCP Sender Report: `ntp_sr + (rtp_ts - rtp_sr)/90000`. Available now — see `camera/rtsp_source.py`. |
| uwb | the **earliest anchor timestamp** in that frame group, keyed by `frame_nr`. |
| omron | the instant the Pi polled ARCL, i.e. what it already puts in field 0. |

If a source genuinely cannot produce `t_valid`, send `null` and set
`"t_valid_is_arrival": true`. Do not quietly substitute the publish time —
that turns an unknown into a wrong number, which is worse.

## Envelope — every message

```jsonc
{
  "schema":       "lit.fusion.v1",
  "source":       "camera",          // camera | uwb | omron | agilox
  "seq":          12345,             // monotonic per source; lets the collector count drops
  "t_valid":      1788526415.462238, // UTC unix seconds, float
  "t_published":  1788526415.502111, // just before the MQTT publish
  "clock_synced": true,              // this publisher's NTP state, see below
  "x":            12.3456,           // metres, factory floor frame
  "y":             5.6789,
  "frame":        "factory_interior" // name the frame; do not assume
}
```

`t_published - t_valid` is the source's own internal pipeline delay, measured
by the source. The collector adds arrival time, and the difference is the total.
Both numbers are needed: one tells you where the delay is, the other how big
it is.

### `clock_synced`

```python
subprocess.run(["timedatectl","show","-p","NTPSynchronized","--value"],
               capture_output=True, text=True).stdout.strip() == "yes"
```

Publish it in every message. When it is `false`, `t_valid` is meaningless and
the collector must be able to discard that data automatically rather than
someone discovering it three weeks later. This has already happened once in
this project: a workstation that had never synced in four days, off by 4.3 s.

## Per-source additions

### camera

```jsonc
{
  "class":        "omron",
  "vx": 0.42, "vy": -0.11,        // m/s, derived; null if not tracked yet
  "n_cams":       2,               // how many cameras contributed
  "cams":         [1, 3],
  "conf":         0.87,            // max confidence among contributors
  "per_cam": [                     // keep the un-aggregated view
    {"cam": 1, "px": 1204.5, "py": 903.2, "x": 12.30, "y": 5.64,
     "conf": 0.87, "t_valid": 1788526415.462238},
    {"cam": 3, "px":  431.0, "py": 771.9, "x": 12.39, "y": 5.71,
     "conf": 0.62, "t_valid": 1788526415.459102}
  ],
  "t_frame_arrived": 1788526415.487,  // last RTP packet of the frame
  "t_detect_done":   1788526415.498   // YOLO + homography finished
}
```

`per_cam` matters more than it looks. Four cameras have different latencies,
different viewing angles and different accuracy, and once you aggregate them
you can never separate those again. Keeping the parts lets the analysis ask
"which camera is good where", which is the whole point of the trust map.
`t_valid` at the top level should be the earliest contributing shutter.

Cameras seeing nothing publish nothing. Do not send empty messages.

### uwb

```jsonc
{
  "frame_nr":        4277,
  "n_anchors":       7,
  "anchor_earliest": "0x3333",     // WHICH anchor gave t_valid
  "anchors_used":    ["0x1111","0x2222","0x3333","0x4444","0x5555","0x7777","0x9999"],
  "residual_m":      0.08,
  "sigma_m":         0.11,          // 1-sigma from the solver, if available
  "method":          "tdoa"         // tdoa | twr
}
```

`anchor_earliest` is not optional. Measured on the 28 Apr log: the anchor Pis
differ from each other by a **stable per-node bias** — NODE 03 always earliest,
NODE 04 always ~2.75 ms later, each with only ~0.2 ms of scatter. Taking "the
earliest" therefore silently inherits whichever node is fastest. Record which
one and the bias subtracts out, taking UWB timing from ~3 ms to sub-millisecond.

### omron (ground truth)

```jsonc
{
  "theta_deg":  93.0,
  "loc_score":  0.834,             // the robot's own confidence in its SLAM pose
  "status":     "Parking",
  "poll_interval_s": 0.2
}
```

`loc_score` is important. It is the ground truth telling you how much to trust
itself, and it varies with position — near featureless walls it drops. Without
it you cannot separate "the camera was wrong here" from "the ground truth was
wrong here", and those look identical in a heatmap.

## Rules

**Units and frame.** Metres, one agreed floor frame, stated in every message.
The three systems currently use three different frames reconciled by hardcoded
constants like `XSHIFT_OMRON = -5.88`. Publish raw-but-labelled rather than
silently pre-transformed; the collector records what it was given.

**Rates.** Camera ~10-25 Hz, UWB 10 Hz, Omron 5 Hz. Do not resample or align
before publishing. Send each measurement when it happens.

**QoS 0, no retain.** Retained messages arrive stale and will be recorded with
a fresh arrival timestamp, producing a fake latency of minutes.

**One measurement per message.** Do not batch.

## What the collector adds

```
t_recv        arrival at the collector, its own NTP-synced clock
latency_s     t_recv - t_valid          <- total, source to dataset
internal_s    t_published - t_valid     <- inside the source
transport_s   t_recv - t_published      <- network + broker
```

Those three decompose the delay, so when something is slow you know which part
to fix instead of guessing.

## The precondition

`latency_s` is only a latency if the publisher's clock and the collector's
clock agree. Otherwise it is latency plus clock offset, and this project has
already measured clock offsets of **4.3 s** on one machine and **14.5 s of
drift** across an archive of runs.

So, before any collection:

* the collector laptop, the Omron Pi, the UWB server and its anchor Pis, and
  the machine running the camera pipeline all point at **the same NTP server**;
* `timedatectl` says `System clock synchronized: yes` on every one;
* every message carries `clock_synced`.

This is achievable — the factory network permits outbound NTP (proven: the
Reolink cameras track pool.ntp.org and the anchor Pis are already synced). It
is the SAL subnet that blocks it, which is why the collector belongs on the
factory laptop.

Verify with `tools/clock_audit.py live`, and again from the dataset afterwards:
if `latency_s` for any source is negative, or jumps by seconds between runs,
a clock moved and that data is not usable.
