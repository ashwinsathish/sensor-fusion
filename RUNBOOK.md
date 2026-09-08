# Collection day — what to run, where

Three machines. Each runs one thing.

```
  GPU machine (your VM, or the server laptop if it has a GPU)
      publish_camera.py    RTSP -> YOLO -> homography -> MQTT

  UWB server
      localization_gui.py  (patched)   +   publish_uwb.py -> MQTT

  Omron Raspberry Pi
      already publishing Omron/status — nothing to change

  Server laptop  (the collector)
      preflight.py   then   collect.py
```

The collector is the only machine whose clock enters the measurement, because
every latency is `t_recv − t_valid` against its clock. Everything else can be
wrong without harming the data, **as long as it never supplies a timestamp.**

---

## Before anything: 60 seconds that can save the day

On the **server laptop**:

```bash
python3 preflight.py --broker <broker-ip>
```

It checks, in this order: this machine's clock, the broker, the coordinate
transforms, each source's rate and timestamps, whether every source puts the
robot inside the hall, and whether the sources agree with each other about
where the robot is.

**Park the robot somewhere all four cameras and the UWB can see it before you
run this.** Check 6 compares the sources against the ground truth. Parked, they
should agree within about a metre. If one is several metres out, a coordinate
transform is wrong and the whole dataset would be quietly ruined.

Do not collect while it reports a blocking problem. Right now, on the SAL VM,
it correctly reports two:

```
✗ NOT NTP-synchronized
✗ omron(legacy): latency median -988 ms — NEGATIVE
```

The second is caused by the first. **On the server laptop, fix NTP first.**
The factory network permits it — the Reolink cameras and the anchor Pis are
already synced.

```bash
sudo timedatectl set-ntp true
timedatectl            # must say: System clock synchronized: yes
```

---

## 1. Camera publisher — GPU machine

```bash
cd /home/sathishkumara/Cam-tracking-LIT
export SF_RTSP_PROXY='user:pass@lnzproxy01.research.silicon-austria.com:3128'   # only from SAL

python3 /home/sathishkumara/sensor-fusion/camera/publish_camera.py \
    --rtsp-endpoint 193.171.203.67:8502 \
    --broker <broker-ip> --port 1833
```

On the factory LAN drop the proxy line and use `--rtsp-endpoint 50.0.0.2:554`.

Add `--dry-run` first to watch messages without publishing. Expect
`capture clock locked` from each camera within ~20 s; if one never locks, RTSP
is off on the NVR (Reolink → Network → Advanced → Server Settings).

## 2. UWB — UWB server

Use the **`feat/tag_update_rate`** branch of `tdoa_uwb`. That is the one with
real TDoA (`uwb_tdoa_localization_backend.py`), the Kalman filter, the
9-anchor OIC environment, and its own MQTT publisher. `master` has none of it.

The backend is already patched here — verify:

```bash
python3 /home/sathishkumara/sensor-fusion/uwb/patch_backend.py \
        /home/sathishkumara/tdoa_uwb --dry-run
# "already patched" is what you want.   --revert undoes it.
```

Then run it as normal:

```bash
python3 localization_gui.py --env environments/environment_oic9_M2.json --ip <ros-central-ip>
```

It publishes to **`UWB/position`** on `10.0.0.3:1883` by itself — no bridge
needed, the collector reads that topic directly.

`uwb/publish_uwb.py` is only a fallback for the websocket-only setup.

**Dependencies.** `requirements.txt` on this branch is missing `filterpy`,
`shapely` and `scipy`, which `kalman_filter.py` needs. A venv with them is at
`/home/sathishkumara/tdoa_uwb/.venv`. `ranging_utils` (Infineon-internal) is
now an optional import — it is only used by the legacy TWR path, and TDoA uses
scipy directly. `applab_pylib` is still required, because that is how the
backend receives the ROS stream; the UWB server has it.

## 3. Collector — server laptop

```bash
python3 /home/sathishkumara/sensor-fusion/record/collect.py --broker <broker-ip>
```

```
  1   start a PARKING run
  2   start a DRIVING run
  s   stop  (it grades the run immediately)
  q   quit
```

---

## The two runs

**Parking (~20 min).** Drive to 8–10 well-spread spots. At each, stop and hold
still until the on-screen bar fills. Cover every camera's field of view plus a
few places no camera sees. **Turn the robot to a different heading at each
spot** — that separates a real sensor offset from the geometry of where the
camera sees the robot's footprint.

Stationary means latency cannot contaminate anything, so this run answers
"where is each sensor's frame" and "how accurate is it" cleanly. It also
re-derives the camera calibration, which is currently the weakest link.

**Driving (~20 min).** Sharp starts and stops, all over the floor. **Not** a
smooth loop — a constant-speed circuit carries almost no timing information and
the delay estimate comes out meaningless. Aim for 15+ stop/go transitions; the
display counts them.

**Do both twice** if you have time. Not for more data — so the trust map can be
built on one pair and tested on the other. Fitting and evaluating on the same
runs makes any improvement look real when it is not.

---

## Afterwards

```bash
cd /home/sathishkumara/sensor-fusion

# camera calibration, from the parking run
python3 tools/fit_homography.py sessions/<parking_run> \
        --compare /home/sathishkumara/Cam-tracking-LIT/config/cameras/cam*.yaml

# frame alignment per sensor (latency-immune, from the parking run)
python3 tools/estimate_delay.py static sessions/<park>/omron.csv \
        sessions/<park>/camera.csv --out cam_extr.json

# latency, from the driving run, with alignment held fixed
python3 tools/estimate_delay.py delay sessions/<drive>/omron.csv \
        sessions/<drive>/camera.csv --extrinsics cam_extr.json --plot cam_delay.png
```

---

## Coordinate frames

Everything is recorded in **factory interior** metres: origin at the inner wall
corner, x ∈ [0, 39], y ∈ [0, 12.5].

| source | native | transform |
|---|---|---|
| camera | already factory interior | none |
| UWB | mirrored both axes, origin at x = 14.8 | `frames.uwb_to_factory` |
| Omron | raw mm, two-point affine | `frames.omron_to_factory` |
| Agilox | raw mm, offset | `frames.agilox_to_factory` |

All in [frames.py](frames.py), self-tested: the nine UWB anchors transform to
x 0.6–38.7, y 0.3–12.5 inside a 39 × 12.5 m hall. Both raw and converted
coordinates are stored, so a transform can be corrected afterwards without
recollecting.

```bash
python3 frames.py     # self-test
```

---

## If something goes wrong mid-run

- **A sensor row turns red / `STALLED`** — stop the run, fix it, start a new one.
  A run with a dead sensor is not worth driving out.
- **Latency shows as negative** — a publisher's clock moved. Stop; that run's
  timing is unusable.
- **The camera publisher prints `NO RTCP Sender Report`** — RTSP was turned off
  on the NVR again, or the port forward is down.
- **UWB shows `t_valid_is_arrival`** — the backend patch is not active. Data is
  still positionally useful, only its latency is unknown.
