# LIT factory sensor fusion

Recording and calibration tooling for fusing camera + UWB localisation of the
Omron AMR at the LIT factory, with the robot's own SLAM pose as ground truth.

| you are | read |
|---|---|
| setting this up on a new machine | **[START_HERE.md](START_HERE.md)** |
| an AI agent picking this up | **[AGENT_HANDOVER.md](AGENT_HANDOVER.md)** |
| running collection today | [RUNBOOK.md](RUNBOOK.md) |
| writing a publisher | [MESSAGE_SPEC.md](MESSAGE_SPEC.md) |

## Quick start

```bash
./setup.sh --with-camera      # omit --with-camera if this machine has no GPU
python3 endpoints.py          # what can this machine reach?
python3 preflight.py          # go/no-go — park the robot first
python3 record/collect.py     # 1 = parking run, 2 = driving run, s = stop
```

Addresses are found automatically, so the same commands work on the factory
wifi, on the SAL VM through its tunnel, and over the public forward.

## Where the data goes

```
sessions/<timestamp>_<parking|driving>_<n>/
    omron.csv      ground truth, 5 Hz
    camera.csv     fused camera detections
    per_cam.csv    the same, un-aggregated per camera
    uwb.csv        UWB fixes, filtered and unfiltered
    agilox.csv     the other AMR
    clock.csv      clock offset once per second, for the record
    session.json   manifest: rows, dwells, latencies, time sources
```

Every row carries, in factory-interior metres:

```
t              when the measurement was TRUE      <- the validity instant
t_recv         when the collector received it
latency_s      t_recv - t                        <- the thing being measured
internal_s     delay inside the source
x, y           position
x_native, y_native, frame_in   what the publisher actually sent
```

plus each sensor's own quality fields — anchor count and residual for UWB,
contributing cameras and confidence for the camera, `loc_score` for the Omron.

Long format, one row per measurement, never merged across sources: the three
streams run at different rates and any alignment baked in at record time could
not be undone later.

## Coordinate frames

Everything lands in **factory interior** metres — origin at the inner wall
corner, x ∈ [0, 39], y ∈ [0, 12.5]. Camera output is already in this frame;
UWB and Omron are converted. All transforms live in [frames.py](frames.py):

```bash
python3 frames.py     # self-test
```

Verified: the nine UWB anchors transform to x 0.59–38.65, y 0.32–12.46 inside a
39 × 12.5 m hall, and an Omron reading and a UWB reading of the same spot come
out 5 mm apart end to end.

## What is here

```
frames.py       coordinate transforms + self-test
endpoints.py    finds the broker and cameras on whatever network you are on
preflight.py    go/no-go check
setup.sh        one-command install

record/         the collector
camera/         RTSP capture timestamps, YOLO worker, MQTT publisher
uwb/            backend patches, solver, raw-log analysis
tools/          delay estimation, homography fitting, clock auditing
```

Each folder has its own README with the details and the measurements behind
the design choices.

## Status

Built and tested: coordinate transforms, the collector, camera capture
timestamps (live against all four cameras), the UWB backend patch, calibration
and delay-estimation tooling, pre-flight checks.

Not yet done: **no dataset has been collected.** That is the next step, and
everything here exists to make it produce something usable on the first try.
