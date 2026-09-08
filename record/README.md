# Collecting data

```bash
python3 collect.py
```

That is the whole command. Sensors connect once and stay up; you drive and
press single keys.

```
  1   start a PARKING run     frame alignment + accuracy
  2   start a DRIVING run     latency
  s   stop the current run
  m   mark this moment
  q   quit
```

On `s` it immediately tells you whether the run was usable, so you can redo it
while you are still standing next to the robot:

```
  20260904T141424_parking_1  0.4 min
    ✓ 125 ground-truth rows over 0.4 min
    ! no camera data — that sensor contributes nothing
    ✗ only 1 dwells — need 3 minimum, 8 ideally. Park longer and hold still.
```

## The two runs

**Parking** — drive to 8-10 well-spread spots, hold still at each until the bar
fills. While the robot is stationary latency cannot affect anything, so this
run answers "where is each sensor's coordinate frame?" and "how accurate is it
really?" with no timing question mixed in.

**Driving** — sharp starts and stops, all over the floor. Not a smooth loop: a
constant-speed circuit carries almost no timing information and the delay
estimate comes out meaningless. The display counts stop/go transitions; aim
for 15+.

## The clock

This machine's wall clock has been wrong by seconds, so nothing trusts it. The
Omron Pi stamps every MQTT message and is the master; anything measured
locally is converted onto it with an NTP-style minimum filter, stable to under
a millisecond. Every CSV column `t` is Pi time; `t_local` is this host's clock,
kept only for diagnostics. `clock.csv` logs the offset each second so you can
prove afterwards that it held.

## Output

`../sessions/<timestamp>_<mode>_<n>/` containing `omron.csv`, `camera.csv`,
`uwb.csv`, `agilox.csv`, `clock.csv`, `session.json`.

Every CSV starts `t,x,y`, which is what `../tools/estimate_delay.py` reads, so
the analysis runs straight off the output.

`session.json` records where each stream's time actually came from:

| stream | time source | quality |
|---|---|---|
| omron | producer timestamp in the payload | the master |
| camera | shutter instant from RTCP, mapped to Pi time | best available |
| uwb | `t_round` once the solver patch is live, else arrival | good / poor |
| agilox | arrival only — that topic carries no timestamp | poor |

## Options

```
--camera-config camN.yaml ...   record the cameras (omit to skip)
--no-uwb                        skip the UWB websocket
--uwb-ws ws://host:8001/ws      where the UWB GUI serves
--broker-host / --broker-port   MQTT, default 127.0.0.1:1833
--want-dwells 8                 target shown while parking
```

## Then

```bash
# frame alignment, from the parking run — immune to latency
python3 ../tools/estimate_delay.py static <park>/omron.csv <park>/camera.csv \
        --out cam_extr.json

# latency, from the driving run, with that alignment held fixed
python3 ../tools/estimate_delay.py delay <drive>/omron.csv <drive>/camera.csv \
        --extrinsics cam_extr.json --plot cam_delay.png
```

Never fit both from the same moving data — one silently absorbs the other.

## Verified

Run against the live factory broker 4 Sep 2026: clock locked in under 4 s,
Omron 5.0 Hz, Agilox 8.9 Hz, dwell detection fired on the parked robot,
quality report correctly flagged the missing camera/UWB streams and the
insufficient dwell count.
