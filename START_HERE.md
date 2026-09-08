# Start here

Plain version. No jargon.

## What we are doing

Three systems each claim to know where the Omron is: the cameras, the UWB
radios, and the robot's own laser navigation. We want to record all three at
once, with an honest timestamp on every measurement, so we can afterwards work
out which one is accurate where.

Everything already talks to one MQTT broker. **One program on one laptop
listens to all of it and writes the CSV files.** That program is
`record/collect.py`. That is the whole job.

## "The UWB server" — there is no such machine

Sorry, that was sloppy wording on my part. There is no dedicated UWB server.

Switching the tag on is not by itself enough. What actually happens:

```
  tag blinks
    -> each anchor hears it (every anchor is a Raspberry Pi running a ROS node)
    -> a ROS central node collects those messages
    -> a PROGRAM reads that stream, computes the position, and publishes it
```

That last program is `localization_gui.py`, and it has to be running on *some*
computer. Nothing localises the tag until it does.

**And it already is — just not on your machine.** When you click "show live UWB
visualization" in the Sionna GUI, the default setting is *Remote*: the GUI opens
a tunnel to `193.171.203.67:8500` and reads a UWB server that is already
running at the factory. It starts nothing locally. That is why it feels like it
needs nothing — the work happens on a machine at the OIC and the GUI hides it.

So for collection, two things follow:

- **That server has to be up.** It was down when last checked. If UWB is missing
  on the day, chase that server, not the tag.
- **It serves a websocket, not MQTT.** Only the newest branch publishes to MQTT,
  and the factory server may still run the older code. If so, one extra command
  bridges it — `preflight.py` tells you exactly which case you are in and prints
  the command.

```bash
python3 preflight.py
```

Three possible answers: UWB is already on MQTT (do nothing), UWB is on
websocket only (run the bridge it prints), or no UWB anywhere (ask whoever
runs that server to start it).

## Which machine runs what

It depends on one thing: **does the OIC laptop have a GPU?**

`./setup.sh` tells you.

### If the laptop has a GPU — run everything on it. Simplest.

```
OIC laptop      camera pipeline  +  collector       ← everything
Omron Pi        already publishing, don't touch
UWB             already publishing (check), or run it here too
```

No VPN, no proxy, no tunnels. The laptop is on the factory network, so it can
reach the cameras, the broker and the ROS node directly.

### If the laptop has no GPU — split in two.

```
your VM         camera pipeline (through the SAL proxy, as tested)
OIC laptop      collector
```

Slightly worse: the camera images travel factory → SAL → back, which adds real
delay to the measurement. Honest, but inflated. Fine for a first dataset.

**Your VM cannot be the collector.** It is not on the factory network, and its
clock is nearly a second wrong — and the collector's clock is the one every
latency is measured against.

## Getting this folder onto the laptop

Any of these. Pick whichever is least annoying.

**Git** — a private repo works fine, this is all text and small.

```bash
cd ~/sensor-fusion && git init && git add -A && git commit -m "fusion tooling"
# push to a private repo, then on the laptop:  git clone <url> ~/sensor-fusion
```

**USB stick** — `tar czf sensor-fusion.tar.gz ~/sensor-fusion`, copy, extract.

**Directly**, if the laptop can reach your VM (SAL VPN on):

```bash
scp -r ~/sensor-fusion <user>@<laptop>:~/     # from the VM
```

The `sessions/` folder is your recorded data — keep that on the laptop.

## Then, on the laptop

```bash
cd ~/sensor-fusion
./setup.sh                  # add --with-camera if it also runs YOLO
```

It creates a virtualenv, installs what is needed, checks whether there is a
GPU, checks the clock, and self-tests the coordinate transforms.

**About the clock.** Every latency in the dataset is `arrival − source time`,
measured against this laptop's clock. If it is a second off, every latency is a
second off.

Try to fix it properly:

```bash
sudo timedatectl set-ntp true
timedatectl                 # want: System clock synchronized: yes
```

**But if it will not sync, you are still fine.** The collector notices and
falls back to referencing the Omron Pi's clock, which is synced and arrives in
every MQTT message. It measures the offset to under a millisecond and corrects
the arrival times, and `session.json` records which mode was used. Tested with
the host deliberately 3 seconds wrong: a 250 ms latency was still recorded as
250.6 ms.

So do not cancel the trip over NTP. Just do not ignore it either.

## Collection day

```bash
python3 preflight.py --broker <broker-ip>     # park the robot first
python3 record/collect.py --broker <broker-ip>
```

```
  1   start a PARKING run
  2   start a DRIVING run
  s   stop  (it grades the run on the spot)
  q   quit
```

**Parking run** — drive to 8–10 spread-out spots, stop and sit still ~10 s at
each, turning the robot to a different heading each time. Watch the on-screen
bar fill before moving on.

**Driving run** — sharp starts and stops all over the floor. Not a smooth loop.

Do both twice if there is time.

## What you end up with

`sessions/<timestamp>_parking_1/` containing `omron.csv`, `camera.csv`,
`uwb.csv`, `agilox.csv`, `clock.csv`, `session.json`.

Every row: position in factory metres, when the measurement was true, when it
arrived, and the difference — its latency. Plus each sensor's own quality
fields.

## The three questions to answer before tomorrow

1. **Does the OIC laptop have a GPU?** → `./setup.sh` says.
2. **Is UWB already publishing to `UWB/position`?** → `preflight.py` says.
3. **Is the laptop's clock NTP-synced?** → `timedatectl` says.

Everything else is already built and tested.

---

Deeper detail, if you want it: [RUNBOOK.md](RUNBOOK.md) for exact commands per
machine, [MESSAGE_SPEC.md](MESSAGE_SPEC.md) for the payload contract.
