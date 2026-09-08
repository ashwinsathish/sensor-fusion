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

Sorry, that was my sloppy wording. There is no dedicated UWB server. What
exists is a **program** (`localization_gui.py` from the `tdoa_uwb` repo) that
has to run on *some* computer. It reads raw radio data from the ROS central
node, computes the tag's position, and publishes it to MQTT.

So the real question is: **is somebody already running it?**

Your colleagues wrote the MQTT publisher and a test receiver
(`UWB_mqtt_receiver.py`) pointing at `10.0.0.3:1883`, which strongly suggests
they run it themselves. If so, **you run nothing for UWB.** You just listen.

**Find out in 60 seconds**, from any machine that can reach the broker:

```bash
python3 preflight.py --broker <broker-ip>
```

If it lists a UWB source, it is already running and you are done thinking about
it. If not, someone has to start it — either them, or you on the OIC laptop.

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

**If it says the clock is not synchronized, fix that before anything else:**

```bash
sudo timedatectl set-ntp true
timedatectl                 # must say: System clock synchronized: yes
```

Every latency in the dataset is `arrival − source time`, measured against this
laptop's clock. If it is wrong by a second, every latency is wrong by a second,
and you would not notice until the analysis.

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
