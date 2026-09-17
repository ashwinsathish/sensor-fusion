# Agent handover — read this first, then act

You are picking up a sensor-fusion project mid-flight, on a new machine. This
file is the full context. Everything here was established by measurement, not
assumption; where something is still a guess it says so.

**Your job today: get a dataset recorded without wasting the user's trip.**
The user should not have to test terminal commands. Run the checks yourself,
report what you find, and only ask them for things that need physical access
to the robot or a decision only they can make.

---

## 1. The project in one paragraph

At the LIT factory (JKU / Silicon Austria Labs) three systems each report where
an Omron LD mobile robot is: four Reolink cameras with YOLO + ground-plane
homography, a 9-anchor UWB network, and the robot's own LiDAR SLAM. The goal is
to fuse camera and UWB into one estimate, using the robot's own pose as ground
truth, and to produce a per-region map of which modality to trust where. A
paper is intended (IPIN is the natural venue).

The whole thing hinges on **knowing when each measurement was taken**. The
robot moves ~1 m/s, so 100 ms of timing error is 10 cm of position error — and
that error is indistinguishable from sensor noise, so it never gets diagnosed.

---

## 2. What was found (all measured, with numbers)

**The clocks were catastrophically wrong and nobody knew.** The user's SAL VM
was 4.3 s slow; the Proxmox host under it was 2.4 s off; the offset between the
VM and the Omron's Raspberry Pi swung **14.5 s across 98 days** of archived
runs. None of this was latency — it was undisciplined clocks. NTP is blocked on
SAL's subnet (outbound UDP to the internet is firewalled; UDP/53 to a local
resolver answers in 22 ms, to 1.1.1.1 it times out). **The factory network does
permit NTP** — the Reolink cameras track pool.ntp.org and the anchor Pis are
synced.

**The cameras do expose capture timestamps.** The previous belief was that
Reolink provides none. False: RTCP Sender Reports carry
`(sender NTP wall clock, RTP timestamp)`, so per-frame capture time is
`ntp_sr + (rtp_ts - rtp_sr)/90000`. It is absent from the `/flv?…bcs` endpoint
the old pipeline used, which is why it looked missing. RTSP was switched *off*
on the NVR and had to be enabled (Reolink → Network → Advanced → Server
Settings). Verified live on all four channels: Sender Reports every 4–6 s,
camera clocks NTP-disciplined, agreeing with each other to ~15 ms.

**Camera latency is bimodal**, roughly 20 ms for most frames and ~120 ms for
others — the I-frame pattern. Any "measure the average lag and subtract it"
approach (including the stopwatch method originally proposed) is wrong on ~30%
of frames by 100 ms. Per-frame timestamps make this a non-issue.

**FLV and RTSP are pixel-identical** — 2560×1440 both, 497/500 matched features
inliers, exact identity transform. So the existing camera calibration carries
over unchanged.

**The camera calibrations are weak.** Three of cam1's nine reference points and
one of cam2's sit *outside* the captured image (pixel y down to −188 in an
852-px frame) — placed by extrapolation. Self-consistency is 5–16 cm median,
up to 52 cm. Each camera's points cover a narrow band of floor. This is
probably the dominant camera error, not the detector. `tools/fit_homography.py`
re-derives them from a parking run with no clicking.

**UWB anchor Pi timestamps carry a stable per-node bias, not jitter.** From the
28 Apr raw ROS log: NODE 03 always earliest, NODE 04 always ~2.75 ms later,
each with only ~0.2 ms of scatter. So "take the earliest timestamp" silently
inherits the fastest node's bias — recording *which* anchor it came from lets
that be subtracted, taking UWB timing from ~3 ms to sub-millisecond.

**UWB anchor radio clocks are free-running**, ~2 ppm apart — tens of metres of
range-equivalent drift per 100 ms frame. Raw TDoA is impossible without
correction. The reference-anchor broadcasts can correct it to **4–8 cm**
(leave-one-out, measured), but only with a tight fit window (±2 frames). The
reference anchor (`0x1111`) is therefore a single point of failure: block it
and TDoA degrades in that region even with every other anchor visible.

**Consequence for the paper:** with all 9 anchors reporting, UWB position
quality is near-uniform across the hall (HDOP 0.76–0.90, a 1.2× spread). So a
GDOP-driven trust map measures almost nothing. **Blockage — which anchors are
heard — is what varies**, and that is what the Sionna RT twin can predict. The
camera side does vary genuinely with range and obliqueness.

---

## 3. The architecture (agreed with the user's supervisor)

Every source publishes to one MQTT broker. One collector on the factory laptop
subscribes to all topics, stamps arrival, and writes the dataset.

```
  camera pipeline  ──┐
  UWB backend      ──┼──►  MQTT broker  ──►  collector  ──►  sessions/*.csv
  Omron Pi         ──┘
```

`latency = t_recv − t_valid`. **The collector's clock is the only one that
enters the measurement.** Every other machine can have a broken clock without
harming the data, *provided it never supplies a timestamp*. That is a design
rule worth defending — it is cheap to violate accidentally.

`t_valid` per source:
- camera — the shutter instant, from RTCP
- UWB — the earliest anchor Pi timestamp of the **solved** round
- Omron — the instant the Pi polled ARCL (already field 0 of its payload)

Payload contract: `MESSAGE_SPEC.md`.

---

## 4. Coordinate frames

Everything is recorded in **factory interior** metres: origin at the inner wall
corner, x ∈ [0, 39], y ∈ [0, 12.5].

| source | native | transform |
|---|---|---|
| camera | already factory interior | none |
| UWB | mirrored both axes, origin at x = 14.8 | `frames.uwb_to_factory` |
| Omron | raw mm, two-point affine from `sites/lit_factory.yaml` | `frames.omron_to_factory` |
| Agilox | raw mm, offset | `frames.agilox_to_factory` |

All in `frames.py`, with a self-test. Verified: the nine anchors transform to
x 0.59–38.65, y 0.32–12.46 inside a 39 × 12.5 m hall — nine anchors cannot all
land inside by accident. And an Omron raw-mm reading and a UWB native reading
of the same spot come out 5 mm apart end to end.

Both raw and converted coordinates are stored in every row, so a wrong
transform can be undone without recollecting.

---

## 5. What is built

```
frames.py            all coordinate transforms + self-test
endpoints.py         finds the broker and cameras on whatever network you are on
preflight.py         the go/no-go check — run this first, always
setup.sh             one-command install on a fresh machine

record/collect.py    THE COLLECTOR. keypress-driven, grades each run
record/recorder.py   its machinery (clock, streams, dwell/motion detection, QC)

camera/rtsp_source.py        RTSP + RFC6184 depacketise + H.264 decode + capture time
camera/camera_worker_rtsp.py drop-in replacement for Cam-tracking-LIT's worker
camera/publish_camera.py     runs the workers, fuses, publishes to MQTT
camera/test_rtsp_source.py   offline round-trip test (no camera needed)

uwb/patch_backend.py         patches the SAL UWB backends to emit timestamps
uwb/publish_uwb.py           websocket→MQTT bridge (fallback only; see below)
uwb/solver.py, test_solver.py  standalone TWR solver + tests
uwb/analyse_ros_log.py       decodes a raw ROS log; produced the findings above

tools/estimate_delay.py      spatio-temporal calibration (static / delay / joint)
tools/fit_homography.py      re-derive camera calibration from a parking run
tools/clock_audit.py         live + historical clock offset analysis
tools/pi_clock_ref.py        use the Omron Pi as a master clock (SAL fallback)
tools/rtsp_clock_probe.py    standalone: does a camera expose capture times?
```

---

## 6. The UWB situation — important

The repo the user was given (`~/tdoa_uwb`) has **branches**. `master` is the
AppLab demo: two-way-ranging multilateration, 4–5 anchor environments, no MQTT
publishing. **The real code is on `feat/tag_update_rate`** — actual TDoA
(`backend/uwb_tdoa_localization_backend.py`), a Kalman filter, the 9-anchor
`environments/environment_oic9_M2.json`, and its own MQTT publisher to
**`UWB/position`**.

That branch is already checked out locally and **patched** (nothing pushed;
`.orig` backups next to each file; `uwb/patch_backend.py <repo> --revert`
undoes it). Two things the patch fixes:

1. **The published `timestamp` is the wrong one.** It is the arrival of
   whichever message *triggered* the solve, but `evaluate_measurements()`
   deliberately waits until a round is ≥2 rounds old. At 10 Hz that makes the
   published timestamp ~200 ms younger than the round it labels. The patch adds
   `t_round_s` — the earliest anchor timestamp of the round actually solved.
   Verified: recovers the true round start to 0.00 ms in a unit test.
2. **The published position is Kalman-filtered.** Good for the live plot, wrong
   for evaluation — a filter lags and smooths, biasing both the latency
   estimate and the accuracy numbers. The patch carries `x_raw`/`y_raw`, the
   unfiltered TDoA fix, alongside.

The collector reads `UWB/position` natively, so **no bridge process is needed**.
`uwb/publish_uwb.py` is only for a websocket-only setup.

**Dependency traps on that branch:** `requirements.txt` omits `filterpy`,
`shapely` and `scipy`, which `kalman_filter.py` imports. `ranging_utils` is
Infineon-internal (`gitlab.intra.infineon.com`) — the patch makes it an
optional import, since it is only used by the legacy TWR path and TDoA uses
`scipy.least_squares`. `applab_pylib` (SAL-internal) **is** required, because
it is how the backend receives the ROS stream.


### Where the UWB localisation actually runs — settled by reading the code

The user reasonably objected that they never start anything: in the Sionna GUI
they click "show live UWB visualization" and a blue dot appears. Both things
are true. Here is what that button does
(`LIT_fac_ray_tracing/src/sionna_rt_gui/uwb_localization.py`):

```python
def start_uwb_server(state):
    if _remote_source_enabled(state):
        _start_remote_tunnel(state)
        return                      # <- launches NOTHING locally
    ...
    state.process = subprocess.Popen(cmd)   # ROS/Serial mode only
```

and the default is
```python
self.uwb_comm_index: int = 2     #  2 == "Remote"
_DEFAULT_REMOTE_UWB_URL = "http://193.171.203.67:8500"
```

So on the default setting the GUI **opens a tunnel to a UWB server already
running at the factory** and reads its websocket. Nothing is started on the
user's machine. That is why it feels like it needs nothing — the compute is
someone else's machine, and the GUI hides it.

(It could not run locally anyway: `applab_pylib`, which reads the ROS stream,
is not installed in the `uwb-visualization` venv on that VM.)

Two consequences for collection:

1. **The UWB server is a separate machine that has to be up.** It was down when
   this was checked (`193.171.203.67:8500` -> 503 through the proxy). If UWB is
   missing on collection day, that server is the thing to chase, not the tag.
2. **It may serve only a websocket, not MQTT.** Only the `feat/tag_update_rate`
   branch publishes to `UWB/position`; the factory server may still run older
   code. **This needs no extra process** — `collect.py` reads the websocket
   directly as a source, and `--uwb-ws auto` finds it. `uwb/publish_uwb.py` is
   only for putting UWB on MQTT for *other* consumers; do not run it just to
   feed the collector, that would be a pointless round trip.

   If both paths are live the collector records MQTT and puts the websocket on
   standby, so nothing is written twice. Verified: websocket-only 50 rows in
   5 s, then with MQTT also live only 25 more (the MQTT rate), status
   "standby (MQTT is providing UWB)".

**What the server actually computes**, so the trade-offs are clear: it receives
per-anchor arrival timestamps over ROS, groups them by ranging round, and
solves for position — distances from time-of-flight then multilateration for
TWR, or anchor-clock correction from the sync exchanges then hyperbolic
equations for TDoA. It has to run somewhere with access to the ROS network.

**Whether UWB carries a source timestamp depends on which code that server
runs, and the patch in this repo only affects the local copy.** If the factory
server runs unpatched older code, UWB fixes arrive with no time attached, the
collector stamps arrival, and `t_valid_is_arrival` is set true — positions stay
usable, only UWB *latency* becomes unmeasurable. To fix it properly the patch
has to be applied where the server actually runs.


**First question to settle: is the UWB localisation program running?**

This trips people up, so be explicit with the user. Switching the tag on is
**not** enough. The chain is:

```
  tag blinks  ->  anchors hear it, each anchor a Raspberry Pi running a ROS node
              ->  ROS central node collects those messages
              ->  localization_gui.py reads that stream, computes the position,
                  publishes it to MQTT topic UWB/position
```

That last program has to be running on some computer. It could be a colleague's
machine (they wrote the publisher and a test receiver pointing at
`10.0.0.3:1883`, so probably yes) or it could be this laptop.

`preflight.py` answers it empirically: if it lists a UWB source, someone is
running it and there is nothing to do. If it warns that no UWB source is
publishing, either ask the colleagues to start it, or start it here — which
needs `applab_pylib` (SAL-internal, for the ROS stream) and the ROS central
node's IP.

Camera and Omron data are unaffected either way, so a run without UWB is still
worth doing.

---

## 6b. The factory machines, as found on 9 Sep 2026

```
10.0.0.3    oic-server2     MQTT broker (1883); a UWB checkout; the orchestrator
                            / node-controller in tmux `services:1` (must stay up)
10.0.0.2    server1         UWB_system web
40.0.0.37   sal-UPN-APL01   the Omron status collector. An UpBoard, NOT a
                            Raspberry Pi. DataCollector.py from
                            ~/workspace/repos/iws-testbed (branch `ashwin-fix`),
                            tmux session `omron`, user `sal`.
50.0.0.2    Reolink NVR     RTSP 554
```

### The Omron collector hangs, and it hangs *quietly*

On 9 Sep it had been publishing nothing for 68 minutes while looking perfectly
healthy: process alive, six threads, no traceback, ARCL socket `ESTAB` with
empty queues. **0.2% CPU over 81 minutes** and a thread parked in `do_select` —
blocked on a read that never returns. The CSV it writes had simply stopped
growing. A fresh ARCL connection answered in 0.13 s, so the robot was fine.

Diagnosed cause: `DataCollector.on_message` calls `omro.DO_TASK(payload)` on the
**MQTT callback thread** while `omro_thread` calls `STATUS()` on the **same
telnet connection**, with no lock. Under a burst of `gotoPoint` commands the two
interleave, one consumes the reply the other is waiting for, and the waiting
loop has no timeout. An earlier run died from the same race with
`KeyError: 'Status'`.

**Two things follow, and the second matters more tomorrow:**

1. Restarting the tmux session clears it. That only buys time.
2. **The race is triggered by MQTT `gotoPoint` commands.** So during a
   collection run, drive the robot with the pendant or MobilePlanner —
   **not** by sending ARCL goto commands over MQTT. That sidesteps the bug
   entirely without touching anyone's code, and losing ground truth mid-run
   would waste the whole session.

`preflight.py` now treats a silent `Omron/status` as blocking and prints where
to go. Note that `ps` will look healthy — check whether the CSV is still
growing.

### The UWB checkout on Server2 is the wrong branch

```
/home/oic-wlc2/Workspace/repos/localization-visualization/localization_gui.py
branch: demo/OIC-dataExport          (clean)
```

- `backend/uwb_tdoa_localization_backend.py` **does not exist** — no TDoA here
- `grep "UWB/position"` → **no matches** — this version does not publish to MQTT
- venv has `applab-pylib` and `ranging-utils` (both editable installs from
  `Workspace/repos/UWB_repos/`), plus `scipy` and `shapely` — but **not
  `filterpy`**, which `kalman_filter.py` needs
- it was **not running**; tmux `services:0` sits parked in that directory, idle
- the node-controller that must stay up is `services:1`, `main.py` from
  `Workspace/repos/orchestrator/src`

So Server2 is the *right* machine for the TDoA branch — it already has the two
private packages that are hardest to obtain — but it needs
`git checkout feat/tag_update_rate` and `pip install filterpy`. That is
someone else's machine, so it needs asking.

**UWB latency is a required output, not a nice-to-have.** Do not treat
"positions without a source timestamp" as an acceptable fallback — measuring
how late UWB is *is* half the point of the dataset. Three things stand between
here and a correctly-timestamped UWB feed, all small:

1. **The branch.** Server2 is on `demo/OIC-dataExport`; the TDoA code is on
   `feat/tag_update_rate`. Same remote, so `git fetch && git checkout` works.
2. **`filterpy`** is not installed there. One `pip install`.
3. **The patch in `uwb/patch_backend.py`** — and this is the one that is easy
   to miss. Even on the TDoA branch the published `timestamp` is the arrival of
   whatever message *triggered* the solve, while `evaluate_measurements()`
   deliberately waits until a round is >= 2 rounds old. At 10 Hz that is a
   systematic ~200 ms error: UWB would look faster than it is and every
   position would be attributed to the wrong instant. Switching the branch
   alone yields a number that looks fine and is wrong.

`applab_pylib` is unavoidable — the backend's `start()` only implements `ROS`
and `Serial`; `ROS_legacy` (which would have used plain HTTP) is accepted in
`__init__` but has no branch in `start()`, so it is dead code. The package
exists on Server2 as an editable install at
`Workspace/repos/UWB_repos/applab_pylib` and can be copied.

Preferred: **run it on the Legion**, where the branch, the patch and restarts
are all under our control. Failing that, get Server2 switched — it already has
the two private packages, which is what makes it convenient.

---

## 6c. Facts established 16 Sep 2026 (supersede anything above that conflicts)

**Reference clock: `10.0.0.2`** ("NTP-server-host", stratum-3 NTP on the factory
LAN). Server2 is PTP-locked (`phc2sys` ~2 ns) and agrees with it to 0.05 ms.
Anchor Pi `30.0.0.17` is on public pool NTP but within 0.15 ms of it.
**The Omron UpBoard (`40.0.0.37`) is the outlier** — public `ntp.ubuntu.com`
over 5G, ~30 ms behind, 90 ms jitter, last sample rejected. That clock stamps
the ground truth, so before collecting run `sudo tools/set_ntp.sh` on it.

The collector no longer trusts `timedatectl`'s yes/no. It measures its own
offset to 10.0.0.2 by SNTP every 30 s (`timeref.py`) and stamps arrivals on
that clock. Verified: host deliberately 45 ms off, latencies still recorded as
0.7 ms (Omron) and 180.9 ms (UWB, true 180). Fallbacks: local NTP, then the Omron
collector clock — the latter now graded **bad**, since it is the worst clock.

**UWB solver, as actually run on Server2** (30 runs 25 Aug – 16 Sep, always bare
`python3 localization_gui.py`, so the code defaults apply):
```
python3 localization_gui.py --env environments/environment_oic8_M2.json \
        --ip 10.0.0.2 --server_ip 0.0.0.0 --server_port 8000
```
- ROS central node **10.0.0.2** (same machine as the NTP reference), API on 8000
- **`environment_oic8_M2.json`** — 8 anchors (oic9 minus `0x6666`), initiator
  `0x1111`, plus a `tags` list mapping IMSI -> device. **`oic9_M2` crashes at
  startup**: no `tags` key, read outside the try block.
- serves on **8000**. README says 8001, which is wrong and is taken by the
  orchestrator on Server2.
- Server2's checkout is **now on `feat/tag_update_rate`**, 10 commits past what
  was first patched (outlier rejection, per-tag topics `UWB/position/<tag>`,
  IMSI API). The patch was re-applied to the new head and re-verified at runtime:
  `t_round_s` still recovers the true round start to 0.00 ms.

**MQTT client-id collision.** The GUI hardcodes client id `LOCClient`. A second
instance anywhere makes the broker evict the first; both auto-reconnect and evict
each other ~once a second. Broker log: 20 collisions on 15 Sep between 40.0.0.11
(most likely Andreas's PC) and 40.0.0.29. `patch_backend.py` now also makes the
id unique per host+pid — but that only stops the eviction loop. **Two solvers
running still interleave two different position streams on `UWB/position`.** Only
one may run during collection. `preflight.py` detects it (round index going
backwards).

**`applab_pylib`**: clone URL
`https://git.silicon-austria.com/wescom/testbed-orchestration/applab_pylib.git`
(branch `main`), but Server2's copy has an **uncommitted edit to
`data/UWBLab_IPs.json`** switching the anchor list to OIC's 9 `30.0.0.x` nodes. A
fresh clone lacks it. Use the tarball `/tmp/applab_pylib.tgz` on Server2 (25 KB,
no `.git`). Its pins are old: `msgpack_python==0.5.6` (2018) and
`setuptools==65.5.0`. msgpack 1.0 changed the default for decoding bytes vs str
keys, so a modern msgpack could change what the parser returns — match Server2's
Python and msgpack versions if installing elsewhere, or run the solver on
Server2 where the environment is proven.

**Which tag is on the Omron is still unknown.** The environment maps IMSIs to
devices but not tag node ids; the 28 Apr log's tag was `0x3c14`. The collector
records `tag_node_id` on every row, so this can be resolved afterwards, but ask.

---

## 7. What to do on arrival — in this order

Run these yourself. Report results; do not make the user type them.

**1. Where are we?**
```bash
python3 endpoints.py
```
Reports which broker and camera addresses this machine can reach and picks the
best. On the OIC wifi expect `10.0.0.3:1883` (or `193.171.203.67:1833`) and
`50.0.0.2:554`. Everything else defaults to `auto` and uses this.

**2. Install.** The Legion has a GPU, so it runs everything:
```bash
./setup.sh --with-camera
```

**3. The clock.** This laptop is being set up in the factory for the first
time, so assume nothing.

```bash
timedatectl
```
Want: `System clock synchronized: yes`. If not:
```bash
sudo timedatectl set-ntp true
timedatectl                                   # check again
```
Still not syncing? Diagnose rather than guess — the factory network *does*
permit NTP (the Reolink cameras track pool.ntp.org and the anchor Pis are
synced), so it should work:
```bash
systemctl status systemd-timesyncd            # or chronyd
timedatectl show-timesync --all 2>/dev/null   # what server, has it ever reached it
```
If UDP/123 turns out to be blocked on this segment, point it at a local
server instead — the same one the anchor Pis use is the right answer.

**You are not blocked if it will not sync.** The collector detects an
undisciplined clock and falls back to referencing the Omron Pi, which IS
synced and stamps every MQTT message. It measures the offset to sub-millisecond
and subtracts it from arrival times, so the latencies come out right anyway.
`session.json` records `clock.mode` as `local_ntp` or `pi_referenced` so the
dataset always says which was used. Verified: with the host deliberately 3.0 s
slow, a 250 ms injected latency was still recorded as 250.6 ms.

Prefer real NTP — the fallback depends on MQTT flowing and on the Omron Pi
being correct — but do not cancel a collection trip over it.

**4. Go/no-go.** Ask the user to park the robot where cameras and UWB can see
it, then:
```bash
python3 preflight.py --seconds 60
```
It checks the clock, the broker, the transforms, each source's rate and
timestamps, whether every source puts the robot inside the hall, and — the
important one — whether the sources **agree with each other** about where the
parked robot is. Under ~1 m or do not collect; a larger gap means a coordinate
transform is wrong, which is invisible in a live plot and ruins the dataset
silently.

**5. Start the camera publisher** (leave it running):
```bash
python3 camera/publish_camera.py --dry-run     # confirm messages look right
python3 camera/publish_camera.py               # then go live
```
Expect `capture clock locked` from each camera within ~20 s. If one never
locks, RTSP is off on the NVR.

**6. UWB** — only if `preflight.py` showed no UWB source:
```bash
cd ~/tdoa_uwb && git branch --show-current      # must be feat/tag_update_rate
python3 ../sensor-fusion/uwb/patch_backend.py . --dry-run   # "already patched"
.venv/bin/python localization_gui.py --env environments/environment_oic9_M2.json --ip <ros-central-ip>
```

**7. Collect:**
```bash
python3 record/collect.py
```
`1` parking, `2` driving, `s` stop, `q` quit.

---

## 8. The two runs, and why they must be separate

**Parking** — 8–10 well-spread spots, stop and hold still ~10 s at each,
**turning the robot to a different heading each time**. Stationary means
latency cannot contaminate anything, so this run answers "where is each
sensor's frame" and "how accurate is it" cleanly. Varying the heading separates
a real sensor offset from the geometry of where a camera sees the robot's
footprint. It also re-derives the camera calibration.

**Driving** — sharp starts and stops all over the floor. **Not** a smooth loop:
a constant-speed circuit carries almost no timing information, the correlation
peak goes flat, and the delay estimate comes out confident and meaningless. The
display counts stop/go transitions; aim for 15+.

**Never fit both from the same moving data.** A constant lag along a straight
run is observationally identical to a translation, so extrinsics and latency
silently absorb each other and you get two wrong answers that look right.

**Do both twice** if there is time — so the trust map can be built on one pair
and evaluated on the other. Fitting and evaluating on the same runs makes any
improvement look real when it is not. This is the single most important
methodological point in the project.

---

## 9. After collection

```bash
python3 tools/fit_homography.py sessions/<parking_run> \
        --compare ~/Cam-tracking-LIT/config/cameras/cam*.yaml

python3 tools/estimate_delay.py static sessions/<park>/omron.csv \
        sessions/<park>/camera.csv --out cam_extr.json

python3 tools/estimate_delay.py delay sessions/<drive>/omron.csv \
        sessions/<drive>/camera.csv --extrinsics cam_extr.json --plot cam_delay.png
```

`estimate_delay.py delay` reports two independent estimators — speed-profile
cross-correlation (needs no extrinsics) and residual minimisation. If they
disagree by more than ~50 ms, the delay is not constant, frames are dropping,
or the extrinsics are wrong.

---

## 10. Open questions and known weaknesses

- **The ground truth has its own error.** The Omron's `loc_score` ran 0.83–0.94
  and drops near featureless walls. It is recorded in every row. Until it is
  bounded (a total-station check at the parking spots would do it), camera and
  UWB errors of 10–30 cm are being measured against something with unquantified
  error of its own.
- **Detection reliability is unmeasured.** YOLO misses and misclassifies; frame
  rate varies by machine. Report recall/precision against ground truth as a
  first-class result — a camera that sees the robot 60% of the time has a duty
  cycle, not just an accuracy.
- **Millimetre accuracy is not on the table.** UWB gives 10–30 cm in a
  metal-rich hall; oblique homography 10–50 cm. Decimetre-level fused accuracy
  is the honest target. Framing the paper around millimetres invites rejection
  on the abstract.
- **The UWB range-noise figure (10 cm) is a literature value**, not measured
  for these radios. The covariance in `uwb/solver.py` is correctly *shaped* but
  its absolute scale is inherited from an assumption. Measure it from the
  parking run and set `range_sigma_m`.
- **Binary "trust A or B" is worse than inverse-variance weighting.** Keep the
  binary map as a figure; do not use it as the estimator.

---

## 11. Tone and working style the user prefers

Direct and plain. No jargon unless it is doing work. State findings with the
numbers behind them. If something they proposed is wrong, say so plainly and
say what to do instead — they have said several times they want to be corrected
rather than agreed with. Always visually inspect any figure you generate before
describing it. Do not push to git without being asked.
