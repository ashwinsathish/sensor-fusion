# UWB: keep the time, report the uncertainty

`uwb-visualization`'s backend receives `(frame_nr, anchor_id, tof, timestamp)`
per measurement, solves a fix, and emits `{x, y, FrameNr, type}`. The round
time, the residual and the anchor count are computed and then discarded — so a
UWB position currently has **no time attached to it anywhere in the pipeline**,
and no way to say how much to trust it.

`solver.py` is a drop-in replacement for the maths. `test_solver.py` checks it
against the **real nine-anchor LIT geometry** from `environment_oic.json`.

## What the tests actually found

Two of my assumptions were wrong, and the numbers say so.

**Fixing z is not the big win I claimed — except when it is.** The tag sits at a
known height on the AGV, so solving for z is a wasted parameter. With all nine
anchors reporting that barely matters. With a blocked round it is the
difference between a fix and garbage:

```
anchors    free z (current)    fixed z     gain
   4            99.3 cm        10.8 cm    9.2x
   5            44.6 cm         9.0 cm    5.0x
   6            12.6 cm         7.4 cm    1.7x
   9             6.3 cm         5.8 cm    1.1x
```

Racks, the crane and the AGV's own body block anchors constantly, so degraded
rounds are the normal case, not the exception — and the current backend has no
minimum-anchor check at all, so it emits a metre-scale fix without complaint.

**The anchor layout is good, which kills a hypothesis.** I expected position
quality to vary a lot across the hall, and for a twin-predicted GDOP map to be
the interesting artefact. It doesn't:

```
HDOP, all 9 anchors : 0.76 .. 0.90   (1.2x spread across the hall)
HDOP, 4 anchors     : 1.10 .. 2.44   (2.2x spread)
```

With everything reporting, UWB quality is near-uniform. So **for UWB, geometry
is not what separates good fixes from bad ones — blockage is.** A trust map
should be driven by *which anchors can see the tag* (which the twin can predict
from the scene) rather than by DOP of the full anchor set. The camera side is
different: its covariance genuinely does vary with range and obliqueness.

## What else it does

**Covariance that predicts the error.** From the LS Jacobian and residuals:

```
mean error 7.0 cm, mean predicted sigma 8.7 cm  (ratio 0.80)
98.2% of fixes fall inside 2 sigma
```

That is the number your colleague's weighting scheme should use. It is derived,
not tuned, and it is checked against ground truth rather than asserted.

**NLOS rejection.** One bad anchor wrecks a fix; dropping the worst residual and
re-solving recovers much of it:

```
clean             7.0 cm
one NLOS anchor  71.4 cm   (no rejection — current behaviour)
after rejection  49.8 cm
```

Honest read: rejection helps a lot but a contaminated round is still ~7x worse
than a clean one. The `ok` flag and `residual_m` let the filter down-weight
rather than pretend.

**Round time from the frame counter.** Ranging rounds are periodic, so
regressing arrival time on frame number recovers when the round happened far
better than any single arrival stamp:

```
raw arrival timestamp : bias +10.14 ms   std 12.38 ms   p95 54.2 ms
frame-clock estimate  : bias  +6.64 ms   std  0.45 ms   p95  7.4 ms
recovered period 99.998 ms (true 100.0 ms)
```

**27x less jitter.** The remaining bias is the constant pipeline delay, which is
absorbed by the per-sensor delay estimated in `../tools/estimate_delay.py`.
Frame counter wrap-around is handled.

## Wiring it into uwb-visualization

In `backend/uwb_localization_backend.py`, replace the body of
`add_measurement` / `evaluate_measurements` / `perform_localization` with:

```python
import sys; sys.path.insert(0, "/home/sathishkumara/sensor-fusion/uwb")
from solver import UwbLocalizer, load_anchors

# once, in __init__ (z_tag = tag height above the floor on the Omron)
self.loc = UwbLocalizer(self.anchor_dict, z_tag=0.4)

# wherever add_measurement is called today
fix = self.loc.add(frame_nr, ID, ToF, t_arrival)
if fix is not None and fix.ok:
    data_point = fix.as_dict()          # carries x, y, t_round_s, sigma_m, ...
    data_point["FrameNr"] = frame_nr
    await self._put_latest(data_point)
    await self.data_buffer.append(data_point)
```

`as_dict()` is a superset of the current payload, so the GUI keeps working.

Then in the **LIT GUI's websocket reader** (`uwb_localization.py`,
`_websocket_reader`), stamp arrival — that is what puts UWB on the same
timeline as everything else via `pi_clock_ref.py`:

```python
payload = json.loads(message)
payload["arrival_host_s"] = time.time()
```

## Emitted fields

```
x, y, z              position, z is the fixed tag height
t_round_s            when the ranging round happened  <- the validity instant
t_solved_s           when the solve finished
n_anchors            how many contributed
residual_m           RMS range residual
sigma_m, cov_xx/xy/yy   position uncertainty for weighting
hdop                 geometry quality
anchors_used / anchors_rejected
ok, reason           whether this fix should be trusted at all
```

## Run

```bash
python3 test_solver.py     # no hardware needed
```

## Not yet verified

Everything here is simulated ranging on the real anchor coordinates. The
10 cm per-range noise figure is a literature-typical value for TWR, not a
measured one for these radios. Measure it against the Omron ground truth
(`../tools/estimate_delay.py static`) and set `range_sigma_m` from the result —
until then the covariance is correctly *shaped* but its absolute scale is
inherited from an assumption.
