# What the raw ROS log actually says

Analysis of `3d_meas.txt` (Damir, 28 Apr 2026) — 5495 lines, 2747 msgpack
records, 39 s of live ranging. Decoded with base64 padding repair; every
record parsed.

Reproduce: `python3 analyse_ros_log.py <file>` (uwb-visualization venv, needs msgpack).

## The setup this log captures

```
logging nodes      NODE 01..04
radio node_ids     0x1111 0x2222 0x3333 0x4444
transmitters       0x1111  (393 frames)   0x3c14  (392 frames)
ranging period     99.20 ms  ->  10.08 Hz   (each transmitter)
anchors per round  min 2, median 3, max 3
```

Message types are `DT_ANCHOR_INITIATOR` / `DT_ANCHOR_RESPONDER` handling
"DTM Poll" — **this is already TDoA**, not the double-sided TWR in
`uwb_localization_backend.py`. A transmitter blinks, the anchors record
`dtm_poll_rx` (their own DW3000 receive timestamp), position comes from
differences between those.

`0x3c14` is not in the anchor list, so it is almost certainly **the tag**, and
`0x1111` is a **reference anchor** blinking at the same rate. Worth confirming.

## 1. The 2-3 ms figure is right — but it is bias, not jitter

Spread of Raspberry Pi timestamps across anchors within one ranging round,
785 rounds:

```
median 2.75 ms   p90 2.94 ms   p99 3.12 ms   max 3.90 ms
```

Alireza's "2-3 ms" is confirmed. But split it per node and it is not random:

```
NODE 03   median 0.00 ms   p90 0.00   max 0.16     <- always earliest
NODE 02   median 0.50 ms   p90 0.60   max 1.56
NODE 04   median 2.75 ms   p90 2.94   max 3.90
```

Each anchor has a **stable offset with only ~0.2 ms of scatter around it**.
That is not NTP jitter; it is a fixed per-node processing or transport delay.

Consequences for the plan they agreed:

* "Take the earliest timestamp in the frame group" is the right call, and the
  frame number is the right key — both confirmed by this data.
* But the earliest will nearly always be NODE 03, so the published time
  inherits *that node's* bias. Constant, therefore removable — **if** the
  publisher also records which node it came from.
* Once the bias is removed the real per-round timing uncertainty is
  **sub-millisecond**, better than the 1-10 ms they were assuming.

## 2. The anchor radio clocks are completely free-running

Raw `dtm_poll_rx` differences between anchor pairs, converted to metres:

```
0x2222-0x3333   median -1 508 554 453 m   drift -15.31 m per frame
0x2222-0x4444   median  1 348 625 718 m   drift +47.96 m per frame
0x3333-0x4444   median -2 299 921 752 m   drift +63.27 m per frame
```

Offsets of ~1.5 billion metres are seconds of clock offset. The drift — 63 m
per 100 ms frame — is 630 m/s of range-equivalent, i.e. **~2.1 ppm** between
crystals. Utterly normal for free-running oscillators, and utterly fatal to
raw TDoA: 1 ns of anchor clock error is 30 cm of position error.

So nothing in hardware synchronises these anchors. It has to be done in
software, from the reference broadcasts.

## 3. How well the reference broadcasts can fix it

Leave-one-out: fit the inter-anchor clock offset from the surrounding
reference frames, predict the held-out one, measure the miss.

```
anchor pair       window   residual median      p95
0x2222-0x3333     +/-2          4.10 cm      12.9 cm
0x2222-0x4444     +/-2          6.86 cm      27.9 cm
0x3333-0x4444     +/-2          8.09 cm      35.0 cm
```

A tight window (+/-2 frames) beats a wide one, which says the clock offset is
not a straight line over long spans — correct it locally and often.

**So software clock sync works, at the 4-8 cm level.** That is the floor TDoA
positioning inherits before any geometry is applied.

Applying the same correction to the tag blinks:

```
0x2222-0x3333   n=392   sd 40.2 cm   frame-to-frame sd 17.5 cm   ratio 0.31
0x2222-0x4444   n=392   sd 59.6 cm   frame-to-frame sd 24.4 cm   ratio 0.29
0x3333-0x4444   n=392   sd 46.8 cm   frame-to-frame sd 30.2 cm   ratio 0.46
```

The ratio (frame-to-frame sd over sqrt(2) x overall sd) is well below 1, so
most of that spread is the tag genuinely moving, not noise. Subtracting a
plausible ~10 cm of real motion per 100 ms frame leaves roughly
**10-20 cm of TDoA range-difference noise** — somewhat worse than TWR's ~10 cm,
which is expected: TDoA stacks two receive timestamps plus a clock correction.

## What this means for us

**The reference anchor is now a single point of failure.** Position accuracy
depends on the clock model, the clock model depends on `0x1111` being heard.
If a rack or the crane blocks the reference for a region, TDoA there degrades
even when every other anchor is visible. That sharpens the earlier finding
that **blockage, not geometry, is what varies across this hall** — and it is
exactly what the Sionna twin can predict.

**UWB runs at 10 Hz**, twice the Omron's 5 Hz. The ground truth is the coarser
of the two, so interpolate the Omron rather than decimating UWB.

**This log has 4 anchors, not 9.** Either a subset test, or the deployment is
smaller than `environment_oic.json` describes. Worth confirming before sizing
anything on nine.

## What survives in `solver.py`

Unchanged: `FrameClock`, the covariance-from-Jacobian machinery, residual
gating, minimum-anchor checks, the emitted field contract. All of that is
statistics and bookkeeping, independent of how ranges become a position.

Needs replacing: `solve_2d` and `_residuals`, which assume absolute ranges.
TDoA minimises range *differences* against a reference anchor, and the
Jacobian gains a row-differencing term. Roughly 30 lines — plus the clock
model above, which is the real work and does not exist in the current backend
at all.
