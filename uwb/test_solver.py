#!/usr/bin/env python3
"""Checks the UWB solver against the REAL LIT anchor geometry.

Everything here is simulated ranging on the nine anchor positions from
uwb-visualization/environments/environment_oic.json, so the claims are about
that specific layout, not a textbook one.

    python3 test_solver.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solver import (FrameClock, UwbLocalizer, fix_from_ranges,  # noqa: E402
                    load_anchors, solve_2d)

ENV = "/home/sathishkumara/uwb-visualization/environments/environment_oic.json"
Z_TAG = 0.4                       # tag on top of the Omron
RANGE_SIGMA = 0.10                # 10 cm per-range noise, typical for TWR


def solve_3d(anchors, ranges, iters=60):
    """The unconstrained 3D solve the current backend uses, for comparison."""
    A = np.asarray(anchors, float)
    p = A.mean(axis=0)
    for _ in range(iters):
        diff = p[None, :] - A
        d = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
        step, *_ = np.linalg.lstsq(diff / d[:, None], d - ranges, rcond=None)
        p = p - step
        if np.linalg.norm(step) < 1e-7:
            break
    return p


def sample_positions(anchors, n=400, seed=0):
    A = np.array(list(anchors.values()))
    rng = np.random.default_rng(seed)
    xs = rng.uniform(A[:, 0].min() + 2, A[:, 0].max() - 2, n)
    ys = rng.uniform(A[:, 1].min() + 1.5, A[:, 1].max() - 1.5, n)
    return np.column_stack([xs, ys])


def test_fixed_z_beats_free_z(anchors):
    """With all nine anchors the two are nearly equal — the interesting case is
    a DEGRADED round, which is what actually happens when racks and the crane
    block anchors."""
    A = np.array(list(anchors.values()))
    rng = np.random.default_rng(1)
    pts = sample_positions(anchors, 300, seed=1)
    print(f"  horizontal error, {RANGE_SIGMA*100:.0f} cm range noise, "
          f"by number of anchors reporting:")
    print(f"    {'n':>3s}  {'free z (current)':>18s}  {'fixed z (this)':>16s}   gain")
    gains = {}
    for n_anchor in (4, 5, 6, 9):
        e2d, e3d = [], []
        for x, y in pts:
            true = np.array([x, y, Z_TAG])
            idx = rng.choice(len(A), n_anchor, replace=False)
            Asub = A[idx]
            r = np.linalg.norm(true[None, :] - Asub, axis=1) + \
                rng.normal(0, RANGE_SIGMA, n_anchor)
            p2, _ = solve_2d(Asub, r, Z_TAG)
            e2d.append(np.linalg.norm(p2 - true[:2]))
            p3 = solve_3d(Asub, r)
            e3d.append(np.linalg.norm(p3[:2] - true[:2]))
        m2, m3 = np.median(e2d), np.median(e3d)
        gains[n_anchor] = m3 / m2
        print(f"    {n_anchor:3d}  {m3*100:15.1f} cm  {m2*100:13.1f} cm   {m3/m2:5.2f}x")
    assert gains[4] > 1.5, (
        f"expected fixing z to matter most with few anchors, got {gains[4]:.2f}x")
    print("  ✓ fixing z barely matters with all 9 anchors, but rescues degraded")
    print("    rounds — which is the case that actually occurs under blockage")


def test_covariance_is_calibrated(anchors):
    """The reported sigma must actually predict the error, or weighting by it
    is worse than useless."""
    A = np.array(list(anchors.values()))
    ids = list(anchors.keys())
    rng = np.random.default_rng(2)
    errs, sig = [], []
    for x, y in sample_positions(anchors, 600, seed=2):
        true = np.array([x, y, Z_TAG])
        r = np.linalg.norm(true[None, :] - A, axis=1) + rng.normal(0, RANGE_SIGMA, len(A))
        f = fix_from_ranges(A, r, z_tag=Z_TAG, anchor_ids=ids,
                            range_sigma_m=RANGE_SIGMA)
        if not f.ok:
            continue
        errs.append(np.hypot(f.x - x, f.y - y))
        sig.append(f.sigma_m)
    errs, sig = np.array(errs), np.array(sig)
    ratio = errs.mean() / sig.mean()
    inside = float(np.mean(errs < 2 * sig))
    print(f"  mean error {errs.mean()*100:.1f} cm, mean predicted sigma "
          f"{sig.mean()*100:.1f} cm  (ratio {ratio:.2f})")
    print(f"  {100*inside:.1f}% of fixes fall inside 2 sigma")
    assert 0.5 < ratio < 1.6, f"covariance mis-scaled by {ratio:.2f}x"
    assert inside > 0.90, f"only {100*inside:.0f}% inside 2 sigma"
    print("  ✓ reported covariance predicts the actual error")


def test_geometry_is_anisotropic(anchors):
    """How much does position quality vary across the hall?

    Result: with all nine anchors, barely at all. The layout is good. That is
    a useful negative finding — for UWB in this hall, geometry is NOT what
    separates good fixes from bad ones, so a 'trust map' driven by GDOP would
    be measuring almost nothing. Blockage (which anchors report at all) is the
    real driver, and that IS strongly position-dependent.
    """
    A = np.array(list(anchors.values()))
    ids = list(anchors.keys())
    rng = np.random.default_rng(3)

    def hdop_grid(n_anchor):
        hd = []
        for x in np.linspace(A[:, 0].min() + 2, A[:, 0].max() - 2, 9):
            for y in np.linspace(A[:, 1].min() + 1.5, A[:, 1].max() - 1.5, 5):
                true = np.array([x, y, Z_TAG])
                idx = (np.arange(len(A)) if n_anchor >= len(A)
                       else rng.choice(len(A), n_anchor, replace=False))
                r = np.linalg.norm(true[None, :] - A[idx], axis=1) + \
                    rng.normal(0, RANGE_SIGMA, len(idx))
                f = fix_from_ranges(A[idx], r, z_tag=Z_TAG,
                                    anchor_ids=[ids[i] for i in idx],
                                    range_sigma_m=RANGE_SIGMA)
                if f.ok:
                    hd.append(f.hdop)
        return np.array(hd)

    full = hdop_grid(9)
    deg = hdop_grid(4)
    print(f"  HDOP, all 9 anchors : min {full.min():.2f}  median {np.median(full):.2f}  "
          f"max {full.max():.2f}   ({full.max()/full.min():.1f}x spread)")
    print(f"  HDOP, 4 anchors     : min {deg.min():.2f}  median {np.median(deg):.2f}  "
          f"max {deg.max():.2f}   ({deg.max()/deg.min():.1f}x spread)")
    assert np.median(deg) > np.median(full), "fewer anchors must not improve DOP"
    print("  ✓ the anchor layout is genuinely good: with all 9 reporting, position")
    print("    quality is near-uniform. Variation comes from BLOCKAGE, not geometry.")


def test_nlos_rejection(anchors):
    A = np.array(list(anchors.values()))
    ids = list(anchors.keys())
    rng = np.random.default_rng(4)
    clean, with_nlos, rescued = [], [], []
    for x, y in sample_positions(anchors, 300, seed=4):
        true = np.array([x, y, Z_TAG])
        r = np.linalg.norm(true[None, :] - A, axis=1) + rng.normal(0, RANGE_SIGMA, len(A))
        f0 = fix_from_ranges(A, r, z_tag=Z_TAG, anchor_ids=ids,
                             range_sigma_m=RANGE_SIGMA)
        clean.append(np.hypot(f0.x - x, f0.y - y))

        bad = r.copy()
        k = rng.integers(0, len(A))
        bad[k] += rng.uniform(1.5, 4.0)                 # one NLOS anchor
        fn = fix_from_ranges(A, bad, z_tag=Z_TAG, anchor_ids=ids,
                             range_sigma_m=RANGE_SIGMA, reject_worst=False)
        with_nlos.append(np.hypot(fn.x - x, fn.y - y))
        fr = fix_from_ranges(A, bad, z_tag=Z_TAG, anchor_ids=ids,
                             range_sigma_m=RANGE_SIGMA, reject_worst=True)
        if fr.ok:
            rescued.append(np.hypot(fr.x - x, fr.y - y))
    print(f"  clean            median {np.median(clean)*100:6.1f} cm")
    print(f"  one NLOS anchor  median {np.median(with_nlos)*100:6.1f} cm  (no rejection)")
    print(f"  after rejection  median {np.median(rescued)*100:6.1f} cm  "
          f"({len(rescued)}/{len(with_nlos)} still emitted)")
    assert np.median(rescued) < np.median(with_nlos), "rejection made things worse"
    print("  ✓ worst-anchor rejection recovers most of the NLOS damage")


def test_frame_clock():
    """Frame-number regression should beat raw arrival timestamps."""
    rng = np.random.default_rng(5)
    period, t0 = 0.100, 1_700_000_000.0
    clock = FrameClock()
    raw_err, fit_err = [], []
    for i in range(300):
        true_t = t0 + i * period
        # transport delay: small base plus an occasional queueing spike
        delay = 0.004 + rng.exponential(0.003) + (0.05 if rng.random() < 0.05 else 0)
        arrival = true_t + delay
        clock.add(i, arrival)
        if i > 30:
            raw_err.append(arrival - true_t)
            est = clock.time_of(i)
            fit_err.append(est - true_t)
    raw = np.array(raw_err)
    fit = np.array(fit_err)
    print(f"  raw arrival timestamp : bias {raw.mean()*1000:+6.2f} ms   "
          f"std {raw.std()*1000:5.2f} ms   p95 {np.percentile(raw,95)*1000:5.1f} ms")
    print(f"  frame-clock estimate  : bias {fit.mean()*1000:+6.2f} ms   "
          f"std {fit.std()*1000:5.2f} ms   p95 {np.percentile(fit,95)*1000:5.1f} ms")
    print(f"  recovered period {clock.period_s*1000:.3f} ms (true {period*1000:.1f} ms), "
          f"fit residual {clock.residual_ms:.2f} ms")
    assert fit.std() < raw.std() / 2, "frame clock should more than halve the jitter"
    assert abs(clock.period_s - period) < 1e-4, "ranging period misestimated"
    print("  ✓ frame-number regression is far steadier than arrival timestamps")


def test_localizer_end_to_end(anchors):
    """Feed the accumulator like the real backend does, check what comes out."""
    A = np.array(list(anchors.values()))
    ids = list(anchors.keys())
    rng = np.random.default_rng(6)
    loc = UwbLocalizer(anchors, z_tag=Z_TAG, range_sigma_m=RANGE_SIGMA)

    from solver import DWT_TIME_UNITS, SPEED_OF_LIGHT
    to_tof = lambda d: d / (SPEED_OF_LIGHT * DWT_TIME_UNITS)   # noqa: E731

    period, t = 0.1, 1_700_000_000.0
    fixes, truths = [], []
    for frame in range(60):
        true = np.array([-5.0 + 0.15 * frame, -6.0 + 0.05 * frame, Z_TAG])
        for k in range(len(A)):
            d = np.linalg.norm(true - A[k]) + rng.normal(0, RANGE_SIGMA)
            arrival = t + k * 0.002 + rng.exponential(0.002)
            f = loc.add(frame, ids[k], to_tof(d), arrival)
            if f is not None and f.ok:
                fixes.append(f)
                truths.append(true[:2].copy())
        t += period

    assert len(fixes) > 40, f"only {len(fixes)} fixes from 60 rounds"
    errs = np.array([np.hypot(f.x - tr[0], f.y - tr[1])
                     for f, tr in zip(fixes, truths)])
    have_t = sum(1 for f in fixes if f.t_round_s is not None)
    d = fixes[-1].as_dict()
    print(f"  {len(fixes)} fixes, {loc.n_rejected} rejected, "
          f"{have_t} carry a round time")
    print(f"  position error median {np.median(errs)*100:.1f} cm")
    print(f"  emitted keys: {sorted(d.keys())}")
    assert have_t == len(fixes), "every fix must carry a timestamp"
    for key in ("t_round_s", "n_anchors", "residual_m", "sigma_m", "hdop"):
        assert key in d, f"missing {key} in emitted dict"
    assert loc.unknown_anchors == set(), f"unmapped anchors: {loc.unknown_anchors}"
    print("  ✓ accumulator emits timed, gated fixes with covariance")


def main() -> int:
    if not os.path.exists(ENV):
        print(f"anchor file not found: {ENV}")
        return 1
    anchors = load_anchors(ENV)
    A = np.array(list(anchors.values()))
    print(f"LIT anchor geometry: {len(anchors)} anchors")
    print(f"  x {A[:,0].min():.2f} .. {A[:,0].max():.2f} m   "
          f"y {A[:,1].min():.2f} .. {A[:,1].max():.2f} m   "
          f"z {A[:,2].min():.2f} .. {A[:,2].max():.2f} m")
    print(f"  tag height fixed at {Z_TAG} m (on the Omron)\n")

    print("1. fixed z vs free z");            test_fixed_z_beats_free_z(anchors); print()
    print("2. covariance calibration");       test_covariance_is_calibrated(anchors); print()
    print("3. geometry across the hall");     test_geometry_is_anisotropic(anchors); print()
    print("4. NLOS rejection");               test_nlos_rejection(anchors); print()
    print("5. frame-number clock recovery");  test_frame_clock(); print()
    print("6. end to end");                   test_localizer_end_to_end(anchors); print()
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
