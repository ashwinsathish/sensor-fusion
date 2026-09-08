#!/usr/bin/env python3
"""Re-derive each camera's floor homography from a parking run.

The current calibrations were made by clicking floor features on a screenshot
of the camera's web view. That has three problems, all visible in the stored
configs:

* three of cam1's nine reference points, and one of cam2's, sit OUTSIDE the
  captured image (pixel y down to -188 in an 852-pixel-tall frame). Those were
  placed by extrapolation, not by looking at anything.
* self-consistency is only 5-16 cm median and up to 52 cm worst case — and
  that is the fit against its OWN points, so true accuracy is worse.
* each camera's reference points cover a narrow band of floor (cam1: 2 m of y),
  so most of the hall is extrapolated far outside the calibrated region.

A parking run fixes all three at once and needs no clicking. The robot goes to
known places and stops; the Omron says exactly where it is, the camera says
which pixel it is at. That is a pixel-to-world correspondence measured by the
robot itself, at as many well-spread points as you care to drive to.

    python3 fit_homography.py ../sessions/<parking_run>

Writes `homography_fit.yaml` beside the session. It does NOT touch the live
Cam-tracking-LIT configs — compare first, copy across when you are satisfied.

One caveat: the camera reports the bottom-centre of the robot's bounding box,
which is the robot's floor footprint centre, while the Omron reports its own
navigation origin. For a roughly symmetric robot these differ by a small,
heading-dependent amount (<10 cm on an LD-series). Varying the robot's heading
between dwells averages it out; parking at all spots facing the same way bakes
it in as a bias.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np


def load_csv(path, cols):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            try:
                out.append(tuple(float(r[c]) if c not in ("cam", "class") else r[c]
                                 for c in cols))
            except (KeyError, ValueError, TypeError):
                continue
    return out


def fit_dlt(px, world):
    """Homography from pixel to world by direct linear transform."""
    px = np.asarray(px, float)
    world = np.asarray(world, float)
    n = len(px)
    A = np.zeros((2 * n, 9))
    for i in range(n):
        x, y = px[i]
        X, Y = world[i]
        A[2 * i] = [x, y, 1, 0, 0, 0, -X * x, -X * y, -X]
        A[2 * i + 1] = [0, 0, 0, x, y, 1, -Y * x, -Y * y, -Y]
    _u, _s, vt = np.linalg.svd(A)
    H = vt[-1].reshape(3, 3)
    return H / H[2, 2]


def apply_H(H, px):
    px = np.atleast_2d(np.asarray(px, float))
    p = np.hstack([px, np.ones((len(px), 1))])
    q = (H @ p.T).T
    return q[:, :2] / q[:, 2:3]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="a parking-run directory")
    ap.add_argument("--class-name", default="omron")
    ap.add_argument("--guard", type=float, default=1.0,
                    help="seconds trimmed from each end of a dwell")
    ap.add_argument("--min-det", type=int, default=5,
                    help="detections needed at a dwell to use it")
    ap.add_argument("--compare", nargs="*", default=[],
                    help="existing camN.yaml files to compare against")
    args = ap.parse_args()

    man_path = os.path.join(args.session, "session.json")
    if not os.path.exists(man_path):
        print(f"no session.json in {args.session}")
        return 1
    man = json.load(open(man_path))
    dwells = man.get("dwells", [])
    if len(dwells) < 4:
        print(f"only {len(dwells)} dwells — a homography needs 4 minimum, "
              "8+ well spread for a good one.")
        return 1

    gt = load_csv(os.path.join(args.session, "omron.csv"), ("t", "x", "y"))
    cam = load_csv(os.path.join(args.session, "camera.csv"),
                   ("t", "x", "y", "px", "py", "cam", "class"))
    cam = [c for c in cam if str(c[6]) == args.class_name]
    if not gt or not cam:
        print(f"need both omron.csv and camera.csv with '{args.class_name}' rows")
        return 1

    gt = np.array(gt)
    cams = sorted({str(c[5]) for c in cam})
    print(f"{len(dwells)} dwells, {len(gt)} ground-truth rows, "
          f"{len(cam)} camera detections from camera(s) {cams}\n")

    out = {}
    for cid in cams:
        rows = [c for c in cam if str(c[5]) == cid]
        pxs, wds, kept = [], [], []
        for d in dwells:
            a, b = d["t_start"] + args.guard, d["t_end"] - args.guard
            if b <= a:
                continue
            g = gt[(gt[:, 0] >= a) & (gt[:, 0] <= b)]
            r = [c for c in rows if a <= c[0] <= b]
            if len(g) < 3 or len(r) < args.min_det:
                continue
            pxs.append([float(np.median([c[3] for c in r])),
                        float(np.median([c[4] for c in r]))])
            wds.append([float(np.median(g[:, 1])), float(np.median(g[:, 2]))])
            kept.append((len(r), float(np.std([c[3] for c in r]))))

        print(f"── camera {cid}: {len(pxs)} usable dwells")
        if len(pxs) < 4:
            print("   not enough — this camera did not see the robot at enough spots\n")
            continue

        H = fit_dlt(pxs, wds)
        pred = apply_H(H, pxs)
        err = np.linalg.norm(pred - np.array(wds), axis=1)
        # leave-one-out: the honest estimate of how it will generalise
        loo = []
        for i in range(len(pxs)):
            idx = [j for j in range(len(pxs)) if j != i]
            if len(idx) < 4:
                continue
            Hi = fit_dlt([pxs[j] for j in idx], [wds[j] for j in idx])
            loo.append(np.linalg.norm(apply_H(Hi, [pxs[i]])[0] - wds[i]))
        loo = np.array(loo) if loo else np.array([np.nan])

        span = np.ptp(np.array(wds), axis=0)
        print(f"   world coverage {span[0]:.1f} x {span[1]:.1f} m")
        print(f"   fit residual        median {np.median(err)*100:6.1f} cm   "
              f"max {err.max()*100:6.1f} cm")
        print(f"   leave-one-out error median {np.nanmedian(loo)*100:6.1f} cm   "
              f"max {np.nanmax(loo)*100:6.1f} cm   <- expect this in use")
        out[cid] = {"homography": H.tolist(),
                    "homography_image_size": None,
                    "n_dwells": len(pxs),
                    "fit_residual_median_m": float(np.median(err)),
                    "loo_median_m": float(np.nanmedian(loo)),
                    "points": [{"pixel": p, "world": w} for p, w in zip(pxs, wds)]}
        print()

    for path in args.compare:
        import yaml
        c = yaml.safe_load(open(path))
        cid = str(c.get("id"))
        if cid not in out:
            continue
        Hold = np.array(c["homography"], float)
        cw, ch = c.get("homography_image_size", [0, 0])
        pts = out[cid]["points"]
        # old homography expects calibration-frame pixels; scale ours into it
        sx, sy = cw / 2560.0, ch / 1440.0
        old_pred = apply_H(Hold, [[p["pixel"][0] * sx, p["pixel"][1] * sy] for p in pts])
        old_err = np.linalg.norm(old_pred - np.array([p["world"] for p in pts]), axis=1)
        print(f"── camera {cid}: existing calibration measured against the robot")
        print(f"   median {np.median(old_err)*100:6.1f} cm   max {old_err.max()*100:6.1f} cm"
              f"   (new fit LOO: {out[cid]['loo_median_m']*100:.1f} cm)")

    if not out:
        print("nothing fitted")
        return 1
    dst = os.path.join(args.session, "homography_fit.yaml")
    try:
        import yaml
        with open(dst, "w") as fh:
            yaml.safe_dump({"cameras": out}, fh, sort_keys=False)
    except ImportError:
        dst = dst.replace(".yaml", ".json")
        json.dump({"cameras": out}, open(dst, "w"), indent=2)
    print(f"\nwrote {dst}")
    print("Pixels in this fit are FULL-FRAME 2560x1440 coordinates, so set")
    print("`homography_image_size: [2560, 1440]` if you copy it into a camN.yaml.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
