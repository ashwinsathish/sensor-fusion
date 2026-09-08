#!/usr/bin/env python3
"""One place for every coordinate transform in the LIT fusion dataset.

Three systems, three native frames, one target. Getting any of these wrong
produces a dataset where the sensors disagree by metres in a way that looks
like sensor error and is almost impossible to diagnose afterwards — so they
live here, together, with a self-test that checks them against known geometry.

**Target frame — "factory interior".** Origin at the inner wall corner of the
hall, x right, y up, metres. x in [0, 39], y in [0, 12.5]. This is the frame
`Cam-tracking-LIT/config/factory.yaml` already uses, and the same one the
floor-plan overlay uses.

| source | native frame | transform |
|---|---|---|
| camera | already factory interior | none |
| UWB | mirrored on both axes, origin offset | `uwb-factory-overlay/coords.py` |
| Omron | raw millimetres, two-point affine | `LIT_fac_ray_tracing` site profile |
| Agilox | raw millimetres, offset | same site profile |

Run `python3 frames.py` to self-test.
"""

from __future__ import annotations

# ── UWB → factory interior ───────────────────────────────────────────────────
# From uwb-factory-overlay/coords.py. Both axes are mirrored and UWB's origin
# sits at x = 14.8 in factory metres, which makes the map its own inverse.
UWB_X_OFFSET_M = 14.8


def uwb_to_factory(x: float, y: float) -> tuple[float, float]:
    """UWB frame -> factory interior. Self-inverse."""
    return UWB_X_OFFSET_M - x, -y


factory_to_uwb = uwb_to_factory


# ── Omron → factory interior ─────────────────────────────────────────────────
# Two-point affine calibration from sites/lit_factory.yaml (`coords:`), with
# the same fallbacks scripts/robot_mqtt_positions.py carries. The Omron reports
# millimetres in its own MobilePlanner map frame.
OMRON_X_SCALE = 1.0178592548787697
OMRON_Y_SCALE = 1.005509641873278
OMRON_X_OFFSET_M = 21.029613719692485
OMRON_Y_OFFSET_M = -1.3286639118457295
OMRON_Z_M = 0.5

AGILOX_SCALE_M_PER_MM = 0.001
AGILOX_X_OFFSET_M = -3.468
AGILOX_Y_OFFSET_M = -6.096


def omron_to_factory(x_mm: float, y_mm: float) -> tuple[float, float]:
    """Omron raw millimetres -> factory interior metres."""
    return (x_mm * 1e-3 * OMRON_X_SCALE + OMRON_X_OFFSET_M,
            y_mm * 1e-3 * OMRON_Y_SCALE + OMRON_Y_OFFSET_M)


def agilox_to_factory(x_mm: float, y_mm: float) -> tuple[float, float]:
    return (x_mm * AGILOX_SCALE_M_PER_MM + AGILOX_X_OFFSET_M,
            y_mm * AGILOX_SCALE_M_PER_MM + AGILOX_Y_OFFSET_M)


def load_site_overrides(repo: str = "/home/sathishkumara/LIT_fac_ray_tracing") -> dict:
    """Pull the live calibration from sites/<site>.yaml if it is readable.

    The constants above are a snapshot. If someone recalibrates the Omron in
    the site profile and this module keeps its own copy, the dataset silently
    drifts away from the rest of the stack — so prefer the profile when present.
    """
    import glob
    import os
    out = {}
    for path in glob.glob(os.path.join(repo, "sites", "*.yaml")):
        try:
            import yaml
            c = (yaml.safe_load(open(path)) or {}).get("coords") or {}
        except Exception:
            continue
        for key, name in (("omron_x_scale", "OMRON_X_SCALE"),
                          ("omron_y_scale", "OMRON_Y_SCALE"),
                          ("omron_x_offset_m", "OMRON_X_OFFSET_M"),
                          ("omron_y_offset_m", "OMRON_Y_OFFSET_M"),
                          ("agilox_x_offset_m", "AGILOX_X_OFFSET_M"),
                          ("agilox_y_offset_m", "AGILOX_Y_OFFSET_M")):
            if key in c:
                out[name] = float(c[key])
        if out:
            out["_source"] = path
            break
    return out


def apply_site_overrides(repo: str = "/home/sathishkumara/LIT_fac_ray_tracing") -> dict:
    g = globals()
    over = load_site_overrides(repo)
    for k, v in over.items():
        if not k.startswith("_"):
            g[k] = v
    return over


FLOOR_X = (0.0, 39.0)
FLOOR_Y = (0.0, 12.5)


def in_hall(x: float, y: float, margin: float = 1.0) -> bool:
    return (FLOOR_X[0] - margin <= x <= FLOOR_X[1] + margin
            and FLOOR_Y[0] - margin <= y <= FLOOR_Y[1] + margin)


# ── self-test ────────────────────────────────────────────────────────────────

def _selftest() -> int:
    import json
    import os
    ok = True
    print("coordinate frames self-test\n")

    over = apply_site_overrides()
    if over:
        print(f"  site profile overrides loaded from {over['_source']}")
        for k, v in over.items():
            if not k.startswith("_"):
                print(f"    {k} = {v}")
    else:
        print("  no site profile found — using the built-in snapshot")
    print()

    # 1. UWB anchors must land inside the hall once transformed. This is a
    #    strong check: nine anchors mounted around a 39 x 12.5 m hall cannot
    #    all fall inside it by accident if the transform is wrong.
    env = next((e for e in (
        "/home/sathishkumara/tdoa_uwb/environments/environment_oic9_M2.json",
        "/home/sathishkumara/uwb-visualization/environments/environment_oic.json")
        if os.path.exists(e)), None)
    if env:
        anchors = json.load(open(env))["anchors"]
        xs, ys, outside = [], [], []
        for a in anchors:
            p = a["position"]
            x, y = uwb_to_factory(p["x"], p["y"])
            xs.append(x); ys.append(y)
            if not in_hall(x, y, margin=0.5):
                outside.append((a["id"], round(x, 2), round(y, 2)))
        print(f"  UWB: {len(anchors)} anchors from {os.path.basename(env)} "
              f"-> factory frame")
        print(f"    x {min(xs):6.2f} .. {max(xs):6.2f}   y {min(ys):6.2f} .. {max(ys):6.2f}")
        print(f"    hall is x {FLOOR_X[0]}..{FLOOR_X[1]}, y {FLOOR_Y[0]}..{FLOOR_Y[1]}")
        if outside:
            print(f"    ✗ {len(outside)} anchor(s) land outside the hall: {outside}")
            ok = False
        else:
            print("    ✓ every anchor lands inside the hall — transform is right")
        rx, ry = uwb_to_factory(*uwb_to_factory(3.0, -4.0))
        if abs(rx - 3.0) > 1e-9 or abs(ry + 4.0) > 1e-9:
            print("    ✗ transform is not self-inverse")
            ok = False
    else:
        print("  UWB: environment_oic.json not found, skipped")
    print()

    # 2. Omron: a live-observed raw position must land inside the hall.
    for raw, label in (((-11782.0, 2093.0), "observed 2 Sep"),
                       ((-9665.18, 2443.85), "dock reference")):
        x, y = omron_to_factory(*raw)
        good = in_hall(x, y, margin=0.5)
        print(f"  Omron {label}: raw {raw} mm -> ({x:6.2f}, {y:5.2f}) m  "
              f"{'✓ inside' if good else '✗ OUTSIDE the hall'}")
        ok = ok and good
    print()

    # 3. Agilox
    x, y = agilox_to_factory(19060.0, 11092.0)
    good = in_hall(x, y, margin=1.5)
    print(f"  Agilox observed raw (19060, 11092) mm -> ({x:6.2f}, {y:5.2f}) m  "
          f"{'✓ inside' if good else '✗ OUTSIDE the hall'}")
    ok = ok and good

    print("\n" + ("all transforms check out" if ok else "TRANSFORM PROBLEM — do not collect"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
