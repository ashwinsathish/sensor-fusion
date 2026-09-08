#!/usr/bin/env python3
"""Run this before collecting. It takes 60 seconds and it can save a day.

Checks, in the order that matters:

  1. this machine's clock is disciplined       — everything rests on it
  2. the broker is reachable
  3. the coordinate transforms are right
  4. each source is publishing, at what rate, with usable timestamps
  5. every source puts the robot INSIDE the hall
  6. the sources agree with each other about where the robot is

Check 6 is the one that catches a wrong coordinate transform. Three systems
reporting the same robot several metres apart is invisible in a live plot and
obvious here.

    python3 preflight.py
    python3 preflight.py --broker 10.0.0.3 --seconds 90
"""

from __future__ import annotations

import argparse
import json
import os
import statistics as st
import subprocess
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frames                                              # noqa: E402
from endpoints import resolve_broker                       # noqa: E402

G, Y, R, B, X = "\033[32m", "\033[33m", "\033[31m", "\033[1m", "\033[0m"
OK, WARN, BAD = f"{G}✓{X}", f"{Y}!{X}", f"{R}✗{X}"

_fatal = []
_warn = []


def say(level, msg):
    print({"ok": OK, "warn": WARN, "bad": BAD}[level], msg)
    if level == "bad":
        _fatal.append(msg)
    elif level == "warn":
        _warn.append(msg)


def check_clock() -> None:
    print(f"\n{B}1. this machine's clock{X}")
    try:
        out = subprocess.run(["timedatectl"], capture_output=True, text=True,
                             timeout=5).stdout
        synced = "System clock synchronized: yes" in out
    except Exception:
        synced = False
        out = ""
    if synced:
        say("ok", "NTP synchronized")
    else:
        say("warn", "NOT NTP-synchronized. The collector will fall back to "
                    "referencing the Omron Pi's clock (which IS synced), so the "
                    "latencies still come out right — but fix it if you can:  "
                    "sudo timedatectl set-ntp true")
    globals()["LOCAL_SYNCED"] = synced
    for line in out.splitlines():
        if "Local time" in line or "synchronized" in line:
            print(f"      {line.strip()}")


def check_transforms() -> None:
    print(f"\n{B}3. coordinate transforms{X}")
    over = frames.apply_site_overrides()
    if over:
        say("ok", f"site calibration loaded from {os.path.basename(over['_source'])}")
    else:
        say("warn", "no site profile found — using the built-in Omron calibration "
                    "snapshot. If someone recalibrated, this dataset will disagree "
                    "with the rest of the stack.")
    env = next((e for e in (
        "/home/sathishkumara/tdoa_uwb/environments/environment_oic9_M2.json",
        "/home/sathishkumara/uwb-visualization/environments/environment_oic.json")
        if os.path.exists(e)), None)
    if env:
        anchors = json.load(open(env))["anchors"]
        pts = [frames.uwb_to_factory(a["position"]["x"], a["position"]["y"])
               for a in anchors]
        bad = [p for p in pts if not frames.in_hall(*p, margin=0.5)]
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        if bad:
            say("bad", f"{len(bad)} UWB anchor(s) transform to outside the hall — "
                       "the UWB transform is wrong")
        else:
            say("ok", f"all {len(pts)} UWB anchors land inside the hall "
                      f"(x {min(xs):.1f}–{max(xs):.1f}, y {min(ys):.1f}–{max(ys):.1f})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default="auto",
                    help="host, or 'auto' to find it (see endpoints.py)")
    ap.add_argument("--port", type=int, default=1833)
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--topics", nargs="*",
                    default=["lit/fusion/v1/camera", "lit/fusion/v1/uwb",
                             "lit/fusion/v1/omron", "Omron/status", "Agilox/status"])
    args = ap.parse_args()
    args.broker, args.port = resolve_broker(args.broker, args.port)

    print(f"{B}LIT fusion pre-flight{X}")
    check_clock()

    print(f"\n{B}2. broker{X}")
    try:
        import paho.mqtt.client as mqtt
    except ModuleNotFoundError:
        say("bad", "paho-mqtt not installed")
        return 2

    # When this machine's clock is not disciplined, correct arrival times by
    # the measured offset to the Omron Pi — exactly what the collector does —
    # so the latencies reported here are the ones that will be recorded.
    pi_offsets = []

    def now_corrected() -> float:
        t = time.time()
        if globals().get("LOCAL_SYNCED", True) or not pi_offsets:
            return t
        return t - min(pi_offsets)

    seen = defaultdict(list)          # topic -> [(t_recv, t_valid|None, x, y, kind)]
    raw_seen = defaultdict(int)
    clock_flags = defaultdict(set)

    def on_connect(c, u, f, rc, p=None):
        for t in args.topics:
            c.subscribe(t, 0)

    def on_message(c, u, m):
        raw_seen[m.topic] += 1
        body = m.payload.decode("utf-8", "replace")
        # learn the offset first, so `now` is already corrected for this message
        if m.topic.startswith("Omron") and not body.lstrip().startswith("{"):
            try:
                pi_offsets.append(time.time() - float(body.split(",")[0]))
            except (ValueError, IndexError):
                pass
        elif body.lstrip().startswith("{"):
            try:
                _d = json.loads(body)
                if _d.get("source") == "omron" and _d.get("t_valid"):
                    pi_offsets.append(time.time() - float(_d["t_valid"]))
            except Exception:
                pass
        now = now_corrected()
        if body.lstrip().startswith("{"):
            try:
                d = json.loads(body)
            except Exception:
                return
            kind = d.get("source", m.topic)
            try:
                xr, yr = float(d["x"]), float(d["y"])
            except (KeyError, TypeError, ValueError):
                return
            fin = str(d.get("frame", "factory_interior"))
            if fin in ("uwb", "uwb_frame"):
                xr, yr = frames.uwb_to_factory(xr, yr)
            elif fin in ("omron_raw_mm", "omron_mm"):
                xr, yr = frames.omron_to_factory(xr, yr)
            tv = d.get("t_valid") if not d.get("t_valid_is_arrival") else None
            clock_flags[kind].add(bool(d.get("clock_synced")))
            seen[kind].append((now, tv, xr, yr))
        else:
            row = body.split(",")
            if m.topic.startswith("Omron"):
                try:
                    tv = float(row[0])
                    x, y = frames.omron_to_factory(float(row[3]), float(row[4]))
                except (ValueError, IndexError):
                    return
                seen["omron(legacy)"].append((now, tv, x, y))
            elif m.topic.startswith("Agilox"):
                try:
                    x, y = frames.agilox_to_factory(float(row[1]), float(row[2]))
                except (ValueError, IndexError):
                    return
                seen["agilox(legacy)"].append((now, None, x, y))

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, f"lit-preflight-{os.getpid()}")
    cli.on_connect, cli.on_message = on_connect, on_message
    try:
        cli.connect(args.broker, args.port, 20)
    except Exception as exc:                                # noqa: BLE001
        say("bad", f"cannot reach broker {args.broker}:{args.port} — {exc}")
        return 2
    cli.loop_start()
    say("ok", f"connected {args.broker}:{args.port}")
    print(f"      listening {args.seconds:.0f} s …")
    time.sleep(args.seconds)
    cli.loop_stop(); cli.disconnect()

    check_transforms()

    print(f"\n{B}4. sources{X}")
    if not seen:
        say("bad", "nothing arrived on any topic. Is anything publishing?")
    for kind, rows in sorted(seen.items()):
        t = [r[0] for r in rows]
        gaps = [b - a for a, b in zip(t, t[1:])]
        hz = 1 / st.median(gaps) if gaps else 0.0
        lat = [r[0] - r[1] for r in rows if r[1] is not None]
        line = f"{kind}: {len(rows)} msgs, {hz:.1f} Hz"
        if lat:
            med = st.median(lat)
            line += f", latency median {med*1000:.0f} ms"
            if med < 0:
                say("bad", line + "  — NEGATIVE. A publisher's clock is ahead of "
                                  "this machine; the dataset would be unusable.")
                continue
            if med > 1.0:
                say("bad", line + "  — far too large for a LAN. Check that "
                                  "publisher's clock.")
                continue
            if not globals().get("LOCAL_SYNCED", True):
                line += "  (corrected via the Omron Pi)"
            say("ok", line)
        else:
            say("warn", line + ", NO source timestamp — its latency cannot be "
                               "measured and t_valid will just be arrival time")
        if False in clock_flags.get(kind, set()):
            say("bad", f"{kind} reports clock_synced=false")

    print(f"\n{B}5. positions inside the hall{X}")
    latest = {}
    for kind, rows in sorted(seen.items()):
        pts = [(r[2], r[3]) for r in rows[-30:]]
        if not pts:
            continue
        mx = st.median([p[0] for p in pts]); my = st.median([p[1] for p in pts])
        latest[kind] = (mx, my, rows[-1][0])
        if frames.in_hall(mx, my, margin=0.5):
            say("ok", f"{kind} at ({mx:6.2f}, {my:5.2f}) m")
        else:
            say("bad", f"{kind} at ({mx:6.2f}, {my:5.2f}) m — OUTSIDE the "
                       f"{frames.FLOOR_X[1]:.0f} x {frames.FLOOR_Y[1]:.1f} m hall. "
                       "Its coordinate transform is wrong.")

    if not any(k.startswith("uwb") or k == "uwb" for k in seen):
        say("warn", "no UWB source is publishing. Someone has to be running the "
                    "localisation program (tdoa_uwb, branch feat/tag_update_rate, "
                    "localization_gui.py) — turning the tag on is not enough on "
                    "its own. See AGENT_HANDOVER.md section 6.")

    print(f"\n{B}6. do the sources agree?{X}")
    gt = next((k for k in latest if k.startswith("omron")), None)
    if gt is None:
        say("warn", "no ground truth seen — cannot cross-check the others")
    else:
        gx, gy, _ = latest[gt]
        others = [k for k in latest if not k.startswith("omron")
                  and not k.startswith("agilox")]
        if not others:
            say("warn", "only the ground truth is publishing — nothing to compare")
        for k in others:
            x, y, _ = latest[k]
            d = ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
            msg = f"{k} is {d:.2f} m from the ground truth"
            if d < 1.5:
                say("ok", msg)
            elif d < 4.0:
                say("warn", msg + " — larger than expected. Was the robot moving? "
                                  "If it was parked, suspect a transform.")
            else:
                say("bad", msg + " — a coordinate transform is almost certainly "
                                 "wrong. Do not collect until this is under ~1 m "
                                 "with the robot parked.")

    print(f"\n{B}verdict{X}")
    if _fatal:
        print(f"  {R}{len(_fatal)} blocking problem(s){X} — collecting now would "
              f"produce an unusable dataset:")
        for m in _fatal:
            print(f"    {R}✗{X} {m}")
        return 1
    if _warn:
        print(f"  {Y}{len(_warn)} warning(s){X} — collectable, but read them:")
        for m in _warn:
            print(f"    {Y}!{X} {m}")
        return 0
    print(f"  {G}everything checks out. Go collect.{X}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
