#!/usr/bin/env python3
"""Run this before collecting. It takes 60 seconds and it can save a day.

Checks, in the order that matters:

  1. this machine's clock is disciplined       — everything rests on it
  2. the broker is reachable
  3. the coordinate transforms are right
  4. each source is publishing, at what rate, with usable timestamps
  5. how well the source clocks agree with the collector's
  6. every source puts the robot INSIDE the hall
  7. the sources agree with each other about where the robot is

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
DIM = "\033[2m"
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
    import timeref
    print(f"\n{B}1. this machine's clock{X}")
    try:
        out = subprocess.run(["timedatectl"], capture_output=True, text=True,
                             timeout=5).stdout
        synced = "System clock synchronized: yes" in out
    except Exception:
        synced, out = False, ""
    globals()["LOCAL_SYNCED"] = synced

    # "synchronized: yes" is not enough — the Omron UpBoard said yes while
    # sitting 30 ms off on public NTP. Measure against the factory reference.
    r = timeref.measure(timeref.REFERENCE_NTP, samples=8)
    globals()["REF"] = r
    if r is None:
        say("warn", f"factory reference {timeref.REFERENCE_NTP} not reachable over "
                    f"NTP (off the factory network, or UDP/123 blocked). Local NTP "
                    f"synchronized: {'yes' if synced else 'NO'}.")
        return
    off = r["offset_s"] * 1000
    msg = (f"{abs(off):.2f} ms {'behind' if off > 0 else 'ahead of'} the factory "
           f"reference {timeref.REFERENCE_NTP} (round trip {r['delay_s']*1000:.2f} ms)")
    if abs(off) < 2:
        say("ok", msg)
    elif abs(off) < 50:
        say("warn", msg + ". The collector corrects for this, but fix it: "
                          "sudo tools/set_ntp.sh")
    else:
        say("warn", msg + ". The collector corrects for this, but this machine's "
                          "clock is badly off: sudo tools/set_ntp.sh")


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
        "/home/sathishkumara/tdoa_uwb/environments/environment_oic8_M2.json",
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
            say("ok", f"all {len(pts)} UWB anchors ({os.path.basename(env)}) land inside the hall "
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
                             "lit/fusion/v1/omron", "UWB/position",
                             "Omron/status", "Agilox/status"])
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
        # Same order as the collector: factory reference, then local NTP, then
        # the Omron collector clock as a last resort.
        t = time.time()
        ref = globals().get("REF")
        if ref is not None:
            return t + ref["offset_s"]
        if globals().get("LOCAL_SYNCED", True) or not pi_offsets:
            return t
        return t - min(pi_offsets)

    seen = defaultdict(list)          # topic -> [(t_recv, t_valid|None, x, y, kind)]
    raw_seen = defaultdict(int)
    clock_flags = defaultdict(set)
    uwb_unpatched = set()
    uwb_rounds = defaultdict(list)

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
        if m.topic == "UWB/position":
            # The SAL TDoA backend: UWB frame, per-tag, t_round_s once patched.
            try:
                d = json.loads(body)
                xr, yr = frames.uwb_to_factory(float(d["x"]), float(d["y"]))
            except Exception:
                return
            tag = str(d.get("tag_node_id"))
            kind = f"uwb[{tag}]"
            tv = d.get("t_round_s")
            if tv is None:
                uwb_unpatched.add(tag)
            seen[kind].append((now, tv, xr, yr))
            uwb_rounds[tag].append(d.get("solved_round_idx", d.get("round_idx")))
            return
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
            if globals().get("REF") is not None:
                line += "  (on 10.0.0.2 time)"
            elif not globals().get("LOCAL_SYNCED", True):
                line += "  (corrected via the Omron collector)"
            say("ok", line)
        else:
            say("warn", line + ", NO source timestamp — its latency cannot be "
                               "measured and t_valid will just be arrival time")
        if False in clock_flags.get(kind, set()):
            say("bad", f"{kind} reports clock_synced=false")

    # ── clock agreement ──────────────────────────────────────────────────
    # latency = true_transit + (collector_clock - source_clock), and
    # true_transit is never negative. So the MINIMUM observed latency is an
    # upper bound on how far that source's clock is ahead of the collector's.
    # It is the only direct read on clock agreement we get without touching
    # the source machines.
    print(f"\n{B}5. clock agreement{X}")
    print(f"      {'source':16s} {'min':>9s} {'median':>9s} {'p95':>9s}   reading")
    any_ts = False
    for kind, rows in sorted(seen.items()):
        lat = sorted(r[0] - r[1] for r in rows if r[1] is not None)
        if len(lat) < 10:
            continue
        any_ts = True
        lo, med, hi = lat[0], lat[len(lat)//2], lat[int(len(lat)*0.95)]
        if kind.startswith("uwb["):
            # UWB's floor is not transit: the solver only processes a round once
            # it is >= 2 rounds old, so ~200 ms is by design, not clock error.
            note = "floor includes the solver's >=2-round deferral, not a clock reading"
            if lo < -0.005:
                note = f"{R}clock is >={abs(lo)*1000:.0f} ms AHEAD of this machine{X}"
                _fatal.append(f"{kind}: clock {abs(lo)*1000:.0f} ms ahead of the collector")
        elif lo < -0.005:
            note = f"{R}clock is >={abs(lo)*1000:.0f} ms AHEAD of this machine{X}"
            _fatal.append(f"{kind}: clock {abs(lo)*1000:.0f} ms ahead of the collector")
        elif lo < 0.002:
            note = "agrees to within ~2 ms"
        elif lo < 0.030:
            note = f"clock offset + transit <= {lo*1000:.0f} ms"
        else:
            note = f"{Y}offset + transit <= {lo*1000:.0f} ms — check this clock{X}"
        print(f"      {kind:16s} {lo*1000:8.1f}ms {med*1000:8.1f}ms "
              f"{hi*1000:8.1f}ms   {note}")
    if not any_ts:
        say("warn", "no source carries a timestamp, so clock agreement cannot "
                    "be checked at all")
    else:
        print(f"      {DIM}min latency bounds the clock offset: transit is never "
              f"negative,{X}")
        print(f"      {DIM}so a source cannot appear to arrive before it was "
              f"measured.{X}")
        if globals().get("REF") is None and not globals().get("LOCAL_SYNCED", True):
            # Arrival times were corrected using the Omron's own clock, so the
            # Omron row is zero by construction and says nothing. Only the other
            # sources carry information in this mode.
            print(f"      {Y}No factory reference and no NTP here, so arrival times "
                  f"were corrected{X}")
            print(f"      {Y}against the Omron — its row above is circular. Read the "
                  f"others.{X}")

    print(f"\n{B}6. positions inside the hall{X}")
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

    for tag in sorted(uwb_unpatched):
        say("bad", f"uwb[{tag}] has no t_round_s — the UWB solver is running "
                   f"UNPATCHED. Its `timestamp` is ~200 ms late, so UWB latency "
                   f"would be wrong. Apply sensor-fusion/uwb/patch_backend.py to "
                   f"the repo it runs from and restart it.")
    for tag, rounds in sorted(uwb_rounds.items()):
        r = [x for x in rounds if isinstance(x, int)]
        back = sum(1 for a, b in zip(r, r[1:]) if b < a)
        if len(r) > 20 and back > 2:
            say("bad", f"uwb[{tag}]: round index went backwards {back} times in "
                       f"{len(r)} fixes — two solvers are publishing at once (e.g. "
                       f"someone else running localization_gui.py). Their outputs "
                       f"interleave on the same topic. Make sure only one runs.")
    if not any(k.startswith("uwb") or k == "uwb" for k in seen):
        from endpoints import find_uwb_ws
        ws = find_uwb_ws()
        if ws:
            say("warn", f"UWB is running at {ws[0]}:{ws[1]} ({ws[2]}) but only "
                        f"serves a websocket — it is not on MQTT. Bridge it:\n"
                        f"      python3 uwb/publish_uwb.py --ws ws://{ws[0]}:{ws[1]}/ws")
        else:
            say("warn", "no UWB anywhere — not on MQTT, and no localisation "
                        "server answering. Switching the tag on is not enough; a "
                        "program has to read the anchors and compute positions. "
                        "See AGENT_HANDOVER.md section 6.")

    print(f"\n{B}7. do the sources agree?{X}")
    gt = next((k for k in latest if k.startswith("omron")), None)
    if gt is None:
        # This is not a warning. Without Omron/status there is no ground truth
        # and no fallback master clock — the session would be worthless.
        say("bad", "NO GROUND TRUTH. Omron/status is silent, so there is nothing "
                   "to measure the other sensors against and no clock to fall "
                   "back on. Do not collect.\n"
                   "      The publisher is DataCollector.py on 40.0.0.37 "
                   "(sal-UPN-APL01, an UpBoard), in tmux session `omron` as user "
                   "`sal`, repo ~/workspace/repos/iws-testbed.\n"
                   "      Seen 9 Sep 2026: the process stays alive with no "
                   "traceback and 0.2% CPU, blocked in a socket read. It is HUNG, "
                   "not crashed, so `ps` looks healthy — check whether the CSV it "
                   "writes has stopped growing. Restarting the tmux session "
                   "clears it.")
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
