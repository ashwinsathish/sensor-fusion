#!/usr/bin/env python3
"""Data collection, driven by single keypresses.

    python3 collect.py

That is the whole command. Sensors connect once and stay up; you press a key
to start a run, drive, press a key to stop. It tells you immediately whether
the run was any good, so you can redo it while you are still standing there.

    1   start a PARKING run    (frame alignment + accuracy)
    2   start a DRIVING run    (latency)
    s   stop the current run
    m   mark this moment       (optional note in the log)
    q   quit

Everything else is automatic: the shared clock, the file layout, the dwell
counting, the quality check.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from recorder import SESSIONS, Health, PiClock, Run   # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import frames                                        # noqa: E402
from endpoints import resolve_broker                 # noqa: E402
frames.apply_site_overrides()


def local_clock_is_synced() -> bool:
    import subprocess
    try:
        return subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized",
                               "--value"], capture_output=True, text=True,
                              timeout=5).stdout.strip().lower() == "yes"
    except Exception:
        return False


class Session:
    def __init__(self, args):
        self.args = args
        self.clock = PiClock()
        # Every latency is `arrival - source time`, so arrival must be stamped on
        # the SAME clock the sources use. That clock is the factory reference,
        # 10.0.0.2 (local stratum-3 NTP; Server2 is PTP-locked and agrees to
        # 0.05 ms). "timedatectl says synchronized" is not enough: the Omron
        # UpBoard reported synchronized while sitting 30 ms off on public NTP.
        #
        # So measure the offset to 10.0.0.2 directly, keep it fresh, and stamp
        # arrivals as local + offset. Fallbacks, in order: this host's own NTP
        # if 10.0.0.2 is unreachable; the Omron collector's clock as a last
        # resort — which is the WORST clock in the system, so it is flagged.
        self.stop = threading.Event()        # needed by the reference thread
        self.clock_ok = local_clock_is_synced()
        self.ref = None                      # latest timeref.measure() result
        self._ref_lock = threading.Lock()
        self._start_ref_thread()

        # UWB can arrive two ways: published to MQTT by the newest backend, or
        # read straight off the localisation server's websocket. Both at once
        # would write every fix twice, so MQTT wins and the websocket stands by.
        self.uwb_mqtt_last: float = 0.0
        self.health = {k: Health() for k in ("omron", "uwb", "camera", "agilox")}
        self.run: Run | None = None
        self.status = {"mqtt": "connecting…", "uwb": "off", "camera": "off"}
        self.root = os.path.abspath(args.out)
        self.finished: list[dict] = []
        os.makedirs(self.root, exist_ok=True)

    def _start_ref_thread(self) -> None:
        import timeref

        def loop():
            while not self.stop.is_set():
                r = timeref.measure(timeref.REFERENCE_NTP, samples=6, timeout=1.0)
                with self._ref_lock:
                    if r is not None:
                        r["measured_at"] = time.time()
                        self.ref = r
                    elif self.ref and time.time() - self.ref["measured_at"] > 180:
                        self.ref = None      # stale: stop trusting it
                self.stop.wait(30.0)

        threading.Thread(target=loop, daemon=True).start()

    @property
    def clock_mode(self) -> str:
        if self.ref is not None:
            return "ref_10.0.0.2"
        if self.clock_ok:
            return "local_ntp"
        return "omron_referenced"

    def now(self) -> float:
        """Arrival time on the factory reference clock, as best we can."""
        t = time.time()
        ref = self.ref
        if ref is not None:
            return t + ref["offset_s"]
        if self.clock_ok:
            return t
        off = self.clock.offset
        return t if off is None else t - off

    # -- routing ---------------------------------------------------------
    def emit(self, kind: str, row: dict) -> None:
        self.health[kind].beat()
        r = self.run
        if r is not None:
            r.write(kind, row)

    def start_run(self, mode: str) -> str | None:
        if self.run is not None:
            return "a run is already going — press s to stop it first"
        if self.clock.offset is None:
            return "no Omron messages yet — no ground truth. Is Omron/status publishing?"
        n = 1 + sum(1 for f in self.finished if f["mode"] == mode)
        name = f"{time.strftime('%Y%m%dT%H%M%S')}_{mode}_{n}"
        self.run = Run(self.root, name, mode)
        return None

    def stop_run(self):
        if self.run is None:
            return None, None
        r = self.run
        checks = r.check(self.clock_mode)
        manifest = r.finish(self.clock, self.clock_mode, self.ref)
        self.run = None
        self.finished.append(manifest)
        return manifest, checks


# ── sources ──────────────────────────────────────────────────────────────────

def _flag(d: dict, S=None) -> dict:
    """The three fields every source shares, computed the same way for all."""
    now = S.now() if S is not None else time.time()
    tv = d.get("t_valid")
    is_arrival = bool(d.get("t_valid_is_arrival")) or tv is None
    if is_arrival:
        tv = now
    tp = d.get("t_published")
    return {
        "t": f"{float(tv):.6f}",
        "t_recv": f"{now:.6f}",
        "latency_s": round(now - float(tv), 6),
        "internal_s": (round(float(tp) - float(tv), 6)
                       if isinstance(tp, (int, float)) else ""),
        "t_published": tp if tp is not None else "",
        "clock_synced": d.get("clock_synced"),
        "seq": d.get("seq", ""),
        "t_valid_is_arrival": is_arrival,
    }


def start_mqtt(S: Session):
    """Subscribe to every source topic. See ../MESSAGE_SPEC.md.

    Both message families are accepted: the `lit/fusion/v1/*` JSON contract,
    and the legacy CSV topics (`Omron/status`, `Agilox/status`) so recording
    works before every publisher has migrated.
    """
    import paho.mqtt.client as mqtt
    a = S.args

    def on_connect(c, u, rc_or_flags, rc=None, p=None):
        S.status["mqtt"] = "connected"
        for t in (a.topic_camera, a.topic_uwb, a.topic_omron, a.topic_agilox,
                  a.topic_uwb_sal, a.omron_topic, a.agilox_topic):
            if t:
                c.subscribe(t, 0)

    def on_disconnect(c, u, f, rc=None, p=None):
        S.status["mqtt"] = "DISCONNECTED"

    def on_message(c, u, m):
        raw = m.payload.decode("utf-8", "replace")
        topic = m.topic

        # ---- lit.fusion.v1 JSON --------------------------------------
        if raw.lstrip().startswith("{"):
            try:
                d = json.loads(raw)
            except Exception:
                return
            if topic == a.topic_uwb_sal:
                now = S.now()
                try:
                    xn, yn = float(d["x"]), float(d["y"])
                except (KeyError, TypeError, ValueError):
                    return
                x, y = frames.uwb_to_factory(xn, yn)
                tr = d.get("t_round_s")
                is_arr = tr is None
                tv = float(tr) if tr is not None else now
                xu = yu = ""
                if d.get("x_raw") is not None and d.get("y_raw") is not None:
                    ux, uy = frames.uwb_to_factory(float(d["x_raw"]), float(d["y_raw"]))
                    xu, yu = f"{ux:.4f}", f"{uy:.4f}"
                S.emit("uwb", {
                    "t": f"{tv:.6f}", "x": f"{x:.4f}", "y": f"{y:.4f}",
                    "x_unfiltered": xu, "y_unfiltered": yu,
                    "x_native": xn, "y_native": yn, "frame_in": "uwb",
                    "t_recv": f"{now:.6f}", "latency_s": round(now - tv, 6),
                    "internal_s": (round(float(d["t_solved_s"]) - tv, 6)
                                   if d.get("t_solved_s") else ""),
                    "t_published": d.get("t_solved_s", ""),
                    "clock_synced": d.get("clock_synced"),
                    "seq": d.get("solved_round_idx", d.get("round_idx", "")),
                    "t_valid_is_arrival": is_arr,
                    "frame_nr": d.get("solved_round_idx", d.get("round_idx", "")),
                    "n_anchors": d.get("n_anchors", ""),
                    "anchor_earliest": d.get("anchor_earliest", ""),
                    "anchors_used": "|".join(str(v) for v in (d.get("anchors_used") or [])),
                    "residual_m": "", "sigma_m": "",
                    "method": "tdoa", "tag_node_id": d.get("tag_node_id", ""),
                    "rounds_deferred": d.get("rounds_deferred", ""),
                })
                S.uwb_mqtt_last = time.time()
                S.status["uwb"] = "via MQTT"
                return

            kind = d.get("source") or {a.topic_camera: "camera",
                                       a.topic_uwb: "uwb",
                                       a.topic_omron: "omron",
                                       a.topic_agilox: "agilox"}.get(topic)
            if kind not in S.health:
                return
            base = _flag(d, S)
            try:
                xr, yr = float(d["x"]), float(d["y"])
            except (KeyError, TypeError, ValueError):
                return
            # Publishers may send their native frame; the collector normalises
            # to factory interior and keeps the raw values so nothing is lost
            # and a wrong transform can be undone later.
            fin = str(d.get("frame", "factory_interior"))
            if fin in ("uwb", "uwb_frame"):
                x, y = frames.uwb_to_factory(xr, yr)
            elif fin in ("omron_raw_mm", "omron_mm"):
                x, y = frames.omron_to_factory(xr, yr)
            elif fin in ("agilox_raw_mm", "agilox_mm"):
                x, y = frames.agilox_to_factory(xr, yr)
            else:
                x, y = xr, yr
            base["x"] = f"{x:.4f}"; base["y"] = f"{y:.4f}"
            base["x_native"] = xr; base["y_native"] = yr; base["frame_in"] = fin
            if kind == "camera":
                base.update({"class": d.get("class", ""), "conf": d.get("conf", ""),
                             "n_cams": d.get("n_cams", ""),
                             "cams": "|".join(str(v) for v in d.get("cams", [])),
                             "vx": d.get("vx", ""), "vy": d.get("vy", ""),
                             "t_frame_arrived": d.get("t_frame_arrived", ""),
                             "t_detect_done": d.get("t_detect_done", "")})
            elif kind == "uwb":
                base.update({"frame_nr": d.get("frame_nr", ""),
                             "n_anchors": d.get("n_anchors", ""),
                             "anchor_earliest": d.get("anchor_earliest", ""),
                             "anchors_used": "|".join(d.get("anchors_used", [])),
                             "residual_m": d.get("residual_m", ""),
                             "sigma_m": d.get("sigma_m", ""),
                             "method": d.get("method", "")})
            elif kind == "omron":
                base.update({"theta_deg": d.get("theta_deg", ""),
                             "status": d.get("status", ""),
                             "loc_score": d.get("loc_score", "")})
            S.emit(kind, base)

            # The un-aggregated per-camera view: which camera saw it, at which
            # pixel. Aggregating four cameras destroys exactly the information
            # a per-region trust map is built from, so keep the parts too.
            if kind == "camera" and S.run is not None:
                for pc in d.get("per_cam", []) or []:
                    try:
                        S.run.write("per_cam", {
                            "t": f"{float(pc.get('t_valid', d['t_valid'])):.6f}",
                            "x": f"{float(pc['x']):.4f}", "y": f"{float(pc['y']):.4f}",
                            "t_recv": base["t_recv"], "cam": pc.get("cam", ""),
                            "px": pc.get("px", ""), "py": pc.get("py", ""),
                            "conf": pc.get("conf", ""), "seq": d.get("seq", "")})
                    except (KeyError, TypeError, ValueError):
                        continue
            if kind == "uwb":
                S.uwb_mqtt_last = time.time()
                S.status["uwb"] = "via MQTT"
            if kind == "omron":
                try:
                    S.clock.add(time.time(), float(d["t_valid"]))
                    if S.run is not None:
                        S.run.dwell.add(float(d["t_valid"]), x, y)
                        S.run.motion.add(float(d["t_valid"]), x, y)
                except (KeyError, TypeError, ValueError):
                    pass
            return

            return

        # ---- SAL UWB publisher (topic UWB/position) ------------------------
        # Their TDoA backend publishes {x, y, z, tag_node_id, timestamp} in the
        # UWB frame. With uwb/patch_backend.py applied it also carries
        # t_round_s (when the round happened, as opposed to `timestamp` which
        # is the arrival of whatever message triggered the solve) and x_raw/
        # y_raw (the unfiltered TDoA fix, before the Kalman filter).
        # Handled here so no extra bridge process is needed.

        # ---- legacy CSV topics ---------------------------------------
        row = raw.split(",")
        now = S.now()
        if topic == a.omron_topic:
            try:
                t_pi = float(row[0])
                xr, yr = float(row[3]), float(row[4])
            except (ValueError, IndexError):
                return
            if not (1.0e9 < t_pi < 4.0e9):
                return
            # mm -> factory interior metres via the site two-point calibration.
            # A bare 1e-3 here (what this did before) lands the ground truth
            # ~21 m from where the cameras put the same robot.
            x, y = frames.omron_to_factory(xr, yr)
            S.clock.add(time.time(), t_pi)
            S.emit("omron", {
                "t": f"{t_pi:.6f}", "x": f"{x:.4f}", "y": f"{y:.4f}",
                "x_native": xr, "y_native": yr, "frame_in": "omron_raw_mm",
                "t_recv": f"{now:.6f}", "latency_s": round(now - t_pi, 6),
                "clock_synced": "", "seq": "",
                "theta_deg": row[5] if len(row) > 5 else "",
                "status": row[1].strip().strip("\'\"") if len(row) > 1 else "",
                "loc_score": row[6] if len(row) > 6 else "",
                "t_valid_is_arrival": False})
            if S.run is not None:
                S.run.dwell.add(t_pi, x, y)
                S.run.motion.add(t_pi, x, y)
        elif topic == a.agilox_topic:
            try:
                xr, yr = float(row[1]), float(row[2])
            except (ValueError, IndexError):
                return
            x, y = frames.agilox_to_factory(xr, yr)
            S.emit("agilox", {"t": f"{now:.6f}", "x": f"{x:.4f}", "y": f"{y:.4f}",
                              "x_native": xr, "y_native": yr, "frame_in": "agilox_raw_mm",
                              "t_recv": f"{now:.6f}", "latency_s": "",
                              "clock_synced": "", "seq": row[0] if row else "",
                              "t_valid_is_arrival": True})

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, f"lit-collect-{os.getpid()}")
    cli.on_connect, cli.on_message, cli.on_disconnect = on_connect, on_message, on_disconnect
    cli.connect(S.args.broker_host, S.args.broker_port, 30)
    cli.loop_start()
    return cli


def start_uwb(S: Session):
    import asyncio

    async def reader():
        import websockets
        while not S.stop.is_set():
            try:
                async with websockets.connect(S.args.uwb_ws, ping_interval=5,
                                              ping_timeout=5) as ws:
                    S.status["uwb"] = "connected"
                    while not S.stop.is_set():
                        try:
                            msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
                        except asyncio.TimeoutError:
                            continue
                        now = time.time()
                        try:
                            d = json.loads(msg)
                        except Exception:
                            continue
                        if not isinstance(d, dict) or d.get("type") != "UWB":
                            continue
                        if now - S.uwb_mqtt_last < 10.0:
                            # the same fixes are already arriving on MQTT with
                            # better metadata; do not record them twice
                            S.status["uwb"] = "standby (MQTT is providing UWB)"
                            continue
                        src = "t_round" if d.get("t_round_s") else "arrival"
                        base = d["t_round_s"] if d.get("t_round_s") else now
                        t_pi = S.clock.to_pi(base)
                        if t_pi is None:
                            continue
                        S.emit("uwb", {
                            "t": f"{t_pi:.6f}",
                            "x": f"{float(d.get('x', 'nan')):.4f}",
                            "y": f"{float(d.get('y', 'nan')):.4f}",
                            "t_local": f"{now:.6f}", "t_source": src,
                            "frame_nr": d.get("FrameNr", ""),
                            "n_anchors": d.get("n_anchors", ""),
                            "residual_m": d.get("residual_m", ""),
                            "sigma_m": d.get("sigma_m", ""),
                            "hdop": d.get("hdop", "")})
            except Exception as exc:                       # noqa: BLE001
                S.status["uwb"] = f"retrying ({type(exc).__name__})"
                await asyncio.sleep(1.0)

    threading.Thread(target=lambda: asyncio.run(reader()), daemon=True).start()


def start_cameras(S: Session):
    import multiprocessing as mp
    import queue as queuelib
    import yaml
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "camera"))
    from camera_worker_rtsp import camera_worker            # noqa: E402

    factory = yaml.safe_load(open(S.args.factory_config))
    q: "mp.Queue" = mp.Queue()
    stopev = mp.Event()
    procs = []
    for path in S.args.camera_config:
        cfg = yaml.safe_load(open(path))
        p = mp.Process(target=camera_worker, args=(cfg, factory, q, stopev), daemon=True)
        p.start()
        procs.append(p)
    S.status["camera"] = f"{len(procs)} worker(s)"

    def drain():
        while not S.stop.is_set():
            try:
                d = q.get(timeout=0.3)
            except queuelib.Empty:
                continue
            t_pi = S.clock.to_pi(d["ts"])
            if t_pi is None:
                continue
            S.emit("camera", {
                "t": f"{t_pi:.6f}", "x": f"{d['world'][0]:.4f}",
                "y": f"{d['world'][1]:.4f}", "t_local": f"{d['ts']:.6f}",
                "t_source": "shutter", "cam": d.get("cam", ""),
                "class": d.get("class", ""), "conf": f"{d.get('conf', 0):.3f}",
                "track_id": d.get("local_id", ""),
                "px": f"{d['pixel'][0]:.1f}", "py": f"{d['pixel'][1]:.1f}",
                "transport_s": d.get("transport_s", ""),
                "inference_s": d.get("inference_s", "")})

    threading.Thread(target=drain, daemon=True).start()
    return procs, stopev


# ── display ──────────────────────────────────────────────────────────────────

BOLD, DIM, RED, GRN, YEL, RST = "\033[1m", "\033[2m", "\033[31m", "\033[32m", "\033[33m", "\033[0m"


def screen(S: Session) -> str:
    L = []
    r = S.run
    if r is None:
        L.append(f"{BOLD}  IDLE — nothing is being recorded{RST}")
    else:
        m, e = r.mode.upper(), r.elapsed
        L.append(f"{BOLD}{GRN}  ● RECORDING  {m}  {r.name}   "
                 f"{int(e)//60:02d}:{int(e)%60:02d}{RST}")
    L.append("")
    off = (f"{S.clock.offset:+.4f} s ({S.clock.n} samples)"
           if S.clock.offset is not None else f"{RED}waiting for Omron…{RST}")
    mode = S.clock_mode
    if mode == "ref_10.0.0.2":
        ro = S.ref["offset_s"] * 1000
        col = GRN if abs(ro) < 2 else YEL
        L.append(f"  clock  {col}factory reference 10.0.0.2{RST}   "
                 f"this host {ro:+.2f} ms (corrected)")
    elif mode == "local_ntp":
        L.append(f"  clock  {YEL}10.0.0.2 unreachable — using this host's own NTP{RST}")
    else:
        L.append(f"  clock  {RED}no reference, no NTP — corrected via the Omron "
                 f"collector, the least accurate clock (~30 ms){RST}")
    L.append(f"         Omron collector vs this host: {off}")
    L.append("")
    L.append(f"  {'sensor':10s} {'Hz':>6s} {'last':>7s}  {'this run':>9s}   state")
    for key, label in (("omron", "OMRON gt"), ("uwb", "UWB"),
                       ("camera", "CAMERA"), ("agilox", "agilox")):
        h = S.health[key]
        age = h.age_s
        n = r.streams[key].n if r else 0
        if h.total == 0:
            flag = f"{DIM}not connected{RST}"
        elif age > 3:
            flag = f"{RED}!! STALLED {age:.0f}s{RST}"
        else:
            flag = f"{GRN}ok{RST}"
        extra = S.status.get({"uwb": "uwb", "camera": "camera"}.get(key, ""), "")
        L.append(f"  {label:10s} {h.hz:6.1f} "
                 f"{('--' if age == math.inf else f'{age:5.1f}s'):>7s}  "
                 f"{n:9d}   {flag} {DIM}{extra}{RST}")
    L.append("")
    if r is not None and r.mode == "parking":
        held, need = r.dwell.held_s, r.dwell.min_dur_s
        bar = "█" * min(24, int(held / need * 24))
        L.append(f"  {BOLD}DWELLS {len(r.dwell.dwells)}/{S.args.want_dwells}{RST}"
                 f"    holding {held:4.1f}s [{bar:<24s}]")
        L.append(f"  {DIM}park, hold until the bar fills, then drive to the next spot{RST}")
    elif r is not None:
        L.append(f"  {BOLD}stop/go transitions {r.motion.stop_go}{RST}"
                 f"    speed {r.motion.speed_mps:4.2f} m/s")
        L.append(f"  {DIM}drive with sharp starts and stops — not a smooth loop{RST}")
    else:
        done = ", ".join(f["name"] for f in S.finished[-3:]) or "none yet"
        L.append(f"  runs so far: {len(S.finished)}   {DIM}{done}{RST}")
    L.append("")
    L.append(f"  {BOLD}[1]{RST} parking run   {BOLD}[2]{RST} driving run   "
             f"{BOLD}[s]{RST} stop   {BOLD}[m]{RST} mark   {BOLD}[q]{RST} quit")
    return "\n".join(L)


def report(checks) -> None:
    for lvl, msg in checks:
        icon = {"ok": f"{GRN}✓{RST}", "warn": f"{YEL}!{RST}", "bad": f"{RED}✗{RST}"}[lvl]
        print(f"    {icon} {msg}")


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=SESSIONS)
    ap.add_argument("--broker-host", default="auto",
                    help="host, or 'auto' to find it (see ../endpoints.py)")
    ap.add_argument("--broker-port", type=int, default=1833)
    ap.add_argument("--omron-topic", default="Omron/status")
    ap.add_argument("--agilox-topic", default="Agilox/status")
    ap.add_argument("--topic-camera", default="lit/fusion/v1/camera")
    ap.add_argument("--topic-uwb", default="lit/fusion/v1/uwb")
    ap.add_argument("--topic-omron", default="lit/fusion/v1/omron")
    ap.add_argument("--topic-agilox", default="lit/fusion/v1/agilox")
    ap.add_argument("--topic-uwb-sal", default="UWB/position",
                    help="the SAL TDoA backend's own topic")
    ap.add_argument("--uwb-ws", default="auto",
                    help="the localisation server's websocket, or 'auto' to "
                         "find it (see ../endpoints.py)")
    ap.add_argument("--no-uwb", action="store_true")
    ap.add_argument("--camera-config", nargs="*", default=[])
    ap.add_argument("--factory-config",
                    default=os.path.join(os.environ.get("CAM_TRACKING_REPO", os.path.expanduser("~/Cam-tracking-LIT")), "config", "factory.yaml"))
    ap.add_argument("--want-dwells", type=int, default=8)
    args = ap.parse_args()
    args.broker_host, args.broker_port = resolve_broker(
        args.broker_host, args.broker_port)
    if args.uwb_ws == "auto" and not args.no_uwb:
        from endpoints import find_uwb_ws
        found = find_uwb_ws()
        if found:
            args.uwb_ws = f"ws://{found[0]}:{found[1]}/ws"
            print(f"uwb websocket: {found[0]}:{found[1]}  ({found[2]})")
        else:
            args.uwb_ws = ""
            print("uwb websocket: none found — UWB will only be recorded if it "
                  "arrives on MQTT")

    S = Session(args)
    print("connecting sensors…")
    cli = start_mqtt(S)
    if not args.no_uwb and args.uwb_ws:
        start_uwb(S)
    cams = None
    if args.camera_config:
        try:
            cams = start_cameras(S)
        except Exception as exc:                            # noqa: BLE001
            S.status["camera"] = f"failed: {exc}"
    time.sleep(2.0)

    fd = sys.stdin.fileno()
    interactive = sys.stdin.isatty()
    old = termios.tcgetattr(fd) if interactive else None
    if interactive:
        tty.setcbreak(fd)
    height = 0
    msg = ""
    try:
        while not S.stop.is_set():
            key = ""
            if interactive and select.select([sys.stdin], [], [], 0.5)[0]:
                key = sys.stdin.read(1).lower()
            else:
                time.sleep(0.5 if not interactive else 0)

            if key in ("1", "2"):
                err = S.start_run("parking" if key == "1" else "driving")
                msg = err or ""
            elif key == "s" and S.run is not None:
                manifest, checks = S.stop_run()
                if interactive:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
                print("\n" * 2)
                print(f"  {BOLD}{manifest['name']}{RST}  "
                      f"{manifest['duration_s']/60:.1f} min")
                report(checks)
                print(f"    {DIM}{os.path.join(S.root, manifest['name'])}{RST}\n")
                if interactive:
                    tty.setcbreak(fd)
                height = 0
                msg = ""
            elif key == "m" and S.run is not None:
                S.run.mark("manual", S.clock.to_pi(time.time()))
                msg = f"marked ({len(S.run.marks)})"
            elif key == "q":
                break

            if S.run is not None:
                ref = S.ref
                S.run.clock_log.write({
                    "t_local": f"{time.time():.3f}",
                    "mode": S.clock_mode,
                    "ref_offset_s": f"{ref['offset_s']:.6f}" if ref else "",
                    "ref_delay_s": f"{ref['delay_s']:.6f}" if ref else "",
                    "omron_offset_s": (f"{S.clock.offset:.6f}"
                                       if S.clock.offset is not None else ""),
                    "n": S.clock.n})
            block = screen(S) + (f"\n\n  {YEL}{msg}{RST}" if msg else "")
            if height:
                sys.stdout.write(f"\033[{height}A")
            sys.stdout.write("\033[J" + block + "\n")
            sys.stdout.flush()
            height = block.count("\n") + 1
    except KeyboardInterrupt:
        pass
    finally:
        if S.run is not None:
            manifest, checks = S.stop_run()
            if interactive:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print(f"\n\n  auto-stopped {manifest['name']}")
            report(checks)
        elif interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        S.stop.set()
        try:
            cli.loop_stop(); cli.disconnect()
        except Exception:                                   # noqa: BLE001
            pass
        if cams:
            procs, stopev = cams
            stopev.set()
            for p in procs:
                p.join(timeout=5)

    print(f"\n{len(S.finished)} run(s) in {S.root}")
    for f in S.finished:
        print(f"  {f['name']:34s} {f['duration_s']/60:5.1f} min  "
              f"gt {f['rows']['omron']:6d}  cam {f['rows']['camera']:6d}  "
              f"uwb {f['rows']['uwb']:6d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
