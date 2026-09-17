#!/usr/bin/env python3
"""Make a UWB backend emit *when* each fix happened, plus its quality.

Handles both backends in the SAL repo:

  backend/uwb_localization_backend.py       two-way-ranging multilateration
  backend/uwb_tdoa_localization_backend.py  TDoA (feat/tag_update_rate branch)

Neither publishes a usable timestamp as shipped, for different reasons.

**TWR backend** drops the time entirely — it emits `{x, y, FrameNr, type}`.

**TDoA backend** publishes `timestamp`, but that is the arrival time of
whichever message happened to *trigger* the solve, not the instant of the round
being solved. `evaluate_measurements()` deliberately waits until a round is at
least 2 rounds old before solving it, so at 10 Hz the published timestamp is
~200 ms younger than the measurement it labels. Using it as-is would make UWB
look 200 ms faster than it is and attribute every position to the wrong moment.

It also publishes the **Kalman-filtered** position. Good for the live plot,
wrong for evaluation: a filter lags and smooths, which biases both the latency
estimate and the accuracy numbers. The patch carries the raw fix alongside.

    python3 patch_backend.py /home/sathishkumara/tdoa_uwb
    python3 patch_backend.py <repo> --dry-run
    python3 patch_backend.py <repo> --revert

`.orig` backups are written next to each file, so `--revert` always works.
"""

from __future__ import annotations

import argparse
import ast
import os
import shutil
import sys

MARK = "lit-fusion"


# ── TWR backend ──────────────────────────────────────────────────────────────

TWR_FILE = os.path.join("backend", "uwb_localization_backend.py")

TWR_GROUP = ("        frame_measurements = [(anchor_id, tof) for frame_nr, "
             "anchor_id, tof, _timestamp in self.measurements "
             "if frame_nr == ready_frame]")

TWR_META = '''
        # --- lit-fusion: keep the round's time and quality -------------------
        # `timestamp` is stamped by the anchor's Raspberry Pi when the UWB chip
        # hands over the measurement, and those Pis are NTP-synced. The EARLIEST
        # of them is the closest thing to the instant the round happened. Which
        # anchor it came from matters too: the Pis carry a stable per-node bias
        # (~2.75 ms fastest to slowest on the 28 Apr log), so recording the
        # source anchor lets that be subtracted.
        try:
            import datetime as _dt

            def _unix(ts):
                if ts is None:
                    return None
                if isinstance(ts, (int, float)):
                    return float(ts)
                if isinstance(ts, _dt.datetime):
                    return ts.timestamp()          # serial path: naive local now()
                try:
                    return float(ts)
                except (TypeError, ValueError):
                    d = _dt.datetime.fromisoformat(str(ts).strip().replace("Z", "+00:00"))
                    if d.tzinfo is None:
                        d = d.replace(tzinfo=_dt.timezone.utc)   # ROS stream is UTC
                    return d.timestamp()

            _rows = [(a, _unix(ts)) for fn, a, _tof, ts in self.measurements
                     if fn == ready_frame]
            _rows_t = [r for r in _rows if r[1] is not None]
            _earliest = min(_rows_t, key=lambda r: r[1]) if _rows_t else (None, None)
            self._lit_meta = {
                "t_round_s": _earliest[1],
                "anchor_earliest": (f"0x{_earliest[0]:04X}"
                                    if isinstance(_earliest[0], int) else _earliest[0]),
                "n_anchors": len(_rows),
                "anchors_used": [f"0x{a:04X}" if isinstance(a, int) else str(a)
                                 for a, _ in _rows],
                "frame_nr": ready_frame,
            }
        except Exception:
            self._lit_meta = {}              # bookkeeping must never stop a fix
        # --- end lit-fusion --------------------------------------------------
'''

TWR_PUT = "    async def _put_latest(self, payload):"

TWR_MERGE = '''    async def _put_latest(self, payload):
        # --- lit-fusion: attach the round metadata to every emitted payload.
        # Every emit path goes through here, and the same dict object is also
        # handed to the data buffer, so one hook covers them all.
        if isinstance(payload, dict):
            meta = getattr(self, "_lit_meta", None)
            if meta:
                payload.update(meta)
                payload.setdefault("t_solved_s", __import__("time").time())
        # --- end lit-fusion --------------------------------------------------
'''


# ── TDoA backend ─────────────────────────────────────────────────────────────

TDOA_FILE = os.path.join("backend", "uwb_tdoa_localization_backend.py")

# The Infineon `ranging_utils` package is imported at module scope but is only
# used by perform_localization(), the OLD two-way-ranging path. TDoA uses
# scipy's least_squares directly. Making the import optional lets the TDoA
# backend run without access to gitlab.intra.infineon.com.
TDOA_IMPORT = ("from ranging_utils.algorithms import localization_3D_linear_ls,"
               "localization_3D_nonlinear_ls, localization_2D_nonlinear_ls")

TDOA_IMPORT_NEW = '''# --- lit-fusion: ranging_utils is Infineon-internal and is only needed by the
# legacy TWR perform_localization(); the TDoA path uses scipy.least_squares.
try:
    from ranging_utils.algorithms import (localization_3D_linear_ls,
                                          localization_3D_nonlinear_ls,
                                          localization_2D_nonlinear_ls)
except ModuleNotFoundError:  # pragma: no cover
    localization_3D_linear_ls = None
    localization_3D_nonlinear_ls = None
    localization_2D_nonlinear_ls = None


def _lit_unix(ts):
    """Anchor timestamp -> unix seconds, or None.

    The ROS log stream delivers naive ISO strings in UTC, e.g.
    '2026-09-17T12:39:07.945776' (verified on Server2: they read 12:39 when
    local time was 14:39 CEST). The serial path uses datetime.now(), a naive
    LOCAL datetime. Numbers pass through. Anything unparseable -> None.
    """
    import datetime as _dt
    if ts is None:
        return None
    if isinstance(ts, (int, float)):
        return float(ts)
    if isinstance(ts, _dt.datetime):
        return ts.timestamp()                       # naive = local, as now() gives
    if isinstance(ts, str):
        txt = ts.strip()
        try:
            return float(txt)
        except ValueError:
            pass
        try:
            d = _dt.datetime.fromisoformat(txt.replace("Z", "+00:00"))
        except ValueError:
            return None
        if d.tzinfo is None:
            d = d.replace(tzinfo=_dt.timezone.utc)  # the ROS stream is UTC
        return d.timestamp()
    return None
# --- end lit-fusion'''

TDOA_ROUND = '        round_idx = payload.get("round_idx", 0)'

TDOA_TRACK = '''
        # --- lit-fusion: remember when each round actually happened ----------
        # `timestamp` is stamped by the receiving anchor's NTP-synced Raspberry
        # Pi. The EARLIEST across a round is the closest available proxy for the
        # instant the tag blinked. It has to be tracked per round because a
        # round is only solved once it is >= 2 rounds old, so the message that
        # triggers the solve is much younger than the round it describes.
        # Bookkeeping only: it must never be able to drop a measurement, so any
        # failure here is swallowed and the solver carries on untouched.
        try:
            _lit_ts = _lit_unix(timestamp)
            if _lit_ts is not None:
                if not hasattr(self, "_lit_round_ts"):
                    self._lit_round_ts = {}
                _cur = self._lit_round_ts.get(round_idx)
                if _cur is None or _lit_ts < _cur[0]:
                    self._lit_round_ts[round_idx] = (_lit_ts, node_id)
        except Exception:
            pass
        # --- end lit-fusion --------------------------------------------------
'''

TDOA_RETURN = "        return results if results else None"

TDOA_META = '''        # --- lit-fusion: describe the round that was actually solved --------
        try:
            self._lit_describe_round(ready_round, frame_measurements_by_tag, results)
        except Exception:
            self._lit_meta = {}
        # --- end lit-fusion --------------------------------------------------

        return results if results else None'''

TDOA_METHOD_ANCHOR = "    def evaluate_measurements(self):"

TDOA_METHOD = '''    def _lit_describe_round(self, ready_round, frame_measurements_by_tag, results):
        # --- lit-fusion: metadata for the round that was actually solved ------
        _lit_rts = getattr(self, "_lit_round_ts", {})
        _lit_t, _lit_who = _lit_rts.get(ready_round, (None, None))
        self._lit_meta = {
            "t_round_s": _lit_t,
            "anchor_earliest": _lit_who,
            "solved_round_idx": ready_round,
            "rounds_deferred": (self.last_seen_frame - ready_round),
            "reference_anchor": getattr(self, "initiator_id", None),
            "n_anchors": {t: len(v) for t, v in frame_measurements_by_tag.items()},
            "anchors_used": {t: sorted({a for a, _ in v})
                             for t, v in frame_measurements_by_tag.items()},
            "raw_xy": {t: [float(p[0]), float(p[1])]
                       for t, p in (results or {}).items() if p is not None},
        }
        for _lit_old in [r for r in _lit_rts if r <= self.threshold - 20]:
            _lit_rts.pop(_lit_old, None)
        # --- end lit-fusion --------------------------------------------------

'''


TDOA_PAYLOAD = '''                            uwb_data = {
                                "x": float(updated_pos[0]),
                                "y": float(updated_pos[1]),
                                "z": float(updated_pos[2]) if len(updated_pos) > 2 else 0.0,
                                "round_idx": round_idx,
                                "tag_node_id": tag_id,
                                "type": "UWB",
                                "timestamp": loc_timestamp
                            }'''

TDOA_PAYLOAD_NEW = '''                            # --- lit-fusion -------------------------------
                            _lit_m = getattr(self, "_lit_meta", {}) or {}
                            _lit_raw = (_lit_m.get("raw_xy") or {}).get(tag_id)
                            # --- end lit-fusion ---------------------------
                            uwb_data = {
                                "x": float(updated_pos[0]),
                                "y": float(updated_pos[1]),
                                "z": float(updated_pos[2]) if len(updated_pos) > 2 else 0.0,
                                "round_idx": round_idx,
                                "tag_node_id": tag_id,
                                "type": "UWB",
                                "timestamp": loc_timestamp,
                                # --- lit-fusion ---------------------------
                                # `timestamp` above is the arrival of the message
                                # that triggered this solve. `t_round_s` is when
                                # the round being solved actually happened; they
                                # differ by `rounds_deferred` rounds.
                                "t_round_s": _lit_m.get("t_round_s"),
                                "t_solved_s": __import__("time").time(),
                                "anchor_earliest": _lit_m.get("anchor_earliest"),
                                "solved_round_idx": _lit_m.get("solved_round_idx"),
                                "rounds_deferred": _lit_m.get("rounds_deferred"),
                                "reference_anchor": _lit_m.get("reference_anchor"),
                                "n_anchors": (_lit_m.get("n_anchors") or {}).get(tag_id),
                                "anchors_used": (_lit_m.get("anchors_used") or {}).get(tag_id),
                                # x,y above are Kalman-FILTERED. A filter lags and
                                # smooths, which would bias both the latency
                                # estimate and the accuracy numbers, so keep the
                                # raw TDoA fix too.
                                "x_raw": (_lit_raw[0] if _lit_raw else None),
                                "y_raw": (_lit_raw[1] if _lit_raw else None),
                                # --- end lit-fusion -----------------------
                            }'''


# ── localization_gui.py: unique MQTT client id ──────────────────────────────
# The GUI connects with the hardcoded client id 'LOCClient'. MQTT allows one
# connection per id, so a second instance anywhere (Andreas runs it on his PC)
# makes the broker drop the first; both auto-reconnect and evict each other
# about once a second. The broker log shows this happening on 15 Sep: 20
# collisions between 40.0.0.11 and 40.0.0.29. The result is gaps plus positions
# from two separate solvers interleaved on the same topic.
GUI_FILE = "localization_gui.py"
GUI_CLIENT = "'LOCClient', data_queue, data_buffer)"
GUI_CLIENT_NEW = ("f\"LOCClient-{__import__('socket').gethostname()}-"
                  "{__import__('os').getpid()}\", data_queue, data_buffer)  "
                  "# lit-fusion: unique id, see sensor-fusion/uwb/patch_backend.py")


def _apply(path: str, edits: list, dry: bool) -> str:
    """edits = [(find, replace)]. Returns a status string."""
    src = open(path).read()
    if MARK in src:
        return "already patched"
    out = src
    for find, repl in edits:
        if find not in out:
            return f"PATTERN NOT FOUND — this file differs from the known version:\n      {find.strip()[:90]}"
        out = out.replace(find, repl, 1)
    try:
        ast.parse(out)
    except SyntaxError as exc:
        return f"patch would produce invalid Python: {exc}"
    if dry:
        return f"would patch (+{len(out.splitlines()) - len(src.splitlines())} lines)"
    backup = path + ".orig"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    open(path, "w").write(out)
    return f"patched (backup {os.path.basename(backup)})"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("repo")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    targets = {
        TWR_FILE: [(TWR_GROUP, TWR_GROUP + "\n" + TWR_META),
                   (TWR_PUT, TWR_MERGE)],
        TDOA_FILE: [(TDOA_IMPORT, TDOA_IMPORT_NEW),
                    (TDOA_ROUND, TDOA_ROUND + "\n" + TDOA_TRACK),
                    (TDOA_RETURN, TDOA_META),
                    (TDOA_METHOD_ANCHOR, TDOA_METHOD + TDOA_METHOD_ANCHOR),
                    (TDOA_PAYLOAD, TDOA_PAYLOAD_NEW)],
        GUI_FILE: [(GUI_CLIENT, GUI_CLIENT_NEW)],
    }

    found = 0
    for rel, edits in targets.items():
        path = os.path.join(args.repo, rel)
        if not os.path.isfile(path):
            continue
        found += 1
        if args.revert:
            backup = path + ".orig"
            if os.path.exists(backup):
                shutil.copy2(backup, path)
                print(f"  {rel}: reverted")
            else:
                print(f"  {rel}: no backup, unchanged")
            continue
        print(f"  {rel}: {_apply(path, edits, args.dry_run)}")

    if not found:
        print(f"no UWB backend found under {args.repo}")
        return 1
    if not args.revert and not args.dry_run:
        print("\npayloads now carry: t_round_s, t_solved_s, anchor_earliest, "
              "n_anchors,\nanchors_used, and (TDoA) solved_round_idx, "
              "rounds_deferred, reference_anchor, x_raw, y_raw")
    return 0


if __name__ == "__main__":
    sys.exit(main())
