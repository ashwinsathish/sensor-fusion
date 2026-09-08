#!/usr/bin/env python3
"""Does the camera actually tell us WHEN it captured a frame?

Answers the question empirically, with no dependencies beyond the stdlib.

An RTSP/RTP sender is required by RFC 3550 to periodically emit RTCP Sender
Reports (PT=200). Each SR carries a pair:

    (NTP wall-clock time at the sender, RTP timestamp of that same instant)

That pair is *exactly* the capture-side timestamp people claim IP cameras
"do not provide". If the camera emits SRs and its clock is NTP-disciplined,
every frame's 90 kHz RTP timestamp can be converted to sender wall-clock, and
end-to-end latency is just  t_arrival_host - t_capture_camera.

This tool:
  1. Speaks RTSP (OPTIONS / DESCRIBE / SETUP / PLAY) with Basic+Digest auth.
  2. Uses TCP-interleaved transport so it works through NAT/tunnels.
  3. Collects RTP packets (channel 0) and RTCP SRs (channel 1).
  4. Reports: SR present? NTP epoch sane? camera-vs-host clock offset,
     SR interval, and the per-frame arrival delay implied by the SR mapping.

Usage:
    python3 rtsp_clock_probe.py rtsp://admin:PASS@50.0.0.2:554/h264Preview_01_main
    python3 rtsp_clock_probe.py <url> --seconds 60 --csv out.csv

Reolink RTSP paths are usually:
    /h264Preview_01_main   (channel 1, main stream)
    /h264Preview_01_sub    (channel 1, sub stream)
"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import os
import socket
import struct
import sys
import time
import urllib.parse

NTP_UNIX_DELTA = 2208988800  # seconds between 1900-01-01 and 1970-01-01


# ── RTSP plumbing ────────────────────────────────────────────────────────────

def _proxy_connect(proxy: str, host: str, port: int, timeout: float):
    """Open a TCP tunnel to host:port through an HTTP CONNECT proxy.

    Needed from the SAL VM, whose only route to the factory is
    lnzproxy01:3128. RTSP over TCP-interleaved transport tunnels cleanly
    through CONNECT because control, RTP and RTCP all share one connection —
    there are no side-channel UDP ports for a proxy to break.

    Format: [user:pass@]host:port  (or set SF_RTSP_PROXY / SAL_PROXY_PASS)
    """
    creds, _, hostport = proxy.rpartition("@")
    phost, _, pport = hostport.partition(":")
    sock = socket.create_connection((phost, int(pport or 3128)), timeout)
    sock.settimeout(timeout)
    req = (f"CONNECT {host}:{port} HTTP/1.1\r\nHost: {host}:{port}\r\n"
           "Proxy-Connection: Keep-Alive\r\n")
    if creds:
        req += ("Proxy-Authorization: Basic "
                + base64.b64encode(creds.encode()).decode() + "\r\n")
    sock.sendall((req + "\r\n").encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("proxy closed the connection")
        resp += chunk
    status = resp.split(b"\r\n", 1)[0].decode("iso-8859-1", "replace")
    if " 200" not in status:
        raise ConnectionError(
            f"proxy refused: {status}. "
            "Port 554 is blocked by SAL policy — forward RTSP to 8554 on the "
            "factory router and target that instead.")
    return sock


class RtspClient:
    def __init__(self, url: str, timeout: float = 8.0, proxy: str | None = None):
        p = urllib.parse.urlparse(url)
        self.user = urllib.parse.unquote(p.username or "")
        self.password = urllib.parse.unquote(p.password or "")
        self.host = p.hostname
        self.port = p.port or 554
        netloc = self.host if not p.port else f"{self.host}:{p.port}"
        self.url = urllib.parse.urlunparse(
            (p.scheme, netloc, p.path, p.params, p.query, p.fragment))
        self.cseq = 0
        self.session = None
        self.auth = None          # ("basic", None) | ("digest", {params})
        proxy = proxy or os.environ.get("SF_RTSP_PROXY")
        if proxy:
            self.sock = _proxy_connect(proxy, self.host, self.port, timeout)
        else:
            self.sock = socket.create_connection((self.host, self.port), timeout)
            self.sock.settimeout(timeout)
        self.buf = b""

    # -- auth ---------------------------------------------------------------
    def _auth_header(self, method: str, uri: str) -> str | None:
        if self.auth is None:
            return None
        kind, prm = self.auth
        if kind == "basic":
            tok = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()
            return f"Basic {tok}"
        realm, nonce = prm.get("realm", ""), prm.get("nonce", "")
        h = lambda s: hashlib.md5(s.encode()).hexdigest()  # noqa: E731
        ha1 = h(f"{self.user}:{realm}:{self.password}")
        ha2 = h(f"{method}:{uri}")
        resp = h(f"{ha1}:{nonce}:{ha2}")
        return (f'Digest username="{self.user}", realm="{realm}", nonce="{nonce}", '
                f'uri="{uri}", response="{resp}"')

    @staticmethod
    def _parse_www_auth(line: str):
        kind = "digest" if line.lower().lstrip().startswith("digest") else "basic"
        prm = {}
        for part in line.split(" ", 1)[1].split(","):
            if "=" in part:
                k, v = part.split("=", 1)
                prm[k.strip().lower()] = v.strip().strip('"')
        return kind, prm

    # -- request/response ---------------------------------------------------
    def request(self, method: str, uri: str | None = None, extra: dict | None = None):
        uri = uri or self.url
        for attempt in (0, 1):                      # retry once after a 401
            self.cseq += 1
            lines = [f"{method} {uri} RTSP/1.0",
                     f"CSeq: {self.cseq}",
                     "User-Agent: lit-sensor-fusion-probe"]
            if self.session:
                lines.append(f"Session: {self.session}")
            a = self._auth_header(method, uri)
            if a:
                lines.append(f"Authorization: {a}")
            for k, v in (extra or {}).items():
                lines.append(f"{k}: {v}")
            self.sock.sendall(("\r\n".join(lines) + "\r\n\r\n").encode())

            status, headers, body = self._read_response()
            if status == 401 and attempt == 0 and "www-authenticate" in headers:
                self.auth = self._parse_www_auth(headers["www-authenticate"])
                continue
            return status, headers, body
        return status, headers, body

    def _read_response(self):
        while b"\r\n\r\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("RTSP connection closed")
            self.buf += chunk
        head, self.buf = self.buf.split(b"\r\n\r\n", 1)
        text = head.decode("iso-8859-1")
        first, *rest = text.split("\r\n")
        status = int(first.split(" ")[1])
        headers = {}
        for line in rest:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        n = int(headers.get("content-length", 0))
        while len(self.buf) < n:
            self.buf += self.sock.recv(4096)
        body, self.buf = self.buf[:n], self.buf[n:]
        return status, headers, body.decode("iso-8859-1")

    def read_interleaved(self):
        """Yield (channel, payload) for TCP-interleaved RTP/RTCP frames."""
        while True:
            while len(self.buf) < 4:
                chunk = self.sock.recv(65535)
                if not chunk:
                    return
                self.buf += chunk
            if self.buf[0:1] != b"$":                # stray RTSP message
                idx = self.buf.find(b"$")
                if idx < 0:
                    self.buf = b""
                    continue
                self.buf = self.buf[idx:]
                continue
            channel = self.buf[1]
            length = struct.unpack(">H", self.buf[2:4])[0]
            while len(self.buf) < 4 + length:
                chunk = self.sock.recv(65535)
                if not chunk:
                    return
                self.buf += chunk
            payload = self.buf[4:4 + length]
            self.buf = self.buf[4 + length:]
            yield channel, payload


# ── RTP / RTCP parsing ───────────────────────────────────────────────────────

def parse_rtcp(payload: bytes):
    """Yield dicts for every Sender Report in a (possibly compound) RTCP packet."""
    off = 0
    while off + 4 <= len(payload):
        b0 = payload[off]
        pt = payload[off + 1]
        length = (struct.unpack(">H", payload[off + 2:off + 4])[0] + 1) * 4
        if pt == 200 and off + 28 <= len(payload):      # Sender Report
            ssrc = struct.unpack(">I", payload[off + 4:off + 8])[0]
            ntp_s, ntp_f = struct.unpack(">II", payload[off + 8:off + 16])
            rtp_ts = struct.unpack(">I", payload[off + 16:off + 20])[0]
            pkts, octets = struct.unpack(">II", payload[off + 20:off + 28])
            yield {
                "ssrc": ssrc,
                "ntp_unix": ntp_s + ntp_f / 2**32 - NTP_UNIX_DELTA,
                "ntp_raw_s": ntp_s,
                "rtp_ts": rtp_ts,
                "packets": pkts,
                "octets": octets,
                "version": b0 >> 6,
            }
        if length <= 0:
            break
        off += length


def parse_rtp(payload: bytes):
    if len(payload) < 12:
        return None
    b0 = payload[0]
    marker = (payload[1] >> 7) & 1
    seq = struct.unpack(">H", payload[2:4])[0]
    ts = struct.unpack(">I", payload[4:8])[0]
    ssrc = struct.unpack(">I", payload[8:12])[0]
    return {"marker": marker, "seq": seq, "rtp_ts": ts, "ssrc": ssrc,
            "cc": b0 & 0x0F}


# ── main probe ───────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("url", help="rtsp://user:pass@host:554/path")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--clock-hz", type=float, default=90000.0,
                    help="RTP clock rate (90000 for H.264/H.265 video)")
    ap.add_argument("--csv", help="write per-frame arrival records here")
    ap.add_argument("--proxy", help="HTTP CONNECT proxy: [user:pass@]host:port "
                                    "(or set SF_RTSP_PROXY)")
    args = ap.parse_args()

    print(f"→ connecting {args.url}")
    try:
        cli = RtspClient(args.url, proxy=args.proxy)
    except OSError as exc:
        print(f"  ! cannot reach the camera: {exc}")
        print("    Run this ON the factory LAN (or through the NVR tunnel),")
        print("    and use the RTSP port 554 URL, not the /flv?…bcs one.")
        return 2

    st, hd, _ = cli.request("OPTIONS")
    print(f"  OPTIONS  {st}   server={hd.get('server', '?')}")

    st, hd, sdp = cli.request("DESCRIBE", extra={"Accept": "application/sdp"})
    print(f"  DESCRIBE {st}")
    if st != 200:
        print("  ! DESCRIBE failed — check credentials / stream path")
        return 2
    for line in sdp.splitlines():
        if line.startswith(("m=", "a=rtpmap", "a=control", "a=framerate", "a=x-")):
            print(f"      {line}")

    # first video track control URL
    track, in_video = cli.url, False
    for line in sdp.splitlines():
        if line.startswith("m="):
            in_video = line.startswith("m=video")
        elif in_video and line.startswith("a=control:"):
            ctl = line.split(":", 1)[1].strip()
            if ctl.startswith("rtsp://"):
                # Some cameras answer with an ABSOLUTE control URL naming their
                # own LAN address. Through an SSH/port-forward tunnel that host
                # is unroutable, and a few servers then reject the SETUP. Point
                # it back at the host we are actually connected to.
                cp = urllib.parse.urlparse(ctl)
                netloc = cli.host if cli.port == 554 else f"{cli.host}:{cli.port}"
                track = urllib.parse.urlunparse(
                    (cp.scheme, netloc, cp.path, cp.params, cp.query, cp.fragment))
                if cp.netloc.split("@")[-1] != netloc:
                    print(f"      (rewrote control host {cp.netloc} -> {netloc} "
                          f"for the tunnel)")
            else:
                track = cli.url.rstrip("/") + "/" + ctl
            break

    st, hd, _ = cli.request(
        "SETUP", track,
        extra={"Transport": "RTP/AVP/TCP;unicast;interleaved=0-1"})
    print(f"  SETUP    {st}   transport={hd.get('transport', '?')}")
    if st != 200:
        print("  ! SETUP failed (camera may refuse TCP interleave)")
        return 2
    cli.session = hd.get("session", "").split(";")[0]

    st, hd, _ = cli.request("PLAY", extra={"Range": "npt=0.000-"})
    print(f"  PLAY     {st}   rtp-info={hd.get('rtp-info', '-')}")
    if st != 200:
        return 2

    print(f"\n→ listening {args.seconds:.0f} s for RTCP Sender Reports "
          f"(channel 1) and RTP (channel 0)…\n")

    t0 = time.time()
    srs, frames, rows = [], 0, []
    last_map = None           # (ntp_unix, rtp_ts) from the newest SR
    rtp_pkts = 0

    try:
        for channel, payload in cli.read_interleaved():
            now = time.time()
            if now - t0 > args.seconds:
                break
            if channel == 1:
                for sr in parse_rtcp(payload):
                    sr["host_time"] = now
                    srs.append(sr)
                    last_map = (sr["ntp_unix"], sr["rtp_ts"])
                    off = now - sr["ntp_unix"]
                    print(f"  [SR] ssrc={sr['ssrc']:08x} "
                          f"ntp={sr['ntp_unix']:.6f} rtp_ts={sr['rtp_ts']} "
                          f"→ host-camera clock offset = {off:+.3f} s")
            elif channel == 0:
                pkt = parse_rtp(payload)
                if pkt is None:
                    continue
                rtp_pkts += 1
                if pkt["marker"]:                       # end of an access unit
                    frames += 1
                    rec = {"host_time": now, "rtp_ts": pkt["rtp_ts"],
                           "seq": pkt["seq"], "capture_time": "", "e2e_s": ""}
                    if last_map:
                        ntp0, rtp0 = last_map
                        dt = ((pkt["rtp_ts"] - rtp0) & 0xFFFFFFFF)
                        if dt > 2**31:
                            dt -= 2**32
                        cap = ntp0 + dt / args.clock_hz
                        rec["capture_time"] = f"{cap:.6f}"
                        rec["e2e_s"] = f"{now - cap:.6f}"
                    rows.append(rec)
    except KeyboardInterrupt:
        pass
    except (ConnectionError, socket.timeout) as exc:
        print(f"  ! stream ended: {exc}")

    # ── verdict ──────────────────────────────────────────────────────────────
    dur = time.time() - t0
    print("\n" + "=" * 68)
    print(f"RTP packets {rtp_pkts}   frames(marker) {frames}   "
          f"≈{frames / max(dur, 1e-9):.1f} fps over {dur:.1f} s")
    print(f"RTCP Sender Reports received: {len(srs)}")

    if not srs:
        print("\nVERDICT: no RTCP SR seen in this window.")
        print("  → The camera/NVR is not giving a capture-side wall clock on this")
        print("    path. Try the sub-stream, the camera directly instead of the")
        print("    NVR, a longer --seconds (SR interval can be 5 s+), or UDP")
        print("    transport. If it stays empty, fall back to the estimated-delay")
        print("    route (tools/estimate_delay.py).")
        return 1

    ints = [b["host_time"] - a["host_time"] for a, b in zip(srs, srs[1:])]
    offs = [s["host_time"] - s["ntp_unix"] for s in srs]
    sane = all(1.0e9 < s["ntp_unix"] < 4.0e9 for s in srs)
    print(f"SR interval: median {sorted(ints)[len(ints)//2]:.2f} s" if ints else "")
    print(f"host - camera clock offset: min {min(offs):+.3f}  "
          f"max {max(offs):+.3f}  spread {1000*(max(offs)-min(offs)):.1f} ms")
    print(f"camera NTP epoch sane (real wall clock, not uptime): {sane}")

    e2e = [float(r["e2e_s"]) for r in rows if r["e2e_s"]]
    if e2e and sane:
        e2e.sort()
        n = len(e2e)
        print(f"\nimplied capture→host delay over {n} frames:")
        print(f"  p05 {e2e[n//20]*1000:7.1f} ms   median {e2e[n//2]*1000:7.1f} ms"
              f"   p95 {e2e[int(n*0.95)]*1000:7.1f} ms")
        print("  (absolute value is only meaningful if the CAMERA is NTP-synced")
        print("   to the same server as this host; the SPREAD is meaningful either")
        print("   way — it is the per-frame jitter you must model in fusion.)")

    print("\nVERDICT: the camera DOES expose a transmit-side timestamp via RTCP SR.")
    print("  → per-frame capture time = ntp_sr + (rtp_ts - rtp_sr)/90000")
    print("  → point the camera and every fusion host at the SAME NTP/PTP server")
    print("    and this becomes a true absolute capture clock.")

    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {len(rows)} frame records → {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
