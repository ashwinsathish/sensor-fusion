#!/usr/bin/env python3
"""Round-trip test for the RTP depacketiser and timestamp maths.

The RTSP transport half is already proven against the real cameras (all four
LIT channels, 4 Sep 2026). What is NOT proven by that run is the part which
turns RTP payloads back into decodable H.264 — fragmentation, aggregation,
timestamp extension. Those are exactly the places a subtle bug hides for weeks
and shows up as "the detector randomly sees garbage".

So: encode real H.264, chop it into RTP packets the way a camera does
(FU-A for anything over the MTU, STAP-A for parameter sets), push it through
the depacketiser, decode it again, and check the pixels survived.

    python3 test_rtsp_source.py
"""

from __future__ import annotations

import struct
import sys

import numpy as np

from rtsp_source import H264Depacketizer, RtpTimestampExtender, _START_CODE

MTU_PAYLOAD = 1400


# ── helpers: act like a camera ───────────────────────────────────────────────

def split_annexb(buf: bytes) -> list[bytes]:
    """Annex-B bitstream -> list of raw NAL units (no start codes)."""
    out, i, n = [], 0, len(buf)
    starts = []
    while i < n - 3:
        if buf[i:i + 3] == b"\x00\x00\x01":
            starts.append((i, 3))
            i += 3
        elif buf[i:i + 4] == b"\x00\x00\x00\x01":
            starts.append((i, 4))
            i += 4
        else:
            i += 1
    for k, (pos, ln) in enumerate(starts):
        end = starts[k + 1][0] if k + 1 < len(starts) else n
        nal = buf[pos + ln:end]
        if nal:
            out.append(nal)
    return out


def packetize(nal: bytes, mtu: int = MTU_PAYLOAD) -> list[bytes]:
    """One NAL -> RTP payloads, fragmenting with FU-A when it does not fit."""
    if len(nal) <= mtu:
        return [nal]
    hdr = nal[0]
    body = nal[1:]
    indicator = bytes([(hdr & 0xE0) | 28])
    out, off, first = [], 0, True
    while off < len(body):
        chunk = body[off:off + mtu - 2]
        off += len(chunk)
        last = off >= len(body)
        fu_hdr = (0x80 if first else 0) | (0x40 if last else 0) | (hdr & 0x1F)
        out.append(indicator + bytes([fu_hdr]) + chunk)
        first = False
    return out


def stap_a(nals: list[bytes]) -> bytes:
    """Aggregate several small NALs into one STAP-A payload, as cameras do
    for SPS+PPS."""
    body = b"".join(struct.pack(">H", len(n)) + n for n in nals)
    return bytes([(nals[0][0] & 0xE0) | 24]) + body


# ── the tests ────────────────────────────────────────────────────────────────

def test_timestamp_extender() -> None:
    ext = RtpTimestampExtender()
    base = 0xFFFFFF00
    seq = [(base + i * 3000) & 0xFFFFFFFF for i in range(20)]   # wraps mid-way
    out = [ext.extend(t) for t in seq]
    diffs = np.diff(out)
    assert (diffs == 3000).all(), f"wrap handling broke: {diffs}"
    assert out[-1] > out[0], "extended timestamp went backwards over the wrap"
    print("  ✓ 32-bit RTP timestamp wrap handled (13.25 h boundary)")


def test_depacketizer_roundtrip() -> int:
    import av

    n_frames = 24
    w, h = 320, 240

    enc = av.CodecContext.create("h264", "w")
    enc.width, enc.height, enc.pix_fmt = w, h, "yuv420p"
    enc.framerate, enc.time_base = 25, __import__("fractions").Fraction(1, 25)
    enc.options = {"preset": "ultrafast", "tune": "zerolatency", "g": "12"}

    originals, packets = [], []
    for i in range(n_frames):
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :, 0] = (i * 9) % 256                       # ramp, so a swapped
        img[40:120, 20 + i * 8:60 + i * 8] = (0, 255, 0)   # frame is obvious
        originals.append(img)
        frame = av.VideoFrame.from_ndarray(img, format="bgr24").reformat(format="yuv420p")
        frame.pts = i
        packets.extend(enc.encode(frame))
    packets.extend(enc.encode(None))

    # --- act like the camera: NALs -> RTP payloads ------------------------
    depack = H264Depacketizer()
    dec = av.CodecContext.create("h264", "r")
    n_fu = n_stap = n_single = 0
    decoded = []

    for pkt in packets:
        nals = split_annexb(bytes(pkt))
        # parameter sets go out aggregated, like a real camera
        params = [n for n in nals if (n[0] & 0x1F) in (7, 8)]
        rest = [n for n in nals if (n[0] & 0x1F) not in (7, 8)]
        payloads = []
        if len(params) > 1:
            payloads.append(stap_a(params))
            n_stap += 1
        else:
            payloads.extend(params)
        for nal in rest:
            # A small MTU here forces fragmentation on almost every NAL. Real
            # cameras fragment because their frames are large; shrinking the
            # MTU stresses the same code path with a frame size that keeps the
            # pixel comparison meaningful.
            frags = packetize(nal, mtu=200)
            if len(frags) > 1:
                n_fu += 1
            else:
                n_single += 1
            payloads.extend(frags)

        au = []
        for p in payloads:
            au.extend(depack.push(p))
        if not au:
            continue
        out = av.Packet(b"".join(au))
        for f in dec.decode(out):
            decoded.append(f.to_ndarray(format="bgr24"))
    for f in dec.decode(None):
        decoded.append(f.to_ndarray(format="bgr24"))

    print(f"  packetisation exercised: {n_single} single-NAL, {n_fu} FU-A "
          f"fragmented, {n_stap} STAP-A")
    assert n_fu > 0, "test did not exercise FU-A — raise the frame size"
    assert n_stap > 0, "test did not exercise STAP-A"
    assert depack.dropped_fragments == 0, \
        f"{depack.dropped_fragments} fragments dropped in a lossless test"

    assert len(decoded) == n_frames, \
        f"decoded {len(decoded)} frames, encoded {n_frames}"

    errs = [float(np.abs(a.astype(int) - b.astype(int)).mean())
            for a, b in zip(originals, decoded)]
    worst = max(errs)
    print(f"  ✓ {len(decoded)}/{n_frames} frames recovered, "
          f"worst mean abs pixel error {worst:.2f} (lossy codec)")
    assert worst < 12.0, f"frames came back wrong (error {worst:.1f})"

    # a swapped frame would show up as a large error against the WRONG original
    cross = float(np.abs(originals[0].astype(int) - decoded[-1].astype(int)).mean())
    assert cross > worst * 2, "frames may be out of order"
    print(f"  ✓ frame order preserved (cross-check error {cross:.1f} vs {worst:.1f})")
    return len(decoded)


def test_fu_a_loss_recovery() -> None:
    """A dropped first fragment must not corrupt the following frame."""
    depack = H264Depacketizer()
    big = bytes([0x65]) + bytes(range(256)) * 20          # an IDR-ish NAL
    frags = packetize(big)
    assert len(frags) > 2
    for p in frags[1:]:                                   # lose the S fragment
        depack.push(p)
    assert depack.dropped_fragments > 0
    good = depack.push(bytes([0x41]) + b"\x00" * 50)      # next, intact NAL
    assert len(good) == 1 and good[0].startswith(_START_CODE)
    print("  ✓ lost first fragment discarded cleanly, next NAL still parsed")


def main() -> int:
    print("RTP depacketiser round-trip\n")
    test_timestamp_extender()
    test_fu_a_loss_recovery()
    n = test_depacketizer_roundtrip()
    print(f"\nall checks passed ({n} frames through encode -> RTP -> decode)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
