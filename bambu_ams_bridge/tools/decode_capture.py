#!/usr/bin/env python3
"""
Decode a bus capture into named frames -- BOTH dialects.

Our own analysis notes admit the gap this closes: CAPTURE_FINDINGS has a
section headed "Blind spot: the short dialect" recording that every earlier
pass discarded `flag & 0x80` frames, which are roughly half the bus, with only
byte[5] ever mapped. Everything downstream of that -- cadences, the tick
hypothesis, which frame keeps a unit alive -- was reasoned about from the long
frames alone.

Sources for the naming, neither of them ours:
  * Bambu-Research-Group/Bambu-Bus README -- frame layouts, CRC parameters and
    the device address table.
  * that repo's AMCU AMS emulator (AMCU-v2/src/BambuBus.cpp,
    get_packge_type()) -- the short opcode -> name mapping, which the README
    itself does not give. Note it names 0x08 (set_filament), an opcode absent
    from the README table and one we have never sent.
  * scripts/parseproto.py -- the long-frame (cmd_set, cmd_id) command names.

Treat the names as a third party's reconstruction, not vendor truth. They are
a hypothesis to check against a capture, which is exactly how this is used: it
reports what it could NOT parse just as loudly as what it could, because a
decoder that silently drops what it does not understand would recreate the
blind spot it exists to remove.

Usage:
    decode_capture.py <capture.txt> [...]        summary per file
    decode_capture.py --frames <capture.txt>     every frame, in order
"""
from __future__ import annotations

import argparse
import gzip
import json
import re
import sys
from collections import Counter, defaultdict

# CRC parameters, from the Bambu-Bus README. These match bambu_ams_bridge's
# crc.c bit for bit, which is the first thing this script re-verifies: if the
# CRCs did not agree, every name below would be attached to the wrong bytes.
CRC8_POLY, CRC8_INIT = 0x39, 0x66
CRC16_POLY, CRC16_INIT = 0x1021, 0x913D

DEVICES = {
    0x03: "MC", 0x06: "AP", 0x07: "AMS", 0x08: "TH", 0x09: "AP2",
    0x0E: "AHB", 0x0F: "EXT", 0x12: "AMS-lite", 0x13: "CTC",
    0x0700: "AMS", 0x1200: "AMS-lite", 0x1800: "AMS-HT", 0x0300: "MC",
    0x0600: "AP", 0x0900: "AP2", 0x0F00: "EXT",
}

# From AMCU-v2 get_packge_type(). 0x08 is NOT in the published README table.
SHORT_OPS = {
    0x03: "filament_motion_short",
    0x04: "filament_motion_long",
    0x05: "online_detect",
    0x06: "REQx6",
    0x07: "NFC_detect",
    0x08: "set_filament",
    0x20: "heartbeat",
}

# From AMCU-v2, keyed on the long packet's `type` field.
LONG_TYPES = {0x21A: "MC_online", 0x211: "filament", 0x103: "version",
              0x402: "version"}


def crc8(data: bytes) -> int:
    crc = CRC8_INIT
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc16(data: bytes) -> int:
    crc = CRC16_INIT
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Frame:
    __slots__ = ("us", "dialect", "op", "name", "src", "dst", "payload",
                 "raw", "crc_ok")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def key(self) -> str:
        if self.dialect == "short":
            return f"C5/{self.op:02X} {self.name} len={len(self.raw)}"
        return (f"long {self.src}->{self.dst} "
                f"{self.op} {self.name} len={len(self.raw)}")


def split_frames(blob: bytes):
    """Yield (frame_bytes, reason_if_unparsable). Never silently drops."""
    i = 0
    while i < len(blob):
        if blob[i] != 0x3D:
            j = blob.find(b"\x3d", i + 1)
            yield blob[i:j if j > 0 else len(blob)], "no-0x3D-start"
            if j < 0:
                return
            i = j
            continue
        if i + 2 >= len(blob):
            yield blob[i:], "truncated-header"
            return
        flag = blob[i + 1]
        if flag & 0x80:                       # short dialect
            n = blob[i + 2]
        else:                                 # long dialect
            if i + 6 >= len(blob):
                yield blob[i:], "truncated-header"
                return
            n = blob[i + 4] | (blob[i + 5] << 8)
        if n < 5 or i + n > len(blob):
            yield blob[i:], "bad-or-truncated-length"
            return
        yield blob[i:i + n], None
        i += n


def parse(fr: bytes, us: int):
    flag = fr[1]
    body, want = fr[:-2], fr[-2] | (fr[-1] << 8)
    crc_ok = crc16(body) == want
    if flag & 0x80:
        op = fr[4]
        return Frame(us=us, dialect="short", op=op,
                     name=SHORT_OPS.get(op, "UNKNOWN"),
                     payload=fr[5:-2], raw=fr, crc_ok=crc_ok)
    d = fr[7:]
    dst, src = (d[0] << 8) | d[1], (d[2] << 8) | d[3]
    op = f"{d[4]:02X}/{d[5]:02X}"
    return Frame(us=us, dialect="long", op=op,
                 name=LONG_TYPES.get((d[5] << 8) | d[4], ""),
                 src=DEVICES.get(src, f"0x{src:04X}"),
                 dst=DEVICES.get(dst, f"0x{dst:04X}"),
                 payload=d[6:-2], raw=fr, crc_ok=crc_ok)


def join_capture(path: str):
    """Concatenate every capture blob, keeping a byte-offset -> timestamp map."""
    buf, stamps = bytearray(), []
    for blob, us in _iter_blobs(path):
        stamps.append((len(buf), us))
        buf += blob
    return bytes(buf), stamps


def stamp_at(stamps, off: int) -> int:
    """The microsecond stamp of the capture blob that byte `off` came from."""
    lo, hi, best = 0, len(stamps) - 1, (stamps[0][1] if stamps else 0)
    while lo <= hi:
        mid = (lo + hi) // 2
        if stamps[mid][0] <= off:
            best = stamps[mid][1]; lo = mid + 1
        else:
            hi = mid - 1
    return best


def frame_scan(buf: bytes):
    """Yield (offset, frame_bytes) for every CRC-clean frame in `buf`.

    Anchored on the CRC, not on the declared length. split_frames believes
    fr[2] / fr[4:6] even when the body fails its CRC, so ONE corrupted length
    byte consumes whatever sits behind it and every frame after that is framed
    from the wrong place -- a single bad byte silently costs a run of good
    frames, and the discard counter reports it as ordinary noise.

    Re-anchoring instead (accept only what checks out, otherwise advance one
    byte and look again) recovered 8687 frames from
    ams2_postupd_insert_measure where the length-trusting join found 6377 --
    36% more of the same capture, including the AMS 2's own measurement
    narration. The CRC is what makes this safe: a mis-framed 16-bit check
    passes by accident about once in 65536 tries, so on a 283 KB capture the
    expected number of invented frames is about four.
    """
    i, n = 0, len(buf)
    while i < n - 6:
        if buf[i] != 0x3D:
            i += 1
            continue
        flag = buf[i + 1]
        ln = buf[i + 2] if flag & 0x80 else (buf[i + 4] | (buf[i + 5] << 8))
        if ln < 7 or i + ln > n:
            i += 1
            continue
        fr = buf[i:i + ln]
        if crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8)):
            i += 1
            continue
        yield i, fr
        i += ln


def read_capture_joined(path: str):
    """Read a capture with the blobs CONCATENATED before framing.

    A frame that straddles two capture blobs is destroyed twice over -- its
    head closes one blob as `bad-or-truncated-length`, its tail opens the next
    as `no-0x3D-start` -- so it is missing from BOTH and shows up only as a
    discard count. On a real printer's bus that was 19% of one capture, and it
    hid the frame that capture had been taken to find.

    Joining first restores those frames, and frame_scan re-anchors on the CRC
    so a corrupt length byte costs one frame instead of the run behind it.
    What is left over is reported as `lost-bytes`: a real sniffer drops bytes
    under load, and the honest number for that is how much of the capture is
    not inside any frame that checks out.
    """
    frames, bad = [], Counter()
    buf, stamps = join_capture(path)
    covered = 0
    for off, raw in frame_scan(buf):
        try:
            frames.append(parse(raw, stamp_at(stamps, off)))
            covered += len(raw)
        except Exception:
            bad["decode-error"] += 1
    if len(buf) > covered:
        bad["lost-bytes"] += len(buf) - covered
    return frames, bad


# A narration run: an [AMS_xxx] or [RF] tag and the printable text after it.
# Searched over the JOINED BYTES, not over decoded frames, deliberately -- the
# unit's log lines ride in the biggest frames on the bus (up to ~220 bytes),
# which are exactly the ones a dropped byte ruins, so insisting on a clean
# frame first is what hid them. In ams2_postupd_insert_measure the decoder
# surfaced ONE narration frame; the raw scan finds all thirteen, including
# `odom C:0.507,R:0.081,P:93%` -- the sentence the capture was taken for.
NARRATION = re.compile(rb"\[(?:AMS_[A-Z0-9]+|RF)\][ -~]{0,160}")


def narration(path: str):
    """Yield (us, text) for every narration run in a capture, in order."""
    buf, stamps = join_capture(path)
    for m in NARRATION.finditer(buf):
        yield stamp_at(stamps, m.start()), m.group().decode("ascii", "replace")


def _iter_blobs(path: str):
    """Yield (bytes, us) for every capture line, in any of our formats."""
    hexre = re.compile(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"')
    usre = re.compile(r'"us"\s*:\s*(\d+)')
    bare = re.compile(r"^\s*([0-9A-Fa-f]{10,})\s*$")
    embedded = re.compile(r"\b(3[Dd][0-9A-Fa-f]{16,})\b")
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", errors="replace") as fh:
        for line in fh:
            h, us = None, 0
            if '"hex"' in line:
                m = hexre.search(line)
                if m:
                    h = m.group(1)
                    mu = usre.search(line)
                    us = int(mu.group(1)) if mu else 0
            else:
                m = bare.match(line) or embedded.search(line)
                if m:
                    h = m.group(1)
            if not h or len(h) % 2:
                continue
            yield bytes.fromhex(h), us


def read_capture(path: str):
    """Read a capture in any of the three formats we have on disk.

    The shapes are not distinguishable from the filename: the timestamped
    sniffs are newline-JSON ({"evt":"sniff","us":...,"hex":"..."}), the older
    *_frames.txt files are one bare hex frame per line with no timestamp, and
    the hand-annotated captures carry frames indented inside prose. Handling
    only the JSON form silently skipped 22 of 32 captures and still printed a
    census that looked complete -- the exact failure this tool exists to stop,
    which is why the caller is also told which files yielded no frames at all.
    """
    frames, bad = [], Counter()
    hexre = re.compile(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"')
    bare = re.compile(r"^\s*([0-9A-Fa-f]{10,})\s*$")
    embedded = re.compile(r"\b(3[Dd][0-9A-Fa-f]{16,})\b")
    # Captures are archived gzipped -- they are ~10:1 hex text, and a repo
    # carries every byte forever. Open either form transparently so nobody has
    # to remember which is which.
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", errors="replace") as fh:
        for line in fh:
            if '"hex"' not in line:
                # Three tiers, because three shapes exist on disk: a bare hex
                # line, and -- in the hand-annotated captures -- frames
                # indented inside prose. The embedded form needs a 0x3D start
                # to avoid matching hex-looking words in the commentary.
                m = bare.match(line) or embedded.search(line)
                if not m or len(m.group(1)) % 2:
                    continue          # comment/header line, not data
                # No timestamp in this format: 0 means "unknown", and
                # summarise() drops zero stamps rather than reporting a
                # fabricated 0 ms cadence.
                blob, us = bytes.fromhex(m.group(1)), 0
                for raw, reason in split_frames(blob):
                    if reason:
                        bad[reason] += 1
                        continue
                    try:
                        frames.append(parse(raw, us))
                    except Exception:
                        bad["decode-error"] += 1
                continue
            try:
                obj = json.loads(line)
                blob, us = bytes.fromhex(obj.get("hex", "")), obj.get("us", 0)
            except Exception:
                m = hexre.search(line)
                if not m:
                    bad["unparsable-line"] += 1
                    continue
                blob, us = bytes.fromhex(m.group(1)), 0
            for raw, reason in split_frames(blob):
                if reason:
                    bad[reason] += 1
                    continue
                try:
                    frames.append(parse(raw, us))
                except Exception:
                    bad["decode-error"] += 1
    return frames, bad


def summarise(path: str, frames, bad):
    print(f"\n=== {path.split('/')[-1]} ===")
    print(f"{len(frames)} frames, {sum(bad.values())} unparsable")
    for k, v in bad.most_common():
        print(f"    ! {v:6d}  {k}")
    n_short = sum(1 for f in frames if f.dialect == "short")
    n_bad = sum(1 for f in frames if not f.crc_ok)
    print(f"    short {n_short} ({100.0*n_short/max(1,len(frames)):.0f}%), "
          f"long {len(frames)-n_short}, CRC16 bad {n_bad}")

    times = defaultdict(list)
    for f in frames:
        times[f.key()].append(f.us)
    rows = sorted(times.items(), key=lambda kv: -len(kv[1]))
    print(f"    {'frame':<52}{'count':>8}  {'median gap':>11}")
    for k, ts in rows[:24]:
        ts = [t for t in ts if t]
        gap = ""
        if len(ts) > 4:
            ts.sort()
            d = sorted(b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 5_000_000)
            if d:
                gap = f"{d[len(d)//2]/1000.0:.1f} ms"
        print(f"    {k:<52}{len(ts):>8}  {gap:>11}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--join", action="store_true",
                    help="concatenate capture blobs before framing, so frames "
                         "split across a blob boundary are recovered instead "
                         "of counted twice as unparsable")
    ap.add_argument("--frames", action="store_true",
                    help="print every frame in order instead of a summary")
    ap.add_argument("--text", action="store_true",
                    help="print the units' narration lines in order, read out "
                         "of the joined bytes -- these ride in the biggest "
                         "frames on the bus, so a dropped byte hides them "
                         "from the frame decoder but not from here")
    args = ap.parse_args()
    for path in args.captures:
        if args.text:
            print(f"\n=== {path.split('/')[-1]} ===")
            for us, txt in narration(path):
                print(f"{us:>12} {txt}")
            continue
        frames, bad = (read_capture_joined(path) if args.join
                       else read_capture(path))
        if args.frames:
            for f in frames:
                print(f"{f.us:>12} {f.key():<52} "
                      f"{'' if f.crc_ok else '[CRC BAD] '}"
                      f"{f.payload.hex().upper()}")
        else:
            summarise(path, frames, bad)
    return 0


if __name__ == "__main__":
    sys.exit(main())
