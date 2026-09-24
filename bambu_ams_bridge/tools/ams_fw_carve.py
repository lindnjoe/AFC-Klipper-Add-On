#!/usr/bin/env python3
"""
Carve an AMS firmware image out of a passive bus capture, and REFUSE to write
one the capture cannot vouch for.

    ams_fw_carve.py <capture> [-o image.bin]

The capture is either form the sniff produces:

    "<sq> <ov> <rf> <hex>"        the harvested form (one line per blob)
    ...{"evt":"sniff",...}...     raw board lines, or an AFC.log with SNIFF lines

WHY THE INTEGRITY CHECK IS NOT OPTIONAL. An image rebuilt from 99% of its bytes
is not 99% of an image, it is a brick, and nothing downstream can tell it from a
good one -- while the update's own `erase fw_flash` runs BEFORE the first data
block, so a bad replay fails with the old firmware already gone. The firmware
stamps every blob with sq/ov/rf (see bb_sniff_poll) precisely so this tool can
tell. A gap inside the image window is fatal and this exits non-zero.

Protocol, from the capture of 2026-09-14 (the printer narrates it in plaintext
on its own [MCU_UP] channel, which is how it was read rather than guessed):

    cmd 1   handshake / identify      "0 resev cmd 0x1" -> "0 send cmd 0x1 back"
    cmd 2   header, 416 bytes         BIMH magic, size, target filename
            "0 erase fw_flash", "0 total seq : 168"
    cmd 3   data block                strict send -> ack -> next, with a retry
            counter; the AMS acks each with "0 resv seq num: N"
"""
from __future__ import annotations

import argparse
import binascii
import collections
import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_capture import crc8, crc16          # noqa: E402

#: Long-frame field offsets. Established against the 2026-09-14 capture and
#: cross-checked three ways: the declared block lengths sum to the header's size
#: field exactly, the block sequence runs 0..167 with no duplicates, and the
#: printer's own narration says "total seq : 168".
OP, SUB = 11, 12
CMD = 13                 #: 0x02 header, 0x03 data
SEQ = slice(27, 31)      #: block sequence, LE32
PLEN = slice(35, 39)     #: block payload length, LE32
DATA_OFF = 43            #: payload start in a cmd-3 frame
HDR_OFF = 39             #: payload start in a cmd-2 frame
UPD_OP, UPD_SUB = 0x06, 0x01


def read_capture(path: str) -> tuple:
    """
    Pull the byte stream and the integrity counters out of a capture file.

    :param path: capture file, harvested or raw
    :return tuple: (stream bytes, list of (sq, ov, rf, nbytes) per blob)
    """
    hexes, meta = [], []
    for line in open(path, errors="replace"):
        if line.startswith("#"):
            continue
        m = re.match(r"\s*(\d+)\s+(\d+)\s+(\d+)\s+([0-9A-Fa-f]*)\s*$", line)
        if not m:
            m2 = re.search(r'"sq"\s*:\s*(\d+).*?"ov"\s*:\s*(\d+).*?'
                           r'"rf"\s*:\s*(\d+).*?"hex"\s*:\s*"([0-9A-Fa-f]*)"',
                           line)
            if not m2:
                m3 = re.search(r"SNIFF ([0-9A-F]*) sq=(\d+) ov=(\d+) rf=(\d+)",
                               line)
                if not m3:
                    continue
                h, s, o, r = m3.group(1), m3.group(2), m3.group(3), m3.group(4)
            else:
                s, o, r, h = m2.groups()
        else:
            s, o, r, h = m.groups()
        hexes.append(h)
        meta.append((int(s), int(o), int(r), len(h) // 2))
    return binascii.unhexlify("".join(hexes)), meta


def frames_of(data: bytes) -> list:
    """
    Split the stream into CRC-valid frames.

    Long frames carry a 16-bit length at [4:6] and a crc8 at [6]; the short
    "C5" dialect carries its length at [2] and crc8 at [3]. Anything that fails
    either check is resynchronised past, byte by byte -- a passive tap sees line
    turnarounds and noise, and a frame is only a frame if both CRCs agree.

    :param data: the concatenated capture stream
    :return list: [(offset, frame bytes)]
    """
    out, i = [], 0
    while i < len(data) - 8:
        if data[i] != 0x3D:
            i += 1
            continue
        if data[i + 1] == 0xC5:
            n = data[i + 2]
            ok = crc8(data[i:i + 3]) == data[i + 3] and 6 <= n <= 64
        else:
            n = data[i + 4] | (data[i + 5] << 8)
            ok = crc8(data[i:i + 6]) == data[i + 6] and 10 <= n <= 4096
        if not ok or i + n > len(data):
            i += 1
            continue
        fr = data[i:i + n]
        if crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8)):
            i += 1
            continue
        out.append((i, fr))
        i += n
    return out


def entropy(b: bytes) -> float:
    """Shannon entropy in bits per byte."""
    h = collections.Counter(b)
    return -sum((v / len(b)) * math.log2(v / len(b)) for v in h.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("-o", "--out", help="write the carved image here")
    ap.add_argument("--force", action="store_true",
                    help="write the image even if the capture lost data. It "
                         "will be WRONG in ways nothing can detect later; this "
                         "exists for protocol work, never for flashing.")
    a = ap.parse_args()

    data, meta = read_capture(a.capture)
    if not meta:
        print("no sniff blobs found in that file", file=sys.stderr)
        return 2
    print(f"capture: {len(meta)} blobs, {len(data) / 1024:.1f} KB")

    frames = frames_of(data)
    print(f"frames : {len(frames)} CRC-valid")

    upd = [(o, f) for o, f in frames
           if f[1] == 0x00 and len(f) > CMD
           and f[OP] == UPD_OP and f[SUB] == UPD_SUB]
    hdrs = [(o, f) for o, f in upd if f[CMD] == 0x02]
    blks = [(o, f) for o, f in upd if f[CMD] == 0x03]
    if not hdrs or not blks:
        print("no firmware update in this capture "
              f"({len(hdrs)} header frame(s), {len(blks)} data frame(s))")
        return 1

    lo = min(o for o, _f in upd)
    hi = max(o + len(f) for o, f in upd)

    # ── integrity, scoped to the window that actually matters ──────────────
    off, pos = [], 0
    for _s, _o, _r, n in meta:
        off.append(pos)
        pos += n
    gaps = lost = 0
    ov = rf = 0
    for i in range(1, len(meta)):
        if not (lo <= off[i] <= hi):
            continue
        if meta[i][0] > meta[i - 1][0] + 1:
            gaps += 1
            lost += meta[i][0] - meta[i - 1][0] - 1
        ov += meta[i][1] - meta[i - 1][1]
        rf += meta[i][2] - meta[i - 1][2]
    print(f"image window bytes {lo}..{hi}: {gaps} gap(s)/{lost} blob(s) lost, "
          f"{ov} UART overrun(s), {rf} ring lap(s)")
    clean = not (lost or ov or rf)

    hdr = hdrs[0][1]
    hpay = hdr[HDR_OFF:len(hdr) - 2]
    seen = {}
    for _o, f in blks:
        seen[int.from_bytes(f[SEQ], "little")] = f
    n_seq = max(seen) + 1
    missing = sorted(set(range(n_seq)) - set(seen))
    print(f"blocks : {len(seen)} unique, seq 0..{n_seq - 1}"
          + (f", MISSING {missing[:8]}" if missing else ", none missing"))

    img = bytearray(hpay)
    for s in range(n_seq):
        f = seen.get(s)
        if f is None:
            break
        n = int.from_bytes(f[PLEN], "little")
        img += f[DATA_OFF:DATA_OFF + n]

    size = struct.unpack_from("<I", img, 8)[0] if len(img) > 12 else 0
    magic = bytes(img[:4])
    print(f"\nmagic {magic!r}  declared size {size}  carved {len(img)}"
          + ("  MATCH" if size == len(img) else "  *** MISMATCH ***"))
    if len(img) > 0x70:
        name_end = img.find(b"\x00", 0x30)
        print(f"  header len {struct.unpack_from('<I', img, 0x20)[0]}, "
              f"sig len {struct.unpack_from('<I', img, 0x24)[0]}, "
              f"payload {struct.unpack_from('<I', img, 0x28)[0]}")
        print(f"  target: {img[0x30:name_end].decode('ascii', 'replace')}")
    if len(img) > 0x200:
        e = entropy(bytes(img[0x80:]))
        print(f"  body entropy {e:.4f} bits/byte -> "
              + ("ENCRYPTED/compressed" if e > 7.5 else "looks like plain code"))

    ok = clean and not missing and size == len(img) and magic == b"BIMH"
    print("\n" + ("CAPTURE IS SOUND -- image is byte-complete" if ok else
                  "DO NOT FLASH THIS: the capture or the carve did not verify"))
    if a.out:
        if not ok and not a.force:
            print("refusing to write; pass --force only for protocol work",
                  file=sys.stderr)
            return 1
        open(a.out, "wb").write(img)
        print(f"wrote {a.out} ({len(img)} bytes)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
