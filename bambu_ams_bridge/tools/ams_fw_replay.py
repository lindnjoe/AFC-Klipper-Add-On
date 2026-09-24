#!/usr/bin/env python3
"""
Replay driver for the AMS firmware update, and a dry-run that proves it against
a real capture WITHOUT touching hardware.

A Bambu printer delivers the update as a header frame plus 168 data-block frames
on the AMS bootloader's bus. Every byte of those frames EXCEPT the rolling
counter (and the CRCs derived from it) is a fixed, per-model artifact -- proven
byte-identical across three captures from two units (capA/capA2/capB). So the
replay carries each frame's invariant BODY and regenerates only the counter and
CRCs.

An artifact no longer needs a capture at all: `pack` builds one from Bambu's own
`*_product.bin.sig`. See the block above pack_artifact.

    ams_fw_replay.py extract   <capture.txt> <artifact.json>
    ams_fw_replay.py pack      <image.bin.sig> <model> <artifact.json>
    ams_fw_replay.py packcheck <image.bin.sig> <model> <captured.json>
    ams_fw_replay.py dryrun    <capture.txt>          # prove vs a capture
    ams_fw_replay.py plan      <artifact.json>        # print the send plan

NOTHING HERE TRANSMITS. Driving a real unit is a separate, deliberate step: the
erase is destructive, runs right after the header, and there is no BOOTSEL on
the far side. See docs/AMS_FW_UPDATE.md.

Frame layout (long, in the bootloader):

    [0]      0x3D
    [1]      class (0x00 on the master's payload frames)
    [2:4]    rolling frame counter, LE16   -- MASTER-OWNED, regenerated here
    [4:6]    frame length, LE16
    [6]      crc8 over [0:6]
    [7:11]   routing (00 07 00 09)
    [11:13]  op/sub = 06 01
    [13]     cmd = 0x02 header / 0x03 data
    [23:26]  total image size, LE24        (header frame only)
    [27:31]  block sequence, LE32          (data frames)
    [35:39]  firmware bytes in this block, LE32
    last 2   crc16

AND THE REST OF A DATA FRAME IS PER-MODEL. Do not write one rule here:

    boxed n3f   [39:43]  per-block tag (high entropy, 168 distinct of 168)
                [43:]    firmware bytes
    HT    n3s   [39:]    firmware bytes
                last 6   4-byte trailer (zero except on the last block) + crc16

Both were measured against Bambu's own images, in both directions:

    n3s  header[32:] + blocks[32:-4]  -> sha256 ce0fd615..  matches vendor file
    n3f  header[32:] + blocks[36:]    -> sha256 f871eb7b..  matches vendor file

(Offsets in the artifact are frame offsets minus 7; an artifact body is
frame[7:-2].) Using one model's layout on the other gives an image of exactly
the right LENGTH that is wrong from the first payload byte -- 0.64% of bytes
happen to agree, which is barely above chance and nothing a length or block
count would catch.

THE ARTIFACT is the ordered list of frame bodies (class + [7:-2]) for the header
and the 168 data blocks. It is what one model's bootloader accepts; a different
unit of the same model + version takes the identical bytes (proven). The driver
wraps each body with a fresh monotonic counter, because after a power cycle the
unit cannot know what counter to expect and does not check it -- only the CRCs.
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_capture import crc8, crc16          # noqa: E402
from ams_fw_carve import read_capture, frames_of  # noqa: E402


def wrap(cls: int, counter: int, body: bytes) -> bytes:
    """Wrap a frame BODY (bytes [7:-2]) into a full frame with our counter+CRCs.

    :param cls: the class byte [1]
    :param counter: rolling counter for [2:4]
    :param body: the invariant frame body, i.e. captured frame[7:-2]
    :return bytes: a complete, CRC-correct frame
    """
    fr = bytearray(7 + len(body) + 2)
    fr[0] = 0x3D
    fr[1] = cls
    fr[2] = counter & 0xFF
    fr[3] = (counter >> 8) & 0xFF
    n = len(fr)
    fr[4] = n & 0xFF
    fr[5] = (n >> 8) & 0xFF
    fr[6] = crc8(bytes(fr[:6]))
    fr[7:7 + len(body)] = body
    c = crc16(bytes(fr[:-2]))
    fr[-2] = c & 0xFF
    fr[-1] = (c >> 8) & 0xFF
    return bytes(fr)


def master_frames(capture_path: str):
    """The master's firmware-carrying frames from a capture, header then blocks.

    :return tuple: (header frame, [data frames sorted by block seq])
    """
    data, _meta = read_capture(capture_path)
    fr = [f for _off, f in frames_of(data)]          # frames_of yields (offset, frame)
    pay = [f for f in fr if f[1] == 0x00 and len(f) > 13
           and f[11] == 0x06 and f[12] == 0x01]
    hdr = [f for f in pay if f[13] == 0x02]
    dat = sorted((f for f in pay if f[13] == 0x03),
                 key=lambda f: int.from_bytes(f[27:31], "little"))
    return (bytes(hdr[0]) if hdr else None), [bytes(f) for f in dat]


def extract_artifact(capture_path: str):
    """Build the per-model replay artifact from a verified capture.

    :return dict: {"header": body, "blocks": [body, ...]} of invariant bodies
    """
    hdr, dat = master_frames(capture_path)
    if hdr is None or not dat:
        raise ValueError("no master transfer frames in that capture")
    seqs = [int.from_bytes(f[27:31], "little") for f in dat]
    missing = sorted(set(range(max(seqs) + 1)) - set(seqs))
    if missing:
        raise ValueError(f"capture missing blocks {missing[:8]}")
    return {"header": hdr[7:-2], "blocks": [f[7:-2] for f in dat]}


# ── PACKING A VENDOR IMAGE INTO AN ARTIFACT ─────────────────────────────────
#
# The reverse of extract: take Bambu's own `*_product.bin.sig` and build the
# frame bodies a bootloader accepts, without needing a capture of that firmware
# being pushed. Proven by round-tripping the one image we hold BOTH sides of --
# pack(vendor n3s v05.00.22.19) against the ht_artifact.json carved from the
# wire (see ams_firmware/vendor/README.md).
#
# ══ THE FRAME BODY. THE TAIL OF IT IS PER-MODEL -- SEE THE DOCSTRING. ══
#
# On the HT the four non-firmware bytes are a TRAILER at the end of the body,
# and [32:36] really is firmware:
#
#     block[32:36] = 9c0ec080  ==  image[416:420]      <- firmware on the n3s
#     distinct block[-4:] : 2 of 157   (zero x156, one value on the last)
#
# On a boxed unit they are a TAG at the FRONT of the payload instead, and there
# is no trailer:
#
#     block[32:36] = 49ad0daf  -- absent from the image, 168 distinct of 168
#
# Entropy cannot tell these apart: the payload is encrypted, so every candidate
# field looks random. Only a byte-exact reassembly against the vendor image
# can, which is what packcheck is for.
#
# Body layout, all little-endian ([32:] differs per model, as above):
#
#     [0:2]    device address          0x1800 HT, 0x0700 boxed
#     [2:4]    0x0900                  constant
#     [4:6]    0x06 0x01               op/sub
#     [6]      0x02 header / 0x03 data
#     [7]      device address, high byte
#     [8:12]   zero
#     [12:16]  unit id                 0x80 HT, 0x00 boxed
#     [16:20]  header: total image size;  data: region size 0x2d000
#     [20:24]  header: zero;              data: block sequence
#     [24:28]  header: zero;              data: chunk size 0x400
#     [28:32]  this frame's payload length
#     [32:-4]  payload
#     [-4:]    trailer (see below)
#
# ══ THE ONE FIELD WE CANNOT DERIVE. ══
#
# The trailer is zero on every block but the last, which carried 0x007f8d40 in
# the captured HT transfer. That value is not a CRC32, not a byte sum, not an
# offset, and it appears nowhere in the image or its header. 0x7f8d40 is
# 8,359,232 -- consistent with a millisecond uptime from the printer that sent
# it (~2.3 h), which would make it transfer metadata the unit ignores. THAT IS A
# HYPOTHESIS. pack() emits zero and says so; a packed artifact therefore differs
# from a captured one in exactly those four bytes and nowhere else, which is
# what the round-trip check asserts.
#
# The image describes its own split, so none of this needs per-model constants:
# the BIMH container carries its total size at +0x08 and its header payload
# length at +0x20 (416 for the HT). Read them, do not assume them.

#: Device address and unit id per model, and whether that pair has been checked
#: against a REAL captured transfer of that model rather than assumed.
#:
#: ht   -- packed and diffed against ht_artifact.json: identical but for the
#:         final trailer.
#: ams2 -- checked against the captured ams_artifact.json (v05.00.22.22) by
#:         packing a DIFFERENT version (v05.00.22.19) of the same model. The
#:         bodies cannot match, but every prefix field must, and does: blocks 0
#:         and 1 byte-identical, and the only differences anywhere are the two
#:         size-derived fields (header total size, last block's payload length),
#:         which are obliged to differ between versions. That is a stronger
#:         check than a same-version diff, because it isolates the addressing
#:         from the payload.
#: ams1 -- boxed like the ams2 and assumed to share its pair. NOT checked: no
#:         capture of an AMS 1 transfer exists, and it is a different model with
#:         a different bootloader generation (v01.x against v05.x).
#: `packable` is whether a vendor image is ENOUGH to build a flashable
#: artifact. See THE BOXED BLOCK CARRIES A TAG WE CANNOT DERIVE, below.
PACK_MODELS = {
    "ht":   {"dev": 0x1800, "unit": 0x80, "confirmed": True,  "packable": True},
    "ams2": {"dev": 0x0700, "unit": 0x00, "confirmed": True,  "packable": False},
    "ams1": {"dev": 0x0700, "unit": 0x00, "confirmed": False, "packable": False},
}

# ══ THE BOXED BLOCK CARRIES A TAG WE CANNOT DERIVE. ══
#
# THE TWO MODELS DO NOT SHARE A BLOCK LAYOUT, and assuming they did is the
# mistake this guard exists to stop:
#
#     HT    n3s   [0:32] prefix | firmware          | 4-byte trailer (zeros)
#     boxed n3f   [0:32] prefix | 4-byte TAG | firmware        (no trailer)
#
# Measured, both directions, against Bambu's own files:
#
#     n3s: header[32:] + blocks[32:-4]  -> sha256 ce0fd615..  MATCHES vendor
#     n3f: header[32:] + blocks[36:]    -> sha256 f871eb7b..  MATCHES vendor
#     n3f with the HT layout            -> same length, 0.64% of bytes equal
#
# The n3f tags are 168 DISTINCT 4-byte values across 168 blocks. What they are
# NOT, all checked over the full 168 rather than a sample:
#
#     present anywhere in the vendor file      0 of 168
#     last 4 bytes of the previous block       0 of 167   (not an off-by-one)
#     the image bytes just after the payload   0 of 168   (not adjacent data)
#     monotonic LE / BE                        no / no    (not a counter)
#     crc32 / bus crc16 / sum32 of the payload 0 of 168
#     sha256[:4] / md5[:4] of the payload      0 of 168
#
# And the docstring records them as byte-identical across three captures from
# two units, so they are fixed for a model+version rather than per-device or
# per-transfer. Extra data Bambu's side supplies, in other words: the image does
# not encode it and we cannot compute it.
#
# The practical consequence is narrow and worth stating exactly. ONE capture of
# a given boxed firmware yields its tags permanently -- which is why the .22
# artifact we hold still works. A boxed firmware we have never captured cannot
# be built from its vendor file, however complete that file looks.
#
# ── HOW THIS WAS GOT WRONG, BECAUSE IT WILL HAPPEN AGAIN ──
#
# The module docstring's original "[39:43] per-block tag" was RIGHT, for boxed
# units. It was disproved on the HT -- where [32:36] really is firmware, shown
# by a byte-exact reassembly -- and the disproof was then generalised to "there
# is no tag anywhere". One model was checked and two were described. The HT
# flash succeeded on that reading, which made it look settled; the boxed
# packcheck is what caught it, before anything was written to hardware.
#
# So: pack() refuses a model it cannot fully build rather than emitting an
# artifact that is right in every field except the ones nobody checked. A
# boxed artifact still comes from a capture (`extract`), as it always did.

BIMH_MAGIC = b"BIMH"
CHUNK = 0x400
REGION = 0x2d000


def _body(dev: int, unit: int, cmd: int, f16: int, f20: int, f24: int,
          payload: bytes, trailer: bool) -> bytes:
    """One frame body: 32-byte prefix + payload, and a trailer on DATA only.

    THE HEADER FRAME HAS NO TRAILER. Data blocks carry four bytes after the
    payload; the header frame ends at its payload, which is why reassembly is
    `header[32:]` but `block[32:-4]`. Appending one here makes the header 452
    bytes against the captured 448 -- a length mismatch, not a content one, so a
    byte-diff over the common prefix reports zero differences and looks clean.
    """
    return (dev.to_bytes(2, "little") + b"\x00\x09" + b"\x06\x01"
            + bytes([cmd, (dev >> 8) & 0xFF]) + b"\x00" * 4
            + unit.to_bytes(4, "little")
            + f16.to_bytes(4, "little") + f20.to_bytes(4, "little")
            + f24.to_bytes(4, "little") + len(payload).to_bytes(4, "little")
            + payload + (b"\x00" * 4 if trailer else b""))


def pack_artifact(image: bytes, model: str) -> dict:
    """Build a replay artifact from a vendor `*_product.bin.sig` image.

    :param image: the vendor file, whole
    :param model: a key of PACK_MODELS
    :return dict: {"header": body, "blocks": [body, ...]}
    """
    if image[:4] != BIMH_MAGIC:
        raise ValueError(f"not a BIMH container (magic {image[:4]!r})")
    total = int.from_bytes(image[8:12], "little")
    if total != len(image):
        raise ValueError(f"header says {total} bytes, file is {len(image)}")
    hlen = int.from_bytes(image[0x20:0x24], "little")
    if not 0 < hlen < len(image):
        raise ValueError(f"implausible header payload length {hlen}")
    m = PACK_MODELS[model]
    if not m.get("packable"):
        raise ValueError(
            f"{model}: a vendor image is not enough to build this artifact. "
            f"Its data blocks carry a 4-byte per-block tag that is absent from "
            f"the image (168 distinct values across 168 blocks on the n3f we "
            f"checked), so the firmware bytes are all present and the wrapper "
            f"is not. Use `extract` on a capture of that model instead.")
    dev, unit = m["dev"], m["unit"]
    art = {"header": _body(dev, unit, 0x02, total, 0, 0, image[:hlen],
                          trailer=False),
           "blocks": []}
    rest = image[hlen:]
    for seq in range((len(rest) + CHUNK - 1) // CHUNK):
        chunk = rest[seq * CHUNK:(seq + 1) * CHUNK]
        art["blocks"].append(
            _body(dev, unit, 0x03, REGION, seq, CHUNK, chunk, trailer=True))
    return art


def pack_check(image_path: str, model: str, against: str) -> int:
    """Pack an image and diff it against a captured artifact.

    The whole point: we hold both sides for exactly one firmware, so the packer
    can be proven rather than trusted. Differences are reported per frame and
    per byte range, because "it differs" is not a useful answer -- the expected
    result is a difference confined to the final trailer.

    :return int: 0 when the only differences are known-underivable fields
    """
    art = pack_artifact(open(image_path, "rb").read(), model)
    ref = json.load(open(against))
    ref = {"header": bytes.fromhex(ref["header"]),
           "blocks": [bytes.fromhex(b) for b in ref["blocks"]]}
    ok = True
    if len(art["blocks"]) != len(ref["blocks"]):
        print(f"FAIL block count: packed {len(art['blocks'])} "
              f"vs captured {len(ref['blocks'])}")
        return 1
    if art["header"] != ref["header"]:
        ok = False
        d = [i for i in range(min(len(art["header"]), len(ref["header"])))
             if art["header"][i] != ref["header"][i]]
        print(f"header differs: packed {len(art['header'])} B vs captured "
              f"{len(ref['header'])} B, {len(d)} differing bytes in the "
              f"common prefix, first {d[:8]}")
    else:
        print(f"header  OK  ({len(art['header'])} bytes)")
    trailer_only = True
    shown = 0
    nbody = ntrail = 0
    for i, (p, c) in enumerate(zip(art["blocks"], ref["blocks"])):
        if p == c:
            continue
        ok = False
        d = [j for j in range(min(len(p), len(c))) if p[j] != c[j]]
        body_only = [j for j in d if j < len(p) - 4]
        # CAPPED. A systematic one-field error differs on all 157 blocks, and
        # 157 identical lines bury the one fact that matters (which offsets).
        if body_only:
            trailer_only = False
            nbody += 1
            if shown < 5:
                print(f"block {i}: differs at {len(d)} bytes INCLUDING BODY "
                      f"{body_only[:6]}")
                shown += 1
        else:
            ntrail += 1
            if shown < 5:
                print(f"block {i}: trailer only  packed={p[-4:].hex()} "
                      f"captured={c[-4:].hex()}")
                shown += 1
    if nbody + ntrail > 5:
        print(f"... {nbody} blocks differ in body, {ntrail} in trailer only")
    if ok:
        print("PACKED ARTIFACT IS BYTE-IDENTICAL to the captured one")
        return 0
    if trailer_only:
        print("\nOnly the 4-byte trailer differs -- the underivable field. "
              "Every firmware byte and every header field matches.")
        return 0
    print("\nBODY BYTES DIFFER -- the packer is wrong, do not flash from it.")
    return 1


def plan(artifact: dict, counter0: int = 0):
    """The ordered master send plan.

    :yield: (kind, seq, frame) with fresh counters
    """
    c = counter0
    yield ("header", -1, wrap(0x00, c, artifact["header"]))
    c += 1
    for s, body in enumerate(artifact["blocks"]):
        yield ("data", s, wrap(0x00, c, body))
        c += 1


def dryrun(capture_path: str) -> int:
    """Prove the wrapper reproduces the captured frames byte-for-byte.

    Rebuilds each master frame from its own captured body, using the SAME
    counter the capture carried, and diffs against the captured frame. A clean
    diff proves the length/crc8/crc16/counter math -- everything the driver adds
    on top of the per-model bodies.
    """
    hdr, dat = master_frames(capture_path)
    if hdr is None or not dat:
        print("no master transfer frames", file=sys.stderr)
        return 2
    print(f"capture: header + {len(dat)} data frames")

    ok = True
    got = wrap(hdr[1], hdr[2] | (hdr[3] << 8), hdr[7:-2])
    if got != hdr:
        ok = False
        print("header: MISMATCH")
    else:
        print("header: byte-exact")

    mism = 0
    for f in dat:
        got = wrap(f[1], f[2] | (f[3] << 8), f[7:-2])
        if got != f:
            mism += 1
    if mism:
        ok = False
        print(f"{mism}/{len(dat)} data frames MISMATCH")
    else:
        print(f"all {len(dat)} data frames: byte-exact")

    # And prove the plan() path (fresh counters) yields well-formed frames whose
    # bodies match the artifact -- i.e. the only change is the counter+CRC.
    art = extract_artifact(capture_path)
    steps = list(plan(art, counter0=0x1000))
    bad = sum(1 for _k, _s, fr in steps
              if crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8))
              or crc8(fr[:6]) != fr[6])
    print(f"plan(): {len(steps)} frames, {bad} with bad CRC")
    ok = ok and bad == 0 and len(steps) == 1 + len(dat)

    print("\n" + ("DRY-RUN PASSED -- driver reproduces the printer's transfer "
                  "byte-for-byte, and rebuilds it cleanly with fresh counters"
                  if ok else "DRY-RUN FAILED"))
    return 0 if ok else 1


def main() -> int:
    a = sys.argv
    if len(a) >= 4 and a[1] == "extract":
        art = extract_artifact(a[2])
        blob = {"header": art["header"].hex(),
                "blocks": [b.hex() for b in art["blocks"]]}
        open(a[3], "w").write(json.dumps(blob))
        print(f"wrote {a[3]}: header + {len(art['blocks'])} blocks")
        return 0
    if len(a) >= 5 and a[1] == "pack":
        # pack <image.bin.sig> <model> <out.json>
        art = pack_artifact(open(a[2], "rb").read(), a[3])
        blob = {"header": art["header"].hex(),
                "blocks": [b.hex() for b in art["blocks"]]}
        open(a[4], "w").write(json.dumps(blob))
        print(f"wrote {a[4]}: header + {len(art['blocks'])} blocks")
        if not PACK_MODELS[a[3]]["confirmed"]:
            print(f"NOTE: the {a[3]} device/unit pair is inferred, not "
                  f"confirmed against a captured transfer of that model.")
        return 0
    if len(a) >= 5 and a[1] == "packcheck":
        # packcheck <image.bin.sig> <model> <captured_artifact.json>
        return pack_check(a[2], a[3], a[4])
    if len(a) >= 3 and a[1] == "dryrun":
        return dryrun(a[2])
    if len(a) >= 3 and a[1] == "plan":
        d = json.load(open(a[2]))
        art = {"header": bytes.fromhex(d["header"]),
               "blocks": [bytes.fromhex(b) for b in d["blocks"]]}
        for kind, seq, fr in plan(art):
            print(f"{kind:6s} seq={seq:4d} len={len(fr):4d}  {fr[:16].hex().upper()}")
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
