#!/usr/bin/env python3
"""Locate the i2c-read hook site in a stock OpenAMS image for build_spi_patch.py.

Because the firmware is `-flto`, there is no standalone `i2c_dev_read` to detour —
the i2c command path is inlined into one large dispatch function. This tool uses
radare2 to surface the viable hook points and, for each, the prologue bytes to
save and whether they are position-independent (safe to relocate into the shim):

  * the clean i2c transfer primitives that survive LTO as real functions
    (object-ABI: r0 = i2c transaction struct), and
  * the enclosing dispatch function range (for an interior detour).

It prints ready-to-paste `--hook`/`--hook-prologue` values. Pick the one that makes
OAMS_RFID_PROBE answer on the bench (a wrong choice is harmless — the magic-reg
guard keeps real i2c untouched, and USB-DFU restores stock).

Usage:  python3 find_hook.py oams_2.0.231.bin
Requires radare2 on PATH.
"""
from __future__ import annotations
import sys
import struct

APP_BASE = 0x08004000
# Anchors found by analysis (see PATCH_PLAN.md). These are stable across the
# 2.0.x line but are re-confirmed here against the actual image.
CANDIDATES = [
    ("fcn.0800fe44", 0x0800FE44, "i2c transfer primitive (obj ABI: r0=i2c xfer)"),
    ("fcn.080107b0", 0x080107B0, "i2c read/write wrapper (obj ABI: r0=i2c xfer)"),
]
DISPATCH = (0x0800ECA0, 0x0800F3DA, "inlined i2c+oams command dispatch (interior hook)")

# 16-bit Thumb opcodes that are NOT position-independent (pc-relative / control
# flow) — a saved prologue must avoid these so it runs correctly relocated.
def _pic_safe(word: int) -> bool:
    hi = word >> 8
    if 0x48 <= hi <= 0x4F:            # LDR Rd,[pc,#imm]  (literal pool)
        return False
    if 0xE000 <= word <= 0xE7FF:      # B <label> (T2)
        return False
    if 0xD000 <= word <= 0xDFFF:      # B<cond> (T1)
        return False
    if 0xA000 <= word <= 0xAFFF:      # ADR / ADD Rd,pc,#imm
        return False
    if 0xF000 <= (word & 0xF800) <= 0xF800:  # first half of a 32-bit BL/B.W
        return False
    if (word & 0xF500) == 0xB100:     # CBZ/CBNZ
        return False
    return True


def prologue_for(data: bytes, vaddr: int, need: int = 4) -> bytes | None:
    """Return >=need bytes of PIC-safe 16-bit insns from vaddr, or None."""
    off = vaddr - APP_BASE
    got = 0
    while got < need:
        if off + got + 2 > len(data):
            return None
        w = struct.unpack_from("<H", data, off + got)[0]
        if not _pic_safe(w):
            return None                # can't safely relocate; caller picks another site
        got += 2
    return data[off:off + got]


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: find_hook.py <oams_2.0.231.bin>")
    path = sys.argv[1]
    data = open(path, "rb").read()
    # sanity: confirm this looks like the app image at APP_BASE
    sp = struct.unpack_from("<I", data, 0)[0]
    if not (0x20000000 <= sp <= 0x20005000):
        print("warning: initial SP %#x — is this the app image (base 0x08004000)?"
              % sp)

    print("Hook candidates (feed one pair to build_spi_patch.py):\n")
    for name, va, desc in CANDIDATES:
        pro = prologue_for(data, va, 4)
        tag = pro.hex() if pro else "NOT PIC-SAFE — use a different site"
        print("  %-14s %#010x  %s" % (name, va, desc))
        print("       --hook %#x --hook-prologue %s\n" % (va, tag))
    lo, hi, desc = DISPATCH
    print("  dispatch giant %#010x..%#010x  %s" % (lo, hi, desc))
    print("       (interior detour — pick the i2c-read call site from r2:")
    print("        r2 -a arm -b 16 -m 0x08004000 -c 'e asm.bits=16; aaa; pdf @ %#x' %s)"
          % (lo, path))
    print("\nNote: the shim assumes the i2c_dev_read register ABI (r0=i2c, r1=reg_len,")
    print("r2=reg, r3=read_len, read@[sp,#0]). If you hook an object-ABI primitive")
    print("instead, adjust build_spi_patch.build shim arg handling to match the")
    print("transaction struct (obj+offsets), confirmed by 'pdf' + a bench probe.")


if __name__ == "__main__":
    main()
