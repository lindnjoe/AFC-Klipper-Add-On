#!/usr/bin/env python3
"""Tiny two-pass Thumb assembler for the OpenAMS SPI patch (Cortex-M0 / ARMv6-M).

Why this exists: the STM32F072 on the OpenAMS mainboard is a Cortex-M0. That means
**no Thumb-2** — no ``movw``/``movt``/``ubfx``/``ldr.w``. 32-bit constants have to
come from a PC-relative literal pool, and there are real branch loops (SPI TXE/RXNE
waits) that need label resolution. rasm2 assembles individual mnemonics but has no
labels and no pools, so this wraps it.

Supported source lines (a Python list of strings):
  ":name"            label at the current address
  "ldr rX, =0xVAL"   load a 32-bit constant from the pool (M0-safe)
  "ldr rX, =name"    load the address of a label (Thumb bit NOT added)
  "b @name" / "beq @name" / "bne @name"   branch to a label
  ".word 0xVAL"      raw 32-bit word (also used to seed the pool)
  "<any thumb insn>" assembled verbatim via rasm2 (must encode to 16 bits on M0)

Everything is verified afterwards by disassembling the output with capstone in
CS_MODE_THUMB and eyeballing it against the source — there is no hardware in this
environment, so round-trip disassembly is the correctness gate.
"""
from __future__ import annotations
import subprocess
import struct
import re

_POOL = re.compile(r"^ldr\s+(r\d+),\s*=(.+)$", re.I)
_BR = re.compile(r"^(b|beq|bne|bcs|bcc|bhi|bls)\s+@(\w+)$", re.I)
_BL = re.compile(r"^bl\s+@(\w+)$", re.I)     # BL is a 4-byte insn on ARMv6-M


def _rasm(instr: str, origin: int) -> bytes:
    out = subprocess.check_output(
        ["rasm2", "-a", "arm", "-b", "16", "-o", "%#x" % origin, instr])
    return bytes.fromhex(out.decode().strip())


class Asm:
    """Two-pass Thumb assembler. ``base`` is the load address of the first byte."""

    def __init__(self, base: int):
        self.base = base
        self.lines: list[str] = []

    def add(self, *lines: str) -> "Asm":
        for ln in lines:
            self.lines.append(ln.strip())
        return self

    # -- pass helpers ---------------------------------------------------------
    def _size(self, ln: str) -> int:
        if ln.startswith(":"):
            return 0
        if ln.startswith(".word"):
            return 4
        if ln.startswith(".hword"):
            return 2
        if _BL.match(ln):
            return 4          # BL (T1) is 32-bit even on M0
        if _POOL.match(ln):
            return 2          # ldr rX,[pc,#imm]
        return 2              # every other M0 insn is 16-bit

    def assemble(self) -> bytes:
        # Pass 1: addresses of labels + collect pool constants (dedup).
        addr = self.base
        labels: dict[str, int] = {}
        pool: list[int | str] = []          # values or label-names
        pool_key: dict[str, int] = {}
        body: list[tuple[int, str]] = []     # (addr, line) for non-label lines
        for ln in self.lines:
            if ln.startswith(":"):
                labels[ln[1:]] = addr
                continue
            body.append((addr, ln))
            m = _POOL.match(ln)
            if m:
                key = m.group(2).strip()
                if key not in pool_key:
                    pool_key[key] = len(pool)
                    pool.append(key)
            addr += self._size(ln)
        # Pool goes after the code, 4-byte aligned (pad with a 16-bit nop).
        pool_base = (addr + 3) & ~3
        pad = pool_base - addr        # 0 or 2 bytes
        if pad:
            body.append((addr, "nop"))
        pool_addr = {k: pool_base + 4 * i for i, k in enumerate(pool)}

        def resolve(tok: str) -> int:
            tok = tok.strip()
            if tok in labels:
                return labels[tok]
            return int(tok, 0)

        # Pass 2: emit.
        out = bytearray()
        cur = self.base
        for a, ln in body:
            # pad to address a
            while cur < a:
                out += b"\x00"
                cur += 1
            m = _POOL.match(ln)
            b = _BR.match(ln)
            bl = _BL.match(ln)
            if bl:
                tgt = labels[bl.group(1)]
                enc = _rasm("bl %#x" % tgt, a)
                assert len(enc) == 4, "bl not 32-bit: %s" % ln
                out += enc
                cur += 4
            elif ln.startswith(".word"):
                val = ln.split(None, 1)[1]
                out += struct.pack("<I", int(val, 0) & 0xFFFFFFFF)
                cur += 4
            elif ln.startswith(".hword"):
                val = ln.split(None, 1)[1]
                out += struct.pack("<H", int(val, 0) & 0xFFFF)
                cur += 2
            elif m:
                rd, key = m.group(1), m.group(2).strip()
                dst = pool_addr[key]
                pc = (a + 4) & ~3
                imm = dst - pc
                assert 0 <= imm < 1024 and imm % 4 == 0, "pool reach %d @ %#x" % (imm, a)
                enc = _rasm("ldr %s, [pc, %#x]" % (rd, imm), a)
                assert len(enc) == 2, "pool ldr not 16-bit: %s" % ln
                out += enc
                cur += 2
            elif b:
                mnem, name = b.group(1), b.group(2)
                tgt = labels[name]
                enc = _rasm("%s %#x" % (mnem, tgt), a)
                assert len(enc) == 2, "branch not 16-bit (out of range?): %s -> %#x" % (ln, tgt)
                out += enc
                cur += 2
            else:
                enc = _rasm(ln, a)
                assert len(enc) == 2, "insn not 16-bit on M0: %r -> %s" % (ln, enc.hex())
                out += enc
                cur += 2
        # emit pool
        while cur < pool_base:
            out += b"\x00"
            cur += 1
        for k in pool:
            out += struct.pack("<I", resolve(k) & 0xFFFFFFFF)
        self.labels = labels
        self.pool_addr = pool_addr
        return bytes(out)


if __name__ == "__main__":
    # self-test: a literal load + a wait loop must round-trip.
    a = Asm(0x0801B000)
    a.add(
        ":start",
        "ldr r3, =0x40021000",     # RCC
        "movs r1, 1",
        ":wait",
        "ldr r2, [r3, 0x08]",
        "ands r2, r1",
        "beq @wait",
        "bx lr",
    )
    code = a.assemble()
    from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB)
    for ins in md.disasm(code, 0x0801B000):
        print("  %#010x  %-8s %s" % (ins.address, ins.mnemonic, ins.op_str))
    print("labels:", {k: hex(v) for k, v in a.labels.items()})
