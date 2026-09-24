#!/usr/bin/env python3
"""Assemble the OpenAMS RFID-over-SPI firmware patch and emit a flashable image.

Strategy (see PATCH_PLAN.md): the firmware has no SPI, so we inject a Cortex-M0
SPI+MFRC522 driver (spi_driver.py) into free flash and reach it by **detouring
i2c_dev_read** — the small sync wrapper `command_i2c_read` calls to fill its read
buffer before `sendf("i2c_read_response …")`. Our shim matches that ABI:

    int i2c_dev_read(i2c, reg_len, reg, read_len, read)   // r0..r3, read @ [sp,#0]

On a magic `reg[]` prefix it does an MFRC522 register op over SPI and fills
`read[]`, then returns I2C_BUS_SUCCESS (0); the untouched `command_i2c_read` sends
the bytes back as a normal `i2c_read_response`. Any non-magic call tail-executes
the saved original prologue and branches back — real I2C devices are unaffected.

    reg = [0x52,0x46, 'R', which, mfreg]           -> read[0] = mfrc522_read(which,mfreg)
    reg = [0x52,0x46, 'W', which, mfreg, val]      -> mfrc522_write(...);  read[0]=0

No response encoder / sendf work, no dictionary surgery. Flashing: the kancan
bootloader validates a deployer trailer (magic 'OAMS' + CRC-32 over the whole app)
and refuses to boot on a mismatch, so the patched image MUST regenerate that
trailer — append_deployer_trailer() does this (verified against the stock image).

The ONE firmware-specific input is the address+prologue of i2c_dev_read; pin it
with find_hook.py (or --hook/--hook-prologue) — everything else is our own code,
verified by capstone round-trip disassembly. NOTE: assembled + disassembly-checked
here; SPI timing / CS framing / the exact hook site are confirmed on the bench
(flash over CAN, OAMS_RFID_PROBE), with USB-DFU as the recovery net.
"""
from __future__ import annotations
import argparse
import struct
import zlib
from spi_asm import Asm
import spi_driver

PATCH_BASE = 0x0801B000        # first free 2 KB flash page above the app (0x0801AF20)
APP_BASE = 0x08004000
MAGIC0, MAGIC1 = 0x52, 0x46    # 'R','F' — magic reg[] prefix
OP_READ, OP_WRITE = 0x52, 0x57  # 'R','W'
OAMS_MAGIC = 0x4F414D53         # 'OAMS' deployer trailer signature


def append_deployer_trailer(img: bytes) -> bytes:
    """Append the kancan 'OAMS' deployer trailer so the bootloader boots the app.

    The bootloader scans flash for this trailer, checks the magic + a header CRC,
    then CRC-32s the whole app (APP_BASE for app_len bytes) against the stored
    value and only jumps to the app on a match. Layout (verified against stock
    oams_2.0.231): 28-byte header {magic, app_len, app_crc, 0,0,0,0} + 4-byte
    header CRC. app_len/app_crc cover the content BEFORE the trailer."""
    content = bytes(img)
    app_len = len(content)
    app_crc = zlib.crc32(content) & 0xFFFFFFFF
    hdr = struct.pack("<7I", OAMS_MAGIC, app_len, app_crc, 0, 0, 0, 0)
    hdr_crc = zlib.crc32(hdr) & 0xFFFFFFFF
    return content + hdr + struct.pack("<I", hdr_crc)


def _bw(src: int, dst: int) -> bytes:
    """Encode B.W (T4) unconditional 32-bit branch (available on ARMv6-M)."""
    off = dst - (src + 4)
    S = (off >> 24) & 1
    I1 = (off >> 23) & 1
    I2 = (off >> 22) & 1
    imm10 = (off >> 12) & 0x3FF
    imm11 = (off >> 1) & 0x7FF
    J1 = (~(I1 ^ S)) & 1
    J2 = (~(I2 ^ S)) & 1
    return struct.pack("<HH", 0xF000 | (S << 10) | imm10,
                       0x9000 | (J1 << 13) | (J2 << 11) | imm11)


def _reassemble_with_tail(base, dl, hook_addr, saved_prologue, hook_resume,
                          init, mr, mw):
    """Clean single-pass assembly with the saved prologue emitted as raw words."""
    a = Asm(base)
    a.add(
        ":shim",
        "push {r4, r5, r6, r7, lr}",
        "mov r4, r0", "mov r5, r1", "mov r6, r2", "mov r7, r3",
        "cmp r5, 5", "bcc @orig",
        "ldrb r0, [r6, 0]", "cmp r0, 0x52", "bne @orig",
        "ldrb r0, [r6, 1]", "cmp r0, 0x46", "bne @orig",
        "ldr r0, =0x20000004", "ldr r1, [r0]", "ldr r2, =0x5346abcd",
        "cmp r1, r2", "beq @ready",
        "str r2, [r0]",
        "ldr r3, =0x%08x" % init, "blx r3",
        ":ready",
        "ldrb r0, [r6, 2]", "cmp r0, 0x57", "beq @do_write",
        "ldrb r0, [r6, 3]", "ldrb r1, [r6, 4]",
        "ldr r3, =0x%08x" % (mr | 1), "blx r3",
        "ldr r2, [sp, 0x14]", "cmp r7, 0", "beq @ok", "strb r0, [r2]", "b @ok",
        ":do_write",
        "ldrb r0, [r6, 3]", "ldrb r1, [r6, 4]", "ldrb r2, [r6, 5]",
        "ldr r3, =0x%08x" % (mw | 1), "blx r3",
        ":ok",
        "movs r0, 0",
        "pop {r4, r5, r6, r7, pc}",     # magic path returns to command_i2c_read
        # --- non-magic: restore entry state, run saved prologue, resume in place.
        # We arrived via a branch (not a call), so lr is still the caller's return
        # and only needs r4-r7 + sp restored (M0 can't `pop {...,lr}`).
        ":orig",
        "mov r0, r4", "mov r1, r5", "mov r2, r6", "mov r3, r7",
        "pop {r4, r5, r6, r7}",         # restore caller-saved regs
        "add sp, 4",                    # discard the pushed lr copy (lr reg intact)
    )
    # saved original prologue as raw 16-bit words, then a B.W back to the hook.
    # The B.W is emitted INLINE (before the auto-appended literal pool) as two
    # placeholder hwords, then patched once we know resume_br's final address —
    # its size is fixed (4 B), so patching in place doesn't move anything.
    for i in range(0, len(saved_prologue), 2):
        w = saved_prologue[i] | (saved_prologue[i + 1] << 8)
        a.add(".hword 0x%04x" % w)
    a.add(":resume_br", ".hword 0x0000", ".hword 0x0000")
    code = bytearray(a.assemble())
    br_at = a.labels["resume_br"]
    code[br_at - base: br_at - base + 4] = _bw(br_at, hook_resume)
    return bytes(code), a


def emit_patch(fw_path: str, hook_addr: int, saved_prologue: bytes,
               out_path: str):
    fw = bytearray(open(fw_path, "rb").read())
    app_end = APP_BASE + len(fw)
    assert PATCH_BASE >= app_end, "patch base overlaps app (app ends %#x)" % app_end

    # 1) driver
    drv, da = spi_driver.emit(PATCH_BASE)
    shim_base = (PATCH_BASE + len(drv) + 3) & ~3
    hook_resume = hook_addr + len(saved_prologue)
    shim, sa = _reassemble_with_tail(
        shim_base, da.labels, hook_addr, saved_prologue, hook_resume,
        da.labels["spi_init"], da.labels["mfrc522_read"], da.labels["mfrc522_write"])

    # 2) lay out patch region
    region = bytearray(drv)
    region += b"\xff" * (shim_base - (PATCH_BASE + len(drv)))
    region += shim

    # 3) assemble full image: fw padded to PATCH_BASE, then region
    img = bytearray(fw)
    img += b"\xff" * (PATCH_BASE - app_end)
    img += region

    # 4) trampoline: overwrite the hook prologue with B.W shim
    tramp = _bw(hook_addr, sa.labels["shim"])
    assert len(tramp) <= len(saved_prologue), (
        "saved prologue (%d B) shorter than trampoline (%d B); save more insns"
        % (len(saved_prologue), len(tramp)))
    ho = hook_addr - APP_BASE
    img[ho:ho + len(tramp)] = tramp
    # if the prologue was longer than the B.W, pad the remainder with NOPs so we
    # don't leave a stray half-instruction before shim resumes.
    for i in range(ho + len(tramp), ho + len(saved_prologue), 2):
        img[i:i + 2] = b"\x00\xbf"     # nop

    img = bytearray(append_deployer_trailer(bytes(img)))   # bootloader accepts it
    open(out_path, "wb").write(img)
    return dict(patch_base=PATCH_BASE, shim=sa.labels["shim"],
                driver=PATCH_BASE, size=len(region),
                image=out_path, image_size=len(img), hook=hook_addr,
                resume=hook_resume)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("firmware", help="stock oams_2.0.231.bin")
    ap.add_argument("--hook", required=True, type=lambda s: int(s, 0),
                    help="i2c_dev_read address (from find_hook.py)")
    ap.add_argument("--hook-prologue", required=True,
                    help="hex of the original prologue bytes to save (>=4, even)")
    ap.add_argument("-o", "--out", default="oams_2.0.231_rfid.bin")
    args = ap.parse_args()
    saved = bytes.fromhex(args.hook_prologue)
    info = emit_patch(args.firmware, args.hook, saved, args.out)
    for k, v in info.items():
        print("  %-12s %s" % (k, hex(v) if isinstance(v, int) else v))
