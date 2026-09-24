#!/usr/bin/env python3
"""Bench smoke-test image: prove the injected SPI driver runs on real silicon.

This is the *first* on-hardware step, before any i2c/response hooking. It detours
the i2c-setup function (which runs at boot) to, one extra time, call `spi_init`
then `mfrc522_read(VersionReg)` on BOTH readers — results discarded. The SPI loops
are timeout-protected, so a dead/miswired reader returns 0xFF rather than hanging
the MCU.

Observable: if the OAMS CAN node stays connected after flashing (no MCU shutdown
in klippy.log), the whole SPI1/SPI2 bring-up + MFRC522 register access is safe on
the real STM32F072 — the hard part proven. If the node drops, the driver faulted
(we debug from there). USB-DFU restores stock either way.

Default hook: fcn.080105f8 (i2c_setup), prologue `f0b5 85b0` (push {r4-r7,lr};
sub sp,#0x14) — PIC-safe. Override with --hook/--hook-prologue if your image
differs (confirm with find_hook.py / r2).

    python3 build_bench_test.py oams_2.0.231.bin -o oams_bench.bin
"""
from __future__ import annotations
import argparse
import struct
from spi_asm import Asm
import spi_driver
from build_spi_patch import _bw, PATCH_BASE, APP_BASE, append_deployer_trailer


def _m0_tramp(hook_addr, stub_addr):
    """M0-legal 16-byte trampoline (B.W is ARMv7-M only and HardFaults on M0):
        push {r0}; ldr r0,[pc,#8]; mov ip,r0; pop {r0}; bx ip; nop; .word stub|1
    Preserves lr and all regs except r12 (scratch at a call boundary). The literal
    sits at hook+12; ldr at hook+2 has pc=(hook+2+4)&~3=hook+4, +8 -> hook+12."""
    assert hook_addr % 4 == 0, "hook must be 4-aligned for the pool math"
    return (bytes.fromhex("01b4" "0248" "8446" "01bc" "6047" "00bf")
            + struct.pack("<I", stub_addr | 1))

DEFAULT_HOOK = 0x080105F8
DEFAULT_PROLOGUE = bytes.fromhex("f0b585b0")     # push {r4-r7,lr}; sub sp,#0x14


def build_null(base, dl, hook_addr, saved_prologue, hook_resume):
    """Diagnostic: a transparent detour. The stub does NOTHING but run the saved
    original prologue and branch back — proves the trampoline + appended region +
    hook point are sound in isolation (no spi_init side effects). If this boots,
    the fault is inside spi_init; if it doesn't, the fault is the patch mechanism."""
    a = Asm(base)
    a.add(":bench")                       # entered via branch; lr untouched
    for i in range(0, len(saved_prologue), 2):
        w = saved_prologue[i] | (saved_prologue[i + 1] << 8)
        a.add(".hword 0x%04x" % w)
    a.add(":resume_br", ".hword 0x0000", ".hword 0x0000")
    code = bytearray(a.assemble())
    br_at = a.labels["resume_br"]
    code[br_at - base: br_at - base + 4] = _bw(br_at, hook_resume)
    return bytes(code), a


def build_replace(base, dl, hook_addr, which, reg, const=None):
    """Full-replace stub for a no-arg leaf. If ``const`` is given, just return it
    (diagnostic, no SPI). Otherwise return mfrc522_read(which, reg) as r0, NO
    resume. Entered via B.W (lr = caller return), so save lr and pop into pc.
    Hijacks fcn.0800bf28 (spool-index query) -> reader VersionReg, surfacing in
    the host's current_spool query + klippy.log."""
    a = Asm(base)
    if const is not None:
        a.add(":bench", "movs r0, %d" % (const & 0xFF), "bx lr")
        return a.assemble(), a
    init = dl["spi_init"] | 1
    mr = dl["mfrc522_read"] | 1
    a.add(
        ":bench",
        "push {r4, lr}",
        "ldr r4, =0x%08x" % init, "blx r4",     # spi_init (idempotent)
        "movs r0, %d" % (which & 0xFF),
        "movs r1, 0x%02x" % (reg & 0xFF),
        "ldr r4, =0x%08x" % mr, "blx r4",        # r0 = mfrc522_read(which, reg)
        "pop {r4, pc}",                          # return r0 to the caller
    )
    return a.assemble(), a


def build_spiprobe(base, dl, hook_addr, saved_prologue, hook_resume,
                   probe_addr, which, reg):
    """At a proven post-CAN detour: spi_init + mfrc522_read(which,reg), write the
    byte to probe_addr, then transparently resume. Tests spi_init post-CAN safety
    AND surfaces the reader byte if the hook fires often enough to beat the race.
    Preserves the hooked fn's args (r0-r3) + lr (blx clobbers lr)."""
    init = dl["spi_init"] | 1
    mr = dl["mfrc522_read"] | 1
    a = Asm(base)
    a.add(
        ":bench",
        "mov r12, lr",
        "push {r0, r1, r2, r3}",
        "ldr r3, =0x%08x" % init, "blx r3",      # spi_init
        "movs r0, %d" % (which & 0xFF),
        "movs r1, 0x%02x" % (reg & 0xFF),
        "ldr r3, =0x%08x" % mr, "blx r3",         # r0 = mfrc522_read(which,reg)
        "ldr r1, =0x%08x" % probe_addr,
        "strb r0, [r1]",                          # HES[0] = version byte
        "pop {r0, r1, r2, r3}",
        "mov lr, r12",
    )
    for i in range(0, len(saved_prologue), 2):
        w = saved_prologue[i] | (saved_prologue[i + 1] << 8)
        a.add(".hword 0x%04x" % w)
    a.add(":resume_br", ".hword 0x0000", ".hword 0x0000")
    code = bytearray(a.assemble())
    br_at = a.labels["resume_br"]
    code[br_at - base: br_at - base + 4] = _bw(br_at, hook_resume)
    return bytes(code), a


def build_dualcheck(base, dl):
    """Replace stub: read VersionReg on BOTH readers, return a spool-query code:
      reader A answered  -> A's version byte (0x91/0x92 = 145/146)
      only reader B      -> 201
      neither            -> A's raw byte (0xFF=255 no-answer, 0xA5=165 SPI timeout,
                            0x00 = MISO low)
    'valid' = not in {0x00, 0xFF, 0xA5}."""
    init = dl["spi_init"] | 1
    mr = dl["mfrc522_read"] | 1
    a = Asm(base)
    a.add(
        ":bench",
        "push {r4, r5, lr}",
        "ldr r4, =0x%08x" % init, "blx r4",
        "movs r0, 0", "movs r1, 0x37", "ldr r4, =0x%08x" % mr, "blx r4",  # read A
        "mov r5, r0",                         # save A
        "cmp r0, 0", "beq @tryB",
        "cmp r0, 0xFF", "beq @tryB",
        "cmp r0, 0xA5", "beq @tryB",
        "pop {r4, r5, pc}",                   # A valid -> return A's version
        ":tryB",
        "movs r0, 1", "movs r1, 0x37", "ldr r4, =0x%08x" % mr, "blx r4",  # read B
        "cmp r0, 0", "beq @none",
        "cmp r0, 0xFF", "beq @none",
        "cmp r0, 0xA5", "beq @none",
        "movs r0, 201", "pop {r4, r5, pc}",   # only B valid
        ":none",
        # neither answered: return a VISIBLE code so we can tell this apart from
        # "query never reached the hook". 199 = both idle (0xFF); 165 if A timed out.
        "cmp r5, 0xA5", "bne @none_ff",
        "movs r0, 165", "pop {r4, r5, pc}",
        ":none_ff",
        "movs r0, 199", "pop {r4, r5, pc}",
    )
    return a.assemble(), a


def build_probe(base, dl, hook_addr, saved_prologue, hook_resume,
                probe_addr, probe_val):
    """Observability probe: unconditionally write ``probe_val`` (u32) to
    ``probe_addr`` each time the hooked (post-CAN) function runs, then run the
    saved prologue and resume. Preserves the hooked fn's args (r0-r3) and lr.
    Used to empirically confirm which RAM address backs oams_fps_value."""
    a = Asm(base)
    a.add(
        ":bench",
        "mov r12, lr",
        "push {r0, r1, r2, r3}",
        "ldr r0, =0x%08x" % probe_addr,
        "ldr r1, =0x%08x" % probe_val,
        "strb r1, [r0]" if probe_val < 0x100 else "str r1, [r0]",
        "pop {r0, r1, r2, r3}",
        "mov lr, r12",
    )
    for i in range(0, len(saved_prologue), 2):
        w = saved_prologue[i] | (saved_prologue[i + 1] << 8)
        a.add(".hword 0x%04x" % w)
    a.add(":resume_br", ".hword 0x0000", ".hword 0x0000")
    code = bytearray(a.assemble())
    br_at = a.labels["resume_br"]
    code[br_at - base: br_at - base + 4] = _bw(br_at, hook_resume)
    return bytes(code), a


def build_bench(base, dl, hook_addr, saved_prologue, hook_resume):
    """Bench stub: save state, run spi_init + 2 VersionReg reads (discard),
    restore, run the saved prologue, branch back. Entered via a branch, so lr is
    the caller's return — we stash it in ip (r12, untouched by our leaves) across
    the blx calls and restore it before the saved prologue's `push {..,lr}`."""
    init = dl["spi_init"] | 1
    mr = dl["mfrc522_read"] | 1
    a = Asm(base)
    a.add(
        ":bench",
        "mov r12, lr",                   # preserve original return (blx clobbers lr)
        "push {r0, r1, r2, r3}",         # preserve i2c_setup's args
        "ldr r3, =0x%08x" % init, "blx r3",
        "movs r0, 0", "movs r1, 0x37", "ldr r3, =0x%08x" % mr, "blx r3",   # reader A
        "movs r0, 1", "movs r1, 0x37", "ldr r3, =0x%08x" % mr, "blx r3",   # reader B
        "pop {r0, r1, r2, r3}",
        "mov lr, r12",
    )
    for i in range(0, len(saved_prologue), 2):
        w = saved_prologue[i] | (saved_prologue[i + 1] << 8)
        a.add(".hword 0x%04x" % w)
    a.add(":resume_br", ".hword 0x0000", ".hword 0x0000")
    code = bytearray(a.assemble())
    br_at = a.labels["resume_br"]
    code[br_at - base: br_at - base + 4] = _bw(br_at, hook_resume)
    return bytes(code), a


def emit(fw_path, hook_addr, saved_prologue, out_path, null=False, no_hook=False,
         probe=None, replace=None):
    fw = bytearray(open(fw_path, "rb").read())
    app_end = APP_BASE + len(fw)
    drv, da = spi_driver.emit(PATCH_BASE)
    stub_base = (PATCH_BASE + len(drv) + 3) & ~3
    hook_resume = hook_addr + len(saved_prologue)
    if replace is not None:
        stub, sa = build_replace(stub_base, da.labels, hook_addr,
                                 replace[0], replace[1])
    elif probe is not None:
        stub, sa = build_probe(stub_base, da.labels, hook_addr, saved_prologue,
                               hook_resume, probe[0], probe[1])
    else:
        builder = build_null if null else build_bench
        stub, sa = builder(stub_base, da.labels, hook_addr, saved_prologue,
                           hook_resume)
    region = bytearray(drv)
    region += b"\xff" * (stub_base - (PATCH_BASE + len(drv)))
    region += stub
    img = bytearray(fw)
    img += b"\xff" * (PATCH_BASE - app_end)
    img += region
    if not no_hook:                      # --no-hook: leave the app untouched, region dead
        # M0-safe 16-byte trampoline (B.W is invalid on Cortex-M0). For a full
        # REPLACE this overwrites 16 B of the target's entry — fine, it's replaced.
        tramp = _m0_tramp(hook_addr, sa.labels["bench"])
        ho = hook_addr - APP_BASE
        img[ho:ho + len(tramp)] = tramp
    img = bytearray(append_deployer_trailer(bytes(img)))   # bootloader accepts it
    open(out_path, "wb").write(img)
    return dict(bench=sa.labels["bench"], driver=PATCH_BASE,
                hook=hook_addr, resume=hook_resume, image_size=len(img))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("firmware")
    ap.add_argument("--hook", type=lambda s: int(s, 0), default=DEFAULT_HOOK)
    ap.add_argument("--hook-prologue", default=DEFAULT_PROLOGUE.hex())
    ap.add_argument("-o", "--out", default="oams_bench.bin")
    ap.add_argument("--null", action="store_true",
                    help="transparent detour (diagnostic; no spi_init)")
    ap.add_argument("--no-hook", action="store_true",
                    help="append region + trailer but DON'T install the hook "
                         "(diagnostic: isolates the hook edit from the grow/trailer)")
    args = ap.parse_args()
    info = emit(args.firmware, args.hook, bytes.fromhex(args.hook_prologue),
                args.out, null=args.null, no_hook=args.no_hook)
    for k, v in info.items():
        print("  %-12s %s" % (k, hex(v) if isinstance(v, int) else v))
    print("flash oams_bench.bin over CAN; if the oams node stays up, SPI is proven.")
