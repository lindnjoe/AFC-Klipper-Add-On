#!/usr/bin/env python3
"""RST-pin sweep + observability beacon for the OpenAMS RFID bring-up.

The reader returns 0xFF (SPI transacts, MFRC522 silent) — a hardware condition,
almost certainly NRSTPD/reset held low so the chip sits in power-down. RST is
common to both readers; we don't know which MCU pin (if any) it lands on. This
tool finds it by *process of elimination in firmware*: drive each candidate pin
high one at a time and see which one makes reader A answer 0x91/0x92.

HOOK (clean, M0-safe, no prologue surgery): the OAMS periodic task fcn.0800d550
is called from exactly one site — `bl 0x800d550` at 0x0800d5c0, inside the
registered scheduler task fcn.0800d5a8 (DECL_TASK, fires on a timer at idle). We
rewrite that single `bl` to call our stub; the stub does its work then tail-calls
the real task via `blx` and returns to 0x0800d5c4. Nothing else moves.

CHANNEL (motor-free): results are written to hub_hes[2]/[3] (RAM 0x2000205f/60),
which the firmware streams to the host as `hub_hes_value_2/3` in oams_cmd_stats —
a pure read-out. We do NOT touch f1s_hes (0x20002059), which is the first-stage
feeder trigger. The write happens AFTER the tail-call (post state-update) to best
survive to the stats send.

Two modes:
  --beacon        no pins touched: write (idx=42, ver=0x91) every tick. Proves the
                  hook fires and the hub_hes channel reaches the host. Zero risk.
  (default)       sweep the candidate pins; on the pin that wakes reader A, leave
                  it high and report hub_hes[2]=pin_index(1-based), hub_hes[3]=ver.
                  hub_hes_value_2 == 0 means "no candidate woke the reader".

    python3 build_sweep.py oams_2.0.231.bin --beacon -o oams_beacon.bin
    python3 build_sweep.py oams_2.0.231.bin           -o oams_sweep.bin

Read the result over the tunnel from the OAMS stats log line:
    ... hub_hes_value_2=<pin index> hub_hes_value_3=<version> ...
Then map the index back to a pin via PIN_TABLE below, and flash stock to stop.
"""
from __future__ import annotations
import argparse
import struct
from spi_asm import Asm
import spi_driver
from build_spi_patch import PATCH_BASE, APP_BASE, append_deployer_trailer

# Hook the STATS SENDER fcn.0800d744 (the oams_cmd_stats task): it reads the
# REPORTED hub_hes from 0x20000210 and f1s from 0x20002059, packs a buffer, and
# sendf()s it. A separate hook can't win the race (the HES handler fcn.0800cfe8
# rewrites 0x20000210 from live sensors), but Klipper's MCU is cooperative/
# non-preemptive: if we write 0x20000210[2..3] as the FIRST thing in a trampoline
# on the sender itself, nothing runs before it reads them a few insns later, so
# the value reaches the wire. We save the sender's first 16 bytes (a self-
# contained prologue), run our payload, replay the prologue (relocated), and jump
# back to the resume point.
STATS_SENDER = 0x0800D744              # trampoline target (16-byte prologue saved)
STATS_RESUME = 0x0800D754             # first instruction after the saved 16 bytes
STATS_FPS_SRC = 0x20002060            # fps double source (loaded by the prologue)
STATS_FMT_FN = 0x080141C4             # bl target inside the prologue (double conv)
HUB_REPORTED = 0x20000210             # reported hub_hes[0..3] (what the sender sends)
HUB_R2 = HUB_REPORTED + 2             # hub_hes[2] -> host hub_hes_value_2 (+3 = value_3)
# Original 16 bytes at STATS_SENDER (sanity check before trampolining):
STATS_ORIG16 = bytes.fromhex("1fb5154b1868596806f03afd011c6846")

GPIOA, GPIOB, GPIOC = 0x48000000, 0x48000400, 0x48000800

# Candidate common-RST pins, ordered most-likely-first (adjacent to the reader
# solder points), with the provably-used pins already eliminated:
#   USED (excluded): PA5/6/7 SPI1, PB13/14/15 SPI2, PC4+PB11 CS, PB6/7 I2C1,
#   PB8/9 CAN, PA11/12 USB, PA13/14 SWD.
# Each entry: (port_base, pin_number, human_name). Kept SHORT so the stateless
# per-tick sweep stays cheap (~a few ms/tick).
PIN_TABLE = [
    (GPIOC, 5,  "PC5"),
    (GPIOC, 6,  "PC6"),
    (GPIOC, 7,  "PC7"),
    (GPIOA, 4,  "PA4"),
    (GPIOA, 8,  "PA8"),
    (GPIOB, 10, "PB10"),
    (GPIOB, 12, "PB12"),
    (GPIOB, 2,  "PB2"),
    (GPIOB, 1,  "PB1"),
    (GPIOB, 0,  "PB0"),
    (GPIOA, 3,  "PA3"),
    (GPIOA, 15, "PA15"),
]


def _bl(src: int, dst: int) -> bytes:
    """Encode BL (T1) — valid on ARMv6-M — from `src` to `dst`."""
    off = dst - (src + 4)
    S = (off >> 24) & 1
    I1 = (off >> 23) & 1
    I2 = (off >> 22) & 1
    imm10 = (off >> 12) & 0x3FF
    imm11 = (off >> 1) & 0x7FF
    J1 = (~(I1 ^ S)) & 1
    J2 = (~(I2 ^ S)) & 1
    return struct.pack("<HH", 0xF000 | (S << 10) | imm10,
                       0xD000 | (J1 << 13) | (J2 << 11) | imm11)


def _m0_tramp(hook_addr, stub_addr):
    """M0-legal 16-byte trampoline (preserves lr; clobbers only r12/ip):
        push {r0}; ldr r0,[pc,#8]; mov ip,r0; pop {r0}; bx ip; nop; .word stub|1"""
    assert hook_addr % 4 == 0
    return (bytes.fromhex("01b4" "0248" "8446" "01bc" "6047" "00bf")
            + struct.pack("<I", stub_addr | 1))


def _replay_prologue():
    """The saved fcn.0800d744 prologue (0x800d744..0x800d753), relocated for our
    stub: same effect, but the bl is done via blx-register and we jump back to
    STATS_RESUME. Assumes lr already holds the original caller return."""
    return [
        "push {r0, r1, r2, r3, r4, lr}",         # reserve msg buffer + save lr
        "ldr r3, =0x%08x" % STATS_FPS_SRC,
        "ldr r0, [r3, 0]", "ldr r1, [r3, 4]",    # load fps double
        "ldr r3, =0x%08x" % (STATS_FMT_FN | 1), "blx r3",   # relocated bl 0x80141c4
        ".hword 0x1c01",                         # adds r1, r0, #0  (exact original)
        "mov r0, sp",
        "ldr r3, =0x%08x" % (STATS_RESUME | 1), "bx r3",    # resume at 0x800d754
    ]


def build_beacon(base, dl):
    """Trampoline stub on the stats sender: stamp reported hub_hes[2]=42/[3]=145
    right before the sender reads 0x20000210, then replay the saved prologue and
    resume. If hub_hes_value_2/3 show 42/145, the channel works end-to-end."""
    a = Asm(base)
    a.add(
        ":stub",                                  # entered via bx ip; lr = caller
        "ldr r3, =0x%08x" % HUB_R2,
        "movs r2, 42", "strb r2, [r3, 0]",        # reported hub_hes[2] = 42
        "movs r2, 0x91", "strb r2, [r3, 1]",      # reported hub_hes[3] = 145
        *_replay_prologue(),
    )
    return a.assemble(), a


def build_bitbang(base, dl):
    """Bit-banged SPI auto-probe over the 4 safe left-bank digital pins
    (PC13,PC14,PC15,PA2 — the only non-ADC, non-motor pins on the RFID side).
    Tries all 24 orderings of (SCK,MOSI,MISO,CS), reads MFRC522 VersionReg each
    way, reports the ordering that answers 0x91/0x92. RST is assumed tied to 3V3
    (standard MFRC522 module); IRQ unused. Report via 0x20000210 (f1s buffer):
        f1s_hes_value_0 = version (0 = none found)
        f1s_hes_value_1 = packed order  (sck<<6 | mosi<<4 | miso<<2 | cs)
        f1s_hes_value_2 = 0xAA if found else 0
    Pin indices: 0=PC13 1=PC14 2=PC15 3=PA2. Only these 4 are ever driven -> zero
    motor risk. Hooked on the stats sender (same trampoline as the beacon)."""
    # 10-pin descriptor table (port_base, bit). idx:
    #   0=PC13 1=PC14 2=PC15 3=PA2 4=PC0 5=PC1 6=PC2 7=PC3 8=PA0 9=PA1
    PINS = [(0x48000800, 13), (0x48000800, 14), (0x48000800, 15), (0x48000000, 2),
            (0x48000800, 0), (0x48000800, 1), (0x48000800, 2), (0x48000800, 3),
            (0x48000000, 0), (0x48000000, 1)]
    # SCK/MOSI search candidates (exclude 2=PC15 MISO and 3=PA2 CS):
    CAND = [0, 1, 4, 5, 6, 7, 8, 9]

    CODE = [
        ":stub",                                       # lr = caller
        "mov ip, lr",
        # enable GPIOA+B+C clocks
        "ldr r0, =0x40021000", "ldr r1, [r0, 0x14]",
        "ldr r2, =0x000E0000", "orrs r1, r2", "str r1, [r0, 0x14]",
        "ldr r1, [r0, 0x14]",
        # --- drive ALL 10 pins HIGH (RST coverage: whichever is RST is now high) ---
        "movs r4, 0",
        ":dah", "cmp r4, 10", "bcs @dahd",
        "mov r0, r4", "bl @res_out", "str r1, [r0, 0x18]",   # config output + BSRR high
        "adds r4, 1", "b @dah",
        ":dahd",
        "ldr r0, =0x00040000", ":pws", "subs r0, 1", "bne @pws",   # settle
        # vary MISO(r8) and CS(r9) across passes via SysTick (0..9 each)
        "ldr r0, =0xE000E018", "ldr r0, [r0]",
        "lsrs r1, r0, 10", "movs r2, 0xF", "ands r1, r2",
        "cmp r1, 10", "bcc @m_ok", "subs r1, 6", ":m_ok",
        "lsrs r2, r0, 14", "movs r3, 0xF", "ands r2, r3",
        "cmp r2, 10", "bcc @c_ok", "subs r2, 6", ":c_ok",
        "mov r8, r1", "mov r9, r2",
        # --- search SCK,MOSI over CAND; CS=PA2(3) and MISO=PC15(2) fixed ---
        "movs r4, 0", "movs r6, 0xFF", "movs r7, 0",   # i, min version, packed(i<<4|j)
        ":iloop", "cmp r4, 8", "bcs @report",
        "movs r5, 0",
        ":jloop", "cmp r5, 8", "bcs @inext",
        "cmp r4, r5", "beq @jnext",
        "ldr r0, =cand", "ldrb r0, [r0, r4]",          # sck  pinidx
        "ldr r1, =cand", "ldrb r1, [r1, r5]",          # mosi pinidx
        "mov r2, r8", "mov r3, r9",                    # miso, cs (this pass)
        "bl @bb_read",                                 # r0 = version
        "cmp r0, r6", "bcs @jnext",
        "mov r6, r0", "lsls r7, r4, 4", "orrs r7, r5", # remember best (i<<4|j)
        ":jnext", "adds r5, 1", "b @jloop",
        ":inext", "adds r4, 1", "b @iloop",
        ":report",
        "ldr r0, =0x20000210",
        "strb r6, [r0, 0]",                            # min version
        "strb r7, [r0, 1]",                            # packed cand indices (i<<4|j)
        "movs r1, r6", "movs r2, 0xF0", "ands r1, r2", "cmp r1, 0x90",
        "bne @nofound", "movs r1, 0xAA", "b @wmark", ":nofound", "movs r1, 0", ":wmark",
        "strb r1, [r0, 2]",                            # found marker
        "mov r1, r8", "lsls r1, r1, 4", "mov r2, r9", "orrs r1, r2", "strb r1, [r0, 3]",
        "mov lr, ip",
        *_replay_prologue(),

        # ---- res_out(idx r0) -> r0=port, r1=mask ; config pin as output ----
        ":res_out",
        "push {r4, lr}",
        "ldr r4, =pindesc", "lsls r1, r0, 3", "adds r4, r4, r1",
        "ldr r0, [r4, 0]", "ldr r4, [r4, 4]",          # r0=port, r4=bit
        "movs r1, 1", "lsls r1, r4",                   # r1 = mask = 1<<bit
        "ldr r2, [r0, 0]", "lsls r3, r4, 1",           # r2=moder, r3=2*bit
        "movs r4, 3", "lsls r4, r3", "bics r2, r4",    # clear the 2 bits
        "movs r4, 1", "lsls r4, r3", "orrs r2, r4",    # set 01 (output)
        "str r2, [r0, 0]",
        "pop {r4, pc}",

        # ---- res_in(idx r0) -> r0=port, r1=bit ; config pin as input ----
        ":res_in",
        "push {r4, lr}",
        "ldr r4, =pindesc", "lsls r1, r0, 3", "adds r4, r4, r1",
        "ldr r0, [r4, 0]", "ldr r4, [r4, 4]",          # r0=port, r4=bit
        "ldr r2, [r0, 0]", "lsls r3, r4, 1",
        "movs r1, 3", "lsls r1, r3", "bics r2, r1",    # clear 2 bits (input=00)
        "str r2, [r0, 0]",
        "mov r1, r4",                                  # return bit
        "pop {r4, pc}",

        # ---- bb_read(sck r0,mosi r1,miso r2,cs r3 indices) -> r0=version ----
        ":bb_read",
        "push {r4, r5, r6, r7, lr}",
        "sub sp, 32", "mov r7, sp",                    # r7 = frame base
        "mov r4, r1", "mov r5, r2", "mov r6, r3",      # save mosi/miso/cs idx
        "bl @res_out", "str r0, [r7, 0]", "str r1, [r7, 4]",     # sck (r0 in)
        "mov r0, r4", "bl @res_out", "str r0, [r7, 8]", "str r1, [r7, 12]",   # mosi
        "mov r0, r5", "bl @res_in", "str r0, [r7, 16]", "str r1, [r7, 20]",   # miso
        "mov r0, r6", "bl @res_out", "str r0, [r7, 24]", "str r1, [r7, 28]",  # cs
        "ldr r0, [r7, 24]", "ldr r1, [r7, 28]",
        "str r1, [r0, 0x18]",                          # CS high (idle)
        "str r1, [r0, 0x28]",                          # CS low (select)
        "movs r0, 0xEE", "bl @bb_byte",                # send addr (read VersionReg)
        "movs r0, 0", "bl @bb_byte",                   # send 0 -> r0 = version
        "mov r4, r0",
        "ldr r0, [r7, 24]", "ldr r1, [r7, 28]", "str r1, [r0, 0x18]",   # CS high
        "mov r0, r4",
        "add sp, 32",
        "pop {r4, r5, r6, r7, pc}",

        # ---- bb_byte(r0=byte) -> r0=rx ; r7 = frame base, MSB-first mode 0 ----
        ":bb_byte",
        "push {r4, r5, r6, lr}",
        "movs r5, 8", "movs r4, 0",                    # cnt, rx
        ":bb_loop",
        "lsls r0, r0, 1",                              # MSB -> carry
        "ldr r1, [r7, 8]", "ldr r2, [r7, 12]",         # mosi port/mask
        "bcs @bb_hi",
        "str r2, [r1, 0x28]", "b @bb_clk",             # MOSI low
        ":bb_hi", "str r2, [r1, 0x18]",                # MOSI high
        ":bb_clk",
        "ldr r1, [r7, 0]", "ldr r2, [r7, 4]",          # sck port/mask
        "str r2, [r1, 0x18]",                          # SCK high
        "movs r3, 8", ":bhd", "subs r3, 1", "bne @bhd",  # clock-high delay
        "ldr r3, [r7, 16]", "ldr r6, [r3, 0x10]",      # MISO IDR (sampled while high)
        "ldr r3, [r7, 20]", "lsrs r6, r3",             # >> miso_bit
        "movs r3, 1", "ands r6, r3",                   # isolate
        "lsls r4, r4, 1", "orrs r4, r6",               # rx = rx<<1 | miso
        "str r2, [r1, 0x28]",                          # SCK low
        "movs r3, 8", ":bld", "subs r3, 1", "bne @bld",  # clock-low delay
        "subs r5, r5, 1", "bne @bb_loop",
        "mov r0, r4",
        "pop {r4, r5, r6, pc}",
    ]
    TABLES = [":pindesc"]
    for port, bit in PINS:
        TABLES += [".word 0x%08x" % port, ".word %d" % bit]
    # SCK/MOSI candidate pin-index list (byte table)
    TABLES.append(":cand")
    for i in range(0, len(CAND), 2):
        TABLES.append(".hword 0x%04x" % (CAND[i] | (CAND[i + 1] << 8)))

    def mk(pad):
        a = Asm(base)
        a.add(*CODE)
        if pad:
            a.add(".hword 0x0000")           # 4-align pindesc (data; never executed)
        a.add(*TABLES)
        return a
    a = mk(False)
    a.assemble()
    if a.labels["pindesc"] % 4:              # word-indexed table must be 4-aligned
        a = mk(True)
    assert a.assemble() is not None and a.labels["pindesc"] % 4 == 0
    return a.assemble(), a


def build_gpioprobe(base, dl):
    """READ-ONLY GPIO survey — never drives a pin. Each stats firing it reads one
    GPIO register (SysTick-selected) and reports a 16-bit half of it through the
    safe buffer 0x20000210 (-> f1s_hes_value):
        f1s_hes_value_0 = tag = reg_index*2 + half   (0..31)
        f1s_hes_value_1 = 0
        f1s_hes_value_2 = half16 low byte
        f1s_hes_value_3 = half16 high byte
    Poll until all 32 tags are seen, then reassemble each 32-bit register. No motor
    can move (only GPIO regs are read; only the reported buffer is written)."""
    # 16 registers (index 0..15); MODER/ODR/IDR/AFRL/AFRH for A,B,C + 1 pad.
    A, B, C = 0x48000000, 0x48000400, 0x48000800
    REGS = [A + 0, A + 0x14, A + 0x10, A + 0x20, A + 0x24,
            B + 0, B + 0x14, B + 0x10, B + 0x20, B + 0x24,
            C + 0, C + 0x14, C + 0x10, C + 0x20, C + 0x24, A + 0]

    def build(pad):
        a = _gpioprobe_body(base, REGS, pad)
        a.assemble()
        return a
    a = build(0)
    if a.labels["regs"] % 4:                # word-indexed table MUST be 4-aligned
        a = build(1)                        # else `ldr r2,[r2,r0]` HardFaults on M0
    assert a.labels["regs"] % 4 == 0, "regs table not 4-aligned"
    return a.assemble(), a


def _gpioprobe_body(base, REGS, pad):
    a = Asm(base)
    a.add(
        ":stub",                                       # lr = caller (no calls in payload)
        # Enable GPIOA/B/C clocks first: reading an unclocked GPIO port HardFaults
        # on M0. IOPAEN|IOPBEN|IOPCEN = bits 17/18/19 of RCC->AHBENR (0x40021014).
        "ldr r0, =0x40021000", "ldr r1, [r0, 0x14]",
        "ldr r3, =0x000E0000", "orrs r1, r3", "str r1, [r0, 0x14]",
        "ldr r1, [r0, 0x14]",                          # readback settle
        "ldr r2, =0xE000E018", "ldr r2, [r2]",         # SysTick VAL
        "lsrs r0, r2, 10", "movs r1, 0x0F", "ands r0, r1",   # r0 = reg index 0..15
        "lsrs r1, r2, 9", "movs r3, 1", "ands r1, r3",       # r1 = half 0/1
        "lsls r3, r0, 1", "adds r3, r3, r1",           # r3 = tag = index*2+half
        "ldr r2, =regs", "lsls r0, r0, 2", "ldr r2, [r2, r0]",  # r2 = reg addr
        "ldr r2, [r2]",                                # r2 = 32-bit reg value
        "cmp r1, 0", "beq @lo", "lsrs r2, r2, 16",     # high half -> shift down
        ":lo", "uxth r2, r2",                          # r2 = 16-bit half
        "ldr r0, =0x20000210",
        "strb r3, [r0, 0]",                            # f1s_hes_value_0 = tag
        "movs r1, 0", "strb r1, [r0, 1]",              # f1s_hes_value_1 = 0
        "strb r2, [r0, 2]",                            # f1s_hes_value_2 = half lo
        "lsrs r2, r2, 8", "strb r2, [r0, 3]",          # f1s_hes_value_3 = half hi
        *_replay_prologue(),
    )
    if pad:
        a.add(".hword 0x0000")                         # align :regs to 4 (never executed)
    a.add(":regs")
    for r in REGS:
        a.add(".word 0x%08x" % r)
    return a


def build_sweep(base, dl, pins=None):
    """Stub wrapping the gate (runs every scheduler pass). Because that's a hot
    path, we THROTTLE with SysTick and test ONE pin per firing (stateless):

      * always: call the gate, keep its r0 return.
      * read SysTick VAL (0xE000E018, 24-bit free-running down-counter).
      * throttle: act only when (VAL & 0x3FF)==0  (~1 pass in 1024).
      * pick pin = (VAL>>10) & 0xF; skip if >= n (so we visit 0..n-1 over time).
      * spi_init; drive that pin high; settle ~2ms; read reader A VersionReg.
      * valid 0x9x -> report hub_hes[2]=pin+1, hub_hes[3]=ver, LEAVE PIN HIGH
        (only written on success, so the reading latches persistently); else Hi-Z.

    The winner, once high, stays powered; later firings that pick it re-confirm.
    Registers inside the act use r0-r3,r6,r7; r4 holds the saved gate result."""
    pins = pins if pins is not None else PIN_TABLE
    n = len(pins)
    init = dl["spi_init"] | 1
    mr = dl["mfrc522_read"] | 1
    a = Asm(base)
    a.add(
        ":stub",                                       # entered via bx ip; lr = caller
        "mov ip, lr",                                  # preserve caller lr across SPI calls
        # --- pick ONE pin from SysTick (stateless); the sender fires ~1/sec so no
        #     throttle needed. Over many firings this cycles all candidates. ---
        "ldr r0, =0xE000E018", "ldr r0, [r0]",         # SysTick VAL (24-bit)
        "lsrs r5, r0, 10", "movs r2, 0x0F", "ands r5, r2",  # r5 = slot 0..15
        "cmp r5, %d" % n, "bcs @after",                # out of range -> no test this firing
        # --- spi_init (preserves r4,r5), drive PIN_TABLE[r5] high ---
        "ldr r3, =0x%08x" % init, "blx r3",
        "ldr r0, =tab", "lsls r1, r5, 3", "adds r0, r0, r1",
        "ldr r6, [r0, 0]",                             # r6 = port base
        "ldr r2, [r0, 4]", "mov r7, r2",               # r7 = pin number
        # --- SAFETY GUARD: skip pins the firmware already drives (MODER != 00 =
        #     output/AF/analog -> motors, PWM, comms). Only ever drive plain-input
        #     pins, where a spare RST wire would sit. This prevents touching the
        #     feeder-motor control pins. ---
        "lsls r2, r7, 1",                              # 2*pin
        "ldr r1, [r6, 0]",                             # MODER
        "movs r0, 3", "lsls r0, r2", "ands r1, r0",    # isolate this pin's 2 bits
        "bne @after",                                  # in use -> DO NOT drive
        # --- pin is a plain input: safe to drive high and test ---
        "lsls r2, r7, 1",
        "ldr r1, [r6, 0]",
        "movs r0, 3", "lsls r0, r2", "bics r1, r0",
        "movs r0, 1", "lsls r0, r2", "orrs r1, r0",
        "str r1, [r6, 0]",                             # MODER -> output
        "movs r0, 1", "lsls r0, r7", "str r0, [r6, 0x18]",   # BSRR high
        "ldr r0, =0xC000",                             # settle ~3ms (MFRC522 startup)
        ":swdly", "subs r0, 1", "bne @swdly",
        "movs r0, 0", "movs r1, 0x37",
        "ldr r3, =0x%08x" % mr, "blx r3",              # r0 = version (preserves r4-r7)
        # --- ALWAYS Hi-Z the pin (so next firing's reader is asleep unless its own
        #     pin is RST -> no cross-contamination / false positives) ---
        "mov r4, r0",                                  # save version
        "lsls r2, r7, 1", "ldr r1, [r6, 0]",
        "movs r0, 3", "lsls r0, r2", "bics r1, r0", "str r1, [r6, 0]",
        "mov r0, r4",                                  # restore version
        "movs r1, r0", "movs r2, 0xF0", "ands r1, r2",
        "cmp r1, 0x90", "bne @after",
        # found: stamp reported hub_hes[2]=index+1, [3]=version (surfaces this firing)
        "adds r5, r5, 1",
        "ldr r1, =0x%08x" % HUB_R2,
        "strb r5, [r1, 0]", "strb r0, [r1, 1]",
        ":after",
        "mov lr, ip",                                  # restore caller lr
        *_replay_prologue(),
    )
    a.add(":tab")
    for port, pin, _name in pins:
        a.add(".word 0x%08x" % port, ".word %d" % pin)
    return a.assemble(), a


def emit(fw_path, out_path, beacon=False, pins=None, gpioprobe=False):
    fw = bytearray(open(fw_path, "rb").read())
    app_end = APP_BASE + len(fw)
    assert PATCH_BASE >= app_end, "patch base overlaps app (ends %#x)" % app_end
    drv, da = spi_driver.emit(PATCH_BASE)
    stub_base = (PATCH_BASE + len(drv) + 3) & ~3
    if gpioprobe == "bitbang":
        stub, sa = build_bitbang(stub_base, da.labels)
    elif gpioprobe:
        stub, sa = build_gpioprobe(stub_base, da.labels)
    elif beacon:
        stub, sa = build_beacon(stub_base, da.labels)
    else:
        stub, sa = build_sweep(stub_base, da.labels, pins)

    region = bytearray(drv)
    region += b"\xff" * (stub_base - (PATCH_BASE + len(drv)))
    region += stub

    img = bytearray(fw)
    img += b"\xff" * (PATCH_BASE - app_end)
    img += region

    # install hook: 16-byte M0 trampoline over the stats sender's saved prologue.
    off = STATS_SENDER - APP_BASE
    assert bytes(img[off:off + 16]) == STATS_ORIG16, (
        "stats-sender bytes %s != expected %s (wrong image/offset)"
        % (bytes(img[off:off + 16]).hex(), STATS_ORIG16.hex()))
    img[off:off + 16] = _m0_tramp(STATS_SENDER, sa.labels["stub"])

    img = bytearray(append_deployer_trailer(bytes(img)))
    open(out_path, "wb").write(img)
    npins = 0 if beacon else len(pins if pins is not None else PIN_TABLE)
    return dict(stub=sa.labels["stub"], driver=PATCH_BASE, hook=STATS_SENDER,
                mode="beacon" if beacon else "sweep",
                pins=npins, image=out_path, image_size=len(img))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("firmware")
    ap.add_argument("--beacon", action="store_true",
                    help="plumbing test only (no pins driven)")
    ap.add_argument("--pins", default=None,
                    help="comma-separated subset to sweep (e.g. PC5,PC6). "
                         "Default: all. Use to bisect a couple at a time.")
    ap.add_argument("--gpioprobe", action="store_true",
                    help="READ-ONLY GPIO survey (drives nothing)")
    ap.add_argument("--bitbang", action="store_true",
                    help="bit-bang SPI auto-probe over PC13/14/15/PA2 (safe)")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()
    if args.bitbang:
        out = args.out or "oams_bitbang.bin"
        info = emit(args.firmware, out, gpioprobe="bitbang")
        for k, v in info.items():
            print("  %-12s %s" % (k, hex(v) if isinstance(v, int) else v))
        raise SystemExit(0)
    if args.gpioprobe:
        out = args.out or "oams_gpioprobe.bin"
        info = emit(args.firmware, out, gpioprobe=True)
        for k, v in info.items():
            print("  %-12s %s" % (k, hex(v) if isinstance(v, int) else v))
        raise SystemExit(0)
    sel = None
    if args.pins:
        want = [p.strip().upper() for p in args.pins.split(",")]
        sel = [e for e in PIN_TABLE if e[2] in want]
        missing = [w for w in want if w not in {e[2] for e in PIN_TABLE}]
        if missing:
            raise SystemExit("unknown pins: %s (known: %s)"
                             % (missing, ",".join(e[2] for e in PIN_TABLE)))
    out = args.out or ("oams_beacon.bin" if args.beacon else "oams_sweep.bin")
    info = emit(args.firmware, out, beacon=args.beacon, pins=sel)
    for k, v in info.items():
        print("  %-12s %s" % (k, hex(v) if isinstance(v, int) else v))
    if not args.beacon:
        print("\n  pin index -> pin (this bin):")
        for i, (_p, _n, name) in enumerate(sel if sel else PIN_TABLE, 1):
            print("    %2d -> %s" % (i, name))
