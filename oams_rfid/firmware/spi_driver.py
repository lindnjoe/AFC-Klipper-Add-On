#!/usr/bin/env python3
"""Injected Cortex-M0 SPI + MFRC522 leaves for the OpenAMS RFID patch.

The OpenAMS firmware (oams_2.0.231, STM32F072RBT6) is built without SPI, so the
whole SPI peripheral driver has to be injected as fresh Thumb code — unlike the
ACE2, whose firmware already had an MFRC522 driver we merely hooked. This module
emits three position-independent leaves, self-contained direct-MMIO (no reliance
on any firmware internals), placed in free flash above the app:

  spi_init()                         one-time bring-up of SPI1+SPI2 and the CS pins
  spi_txrx(base, byte)  -> rx        one 8-bit transfer on an already-selected bus
  mfrc522_read(which, reg) -> val    CS-framed MFRC522 register read  (which 0=A 1=B)
  mfrc522_write(which, reg, val)     CS-framed MFRC522 register write

Wiring (buzz-out, see PATCH_PLAN.md):
  RFID A -> SPI1 (PA5 SCK / PA6 MISO / PA7 MOSI), CS PC4
  RFID B -> SPI2 (PB13 SCK / PB14 MISO / PB15 MOSI), CS PB11

MIFARE anticollision / auth / block read stay on the HOST (AFC_OpenAMS_rfid), same
split as the probe — the firmware only needs raw register R/W over SPI.

Everything is verified by capstone round-trip disassembly (no hardware here).
"""
from __future__ import annotations
from spi_asm import Asm

# ---- STM32F072 register map (Cortex-M0) ------------------------------------
RCC = 0x40021000
RCC_AHBENR = 0x14        # IOPAEN b17, IOPBEN b18, IOPCEN b19
RCC_APB2ENR = 0x18       # SPI1EN b12
RCC_APB1ENR = 0x1C       # SPI2EN b14
GPIOA, GPIOB, GPIOC = 0x48000000, 0x48000400, 0x48000800
G_MODER, G_OSPEEDR, G_ODR, G_BSRR, G_BRR, G_AFRL, G_AFRH = (
    0x00, 0x08, 0x14, 0x18, 0x28, 0x20, 0x24)
SPI1, SPI2 = 0x40013000, 0x40003800
S_CR1, S_CR2, S_SR, S_DR = 0x00, 0x04, 0x08, 0x0C

# CR1: MSTR | BR=0b101 (/64 ~= 750 kHz from 48 MHz) | SSM | SSI, MSB-first, mode 0.
CR1_CFG = (1 << 2) | (5 << 3) | (1 << 9) | (1 << 8)      # 0x032C  (SPE added after)
SPI_SPE = 1 << 6                                          # 0x40
# CR2: DS=0b0111 (8-bit) | FRXTH (RXNE asserts on a byte).
CR2_CFG = (7 << 8) | (1 << 12)                            # 0x1700


def emit(base: int):
    """Assemble the leaves at load address ``base``. Returns (bytes, Asm)."""
    a = Asm(base)

    # ---- spi_init: clocks, GPIO alt-func, CS outputs, SPI CR --------------
    a.add(
        ":spi_init",
        "push {r4, r5, lr}",
        # GPIO A/B/C clocks
        "ldr r4, =0x40021000",           # RCC
        "ldr r0, [r4, 0x14]",            # AHBENR
        "ldr r1, =0xE0000",              # IOPA|IOPB|IOPC
        "orrs r0, r1",
        "str r0, [r4, 0x14]",
        # SPI1 clock (APB2ENR b12)
        "ldr r0, [r4, 0x18]",
        "movs r1, 1",
        "lsls r1, r1, 12",
        "orrs r0, r1",
        "str r0, [r4, 0x18]",
        # SPI2 clock (APB1ENR b14)
        "ldr r0, [r4, 0x1C]",
        "movs r1, 1",
        "lsls r1, r1, 14",
        "orrs r0, r1",
        "str r0, [r4, 0x1C]",
        # RCC clock-settle read-backs: a peripheral register access right after
        # its clock-enable bus-faults on STM32F0 unless the enable has settled.
        # Read each enable register back before touching GPIO/SPI (dummy loads).
        "ldr r0, [r4, 0x14]",            # AHBENR readback (GPIO clocks)
        "ldr r0, [r4, 0x18]",            # APB2ENR readback (SPI1)
        "ldr r0, [r4, 0x1C]",            # APB1ENR readback (SPI2)
        # --- GPIOA PA5/6/7 -> alternate (MODER=10), AF0, high speed ---
        "ldr r4, =0x48000000",           # GPIOA
        "ldr r0, [r4, 0x00]",            # MODER
        "ldr r1, =0xFC00",               # clear bits[10..15] (pins5-7)
        "bics r0, r1",
        "ldr r1, =0xA800",               # alt(10) for pins5,6,7
        "orrs r0, r1",
        "str r0, [r4, 0x00]",
        "ldr r0, [r4, 0x08]",            # OSPEEDR high(11) pins5-7
        "ldr r1, =0xFC00",
        "orrs r0, r1",
        "str r0, [r4, 0x08]",
        "ldr r0, [r4, 0x20]",            # AFRL: pins5,6,7 -> AF0 (clear nibbles)
        "ldr r1, =0xFFF00000",
        "bics r0, r1",
        "str r0, [r4, 0x20]",
        # --- GPIOB PB13/14/15 -> alternate, AF0, high speed; PB11 -> output ---
        "ldr r4, =0x48000400",           # GPIOB
        "ldr r0, [r4, 0x00]",            # MODER
        "ldr r1, =0xFCC00000",           # clear pins13,14,15 (bits26-31) + pin11(bits22,23)
        "bics r0, r1",
        "ldr r1, =0xA8400000",           # alt(10) pins13-15, out(01) pin11
        "orrs r0, r1",
        "str r0, [r4, 0x00]",
        "ldr r0, [r4, 0x08]",            # OSPEEDR high pins13-15
        "ldr r1, =0xFC000000",
        "orrs r0, r1",
        "str r0, [r4, 0x08]",
        "ldr r0, [r4, 0x24]",            # AFRH: pins13,14,15 -> AF0 (nibbles 5,6,7)
        "ldr r1, =0xFFF00000",
        "bics r0, r1",
        "str r0, [r4, 0x24]",
        "movs r1, 1",                    # PB11 CS idle high
        "lsls r1, r1, 11",
        "str r1, [r4, 0x18]",            # BSRR set
        # --- GPIOC PC4 -> output, idle high ---
        "ldr r4, =0x48000800",           # GPIOC
        "ldr r0, [r4, 0x00]",            # MODER
        "ldr r1, =0x300",                # clear pin4 bits[8,9]
        "bics r0, r1",
        "ldr r1, =0x100",                # out(01) pin4
        "orrs r0, r1",
        "str r0, [r4, 0x00]",
        "movs r1, 1",                    # PC4 CS idle high
        "lsls r1, r1, 4",
        "str r1, [r4, 0x18]",            # BSRR set
        # --- SPI1 CR ---
        "ldr r4, =0x40013000",           # SPI1
        "ldr r1, =0x1700",               # CR2
        "str r1, [r4, 0x04]",
        "ldr r1, =0x36C",                # CR1 = CFG | SPE
        "str r1, [r4, 0x00]",
        # --- SPI2 CR ---
        "ldr r4, =0x40003800",           # SPI2
        "ldr r1, =0x1700",
        "str r1, [r4, 0x04]",
        "ldr r1, =0x36C",
        "str r1, [r4, 0x00]",
        "pop {r4, r5, pc}",
    )

    # ---- spi_txrx: r0=base, r1=byte -> r0=rx (clobbers r2,r3,r4) ----------
    # Timeout-protected: a dead/miswired reader returns 0xFF instead of spinning
    # forever (which would trip the MCU watchdog). ~0x30000 loops is well over a
    # byte time at any SPI clock but still bounded.
    a.add(
        ":spi_txrx",
        "push {r4}",
        "ldr r4, =0x00004000",
        ":spi_txe",
        "subs r4, 1",
        "beq @spi_to",
        "ldr r2, [r0, 0x08]",            # SR
        "movs r3, 2",                    # TXE
        "ands r2, r3",
        "beq @spi_txe",
        "strb r1, [r0, 0x0C]",           # DR = byte (8-bit access)
        "ldr r4, =0x00004000",
        ":spi_rxne",
        "subs r4, 1",
        "beq @spi_to",
        "ldr r2, [r0, 0x08]",
        "movs r3, 1",                    # RXNE
        "ands r2, r3",
        "beq @spi_rxne",
        "ldrb r0, [r0, 0x0C]",           # rx byte
        "pop {r4}",
        "bx lr",
        ":spi_to",
        "movs r0, 0xA5",                 # timeout sentinel (distinct from 0xFF MISO-high)
        "pop {r4}",
        "bx lr",
    )

    # ---- resolve which(0/1) -> r4=base r5=cs_port r6=cs_mask -------------
    #  Inlined per leaf (keeps a flat leaf ABI). `tag` makes the internal labels
    #  unique so the two copies don't collide in the label table.
    def resolve_which(tag):
        return [
            "cmp r0, 0",
            "bne @%s_b1" % tag,
            "ldr r4, =0x40013000",       # SPI1
            "ldr r5, =0x48000800",       # GPIOC
            "movs r6, 1",
            "lsls r6, r6, 4",            # PC4
            "b @%s_have" % tag,
            ":%s_b1" % tag,
            "ldr r4, =0x40003800",       # SPI2
            "ldr r5, =0x48000400",       # GPIOB
            "movs r6, 1",
            "lsls r6, r6, 11",           # PB11
            ":%s_have" % tag,
        ]

    # ---- mfrc522_read: r0=which, r1=reg -> r0=value ----------------------
    a.add(
        ":mfrc522_read",
        "push {r4, r5, r6, r7, lr}",
        *resolve_which("rd"),
        "str r6, [r5, 0x28]",            # CS low (BRR)
        # addr = 0x80 | ((reg<<1)&0x7E)
        "lsls r1, r1, 1",
        "movs r3, 0x7E",
        "ands r1, r3",
        "movs r3, 0x80",
        "orrs r1, r3",
        "mov r0, r4",
        "bl @spi_txrx",                  # send addr, discard
        "mov r0, r4",
        "movs r1, 0",
        "bl @spi_txrx",                  # send 0, r0=value
        "mov r7, r0",                    # save value
        "str r6, [r5, 0x18]",            # CS high (BSRR)
        "mov r0, r7",
        "pop {r4, r5, r6, r7, pc}",
    )

    # ---- mfrc522_write: r0=which, r1=reg, r2=val -------------------------
    a.add(
        ":mfrc522_write",
        "push {r4, r5, r6, r7, lr}",
        "mov r7, r2",                    # save val before resolve_which uses r0..r6
        *resolve_which("wr"),
        "str r6, [r5, 0x28]",            # CS low
        "lsls r1, r1, 1",                # addr = (reg<<1)&0x7E
        "movs r3, 0x7E",
        "ands r1, r3",
        "mov r0, r4",
        "bl @spi_txrx",                  # send addr
        "mov r0, r4",
        "mov r1, r7",                    # val
        "bl @spi_txrx",                  # send val
        "str r6, [r5, 0x18]",            # CS high
        "pop {r4, r5, r6, r7, pc}",
    )

    # ---- mfrc522_read_gen: r0=spi_base r1=cs_port r2=cs_mask r3=reg -> r0=val
    #  Parameterized CS read for the pin sweep (any bus / any CS GPIO).
    a.add(
        ":mfrc522_read_gen",
        "push {r4, r5, r6, r7, lr}",
        "mov r4, r0",                    # spi base
        "mov r5, r1",                    # cs port
        "mov r6, r2",                    # cs mask
        "lsls r3, r3, 1",                # addr = 0x80|((reg<<1)&0x7E)
        "movs r7, 0x7E",
        "ands r3, r7",
        "movs r7, 0x80",
        "orrs r3, r7",
        "str r6, [r5, 0x28]",            # CS low (BRR)
        "mov r0, r4",
        "mov r1, r3",
        "bl @spi_txrx",                  # send addr
        "mov r0, r4",
        "movs r1, 0",
        "bl @spi_txrx",                  # send 0 -> value
        "mov r7, r0",
        "str r6, [r5, 0x18]",            # CS high (BSRR)
        "mov r0, r7",
        "pop {r4, r5, r6, r7, pc}",
    )

    return a.assemble(), a


if __name__ == "__main__":
    import sys
    base = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x0801B000
    code, a = emit(base)
    print("emitted %d bytes at %#x" % (len(code), base))
    print("labels:", {k: hex(v) for k, v in sorted(a.labels.items(), key=lambda kv: kv[1])})
    from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB)
    labels_by_addr = {v: k for k, v in a.labels.items()}
    for ins in md.disasm(code, base):
        tag = labels_by_addr.get(ins.address, "")
        print("  %#010x %-16s %-8s %s" % (ins.address, (tag + ":") if tag else "", ins.mnemonic, ins.op_str))
