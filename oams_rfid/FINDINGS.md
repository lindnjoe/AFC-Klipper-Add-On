# OpenAMS RFID bring-up — findings & hand-off

Goal: read the AMS RFID reader (glued into the housing) from the OpenAMS mainboard
(KnightRadiant's OpenAMS Mainboard, STM32F072RBT6, `oams_2.0.231` firmware) by
binary-patching the prebuilt firmware — no MCU source, RFID not yet in firmware.

## What is confirmed

- **Reader**: `AMSRFID_V5_R06`, a standard MFRC522/FM17580-class SPI reader
  (chip + antenna coil + 8-pin `J74`). The host-side decode stack (Bambu/Anycubic/
  Snapmaker/Creality via the shared `read_tag`/`Mfrc522`/`MifareClassic` code) is
  ready and unchanged — only raw register R/W over the wire is needed from the MCU.
- **Reader location**: the RFID-A connector traces run to the **left bank** of the
  MCU (crystal X1 = PF0/PF1 is on that edge). The usable left-bank GPIO are
  **PC13, PC14, PC15, PA2** (digital) and **PC0–PC3, PA0, PA1** (stock = analog/ADC:
  FPS pressure + motor current). RFID A and RFID B share the bus; B's few unique
  signals via-jump to pads just above A's.
- The reader is **NOT** on PA5/PA6/PA7 + PC4 (the naive "SPI1" datasheet pins) —
  those are the follower-BLDC timer PWM (PA6/PA7) and ADC (PA5/PC4).

## Firmware toolchain (proven working on the live board)

All of the hard MCU-patching problems are solved and demonstrated on hardware:

- **Hook**: 16-byte M0-safe trampoline over the `oams_cmd_stats` sender
  (`fcn.0800d744`), replaying its saved prologue and resuming at `0x0800d754`.
- **Injected driver**: Cortex-M0 SPI (hardware + bit-bang) + MFRC522 register R/W
  leaves in free flash, all ARMv6-M-legal, self-contained direct-MMIO.
- **Observable channel (motor-free)**: results written to `0x20000210` surface to
  the host as `AFC_OAMS.f1s_hes_value[0..3]` (read over Moonraker) — Klipper's MCU
  is cooperative/non-preemptive, so writing right before the sender reads it wins.
- **Deployer trailer / CRC** regenerated so the kancan bootloader accepts the image.

## Output-pin map (discovered by driving each output high and observing)

| MCU pins | Function |
|---|---|
| PA6, PA7 | follower BLDC (timer PWM) |
| PC9, PC10, PC11 | slot 0 + slot 3 first-stage feeders |
| PC6, PC7, PC8 | feeder-stage LEDs (incl. red indicator) |
| PB5, PB13, PB14 | feeder 2 LED |
| PA3, PA15, PB4 | lane load/unload state |
| PC12 | (unmapped) |
| PB6/PB7 = I2C1 (AS5600 encoder), PB8/PB9 = CAN, PA11/PA12 = USB, PA13/PA14 = SWD |

## Exhaustive negative result

With the reader glued in (no probing possible), we searched entirely in firmware:

1. **All 24 role orderings** of SCK/MISO/MOSI/CS over PC13/PC14/PC15/PA2 → no version.
2. **Full 4-role search** (MISO+CS cycled too) across all 10 left-bank pins, with all
   10 driven high to release any on-bank RST → no version.
3. **Every output pin** driven as a power/RST-enable candidate (groups of 3, then all
   13 flipped to opposite-of-stock to cover active-high AND active-low) → no version.

Across 100s of samples the reader byte was **only ever `0x00` or `0xFF`** — never an
intermediate value. A clocked, powered MFRC522 returns varied bytes; a pure
`0x00`/`0xFF` bus is floating/coupled with nothing driving it. Conclusion: **the
reader is not being powered, and no CPU GPIO enables it** — its 3V3 is a hardware
rail (always-on or hardware-switched), off-limits to firmware.

## The one question that finishes this

For KnightRadiant / whoever has the mainboard schematic:

> On the mainboard's RFID connector, which STM32F072 pins are **SCK, MISO, MOSI,
> CS (NSS), and RST**? And is the reader's **3V3 always-on, or switched** (by what)?

With that, the driver is a ~20-minute change: point the injected bit-bang SPI at the
real pins, drive RST, and the proven observable + host decode stack do the rest.
