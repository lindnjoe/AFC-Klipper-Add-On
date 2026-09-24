# OpenAMS Mainboard RFID (RFID A / RFID B) — request to enable SPI in firmware

**TL;DR:** The OpenAMS mainboard has two RFID reader connectors (RFID A / RFID B)
wired to the MCU as standard MFRC522 SPI. The current mainboard firmware
(`oams_2.0.231`) is a Klipper CAN node built **without SPI**, so those readers
can't be reached. Enabling SPI is a stock-Klipper build option that already
exists in the OpenAMS Klipper source (`src/spicmds.c`, `src/spi_software.c`).
If SPI is enabled on the RFID pins and exposed as a normal Klipper SPI bus, a
host-side plugin does the rest — no MCU driver work needed. We've already
written that plugin.

## What the readers are

- **Chip:** Fudan **FM17580** — an MFRC522-class 13.56 MHz reader (ISO/IEC
  14443 Type A, MIFARE Classic / "M1"), SPI host interface up to 10 MHz.
  Register-compatible with the NXP MFRC522, so the standard MFRC522 driver
  applies unchanged.
- **Connector:** RFID A (`CN9`) is an **8-pin** header matching the classic
  MFRC522/RC522 pinout: **CS(SDA), SCK, MOSI, MISO, IRQ, RST, GND, 3V3**.
  RFID B (`CN12`) is the same.
- **Wiring (confirmed by continuity buzz-out on the STM32F072 mainboard):** the
  two readers are on **separate hardware SPI peripherals** — **RFID A → SPI1**,
  **RFID B → SPI2** (their SCK/MOSI/MISO do NOT connect to each other; only
  3V3/GND/RST are common). CS is a per-reader GPIO.

## What the firmware currently is

- **MCU:** `STM32F072RBT6` (Cortex-M0, 128 KB flash, 16 KB RAM).
- **Firmware `oams_2.0.231`:** a Klipper/Kalico CAN node — commands seen in the
  image include `allocate_oids`, `config_reset`, `get_canbus_id`,
  `config_oams_buffer`, `config_oams_f1s_hes`, `config_oams_pid`, and
  **`config_i2c` / `i2c_read` / `i2c_write`**.
- **No SPI:** there is **no `config_spi` / `spi_transfer`** command and no SPI
  peripheral base address referenced anywhere in the image. SPI is simply not
  compiled into this build.
- **Bootloader:** Katapult (`kancan`), CRC32-verified — normal reflash over CAN.

## The ask (small)

1. Rebuild the mainboard firmware with **hardware SPI enabled for both SPI1 and
   SPI2** (`CONFIG_WANT_SPI`). RFID A is on SPI1 (PA5/PA6/PA7), RFID B on SPI2
   (PB13/PB14/PB15) — both stock STM32F072 SPI peripherals.
2. CS lines (from buzz-out): **RFID A CS = PC4**, **RFID B CS = PB11** (both
   plain GPIOs; SCK/MOSI/MISO are the fixed SPI1/SPI2 peripheral pins above).
   The RST line is common to both readers.
3. That's it on the firmware side — the readers then appear as normal Klipper
   SPI buses (`spi_bus: spi1` / `spi_bus: spi2` + `cs_pin:` per reader).

## What we provide (host side, already done)

A Klipper plugin, `AFC_OpenAMS_rfid`, that:
- builds an `MCU_SPI` per reader (shared bus, per-reader CS, optional RST),
- talks the MFRC522 register protocol to each FM17580,
- runs ISO14443A anticollision → MIFARE Classic auth → block read,
- decodes **Bambu (HKDF), Anycubic, Snapmaker, Creality** tags,
- and syncs to Spoolman (one tag = one spool, multi-colour aware).

It mirrors our existing BigTreeTech ViViD reader, which does exactly this over a
Klipper SPI bus with two MFRC522 readers and two CS lines. Once the firmware
exposes the bus, configuration is just:

```ini
[AFC_OpenAMS_rfid]
lane_slot_map: lane4:0, lane5:1, lane6:2, lane7:3

[AFC_OpenAMS_rfid readerA]
spi_bus: spi1               # RFID A → SPI1 (PA5/6/7), confirmed by buzz-out
cs_pin: oams_mcu1:PC4       # RFID A CS (buzz-out; confirm with OAMS_RFID_PROBE)
slots: 0, 1

[AFC_OpenAMS_rfid readerB]
spi_bus: spi2               # RFID B → SPI2 (PB13/14/15), confirmed by buzz-out
cs_pin: oams_mcu1:PB11      # RFID B CS (buzz-out; confirm with OAMS_RFID_PROBE)
slots: 2, 3
```

## Candidate STM32F072 SPI pins (for confirming the wiring)

| Bus | SCK | MISO | MOSI |
|---|---|---|---|
| SPI1 (A) | PA5 | PA6 | PA7 |
| SPI1 (B) | PB3 | PB4 | PB5 |
| SPI2 | PB13 | PB14 | PB15 |

CS/RST land on whatever GPIOs the connectors route to.

## Why this is worth it

RFID gives OpenAMS the same automatic filament identification the Bambu AMS has
— material, colour, and Spoolman binding on insert — with no MCU driver
development, because the readers are stock MFRC522 hardware and the host stack
is already written and in production on other units (ACE Pro 2, ViViD).
