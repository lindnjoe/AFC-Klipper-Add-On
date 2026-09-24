<!-- Ready-to-post GitHub issue for the OpenAMS project (e.g. OpenAMSOrg/klipper_openams
     or the mainboard firmware repo). Paste the body below. -->

# Enable SPI1 + SPI2 in mainboard firmware for the RFID A / RFID B readers

## What
The OpenAMS mainboard has two RFID reader connectors — **RFID A (CN9)** and
**RFID B (CN12)** — populated with **Fudan FM17580** readers (an MFRC522-class
13.56 MHz ISO14443A / MIFARE Classic chip). They're wired to the STM32F072RBT6,
but the current mainboard firmware (`oams_2.0.231`) is a Klipper CAN node built
**without SPI**, so the readers can't be reached.

**The ask:** rebuild the mainboard firmware with **hardware SPI1 and SPI2
enabled**. That's it — no MCU driver work. Everything above the SPI bus (reader
protocol, tag decode, Spoolman) is done on the host and already written.

## How I know the wiring (continuity buzz-out on the board)
The two readers are on **separate hardware SPI peripherals**:

| Signal | RFID A | RFID B |
|---|---|---|
| Bus | **SPI1** | **SPI2** |
| SCK | PA5 | PB13 |
| MISO | PA6 | PB14 |
| MOSI | PA7 | PB15 |
| CS | PC4 | PB11 |
| RST | common | common |
| 3V3 / GND | common | common |

Verified that A's and B's SCK/MOSI/MISO do **not** connect (separate buses), and
that the SPI signal pins land on the STM32F072 SPI1 / SPI2 peripheral pins.

## What the firmware currently exposes (and doesn't)
From `oams_2.0.231.bin`: standard Klipper commands (`allocate_oids`,
`config_reset`, `get_canbus_id`) + custom OAMS (`config_oams_buffer`,
`config_oams_f1s_hes`, …) + **`config_i2c` / `i2c_read` / `i2c_write`**. There is
**no `config_spi` / `spi_transfer`**, and no SPI peripheral base referenced in
the image — SPI simply isn't compiled in. Bootloader is Katapult (`kancan`), so
it's a normal CAN reflash.

## What's already built on the host side (ready to use)
A Klipper plugin, `AFC_OpenAMS_rfid`, that:
- builds an `MCU_SPI` per reader (SPI1 for A, SPI2 for B, CS as above),
- runs the MFRC522 register protocol → ISO14443A anticollision → MIFARE Classic
  auth → block read,
- decodes Bambu (HKDF), Anycubic, Snapmaker, and Creality tags,
- syncs to Spoolman (one tag = one spool, multi-colour aware).

Plus `AFC_OpenAMS_rfid_probe` — an `OAMS_RFID_PROBE` command that auto-confirms
the CS pins once SPI is live. This mirrors our in-production BigTreeTech ViViD
reader, which does exactly this over Klipper SPI.

Config once SPI is enabled:
```ini
[AFC_OpenAMS_rfid readerA]
spi_bus: spi1
cs_pin: <mcu>:PC4
slots: 0, 1

[AFC_OpenAMS_rfid readerB]
spi_bus: spi2
cs_pin: <mcu>:PB11
slots: 2, 3
```

## Why it's worth it
This gives OpenAMS the same automatic filament ID the Bambu AMS has — material,
colour, and Spoolman binding on insert — for the cost of one build-flag change,
because the readers are stock MFRC522 hardware and the host stack already exists.

Happy to provide the host module, test against the new firmware, and help wire it
into `klipper_openams`.
