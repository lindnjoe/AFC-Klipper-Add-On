# OpenAMS RFID — binary-patch feasibility (no firmware source)

Assessment of adding SPI + RFID to the prebuilt `oams_2.0.231.bin` (STM32F072RBT6)
**without** the firmware source, by binary patching — the fallback if the
maintainers don't add SPI upstream (see `openams_rfid_firmware_request.md`).

## Verdict: possible, ACE2-style, and unbrickable — but a real project

### Flashing barrier: none
- Bootloader is **Katapult (`kancan`)**, **CRC32-verified only** — no signature
  (image strings: `BL: Staging verified: CRC32=…`, `BL: CRC mismatch…`). A
  modified image with a corrected CRC flashes normally over CAN.
- **USB-DFU recovery exists** (hold BOOT, tap NRST — per OpenAMS docs), so a bad
  patch is always recoverable by reflashing stock `oams_2.0.231.bin`.

### Space for injected code
- The image is **0x16F20 (92 KB)** with **no internal code cave** (0 runs of
  `0xFF`/`0x00` ≥ 256 B).
- The MCU has **128 KB flash**; the app ends well below that, leaving **~36 KB
  of free flash above the image** for the injected SPI/MFRC522 driver. The
  patch tool extends the app region and re-computes the app CRC.

### The hard part, and how to dodge it
- The host learns the command set from a **zlib (`0x78da`) dictionary at
  `0x15e10`**: `{"build_version":"master-39a5cd2","commands":{…}}` (stock
  Klipper master + `config_oams_*`). Adding a **new** command (`config_spi` /
  `spi_transfer`) means rebuilding + relocating this dictionary and wiring a new
  entry into the compile-time dispatch table — painful.
- **Dodge it: hook an existing command instead of adding one.** The firmware
  already ships `i2c_read` / `i2c_write` and periodic `config_oams_*` status.
  Repurpose one (magic address/arg → do an SPI/MFRC522 op, return bytes through
  the existing response path). No dictionary surgery. This is the same
  hook-an-existing-leaf approach that worked on the ACE2.

## Confirmed wiring (buzz-out on hardware)
RFID A → **SPI1** (PA5/PA6/PA7) + CS **PC4**, RFID B → **SPI2** (PB13/PB14/PB15)
+ CS **PB11** — two separate hardware SPI peripherals (SCK/MOSI/MISO of A and B
do not connect; only 3V3/GND/RST are common). This means the patch drives the
real SPI1/SPI2 peripherals (not GPIO bit-bang), asserting PC4/PB11 as CS.

## Work items for a patch
1. **Pins are known** — SPI1/SPI2 fixed pins + CS PC4 (A) / PB11 (B). (Confirm
   live with `OAMS_RFID_PROBE` once SPI exists.)
2. **Author Thumb code** placed in free flash:
   - STM32F072 **SPI1 + SPI2 peripheral bring-up** (RCC clock enable, GPIO
     alt-function to AF0 for SPI, CR1/CR2 config) + the two CS GPIOs,
   - MFRC522 register R/W + MIFARE anticoll/auth/read.
3. **Hook `i2c_read`/`i2c_write`** (trampoline → stub → original), using the
   existing `ace2_rfid/firmware/build_patch.py` mechanics (BL/B thumb patch,
   CRC fix).
4. **Host side:** point `AFC_OpenAMS_rfid` at the hooked command instead of a
   real Klipper SPI bus (a thin transport shim over the magic i2c command).

## Recommendation
Try the **upstream ask first** (`openams_rfid_firmware_request.md`): enabling
SPI is a one-line build change for whoever holds the OAMS source, and the host
stack is already written. The binary patch is the viable-but-heavier plan B if
that stalls — de-risked by CRC-only flashing and USB-DFU recovery.
