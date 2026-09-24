# OpenAMS RFID-over-SPI patch — build & flash

Adds RFID A/B (FM17580 / MFRC522) to a stock OpenAMS mainboard (`oams_2.0.231`,
STM32F072RBT6) with **no firmware source**, by injecting a Cortex-M0 SPI driver and
detouring the i2c read path. Wiring is confirmed: **RFID A → SPI1 (PA5/6/7) CS PC4**,
**RFID B → SPI2 (PB13/14/15) CS PB11**. USB-DFU makes a bad flash a 30-second
recovery, so it's safe to iterate.

## Pieces
| File | Role |
|---|---|
| `spi_asm.py` | Cortex-M0 (ARMv6-M) Thumb assembler over rasm2 (labels + literal pool) |
| `spi_driver.py` | injected SPI1/SPI2 + MFRC522 leaves (`spi_init`, `spi_txrx`, `mfrc522_read/write`) |
| `build_spi_patch.py` | driver + i2c_dev_read detour + trampoline → flashable image |
| `find_hook.py` | locate the i2c-read hook site + its prologue on your firmware |
| host: `extras/AFC_OpenAMS_rfid.py` `transport: i2c_hook` | talks the hooked-i2c protocol |

Requires `radare2` (rasm2) and `pip install capstone`. The vendor `.bin` is not in
the repo (copyright) — drop your `oams_2.0.231.bin` in this folder.

## 1. Find the hook site
The firmware is `-flto`, so `i2c_dev_read` is inlined into one big dispatch
function; the hook site is the interior i2c-read call. Locate it (+ the bytes to
save) with:

```bash
python3 find_hook.py oams_2.0.231.bin
#   candidate hook: 0x0800XXXX  prologue: <hex bytes>
```

The prologue must be ≥ 4 bytes, even-aligned, and **position-independent** (no
pc-relative `ldr =`/`b`/`bl` in the saved bytes) — `find_hook.py` checks this and
walks forward to a safe cut if needed.

## 2. Build the image

```bash
python3 build_spi_patch.py oams_2.0.231.bin \
    --hook 0x0800XXXX --hook-prologue <hexbytes> \
    -o oams_2.0.231_rfid.bin
```

This appends the driver at `0x0801B000` (free flash above the app), places the shim
after it, and overwrites the hook prologue with a `B.W shim`. No CRC step is needed
— Katapult CRCs the uploaded bytes at flash time.

## 3. Flash over CAN (Katapult)
Same as any Katapult reflash of the OAMS node:

```bash
python3 ~/katapult/scripts/flashtool.py -i can0 -u <oams-uuid> -f oams_2.0.231_rfid.bin
```

If it hangs or the node won't come up: **USB-DFU recovery** — hold BOOT, tap NRST,
then flash stock `oams_2.0.231.bin` back with `dfu-util`. No damage possible.

## 4. Configure the host (patched-firmware transport)

```ini
[AFC_OpenAMS_rfid readerA]
transport: i2c_hook
i2c_mcu: oams_mcu            # your OAMS CAN mcu name
i2c_bus: i2c1               # any valid bus; magic reads never hit it
reader_index: 0             # RFID A
slots: 0, 1

[AFC_OpenAMS_rfid readerB]
transport: i2c_hook
i2c_mcu: oams_mcu
i2c_bus: i2c2
reader_index: 1             # RFID B
slots: 2, 3
```

Everything above the register R/W — MFRC522 protocol, MIFARE auth, and Bambu /
Anycubic / Snapmaker / Creality decode — is the same shared `read_tag` stack the
ACE2 and ViViD use, so all tag types work unchanged.

## 5. Bring-up
Run `OAMS_RFID_PROBE` (from `AFC_OpenAMS_rfid_probe`, pointed at the same
transport) to confirm each reader answers, then scan a tag. If a reader reads
`0x00`/`0xFF` for VersionReg, re-check CS wiring or the hook site.

## How the hook works
`command_i2c_read` fills a buffer via the (inlined) i2c read, then
`sendf("i2c_read_response …")`. The shim intercepts that read: on a magic `reg[]`
prefix `52 46` it runs an MFRC522 op over SPI and returns the byte; the untouched
response path ships it back. Any non-magic i2c read runs the saved original code.

```
read  reg -> i2c_read([0x52,0x46,'R',which,reg], 1) -> response[0] = value
write reg -> i2c_read([0x52,0x46,'W',which,reg,val], 1)   (ack ignored)
```
