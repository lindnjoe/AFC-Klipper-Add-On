# Flashing `AFC_ACE2PRO.bin` to an ACE 2 Pro

This is the AFC build of the ACE 2 Pro application firmware, **AFCACE2
1.5**, flashed as 1.5.0. It is Anycubic's stock app with three additions:

- **ACE2-Open** by Simon-CR decodes spool tags on the ACE itself: Anycubic,
  Bambu Lab, OpenSpool, Spoolman/FilaMan, Prusament and Creality. The source
  is <https://github.com/Simon-CR/ace2-pro-firmware-research> (MIT licence, see
  `ACE2-Open-LICENSE.txt`). AFC also corrects its Bambu colour, weight and
  temperature fields.
- **AFC's speed patch** raises the feed/unwind cap from 100 to 140 mm/s.
- **AFC's register passthrough** from the earlier build is kept, so AFC can
  still drive the reader itself to write tags.

On this firmware AFC asks the ACE for each tag through `[AFC_ACE2_rfid]`, which
switches over by itself when the unit reports `AFCACE2 1.5`. The earlier
build (`AFCACE2PRO`, flashed as 1.4.0) decoded every tag on the printer
through the passthrough. Set `tag_decode: host` to keep doing that on 1.5.

AFC keeps the ACE's own tag reading on with this firmware, because the
firmware ignores a spool inserted into a slot where it is off. The ACE then
starts its factory autoload on each insert, and AFC stops it as soon as the
feeder has gripped the tip and stages the lane itself, as before.

The ACE 2 protocol, this updater and the firmware map the work builds on are
hakimio's: <https://gist.github.com/hakimio/4916ff69add458fdc51aeea76f21efb9>.

Flashing is done over the ACE's own serial link with
`Firmwares/ACE2/ace2-ota-update.py`, which drives the same IAP sequence the
Kobra S1 uses. **No printer, no SD card, no disassembly.**


---

## Before you start

| | |
|---|---|
| Cable | USB direct to the **ACE 2 Pro**, not to the printer |
| Dependency | `pip install pyserial` |
| Port |  `/dev/ttyCH343USB0`-style on Linux |
| Link speed | 230400 baud — the script sets this itself |
| Duration | roughly a minute; the image goes out in 64-byte chunks |

**Have the stock firmware to hand before you begin.** If a flash is
interrupted the unit stays in its IAP loader and can be re-flashed, but you
want the fallback image already downloaded rather than going looking for it
mid-recovery.

---

## Flash it

**1. Dry run first.** This talks to the unit, reads its current version and
parses the image, then exits without writing anything. If this does not work,
nothing else will:

```bash
python3 ace2-ota-update.py /dev/ttyCH343USB0 AFC_ACE2PRO.bin \
        --version 1.5.0 --dry-run
```

**2. Flash.**

```bash
python3 ace2-ota-update.py /dev/ttyCH343USB0 AFC_ACE2PRO.bin \
        --version 1.5.0
```

It prints what it is about to do and waits for confirmation:

```
  About to flash: AFCACE2PRO  ->  1.5.0
  Image: 80072 bytes  CRC16=0x2D24
  Proceed? [y/N]
```

Answer `y`. Then leave it alone until it prints `[done] Flash complete`.

**3. Power cycle the ACE.** This is not optional and not a suggestion — the
unit commits the image but keeps running the old firmware until it is
physically power cycled. It does not reboot itself. Pull the power, wait a few
seconds, plug it back in.

---

## Checking what the unit runs

This build reports **`AFCACE2 1.5`** through GET_INFO, so the version
tells you what is running. GET_INFO carries 11 characters at most. After the power cycle, AFC's log shows it in the
`ACE device info` line, and the updater prints it when it reconnects.

| Reported | Firmware |
|---|---|
| `AFCACE2 1.5` | this build (tags decoded on the ACE) |
| `AFCACE2PRO ` (trailing space) | a first test copy of this build, stamped `AFCACE2PRO 1.5.0` and cut off at 11 characters; reflash this one |
| `AFCACE2PRO` | the earlier AFC build (register passthrough) |
| `V1.1.31` | stock |

The updater skips the flash when `--version` already matches what the unit
reports. If you are re-flashing the same build, add `--force`.

---

## Other options

| flag | what it does |
|---|---|
| `--dry-run` | connect, read version, parse image, exit without writing |
| `--force` | flash even when the reported version already matches |
| `--verbose` | per-chunk progress; use it when a flash fails partway |
| `--md5 HASH` | verify an archive's checksum before extracting |
| `--swu-password PASS` | password for an encrypted `.swu` |
| `--chunk-size N` | leave alone; 64 is what the IAP expects |

The script also takes a Kobra S1 `.swu` package directly and extracts the ACE
binary itself — useful for going *back* to a stock image, which is the most
likely reason you would want it.

---

## If it goes wrong

**Nothing on the port.** Check you are on the ACE's own USB socket and not the
printer's. On Linux, `ls /dev/ttyCH343USB*` or `dmesg | tail` after plugging
in; the ACE uses a CH343 USB-serial bridge, which needs a driver on some
systems.

**Flash stops partway.** Power cycle and re-run with `--force --verbose`. The
unit stays in its IAP loader, so an interrupted flash is recoverable — that is
the whole point of the IAP design.

**Flashed, power cycled, no RFID.** The firmware is only one half. AFC needs
`[AFC_ACE2_rfid]` configured for the unit; see the RFID section of
`templates/AFC_ACE2_1.cfg` and `extras/AFC_ACE2_rfid.py`.

**Back to stock.** Flash the stock image the same way, with `--force`.
Keep one archived — this is the reason to.
