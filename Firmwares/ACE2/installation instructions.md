# Flashing `AFC_ACE2PRO.bin` to an ACE 2 Pro

This is the AFC build of the ACE 2 Pro application firmware: stock **v1.4.0**
plus the RFID (always-on / "ungated") and speed patches. It is what makes the
unit report tags to AFC.

Flashing is done over the ACE's own serial link with
`ace2-ota-update.py`, which drives the same IAP sequence the
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
        --version 1.4.0 --dry-run
```

**2. Flash.**

```bash
python3 ace2-ota-update.py /dev/ttyCH343USB0 AFC_ACE2PRO.bin \
        --version 1.4.0
```

It prints what it is about to do and waits for confirmation:

```
  About to flash: 1.4.0  ->  1.4.0
  Image: 71592 bytes  CRC16=0x93AC
  Proceed? [y/N]
```

Answer `y`. Then leave it alone until it prints `[done] Flash complete`.

**3. Power cycle the ACE.** This is not optional and not a suggestion — the
unit commits the image but keeps running the old firmware until it is
physically power cycled. It does not reboot itself. Pull the power, wait a few
seconds, plug it back in.

---

## `--version` and the skip you will hit

The script compares `--version` against what the unit reports and **skips the
flash if they match**:

```
[skip] ACE already reports version 1.4.0. Use --force to flash anyway.
```

This build keeps the stock version string `1.4.0`, so a unit already on stock
v1.4.0 will refuse the first attempt. That is the expected path, not a fault.
Add `--force`:

```bash
python3 ace2-ota-update.py /dev/ttyCH343USB0 AFC_ACE2PRO.bin \
        --version 1.4.0 --force
```

The corollary is worth knowing: **the reported version cannot tell you whether
the patch is applied**, because patched and stock both say `1.4.0`. Confirm by
behaviour instead — the unit reporting RFID tags to AFC — or by hashing the
file you flashed.

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
`[AFC_ACE2_rfid]` configured for the unit; see `extras/AFC_ACE2_rfid.py` and
the ACE templates in `templates/`.

**Back to stock.** Flash a stock v1.4.0 image the same way, with `--force`.
Keep one archived — this is the reason to.
