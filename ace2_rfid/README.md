# ACE2 RFID — read arbitrary filament tags on the Anycubic ACE Pro 2

Goal: let the ACE Pro 2 read **manufacturer tags** the way the Snapmaker U1's
OpenRFID does — Bambu (MIFARE Classic, encrypted), Anycubic (NTAG), Creality,
etc. — including the raw UID of any spool.

## Architecture: ACE2 as an MFRC522 bridge, OpenRFID on the host

The ACE2's reader is an **MFRC522** (ISO14443-A) controlled by its MCU, which
only hands the host a pre-decoded Anycubic summary — no raw tag access, no
MIFARE auth, and the firmware has **zero crypto**. Rather than port crypto +
MIFARE + every brand into the MCU, we add **two tiny firmware commands** that
expose the MFRC522's register read/write, then run the entire OpenRFID stack
**on the host** (Python), exactly like the U1.

```
host (Python, this repo)                     ACE2 firmware (2 new cmds)
  read_tag()                                   MFRC522_REG_READ  -> sub_0800F574
   ├─ MifareClassic: REQA/anticoll/select  ──▶ MFRC522_REG_WRITE -> sub_0800F5D0
   ├─ MFAuthent + block read (Bambu)        ◀─  (raw MFRC522 register access)
   ├─ NTAG read (Anycubic)
   ├─ HKDF-SHA256 key derivation
   └─ Bambu / Anycubic / … decoders (OpenRFID port)
```

All brand/key/crypto logic stays host-side and editable — the firmware is
written **once** and never needs reflashing to add a brand or key.

## Status

**Host half — built & verified** (`host/`, 9/9 tests pass):
- ACE2 V2 frame + CRC16/Kermit (matches firmware algorithm)
- HKDF-SHA256 (RFC-5869 verified); Bambu master key verified vs `BAMBU_SALT_HASH`
- MIFARE Classic activate → per-sector auth → read → **Bambu decode**
- NTAG read → **Anycubic decode**
- Unified `read_tag()` — returns the UID for **any** tag, decodes brand when known
- Verified end-to-end against a register-level MFRC522+tag **emulator** (only the
  two firmware primitives are stubbed — exactly the firmware contract)

**Firmware half — feasibility CONFIRMED on real silicon** (`firmware/`):
- Reader = MFRC522; SPI leaf funcs `sub_0800F574/F5D0` reusable
- Full 256 KB SWD dump captured (RDP Level 0); app region is byte-identical to
  the RE'd `V1.1.31` (71592 B, CRC `0x91A8`) ⇒ all addresses confirmed
- **Bootloader is CRC-16/Kermit only — no signature, no crypto, no key.** The
  go/no-go is answered: a patched image with a correct CRC flashes via the normal
  OTA path with no repack. See `firmware/BOOTLOADER_ANALYSIS.md`.
- MCU is a **GD32F303** (Cortex-M4 STM32F103 drop-in); ~43 KB free app flash
- **Guaranteed restore:** full SWD backup image in hand before any flash

## Verdict

Concept validated end-to-end on the host; firmware patch is small, feasible, and
now **confirmed flashable** — the bootloader signature question is settled
(CRC-only) by a full SWD dump, with a byte-perfect backup as the safety net.

## Using it with AFC / Klipper

The ACE2 firmware patch (commands `0x50`/`0x51`/`0x52`) is flashed; the host side
rides the **existing** ACE2 serial link — no new wiring. Two repo modules do it:

- `extras/AFC_ACE2.py` — adds the `mfrc522_reg_read`/`mfrc522_reg_write`/
  `mfrc522_reader_power` methods (opcodes `0x50`/`0x51`/`0x52`) to the ACE2 codec.
- `extras/AFC_ACE2_rfid.py` — `[AFC_ACE2_rfid]` module: a **single self-contained
  file** that inlines the whole reader stack (frame → MFRC522 → MIFARE/NTAG →
  HKDF → Bambu/Anycubic) and the `[AFC_ACE2_rfid]` Klipper object that reads a
  lane/slot tag and applies it to lanes + Spoolman.

Config:
```ini
[AFC_ACE2_rfid]
# 32-hex Bambu master key (omit for Anycubic/UID-only)
bambu_master_key: 9A759CF2C4F7CAFF222CB9769B41BC96
# Creality CFS keys (both needed to decode Creality tags; omit if you have none).
# key = the UID->MIFARE-key AES key, encryption_key = the AES-CBC payload key.
#creality_key: 713362755e74316e71665a2870662431
#creality_encryption_key: 484043466b526e7a404b4174424a7032
# OPTIONAL. The module reads the lane->slot numbering from the ACE unit itself,
# so normally you can omit this entirely. If given, it acts as an explicit
# allow-list of which lanes get RFID reads (and can override the slot for an
# odd wiring). Use the SAME lane names the ACE unit defines (e.g. lane0..lane3)
# — a stale/wrong map here no longer misroutes reads (the ACE map wins), but a
# lane you forget to list simply won't be read.
#lane_slot_map: lane0:0, lane1:1, lane2:2, lane3:3
auto_spoolman_create: False
# Two physical slots share one reader (0/1 -> reader0, 2/3 -> reader1); its
# antenna sees whichever tag is in range, so a read for one slot can catch the
# NEIGHBOUR slot's parked tag and assign the same spool to both lanes. With
# dedup on (default) a stage read whose UID matches the shared sibling's spool
# (while that sibling still has a spool present) is HALTED — the neighbour tag is
# put to sleep (ISO14443 HLTA) so it stops answering, and the read returns THIS
# lane's own tag instead (the one spinning past the reader). If only the
# neighbour is in range the staging keeps probing the next chunk. The match is
# restart-proof: it uses both the UID read this session AND the sibling lane's
# persisted spool_id resolved through the offline Spoolman cache, so a spool
# left in the neighbour slot across a reboot is still recognised.
shared_reader_dedup: True
# Auto-read the tag when a spool is inserted (the insert preload/feed spins the
# spool, sweeping the tag past the reader). On by default.
read_on_insert: True
read_on_insert_attempts: 3       # retries to catch the tag as it settles
read_on_insert_delay: 1.0        # seconds between attempts
# Skip the ACE's slow (~30-45s) autonomous load-to-toolhead-and-back autostage
# on insert and go straight to our dist_hub staging (which spins the spool ~2
# revolutions — plenty to read the tag). Disables the factory identify on ALL
# slots at boot; identify stays off (no re-enable flash/lock). DEFAULT ON — it's
# the whole point of this module; set False only to use factory autostaging.
# The disable waits for the ACE serial to finish connecting first (its connect
# races startup), retrying up to identify_disable_max_tries * identify_disable_retry.
#skip_factory_autostage: True
#identify_disable_retry: 0.5       # seconds between boot-disable attempts
#identify_disable_max_tries: 60    # ~30s window for the ACE serial to come up
# Restore the factory identify after each read. Default follows the switch
# above: False when skipping autostage, True when using it (autostaging needs
# identify on for the next insert).
#restore_identify: True
# Each ACE slot has its OWN MFRC522 (per-slot chip-select on the shared SPI2
# bus); the reg r/w "slot" field selects the chip while power is per-reader-pair.
# Default per-slot; set False only if a unit needs the legacy per-pair index.
#reader_reg_per_slot: True
```

Diagnose the chip-select mapping on a unit: `ACE_RFID_READ SLOT=2 REG=n` forces
the MFRC522 chip-select index to `n` (0..3) while powering that reader pair, so
you can find which reg index reads which physical slot's tag (omit `REG=` for
normal per-slot addressing).

Because `skip_factory_autostage` bypasses the factory's initial load, staging
first feeds a mandatory **initial load** (`stage_initial_dist`, default 300mm)
that both loads the filament into the unit AND is the RFID scan window — the tag
only enters the reader's arc for a moment each spool revolution, so this window
is fed in small `stage_read_chunk` steps (default 25mm; a large step aliases past
the tag) and read between each. Then `dist_hub` is fed on top, so the net load is
`stage_initial_dist + dist_hub` (e.g. 300 + 1250 = 1550). Once the tag reads, the
remainder is fed in one move.
```ini
#stage_read_chunk: 25       # mm per sample step during the initial scan window
#stage_initial_dist: 300    # mandatory initial load fed (+scanned) before dist_hub
```

On insert, `AFC_ACE.on_filament_insert` feeds the spool to the hub (spinning it)
and fires an `afc_ace:post_insert` event; this module reads the tag then and
applies it to the lane — no manual `ACE_RFID_READ` needed. A read error (serial
timeout, no tag, reader glitch) is logged and reported, never fatal — it can't
shut Klipper down.
Read a tag manually: `ACE_RFID_READ SLOT=0` (or `LANE=lane1`). `SLOT=` is the
**physical** slot (0..3); the module maps it to the shared reader (`slot >> 1`,
two readers cover four slots). With the **v2** firmware the module takes
ownership of the reader for the read — it disables the firmware's identify loop,
powers the reader itself (cmd `0x52`), reads, then restores — so encrypted Bambu
tags read cleanly without the firmware's power-gating interrupting the sequence.

Install note: the Klipper module is fully self-contained in
`extras/AFC_ACE2_rfid.py`; nothing outside `extras/` is loaded at runtime.

## Layout
```
extras/AFC_ACE2_rfid.py  self-contained: reader stack + [AFC_ACE2_rfid] module
tests/test_ace2_rfid_reader.py  driver tests vs an emulated reader+tag
tests/test_AFC_ACE2_rfid.py     reader-power sequence + slot-mapping wiring
firmware/DESIGN.md       the passthrough commands + integration + packaging
firmware/READER_POWER_PLAN.md  v2 reader-power capture, patch, validation
firmware/swu_tool.py     CRC verify / .swu unpack / bin surgery (CRC=0x91A8)
```

Brands decoded host-side:
- **Bambu** — MIFARE Classic, HKDF-derived keys from the master key (config).
- **Snapmaker U1** — MIFARE Classic, per-tag HKDF keys from public salts (no key;
  RSA signature not checked).
- **Creality CFS** — MIFARE Classic, MIFARE key = AES(UID) and an AES-128-CBC
  payload; both AES keys come from config (`creality_key` / `creality_encryption_key`).
- **Anycubic** — NTAG/Ultralight, no auth.
- **Elegoo** — NTAG213, plain EPC-256, no auth.

A MIFARE Classic tag is tried as Bambu (if a master key is set), then Snapmaker,
then Creality (if its keys are set); an NTAG is tried as Anycubic then Elegoo. A
small pure-Python AES-128 is bundled (klippy-env has no crypto lib). Creality and
Elegoo decoding is implemented from the published specs but not yet verified on a
physical spool.

Reference: OpenRFID by @suchmememanyskill (Snapmaker U1 Extended Firmware);
Bambu from Bambu-Research-Group/RFID-Tag-Guide; Anycubic + Snapmaker U1 + Creality
from DnG-Crafts (ACE-RFID, U1-RFID, cfs-programmer); Elegoo from
ELEGOO-3D/ELEGOO-RFID-Tag-Guide.
