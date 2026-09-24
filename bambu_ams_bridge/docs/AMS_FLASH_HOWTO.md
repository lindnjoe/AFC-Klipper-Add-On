# Flashing an AMS from the bridge (operator how-to)

Update a stock Bambu AMS's own firmware over the bus, driven from Klipper -- no
BOOTSEL, no cable to the AMS. For the protocol and the design behind this, see
BRIDGE_OTA_RUNBOOK.md and AMS_FW_UPDATE.md; this page is the checklist.

## What you need (one-time)

- **Bridge firmware >= AFC-1.75** on the bridge. Check with `AFC_BAMBU_UIDS` --
  the firmware line must read `AFC-1.75` or higher. That build carries the
  AMS-flash transmit path and the self-verifying flasher. Either bridge works: a
  Pico 2 W (`tcp://` in `serial_port:`) or a USB Pico 2 (`/dev/serial/by-id/...`).
  `--cmd1flash` uses whichever the config points at; the USB path is proven and
  flashes in one pass on a good link.
- The Pi's repo checked out on **`claude/matched-set`** (it carries the flash
  tooling). `--cmd1flash` pulls the latest of that branch automatically.
- The per-model firmware artifact on the Pi at
  **`~/printer_data/config/ams_firmware/ams_artifact.json`** (header + 168
  blocks, CRC-verified on load).

## The one hard rule

**The AMS being flashed must be the ONLY unit on the bus.** Unplug every other
unit (the HT, any other AMS) first. A second unit contends on the bus and
corrupts the transfer. Not optional.

## Also before you start

- Printer **idle** (not printing or paused).
- The AMS powered and on the wire (24 V into the buffer's 4-pin, as normal).

## Flash it (one command)

From the console (Mainsail / Fluidd):

    RUN_SHELL_COMMAND CMD=bridge_update PARAMS="claude/matched-set --cmd1flash AMS2"

- The last token (`AMS2`) is just a confirmation label -- any string; the AMS
  serial is a good choice. It is printed, not checked against the hardware.
- This stops Klipper, enters the bridge's exclusive flash mode, erases +
  rewrites the AMS, **verifies the image by the loader's own hash**, then
  restarts Klipper. It **retries the whole flash automatically until the loader
  reports `success!`** -- typically 1-4 passes (the transfer is noisy, so a few
  passes is normal). **No power cycle needed.**
- Takes a few minutes. Watch `~/printer_data/logs/bridge_update.log`; the finish
  line is:

      FLASH CONFIRMED: the loader verified the image and reset into the new firmware.

## Verify

- The AMS does its normal **LED flash on reboot** and rejoins the bus.
- Insert a spool -- it should **pull it in**.
- `AFC_BAMBU_UIDS` shows the unit enrolled.

## If it does not come up

- Log ends with **`out of attempts`** (not verified in 7 passes): **run the same
  command again.** The AMS is left safely in its loader (recoverable) -- a
  re-run re-erases and re-flashes. Do NOT power it off mid-way; the flash is
  complete only when you see `FLASH CONFIRMED`.
- `--cmd1flash` errors that it cannot find `update_bridge.sh`: the Pi's repo is
  on the wrong branch. From a Pi shell:

      cd ~/Sovoron_klipper && git fetch origin claude/matched-set && \
        git checkout -f claude/matched-set

## After

- Re-plug the HT / other units and restart Klipper; they re-enroll on their own.

## Why it takes a few passes (not a bug)

The sustained ~180 KB transfer at 1.2 Mbaud over RS485 occasionally corrupts a
chunk. The AMS loader hashes the whole image and refuses a bad one; the flasher
simply retries until the loader verifies a clean image. It ALWAYS lands a
verified-good flash -- it is just not always one-shot. Getting it to one pass is
a hardware signal-integrity job (RS485 termination/bias/cable/ground), not the
flasher. The loader offers no per-chunk resend hook to exploit -- it advances
through all blocks and only checks the hash at the end -- so whole-flash retry is
the reliable ceiling in firmware.
