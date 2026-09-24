# Flashing an AMS from the bridge — step by step

Update an un-updated AMS to the current firmware **from our bridge, with no
Bambu printer** — `v05.00.22.22` for an AMS 2 (adds load-while-drying), or
`v05.00.22.19` for an AMS HT (adds heating while printing). This is the
operator recipe; the how-and-why is in `AMS_FW_UPDATE.md`.

It is the vendor's own signed image applied to the owner's own hardware.

---

## The one-command way (recommended)

From the console, use the gcode command:

    AFC_BAMBU_AMS_UPDATE LABEL=<serial>              # PREFLIGHT (probes, no erase)
    AFC_BAMBU_AMS_UPDATE LABEL=<serial> MODE=check   # read-only, changes nothing
    AFC_BAMBU_AMS_UPDATE LABEL=<serial> MODE=go      # actually update (erases)

`<serial>` is just a name for the unit. Read progress/result in
`~/printer_data/logs/bridge_update.log`. (The macro is a thin front end for
`RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--amsupdate <label> [check|go]"`,
which runs `tools/ams_update.py` — use that form directly if you prefer.)

**NAME THE MODEL FOR AN AMS HT.** The update image is per-model and the two are
not interchangeable, so the default (`ams2`) must be overridden for an HT:

    RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--amsupdate <label> ht"
    RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--amsupdate <label> go ht"

Order after the label does not matter (`go ht` and `ht go` both work). The model
picks the artifact *and* the unit names step 4 counts — a boxed default on an HT
bus finds zero units online and stops at step 4, which is safe but reads as
though the HT were missing. Confirm it from the first line of the log:

    model: ht  artifact: ht_artifact.json

The gates it enforces, in order:

  1. printer is not printing/paused,
  2. the per-model artifact verifies (header + its blocks, CRCs, and the
     blocks add up to the image the header declares),
  3. the bridge runs fw >= 1.68 (OTAs a *staged* image if needed, only with `go`),
  4. exactly one AMS is online on the bridge (refuses if 0 or >1),
  5. cmd1 makes that unit enter its loader (non-destructive probe),
  6. only past all of the above, with `go`: stream header + blocks (the erase),
  7. the unit reboots and is confirmed back online.

`check` stops after step 4 (touches nothing). Plain `--amsupdate` stops after
step 5 (probes but never erases). `go` is the only form that erases, and only
after 1-5 pass. It is idempotent -- a stopped transfer leaves a recoverable unit
in its loader; run `go` again. If step 3 finds the bridge below 1.68 and no
signed image is staged, it stops and tells you the one-time build command.

The rest of this document is the same flow done by hand -- read it to understand
what the one command is doing, or to run a step in isolation.

---

## What this does, in one paragraph

The bridge sends the AMS a `cmd1` frame, which makes the running AMS jump into
its bootloader. The bridge then streams the firmware (one header + the image's
data blocks — 168 for an AMS 2, 157 for an HT); the AMS erases its app flash
and writes the new image, reboots into it, and rejoins the bus. The bootloader itself is never erased, so an interrupted
transfer leaves a **recoverable** unit sitting in its loader — not a brick — and
you just run the flash again.

---

## Before you touch anything — read this

* **Not during a print.** The flash stops the bus and reboots the master.
* **One AMS on the bus for the flash.** The update is addressed to the unit's
  own device — `0x0700` for a boxed AMS, `0x1800` (AMS id `0x80`) for an AMS HT
  — and with more than one unit on the wire you cannot be sure which one
  answers. Also honour the heater interlock: one bus-powered AMS 2 at a time.
* **Right model.** Two artifacts are staged in `config/ams_firmware/`, each
  per-model and identical across units of that model:

      ams_artifact.json   AMS 2   n3f_rev5  v05.00.22.22  168 blocks
      ht_artifact.json    AMS HT  n3s_rev5  v05.00.22.19  157 blocks

  Pass `ht` for an HT (see above); the default is `ams2`. Any other model must
  first be captured being updated by a real printer and extracted with
  `ams_fw_replay.py extract` — do NOT flash one model's image at another. The
  tooling makes that hard rather than relying on you: the enter-loader poke is
  addressed FROM the artifact, so a mismatched pair is simply aimed at a device
  that is not on the wire, and the flash refuses before erasing anything.
* **Destructive but recoverable.** The header triggers an erase. If the transfer
  stops partway the unit stays in its loader; re-running `--cmd1flash` re-erases
  and rewrites from scratch. It is idempotent.

---

## One-time per bridge: get the bridge onto fw ≥ 1.68

The flash needs the `fwreplay`/`txbuf`/`txsend` transmit path, which is 1.68+.
Check first — skip this whole section if it already reports 1.68 or newer:

    AFC_BAMBU_UIDS            # look for "Bambu AMS bus (firmware AFC-1.6x)"

If it is older than 1.68, update the bridge. A **WiFi bridge updates by OTA over
the link, not USB** (the USB flasher cannot see it). Run each on the Klipper
console (or via RUN_SHELL_COMMAND for the first):

1. Build the signed encrypted WiFi image (stages `enc_flash_pico2w.uf2` into
   `config/`; ~2–5 min):

        RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--encpackage 168 w"

2. Dry-run the OTA — transfers, checks CRC + signature, writes nothing:

        AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=enc_flash_pico2w.uf2 APPLY=0

   Expect: `signed manifest v168 accepted` … `APPLY=0 … discarded`.

3. Apply it — the bridge writes its own flash and reboots (~10 s of link drop):

        AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=enc_flash_pico2w.uf2 APPLY=1

4. Confirm it came back on the new build:

        AFC_BAMBU_UIDS           # must now say firmware AFC-1.68

`UNIT=` can be any `AFC_BambuAMS` unit on that bridge — it targets the bridge,
not the AMS. A non-WiFi (USB) bridge is the exception: flash it with
`flash_pico.sh` / BOOTSEL instead of the OTA above.

---

## Flashing the AMS

1. **Wire it up.** Connect the un-updated AMS to the bridge's 4-pin bus, with no
   other AMS on the wire, and power it.

2. **Confirm it is talking.** In the console or the status, the unit should come
   online (its lane/slot shows up, humidity/temp read). If it never comes online,
   fix that first — the flash needs a live unit to answer `cmd1`.

3. **(Recommended) Non-destructive sanity check.** Prove it will enter the loader
   before you erase anything:

        RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--cmd1probe"       # AMS 2
        RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--cmd1probe ht"    # AMS HT

   A good result ends with `==> LOADER ENTERED … [MCU_UP] Loader Version / wait
   cmd1`. The unit is now sitting in its loader; it boots back to the app on a
   power cycle, or you can go straight to the flash. `NO JUMP` means the unit did
   not answer — stop and investigate (wrong unit online? wrong model, so the poke
   went to the wrong device?), do NOT flash. The header line echoes what it
   aimed at, so check that first:

        model=ht  device=0x1800  ams id=0x80

4. **Flash it.** Use a label that identifies the unit (its serial is ideal; the
   label is only a confirmation string):

        RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--cmd1flash 19C0-serial-here"
        RUN_SHELL_COMMAND CMD=bridge_update PARAMS="--cmd1flash <serial> ht"

   Watch `~/printer_data/logs/bridge_update.log`. A good run shows:

        enter-loader: cmd1 -> device 0x1800, ams id 0x80
        loader confirmed after cmd1 #1: [MCU_UP],wait cmd1,ams-origin/op0601
        header sent, erase triggered; streaming blocks...
          at block 24/156
          …
          at block 144/156
        loader: success! -- image verified, resetting into the app
        FLASH CONFIRMED: the loader verified the image and reset into the new firmware.

   **`success!` is the confirmation, and nothing else is.** It is the loader's
   own hash of the whole image passing, immediately before it resets into the
   app. Block acks are receipt, not integrity. A pass that ends any other way
   leaves the unit safely in its loader — just run it again.

   The whole transfer takes roughly 1–2 minutes. Klipper is stopped during it
   (the board allows only one client) and restarted automatically afterwards.

5. **Verify.** After Klipper restarts, the unit should reboot into the new
   firmware and rejoin online (its spool re-reads, lane shows LOCKED AND LOADED).
   Then do the real test: **start the dryer and load a tray** — loading while
   drying working is the proof the new firmware is in. (On an AMS HT that is the
   headline feature of `v05.00.22.19`, and the host-side gates that used to
   block it were removed on 2026-09-17.)

   Note you cannot read the version back to confirm a flash. An HT prints
   `[AMS_PMSM_C]get ams_id, N3S05-SN:<serial>, version:05.00.22.19` during
   power-on calibration only, and a loader-driven reset does not produce it —
   power-cycle the unit if you want to see it.

---

## If it stops partway

* `header … NO ACK` or a block `NO ACK`: the transfer stopped. The unit is in its
  loader (recoverable). Just run step 4 again — it re-erases and rewrites from
  scratch.
* Unit unresponsive after a stopped flash: it is waiting in its loader with no
  app. Re-run step 4; it will complete. (A plain power cycle boots a *fully
  written* unit to the app, but an erased one has no app to boot — finish the
  flash.)
* `--cmd1flash` refused with "no artifact": the artifact is missing from
  `config/ams_firmware/ams_artifact.json` — restage it
  (`ams_fw_replay.py extract <capture> ams_artifact.json`, then upload).
* "bridge did not enter fwreplay mode": the bridge is still on pre-1.68 firmware
  — do the one-time OTA section above.

---

## Doing several units

Flash them one at a time: power down the finished unit, wire up the next
un-updated one, and repeat "Flashing the AMS". The bridge stays on 1.68; only the
per-unit steps repeat.
