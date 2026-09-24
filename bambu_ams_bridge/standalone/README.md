# Self-contained AMS updater (drop-in)

No dependency on the bridge repo or `update_bridge.sh`.

| file | goes in | what it is |
|---|---|---|
| `ams_flash.py` | your Klipper **config** dir | the whole updater (Python stdlib only) |
| `ams_flash.cfg` | included by your config | the update gcode commands |
| `ams_artifact.json` | beside `ams_flash.py` | **boxed** AMS firmware (AMS 2 / AMS 1) |
| `ht_artifact.json` | beside `ams_flash.py` | **AMS HT** firmware |

Ship only the artifacts for models you actually have — a missing one is
reported cleanly at step [2] and nothing is touched.

## One command per model

The artifact is per-model and they are NOT interchangeable: a boxed image is
168 blocks and an HT image is 157, and each carries its own enter-loader poke
(boxed `0x0700` id `0x00`, HT `0x1800` id `0x80`). So the model is named in the
command rather than in a parameter:

    AFC_BAMBU_AMS_UPDATE   ->  ams_artifact.json   (AMS 2 / AMS 1)
    AFC_BAMBU_HT_UPDATE    ->  ht_artifact.json    (AMS HT)

Using the wrong one is not destructive — the poke lands on a device that is not
on the wire and the flash refuses before erasing anything — but it is opaque
when it happens, which is why they are separate commands.

## Install

1. Copy `ams_flash.py`, `ams_flash.cfg` and your artifact(s) into
   `~/printer_data/config/` (or wherever your config lives).

2. **Edit the one path in `ams_flash.cfg`.** This is the only thing you have to
   change, and nothing works until you do.

   Find the absolute path by SSHing in and running:

   ```
   ls -l ~/printer_data/config/ams_flash.py
   ```

   Then put exactly that path into the `command:` line:

   ```ini
   [gcode_shell_command ams_flash]
   command: /usr/bin/python3 /home/YOURUSER/printer_data/config/ams_flash.py
   ```

   It must be **absolute** — Klipper does not expand `~` and will not resolve a
   relative path. On most installs only the username differs:

   | board / image | path |
   |---|---|
   | Raspberry Pi OS | `/home/pi/printer_data/config/ams_flash.py` |
   | MKS | `/home/mks/printer_data/config/ams_flash.py` |
   | BTT CB1 / Pi4B | `/home/biqu/printer_data/config/ams_flash.py` |
   | Orange Pi | `/home/orangepi/printer_data/config/ams_flash.py` |

   If the path is wrong the update command fails immediately with
   `can't open file ... No such file or directory` in `logs/ams_flash.log`.
   Nothing is sent to the AMS and nothing is erased — a wrong path here is
   harmless, just non-functional.

   Nothing else needs a path. The artifacts are found beside `ams_flash.py`.

3. Add `[include ams_flash.cfg]` to `printer.cfg` (or drop the `.cfg` where your
   config already globs `*.cfg`).

4. `FIRMWARE_RESTART`.

5. Check it took — this touches no hardware:

   ```
   AFC_BAMBU_AMS_UPDATE MODE=check
   ```

Needs Klipper's `gcode_shell_command` component (built in on Kalico; installable
on stock Klipper). The bridge must already run fw >= 1.68.

## Use

    AFC_BAMBU_AMS_UPDATE MODE=check          # read-only, changes nothing
    AFC_BAMBU_AMS_UPDATE                     # PREFLIGHT (probes, no erase)
    AFC_BAMBU_AMS_UPDATE MODE=go             # actually update (erases)

    AFC_BAMBU_HT_UPDATE MODE=go              # same, for an AMS HT

`LABEL` is an optional log tag (e.g. the unit's sticker serial) -- not required,
not validated. Progress and result go to `~/printer_data/logs/ams_flash.log`.

### Detaching for PREFLIGHT and GO (normally zero setup)

`MODE=check` is read-only and runs inline. `PREFLIGHT` and `GO` bounce klipper,
so the script relaunches itself into its own transient systemd unit first (a
child of the klipper service would be killed mid-flash). It finds a way to do
that automatically, trying in order:

1. **`systemd-run --user`** -- no sudo, no password. Works on any normal Klipper
   Pi (one that already runs a user service, e.g. the USB QR scanner). **This is
   the default and needs nothing from you.**
2. **`sudo -n systemd-run`** -- if you happen to have a passwordless sudoers
   grant.
3. **Password file** -- last resort only. Create `ams_sudo.pw` next to
   `ams_flash.py` (`printf '%s' 'YOURPASSWORD' > .../ams_sudo.pw`); the script
   uses it once and **deletes it automatically**, so the secret exists only for
   the moment of the flash.

On a stock setup you never touch any of this -- `MODE=go` just works.

Put **one** un-updated AMS on the bridge, then run it. It gates on: not printing,
artifact verified, bridge fw >= 1.68, exactly one AMS online, and cmd1 entering
the loader -- only `MODE=go`, past all of those, erases and rewrites. It is
idempotent: a stopped transfer leaves a recoverable unit in its loader; run
`MODE=go` again. Full background: `../docs/AMS_FLASH_RUNBOOK.md`.

### Automatic retry until the loader verifies (MODE=go)

The transfer is a sustained ~180 KB stream at 1.2 Mbaud and occasionally
corrupts a chunk. The AMS loader hashes the **whole** image and refuses a bad
one, so `MODE=go` **resends the whole flash until the loader itself reports
`success!`** -- typically **1-4 passes** (a few `chunk hash error` / `badimage`
passes before a clean one is normal, not a failure). Within a pass it also
follows the loader's own `seq_num` requests to resend a single glitched chunk in
place. Every non-success pass leaves the unit safely in its loader; the run ends
only on a loader-verified `success!` (or after 7 passes, when you just re-run).
The finish line in `logs/ams_flash.log` is:

    FLASH CONFIRMED: the loader verified the image and reset into the new firmware.

## Getting a per-model artifact

An artifact is built from a capture of a real printer updating that model:

    ../tools/ams_fw_replay.py extract <capture> <model>_artifact.json

For the AMS HT only, one can also be packed straight from Bambu's own image:

    ../tools/ams_fw_replay.py pack <n3s_...bin.sig> ht ht_artifact.json

Boxed models cannot be packed that way — their data blocks carry a 4-byte
per-block tag that is absent from the vendor image and is not derivable from
it, so a boxed artifact has to come from a capture.
