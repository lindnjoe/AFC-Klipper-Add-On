#!/usr/bin/env python3
"""
One-command AMS firmware update, with every safety gate in front of the erase.

    ams_update.py --label <serial>              # PREFLIGHT only -- changes nothing
    ams_update.py --label <serial> --go         # actually update the AMS

It runs the whole path from `docs/AMS_FLASH_RUNBOOK.md` as one sequence and
refuses to reach the destructive step unless every check ahead of it passed:

  1. printer is not printing/paused          (else abort -- never stop klipper mid-print)
  2. the per-model artifact verifies          (header + N blocks, CRCs, and
                                               the blocks add up to the image)
  3. the bridge runs fw >= 1.68               (the transmit path). If not and --go,
     it is OTA-updated (build -> APPLY=0 verify -> APPLY=1) and re-checked.
  4. exactly one AMS is online on the bridge  (the target; refuses if ambiguous)
  5. cmd1 makes that unit enter its loader     (non-destructive probe)
  --- everything above is non-destructive; the erase is only reached past here ---
  6. stream header + its blocks (the erase)    (only with --go)
  7. the unit reboots and comes back online    (verified)

WITHOUT --go it stops after step 5 and reports GO / NO-GO, having changed
nothing on the AMS. Klipper is stopped only around the probe/flash (the board
allows one client) and always restarted. The flash is idempotent: a stopped
transfer leaves a recoverable unit in its loader; just run again with --go.

Moonraker (localhost) is used for the not-printing check, the bridge fw version,
the OTA gcode and the final online check; klipper is bounced with systemctl.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sniff_capture_link import Link                               # noqa: E402
from ams_cmd1_probe import build_cmd1, loader_hits, authenticate  # noqa: E402
import ams_fw_flash as F                                          # noqa: E402

TARGET_FW = 168                    # bridge fw the transmit path needs
ENC_IMAGE = "enc_flash_pico2w.uf2"
# Config dir. Overridden by --cfg-dir because this often runs as root (via the
# gcode_shell_command), where ~ is /root, not the printer user's home.
CFG_DIR = os.path.expanduser("~/printer_data/config")


#: Per-model update artifacts, and the units each model can appear as.
#:
#: THE ARTIFACT IS PER-MODEL AND NOT INTERCHANGEABLE. This used to be one
#: hardcoded path to the AMS 2 image, from when that was the only model we had
#: captured. An AMS HT is a different device (0x1800, ams id 0x80) with its own
#: image, and flashing one model's artifact at the other cannot work -- the
#: enter-loader poke is addressed from the artifact (F.loader_target_of), so it
#: would simply be aimed at a device that is not on the wire and the flash would
#: refuse before erasing anything. Refusing is the right outcome; needing to
#: have chosen correctly is better.
MODELS = {
    "ams2": {"artifact": "ams_artifact.json",
             "units": ["Bambu_AMS_1", "Bambu_AMS_2",
                       "Bambu_AMS_3", "Bambu_AMS_4"]},
    "ht":   {"artifact": "ht_artifact.json",
             "units": ["Bambu_AMS_HT_%d" % i for i in range(1, 9)]},
}
MODEL = "ams2"


def artifact_path(model=None):
    return os.path.join(CFG_DIR, "ams_firmware",
                        MODELS[model or MODEL]["artifact"])


def cfg_value(key):
    """Read a `key:` value from AFC/AFC_BridgeBox.cfg (serial_port, tcp_key).

    INLINE COMMENTS ARE NOT PART OF THE VALUE. This returned everything after
    the colon, so a perfectly ordinary config line

        serial_port: tcp://192.168.6.3:8888   # Pico 2 W (WiFi)

    yielded the note as well, and step 5 died parsing the port out of
    '8888   # Pico 2 W (WiFi)'. It crashed safely -- before cmd1, so nothing
    was erased -- but it crashed on the FIRST line of an armed flash, after
    every read-only gate had passed, because those gates never open the link.

    The identical bug was fixed in standalone/ams_flash.py earlier the same
    day (find_in_cfg); this is the same stripper. serial_port and tcp_key
    never contain whitespace, so taking the first token after dropping a
    Klipper comment (# or ;) is exact, not a heuristic.
    """
    path = os.path.join(CFG_DIR, "AFC", "AFC_BridgeBox.cfg")
    try:
        for ln in open(path, errors="replace"):
            s = ln.strip()
            if s.startswith(key + ":"):
                v = s.split(":", 1)[1]
                v = v.split("#", 1)[0].split(";", 1)[0].strip()
                v = v.split()[0] if v.split() else ""
                if v:
                    return v
    except Exception:
        pass
    return None


# ── tiny Moonraker client (localhost, trusted; falls back to an API key) ──────
class Moon:
    def __init__(self, base="http://127.0.0.1:7125", key=None):
        self.base, self.key = base, key

    def _req(self, method, path, timeout=30):
        h = {"X-Api-Key": self.key} if self.key else {}
        r = urllib.request.Request(self.base + path, headers=h, method=method)
        return json.loads(urllib.request.urlopen(r, timeout=timeout).read())

    def q(self, obj):
        import urllib.parse
        p = "/printer/objects/query?" + urllib.parse.quote(obj)
        return self._req("GET", p)["result"]["status"].get(obj, {})

    def gcode(self, script, timeout=120):
        import urllib.parse
        return self._req("POST", "/printer/gcode/script?script="
                         + urllib.parse.quote(script), timeout=timeout)

    def store(self, n=20):
        try:
            r = self._req("GET", "/server/gcode_store?count=%d" % n)
            return [x["message"] for x in r["result"]["gcode_store"]]
        except Exception:
            return []

    def state(self):
        try:
            return self.q("webhooks").get("state", "?")
        except Exception:
            return "?"


def klipper(action):
    """stop|start klipper via systemctl (sudo -n fallback), like update_bridge.sh."""
    for cmd in (["systemctl", action, "klipper"],
                ["sudo", "-n", "systemctl", action, "klipper"]):
        try:
            if subprocess.run(cmd, capture_output=True).returncode == 0:
                return True
        except Exception:
            pass
    return False


def wait_state(moon, want="ready", timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if moon.state() == want:
            return True
        time.sleep(3)
    return moon.state() == want


def fw_int(s):
    # "AFC-1.68" -> 168 ; "AFC-1.67" -> 167
    import re
    m = re.search(r"(\d+)\.(\d+)", s or "")
    return int(m.group(1)) * 100 + int(m.group(2)) if m else -1


def step(n, msg):
    print(f"\n[{n}] {msg}", flush=True)


# ── the phases ───────────────────────────────────────────────────────────────
def check_not_printing(moon) -> bool:
    st = None
    try:
        st = moon.q("print_stats").get("state")
    except Exception as e:
        print(f"    could not read print_stats ({e}); refusing to assume idle")
        return False
    print(f"    print_stats.state = {st}")
    return st not in ("printing", "paused")


def check_artifact() -> bool:
    ap_ = artifact_path()
    if not os.path.exists(ap_):
        print(f"    MISSING artifact: {ap_}")
        return False
    try:
        art = F.load_artifact(ap_)               # re-verifies CRCs, refuses bad
        total, summed = F.artifact_image_bytes(art)
        print(f"    artifact OK: header + {len(art['blocks'])} blocks, "
              f"CRCs verified")
        print(f"    image: header declares {total} bytes, blocks carry "
              f"{summed} + {total - summed} container header")
        if total - summed != F.BIMH_HEADER_LEN:
            print(f"    artifact INCOMPLETE: blocks do not add up to the "
                  f"declared image ({total} - {summed} != "
                  f"{F.BIMH_HEADER_LEN})")
            return False
        return True
    except Exception as e:
        print(f"    artifact FAILED verification: {e}")
        return False


def bridge_fw(moon, unit) -> int:
    return fw_int(moon.q(f"AFC_BambuAMS {unit}").get("bridge_fw", ""))


def online_units(moon, units) -> list:
    on = []
    for u in units:
        try:
            if moon.q(f"AFC_BambuAMS {u}").get("bridge_online"):
                on.append(u)
        except Exception:
            pass
    return on


def ota_bridge(moon, unit) -> bool:
    """OTA a pre-staged signed 1.68 image to the bridge. Returns True on success.

    The build is a one-time-per-version step and is NOT run from here (it would
    nest update_bridge.sh and can take minutes); if no image is staged this
    returns False with the one command to run once.
    """
    img = os.path.join(CFG_DIR, ENC_IMAGE)
    if not os.path.exists(img):
        print(f"    no staged bridge image at {img}")
        print(f"    build it once, then re-run:  RUN_SHELL_COMMAND "
              f"CMD=bridge_update PARAMS=\"--encpackage {TARGET_FW} w\"")
        return False
    print("    using staged " + ENC_IMAGE)
    step("3b", "OTA dry-run (APPLY=0): transfer + verify, write nothing")
    moon.gcode(f"AFC_BAMBU_FLASH UNIT={unit} FILE={ENC_IMAGE} APPLY=0")
    time.sleep(2)
    tail = " | ".join(moon.store(12))
    if "accepted" not in tail:
        print("    dry-run did not report the image accepted:\n    " + tail[-400:])
        return False
    print("    dry-run accepted")
    step("3c", "OTA APPLY=1: the bridge writes its own flash and reboots")
    try:
        moon.gcode(f"AFC_BAMBU_FLASH UNIT={unit} FILE={ENC_IMAGE} APPLY=1", timeout=60)
    except Exception:
        pass                                     # link drops as it reboots -- expected
    time.sleep(20)
    wait_state(moon, "ready", 90)
    return True


def probe_loader(target, key) -> bool:
    """Non-destructive: send cmd1, confirm the loader announces. klipper must be stopped."""
    link = Link(target)
    try:
        if not authenticate(link, key):
            print("    link auth FAILED")
            return False
        link.send({"cmd": "txecho", "on": 1})
        t0 = time.time()
        for line in link.lines():
            if line and '"txecho"' in line and 'true' in line:
                break
            if time.time() - t0 > 4:
                print("    txecho not confirmed")
                return False
        # ADDRESSED FROM THE ARTIFACT, not from a constant. F.LOADER_TARGET
        # was 0x0700 -- right for an AMS 2, silently wrong for an HT (0x1800,
        # ams id 0x80), and this is the gate that proves a unit is in its
        # loader before the erase. Reading it off the image guarantees the
        # poke and the bytes that follow name the same unit.
        _tgt, _aid = F.loader_target_of(F.load_artifact(artifact_path()))
        print(f"    cmd1 -> device 0x{_tgt:04X}, ams id 0x{_aid:02X}")
        cmd1 = build_cmd1(_tgt, 0x0077, _aid)
        link.send({"cmd": "raw", "hex": cmd1.hex(), "us": 600000})
        seen = []
        deadline = time.time() + 4
        for line in link.lines():
            if line:
                import re
                if '"evt":"raw"' in line or '"evt":"tx"' in line:
                    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line) \
                        or re.search(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"', line)
                    if m:
                        seen += loader_hits(m.group(1))
            if time.time() > deadline:
                break
        link.send({"cmd": "txecho", "on": 0})
        if seen:
            print(f"    LOADER ENTERED: {','.join(sorted(set(seen)))}")
            return True
        print("    NO JUMP -- the unit did not enter its loader")
        return False
    finally:
        link.close()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="unit label/serial (confirmation)")
    ap.add_argument("--go", action="store_true",
                    help="perform the update; without it, PREFLIGHT only")
    ap.add_argument("--check", action="store_true",
                    help="read-only: run steps 1-4 (no klipper stop, no probe)")
    ap.add_argument("--model", choices=sorted(MODELS), default="ams2",
                    help="which AMS model is being updated -- picks the "
                         "per-model artifact AND the unit names to count "
                         "(default ams2)")
    ap.add_argument("--unit", default=None,
                    help="an AFC_BambuAMS unit on this bridge "
                         "(default: the model's first)")
    ap.add_argument("--units", default=None,
                    help="candidate units to count as online "
                         "(default: the model's own)")
    ap.add_argument("--target", default=None, help="bridge tcp:// (default: from config)")
    ap.add_argument("--cfg-dir", default=None,
                    help="printer config dir (default ~/printer_data/config; "
                    "pass the real one when run as root)")
    ap.add_argument("--moonraker", default="http://127.0.0.1:7125")
    a = ap.parse_args()

    global CFG_DIR, MODEL
    if a.cfg_dir:
        CFG_DIR = a.cfg_dir
    MODEL = a.model
    # Resolved AFTER --model, so the unit names follow the model unless the
    # operator named them. A boxed default on an HT run would count zero units
    # online and stop at step 4 -- safe, but for the wrong reason.
    if not a.units:
        a.units = ",".join(MODELS[MODEL]["units"])
    if not a.unit:
        a.unit = MODELS[MODEL]["units"][0]
    print(f"    model: {MODEL}  artifact: {os.path.basename(artifact_path())}")

    moon = Moon(a.moonraker)
    key = cfg_value("tcp_key") or ""            # from config, root-safe
    units = [u.strip() for u in a.units.split(",") if u.strip()]

    # target address of the bridge: prefer the arg, else the config serial_port
    target = a.target or cfg_value("serial_port")
    if not target or not target.startswith("tcp://"):
        print(f"no tcp:// bridge target (got {target!r}); pass --target")
        return 2
    # PARSE IT NOW, not when the link opens.
    #
    # The startswith() above is all this used to do, so a target that merely
    # LOOKED like a URL sailed through every read-only gate -- none of steps
    # 1-4 opens the link -- and blew up on the first line of an armed run, at
    # step 5, with a ValueError from deep inside socket code. It failed safe
    # (before cmd1, nothing erased) but it failed at the worst moment it could
    # still fail at, and `check` had reported everything fine.
    #
    # A preflight whose job is to find problems before the erase must exercise
    # this, so the same parse the Link will do happens here, where its failure
    # is a sentence instead of a traceback.
    try:
        _hostport = target[len("tcp://"):]
        _h, _, _p = _hostport.partition(":")
        if not _h or (_p and not (0 < int(_p) < 65536)):
            raise ValueError(f"bad host/port in {target!r}")
    except ValueError as e:
        print(f"    unusable bridge target {target!r}: {e}")
        print("    (an inline `# comment` after serial_port: is a common "
              "cause -- cfg_value strips those)")
        return 2

    print(f"=== AMS update: label={a.label}  bridge={target}  "
          f"mode={'GO (will erase)' if a.go else 'PREFLIGHT (no changes)'} ===")

    # 1. not printing
    step(1, "printer not printing")
    if not check_not_printing(moon):
        print("    ABORT: a print is active (or state unknown).")
        return 1

    # 2. artifact
    step(2, "per-model artifact verifies")
    if not check_artifact():
        print("    ABORT: artifact missing or failed verification.")
        return 1

    # 3. bridge fw >= 1.68
    step(3, f"bridge firmware >= {TARGET_FW}")
    fw = bridge_fw(moon, a.unit)
    print(f"    bridge_fw = {fw if fw > 0 else 'unknown'}")
    if fw < TARGET_FW:
        if not a.go:
            print(f"    NO-GO: bridge is < {TARGET_FW}. Re-run with --go to OTA it "
                  f"first, or do the one-time OTA per the runbook.")
        else:
            if not ota_bridge(moon, a.unit):
                print("    ABORT: bridge OTA failed; nothing was done to any AMS.")
                return 1
            fw = bridge_fw(moon, a.unit)
            print(f"    bridge_fw now = {fw}")
            if fw < TARGET_FW:
                print("    ABORT: bridge did not come back on >= 1.68.")
                return 1

    # 4. exactly one online unit
    step(4, "exactly one AMS online on the bridge")
    on = online_units(moon, units)
    print(f"    online units: {on or '(none)'}")
    if len(on) != 1:
        print(f"    ABORT: need exactly one AMS on the bus for the flash "
              f"(found {len(on)}). Leave only the target unit powered.")
        return 1

    if a.check:
        print("\n==> READ-ONLY CHECKS PASSED (steps 1-4). klipper untouched, "
              "no probe, no erase.")
        return 0

    # klipper down for the socket-owning steps (probe + flash)
    step(5, "cmd1 enters the loader (non-destructive probe)")
    if not klipper("stop"):
        print("    ABORT: could not stop klipper to free the board socket.")
        return 1
    time.sleep(3)
    try:
        if not probe_loader(target, key):
            print("    NO-GO: the unit did not enter its loader on cmd1.")
            return 1
        if not a.go:
            print("\n==> PREFLIGHT PASSED. Everything up to the erase is green.")
            print("    Nothing on the AMS was changed. Re-run with --go to flash.")
            return 0
        # The COUNT COMES FROM THE ARTIFACT. A literal here announced "168
        # blocks" over an HT flash that then streamed 157 -- harmless, and
        # exactly the kind of stale reassurance that makes a log untrustworthy
        # at the one moment someone is reading it closely.
        _n = len(F.load_artifact(artifact_path())["blocks"])
        step(6, f"FLASH: header (erase) + {_n} blocks")
        rc = F.flash(artifact_path(), target, key, a.label)
        if rc != 0:
            print("    FLASH did not complete. The loader is intact; re-run "
                  "with --go to erase+rewrite from scratch.")
            return 1
    finally:
        klipper("start")
        wait_state(moon, "ready", 90)

    # 7. verify online
    step(7, "unit reboots and comes back online")
    t0 = time.time()
    while time.time() - t0 < 60:
        if a.unit in online_units(moon, [a.unit]):
            print(f"    {a.unit} is ONLINE on the new firmware.")
            print("\n==> DONE. Start the dryer and load a tray to confirm "
                  "load-while-drying.")
            return 0
        time.sleep(5)
    print("    unit not seen online within 60s -- check AFC_BAMBU_UIDS / status.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
