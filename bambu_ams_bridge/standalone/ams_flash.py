#!/usr/bin/env python3
"""
Self-contained AMS firmware updater for the Bambu-AMS bridge.

Drop this file and `ams_artifact.json` into your Klipper config directory, add
`ams_flash.cfg` (the gcode_shell_command + macro), and run from the console:

    AFC_BAMBU_AMS_UPDATE LABEL=<serial>              # PREFLIGHT (probes, no erase)
    AFC_BAMBU_AMS_UPDATE LABEL=<serial> MODE=check   # read-only, changes nothing
    AFC_BAMBU_AMS_UPDATE LABEL=<serial> MODE=go      # actually update (erases)

Nothing here imports from the bridge repo -- only the Python standard library --
so the two files above (plus the artifact) are all a machine needs.

WHAT IT DOES. Sends the AMS a cmd1 frame, which makes the running unit jump into
its bootloader, then streams the firmware (one header + N data blocks); the
unit erases its app flash, writes the new image, reboots and rejoins the bus.
The bootloader is never erased, so an interrupted transfer leaves a RECOVERABLE
unit in its loader -- just run MODE=go again (idempotent).

SAFETY GATES, all of which must pass before the erase:
    1 not printing   2 artifact verifies   3 bridge fw >= 1.68
    4 exactly one AMS online   5 cmd1 enters the loader (non-destructive)
Only MODE=go, and only past 1-5, streams the erase+image.

Requirements: the bridge must already run fw >= 1.68 (the transmit path). If it
is older this refuses with instructions -- updating the bridge is a separate,
one-time step.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
import re
import socket
import sys
import time
import urllib.request

# ── constants (all confirmed against real printer<->AMS captures) ─────────────
TARGET_FW = 168                 # bridge fw with fwreplay/txbuf/txsend
LOADER_TARGET = 0x0700          # AMS update target address
CONFIRM = "ERASE-AMS-FW"        # the firmware's fwreplay confirm token
CHUNK = 100                     # hex-payload bytes per txbuf line
CRC8_POLY, CRC8_INIT = 0x39, 0x66
CRC16_POLY, CRC16_INIT = 0x1021, 0x913D
MOON = "http://127.0.0.1:7125"
# The loader's OWN verdict, read from the block-send reply windows (fw >= 1.74).
# A block ack is only receipt; success! is the whole-image hash passing and the
# loader resetting into the app -- the only trustworthy "it took and rebooted".
SUCCESS_HEX = "7375636365737321"       # "success!"
CHUNKERR_HEX = "6368756e6b2068617368"  # "chunk hash" (per-chunk hash failure)
FLASH_RETRIES = 6               # whole-flash resends after the first attempt (7 total)


# ── CRCs ──────────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    c = CRC8_INIT
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ CRC8_POLY) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def crc16(data: bytes) -> int:
    c = CRC16_INIT
    for b in data:
        c = (c ^ (b << 8)) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ CRC16_POLY) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c


def build_cmd1(target: int = LOADER_TARGET, counter: int = 0x0077) -> bytes:
    """The enter-loader / loader-handshake frame: op 0601, cmd 01, source 0x0900."""
    f = bytearray(49)
    f[0] = 0x3D; f[1] = 0x04
    f[2] = counter & 0xFF; f[3] = (counter >> 8) & 0xFF
    f[4] = 0x31; f[5] = 0x00
    f[6] = crc8(bytes(f[:6]))
    f[7] = target & 0xFF; f[8] = (target >> 8) & 0xFF
    f[9] = 0x00; f[10] = 0x09
    f[11] = 0x06; f[12] = 0x01; f[13] = 0x01; f[14] = 0x07
    c = crc16(bytes(f[:47]))
    f[47] = c & 0xFF; f[48] = (c >> 8) & 0xFF
    assert f[11] == 0x06 and f[12] == 0x01 and f[13] == 0x01, "not a cmd1 frame"
    return bytes(f)


def wrap(cls: int, counter: int, body: bytes) -> bytes:
    """Rebuild a full frame from its invariant body with a fresh counter+CRCs."""
    fr = bytearray(7 + len(body) + 2)
    fr[0] = 0x3D
    fr[1] = cls
    fr[2] = counter & 0xFF; fr[3] = (counter >> 8) & 0xFF
    n = len(fr)                                     # [4:6] = total frame length
    fr[4] = n & 0xFF; fr[5] = (n >> 8) & 0xFF
    fr[6] = crc8(bytes(fr[:6]))
    fr[7:7 + len(body)] = body
    c = crc16(bytes(fr[:-2]))
    fr[-2] = c & 0xFF; fr[-1] = (c >> 8) & 0xFF
    return bytes(fr)


def load_artifact(path: str):
    """Read + verify the per-model artifact (header + N block bodies)."""
    d = json.load(open(path))
    art = {"header": bytes.fromhex(d["header"]),
           "blocks": [bytes.fromhex(b) for b in d["blocks"]]}
    steps = list(plan(art))
    bad = [s for _k, s, fr in steps
           if crc8(fr[:6]) != fr[6] or crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8))]
    if bad or len(steps) != 1 + len(art["blocks"]):
        raise ValueError("artifact failed re-verification")
    return art


#: Bytes of BIMH container header counted in the update's declared total but
#: NOT carried in the data blocks' own firmware-byte counts. Measured as
#: exactly 416 on both models (AMS 2: 172132 - 171716; AMS HT: 160552 -
#: 160136).
BIMH_HEADER_LEN = 416


def artifact_image_bytes(art):
    """(declared total image bytes, bytes actually carried by the blocks).

    Bodies are captured frame[7:-2], hence the -7 on both offsets: the header
    frame's total image size lives at frame [23:26] and each data block's own
    firmware-byte count at frame [35:39].
    """
    hdr = art["header"]
    total = int.from_bytes(hdr[23 - 7:26 - 7], "little")
    summed = sum(int.from_bytes(b[35 - 7:39 - 7], "little")
                 for b in art["blocks"])
    return total, summed


def plan(artifact, counter0: int = 0):
    c = counter0
    yield ("header", -1, wrap(0x00, c, artifact["header"])); c += 1
    for s, body in enumerate(artifact["blocks"]):
        yield ("data", s, wrap(0x00, c, body)); c += 1


# ── bridge link (tcp:// or serial) + link-key handshake ───────────────────────
class Link:
    def __init__(self, target: str, timeout: float = 1.0):
        self.tcp = target.startswith("tcp://")
        self.buf = b""
        if self.tcp:
            host, _, port = target[6:].partition(":")
            self.sock = socket.create_connection((host, int(port or 8888)), timeout=5)
            self.sock.settimeout(timeout)
        else:
            import serial
            self.ser = serial.Serial(target, 115200, timeout=timeout)

    def send(self, obj: dict):
        b = (json.dumps(obj) + "\n").encode()
        self.sock.sendall(b) if self.tcp else self.ser.write(b)

    def lines(self):
        while True:
            try:
                chunk = self.sock.recv(4096) if self.tcp else self.ser.read(4096)
            except socket.timeout:
                yield None; continue
            if self.tcp and not chunk:
                return
            if not chunk:
                yield None; continue
            self.buf += chunk
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                yield line.decode("utf-8", "replace").rstrip("\r")

    def close(self):
        try:
            (self.sock if self.tcp else self.ser).close()
        except Exception:
            pass


def authenticate(link: Link, key: str, timeout: float = 8.0) -> bool:
    """board -> {"nonce"}; host -> HMAC-SHA512(key,nonce)[:32]; board -> {"ok":1}."""
    sock = getattr(link, "sock", None)
    if sock is None:
        return not key
    deadline = time.time() + max(timeout, 2.0)
    sock.settimeout(0.2)
    buf = b""
    while time.time() < deadline and b"\n" not in buf:
        try:
            c = sock.recv(256)
        except (socket.timeout, TimeoutError):
            if not key:
                return True
            continue
        if not c:
            return False
        buf += c
    if b"\n" not in buf:
        return not key
    line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
    if '"nonce"' not in line:
        link.buf = buf
        return True
    if not key:
        return False
    nonce = bytes.fromhex(json.loads(line)["nonce"])
    mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
    sock.sendall(f'{{"cmd":"auth","mac":"{mac.hex()}"}}\n'.encode())
    reply = buf.split(b"\n", 1)[1]
    while time.time() < deadline and b"\n" not in reply:
        try:
            c = sock.recv(256)
        except (socket.timeout, TimeoutError):
            continue
        if not c:
            return False
        reply += c
    rl = reply.split(b"\n", 1)[0].decode("utf-8", "replace")
    link.buf = reply.split(b"\n", 1)[1] if b"\n" in reply else b""
    sock.settimeout(1.0)
    return '"ok":1' in rl.replace(" ", "")


def _await(link: Link, evt: str, timeout: float = 8.0):
    t0 = time.time()
    for line in link.lines():
        if line and f'"{evt}"' in line:
            return line
        if time.time() - t0 > timeout:
            return None


def _rx_of(line: str) -> bytes:
    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line or "")
    return bytes.fromhex(m.group(1)) if m else b""


def _seqretry(rx: bytes):
    """Parse the loader's '[MCU_UP] N seq_num: X, retry: M' out of a reply.

    Returns (want, retry): `want` is the NEXT seq the loader expects (hex on the
    wire -> int), `retry` its attempt counter. After block N is accepted the
    loader answers want == N+1; if it does not advance it is asking for that
    chunk again. None when the reply carries no such narration (e.g. the
    completion frame, which instead carries success! / chunk hash)."""
    a = "".join(chr(b) if 32 <= b < 127 else " " for b in rx)
    m = re.findall(r"seq_num:\s*([0-9a-fA-F]+),\s*retry:\s*(\d+)", a)
    if not m:
        return None
    want, rt = m[-1]
    try:
        return int(want, 16), int(rt)
    except ValueError:
        return None


def send_frame(link: Link, frame: bytes, window_us: int = 8000) -> bytes:
    for off in range(0, len(frame), CHUNK):
        link.send({"cmd": "txbuf", "off": off, "hex": frame[off:off + CHUNK].hex()})
        if not _await(link, "txbuf", 4.0):
            raise IOError(f"txbuf not acked at off {off}")
    link.send({"cmd": "txsend", "n": len(frame), "us": window_us})
    line = _await(link, "txsend", timeout=max(2.0, window_us / 1e6 + 2))
    if line is None:
        raise IOError("txsend not acked")
    return _rx_of(line)


SIGS = {"[MCU_UP]": "5b4d43555f55505d",
        "Loader Version": "4c6f61646572205665727369",
        "wait cmd1": "7761697420636d6431"}


def loader_hits(hexstr: str):
    h = hexstr.lower()
    out = [n for n, p in SIGS.items() if p in h]
    if "0601" in h and h.startswith("3d04"):
        out.append("class04/op0601")
    return out


def enter_loader(link: Link, tries: int = 6) -> bool:
    """cmd1 until the loader announces. cmd1 only -- can never erase. No answer -> False.

    Uses txbuf/txsend, so it is only valid INSIDE fwreplay mode (do_flash)."""
    cmd1 = build_cmd1()
    for i in range(tries):
        rx = send_frame(link, cmd1, window_us=600000)
        if rx and loader_hits(rx.hex()):
            print(f"    loader confirmed after cmd1 #{i + 1}")
            return True
    return False


def probe_cmd1(link: Link, tries: int = 4) -> bool:
    """Non-destructive loader probe, OUTSIDE fwreplay: send cmd1 via `raw`.

    `raw` sends one frame and captures the bus for the window (no fwreplay
    needed), so this works on a running unit without arming the erase path.
    cmd1 only; it can never carry a header."""
    cmd1 = build_cmd1()
    for i in range(tries):
        link.send({"cmd": "raw", "hex": cmd1.hex(), "us": 600000})
        line = _await(link, "raw", timeout=3.0)
        rx = _rx_of(line)
        if rx and loader_hits(rx.hex()):
            print(f"    loader confirmed after cmd1 #{i + 1}")
            return True
    return False


def _one_pass(link: Link, art, steps) -> str:
    """One erase+rewrite pass, LOADER-DRIVEN: send the block the loader's reply
    asks for (seq_num), resending in place any chunk it will not accept, so a
    glitched chunk is fixed within the pass instead of failing the whole image.
    Returns the loader's OWN verdict, read from the block windows:
      'success'  -- loader hashed the whole image OK; it reset into the app
      'badimage' -- chunk hash error, a stuck chunk, or no success! after all blocks
      'stall'    -- no loader answer / a block went unacked
    'badimage' and 'stall' both leave the unit IN the loader, so the caller may
    safely resend the whole pass. No probe after the verdict, so a good flash is
    never mistaken for a failure and re-erased."""
    link.send({"cmd": "fwreplay", "on": 1, "confirm": CONFIRM})
    if not _await(link, "fwreplay", 4.0):
        print("    bridge did not enter fwreplay mode (fw < 1.68?)", file=sys.stderr)
        return "stall"
    try:
        # cmd1 handshake FIRST -- the loader ignores a header until it handshakes;
        # a header before this strands a half-written image. No answer -> no erase.
        if not enter_loader(link):
            print("    no loader response to cmd1 -- NOTHING erased", file=sys.stderr)
            return "stall"
        blocks = art["blocks"]
        nblk = len(blocks)
        header = next(f for k, s, f in steps if k == "header")
        bframe = {s: f for k, s, f in steps if k == "data"}

        rx = send_frame(link, header, window_us=3_500_000)
        if not rx:
            print("    header NO ACK -- loader idle; unit left for a re-erase",
                  file=sys.stderr)
            return "stall"
        print("    header sent, erase triggered; streaming blocks...")

        # LOADER-DRIVEN transfer: each reply names the next seq the loader wants.
        # want == last+1 means accepted; a non-advancing want means resend that
        # chunk -- following it fixes a corrupted chunk WITHIN the pass (the way
        # the printer does) instead of failing the whole image. A corrupt
        # header/handshake still dooms the pass; that falls to the pass-level
        # retry in do_flash, which re-does the handshake.
        want = 0
        resends = {}
        total = 0
        last_rx = rx
        MAX_PER = 8
        MAX_TOTAL = nblk * 6
        while want < nblk:
            frame = bframe.get(want)
            if frame is None:
                print(f"    loader wants seq {want} (outside 0..{nblk-1}) -- "
                      f"resending the whole pass", file=sys.stderr)
                return "badimage"
            win = 3_000_000 if want == nblk - 1 else 400_000
            rx = send_frame(link, frame, window_us=win)
            if not rx:
                print(f"    data {want}: NO ACK -- stopping (unit left in loader)",
                      file=sys.stderr)
                return "stall"
            last_rx = rx
            total += 1
            if total > MAX_TOTAL:
                print("    too many block sends this pass -- resending the whole "
                      "pass", file=sys.stderr)
                return "badimage"
            rxl = rx.hex().lower()
            if SUCCESS_HEX in rxl:
                print("    loader: success! -- image verified, resetting into the app")
                return "success"
            if CHUNKERR_HEX in rxl:
                print("    loader: chunk hash error -- resending the whole pass",
                      file=sys.stderr)
                return "badimage"
            sr = _seqretry(rx)
            if sr is None:
                want += 1                  # no seq narration: treat as accepted
                continue
            nxt, rt = sr
            if nxt > want:
                want = nxt                 # accepted -- follow the loader forward
            else:
                resends[nxt] = resends.get(nxt, 0) + 1
                print(f"    loader wants seq {nxt} resent "
                      f"(#{resends[nxt]}, retry {rt})", file=sys.stderr)
                if resends[nxt] > MAX_PER:
                    print(f"    seq {nxt} will not take after {MAX_PER} resends -- "
                          f"resending the whole pass", file=sys.stderr)
                    return "badimage"
                want = nxt
            if want and want % 24 == 0:
                print(f"    at block {want}/{nblk - 1}")

        # All blocks accepted; the verdict is in the final reply's window.
        rxhex = last_rx.hex().lower()
        if SUCCESS_HEX in rxhex:
            print("    loader: success! -- image verified, resetting into the app")
            return "success"
        if CHUNKERR_HEX in rxhex:
            print("    loader: chunk hash error -- resending the whole pass",
                  file=sys.stderr)
            return "badimage"
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in last_rx)
        print(f"    loader: no success! after all blocks ({len(last_rx)} B) -- "
              f"{ascii_[-120:]}", file=sys.stderr)
        return "badimage"
    finally:
        link.send({"cmd": "fwreplay", "on": 0})
        _await(link, "fwreplay", 4.0)


def do_flash(link: Link, art, retries: int = FLASH_RETRIES) -> int:
    """Erase + rewrite the AMS, retrying the WHOLE flash until the loader itself
    reports success!. The transfer intermittently corrupts a chunk; the loader
    catches it ('chunk hash error') and refuses to boot, so a single pass is a
    coin flip. Every non-success pass leaves the unit safely in its loader; a
    success! pass means it already reset into the app, so the loop returns and
    never re-erases a live unit. Returns 0 only on a loader-verified success."""
    steps = list(plan(art))
    print(f"    up to {retries + 1} attempt(s); only a loader 'success!' ends it.")
    for attempt in range(1, retries + 2):
        print(f"    == flash attempt {attempt}/{retries + 1} ==")
        try:
            verdict = _one_pass(link, art, steps)
        except IOError as e:
            # A staging/link stall (e.g. USB-CDC back-pressure) aborts this pass
            # but leaves the unit in the loader -- safe to re-erase.
            print(f"    attempt {attempt}: link stall -- {e}", file=sys.stderr)
            verdict = "stall"
        if verdict == "success":
            print("    FLASH CONFIRMED: the loader verified the image and reset "
                  "into the new firmware.")
            return 0
        if attempt <= retries:
            print(f"    attempt {attempt}: {verdict} -- resending the full flash",
                  file=sys.stderr)
    print("    out of attempts: the loader never verified success!. The unit is "
          "left in its loader (recoverable) -- re-run MODE=go. If every pass acks "
          "but reports chunk hash error, the bus/link is dropping bytes.",
          file=sys.stderr)
    return 1


# ── Moonraker (localhost) for print-state, klipper control, verify ────────────
class Moon:
    def __init__(self, base=MOON):
        self.base = base

    def _req(self, method, path, timeout=30):
        r = urllib.request.Request(self.base + path, method=method)
        return json.loads(urllib.request.urlopen(r, timeout=timeout).read())

    def q(self, obj):
        import urllib.parse
        p = "/printer/objects/query?" + urllib.parse.quote(obj)
        return self._req("GET", p)["result"]["status"].get(obj, {})

    def state(self):
        try:
            return self.q("webhooks").get("state", "?")
        except Exception:
            return "?"

    def service(self, action, name="klipper"):
        try:
            self._req("POST", f"/machine/services/{action}?service={name}", timeout=20)
            return True
        except Exception:
            return False


def klipper(moon, action):
    if moon.service(action):
        return True
    import subprocess
    for cmd in (["systemctl", action, "klipper"],
                ["sudo", "-n", "systemctl", action, "klipper"]):
        try:
            if subprocess.run(cmd, capture_output=True).returncode == 0:
                return True
        except Exception:
            pass
    return False


def relaunch_detached(pass_file: str) -> tuple:
    """Re-run this script in its OWN transient systemd unit so it survives the
    klipper bounce -- a child of the klipper service is killed when klipper
    stops, which mid-flash would strand a half-written AMS.

    Three ways, tried in order, so no manual setup is needed on a normal Klipper
    Pi:
      1. `systemd-run --user` -- NO sudo, NO password. Works wherever the user
         has a systemd manager, which any Pi already running a --user service
         (e.g. the USB QR scanner) does. This is the default path.
      2. `sudo -n systemd-run` -- a passwordless (NOPASSWD) sudoers grant, if one
         happens to be present.
      3. `sudo -S systemd-run` reading the password from `pass_file` (used once,
         then DELETED). Last resort, only if 1 and 2 both fail.

    Returns (rc, output); rc == 0 means the detached unit started.
    """
    import subprocess
    args = [x for x in sys.argv[1:] if x != "--detached"] + ["--detached"]
    unit = ["systemd-run", "--collect", "/usr/bin/python3",
            os.path.abspath(__file__)] + args

    # 1) user manager -- needs XDG_RUNTIME_DIR pointing at the user's runtime dir
    #    (not always set in the gcode_shell_command environment).
    env = dict(os.environ)
    env.setdefault("XDG_RUNTIME_DIR", "/run/user/%d" % os.getuid())
    r1 = subprocess.run(["systemd-run", "--user", "--collect", "/usr/bin/python3",
                         os.path.abspath(__file__)] + args,
                        capture_output=True, env=env)
    if r1.returncode == 0:
        return 0, "detached via systemd-run --user\n" + \
                  (r1.stdout + r1.stderr).decode("utf-8", "replace")

    # 2) passwordless sudo grant, if present.
    r2 = subprocess.run(["sudo", "-n"] + unit, capture_output=True)
    if r2.returncode == 0:
        return 0, "detached via sudo -n\n" + \
                  (r2.stdout + r2.stderr).decode("utf-8", "replace")

    # 3) password file (used once, then deleted so the secret never lingers in
    #    the Moonraker-served config dir).
    pf = os.path.expanduser(pass_file)
    if not os.path.exists(pf):
        u1 = (r1.stderr or r1.stdout).decode("utf-8", "replace").strip()
        u2 = (r2.stderr or r2.stdout).decode("utf-8", "replace").strip()
        return 2, ("could not detach without root:\n"
                   f"  systemd-run --user failed: {u1}\n"
                   f"  sudo -n systemd-run failed: {u2}\n"
                   f"  last resort -- create a sudo password file (chmod 600):\n"
                   f"    printf '%s' 'YOURPASSWORD' > {pf}")
    pw = open(pf).read().rstrip("\n") + "\n"
    r3 = subprocess.run(["sudo", "-S", "-p", ""] + unit,
                        input=pw.encode(), capture_output=True)
    try:
        os.remove(pf)
    except Exception:
        pass
    return r3.returncode, (r3.stdout + r3.stderr).decode("utf-8", "replace")


def wait_ready(moon, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if moon.state() == "ready":
            return True
        time.sleep(3)
    return moon.state() == "ready"


# ── config auto-detection (self-locating from this file's directory) ──────────
def config_root(start: str) -> str:
    """Walk up from `start` to the dir holding printer.cfg (the config root)."""
    d = os.path.abspath(start)
    for _ in range(6):
        if os.path.exists(os.path.join(d, "printer.cfg")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return start


def find_in_cfg(cfg_dir: str, key: str):
    for root, _dirs, files in os.walk(cfg_dir):
        for fn in files:
            if not fn.endswith(".cfg"):
                continue
            try:
                for ln in open(os.path.join(root, fn), errors="replace"):
                    s = ln.strip()
                    if s.startswith(key + ":"):
                        v = s.split(":", 1)[1]
                        # Drop a Klipper inline comment (# or ;) and take the
                        # first token -- serial_port/tcp_key never contain spaces,
                        # and "tcp://host:8888   # note" must not carry the note
                        # into port parsing.
                        v = v.split("#", 1)[0].split(";", 1)[0].strip()
                        v = v.split()[0] if v.split() else ""
                        if v:
                            return v
            except Exception:
                pass
    return None


def online_units(moon, units):
    on = []
    for u in units:
        try:
            if moon.q(f"AFC_BambuAMS {u}").get("bridge_online"):
                on.append(u)
        except Exception:
            pass
    return on


def bridge_fw(moon, units):
    for u in units:
        v = moon.q(f"AFC_BambuAMS {u}").get("bridge_fw")
        if v:
            m = re.search(r"(\d+)\.(\d+)", v)
            if m:
                return int(m.group(1)) * 100 + int(m.group(2))
    return -1


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="",
                    help="optional log label for the unit (e.g. its sticker serial); "
                    "not validated -- defaults to the online unit name")
    ap.add_argument("--mode", default="preflight", choices=["check", "preflight", "go"])
    ap.add_argument("--cfg-dir", default=None,
                    help="Klipper config root (default: auto-detected from this "
                    "file's location)")
    ap.add_argument("--target", default=None, help="bridge tcp:// (default: from config)")
    ap.add_argument("--key", default=None, help="link key (default: tcp_key from config)")
    ap.add_argument("--artifact", default=None,
                    help="firmware artifact json (default: ams_artifact.json beside this file)")
    ap.add_argument("--units", default="Bambu_AMS_1,Bambu_AMS_2,Bambu_AMS_3,Bambu_AMS_4")
    ap.add_argument("--sudo-pass-file", default=os.path.join(here, "ams_sudo.pw"),
                    help="file holding the sudo password, used once to relaunch "
                    "detached and then DELETED automatically (default: ams_sudo.pw "
                    "beside this file)")
    ap.add_argument("--detached", action="store_true",
                    help=argparse.SUPPRESS)   # internal: set on the relaunched copy
    a = ap.parse_args()
    if not a.cfg_dir:
        a.cfg_dir = config_root(here)

    # Tee all output to printer_data/logs/ams_flash.log -- when launched detached
    # (systemd-run) there is no console, and this log is Moonraker-served.
    try:
        logdir = os.path.join(os.path.dirname(a.cfg_dir.rstrip("/")), "logs")
        os.makedirs(logdir, exist_ok=True)
        lf = open(os.path.join(logdir, "ams_flash.log"), "a", buffering=1)

        class _Tee:
            def __init__(self, *s): self.s = s
            def write(self, x):
                for f in self.s:
                    try: f.write(x); f.flush()
                    except Exception: pass
            def flush(self):
                for f in self.s:
                    try: f.flush()
                    except Exception: pass
        sys.stdout = _Tee(sys.stdout, lf)
        sys.stderr = _Tee(sys.stderr, lf)
        print(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} =====")
    except Exception:
        pass

    moon = Moon()
    units = [u.strip() for u in a.units.split(",") if u.strip()]
    target = a.target or find_in_cfg(a.cfg_dir, "serial_port")
    key = a.key if a.key is not None else (find_in_cfg(a.cfg_dir, "tcp_key") or "")
    # ══ A BARE FILENAME RESOLVES BESIDE THIS SCRIPT. ══
    #
    # So the gcode macro can say `--artifact ht_artifact.json` instead of
    # carrying an absolute path that every recipient has to edit for their own
    # username. The script already knows where it lives; the caller should not
    # have to. An absolute path still wins, and a path with a directory in it
    # (./x, ../x) is left alone so it stays relative to the CWD as before.
    if a.artifact and not os.path.isabs(a.artifact) and os.sep not in a.artifact:
        artifact = os.path.join(here, a.artifact)
    else:
        artifact = a.artifact or os.path.join(here, "ams_artifact.json")

    print(f"=== AMS update: label={a.label or '(auto)'} mode={a.mode.upper()} "
          f"bridge={target} ===")
    if not target or not target.startswith("tcp://"):
        print(f"no tcp:// bridge target (got {target!r}); pass --target"); return 2

    # Detach: preflight/go must stop klipper, which would kill this process if it
    # stayed a child of the klipper service. Relaunch into our own systemd unit
    # (once). `check` is read-only and never stops klipper, so it runs inline.
    if a.mode != "check" and not a.detached:
        rc, out = relaunch_detached(a.sudo_pass_file)
        if rc != 0:
            print("could not launch detached:\n" + out); return rc
        print("launched detached; progress in logs/ams_flash.log"); return 0

    # 1 not printing
    print("[1] printer not printing")
    try:
        st = moon.q("print_stats").get("state")
    except Exception as e:
        print(f"    ABORT: cannot read print state ({e})"); return 1
    print(f"    state = {st}")
    if st in ("printing", "paused"):
        print("    ABORT: a print is active."); return 1

    # 2 artifact
    print("[2] artifact verifies")
    try:
        art = load_artifact(artifact)
        print(f"    OK: header + {len(art['blocks'])} blocks, CRCs verified")
    except Exception as e:
        print(f"    ABORT: artifact {artifact}: {e}"); return 1
    # ══ THE BLOCKS MUST ADD UP TO THE DECLARED IMAGE -- NOT COUNT 168. ══
    #
    # `len(blocks) != 168` was the AMS 2's block count and nothing else. It
    # refused a perfectly good AMS HT artifact (157 blocks) while reporting it
    # as a verification failure, and it would equally have waved through a
    # 168-block artifact missing half its bytes, because a count says nothing
    # about content.
    #
    # The artifact describes itself instead: the header frame carries the total
    # image size and every data block carries its own firmware-byte count, so
    # the blocks must sum to the declared image less the BIMH container header.
    # True on both models, independent of size, version and block count.
    #
    # (The tools/ copy of this flasher was fixed for exactly this and the fix
    # was never carried across to the standalone. Found while listing what a
    # demo unit needs in order to flash BOTH an AMS 2 and an HT.)
    total, summed = artifact_image_bytes(art)
    print(f"    image: header declares {total} bytes, blocks carry {summed} "
          f"+ {total - summed} container header")
    if total - summed != BIMH_HEADER_LEN:
        print(f"    ABORT: artifact INCOMPLETE -- blocks do not add up to the "
              f"declared image ({total} - {summed} != {BIMH_HEADER_LEN})")
        return 1

    # 3 bridge fw
    print(f"[3] bridge fw >= {TARGET_FW}")
    fw = bridge_fw(moon, units)
    print(f"    bridge_fw = {fw if fw > 0 else 'unknown'}")
    if fw < TARGET_FW:
        print(f"    ABORT: bridge is below {TARGET_FW}. Update the bridge first "
              f"(one-time), then retry."); return 1

    # 4 one unit online
    print("[4] exactly one AMS online")
    on = online_units(moon, units)
    print(f"    online: {on or '(none)'}")
    if len(on) != 1:
        print(f"    ABORT: need exactly one AMS on the bus (found {len(on)})."); return 1
    label = a.label or on[0]
    print(f"    target unit: {on[0]}  (label: {label})")

    if a.mode == "check":
        print("\n==> READ-ONLY CHECKS PASSED. Nothing was changed."); return 0

    # 5 probe (klipper down: the board allows one client)
    print("[5] cmd1 enters the loader (non-destructive)")
    if not klipper(moon, "stop"):
        print("    ABORT: could not stop klipper to free the board socket."); return 1
    time.sleep(3)
    rc = 1
    try:
        link = Link(target)
        try:
            if not authenticate(link, key):
                print("    ABORT: link auth failed"); return 1
            if not probe_cmd1(link):
                print("    NO-GO: unit did not enter its loader on cmd1"); return 1
            if a.mode == "preflight":
                print("\n==> PREFLIGHT PASSED. Everything up to the erase is green; "
                      "nothing on the AMS changed. Re-run MODE=go to flash.")
                rc = 0
                return 0
            # 6 flash
            print(f"[6] FLASH: header (erase) + {len(art['blocks'])} blocks")
            rc = do_flash(link, art)
            if rc != 0:
                print("    FLASH incomplete; loader intact -- re-run MODE=go.")
                return rc
        finally:
            link.close()
    finally:
        klipper(moon, "start")
        wait_ready(moon, 90)

    # 7 verify online
    print("[7] unit reboots and comes back online")
    t0 = time.time()
    while time.time() - t0 < 60:
        if online_units(moon, [on[0]]):
            print(f"    {on[0]} ONLINE on the new firmware.")
            print("\n==> DONE. Start the dryer and load a tray to confirm.")
            return 0
        time.sleep(5)
    print("    not seen online within 60s -- check AFC_BAMBU_UIDS / status.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
