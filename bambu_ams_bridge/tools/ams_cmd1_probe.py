#!/usr/bin/env python3
"""
Probe the AMS enter-loader trigger -- NON-DESTRUCTIVE, no flash, no erase.

    ams_cmd1_probe.py --target tcp://<bridge>:8888

The captures showed one distinctive frame right before every AMS loader entry
(docs/AMS_FW_UPDATE.md, "What's sent the second before it enters the loader"):
a single long frame, source 0x0900 (the printer's "updater" role), op 0601,
cmd 01 -- the cmd1 handshake -- sent ~47 frames (~270 ms) before the unit
announces "Loader Version" / "wait cmd1". It appears exactly once in the whole
pre-update stream and is byte-identical (modulo counter/CRC) across both units
and every session.

This tool sends THAT ONE FRAME to a running unit and watches whether it drops
into its bootloader. It answers the only open question:

    A) cmd1 alone makes a running app jump -> entry is solved.
    B) the jump is gated on some other (off-bus) flag -> cmd1 alone does nothing.

WHY IT IS SAFE. It sends cmd1 ONLY. cmd1 is the handshake, not the transfer:
the destructive erase happens after cmd2 (the header), which this tool never
builds or sends -- it asserts op==0601 and cmd==01 on the frame before it goes
out and refuses anything else. Worst case the unit sits in its loader waiting;
a power cycle boots it straight back to the application (proven by capD, a plain
boot with zero loader narration). Nothing is erased.

HOW IT LISTENS. It does not switch to passive sniff (a loader with no master
may go quiet). It turns on `txecho`, so the firmware -- which keeps mastering
the unit -- streams back every frame it sends AND every frame the AMS sends
(the RX funnel, bambubus.c read_reply). The loader announce arrives on that
stream as a dir="rx" frame while the unit is still being polled, exactly as it
did from a real printer.

It uses only long-standing firmware commands (`raw`, `txecho`) that are already
on the running bridge, so NO reflash is needed.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sniff_capture_link import Link, find_key, auth_reply   # noqa: E402

# The two Bambu-Bus CRCs, confirmed params (firmware crc.h / crc.c):
#   CRC-8 : poly 0x39, init 0x66, MSB-first, no reflection
#   CRC-16: poly 0x1021, init 0x913D, MSB-first, no reflection
def crc8(data: bytes, poly=0x39, init=0x66) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def crc16(data: bytes, poly=0x1021, init=0x913D) -> int:
    c = init
    for b in data:
        c = (c ^ (b << 8)) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c


def build_cmd1(target: int, counter: int, ams_id: int = 0x00) -> bytes:
    """The captured enter-loader poke, rebuilt with fresh CRCs.

    Layout (49 bytes), from docs/AMS_FW_UPDATE.md:
        3D 04 |cnt16| |len16=0x0031| |crc8| |tgt16| 00 09 | 06 01 | 01 | dev
        |0000 0000| |id| |27x00| |crc16|
    tgt = update target address; source is 0x0900, the "updater" role; op 0601,
    cmd 01.

    :param target: device address -- 0x0700 for a boxed AMS, 0x1800 for an HT
    :param counter: rolling frame counter for [2:4]
    :param ams_id: the unit's own id at [19] -- 0x00 on a boxed AMS, 0x80 on
      an HT ("ams 128" in the loader's own narration)

    THIS USED TO BE AMS-2-ONLY IN THREE PLACES, and two of them were invisible
    because they sat in the payload rather than in the address. [14] was
    written as `0x07  # constant seen in every capture` -- true, and useless,
    because every capture we held was an AMS 2. The HT update capture
    (2026-09-16) has a cmd1 that differs in exactly two body bytes:

        AMS 2   3d04|cnt|3100|c8|0007 0009 0601 01 07 00000000 00 ...
        AMS HT  3d04|cnt|3100|c8|0018 0009 0601 01 18 00000000 80 ...

    [14] turns out to be the target's own HIGH BYTE, so it is derived here
    rather than passed -- one fewer thing to get out of step with the address.
    [19] is the unit id and has to be given. Sending the AMS 2 shape at an HT
    is not dangerous (the unit simply does not enter its loader, and the flash
    refuses at the loader check) but it cannot work, which is worse to debug
    than a refusal.
    """
    f = bytearray(49)
    f[0] = 0x3D
    f[1] = 0x04
    f[2] = counter & 0xFF
    f[3] = (counter >> 8) & 0xFF
    f[4] = 0x31            # len = 49, LE
    f[5] = 0x00
    f[6] = crc8(bytes(f[:6]))
    f[7] = target & 0xFF
    f[8] = (target >> 8) & 0xFF
    f[9] = 0x00            # source 0x0900 (updater)
    f[10] = 0x09
    f[11] = 0x06           # op 0601
    f[12] = 0x01
    f[13] = 0x01           # cmd 01
    f[14] = (target >> 8) & 0xFF   # device class: 0x07 boxed, 0x18 HT
    # f[15:19] already zero
    f[19] = ams_id & 0xFF
    # f[20:47] already zero
    c = crc16(bytes(f[:47]))
    f[47] = c & 0xFF
    f[48] = (c >> 8) & 0xFF
    # HARD GUARD: this tool sends cmd1 and nothing else. op 0601, cmd 01.
    assert f[11] == 0x06 and f[12] == 0x01 and f[13] == 0x01, "not a cmd1 frame"
    return bytes(f)


def authenticate(link: Link, key: str, timeout: float = 8.0) -> bool:
    """The board's link-key challenge, done exactly as the Klipper module does.

        board -> {"evt":"auth","nonce":"<32 hex>"}
        host  -> {"cmd":"auth","mac":"<64 hex>"}   HMAC-SHA512(key, nonce)[:32]
        board -> {"evt":"auth","ok":1}             or it drops the socket

    Verbose on stderr: a silent drop after the mac (wrong key) and a missing
    challenge (open board / socket still held by klipper) look identical to a
    caller otherwise, and we have burned a deploy cycle on exactly that.
    """
    import hashlib, hmac as _hmac
    sock = getattr(link, "sock", None)
    if sock is None:                       # serial (USB) link: line read, BOUNDED
        # link.lines() is an infinite generator (yields None on a quiet tick),
        # so this MUST have a deadline or it hangs forever -- which it did over
        # USB, where the board typically does not send an auth challenge at all
        # (link auth is a TCP/network feature). Wait briefly for a possible
        # challenge, answer it if it comes, then proceed: a quiet USB link needs
        # no auth, and if auth WERE required the board would just reject the
        # following commands (a clean failure, not a hang).
        deadline = time.time() + max(timeout, 2.0)
        sent_mac = False
        for line in link.lines():
            if time.time() > deadline:
                break
            if not line:
                continue                   # quiet tick
            s = line.replace(" ", "")
            if '"ok":1' in s:
                print("  auth: ok (serial)", file=sys.stderr)
                return True
            if '"nonce"' in line:
                m = re.search(r'"nonce"\s*:\s*"([0-9a-fA-F]+)"', line)
                if m and key:
                    link.send({"cmd": "auth", "mac": auth_reply(key, m.group(1))})
                    sent_mac = True
                    print("  auth: sent mac (serial); awaiting ok", file=sys.stderr)
        if sent_mac:
            print("  auth: mac sent, no explicit ok within timeout -- proceeding",
                  file=sys.stderr)
        else:
            print("  auth: no challenge over serial (USB link, no auth expected)"
                  " -- proceeding", file=sys.stderr)
        return True
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
            print("  auth: board closed before any challenge", file=sys.stderr)
            return False
        buf += c
    if b"\n" not in buf:
        if not key:
            return True
        print("  auth: no challenge line arrived (open board? klipper still "
              "holding the socket?)", file=sys.stderr)
        return False
    line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
    print(f"  auth: challenge = {line}", file=sys.stderr)
    if '"nonce"' not in line:
        # Board is open and already streaming; treat as authenticated.
        link.buf = buf                      # hand the bytes to the reader
        return True
    if not key:
        print("  auth: board wants a key but none configured", file=sys.stderr)
        return False
    nonce = bytes.fromhex(json.loads(line)["nonce"])
    mac = _hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
    print(f"  auth: nonce {nonce.hex()} keylen {len(key)} -> mac {mac.hex()[:16]}...",
          file=sys.stderr)
    sock.sendall(f'{{"cmd":"auth","mac":"{mac.hex()}"}}\n'.encode())
    reply = buf.split(b"\n", 1)[1]
    while time.time() < deadline and b"\n" not in reply:
        try:
            c = sock.recv(256)
        except (socket.timeout, TimeoutError):
            continue
        if not c:
            print("  auth: board dropped after mac -> WRONG KEY", file=sys.stderr)
            return False
        reply += c
    rl = reply.split(b"\n", 1)[0].decode("utf-8", "replace")
    print(f"  auth: reply = {rl}", file=sys.stderr)
    ok = '"ok":1' in rl.replace(" ", "")
    link.buf = reply.split(b"\n", 1)[1] if b"\n" in reply else b""
    sock.settimeout(1.0)
    return ok


# ASCII fingerprints of the loader narration, as they ride the bus (hex).
SIGS = {
    "[MCU_UP]":        "5b4d43555f55505d",
    "Loader Version":  "4c6f61646572205665727369",
    "wait cmd1":       "7761697420636d6431",
    "wait cmd":        "776169742063 6d64".replace(" ", ""),
}


def loader_hits(hexstr: str) -> list[str]:
    h = hexstr.lower()
    out = [name for name, pat in SIGS.items() if pat in h]
    # An op-0601 frame FROM the AMS is the loader answering the handshake.
    #
    # THIS USED TO TEST `h.startswith("3d04")`, WHICH IS THE MASTER'S OWN
    # FRAME. The comment already had it right -- "the master's pokes are
    # source 0900; an AMS-origin 0601 is the unit in its loader" -- and then
    # the code checked the class byte instead of the source. Class 0x04 is
    # exactly the shape build_cmd1() emits, so anything that put our own
    # transmission into this window would read as "the loader answered", and
    # enter_loader() would hand off to the header -- an ERASE -- at a unit that
    # had said nothing at all. Measured across both models' update captures,
    # the classes carry no model information and do not separate the roles:
    #
    #   cls 0x04  tgt <unit>  src 0009   the master's cmd1 poke       (x5)
    #   cls 0x00  tgt <unit>  src 0009   the master's data blocks
    #   cls 0x00  tgt 0009    src <unit> the loader's announce        (x3)
    #   cls 0x05  tgt 0009    src <unit> the loader's block acks
    #
    # The SOURCE is what separates them, identically on an AMS 2 (unit 0x0700)
    # and an AMS HT (unit 0x1800). Source sits at bytes [9:11], immediately
    # before the opcode at [11:13], so an AMS-origin 0601 is any "0601" not
    # preceded by the updater's own 0009 -- no frame boundary needed, which
    # matters because this is handed a raw capture window, not a clean frame.
    i = h.find("0601")
    while i >= 0:
        if i < 4 or h[i - 4:i] != "0009":
            out.append("ams-origin/op0601")
            break
        i = h.find("0601", i + 4)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True,
                    help="tcp://host:port or a serial path to the bridge")
    ap.add_argument("--key", default=None, help="link key (default: from config)")
    ap.add_argument("--target-addr", default="0x0700",
                    help="update target address: 0x0700 boxed AMS (default), "
                         "0x1800 AMS HT")
    ap.add_argument("--ams-id", default="0x00",
                    help="the unit's own AMS id at frame[19]: 0x00 boxed "
                         "(default), 0x80 for an AMS HT. MUST match "
                         "--target-addr -- an HT ignores a poke carrying the "
                         "boxed id and never enters its loader")
    ap.add_argument("--counter", default="0x0077",
                    help="rolling counter for the cmd1 frame (default 0x0077)")
    ap.add_argument("--window-ms", type=int, default=500,
                    help="immediate raw capture window after the poke (ms)")
    ap.add_argument("--tail", type=float, default=3.0,
                    help="seconds to keep listening on txecho after the poke")
    a = ap.parse_args()

    target = int(a.target_addr, 0)
    counter = int(a.counter, 0)
    frame = build_cmd1(target, counter, int(a.ams_id, 0))
    print(f"cmd1 poke ({len(frame)} B): {frame.hex().upper()}")
    print(f"  target=0x{target:04X}  source=0x0900  op=0601 cmd=01  counter=0x{counter:04X}")

    key = find_key(a.key)
    link = Link(a.target)
    collected: list[tuple[str, str]] = []    # (dir, hex)
    immediate = ""
    try:
        if not authenticate(link, key):
            print("link auth FAILED", file=sys.stderr)
            return 1
        print("link authenticated; arming txecho ...")
        link.send({"cmd": "txecho", "on": 1})
        # let the echo ring clear and confirm it is on
        t0 = time.time()
        for line in link.lines():
            if line and '"txecho"' in line and 'true' in line:
                break
            if time.time() - t0 > 4:
                print("txecho not confirmed on", file=sys.stderr)
                return 1

        print(">>> sending cmd1 poke")
        link.send({"cmd": "raw", "hex": frame.hex(),
                   "us": a.window_ms * 1000})

        # Read the immediate raw reply AND the txecho tail together.
        deadline = time.time() + a.tail + a.window_ms / 1000.0 + 1.0
        for line in link.lines():
            if line:
                if '"evt":"raw"' in line or '"evt": "raw"' in line:
                    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line)
                    if m:
                        immediate = m.group(1)
                elif '"evt":"tx"' in line or '"evt": "tx"' in line:
                    d = re.search(r'"dir"\s*:\s*"(\w+)"', line)
                    m = re.search(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"', line)
                    if m:
                        collected.append((d.group(1) if d else "?", m.group(1)))
            if time.time() > deadline:
                break
    finally:
        link.send({"cmd": "txecho", "on": 0})
        time.sleep(0.2)
        link.close()

    # ---- verdict -------------------------------------------------------------
    print(f"\nimmediate raw capture ({len(immediate)//2} B): "
          f"{immediate.upper() or '(nothing)'}")
    rx_frames = [h for d, h in collected if d == "rx"]
    print(f"txecho: {len(collected)} frames "
          f"({len(rx_frames)} from the AMS) over {a.tail:.1f}s tail")

    hits: dict[str, list[str]] = {}
    # scan the immediate reply as one blob and every echoed frame
    for label, h in [("immediate", immediate)] + [("rx", h) for h in rx_frames]:
        for name in loader_hits(h):
            hits.setdefault(name, []).append(h)

    print()
    if hits:
        print("==> LOADER ENTERED. cmd1 alone triggered the jump (hypothesis A).")
        for name, frames in hits.items():
            print(f"    {name}: {len(frames)} frame(s)")
            for fr in frames[:3]:
                print(f"        {fr.upper()[:120]}")
        print("\n    The unit is now in its bootloader. Power-cycle to return it")
        print("    to the application, or run the flash driver to update it.")
        return 0
    else:
        print("==> NO JUMP. No loader narration on the bus after the poke.")
        print("    Either the jump is gated on an off-bus flag (hypothesis B),")
        print("    or this frame's counter/target/timing did not match. Things")
        print("    to vary before concluding B: --counter, --target-addr, and")
        print("    whether the unit was actually online/mastered at send time.")
        # a small sample so the operator can eyeball what WAS on the wire
        if rx_frames:
            print("\n    sample AMS frames seen (should be ordinary telemetry):")
            for h in rx_frames[:4]:
                print(f"        {h.upper()[:100]}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
