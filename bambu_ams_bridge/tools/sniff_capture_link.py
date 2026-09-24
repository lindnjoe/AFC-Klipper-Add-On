#!/usr/bin/env python3
"""
Passive bus capture from a bridge that is TOGGLED into sniff mode.

sniff_capture.sh is for a board running the SNIFF_BOOT image: listen-only from
reset, read over its serial port. That image is RP2040 and unsigned, so it will
not run on a SEALED Pico 2 / Pico 2 W -- a sealed RP2350 rejects an unsigned
image at the ROM, before any of our code runs, and the board looks dead rather
than refusing. Those boards reach sniff mode the other way, through the
firmware's own `{"cmd":"sniff","on":1}`, and the WiFi ones are not on a serial
port at all.

    sniff_capture_link.py --target tcp://192.168.1.50:8888 --label dry_then_print
    sniff_capture_link.py --target /dev/serial/by-id/usb-...  --label print_no_dry

THE TOGGLE IS RUNTIME, AND THAT IS A REAL DIFFERENCE. A SNIFF_BOOT board is
silent from reset. A toggled one is MASTER from power-up until this script's
command lands, and a second master on a bus the printer is driving corrupts the
traffic being captured -- including, at the wrong moment, a running print. So:

  * set sniff mode BEFORE the A/B tap is live, or with the printer powered down
  * power the board from the Pi, not the printer, so the printer cannot
    power-cycle it back into master mode mid-capture
  * if it does reboot, this script says so (the board re-announces itself) and
    the capture from that point is polluted

POLARITY. A correct A/B tap gives ~54 valid frames/s from the first second. A
swapped one gives near zero. This prints the live rate for exactly that reason
-- it is the difference between knowing in five seconds and finding out after a
900 s capture that the file is empty.

Output matches sniff_capture.sh's: one timestamped line per board line, under
~/printer_data/logs/sniff_<label>.log, which Moonraker serves. decode_capture.py
and capture_diff.py read it directly.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import re
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from decode_capture import read_capture  # noqa: E402,F401  (shared frame rules)
from decode_capture import crc16, split_frames  # noqa: E402

DEF_LOGDIR = Path.home() / "printer_data" / "logs"
DEF_CFGDIR = Path.home() / "printer_data" / "config"


def find_key(explicit: str | None) -> str:
    """The bridge link key, as Klipper holds it.

    --linkkey-set writes it into the printer config as `tcp_key:` beside
    `serial_port:`, so that is where it is read from rather than asked for.
    An empty key means the board is open and the handshake never happens.
    """
    if explicit is not None:
        return explicit
    for cfg in sorted(DEF_CFGDIR.glob("*.cfg")):
        m = re.search(r"^\s*tcp_key:\s*(\S+)", cfg.read_text(errors="replace"),
                      re.M)
        if m:
            return m.group(1)
    return ""


def auth_reply(key: str, nonce_hex: str) -> str:
    """HMAC-SHA512(key, nonce)[:32], hex.

    The key is the ASCII string as stored -- NOT the bytes it would hex-decode
    to; the firmware passes it to crypto_sha512_hmac with strlen(). The nonce
    is the opposite: 16 RAW bytes, hex-decoded from the wire. Getting either
    one the other way round produces a plausible-looking 64-hex answer that is
    silently rejected, so both are spelled out here.
    """
    mac = hmac.new(key.encode(), bytes.fromhex(nonce_hex), hashlib.sha512)
    return mac.digest()[:32].hex()


class Link:
    """A board link, over TCP or a serial port. Line in, line out."""

    def __init__(self, target: str, timeout: float = 1.0,
                 write_timeout: float | None = None):
        self.tcp = target.startswith("tcp://")
        self.buf = b""
        if self.tcp:
            host, _, port = target[6:].partition(":")
            self.sock = socket.create_connection((host, int(port or 8888)),
                                                 timeout=5)
            self.sock.settimeout(timeout)
        else:
            import serial            # pyserial, as the rest of the tools use
            # write_timeout guards a USB-CDC bridge that stops draining its RX
            # FIFO mid-transfer: a bare pyserial write blocks FOREVER when the
            # device NAKs the OUT endpoint, stranding the flasher (and Klipper,
            # stopped for the flash). With a timeout the write raises
            # SerialTimeoutException, which the flasher turns into a pass-level
            # retry instead of a hang.
            self.ser = serial.Serial(target, 115200, timeout=timeout,
                                     write_timeout=write_timeout)

    def send(self, obj: dict) -> None:
        self.write((json.dumps(obj) + "\n").encode())

    def write(self, b: bytes) -> None:
        if self.tcp:
            self.sock.sendall(b)
        else:
            self.ser.write(b)

    def lines(self):
        """Yield complete lines; None on a quiet interval so callers can tick.

        NEVER issue a blocking read on a serial port. pyserial's timed read()
        can block INDEFINITELY on a USB-CDC device (Linux cdc-acm: select()
        reports the fd ready, then os.read() blocks in the kernel while the
        device is mid-transfer). When that happens `for line in lines()` never
        yields, so a caller waiting on a deadline can never enforce it -- the
        whole flash wedges and takes Klipper down until an outside guard kills
        it. So for serial we read ONLY what the OS already has buffered
        (`in_waiting`) and sleep briefly when it is empty: a read that never
        blocks. Every read that does not complete a line yields None so the
        caller always regains control to check its own timeout."""
        while True:
            if self.tcp:
                try:
                    chunk = self.sock.recv(4096)
                except socket.timeout:
                    yield None
                    continue
                if not chunk:
                    return                  # board closed the connection
            else:
                # Non-blocking: take only already-buffered bytes, never wait in
                # a kernel read. in_waiting is a cheap ioctl that cannot hang.
                try:
                    navail = self.ser.in_waiting
                    chunk = self.ser.read(navail) if navail else b""
                except Exception:
                    chunk = b""
                if not chunk:
                    time.sleep(0.02)        # idle poll; keeps CPU sane
                    yield None
                    continue
            self.buf += chunk
            if b"\n" not in self.buf:
                yield None                  # bytes arrived but no full line yet
                continue
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                yield line.decode("utf-8", "replace").rstrip("\r")

    def close(self):
        try:
            (self.sock if self.tcp else self.ser).close()
        except Exception:
            pass


def valid_frames(hexstr: str) -> tuple:
    """(total, crc_ok) for one sniff line -- the polarity check.

    A swapped A/B does not give nothing, it gives NOISE, and noise still
    arrives as bytes. Only the CRC separates the two, which is why the live
    counter counts validated frames and not lines.
    """
    try:
        blob = bytes.fromhex(hexstr)
    except ValueError:
        return 0, 0
    tot = ok = 0
    for fr, why in split_frames(blob):
        if why:
            continue
        tot += 1
        if len(fr) > 2 and crc16(fr[:-2]) == (fr[-2] | (fr[-1] << 8)):
            ok += 1
    return tot, ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True,
                    help="tcp://host[:8888] or a serial device path")
    ap.add_argument("--label", default="capture")
    ap.add_argument("--seconds", type=float, default=900.0)
    ap.add_argument("--key", default=None,
                    help="link key (default: tcp_key from the printer config)")
    ap.add_argument("--logdir", type=Path, default=DEF_LOGDIR)
    ap.add_argument("--keep-master", action="store_true",
                    help="do NOT put the board back to master mode at the end")
    args = ap.parse_args()

    args.logdir.mkdir(parents=True, exist_ok=True)
    out = args.logdir / f"sniff_{args.label}.log"
    key = find_key(args.key)
    link = Link(args.target)
    print(f"target {args.target}   key {'yes' if key else 'none (open board)'}")

    fh = out.open("a")
    fh.write("# ── bus capture (toggled sniff mode) ────────────────────────\n")
    for k, v in (("label", args.label), ("started", time.strftime("%FT%T")),
                 ("target", args.target), ("duration", f"{args.seconds:g}s")):
        fh.write(f"# {k}:  {v}\n")
    fh.write("#\n# FILL IN BEFORE COMMITTING:\n"
             "#   on the bus:   (e.g. 'one AMS 2 Pro + a real P2S')\n"
             "#   AMS firmware: (from its boot narration)\n"
             "#   printer:      (model, and what it was doing)\n"
             "#   purpose:      (which claim this confirms or kills)\n"
             "# ───────────────────────────────────────────────────────────\n")
    fh.flush()

    armed = False
    t0 = time.time()
    tot = ok = 0
    last_report = t0
    reboots = 0
    # ── INTEGRITY, NOT JUST CONTENT ────────────────────────────────────────
    # Protocol archaeology survives a lossy capture; a BULK TRANSFER does not.
    # An image reconstructed from 99% of its bytes is a brick, and nothing
    # downstream can tell it from a good one. The firmware stamps every blob
    # with a sequence number and a cumulative UART overrun count (see
    # bb_sniff_poll): a gap in sq is whole lines the pipe dropped, and any
    # movement in ov is bytes the UART lost before the firmware saw them --
    # which sq cannot catch, because the line still arrives well-formed.
    seq_prev = None
    seq_gaps = 0
    seq_lost = 0
    ovr_first = None
    ovr_last = 0
    rf_first = None
    rf_last = 0
    try:
        for line in link.lines():
            now = time.time()
            if line is not None:
                fh.write(f"{time.strftime('%H:%M:%S')}"
                         f".{int((now % 1) * 1000):03d} {line}\n")
                if '"nonce"' in line:
                    if armed:
                        # A fresh challenge on a live link means the board
                        # restarted -- and it restarts as MASTER.
                        reboots += 1
                        print(f"\n!! the board re-challenged at "
                              f"{now - t0:.0f}s -- it REBOOTED, so it was "
                              f"master on the bus until this re-arms. "
                              f"Everything after here is suspect.")
                    m = re.search(r'"nonce"\s*:\s*"([0-9a-fA-F]+)"', line)
                    if m and key:
                        link.send({"cmd": "auth", "mac": auth_reply(key,
                                                                   m.group(1))})
                    continue
                if '"auth"' in line and '"ok"' in line:
                    link.send({"cmd": "sniff", "on": 1})
                    continue
                if '"sniff_mode"' in line:
                    armed = '"on":true' in line.replace(" ", "")
                    print(f"sniff mode {'ON -- listening' if armed else 'OFF'}")
                    continue
                ms = re.search(r'"sq"\s*:\s*(\d+)', line)
                if ms:
                    sq = int(ms.group(1))
                    if seq_prev is not None and sq != seq_prev + 1:
                        seq_gaps += 1
                        seq_lost += max(0, sq - seq_prev - 1)
                    seq_prev = sq
                mo = re.search(r'"ov"\s*:\s*(\d+)', line)
                if mo:
                    ovr_last = int(mo.group(1))
                    if ovr_first is None:
                        ovr_first = ovr_last
                mr = re.search(r'"rf"\s*:\s*(\d+)', line)
                if mr:
                    rf_last = int(mr.group(1))
                    if rf_first is None:
                        rf_first = rf_last
                m = re.search(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"', line)
                if m:
                    a, b = valid_frames(m.group(1))
                    tot += a
                    ok += b
            # An OPEN board never challenges, so nothing above ever fires.
            # Arm it after a moment of quiet rather than waiting forever.
            if not armed and not key and now - t0 > 2.0:
                link.send({"cmd": "sniff", "on": 1})
                armed = None            # sent; waiting for the echo
            if now - last_report >= 5.0:
                el = now - t0
                rate = ok / el if el else 0
                warn = ("   <== near zero: SWAP A AND B" if rate < 5 and el > 8
                        else "")
                bad = ""
                if seq_lost:
                    bad += f"  !! {seq_lost} BLOBS LOST"
                if ovr_first is not None and ovr_last > ovr_first:
                    bad += f"  !! {ovr_last - ovr_first} UART OVERRUNS"
                if rf_first is not None and rf_last > rf_first:
                    bad += f"  !! {rf_last - rf_first} RING LAPS"
                print(f"\r{el:6.0f}s  {ok} valid frames  {rate:5.1f}/s"
                      f"  ({tot - ok} bad crc){bad}{warn}   ",
                      end="", flush=True)
                last_report = now
            if now - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        el = time.time() - t0
        if not args.keep_master:
            # Back to master, so the board is usable as a bridge again. Skip it
            # with --keep-master while the tap is still on a live printer bus:
            # master mode there is a second transmitter.
            try:
                link.send({"cmd": "sniff", "on": 0})
                time.sleep(0.3)
            except Exception:
                pass
        ovr = (ovr_last - ovr_first) if ovr_first is not None else 0
        rf = (rf_last - rf_first) if rf_first is not None else 0
        fh.write(f"# ended: {time.strftime('%FT%T')}  "
                 f"{ok} valid frames in {el:.0f}s\n")
        # IN THE FILE, not just on the terminal. Whoever reads this capture in
        # six months needs to know whether it is complete without re-running it.
        fh.write(f"# integrity: {seq_lost} blob(s) lost in {seq_gaps} gap(s), "
                 f"{ovr} UART overrun(s), {rf} ring lap(s) -- "
                 f"{'LOSSLESS' if not seq_lost and not ovr and not rf else 'INCOMPLETE, do not reconstruct binaries from this'}\n")
        fh.close()
        link.close()

    rate = ok / el if el else 0
    print(f"\n{out}")
    print(f"{ok} valid frames, {tot - ok} bad crc, {rate:.1f}/s over {el:.0f}s")
    if seq_lost or ovr or rf:
        print(f"!! NOT LOSSLESS: {seq_lost} blob(s) lost in {seq_gaps} gap(s), "
              f"{ovr} UART overrun(s), {rf} ring lap(s).\n"
              f"   Fine for protocol archaeology, USELESS for reconstructing a\n"
              f"   firmware image -- take it again with less on the link.")
    else:
        print("lossless: no blob gaps, no UART overruns, no ring laps")
    if reboots:
        print(f"!! {reboots} reboot(s) during the capture -- the board was "
              f"master on the bus for part of it")
    if rate < 5:
        print("!! a correct tap gives ~54 valid frames/s. This is not one:\n"
              "   swap A and B, check the printer was actually talking to the\n"
              "   AMS, and confirm the board reached sniff mode above.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
