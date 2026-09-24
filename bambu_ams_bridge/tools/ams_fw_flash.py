#!/usr/bin/env python3
"""
Drive an AMS firmware update from the bridge -- the transmit path for the replay.

  ams_fw_flash.py precheck <artifact.json> --target tcp://<bridge>:8888
  ams_fw_flash.py flash    <artifact.json> --target tcp://<bridge>:8888 \
                           --arm <ams-serial> --i-understand-this-erases

This is the destructive end of everything in docs/AMS_FW_UPDATE.md. It streams
the per-model update transfer (header + the image's data blocks) to an AMS in
its bootloader, using the firmware's fwreplay/txbuf/txsend primitives. The
bootloader ERASES its firmware flash right after the header and before the first
data block; there is no BOOTSEL on the far side, so a wrong or truncated stream
bricks the unit.

Every guard here exists because of that:

  * `flash` refuses without BOTH `--arm <serial>` and `--i-understand-this-erases`.
    `precheck` never sends an update frame at all -- it only confirms the link,
    the artifact, and that a unit is actually in the loader.
  * The artifact is re-verified (frame count, CRCs) before a byte is sent.
  * The unit must be seen announcing the loader ("wait cmd1" / a cmd-1 frame)
    before the header/erase goes out. A running unit is never erased.
  * Each block is sent only after the previous one is acknowledged; a missing
    or wrong ack STOPS the stream rather than pressing on into a half-written
    image.

NOTHING RUNS WITHOUT THE ARM FLAGS. Importing this module does nothing; even
`flash` returns before opening the link unless both flags are present.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_capture import crc8, crc16                      # noqa: E402
from ams_fw_replay import plan, wrap                        # noqa: E402
from ams_cmd1_probe import build_cmd1, loader_hits, authenticate  # noqa: E402
from sniff_capture_link import Link, find_key, auth_reply   # noqa: E402

CONFIRM = "ERASE-AMS-FW"          # the firmware's fwreplay confirm token
CHUNK = 100                       # hex-payload bytes per txbuf line (fits LINE_SZ)
# The loader's own completion narration, ASCII inside the reply frames right
# after the last block: "[MCU_UP] up finish , check" then "[MCU_UP] success!".
# success! means the WHOLE-image check passed and the loader is resetting into
# the new app. Block acks are receipt, not integrity -- this is the only
# trustworthy "the flash took and it rebooted" signal.
SUCCESS_HEX = "7375636365737321"  # "success!"
CHUNKERR_HEX = "6368756e6b2068617368"  # "chunk hash" (loader's per-chunk hash fail)
# "img hash" -- the loader's WHOLE-IMAGE verdict, and a different thing entirely
# from the per-chunk one above. A chunk hash error means those bytes arrived
# mangled: the loader refuses that chunk, we resend it, and a fresh pass usually
# lands. An img hash error means every chunk was ACCEPTED on arrival and the
# assembled image still did not hash -- the bytes we sent are not the bytes that
# firmware is supposed to be. Resending them cannot change that, and each pass
# costs a full erase+rewrite, so flash() stops on this one instead of retrying.
IMGERR_HEX = "696d672068617368"


def load_artifact(path: str) -> dict:
    d = json.load(open(path))
    art = {"header": bytes.fromhex(d["header"]),
           "blocks": [bytes.fromhex(b) for b in d["blocks"]]}
    # Re-verify: the plan must build header+N frames, all CRC-correct.
    steps = list(plan(art))
    bad = [s for _k, s, fr in steps
           if crc8(fr[:6]) != fr[6] or crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8))]
    if bad or len(steps) != 1 + len(art["blocks"]):
        raise ValueError("artifact failed re-verification -- refusing to use it")
    return art


#: Bytes of BIMH container header that ride in the update's declared total but
#: are NOT carried in the data blocks' own firmware-byte counts. Measured as
#: exactly 416 on both models' captured artifacts (AMS 2: 172132 - 171716; AMS
#: HT: 160552 - 160136), and it is the same container header the carve reports.
BIMH_HEADER_LEN = 416


def artifact_image_bytes(art: dict) -> tuple:
    """(declared total, bytes actually carried by the blocks).

    THE COMPLETENESS CHECK, replacing `len(blocks) == 168`. That literal was
    the AMS 2's block count and nothing else -- it refused a perfectly good
    AMS HT artifact (157 blocks) while claiming it had "failed verification",
    and it would equally have waved through a 168-block artifact missing half
    its bytes, because a count says nothing about content.

    The artifact describes itself instead. The header frame carries the total
    image size (frame [23:26], LE24) and every data block carries its own
    firmware-byte count (frame [35:39], LE32), so the blocks must add up to the
    declared image less the container header. That holds on both models, is
    independent of image size and version, and actually checks the lengths the
    transfer will use.

    Bodies are captured frame[7:-2], hence the -7 on every offset.
    """
    hdr = art["header"]
    total = int.from_bytes(hdr[23 - 7:26 - 7], "little")
    summed = sum(int.from_bytes(b[35 - 7:39 - 7], "little")
                 for b in art["blocks"])
    return total, summed


def _await(link: Link, evt: str, timeout: float = 8.0):
    """Wait for a specific {"evt":...} line back from the board.

    On timeout, dump any bytes still sitting in the link buffer (a partial line
    the device dribbled but never terminated) so the log shows WHAT the bridge
    was sending when it stopped acking -- the key clue for a USB-CDC stall."""
    t0 = time.time()
    for line in link.lines():
        if line and f'"{evt}"' in line:
            return line
        if time.time() - t0 > timeout:
            pend = getattr(link, "buf", b"") or b""
            if pend:
                head = pend[:160]
                ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in head)
                print(f"    [await '{evt}' timeout] {len(pend)} pending byte(s), "
                      f"head: {head.hex()} | {ascii_}", file=sys.stderr)
            else:
                print(f"    [await '{evt}' timeout] no pending bytes "
                      f"(device silent)", file=sys.stderr)
            return None


def _rx_of(line: str) -> bytes:
    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line or "")
    return bytes.fromhex(m.group(1)) if m else b""


def _seqretry(rx: bytes):
    """Parse the loader's '[MCU_UP] N seq_num: X, retry: M' out of a reply.

    Returns (want, retry): `want` is the NEXT seq the loader expects (hex on the
    wire -> int), `retry` its attempt counter. Confirmed by capture: after block
    N is accepted the loader answers want == N+1; if it does not advance it is
    asking for that chunk again. None when the reply carries no such narration
    (e.g. the completion frame, which instead carries success! / chunk hash)."""
    a = "".join(chr(b) if 32 <= b < 127 else " " for b in rx)
    m = re.findall(r"seq_num:\s*([0-9a-fA-F]+),\s*retry:\s*(\d+)", a)
    if not m:
        return None
    want, rt = m[-1]
    try:
        return int(want, 16), int(rt)
    except ValueError:
        return None


def send_frame(link: Link, frame: bytes, window_us: int = 8000,
               stage_tries: int = 4) -> bytes:
    """Stage one frame across txbuf lines, then txsend; return the AMS reply.

    The bridge verifies the staged frame's own CRC16 before transmitting (fw >=
    1.75). If a txbuf line was mangled over WiFi/TCP it replies crc:bad and we
    RE-STAGE this one frame -- fixing host->bridge corruption per-frame, so a
    single dropped byte no longer makes the loader reject (and re-erase) the
    whole image. A frame that will not stage clean after several tries means the
    link is dropping bytes; that surfaces as an IOError -> pass-level retry."""
    for attempt in range(stage_tries):
        for off in range(0, len(frame), CHUNK):
            try:
                link.send({"cmd": "txbuf", "off": off,
                           "hex": frame[off:off + CHUNK].hex()})
            except Exception as e:
                # A blocked/timed-out write means the bridge stopped draining
                # its host link mid-frame (USB-CDC back-pressure). Surface the
                # exact offset so the log shows where, and let flash() retry.
                raise IOError(f"txbuf WRITE stalled at off {off}/{len(frame)}"
                              f" ({type(e).__name__}: {e})")
            if not _await(link, "txbuf", 4.0):
                raise IOError(f"txbuf not acked at off {off}/{len(frame)}")
        try:
            link.send({"cmd": "txsend", "n": len(frame), "us": window_us})
        except Exception as e:
            raise IOError(f"txsend WRITE stalled ({type(e).__name__}: {e})")
        line = _await(link, "txsend", timeout=max(2.0, window_us / 1e6 + 2))
        if line is None:
            raise IOError("txsend not acked")
        if '"crc":"bad"' in line:
            print(f"    staged frame CRC bad (WiFi) -- re-staging "
                  f"({attempt + 1}/{stage_tries})", file=sys.stderr)
            continue
        return _rx_of(line)
    raise IOError("staged frame CRC bad after re-staging -- link dropping bytes")


def unit_in_loader(link: Link, watch_s: float = 6.0) -> bool:
    """True if the AMS is announcing its bootloader on the bus.

    Uses a passive sniff so we listen WITHOUT transmitting -- a running unit
    must never be driven by this tool. The loader repeats 'wait cmd1' and small
    06/01 cmd-1 frames; either is proof it is waiting to be flashed.
    """
    link.send({"cmd": "sniff", "on": 1})
    seen = False
    t0 = time.time()
    for line in link.lines():
        if line:
            if "wait cmd1" in line or "Loader Version" in line:
                seen = True
                break
            m = re.search(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"', line)
            if m:
                h = m.group(1).upper()
                # a master/unit 06/01 cmd-1 frame: ...0601 01...
                if "060101" in h:
                    seen = True
                    break
        if time.time() - t0 > watch_s:
            break
    link.send({"cmd": "sniff", "on": 0})
    _await(link, "sniff_mode", 3.0)
    return seen


def precheck(artifact_path: str, target: str, key: str) -> int:
    art = load_artifact(artifact_path)
    print(f"artifact OK: header + {len(art['blocks'])} blocks, CRCs verified")
    link = Link(target)
    try:
        if not authenticate(link, key):
            print("link auth FAILED", file=sys.stderr)
            return 1
        print("link authenticated")
        loader = unit_in_loader(link)
        print(f"AMS in bootloader: {'YES' if loader else 'no (not seen)'}")
        print("\nprecheck only -- no update frames were sent")
        return 0
    finally:
        link.close()


def loader_target_of(art: dict) -> tuple:
    """Who this artifact is addressed to: (device address, AMS id).

    READ OFF THE ARTIFACT, never configured. This used to be
    `LOADER_TARGET = 0x0700  # update target address, as in every capture` --
    true of every capture we had, because they were all AMS 2. An AMS HT is
    device 0x1800, AMS id 0x80, and the same two fields the cmd1 frame needs
    sit in the artifact's own frames at exactly the offsets cmd1 uses:

        header/block  [7:9] target      0007 boxed / 0018 HT
                      [19]  AMS id      0x00 boxed / 0x80 HT

    Deriving them makes the dangerous mistake structurally impossible: you
    cannot aim the enter-loader poke at one device and then stream another
    device's image at it. A flag would have allowed exactly that, and the
    failure mode is an erase on the wrong unit.
    """
    hdr = art["header"]
    # The artifact stores frame BODIES (captured frame[7:-2]), so the header
    # body starts at what is byte 7 of a whole frame -- hence the -7 shift.
    target = hdr[0] | (hdr[1] << 8)
    ams_id = hdr[19 - 7]
    return target, ams_id


def enter_loader(link: Link, art: dict, tries: int = 6) -> bool:
    """Send cmd1 until the loader announces itself, then hand off to the header.

    cmd1 (op 0601 cmd 01) both wakes a running app INTO its bootloader and is
    the bootloader's own handshake ("wait cmd1"), so this one frame covers both
    states -- a unit still running from a klipper poll, or one already waiting.
    It sends cmd1 ONLY (build_cmd1 asserts op/cmd), never a header, so a unit
    that does not answer is left completely untouched: no erase can happen until
    this returns True and the header goes out.

    The poke is addressed from the artifact (loader_target_of), so it always
    names the same unit the image is for.
    """
    target, ams_id = loader_target_of(art)
    print(f"enter-loader: cmd1 -> device 0x{target:04X}, ams id 0x{ams_id:02X}")
    cmd1 = build_cmd1(target, 0x0077, ams_id)
    for i in range(tries):
        rx = send_frame(link, cmd1, window_us=500000)   # long: catch the announce
        hits = loader_hits(rx.hex()) if rx else []
        if hits:
            print(f"loader confirmed after cmd1 #{i+1}: {','.join(hits)}")
            return True
    return False


def _one_pass(link: Link, art: dict, steps: list) -> str:
    """One erase+rewrite pass, LOADER-DRIVEN: send the block the loader's reply
    asks for (seq_num), resending in place any chunk it does not accept, so a
    glitched chunk is fixed within the pass instead of failing the whole image.
    Returns the loader's OWN verdict, read gap-free from the block windows
    (fw >= 1.74 captures it there):
      'success'  -- loader hashed the whole image OK; it is resetting into the app
      'badimage' -- chunk hash error, a stuck chunk, or no success! after all blocks
      'stall'    -- no loader answer / a block went unacked
    'badimage' and 'stall' both leave the unit IN the loader, so the caller may
    safely resend the whole pass. It never probes after the verdict, so a good
    flash is never mistaken for a failure and re-erased -- the verdict is the
    loader's, not a guess about online/tag state."""
    link.send({"cmd": "fwreplay", "on": 1, "confirm": CONFIRM})
    if not _await(link, "fwreplay", 4.0):
        print("bridge did not enter fwreplay mode", file=sys.stderr)
        return "stall", 0
    try:
        # cmd1 handshake FIRST. The loader ignores a header until it handshakes;
        # sending the header (erase) before this strands a half-written image.
        # No loader answer -> no erase.
        if not enter_loader(link, art):
            print("no loader response to cmd1 -- NOTHING was erased.",
                  file=sys.stderr)
            return "stall", 0
        blocks = art["blocks"]
        nblk = len(blocks)
        header = next(f for k, s, f in steps if k == "header")
        bframe = {s: f for k, s, f in steps if k == "data"}

        # Header first -- triggers the ~2.3 s erase; the loader is then ready for
        # block 0.
        rx = send_frame(link, header, window_us=3_500_000)
        if not rx:
            print("header NO ACK -- loader idle; unit left for a re-erase",
                  file=sys.stderr)
            return "stall", 0
        print("header sent, erase triggered; streaming blocks...")

        # LOADER-DRIVEN transfer. Each block reply names the next seq the loader
        # wants: want == last+1 means the block was accepted; want not advancing
        # means it wants that chunk resent. Following it resends a corrupted chunk
        # WITHIN the pass (the way the printer does) instead of failing the whole
        # image -- so a single glitched chunk no longer costs a full re-flash. A
        # corrupt header/handshake still dooms the pass; that falls to flash()'s
        # pass-level retry, which re-does the handshake.
        want = 0
        resends = {}                       # seq -> times resent this pass
        total = 0
        last_rx = rx
        MAX_PER = 8
        MAX_TOTAL = nblk * 6
        while want < nblk:
            frame = bframe.get(want)
            if frame is None:
                print(f"loader wants seq {want} (outside 0..{nblk-1}) -- "
                      f"resending the whole pass", file=sys.stderr)
                return "badimage", sum(resends.values())
            win = 3_000_000 if want == nblk - 1 else 400_000
            rx = send_frame(link, frame, window_us=win)
            if not rx:
                print(f"\ndata {want}: NO ACK -- stopping (unit left in loader)",
                      file=sys.stderr)
                return "stall", sum(resends.values())
            last_rx = rx
            total += 1
            if total > MAX_TOTAL:
                print("too many block sends this pass -- resending the whole "
                      "pass", file=sys.stderr)
                return "badimage", sum(resends.values())
            rxl = rx.hex().lower()
            if SUCCESS_HEX in rxl:
                print("loader: success! -- image verified, resetting into the app")
                return "success", sum(resends.values())
            if CHUNKERR_HEX in rxl:
                print("loader: chunk hash error -- resending the whole pass",
                      file=sys.stderr)
                return "badimage", sum(resends.values())
            sr = _seqretry(rx)
            if sr is None:
                want += 1                  # no seq narration: treat as accepted
                continue
            nxt, rt = sr
            if nxt > want:
                want = nxt                 # accepted -- follow the loader forward
            else:
                resends[nxt] = resends.get(nxt, 0) + 1
                print(f"  loader wants seq {nxt} resent "
                      f"(#{resends[nxt]}, retry {rt})", file=sys.stderr)
                if resends[nxt] > MAX_PER:
                    print(f"seq {nxt} will not take after {MAX_PER} resends -- "
                          f"resending the whole pass", file=sys.stderr)
                    return "badimage", sum(resends.values())
                want = nxt                 # resend exactly what it asked for
            if want and want % 24 == 0:
                print(f"  at block {want}/{nblk - 1}")

        # All blocks accepted; the verdict is in the final reply's window.
        rxhex = last_rx.hex().lower()
        if SUCCESS_HEX in rxhex:
            print("loader: success! -- image verified, resetting into the app")
            return "success", sum(resends.values())
        if CHUNKERR_HEX in rxhex:
            print("loader: chunk hash error -- resending the whole pass",
                  file=sys.stderr)
            return "badimage", sum(resends.values())
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in last_rx)
        if IMGERR_HEX in rxhex:
            print(f"loader: IMG HASH ERROR after all blocks were accepted "
                  f"({len(last_rx)} B) -- {ascii_[-120:]}", file=sys.stderr)
            return "badimg", sum(resends.values())
        print(f"loader: no success! after all blocks ({len(last_rx)} B) -- "
              f"{ascii_[-120:]}", file=sys.stderr)
        return "badimage", sum(resends.values())
    finally:
        link.send({"cmd": "fwreplay", "on": 0})
        _await(link, "fwreplay", 4.0)


def flash(artifact_path: str, target: str, key: str, arm: str,
          retries: int = 6) -> int:
    """Erase + rewrite an AMS, retrying the WHOLE flash until the loader itself
    reports success!. The block transfer intermittently corrupts a chunk -- the
    loader catches it ('chunk hash error') and refuses to boot -- so a single
    pass is a coin flip; this resends until a clean transfer verifies. Every
    non-success pass leaves the unit in the loader (safe to re-erase); a success!
    pass means the loader already reset into the app, so the loop returns and
    never re-erases a live unit. Retries default high because only a real
    success! ends it."""
    art = load_artifact(artifact_path)
    steps = list(plan(art))
    print(f"ARMED for {arm}: about to ERASE and rewrite an AMS. "
          f"{len(steps)} frames; up to {retries + 1} attempt(s).")
    # A USB-CDC bridge (/dev/...) can stop draining its RX FIFO mid-transfer,
    # which blocks a bare pyserial write forever. Bound it so a stuck write
    # surfaces as an IOError -> pass retry instead of hanging the flash (and
    # Klipper, which is stopped for the duration). TCP has its own timeouts.
    link = Link(target, write_timeout=None if target.startswith("tcp://") else 15.0)
    try:
        if not authenticate(link, key):
            print("link auth FAILED", file=sys.stderr)
            return 1
        for attempt in range(1, retries + 2):
            print(f"== flash attempt {attempt}/{retries + 1} ==")
            try:
                verdict, resent = _one_pass(link, art, steps)
            except IOError as e:
                # A staging/link stall (e.g. USB-CDC back-pressure) aborts this
                # pass but leaves the unit in the loader -- safe to re-erase.
                print(f"attempt {attempt}: link stall -- {e}", file=sys.stderr)
                verdict, resent = "stall", -1
            if verdict == "success":
                print("FLASH CONFIRMED: the loader verified the image and reset "
                      "into the new firmware.")
                return 0
            # ══ A CLEAN PASS THAT STILL FAILS THE HASH IS A BAD IMAGE. ══
            #
            # Retrying is right for TRANSPORT corruption -- a chunk gets mangled,
            # the loader catches it, and a fresh pass usually lands. It cannot
            # help when the bytes arrived intact and the IMAGE is simply wrong:
            # resending identical bytes gets an identical verdict, and each pass
            # costs a full erase+rewrite of the unit's flash.
            #
            # The two are distinguishable and nothing was distinguishing them.
            # If the loader never asked for a single resend, every block was
            # accepted first time, so the bytes it hashed are the bytes we meant
            # to send. A hash error on top of that is about the content.
            #
            # Measured 2026-09-19: a deliberately mis-built image (a boxed
            # artifact carrying another firmware's per-block tags) failed with
            # "img hash error" after a transport-clean pass, and the flasher
            # then re-erased and re-sent the same bytes six more times. Seven
            # erase/write cycles to learn what the first pass already knew.
            if verdict == "badimg":
                print(f"\nattempt {attempt}: IMG HASH ERROR -- the loader "
                      f"accepted every chunk and then rejected the assembled "
                      f"image. That is the CONTENT being wrong, not the link "
                      f"({resent} chunk resend(s) happened and were resolved), "
                      f"so resending identical bytes gets an identical verdict "
                      f"and costs another erase+rewrite.\n"
                      f"The unit is left in its loader (recoverable): flash a "
                      f"known-good artifact for this model and version.",
                      file=sys.stderr)
                return 1
            if attempt <= retries:
                extra = "" if resent < 0 else f" ({resent} chunk resend(s))"
                print(f"attempt {attempt}: {verdict}{extra} -- resending the "
                      f"full flash", file=sys.stderr)
        print("\nout of attempts: the loader never verified success!. The unit "
              "is left in its loader (recoverable) -- re-run. If every pass acks "
              "but reports chunk hash error, the bus/link is dropping bytes.",
              file=sys.stderr)
        return 1
    finally:
        link.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["precheck", "flash"])
    ap.add_argument("artifact")
    ap.add_argument("--target", required=True, help="tcp://host:port or serial path")
    ap.add_argument("--key", default=None, help="link key (default: from config)")
    ap.add_argument("--arm", default=None, help="AMS serial being flashed")
    ap.add_argument("--i-understand-this-erases", action="store_true",
                    dest="confirm")
    ap.add_argument("--retries", type=int, default=6,
                    help="extra passes until the loader verifies success! "
                         "(default 6). Resends only leave-in-loader failures "
                         "(chunk hash error / stall); success! ends it and never "
                         "re-erases a live unit.")
    a = ap.parse_args()
    key = find_key(a.key)

    # Optional debug aid, OFF by default. AMSFLASH_FAULTDUMP=<secs> arms a
    # non-fatal watchdog that dumps every thread's stack to stderr every <secs>
    # -- useful if a link ever stalls again. It does NOT exit (a legitimate noisy
    # flash can take several passes / minutes), so it never kills a real flash;
    # the outer `timeout` guard in update_bridge.sh is the hard backstop.
    _fd = os.environ.get("AMSFLASH_FAULTDUMP", "0")
    if _fd != "0":
        import faulthandler
        faulthandler.dump_traceback_later(float(_fd), repeat=True,
                                          file=sys.stderr)

    if a.action == "precheck":
        return precheck(a.artifact, a.target, key)

    # flash: both guards, or nothing happens.
    if not a.arm or not a.confirm:
        print("REFUSED: `flash` needs --arm <ams-serial> AND "
              "--i-understand-this-erases.\n"
              "It erases the AMS firmware before the first data block and there "
              "is no BOOTSEL on the far side.\n"
              "Use `precheck` to validate the link and artifact without sending "
              "anything.", file=sys.stderr)
        return 2
    return flash(a.artifact, a.target, key, a.arm, retries=a.retries)


if __name__ == "__main__":
    sys.exit(main())
