# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shared host-side RFID reader stack: transport-agnostic MFRC522 primitives,
# MIFARE Classic access, and the multi-brand tag decoders (Bambu, Anycubic,
# Snapmaker, Creality CFS, Elegoo, BTT/ViViD "BQ Tech").
#
# Everything runs on a minimal link contract, an object exposing
# reg_read(reg) -> byte and reg_write(reg, val), plus an optional
# reader_power(on), so the same stack serves every reader transport: the
# ACE2 firmware serial passthrough, the ViViD/OpenAMS MCU-SPI links, and the
# OpenAMS i2c-hook shim. read_tag() is the unified entry point.
#
# Nothing here talks to Klipper: no printer object, no config, no reactor.
#
# CREDITS — the per-brand key derivation and tag layouts are not original work.
# They come from the projects and write-ups below, ported to the link contract
# above. Each section banner names its own source as well.
#
#   OpenRFID — https://github.com/suchmememanyskill/OpenRFID
#       Bambu: the HKDF-SHA256 key derivation (salt -> per-sector Key A) and
#       the block field layout. Anycubic: the NTAG page layout, jointly with
#       DnG-Crafts. The bulk of what this module knows about reading a tag.
#   DnG-Crafts
#       Anycubic (with OpenRFID) and the Snapmaker U1 per-tag sector keys.
#   Bambu-Research-Group/RFID-Tag-Guide
#       Bambu block 16: colour count and the second colour's reversed ABGR.
#   ELEGOO-3D/ELEGOO-RFID-Tag-Guide
#       Elegoo NTAG213 EPC-256 field offsets.
#   bigtreetech/BIGTREETECH_MMS
#       BTT/ViViD "BQ Tech" tag layout, from its mfrc522.py.

from __future__ import annotations
import hashlib
import hmac
import struct
from typing import Any, Callable, Dict, List, Optional, Tuple


# ── MFRC522 register map (subset used) ───────────────
CommandReg, ComIrqReg, ErrorReg, Status2Reg = 0x01, 0x04, 0x06, 0x08
FIFODataReg, FIFOLevelReg, ControlReg, BitFramingReg, CollReg = 0x09, 0x0A, 0x0C, 0x0D, 0x0E
TxControlReg = 0x14
# commands
PCD_IDLE, PCD_AUTHENT, PCD_TRANSCEIVE, PCD_MFAUTHENT = 0x00, 0x0E, 0x0C, 0x0E
# PICC (card) commands
PICC_REQA, PICC_WUPA = 0x26, 0x52
PICC_ANTICOLL, PICC_SELECT = 0x93, 0x93          # cascade level 1
PICC_ANTICOLL_CL2, PICC_SELECT_CL2 = 0x95, 0x95  # cascade level 2
SAK_UID_INCOMPLETE = 0x04              # "keep cascading", not a tag type
CASCADE_TAG = 0x88                     # stands in for the UID bytes CL1 lacks
PICC_AUTH_KEY_A, PICC_AUTH_KEY_B = 0x60, 0x61
PICC_READ = 0x30
PICC_WRITE_PAGE = 0xA2                 # NTAG / Ultralight 4-byte page write
PICC_WRITE_CLASSIC = 0xA0              # MIFARE Classic 16-byte block write
NTAG_ACK = 0x0A                        # 4-bit reply; anything else is a NAK
# The writable window. Pages 0-3 hold the UID, the one-way static lock bits and
# the capability container; past the user area sit the dynamic lock bits, the
# config pages and the password. Writing any of those is irreversible, so a
# tag whose capability container will not parse is held to the NTAG213 bound,
# the smallest of the family (see MifareClassic.user_last_page).
NTAG_USER_FIRST_PAGE = 4
NTAG213_LAST_PAGE = 39

#: Capability-container size byte -> chip name, for the NTAG21x family. The CC
#: at page 3 reads "E1 10 <size> <access>", where size counts the user memory
#: in 8-byte units, so it names the chip as well as GET_VERSION does and costs
#: nothing extra: page 3 is already inside the first 16 bytes every NTAG read
#: starts with.
NTAG_CC_CHIPS = {0x12: "NTAG213", 0x3E: "NTAG215", 0x6D: "NTAG216"}


def ntag_chip(data: bytes) -> Tuple[Optional[str], Optional[int]]:
    """
    Name an Ultralight-family chip from its capability container.

    A plain MIFARE Ultralight has no CC -- page 3 is its OTP area and reads
    00 00 00 00 until someone writes it -- and a factory-fresh NTAG that has
    never been NDEF-formatted can be blank there too. Both come back (None,
    None) rather than being guessed at, which is the honest answer: SAK 0x00
    alone does not say which chip is in the field.

    :param data: a tag read starting at page 0 (needs the first 16 bytes)
    :return tuple: (chip name or None, user-memory bytes or None)
    """
    if len(data) < 16 or data[12] != 0xE1 or not data[14]:
        return None, None
    return NTAG_CC_CHIPS.get(data[14]), data[14] * 8


class Mfrc522:
    """MFRC522 primitives built on any link object with reg_read/reg_write."""
    def __init__(self, link: Any) -> None:
        """
        MFRC522 driver over a register link.

        :param link: the register link
        """
        self.l = link

    def _set(self, reg: int, mask: int) -> None:
        """
        Set bits in a register.

        :param reg: register address
        :param mask: bits to set
        """
        self.l.reg_write(reg, self.l.reg_read(reg) | mask)

    def _clr(self, reg: int, mask: int) -> None:
        """
        Clear bits in a register.

        :param reg: register address
        :param mask: bits to clear
        """
        self.l.reg_write(reg, self.l.reg_read(reg) & (~mask & 0xFF))

    def reset(self) -> None:
        """Soft-reset + minimal timer/ASK/mode config.

        When the host powers the reader itself (host-owned power), the MFRC522
        comes up cold, the firmware normally does this bring-up during its own
        identify. Without it a cold chip can NAK/hang the first register read.
        These are register WRITES only (no reads) so a cold chip can't stall the
        bring-up; the caller dwells briefly afterwards for the oscillator.
        """
        self.l.reg_write(CommandReg, 0x0F)     # SoftReset
        self.l.reg_write(0x2A, 0x8D)           # TModeReg  (auto timer, TAuto)
        self.l.reg_write(0x2B, 0x3E)           # TPrescalerReg
        self.l.reg_write(0x2D, 30)             # TReloadReg low
        self.l.reg_write(0x2C, 0)              # TReloadReg high
        self.l.reg_write(0x15, 0x40)           # TxASKReg  (Force100ASK)
        self.l.reg_write(0x11, 0x3D)           # ModeReg   (CRC preset 0x6363)

    def stop_crypto(self) -> None:
        """End any MIFARE crypto session (clear MFCrypto1On in Status2Reg) so a
        fresh anticoll/select works, WITHOUT a SoftReset. A reset drops the RF
        field, which unpowers a marginal at-rest tag being re-selected between
        decode schemes; clearing crypto keeps the field up so the tag survives."""
        self._clr(Status2Reg, 0x08)

    def antenna_on(self) -> None:
        """
        Enable the TX antenna drivers if they are off.
        """
        if not (self.l.reg_read(TxControlReg) & 0x03):
            self._set(TxControlReg, 0x03)

    def _to_card(self, command: int, send: bytes,
                 tx_last_bits: int = 0,
                 err_mask: int = 0x1B) -> Tuple[bool, bytes, int]:
        """Execute Transceive/MFAuthent; return (ok, rx_bytes, rx_last_bits).

        ``err_mask`` selects which ErrorReg bits abort the exchange, default
        BufferOvfl|CollErr|ParityErr|ProtocolErr. A 4-bit ACK (the NTAG write
        reply) is a short frame carrying neither parity nor CRC, and the chip
        raises ProtocolErr on it, so that path passes a narrower mask and
        validates the ACK by its value instead.
        """
        irq_en, wait_irq = (0x12, 0x10) if command == PCD_TRANSCEIVE else (0x12, 0x10)
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)                      # flush FIFO
        for b in send:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, command)
        if command == PCD_TRANSCEIVE:
            self._set(BitFramingReg, 0x80)                 # StartSend
        for _ in range(2000):                              # poll ComIrq
            n = self.l.reg_read(ComIrqReg)
            if n & 0x30:                                   # Rx or Idle
                break
            if n & 0x01:                                   # Timer
                return False, b"", 0
        self._clr(BitFramingReg, 0x80)
        if self.l.reg_read(ErrorReg) & err_mask:            # see err_mask above
            return False, b"", 0
        rx = b""
        if command == PCD_TRANSCEIVE:
            n = self.l.reg_read(FIFOLevelReg)
            rx = bytes(self.l.reg_read(FIFODataReg) for _ in range(n))
            last = self.l.reg_read(ControlReg) & 0x07
            return True, rx, last
        return True, rx, 0

    def request(self, req: int = PICC_REQA) -> Optional[bytes]:
        """
        Send REQA/WUPA to wake a tag.

        :param req: request command byte
        :return Optional[bytes]: the 2-byte ATQA, None when nothing answered
        """
        self.l.reg_write(BitFramingReg, 0x07)              # 7-bit frame
        ok, atqa, _ = self._to_card(PCD_TRANSCEIVE, bytes([req]), 7)
        return atqa if ok and len(atqa) == 2 else None

    def anticoll(self, cmd: int = PICC_ANTICOLL) -> Optional[bytes]:
        """
        Run one anticollision cascade level.

        :param cmd: PICC_ANTICOLL for level 1, PICC_ANTICOLL_CL2 for level 2
        :return Optional[bytes]: the 4 UID bytes this level carries, None on
            failure. At level 1 on a 7-byte-UID tag the first of those is the
            cascade tag, not a UID byte.
        """
        self.l.reg_write(BitFramingReg, 0x00)
        ok, uid5, _ = self._to_card(PCD_TRANSCEIVE, bytes([cmd, 0x20]))
        if not ok or len(uid5) != 5:
            return None
        if uid5[0] ^ uid5[1] ^ uid5[2] ^ uid5[3] != uid5[4]:  # BCC
            return None
        return uid5[:4]

    def select(self, uid4: bytes, cmd: int = PICC_SELECT) -> Optional[int]:
        """Return SAK byte (or None). SAK 0x00 = Ultralight/NTAG, 0x08 = Classic
        1K, and bit 2 (SAK_UID_INCOMPLETE) means another cascade level follows.

        :param uid4: the 4 bytes this cascade level selects on
        :param cmd: PICC_SELECT for level 1, PICC_SELECT_CL2 for level 2
        """
        buf = bytes([cmd, 0x70]) + uid4 + bytes([uid4[0]^uid4[1]^uid4[2]^uid4[3]])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, sak, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return sak[0] if ok and len(sak) >= 1 else None

    def auth(self, key_type: int, block: int, key6: bytes, uid4: bytes) -> bool:
        """
        Authenticate a sector with a key.

        :param key_type: PICC_AUTHENT1A or 1B
        :param block: block number
        :param key6: 6-byte key
        :param uid4: 4-byte UID
        :return bool: True when Crypto1 is on
        """
        send = bytes([key_type, block]) + key6 + uid4
        ok, _, _ = self._to_card(PCD_MFAUTHENT, send)
        return ok and bool(self.l.reg_read(Status2Reg) & 0x08)  # MFCrypto1On

    def read_block(self, block: int) -> Optional[bytes]:
        """
        Read one 16-byte block.

        :param block: block number
        :return Optional[bytes]: the data, None on failure
        """
        buf = bytes([PICC_READ, block])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, data, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return data[:16] if ok and len(data) >= 16 else None

    def _short_ack(self) -> bool:
        """
        Read the 4-bit ACK that a MIFARE Classic write phase returns.

        Same shape as the NTAG write ACK: wait for RxIRq, let FIFOLevel settle
        over a slow bridge, then check for exactly one byte, 4 bits, 0x0A.

        :return bool: True on a valid ACK
        """
        level = 0
        for _ in range(8):
            level = self.l.reg_read(FIFOLevelReg)
            if level:
                break
        last = self.l.reg_read(ControlReg) & 0x07
        if level != 1 or last != 4:
            return False
        return (self.l.reg_read(FIFODataReg) & 0x0F) == NTAG_ACK

    def _classic_phase(self, frame: bytes) -> bool:
        """
        Send one CRC'd frame in an active Crypto1 session and read its 4-bit
        ACK. Used for both halves of the MIFARE Classic write.

        :param frame: bytes to send (CRC appended here)
        :return bool: True when the tag ACKed
        """
        buf = frame + struct.pack("<H", self._crc_a(frame))
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)
        for b in buf:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, PCD_TRANSCEIVE)
        self._set(BitFramingReg, 0x80)
        for _ in range(4000):
            n = self.l.reg_read(ComIrqReg)
            if n & 0x20:                                   # RxIRq
                break
            if n & 0x01:                                   # TimerIRq
                self._clr(BitFramingReg, 0x80)
                return False
        self._clr(BitFramingReg, 0x80)
        if self.l.reg_read(ErrorReg) & 0x12:               # BufferOvfl|ParityErr
            return False
        return self._short_ack()

    def write_classic_block(self, block: int, data16: bytes) -> bool:
        """
        MIFARE Classic WRITE (0xA0): a two-phase write of one 16-byte block.

        The sector MUST already be authenticated (Crypto1 on) -- the transceive
        rides the encrypted session. First the command frame (0xA0, block) gets
        a 4-bit ACK, then the 16 data bytes get another. Unlike the NTAG write
        this is two frames, matching the sibling FM175xx driver.

        Callers guard which block this touches -- a sector trailer or the
        manufacturer block are not written here (see classic_write_block).

        :param block: absolute block number
        :param data16: exactly 16 bytes
        :return bool: True when both phases ACKed
        """
        if len(data16) != 16:
            raise ValueError(f"Classic block write needs 16 bytes, got "
                             f"{len(data16)}")
        if not self._classic_phase(bytes([PICC_WRITE_CLASSIC, block])):
            return False
        return self._classic_phase(bytes(data16))

    def write_page(self, page: int, data4: bytes) -> bool:
        """
        NTAG / Ultralight WRITE: one 4-byte page, no authentication.

        Single-phase, unlike the MIFARE Classic 0xA0 write: the whole frame
        goes out at once and the tag answers a 4-bit ACK. Callers must have
        bounds-checked the page (see MifareClassic.write_ntag) -- this is the
        raw wire op and will happily program a lock page.

        :param page: page number
        :param data4: exactly 4 bytes
        :return bool: True when the tag ACKed
        """
        if len(data4) != 4:
            raise ValueError(f"NTAG page write needs 4 bytes, got {len(data4)}")
        buf = bytes([PICC_WRITE_PAGE, page]) + bytes(data4)
        buf += struct.pack("<H", self._crc_a(buf))
        # The NTAG WRITE answers with a bare 4-bit ACK (0x0A) after programming
        # its EEPROM, ~4ms later. The generic _to_card breaks on IdleIRq, which
        # a WRITE raises the instant TX finishes -- before the ACK -- so this
        # drives the transceive itself and waits for RxIRq specifically.
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)                      # flush FIFO
        for b in buf:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, PCD_TRANSCEIVE)
        self._set(BitFramingReg, 0x80)                     # StartSend
        got = False
        for _ in range(4000):
            n = self.l.reg_read(ComIrqReg)
            if n & 0x20:                                   # RxIRq: the ACK
                got = True
                break
            if n & 0x01:                                   # TimerIRq: no answer
                break
        self._clr(BitFramingReg, 0x80)
        err = self.l.reg_read(ErrorReg)
        # FIFOLevel can lag RxIRq over a slow (serial-bridged) link: the 4-bit
        # ACK is flagged (RxLastBits=4) a beat before it is counted in the
        # FIFO. Give it a few re-reads to appear rather than reading empty.
        level = 0
        for _ in range(8):
            level = self.l.reg_read(FIFOLevelReg)
            if level:
                break
        last = self.l.reg_read(ControlReg) & 0x07
        ack = self.l.reg_read(FIFODataReg) if level else None
        # Raw response for a failed write: (got_rx, ErrorReg, FIFOlevel,
        # RxLastBits, ack). Empty is a timeout; a 4-bit 0x00/0x01/0x05 is a NAK
        # (locked page / bad CRC / invalid arg); 0x0a is the ACK.
        self.last_write_ack = (got, hex(err), level, last,
                               None if ack is None else hex(ack))
        # BufferOvfl or ParityErr is a real failure; ProtocolErr is expected on
        # a 4-bit frame and ignored. A valid ACK is exactly one byte, 4 bits,
        # value 0x0A.
        if err & 0x12:
            return False
        return (got and level == 1 and last == 4 and ack is not None
                and (ack & 0x0F) == NTAG_ACK)

    def halt(self) -> None:
        """HLTA, put the selected tag into HALT so it stops answering REQA (only
        WUPA wakes it). Used to silence a shared-reader neighbour tag so this
        lane's own tag can respond. HLTA returns no reply; the resulting
        timeout/NAK is the expected success signal, so we ignore the result."""
        buf = bytes([0x50, 0x00])
        buf += struct.pack("<H", self._crc_a(buf))
        self._to_card(PCD_TRANSCEIVE, buf)

    @staticmethod
    def _crc_a(data: bytes) -> int:
        """
        ISO14443-A CRC (poly 0x8408, init 0x6363), done in software since we drive the FIFO.

        :param data: bytes to checksum
        :return int: the CRC
        """
        c = 0x6363
        for b in data:
            b ^= c & 0xFF
            b = (b ^ (b << 4)) & 0xFF
            c = ((c >> 8) ^ (b << 8) ^ (b << 3) ^ (b >> 4)) & 0xFFFF
        return c


class MifareClassic:
    """Full-tag read: activate → per-sector auth+read → 1024-byte image."""
    def __init__(self, mfrc: Mfrc522) -> None:
        """
        MIFARE Classic reader over an Mfrc522.

        :param mfrc: the MFRC522 driver
        """
        self.m = mfrc

    def activate(self, is_excluded: Optional[Callable[[str], bool]] = None,
                 seen: Optional[List[Any]] = None,
                 reset: bool = True) -> Tuple[Optional[bytes], Optional[int]]:
        """Return (uid, sak) for an ISO14443-A tag, or (None, None).

        With ``is_excluded`` (a ``uid_hex -> bool`` predicate) this handles a
        SHARED reader: if the tag it selects is excluded (a neighbour slot's tag
        bleeding into the antenna), HALT it so it stops answering REQA and
        request again, until this lane's own tag responds or the field is empty.
        The reader is reset once up front (never between tries, a reset would
        drop the RF field and wake the halted neighbour).

        ``reset=False`` RE-selects without a SoftReset: it keeps the RF field up
        (so a marginal at-rest tag isn't unpowered) and just ends any crypto
        session, for re-acquiring the SAME card between decode schemes.

        ``seen``, if given, is a list this appends ``(uid_hex, sak, excluded)``
        to for every tag it detects, a read diagnostic so callers can tell a
        halted-neighbour from an empty field."""
        if reset:
            self.m.reset()          # bring up a cold, host-powered reader
        else:
            self.m.stop_crypto()    # re-select without dropping the RF field
        self.m.antenna_on()
        if is_excluded is None:
            uid, sak = self._activate_once(wake=True)
            if seen is not None and uid is not None:
                seen.append((uid.hex(), sak, False))
            return uid, sak
        for i in range(MAX_FIELD_TAGS):
            uid, sak = self._activate_once(wake=(i == 0))
            if uid is None:
                return None, None
            excluded = bool(is_excluded(uid.hex()))
            if seen is not None:
                seen.append((uid.hex(), sak, excluded))
            if excluded:
                self.m.halt()       # sleep the neighbour; next REQA skips it
                continue
            return uid, sak
        return None, None

    def _activate_once(self, wake: bool) -> Tuple[Optional[bytes], Optional[int]]:
        """One REQA/anticoll/select. ``wake`` uses WUPA (wakes halted tags too)
        for first detection; otherwise REQA, which a halted neighbour ignores."""
        if wake:
            if (self.m.request(PICC_WUPA) is None
                    and self.m.request(PICC_REQA) is None):
                return None, None
        elif self.m.request(PICC_REQA) is None:
            return None, None
        uid = self.m.anticoll()
        if uid is None:
            return None, None
        sak = self.m.select(uid)
        if sak is None:
            return None, None
        if not sak & SAK_UID_INCOMPLETE:
            return uid, sak                    # 4-byte UID, one level, done
        # A 7-byte UID (every NTAG21x) does not fit one cascade level. Level 1
        # returns the cascade tag plus UID bytes 0-2 and a SAK that says so;
        # level 2 carries bytes 3-6. Stopping at level 1 leaves the tag only
        # HALF SELECTED, and reports "88" + three UID bytes as the UID -- and
        # since every NXP UID starts 04, that is two varying bytes, which
        # collides inside a single roll of stickers.
        if uid[0] != CASCADE_TAG:
            return None, None
        rest = self.m.anticoll(PICC_ANTICOLL_CL2)
        if rest is None:
            return None, None
        sak = self.m.select(rest, PICC_SELECT_CL2)
        if sak is None:
            return None, None
        return uid[1:4] + rest, sak

    def read_ntag(self, nbytes: int = 128,
                  start_page: int = 0) -> Optional[bytes]:
        """MIFARE Ultralight / NTAG read (no auth). READ page returns 4 pages.

        :param nbytes: how many bytes to read
        :param start_page: page to start at
        :return Optional[bytes]: the bytes, None if any read failed
        """
        data = bytearray()
        page = start_page
        while len(data) < nbytes:
            blk = self.m.read_block(page)      # NTAG uses the same 0x30 READ
            if blk is None:
                return None
            data += blk
            page += 4                          # each read returns 4 pages (16 B)
        return bytes(data[:nbytes])

    def user_last_page(self) -> int:
        """
        Highest page a writer may touch, read from the tag's own capability
        container rather than assumed from the chip we hope is in the field.

        Page 3 byte 2 is the user-memory size in 8-byte units, so the last user
        page is ``3 + size/4``. The value the CC reports is a few pages short of
        the datasheet figure on NTAG215/216, which is the direction we want: an
        under-estimate refuses a legal write, an over-estimate programs a lock
        page. A tag with no readable CC falls back to the NTAG213 bound, the
        smallest of the family.

        :return int: last writable page number
        """
        blk = self.m.read_block(3)
        if blk is None or blk[0] != 0xE1 or not blk[2]:
            return NTAG213_LAST_PAGE
        return 3 + blk[2] * 2

    def write_ntag(self, start_page: int, payload: bytes) -> Optional[str]:
        """
        Write a payload across consecutive NTAG pages and read it back.

        Refuses anything outside the user area. Pages 0-3 are the UID, the
        one-way static lock bits and the capability container; the pages past
        the user area are the dynamic lock bits, the config and the password.
        Those are fuses, not storage: a stray write there kills the tag for
        good, so the bound is enforced here rather than trusted from the
        caller's arithmetic.

        Every page is read back and compared before this reports success. That
        is the real check on the write, and it is why the ACK handling in
        write_page can afford to be lenient about a short frame's error bits.

        :param start_page: first page to write
        :param payload: bytes to write, a multiple of 4
        :return Optional[str]: None on success, else why it failed
        """
        if len(payload) % 4:
            return f"payload is {len(payload)} bytes, not a multiple of 4"
        pages = len(payload) // 4
        last = self.user_last_page()
        if start_page < NTAG_USER_FIRST_PAGE:
            return (f"page {start_page} is in the reserved area; the first "
                    f"writable page is {NTAG_USER_FIRST_PAGE}")
        if start_page + pages - 1 > last:
            return (f"{pages} pages from {start_page} runs past the last "
                    f"writable page ({last})")
        for i in range(pages):
            chunk = payload[i * 4:i * 4 + 4]
            # Retry a page: over a slow bridge the ACK read occasionally misses
            # even though the write took. Re-writing the same bytes is
            # idempotent, and the read-back below is the real proof.
            if not (self.m.write_page(start_page + i, chunk)
                    or self.m.write_page(start_page + i, chunk)
                    or self.m.write_page(start_page + i, chunk)):
                dbg = getattr(self.m, "last_write_ack", None)
                return (f"tag did not ACK the write to page {start_page + i} "
                        f"[resp {dbg}]")
        return self._verify_ntag(start_page, payload)

    def _verify_ntag(self, start_page: int, payload: bytes) -> Optional[str]:
        """
        Read written pages back and compare them to what was sent.

        :param start_page: first page written
        :param payload: the bytes that were written
        :return Optional[str]: None when the tag matches, else the first
            page that does not
        """
        pages = len(payload) // 4
        for i in range(0, pages, 4):                   # READ returns 4 pages
            blk = self.m.read_block(start_page + i)
            if blk is None:
                return f"could not read page {start_page + i} back"
            want = payload[i * 4:i * 4 + 16]
            if blk[:len(want)] != want:
                return (f"page {start_page + i} reads back as "
                        f"{blk[:len(want)].hex()}, not {want.hex()}")
        return None

    def read_all(self, uid: bytes, keys_a: List[List[int]],
                 sectors: int = 16) -> Optional[bytes]:
        """Read the first ``sectors`` sectors (auth + 4 blocks each). Returns
        ``sectors*64`` bytes; callers that need a full 1K image pad the rest.
        Every register op here is a serial round-trip, so reading only the
        sectors a decoder actually needs is a big speedup."""
        sectors = max(1, min(int(sectors), 16))
        data = bytearray(sectors * 64)
        for sector in range(sectors):
            if not self.m.auth(PICC_AUTH_KEY_A, sector * 4, bytes(keys_a[sector]), uid):
                return None
            for b in range(4):
                block = sector * 4 + b
                blk = self.m.read_block(block)
                if blk is None:
                    return None
                data[block * 16:block * 16 + 16] = blk
        return bytes(data)

    def read_blocks(self, uid: bytes, keys_a: List[List[int]],
                    blocks: Any) -> Optional[bytes]:
        """Read only the given block numbers into a 1024-byte image (rest zero),
        authenticating each sector once. Over the ACE serial passthrough every
        block read is a round-trip, so reading only the blocks a decoder needs
        (vs whole sectors) is a large speedup."""
        data = bytearray(1024)
        by_sector: Dict[int, List[int]] = {}
        for b in blocks:
            by_sector.setdefault(b // 4, []).append(b)
        for sector in sorted(by_sector):
            if not self.m.auth(PICC_AUTH_KEY_A, sector * 4, bytes(keys_a[sector]), uid):
                return None
            for b in sorted(by_sector[sector]):
                blk = self.m.read_block(b)
                if blk is None:
                    return None
                data[b * 16:b * 16 + 16] = blk
        return bytes(data)


# ── Bambu (HKDF key derivation + decode), port of OpenRFID ───────
def hkdf_sha256(salt: bytes, ikm: bytes, info: bytes, length: int) -> bytes:
    """
    HKDF-SHA256 extract-and-expand.

    :param salt: HKDF salt
    :param ikm: input keying material
    :param info: context bytes
    :param length: bytes to derive
    :return bytes: the derived key material
    """
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    okm, t, i = b"", b"", 0
    while len(okm) < length:
        i += 1
        t = hmac.new(prk, t + info + bytes([i]), hashlib.sha256).digest()
        okm += t
    return okm[:length]

BAMBU_SALT_HASH = "19cc3c63cb8802668800c3b3bf3fee05b3c59bf59fc5fd256b68e868084ec304"


def bambu_keys(uid: bytes, master_key: bytes) -> List[List[int]]:
    """
    Derive the 16 sector A keys for a Bambu tag.

    :param uid: 4-byte tag UID
    :param master_key: Bambu master key
    :return list: 16 six-byte keys
    """
    okm = hkdf_sha256(master_key, uid, b"RFID-A\x00", 6 * 16)
    return [list(okm[s * 6:(s + 1) * 6]) for s in range(16)]


def _s(d: bytes, p: int, n: int) -> str:
    """
    NUL-terminated ASCII string at an offset.

    :param d: tag image
    :param p: offset
    :param n: field width
    :return str: the decoded string
    """
    return d[p:p + n].split(b"\x00")[0].decode("ascii", "replace")


def _u16(d: bytes, p: int) -> int:
    """
    Little-endian u16 at an offset.

    :param d: tag image
    :param p: offset
    :return int: the value
    """
    return struct.unpack_from("<H", d, p)[0]


def _f32(d: bytes, p: int) -> float:
    """
    Little-endian float32 at an offset.

    :param d: tag image
    :param p: offset
    :return float: the value
    """
    return struct.unpack_from("<f", d, p)[0]


def decode_bambu(d: bytes) -> dict:
    """
    Decode a Bambu MIFARE Classic 1K tag image.

    :param d: 1024-byte tag image
    :return dict: decoded filament fields
    """
    if len(d) != 1024:
        error_str = "expected 1024-byte MIFARE Classic 1K image"
        raise ValueError(error_str)
    r, g, b, a = d[80], d[81], d[82], d[83]
    nozzle = round(_f32(d, 140), 2)                   # blk8 @140
    spool_width = _u16(d, 164)                        # blk10 @164, mm*100
    return dict(
        manufacturer="Bambu",
        type=_s(d, 32, 16), detailed=_s(d, 64, 16),
        color_argb=(a << 24) | (r << 16) | (g << 8) | b,
        weight_g=_u16(d, 84), diameter_mm=round(_f32(d, 88), 3),
        drying_temp_c=_u16(d, 96), drying_time_h=_u16(d, 98),
        bed_temp_c=_u16(d, 102), hotend_max_c=_u16(d, 104), hotend_min_c=_u16(d, 106),
        nozzle_diameter=nozzle if 0 < nozzle < 2 else None,
        spool_width_mm=round(spool_width / 100.0, 2) if spool_width else None,
        length_m=_u16(d, 228) or None,                # blk14 @228, metres
        tray_uid=d[144:160].hex(), production=_s(d, 192, 16),
    )


# ── Anycubic (MIFARE Ultralight/NTAG, no auth), port of OpenRFID/DnG-Crafts ──
ANYCUBIC_MAGIC = b"\x7b\x00\x65\x00"
_ANY_WEIGHT = {330: 1000, 247: 750, 198: 600, 165: 500, 82: 250}


def decode_anycubic(d: bytes) -> Optional[dict]:
    """
    Decode an Anycubic tag image.

    :param d: tag image
    :return Optional[dict]: decoded fields, None without the Anycubic magic
    """
    if len(d) < 0x7C or d[0x10:0x14] != ANYCUBIC_MAGIC:
        return None
    sku = _s(d, 0x14, 16)
    brand = _s(d, 0x28, 16)
    ftype = _s(d, 0x3C, 16)
    a, b, g, r = d[0x50], d[0x51], d[0x52], d[0x53]
    length_m = _u16(d, 0x7A)
    return dict(
        manufacturer=brand or "Anycubic", sku=sku, type=ftype,
        color_argb=(a << 24) | (r << 16) | (g << 8) | b,
        diameter_mm=round(_u16(d, 0x78) / 100.0, 2),
        weight_g=_ANY_WEIGHT.get(length_m, 1000),
        length_m=length_m or None,
        hotend_min_c=_u16(d, 0x60), hotend_max_c=_u16(d, 0x62),
        bed_temp_c=_u16(d, 0x76),
    )


# The image bytes encode_anycubic fills, and the NTAG page they land on. The
# magic sits at 0x10, which is page 4 -- the first page an NTAG lets anyone
# write -- so the whole record fits the user area of even an NTAG213.
ANYCUBIC_FIRST_PAGE = NTAG_USER_FIRST_PAGE
ANYCUBIC_IMAGE_START = 0x10
ANYCUBIC_IMAGE_END = 0x80              # exclusive; 112 bytes, 28 pages
# length_m -> weight, inverted from _ANY_WEIGHT. The layout carries no weight
# field: the decoder recovers grams from the length, so only these five spool
# sizes survive a round trip and anything else reads back as the 1000g default.
_ANY_LENGTH = {g: m for m, g in _ANY_WEIGHT.items()}


def _put_s(buf: bytearray, p: int, n: int, s: str) -> None:
    """
    Write a NUL-padded ASCII string into an image, truncating to fit.

    :param buf: image being built
    :param p: offset
    :param n: field width
    :param s: text to write
    """
    raw = (s or "").encode("ascii", "replace")[:n]
    buf[p:p + n] = raw + b"\x00" * (n - len(raw))


def _put_u16(buf: bytearray, p: int, v: Optional[int]) -> None:
    """
    Write a little-endian u16 into an image, clamped to the field.

    :param buf: image being built
    :param p: offset
    :param v: value, None or negative writes zero
    """
    struct.pack_into("<H", buf, p, max(0, min(int(v or 0), 0xFFFF)))


def anycubic_length_for_weight(weight_g: Optional[int]) -> int:
    """
    The length that makes decode_anycubic report a given spool weight.

    :param weight_g: desired weight in grams
    :return int: length in metres, 0 when the weight has no encoding
    """
    return _ANY_LENGTH.get(int(weight_g or 0), 0)


def encode_anycubic(manufacturer: str = "", sku: str = "", ftype: str = "",
                    color_argb: int = 0xFFFFFFFF, diameter_mm: float = 1.75,
                    weight_g: Optional[int] = None,
                    length_m: Optional[int] = None,
                    hotend_min_c: Optional[int] = None,
                    hotend_max_c: Optional[int] = None,
                    bed_temp_c: Optional[int] = None) -> bytes:
    """
    Build the Anycubic-layout bytes for pages 4-31 of a blank NTAG.

    The exact inverse of decode_anycubic, so a written tag reads back through
    the normal scan path on every host-side reader with no new decode code.
    Only the fields that decoder reads are populated; the gaps between them are
    left zero, which is what a blank tag already holds.

    ``manufacturer`` is a free string on the tag, so a spool keeps its real
    brand: the magic at 0x10 is a format fingerprint, not a claim about who
    made the filament.

    :param manufacturer: brand string, 16 bytes
    :param sku: SKU / part number, 16 bytes
    :param ftype: material name such as PLA or PETG, 16 bytes
    :param color_argb: packed ARGB colour
    :param diameter_mm: filament diameter
    :param weight_g: spool weight; stored as the length that decodes back to
        it when one of the five standard sizes, otherwise left blank here and
        carried exactly by the AFC block (see encode_tag_payload)
    :param length_m: filament length, overriding the weight lookup
    :return bytes: 112 bytes to write at page 4
    """
    if length_m is None:
        length_m = anycubic_length_for_weight(weight_g)
    buf = bytearray(ANYCUBIC_IMAGE_END)
    buf[0x10:0x14] = ANYCUBIC_MAGIC
    _put_s(buf, 0x14, 16, sku)
    _put_s(buf, 0x28, 16, manufacturer)
    _put_s(buf, 0x3C, 16, ftype)
    # Byte order on the tag is A, B, G, R (decode_anycubic reads it that way).
    buf[0x50] = (color_argb >> 24) & 0xFF
    buf[0x51] = color_argb & 0xFF
    buf[0x52] = (color_argb >> 8) & 0xFF
    buf[0x53] = (color_argb >> 16) & 0xFF
    _put_u16(buf, 0x60, hotend_min_c)
    _put_u16(buf, 0x62, hotend_max_c)
    _put_u16(buf, 0x76, bed_temp_c)
    _put_u16(buf, 0x78, round(float(diameter_mm) * 100))
    _put_u16(buf, 0x7A, length_m)
    return bytes(buf[ANYCUBIC_IMAGE_START:ANYCUBIC_IMAGE_END])


# ── AFC extension block ─────────────────────────────────────────────────────
# The Anycubic layout has no weight field: the decoder recovers grams from the
# stored length through a five-entry table, so an 800g spool cannot be said at
# all, and nothing in it points at a Spoolman record. Rather than redefine a
# field on a layout other people's tags also use, the exact values go in a
# block of our own at page 32 -- past everything decode_anycubic reads, and
# inside the user area of even an NTAG213, which ends at page 39.
#
# A genuine Anycubic tag has no magic there and is completely unaffected. A tag
# we wrote decodes through the Anycubic path exactly as before, and this block
# then overrides the fields that path could only approximate.
AFC_BLOCK_MAGIC = b"AFC1"
AFC_BLOCK_VERSION = 1
AFC_BLOCK_PAGE = 32
AFC_BLOCK_OFFSET = AFC_BLOCK_PAGE * 4          # 0x80
AFC_BLOCK_LEN = 32                             # pages 32-39
#: Bytes a full NTAG scan reads: through the end of the block.
NTAG_SCAN_BYTES = AFC_BLOCK_OFFSET + AFC_BLOCK_LEN


def encode_afc_block(weight_g=None, spool_id=None, density=None,
                     drying_temp_c=None, drying_time_h=None) -> bytes:
    """
    Build the AFC extension block: what the Anycubic layout cannot hold.

    :param weight_g: exact net filament weight in grams
    :param spool_id: Spoolman spool id this tag belongs to
    :param density: g/cm^3
    :param drying_temp_c: drying temperature
    :param drying_time_h: drying time in hours
    :return bytes: AFC_BLOCK_LEN bytes, to be written at AFC_BLOCK_PAGE
    """
    buf = bytearray(AFC_BLOCK_LEN)
    buf[0:4] = AFC_BLOCK_MAGIC
    buf[4] = AFC_BLOCK_VERSION
    _put_u16(buf, 6, weight_g)
    struct.pack_into("<I", buf, 8, max(0, min(int(spool_id or 0), 0xFFFFFFFF)))
    _put_u16(buf, 12, round(float(density) * 1000) if density else 0)
    _put_u16(buf, 14, drying_temp_c)
    _put_u16(buf, 16, drying_time_h)
    return bytes(buf)


def decode_afc_block(d: bytes, offset: int = AFC_BLOCK_OFFSET
                     ) -> Optional[dict]:
    """
    Decode the AFC extension block out of a tag image.

    Only fields that were actually set come back, so a caller can overlay the
    result onto a brand decode without blanking what the brand layout knew.

    :param d: tag image, or just the block when ``offset`` is 0
    :param offset: where the block starts in ``d``
    :return Optional[dict]: the fields that are set, None without the magic
    """
    p = offset
    if len(d) < p + AFC_BLOCK_LEN or d[p:p + 4] != AFC_BLOCK_MAGIC:
        return None
    if d[p + 4] > AFC_BLOCK_VERSION:
        return None                            # written by something newer
    out: Dict[str, Any] = {}
    if _u16(d, p + 6):
        out["weight_g"] = _u16(d, p + 6)
    spool_id = struct.unpack_from("<I", d, p + 8)[0]
    if spool_id:
        out["spool_id"] = spool_id
    if _u16(d, p + 12):
        out["density"] = round(_u16(d, p + 12) / 1000.0, 3)
    if _u16(d, p + 14):
        out["drying_temp_c"] = _u16(d, p + 14)
    if _u16(d, p + 16):
        out["drying_time_h"] = _u16(d, p + 16)
    return out


def encode_tag_payload(weight_g=None, spool_id=None, density=None,
                       drying_temp_c=None, drying_time_h=None,
                       **anycubic) -> bytes:
    """
    The whole record for a blank NTAG: Anycubic layout plus the AFC block.

    One contiguous 144-byte payload written from page 4, ending at page 39.
    The Anycubic half still carries an approximate weight-as-length so a reader
    that knows nothing of the AFC block degrades to the nearest standard spool
    size rather than to nothing.

    :param weight_g: exact net filament weight in grams
    :param spool_id: Spoolman spool id this tag belongs to
    :param density: g/cm^3
    :param drying_temp_c: drying temperature
    :param drying_time_h: drying time in hours
    :return bytes: 144 bytes to write at ANYCUBIC_FIRST_PAGE
    """
    return (encode_anycubic(weight_g=weight_g, **anycubic)
            + encode_afc_block(weight_g=weight_g, spool_id=spool_id,
                               density=density, drying_temp_c=drying_temp_c,
                               drying_time_h=drying_time_h))


# ── Snapmaker U1 (MIFARE Classic, HKDF-derived keys), port of DnG-Crafts U1 ──
# Per-tag sector keys: PRK = HMAC-SHA256(salt, UID); sector key A =
# HMAC-SHA256(PRK, b"key_a_<sector>\x01")[:6], one HKDF-expand block, exactly
# what hkdf_sha256() computes for length 6. (RSA signature check is skipped.)
SNAPMAKER_SALT_A = b"Snapmaker_qwertyuiop[,.;]"
SNAPMAKER_SALT_B = b"Snapmaker_qwertyuiop[,.;]_1q2w3e"
SNAPMAKER_MAIN_TYPE = {1: "PLA", 2: "PETG", 3: "ABS", 4: "TPU", 5: "PVA"}
SNAPMAKER_SUB_TYPE = {1: "Basic", 2: "Matte", 3: "SnapSpeed", 4: "Silk",
                      5: "Support", 6: "HF", 7: "95A", 8: "95A-HF"}
# Blocks holding the fields decode_snapmaker reads (vendor@16=blk1,
# manufacturer@32=blk2, version/type/color-nums/alpha@64=blk4, RGB@80=blk5,
# SKU@96=blk6, diameter/weight@128=blk8, temps@144=blk9, mfg-date@160=blk10).
SNAPMAKER_READ_BLOCKS = (1, 2, 4, 5, 6, 8, 9, 10)


def snapmaker_keys(uid: bytes) -> List[List[int]]:
    """
    Derive the 16 sector A keys for a Snapmaker tag.

    :param uid: 4-byte tag UID
    :return list: 16 six-byte keys
    """
    return [list(hkdf_sha256(SNAPMAKER_SALT_A, uid,
                             (f"key_a_{s}").encode(), 6)) for s in range(16)]


def decode_snapmaker(d: bytes) -> Optional[dict]:
    """Decode a Snapmaker U1 MIFARE Classic 1K image. Offsets are linear into the
    1024-byte image (sector*64 + block*16 + byte). Returns None if the main
    filament type isn't a known Snapmaker code (i.e. not a Snapmaker tag)."""
    if len(d) < 176:
        return None
    main = SNAPMAKER_MAIN_TYPE.get(_u16(d, 66))
    if main is None:                                 # wrong keys / not Snapmaker
        return None
    alpha = 0xFF - d[73]
    rgb1 = (d[80] << 16) | (d[81] << 8) | d[82]
    sku = d[96] | (d[97] << 8) | (d[98] << 16) | (d[99] << 24)
    diameter = _u16(d, 128)
    hot_max, hot_min = _u16(d, 148), _u16(d, 150)
    return dict(
        manufacturer=_s(d, 16, 16) or "Snapmaker",
        type=main, detailed=SNAPMAKER_SUB_TYPE.get(_u16(d, 68), ""),
        color_argb=(alpha << 24) | rgb1,
        weight_g=_u16(d, 130) or 1000,
        diameter_mm=round(diameter / 100.0, 2) if diameter else 1.75,
        hotend_max_c=hot_max or None, hotend_min_c=hot_min or None,
        bed_temp_c=_u16(d, 154) or None,
        sku=str(sku) if sku else "", production=_s(d, 160, 8),
    )


# ── AES-128 (pure Python, dependency-free), for Creality CFS tags ─────
# klippy-env has no working crypto lib, so bundle a small AES-128: Creality
# derives the MIFARE key as AES(UID) and stores the payload as AES-128-CBC.
_AES_SBOX = bytes.fromhex(
    "637c777bf26b6fc53001672bfed7ab76ca82c97dfa5947f0add4a2af9ca472c0"
    "b7fd9326363ff7cc34a5e5f171d8311504c723c31896059a071280e2eb27b275"
    "09832c1a1b6e5aa0523bd6b329e32f8453d100ed20fcb15b6acbbe394a4c58cf"
    "d0efaafb434d338545f9027f503c9fa851a3408f929d38f5bcb6da2110fff3d2"
    "cd0c13ec5f974417c4a77e3d645d197360814fdc222a908846eeb814de5e0bdb"
    "e0323a0a4906245cc2d3ac629195e479e7c8376d8dd54ea96c56f4ea657aae08"
    "ba78252e1ca6b4c6e8dd741f4bbd8b8a703eb5664803f60e613557b986c11d9e"
    "e1f8981169d98e949b1e87e9ce5528df8ca1890dbfe6426841992d0fb054bb16")
_AES_INV_SBOX = bytes(_AES_SBOX.index(i) for i in range(256))
_AES_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36)


def _aes_gmul(a: int, b: int) -> int:
    """
    GF(2^8) multiply for AES MixColumns.

    :param a: first operand
    :param b: second operand
    :return int: the product
    """
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        hi = a & 0x80
        a = (a << 1) & 0xFF
        if hi:
            a ^= 0x1B
        b >>= 1
    return p & 0xFF


def _aes_expand(key: bytes) -> List[List[int]]:
    """
    AES-128 key schedule.

    :param key: 16-byte key
    :return list: 44 round-key words
    """
    w = [list(key[i:i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = [_AES_SBOX[b] for b in t[1:] + t[:1]]
            t[0] ^= _AES_RCON[i // 4 - 1]
        w.append([w[i - 4][j] ^ t[j] for j in range(4)])
    return w


def _aes_ark(s: List[List[int]], w: List[List[int]], rnd: int) -> None:
    """
    AES AddRoundKey, in place.

    :param s: state columns
    :param w: expanded key
    :param rnd: round number
    """
    for c in range(4):
        for r in range(4):
            s[r][c] ^= w[rnd * 4 + c][r]


def _aes_encrypt_block(pt: bytes, key: bytes) -> bytes:
    """
    AES-128 encrypt one block.

    :param pt: 16-byte plaintext
    :param key: 16-byte key
    :return bytes: the ciphertext block
    """
    w = _aes_expand(key)
    s = [[pt[r + 4 * c] for c in range(4)] for r in range(4)]
    _aes_ark(s, w, 0)
    for rnd in range(1, 10):
        for r in range(4):
            for c in range(4):
                s[r][c] = _AES_SBOX[s[r][c]]
        for r in range(1, 4):
            s[r] = s[r][r:] + s[r][:r]
        for c in range(4):
            a = [s[r][c] for r in range(4)]
            s[0][c] = _aes_gmul(a[0], 2) ^ _aes_gmul(a[1], 3) ^ a[2] ^ a[3]
            s[1][c] = a[0] ^ _aes_gmul(a[1], 2) ^ _aes_gmul(a[2], 3) ^ a[3]
            s[2][c] = a[0] ^ a[1] ^ _aes_gmul(a[2], 2) ^ _aes_gmul(a[3], 3)
            s[3][c] = _aes_gmul(a[0], 3) ^ a[1] ^ a[2] ^ _aes_gmul(a[3], 2)
        _aes_ark(s, w, rnd)
    for r in range(4):
        for c in range(4):
            s[r][c] = _AES_SBOX[s[r][c]]
    for r in range(1, 4):
        s[r] = s[r][r:] + s[r][:r]
    _aes_ark(s, w, 10)
    return bytes(s[r][c] for c in range(4) for r in range(4))


def _aes_decrypt_block(ct: bytes, key: bytes) -> bytes:
    """
    AES-128 decrypt one block.

    :param ct: 16-byte ciphertext
    :param key: 16-byte key
    :return bytes: the plaintext block
    """
    w = _aes_expand(key)
    s = [[ct[r + 4 * c] for c in range(4)] for r in range(4)]
    _aes_ark(s, w, 10)
    for rnd in range(9, 0, -1):
        for r in range(1, 4):
            s[r] = s[r][-r:] + s[r][:-r]
        for r in range(4):
            for c in range(4):
                s[r][c] = _AES_INV_SBOX[s[r][c]]
        _aes_ark(s, w, rnd)
        for c in range(4):
            a = [s[r][c] for r in range(4)]
            s[0][c] = (_aes_gmul(a[0], 14) ^ _aes_gmul(a[1], 11)
                       ^ _aes_gmul(a[2], 13) ^ _aes_gmul(a[3], 9))
            s[1][c] = (_aes_gmul(a[0], 9) ^ _aes_gmul(a[1], 14)
                       ^ _aes_gmul(a[2], 11) ^ _aes_gmul(a[3], 13))
            s[2][c] = (_aes_gmul(a[0], 13) ^ _aes_gmul(a[1], 9)
                       ^ _aes_gmul(a[2], 14) ^ _aes_gmul(a[3], 11))
            s[3][c] = (_aes_gmul(a[0], 11) ^ _aes_gmul(a[1], 13)
                       ^ _aes_gmul(a[2], 9) ^ _aes_gmul(a[3], 14))
    for r in range(1, 4):
        s[r] = s[r][-r:] + s[r][:-r]
    for r in range(4):
        for c in range(4):
            s[r][c] = _AES_INV_SBOX[s[r][c]]
    _aes_ark(s, w, 0)
    return bytes(s[r][c] for c in range(4) for r in range(4))


def _aes_cbc_decrypt(data: bytes, key: bytes, iv: bytes = b"\x00" * 16) -> bytes:
    """
    AES-128-CBC decrypt the whole blocks of a buffer.

    :param data: ciphertext
    :param key: 16-byte key
    :param iv: initialisation vector
    :return bytes: the plaintext
    """
    out, prev = bytearray(), iv
    for i in range(0, len(data) - len(data) % 16, 16):
        blk = data[i:i + 16]
        out += bytes(d ^ p for d, p in zip(_aes_decrypt_block(blk, key), prev))
        prev = blk
    return bytes(out)


# ── Creality CFS (MIFARE Classic, UID-derived key + AES-CBC payload) ─────
# The 48-byte CFS payload is
# in sector 1 (blocks 4-6); the MIFARE Key A is AES(UID-repeated-to-16) with the
# u_key; the payload is AES-128-CBC (zero IV) with the d_key. Both keys are set
# in config (creality_key = u_key, creality_encryption_key = d_key).
CREALITY_READ_BLOCKS = (4, 5, 6)
CREALITY_FILM_ID = {"101001": "PLA", "101002": "PETG", "101003": "ABS",
                    "101004": "TPU", "101005": "PA", "E00003": "PLA"}
CREALITY_LENGTH_WEIGHT = {130: 250, 357: 500, 408: 600, 583: 750, 816: 1000}


def creality_mifare_key(uid: bytes, u_key: bytes) -> bytes:
    """Derive the 6-byte MIFARE Key A: AES(UID repeated to 16 bytes) with u_key,
    first 6 bytes."""
    uid16 = bytes(uid[i % len(uid)] for i in range(16))
    return _aes_encrypt_block(uid16, bytes(u_key))[:6]


def decode_creality(data48: bytes) -> Optional[dict]:
    """Decode the 48-byte DECRYPTED CFS payload (ASCII fields). Returns None when
    it doesn't look like a Creality tag (wrong keys give garbage)."""
    if len(data48) < 34:
        return None
    s = data48.decode("ascii", "replace")
    vendor = s[5:9]
    film = s[11:17]
    material = CREALITY_FILM_ID.get(film, "")
    if vendor != "0276" and not material:            # not a Creality payload
        return None
    color = s[17:24]                                 # "0RRGGBB"
    argb = None
    try:
        argb = 0xFF000000 | (int(color[1:7], 16) & 0xFFFFFF)
    except ValueError:
        pass
    try:
        length_m = int(s[24:28], 16)
    except ValueError:
        length_m = 0
    return dict(
        manufacturer="Creality", type=material, detailed="",
        color_argb=argb, sku=film.strip("\x00 ") or "",
        diameter_mm=1.75,                            # not on the tag
        weight_g=CREALITY_LENGTH_WEIGHT.get(length_m),
        length_m=length_m or None,
        serial=s[28:34], production=s[0:5],
    )


# ── Elegoo (NTAG213, plain EPC-256, no auth), ELEGOO-3D/ELEGOO-RFID-Tag-Guide ─
# User memory starts at NTAG page 4 (byte 16 of a page-0 read). Fields are packed
# big-endian from there: header@16, mfr@17, code@21, material@23, subtype@27,
# color@31, diameter@34, weight@36, date@38.
ELEGOO_HEADER = 0x36
ELEGOO_MFR = b"\xee\xee\xee\xee"


def decode_elegoo(d: bytes) -> Optional[dict]:
    """
    Decode an Elegoo tag image.

    :param d: tag image
    :return Optional[dict]: decoded fields, None without the Elegoo header
    """
    if len(d) < 40 or d[16] != ELEGOO_HEADER or d[17:21] != ELEGOO_MFR:
        return None
    rgb = (d[31] << 16) | (d[32] << 8) | d[33]
    return dict(
        manufacturer="Elegoo",
        type=_s(d, 23, 4).strip(), detailed=_s(d, 27, 4).strip(),
        color_argb=0xFF000000 | rgb,
        diameter_mm=round(((d[34] << 8) | d[35]) / 100.0, 2),
        weight_g=(d[36] << 8) | d[37],
        production=f"{(d[38] << 8) | d[39]:04d}",
    )


# ── unified entry: read UID for ANY tag, decode brand when we can ──────
SAK_CLASSIC_1K = 0x08
SAK_ULTRALIGHT = 0x00
# The exact blocks decode_bambu reads (type@32=blk2, detailed@64=blk4,
# color/weight/diameter@80=blk5, temps@96=blk6, nozzle@140=blk8, tray_uid@144=blk9,
# spool-width@164=blk10, production@192=blk12, length@228=blk14). Reading only
# these (vs all 16 blocks) still roughly halves the transceives, a big deal
# over the slow serial passthrough.
BAMBU_READ_BLOCKS = (2, 4, 5, 6, 8, 9, 10, 12, 14)
# Max tags to enumerate/halt in one shared-reader activation (2 slots per
# reader, so at most the neighbour + this lane's tag; a little slack).
MAX_FIELD_TAGS = 4


def _bambu_apply_multicolor(fil: dict, block16: Optional[bytes]) -> None:
    """Fill fil['color_count'] and fil['colors_argb'] (list, ARGB) from a Bambu
    block-16 image. Per the Bambu-Research-Group RFID-Tag-Guide, block 16 (image
    bytes 256-271) holds: Color Count @258 (u16 LE) and the Second colour @260 in
    REVERSED ABGR order (bytes A,B,G,R). Single colour (or missing block) leaves
    just the primary. block16 is a 1024-byte image with block 16 populated."""
    primary = fil.get("color_argb")
    colors = [primary] if primary is not None else []
    count = 1
    if block16 and len(block16) >= 264:
        count = _u16(block16, 258) or 1
        if count >= 2:
            a2, b2, g2, r2 = (block16[260], block16[261],
                              block16[262], block16[263])
            if a2 | b2 | g2 | r2:                 # block 16 actually present
                colors.append((a2 << 24) | (r2 << 16) | (g2 << 8) | b2)
    fil["color_count"] = count
    fil["colors_argb"] = colors


def _classic_bambu(mc: MifareClassic, uid: bytes, master_key: bytes,
                   blocks: Any) -> Optional[dict]:
    """
    Read and decode a Bambu Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :param master_key: Bambu master key
    :param blocks: blocks to read
    :return Optional[dict]: decoded fields, None when the read fails
    """
    keys = bambu_keys(uid, master_key)
    data = mc.read_blocks(uid, keys, blocks)
    if not data:
        return None
    fil = decode_bambu(data)
    # The SECOND colour of a dual/gradient Bambu spool lives in block 16, a
    # different sector than the primary blocks. Read it best-effort (separate
    # read) so a sector-4 auth hiccup can never break the primary decode; a
    # single-colour tag just yields color_count=1.
    try:
        block16 = mc.read_blocks(uid, keys, (16,))
    except Exception:
        block16 = None
    _bambu_apply_multicolor(fil, block16)
    return fil


def _classic_snapmaker(mc: MifareClassic, uid: bytes) -> Optional[dict]:
    """
    Read and decode a Snapmaker Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :return Optional[dict]: decoded fields, None when the read fails
    """
    data = mc.read_blocks(uid, snapmaker_keys(uid), SNAPMAKER_READ_BLOCKS)
    return decode_snapmaker(data) if data else None


def _classic_creality(mc: MifareClassic, uid: bytes, u_key: bytes,
                      d_key: bytes) -> Optional[dict]:
    """
    Read and decode a Creality Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :param u_key: Creality UID key
    :param d_key: Creality data key
    :return Optional[dict]: decoded fields, None when the read fails
    """
    keyA = creality_mifare_key(uid, u_key)
    data = mc.read_blocks(uid, [list(keyA)] * 16, CREALITY_READ_BLOCKS)
    if not data:
        return None
    return decode_creality(_aes_cbc_decrypt(data[64:112], bytes(d_key)))


# ── BigTreeTech MMS / ViViD "BQ Tech" (MIFARE Classic, default FF Key A) ──────
# Port of bigtreetech/BIGTREETECH_MMS klippy/extras/mms/hardware/mfrc522.py:
# MIFARE Classic 1K, tag_version 1000, every sector under the default key
# FFFFFFFFFFFF. Fields are ASCII and little-endian u16 (BTT's nibble offsets
# halved to bytes); color_code is a raw RRGGBB triple. tag_version==1000 is
# the fingerprint, so trying this scheme cannot mis-decode other tags.
# Shared by the ACE2 and ViViD readers.
BTT_DEFAULT_KEY = (0xFF,) * 6
# Blocks decode_btt reads: version+manufacturer@blk1, mfg-date@blk2, material@blk4,
# detailed@blk5, serial@blk6, colour@blk8, diameter/density@blk10, spool
# weight@blk17, temps@blk18, bed temp@blk20.
BTT_READ_BLOCKS = (1, 2, 4, 5, 6, 8, 10, 17, 18, 20)


def btt_keys() -> List[List[int]]:
    """The 16 per-sector Key-A entries for a BQ Tech tag, all the default FF."""
    return [list(BTT_DEFAULT_KEY) for _ in range(16)]


def decode_btt(d: bytes) -> Optional[dict]:
    """Decode a BigTreeTech MMS "BQ Tech" MIFARE Classic 1K image (block N at
    image byte N*16). Returns None unless tag_version (block 1, LE u16) is 1000,
    the format fingerprint that stops a non-BTT tag (read with the default FF key)
    from being mis-decoded. Fields are LE u16 ints / ASCII strings; color_code is
    a raw RRGGBB triple (single colour, no alpha)."""
    if len(d) < 336:                                  # need through block 20
        return None
    if _u16(d, 16) != 1000:                           # tag_version fingerprint
        return None
    r, g, b = d[128], d[129], d[130]                  # color_code RRGGBB @ blk8
    diameter = _u16(d, 160)                            # 1750 -> 1.750 mm
    density = _u16(d, 162)                             # 1240 -> 1.240 g/cm^3
    hot_min = _u16(d, 298) or None                     # printing_temperature_min
    hot_max = _u16(d, 300) or None                     # printing_temperature_max
    bed = _u16(d, 320) or _u16(d, 296) or None         # bed_temperature / _max
    return dict(
        manufacturer=_s(d, 18, 14) or "BQ Tech",       # filament_manufacturer
        type=_s(d, 64, 16),                            # filament_material_type
        detailed=_s(d, 80, 16),                        # filament_type_detailed
        color_argb=(0xFF << 24) | (r << 16) | (g << 8) | b,
        weight_g=_u16(d, 272) or None,                 # spool_weight
        diameter_mm=round(diameter / 1000.0, 3) if diameter else 1.75,
        density=round(density / 1000.0, 3) if density else None,
        hotend_min_c=hot_min, hotend_max_c=hot_max,
        bed_temp_c=bed,
        drying_time_h=_u16(d, 288) or None,            # drying_time (hours)
        drying_temp_c=_u16(d, 292) or None,            # drying_temp_max (C)
        sku=_s(d, 96, 16),                             # serial_number
        production=_s(d, 32, 16),                       # manufacture_datetime
    )


def _classic_btt(mc: MifareClassic, uid: bytes) -> Optional[dict]:
    """
    Read and decode a BTT Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :return Optional[dict]: decoded fields, None when the read fails
    """
    data = mc.read_blocks(uid, btt_keys(), BTT_READ_BLOCKS)
    return decode_btt(data) if data else None


def read_tag(link: Any, bambu_master_key: Optional[bytes] = None,
             bambu_blocks: Any = BAMBU_READ_BLOCKS,
             is_excluded: Optional[Callable[[str], bool]] = None,
             creality_key: Optional[bytes] = None,
             creality_encryption_key: Optional[bytes] = None,
             seen: Optional[List[Any]] = None,
             dump_blocks: Any = None) -> Optional[dict]:
    """Activate an ISO14443-A tag; return its UID + tag type, and the decoded
    filament when we recognise the layout. MIFARE Classic is tried as Bambu (if a
    master key is set), Snapmaker (public salts), then Creality (if its keys are
    set); NTAG is tried as Anycubic then Elegoo. ``is_excluded`` (uid_hex -> bool)
    halts neighbour tags on a shared reader so this lane's own tag is read.
    ``seen`` (optional list) collects every detected (uid, sak, excluded) for
    read diagnostics."""
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate(is_excluded=is_excluded, seen=seen)
    if uid is None:
        return None
    res = {"uid": uid.hex(), "sak": sak, "tag_type": None, "filament": None}

    if sak & 0x08:                                   # MIFARE Classic
        res["tag_type"] = "MifareClassic1k"
        schemes = []
        if bambu_master_key:
            schemes.append(lambda u: _classic_bambu(mc, u, bambu_master_key,
                                                    bambu_blocks))
        schemes.append(lambda u: _classic_snapmaker(mc, u))
        if creality_key and creality_encryption_key:
            schemes.append(lambda u: _classic_creality(
                mc, u, creality_key, creality_encryption_key))
        # BigTreeTech MMS / ViViD "BQ Tech" tags, default FF Key A, no config
        # key needed, so always tried. Fingerprinted by tag_version==1000, so it
        # never mis-decodes a Bambu/Snapmaker/Creality tag (whose data sectors
        # won't authenticate with the FF key anyway). Last so the keyed schemes
        # win first for their own tags.
        schemes.append(lambda u: _classic_btt(mc, u))
        # A failed MIFARE auth deauthenticates the card, so re-select it before
        # each subsequent scheme. Pin the re-selection to THIS tag's UID, the
        # one the first activate already confirmed is the active lane's, having
        # passed is_excluded. Re-running the sibling predicate here can halt the
        # active tag itself (its UID may resolve to the sibling's spool), which
        # broke Snapmaker: it is only reached on the 2nd scheme (after Bambu
        # fails), so its re-select got excluded while Bambu, matched on the 1st
        # scheme, no re-select, kept working. Pinning still halts the neighbour
        # (anything != this UID) so shared-reader dedup is preserved.
        first_uid_hex = uid.hex()

        def _not_this_tag(h: str) -> bool:
            """
            Whether a UID differs from the first one seen.

            :param h: UID as hex
            :return bool: True when it is a different tag
            """
            return h != first_uid_hex

        for i, scheme in enumerate(schemes):
            u = uid
            if i:
                # Re-select WITHOUT a SoftReset (reset=False): keep the RF field
                # up so a marginal at-rest tag isn't unpowered between schemes,
                # a field drop here is what made Snapmaker/Creality (2nd/3rd
                # scheme) fail to decode at rest while Bambu (1st, no re-select)
                # read fine.
                u = mc.activate(is_excluded=_not_this_tag, seen=seen,
                                reset=False)[0]
                if u is None:
                    break
                res["uid"] = u.hex()
            fil = scheme(u)
            if fil:
                res["filament"] = fil
                break
        # Raw block dump (ACE_RFID_BLOCKS diagnostic): re-select the card and read
        # the requested blocks with the Bambu keys, attaching {block: hex}. Lets
        # us inspect e.g. block 16 (colour count @258, second colour @260) to
        # confirm whether a tag is really dual-colour. Best-effort.
        if dump_blocks and bambu_master_key:
            try:
                u2 = mc.activate(reset=True)[0]
                if u2 is not None:
                    img = mc.read_blocks(u2, bambu_keys(u2, bambu_master_key),
                                         dump_blocks)
                    if img:
                        res["raw_blocks"] = {
                            b: img[b * 16:b * 16 + 16].hex() for b in dump_blocks}
            except Exception:
                pass
    else:                                            # Ultralight / NTAG
        res["tag_type"] = "MifareUltralight"
        data = mc.read_ntag(0x80)
        if data:
            # Which chip, and how much room a write has. Free here: the read
            # starts at page 0, so the CC is already in hand.
            res["chip"], res["user_bytes"] = ntag_chip(data)
            fil = decode_anycubic(data) or decode_elegoo(data)
            # The AFC block is probed SEPARATELY and best-effort. Folding it
            # into the main read instead would mean demanding 160 bytes from
            # every NTAG, and a plain Ultralight only has 80 -- the read would
            # fail outright and a genuine Anycubic tag would stop decoding.
            ext = mc.read_ntag(AFC_BLOCK_LEN, start_page=AFC_BLOCK_PAGE)
            afc = decode_afc_block(ext, offset=0) if ext else None
            if afc:
                # Ours: the exact weight and the Spoolman id override what the
                # brand layout could only approximate.
                fil = dict(fil or {})
                fil.update(afc)
            res["filament"] = fil
    return res


#: Blocks that must never be written on a Classic tag. Every 4th block is the
#: sector trailer (keys + access bits) -- writing it can lock the sector for
#: good -- and block 0 is the manufacturer block.
def _is_writable_classic_block(block: int) -> bool:
    return block > 0 and (block % 4) != 3


CLASSIC_DEFAULT_KEY = b"\xff" * 6      # a blank / magic card ships with this


def classic_write_block(link: Any, block: int, data16: bytes,
                        key6: bytes = CLASSIC_DEFAULT_KEY,
                        key_type: int = PICC_AUTH_KEY_A
                        ) -> Tuple[Optional[str], Optional[str]]:
    """
    Write one 16-byte data block of a MIFARE Classic tag and read it back.

    Authenticates the block's sector with ``key6`` (default the blank-card
    key), writes, then reads to confirm. Refuses a sector trailer or the
    manufacturer block -- those carry the keys and access bits and a wrong
    write locks the sector for good.

    :param link: reader link exposing reg_read / reg_write
    :param block: absolute block number (a data block)
    :param data16: exactly 16 bytes
    :param key6: 6-byte sector key
    :param key_type: PICC_AUTH_KEY_A or PICC_AUTH_KEY_B
    :return tuple: (uid hex or None, error or None)
    """
    if len(data16) != 16:
        return None, f"data is {len(data16)} bytes, must be 16"
    if not _is_writable_classic_block(block):
        return None, (f"block {block} is a sector trailer or block 0; refusing "
                      f"to write it")
    mfrc = Mfrc522(link)
    mc = MifareClassic(mfrc)
    uid, sak = mc.activate()
    if uid is None:
        return None, "no tag in the reader's field"
    if not sak & SAK_CLASSIC_1K:
        return uid.hex(), (f"that is not a MIFARE Classic tag "
                           f"(sak 0x{sak:02X})")
    if not mfrc.auth(key_type, (block // 4) * 4, bytes(key6), uid):
        mfrc.stop_crypto()
        return uid.hex(), (f"key did not authenticate sector {block // 4} "
                           f"(wrong key for this card?)")
    ok = mfrc.write_classic_block(block, data16)
    back = mfrc.read_block(block)
    mfrc.stop_crypto()
    if not ok:
        return uid.hex(), f"tag did not ACK the write to block {block}"
    if back != data16:
        return uid.hex(), (f"block {block} reads back {back.hex() if back else None}"
                           f", not what was written")
    return uid.hex(), None


def bambu_classic_write_test(link: Any, master_key: bytes,
                             block: int = 13) -> dict:
    """
    Prove the MIFARE Classic write works on a genuine Bambu tag, WITHOUT
    permanently changing it.

    Reads a data block, writes it back with one byte flipped, reads to confirm
    the change took, then writes the original bytes back and confirms the tag
    is exactly as it was. So a full 0xA0 write is exercised on real data, and
    the tag is left untouched.

    Defaults to block 13, a data block no Bambu decoder reads, so even a failed
    restore would not disturb a field the AMS or our readers use. Refuses a
    sector trailer or the manufacturer block outright.

    :param link: reader link exposing reg_read / reg_write
    :param master_key: the Bambu master key
    :param block: absolute block to test on (a data block)
    :return dict: {uid, block, wrote, changed, restored, orig, modified, ...}
    """
    if not _is_writable_classic_block(block):
        return {"error": f"block {block} is a sector trailer or block 0; "
                         f"refusing to write it"}
    mfrc = Mfrc522(link)
    mc = MifareClassic(mfrc)
    uid, sak = mc.activate()
    if uid is None:
        return {"error": "no tag in the field"}
    if not sak & SAK_CLASSIC_1K:
        return {"error": f"not a MIFARE Classic tag (sak 0x{sak:02X})",
                "uid": uid.hex()}
    sector = block // 4
    key6 = bytes(bambu_keys(uid, master_key)[sector])
    if not mfrc.auth(PICC_AUTH_KEY_A, sector * 4, key6, uid):
        mfrc.stop_crypto()
        return {"error": f"Key A did not authenticate sector {sector}",
                "uid": uid.hex()}
    orig = mfrc.read_block(block)
    if orig is None:
        mfrc.stop_crypto()
        return {"error": f"could not read block {block}", "uid": uid.hex()}
    # The sector trailer's access bytes (6-8) say whether Key A may WRITE the
    # data blocks. Keys read back as zero; the access bits are readable and
    # tell us if a write NAK is a lock (Bambu's doing) rather than a bad frame.
    trailer = mfrc.read_block(sector * 4 + 3)
    access = trailer[6:9].hex() if trailer is not None else None
    modified = bytes([orig[0] ^ 0xFF]) + orig[1:]
    out = {"uid": uid.hex(), "block": block, "orig": orig.hex(),
           "modified": modified.hex(), "access": access}
    # Write the flipped copy and confirm it landed.
    out["wrote"] = mfrc.write_classic_block(block, modified)
    back = mfrc.read_block(block)
    out["changed"] = back == modified
    out["read_after_write"] = back.hex() if back is not None else None
    # Restore the original, always, and confirm.
    out["restored_ack"] = mfrc.write_classic_block(block, orig)
    final = mfrc.read_block(block)
    out["restored"] = final == orig
    out["read_after_restore"] = final.hex() if final is not None else None
    mfrc.stop_crypto()
    return out


def read_bambu(link: Any, master_key: bytes) -> Optional[dict]:
    """Convenience: Bambu-only read (used by tests)."""
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate()
    if uid is None:
        return None
    data = mc.read_all(uid, bambu_keys(uid, master_key))
    if data is None:
        return None
    out = decode_bambu(data)
    out["uid"] = uid.hex()
    return out


def write_tag(link: Any, payload: bytes,
              is_excluded: Optional[Callable[[str], bool]] = None,
              ) -> Tuple[Optional[str], Optional[str]]:
    """
    Write an Anycubic-layout record to a blank NTAG in the reader's field.

    The counterpart to read_tag: activate whatever is there, refuse anything
    that is not an NTAG, then write and verify. A MIFARE Classic tag is
    rejected by name rather than attempted, because the Classic write is a
    different opcode behind Crypto1 and the keys for a Bambu or Snapmaker tag
    are not ours to forge.

    :param link: reader link exposing reg_read / reg_write
    :param payload: bytes from encode_anycubic
    :param is_excluded: uid_hex -> bool, halts a neighbour tag on a shared
        reader so this station's own tag is the one written
    :return tuple: (uid hex or None, error message or None)
    """
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate(is_excluded=is_excluded)
    if uid is None:
        return None, "no tag in the reader's field"
    if sak & SAK_CLASSIC_1K:
        return uid.hex(), (
            "that is a MIFARE Classic tag, not an NTAG; this writer only "
            "programs blank NTAG stickers")
    return uid.hex(), mc.write_ntag(ANYCUBIC_FIRST_PAGE, payload)
