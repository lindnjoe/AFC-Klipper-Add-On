# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# ACE Pro 2 RFID integration — a single self-contained module.
#
# Contains, bottom-to-top:
#   Ace2Link      ACE2 V2 serial frame + the firmware passthrough commands:
#                 MFRC522 register read / write and (v2) reader power.
#   Mfrc522       MFRC522 primitives on top of reg r/w: FIFO, transceive,
#                 antenna, MIFARE authenticate.
#   MifareClassic REQA/anticoll/select (UID) -> auth+read -> full 1K image.
#   Bambu/Anycubic HKDF-SHA256 key derivation + block/page -> field decode.
#   AFC_ACE2_RFID  the Klipper [AFC_ACE2_rfid] object: read a lane/slot tag and
#                 apply the decoded filament to an AFC lane (+ optional Spoolman).
#
# The firmware only ever exposes reg_read(reg)->byte, reg_write(reg,val) and
# reader_power(on); every bit of MIFARE/crypto/brand logic lives here and stays
# editable without reflashing.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import traceback
from configparser import Error as error
import hashlib
import hmac
import logging
import struct
from typing import (TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple)

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_RFID import (sync_rfid_to_spoolman,
    build_filament_name, prompt_hold_spool,
    dismiss_prompt, AFCUnitRFID,
    map_tag_to_slot_info,
    get_auto_spoolman_create, resolve_rfid_keys)
except: raise error(ERROR_STR.format(import_lib="AFC_RFID", trace=traceback.format_exc()))


# ═════════════════════════════
# Host reader stack — pure logic on top of the firmware reg r/w + reader-power
# passthrough. No Klipper dependencies below this point until the integration
# class; the register/crypto/brand code stays unit-testable in isolation.
# ═════════════════════════════

# ── ACE2 V2 frame ──────────────────────
PREAMBLE = b"\xff\xaa"
END = b"\xfe"


def crc16_kermit(data: bytes, init: int = 0xFFFF) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if (c & 1) else c >> 1
    return c & 0xFFFF


# Opcodes for the passthrough commands, registered in firmware.
CMD_MFRC522_REG_READ = 0x50
CMD_MFRC522_REG_WRITE = 0x51
CMD_MFRC522_READER_POWER = 0x52     # v2: host-owned reader power (Bambu reads)


def _varint(n: int) -> bytes:
    """protobuf base-128 varint (unsigned)."""
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)


def _varint_decode(data: bytes, i: int = 0) -> Tuple[int, int]:
    result = shift = 0
    while True:
        b = data[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, i
        shift += 7


class Ace2Link:
    """Frames the ACE2 protocol and issues the passthrough commands.

    `transport(frame:bytes) -> bytes` sends a framed request and returns the
    raw response frame (whatever your serial layer does). For unit tests, a
    fake transport / a fake Mfrc522 is injected higher up instead.
    """
    def __init__(self, transport: Callable[[bytes], bytes], slot: int = 0,
                 ftype: int = 0x0000) -> None:
        self._tx = transport
        self._ftype = ftype
        self._seq = 0
        self.slot = slot            # which ACE2 reader slot (0..3) this link drives

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def build_frame(self, cmd: int, payload: bytes) -> bytes:
        body = struct.pack(">H", self._ftype) + bytes([cmd, len(payload)]) + payload
        crc = crc16_kermit(body)
        return PREAMBLE + bytes([self._next_seq()]) + body + struct.pack("<H", crc) + END

    # Firmware request/response are a single protobuf field #1 (varint):
    #   reg-read     req arg = (slot<<16) | reg               -> resp{val}
    #   reg-write    req arg = (slot<<16) | (reg<<8) | val     -> resp{} (empty)
    #   reader-power req arg = (reader_index<<16) | on          -> resp{} (empty)
    def reg_read(self, reg: int) -> int:
        arg = (self.slot << 16) | (reg & 0xFF)
        resp = self._tx(self.build_frame(CMD_MFRC522_REG_READ, b"\x08" + _varint(arg)))
        return self._parse_field1(resp)

    def reg_write(self, reg: int, val: int) -> None:
        arg = (self.slot << 16) | ((reg & 0xFF) << 8) | (val & 0xFF)
        self._tx(self.build_frame(CMD_MFRC522_REG_WRITE, b"\x08" + _varint(arg)))

    def reader_power(self, on: bool) -> None:
        arg = (self.slot << 16) | (1 if on else 0)
        self._tx(self.build_frame(CMD_MFRC522_READER_POWER, b"\x08" + _varint(arg)))

    @staticmethod
    def _parse_field1(resp: bytes) -> int:
        # response frame: FF AA seq TYPE(2) CMD LEN PAYLOAD CRC(2) FE
        # PAYLOAD is protobuf {field1: varint val} = 08 <varint>
        if not resp or resp[:2] != PREAMBLE:
            raise IOError("bad ACE2 response preamble")
        ln = resp[6]
        payload = resp[7:7 + ln]
        if len(payload) >= 2 and payload[0] == 0x08:
            return _varint_decode(payload, 1)[0] & 0xFF
        return 0


# ── MFRC522 register map (subset used) ───────────────
CommandReg, ComIrqReg, ErrorReg, Status2Reg = 0x01, 0x04, 0x06, 0x08
FIFODataReg, FIFOLevelReg, ControlReg, BitFramingReg, CollReg = 0x09, 0x0A, 0x0C, 0x0D, 0x0E
TxControlReg = 0x14
# commands
PCD_IDLE, PCD_AUTHENT, PCD_TRANSCEIVE, PCD_MFAUTHENT = 0x00, 0x0E, 0x0C, 0x0E
# PICC (card) commands
PICC_REQA, PICC_WUPA = 0x26, 0x52
PICC_ANTICOLL, PICC_SELECT = 0x93, 0x93
PICC_AUTH_KEY_A, PICC_AUTH_KEY_B = 0x60, 0x61
PICC_READ = 0x30


class Mfrc522:
    """MFRC522 primitives built on an Ace2Link (or any obj with reg_read/write)."""
    def __init__(self, link: Any) -> None:
        self.l = link

    def _set(self, reg: int, mask: int) -> None:
        self.l.reg_write(reg, self.l.reg_read(reg) | mask)

    def _clr(self, reg: int, mask: int) -> None:
        self.l.reg_write(reg, self.l.reg_read(reg) & (~mask & 0xFF))

    def reset(self) -> None:
        """Soft-reset + minimal timer/ASK/mode config.

        When the host powers the reader itself (host-owned power), the MFRC522
        comes up cold — the firmware normally does this bring-up during its own
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
        fresh anticoll/select works — WITHOUT a SoftReset. A reset drops the RF
        field, which unpowers a marginal at-rest tag being re-selected between
        decode schemes; clearing crypto keeps the field up so the tag survives."""
        self._clr(Status2Reg, 0x08)

    def antenna_on(self) -> None:
        if not (self.l.reg_read(TxControlReg) & 0x03):
            self._set(TxControlReg, 0x03)

    def _to_card(self, command: int, send: bytes,
                 tx_last_bits: int = 0) -> Tuple[bool, bytes, int]:
        """Execute Transceive/MFAuthent; return (ok, rx_bytes, rx_last_bits)."""
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
        if self.l.reg_read(ErrorReg) & 0x1B:               # buffer/coll/parity/proto
            return False, b"", 0
        rx = b""
        if command == PCD_TRANSCEIVE:
            n = self.l.reg_read(FIFOLevelReg)
            rx = bytes(self.l.reg_read(FIFODataReg) for _ in range(n))
            last = self.l.reg_read(ControlReg) & 0x07
            return True, rx, last
        return True, rx, 0

    def request(self, req: int = PICC_REQA) -> Optional[bytes]:
        self.l.reg_write(BitFramingReg, 0x07)              # 7-bit frame
        ok, atqa, _ = self._to_card(PCD_TRANSCEIVE, bytes([req]), 7)
        return atqa if ok and len(atqa) == 2 else None

    def anticoll(self) -> Optional[bytes]:
        self.l.reg_write(BitFramingReg, 0x00)
        ok, uid5, _ = self._to_card(PCD_TRANSCEIVE, bytes([PICC_ANTICOLL, 0x20]))
        if not ok or len(uid5) != 5:
            return None
        if uid5[0] ^ uid5[1] ^ uid5[2] ^ uid5[3] != uid5[4]:  # BCC
            return None
        return uid5[:4]

    def select(self, uid4: bytes) -> Optional[int]:
        """Return SAK byte (or None). SAK 0x00 = Ultralight/NTAG, 0x08 = Classic 1K."""
        buf = bytes([PICC_SELECT, 0x70]) + uid4 + bytes([uid4[0]^uid4[1]^uid4[2]^uid4[3]])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, sak, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return sak[0] if ok and len(sak) >= 1 else None

    def auth(self, key_type: int, block: int, key6: bytes, uid4: bytes) -> bool:
        send = bytes([key_type, block]) + key6 + uid4
        ok, _, _ = self._to_card(PCD_MFAUTHENT, send)
        return ok and bool(self.l.reg_read(Status2Reg) & 0x08)  # MFCrypto1On

    def read_block(self, block: int) -> Optional[bytes]:
        buf = bytes([PICC_READ, block])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, data, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return data[:16] if ok and len(data) >= 16 else None

    def halt(self) -> None:
        """HLTA — put the selected tag into HALT so it stops answering REQA (only
        WUPA wakes it). Used to silence a shared-reader neighbour tag so this
        lane's own tag can respond. HLTA returns no reply; the resulting
        timeout/NAK is the expected success signal, so we ignore the result."""
        buf = bytes([0x50, 0x00])
        buf += struct.pack("<H", self._crc_a(buf))
        self._to_card(PCD_TRANSCEIVE, buf)

    @staticmethod
    def _crc_a(data: bytes) -> int:
        # ISO14443-A CRC (poly 0x8408, init 0x6363) — MFRC522 computes this in
        # HW normally; done here since we drive the FIFO directly.
        c = 0x6363
        for b in data:
            b ^= c & 0xFF
            b = (b ^ (b << 4)) & 0xFF
            c = ((c >> 8) ^ (b << 8) ^ (b << 3) ^ (b >> 4)) & 0xFFFF
        return c


class MifareClassic:
    """Full-tag read: activate → per-sector auth+read → 1024-byte image."""
    def __init__(self, mfrc: Mfrc522) -> None:
        self.m = mfrc

    def activate(self, is_excluded: Optional[Callable[[str], bool]] = None,
                 seen: Optional[List[Any]] = None,
                 reset: bool = True) -> Tuple[Optional[bytes], Optional[int]]:
        """Return (uid, sak) for an ISO14443-A tag, or (None, None).

        With ``is_excluded`` (a ``uid_hex -> bool`` predicate) this handles a
        SHARED reader: if the tag it selects is excluded (a neighbour slot's tag
        bleeding into the antenna), HALT it so it stops answering REQA and
        request again, until this lane's own tag responds or the field is empty.
        The reader is reset once up front (never between tries — a reset would
        drop the RF field and wake the halted neighbour).

        ``reset=False`` RE-selects without a SoftReset: it keeps the RF field up
        (so a marginal at-rest tag isn't unpowered) and just ends any crypto
        session, for re-acquiring the SAME card between decode schemes.

        ``seen``, if given, is a list this appends ``(uid_hex, sak, excluded)``
        to for every tag it detects — a read diagnostic so callers can tell a
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
        return uid, sak

    def read_ntag(self, nbytes: int = 128) -> Optional[bytes]:
        """MIFARE Ultralight / NTAG read (no auth). READ page returns 4 pages."""
        data = bytearray()
        page = 0
        while len(data) < nbytes:
            blk = self.m.read_block(page)      # NTAG uses the same 0x30 READ
            if blk is None:
                return None
            data += blk
            page += 4                          # each read returns 4 pages (16 B)
        return bytes(data[:nbytes])

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


# ── Bambu (HKDF key derivation + decode) — port of OpenRFID ───────
def hkdf_sha256(salt: bytes, ikm: bytes, info: bytes, length: int) -> bytes:
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    okm, t, i = b"", b"", 0
    while len(okm) < length:
        i += 1
        t = hmac.new(prk, t + info + bytes([i]), hashlib.sha256).digest()
        okm += t
    return okm[:length]

BAMBU_SALT_HASH = "19cc3c63cb8802668800c3b3bf3fee05b3c59bf59fc5fd256b68e868084ec304"


def bambu_keys(uid: bytes, master_key: bytes) -> List[List[int]]:
    okm = hkdf_sha256(master_key, uid, b"RFID-A\x00", 6 * 16)
    return [list(okm[s * 6:(s + 1) * 6]) for s in range(16)]


def _s(d: bytes, p: int, n: int) -> str:
    return d[p:p + n].split(b"\x00")[0].decode("ascii", "replace")


def _u16(d: bytes, p: int) -> int:
    return struct.unpack_from("<H", d, p)[0]


def _f32(d: bytes, p: int) -> float:
    return struct.unpack_from("<f", d, p)[0]


def decode_bambu(d: bytes) -> dict:
    if len(d) != 1024:
        raise ValueError("expected 1024-byte MIFARE Classic 1K image")
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


# ── Anycubic (MIFARE Ultralight/NTAG, no auth) — port of OpenRFID/DnG-Crafts ──
ANYCUBIC_MAGIC = b"\x7b\x00\x65\x00"
_ANY_WEIGHT = {330: 1000, 247: 750, 198: 600, 165: 500, 82: 250}


def decode_anycubic(d: bytes) -> Optional[dict]:
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


# ── Snapmaker U1 (MIFARE Classic, HKDF-derived keys) — port of DnG-Crafts U1 ──
# Per-tag sector keys: PRK = HMAC-SHA256(salt, UID); sector key A =
# HMAC-SHA256(PRK, b"key_a_<sector>\x01")[:6] — one HKDF-expand block, exactly
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
    return [list(hkdf_sha256(SNAPMAKER_SALT_A, uid,
                             ("key_a_%d" % s).encode(), 6)) for s in range(16)]


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


# ── AES-128 (pure Python, dependency-free) — for Creality CFS tags ─────
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
    w = [list(key[i:i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = [_AES_SBOX[b] for b in t[1:] + t[:1]]
            t[0] ^= _AES_RCON[i // 4 - 1]
        w.append([w[i - 4][j] ^ t[j] for j in range(4)])
    return w


def _aes_ark(s: List[List[int]], w: List[List[int]], rnd: int) -> None:
    for c in range(4):
        for r in range(4):
            s[r][c] ^= w[rnd * 4 + c][r]


def _aes_encrypt_block(pt: bytes, key: bytes) -> bytes:
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


# ── Elegoo (NTAG213, plain EPC-256, no auth) — ELEGOO-3D/ELEGOO-RFID-Tag-Guide ─
# User memory starts at NTAG page 4 (byte 16 of a page-0 read). Fields are packed
# big-endian from there: header@16, mfr@17, code@21, material@23, subtype@27,
# color@31, diameter@34, weight@36, date@38.
ELEGOO_HEADER = 0x36
ELEGOO_MFR = b"\xee\xee\xee\xee"


def decode_elegoo(d: bytes) -> Optional[dict]:
    if len(d) < 40 or d[16] != ELEGOO_HEADER or d[17:21] != ELEGOO_MFR:
        return None
    rgb = (d[31] << 16) | (d[32] << 8) | d[33]
    return dict(
        manufacturer="Elegoo",
        type=_s(d, 23, 4).strip(), detailed=_s(d, 27, 4).strip(),
        color_argb=0xFF000000 | rgb,
        diameter_mm=round(((d[34] << 8) | d[35]) / 100.0, 2),
        weight_g=(d[36] << 8) | d[37],
        production="%04d" % ((d[38] << 8) | d[39]),
    )


# ── unified entry: read UID for ANY tag, decode brand when we can ──────
SAK_CLASSIC_1K = 0x08
SAK_ULTRALIGHT = 0x00
# The exact blocks decode_bambu reads (type@32=blk2, detailed@64=blk4,
# color/weight/diameter@80=blk5, temps@96=blk6, nozzle@140=blk8, tray_uid@144=blk9,
# spool-width@164=blk10, production@192=blk12, length@228=blk14). Reading only
# these (vs all 16 blocks) still roughly halves the transceives — a big deal
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
    data = mc.read_blocks(uid, snapmaker_keys(uid), SNAPMAKER_READ_BLOCKS)
    return decode_snapmaker(data) if data else None


def _classic_creality(mc: MifareClassic, uid: bytes, u_key: bytes,
                      d_key: bytes) -> Optional[dict]:
    keyA = creality_mifare_key(uid, u_key)
    data = mc.read_blocks(uid, [list(keyA)] * 16, CREALITY_READ_BLOCKS)
    if not data:
        return None
    return decode_creality(_aes_cbc_decrypt(data[64:112], bytes(d_key)))


# ── BigTreeTech MMS / ViViD "BQ Tech" (MIFARE Classic, default FF Key A) ──────
# Port of bigtreetech/BIGTREETECH_MMS klippy/extras/mms/hardware/mfrc522.py. The
# ViViD tags are MIFARE Classic 1K, tag_version 1000, and every sector uses the
# DEFAULT key FFFFFFFFFFFF (Key A) — no per-tag derivation. Fields are ASCII
# strings and little-endian u16 ints (block N lives at image byte N*16); the
# offsets in BTT's source are hex-character (nibble) counts, halved here to bytes.
# color_code is a raw RRGGBB triple (single colour, no alpha — a second colour is
# only a text NAME on the tag, so there's no dual-colour hex). The on-tag SHA256
# over blocks 0-59 (blocks 60-61) is an integrity check we don't require: a
# genuine BTT tag is fingerprinted by tag_version==1000, which a foreign tag read
# with the FF key won't carry, so trying this scheme can't mis-decode other tags.
# Shared so BOTH the ACE2 and the ViViD readers decode BQ Tech tags.
BTT_DEFAULT_KEY = (0xFF,) * 6
# Blocks decode_btt reads: version+manufacturer@blk1, mfg-date@blk2, material@blk4,
# detailed@blk5, serial@blk6, colour@blk8, diameter/density@blk10, spool
# weight@blk17, temps@blk18, bed temp@blk20.
BTT_READ_BLOCKS = (1, 2, 4, 5, 6, 8, 10, 17, 18, 20)


def btt_keys() -> List[List[int]]:
    """The 16 per-sector Key-A entries for a BQ Tech tag — all the default FF."""
    return [list(BTT_DEFAULT_KEY) for _ in range(16)]


def decode_btt(d: bytes) -> Optional[dict]:
    """Decode a BigTreeTech MMS "BQ Tech" MIFARE Classic 1K image (block N at
    image byte N*16). Returns None unless tag_version (block 1, LE u16) is 1000 —
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
        # BigTreeTech MMS / ViViD "BQ Tech" tags — default FF Key A, no config
        # key needed, so always tried. Fingerprinted by tag_version==1000, so it
        # never mis-decodes a Bambu/Snapmaker/Creality tag (whose data sectors
        # won't authenticate with the FF key anyway). Last so the keyed schemes
        # win first for their own tags.
        schemes.append(lambda u: _classic_btt(mc, u))
        # A failed MIFARE auth deauthenticates the card, so re-select it before
        # each subsequent scheme. Pin the re-selection to THIS tag's UID — the
        # one the first activate already confirmed is the active lane's, having
        # passed is_excluded. Re-running the sibling predicate here can halt the
        # active tag itself (its UID may resolve to the sibling's spool), which
        # broke Snapmaker: it is only reached on the 2nd scheme (after Bambu
        # fails), so its re-select got excluded while Bambu — matched on the 1st
        # scheme, no re-select — kept working. Pinning still halts the neighbour
        # (anything != this UID) so shared-reader dedup is preserved.
        first_uid_hex = uid.hex()

        def _not_this_tag(h: str) -> bool:
            return h != first_uid_hex

        for i, scheme in enumerate(schemes):
            u = uid
            if i:
                # Re-select WITHOUT a SoftReset (reset=False): keep the RF field
                # up so a marginal at-rest tag isn't unpowered between schemes —
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
            res["filament"] = decode_anycubic(data) or decode_elegoo(data)
    return res


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


# ═════════════════════════════
# Klipper integration — [AFC_ACE2_rfid]
# ═════════════════════════════

class _Ace2RegLink:
    """Adapts an ACE2 serial object to the reg_read/reg_write reader contract.

    Each read/write is one passthrough command over the existing ACE2 link:
      reg-read     -> send_command('mfrc522_reg_read',     {'arg': (slot<<16)|reg})
      reg-write    -> send_command('mfrc522_reg_write',    {'arg': (slot<<16)|(reg<<8)|val})
      reader-power -> send_command('mfrc522_reader_power', {'arg': (slot<<16)|on})
    """
    def __init__(self, ace2: Any, slot: int, power_index: Optional[int] = None,
                 reg_timeout: float = 2.0) -> None:
        self._ace2 = ace2            # the afcACE2 printer object
        # reg r/w chip-select index (each slot has its own MFRC522 on the shared
        # SPI2 bus, selected by this field).
        self.slot = slot & 0xFF
        # reader-power index. Power is per-reader-PAIR (PD12/PD13 cover slots
        # 0-1 / 2-3), so it can differ from the per-slot reg index above.
        self.power_index = (self.slot if power_index is None
                            else power_index & 0xFF)
        # Per-register serial timeout. A healthy reg op answers in ms; a reader
        # whose SPI leaf hasn't been brought up yet (cold, before reset()) can
        # stall the default 5s each — bound it so an occasional hang fails fast
        # and a long read of hundreds of regs can't wedge the link for minutes.
        self._reg_timeout = reg_timeout

    def _conn(self) -> Any:
        # send_command lives on the serial connection (afcACE._ace), which may
        # be (re)created, so fetch it at call time.
        conn = getattr(self._ace2, "_ace", None)
        if conn is None:
            raise IOError("ACE2 serial not connected")
        return conn

    def reg_read(self, reg: int) -> int:
        arg = (self.slot << 16) | (reg & 0xFF)
        res = self._conn().send_command(
            "mfrc522_reg_read", {"arg": arg}, timeout=self._reg_timeout)
        return int((res or {}).get("val", 0)) & 0xFF

    def reg_write(self, reg: int, val: int) -> None:
        arg = (self.slot << 16) | ((reg & 0xFF) << 8) | (val & 0xFF)
        self._conn().send_command(
            "mfrc522_reg_write", {"arg": arg}, timeout=self._reg_timeout)

    def reader_power(self, on: bool) -> None:
        """ace2_rfid v2 (firmware cmd 0x52): host-owned reader power, indexed by
        the per-pair power_index (may differ from the per-slot reg index)."""
        arg = (self.power_index << 16) | (1 if on else 0)
        self._conn().send_command(
            "mfrc522_reader_power", {"arg": arg}, timeout=self._reg_timeout)

    def set_rfid_enable(self, phys_slot: int, enable: bool) -> None:
        """Firmware cmd 0x0E — enable/disable the firmware's own identify loop for
        a PHYSICAL slot (the firmware maps slot>>1 to the shared reader). Disabling
        frees the reader (and gates its power off) so the host can drive it.

        Sent fire-and-forget (matching the connect-time usage) so a command that
        may not ack can't stall the read; wire ordering still guarantees it is
        processed before the following synchronous reader_power/reg reads."""
        conn = self._conn()
        conn.send_command_async(
            "set_rfid_enable", {"index": phys_slot & 0xFF, "enable": bool(enable)})


class AFC_ACE2_RFID(AFCUnitRFID):
    """Reads ACE Pro 2 spool tags via the firmware passthrough and applies them
    to AFC lanes. Configure with ``[AFC_ACE2_rfid]``."""

    def __init__(self, config: "ConfigWrapper") -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger("AFC_ACE2_rfid")
        self.gcode = self.printer.lookup_object("gcode")

        key = (config.get("bambu_master_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(key) if key else None
        # Creality CFS keys (both required to decode Creality tags): key = the
        # UID->MIFARE-key AES key (u_key), encryption_key = the AES-CBC payload
        # key (d_key). Snapmaker/Anycubic/Elegoo need no config keys.
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)
        self.log_prefix = "ACE2 RFID"          # AFCUnitRFID.apply_to_lane / Spoolman
        # Auto-read the tag when a spool is inserted (the insert preload/feed
        # spins the spool, sweeping the tag past the reader). Retries a few times
        # to catch it as it settles.
        self.read_on_insert = config.getboolean("read_on_insert", True)
        self.read_on_insert_attempts = config.getint(
            "read_on_insert_attempts", 3, minval=1)
        self.read_on_insert_delay = config.getfloat(
            "read_on_insert_delay", 1.0, minval=0.0)
        # Per-register serial timeout (see _Ace2RegLink). A healthy reg op
        # answers in a few ms; this only bounds a WEDGE (the ACE2 firmware's
        # passthrough briefly not responding), so keep it short so a wedged read
        # fails fast and the scan/probe retries instead of burning seconds. The
        # reader is warm during reads (reset already done), so a sub-second bound
        # is safe.
        self.read_reg_timeout = config.getfloat(
            "read_reg_timeout", 0.8, minval=0.2)
        # Reg chip-select index. This unit has 2 MFRC522 readers (index 0/1),
        # each covering a slot PAIR — reg-index 2/3 don't exist. So reg r/w and
        # power both use the per-pair index (slot>>1). per-slot (each slot its
        # own chip) is left as an opt-in for units that differ, but is OFF by
        # default.
        self.reader_reg_per_slot = config.getboolean("reader_reg_per_slot", False)
        # disable_rfid: turn OFF all automatic tag reading. Inserts still load
        # and stage to the hub, but as a plain feed — no chunk-probe moves and no
        # reads (the manual ACE_RFID_READ command still works). Default OFF.
        self.disable_rfid = config.getboolean("disable_rfid", False)
        # stage_read: read DURING the staging feed so the tag is caught while the
        # spool spins (it can't be relied on to park in the antenna's range
        # afterwards). The reader is polled CONTINUOUSLY as the filament feeds in
        # (a smooth load, no chunking); on the first UID the feed is STOPPED so the
        # tag is read at rest. The ACE's feed assist watches the encoder and would
        # trip its stuck-spool error (fast flash) when the read pauses the feed, so
        # we STOP+suppress feed assist for the stage read and restore it after.
        # Default ON.
        self.stage_read = config.getboolean("stage_read", True)
        # Speed of the SCAN feed (mm/s). Slower = less coast past the antenna after
        # the stop-on-detect and a longer dwell in the read zone, so the block read
        # completes at rest. 25 is the default. After the read the REMAINDER is fed
        # at the full feed_speed so staging isn't slow overall.
        self.stage_scan_speed = min(90.0, config.getfloat(
            "stage_scan_speed", 25.0, minval=1.0))
        # Parked-read attempts: after stop-on-detect the tag is read at rest, and a
        # read can be flaky the instant motion stops — retry the read this many
        # times before falling back to the re-center sweep.
        self.stage_read_hold_attempts = config.getint(
            "stage_read_hold_attempts", 3, minval=1)
        # If the parked read still misses, step the tag back toward the antenna in
        # small retracts (it coasts PAST the read zone after STOP), reading at each
        # step, up to this total. The extra retract is fed back when the remainder
        # is staged. 0 disables the sweep.
        self.stage_recenter_max = config.getfloat(
            "stage_recenter_max", 42.0, minval=0.0)
        self.stage_recenter_step = config.getfloat(
            "stage_recenter_step", 3.0, minval=0.5)
        # Deprecated (chunked staging is gone) — still read so existing configs
        # that set it don't raise an "unused option" error at startup.
        self.stage_read_chunk = config.getfloat(
            "stage_read_chunk", 25.0, minval=2.0)
        # When a shared-reader SISTER spool's tag dominates the antenna and keeps
        # blocking a read (during staging OR a spool_scanner scan), automatically
        # retract that sibling a little to rotate its tag off the coil, finish the
        # read, then restore the sibling. Moves LOADED filament, so it's hard-gated
        # to only ever an IDLE, hub-staged sibling (never tool-loaded, never during
        # a print). Set false to disable the automatic move — the user is then
        # asked on the console to nudge the sibling spool a quarter turn by hand.
        self.auto_tag_adjust = config.getboolean("auto_tag_adjust", True)
        self.auto_tag_adjust_dist = config.getfloat(
            "auto_tag_adjust_dist", 75.0, minval=1.0)
        # Mandatory initial load fed BEFORE dist_hub — it loads the filament into
        # the unit (replacing the factory initial staging we bypass with
        # skip_factory_autostage). Net load = this + dist_hub. Only applied when
        # we've skipped the factory autostage (else the factory loads it).
        self.stage_initial_dist = config.getfloat(
            "stage_initial_dist", 500.0, minval=0.0)
        # RFID scan window: how far to feed at scan speed while polling the reader
        # before giving up and feeding the remainder at full speed. The tag is in
        # the shared antenna's range for only a narrow arc per spool revolution, so
        # the window must comfortably exceed one revolution or a run can alias past
        # it — one spool reads, the identical next one doesn't. 500mm just clears
        # the unit; 600 gives leeway so the tag reliably sweeps past. Spans the
        # initial load and into the start of dist_hub — it adds NO net distance
        # (that filament is fed anyway), it just polls further before the handoff.
        self.stage_scan_dist = config.getfloat(
            "stage_scan_dist", 600.0, minval=0.0)
        # Two physical slots share one reader (slots 0/1 -> reader0, 2/3 ->
        # reader1); its antenna picks up whichever tag is in range, so a stage
        # read for one slot can return the NEIGHBOUR slot's parked tag — leaving
        # both lanes pointed at the same spool. With dedup on (default) a read
        # whose UID matches the shared sibling's known tag (and that sibling
        # still has a spool present) is rejected as "not this lane's", so the
        # chunked staging keeps probing until the active lane's OWN tag — which
        # is the one actually spinning past the reader — comes into range.
        self.shared_reader_dedup = config.getboolean("shared_reader_dedup", True)
        # Last UID successfully read per PHYSICAL slot, used for the dedup above.
        self._slot_uid: Dict[int, str] = {}
        # Skip the ACE's slow (~30-45s) autonomous load-to-toolhead-and-back
        # autostage: it's tied to the factory identify, so disabling identify on
        # ALL slots at boot makes the unit go straight to our (faster) staging
        # (initial load + scan window, then dist_hub). Default ON — it's the whole
        # point of this module; set False only to fall back to factory autostaging.
        self.skip_factory_autostage = config.getboolean(
            "skip_factory_autostage", True)
        # The boot-time identify-disable races the ACE's own deferred serial
        # connect (which re-enables identify), so poll until connected then
        # disable. These bound that poll (default ~30s window).
        self._identify_disable_delay = config.getfloat(
            "identify_disable_retry", 0.5, minval=0.05)
        self._identify_disable_max = config.getint(
            "identify_disable_max_tries", 60, minval=1)
        self._identify_disable_tries = 0
        # Restore the factory identify after each read. Default follows the
        # autostage switch: with factory autostaging in use (skip off) we must
        # restore identify or the NEXT insert won't autostage; when skipping it
        # we keep identify off. The post-read final unwind (rfid_recover_unwind
        # in AFC_ACE) clears the latch that re-enabling used to cause.
        self.probe_restore_identify = config.getboolean(
            "restore_identify", not self.skip_factory_autostage)
        self.probe_power_off = config.getboolean("probe_power_off", True)
        self.probe_settle = config.getfloat("probe_settle", 0.2, minval=0.0)
        # Name of the ACE2 printer object exposing send_command (afcACE subclass).
        self.ace2_name = config.get("ace2_object", "AFC_ACE2")
        # "lane1:0, lane2:1" -> {lane_name: physical slot}
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            name, _, slot = pair.partition(":")
            self._lane_slot[name.strip()] = int(slot or 0)

        # Scanner lanes: a lane whose reader is used to SCAN a presented tag and
        # stage it as the next spool id (consumed by the next lane load), like the
        # U1's scanner. ACE_RFID_SCAN LANE=<name> powers that lane's reader and
        # reads for scan_seconds. Each scanner lane needs a slot in lane_slot_map.
        self._scanner_lanes = set()
        for n in (config.get("scanner_lanes", "") or "").split(","):
            n = n.strip()
            if n:
                self._scanner_lanes.add(n)
        # How long ACE_RFID_SCAN reads for (seconds) and the dwell between read
        # attempts. Present the spool's tag to the reader during this window.
        self.scan_seconds = config.getfloat("scan_seconds", 30.0, minval=1.0)
        self.scan_interval = config.getfloat("scan_interval", 0.3, minval=0.05)
        # Require the SAME decoded UID on this many CONSECUTIVE reads before a
        # scan accepts it — so a single stray read (a neighbour tag that flicked
        # into the shared antenna, or any one-off) can't match/create the wrong
        # Spoolman spool. A spool held on the reader confirms in that many reads;
        # a stray resets the streak. Mirrors the U1's scanner_confirm_reads.
        self.scanner_confirm_reads = config.getint(
            "scanner_confirm_reads", 2, minval=1)

        self.afc: Optional["afc"] = None
        self.ace2: Optional[Any] = None
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler("afc_ace:post_insert",
                                            self._on_post_insert)
        # Continuous read during staging (see stage_read).
        self._probe: Optional[dict] = None
        # slots whose feed assist we suppressed
        self._assist_slots: Optional[Tuple[int, ...]] = None
        # (sibling_slot, dist, speed) when a shared-reader sister was retracted to
        # clear its tag off the antenna during this stage; None otherwise. Restored
        # (re-fed the same distance) in _stage_probe_end.
        self._sister_retracted: Optional[Tuple[Any, float, float]] = None
        # True once we've told the user (this stage) to nudge the sister spool by
        # hand — so the console isn't spammed with repeat hints during one stage.
        self._sister_hint_shown = False
        # The staging feed fires begin (we run the whole scan-and-read there) and
        # end (teardown). No per-chunk event any more — the read rides a continuous
        # feed, polled from inside the begin handler.
        self.printer.register_event_handler("afc_ace:stage_probe_begin",
                                            self._stage_probe_begin)
        self.printer.register_event_handler("afc_ace:stage_probe_end",
                                            self._stage_probe_end)
        # NB: no digit in the command name — Klipper parses a token like "ACE2"
        # as a classic G/M-style command ("Unknown command ACE2"), so the ACE2
        # RFID read is exposed as ACE_RFID_READ.
        self.gcode.register_command(
            "ACE_RFID_READ", self.cmd_ACE_RFID_READ,
            desc="Read the ACE Pro 2 RFID tag for a lane (LANE=) or slot (SLOT=)")
        self.gcode.register_command(
            "ACE_RFID_SCAN", self.cmd_ACE_RFID_SCAN,
            desc="Scan a scanner lane's RFID for a presented tag and stage it as "
                 "the next spool id (LANE=, optional SECONDS=)")
        self.gcode.register_command(
            "ACE_RFID_BLOCKS", self.cmd_ACE_RFID_BLOCKS,
            desc="Dump raw MIFARE blocks of a Bambu tag (LANE=/SLOT=, optional "
                 "BLOCKS=5,16) — shows colour count + second colour for dual-colour")
        self.gcode.register_command(
            "ACE_RFID_STAGE_TEST", self.cmd_ACE_RFID_STAGE_TEST,
            desc="DIAGNOSTIC: on a STAGED lane, retract DIST then feed DIST while "
                 "polling; STOP the feed on detect, read parked. Reveals whether a "
                 "concurrent poll works + trips the stuck-spool watchdog (LANE=, "
                 "optional DIST=500)")

    def _on_ready(self) -> None:
        self.afc = self.printer.lookup_object("AFC", None)
        # Fall back to the shared [AFC_rfid_keys] section for any brand key this
        # section didn't set — configure the keys once (in any include), use them
        # from every host-side reader.
        if resolve_rfid_keys is not None:
            (self.bambu_master_key, self.creality_key,
             self.creality_encryption_key) = resolve_rfid_keys(
                self.printer, self.bambu_master_key, self.creality_key,
                self.creality_encryption_key)
        # Mark configured scanner lanes so scanner-aware code (and any bridge
        # hooks) sees lane.spool_scanner, mirroring the U1.
        if self.afc is not None and hasattr(self.afc, "lanes"):
            for n in self._scanner_lanes:
                lane = self.afc.lanes.get(n)
                if lane is not None:
                    try:
                        lane.spool_scanner = True
                    except Exception:
                        pass
        # The ACE2 serial object is a named section: "AFC_ACE2 <name>" (e.g.
        # "AFC_ACE2 Ace2_1"). Try the configured/bare name, else auto-discover
        # the first AFC_ACE2* object.
        self.ace2 = (self.printer.lookup_object(self.ace2_name, None)
                     or self.printer.lookup_object("AFC_ACE2", None))
        if self.ace2 is None:
            for name, obj in self.printer.lookup_objects():
                if name == self.ace2_name or name.startswith("AFC_ACE2 "):
                    self.ace2 = obj
                    break
        if self.ace2 is None:
            self.logger.warning("ACE2 object not found; RFID disabled")
        else:
            self.logger.info("ACE2 RFID bound to %s",
                             getattr(self.ace2, "name", self.ace2))
            if self.skip_factory_autostage:
                # Turn identify off on every slot at boot so the unit skips its
                # slow autonomous autostage; our dist_hub staging takes over.
                # Retried until the ACE serial finishes connecting (below).
                self._identify_disable_tries = 0
                self.reactor.register_callback(self._retry_disable_identify)

    def _retry_disable_identify(self, eventtime: float) -> None:
        """Disable factory identify once the ACE serial is connected. The ACE's
        deferred connect races klippy:ready and re-enables identify as it comes
        up, so poll until connected, then disable so it sticks."""
        conn = getattr(self.ace2, "_ace", None) if self.ace2 is not None else None
        if conn is not None and getattr(conn, "connected", False):
            self._disable_factory_identify()
            return
        self._identify_disable_tries += 1
        if self._identify_disable_tries >= self._identify_disable_max:
            self.logger.warning(
                "ACE2 RFID: ACE serial not ready after %d tries; factory "
                "identify not disabled at boot (autostage may run on insert)",
                self._identify_disable_tries)
            return
        self.reactor.register_callback(
            self._retry_disable_identify,
            self.reactor.monotonic() + self._identify_disable_delay)

    def _disable_factory_identify(self, slots: Optional[Any] = None) -> None:
        """Disable the firmware identify so the ACE stops driving the reader (and
        stops its autonomous autostage). Defaults to ALL of the unit's slots.
        Best-effort — retried at each insert's stage_probe_begin too."""
        if self.ace2 is None:
            return
        if slots is None:
            n = int(getattr(self.ace2, "slot_count", 4) or 4)
            slots = range(n)
        link = _Ace2RegLink(self.ace2, 0, reg_timeout=self.read_reg_timeout)
        for s in slots:
            try:
                link.set_rfid_enable(int(s), False)
                self.logger.info("ACE2 RFID: factory identify disabled on slot %d", s)
            except Exception:
                self.logger.info("ACE2 RFID: could not disable factory identify "
                                 "on slot %d yet (serial not ready?)", s)

    # ── mapping ───────────────────────
    def _slot_for_lane(self, lane_name: str) -> Optional[int]:
        # Prefer the ACE unit's authoritative slot map so a stale/omitted
        # lane_slot_map can't point a read at the wrong reader.
        return self._physical_slot(lane_name)

    def _map(self, tag: dict) -> dict:
        """read_tag() result -> AFC slot_info (shared rich shape, AFC_RFID)."""
        return map_tag_to_slot_info(tag)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """Report the lane->slot map and the per-lane last-read records.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The lane_slot_map and last_reads records.
        """
        return {
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }

    # ── read + apply ─────────────────────
    def _read_tag(self, link: Any,
                  is_excluded: Optional[Callable[[str], bool]] = None,
                  seen: Optional[List[Any]] = None,
                  dump_blocks: Any = None) -> Optional[dict]:
        """read_tag with all configured brand keys (Bambu + Creality)."""
        return read_tag(link, bambu_master_key=self.bambu_master_key,
                        creality_key=self.creality_key,
                        creality_encryption_key=self.creality_encryption_key,
                        is_excluded=is_excluded, seen=seen,
                        dump_blocks=dump_blocks)

    def read_slot(self, slot: int, manage_power: bool = True,
                  reg_slot: Optional[int] = None,
                  dump_blocks: Any = None) -> Optional[dict]:
        """Read the tag on a PHYSICAL slot (0..3); returns the raw tag dict.

        Two physical readers cover the four slots, so the passthrough reader
        index is ``slot >> 1`` (slots 0/1 -> reader 0, slots 2/3 -> reader 1).

        With ``manage_power`` (default), take ownership of the reader for the
        read: stop the firmware's identify loop (which power-gates the reader and
        soft-resets it every cycle — fatal to a multi-step encrypted Bambu read),
        power the reader ourselves via the v2 firmware command, read, then restore
        power-off + identify. Requires the v2 firmware (cmd 0x52); set
        ``manage_power=False`` to fall back to the v1 best-effort behaviour.
        """
        if self.ace2 is None:
            raise self.printer.command_error("ACE2 not available")
        # reg chip-select index (per-slot); REG= overrides it for diagnostics.
        # Power is per-reader-pair (slot>>1), independent of the reg index.
        if reg_slot is None:
            reg_idx = self._reg_index(slot) & 0xFF
            power_idx = (slot >> 1) & 0xFF
        else:
            # Diagnostic: REG is the reader index; power the same reader.
            reg_idx = int(reg_slot) & 0xFF
            power_idx = reg_idx
        link = _Ace2RegLink(self.ace2, reg_idx, power_index=power_idx,
                            reg_timeout=self.read_reg_timeout)
        if not manage_power:
            return self._read_tag(link, dump_blocks=dump_blocks)
        # Disable the firmware's identify loop on BOTH slots of this reader pair:
        # it frees the reader for host control AND suppresses the firmware's
        # autonomous autostage on a fresh insert. Re-enabling (restore) is what's
        # optional (see probe_restore_identify) — disabling always happens.
        pair = (slot >> 1) & 0xFF
        sib = pair * 2
        shared = (sib, sib + 1)
        for s in shared:
            link.set_rfid_enable(s, False)
        sibling = sib if sib + 1 == slot else sib + 1
        self._dwell(0.1)                        # let the firmware stop identifying
        link.reader_power(True)                 # host powers the reader itself
        self._dwell(0.1)                        # MFRC522 oscillator/power-on settle
        try:
            # HALT the shared sibling's tag so a read for THIS slot returns this
            # slot's own tag, not the neighbour's parked one.
            excluder = self._probe_excluder(slot, sibling)
            tag = self._read_tag(link, is_excluded=excluder,
                                 dump_blocks=dump_blocks)
            uid = (tag or {}).get("uid")
            if uid is not None:
                if self._is_sibling_tag(slot, sibling, uid):
                    self.logger.warning(
                        "ACE2 RFID: slot %d read matches shared sibling slot %d's "
                        "tag (uid=%s) — check the spool is in slot %d, not %d",
                        slot, sibling, uid, slot, sibling)
                self._slot_uid[slot] = uid
            return tag
        finally:
            # Best-effort restore — a wedged serial during restore must not mask
            # the result or add another hard failure.
            try:
                link.reader_power(False)
            except Exception:
                self.logger.exception("ACE2 RFID: reader power-off failed")
            # Re-enable identify only when we're NOT keeping it off for
            # skip_factory_autostage (else the next insert would autostage).
            if self.probe_restore_identify:
                for s in shared:
                    try:
                        link.set_rfid_enable(s, True)
                    except Exception:
                        self.logger.exception(
                            "ACE2 RFID: re-enable identify failed")

    def _dwell(self, seconds: float) -> None:
        """Yield to the reactor for a short settle delay (no-op without one)."""
        r = getattr(self, "reactor", None)
        if r is not None:
            r.pause(r.monotonic() + seconds)

    # apply_to_lane() is inherited from AFCUnitRFID (shared AFC_RFID path); it
    # uses self._map, self.log_prefix ("ACE2 RFID") and self.auto_create. The
    # ACE unit/extruder's auto_spoolman_create still wins via
    # get_auto_spoolman_create(lane, ...); the [AFC_ACE2_rfid] value is only the
    # fallback default.

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """Read the tag on the reader slot mapped to the given lane and, when
        that lane exists in AFC, apply the tag's filament to it. Raises a command
        error if the lane has no slot mapped (set lane_slot_map).

        :param lane_name: Name of the AFC lane to read the tag for
        :return dict: Tag data that was read, or None when no tag is present
        """
        slot = self._slot_for_lane(lane_name)
        if slot is None:
            raise self.printer.command_error(
                "lane %r has no ACE2 reader slot (set lane_slot_map)" % lane_name)
        tag = self.read_slot(slot)
        if not tag:
            self.logger.info("ACE2 RFID: no tag read on slot %d", slot)
            return None
        lane = None
        if self.afc is not None:
            lane = self.afc.lanes.get(lane_name) if hasattr(self.afc, "lanes") else None
        if lane is not None:
            self.apply_to_lane(lane, tag)
        return tag

    # ── scanner lane: read a presented tag, stage it as the next spool id ─────
    def _scan_slot(self, slot: int, duration: float,
                   lane_name: str = "") -> Optional[dict]:
        """Power the slot's reader and read repeatedly for up to ``duration``
        seconds, returning the first tag read (or None). At rest (no spin), so
        the tag must be presented within the antenna's range for the window.

        On the first sight of the presented tag a "hold the spool" popup is
        shown (``lane_name`` labels it); it's dismissed on a timeout and
        replaced by the result notification on a successful read.

        Unlike read_slot's single read, the reader is powered ONCE and held for
        the whole scan (no feed happens during a scan, so holding power is safe)
        while read attempts are retried until a tag reads or the deadline hits."""
        reg_idx = self._reg_index(slot) & 0xFF
        power_idx = (slot >> 1) & 0xFF
        link = _Ace2RegLink(self.ace2, reg_idx, power_index=power_idx,
                            reg_timeout=self.read_reg_timeout)
        pair = (slot >> 1) & 0xFF
        sib = pair * 2
        shared = (sib, sib + 1)
        sibling = sib if sib + 1 == slot else sib + 1
        # Free the reader for host control (and suppress firmware autostage);
        # reader_power(OFF) is the sync barrier that flushes the async disable.
        for s in shared:
            link.set_rfid_enable(s, False)
        link.reader_power(False)
        self._dwell(0.1)
        link.reader_power(True)
        self._dwell(0.1)
        deadline = self.reactor.monotonic() + max(0.0, duration)
        excluder = self._probe_excluder(slot, sibling)
        seen_uids: Dict[str, bool] = {}   # uid_hex -> whether it was decoded
        confirm = max(1, self.scanner_confirm_reads)
        cand_uid = None         # UID of the current confirmation candidate
        streak = 0              # consecutive full decodes of cand_uid
        detected_shown = False  # "hold the spool" popup shown once per scan
        # Fresh scan: no sister retracted, no manual-nudge hint shown yet.
        self._sister_retracted = None
        self._sister_hint_shown = False
        try:
            while self.reactor.monotonic() < deadline:
                seen: List[Any] = []
                try:
                    tag = self._read_tag(link, is_excluded=excluder, seen=seen)
                except Exception:
                    self.logger.exception("ACE2 RFID scan: read error")
                    tag = None
                for h, _s, _e in seen:
                    seen_uids.setdefault(h, False)
                # First sight of the PRESENTED tag (a non-excluded UID in the
                # field, or a decode) -> tell the user to hold it steady while
                # the confirmation reads complete. Once per scan.
                if not detected_shown and (
                        any(not e for _h, _s, e in seen)
                        or (tag and tag.get("uid"))):
                    prompt_hold_spool(self.gcode.respond_raw,
                                      lane_name or ("slot %d" % slot))
                    detected_shown = True
                # A shared-reader SISTER tag dominating the antenna while the
                # presented tag never appears is the classic coil block. Clear it
                # automatically (retract the sibling a quarter rotation) if enabled
                # and safe; otherwise tell the user once to nudge it by hand.
                if (any(e for _h, _s, e in seen)
                        and not any(not e for _h, _s, e in seen)):
                    self._handle_sister_domination(sibling)
                # ONLY trust a fully-decoded read (the brand keys are UID-derived,
                # so a wrong UID can't decode), AND require the SAME UID on
                # `confirm` consecutive decodes — so a single stray tag that
                # flicked into the shared antenna can't match/create the wrong
                # Spoolman spool. A held spool confirms in `confirm` reads.
                if tag and tag.get("filament"):
                    uid = tag.get("uid")
                    if uid == cand_uid:
                        streak += 1
                    else:
                        cand_uid, streak = uid, 1
                    if streak >= confirm:
                        if uid is not None:
                            self._slot_uid[slot] = uid
                        return tag
                    # decoded but not yet confirmed — already in seen_uids (False)
                    self._dwell(self.scan_interval)
                    continue
                # A miss/no-decode breaks the confirmation streak.
                cand_uid, streak = None, 0
                if tag and tag.get("uid"):
                    seen_uids[tag["uid"]] = True   # read a UID but it didn't decode
                self._dwell(self.scan_interval)
            # Diagnostic: report exactly what the antenna saw so a misread UID
            # (differs from another reader) or a decode-that-never-completes is
            # visible instead of a bare "no tag".
            if seen_uids:
                self.logger.info(
                    "ACE2 RFID scan: no full decode; saw UID(s) %s",
                    ", ".join("%s%s" % (h, " (uid-only, no decode)" if v else "")
                              for h, v in sorted(seen_uids.items())))
            # Nothing to report -> close the "hold the spool" popup (on success
            # the result notification replaces it instead).
            if detected_shown:
                dismiss_prompt(self.gcode.respond_raw)
            return None
        finally:
            try:
                link.reader_power(False)
            except Exception:
                self.logger.exception("ACE2 RFID scan: reader power-off failed")
            # Put any sister we retracted to clear the antenna back where it was.
            self._restore_sister()
            if self.probe_restore_identify:
                for s in shared:
                    try:
                        link.set_rfid_enable(s, True)
                    except Exception:
                        self.logger.exception(
                            "ACE2 RFID scan: re-enable identify failed")

    def scan_lane(self, lane_name: str,
                  seconds: Optional[float] = None) -> Optional[dict]:
        """Scan the lane's reader for a presented tag for up to ``seconds`` and,
        on a read, stage it as the NEXT spool id — consumed by the next lane
        load (like the U1 scanner). Does NOT apply to the scanner lane itself."""
        if self.ace2 is None:
            raise self.printer.command_error("ACE2 not available")
        slot = self._slot_for_lane(lane_name)
        if slot is None:
            raise self.printer.command_error(
                "lane %r has no ACE2 reader slot (set lane_slot_map)" % lane_name)
        duration = self.scan_seconds if seconds is None else float(seconds)
        tag = self._scan_slot(int(slot), duration, lane_name=lane_name)
        if not tag:
            self.logger.info(
                "ACE2 RFID scan: no tag on lane %s in %.0fs", lane_name, duration)
            return None
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        slot_info = self._map(tag)
        self.record_tag_read(lane_name, slot_info)
        allow_create = self.auto_create
        if get_auto_spoolman_create is not None and lane is not None:
            try:
                allow_create = get_auto_spoolman_create(lane, self.auto_create)
            except Exception:
                pass
        if self.afc is not None and getattr(self.afc, "spoolman", None):
            try:
                sync_rfid_to_spoolman(
                    self.afc, lane, slot_info, self.logger, "ACE2 RFID scan",
                    allow_create=allow_create, set_next=True)
            except Exception as e:
                self.logger.warning("ACE2 RFID scan Spoolman sync failed: %s", e)
        # Pop a UI notification (Mainsail/Fluidd action:prompt) with what we read,
        # like the U1 scanner does on a match/create.
        spool = getattr(self.afc, "spool", None) if self.afc is not None else None
        self._notify_scan(slot_info, lane_name,
                          getattr(spool, "next_spool_id", None))
        return tag

    def _spool_details(self, spool_id: Any, slot_info: dict) -> dict:
        """Build the richest available spool description for the scan popup:
        prefer the matched Spoolman spool's stored record (complete even when the
        at-rest tag read only got the UID), falling back to the tag decode."""
        d = {
            "brand": slot_info.get("brand", ""),
            "material": slot_info.get("material", ""),
            "color": (slot_info.get("color_hex", "") or "").lstrip("#"),
            "diameter": slot_info.get("diameter"),
            "ext": slot_info.get("extruder_temp"),
            "bed": slot_info.get("bed_temp"),
            "weight": slot_info.get("weight_g"),
            "name": "",
        }
        mr = getattr(self.afc, "moonraker", None) if self.afc is not None else None
        if not spool_id or mr is None:
            return d
        try:
            sp = mr.get_spool(spool_id)
        except Exception:
            sp = None
        if not isinstance(sp, dict):
            return d
        fil = sp.get("filament") or {}
        vendor = (fil.get("vendor") or {}).get("name")
        d["name"] = fil.get("name") or d["name"]
        d["brand"] = vendor or d["brand"]
        d["material"] = fil.get("material") or d["material"]
        d["color"] = (fil.get("color_hex") or d["color"] or "").lstrip("#")
        d["diameter"] = fil.get("diameter") or d["diameter"]
        d["ext"] = fil.get("settings_extruder_temp") or d["ext"]
        d["bed"] = fil.get("settings_bed_temp") or d["bed"]
        # Prefer remaining weight (what's actually left on the spool).
        rem = sp.get("remaining_weight")
        d["weight"] = rem if rem is not None else (
            fil.get("weight") or d["weight"])
        return d

    def _notify_scan(self, slot_info: dict, lane_name: str,
                     spool_id: Any) -> None:
        """Show a user-visible popup (Mainsail/Fluidd action:prompt) + console
        summary of the scanned spool, mirroring the U1 scanner. Enriched from the
        matched Spoolman spool. Best-effort — a UI notification (or the Spoolman
        lookup) must never fault the scan."""
        try:
            d = self._spool_details(spool_id, slot_info)
            color = d["color"]
            # Prefer the matched Spoolman name; fall back to the shared rich
            # "<brand> <material> <sub_type>" builder so an unmatched tag still
            # reads e.g. "Bambu PLA Basic".
            name = d["name"] or build_filament_name(
                d["brand"], d["material"], slot_info.get("sub_type", ""))
            lines = []
            if name:
                lines.append("Name: %s" % name)
            if d["brand"]:
                lines.append("Brand: %s" % d["brand"])
            if d["material"]:
                lines.append("Material: %s" % d["material"])
            if color:
                lines.append("Color: #%s" % color)
            if d["diameter"]:
                lines.append("Diameter: %smm" % d["diameter"])
            if d["ext"]:
                lines.append("Nozzle temp: %s°C" % d["ext"])
            if d["bed"]:
                lines.append("Bed temp: %s°C" % d["bed"])
            if d["weight"]:
                lines.append("Remaining: %sg" % round(float(d["weight"])))
            if spool_id:
                lines.append("Spoolman ID: %s" % spool_id)
            if not lines:
                lines.append("uid: %s" % slot_info.get("uid", ""))
            title = "Spool Scanned on %s" % lane_name if lane_name else \
                "Spool Scanned"
            self.gcode.respond_info("%s:\n  %s" % (title, "\n  ".join(lines)))
            respond = self.gcode.respond_raw
            respond("// action:prompt_begin %s" % title)
            for pl in lines:
                respond("// action:prompt_text %s" % pl)
            respond("// action:prompt_footer_button "
                    "OK|RESPOND TYPE=command MSG=action:prompt_end|info")
            respond("// action:prompt_show")
            # Auto-dismiss after 10s so it doesn't linger if unattended.
            self.reactor.register_callback(
                lambda e: self.gcode.respond_raw("// action:prompt_end"),
                self.reactor.monotonic() + 10.0)
        except Exception as e:
            self.logger.warning("ACE2 RFID scan: notification error: %s", e)

    # ── auto-read on insert ───────────────────
    def _on_post_insert(self, lane: "AFCLane") -> None:
        """`afc_ace:post_insert` handler — a spool was just inserted and fed to
        the hub (the spool spun, sweeping the tag past the reader). Auto-read the
        tag for lanes we're configured for. Deferred onto the reactor so it never
        blocks the insert path, and best-effort so it can never fault it."""
        # RFID off: no automatic insert read.
        if self.disable_rfid:
            return
        # When stage_read is on, the tag is read DURING staging (stage_probe*),
        # so skip the after-staging read to avoid a redundant reader-power cycle.
        if self.stage_read or not self.read_on_insert:
            return
        name = getattr(lane, "name", None)
        if name is None or not self._rfid_enabled(name):
            return
        self.reactor.register_callback(lambda et, n=name: self._auto_read(n))

    def _auto_read(self, name: str) -> None:
        """Read the lane's tag, retrying a few times to catch the tag as the
        spool settles. Stops on the first successful read."""
        for attempt in range(self.read_on_insert_attempts):
            try:
                tag = self.read_lane(name)
            except self.printer.command_error as e:
                self.logger.info("ACE2 RFID auto-read %s: %s", name, e)
                return
            except Exception:
                self.logger.exception("ACE2 RFID auto-read failed for %s", name)
                tag = None
            if tag:
                return
            if attempt < self.read_on_insert_attempts - 1 and self.read_on_insert_delay:
                self.reactor.pause(
                    self.reactor.monotonic() + self.read_on_insert_delay)
        self.logger.info("ACE2 RFID auto-read %s: no tag after %d attempts",
                         name, self.read_on_insert_attempts)

    # ── chunked read during staging ────────────────
    def _physical_slot(self, name: str) -> Optional[int]:
        """The lane's ACE slot — from the ACE's authoritative map when available
        (correct for motion + reader), else the RFID lane_slot_map override."""
        sm = getattr(self.ace2, "_slot_map", None)
        if isinstance(sm, dict) and name in sm:
            return sm[name]
        return self._lane_slot.get(name)

    def _reg_index(self, slot: int) -> int:
        """MFRC522 chip-select index for a physical slot's reg r/w. Per-slot by
        default (each slot has its own chip); per-reader-pair (slot>>1) legacy."""
        return int(slot) if self.reader_reg_per_slot else (int(slot) >> 1)

    def _rfid_enabled(self, name: str) -> bool:
        """Whether RFID reads run for this lane. A configured lane_slot_map acts
        as an explicit allow-list; with no map every lane the ACE unit knows
        about is enabled (so correct numbering comes from the ACE, not config)."""
        if self._lane_slot:
            return name in self._lane_slot
        sm = getattr(self.ace2, "_slot_map", None)
        return isinstance(sm, dict) and name in sm

    def _lane_at_slot(self, slot: int) -> Optional["AFCLane"]:
        """The AFC lane object mapped to a PHYSICAL slot (via the ACE's slot map,
        else our lane_slot_map), or None."""
        if self.afc is None or not hasattr(self.afc, "lanes"):
            return None
        sm = getattr(self.ace2, "_slot_map", None)
        maps = [sm] if isinstance(sm, dict) else []
        maps.append(self._lane_slot)
        for m in maps:
            for lname, s in m.items():
                if int(s) == int(slot):
                    lane = self.afc.lanes.get(lname)
                    if lane is not None:
                        return lane
        return None

    def _sibling_present(self, slot: int) -> bool:
        """Whether the shared-reader sibling slot currently has a spool inserted.
        Conservative: assume present when we can't tell, so dedup still guards
        against duplicate assignment."""
        lane = self._lane_at_slot(slot)
        if lane is None:
            return True
        return bool(getattr(lane, "prep_state", True))

    def _is_sibling_tag(self, active_slot: int, sibling_slot: Optional[int],
                        uid: Optional[str]) -> bool:
        """True if ``uid`` belongs to the shared sibling slot, not the active
        lane, so assigning it here would duplicate the spool: within this session
        we already read this exact UID on the sibling slot, and that slot still
        holds a spool.
        """
        if (not self.shared_reader_dedup
                or not uid
                or sibling_slot is None):
            return False
        if not self._sibling_present(sibling_slot):
            return False
        return self._slot_uid.get(sibling_slot) == uid

    def _stage_probe_begin(self, lane: "AFCLane", ctx: dict) -> None:
        """`afc_ace:stage_probe_begin` — the staging feed is about to run. If we're
        configured for this lane, arm the reader and run the whole scan-and-read
        HERE on a continuous feed: feed at scan speed while polling, stop on the
        first UID, read the tag at rest, apply it. Reports ``ctx['fed']`` (net
        distance fed) and ``ctx['initial']`` so the feeder stages the remainder at
        full speed, and ``ctx['done']`` on a successful read."""
        try:
            if not self.stage_read or self.ace2 is None:
                return
            name = getattr(lane, "name", None)
            if name is None:
                return
            # RFID off: stage as a plain feed — no chunk-probe, no reads. Still
            # request the mandatory initial load so a skip_factory_autostage
            # insert reaches the hub (fed in one move, not chunked).
            if self.disable_rfid:
                ctx["initial"] = (self.stage_initial_dist
                                  if self.skip_factory_autostage else 0.0)
                return
            if not self._rfid_enabled(name):
                return
            slot = self._physical_slot(name)
            if slot is None:
                return
            slot = int(slot)
            pair = (slot >> 1) & 0xFF          # per-pair power domain
            link = _Ace2RegLink(self.ace2, self._reg_index(slot) & 0xFF,
                                power_index=pair,
                                reg_timeout=self.read_reg_timeout)
            shared = (pair * 2, pair * 2 + 1)
            sibling = shared[0] if shared[1] == slot else shared[1]
            # Disable identify on the pair: frees the reader AND suppresses the
            # firmware autostage for this fresh insert (restore stays optional).
            for s in shared:
                link.set_rfid_enable(s, False)
            # set_rfid_enable is fire-and-forget, so it needs a following
            # SYNCHRONOUS command to guarantee the firmware processes the
            # identify-disable (autostage suppression) before the feed starts —
            # otherwise the firmware autostages the fresh insert. Use a
            # reader_power(OFF) as that barrier: it flushes the disable before
            # the scan feed. (Holding reader power across the feed is FINE — the
            # continuous scan keeps it powered and polls throughout.
            # _run_stage_scan powers it back on for the scan.)
            link.reader_power(False)
            self._dwell(0.1)
            # Fresh spool in the active slot: forget its old UID so its own new
            # tag is never mistaken for stale history.
            self._slot_uid.pop(slot, None)
            self._probe = {"link": link, "shared": shared, "lane": lane,
                           "slot": slot, "sibling": sibling}
            # Fresh stage: no sister has been retracted, no manual-nudge hint
            # shown yet.
            self._sister_retracted = None
            self._sister_hint_shown = False
            # Stop + suppress the ACE feed assist on these slots for the whole
            # stage read: the read pauses the feed for seconds, and if assist is
            # watching the encoder it trips the stuck-spool error (fast flash).
            self._suppress_assist(shared)
            ctx["active"] = True
            # Mandatory initial load, only when we bypassed the factory initial
            # staging (otherwise the factory already loaded it).
            ctx["initial"] = (self.stage_initial_dist
                              if self.skip_factory_autostage else 0.0)
            # Run the scan-and-read now on a continuous feed; report how far it fed
            # so the feeder stages the remainder at full speed.
            ctx["fed"] = self._run_stage_scan(lane, ctx)
        except Exception:
            self.logger.exception("ACE2 RFID: stage_probe_begin failed")
            self._safe_probe_teardown()
            self._restore_assist()

    def _suppress_assist(self, slots: Any) -> None:
        """Stop feed assist on the given slots and suppress the watchdog so it
        stays off until we restore it (mirrors ACE_FEED_ASSIST ENABLE=0)."""
        ace2 = self.ace2
        if ace2 is None:
            return
        self._assist_slots = tuple(slots)
        sup = getattr(ace2, "_assist_suppressed", None)
        active = getattr(ace2, "_feed_assist_active", None)
        conn = getattr(ace2, "_ace", None)
        for s in slots:
            try:
                if isinstance(sup, set):
                    sup.add(s)
                if isinstance(active, set) and s in active:
                    ace2._stop_feed_assist(s)
                elif conn is not None:
                    conn.stop_feed_assist_sync(s)   # force stop; tracking drifted
                self.logger.info("ACE2 RFID: feed assist stopped on slot %d", s)
            except Exception:
                self.logger.exception(
                    "ACE2 RFID: failed to stop feed assist on slot %d", s)

    def _restore_assist(self) -> None:
        """Un-suppress feed assist on the slots we stopped, letting the ACE's
        watchdog reconcile it normally again."""
        slots = self._assist_slots
        self._assist_slots = None
        if not slots or self.ace2 is None:
            return
        sup = getattr(self.ace2, "_assist_suppressed", None)
        for s in slots:
            try:
                if isinstance(sup, set):
                    sup.discard(s)
                self.logger.info("ACE2 RFID: feed assist restored on slot %d", s)
            except Exception:
                self.logger.exception(
                    "ACE2 RFID: failed to restore feed assist on slot %d", s)

    def _probe_excluder(self, active_slot: int,
                        sibling_slot: Optional[int]
                        ) -> Optional[Callable[[str], bool]]:
        """A ``uid_hex -> bool`` predicate that flags the shared sibling's tag, or
        None when there's nothing to exclude. Passed into read_tag so a neighbour
        tag is HALTED (not decoded) and this lane's own tag is read instead."""
        if not self.shared_reader_dedup or sibling_slot is None:
            return None
        return lambda uid: self._is_sibling_tag(active_slot, sibling_slot, uid)

    def _run_stage_scan(self, lane: "AFCLane", ctx: dict) -> float:
        """Feed the lane in at scan speed while polling the reader; STOP on the
        first of THIS lane's tags, read it at rest (re-centering if it coasted
        past the antenna), and apply it. A shared-reader SISTER tag that dominates
        the antenna is retracted out of the way (auto_tag_adjust) so our own tag
        can be seen. Returns the NET distance fed during the scan (forward feed
        minus any re-centering retract) so the feeder stages the remainder at the
        full feed speed. Sets ``ctx['done']`` on a successful read and
        ``ctx['removed']`` if the spool was pulled mid-scan."""
        p = self._probe
        if not p:
            return 0.0
        link, slot, sibling = p["link"], p["slot"], p["sibling"]
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return 0.0
        speed = float(self.stage_scan_speed)
        # Scan window, bounded to the net load so a tagless spool never overshoots
        # the hub (the remainder is what's left of initial + dist_hub).
        initial = float(ctx.get("initial") or 0.0)
        total = initial + float(getattr(lane, "dist_hub", 0.0) or 0.0)
        scan_dist = float(self.stage_scan_dist)
        if total > 0:
            scan_dist = min(scan_dist, total)
        if scan_dist <= 0:
            return 0.0
        sib_lane = self._lane_at_slot(sibling)
        sib_has_spool = (sib_lane is not None
                         and bool(getattr(sib_lane, "prep_state", False)))
        # Host owns the reader for the whole scan — power is held across the feed
        # (the continuous scan keeps it powered and polls throughout).
        link.reader_power(True)
        self._dwell(0.1)
        try:
            self.ace2._wait_for_ace_ready(timeout=self.read_reg_timeout + 20.0)
        except Exception:
            pass
        # ONLY act on the sister if its tag is ACTUALLY on the antenna. The sister
        # is stationary during our scan, so we probe the antenna ONCE, before our
        # own spool has spun: any tag read here is a parked, interfering tag (the
        # sister's). If nothing's there, the sister isn't in range — leave it be.
        # If a tag IS parked: clear it — retract a movable sister off the coil, or
        # (if it can't be moved) exclude that UID for the whole scan so it's never
        # read as this lane's. Dedup alone can't catch it: a sister staged in a
        # prior session has no recorded UID, so its tag would sail straight through.
        baseline = None
        blocked_sib = None            # a present sibling we couldn't move off the coil
        if sib_has_spool:
            parked = None
            for _ in range(3):                       # a couple retries — read is
                try:                                 # reliable for a still tag
                    puid, _p = MifareClassic(Mfrc522(link)).activate()
                except Exception:
                    puid = None
                if puid is not None:
                    parked = puid.hex()
                    break
                self._dwell(0.05)
            if parked is not None:
                self.logger.info(
                    "ACE2 RFID: stage scan slot %s — tag %s parked on the shared "
                    "reader (sibling slot %s); clearing it before the read",
                    slot, parked, sibling)
                # Movable -> retract it off the antenna. Unmovable -> exclude its
                # tag (baseline) and TRY the read anyway; only if we come up empty
                # do we ask the user to move it / set the id (end of scan).
                self._maybe_retract_sister(sibling)
                if self._sister_retracted is None:   # couldn't move it
                    baseline = parked
                    blocked_sib = sibling
        t0 = self.reactor.monotonic()
        try:
            conn.feed_filament(slot, scan_dist, speed)
        except Exception:
            self.logger.exception("ACE2 RFID: stage scan feed failed")
            self._reader_power_off(link)
            return 0.0
        eff = min(max(speed, 1.0), 90.0)
        expected = scan_dist / eff
        deadline = self.reactor.monotonic() + expected * 1.5 + 15.0
        slot_moving = getattr(self.ace2, "_slot_is_moving", None)
        slot_empty = getattr(self.ace2, "_slot_reports_empty", None)
        get_status = getattr(conn, "get_status", None)
        detected = None
        net_fed = 0.0
        started = False
        idle = 0
        while self.reactor.monotonic() < deadline:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            # Spool pulled back out mid-scan (fumbled insert) — stop and bail so
            # the feeder resets prep instead of ramming an empty slot.
            if callable(slot_empty):
                try:
                    if slot_empty(slot):
                        try:
                            conn.stop_feed_filament(slot)
                        except Exception:
                            pass
                        ctx["removed"] = True
                        self._reader_power_off(link)
                        return min(scan_dist,
                                   (self.reactor.monotonic() - t0) * eff)
                except Exception:
                    pass
            try:
                uid, _sak = MifareClassic(Mfrc522(link)).activate()
            except Exception:
                uid = None
            if uid is not None:
                uidhex = uid.hex()
                # The stationary parked sibling tag captured at baseline — never
                # this lane's; keep feeding so our own tag sweeps in.
                if baseline is not None and uidhex == baseline:
                    continue
                # A neighbour's tag dominating the shared antenna — clear it out of
                # the way (retract the idle sibling) and keep feeding so OUR tag
                # sweeps into range. Restored in _stage_probe_end.
                if self._is_sibling_tag(slot, sibling, uidhex):
                    self.logger.info(
                        "ACE2 RFID: stage scan slot %s saw sibling slot %s's tag "
                        "(uid=%s) — clearing it and continuing",
                        slot, sibling, uidhex)
                    self._maybe_retract_sister(sibling)
                    if self._sister_retracted is None:
                        blocked_sib = sibling
                    continue
                detected = uidhex
                net_fed = min(scan_dist, (self.reactor.monotonic() - t0) * eff)
                try:
                    conn.stop_feed_filament(slot)
                except Exception as e:
                    self.logger.info("ACE2 RFID: stage scan stop_feed err: %s", e)
                break
            # End the scan when the feed finishes with no tag seen.
            if callable(slot_moving) and callable(get_status):
                try:
                    moving = slot_moving(get_status(timeout=2.0), slot)
                except Exception:
                    moving = True
                if moving:
                    started = True
                    idle = 0
                elif started:
                    idle += 1
                    if idle >= 2:
                        net_fed = scan_dist
                        break
        if not detected and net_fed == 0.0:
            net_fed = scan_dist
        # Parked read (+ re-center sweep) now that we've stopped on our own tag.
        if detected:
            tag, extra_retract = self._parked_read(link, slot, sibling, baseline)
            net_fed -= extra_retract
            if tag and tag.get("filament"):
                uid = tag.get("uid")
                if uid is not None:
                    self._slot_uid[int(slot)] = uid
                ctx["done"] = True
                # Release the reader before the feeder stages the remainder.
                self._safe_probe_teardown()
                try:
                    if self.afc is not None and hasattr(self.afc, "lanes"):
                        lane = self.afc.lanes.get(getattr(lane, "name", None), lane)
                    self.apply_to_lane(lane, tag)
                    f = tag.get("filament") or {}
                    self.logger.info(
                        "ACE2 RFID: read %s during staging — uid=%s type=%s %s",
                        getattr(lane, "name", "?"), tag.get("uid", ""),
                        tag.get("tag_type", ""), f.get("type", ""))
                except Exception:
                    self.logger.exception(
                        "ACE2 RFID: apply after stage read failed")
            else:
                self.logger.info(
                    "ACE2 RFID: stage scan slot %s detected uid=%s but the tag "
                    "did not decode — staging without RFID", slot, detected)
                self._reader_power_off(link)
                self._sister_blocked_hint(lane, blocked_sib)
        else:
            self.logger.info(
                "ACE2 RFID: stage scan slot %s — no tag seen in %.0fmm; staging "
                "without RFID", slot, scan_dist)
            self._reader_power_off(link)
            self._sister_blocked_hint(lane, blocked_sib)
        return max(0.0, net_fed)

    def _parked_read(self, link: Any, slot: int, sibling: Optional[int],
                     baseline: Optional[str] = None
                     ) -> Tuple[Optional[dict], float]:
        """Read the just-stopped tag at rest. It coasted PAST the antenna after
        STOP, so if the first reads miss, step it back (small retracts reverse the
        spin) to re-center it in the read zone, reading at each step. Returns
        ``(tag_or_None, total_retracted)`` — the retract is fed back when the
        remainder stages. ``baseline`` (the stationary sibling's parked UID) is
        halted alongside the known sibling so it's never returned as this lane's."""
        conn = getattr(self.ace2, "_ace", None)
        sib_excluder = self._probe_excluder(slot, sibling)

        def excluder(u: str) -> bool:
            """Return true when a read UID should be ignored — either the parked
            baseline UID or a UID the stationary sibling lane already owns.

            :param u: UID read from the tag being evaluated
            :return bool: True when the UID should be excluded from this lane's read
            """
            if baseline is not None and u == baseline:
                return True
            return bool(sib_excluder and sib_excluder(u))

        def _try() -> Optional[dict]:
            try:
                t = self._read_tag(link, is_excluded=excluder)
            except Exception:
                self.logger.exception("ACE2 RFID: parked read error")
                return None
            return t if (t and t.get("filament")) else None

        for _ in range(max(1, self.stage_read_hold_attempts)):
            tag = _try()
            if tag:
                return tag, 0.0
            self._dwell(0.12)
        retracted = 0.0
        step = max(0.5, float(self.stage_recenter_step))
        rspeed = float(getattr(self.ace2, "retract_speed", self.stage_scan_speed))
        while retracted + 1e-6 < self.stage_recenter_max and conn is not None:
            try:
                conn.unwind_filament(slot, step, rspeed)
            except Exception:
                self.logger.exception("ACE2 RFID: re-center retract failed")
                break
            retracted += step
            self._dwell(0.15)
            tag = _try()
            if tag:
                self.logger.info(
                    "ACE2 RFID: re-centered tag after %.0fmm retract", retracted)
                return tag, retracted
        return None, retracted

    def _stage_probe_end(self, lane: "AFCLane", ctx: dict) -> None:
        """`afc_ace:stage_probe_end` — fires after the whole staging feed. Ensure
        the reader is down and restore feed assist (suppressed for the read). The
        reader is usually already released in _stage_probe on a successful read;
        assist is restored only here so it stays off through the remaining feed."""
        self._safe_probe_teardown()
        # Put any sister we retracted to clear the antenna back where it was
        # (re-fed the same distance) before restoring assist.
        self._restore_sister()
        self._restore_assist()

    def _is_printing(self) -> bool:
        """True when AFC reports a print in progress (best-effort). Gates the
        sister retract: never move loaded filament mid-print."""
        try:
            return bool(self.afc.function.is_printing())
        except Exception:
            return False

    def _maybe_retract_sister(self, sibling_slot: Optional[int]) -> str:
        """A shared-reader SISTER spool's tag is dominating the antenna and
        blocking this lane's staging read. If enabled AND the sibling is safely
        movable, retract it a little to rotate its tag off the coil so our read
        can complete; it is re-fed the same distance in _stage_probe_end.

        Hard safety gate: only ever move an IDLE, hub-staged sibling — never one
        that is loaded to the toolhead, and never while a print is running.
        Retracting loaded filament mid-print would wreck the print.

        Returns:
          "retracted" — moved (or already moved earlier this stage);
          "blocked"   — a sibling is present but we won't/can't move it (feature
                        off, or a safety gate says no) → caller should tell the
                        user to nudge it by hand.
        """
        if sibling_slot is None:
            return "blocked"
        if self._sister_retracted is not None:
            return "retracted"          # already cleared it this stage
        if not self.auto_tag_adjust or self.ace2 is None:
            return "blocked"            # feature off → manual nudge
        lane = self._lane_at_slot(sibling_slot)
        if lane is None:
            return "blocked"
        if getattr(lane, "tool_loaded", False):
            return "blocked"            # loaded to toolhead — never move
        if not getattr(lane, "loaded_to_hub", False):
            return "blocked"            # not staged at hub — don't guess its pos
        if self._is_printing():
            return "blocked"            # print in progress — never move
        dist = float(self.auto_tag_adjust_dist)
        speed = float(getattr(self.ace2, "feed_speed", 80.0))
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return "blocked"
        try:
            conn.unwind_filament(sibling_slot, dist, speed)
            self.ace2._wait_for_feed_complete(sibling_slot, dist, speed)
        except Exception:
            self.logger.exception(
                "ACE2 RFID: sister retract on slot %s failed", sibling_slot)
            return "blocked"
        self._sister_retracted = (sibling_slot, dist, speed)
        self.logger.info(
            "ACE2 RFID: retracted sister slot %s by %.0fmm to clear its tag off "
            "the shared antenna", sibling_slot, dist)
        return "retracted"

    def _restore_sister(self) -> None:
        """Re-feed a sister spool we retracted during the stage back to its
        staged position (mirrors the retract). Best-effort — a failure just
        leaves the sibling slightly short, which the next load corrects."""
        info = self._sister_retracted
        self._sister_retracted = None
        if not info or self.ace2 is None:
            return
        slot, dist, speed = info
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return
        try:
            conn.feed_filament(slot, dist, speed)
            self.ace2._wait_for_feed_complete(slot, dist, speed)
            self.logger.info(
                "ACE2 RFID: restored sister slot %s (+%.0fmm) after stage read",
                slot, dist)
        except Exception:
            self.logger.exception(
                "ACE2 RFID: sister restore on slot %s failed", slot)

    def _handle_sister_domination(self, sibling_slot: Optional[int]) -> None:
        """A sibling tag is blocking the read. Clear it automatically if we can;
        otherwise tell the user (once per stage) to nudge that spool by hand.
        (Used by the manual spool_scanner; the stage read defers the hint to
        _sister_blocked_hint after a failed read.)"""
        if self._maybe_retract_sister(sibling_slot) != "blocked":
            return
        if self._sister_hint_shown:
            return
        self._sister_hint_shown = True
        lane = self._lane_at_slot(sibling_slot)
        lname = getattr(lane, "name", None) or ("slot %s" % sibling_slot)
        self.gcode.respond_info(
            "AFC ACE2: the tag on lane %s is sitting on the shared reader and "
            "blocking this read. Give that spool about a quarter turn by hand to "
            "move its tag off the reader, then it should read." % lname)

    def _sister_blocked_hint(self, active_lane: "AFCLane",
                             sibling_slot: Optional[int]) -> None:
        """A stage read came up empty and a present sibling we couldn't move was
        parked on the shared reader — almost certainly blocking it. Tell the user
        (once per stage) to move that sister spool and re-stage, or set the id by
        hand. No-op if there was no blocking sibling."""
        if sibling_slot is None or self._sister_hint_shown:
            return
        self._sister_hint_shown = True
        sib_lane = self._lane_at_slot(sibling_slot)
        sib_name = getattr(sib_lane, "name", None) or ("slot %s" % sibling_slot)
        act = getattr(active_lane, "name", None) or "this lane"
        self.gcode.respond_info(
            "AFC ACE2: couldn't read %s's RFID — lane %s's spool is on the shared "
            "reader blocking it. Manually move lane %s's spool a little and "
            "re-stage %s, or set %s's spool id by hand."
            % (act, sib_name, sib_name, act, act))

    def _reader_power_off(self, link: Any) -> None:
        """Power the reader down after a between-chunk read, WITHOUT tearing the
        probe down — the feeder keeps chunking and the next stop reads again.
        Best-effort: a wedged power-off must not add another hard failure."""
        try:
            link.reader_power(False)
        except Exception:
            self.logger.exception("ACE2 RFID: stage read reader power-off failed")

    def _safe_probe_teardown(self) -> None:
        """Release the active probe: settle, power the reader off, and restore
        firmware identify when configured. Best-effort and instrumented."""
        p = self._probe
        self._probe = None
        if not p:
            return
        link, shared = p["link"], p["shared"]
        # Instrumented so the klippy.log shows exactly which teardown step ran
        # last before a lock-up. Each step is logged before AND after.
        if self.probe_settle:
            self._dwell(self.probe_settle)
        if self.probe_power_off:
            self.logger.info("ACE2 RFID teardown: reader_power(off)...")
            try:
                link.reader_power(False)
                self.logger.info("ACE2 RFID teardown: reader_power(off) done")
            except Exception:
                self.logger.exception("ACE2 RFID: probe power-off failed")
        if self.probe_restore_identify:
            for s in shared:
                self.logger.info("ACE2 RFID teardown: set_rfid_enable(%d,on)...", s)
                try:
                    link.set_rfid_enable(s, True)
                    self.logger.info(
                        "ACE2 RFID teardown: set_rfid_enable(%d,on) done", s)
                except Exception:
                    self.logger.exception(
                        "ACE2 RFID: probe re-enable identify failed")
        else:
            self.logger.info("ACE2 RFID teardown: identify NOT restored (config)")

    # ── gcode ───────────────────────
    def cmd_ACE_RFID_READ(self, gcmd: "GCodeCommand") -> None:
        """Read the tag for a LANE= (or a raw SLOT=/REG= for diagnostics) and
        respond with the decoded uid, type, brand, material and colour. Reader
        glitches are caught so a bad read never shuts Klipper down.

        Usage: ACE_RFID_READ [LANE=<name>] [SLOT=<n>] [REG=<n>]
        Example: ACE_RFID_READ LANE=lane1
        """
        try:
            lane_name = gcmd.get("LANE", None)
            # REG= overrides the MFRC522 chip-select index for diagnostics (map
            # which reg index reads which physical slot's tag). -1 = no override.
            reg_slot = gcmd.get_int("REG", -1)
            reg_slot = None if reg_slot < 0 else reg_slot
            if lane_name is not None:
                tag = self.read_lane(lane_name)
            else:
                slot = gcmd.get_int("SLOT", 0)
                tag = self.read_slot(slot, reg_slot=reg_slot)
        except self.printer.command_error:
            raise                                 # normal user error (no shutdown)
        except Exception as e:
            # A serial timeout / reader glitch must NEVER shut Klipper down.
            self.logger.exception("ACE2 RFID read failed")
            gcmd.respond_info("ACE2 RFID: read error: %s" % (e,))
            return
        if not tag:
            gcmd.respond_info("ACE2 RFID: no tag found")
            return
        f = tag.get("filament") or {}
        gcmd.respond_info(
            "ACE2 RFID: uid=%s type=%s brand=%s material=%s color=%s" % (
                tag.get("uid", ""), tag.get("tag_type", ""),
                f.get("manufacturer", ""), f.get("type", ""),
                ("%06x" % ((f.get("color_argb") or 0) & 0xFFFFFF))
                if f.get("color_argb") is not None else ""))

    def cmd_ACE_RFID_BLOCKS(self, gcmd: "GCodeCommand") -> None:
        """Dump raw MIFARE blocks of a Bambu tag to confirm dual-colour. Reads
        block 5 (primary colour) and block 16 (colour count @258 + second colour
        @260 in reversed ABGR) by default; BLOCKS=5,16,17 overrides. LANE=/SLOT=.
        (Named ACE_RFID_BLOCKS to avoid the V1 unit's firmware ACE_RFID_DUMP.)

        Usage: ACE_RFID_BLOCKS [LANE=<name>] [SLOT=<n>] [BLOCKS=5,16]
        Example: ACE_RFID_BLOCKS LANE=lane1 BLOCKS=5,16
        """
        lane_name = gcmd.get("LANE", None)
        blocks_str = gcmd.get("BLOCKS", "5,16")
        try:
            blocks = tuple(int(b) for b in blocks_str.replace(" ", "").split(",")
                           if b != "")
        except ValueError:
            gcmd.respond_info("ACE2 RFID DUMP: bad BLOCKS=%r" % blocks_str)
            return
        try:
            if lane_name is not None:
                slot = self._slot_for_lane(lane_name)
                if slot is None:
                    raise self.printer.command_error(
                        "lane %r has no ACE2 reader slot (set lane_slot_map)"
                        % lane_name)
            else:
                slot = gcmd.get_int("SLOT", 0)
            tag = self.read_slot(int(slot), dump_blocks=blocks)
        except self.printer.command_error:
            raise
        except Exception as e:
            self.logger.exception("ACE2 RFID dump failed")
            gcmd.respond_info("ACE2 RFID DUMP: error: %s" % (e,))
            return
        if not tag:
            gcmd.respond_info("ACE2 RFID DUMP: no tag found")
            return
        raw = tag.get("raw_blocks") or {}
        lines = ["ACE2 RFID DUMP: uid=%s type=%s"
                 % (tag.get("uid", ""), tag.get("tag_type", ""))]
        b5 = bytes.fromhex(raw[5]) if raw.get(5) else b""
        b16 = bytes.fromhex(raw[16]) if raw.get(16) else b""
        if len(b5) >= 4:
            lines.append("  block5 primary -> #%02x%02x%02x (a=%02x)"
                         % (b5[0], b5[1], b5[2], b5[3]))
        if len(b16) >= 8:
            count = b16[2] | (b16[3] << 8)
            a2, bb2, g2, r2 = b16[4], b16[5], b16[6], b16[7]
            lines.append("  block16 fmt=%04x color_count=%d "
                         "second(ABGR bytes)=%02x%02x%02x%02x -> #%02x%02x%02x"
                         % (b16[0] | (b16[1] << 8), count, a2, bb2, g2, r2,
                            r2, g2, bb2))
            lines.append("  VERDICT: %s"
                         % ("DUAL-COLOR (count=%d)" % count if count >= 2
                            else "single colour (count=%d)" % count))
        for b in blocks:
            lines.append("  raw block %d: %s" % (b, raw.get(b, "(not read)")))
        f = tag.get("filament") or {}
        if f.get("colors_argb"):
            lines.append("  decoded colors: "
                         + ", ".join("#%06x" % (c & 0xFFFFFF)
                                     for c in f["colors_argb"]))
        gcmd.respond_info("\n".join(lines))

    def cmd_ACE_RFID_STAGE_TEST(self, gcmd: "GCodeCommand") -> None:
        """DIAGNOSTIC: exercise the continuous-feed stop-on-detect path on the
        ACE2. On a STAGED lane with room either way:
        retract DIST first, then feed DIST forward WHILE a reactor timer polls the
        reader; on the first UID detect, send STOP_FEED, then read the tag parked
        (retract 1mm and retry once on a read miss). Logs every step so we can see:
          - does the concurrent poll even fire during a blocking feed? (polls>0)
          - does STOP_FEED halt it with the tag in range?
          - does the parked read decode? does the 1mm-retract retry help?
          - does polling during the feed trip the stuck-spool watchdog?
        Best-effort re-stages the lane afterwards (rough — the lane has room)."""
        if self.ace2 is None:
            raise gcmd.error("ACE2 not available")
        lane_name = gcmd.get("LANE")
        dist = gcmd.get_float("DIST", 500.0, minval=10.0)
        slot = self._physical_slot(lane_name)
        if slot is None:
            raise gcmd.error("lane %r has no ACE2 slot (set lane_slot_map)"
                             % lane_name)
        conn = getattr(self.ace2, "_ace", None)
        if conn is None or not getattr(conn, "connected", False):
            raise gcmd.error("STAGE_TEST: ACE serial not connected")
        speed = gcmd.get_float(
            "SPEED", float(getattr(self.ace2, "feed_speed", 80.0)),
            minval=1.0, maxval=90.0)
        rspeed = float(getattr(self.ace2, "retract_speed", speed))
        log = gcmd.respond_info
        reg_idx = self._reg_index(slot) & 0xFF
        power_idx = (slot >> 1) & 0xFF
        link = _Ace2RegLink(self.ace2, reg_idx, power_index=power_idx,
                            reg_timeout=self.read_reg_timeout)
        pair = (power_idx * 2, power_idx * 2 + 1)

        wait_ready = getattr(self.ace2, "_wait_for_ace_ready", None)

        def _ready(where: str) -> None:
            # feed/unwind are acked before the motion finishes, so issuing the
            # next command too soon gets error_2 (busy). Wait for the unit to
            # report ready between steps.
            if callable(wait_ready):
                try:
                    if not wait_ready(timeout=self.read_reg_timeout + 20.0):
                        log("STAGE_TEST: (unit not ready after %s)" % where)
                except Exception:
                    pass

        log("STAGE_TEST %s (slot %d): retracting %.0fmm to make room..."
            % (lane_name, slot, dist))
        try:
            conn.unwind_filament(slot, dist, rspeed)
        except Exception as e:
            raise gcmd.error("STAGE_TEST: initial retract failed: %s" % e)
        _ready("retract")

        # Host-own the reader for the whole feed (stop the firmware identify loop
        # on both shared slots, then power the reader ourselves).
        for s in pair:
            link.set_rfid_enable(s, False)
        self._dwell(0.1)
        link.reader_power(True)
        self._dwell(0.1)
        _ready("reader-setup")

        state = {"detected": None, "polls": 0, "stopped": False, "moved": 0.0}

        # The ACE2 firmware ACKs feed_filament immediately and runs the physical
        # motion asynchronously — so we CANNOT rely on a concurrent reactor timer
        # (the feed call returns before the spool even moves, tearing the timer
        # down). Instead: fire the feed, then poll the reader ourselves inside a
        # reactor.pause loop for the motion's duration (mirrors
        # _wait_for_feed_complete), stopping the feed on the first UID.
        FEED_CEILING = 90.0
        eff_speed = min(max(speed, 1.0), FEED_CEILING)
        expected = dist / eff_speed
        get_status = getattr(conn, "get_status", None)
        slot_moving = getattr(self.ace2, "_slot_is_moving", None)

        log("STAGE_TEST: feeding %.0fmm forward while polling the reader..." % dist)
        feed_ok = True
        t0 = self.reactor.monotonic()
        try:
            conn.feed_filament(slot, dist, speed)
        except Exception as e:
            feed_ok = False
            log("STAGE_TEST: feed returned/err: %s" % e)

        if feed_ok:
            deadline = self.reactor.monotonic() + expected * 1.5 + 15.0
            started = False
            idle = 0
            while self.reactor.monotonic() < deadline:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                state["polls"] += 1
                try:
                    uid, _sak = MifareClassic(Mfrc522(link)).activate()
                except Exception:
                    uid = None
                if uid is not None:
                    state["detected"] = uid.hex()
                    state["stopped"] = True
                    state["moved"] = min(
                        dist, (self.reactor.monotonic() - t0) * eff_speed)
                    log("STAGE_TEST: poll #%d DETECTED uid=%s -> STOP_FEED "
                        "(~%.0fmm in)"
                        % (state["polls"], uid.hex(), state["moved"]))
                    try:
                        conn.stop_feed_filament(slot)
                    except Exception as e:
                        log("STAGE_TEST: stop_feed error: %s" % e)
                    break
                # Track motion so we can end the loop when the feed finishes with
                # no tag seen (rather than waiting out the full deadline).
                if callable(slot_moving) and callable(get_status):
                    try:
                        moving = slot_moving(get_status(timeout=2.0), slot)
                    except Exception:
                        moving = True
                    if moving:
                        started = True
                        idle = 0
                    elif started:
                        idle += 1
                        if idle >= 2:
                            state["moved"] = dist
                            break
            if not state["stopped"] and state["moved"] == 0.0:
                state["moved"] = dist
        log("STAGE_TEST: feed ended — feed_ok=%s polls=%d detected=%s stopped=%s "
            "moved=~%.0fmm"
            % (feed_ok, state["polls"], state["detected"], state["stopped"],
               state["moved"]))

        # Watchdog / stuck-spool check (best-effort).
        checker = getattr(self.ace2, "_slot_in_error", None)
        if callable(checker):
            try:
                log("STAGE_TEST: slot %d in error (watchdog)? %s"
                    % (slot, bool(checker(slot))))
            except Exception:
                pass

        # Parked read. The tag was detected AS IT SWEPT PAST the antenna while the
        # spool was spinning, then coasted forward past the read zone before STOP
        # took effect. Try a few reads at rest, then step the tag back (small
        # retracts reverse the spin) to re-center it in the read zone, reading at
        # each step. Track the extra retract so the re-stage feeds it back.
        if state["detected"]:
            def _try_read() -> Optional[dict]:
                try:
                    t = self._read_tag(link)
                except Exception as e:
                    log("STAGE_TEST: read error: %s" % e)
                    return None
                return t if (t and t.get("filament")) else None

            tag = None
            for _ in range(3):                    # read is flaky at rest — retry
                tag = _try_read()
                if tag:
                    break
                self._dwell(0.12)
            nudge_total = 0.0
            if not tag:
                log("STAGE_TEST: parked read missed — stepping the tag back to "
                    "re-center it in the read zone...")
                step = 3.0
                while nudge_total < 42.0 and not tag:
                    try:
                        conn.unwind_filament(slot, step, rspeed)
                    except Exception as e:
                        log("STAGE_TEST: nudge error: %s" % e)
                        break
                    nudge_total += step
                    self._dwell(0.15)
                    tag = _try_read()
                    if tag:
                        log("STAGE_TEST: re-centered after %.0fmm retract"
                            % nudge_total)
            # the re-centering retract adds to the shortfall the re-stage restores
            state["moved"] -= nudge_total
            decoded = bool(tag and tag.get("filament"))
            log("STAGE_TEST: parked read decoded=%s -> %s"
                % (decoded, (tag or {}).get("filament")))
        else:
            log("STAGE_TEST: no tag detected during the feed")

        # Restore the reader: power it off, and ONLY re-enable the firmware
        # identify if restore_identify is set (default False when
        # skip_factory_autostage). Re-enabling it unconditionally left the
        # factory identify running on the next insert — leave it disabled to
        # match the normal read_slot teardown.
        try:
            link.reader_power(False)
        except Exception:
            pass
        if self.probe_restore_identify:
            for s in pair:
                try:
                    link.set_rfid_enable(s, True)
                except Exception:
                    pass

        # Re-stage: we retracted `dist` up front, then fed `moved` forward before
        # stopping (moved==dist if the feed ran to completion). Feed the shortfall
        # (dist - moved) to return the lane to where it started. A completed feed
        # needs nothing; a stop-on-detect or feed error needs the remainder.
        remaining = max(0.0, dist - state["moved"])
        if remaining > 1.0:
            _ready("pre-restage")
            log("STAGE_TEST: re-feeding %.0fmm to re-stage the lane..." % remaining)
            try:
                conn.feed_filament(slot, remaining, speed)
            except Exception as e:
                log("STAGE_TEST: re-stage feed err: %s — re-stage lane1 manually!"
                    % e)
        log("STAGE_TEST: DONE — verify the lane position.")

    def cmd_ACE_RFID_SCAN(self, gcmd: "GCodeCommand") -> None:
        """Watch a scanner LANE= for a presented tag for up to SECONDS= and stage
        it as AFC's next spool id. Reader glitches are caught so a bad scan never
        shuts Klipper down.

        Usage: ACE_RFID_SCAN [LANE=<name>] [SECONDS=<n>]
        Example: ACE_RFID_SCAN LANE=scanner1 SECONDS=30
        """
        # LANE= a scanner lane; defaults to the sole configured scanner lane.
        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            if len(self._scanner_lanes) == 1:
                lane_name = next(iter(self._scanner_lanes))
            else:
                raise self.printer.command_error(
                    "ACE_RFID_SCAN requires LANE= (no single scanner_lanes lane)")
        seconds = gcmd.get_float("SECONDS", self.scan_seconds,
                                 minval=1.0, maxval=600.0)
        gcmd.respond_info(
            "ACE2 RFID scan: present the tag to %s (scanning %.0fs)..." % (
                lane_name, seconds))
        try:
            tag = self.scan_lane(lane_name, seconds)
        except self.printer.command_error:
            raise                                 # normal user error (no shutdown)
        except Exception as e:
            # A serial timeout / reader glitch must NEVER shut Klipper down.
            self.logger.exception("ACE2 RFID scan failed")
            gcmd.respond_info("ACE2 RFID scan: error: %s" % (e,))
            return
        if not tag:
            gcmd.respond_info("ACE2 RFID scan: no tag found on %s" % lane_name)
            return
        f = tag.get("filament") or {}
        spool = getattr(self.afc, "spool", None) if self.afc is not None else None
        nid = getattr(spool, "next_spool_id", None)
        gcmd.respond_info(
            "ACE2 RFID scan: staged next spool from %s — uid=%s type=%s%s" % (
                lane_name, tag.get("uid", ""), f.get("type", ""),
                (" (spool #%s)" % nid) if nid else ""))


def load_config(config: "ConfigWrapper") -> AFC_ACE2_RFID:
    """Klipper entry point that builds the ACE2 RFID coordinator when this
    module's config section is loaded from the printer config.

    :param config: Klipper config section for this module
    :return AFC_ACE2_RFID: The configured RFID coordinator object
    """
    return AFC_ACE2_RFID(config)
