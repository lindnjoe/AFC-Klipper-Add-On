"""
Driver-level verification of the shared RFID reader stack in
extras/AFC_rfid_readers.py, plus the ACE2 frame layer that feeds it.

The FakeReader emulates an MFRC522 + a synthetic Bambu MIFARE Classic 1K (or
Anycubic NTAG) tag at the *register* level, so the REAL driver code (Mfrc522 +
MifareClassic + Bambu/Anycubic decode) runs unmodified. The only thing stubbed
is reg_read/reg_write — exactly the firmware contract.
"""
import hashlib
import importlib.util
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _load(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


A = _load("AFC_rfid_readers", "extras/AFC_rfid_readers.py")
ACE2 = _load("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")


# ── build synthetic tags with known fields ──────────────────────────────────
def make_bambu_tag():
    d = bytearray(1024)
    d[32:32 + 3] = b"PLA"                       # block2 type
    d[64:64 + 9] = b"PLA Basic"                 # block4 detailed
    d[80:84] = bytes([0x00, 0x86, 0xD6, 0xFF])  # block5 color RGBA -> 0086D6
    struct.pack_into("<H", d, 84, 1000)         # weight g
    struct.pack_into("<f", d, 88, 1.75)         # diameter mm
    struct.pack_into("<H", d, 96, 55)           # drying temp
    struct.pack_into("<H", d, 98, 8)            # drying time
    struct.pack_into("<H", d, 102, 55)          # bed temp
    struct.pack_into("<H", d, 104, 230)         # hotend max
    struct.pack_into("<H", d, 106, 190)         # hotend min
    d[192:192 + 16] = b"2026_03_20_11_48\0"[:16]  # production
    return bytes(d)

MASTER = bytes.fromhex("9A759CF2C4F7CAFF222CB9769B41BC96")
UID = bytes([0x04, 0xA1, 0xB2, 0xC3])


def make_anycubic_tag():
    d = bytearray(0x80)
    d[0x10:0x14] = A.ANYCUBIC_MAGIC
    d[0x14:0x14 + 9] = b"HPL19-102"                 # SKU
    d[0x28:0x28 + 8] = b"Anycubic"                  # brand
    d[0x3C:0x3C + 3] = b"PLA"                        # type
    d[0x50:0x54] = bytes([0xFF, 0xD6, 0x86, 0x00])  # a,b,g,r -> argb FF0086D6
    struct.pack_into("<H", d, 0x60, 190)            # hotend min
    struct.pack_into("<H", d, 0x62, 230)            # hotend max
    struct.pack_into("<H", d, 0x76, 60)             # bed
    struct.pack_into("<H", d, 0x78, 175)            # diameter*100
    struct.pack_into("<H", d, 0x7A, 330)            # length -> 1000g
    return bytes(d)


class FakeReader:
    """Register-level MFRC522 emulator holding one tag (Classic or Ultralight)."""
    def __init__(self, uid, tag, master, sak=0x08):
        self.uid = uid
        self.tag = tag
        self.sak = sak
        self.ntag = (sak == 0x00)
        self.keys = A.bambu_keys(uid, master) if master else None
        self.reg = {i: 0 for i in range(0x40)}
        self.txfifo = bytearray()
        self.rxfifo = bytearray()
        self.crypto_sector = None

    # firmware contract:
    def reg_read(self, r):
        if r == A.FIFOLevelReg:
            return len(self.rxfifo)
        if r == A.FIFODataReg:
            return self.rxfifo.pop(0) if self.rxfifo else 0
        if r == A.ControlReg:
            return 0                               # rx_last_bits = 0
        return self.reg.get(r, 0)

    def reg_write(self, r, v):
        v &= 0xFF
        if r == A.FIFOLevelReg and (v & 0x80):
            self.txfifo.clear(); return
        if r == A.FIFODataReg:
            self.txfifo.append(v); return
        if r == A.CommandReg and v in (A.PCD_TRANSCEIVE, A.PCD_MFAUTHENT):
            self._exec(v)
        self.reg[r] = v

    def _set_rx(self, data):
        self.rxfifo = bytearray(data)
        self.reg[A.ComIrqReg] = 0x20               # RxIRq
        self.reg[A.ErrorReg] = 0

    def _exec(self, cmd):
        buf = bytes(self.txfifo); self.txfifo.clear()
        self.reg[A.ComIrqReg] = 0x10               # IdleIRq default
        self.reg[A.ErrorReg] = 0
        if cmd == A.PCD_MFAUTHENT:
            key_type, block = buf[0], buf[1]
            key, uid = buf[2:8], buf[8:12]
            sector = block // 4
            if key_type == A.PICC_AUTH_KEY_A and list(key) == self.keys[sector] and uid == self.uid:
                self.crypto_sector = sector
                self.reg[A.Status2Reg] = 0x08       # MFCrypto1On
            else:
                self.reg[A.Status2Reg] = 0x00
            return
        # Transceive
        if len(buf) == 1 and buf[0] in (A.PICC_REQA, A.PICC_WUPA):
            self._set_rx(b"\x04\x00"); return       # ATQA
        if buf[:2] == bytes([A.PICC_ANTICOLL, 0x20]):
            bcc = self.uid[0] ^ self.uid[1] ^ self.uid[2] ^ self.uid[3]
            self._set_rx(self.uid + bytes([bcc])); return
        if buf[:2] == bytes([A.PICC_SELECT, 0x70]):
            self._set_rx(bytes([self.sak, 0, 0])); return   # SAK (+crc)
        if buf[0] == A.PICC_READ:
            arg = buf[1]
            if self.ntag:                            # NTAG: arg=page(4B), 4 pages back
                off = arg * 4
                self._set_rx(self.tag[off:off + 16] + b"\x00\x00"); return
            sector = arg // 4                        # Classic: arg=block(16B), needs auth
            if self.crypto_sector == sector:
                self._set_rx(self.tag[arg * 16:arg * 16 + 16] + b"\x00\x00"); return
        self.reg[A.ComIrqReg] = 0x01                # TimerIRq (no reply)


# ── tests ───────────────────────────────────────────────────────────────────
def test_key_matches_bambu_salt_hash():
    assert hashlib.sha256(MASTER).hexdigest() == A.BAMBU_SALT_HASH


def test_hkdf_rfc5869_vector():
    got = A.hkdf_sha256(bytes.fromhex("000102030405060708090a0b0c"),
                        bytes.fromhex("0b" * 22), bytes.fromhex("f0f1f2f3f4f5f6f7f8f9"), 42)
    assert got.hex() == ("3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5"
                         "db02d56ecc4c5bf34007208d5b887185865")


def test_frame_crc_matches_firmware_algo():
    link = ACE2.Ace2Link(lambda f: f)
    f = link.build_frame(0x06, b"")
    assert f[:2] == ACE2.PREAMBLE and f[-1:] == ACE2.END
    body = f[3:-3]
    assert struct.unpack("<H", f[-3:-1])[0] == ACE2.crc16_kermit(body)


def test_reader_power_frame_encodes_cmd_0x52():
    sent = {}
    link = ACE2.Ace2Link(lambda f: sent.setdefault("frame", f) or f, slot=1)
    link.reader_power(True)
    frame = sent["frame"]
    # FF AA seq TYPE(2) CMD LEN PAYLOAD... — CMD byte is at index 5
    assert frame[5] == ACE2.CMD_MFRC522_READER_POWER == 0x52
    # payload varint = (1<<16)|1
    assert frame[7] == 0x08 and ACE2._varint_decode(frame, 8)[0] == (1 << 16) | 1


def test_reg_rw_roundtrip_through_link():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    fake.reg_write(A.TxControlReg, 0x03)
    assert fake.reg_read(A.TxControlReg) == 0x03


def test_full_read_and_decode_end_to_end():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    out = A.read_bambu(fake, MASTER)               # REAL driver, emulated reader
    assert out is not None, "read failed"
    assert out["uid"] == UID.hex()
    assert out["type"] == "PLA"
    assert out["detailed"] == "PLA Basic"
    assert out["color_argb"] == 0xFF0086D6
    assert out["weight_g"] == 1000
    assert out["diameter_mm"] == 1.75
    assert out["hotend_min_c"] == 190 and out["hotend_max_c"] == 230
    assert out["bed_temp_c"] == 55
    assert out["production"] == "2026_03_20_11_48"


def test_wrong_master_key_fails_auth():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    out = A.read_bambu(fake, bytes.fromhex("00" * 16))  # wrong key -> auth fails
    assert out is None


def test_anycubic_ntag_end_to_end():
    fake = FakeReader(UID, make_anycubic_tag(), master=None, sak=0x00)
    out = A.read_tag(fake)                          # no key needed for NTAG
    assert out is not None
    assert out["uid"] == UID.hex()
    assert out["tag_type"] == "MifareUltralight"
    f = out["filament"]
    assert f is not None and f["manufacturer"] == "Anycubic"
    assert f["sku"] == "HPL19-102" and f["type"] == "PLA"
    assert f["color_argb"] == 0xFF0086D6
    assert f["diameter_mm"] == 1.75 and f["weight_g"] == 1000
    assert f["hotend_min_c"] == 190 and f["hotend_max_c"] == 230


def test_read_tag_classic_bambu_with_key():
    fake = FakeReader(UID, make_bambu_tag(), MASTER, sak=0x08)
    out = A.read_tag(fake, MASTER)
    assert out["tag_type"] == "MifareClassic1k"
    assert out["filament"]["type"] == "PLA" and out["filament"]["weight_g"] == 1000


def test_read_tag_returns_uid_even_without_key():
    # Classic (Bambu) tag but no key supplied: still get UID + tag type, no decode
    fake = FakeReader(UID, make_bambu_tag(), MASTER, sak=0x08)
    out = A.read_tag(fake, bambu_master_key=None)
    assert out["uid"] == UID.hex()
    assert out["tag_type"] == "MifareClassic1k"
    assert out["filament"] is None


# ── shared-reader exclusion: HALT the neighbour tag, read this lane's own ────
class _FakeMfrc:
    """Mfrc522 stand-in modelling a shared antenna with several tags. Supports
    WUPA/REQA/anticoll/select/halt so activate()'s exclusion loop can be driven
    without register-level plumbing. A halted tag ignores REQA until a WUPA."""

    def __init__(self, tags):
        self.tags = list(tags)                 # [(uid_bytes, sak), ...], field order
        self.halted = set()
        self.current = None
        self.resets = 0

    def reset(self):
        self.resets += 1

    def stop_crypto(self):
        self.crypto_stops = getattr(self, "crypto_stops", 0) + 1

    def antenna_on(self):
        pass

    def _live(self):
        return [t for t in self.tags if t[0] not in self.halted]

    def request(self, req):
        if req == A.PICC_WUPA:
            self.halted.clear()                # WUPA wakes halted tags
        return b"\x04\x00" if self._live() else None

    def anticoll(self):
        live = self._live()
        if not live:
            return None
        self.current = live[0][0]
        return self.current

    def select(self, uid):
        for u, s in self.tags:
            if u == uid:
                return s
        return None

    def halt(self):
        if self.current is not None:
            self.halted.add(self.current)


NEIGH = b"\xaa\xaa\xaa\xaa"
OWN = b"\xbb\xbb\xbb\xbb"


def test_activate_halts_excluded_neighbour_and_returns_own():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate(is_excluded=lambda u: u == NEIGH.hex())
    assert uid == OWN and sak == 0x08          # neighbour skipped, own tag read
    assert NEIGH in fake.halted                # silenced via HLTA


def test_activate_returns_none_when_only_excluded_present():
    fake = _FakeMfrc([(NEIGH, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate(is_excluded=lambda u: u == NEIGH.hex())
    assert uid is None and sak is None         # neighbour-only -> nothing to read
    assert NEIGH in fake.halted


def test_activate_without_excluder_returns_first_tag():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate()
    assert uid == NEIGH and sak == 0x08        # no exclusion: first tag wins


def test_activate_seen_records_halted_neighbour_and_own():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    seen = []
    mc.activate(is_excluded=lambda u: u == NEIGH.hex(), seen=seen)
    # both tags are recorded, the neighbour flagged excluded, own tag not
    assert (NEIGH.hex(), 0x08, True) in seen
    assert (OWN.hex(), 0x08, False) in seen


def test_activate_seen_empty_when_field_empty():
    fake = _FakeMfrc([])                        # nothing in the field
    mc = A.MifareClassic(fake)
    seen = []
    uid, _ = mc.activate(is_excluded=lambda u: False, seen=seen)
    assert uid is None and seen == []           # empty field -> nothing seen


class _CaptureLink:
    """reg link that lets _to_card complete and records FIFO writes."""

    def __init__(self):
        self.fifo = []

    def reg_read(self, r):
        if r == A.ComIrqReg:
            return 0x30                        # Rx/Idle set -> poll exits at once
        return 0

    def reg_write(self, r, v):
        if r == A.FIFODataReg:
            self.fifo.append(v)


def test_halt_sends_hlta_frame():
    link = _CaptureLink()
    A.Mfrc522(link).halt()
    assert link.fifo[:2] == [0x50, 0x00]       # HLTA opcode
    assert len(link.fifo) == 4                  # opcode + 2 CRC bytes


# ── Snapmaker U1 MIFARE Classic decode + HKDF key derivation ────────────────
def _make_snapmaker_tag():
    d = bytearray(1024)
    d[16:16 + 9] = b"Snapmaker"                 # vendor
    d[66] = 1                                    # MAIN_TYPE PLA (LE u16)
    d[68] = 2                                    # SUB_TYPE Matte
    d[73] = 0                                    # ALPHA byte -> 0xFF - 0 = 0xFF
    d[80], d[81], d[82] = 0x00, 0x00, 0xFF       # RGB_1 = blue
    d[96:100] = (12345).to_bytes(4, "little")    # SKU
    d[128:130] = (175).to_bytes(2, "little")     # diameter -> 1.75mm
    d[130:132] = (1000).to_bytes(2, "little")    # weight g
    d[148:150] = (220).to_bytes(2, "little")     # hotend max
    d[150:152] = (190).to_bytes(2, "little")     # hotend min
    d[154:156] = (60).to_bytes(2, "little")      # bed temp
    d[160:168] = b"20250101"                     # mfg date
    return bytes(d)


def test_decode_snapmaker_fields():
    f = A.decode_snapmaker(_make_snapmaker_tag())
    assert f["manufacturer"] == "Snapmaker"
    assert f["type"] == "PLA" and f["detailed"] == "Matte"
    assert f["color_argb"] == 0xFF0000FF
    assert f["weight_g"] == 1000 and f["diameter_mm"] == 1.75
    assert f["hotend_max_c"] == 220 and f["hotend_min_c"] == 190
    assert f["bed_temp_c"] == 60
    assert f["sku"] == "12345" and f["production"] == "20250101"


def test_decode_snapmaker_rejects_non_snapmaker():
    assert A.decode_snapmaker(bytes(1024)) is None      # main type 0 -> not ours


def test_snapmaker_key_derivation_matches_spec():
    import hmac
    import hashlib
    uid = bytes.fromhex("80dcf43e")
    keys = A.snapmaker_keys(uid)
    assert len(keys) == 16
    prk = hmac.new(A.SNAPMAKER_SALT_A, uid, hashlib.sha256).digest()
    expect0 = hmac.new(prk, b"key_a_0" + bytes([1]), hashlib.sha256).digest()[:6]
    expect3 = hmac.new(prk, b"key_a_3" + bytes([1]), hashlib.sha256).digest()[:6]
    assert bytes(keys[0]) == expect0
    assert bytes(keys[3]) == expect3


class _ReMifare:
    """Fake MifareClassic serving one Snapmaker tag. Bambu auth (its keys) fails,
    Snapmaker keys pass — so the tag is only decoded on the SECOND scheme, which
    means it exercises the re-select. Records the excluder passed to each
    activate() so the test can prove the re-select does not use the caller's
    (sibling) predicate."""

    def __init__(self, uid, image):
        self._uid = uid
        self._image = image
        self.activate_excluders = []
        self.activate_resets = []

    def activate(self, is_excluded=None, seen=None, reset=True):
        self.activate_excluders.append(is_excluded)
        self.activate_resets.append(reset)
        # Model the exclusion loop: a tag the predicate excludes is halted and
        # nothing readable remains.
        if is_excluded is not None and is_excluded(self._uid.hex()):
            if seen is not None:
                seen.append((self._uid.hex(), 0x08, True))
            return None, None
        if seen is not None:
            seen.append((self._uid.hex(), 0x08, False))
        return self._uid, 0x08                        # Classic 1K

    def read_blocks(self, uid, keys_a, blocks):
        return self._image if keys_a == A.snapmaker_keys(uid) else None

    def read_ntag(self, n=128):
        return None


def test_reselect_pins_to_active_uid_not_sibling_predicate(monkeypatch):
    # Regression: a Snapmaker tag is only reached on the 2nd scheme (Bambu fails
    # first). The re-select before it must pin to the tag just activated, NOT
    # re-run the caller's sibling predicate — which can resolve the active tag's
    # own UID to the neighbour and halt it, leaving the lane with defaults.
    uid = b"\xbb\xbb\xbb\xbb"
    fake = _ReMifare(uid, _make_snapmaker_tag())
    monkeypatch.setattr(A, "Mfrc522", lambda link: None)
    monkeypatch.setattr(A, "MifareClassic", lambda mfrc: fake)

    # A sibling predicate that only misfires on the RE-select (the first
    # activate saw a clean field; sibling state changed by the time we re-select
    # for the 2nd scheme). With the bug (re-select uses this predicate) the
    # active tag is halted on re-select and Snapmaker never decodes.
    calls = {"n": 0}

    def sibling_pred(h):
        calls["n"] += 1
        return h == uid.hex() and calls["n"] > 1

    out = A.read_tag(object(), bambu_master_key=MASTER, is_excluded=sibling_pred)

    # Snapmaker still decodes: the re-select pinned to the active UID
    assert out["filament"] is not None
    assert out["filament"]["manufacturer"] == "Snapmaker"
    # first activate used the caller's predicate; the re-select did NOT
    assert fake.activate_excluders[0] is sibling_pred
    reselect = fake.activate_excluders[1]
    assert reselect(uid.hex()) is False              # active tag is kept
    assert reselect("cccccccc") is True              # a real neighbour still halts
    # first activate does a full reset (cold bring-up); the re-select keeps the
    # RF field UP (reset=False) so a marginal at-rest tag isn't unpowered
    assert fake.activate_resets[0] is True
    assert fake.activate_resets[1] is False


def test_activate_reset_false_keeps_field_no_soft_reset():
    fake = _FakeMfrc([(OWN, 0x08)])
    mc = A.MifareClassic(fake)
    mc.activate(reset=True)                           # cold bring-up
    assert fake.resets == 1
    mc.activate(reset=False)                          # re-select, field stays up
    assert fake.resets == 1                           # no additional SoftReset
    assert getattr(fake, "crypto_stops", 0) == 1      # crypto ended instead


# ── AES-128 + Creality CFS + Elegoo decode ──────────────────────────────────
def test_aes128_fips197_vector():
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    ct = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
    assert A._aes_encrypt_block(pt, key) == ct
    assert A._aes_decrypt_block(ct, key) == pt


def test_aes_cbc_decrypt_roundtrip():
    key = bytes.fromhex("484043466b526e7a404b4174424a7032")
    data = bytes(range(48))

    def cbc_enc(d, k, iv=b"\x00" * 16):
        out, prev = bytearray(), iv
        for i in range(0, len(d), 16):
            b = bytes(x ^ y for x, y in zip(d[i:i + 16], prev))
            e = A._aes_encrypt_block(b, k)
            out += e
            prev = e
        return bytes(out)

    assert A._aes_cbc_decrypt(cbc_enc(data, key), key) == data


def test_creality_key_derivation_matches_reference():
    u_key = bytes.fromhex("713362755e74316e71665a2870662431")
    keyA = A.creality_mifare_key(bytes.fromhex("60EA1221"), u_key)
    assert keyA.hex() == "1f1e83a97182"        # exact value from TAG_FORMAT.md


def test_decode_creality_fields():
    payload = (b"ABC21" + b"0276" + b"01" + b"101001" + b"0FF5F0B"
               + b"0165" + b"736314" + b"\x00" * 14)
    assert len(payload) == 48
    f = A.decode_creality(payload)
    assert f["manufacturer"] == "Creality"
    assert f["type"] == "PLA" and f["sku"] == "101001"
    assert f["color_argb"] == 0xFFFF5F0B
    assert f["weight_g"] == 500                 # 0x0165 = 357 -> 500g spool
    assert f["serial"] == "736314"


def test_decode_creality_rejects_garbage():
    assert A.decode_creality(b"\x00" * 48) is None


def test_decode_creality_end_to_end_encrypted():
    # full path: build plaintext, AES-CBC encrypt with d_key, decrypt+decode
    d_key = bytes.fromhex("484043466b526e7a404b4174424a7032")
    payload = (b"ABC21" + b"0276" + b"01" + b"101002" + b"000FF00"
               + b"0330" + b"000123" + b"\x00" * 14)

    def cbc_enc(dat, k, iv=b"\x00" * 16):
        out, prev = bytearray(), iv
        for i in range(0, len(dat), 16):
            b = bytes(x ^ y for x, y in zip(dat[i:i + 16], prev))
            e = A._aes_encrypt_block(b, k)
            out += e
            prev = e
        return bytes(out)

    f = A.decode_creality(A._aes_cbc_decrypt(cbc_enc(payload, d_key), d_key))
    assert f["type"] == "PETG"                  # 101002
    assert f["color_argb"] == 0xFF00FF00        # 000FF00 -> #00FF00
    assert f["weight_g"] == 1000                 # 0x0330 = 816 -> 1kg


def test_decode_elegoo_fields():
    d = bytearray(64)
    d[16] = 0x36
    d[17:21] = b"\xee\xee\xee\xee"
    d[21:23] = b"\x00\x01"
    d[23:27] = b"PLA "                           # 0x504C4120
    d[27:31] = b"CF20"
    d[31:34] = bytes([0xFF, 0x37, 0x00])
    d[34:36] = (175).to_bytes(2, "big")
    d[36:38] = (1000).to_bytes(2, "big")
    d[38:40] = (2502).to_bytes(2, "big")
    f = A.decode_elegoo(bytes(d))
    assert f["manufacturer"] == "Elegoo"
    assert f["type"] == "PLA" and f["detailed"] == "CF20"
    assert f["color_argb"] == 0xFFFF3700
    assert f["diameter_mm"] == 1.75 and f["weight_g"] == 1000
    assert f["production"] == "2502"


def test_decode_elegoo_rejects_non_elegoo():
    assert A.decode_elegoo(bytes(64)) is None    # header != 0x36
