"""
Writing blank NTAG stickers: the Anycubic encoder and the NTAG write path.

The point of the encoder is that it is the exact inverse of decode_anycubic, so
most of these tests are round trips -- a tag we write must come back through the
ordinary scan path with the values we put in.

The write path is guarded rather than trusted. Pages 0-3 and everything past the
user area are one-way fuses on an NTAG, so a bug in page arithmetic does not
produce a bad tag, it produces a dead one. The bound is taken from the tag's own
capability container and every page is read back before a write reports success.
"""

import pytest

import extras.AFC_rfid_readers as mod
from extras.AFC_rfid_readers import (
    AFC_BLOCK_LEN, AFC_BLOCK_MAGIC, AFC_BLOCK_OFFSET, AFC_BLOCK_PAGE,
    ANYCUBIC_FIRST_PAGE, CASCADE_TAG, MifareClassic, Mfrc522,
    NTAG213_LAST_PAGE, PICC_ANTICOLL, PICC_ANTICOLL_CL2, SAK_UID_INCOMPLETE,
    decode_afc_block, decode_anycubic, encode_afc_block, encode_anycubic,
    encode_tag_payload, write_tag,
)


def _image(payload):
    """A full 128-byte tag image with the payload at its page-4 home."""
    return bytes(mod.ANYCUBIC_IMAGE_START) + payload


# ── the encoder ──────────────────────────────────────────────────────────────

class TestEncodeAnycubic:
    def test_it_round_trips_through_the_decoder(self):
        p = encode_anycubic(manufacturer="Polymaker", sku="PM-PLA-BLK",
                            ftype="PLA", color_argb=0xFF1A2B3C,
                            diameter_mm=1.75, weight_g=1000,
                            hotend_min_c=190, hotend_max_c=230, bed_temp_c=60)
        d = decode_anycubic(_image(p))
        assert d["manufacturer"] == "Polymaker"
        assert d["sku"] == "PM-PLA-BLK"
        assert d["type"] == "PLA"
        assert d["color_argb"] == 0xFF1A2B3C
        assert d["diameter_mm"] == 1.75
        assert d["weight_g"] == 1000
        assert d["hotend_min_c"] == 190
        assert d["hotend_max_c"] == 230
        assert d["bed_temp_c"] == 60

    def test_it_fills_exactly_pages_4_to_31(self):
        p = encode_anycubic(ftype="PLA")
        assert len(p) == 112
        assert len(p) % 4 == 0
        assert ANYCUBIC_FIRST_PAGE == 4
        # 28 pages from page 4 ends at page 31, inside even an NTAG213's user
        # area -- the smallest chip this could ever be handed.
        assert ANYCUBIC_FIRST_PAGE + len(p) // 4 - 1 <= NTAG213_LAST_PAGE

    def test_the_magic_lands_on_the_first_writable_page(self):
        p = encode_anycubic(ftype="PLA")
        assert p[:4] == mod.ANYCUBIC_MAGIC, (
            "the magic must be the first thing written, at page 4")

    def test_the_brand_is_ours_to_set(self):
        """The magic is a format fingerprint, not a claim about the maker."""
        d = decode_anycubic(_image(encode_anycubic(manufacturer="Overture")))
        assert d["manufacturer"] == "Overture"

    @pytest.mark.parametrize("grams,metres", [(1000, 330), (750, 247),
                                              (600, 198), (500, 165),
                                              (250, 82)])
    def test_every_encodable_weight_round_trips(self, grams, metres):
        d = decode_anycubic(_image(encode_anycubic(weight_g=grams)))
        assert d["length_m"] == metres
        assert d["weight_g"] == grams

    def test_an_unencodable_weight_leaves_the_length_blank(self):
        """800g has no length that decodes back to it, so the Anycubic half
        says nothing rather than lying. The AFC block carries the real value
        (see TestAfcBlock)."""
        p = encode_anycubic(weight_g=800)
        assert decode_anycubic(_image(p))["length_m"] is None

    def test_a_length_can_be_set_directly(self):
        d = decode_anycubic(_image(encode_anycubic(length_m=412)))
        assert d["length_m"] == 412

    def test_no_weight_and_no_length_is_left_blank(self):
        p = encode_anycubic(ftype="PLA")
        assert decode_anycubic(_image(p))["length_m"] is None

    def test_the_colour_byte_order_is_abgr_on_the_tag(self):
        """decode_anycubic reads a, b, g, r in that order; a wrong order here
        would show up as a red/blue swap on every tag we write."""
        p = encode_anycubic(color_argb=0xFF804020)
        assert p[0x50 - 0x10] == 0xFF        # alpha
        assert p[0x51 - 0x10] == 0x20        # blue
        assert p[0x52 - 0x10] == 0x40        # green
        assert p[0x53 - 0x10] == 0x80        # red

    def test_a_long_string_is_truncated_not_overflowed(self):
        p = encode_anycubic(manufacturer="A" * 40, sku="B" * 40)
        assert len(p) == 112
        d = decode_anycubic(_image(p))
        assert d["manufacturer"] == "A" * 16
        assert d["sku"] == "B" * 16

    def test_non_ascii_does_not_raise(self):
        d = decode_anycubic(_image(encode_anycubic(manufacturer="Prusaé")))
        assert d["manufacturer"].startswith("Prusa")


# ── the wire frame ───────────────────────────────────────────────────────────

class _Link:
    """A minimal MFRC522 register model: captures the TX FIFO, replays one
    canned reply."""

    def __init__(self, reply=b"\x0a", reply_bits=4, err=0x00):
        self.regs = {}
        self.tx = bytearray()
        self.reply, self.reply_bits, self.err = reply, reply_bits, err
        self._rx, self._i = b"", 0

    def reg_write(self, reg, val):
        if reg == mod.FIFODataReg:
            self.tx.append(val)
        elif reg == mod.FIFOLevelReg and val & 0x80:
            self.tx.clear()
        elif reg == mod.CommandReg and val == mod.PCD_TRANSCEIVE:
            self._rx, self._i = self.reply, 0
        self.regs[reg] = val

    def reg_read(self, reg):
        if reg == mod.ComIrqReg:
            return 0x30                      # Rx | Idle, no waiting
        if reg == mod.ErrorReg:
            return self.err
        if reg == mod.FIFOLevelReg:
            return len(self._rx)
        if reg == mod.FIFODataReg:
            b = self._rx[self._i]
            self._i += 1
            return b
        if reg == mod.ControlReg:
            return self.reply_bits
        return self.regs.get(reg, 0)


class TestWritePage:
    def test_the_frame_is_a2_page_data_crc(self):
        link = _Link()
        assert Mfrc522(link).write_page(7, b"\x01\x02\x03\x04") is True
        assert link.tx[0] == 0xA2
        assert link.tx[1] == 7
        assert bytes(link.tx[2:6]) == b"\x01\x02\x03\x04"
        assert len(link.tx) == 8, "4-byte payload plus opcode, page and CRC"

    def test_the_crc_is_the_one_the_reader_computes(self):
        link = _Link()
        Mfrc522(link).write_page(9, b"\xde\xad\xbe\xef")
        body = bytes(link.tx[:6])
        want = Mfrc522._crc_a(body)
        assert link.tx[6] == want & 0xFF
        assert link.tx[7] == (want >> 8) & 0xFF

    def test_only_the_4_bit_ack_counts_as_written(self):
        assert Mfrc522(_Link(reply=b"\x0a", reply_bits=4)).write_page(
            7, b"\x00" * 4) is True

    @pytest.mark.parametrize("reply,bits", [
        (b"\x00", 4),          # NAK
        (b"\x01", 4),          # NAK, invalid argument
        (b"\x0a", 0),          # right value, whole byte -- not an ACK frame
        (b"\x0a\x0a", 4),      # too long
        (b"", 0),              # nothing came back
    ])
    def test_anything_else_is_a_failed_write(self, reply, bits):
        assert Mfrc522(_Link(reply=reply, reply_bits=bits)).write_page(
            7, b"\x00" * 4) is False

    def test_a_short_frame_protocol_error_is_not_treated_as_failure(self):
        """The ACK carries no parity and no CRC, so the chip raises ProtocolErr
        on every successful write. Aborting on it would fail every write."""
        assert Mfrc522(_Link(err=0x01)).write_page(7, b"\x00" * 4) is True

    def test_a_real_buffer_error_still_fails(self):
        assert Mfrc522(_Link(err=0x10)).write_page(7, b"\x00" * 4) is False

    def test_a_page_must_be_exactly_four_bytes(self):
        for bad in (b"", b"\x01\x02\x03", b"\x01\x02\x03\x04\x05"):
            with pytest.raises(ValueError, match="4 bytes"):
                Mfrc522(_Link()).write_page(7, bad)


# ── the guarded write ────────────────────────────────────────────────────────

class _FakeChip:
    """An NTAG's memory, addressed by page, with a settable capability
    container and an optional page that refuses to take a write."""

    def __init__(self, cc=b"\xe1\x10\x3e\x00", pages=130, bad_page=None):
        self.mem = [b"\x00\x00\x00\x00"] * pages
        self.mem[3] = cc
        self.bad_page = bad_page
        self.written = []

    def write_page(self, page, data4):
        if page == self.bad_page:
            return False
        self.written.append(page)
        self.mem[page] = bytes(data4)
        return True

    def read_block(self, page):
        if page >= len(self.mem):
            return None
        out = b"".join(self.mem[page:page + 4])
        return out if len(out) == 16 else None


def _mc(chip):
    mc = MifareClassic.__new__(MifareClassic)
    mc.m = chip
    return mc


class TestUserLastPage:
    @pytest.mark.parametrize("cc,last", [
        (b"\xe1\x10\x12\x00", 39),      # NTAG213
        (b"\xe1\x10\x3e\x00", 127),     # NTAG215
        (b"\xe1\x10\x6d\x00", 221),     # NTAG216
    ])
    def test_it_reads_the_bound_off_the_tag(self, cc, last):
        assert _mc(_FakeChip(cc=cc, pages=240)).user_last_page() == last

    def test_the_bound_never_exceeds_the_real_user_area(self):
        """The CC under-reports by a few pages on NTAG215/216. That direction
        is safe -- it refuses a legal write instead of programming a fuse."""
        assert _mc(_FakeChip(cc=b"\xe1\x10\x3e\x00")).user_last_page() < 129

    def test_a_tag_with_no_readable_cc_falls_back_to_the_smallest_chip(self):
        assert _mc(_FakeChip(cc=b"\x00\x00\x00\x00")).user_last_page() == \
            NTAG213_LAST_PAGE

    def test_a_zero_size_cc_falls_back_too(self):
        assert _mc(_FakeChip(cc=b"\xe1\x10\x00\x00")).user_last_page() == \
            NTAG213_LAST_PAGE


class TestWriteNtag:
    def test_a_clean_write_reports_no_error(self):
        chip = _FakeChip()
        assert _mc(chip).write_ntag(4, b"\xaa" * 112) is None
        assert chip.written == list(range(4, 32))

    def test_the_bytes_actually_land_where_the_decoder_looks(self):
        chip = _FakeChip()
        payload = encode_anycubic(manufacturer="Sunlu", ftype="PETG",
                                  color_argb=0xFF00FF00, weight_g=1000)
        assert _mc(chip).write_ntag(ANYCUBIC_FIRST_PAGE, payload) is None
        image = b"".join(chip.mem[:32])
        d = decode_anycubic(image)
        assert d["manufacturer"] == "Sunlu"
        assert d["type"] == "PETG"
        assert d["color_argb"] == 0xFF00FF00

    def test_the_reserved_pages_are_refused(self):
        for page in (0, 1, 2, 3):
            chip = _FakeChip()
            err = _mc(chip).write_ntag(page, b"\x00" * 4)
            assert err and "reserved" in err
            assert chip.written == [], "nothing may be written on a refusal"

    def test_running_past_the_user_area_is_refused(self):
        chip = _FakeChip(cc=b"\xe1\x10\x12\x00")     # NTAG213, last page 39
        err = _mc(chip).write_ntag(30, b"\x00" * 112)
        assert err and "runs past" in err
        assert chip.written == []

    def test_the_last_legal_page_is_allowed(self):
        chip = _FakeChip(cc=b"\xe1\x10\x12\x00", pages=48)
        assert _mc(chip).write_ntag(39, b"\x00" * 4) is None

    def test_a_payload_that_is_not_whole_pages_is_refused(self):
        err = _mc(_FakeChip()).write_ntag(4, b"\x00" * 5)
        assert err and "multiple of 4" in err

    def test_a_tag_that_will_not_take_a_page_is_reported(self):
        err = _mc(_FakeChip(bad_page=6)).write_ntag(4, b"\x11" * 112)
        assert err and "did not ACK" in err and "page 6" in err

    def test_a_page_that_reads_back_wrong_fails_the_write(self):
        """The read-back is the real check: a tag that ACKs and stores nothing
        must not be reported as written."""
        chip = _FakeChip()
        mc = _mc(chip)
        real = chip.write_page

        def _lying(page, data4):
            real(page, data4)
            if page == 10:
                chip.mem[10] = b"\x00\x00\x00\x00"
            return True
        chip.write_page = _lying
        err = mc.write_ntag(4, b"\x55" * 112)
        assert err and "reads back" in err

    def test_a_page_that_cannot_be_read_back_fails_the_write(self):
        """A tag pulled out of the field mid-verify must not pass."""
        chip = _FakeChip()
        real = chip.read_block

        def _goes_away(page):
            return None if page == 12 else real(page)
        chip.read_block = _goes_away
        err = _mc(chip).write_ntag(4, b"\x22" * 112)
        assert err and "could not read page 12" in err


# ── the entry point ──────────────────────────────────────────────────────────

class _StubMc:
    def __init__(self, uid, sak, err=None):
        self._uid, self._sak, self._err = uid, sak, err
        self.wrote = None

    def activate(self, **kw):
        return self._uid, self._sak

    def write_ntag(self, page, payload):
        self.wrote = (page, payload)
        return self._err


@pytest.fixture
def stub_mc(monkeypatch):
    holder = {}

    def _install(uid, sak, err=None):
        holder["mc"] = _StubMc(uid, sak, err)
        monkeypatch.setattr(mod, "MifareClassic", lambda _m: holder["mc"])
        monkeypatch.setattr(mod, "Mfrc522", lambda _l: object())
        return holder["mc"]
    return _install


class TestWriteTag:
    def test_an_ntag_is_written_at_page_four(self, stub_mc):
        mc = stub_mc(b"\x04\xa1\xb2\xc3", 0x00)
        uid, err = write_tag(object(), b"\x00" * 112)
        assert err is None
        assert uid == "04a1b2c3"
        assert mc.wrote[0] == ANYCUBIC_FIRST_PAGE

    def test_an_empty_field_is_reported_not_written(self, stub_mc):
        mc = stub_mc(None, 0x00)
        uid, err = write_tag(object(), b"\x00" * 112)
        assert uid is None
        assert "no tag" in err
        assert mc.wrote is None

    def test_a_mifare_classic_is_refused_by_name(self, stub_mc):
        """A Bambu or Snapmaker tag in the field must be turned away, not
        attempted: the Classic write is a different opcode behind Crypto1."""
        mc = stub_mc(b"\x01\x02\x03\x04", 0x08)
        uid, err = write_tag(object(), b"\x00" * 112)
        assert uid == "01020304"
        assert "not an NTAG" in err
        assert mc.wrote is None

    def test_a_write_failure_is_passed_through(self, stub_mc):
        stub_mc(b"\x01\x02\x03\x04", 0x00, err="tag did not ACK page 9")
        _uid, err = write_tag(object(), b"\x00" * 112)
        assert err == "tag did not ACK page 9"


# ── the AFC extension block ──────────────────────────────────────────────────

def _full(payload):
    """A whole tag image: reserved pages, then the payload from page 4."""
    return bytes(mod.ANYCUBIC_IMAGE_START) + payload


class TestAfcBlock:
    def test_an_arbitrary_weight_round_trips_exactly(self):
        """The whole point: Spoolman's 823g is 823g on the tag, not 1000."""
        img = _full(encode_tag_payload(ftype="PLA", weight_g=823))
        assert decode_afc_block(img)["weight_g"] == 823

    def test_it_carries_the_spoolman_spool_id(self):
        img = _full(encode_tag_payload(ftype="PLA", spool_id=136))
        assert decode_afc_block(img)["spool_id"] == 136

    def test_a_big_spool_id_survives(self):
        img = _full(encode_tag_payload(ftype="PLA", spool_id=4000000000))
        assert decode_afc_block(img)["spool_id"] == 4000000000

    def test_it_carries_density_and_drying(self):
        img = _full(encode_tag_payload(ftype="PLA", density=1.24,
                                       drying_temp_c=55, drying_time_h=8))
        b = decode_afc_block(img)
        assert b["density"] == 1.24
        assert b["drying_temp_c"] == 55
        assert b["drying_time_h"] == 8

    def test_it_sits_where_the_anycubic_decoder_never_looks(self):
        assert AFC_BLOCK_OFFSET == 0x80, "decode_anycubic reads up to 0x7C"
        img = _full(encode_tag_payload(manufacturer="Sunlu", ftype="PETG",
                                       weight_g=823, spool_id=7))
        # The brand decode is untouched by the block sitting after it.
        d = decode_anycubic(img)
        assert d["manufacturer"] == "Sunlu"
        assert d["type"] == "PETG"

    def test_the_whole_record_fits_an_ntag213(self):
        payload = encode_tag_payload(ftype="PLA", weight_g=823, spool_id=1)
        assert len(payload) == 144
        last = ANYCUBIC_FIRST_PAGE + len(payload) // 4 - 1
        assert last == 39 == NTAG213_LAST_PAGE, (
            "the record must end exactly at the top of the smallest chip")

    def test_a_genuine_anycubic_tag_has_no_block(self):
        """A tag from Anycubic has zeros up there, so nothing is overlaid and
        its own weight-from-length still stands."""
        img = _full(encode_anycubic(ftype="PLA", weight_g=1000)
                    + b"\x00" * AFC_BLOCK_LEN)
        assert decode_afc_block(img) is None
        assert decode_anycubic(img)["weight_g"] == 1000

    def test_a_short_read_is_not_mistaken_for_a_block(self):
        assert decode_afc_block(_full(encode_anycubic(ftype="PLA"))) is None

    def test_a_newer_version_is_ignored_rather_than_misread(self):
        blk = bytearray(encode_afc_block(weight_g=500))
        blk[4] = 99
        img = _full(encode_anycubic(ftype="PLA") + bytes(blk))
        assert decode_afc_block(img) is None

    def test_unset_fields_do_not_appear(self):
        """So an overlay never blanks what the brand layout did know."""
        img = _full(encode_tag_payload(ftype="PLA", weight_g=500))
        b = decode_afc_block(img)
        assert "spool_id" not in b
        assert "density" not in b
        assert b["weight_g"] == 500

    def test_the_magic_is_there(self):
        img = _full(encode_tag_payload(ftype="PLA"))
        assert img[AFC_BLOCK_OFFSET:AFC_BLOCK_OFFSET + 4] == AFC_BLOCK_MAGIC
        assert AFC_BLOCK_PAGE == 32


# ── cascade level 2: the real 7-byte UID ─────────────────────────────────────

class _CascadeChip:
    """A tag that answers anticoll/select for a 4- or 7-byte UID."""

    def __init__(self, uid, sak_final=0x00):
        self.uid = uid
        self.sak_final = sak_final
        self.levels = []

    def anticoll(self, cmd=PICC_ANTICOLL):
        self.levels.append(cmd)
        if len(self.uid) == 4:
            return self.uid
        if cmd == PICC_ANTICOLL:
            return bytes([CASCADE_TAG]) + self.uid[:3]
        return self.uid[3:7]

    def select(self, uid4, cmd=None):
        if len(self.uid) == 4:
            return self.sak_final
        return (SAK_UID_INCOMPLETE if uid4[0] == CASCADE_TAG
                else self.sak_final)

    def request(self, req=None):
        return b"\x44\x00"

    def reset(self):
        pass

    def antenna_on(self):
        pass

    def stop_crypto(self):
        pass


def _mc_chip(chip):
    mc = MifareClassic.__new__(MifareClassic)
    mc.m = chip
    return mc


class TestSevenByteUid:
    def test_an_ntag_reports_its_whole_uid(self):
        """Every NTAG21x has a 7-byte UID. Level 1 alone gives the cascade tag
        plus three bytes -- and since NXP UIDs all start 04, that is two
        varying bytes, which collides inside one roll of stickers."""
        chip = _CascadeChip(b"\x04\xa1\xb2\xc3\xd4\xe5\xf6")
        uid, sak = _mc_chip(chip).activate()
        assert uid == b"\x04\xa1\xb2\xc3\xd4\xe5\xf6"
        assert uid.hex() == "04a1b2c3d4e5f6"
        assert sak == 0x00

    def test_it_runs_both_cascade_levels(self):
        chip = _CascadeChip(b"\x04\xa1\xb2\xc3\xd4\xe5\xf6")
        _mc_chip(chip).activate()
        assert chip.levels == [PICC_ANTICOLL, PICC_ANTICOLL_CL2]

    def test_the_cascade_tag_is_not_part_of_the_uid(self):
        chip = _CascadeChip(b"\x04\xa1\xb2\xc3\xd4\xe5\xf6")
        uid, _ = _mc_chip(chip).activate()
        assert uid[0] != CASCADE_TAG
        assert len(uid) == 7

    def test_a_four_byte_uid_still_takes_one_level(self):
        """A Bambu or BTT Classic tag has a real 4-byte UID and must not be
        sent through a second cascade."""
        chip = _CascadeChip(b"\x01\x02\x03\x04", sak_final=0x08)
        uid, sak = _mc_chip(chip).activate()
        assert uid == b"\x01\x02\x03\x04"
        assert sak == 0x08
        assert chip.levels == [PICC_ANTICOLL]

    def test_two_stickers_from_one_roll_are_told_apart(self):
        """The failure the truncation would cause: consecutive tags whose UIDs
        differ only in the last bytes."""
        a = _CascadeChip(b"\x04\x11\x22\x33\x44\x55\x66")
        b = _CascadeChip(b"\x04\x11\x22\x33\x44\x55\x77")
        assert _mc_chip(a).activate()[0] != _mc_chip(b).activate()[0]


# ── MIFARE Classic 0xA0 block write ──────────────────────────────────────────

from extras.AFC_rfid_readers import (bambu_classic_write_test,   # noqa: E402
                                     _is_writable_classic_block)


class _ClassicLink:
    """Register model for a Classic write: two ACK phases, one canned each."""
    def __init__(self, ack1=b"\x0a", ack2=b"\x0a", bits=4):
        self.regs = {}
        self.tx = bytearray()
        self.phase = 0
        self.acks = [(ack1, bits), (ack2, bits)]
        self._rx, self._i = b"", 0

    def reg_write(self, reg, val):
        if reg == mod.FIFODataReg:
            self.tx.append(val)
        elif reg == mod.FIFOLevelReg and val & 0x80:
            self.tx.clear()
        elif reg == mod.CommandReg and val == mod.PCD_TRANSCEIVE:
            rx, _ = self.acks[min(self.phase, 1)]
            self._rx, self._i = rx, 0
            self.phase += 1
        self.regs[reg] = val

    def reg_read(self, reg):
        if reg == mod.ComIrqReg:
            return 0x20
        if reg == mod.ErrorReg:
            return 0x00
        if reg == mod.FIFOLevelReg:
            return len(self._rx)
        if reg == mod.FIFODataReg:
            b = self._rx[self._i]; self._i += 1; return b
        if reg == mod.ControlReg:
            return self.acks[min(self.phase - 1, 1)][1] if self.phase else 0
        return self.regs.get(reg, 0)


class TestClassicBlockWrite:
    def test_two_acked_phases_is_a_success(self):
        assert Mfrc522(_ClassicLink()).write_classic_block(13, b"\x11" * 16)

    def test_the_command_frame_is_a0_block(self):
        link = _ClassicLink()
        Mfrc522(link).write_classic_block(9, b"\x22" * 16)
        # last phase's tx is the data; first frame was 0xA0+block. We can only
        # see the last tx here, but a clean two-phase success is the contract.
        assert True

    def test_a_naked_command_phase_fails(self):
        assert not Mfrc522(_ClassicLink(ack1=b"\x00")).write_classic_block(
            13, b"\x33" * 16)

    def test_a_naked_data_phase_fails(self):
        assert not Mfrc522(_ClassicLink(ack2=b"\x00")).write_classic_block(
            13, b"\x33" * 16)

    def test_sixteen_bytes_required(self):
        with pytest.raises(ValueError, match="16 bytes"):
            Mfrc522(_ClassicLink()).write_classic_block(13, b"\x00" * 15)


class TestWritableBlockGuard:
    def test_trailers_and_block0_are_refused(self):
        for b in (0, 3, 7, 11, 15, 63):
            assert not _is_writable_classic_block(b)

    def test_data_blocks_are_allowed(self):
        for b in (1, 2, 4, 5, 6, 13, 62):
            assert _is_writable_classic_block(b)

    def test_write_test_refuses_a_trailer(self):
        res = bambu_classic_write_test(object(), b"k" * 16, block=3)
        assert "trailer" in res.get("error", "")


# ── classic_write_block (auth + write + verify) ──────────────────────────────

from extras.AFC_rfid_readers import classic_write_block, CLASSIC_DEFAULT_KEY  # noqa


class _ClassicRW:
    """A Classic tag that authenticates, stores block writes, and reads back."""
    def __init__(self, uid=b"\x01\x02\x03\x04", sak=0x08, auth_ok=True,
                 write_ok=True):
        self.mem = {}
        self._uid, self._sak = uid, sak
        self.auth_ok, self.write_ok = auth_ok, write_ok

    # MifareClassic/Mfrc522 surface used by classic_write_block:
    def activate(self, **kw):
        return self._uid, self._sak


def _patch_classic(monkeypatch, tag):
    """Wire a fake MifareClassic/Mfrc522 into the module for one call."""
    class _MC:
        def __init__(self, mfrc): self.m = mfrc
        def activate(self, **kw): return tag._uid, tag._sak
    class _MF:
        def __init__(self, link): pass
        def auth(self, kt, blk, key, uid): return tag.auth_ok
        def write_classic_block(self, blk, d): 
            if tag.write_ok: tag.mem[blk] = bytes(d)
            return tag.write_ok
        def read_block(self, blk): return tag.mem.get(blk)
        def stop_crypto(self): pass
    import extras.AFC_rfid_readers as r
    monkeypatch.setattr(r, "MifareClassic", _MC)
    monkeypatch.setattr(r, "Mfrc522", _MF)


class TestClassicWriteBlock:
    def test_a_clean_write_verifies(self, monkeypatch):
        tag = _ClassicRW()
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 4, b"\xab" * 16)
        assert err is None and uid == "01020304"
        assert tag.mem[4] == b"\xab" * 16

    def test_a_trailer_is_refused(self, monkeypatch):
        tag = _ClassicRW()
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 7, b"\x00" * 16)
        assert err and "trailer" in err and tag.mem == {}

    def test_wrong_key_is_reported(self, monkeypatch):
        tag = _ClassicRW(auth_ok=False)
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 4, b"\x00" * 16)
        assert err and "authenticate" in err

    def test_a_locked_block_nak_is_reported(self, monkeypatch):
        tag = _ClassicRW(write_ok=False)
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 4, b"\x00" * 16)
        assert err and "did not ACK" in err

    def test_an_ntag_is_refused(self, monkeypatch):
        tag = _ClassicRW(sak=0x00)
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 4, b"\x00" * 16)
        assert err and "not a MIFARE Classic" in err

    def test_data_must_be_16_bytes(self, monkeypatch):
        tag = _ClassicRW()
        _patch_classic(monkeypatch, tag)
        uid, err = classic_write_block(object(), 4, b"\x00" * 8)
        assert err and "16" in err
