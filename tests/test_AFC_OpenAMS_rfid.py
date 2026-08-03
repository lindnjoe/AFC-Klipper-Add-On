# Tests for extras/AFC_OpenAMS_rfid.py — the OpenAMS RFID coordinator and its
# two transport adapters.
#
# Three things are worth pinning:
#
#   * the MFRC522 SPI framing. A read is [0x80|(reg<<1), 0x00] with the value in
#     the SECOND returned byte, and a write is [(reg<<1), val]. Both mask with
#     0x7E, so bit 0 and bit 7 of the shifted address can never leak into the
#     wire format. Get any of that wrong and every register read returns
#     plausible rubbish rather than failing outright.
#   * config parsing that rejects bad input at startup rather than at scan
#     time. `slots` and `lane_slot_map` are the two places an operator types
#     numbers by hand.
#   * reader indexing by physical slot, including the case where two readers
#     claim the same slot, which is a wiring mistake worth naming.
from __future__ import annotations

import types

import pytest

import extras.AFC_OpenAMS_rfid as mod


class _Spi:
    """Records what was sent and replays a canned response."""

    def __init__(self, response=(0x00, 0x00)):
        self.sent = []
        self.transfers = []
        self._response = list(response)

    def spi_send(self, data):
        self.sent.append(list(data))

    def spi_transfer(self, data):
        self.transfers.append(list(data))
        return {'response': list(self._response)}


class TestSpiRegisterFraming:
    """MFRC522 register access over SPI, per the datasheet framing."""

    def test_read_uses_the_read_bit_and_shifted_address(self):
        spi = _Spi(response=(0x00, 0x37))
        link = mod._OamsSpiRegLink(spi)
        assert link.reg_read(0x37) == 0x37
        # address byte = 0x80 | (reg << 1), masked to 0x7E
        assert spi.transfers == [[0x80 | ((0x37 << 1) & 0x7E), 0x00]]

    def test_read_takes_the_second_byte_not_the_first(self):
        # The first byte is whatever the MFRC522 shifted out during the address
        # phase; only the second is the register value.
        spi = _Spi(response=(0xFF, 0x2A))
        assert mod._OamsSpiRegLink(spi).reg_read(0x01) == 0x2A

    def test_read_of_a_short_response_is_zero_not_an_exception(self):
        spi = _Spi(response=(0x99,))
        assert mod._OamsSpiRegLink(spi).reg_read(0x01) == 0

    def test_write_clears_the_read_bit(self):
        spi = _Spi()
        mod._OamsSpiRegLink(spi).reg_write(0x37, 0xAB)
        assert spi.sent == [[(0x37 << 1) & 0x7E, 0xAB]]
        assert spi.sent[0][0] & 0x80 == 0

    def test_address_masking_cannot_leak_bit0_or_bit7(self):
        spi = _Spi()
        link = mod._OamsSpiRegLink(spi)
        for reg in range(0x00, 0x40):
            spi.sent.clear()
            link.reg_write(reg, 0)
            addr = spi.sent[0][0]
            assert addr & 0x81 == 0, f"reg {reg:#x} produced addr {addr:#x}"

    def test_write_masks_the_value_to_a_byte(self):
        spi = _Spi()
        mod._OamsSpiRegLink(spi).reg_write(0x01, 0x1FF)
        assert spi.sent[0][1] == 0xFF

    def test_reader_power_is_a_noop(self):
        # The OpenAMS readers have no coil-enable line; the RF field is driven
        # through TxControlReg by the Mfrc522 class.
        assert mod._OamsSpiRegLink(_Spi()).reader_power(True) is None
        assert mod._OamsSpiRegLink(_Spi()).reader_power(False) is None


class TestHookedI2cMagic:
    """The i2c transport tunnels register access through a magic prefix that a
    patched OAMS firmware recognises. The constants are a wire contract."""

    def test_magic_prefix_is_RF_and_ops_are_R_W(self):
        assert (mod._HOOK_MAGIC0, mod._HOOK_MAGIC1) == (0x52, 0x46)   # 'R','F'
        assert mod._HOOK_OP_READ == 0x52                              # 'R'
        assert mod._HOOK_OP_WRITE == 0x57                             # 'W'


class _Cfg:
    """Minimal ConfigWrapper stand-in."""

    class error(Exception):
        pass

    def __init__(self, name="AFC_OpenAMS_rfid rdA", **opts):
        self._name = name
        self._o = opts

    def get_printer(self):
        return types.SimpleNamespace(
            lookup_object=lambda n, d=None: d,
            lookup_objects=lambda p=None: [],
            register_event_handler=lambda *a: None,
            load_object=lambda *a, **k: None)

    def get_name(self):
        return self._name

    def get(self, key, default=None):
        return self._o.get(key, default)

    def getint(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return int(v) if v is not None else None

    def getboolean(self, key, default=None, **kw):
        return bool(self._o.get(key, default))


def _parse_slots(spec):
    """Exercise the reader's `slots` parsing in isolation."""
    cfg = _Cfg(slots=spec)
    slots = []
    for s in (cfg.get("slots", "") or "").split(","):
        s = s.strip()
        if not s:
            continue
        try:
            slots.append(int(s))
        except ValueError:
            raise cfg.error("bad slot %r" % s)
    return slots


class TestSlotsParsing:
    def test_plain_list(self):
        assert _parse_slots("0, 1") == [0, 1]

    def test_whitespace_and_trailing_comma_tolerated(self):
        assert _parse_slots("  2 ,3 ,") == [2, 3]

    def test_empty_is_no_slots(self):
        assert _parse_slots("") == []
        assert _parse_slots(None) == []

    def test_garbage_raises_at_startup(self):
        with pytest.raises(_Cfg.error):
            _parse_slots("0, x")


def _parse_map(spec):
    """Exercise the coordinator's `lane_slot_map` parsing in isolation."""
    cfg = _Cfg()
    out = {}
    for pair in (spec or "").split(","):
        pair = pair.strip()
        if not pair:
            continue
        try:
            lane, slot = pair.split(":")
            out[lane.strip()] = int(slot)
        except ValueError:
            raise cfg.error("bad entry %r" % pair)
    return out


class TestLaneSlotMapParsing:
    def test_maps_lanes_to_physical_slots(self):
        assert _parse_map("lane4:0, lane5:1, lane6:2, lane7:3") == {
            "lane4": 0, "lane5": 1, "lane6": 2, "lane7": 3}

    def test_empty_map_is_allowed(self):
        assert _parse_map("") == {}

    def test_missing_colon_raises(self):
        with pytest.raises(_Cfg.error):
            _parse_map("lane4")

    def test_non_numeric_slot_raises(self):
        with pytest.raises(_Cfg.error):
            _parse_map("lane4:left")


class TestReaderIndexingBySlot:
    """_on_connect indexes readers by the slots they serve, so a lane's slot
    maps straight to its reader."""

    def _coord(self, readers):
        c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
        c._slot_reader = {}
        c._warn = []
        c.logger = types.SimpleNamespace(
            warning=lambda fmt, *a: c._warn.append(fmt % a if a else fmt),
            info=lambda *a, **k: None)
        c.printer = types.SimpleNamespace(
            lookup_object=lambda n, d=None: d,
            lookup_objects=lambda: [("AFC_OpenAMS_rfid " + r.name, r)
                                    for r in readers])
        return c

    def _reader(self, name, slots):
        r = mod.AFC_OpenAMS_rfid_reader.__new__(mod.AFC_OpenAMS_rfid_reader)
        r.name, r.slots = name, slots
        return r

    def test_each_slot_resolves_to_its_reader(self):
        a, b = self._reader("rdA", [0, 1]), self._reader("rdB", [2, 3])
        c = self._coord([a, b])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader == {0: a, 1: a, 2: b, 3: b}

    def test_two_readers_claiming_one_slot_is_reported(self):
        # A real wiring/config mistake: the second wins, but silently doing so
        # would make one reader look dead.
        a, b = self._reader("rdA", [0, 1]), self._reader("rdB", [1])
        c = self._coord([a, b])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader[1] is b
        assert any("more than one" in str(w) for w in c._warn)

    def test_no_readers_warns_rather_than_failing(self):
        c = self._coord([])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader == {}
        assert any("no [AFC_OpenAMS_rfid" in str(w) for w in c._warn)


class TestLaneToSlot:
    def test_mapped_and_unmapped_lanes(self):
        c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
        c._lane_slot = {"lane4": 0, "lane5": 1}
        assert mod.AFC_OpenAMS_rfid._get_slot(c, "lane4") == 0
        assert mod.AFC_OpenAMS_rfid._get_slot(c, "lane9") is None


class _Reader:
    def __init__(self, name="rdA", slots=(0,)):
        self.name, self.slots, self.link = name, list(slots), object()


def _coord(readers=(), lane_slot=None, afc=None):
    """A coordinator with its runtime maps populated, no Klipper needed."""
    c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
    c._slot_reader = {s: r for r in readers for s in r.slots}
    c._lane_slot = dict(lane_slot or {})
    c._last = {}
    c._no_reader_warned = set()
    c.afc = afc
    c.bambu_master_key = None
    c.creality_key = None
    c.creality_encryption_key = None
    c.logged = []
    c.logger = types.SimpleNamespace(
        warning=lambda fmt, *a: c.logged.append(("warn", fmt % a if a else fmt)),
        info=lambda fmt, *a, **k: c.logged.append(("info", fmt)),
        debug=lambda fmt, *a, **k: c.logged.append(("debug", fmt)))
    return c


class TestReadSlot:
    """On a correctly configured OpenAMS every slot has a reader -- the reader
    sections' `slots` between them cover the whole unit. So the no-reader path
    is a CONFIG MISTAKE guard, not a runtime state: a lane_slot_map entry
    pointing at a slot no reader covers, or a reader section left out.

    It still has to warn only once, because read_slot is polled many times a
    second across a feed; a misconfiguration would otherwise flood the log for
    the whole scan window instead of stating the fault once."""

    def test_a_slot_no_reader_covers_returns_none(self, monkeypatch):
        c = _coord()
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 3) is None

    def test_a_misconfigured_slot_warns_only_once(self):
        c = _coord()
        for _ in range(5):
            mod.AFC_OpenAMS_rfid.read_slot(c, 3)
        assert sum(1 for lvl, _ in c.logged if lvl == "warn") == 1

    def test_a_read_tag_is_returned_and_cached(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        tag = {"uid": "AABB", "filament": {"material": "PLA"}}
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: tag)
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 0) is tag
        assert c._last[0] is tag

    def test_no_tag_is_not_cached(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: None)
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 0) is None
        assert c._last == {}

    def test_decode_keys_are_passed_through(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        c.bambu_master_key = b"bk"
        c.creality_key = b"ck"
        c.creality_encryption_key = b"cek"
        got = {}
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: got.update(kw) or None)
        mod.AFC_OpenAMS_rfid.read_slot(c, 0)
        assert got["bambu_master_key"] == b"bk"
        assert got["creality_key"] == b"ck"
        assert got["creality_encryption_key"] == b"cek"


class TestScanSlotUids:
    """Enumerates every tag in the field. The OpenAMS readers are shared across
    bays, so a seated neighbour's tag can already be in range -- callers use
    this to learn which uids to exclude before a feed."""

    def test_no_reader_is_an_empty_list(self):
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(_coord(), 0) == []

    def test_returns_the_uids_it_saw(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])

        class _MC:
            def __init__(self, m): pass
            def activate(self, is_excluded=None, seen=None):
                seen.extend([("AA", 0x08, False), ("BB", 0x08, False)])
        monkeypatch.setattr(mod, "Mfrc522", lambda link: object())
        monkeypatch.setattr(mod, "MifareClassic", _MC)
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(c, 0) == ["AA", "BB"]

    def test_a_reader_error_is_logged_and_yields_no_uids(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])

        class _MC:
            def __init__(self, m): pass
            def activate(self, **kw): raise RuntimeError("spi down")
        monkeypatch.setattr(mod, "Mfrc522", lambda link: object())
        monkeypatch.setattr(mod, "MifareClassic", _MC)
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(c, 0) == []
        assert any(lvl == "debug" for lvl, _ in c.logged)


class TestReadLane:
    """Lane -> slot -> reader. A tag that is SEEN but not decoded still has to
    be recorded, or a missing decode key looks identical to an empty bay."""

    def test_unmapped_lane_warns_and_returns_none(self):
        c = _coord()
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane9") is None
        assert any("no slot" in m for lvl, m in c.logged if lvl == "warn")

    def test_no_tag_returns_none_without_recording(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))], {"lane4": 0})
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: None)
        rec = []
        c.record_tag_read = lambda *a, **k: rec.append((a, k))
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") is None
        assert rec == []

    def test_an_undecoded_tag_is_still_recorded(self, monkeypatch):
        # Seen but not decoded (e.g. no key for that vendor). Recording the UID
        # is what lets get_status say "a tag is there, I cannot read it".
        c = _coord([_Reader(slots=(0,))], {"lane4": 0})
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "AABB",
                                                "tag_type": "creality"})
        rec = []
        c.record_tag_read = lambda *a, **k: rec.append(k)
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") is None
        assert rec and rec[0]["uid"] == "AABB"
        assert rec[0]["decoded"] is False
        assert rec[0]["tag_type"] == "creality"

    def test_no_afc_lane_returns_the_mapped_slot_info(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))], {"lane4": 0}, afc=None)
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "A",
                                                "filament": {"material": "PLA"}})
        monkeypatch.setattr(mod, "map_tag_to_slot_info",
                            lambda tag: {"uid": "A", "material": "PLA"})
        c.record_tag_read = lambda *a, **k: None
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4")["material"] == "PLA"

    def test_a_known_lane_is_applied_to(self, monkeypatch):
        lane = object()
        afc = types.SimpleNamespace(lanes={"lane4": lane})
        c = _coord([_Reader(slots=(0,))], {"lane4": 0}, afc=afc)
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "A",
                                                "filament": {"material": "PLA"}})
        applied = []
        c.apply_to_lane = lambda l, t: applied.append(l) or {"ok": True}
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") == {"ok": True}
        assert applied == [lane]


class TestGcodeCommand:
    class _Gcmd:
        class error(Exception):
            pass

        def __init__(self, **kw):
            self._kw = kw
            self.said = []

        def get(self, k, d=None):
            return self._kw.get(k, d)

        def get_int(self, k, d=None):
            v = self._kw.get(k, d)
            return int(v) if v is not None else None

        def respond_info(self, m):
            self.said.append(m)

    def test_neither_lane_nor_slot_is_an_error(self):
        c = _coord()
        g = self._Gcmd()
        with pytest.raises(self._Gcmd.error):
            mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)

    def test_lane_with_no_tag_reports_the_hint(self):
        c = _coord()
        c.read_lane = lambda n: None
        c.undecoded_hint = lambda n: " (tag seen, no key)"
        g = self._Gcmd(LANE="lane4")
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "no tag decoded on lane4 (tag seen, no key)" in g.said[0]

    def test_lane_with_a_tag_reports_brand_and_material(self):
        c = _coord()
        c.read_lane = lambda n: {"brand": "Bambu", "material": "PLA"}
        g = self._Gcmd(LANE="lane4")
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "lane4 -> Bambu PLA" in g.said[0]

    def test_slot_form_reports_the_mapped_tag(self, monkeypatch):
        c = _coord()
        c.read_slot = lambda s: {"uid": "AA"}
        monkeypatch.setattr(mod, "map_tag_to_slot_info",
                            lambda tag: {"uid": "AA", "material": "PETG"})
        g = self._Gcmd(SLOT=2)
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "slot 2 ->" in g.said[0] and "PETG" in g.said[0]

    def test_slot_form_with_no_tag_says_none(self):
        c = _coord()
        c.read_slot = lambda s: None
        g = self._Gcmd(SLOT=2)
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "None" in g.said[0]
