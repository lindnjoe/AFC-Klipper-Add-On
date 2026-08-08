"""
Additional branch-coverage tests for extras/AFC_ACE2_rfid.py and the shared
reader stack in extras/AFC_rfid_readers.py.

Complements tests/test_AFC_ACE2_rfid.py and tests/test_ace2_rfid_reader.py by
covering the large previously-untested blocks:
  - Ace2Link frame reg_read/reg_write/_parse_field1
  - Mfrc522 _to_card / anticoll / _activate_once / read_ntag / read_all /
    read_blocks / antenna_on edge branches
  - brand decode edge cases (bad length, ValueError paths)
  - read_tag uid-None / dump_blocks / NTAG-no-decode
  - _Ace2RegLink reg_read/reg_write/_conn
  - _on_ready binding + _disable_factory_identify
  - _auto_read retry/error paths
  - _spool_details / _notify_scan
  - cmd_ACE_RFID_READ / cmd_ACE_RFID_BLOCKS / cmd_ACE_RFID_STAGE_TEST /
    cmd_ACE_RFID_SCAN gcode handlers
  - load_config entry point
"""
from __future__ import annotations

import importlib.util
import os
import struct
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _load(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Package import (not _load) so this is the SAME module object whose globals
# rfid.read_tag resolves — patching readers.MifareClassic must affect it.
import extras.AFC_rfid_readers as readers  # noqa: E402

rfid = _load("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")


# ── stubs ────────────────────────────────────────────────────────────────────
class _Logger:
    """Recording logger: stores (level, fully-formatted-message) tuples."""

    def __init__(self):
        self.messages = []

    def _rec(self, level, msg, args):
        self.messages.append((level, msg % args if args else msg))

    def info(self, msg, *args):
        self._rec("info", msg, args)

    def warning(self, msg, *args):
        self._rec("warning", msg, args)

    def exception(self, msg, *args):
        self._rec("exception", msg, args)

    def debug(self, msg, *args):
        self._rec("debug", msg, args)


class _CmdError(Exception):
    pass


class _Reactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        cb(0.0)


class _RecReactor:
    """Records callbacks without running them."""

    def __init__(self):
        self.callbacks = []

    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        self.callbacks.append((cb, waketime))


class _AdvReactor:
    def __init__(self, step=0.5):
        self._t = 0.0
        self._step = step

    def monotonic(self):
        return self._t

    def pause(self, when):
        self._t += self._step
        return when

    def register_callback(self, cb, when=None):
        pass


class _Gcode:
    def __init__(self):
        self.commands = {}
        self.raw = []
        self.info = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)

    def respond_raw(self, msg):
        self.raw.append(msg)


class _Printer:
    command_error = _CmdError

    def __init__(self):
        self.reactor = _Reactor()
        self.gcode = _Gcode()
        self.events = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return default

    def register_event_handler(self, name, cb):
        self.events.append((name, cb))


class _Config:
    def __init__(self, printer, opts):
        self._printer = printer
        self._opts = opts

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def getboolean(self, key, default=None):
        return bool(self._opts.get(key, default))

    def getint(self, key, default=None, minval=None):
        return int(self._opts.get(key, default))

    def getfloat(self, key, default=None, minval=None):
        return float(self._opts.get(key, default))


class _GCmd:
    error = _CmdError

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=0):
        return int(self.params.get(key, default))

    def get_float(self, key, default=0.0, minval=None, maxval=None):
        return float(self.params.get(key, default))

    def respond_info(self, msg):
        self.responses.append(msg)


class _Conn:
    connected = True

    def __init__(self):
        self.calls = []

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))


class _Ace2:
    def __init__(self):
        self._ace = _Conn()


def _mk(ace2_obj, opts=None):
    """AFC_ACE2_RFID via the real constructor, with a recording logger and the
    ACE2 object bound directly (klippy:ready is not fired in tests)."""
    obj = rfid.AFC_ACE2_RFID(_Config(_Printer(), dict(opts or {})))
    obj.logger = _Logger()
    obj.ace2 = ace2_obj
    return obj


# ── Ace2Link.reg_read / reg_write / _parse_field1 ────────────────────────────
class _CapTx:
    def __init__(self, resp=b""):
        self.frames = []
        self._resp = resp

    def __call__(self, frame):
        self.frames.append(frame)
        return self._resp


def _build_resp(cmd, payload):
    return rfid.Ace2Link(lambda f: f).build_frame(cmd, payload)


class TestAce2LinkRegRead:
    def test_returns_parsed_value_and_encodes_arg(self):
        val = 0x42
        resp = _build_resp(rfid.CMD_MFRC522_REG_READ, b"\x08" + rfid._varint(val))
        tx = _CapTx(resp)
        link = rfid.Ace2Link(tx, slot=1)
        assert link.reg_read(0x0A) == val
        sent = tx.frames[0]
        payload = sent[7:7 + sent[6]]
        arg, _ = rfid._varint_decode(payload, 1)
        assert arg == (1 << 16) | 0x0A


class TestAce2LinkRegWrite:
    def test_encodes_reg_and_val_into_arg(self):
        tx = _CapTx()
        link = rfid.Ace2Link(tx, slot=2)
        assert link.reg_write(0x0A, 0x99) is None
        sent = tx.frames[0]
        payload = sent[7:7 + sent[6]]
        arg, _ = rfid._varint_decode(payload, 1)
        assert arg == (2 << 16) | (0x0A << 8) | 0x99


class TestAce2LinkParseField1:
    def test_bad_preamble_raises(self):
        with pytest.raises(IOError):
            rfid.Ace2Link._parse_field1(b"\x00\x00\x00\x00\x00\x00\x00")

    def test_empty_raises(self):
        with pytest.raises(IOError):
            rfid.Ace2Link._parse_field1(b"")

    def test_field1_value_decoded(self):
        resp = _build_resp(rfid.CMD_MFRC522_REG_READ, b"\x08" + rfid._varint(7))
        assert rfid.Ace2Link._parse_field1(resp) == 7

    def test_non_field1_payload_returns_zero(self):
        resp = _build_resp(rfid.CMD_MFRC522_REG_READ, b"\x0a\x01")
        assert rfid.Ace2Link._parse_field1(resp) == 0

    def test_short_payload_returns_zero(self):
        resp = _build_resp(rfid.CMD_MFRC522_REG_READ, b"")
        assert rfid.Ace2Link._parse_field1(resp) == 0


# ── Mfrc522._to_card ─────────────────────────────────────────────────────────
class _ToCardLink:
    def __init__(self, comirq, errorreg=0, fifo=b"", ctrl=0):
        self.comirq = comirq
        self.errorreg = errorreg
        self.fifo_bytes = list(fifo)
        self.ctrl = ctrl
        self.writes = []

    def reg_read(self, r):
        if r == readers.ComIrqReg:
            return self.comirq
        if r == readers.ErrorReg:
            return self.errorreg
        if r == readers.FIFOLevelReg:
            return len(self.fifo_bytes)
        if r == readers.FIFODataReg:
            return self.fifo_bytes.pop(0) if self.fifo_bytes else 0
        if r == readers.ControlReg:
            return self.ctrl
        return 0

    def reg_write(self, r, v):
        self.writes.append((r, v))


class TestMfrc522ToCard:
    def test_timer_irq_returns_failure(self):
        link = _ToCardLink(comirq=0x01)          # Timer bit, no Rx/Idle
        m = rfid.Mfrc522(link)
        assert m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7) == (False, b"", 0)

    def test_error_reg_returns_failure(self):
        link = _ToCardLink(comirq=0x30, errorreg=0x02)   # Rx set, error bit
        m = rfid.Mfrc522(link)
        assert m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7) == (False, b"", 0)

    def test_poll_times_out_then_reads_fifo(self):
        # ComIrq never sets Rx/Idle/Timer -> the 2000-poll loop runs to
        # completion, then the error check passes and the FIFO is read.
        link = _ToCardLink(comirq=0x00, errorreg=0x00, fifo=b"\xab\xcd", ctrl=0x03)
        m = rfid.Mfrc522(link)
        ok, rx, last = m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7)
        assert ok is True and rx == b"\xab\xcd" and last == 3

    def test_authent_returns_no_rx(self):
        link = _ToCardLink(comirq=0x30, errorreg=0x00)
        m = rfid.Mfrc522(link)
        assert m._to_card(readers.PCD_MFAUTHENT, b"\x60\x00") == (True, b"", 0)


# ── Mfrc522.anticoll ─────────────────────────────────────────────────────────
def _mfrc_with_tocard(result):
    m = rfid.Mfrc522(object())
    m._to_card = lambda cmd, send, tx_last_bits=0: result
    return m


class TestMfrc522Anticoll:
    def test_transceive_failure_returns_none(self):
        m = _mfrc_with_tocard((False, b"", 0))
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() is None

    def test_bad_bcc_returns_none(self):
        m = _mfrc_with_tocard((True, bytes([1, 2, 3, 4, 5]), 0))   # BCC 4 != 5
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() is None

    def test_good_bcc_returns_uid(self):
        m = _mfrc_with_tocard((True, bytes([1, 2, 3, 4, 4]), 0))   # 1^2^3^4 == 4
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() == bytes([1, 2, 3, 4])


# ── Mfrc522.antenna_on ───────────────────────────────────────────────────────
class _AntennaLink:
    def __init__(self, txcontrol):
        self._txcontrol = txcontrol
        self.writes = []

    def reg_read(self, r):
        return self._txcontrol if r == readers.TxControlReg else 0

    def reg_write(self, r, v):
        self.writes.append((r, v))


class TestMfrc522AntennaOn:
    def test_turns_on_when_off(self):
        link = _AntennaLink(txcontrol=0x00)
        rfid.Mfrc522(link).antenna_on()
        assert (readers.TxControlReg, 0x03) in link.writes

    def test_noop_when_already_on(self):
        link = _AntennaLink(txcontrol=0x03)
        rfid.Mfrc522(link).antenna_on()
        assert link.writes == []


# ── MifareClassic._activate_once ─────────────────────────────────────────────
class _AoMfrc:
    def __init__(self, req_wupa=b"\x04\x00", req_reqa=b"\x04\x00",
                 uid=b"\x01\x02\x03\x04", sak=0x08):
        self._req_wupa = req_wupa
        self._req_reqa = req_reqa
        self._uid = uid
        self._sak = sak

    def request(self, req):
        return self._req_wupa if req == readers.PICC_WUPA else self._req_reqa

    def anticoll(self):
        return self._uid

    def select(self, uid):
        return self._sak


class TestMifareClassicActivateOnce:
    def test_wake_success(self):
        mc = rfid.MifareClassic(_AoMfrc())
        assert mc._activate_once(wake=True) == (b"\x01\x02\x03\x04", 0x08)

    def test_wake_both_none_returns_none(self):
        mc = rfid.MifareClassic(_AoMfrc(req_wupa=None, req_reqa=None))
        assert mc._activate_once(wake=True) == (None, None)

    def test_no_wake_reqa_none_returns_none(self):
        mc = rfid.MifareClassic(_AoMfrc(req_reqa=None))
        assert mc._activate_once(wake=False) == (None, None)

    def test_anticoll_none_returns_none(self):
        mc = rfid.MifareClassic(_AoMfrc(uid=None))
        assert mc._activate_once(wake=False) == (None, None)

    def test_select_none_returns_none(self):
        mc = rfid.MifareClassic(_AoMfrc(sak=None))
        assert mc._activate_once(wake=False) == (None, None)


# ── MifareClassic.read_ntag ──────────────────────────────────────────────────
class _BlockMfrc:
    def __init__(self, blocks, auth_ok=True):
        self._blocks = blocks
        self._auth_ok = auth_ok

    def read_block(self, block):
        return self._blocks.get(block)

    def auth(self, key_type, block, key6, uid4):
        return self._auth_ok


class TestMifareClassicReadNtag:
    def test_reads_until_nbytes(self):
        pages = {p: bytes([p]) * 16 for p in range(0, 32, 4)}
        mc = rfid.MifareClassic(_BlockMfrc(pages))
        data = mc.read_ntag(128)
        assert len(data) == 128 and data[:16] == bytes([0]) * 16

    def test_missing_page_returns_none(self):
        mc = rfid.MifareClassic(_BlockMfrc({}))
        assert mc.read_ntag(128) is None


# ── MifareClassic.read_all ───────────────────────────────────────────────────
class TestMifareClassicReadAll:
    def test_auth_failure_returns_none(self):
        mc = rfid.MifareClassic(_BlockMfrc({}, auth_ok=False))
        assert mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1) is None

    def test_read_block_failure_returns_none(self):
        mc = rfid.MifareClassic(_BlockMfrc({}, auth_ok=True))
        assert mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1) is None

    def test_success_returns_image(self):
        blocks = {b: bytes([b]) * 16 for b in range(4)}
        mc = rfid.MifareClassic(_BlockMfrc(blocks, auth_ok=True))
        out = mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1)
        assert len(out) == 64 and out[16:18] == bytes([1, 1])


# ── MifareClassic.read_blocks ────────────────────────────────────────────────
class TestMifareClassicReadBlocks:
    def test_auth_failure_returns_none(self):
        mc = rfid.MifareClassic(_BlockMfrc({}, auth_ok=False))
        assert mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (4,)) is None

    def test_read_block_failure_returns_none(self):
        mc = rfid.MifareClassic(_BlockMfrc({}, auth_ok=True))
        assert mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (4,)) is None

    def test_success_places_blocks(self):
        blocks = {5: bytes([0x55]) * 16}
        mc = rfid.MifareClassic(_BlockMfrc(blocks, auth_ok=True))
        out = mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (5,))
        assert out[80:82] == bytes([0x55, 0x55]) and len(out) == 1024


# ── brand decode edge cases ──────────────────────────────────────────────────
class TestDecodeBambu:
    def test_wrong_length_raises(self):
        with pytest.raises(ValueError):
            readers.decode_bambu(b"\x00" * 100)

    def test_optional_fields_present(self):
        d = bytearray(1024)
        struct.pack_into("<f", d, 140, 0.4)      # nozzle in (0, 2)
        struct.pack_into("<H", d, 164, 6000)     # spool width 60.0 mm
        struct.pack_into("<H", d, 228, 250)      # length 250 m
        out = readers.decode_bambu(bytes(d))
        assert out["nozzle_diameter"] == 0.4
        assert out["spool_width_mm"] == 60.0
        assert out["length_m"] == 250

    def test_optional_fields_absent(self):
        d = bytearray(1024)
        struct.pack_into("<f", d, 140, 5.0)      # nozzle out of (0, 2) -> None
        out = readers.decode_bambu(bytes(d))
        assert out["nozzle_diameter"] is None
        assert out["spool_width_mm"] is None
        assert out["length_m"] is None


class TestDecodeAnycubic:
    def test_short_returns_none(self):
        assert readers.decode_anycubic(b"\x00" * 0x40) is None

    def test_wrong_magic_returns_none(self):
        assert readers.decode_anycubic(b"\x11" * 0x80) is None


class TestDecodeSnapmaker:
    def test_short_returns_none(self):
        assert readers.decode_snapmaker(b"\x00" * 100) is None


class TestDecodeCreality:
    def test_short_returns_none(self):
        assert readers.decode_creality(b"\x00" * 33) is None

    def test_bad_hex_color_and_length_default(self):
        payload = (b"PRODU" + b"0276" + b"XX" + b"101001"
                   + b"ZZZZZZZ" + b"YYYY" + b"SERIAL" + b"\x00" * 14)
        assert len(payload) == 48
        out = readers.decode_creality(payload)
        assert out["type"] == "PLA"
        assert out["color_argb"] is None       # color hex unparsable
        assert out["length_m"] is None         # length hex unparsable
        assert out["weight_g"] is None         # length 0 -> no weight


class TestDecodeBtt:
    def test_short_returns_none(self):
        assert readers.decode_btt(b"\x00" * 300) is None


# ── read_tag branches ────────────────────────────────────────────────────────
class TestReadTag:
    def test_no_tag_returns_none(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return None, None

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        assert readers.read_tag(object()) is None

    def test_dump_blocks_attaches_raw_blocks(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return b"\x04\xa1\xb2\xc3", 0x08

            def read_blocks(self, uid, keys, blocks):
                img = bytearray(1024)
                for b in blocks:
                    img[b * 16:b * 16 + 16] = bytes([b & 0xFF]) * 16
                return bytes(img)

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        master = bytes.fromhex("00" * 16)
        out = readers.read_tag(object(), bambu_master_key=master,
                               dump_blocks=(5, 16))
        assert out["raw_blocks"][5] == (bytes([5]) * 16).hex()
        assert out["raw_blocks"][16] == (bytes([16]) * 16).hex()

    def test_ntag_without_decode(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return b"\x04\xa1\xb2\xc3", 0x00      # Ultralight/NTAG

            def read_ntag(self, n=128):
                return b"\x11" * n                    # not Anycubic/Elegoo

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        out = readers.read_tag(object())
        assert out["tag_type"] == "MifareUltralight"
        assert out["filament"] is None


# ── _Ace2RegLink reg_read / reg_write / _conn ────────────────────────────────
class _ValConn:
    connected = True

    def __init__(self, val=None):
        self.calls = []
        self._val = val

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return self._val

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))


class TestAce2RegLinkRegRead:
    def test_masks_value_and_encodes_arg(self):
        conn = _ValConn({"val": 0x1FF})
        ace2 = types.SimpleNamespace(_ace=conn)
        link = rfid._Ace2RegLink(ace2, 1)
        assert link.reg_read(0x0A) == 0xFF
        assert conn.calls[-1] == ("mfrc522_reg_read", {"arg": (1 << 16) | 0x0A})

    def test_none_response_reads_zero(self):
        conn = _ValConn(None)
        ace2 = types.SimpleNamespace(_ace=conn)
        assert rfid._Ace2RegLink(ace2, 0).reg_read(0x05) == 0


class TestAce2RegLinkRegWrite:
    def test_encodes_arg(self):
        conn = _ValConn({})
        ace2 = types.SimpleNamespace(_ace=conn)
        rfid._Ace2RegLink(ace2, 2).reg_write(0x0A, 0x99)
        assert conn.calls[-1] == (
            "mfrc522_reg_write", {"arg": (2 << 16) | (0x0A << 8) | 0x99})


class TestAce2RegLinkConn:
    def test_missing_serial_raises_ioerror(self):
        ace2 = types.SimpleNamespace(_ace=None)
        with pytest.raises(IOError):
            rfid._Ace2RegLink(ace2, 0).reg_read(0x01)


# ── _on_ready ────────────────────────────────────────────────────────────────
class _ReadyPrinter:
    command_error = _CmdError

    def __init__(self, objects, named=None):
        self._objects = objects
        self._named = named or []

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def lookup_objects(self):
        return list(self._objects.items()) + self._named

    def register_event_handler(self, name, cb):
        pass


class TestOnReady:
    def test_binds_ace2_and_marks_scanner(self, monkeypatch):
        obj = _mk(_Ace2(), {"scanner_lanes": "scan1"})
        monkeypatch.setattr(
            rfid, "resolve_rfid_keys",
            lambda pr, b, c, d: (b"\xaa", c, d))
        ace2obj = types.SimpleNamespace(name="Ace2_1", _ace=None)
        lane = types.SimpleNamespace(name="scan1")
        afc = types.SimpleNamespace(lanes={"scan1": lane})
        obj.printer = _ReadyPrinter({"AFC": afc, "AFC_ACE2": ace2obj})
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is ace2obj
        assert obj.bambu_master_key == b"\xaa"
        assert lane.spool_scanner is True
        assert obj.logger.messages == [("info", "ACE2 RFID bound to Ace2_1")]
        assert len(obj.reactor.callbacks) == 1     # retry-disable scheduled

    def test_warns_when_ace2_missing(self, monkeypatch):
        obj = _mk(_Ace2(), {})
        monkeypatch.setattr(
            rfid, "resolve_rfid_keys", lambda pr, b, c, d: (b, c, d))
        obj.printer = _ReadyPrinter({})
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is None
        assert obj.logger.messages == [
            ("warning", "ACE2 object not found; RFID disabled")]

    def test_discovers_named_object_no_callback_when_not_skipping(
            self, monkeypatch):
        obj = _mk(_Ace2(), {"skip_factory_autostage": False})
        monkeypatch.setattr(rfid, "resolve_rfid_keys", None)
        ace2obj = types.SimpleNamespace(name="Ace2_9", _ace=None)
        obj.printer = _ReadyPrinter({}, named=[("AFC_ACE2 Ace2_9", ace2obj)])
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is ace2obj
        assert obj.logger.messages == [("info", "ACE2 RFID bound to Ace2_9")]
        assert obj.reactor.callbacks == []          # skip off -> no retry


# ── _disable_factory_identify ────────────────────────────────────────────────
class _RaisingConn:
    connected = True

    def send_command_async(self, name, params=None):
        raise RuntimeError("not ready")


class TestDisableFactoryIdentify:
    def test_noop_without_ace2(self):
        obj = _mk(_Ace2(), {})
        obj.ace2 = None
        obj._disable_factory_identify()
        assert obj.logger.messages == []

    def test_logs_when_enable_call_fails(self):
        ace2 = types.SimpleNamespace(_ace=_RaisingConn())
        obj = _mk(ace2, {})
        obj._disable_factory_identify([0])
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: could not disable factory identify on "
                     "slot 0 yet (serial not ready?)")]

    def test_logs_success_per_slot(self):
        obj = _mk(_Ace2(), {})
        obj._disable_factory_identify([1])
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: factory identify disabled on slot 1")]


# ── _auto_read ───────────────────────────────────────────────────────────────
class TestAutoRead:
    def test_command_error_logged_and_returns(self):
        obj = _mk(_Ace2(), {"read_on_insert_attempts": 3})
        obj.read_lane = lambda name: (_ for _ in ()).throw(_CmdError("boom"))
        obj._auto_read("lane1")
        assert obj.logger.messages == [("info", "ACE2 RFID auto-read lane1: boom")]

    def test_retries_then_reports_no_tag(self):
        obj = _mk(_Ace2(), {"read_on_insert_attempts": 2,
                            "read_on_insert_delay": 0.1})
        obj.reactor = _Reactor()

        def _raise(name):
            raise RuntimeError("glitch")

        obj.read_lane = _raise
        obj._auto_read("lane1")
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID auto-read failed for lane1"),
            ("exception", "ACE2 RFID auto-read failed for lane1"),
            ("info", "ACE2 RFID auto-read lane1: no tag after 2 attempts")]

    def test_success_stops_without_logging(self):
        obj = _mk(_Ace2(), {"read_on_insert_attempts": 3})
        obj.read_lane = lambda name: {"uid": "aa"}
        obj._auto_read("lane1")
        assert obj.logger.messages == []


# ── _spool_details ───────────────────────────────────────────────────────────
def _slot_info():
    return {"brand": "Bambu", "material": "PLA", "color_hex": "#112233",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60,
            "weight_g": 1000, "uid": "aa"}


class TestSpoolDetails:
    def test_base_when_no_spool_id(self):
        obj = _mk(_Ace2(), {})
        obj.afc = types.SimpleNamespace(moonraker=object())
        d = obj._spool_details(None, _slot_info())
        assert d["brand"] == "Bambu" and d["color"] == "112233"
        assert d["name"] == "" and d["weight"] == 1000

    def test_base_when_moonraker_missing(self):
        obj = _mk(_Ace2(), {})
        obj.afc = types.SimpleNamespace(moonraker=None)
        d = obj._spool_details(5, _slot_info())
        assert d["material"] == "PLA" and d["name"] == ""

    def test_base_when_get_spool_raises(self):
        obj = _mk(_Ace2(), {})
        mr = types.SimpleNamespace(
            get_spool=lambda sid: (_ for _ in ()).throw(RuntimeError("x")))
        obj.afc = types.SimpleNamespace(moonraker=mr)
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == "" and d["brand"] == "Bambu"

    def test_base_when_spool_not_dict(self):
        obj = _mk(_Ace2(), {})
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: None))
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == ""

    def test_enriched_from_spoolman(self):
        obj = _mk(_Ace2(), {})
        spool = {"remaining_weight": 750,
                 "filament": {"name": "Galaxy", "material": "PETG",
                              "color_hex": "#ff0000", "diameter": 1.75,
                              "settings_extruder_temp": 240,
                              "settings_bed_temp": 70,
                              "vendor": {"name": "Polymaker"}}}
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: spool))
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == "Galaxy" and d["brand"] == "Polymaker"
        assert d["material"] == "PETG" and d["color"] == "ff0000"
        assert d["ext"] == 240 and d["bed"] == 70
        assert d["weight"] == 750                # remaining preferred

    def test_weight_falls_back_when_no_remaining(self):
        obj = _mk(_Ace2(), {})
        spool = {"remaining_weight": None,
                 "filament": {"weight": 500, "vendor": {}}}
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: spool))
        d = obj._spool_details(5, _slot_info())
        assert d["weight"] == 500


# ── _notify_scan ─────────────────────────────────────────────────────────────
class TestNotifyScan:
    def test_full_details_popup(self):
        obj = _mk(_Ace2(), {})
        obj.reactor = _Reactor()
        obj.afc = types.SimpleNamespace(moonraker=None)
        obj._notify_scan(_slot_info(), "lane1", 120)
        raw = obj.gcode.raw
        assert any("Brand: Bambu" in r for r in raw)
        assert any("Diameter: 1.75mm" in r for r in raw)
        assert any("Nozzle temp: 220" in r for r in raw)
        assert any("Bed temp: 60" in r for r in raw)
        assert any("Remaining: 1000g" in r for r in raw)
        assert any("Spoolman ID: 120" in r for r in raw)
        assert any("action:prompt_end" in r for r in raw)   # auto-dismiss ran

    def test_uid_fallback_when_no_fields(self):
        obj = _mk(_Ace2(), {})
        obj.reactor = _Reactor()
        obj.afc = types.SimpleNamespace(moonraker=None)
        obj._notify_scan({"uid": "deadbeef"}, "", None)
        assert any("uid: deadbeef" in r for r in obj.gcode.raw)

    def test_notification_error_logged(self):
        obj = _mk(_Ace2(), {})
        obj._spool_details = lambda sid, si: (_ for _ in ()).throw(
            RuntimeError("boom"))
        obj._notify_scan(_slot_info(), "lane1", 5)
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID scan: notification error: boom")]


# ── cmd_ACE_RFID_READ ────────────────────────────────────────────────────────
class TestCmdRfidRead:
    def test_success_response_with_color(self, monkeypatch):
        obj = _mk(_Ace2(), {})
        monkeypatch.setattr(
            rfid, "read_tag",
            lambda link, **kw: {"uid": "deadbeef", "tag_type": "MifareClassic1k",
                                "filament": {"manufacturer": "Bambu",
                                             "type": "PLA",
                                             "color_argb": 0xFF112233}})
        gcmd = _GCmd({"SLOT": 0})
        obj.cmd_ACE_RFID_READ(gcmd)
        assert gcmd.responses == [
            "ACE2 RFID: uid=deadbeef type=MifareClassic1k brand=Bambu "
            "material=PLA color=112233"]

    def test_success_response_without_color(self, monkeypatch):
        obj = _mk(_Ace2(), {})
        monkeypatch.setattr(
            rfid, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": {"manufacturer": "", "type": "",
                                             "color_argb": None}})
        gcmd = _GCmd({"SLOT": 0})
        obj.cmd_ACE_RFID_READ(gcmd)
        assert gcmd.responses == [
            "ACE2 RFID: uid=aa type=MifareClassic1k brand= material= color="]


# ── cmd_ACE_RFID_BLOCKS ──────────────────────────────────────────────────────
class TestCmdRfidBlocks:
    def test_bad_blocks_string(self):
        obj = _mk(_Ace2(), {})
        gcmd = _GCmd({"SLOT": 0, "BLOCKS": "x,y"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: bad BLOCKS='x,y'"]

    def test_lane_without_slot_raises(self):
        obj = _mk(_Ace2(), {})
        gcmd = _GCmd({"LANE": "nope", "BLOCKS": "5,16"})
        with pytest.raises(_CmdError):
            obj.cmd_ACE_RFID_BLOCKS(gcmd)

    def test_no_tag_found(self):
        obj = _mk(_Ace2(), {})
        obj.read_slot = lambda slot, dump_blocks=None: None
        gcmd = _GCmd({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: no tag found"]

    def test_read_error_swallowed(self):
        obj = _mk(_Ace2(), {})

        def _boom(slot, dump_blocks=None):
            raise RuntimeError("wedge")

        obj.read_slot = _boom
        gcmd = _GCmd({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: error: wedge"]
        assert obj.logger.messages == [("exception", "ACE2 RFID dump failed")]

    def test_dump_reports_dual_color(self):
        obj = _mk(_Ace2(), {})
        b5 = bytes([0x11, 0x22, 0x33, 0xFF]).hex()
        b16 = bytes([0x00, 0x00, 0x02, 0x00,      # fmt=0, count=2
                     0xFF, 0x33, 0x22, 0x11]).hex()   # A,B,G,R
        tag = {"uid": "aabb", "tag_type": "MifareClassic1k",
               "raw_blocks": {5: b5, 16: b16},
               "filament": {"colors_argb": [0xFF112233, 0xFF112233]}}
        obj.read_slot = lambda slot, dump_blocks=None: tag
        gcmd = _GCmd({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        out = gcmd.responses[0]
        assert "block5 primary -> #112233 (a=ff)" in out
        assert "color_count=2" in out
        assert "DUAL-COLOR (count=2)" in out
        assert "decoded colors: #112233, #112233" in out


# ── cmd_ACE_RFID_STAGE_TEST ──────────────────────────────────────────────────
class _StageConn:
    connected = True

    def __init__(self, feed_raises=False):
        self.calls = []
        self._feed_raises = feed_raises

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))

    def unwind_filament(self, index, length, speed, mode="normal"):
        self.calls.append(("unwind_filament", {"index": index,
                                                "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        if self._feed_raises:
            raise RuntimeError("feed rejected")
        self.calls.append(("feed_filament", {"index": index,
                                             "length": length, "speed": speed}))

    def stop_feed_filament(self, index):
        self.calls.append(("stop_feed_filament", {"index": index}))

    def get_status(self, timeout=2.0):
        return {"status": "ready"}


class _StageAce2:
    def __init__(self, conn, moving_polls=1):
        self._ace = conn
        self.feed_speed = 80.0
        self.retract_speed = 80.0
        self._slot_map = {"lane1": 0}
        self._moving_polls = moving_polls

    def _wait_for_ace_ready(self, timeout=30.0):
        return True

    def _slot_is_moving(self, status, slot):
        self._moving_polls -= 1
        return self._moving_polls >= 0

    def _slot_in_error(self, slot):
        return False


def _stage_obj(conn, moving_polls=1, opts=None):
    ace2 = _StageAce2(conn, moving_polls=moving_polls)
    base = {"lane_slot_map": "lane1:0", "skip_factory_autostage": False}
    base.update(opts or {})
    obj = _mk(ace2, base)
    obj.reactor = _AdvReactor(step=0.5)
    return obj, ace2


class TestCmdStageTest:
    def test_ace2_missing_raises(self):
        obj = _mk(_Ace2(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd({"LANE": "lane1"}))

    def test_unknown_lane_raises(self):
        conn = _StageConn()
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd({"LANE": "ghost"}))

    def test_serial_not_connected_raises(self):
        conn = _StageConn()
        conn.connected = False
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd({"LANE": "lane1"}))

    def test_detect_and_decode_restages(self, monkeypatch):
        conn = _StageConn()
        obj, _ = _stage_obj(conn)

        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self):
                return b"\xbe\xef", 0x08          # detected on first poll

        monkeypatch.setattr(rfid, "MifareClassic", _MC)
        monkeypatch.setattr(rfid, "Mfrc522", lambda link: None)
        obj._read_tag = lambda link, **kw: {"uid": "beef",
                                            "filament": {"type": "PLA"}}
        gcmd = _GCmd({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        names = [c[0] for c in conn.calls]
        assert names.count("unwind_filament") >= 1     # initial retract
        assert "stop_feed_filament" in names           # stopped on detect
        assert names[-1] == "feed_filament"            # re-staged remainder
        assert any("DETECTED" in r for r in gcmd.responses)
        assert any("DONE" in r for r in gcmd.responses)

    def test_no_detect_reports_and_restages(self, monkeypatch):
        conn = _StageConn()
        obj, _ = _stage_obj(conn, moving_polls=1)

        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self):
                return None, None                # never a tag

        monkeypatch.setattr(rfid, "MifareClassic", _MC)
        monkeypatch.setattr(rfid, "Mfrc522", lambda link: None)
        gcmd = _GCmd({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        assert any("no tag detected during the feed" in r
                   for r in gcmd.responses)
        assert any("DONE" in r for r in gcmd.responses)

    def test_feed_error_path(self, monkeypatch):
        conn = _StageConn(feed_raises=True)
        obj, _ = _stage_obj(conn)
        monkeypatch.setattr(rfid, "MifareClassic",
                            lambda mfrc: types.SimpleNamespace(
                                activate=lambda: (None, None)))
        monkeypatch.setattr(rfid, "Mfrc522", lambda link: None)
        gcmd = _GCmd({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        assert any("feed returned/err" in r for r in gcmd.responses)

    def test_initial_retract_failure_raises(self):
        conn = _StageConn()
        conn.unwind_filament = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("stuck"))
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd({"LANE": "lane1", "DIST": 500}))


# ── cmd_ACE_RFID_SCAN ────────────────────────────────────────────────────────
class TestCmdRfidScan:
    def test_error_swallowed(self):
        obj = _mk(_Ace2(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: (_ for _ in ()).throw(
            RuntimeError("glitch"))
        gcmd = _GCmd({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert obj.logger.messages == [("exception", "ACE2 RFID scan failed")]
        assert any("scan: error: glitch" in r for r in gcmd.responses)

    def test_no_tag(self):
        obj = _mk(_Ace2(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: None
        gcmd = _GCmd({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert any("no tag found on scan1" in r for r in gcmd.responses)

    def test_success_reports_staged_spool(self):
        obj = _mk(_Ace2(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: {"uid": "beef",
                                            "filament": {"type": "PLA"}}
        obj.afc = types.SimpleNamespace(
            spool=types.SimpleNamespace(next_spool_id=42))
        gcmd = _GCmd({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert any("staged next spool from scan1 — uid=beef type=PLA "
                   "(spool #42)" in r for r in gcmd.responses)


# ── motion stubs for sister-retract / teardown ───────────────────────────────
class _MotionConn:
    connected = True

    def __init__(self, unwind_raises=False):
        self.calls = []
        self._unwind_raises = unwind_raises

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))

    def unwind_filament(self, index, length, speed, mode="normal"):
        if self._unwind_raises:
            raise RuntimeError("unwind wedged")
        self.calls.append(("unwind_filament", {"index": index,
                                               "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        self.calls.append(("feed_filament", {"index": index,
                                             "length": length, "speed": speed}))


class _MotionAce2:
    def __init__(self, conn):
        self._ace = conn
        self.feed_speed = 80.0
        self._slot_map = {"lane2": 2, "lane3": 3}

    def _wait_for_feed_complete(self, slot, length, speed, **kw):
        return True


def _retract_obj(opts=None, sibling=None, printing=False, conn=None):
    a_conn = conn if conn is not None else _MotionConn()
    ace2 = _MotionAce2(a_conn)
    base = {"lane_slot_map": "lane2:2, lane3:3", "auto_tag_adjust": True}
    base.update(opts or {})
    obj = _mk(ace2, base)
    skw = {"name": "lane3", "tool_loaded": False, "loaded_to_hub": True}
    skw.update(sibling or {})
    lane3 = types.SimpleNamespace(**skw)
    obj.afc = types.SimpleNamespace(
        lanes={"lane3": lane3},
        function=types.SimpleNamespace(is_printing=lambda: printing))
    return obj, a_conn


class TestMaybeRetractSister:
    def test_none_sibling_blocked(self):
        obj, _ = _retract_obj()
        assert obj._maybe_retract_sister(None) == "blocked"

    def test_already_retracted_returns_retracted(self):
        obj, _ = _retract_obj()
        obj._sister_retracted = (3, 75.0, 80.0)
        assert obj._maybe_retract_sister(3) == "retracted"

    def test_feature_off_blocked(self):
        obj, _ = _retract_obj(opts={"auto_tag_adjust": False})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_no_lane_at_slot_blocked(self):
        obj, _ = _retract_obj()
        assert obj._maybe_retract_sister(9) == "blocked"

    def test_tool_loaded_blocked(self):
        obj, _ = _retract_obj(sibling={"tool_loaded": True})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_not_hub_staged_blocked(self):
        obj, _ = _retract_obj(sibling={"loaded_to_hub": False})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_printing_blocked(self):
        obj, _ = _retract_obj(printing=True)
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_success_retracts_and_logs(self):
        obj, conn = _retract_obj(opts={"auto_tag_adjust_dist": 75.0})
        assert obj._maybe_retract_sister(3) == "retracted"
        assert obj._sister_retracted == (3, 75.0, 80.0)
        assert ("unwind_filament", {"index": 3, "length": 75.0, "speed": 80.0}) \
            in conn.calls
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear "
                     "its tag off the shared antenna")]

    def test_unwind_error_blocked_and_logged(self):
        obj, _ = _retract_obj(conn=_MotionConn(unwind_raises=True))
        assert obj._maybe_retract_sister(3) == "blocked"
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: sister retract on slot 3 failed")]


class TestRestoreSister:
    def test_noop_without_retract(self):
        obj, conn = _retract_obj()
        obj._sister_retracted = None
        obj._restore_sister()
        assert obj._sister_retracted is None
        assert obj.logger.messages == []

    def test_refeeds_and_logs(self):
        obj, conn = _retract_obj()
        obj._sister_retracted = (3, 75.0, 80.0)
        obj._restore_sister()
        assert obj._sister_retracted is None
        assert ("feed_filament", {"index": 3, "length": 75.0, "speed": 80.0}) \
            in conn.calls
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: restored sister slot 3 (+75mm) after "
                     "stage read")]


class TestIsPrinting:
    def test_true_when_printing(self):
        obj = _mk(_Ace2(), {})
        obj.afc = types.SimpleNamespace(
            function=types.SimpleNamespace(is_printing=lambda: True))
        assert obj._is_printing() is True

    def test_false_on_exception(self):
        obj = _mk(_Ace2(), {})
        obj.afc = None
        assert obj._is_printing() is False


class TestReaderPowerOff:
    def test_exception_logged(self):
        obj = _mk(_Ace2(), {})
        link = types.SimpleNamespace(
            reader_power=lambda on: (_ for _ in ()).throw(RuntimeError("x")))
        obj._reader_power_off(link)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: stage read reader power-off failed")]


class TestSafeProbeTeardown:
    def test_noop_without_probe(self):
        obj = _mk(_Ace2(), {})
        obj._probe = None
        obj._safe_probe_teardown()
        assert obj.logger.messages == []

    def test_power_off_and_restore_identify(self):
        conn = _MotionConn()
        ace2 = types.SimpleNamespace(_ace=conn)
        obj = _mk(ace2, {"probe_settle": 0.0})
        obj.probe_restore_identify = True
        link = rfid._Ace2RegLink(ace2, 0, power_index=0)
        obj._probe = {"link": link, "shared": (0, 1)}
        obj._safe_probe_teardown()
        assert obj._probe is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("info", "ACE2 RFID teardown: reader_power(off) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on) done")]

    def test_power_off_error_and_identify_not_restored(self):
        conn = _MotionConn()
        conn.send_command = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("wedge"))
        ace2 = types.SimpleNamespace(_ace=conn)
        obj = _mk(ace2, {"probe_settle": 0.0})
        obj.probe_restore_identify = False
        link = rfid._Ace2RegLink(ace2, 0, power_index=0)
        obj._probe = {"link": link, "shared": (0, 1)}
        obj._safe_probe_teardown()
        assert obj.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("exception", "ACE2 RFID: probe power-off failed"),
            ("info", "ACE2 RFID teardown: identify NOT restored (config)")]


# ── read_slot finally-path branches ──────────────────────────────────────────
class _SelConn:
    connected = True

    def __init__(self, raise_power_off=False, raise_enable_true=False):
        self.calls = []
        self._rpo = raise_power_off
        self._ret = raise_enable_true

    def send_command(self, name, params, timeout=None):
        p = dict(params)
        self.calls.append((name, p))
        if (self._rpo and name == "mfrc522_reader_power"
                and not (p["arg"] & 1)):
            raise RuntimeError("power off wedged")
        return {}

    def send_command_async(self, name, params=None):
        p = dict(params or {})
        self.calls.append((name, p))
        if self._ret and name == "set_rfid_enable" and p.get("enable"):
            raise RuntimeError("enable wedged")


def _slot_obj(conn, opts=None):
    ace2 = types.SimpleNamespace(_ace=conn, slot_count=4,
                                 _slot_map={"laneA": 0, "laneB": 1})
    base = {"lane_slot_map": "laneA:0, laneB:1"}
    base.update(opts or {})
    obj = _mk(ace2, base)
    obj.afc = types.SimpleNamespace(
        lanes={"laneA": types.SimpleNamespace(name="laneA", prep_state=True),
               "laneB": types.SimpleNamespace(name="laneB", prep_state=True)})
    return obj


class TestReadSlot:
    def test_missing_ace2_raises(self):
        obj = _mk(_Ace2(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError):
            obj.read_slot(0)

    def test_sibling_match_warns(self, monkeypatch):
        conn = _SelConn()
        obj = _slot_obj(conn)
        obj._slot_uid[1] = "beef"                 # sibling slot 1 already read beef
        monkeypatch.setattr(
            rfid, "read_tag",
            lambda link, **kw: {"uid": "beef", "tag_type": "MifareClassic1k",
                                "filament": {"type": "PLA"}})
        tag = obj.read_slot(0)
        assert tag["uid"] == "beef"
        assert obj._slot_uid[0] == "beef"
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID: slot 0 read matches shared sibling slot 1's "
                        "tag (uid=beef) — check the spool is in slot 0, not 1")]

    def test_power_off_exception_logged(self, monkeypatch):
        conn = _SelConn(raise_power_off=True)
        obj = _slot_obj(conn)
        monkeypatch.setattr(
            rfid, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": None})
        obj.read_slot(0)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: reader power-off failed")]

    def test_reenable_exception_logged(self, monkeypatch):
        conn = _SelConn(raise_enable_true=True)
        obj = _slot_obj(conn, opts={"restore_identify": True,
                                    "skip_factory_autostage": False})
        monkeypatch.setattr(
            rfid, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": None})
        obj.read_slot(0)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: re-enable identify failed"),
            ("exception", "ACE2 RFID: re-enable identify failed")]


class TestReadLane:
    def test_missing_slot_raises(self):
        obj = _mk(_Ace2(), {})
        obj.afc = None
        with pytest.raises(_CmdError):
            obj.read_lane("ghost")

    def test_no_tag_logs(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 0})
        obj = _mk(ace2, {"lane_slot_map": "laneA:0"})
        obj.afc = None
        obj.read_slot = lambda slot: None
        assert obj.read_lane("laneA") is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: no tag read on slot 0")]

    def test_applies_to_lane(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 0})
        obj = _mk(ace2, {"lane_slot_map": "laneA:0"})
        lane = types.SimpleNamespace(name="laneA")
        obj.afc = types.SimpleNamespace(lanes={"laneA": lane})
        tag = {"uid": "aa", "filament": {"type": "PLA"}}
        obj.read_slot = lambda slot: tag
        applied = []
        obj.apply_to_lane = lambda ln, tg: applied.append((ln, tg))
        assert obj.read_lane("laneA") is tag
        assert applied == [(lane, tag)]


class TestLaneAtSlot:
    def test_none_when_no_afc(self):
        obj = _mk(_Ace2(), {})
        obj.afc = None
        assert obj._lane_at_slot(0) is None

    def test_finds_lane_via_ace_map(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 2})
        obj = _mk(ace2, {})
        lane = types.SimpleNamespace(name="laneA")
        obj.afc = types.SimpleNamespace(lanes={"laneA": lane})
        assert obj._lane_at_slot(2) is lane

    def test_none_when_slot_unmapped(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 2})
        obj = _mk(ace2, {})
        obj.afc = types.SimpleNamespace(
            lanes={"laneA": types.SimpleNamespace(name="laneA")})
        assert obj._lane_at_slot(7) is None


class TestHandleSisterDomination:
    def test_blocked_shows_hint_once(self):
        obj, _ = _retract_obj(opts={"auto_tag_adjust": False})
        obj._handle_sister_domination(3)
        assert obj._sister_hint_shown is True
        assert len(obj.gcode.info) == 1
        assert "lane lane3" in obj.gcode.info[0]
        obj._handle_sister_domination(3)              # already shown -> no repeat
        assert len(obj.gcode.info) == 1

    def test_retracted_shows_no_hint(self):
        obj, _ = _retract_obj()
        obj._handle_sister_domination(3)              # movable -> auto retract
        assert obj._sister_hint_shown is False
        assert obj.gcode.info == []


class TestScanLane:
    def test_missing_ace2_raises(self):
        obj = _mk(_Ace2(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError):
            obj.scan_lane("scan1")

    def test_no_tag_logs_and_returns_none(self):
        ace2 = types.SimpleNamespace(_slot_map={"scan1": 0})
        obj = _mk(ace2, {"lane_slot_map": "scan1:0", "scanner_lanes": "scan1",
                        "scan_seconds": 30.0})
        obj._scan_slot = lambda slot, dur, lane_name="": None
        assert obj.scan_lane("scan1") is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID scan: no tag on lane scan1 in 30s")]

    def test_spoolman_sync_exception_logged(self, monkeypatch):
        ace2 = types.SimpleNamespace(_slot_map={"scan1": 0})
        obj = _mk(ace2, {"lane_slot_map": "scan1:0", "scanner_lanes": "scan1"})
        obj.reactor = _Reactor()
        tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA"}}
        obj._scan_slot = lambda slot, dur, lane_name="": tag
        lane = types.SimpleNamespace(name="scan1")
        obj.afc = types.SimpleNamespace(
            lanes={"scan1": lane}, spoolman=object(), moonraker=None,
            spool=types.SimpleNamespace(next_spool_id=1))
        monkeypatch.setattr(
            rfid, "get_auto_spoolman_create",
            lambda ln, default: (_ for _ in ()).throw(RuntimeError("x")))
        monkeypatch.setattr(
            rfid, "sync_rfid_to_spoolman",
            lambda *a, **k: (_ for _ in ()).throw(RuntimeError("sync boom")))
        out = obj.scan_lane("scan1", 5)
        assert out is tag
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID scan Spoolman sync failed: sync boom")]


# ── load_config ──────────────────────────────────────────────────────────────
class TestLoadConfig:
    def test_returns_instance(self):
        cfg = _Config(_Printer(), {})
        obj = rfid.load_config(cfg)
        assert isinstance(obj, rfid.AFC_ACE2_RFID)
