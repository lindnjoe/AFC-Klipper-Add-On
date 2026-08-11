"""
Unit tests for extras/AFC_BambuAMS.py — the Klipper end of the Bambu AMS bridge.

Covers the pure parsing/mapping helpers and the BambuBridge connection's line
handling / command writing with fakes (no hardware, no afcUnit base needed).
"""

from __future__ import annotations

import json
import types

import pytest

import re

from extras.AFC_BambuAMS import (
    BambuBridge,
    _BambuBufferADC,
    _BambuBufferChip,
    _ams_box_logo,
    _ams_box_logo_error,
    _buffer_state,
    bridge_color_to_rgb,
    bridge_slot_to_info,
    build_slot_map,
    clamp_speed,
    parse_bridge_line,
    prep_lane_state,
    unit_env,
    _fault_reason,
)


# ── prep logo alignment ─────────────────────────────────────────────────────────

def _box_rows(logo):
    """Return the box rows (the part after the 3-char banner prefix)."""
    text = re.sub(r"</?span[^>]*>", "", logo)
    rows = []
    for line in text.splitlines():
        if len(line) > 3 and line[3:].startswith(("+", "|")):
            rows.append(line[3:])
    return rows


class TestPrepLogo:
    def test_ready_box_rows_all_equal_width(self):
        rows = _box_rows(_ams_box_logo("BambuAMS", 4, "BambuAMS_1"))
        assert len(rows) == 5
        assert len({len(r) for r in rows}) == 1        # every row same width
        assert rows[0].startswith("+") and rows[0].endswith("+")

    def test_error_box_rows_all_equal_width(self):
        rows = _box_rows(_ams_box_logo_error("BambuAMS", 4, "BambuAMS_1"))
        assert len({len(r) for r in rows}) == 1
        assert "X ERROR" in "".join(rows)

    def test_name_appended(self):
        assert "BambuAMS_1" in _ams_box_logo("BambuAMS", 4, "BambuAMS_1")

    def test_widens_for_long_title(self):
        rows = _box_rows(_ams_box_logo("VeryLongTitleHere", 4, "u"))
        assert len({len(r) for r in rows}) == 1        # still aligned


# ── parse_bridge_line ───────────────────────────────────────────────────────────

class TestParseBridgeLine:
    def test_valid_object(self):
        assert parse_bridge_line('{"evt":"info","fw":"0.1.0"}') == {
            "evt": "info", "fw": "0.1.0"}

    def test_blank_is_none(self):
        assert parse_bridge_line("   ") is None
        assert parse_bridge_line("") is None

    def test_invalid_json_is_none(self):
        assert parse_bridge_line("{not json") is None

    def test_non_object_is_none(self):
        assert parse_bridge_line("[1,2,3]") is None
        assert parse_bridge_line('"a string"') is None


# ── bridge_color_to_rgb ─────────────────────────────────────────────────────────

class TestBridgeColorToRgb:
    def test_rgba_to_rgb_upper(self):
        assert bridge_color_to_rgb("00ae42ff") == "00AE42"

    def test_zero_is_none(self):
        assert bridge_color_to_rgb("00000000") is None

    def test_short_is_none(self):
        assert bridge_color_to_rgb("123") is None

    def test_non_string_is_none(self):
        assert bridge_color_to_rgb(None) is None
        assert bridge_color_to_rgb(0x00AE42) is None

    def test_non_hex_is_none(self):
        assert bridge_color_to_rgb("ZZZZZZFF") is None


# ── bridge_slot_to_info ─────────────────────────────────────────────────────────

class TestBridgeSlotToInfo:
    def test_full_slot(self):
        info = bridge_slot_to_info({
            "i": 2, "present": True, "state": "idle", "material": "PLA",
            "sku": "GFA00", "color": "00ae42ff", "tmin": 220, "tmax": 240,
            "weight": 1000})
        assert info == {
            "index": 2, "present": True, "state": "idle", "material": "PLA",
            "sku": "GFA00", "color": "00AE42", "temp_min": 220,
            "temp_max": 240, "weight": 1000, "rfid_uid": None,
            # Added when spool measurement and tray identity landed. An exact
            # dict comparison is worth keeping -- it is what noticed.
            "remain_pct": None, "tray_uid": None,
            # fw >= 1.9.3.0: the firmware's per-bay scan verdict. None on the
            # older status frames this fixture models. fw >= 1.10.0.0 adds the
            # measured percent, attributed the same way.
            "scan_seq": None, "scan_res": None,
            "meas_pct": None, "meas_seq": None,
            # fw >= 1.10.5.0: the firmware still owes this bay a read, so an
            # empty record means "not fetched yet", not "no tag".
            "reread_pending": False}

    def test_no_uid_ever(self):
        # Bambu never exposes a per-spool UID, even if 'rfid' were present
        info = bridge_slot_to_info({"i": 0, "present": True, "rfid": "aabbcc"})
        assert info["rfid_uid"] is None

    def test_empty_slot(self):
        info = bridge_slot_to_info({
            "i": 0, "present": False, "state": "empty",
            "material": "", "sku": "", "color": "00000000",
            "tmin": 0, "tmax": 0})
        assert info["present"] is False
        assert info["material"] is None
        assert info["sku"] is None
        assert info["color"] is None
        assert info["temp_min"] is None and info["temp_max"] is None
        assert info["rfid_uid"] is None

    def test_missing_fields_default(self):
        info = bridge_slot_to_info({"i": 1})
        assert info["present"] is False
        assert info["state"] == "empty"
        assert info["material"] is None
        assert info["sku"] is None


# ── build_slot_map ──────────────────────────────────────────────────────────────

def _lane(index):
    return types.SimpleNamespace(index=index)


class TestBuildSlotMap:
    def test_maps_1based_to_0based(self):
        lanes = {"a": _lane(1), "b": _lane(4)}
        assert build_slot_map(lanes, 4) == {"a": 0, "b": 3}

    def test_out_of_range_raises(self):
        try:
            build_slot_map({"a": _lane(5)}, 4)
            assert False, "expected ValueError"
        except ValueError as e:
            assert "outside this unit's slots" in str(e)

    def test_zero_index_raises(self):
        try:
            build_slot_map({"a": _lane(0)}, 4)
            assert False, "expected ValueError"
        except ValueError as e:
            assert "index 0" in str(e)

    def test_duplicate_index_raises(self):
        try:
            build_slot_map({"a": _lane(2), "b": _lane(2)}, 4)
            assert False, "expected ValueError"
        except ValueError as e:
            assert "both map to slot" in str(e)


# ── prep_lane_state ─────────────────────────────────────────────────────────────

class TestPrepLaneState:
    def test_present_spool_staged_not_live(self):
        prep, staged, live, msg = prep_lane_state(
            {"present": True, "material": "PLA"}, tool_loaded=False, online=True)
        assert (prep, staged, live) == (True, True, False)
        assert "LOCKED AND LOADED" in msg
        assert "(PLA)" in msg
        assert "offline" not in msg

    def test_tool_loaded_drives_live_occupancy(self):
        prep, staged, live, msg = prep_lane_state(
            {"present": True}, tool_loaded=True, online=True)
        assert live is True

    def test_empty_slot(self):
        prep, staged, live, msg = prep_lane_state(
            {"present": False}, tool_loaded=False, online=True)
        assert (prep, staged, live) == (False, False, False)
        assert msg == "EMPTY READY FOR SPOOL"

    def test_offline_appends_warning(self):
        _, _, _, msg = prep_lane_state({}, tool_loaded=False, online=False)
        assert "AMS offline" in msg

    def test_empty_info_dict(self):
        prep, staged, live, msg = prep_lane_state({}, False, True)
        assert (prep, staged, live) == (False, False, False)


# ── clamp_speed ─────────────────────────────────────────────────────────────────

class TestClampSpeed:
    def test_zero_uses_ceiling(self):
        assert clamp_speed(0, 30) == 30

    def test_negative_uses_ceiling(self):
        assert clamp_speed(-5, 30) == 30

    def test_above_ceiling_clamped(self):
        assert clamp_speed(100, 30) == 30

    def test_within_passes(self):
        assert clamp_speed(20, 30) == 20


class TestBufferState:
    """FPS-style buffer state from the 0..100 fullness."""

    def test_full_is_compressed(self):
        assert _buffer_state(100) == "compressed"
        assert _buffer_state(66) == "compressed"

    def test_demand_is_expanded(self):
        assert _buffer_state(0) == "expanded"
        assert _buffer_state(33) == "expanded"

    def test_middle_is_neutral(self):
        assert _buffer_state(50) == "neutral"
        assert _buffer_state(65) == "neutral"

    def test_unknown_is_none(self):
        assert _buffer_state(None) is None


class TestBambuBufferChip:
    """Virtual ADC pin chip that streams the AMS buffer to a stock FPS buffer."""

    def _chip(self, value=0.42):
        pins = types.SimpleNamespace(register_chip=lambda n, c: None)
        printer = types.SimpleNamespace(
            lookup_object=lambda n: pins if n == "pins" else object(),
            register_event_handler=lambda *a: None,
            get_reactor=lambda: None, config_error=ValueError)
        unit = types.SimpleNamespace(printer=printer,
                                     fps_buffer_value=lambda: value)
        return _BambuBufferChip(unit)

    def test_push_klipper_and_kalico_signatures(self):
        got = []
        adc = _BambuBufferADC(types.SimpleNamespace(printer=None))
        adc.setup_adc_callback(0.1, lambda v: got.append(v))   # klipper (rt, cb)
        adc.push(0.5)
        adc.setup_adc_callback(lambda v: got.append(v))        # kalico (cb)
        adc.push(0.9)
        assert got == [0.5, 0.9]

    def test_setup_pin_adc_only(self):
        chip = self._chip()
        assert isinstance(chip.setup_pin("adc", {}), _BambuBufferADC)
        with pytest.raises(ValueError):
            chip.setup_pin("pwm", {})

    def test_update_streams_unit_value(self):
        got = []
        chip = self._chip(value=0.7)
        adc = chip.setup_pin("adc", {})
        adc.setup_adc_callback(lambda v: got.append(v))
        chip._update(0.0)
        assert got == [0.7]

    def test_update_skips_when_none(self):
        got = []
        chip = self._chip(value=None)
        adc = chip.setup_pin("adc", {})
        adc.setup_adc_callback(lambda v: got.append(v))
        chip._update(0.0)
        assert got == []


# ── BambuBridge ─────────────────────────────────────────────────────────────────

class _Reactor:
    def __init__(self):
        self.async_cbs = []
        self._now = 0.0

    def monotonic(self):
        return self._now

    def advance(self, dt):
        self._now += dt

    def register_async_callback(self, cb):
        self.async_cbs.append(cb)

    def run_pending(self):
        cbs, self.async_cbs = self.async_cbs, []
        for cb in cbs:
            cb(0.0)


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, m):
        self.messages.append(("info", m))

    def warning(self, m):
        self.messages.append(("warning", m))

    def debug(self, m, only_debug=False, traceback=None):
        # Mirrors AFC_logger.debug's signature. only_debug=True means "AFC.log
        # only, never echo to the console"; recorded so tests can assert which
        # lines are kept off an operator's console.
        self.messages.append(("debug", m))
        if only_debug:
            self.file_only = getattr(self, "file_only", []) + [m]


class _Serial:
    def __init__(self):
        self.written = []
        self.closed = False

    def write(self, data):
        self.written.append(data)

    def close(self):
        self.closed = True


def _bridge(on_status=None):
    reactor = _Reactor()
    logger = _Logger()
    seen = []
    bridge = BambuBridge(lambda: _Serial(), reactor, logger)
    bridge.add_listener(on_status or (lambda o: seen.append(o)))
    bridge._serial = _Serial()           # attach without starting a thread
    return bridge, reactor, logger, seen


class TestBambuBridgeHandleLine:
    def test_status_caches_and_hops_to_reactor(self):
        bridge, reactor, logger, seen = _bridge()
        frame = {"evt": "status", "online": True, "slots": [{"i": 0}]}
        bridge.handle_line(json.dumps(frame))
        assert bridge.latest_status() == frame     # cached synchronously
        assert seen == []                           # not before the hop
        reactor.run_pending()
        assert seen == [frame]                      # delivered on the reactor

    def test_error_line_logs(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line('{"evt":"error","msg":"feed refused"}')
        assert logger.messages == [
            ("warning", "AFC bambu: bridge error: feed refused")]
        assert seen == []

    def test_junk_ignored(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line("not json")
        bridge.handle_line("")
        assert bridge.latest_status() is None
        assert seen == []
        assert logger.messages == []

    def test_ack_logged_to_afc_log_and_not_dispatched(self):
        # Motion acks are the record of what the bridge was asked to do, so they
        # belong in AFC.log at debug -- not on the console, and not to status
        # listeners. Sending them to python logging.debug discards them
        # entirely -- klipper runs that at INFO -- rather than merely keeping
        # them off screen.
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line('{"evt":"ack","cmd":"feed","slot":0}')
        assert seen == []
        assert logger.messages == [
            ("debug", "AFC bambu: bridge ack feed (slot 0)")]

    def test_chain_caches_uids(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line(
            '{"evt":"chain","ndisc":3,"nexp":3,"uids":'
            '"A9CD393238310D0030383131,68273B498053B0024C303936,'
            '872C3B871C00B0084A343331"}')
        assert bridge.chain_uids() == [
            "A9CD393238310D0030383131", "68273B498053B0024C303936",
            "872C3B871C00B0084A343331"]

    def test_chain_empty_uids(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line('{"evt":"chain","uids":""}')
        assert bridge.chain_uids() == []


class TestAmsLimits:
    def test_within_limits(self):
        assert _mod.check_ams_limits(["ams1", "ams2", "ht"]) is None
        assert _mod.check_ams_limits(["ams2"] * 4 + ["ht"] * 8) is None  # 4+8=12

    def test_too_many_four_slot(self):
        w = _mod.check_ams_limits(["ams2"] * 5)
        assert "four-slot" in w

    def test_too_many_ht(self):
        w = _mod.check_ams_limits(["ht"] * 9)
        assert "AMS HT" in w

    def test_too_many_total(self):
        w = _mod.check_ams_limits(["ams2"] * 4 + ["ht"] * 9)  # 13 total
        assert "total" in w


class TestUidPinning:
    def _shim(self, unit_uid, chain, ams_index=0, follows=True):
        sent, sched = [], []
        shim = types.SimpleNamespace(
            name="BambuAMS_HT", unit_uid=unit_uid, ams_index=ams_index,
            dry_ams_id=ams_index, _dry_id_follows_index=follows,
            has_heater=True, dry_dev_addr=0x1800,
            _bridge=types.SimpleNamespace(
                send=lambda o: sent.append(o),
                chain_uids=lambda: list(chain)),
            afc=types.SimpleNamespace(reactor=types.SimpleNamespace(
                register_callback=lambda cb, t: sched.append((cb, t)),
                monotonic=lambda: 0.0)),
            logger=types.SimpleNamespace(info=lambda m: None,
                                         warning=lambda m: None))
        shim.SLOTS_PER_UNIT = 4
        shim._slots = [{} for _ in range(4)]
        shim._send_ht_flag = afcBambuAMS._send_ht_flag.__get__(shim)
        # MC poll addressing is pushed alongside the other per-unit config.
        shim.mc_dev_addr = 0x0700
        shim.mc_id_base = 0x00
        shim.mc_ams_id = -1              # -1 -> derive from base|index
        shim._send_mc_addr = afcBambuAMS._send_mc_addr.__get__(shim)
        shim._is_ht = afcBambuAMS._is_ht.__get__(shim)
        # Announces are DEFERRED until a UID resolves to a chain index, so
        # adopting one is what releases them. The shim needs the flag that
        # gate reads.
        shim._announce_deferred = False
        shim._announce_defer_warned = False
        shim._id_resolved = True
        shim._announce_unit = lambda: None
        shim._adopt_index = afcBambuAMS._adopt_index.__get__(shim)
        shim._match_uid_index = afcBambuAMS._match_uid_index.__get__(shim)
        shim._resolve_uid_index = afcBambuAMS._resolve_uid_index.__get__(shim)
        return shim, sent, sched

    @staticmethod
    def _drain(sched):
        """Fire the scheduled callbacks once (simulate the reactor waking them)."""
        pending = list(sched)
        sched.clear()
        for cb, _t in pending:
            cb(None)

    def test_pins_to_uid_index(self):
        # The match is deferred (send chain -> wait for the async reply -> match),
        # so resolving takes one reactor wake-up.
        shim, sent, sched = self._shim("CCC", ["AAA", "BBB", "CCC"], ams_index=0)
        afcBambuAMS._resolve_uid_index(shim, 0)
        assert shim.ams_index == 0                     # not resolved yet (deferred)
        assert len(sched) == 1                         # match scheduled
        self._drain(sched)                             # reply landed -> match
        assert shim.ams_index == 2                     # CCC is at chain index 2
        assert shim.dry_ams_id == 2                    # follows the index
        assert {"cmd": "units", "n": 3} in sent        # re-announced
        assert sched == []                             # resolved, no retry

    def test_already_correct_no_change(self):
        shim, sent, sched = self._shim("BBB", ["AAA", "BBB", "CCC"], ams_index=1)
        afcBambuAMS._resolve_uid_index(shim, 0)
        self._drain(sched)
        assert shim.ams_index == 1                     # unchanged
        assert sched == []

    def test_dry_id_not_updated_when_fixed(self):
        # ams_model with a fixed id (follows=False) keeps its id byte on re-pin
        shim, sent, sched = self._shim("CCC", ["AAA", "BBB", "CCC"], ams_index=0,
                                       follows=False)
        shim.dry_ams_id = 0x80
        afcBambuAMS._resolve_uid_index(shim, 0)
        self._drain(sched)
        assert shim.ams_index == 2
        assert shim.dry_ams_id == 0x80                 # fixed id untouched

    def test_not_found_retries(self):
        shim, sent, sched = self._shim("ZZZ", [], ams_index=0)  # chain not ready
        afcBambuAMS._resolve_uid_index(shim, 0)
        self._drain(sched)                             # match runs, UID not present
        assert shim.ams_index == 0                     # unchanged
        assert len(sched) == 1                         # whole request re-scheduled

    def test_gives_up_after_retries(self):
        shim, sent, sched = self._shim("ZZZ", ["AAA"], ams_index=0)
        afcBambuAMS._resolve_uid_index(shim, 40)       # last try (~1 min ceiling)
        self._drain(sched)                             # match runs, gives up
        assert shim.ams_index == 0
        assert sched == []                             # no more retries

    def test_report_uids_lists_with_occupancy(self):
        out = []
        shim = types.SimpleNamespace(
            _bridge=types.SimpleNamespace(
                chain_uids=lambda: ["AAA", "BBB"],
                latest_status=lambda: {"slots": [
                    {"unit": 0, "i": 0, "present": True, "material": "PLA Basic"},
                    {"unit": 1, "i": 0, "present": False, "material": None}]}),
            gcode=types.SimpleNamespace(respond_info=lambda m: out.append(m)))
        afcBambuAMS._report_uids(shim, 0)
        assert "chain index 0: AAA" in out[0] and "PLA Basic" in out[0]
        assert "chain index 1: BBB" in out[0] and "(empty)" in out[0]

    def test_report_uids_empty(self):
        out = []
        shim = types.SimpleNamespace(
            _bridge=types.SimpleNamespace(chain_uids=lambda: [],
                                          latest_status=lambda: None),
            gcode=types.SimpleNamespace(respond_info=lambda m: out.append(m)))
        afcBambuAMS._report_uids(shim, 0)
        assert "no AMS UIDs" in out[0]


class TestBambuBridgeSend:
    def test_writes_json_line(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.send({"cmd": "select", "slot": 2})
        assert bridge._serial.written == [b'{"cmd": "select", "slot": 2}\n']

    def test_write_failure_drops_port_for_reconnect(self):
        bridge, reactor, logger, seen = _bridge()

        class _Bad:
            def write(self, data):
                raise RuntimeError("boom")

            def close(self):
                pass
        bridge._serial = _Bad()
        bridge.send({"cmd": "stop"})
        assert logger.messages == [
            ("warning", "AFC bambu: bridge write failed: boom; reconnecting")]
        assert bridge._serial is None     # dropped so the reader reconnects

    def test_send_without_serial_is_noop(self):
        bridge, reactor, logger, seen = _bridge()
        bridge._serial = None
        bridge.send({"cmd": "stop"})                # no raise


class TestBambuBridgeReconnect:
    def test_reader_reconnects_after_read_error(self):
        import time as _t
        reactor = _Reactor()
        logger = _Logger()

        class _S:
            def __init__(self, boom):
                self.boom = boom
                self.closed = False
                self._did = False

            def read(self, n):
                if self.boom and not self._did:
                    self._did = True
                    raise RuntimeError("io error")
                _t.sleep(0.01)
                return b""

            def close(self):
                self.closed = True

        made = []

        def factory():
            s = _S(boom=(len(made) == 0))   # first serial errors once
            made.append(s)
            return s

        bridge = BambuBridge(factory, reactor, logger)
        bridge.start()
        deadline = _t.time() + 2.0
        while len(made) < 2 and _t.time() < deadline:
            _t.sleep(0.02)
        bridge.stop()
        assert len(made) >= 2               # re-opened the port, didn't die
        assert made[0].closed is True       # dropped the broken port
        assert any(m[1] == "AFC bambu: bridge reconnected"
                   for m in logger.messages)


class TestBambuBridgeStop:
    def test_stop_closes_serial(self):
        bridge, reactor, logger, seen = _bridge()
        ser = bridge._serial
        bridge.stop()
        assert bridge._run is False
        assert ser.closed is True

    def test_latest_status_is_a_copy(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.handle_line('{"evt":"status","online":true,"slots":[]}')
        snap = bridge.latest_status()
        snap["online"] = False
        assert bridge.latest_status()["online"] is True   # cache untouched


# ── virtual-hub behavior (OpenAMS-style) ────────────────────────────────────────

from extras.AFC_BambuAMS import afcBambuAMS, AFCLaneState  # noqa: E402


class TestFollowTick:
    """The follower holds for as long as a lane is loaded to the toolhead.

    Not demand-gated. A real printer streams the hold (op-04 mode 07 / ref 7F)
    at a 149ms median for the whole time a tray is loaded: 3664 frames in
    547.6s, where perfectly continuous would be 3675. It never waits to see the
    extruder move. Asserting the opposite -- "idle does not ping" -- leaves
    the HT arming and dropping once a second instead of assisting.
    """

    def _shim(self, e_start=100.0):
        th_ext = object()
        state = {"e": e_start, "sent": []}
        toolhead = types.SimpleNamespace(
            get_position=lambda: [0, 0, 0, state["e"]],
            get_extruder=lambda: th_ext)
        lane = types.SimpleNamespace(
            name="lane1",
            extruder_obj=types.SimpleNamespace(toolhead_extruder=th_ext))
        shim = types.SimpleNamespace(
            _following_lane=lane, _follow_last_e=None,
            _bridge=types.SimpleNamespace(
                send=lambda o: state["sent"].append(o),
                latest_status=lambda: {"fstate": 4}),
            follow_min_extrude=0.4, follow_when_loaded=False,            _follow_manual_off=False, _unload_in_progress=False,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            follow_poll_interval=0.3, ams_index=0, _slot_map={"lane1": 0},
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_holds_from_the_very_first_tick(self):
        shim, state = self._shim(e_start=100.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]

    def test_holds_while_extruding(self):
        shim, state = self._shim(e_start=100.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        state["e"] = 100.5
        afcBambuAMS._follow_tick(shim, 1.3)
        assert state["sent"] == [{"cmd": "follow"}] * 2

    def test_holds_at_idle_too(self):
        """The case that mattered: no extrusion at all, hold every tick.

        Travel moves, between layers, a paused print -- the tray stays held.
        """
        shim, state = self._shim(e_start=100.0)
        for t in (1.0, 1.3, 1.6):
            afcBambuAMS._follow_tick(shim, t)
        assert state["sent"] == [{"cmd": "follow"}] * 3

    def test_reschedules(self):
        shim, _ = self._shim()
        nxt = afcBambuAMS._follow_tick(shim, 5.0)
        assert nxt == 5.0 + shim.follow_poll_interval


class TestFollowAutoArm:
    """follow_when_loaded: auto-engage + proactive ping while a lane is loaded,
    independent of the buffer readback and the per-lane extruder wiring."""

    def _shim(self, loaded_lane=None, following=None, fstate=4, e=0.0,
              last_e=None):
        state = {"sent": [], "engaged": [], "assist": [], "e": e}
        toolhead = types.SimpleNamespace(
            get_position=lambda: [0, 0, 0, state["e"]])
        shim = types.SimpleNamespace(
            _following_lane=following, _follow_last_e=last_e,
            follow_min_extrude=0.05,
            _bridge=types.SimpleNamespace(
                send=lambda o: state["sent"].append(o),
                latest_status=lambda: {"fstate": fstate, "buff": 120}),
            follow_when_loaded=True,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            follow_debug_interval=0.0, _follow_last_log=0.0,
            follow_poll_interval=0.1, ams_index=0,
            _tool_loaded_lane=lambda: loaded_lane,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            _engage_follower=lambda ln: (state["engaged"].append(ln.name),
                                         setattr(shim, "_following_lane", ln)),
            set_feed_assist=lambda ln, on: (
                state["assist"].append((ln.name, on)),
                setattr(shim, "_following_lane", ln if on else None)),
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_auto_arm_stands_down_during_unload(self):
        # The unload reels via a retract STREAM the bridge cancels on any
        # assist-on (s_motion=0). While _unload_in_progress is set the
        # auto-arm must not touch the follower at all.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=None)
        shim._unload_in_progress = True
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["engaged"] == []
        assert state["assist"] == []

    def test_engages_and_holds_as_soon_as_a_lane_is_loaded(self):
        # Loaded is the whole condition. The tray goes to mode:4 AND the hold
        # starts on the same tick -- no waiting to see the extruder move.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=None)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["engaged"] == ["lane1"]
        assert state["sent"] == [{"cmd": "follow"}]
        assert shim._following_lane is lane

    def test_holds_while_the_extruder_advances(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, e=1.0,
                                 last_e=0.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]

    def test_a_retract_does_not_drop_the_hold(self):
        # E going BACKWARDS must not reset the baseline or skip the ping: a
        # retract is not a reason to stop holding the tray.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, e=2.0,
                                 last_e=5.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]

    def test_reasserts_mode4_when_dropped(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        afcBambuAMS._follow_tick(shim, 10.0)
        assert state["assist"] == [("lane1", True)]     # re-asserted mode:4
        assert state["sent"] == [{"cmd": "follow"}]     # and still held

    def test_state_0_without_extrusion_is_resting_not_dropped(self):
        # Do not re-assert on state 0 alone; the 2s rate limit does not stop
        # that becoming a storm. MEASURED at a healthy, loaded,
        # IDLE unit: 14 assist re-arms in 30 seconds -- one every two seconds,
        # forever, each narrated by the unit and each a motor nudge. The rate
        # limit does not prevent the storm, it sets its period.
        #
        # state 0 at an idle unit means CENTRED, not dropped, and the printer
        # holds a loaded tray without re-issuing anything: 3664 hold frames in
        # 547.6 s, ~100% continuous. The hold is a stream, not a repeated
        # command.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        shim._follow_last_demand = 0.0          # no extrusion for ages
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["assist"] == []            # left alone
        assert state["sent"] == [{"cmd": "follow"}]   # still held by the stream

    def test_state_0_with_recent_extrusion_still_reasserts(self):
        # The other half, and the reason the gate has a window rather than
        # being removed: a tray that drops to IDLE while the printer is between
        # extrusions must not stay dropped once it is asked for filament again.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        shim._follow_last_demand = 99.0         # extruded a moment ago
        afcBambuAMS._follow_tick(shim, 100.0)   # inside follow_rearm_window
        assert state["assist"] == [("lane1", True)]

    def test_the_reassert_is_still_rate_limited(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        afcBambuAMS._follow_tick(shim, 100.0)
        afcBambuAMS._follow_tick(shim, 100.5)   # inside the 2s window
        assert state["assist"] == [("lane1", True)]

    def test_following_state_3_does_not_reassert(self):
        # An AMS HT follows at state:3 -- lane loaded, buffer held, feeding.
        # The old "not 4" test was true on every tick there and turned the
        # auto-arm into a 2s assist storm at a perfectly healthy unit.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=3)
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["assist"] == []

    def test_reassert_is_rate_limited(self):
        # fstate stuck !=4 must NOT become a 10/s assist storm (each
        # assist-on cancels a retract stream in the bridge firmware).
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        afcBambuAMS._follow_tick(shim, 10.0)
        afcBambuAMS._follow_tick(shim, 10.1)     # 100ms later: suppressed
        afcBambuAMS._follow_tick(shim, 11.9)     # still inside 2s window
        assert state["assist"] == [("lane1", True)]
        afcBambuAMS._follow_tick(shim, 12.1)     # window expired: re-assert
        assert state["assist"] == [("lane1", True), ("lane1", True)]

    def test_stops_when_nothing_loaded(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=None, following=lane)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["assist"] == [("lane1", False)]    # stopped
        assert state["sent"] == []                       # silent, no ping
        assert shim._following_lane is None

    def test_short_circuits_while_drying_and_idle(self):
        # Drying with NOTHING threaded to the toolhead: idle the tick so the
        # AMS can run its self-check undisturbed.
        shim, state = self._shim(loaded_lane=None, following=None)
        shim._drying = True
        nxt = afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == []               # no follow/select pumped
        assert state["assist"] == []             # follower left as-is
        assert nxt == 1.0 + shim.follow_poll_interval

    def test_keeps_following_while_drying_when_tool_loaded(self):
        # Dry-while-printing: a lane feeding the toolhead must keep its
        # follower, or the extruder fights the pull (field-observed on an HT
        # purge during a dry cycle).
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=None)
        shim._drying = True
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["engaged"] == ["lane1"]     # follower (re)engaged


class TestToolLoadedLaneActiveGate:
    """_tool_loaded_lane only returns a lane whose toolhead is ON SHUTTLE --
    a docked toolhead is not printing and needs no follower."""

    def _unit(self, lanes, current=None):
        fn = types.SimpleNamespace(get_current_extruder=lambda: current)
        u = types.SimpleNamespace(lanes={l.name: l for l in lanes},
                                  _slot_of=lambda ln: 0,
                                  afc=types.SimpleNamespace(function=fn))
        return u

    def _lane(self, name, tool_loaded=True, on_shuttle=True, ext=True,
              ext_name=None):
        e = None
        if ext:
            e = types.SimpleNamespace(on_shuttle=lambda: on_shuttle,
                                      th_extruder_name=ext_name or name + "_e",
                                      name=ext_name or name + "_e")
        return types.SimpleNamespace(name=name, tool_loaded=tool_loaded,
                                     extruder_obj=e)

    def test_active_extruder_lane_returned(self):
        lane = self._lane("lane1", ext_name="extruder")
        u = self._unit([lane], current="extruder")
        assert afcBambuAMS._tool_loaded_lane(u) is lane

    def test_inactive_extruder_skipped(self):
        lane = self._lane("lane1", ext_name="extruder1")
        u = self._unit([lane], current="extruder")
        assert afcBambuAMS._tool_loaded_lane(u) is None

    def test_active_preferred_over_inactive(self):
        other = self._lane("lane1", ext_name="extruder1")
        active = self._lane("lane2", ext_name="extruder")
        u = self._unit([other, active], current="extruder")
        assert afcBambuAMS._tool_loaded_lane(u) is active

    def test_docked_but_active_extruder_still_follows(self):
        # Async/pre-load: the toolhead is docked yet IS the active extruder --
        # it needs its follower, so on_shuttle() must not veto it.
        lane = self._lane("lane1", on_shuttle=False, ext_name="extruder")
        u = self._unit([lane], current="extruder")
        assert afcBambuAMS._tool_loaded_lane(u) is lane

    def test_no_active_answer_still_follows_a_loaded_lane(self):
        """Uncertainty must not strip the follower.

        Falling back to on_shuttle() breaks this: a docked toolhead answers
        False, so after a G28, or after a Klipper restart with a lane still
        tool_loaded, the follower tick sees "nothing loaded here" and actively
        STOPS the follower. Live: the tray took the arm, held state:4 for
        under a second, dropped to state:0, and filament pulled by hand was
        never recovered. A real printer holds a loaded tray unconditionally.
        """
        lane = types.SimpleNamespace(
            name="lane1", tool_loaded=True,
            extruder_obj=types.SimpleNamespace(on_shuttle=lambda: False))
        shim = types.SimpleNamespace(
            lanes={"lane1": lane}, _slot_of=lambda ln: 0,
            afc=types.SimpleNamespace(
                function=types.SimpleNamespace(
                    get_current_extruder=lambda: None)))
        assert afcBambuAMS._tool_loaded_lane(shim) is lane

    def test_no_extruder_object_treated_as_active(self):
        # Single-toolhead / unwired lanes must not lose their follower.
        lane = self._lane("lane1", ext=False)
        u = self._unit([lane])
        assert afcBambuAMS._tool_loaded_lane(u) is lane

    def test_not_tool_loaded_skipped(self):
        lane = self._lane("lane1", tool_loaded=False)
        u = self._unit([lane])
        assert afcBambuAMS._tool_loaded_lane(u) is None


class _HeaterGcmd:
    """Minimal GCodeCommand stand-in for the heater-start command."""

    def __init__(self, params):
        self._p = params
        self.info = []

    def get_int(self, name, default, minval=None, maxval=None):
        v = int(self._p.get(name, default))
        if minval is not None and v < minval:
            raise ValueError(f"{name} below {minval}")
        if maxval is not None and v > maxval:
            raise ValueError(f"{name} above {maxval}")
        return v

    def respond_info(self, msg):
        self.info.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


def _heater_shim(lanes, name="BambuAMS_2", ams_index=1, dev_addr=0x0700):
    """Heater-command shim.

    Defaults to a BOXED unit (0x0700). The out-of-bay pre-check is HT-only --
    an ACE and an ACE 2 heat while printing and the AMS 2 Pro is untested, so
    only an HT is warned -- and _is_ht() reads dry_dev_addr, which is why it
    is a parameter here rather than a constant.
    """
    sent = []
    assist = []
    shim = types.SimpleNamespace(
        name=name, ams_index=ams_index, lanes=lanes, has_heater=True,
        dry_max_temp=_mod.MAX_DRY_TEMP_C, dry_dev_addr=dev_addr,
        dry_ams_id=ams_index,
        _following_lane=None, _drying=False,
        _bridge=types.SimpleNamespace(send=lambda o: sent.append(o)),
        set_feed_assist=lambda ln, on: assist.append((ln.name, on)))
    shim._committed_lanes = lambda: afcBambuAMS._committed_lanes(shim)
    shim._is_ht = lambda: afcBambuAMS._is_ht(shim)
    return shim, sent, assist


class TestCommittedLanes:
    def _lane(self, name, tool_loaded=False, loaded_to_hub=False):
        return types.SimpleNamespace(
            name=name, tool_loaded=tool_loaded, loaded_to_hub=loaded_to_hub)

    def test_bay_only_lane_not_committed(self):
        lanes = {"a": self._lane("a")}                 # present in bay only
        shim, _, _ = _heater_shim(lanes)
        assert afcBambuAMS._committed_lanes(shim) == []

    def test_tool_loaded_is_committed(self):
        lanes = {"a": self._lane("a", tool_loaded=True)}
        shim, _, _ = _heater_shim(lanes)
        assert [l.name for l in afcBambuAMS._committed_lanes(shim)] == ["a"]

    def test_staged_at_hub_is_committed(self):
        lanes = {"a": self._lane("a", loaded_to_hub=True)}
        shim, _, _ = _heater_shim(lanes)
        assert [l.name for l in afcBambuAMS._committed_lanes(shim)] == ["a"]


class TestHeaterStart:
    def _lane(self, name, tool_loaded=False, loaded_to_hub=False):
        return types.SimpleNamespace(
            name=name, tool_loaded=tool_loaded, loaded_to_hub=loaded_to_hub)

    def test_rotate_allowed_when_bay_only(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 1})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 1
        assert shim._drying is True

    def test_rotate_gated_when_lane_committed(self):
        lanes = {"a": self._lane("a", tool_loaded=True)}
        shim, sent, _ = _heater_shim(lanes)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 1})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 0
        assert any("ROTATE disabled" in m for m in gcmd.info)

    def test_temp_clamped_to_ceiling(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes)
        gcmd = _HeaterGcmd({"TEMP": 85, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == _mod.MAX_DRY_TEMP_C
        assert any("clamping" in m for m in gcmd.info)

    def test_ht_ceiling_allows_85(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3")
        shim.dry_max_temp = 85                              # AMS HT
        gcmd = _HeaterGcmd({"TEMP": 85, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == 85                        # not clamped
        assert not any("clamping" in m for m in gcmd.info)

    def test_ht_addressing_sent(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3", ams_index=2)
        shim.dry_dev_addr = 0x1800                          # AMS HT device addr
        shim.dry_ams_id = 2                                 # HT id = chain index
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["addr"] == 0x1800
        assert sent[0]["amsid"] == 2

    def test_ams2pro_addressing_default(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_2", ams_index=1)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["addr"] == 0x0700
        assert sent[0]["amsid"] == 1                        # ams_index

    def test_dry_models_table(self):
        # (heater, dev_addr, ams_id, max_temp)
        assert _mod._AMS_MODELS["ht"] == (True, 0x1800, None, 85)
        assert _mod._AMS_MODELS["ams2"] == (True, 0x0700, None, 65)
        assert _mod._AMS_MODELS["ams1"] == (False, 0x0700, None, 65)  # no heater

    def test_ht_ceiling_still_clamps_above_85(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3")
        shim.dry_max_temp = 85
        gcmd = _HeaterGcmd({"TEMP": 99, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == 85
        assert any("clamping" in m for m in gcmd.info)

    def test_stop_carries_ht_addressing(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3", ams_index=2)
        shim.dry_dev_addr = 0x1800
        shim.dry_ams_id = 2
        shim._drying = True
        gcmd = _HeaterGcmd({})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_STOP(shim, gcmd)
        assert sent[0]["on"] == 0
        assert sent[0]["addr"] == 0x1800                    # HT hears the stop
        assert sent[0]["amsid"] == 2
        assert shim._drying is False

    def test_ignored_on_heaterless_unit(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_1")
        shim.has_heater = False
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent == []                                   # nothing sent
        assert shim._drying is False
        assert any("no drying heater" in m for m in gcmd.info)


def _vhub(virtual=True):
    return types.SimpleNamespace(is_virtual_pin=lambda: virtual)


def _sync_shim(slot_map, lanes, slots, verdict="none", surfaced=None,
               finalized=None):
    """Duck-typed self for afcBambuAMS._sync_lanes (no Klipper needed).

    ``verdict`` stands in for what the unit said about a scan on the bay --
    the single thing _sync_lanes consults before letting a record reach a lane.
    """
    return types.SimpleNamespace(
        _slot_map=slot_map, lanes=lanes, _slots=slots,
        _ACTIVE_STATES=afcBambuAMS._ACTIVE_STATES,
        _is_virtual_hub=afcBambuAMS._is_virtual_hub,
        _maybe_auto_scan=lambda slot, present, info: None,
        lane_loaded=lambda lane: None,
        lane_not_ready=lambda lane: None,
        lane_illuminate_spool=lambda lane: None,
        # No scan open by default.
        _scan_verdict=lambda slot: verdict,
        _scan_notag=[False] * max(len(slots), 1),
        _afc_owned=set(), _prep_seen=True,
        _measure_settled=lambda slot, info: True,
        _finalize_scan=lambda slot: (finalized if finalized is not None
                                     else []).append(slot),
        _release_scan_hold=lambda slot: None,
        _apply_remain_weight=lambda lane, info: None,
        _surface_slot_info=lambda lane, info: (
            surfaced if surfaced is not None else []).append(lane))


class TestIsVirtualHub:
    def test_no_hub_obj(self):
        assert afcBambuAMS._is_virtual_hub(
            types.SimpleNamespace(hub_obj=None)) is False

    def test_hub_without_is_virtual_pin(self):
        assert afcBambuAMS._is_virtual_hub(
            types.SimpleNamespace(hub_obj=object())) is False

    def test_virtual_hub(self):
        assert afcBambuAMS._is_virtual_hub(
            types.SimpleNamespace(hub_obj=_vhub(True))) is True

    def test_physical_hub(self):
        assert afcBambuAMS._is_virtual_hub(
            types.SimpleNamespace(hub_obj=_vhub(False))) is False


class TestSyncLanes:
    def _lane(self, virtual=True, tool_loaded=False, status=None):
        return types.SimpleNamespace(
            hub_obj=_vhub(virtual) if virtual is not None else None,
            tool_loaded=tool_loaded, prep_state=None, _load_state=None,
            loaded_to_hub=None, status=status)

    def test_presence_drives_prep_state(self):
        lane = self._lane()
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane.prep_state is True

    def test_present_spool_is_staged_and_loaded(self):
        # never "detected but not loaded": present -> loaded_to_hub + LOADED
        lane = self._lane(status=AFCLaneState.NONE)
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane.loaded_to_hub is True
        assert lane.status == AFCLaneState.LOADED

    def test_empty_bay_clears_stage_and_status(self):
        lane = self._lane(status=AFCLaneState.LOADED)
        lane.loaded_to_hub = True
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": False}])
        afcBambuAMS._sync_lanes(shim)
        assert lane.loaded_to_hub is False
        assert lane.status == AFCLaneState.NONE

    def test_active_state_not_overwritten(self):
        # a mid-flight load owns the status; a passive poll must not clobber it
        lane = self._lane(status=AFCLaneState.TOOL_LOADED)
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane.status == AFCLaneState.TOOL_LOADED
        assert lane.loaded_to_hub is True                # still latched staged

    def test_staged_lane_keeps_hub_clear(self):
        # present but NOT tool_loaded: live hub occupancy must stay False so
        # the lane's own load isn't blocked by "hub not clear"
        lane = self._lane(tool_loaded=False)
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane._load_state is False

    def test_tool_loaded_lane_occupies_hub(self):
        lane = self._lane(tool_loaded=True)
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane._load_state is True

    def test_physical_hub_load_state_untouched(self):
        # a real hub switch drives _load_state via its own pin callback
        lane = self._lane(virtual=False, tool_loaded=True)
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}])
        afcBambuAMS._sync_lanes(shim)
        assert lane._load_state is None

    def test_missing_lane_and_empty_info_skipped(self):
        lane = self._lane()
        shim = _sync_shim({"gone": 0, "l": 1}, {"l": lane}, [{}, {}])
        afcBambuAMS._sync_lanes(shim)                    # no raise
        assert lane.prep_state is None                   # empty info skipped


# ── auto-scan on new spool insertion (duck-typed self) ──────────────────────────

class _FakeReactor:
    """Minimal reactor: records deferred callbacks so a test can fire them."""
    def __init__(self):
        self.cbs = []

    def monotonic(self):
        return 0.0

    def register_callback(self, cb, when=0.0):
        self.cbs.append((cb, when))

    def fire_all(self):
        for cb, _when in list(self.cbs):
            cb(None)


def _autoscan_shim(auto_scan=True, in_print=False, prev=False, scanned=False,
                   dev_addr=0x0700, reactor=None):
    """Duck-typed self for afcBambuAMS._maybe_auto_scan.

    dev_addr picks the AMS type: 0x0700 = boxed AMS / AMS2 Pro -- the module
    drives the scan on insert (feed past the bay reader). 0x1800 = AMS HT -- the
    FIRMWARE arms the scan on the insert edge (the HT scans itself on its preload
    switch), so the module must NOT send a scan for it.
    """
    scans = []
    shim = types.SimpleNamespace(
        auto_scan=auto_scan,
        name="AMS_1",
        dry_dev_addr=dev_addr,
        has_heater=(dev_addr == 0x1800),
        _slot_map={},
        lanes={},
        _prev_present=[prev, prev, prev, prev],
        _auto_scanned=[scanned, scanned, scanned, scanned],
        _scan_t0=[None] * 4,
        _scan_notag=[False] * 4,
        SCAN_FALLBACK_CAP=45.0,
        # A bare object() is not enough: a BOXED unit's insert scan goes out
        # as a capscan FRAME rather than through self.scan(), so record that
        # here too. What
        # these tests mean is "bay N got scanned", and that has to stay true
        # however the scan is delivered. The HT path still uses self.scan().
        _bridge=types.SimpleNamespace(
            send=lambda o: (scans.append(o["slot"])
                            if o.get("cmd") == "capscan" else None)),
        # The capscan frame is addressed to a unit, so the shim needs an index.
        ams_index=0,
        logger=_Logger(),
        afc=types.SimpleNamespace(
            function=types.SimpleNamespace(in_print=lambda: in_print),
            reactor=reactor),
        _finalize_scan=lambda s: None,
        scan=lambda slot: scans.append(slot))
    shim._is_ht = afcBambuAMS._is_ht.__get__(shim)
    shim._lane_for_slot = afcBambuAMS._lane_for_slot.__get__(shim)
    shim._release_scan_hold = afcBambuAMS._release_scan_hold.__get__(shim)
    shim._open_scan = afcBambuAMS._open_scan.__get__(shim)
    return shim, scans


class TestMaybeAutoScan:
    def test_insertion_triggers_scan(self):
        shim, scans = _autoscan_shim()
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [0] and shim._auto_scanned[0] is True

    def test_no_edge_when_already_present(self):
        shim, scans = _autoscan_shim(prev=True)
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == []                               # no 0->1 edge

    def test_removal_resets_latch(self):
        shim, scans = _autoscan_shim(prev=True, scanned=True)
        afcBambuAMS._maybe_auto_scan(shim, 0, False, {})
        assert shim._auto_scanned[0] is False            # reinsertion re-scans

    def test_insert_scans_even_when_slot_shows_material(self):
        # A swapped-in spool must re-read even if the slot still shows the
        # previous spool's material (the HT never re-reads on its own).
        shim, scans = _autoscan_shim()
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {"material": "PLA"})
        assert scans == [0] and shim._auto_scanned[0] is True

    def test_ht_insert_no_module_scan(self):
        # AMS HT (0x1800): the firmware arms the scan on the insert edge, so the
        # module must NOT send its own scan (that would read the HT's stale flash
        # and clobber the firmware's min-window).
        shim, scans = _autoscan_shim(dev_addr=0x1800, reactor=_FakeReactor())
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [] and shim._auto_scanned[0] is True

    def test_boxed_ams_scans_on_insert(self):
        # Boxed AMS (0x0700): the module drives the scan on the insert edge.
        shim, scans = _autoscan_shim(dev_addr=0x0700, reactor=_FakeReactor())
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [0]

    def test_not_primed_skips_scan(self):
        # Startup baseline: spools already present at boot are recorded, not
        # scanned (reboot doesn't re-read what AFC restored at prep).
        shim, scans = _autoscan_shim()
        shim._scan_primed = False
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [] and shim._prev_present[0] is True

    def test_scan_after_priming(self):
        # A real 0->1 insert AFTER the baseline is primed still scans, even if a
        # value was left on the lane.
        shim, scans = _autoscan_shim()
        shim._scan_primed = True
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [0]

    def test_disabled_by_config(self):
        shim, scans = _autoscan_shim(auto_scan=False)
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == []

    def test_never_scans_mid_print(self):
        shim, scans = _autoscan_shim(in_print=True)
        afcBambuAMS._maybe_auto_scan(shim, 0, True, {})
        assert scans == [] and shim._auto_scanned[0] is False  # retries later


# ── load: feed-until-sensor stops the AMS the instant filament arrives ──────────

class _Clock:
    """Fake reactor clock: pause() jumps the clock forward to the wake time."""
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def pause(self, until):
        self.t = max(self.t, until)


def _load_shim(sensor_after=1, timeout=5.0, arrivals=None,
               ams_arrival=True):
    """Feed-loop shim.

    ``arrivals`` replays what the bridge reports as motion completions, one
    entry consumed per poll: (sequence, ok). The default never completes, so
    the AMS-arrival path stays out of the way of the sensor tests.
    """
    clock = _Clock()
    calls = {"stop": 0, "feed": [], "sensor": 0, "arm": []}

    def sensor(lane):
        calls["sensor"] += 1
        return calls["sensor"] > sensor_after

    replay = list(arrivals or [])

    def finish_since(start_seq):
        if not replay:
            return (False, False)
        seq, ok = replay.pop(0)
        return (seq != start_seq, ok)

    shim = types.SimpleNamespace(
        name="AMS", logger=_Logger(),
        load_retry_timeout=timeout, load_retry_pulse=100.0,
        load_retry_interval=1.0,
        ams_arrival_completes_load=ams_arrival,
        afc=types.SimpleNamespace(reactor=clock),
        _toolhead_sensor_triggered=sensor,
        _ams_declared_fault=lambda: False,
        _finish_seq_now=lambda: 0,
        _finish_since=finish_since,
        # Default to "there is a sensor", which is the safe reading and what
        # every real lane on this machine has.
        _has_toolhead_sensor=lambda ln: True,
        # The odometer read at the sensor trip -- telemetry, never control.
        # None here is the "unit has not reported one" case, which must not
        # disturb a single thing about the loop.
        _odom_now_mm=lambda: None,
        _load_odom_at_sensor=None,
        stop=lambda: calls.__setitem__("stop", calls["stop"] + 1),
        feed=lambda lane, mm: calls["feed"].append(mm))
    return shim, calls, clock


_LANE = types.SimpleNamespace(name="lane1")


class TestRecover:
    """AFC_BAMBU_RECOVER / eject-based recovery of a stuck load."""

    def _shim(self, present=True):
        order = []
        lane = types.SimpleNamespace(name="lane1", tool_loaded=True,
                                     loaded_to_hub=True,
                                     status="TOOL_LOADING", _load_state=True,
                                     hub_obj=None)
        shim = types.SimpleNamespace(
            name="BambuAMS_1", logger=_Logger(), _bridge=object(),
            _slot_map={"lane1": 0}, lanes={"lane1": lane},
            _slots=[{"present": present}],
            afc=types.SimpleNamespace(save_vars=lambda: order.append("save")),
            set_feed_assist=lambda ln, on: order.append(("assist", on)),
            eject_lane=lambda ln: order.append("eject"),
            lane_loaded=lambda ln: order.append("loaded"),
            lane_illuminate_spool=lambda ln: None,
            lane_not_ready=lambda ln: order.append("not_ready"))
        shim._slot_of = lambda ln: afcBambuAMS._slot_of(shim, ln)
        shim._is_virtual_hub = lambda ln: afcBambuAMS._is_virtual_hub(ln)
        return shim, lane, order

    def test_recover_present_bay_stays_staged(self):
        shim, lane, order = self._shim(present=True)
        afcBambuAMS._recover_to_bay(shim, lane)
        assert ("assist", False) in order      # follower dropped
        assert "eject" in order                # reeled back
        assert lane.tool_loaded is False
        assert lane.loaded_to_hub is True      # staged, not "detected not loaded"
        assert lane.status == AFCLaneState.LOADED
        assert order[-1] == "save"             # persisted

    def test_recover_empty_bay_goes_none(self):
        shim, lane, order = self._shim(present=False)
        afcBambuAMS._recover_to_bay(shim, lane)
        assert lane.loaded_to_hub is False
        assert lane.status == AFCLaneState.NONE

    def test_reset_command_routes_to_recover(self):
        shim = types.SimpleNamespace(name="BambuAMS_1")
        lane = types.SimpleNamespace(name="lane3")
        cmd = afcBambuAMS.get_lane_reset_command(shim, lane, 50.0)
        assert cmd == "AFC_BAMBU_RECOVER UNIT=BambuAMS_1 LANE=lane3"


class TestPrepArmsFollower:
    """prep_post_load arms the follower for a lane tool-loaded at startup prep."""

    @staticmethod
    def _shim(tool_loaded, present, bridge=True):
        engaged = []
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            _bridge=object() if bridge else None,
            _slots=[{"present": present}],
            _slot_of=lambda ln: 0,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            _engage_follower=lambda ln: engaged.append(ln))
        lane = types.SimpleNamespace(
            name="lane1", tool_loaded=tool_loaded, loaded_to_hub=False)
        return shim, lane, engaged

    def test_tool_loaded_arms_follower(self):
        shim, lane, engaged = self._shim(tool_loaded=True, present=True)
        afcBambuAMS.prep_post_load(shim, lane)
        assert engaged == [lane]              # follower engaged at prep
        assert lane.loaded_to_hub is True

    def test_staged_only_does_not_arm(self):
        shim, lane, engaged = self._shim(tool_loaded=False, present=True)
        afcBambuAMS.prep_post_load(shim, lane)
        assert engaged == []                  # merely staged -> no follower
        assert lane.loaded_to_hub is True

    def test_no_bridge_skips_arm(self):
        shim, lane, engaged = self._shim(tool_loaded=True, present=False,
                                         bridge=False)
        afcBambuAMS.prep_post_load(shim, lane)
        assert engaged == []                  # can't arm without the bridge


class TestFeedUntilSensor:
    def test_already_at_sensor_arms_and_returns(self):
        shim, calls, _ = _load_shim(sensor_after=0)      # triggers immediately
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True
        assert calls["stop"] == 1                        # halted right away
        assert calls["feed"] == []                       # no feed needed

    def test_it_arms_the_instant_the_sensor_triggers(self):
        # THE LOAD CHOREOGRAPHY DEPENDS ON THIS. The phase machine gates its
        # load transition on the follow flag -- `phase_to(loaded ? PH_ARRIVED
        # : PH_IDLE)` -- and stop() clears that flag, so a bare stop here sent
        # every load DRIVE -> IDLE and skipped 09/A5 and the 07/00 pre
        # entirely. bb_assist sets the flag and clears the motion in one call.
        shim, calls, _ = _load_shim(sensor_after=3)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True
        assert calls["stop"] == 1                        # exactly one halt
        assert len(calls["feed"]) >= 1                   # kicked the feed

    def test_timeout_stops_and_fails(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.3)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 0.3) is False
        assert calls["stop"] == 1                        # never left it feeding


class TestLoadRecover:
    """On a stalled load, unit_load_lane runs the printer's Retry: re-home reset
    then re-feed, up to load_recover_attempts, before failing."""

    @staticmethod
    def _shim(attempts):
        calls = {"rehome": 0, "feed": [], "stop": 0, "fail": 0}
        afc = types.SimpleNamespace(
            reactor=_Clock(),
            error=types.SimpleNamespace(
                handle_lane_failure=lambda *a, **k:
                    calls.__setitem__("fail", calls["fail"] + 1)),
            function=types.SimpleNamespace(in_print=lambda: False))
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), _bridge=object(), afc=afc,
            afc_bowden_length=100.0, load_retry_timeout=1.0,
            load_recover_attempts=attempts, reel_back_on_load_fail=False,
            select_lane=lambda ln: (True, 0),
            _toolhead_sensor_triggered=lambda ln: False,
            feed=lambda ln, mm: calls["feed"].append(mm),
            stop=lambda: calls.__setitem__("stop", calls["stop"] + 1),
            rehome=lambda: (calls.__setitem__("rehome", calls["rehome"] + 1)
                            or True),
            measured_path_mm=lambda: None,       # unit not calibrated
            _feed_until_sensor=lambda ln, t: False)     # never reaches sensor
        # Bound to the real implementation: with no measurement it is a no-op,
        # which is the case that must not disturb the load path.
        shim._path_measurement = lambda: afcBambuAMS._path_measurement(shim)
        shim._adopt_measured_path = (
            lambda: afcBambuAMS._adopt_measured_path(shim))
        # Also real: _bridge here has no last_fault, so a quiet unit -- which
        # is what a shim that stubs _feed_until_sensor is standing in for.
        shim._fault_seen = 0
        shim._declared_fault_text = None
        shim._ams_fault_since = (
            lambda mark, consume=True:
                afcBambuAMS._ams_fault_since(shim, mark, consume))
        # Odometer + spool telemetry: a quiet unit reports neither, which is
        # the case that must leave the load path completely unchanged.
        shim._odom_now_mm = lambda: None
        shim._load_odom_start = None
        shim._load_odom_at_sensor = None
        shim._load_t0 = 0.0
        shim.ODOM_PATH_MIN_MM = afcBambuAMS.ODOM_PATH_MIN_MM
        shim.ODOM_PATH_MAX_MM = afcBambuAMS.ODOM_PATH_MAX_MM
        shim._measure_path_from_odom = (
            lambda: afcBambuAMS._measure_path_from_odom(shim))
        shim._dw_len_mm = lambda: None
        shim._path_measurement = lambda: afcBambuAMS._path_measurement(shim)
        shim._adopt_load_measured_remain = lambda ln: None
        lane = types.SimpleNamespace(name="lane1", loaded_to_hub=False)
        return shim, lane, calls

    def test_stalled_load_rehomes_then_fails(self):
        shim, lane, calls = self._shim(attempts=2)
        assert afcBambuAMS._unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 2                # one re-home per recover attempt
        assert len(calls["feed"]) == 3            # initial feed + 2 re-feeds
        assert calls["fail"] == 1                 # reported once, after exhausting

    def test_recover_disabled_fails_immediately(self):
        shim, lane, calls = self._shim(attempts=0)
        assert afcBambuAMS._unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 0               # no re-home when disabled
        assert len(calls["feed"]) == 1           # just the initial feed
        assert calls["fail"] == 1


class TestADeclaredFaultGetsOneRetryNotTwo:
    """The re-home IS the right answer to a latch -- mode 0F/0E is what the
    printer's Retry sends, and the capture shows it missing once then
    succeeding. It is not something to do twice.

    Measured on the HT, lane23. The fault break fired correctly, 7ms after
    "TIMEOUT error 0", and then the recovery opened two more full 101s windows
    at a unit that neither moved nor spoke again until both had expired:

        10:46:05  break on the fault           (5 kicks -- correct)
        10:46:08  recover 1/2, 26 kicks, 101s  (nothing)
        10:47:53  recover 2/2, 26 kicks, 101s  (nothing)
        10:49:34  failed

    3.5 minutes, of which the part that worked was the first six seconds.
    """

    def _shim(self, declared, odom=None):
        shim, lane, calls = TestLoadRecover._shim(attempts=2)

        # Both are filled from INSIDE the load, which is where the real ones are
        # filled -- the verdict by _feed_until_sensor -> _ams_declared_fault,
        # the odometer range by _track_odom off the status frames. Setting
        # either on the shim beforehand proves nothing: _unit_load_lane clears
        # both on entry so a previous load's evidence cannot describe this one.
        def feed_until(ln, t):
            if declared:
                shim._declared_fault_text = declared
            if odom is not None:
                shim._load_odom_lo, shim._load_odom_hi = odom
            return False
        shim._feed_until_sensor = feed_until
        shim._load_odom_lo = shim._load_odom_hi = None
        shim._load_odom_span_mm = (
            lambda: afcBambuAMS._load_odom_span_mm(shim))
        shim._jam_location = (
            lambda span=None: afcBambuAMS._jam_location(shim, span))
        shim.ODOM_MOVED_MM = afcBambuAMS.ODOM_MOVED_MM
        return shim, lane, calls

    def test_a_declared_fault_gets_one_rehome(self):
        shim, lane, calls = self._shim("[AMS_LED]TIMEOUT error 0")
        assert afcBambuAMS._unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 1
        assert len(calls["feed"]) == 2            # initial + one re-feed

    def test_a_silent_stall_still_gets_both(self):
        # No verdict means no evidence the unit has given up -- the second
        # Retry is exactly the case the recovery was built for.
        shim, lane, calls = self._shim(None)
        assert afcBambuAMS._unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 2

    def test_the_operator_is_told_what_the_unit_said(self):
        said = []
        shim, lane, calls = self._shim(
            "[AMS_COMMON]state:7,tray_now:0 [AMS_LED]TIMEOUT error 0")
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said
        assert "TIMEOUT error 0" in said[0]
        # ...and NOT sent to measure a bowden that had nothing to do with it.
        assert "afc_bowden_length" not in said[0]

    def test_a_real_timeout_still_blames_the_bowden(self):
        # The other failure is still the other failure: a load that simply ran
        # out of window has no verdict, and the calibration hint is the right
        # advice for it.
        said = []
        shim, lane, calls = self._shim(None)
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said and "afc_bowden_length" in said[0]

    def test_the_failure_says_where_from_the_odometer(self):
        # THE CASE THIS WAS BUILT FOR: the PTFE tube was not connected to the
        # HT, so the AMS fed five metres onto the floor and the toolhead sensor
        # never saw a thing. Every host-side signal said "nothing arrived"; only
        # the unit's odometer knew the filament had gone somewhere. It was in
        # the status frame throughout and we never read it, so the operator was
        # told to check their afc_bowden_length calibration.
        said = []
        shim, lane, calls = self._shim(None, odom=(0.0, 5004.0))
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said
        assert "DOWNSTREAM OF THE AMS" in said[0]
        assert "5004mm" in said[0]        # the number, not just the verdict

    def test_a_unit_that_never_moved_points_at_the_spool_instead(self):
        said = []
        shim, lane, calls = self._shim(None, odom=(1500.0, 1514.0))
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said and "AT THE AMS" in said[0]

    def test_no_odometer_readings_add_nothing(self):
        # Hedging beats guessing: a unit that went quiet gets no verdict, and
        # the message is the same one a reading-less unit always gets.
        said = []
        shim, lane, calls = self._shim(None)
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said
        assert "DOWNSTREAM" not in said[0] and "AT THE AMS" not in said[0]

    def test_the_location_rides_along_with_a_declared_fault_too(self):
        # The two answer different questions -- WHAT the unit said and WHERE
        # the filament got to -- so a failure with both should carry both.
        said = []
        shim, lane, calls = self._shim("[AMS_LED]TIMEOUT error 0",
                                       odom=(0.0, 5004.0))
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert said
        assert "TIMEOUT error 0" in said[0]
        assert "DOWNSTREAM OF THE AMS" in said[0]

    def test_the_verdict_does_not_outlive_its_load(self):
        # A verdict belongs to ONE load. Left set, the previous failure's words
        # would be attached to this failure -- and this one has none.
        shim, lane, calls = self._shim(None)
        shim._declared_fault_text = "[AMS_LED]TIMEOUT error 0"   # stale
        said = []
        shim.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS._unit_load_lane(shim, lane)
        assert shim._declared_fault_text is None
        assert said and "TIMEOUT" not in said[0]


# ── eject: clears stuck motion before rewinding, works from any state ───────────

def _eject_shim():
    order = []
    shim = types.SimpleNamespace(
        name="AMS", logger=_Logger(), _bridge=object(),
        afc_unload_bowden_length=500.0, eject_buffer=200.0,
        stop=lambda: order.append("stop"),
        select_lane=lambda lane: (order.append("select"), (True, 0))[1],
        retract=lambda lane, d: order.append(("retract", d)),
        # Returns True: the AMS normally reports its own completion, and the
        # timeout branch is a distinct case with its own test below.
        _wait_move=lambda d, s=None, fault_mark=None: (
            order.append(("wait", d)), True)[1],
        measured_path_mm=lambda: None,
        _measure_path_from_odom=lambda: None,
        _dw_len_mm=lambda: None,
        # Quiet unit by default; the latch case has its own test below.
        _ams_fault_seq=lambda: 0,
        _ams_fault_since=lambda mark, consume=True: None,
        _is_virtual_hub=lambda lane: True)
    # Bound to the real implementation rather than stubbed, so the distance
    # arithmetic under test is the shipped one.
    shim._eject_distance = lambda: afcBambuAMS._eject_distance(shim)
    return shim, order


class TestEjectLane:
    def test_stops_first_then_full_rewind(self):
        shim, order = _eject_shim()
        lane = types.SimpleNamespace(loaded_to_hub=True, _load_state=True)
        afcBambuAMS.eject_lane(shim, lane)
        assert order[0] == "stop"                        # clear stuck motion 1st
        assert order[-1] == "stop" or "stop" in order[1:]  # halt after too
        assert ("retract", 950.0) in order               # bowden+hub+buffer
        assert lane.loaded_to_hub is False
        assert lane._load_state is False

    def test_no_bridge_is_a_safe_noop(self):
        shim, order = _eject_shim()
        shim._bridge = None
        lane = types.SimpleNamespace(name="lane1", loaded_to_hub=True,
                                     _load_state=True)
        afcBambuAMS.eject_lane(shim, lane)               # no raise
        assert order == [] and lane.loaded_to_hub is True


class TestTheSpoolSummaryIsReadableByAHuman:
    """The three facts worth knowing -- what tag was read, how full it is, and
    whether that reached Spoolman -- were split across three machine-shaped
    lines seconds apart, each phrased for whoever wrote the code:

        applied tag to lane23: Bambu PLA Matte #A3D8E1
        measured spool in slot 0: 102% remaining (~1000 g) [capscan]
        wrote 1000 g remaining to Spoolman spool 87 (physical AMS measurement)

    One sentence instead, at the operator.
    """

    def _u(self, material="Bambu PLA Basic", colour="#0080ff", sid=87,
           sync=True):
        said = []
        log = types.SimpleNamespace(info=said.append)
        u = types.SimpleNamespace(
            name="BambuAMS_2", logger=log, sync_measured_to_spoolman=sync,
            _slots=[{"index": 2, "material": material, "color": colour,
                     "weight": 1000}])
        lane = types.SimpleNamespace(name="lane20", spool_id=sid)
        return u, lane, said

    def test_it_says_the_tag_the_amount_and_where_it_went(self):
        u, lane, said = self._u()
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert len(said) == 1
        m = said[0]
        assert "Bambu PLA Basic" in m and "#0080ff" in m
        assert "73% left" in m and "730 g" in m and "1000 g spool" in m
        assert "Spoolman spool 87" in m

    def test_a_spool_with_no_tag_says_so_plainly(self):
        # Third-party reels have no tag. Ordinary, so stated, not warned about.
        u, lane, said = self._u(material=None, colour=None)
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "no tag on this spool" in said[0]

    def test_an_unbound_spool_explains_where_the_number_went_instead(self):
        # The silence around this has already cost two rounds of "why didn't
        # it write?" -- the figure IS kept, on the lane.
        u, lane, said = self._u(sid=None)
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "not linked to a Spoolman spool" in said[0]
        assert "kept on the lane" in said[0]

    def test_a_read_tag_that_did_not_link_names_the_setting_that_would(self):
        # lane15 read PLA Sparkle / 04C07001 cleanly all day and never linked,
        # and the reason was one config line on the wrong unit's section. The
        # log said "not linked" -- a state, not a cause -- so the answer had to
        # be reconstructed from the outside. Say the cause and the fix.
        u, lane, said = self._u(sid=None)
        u._slots[0]["rfid_uid"] = "04c07001"
        u.auto_spoolman_create = False
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "04C07001" in said[0]
        assert "auto-create is off" in said[0]
        assert "auto_spoolman_create: True" in said[0]
        assert "BambuAMS_2" in said[0]          # names THIS unit's section

    def test_with_auto_create_on_it_does_not_blame_the_setting(self):
        u, lane, said = self._u(sid=None)
        u._slots[0]["rfid_uid"] = "04c07001"
        u.auto_spoolman_create = True
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "auto_spoolman_create" not in said[0]
        assert "no spool carrying 04C07001 yet" in said[0]

    def test_an_undecodable_tag_is_told_to_bind_by_hand(self):
        # No profile to create a spool FROM, so the setting is irrelevant here
        # and naming it would send the operator to the wrong lever.
        u, lane, said = self._u(material=None, colour=None, sid=None)
        u._slots[0]["rfid_uid"] = "84ea7601"
        u.auto_spoolman_create = False
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "bind 84EA7601 to a spool in Spoolman" in said[0]
        assert "auto_spoolman_create" not in said[0]

    def test_sync_turned_off_is_distinguished_from_unbound(self):
        u, lane, said = self._u(sync=False)
        afcBambuAMS._say_spool_summary(u, 2, lane, 54, 540, 1000)
        assert "Spoolman sync is off" in said[0]

    def test_over_100_percent_reads_as_full_not_as_a_number_to_discount(self):
        # 102/107/113/119 all captured on real hardware: a spool sitting proud
        # of the reference radius, not extra filament. The operator should not
        # have to know how to interpret that.
        u, lane, said = self._u()
        afcBambuAMS._say_spool_summary(u, 2, lane, 113, 1000, 1000)
        m = said[0]
        assert "full" in m and "1000 g of a 1000 g spool" in m
        assert "113%" in m               # still stated, just explained

    def test_it_names_the_lane_the_operator_knows(self):
        u, lane, said = self._u()
        afcBambuAMS._say_spool_summary(u, 2, lane, 73, 730, 1000)
        assert "lane20" in said[0]

    def test_no_lane_falls_back_to_the_bay(self):
        u, _lane, said = self._u()
        afcBambuAMS._say_spool_summary(u, 2, None, 73, 730, 1000)
        assert "bay 2" in said[0]


class TestTheSummaryWaitsForTheRecordItDescribes:
    """
    The measurement finishes BEFORE the record it describes catches up, by
    design: the firmware does not read 0x0211 during the capacity window, so it
    clears info_valid at the window close and lets the fill collect the result
    after. The measured percent arrives from narration, which is why it is
    here first.

    Said immediately, the line read the record in exactly the gap it is being
    refreshed through and announced the blank as a conclusion. Captured live:

        13:15  STEP:card auth success! ... read success,valid
        13:15  slot 1 calibration completed -- re-reading the bay
        13:15  lane16: NO TAG ON THIS SPOOL. Measured about 25% left
        13:16  applied tag to lane16: Bambu PLA Sparkle #2D2B28

    The unit had just narrated a successful read. Nothing was wrong except the
    moment we chose to speak.
    """

    def _u(self, material=None, uid=None, now=100.0, verdict="none"):
        said = []
        u = types.SimpleNamespace(
            name="BambuAMS_1", logger=types.SimpleNamespace(info=said.append),
            sync_measured_to_spoolman=True, SCAN_FALLBACK_CAP=45.0,
            _scan_verdict=lambda s: verdict,
            _slots=[{"index": 0, "material": material, "color": "#2D2B28",
                     "rfid_uid": uid, "weight": 1000}],
            _lane_for_slot=lambda s: types.SimpleNamespace(
                name="lane16", spool_id=None),
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: now)))
        u._say_spool_summary = afcBambuAMS._say_spool_summary.__get__(u)
        u._drain_spool_summary = afcBambuAMS._drain_spool_summary.__get__(u)
        u._queue_spool_summary = afcBambuAMS._queue_spool_summary.__get__(u)
        return u, said

    def test_a_blank_record_holds_the_line(self):
        u, said = self._u(material=None)
        u._queue_spool_summary(0, 25, 250, 1000)
        assert said == [], "announced a conclusion mid-refresh"
        assert 0 in u._pending_summary

    def test_it_speaks_once_the_record_lands(self):
        u, said = self._u(material=None)
        u._queue_spool_summary(0, 25, 250, 1000)
        u._slots[0]["material"] = "Bambu PLA Sparkle"      # the re-read landed
        u._drain_spool_summary(0)
        assert len(said) == 1
        assert "Bambu PLA Sparkle" in said[0] and "25% left" in said[0]
        assert "no tag" not in said[0]
        assert 0 not in u._pending_summary

    def test_a_record_that_already_answers_is_not_held(self):
        u, said = self._u(material="Bambu PLA Sparkle")
        u._queue_spool_summary(0, 25, 250, 1000)
        assert len(said) == 1

    def test_a_uid_alone_answers_it(self):
        # A third-party tag has a UID even when its profile will not decode --
        # that is an answer, not a blank, so there is nothing to wait for.
        u, said = self._u(material=None, uid="84ea7601")
        u._queue_spool_summary(0, 25, 250, 1000)
        assert len(said) == 1 and "84EA7601" in said[0]

    def test_a_uid_alone_is_not_an_answer_while_the_scan_is_still_running(self):
        # The two halves of a record do not land together. On an HT the UID
        # comes off the anticollision well before the profile is fetched, so
        # this used to fire mid-scan and print the opposite of the truth:
        #
        #   16:01:58  tag 0A1882AC read but its profile could not be decoded
        #   16:02:06  applied tag to lane23: Bambu PLA Glow #A1FFAC
        u, said = self._u(material=None, uid="0a1882ac", verdict="waiting")
        u._queue_spool_summary(0, 74, 740, 1000)
        assert said == [], "called a tag undecodable while it was still being read"
        assert 0 in u._pending_summary

    def test_and_it_speaks_when_the_scan_resolves(self):
        u, said = self._u(material=None, uid="0a1882ac", verdict="waiting")
        u._queue_spool_summary(0, 74, 740, 1000)
        u._slots[0]["material"] = "Bambu PLA Glow"       # the read landed
        u._scan_verdict = lambda s: "read"
        u._drain_spool_summary(0)
        assert len(said) == 1
        assert "Bambu PLA Glow" in said[0]
        assert "could not be decoded" not in said[0]

    def test_a_uid_only_bay_whose_scan_ENDED_still_answers_at_once(self):
        # The hold is on the window, not on the UID: a resolved no-tag/foreign
        # verdict means the profile is never coming, and a third-party reel
        # must not wait out the backstop for an answer it already has.
        u, said = self._u(material=None, uid="84ea7601", verdict="notag")
        u._queue_spool_summary(0, 25, 250, 1000)
        assert len(said) == 1 and "84EA7601" in said[0]

    def test_a_stand_in_without_the_verdict_hook_keeps_the_old_behaviour(self):
        # _drain_spool_summary is exercised against duck-typed stand-ins; one
        # missing the hook must fall back, not raise.
        u, said = self._u(material=None, uid="84ea7601")
        del u._scan_verdict
        u._queue_spool_summary(0, 25, 250, 1000)
        assert len(said) == 1

    def test_the_backstop_stops_it_waiting_forever(self):
        u, said = self._u(material=None, now=100.0)
        u._queue_spool_summary(0, 25, 250, 1000)
        assert said == []
        u.afc.reactor.monotonic = lambda: 100.0 + 46.0
        u._drain_spool_summary(0)
        assert len(said) == 1
        assert 0 not in u._pending_summary


class TestEachUnitMeasuresItsOwnTubeItsOwnWay:
    """THREE UNITS, THREE TUBES, THREE DIALECTS -- and no single source covers
    them. Measured on this rig 2026-08-07:

        unit    bowden   odometer          dw_len          tube_len
        AMS 1   3000     3338 (adopted)    --              never
        AMS 2   3497     intermittent      3504 observed   never
        HT      3679     ALWAYS None       3672 (x2)       never

    The HT is the case that forces the chain: it reported odom=None on 225
    consecutive samples across a full load and has never narrated tube_len
    here, so without dw_len it can never measure its own path at all.
    """

    def _u(self, odom=None, dw=None, tube=None, addr=0x0700, dw_addr=0x0700):
        u = types.SimpleNamespace(
            ODOM_PATH_MIN_MM=afcBambuAMS.ODOM_PATH_MIN_MM,
            ODOM_PATH_MAX_MM=afcBambuAMS.ODOM_PATH_MAX_MM,
            ams_index=0, dry_dev_addr=addr,
            _load_odom_start=0.0 if odom is not None else None,
            _load_odom_at_sensor=odom,
            measured_path_mm=lambda: tube,
            _bridge=types.SimpleNamespace(
                dw_len=lambda idx: (dw, 1 if dw else 0, dw_addr)))
        u._measure_path_from_odom = (
            lambda: afcBambuAMS._measure_path_from_odom(u))
        u._dw_len_mm = lambda: afcBambuAMS._dw_len_mm(u)
        return u

    def test_ams1_measures_by_odometer(self):
        assert afcBambuAMS._path_measurement(
            self._u(odom=3338.0)) == (3338.0, "odometer")

    def test_the_ht_falls_through_to_dw_len(self):
        # No odometer, ever. Without this the HT has no path measurement.
        u = self._u(odom=None, dw=3672.0, addr=0x1800, dw_addr=0x1800)
        assert afcBambuAMS._path_measurement(u) == (3672.0, "dw_len")

    def test_tube_len_is_still_the_last_resort(self):
        u = self._u(odom=None, dw=None, tube=2186.0)
        assert afcBambuAMS._path_measurement(u) == (2186.0, "tube_len")

    def test_a_unit_with_no_source_measures_nothing(self):
        assert afcBambuAMS._path_measurement(self._u()) == (None, "")

    def test_the_odometer_outranks_dw_len_when_both_exist(self):
        # The odometer is the distance actually travelled to the sensor THIS
        # load; dw_len is the unit's own figure for the same journey.
        u = self._u(odom=3338.0, dw=3504.0)
        assert afcBambuAMS._path_measurement(u)[1] == "odometer"

    def test_a_dw_len_from_another_units_device_is_refused(self):
        # dw_len is filed under _active_unit, which a load SETS and nothing
        # clears -- so it names whichever unit loaded LAST. An HT value must
        # not be adopted as a boxed unit's tube.
        u = self._u(odom=None, dw=3672.0, addr=0x0700, dw_addr=0x1800)
        assert afcBambuAMS._path_measurement(u) == (None, "")

    def test_an_absurd_dw_len_is_refused(self):
        # 0.000 is what a load reported with the PTFE tube disconnected.
        for bad in (0.0, 50.0, 99000.0):
            u = self._u(odom=None, dw=bad)
            assert afcBambuAMS._path_measurement(u) == (None, ""), bad

    def test_every_real_tube_on_this_rig_survives_the_chain(self):
        for mm in (3338.0, 3504.0, 3672.0):
            assert afcBambuAMS._path_measurement(
                self._u(odom=None, dw=mm))[0] == mm


class TestThePathIsMeasuredFromTheOdometer:
    """The odometer at the instant the toolhead sensor trips IS the bay-to-
    sensor distance, and it is a TYPED STATUS FIELD -- so it works on all three
    units, unlike tube_len/dw_len which are text the AMS 1 does not use.

    Traced on an AMS 1 (lane15, alone on the wire, 1 Hz):

        17:44:24  odom 0.0      unload finished, odometer zeroed
        17:44:48  odom 3.346    TOOLHEAD SENSOR TRIPS
        17:45:19  odom 3.469    tool_stn + purge, pulled by the extruder

    3346 mm, against a configured 3000 default.
    """

    def _shim(self, start, at_sensor):
        return types.SimpleNamespace(
            _load_odom_start=start, _load_odom_at_sensor=at_sensor,
            ODOM_PATH_MIN_MM=afcBambuAMS.ODOM_PATH_MIN_MM,
            ODOM_PATH_MAX_MM=afcBambuAMS.ODOM_PATH_MAX_MM)

    def test_the_traced_load_measures_its_own_tube(self):
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, 3346.0)) == 3346.0

    def test_it_is_a_delta_not_the_raw_reading(self):
        # A lane staged at the hub starts partway along. Reading the raw value
        # would record a path shorter than the real one.
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(800.0, 4146.0)) == 3346.0

    def test_the_post_arrival_creep_is_excluded_by_construction(self):
        # 3.469 is what the odometer reads 30s later, and 123mm of that is
        # tool_stn plus the purge -- real filament the toolhead consumed, which
        # the odometer rightly counts. Taking the reading at the trip is what
        # keeps it out; there is no subtraction anywhere to get wrong.
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, 3346.0)) == 3346.0

    def test_a_unit_that_reports_no_odometer_measures_nothing(self):
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(None, 3346.0)) is None
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, None)) is None

    def test_absurd_spans_are_rejected(self):
        # A sign flip or a stuck sentinel, not a tube.
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, -3346.0)) is None
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, 99000.0)) is None
        assert afcBambuAMS._measure_path_from_odom(
            self._shim(0.0, 50.0)) is None

    def test_every_real_tube_on_record_is_inside_the_bounds(self):
        # AMS 1 3346, AMS 2 ~3500, HT ~3660 on this rig; 1693 and 2186 on the
        # capture rig. The guard must not reject a real machine.
        for mm in (1693.0, 2186.0, 3346.0, 3497.0, 3679.0):
            assert afcBambuAMS._measure_path_from_odom(
                self._shim(0.0, mm)) == mm


class TestEjectReportsALatch:
    """eject is what AFC_BAMBU_RECOVER and AFC_RESET run, and it holds
    _unload_in_progress for its whole duration -- which MUTES both fault
    detectors by design.

    So when the unit gave up, nothing inside this path was reading its verdict.
    It sat in the bridge until the mute lifted in the `finally` and the next
    follower tick happened to pick it up: measured 22s late on an AMS 2, and
    lost outright if anything else consumed the sequence first. Recovery told
    the operator "done" while the filament was still out of the bay.
    """

    def _rig(self, fault=None):
        shim, order = _eject_shim()
        shim._ams_fault_seq = lambda: 0
        shim._ams_fault_since = \
            lambda mark, consume=True: fault
        return shim, order

    def _lane(self):
        return types.SimpleNamespace(name="lane20", loaded_to_hub=True,
                                     _load_state=True)

    def test_a_latch_is_said_in_the_units_own_words(self):
        shim, order = self._rig(
            fault="[AMS_COMMON]en:1,mode:3 [AMS_LED]TIMEOUT error 2")
        afcBambuAMS.eject_lane(shim, self._lane())
        said = " ".join(m for _lvl, m in shim.logger.messages)
        assert "TIMEOUT error 2" in said
        assert "latched" in said
        assert "free it by hand" in said.lower()

    def test_the_ams1_state_only_verdict_lands_here_too(self):
        shim, order = self._rig(
            fault="[AMS_COMMON]state:6,tray_now:255,tray_exit:6 "
                  "[AMS_LINK]en:0,mode:7,idx:255,ref:0")
        afcBambuAMS.eject_lane(shim, self._lane())
        said = " ".join(m for _lvl, m in shim.logger.messages)
        assert "en:0,mode:7,idx:255" in said
        assert "state:6" in said

    def test_a_quiet_eject_says_nothing(self):
        shim, order = self._rig(fault=None)
        afcBambuAMS.eject_lane(shim, self._lane())
        assert shim.logger.messages == []

    def test_the_wait_is_given_the_mark_so_it_can_end_early(self):
        # Without it the wait sits out its full deadline at a unit that has
        # already stopped listening -- the 22s that made the fault look like
        # it arrived late when it had been available the whole time.
        seen = {}
        shim, order = self._rig(fault=None)
        shim._wait_move = lambda d, s=None, fault_mark=None: (
            seen.update(mark=fault_mark), True)[1]
        afcBambuAMS.eject_lane(shim, self._lane())
        assert "mark" in seen and seen["mark"] is not None


# ── disconnect / FIRMWARE_RESTART teardown ──────────────────────────────────────

import extras.AFC_BambuAMS as _mod   # noqa: E402
import extras.AFC_BambuAMS as afcBambuAMS_mod   # noqa: E402


class TestHandleDisconnect:
    def test_tears_down_shared_bridge(self):
        stopped = []
        bridge = types.SimpleNamespace(stop=lambda: stopped.append(True))
        _mod._BRIDGES["/dev/ttyBAMBU"] = bridge
        shim = types.SimpleNamespace(
            serial_port="/dev/ttyBAMBU", SLOTS_PER_UNIT=4, _bridge=bridge,
            _slots=[{"present": True}] * 4, _prev_present=[True] * 4,
            _auto_scanned=[True] * 4)
        _mod.afcBambuAMS._handle_disconnect(shim)
        assert stopped == [True]                          # reader thread stopped
        assert "/dev/ttyBAMBU" not in _mod._BRIDGES       # cache cleared
        assert shim._bridge is None
        assert shim._prev_present == [False] * 4          # caches reset
        assert shim._auto_scanned == [False] * 4

    def test_second_unit_disconnect_is_idempotent(self):
        # daisy-chained units share one bridge; the 2nd teardown must not raise
        _mod._BRIDGES.pop("/dev/ttyGONE", None)
        shim = types.SimpleNamespace(
            serial_port="/dev/ttyGONE", SLOTS_PER_UNIT=4, _bridge=object(),
            _slots=[{}], _prev_present=[True], _auto_scanned=[True])
        _mod.afcBambuAMS._handle_disconnect(shim)         # no raise
        assert shim._bridge is None


# ── multi-AMS: shared bridge fan-out ────────────────────────────────────────────

class TestBridgeFanout:
    def test_all_listeners_get_status(self):
        bridge, reactor, logger, _seen = _bridge()      # already has 1 listener
        a, b = [], []
        bridge.add_listener(lambda o: a.append(o))
        bridge.add_listener(lambda o: b.append(o))
        frame = {"evt": "status", "online": True, "slots": []}
        bridge.handle_line(json.dumps(frame))
        reactor.run_pending()
        assert a == [frame] and b == [frame]             # every listener fired


# ── multi-AMS: per-unit filtering (duck-typed self) ─────────────────────────────

def _unit_shim(ams_index, slots=None):
    """Duck-typed self for afcBambuAMS._on_status / _unit_online."""
    return types.SimpleNamespace(
        ams_index=ams_index,
        SLOTS_PER_UNIT=4,
        _slots=slots if slots is not None else [{} for _ in range(4)],
        _sync_lanes=lambda: None,
        logger=_Logger())


class TestUnitOnline:
    def test_reads_matching_unit(self):
        latest = {"units": [{"n": 0, "online": False},
                            {"n": 1, "online": True}]}
        assert afcBambuAMS._unit_online(_unit_shim(1), latest) is True
        assert afcBambuAMS._unit_online(_unit_shim(0), latest) is False

    def test_none_is_offline(self):
        assert afcBambuAMS._unit_online(_unit_shim(0), None) is False

    def test_single_unit_fallback(self):
        # no 'units' list -> fall back to the top-level online flag
        assert afcBambuAMS._unit_online(_unit_shim(0), {"online": True}) is True


class TestUnitEnv:
    def test_humidity_and_unknown_temp(self):
        latest = {"units": [{"n": 0, "humidity": 42, "temp": -1},
                            {"n": 1, "humidity": 55, "temp": 285}]}
        assert unit_env(latest, 0) == (42, None)     # temp -1 -> unknown
        assert unit_env(latest, 1) == (55, 28.5)     # temp x10 -> 28.5 C

    def test_missing_and_none(self):
        assert unit_env(None, 0) == (None, None)
        assert unit_env({"units": [{"n": 0, "humidity": -1}]}, 0) == (None, None)
        assert unit_env({"units": []}, 0) == (None, None)


class TestOnStatusUnitFilter:
    def test_keeps_only_our_units_slots(self):
        shim = _unit_shim(1)
        frame = {"slots": [
            {"unit": 0, "i": 0, "present": True, "material": "PLA"},
            {"unit": 1, "i": 0, "present": True, "material": "PETG"},
            {"unit": 1, "i": 2, "present": False},
        ]}
        afcBambuAMS._on_status(shim, frame)
        assert shim._slots[0].get("material") == "PETG"   # unit 1 slot 0 kept
        assert shim._slots[2].get("present") is False      # unit 1 slot 2 kept
        assert shim._slots[1] == {}                        # unit 0 slot ignored

    def test_untagged_slot_defaults_unit_0(self):
        shim = _unit_shim(0)
        afcBambuAMS._on_status(shim, {"slots": [
            {"i": 0, "present": True, "material": "PLA"}]})
        assert shim._slots[0].get("material") == "PLA"


# ── base-ACE-style profile surfacing (no UID) ───────────────────────────────────

def _bare_lane():
    return types.SimpleNamespace(name="lane1", material=None, color=None,
                                 extruder_temp=None)


def _surface_self(saves=None):
    # _surface_slot_info gained two collaborators after these tests were
    # written: it pushes the tag onward to Spoolman, and it applies the
    # measured remain% to the lane's weight. Both are no-ops here -- this
    # fixture is about what lands ON THE LANE -- but they have to exist.
    return types.SimpleNamespace(
        name="AMS", logger=_Logger(), afc=None,
        _spoolman_sync=lambda lane, info: None,
        _apply_remain_weight=lambda lane, info: None,
        _save_lane_vars=lambda: (saves.append(1) if saves is not None else None))


class TestSurfaceSlotInfo:
    def test_applies_profile_to_bare_lane(self):
        lane = _bare_lane()
        info = bridge_slot_to_info({
            "i": 0, "present": True, "material": "PLA", "sku": "GFA00",
            "color": "00ae42ff", "tmin": 210, "tmax": 230})
        afcBambuAMS._surface_slot_info(_surface_self(), lane, info)
        assert lane.material == "PLA"
        assert lane.color == "#00AE42"
        assert lane.bambu_sku == "GFA00"
        assert lane.extruder_temp == 210.0
        assert lane.bambu_slot_info is info

    def test_tag_overrides_previous_value(self):
        # the AMS tag is authoritative for the bay -- it wins over a value
        # auto-set elsewhere (e.g. an AFC default).
        lane = types.SimpleNamespace(name="lane1", material="ABS",
                                     color="#FF0000", extruder_temp=250.0)
        info = bridge_slot_to_info({
            "i": 0, "present": True, "material": "PLA", "sku": "GFA00",
            "color": "00ae42ff", "tmin": 210, "tmax": 230})
        afcBambuAMS._surface_slot_info(_surface_self(), lane, info)
        assert lane.material == "PLA"                # tag wins
        assert lane.color == "#00AE42"
        assert lane.extruder_temp == 210.0

    def test_a_spoolman_link_does_not_block_the_tag(self):
        # A bound lane takes the tag like any other. Spoolman is a RECORD of
        # what is in the bay; the tag IS the bay -- and every other AFC reader
        # (OpenAMS, ACE 2, U1, Vivid) applies the tag first and syncs after.
        #
        # Gating on the link would leave a bound lane's material and colour
        # coming from Spoolman on Spoolman's schedule and never from the tag,
        # so a scan on that lane would never reach Mainsail at all.
        lane = types.SimpleNamespace(name="lane1", material="ABS",
                                     color="#FF0000", extruder_temp=250.0,
                                     spool_id=42)
        info = bridge_slot_to_info({
            "i": 0, "present": True, "material": "PLA", "color": "00ae42ff",
            "tmin": 210, "tmax": 230})
        afcBambuAMS._surface_slot_info(_surface_self(), lane, info)
        assert lane.material == "PLA"
        assert lane.color == "#00AE42"
        assert lane.spool_id == 42                   # the link itself stands

    def test_unknown_material_not_applied(self):
        lane = _bare_lane()
        afcBambuAMS._surface_slot_info(_surface_self(), lane, bridge_slot_to_info(
            {"i": 0, "present": True, "material": "Unknown"}))
        assert lane.material is None


class TestBridgeFinish:
    """The captured mode 09->07 handoff that commits a load, sent verbatim."""

    def _shim(self, bridge=True):
        sent = []
        shim = types.SimpleNamespace(
            _bridge=types.SimpleNamespace(send=lambda o: sent.append(o))
            if bridge else None,
            _FINISH_FRAMES=afcBambuAMS._FINISH_FRAMES)
        return shim, sent

    def test_no_bridge_sends_nothing(self):
        shim, sent = self._shim(bridge=False)
        assert afcBambuAMS.bridge_finish(shim) is False
        assert sent == []

    def test_sends_every_captured_frame_in_order(self):
        shim, sent = self._shim()
        assert afcBambuAMS.bridge_finish(shim) is True
        # The AMS walks 09 -> 07 gate -> 07 finish; order is the protocol, not
        # an implementation detail, so pin the exact sequence.
        assert sent == [
            {"cmd": "raw", "hex": "3DC50CC803000900A502800C"},
            {"cmd": "raw", "hex": "3DC50CC8030007000002514C"},
            {"cmd": "raw", "hex": "3DC50CC8030007007F023654"},
        ]

    def test_frames_are_defined_on_the_class(self):
        # Regression: bridge_finish reads _FINISH_FRAMES off self, so deleting
        # the tuple raised AttributeError inside the follower's reactor timer
        # and took Klipper down rather than failing any test.
        assert isinstance(afcBambuAMS._FINISH_FRAMES, tuple)
        assert len(afcBambuAMS._FINISH_FRAMES) == 3


class TestEngageFollowerOrder:
    """finish -> select -> assist. select must be the LAST mode change, or the
    tray drops out of mode:4 and the follower never runs."""

    def test_calls_run_in_protocol_order(self):
        calls = []
        lane = types.SimpleNamespace(name="lane1")
        shim = types.SimpleNamespace(
            bridge_finish=lambda ln: calls.append(("finish", ln.name)),
            select_lane=lambda ln: calls.append(("select", ln.name)),
            set_feed_assist=lambda ln, on: calls.append(("assist", ln.name, on)))
        afcBambuAMS._engage_follower(shim, lane)
        assert calls == [
            ("finish", "lane1"), ("select", "lane1"), ("assist", "lane1", True)]

    def test_real_bridge_finish_reaches_the_bridge(self):
        # Exercises _engage_follower against the REAL bridge_finish instead of a
        # stub -- the combination the existing follower tests all mock away,
        # which is how a missing _FINISH_FRAMES reached a printer.
        sent = []
        lane = types.SimpleNamespace(name="lane1")
        shim = types.SimpleNamespace(
            _bridge=types.SimpleNamespace(send=lambda o: sent.append(o)),
            _FINISH_FRAMES=afcBambuAMS._FINISH_FRAMES,
            bridge_finish=lambda ln: afcBambuAMS.bridge_finish(shim, ln),
            select_lane=lambda ln: None,
            set_feed_assist=lambda ln, on: None)
        afcBambuAMS._engage_follower(shim, lane)
        assert [o["hex"] for o in sent] == list(afcBambuAMS._FINISH_FRAMES)


class TestFollowManualOff:
    """AFC_BAMBU_FOLLOWER ENABLE=0 must survive the auto-arm, which otherwise
    re-engages on the next ~100ms tick and makes the stop look like a no-op."""

    def _shim(self, lane, manual_off):
        state = {"engaged": [], "assist": []}
        toolhead = types.SimpleNamespace(get_position=lambda: [0, 0, 0, 0.0])
        shim = types.SimpleNamespace(
            _following_lane=None, _follow_last_e=None, follow_min_extrude=0.05,
            _follow_manual_off=manual_off, _unload_in_progress=False,
            _bridge=types.SimpleNamespace(
                send=lambda o: None,
                latest_status=lambda: {"fstate": 4, "buff": 50}),
            follow_when_loaded=True,
            follow_debug_interval=0.0, _follow_last_log=0.0,
            follow_poll_interval=0.1, ams_index=0,
            _tool_loaded_lane=lambda: lane,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            _engage_follower=lambda ln: (state["engaged"].append(ln.name),
                                         setattr(shim, "_following_lane", ln)),
            set_feed_assist=lambda ln, on: state["assist"].append((ln.name, on)),
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_fault_checks_run_with_the_follower_manually_off(self):
        # AFC_BAMBU_FOLLOWER ENABLE=0 clears _following_lane. Fault detection used
        # to hang off that, so stopping the follower silently stopped stall
        # detection -- in the state most likely to starve the buffer.
        lane = types.SimpleNamespace(name="lane1")
        checked = []
        shim, state = self._shim(lane, manual_off=True)
        shim._check_ams_fault = lambda ln: checked.append(ln.name)
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["engaged"] == []           # still does not re-arm
        # No "buff": the buffer watchdog is gone. What remains is what the AMS
        # itself says -- the narration check here, and byte[19] alongside it.
        assert checked == ["lane1"]             # but still watches for a stall

    def test_latched_off_blocks_the_auto_arm(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(lane, manual_off=True)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["engaged"] == []
        assert shim._following_lane is None

    def test_not_latched_still_auto_arms(self):
        # Same shim, latch clear -> the auto-arm must still run, so the test
        # above is showing the latch working rather than a broken shim.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(lane, manual_off=False)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["engaged"] == ["lane1"]
        assert shim._following_lane is lane


class TestFollowIdlePingIsGone:
    """
    The follower holds whenever a lane is loaded, so no option gates it.

    Guards against follow_idle_ping being reintroduced as a config read: an
    option nothing consults reads as a knob that does something.
    """

    def test_the_option_is_not_read(self):
        import inspect
        from extras import AFC_BambuAMS as m
        assert "follow_idle_ping" not in inspect.getsource(m)

class TestAmsNarrationRepeat:
    """Identical AMS narration is de-duplicated, but a repeating line is how a
    stuck loop looks -- it must not be silently swallowed forever."""

    def _bridge_with_clock(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        return bridge, reactor, logger

    def _emit(self, bridge, text):
        bridge.handle_line('{"evt":"amsdbg","text":"%s"}' % text)

    def test_first_line_logged(self):
        bridge, reactor, logger = self._bridge_with_clock()
        self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert logger.messages == [
            ("debug", "AMS: [AMS_RFID] STEP3,feed with rfid fail!")]

    def test_immediate_repeat_suppressed(self):
        bridge, reactor, logger = self._bridge_with_clock()
        self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert len(logger.messages) == 1     # second one deduped

    def test_repeat_resurfaces_with_a_count(self):
        # A loop stuck for a minute must reappear, or the fault reads as
        # silence -- which is exactly how a stalled RFID retry hid for hours.
        bridge, reactor, logger = self._bridge_with_clock()
        self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        for _ in range(5):
            self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        reactor.advance(61.0)
        self._emit(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert any("repeated" in m for _, m in logger.messages), logger.messages

    def test_a_different_line_resets_the_counter(self):
        bridge, reactor, logger = self._bridge_with_clock()
        self._emit(bridge, "[AMS_PMSM]mode:0->2")
        self._emit(bridge, "[AMS_PMSM]mode:2->0")
        assert [m for _, m in logger.messages] == [
            "AMS: [AMS_PMSM]mode:0->2", "AMS: [AMS_PMSM]mode:2->0"]


class TestAmsFinishTracking:
    """The AMS reports its own motion completions; the bridge records them so a
    move can be waited on rather than timed."""

    def test_feed_finish_marks_success(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        bridge.handle_line(
            '{"evt":"amsdbg","text":"[AMS_SWITCH]feed finish,buff_pos:1.36"}')
        seq, ok, text = bridge.last_finish()
        assert seq == 1 and ok is True and "feed finish" in text

    def test_stall_marks_failure(self):
        # "-1" and "stall" are the AMS saying the move did NOT do what it was
        # asked -- the case a duration-based wait cannot detect at all.
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        bridge.handle_line('{"evt":"amsdbg","text":'
                           '"[AMS_SWITCH]feed finish -1, stall, len_det:1.156 m"}')
        seq, ok, _ = bridge.last_finish()
        assert seq == 1 and ok is False

    def test_sequence_increments_so_waiters_see_a_fresh_event(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        bridge.handle_line(
            '{"evt":"amsdbg","text":"[AMS_PRELOAD]preload finish, sw_sta:1"}')
        first = bridge.last_finish()[0]
        bridge.handle_line(
            '{"evt":"amsdbg","text":"[AMS_SWITCH]feed finish,buff_pos:1.2"}')
        assert bridge.last_finish()[0] == first + 1

    def test_non_finish_narration_leaves_it_alone(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        bridge.handle_line('{"evt":"amsdbg","text":"[AMS_PMSM]mode:0->2"}')
        assert bridge.last_finish() == (0, False, "")


class TestAmsFaultCapture:
    """The AMS names its own stalls; the bridge records them."""

    def _b(self):
        bridge, reactor, logger, seen = _bridge()
        bridge.reactor = reactor
        return bridge

    def test_stall_is_captured(self):
        b = self._b()
        b.handle_line('{"evt":"amsdbg","text":'
                      '"[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m"}')
        seq, text, amps = b.last_fault()
        assert seq == 1 and "stall" in text

    def test_rocker_and_bdc_stalls_are_captured(self):
        for phrase in ("[AMS_SWITCH]switch_feed rocker stall, tray_cnt:0,0,",
                       "[AMS_SWITCH]pull err, bdc stall, mode:1, tray_sw"):
            b = self._b()
            b.handle_line('{"evt":"amsdbg","text":"%s"}' % phrase)
            assert b.last_fault()[0] == 1, phrase

    def test_motor_current_is_parsed(self):
        b = self._b()
        b.handle_line('{"evt":"amsdbg","text":'
                      '"[AMS_SWITCH]feed to dw ok, len_det:0.050 m, bldc_i:1.600A"}')
        assert b.last_fault()[2] == 1.6

    def test_ams2_timeout_error_is_captured(self):
        # A boxed AMS 2 never says "stall". It reports a jam as
        # "[AMS_LED]TIMEOUT error N", which the bridge filter did not match --
        # so a held stuck spool was only ever caught by the slower buffer
        # inference, five seconds later. Verbatim from the live capture.
        b = self._b()
        b.handle_line('{"evt":"amsdbg","text":'
                      '"[AMS_LED]TIMEOUT error 2 [AMS_LED]TIMEOUT error 3 '
                      '[AMS_LED]TRAY 3 in five [AMS_LINK]err_code: 0 -> 23"}')
        seq, text, _amps = b.last_fault()
        assert seq == 1 and "TIMEOUT error" in text

    def test_assist_err_is_not_a_fault(self):
        # assist_err cycles 0->65536->0 around every SUCCESSFUL feed. Treating
        # it as a fault would pause a healthy print.
        b = self._b()
        b.handle_line('{"evt":"amsdbg","text":"[AMS_LINK]assist_err: 0 -> 65536"}')
        assert b.last_fault()[0] == 0

    def test_err_code_is_not_a_fault(self):
        # err_code 0x16 appeared 292 times in normal operation.
        b = self._b()
        b.handle_line('{"evt":"amsdbg","text":"[AMS_LINK]err_code:0x00->0x16"}')
        assert b.last_fault()[0] == 0


class TestAmsFaultRaising:
    """_check_ams_fault raises once per stall, and only while genuinely
    feeding -- a scan or an unload must not pause a print."""

    def _shim(self, fault_text, *, unloading=False, drying=False,
              detect=True, pause=True, seen=0):
        raised, warned, assist, recovered = [], [], [], []
        shim = types.SimpleNamespace(
            name="BambuAMS_2", fault_detect=detect, fault_pause=pause,
            _fault_seen=seen, _unload_in_progress=unloading, _drying=drying,
            _follow_fault_hold=False, _follow_fault_saw_pause=False,
            _starved_since=5.0, _odom_lo=None, _odom_hi=None,
            _bridge=types.SimpleNamespace(
                last_fault=lambda: (1, fault_text, 1.6)),
            set_feed_assist=lambda ln, on: assist.append((ln.name, on)),
            logger=types.SimpleNamespace(warning=lambda m: warned.append(m),
                                         debug=lambda m: None),
            # _raise_ams_fault only asks for a PAUSE when a print is actually
            # running -- added after this test, because pausing outside a print
            # runs a Z move that raises "Must home axis first", and an escaped
            # exception in the follower's reactor timer shuts down every MCU.
            # Without this the shim fell to the "not printing" branch and the
            # test read a correct no-pause as a failure.
            #
            # And once it IS printing, the fault hands off to auto recovery.
            # These tests are about RAISING, so record the handoff and stop
            # there -- the recovery has its own tests.
            _resume_needs_reload=False,
            _maybe_auto_recover=lambda ln: recovered.append(ln.name),
            afc=types.SimpleNamespace(
                function=types.SimpleNamespace(in_print=lambda: True),
                error=types.SimpleNamespace(
                    AFC_error=lambda m, pause=True: raised.append((m, pause)))))
        # The real raiser, so the hold latch is exercised rather than stubbed.
        shim._raise_ams_fault = (
            lambda ln, m: afcBambuAMS._raise_ams_fault(shim, ln, m))
        return shim, raised, warned, assist

    def test_stall_raises_and_pauses(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, _a = self._shim("[AMS_SWITCH]feed finish -1, stall")
        afcBambuAMS._check_ams_fault(shim, lane)
        assert len(raised) == 1
        msg, pause = raised[0]
        assert pause is True and "lane22" in msg and "1.60A" in msg

    def test_same_fault_raises_once(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, _a = self._shim("[AMS_SWITCH]feed finish -1, stall")
        afcBambuAMS._check_ams_fault(shim, lane)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert len(raised) == 1        # sequence already handled

    def test_scan_stall_exit_is_ignored(self):
        # The RFID scan reports "bldc stall exit!" as it finishes its pull-in.
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, _a = self._shim("[AMS_RFID] STEP3,bldc stall exit!")
        afcBambuAMS._check_ams_fault(shim, lane)
        assert raised == []

    def test_unload_does_not_raise(self):
        # An unload retracts against resistance by design.
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, _a = self._shim("[AMS_SWITCH]pull err, bdc stall",
                                     unloading=True)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert raised == []

    def test_drying_does_not_raise(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, _a = self._shim("[AMS_SWITCH]feed finish -1, stall",
                                     drying=True)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert raised == []

    def test_pause_disabled_warns_instead(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, warned, _a = self._shim("[AMS_SWITCH]feed finish -1, stall",
                                          pause=False)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert raised == [] and len(warned) == 1

    def test_pause_latches_the_follower_hold_and_drops_assist(self):
        # Re-arming into a jam makes the AMS grind on filament it cannot move.
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, _, assist = self._shim("[AMS_SWITCH]feed finish -1, stall")
        afcBambuAMS._check_ams_fault(shim, lane)
        assert shim._follow_fault_hold is True
        assert assist == [("lane22", False)]
        assert shim._starved_since == 0.0

    def test_warn_only_does_not_hold_the_follower(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, _, warned, assist = self._shim("[AMS_SWITCH]feed finish -1, stall",
                                             pause=False)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert shim._follow_fault_hold is False and assist == []

    def test_detect_disabled_does_nothing(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, raised, warned, _a = self._shim("[AMS_SWITCH]feed finish -1, stall",
                                          detect=False)
        afcBambuAMS._check_ams_fault(shim, lane)
        assert raised == [] and warned == []


class TestFaultHoldRelease:
    """The stall hold suppresses the auto-arm until the print resumes."""

    def _shim(self, held=True, saw_pause=False, paused=False):
        shim = types.SimpleNamespace(
            name="BambuAMS_2",
            _follow_fault_hold=held, _follow_fault_saw_pause=saw_pause,
            logger=types.SimpleNamespace(info=lambda m: None),
            afc=types.SimpleNamespace(function=types.SimpleNamespace(
                is_paused=lambda: paused)))
        return shim

    def test_not_held_is_inactive(self):
        shim = self._shim(held=False)
        assert afcBambuAMS._fault_hold_active(shim) is False

    def test_held_before_the_pause_lands_stays_active(self):
        # AFC_error queues the pause, so the very next tick still reads
        # "not paused". Releasing there would re-arm straight back into the jam.
        shim = self._shim(paused=False)
        assert afcBambuAMS._fault_hold_active(shim) is True
        assert shim._follow_fault_hold is True

    def test_held_while_paused_stays_active_and_records_the_pause(self):
        shim = self._shim(paused=True)
        assert afcBambuAMS._fault_hold_active(shim) is True
        assert shim._follow_fault_saw_pause is True

    def test_releases_once_resumed(self):
        shim = self._shim(saw_pause=True, paused=False)
        assert afcBambuAMS._fault_hold_active(shim) is False
        assert shim._follow_fault_hold is False
        assert shim._follow_fault_saw_pause is False

    def test_unreadable_pause_state_keeps_the_hold(self):
        shim = self._shim(paused=False)
        shim.afc = types.SimpleNamespace()      # no .function at all
        assert afcBambuAMS._fault_hold_active(shim) is True


class TestLoadAcknowledgesItsOwnStalls:
    """A stall the load recovered from must not be replayed at the follower.

    From AFC.log, an AMS 2 loading lane20: the unit retried a reluctant bay by
    itself -- "switch_feed rocker stall, tray_cnt:0,16,0,0", its own retry
    counter climbing over ten feed kicks -- and then REACHED the toolhead
    sensor. The load succeeded. But those reports were still queued with an
    unhandled sequence number, so the first follower tick after the load raised
    a fault from one, latched the follower hold and dropped the assist on a
    lane that had loaded correctly. Reported as "ams 2 follower not working".
    """

    class _Bridge:
        def __init__(self, seq):
            self._seq = seq

        def last_fault(self):
            return (self._seq, "8 [AMS_SWITCH]switch_feed rocker stall, "
                               "tray_cnt:0,16,0,0", 0.42)

    def _shim(self, bridge, seen=0):
        return types.SimpleNamespace(
            name="BambuAMS_2", _bridge=bridge, _fault_seen=seen,
            fault_detect=True, _unload_in_progress=False, _drying=False,
            fault_pause=True, _follow_fault_hold=False,
            _follow_fault_saw_pause=False, _starved_since=0.0,
            logger=types.SimpleNamespace(
                info=lambda m: None, debug=lambda m, **k: None,
                warning=lambda m: None))

    def test_ack_marks_pending_faults_handled(self):
        shim = self._shim(self._Bridge(seq=7))
        afcBambuAMS._ack_faults(shim)
        assert shim._fault_seen == 7

    def test_a_stall_from_the_load_is_not_re_raised_after_ack(self):
        br = self._Bridge(seq=7)
        shim = self._shim(br)
        afcBambuAMS._ack_faults(shim)
        raised = []
        shim._raise_ams_fault = lambda lane, msg: raised.append(msg)
        afcBambuAMS._check_ams_fault(
            shim, types.SimpleNamespace(name="lane20"))
        assert raised == []
        assert shim._follow_fault_hold is False

    def test_a_stall_AFTER_the_load_still_raises(self):
        # Acknowledging the load's own stalls must not deafen the follower to
        # a genuine jam that starts once it is running.
        br = self._Bridge(seq=7)
        shim = self._shim(br)
        afcBambuAMS._ack_faults(shim)
        br._seq = 8                              # a new, post-load report
        raised = []
        shim._raise_ams_fault = lambda lane, msg: raised.append(msg)
        afcBambuAMS._check_ams_fault(
            shim, types.SimpleNamespace(name="lane20"))
        assert len(raised) == 1
        assert "rocker stall" in raised[0]

    def test_ack_is_safe_without_a_bridge(self):
        shim = self._shim(None)
        afcBambuAMS._ack_faults(shim)            # must not raise
        assert shim._fault_seen == 0


class TestFaultHoldBlocksAutoArm:
    """While held, the follower tick must not engage or re-assert mode:4."""

    def _shim(self, hold):
        state = {"engaged": [], "assist": [], "sent": []}
        lane = types.SimpleNamespace(name="lane22")
        toolhead = types.SimpleNamespace(get_position=lambda: [0, 0, 0, 0.0])
        shim = types.SimpleNamespace(
            _following_lane=None, _follow_last_e=None,
            follow_min_extrude=0.05, follow_when_loaded=True,            follow_debug_interval=0.0, _follow_last_log=0.0,
            follow_poll_interval=0.1, ams_index=0,
            _follow_manual_off=False, _unload_in_progress=False,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            _bridge=types.SimpleNamespace(
                send=lambda o: state["sent"].append(o),
                latest_status=lambda: {"fstate": 0}),
            _tool_loaded_lane=lambda: lane,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: hold,
            _ready_to_follow=lambda lane=None: True,
            _engage_follower=lambda ln: (state["engaged"].append(ln.name),
                                         setattr(shim, "_following_lane", ln)),
            set_feed_assist=lambda ln, on: state["assist"].append((ln.name, on)),
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_held_does_not_engage(self):
        shim, state = self._shim(hold=True)
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["engaged"] == [] and state["assist"] == []

    def test_released_engages_again(self):
        shim, state = self._shim(hold=False)
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["engaged"] == ["lane22"]


class TestChamberTelemetryRegex:
    """The [AMS_CHMB] drying telemetry parser.

    The original pattern required `rf:N|vt:N.N,N`. Every line ever captured
    from an AMS 2 Pro and an AMS HT is `rf:N,N|vt:N.N|ap:...` instead, so it
    matched nothing and chamber temperature/target stayed None for the life of
    the feature.
    """

    # Verbatim from AFC.log, both models.
    HT = ("[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100|ad:2"
          "|wd:0000|fa:98|t:70")
    AMS2 = ("[AMS_CHMB]s:2|rf:65,0|vt:24.1|ap:22.0|hts:52,22,00|pw:100|ad:2"
            "|wd:0000|fa:99|t:40")

    def _fields(self, line):
        from extras.AFC_BambuAMS import _CHMB_STATE_RE
        m = _CHMB_STATE_RE.search(line)
        assert m is not None, "did not match: %s" % (line,)
        return m.group(1), m.group(2), m.group(3), m.group(4)

    def test_ams_ht_line(self):
        assert self._fields(self.HT) == ("2", "55", "44.0", None)

    def test_ams2_pro_line(self):
        assert self._fields(self.AMS2) == ("2", "65", "24.1", None)

    def test_line_with_a_leading_prefix(self):
        # The drain interleaves other text ahead of the record.
        assert self._fields("R " + self.HT)[2] == "44.0"

    def test_humidity_is_read_when_a_model_attaches_it(self):
        assert self._fields("[AMS_CHMB]s:2|rf:55|vt:29.8,38") == \
            ("2", "55", "29.8", "38")

    @pytest.mark.parametrize("line", [
        "[AMS_CHMB]dry_mode:1, check ok! dur:480,tmpr:55,pre_check:1",
        "[AMS_CHMB]s:off->wind_res1",
        "[AMS_CHMB]finish!",
        "[AMS_CHMB]set state CTC_STATE_HEATING, from selfcheck",
    ])
    def test_non_telemetry_chatter_is_ignored(self, line):
        from extras.AFC_BambuAMS import _CHMB_STATE_RE
        assert _CHMB_STATE_RE.search(line) is None

    # Once the log drain was addressed to each unit's OWN device rather than the
    # captured 0x0700, the units switched to a comma-separated, space-padded
    # form carrying an extra `cd:` field. The pipe-only pattern stopped matching
    # and the heater panel went blank for AMS 2 units. Verbatim from the
    # printer's gcode store, mid-dry at 55C.
    COMMA = ("[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 "
             "pw:100, ad:2, wd:0,0,0,0, fa:102")

    def test_comma_separated_form(self):
        assert self._fields(self.COMMA) == ("2", "55", "23.1", None)

    def test_comma_form_with_a_leading_framing_byte(self):
        # The drain reply is often prefixed by one stray rendered byte.
        assert self._fields("\\ " + self.COMMA) == ("2", "55", "23.1", None)

    def test_cd_field_is_not_mistaken_for_the_chamber_probe(self):
        # `cd` sits between rf and vt and happens to equal the target here; the
        # chamber reading must still come from vt.
        assert self._fields(self.COMMA)[2] == "23.1"

    def test_both_separators_still_parse(self):
        # Neither form may regress the other -- both are live, and which one a
        # unit emits follows the addressing, not the model.
        assert self._fields(self.HT)[1] == "55"
        assert self._fields(self.COMMA)[1] == "55"


class TestAmsNarrationNoiseFilter:
    """Pure link-layer chatter is logged but kept off the operator's console.

    The AMS repeats select/mode/ref bookkeeping many times a second, from every
    unit on the wire. It belongs in AFC.log -- a select storm is what a chain-
    addressing fault looks like -- but it was burying every line that carries
    real information.
    """

    @pytest.mark.parametrize("line", [
        "[AMS_CALL] ams0 select,select ams1 [AMS_CALL] ams0 select,select ams1",
        "# [AMS_CALL] ams0 select,select ams0",
        "s [AMS_COMMON]mode: 4 -> 0 [AMS_COMMON]ref: 128 -> 128",
        "[AMS_IDLE]set ams state switch",
        "[AMS_COMMON]mode: 0 -> 4 [AMS_COMMON]ref: 128 -> 128 "
        "[AMS_LINK]ams0 select,req ams0",
    ])
    def test_pure_chatter_is_console_suppressed(self, line):
        from extras.AFC_BambuAMS import _ams_is_noise
        assert _ams_is_noise(line) is True

    @pytest.mark.parametrize("line", [
        # Chatter BUNDLED with real narration must survive: the AMS mixes
        # registers freely within one line, so "contains noise -> drop" would
        # have thrown away the tag-read steps riding alongside it.
        "[AMS_CALL] ams0 select,select ams0 [AMS_DEV] STEP:set 0 tray_preload",
        "g [AMS_DEV] STEP2:pull tray 0 from switch [AMS_DEV] STEP:rfid pull 0",
        "\\ [AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0",
        "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m",
        "< [AMS_TRAY]tray[0] sw_sta update, 0 -> 1, u_in_out:1,0",
        "[AMS_CHMB]ignore dry_mode:1, ams_state:2",
        "[AMS_DOOR]wind_door[1] closing [AMS_BDC_OFF]BDC offline isr enter",
    ])
    def test_anything_informative_stays_on_the_console(self, line):
        from extras.AFC_BambuAMS import _ams_is_noise
        assert _ams_is_noise(line) is False

    def test_text_without_any_bracket_is_not_treated_as_noise(self):
        from extras.AFC_BambuAMS import _ams_is_noise
        assert _ams_is_noise("some unstructured reply") is False
        assert _ams_is_noise("") is False


class TestRfidReadInFlight:
    """The tag-read fallback must wait on the unit, not on a stopwatch.

    Two consecutive inserts in the same bay of the same AMS 1 (AFC.log):
    one stalled at tray_readid and never pulled the tray (defaults at 14s were
    right); the other completed a real read 13.0s after the insert edge, one
    second inside the same 14s fallback.
    """

    # Verbatim, insert #1 -- stalled. None of these may defer the fallback.
    STALLED = [
        "[AMS_CALL] ams0 select,select ams0 [AMS_DEV] STEP:set 0 tray_preload",
        "[AMS_DEV] STEP:set 0 tray_readid [AMS_CALL] ams0 select,select ams0",
        "& [AMS_DEV] STEP:odom tray_id error 255",
    ]
    # Verbatim, insert #2 -- a real read. Each of these must defer it.
    READING = [
        "g [AMS_DEV] STEP2:pull tray 0 from switch [AMS_DEV] STEP:rfid pull 0 "
        "[AMS_DEV] STEP3:start,read all card",
        "q [AMS_DEV] STEP3:search finished, found 0 card [AMS_DEV] STEP4:feed "
        "and judge place [AMS_DEV] STEP5:no card in RF",
        "[AMS_DEV] STEP:card auth success! [RF] tray0: info write to flash "
        "[AMS_DEV] STEP:read success,valid [AMS_DEV] STEP:read_done=1",
        "G [AMS_DEV] STEP:feed with rfid success [AMS_DEV] STEP7:finish,cali "
        "tray",
        "[AMS_RFID]STEP:read success",
    ]

    @pytest.mark.parametrize("line", STALLED)
    def test_preload_and_readid_do_not_count_as_a_read(self, line):
        from extras.AFC_BambuAMS import _RFID_INFLIGHT_RE
        assert _RFID_INFLIGHT_RE.search(line) is None

    @pytest.mark.parametrize("line", READING)
    def test_committed_read_steps_count(self, line):
        from extras.AFC_BambuAMS import _RFID_INFLIGHT_RE
        assert _RFID_INFLIGHT_RE.search(line) is not None

    def test_no_card_in_rf_is_not_terminal(self):
        # Insert #2 emitted it 7s before the tag authenticated, so it must keep
        # the read alive rather than end it.
        from extras.AFC_BambuAMS import _RFID_INFLIGHT_RE
        assert _RFID_INFLIGHT_RE.search("[AMS_DEV] STEP5:no card in RF")

    def test_chamber_telemetry_is_not_a_read(self):
        from extras.AFC_BambuAMS import _RFID_INFLIGHT_RE
        assert _RFID_INFLIGHT_RE.search(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1") is None

    def test_bridge_stamps_and_expires_read_activity(self):
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        assert bridge.rfid_read_in_flight(reactor.monotonic()) is False
        bridge.handle_line(
            '{"evt":"amsdbg","text":"[AMS_DEV] STEP2:pull tray 0 from switch"}')
        now = reactor.monotonic()
        assert bridge.rfid_read_in_flight(now) is True
        # A few seconds of silence ends the read.
        assert bridge.rfid_read_in_flight(now + 10.0) is False

    def test_repeated_read_step_still_stamps_despite_dedupe(self):
        # The narration dedupe blanks an identical repeat, but a repeat is still
        # evidence the read is alive -- the stamp is taken from the raw line
        # before the dedupe runs.
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        line = '{"evt":"amsdbg","text":"[AMS_DEV] STEP:rfid pull 0"}'
        bridge.handle_line(line)
        bridge._rfid_step_t = None         # pretend it aged out
        bridge.handle_line(line)           # identical -> deduped for logging
        assert bridge.rfid_read_in_flight(reactor.monotonic()) is True


from unittest.mock import MagicMock   # noqa: E402
from extras.AFC_BambuAMS import BambuBridge  # noqa: E402


class TestTheHeartbeatCannotBreakTheDedupe:
    """
    The AMS bundles a 10-second liveness heartbeat into whatever frame is
    going out, and its timestamp changes every time. It was already dropped
    from the console -- but only AFTER the dedupe had stored it as the last
    line, so every 10s it broke the run of identical lines and the next repeat
    printed as if it were new.

    That is why a sentence repeating 200 times a minute reached the console
    6 times a minute: 6 = 60/10, the heartbeat period. Nothing about the
    message itself.

    Deliberately NOT solved by adding the repeated line to _AMS_NOISE_RE:
    that sentence turned out to be a real fault (an unbounded re-read of a
    bay holding an untagged spool), and suppressing it would have hidden it.
    """

    LINE = "[AMS_LINK]get_slot ams1 tray0 basic"
    BEAT = " [DBG] ams time: now=42044054ms diff=10005ms"

    def test_the_heartbeat_segment_is_stripped(self):
        from extras.AFC_BambuAMS_bridge import _DBG_AMSTIME_RE
        assert _DBG_AMSTIME_RE.sub("", self.LINE + self.BEAT).strip() == self.LINE

    def test_a_heartbeat_only_frame_reaches_nobody(self):
        # The drain reply opens with one stray framing byte, so a heartbeat-only
        # frame strips down to something like "," -- no bracket, no content.
        # Stripping the segment ALSO stopped the console drop from matching (it
        # tests for the "[DBG] ams time" substring), which put a bare comma on
        # the operator's console every 10 seconds.
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        before = len(logger.messages)
        bridge.handle_line(
            '{"evt":"amsdbg","addr":1792,'
            '"text":", [DBG] ams time: now=49107885ms diff=10004ms"}')
        printed = [m for m in logger.messages[before:] if "AMS:" in str(m)]
        assert not printed, f"heartbeat-only frame was logged: {printed}"

    def test_a_heartbeat_riding_with_real_narration_keeps_the_narration(self):
        from extras.AFC_BambuAMS_bridge import _DBG_AMSTIME_RE
        both = self.LINE + self.BEAT
        kept = _DBG_AMSTIME_RE.sub("", both).strip()
        assert "[" in kept and kept == self.LINE

    def test_a_bundled_heartbeat_does_not_reset_the_repeat_run(self):
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        mk = lambda t: '{"evt":"amsdbg","addr":1792,"text":"%s"}' % t
        bridge.handle_line(mk(self.LINE))
        n0 = bridge._last_dbg_n
        # The same sentence, this time carrying the heartbeat. It must count as
        # a repeat, not start a new run.
        bridge.handle_line(mk(self.LINE + self.BEAT))
        bridge.handle_line(mk(self.LINE))
        assert bridge._last_dbg_n > n0, "the heartbeat restarted the run"

    def test_link_chatter_is_still_visible_not_filtered(self):
        # The fix is the loop, not a mute. This line must stay reportable.
        from extras.AFC_BambuAMS_bridge import _ams_is_noise
        assert _ams_is_noise(self.LINE) is False

    def test_the_90s_preload_housekeeping_is_console_suppressed(self):
        from extras.AFC_BambuAMS_bridge import _ams_is_noise
        assert _ams_is_noise(
            "^ [AMS_COMMON]preload_disable:1, tmpr:25.8, cd:0 "
            "[AMS_COMMON]preload_disable:0, tmpr:25.8, cd:0") is True

    @pytest.mark.parametrize("line", [
        "[AMS_RFID] STEP3,save to flash ,card info valid",   # a tag committed
        "[AMS_COMMON]state:6,tray_now:255,tray_exit:1",      # AMS 1's fault
        "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1",              # chamber telemetry
    ])
    def test_the_filter_does_not_reach_lines_that_matter(self, line):
        from extras.AFC_BambuAMS_bridge import _ams_is_noise
        assert _ams_is_noise(line) is False

    def test_a_suppressed_line_still_reaches_the_parsers(self):
        # only_debug hides a line from the console and nothing else. The
        # dedupe used to be able to cost a PARSE; it cannot now -- everything
        # downstream of it reads the raw line.
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        line = '{"evt":"amsdbg","addr":1792,"text":"%s"}'
        pull = "[AMS_SWITCH]pull sucess, mode change, mode:4"
        bridge.handle_line(line % pull)
        first = bridge.last_pull()
        bridge.handle_line(line % pull)      # byte-identical repeat
        assert bridge.last_pull() > first, \
            "a deduped repeat swallowed a motion completion"


class TestAForeignTagIsNotAnEmptyBay:
    """
    A Mifare chip whose keys are not Bambu's answers anticollision -- so its UID
    is readable -- and then fails authentication. The AMS 2 said so verbatim on
    a Snapmaker spool:

        [AMS_RFID]STEP:stop goto auth
        [AMS_RFID]STEP:auth fail:-4
        [AMS_RFID]STEP7:info_valid 0 or bbl:-1

    Reporting that as "the bay reader saw no chip" is wrong in the direction
    that sends someone looking for a hardware fault.
    """

    @pytest.mark.parametrize("line", [
        "[AMS_RFID]STEP:auth fail:-4",
    ])
    def test_the_refusal_is_recognised(self, line):
        from extras.AFC_BambuAMS_bridge import _RFID_FOREIGN_TAG_RE
        assert _RFID_FOREIGN_TAG_RE.search(line) is not None

    def test_info_valid_zero_is_not_foreign_evidence(self):
        # "info_valid 0 or bbl:N" rode the foreign pattern for months, but
        # the corpus audit showed it on EMPTY-BAY cycles ("no card detected"
        # -> info_valid 0 -> cali end) and mid-retry on HT reads that then
        # SUCCEEDED. It means "no valid record right now", not "chip
        # refused" -- matching it worded empty bays as foreign tags. Only
        # the auth refusal, which requires a chip to refuse, is evidence.
        from extras.AFC_BambuAMS_bridge import _RFID_FOREIGN_TAG_RE
        for line in ("[AMS_RFID]STEP7:info_valid 0 or bbl:-1",
                     "[AMS_RFID] STEP4,info_valid 0 or bbl:1"):
            assert _RFID_FOREIGN_TAG_RE.search(line) is None

    @pytest.mark.parametrize("line", [
        "[AMS_RFID] STEP3,save to flash ,card info valid",
        "[AMS_DEV] STEP:read success,valid",
        "[AMS_RFID]STEP0:checking",
        "[AMS_RFID]STEP:tray pull over 880 mm, but no card detected",
    ])
    def test_a_good_read_or_an_empty_bay_is_not_a_refusal(self, line):
        from extras.AFC_BambuAMS_bridge import _RFID_FOREIGN_TAG_RE
        assert _RFID_FOREIGN_TAG_RE.search(line) is None

    def test_it_is_credited_to_the_unit_that_said_it(self):
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        t0 = reactor.monotonic()
        bridge.handle_line(
            '{"evt":"amsdbg","addr":1792,"text":"[AMS_RFID]STEP:auth fail:-4"}')
        assert bridge.rfid_foreign_tag_since(t0, addr=0x0700) is True
        assert bridge.rfid_foreign_tag_since(t0, addr=0x1800) is False


class TestClearingALaneClearsWhatTheOperatorSees:
    """
    filament_name is the field the Mainsail card DISPLAYS, and
    apply_filament_defaults does not write it -- it sets material, color,
    weight, sub_type and spool_vendor. So a field left out of
    _clear_lane_filament survives a defaulted lane and shows the previous
    spool's name.

    Measured on an untagged insert into a bay that had held Bambu PLA Matte:
    the firmware slot record was correctly blank, the removal unbound, the scan
    finalised and defaults applied -- and the lane still read
    filament_name='Bambu PLA Matte'.
    """

    def _lane(self):
        return types.SimpleNamespace(
            name="lane19", material="PLA Matte", color="#7A7A7A", weight=1000,
            filament_name="Bambu PLA Matte", sub_type="Matte",
            spool_vendor="Bambu", bambu_sku="GFA01", spool_id=None)

    def test_every_tag_written_field_is_cleared(self):
        lane = self._lane()
        afcBambuAMS._clear_lane_filament(
            types.SimpleNamespace(), lane)
        assert lane.filament_name == "", "the displayed name survived"
        for f in ("material", "color", "bambu_sku", "sub_type", "spool_vendor"):
            assert getattr(lane, f) == "", f"{f} survived"
        assert lane.weight == 0

    def test_a_lane_missing_the_attribute_is_survived(self):
        # Best-effort per attribute: a lane object without one of these must
        # not take the clear (or the insert edge) down with it.
        lane = types.SimpleNamespace(name="lane19", material="PLA")
        afcBambuAMS._clear_lane_filament(types.SimpleNamespace(), lane)
        assert lane.material == ""


class TestTheUnitDecidesWhatHappened:
    """
    _scan_verdict is the whole scan state machine: we command a scan and the
    unit tells us what came of it.

    What it replaced was three overlapping predicates, one of which compared
    the record's CONTENT against a pre-scan snapshot. That one could not answer
    a re-scan of the same spool at all -- the profile comes back byte-for-byte
    identical -- so the hold ran until a fallback timer gave up, 14 s boxed /
    25 s HT, with the LANE stale for the whole window while the slot data
    beside it was current. That is the split seen on the panel: weight and
    classification fresh, name and colour lagging.

    The unit narrates its read about a second in. Ask it.
    """

    def _u(self, read_ok=False, ended=False, t0=100.0, now=101.0):
        b = MagicMock()
        b.rfid_read_succeeded_since = lambda since, addr=None: read_ok
        b.rfid_cycle_ended_since = lambda since, addr=None: ended
        return types.SimpleNamespace(
            _bridge=b, _scan_t0=[t0], dry_dev_addr=0x0700,
            SCAN_FALLBACK_CAP=45.0,
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: now)))

    def test_a_narrated_read_is_the_answer(self):
        assert afcBambuAMS._scan_verdict(self._u(read_ok=True), 0) == "read"

    def test_a_read_outranks_a_cycle_end_in_the_same_window(self):
        # A measuring scan narrates its end well after the read. The read is
        # what happened; the end is just when it stopped working.
        u = self._u(read_ok=True, ended=True)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_a_finished_cycle_with_no_read_means_no_tag(self):
        assert afcBambuAMS._scan_verdict(self._u(ended=True), 0) == "notag"

    def test_mid_cycle_it_has_not_answered_yet(self):
        assert afcBambuAMS._scan_verdict(self._u(), 0) == "waiting"

    def test_no_scan_open_is_not_a_verdict(self):
        u = self._u(read_ok=True)
        u._scan_t0 = [None]
        assert afcBambuAMS._scan_verdict(u, 0) == "none"

    def test_an_unknown_slot_is_not_a_verdict(self):
        assert afcBambuAMS._scan_verdict(self._u(), None) == "none"
        assert afcBambuAMS._scan_verdict(self._u(), 9) == "none"

    def test_the_backstop_ends_a_unit_that_says_nothing(self):
        # BACKSTOP ONLY -- reached solely when the unit narrates neither a read
        # nor an end. A bay must not wait forever on a unit that went quiet.
        u = self._u(t0=100.0, now=100.0 + 46.0)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_a_broken_bridge_does_not_leave_the_bay_waiting(self):
        # Nothing can answer, so waiting is the one thing that cannot be right:
        # fall through to defaults rather than hold the lane blank.
        u = self._u(read_ok=True)
        u._bridge.rfid_read_succeeded_since = MagicMock(
            side_effect=RuntimeError("down"))
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"
        u2 = self._u()
        u2._bridge = None
        assert afcBambuAMS._scan_verdict(u2, 0) == "notag"


class TestAMissIsRememberedOnBothBranches:
    """
    "A UID Spoolman does not know is a PERMANENT answer, not a retry."

    _spoolman_sync has two paths -- full decode, and UID-only -- and the memo
    that stops a re-query every status pass was on the first one only. The
    UID-only path is what runs for a bay whose profile has not landed yet,
    which is EVERY scan for its first seconds, so a spool Spoolman does not
    know re-queried on every frame and blocked the reactor in HTTP each time.

    Its documented cost is not cosmetic: 1061 "Resetting prediction variance"
    events and an MCU shutdown.
    """

    def _u(self, found=None):
        u = types.SimpleNamespace(
            name="AMS", logger=_Logger(), _bound_uid={},
            _spoolman_no_match=set(),
            _unbind_spool=lambda ln, reason="": None,
            _remember_bound_uid=lambda s, uid: None,
            _spoolman_slot_info=lambda info: {"material": ""},
            afc=types.SimpleNamespace(spoolman=object(), spool=None))
        return u

    def test_a_uid_only_miss_is_remembered(self, monkeypatch):
        import extras.AFC_BambuAMS as m
        monkeypatch.setattr(m, "find_spool_by_uid", lambda c, u: None)
        monkeypatch.setattr(m, "_bambu_spoolman_client", lambda afc: object())
        u = self._u()
        lane = types.SimpleNamespace(name="lane1", spool_id=None)
        info = {"index": 0, "present": True, "rfid_uid": "deadbeef"}
        afcBambuAMS._spoolman_sync(u, lane, info)
        assert "deadbeef" in u._spoolman_no_match, \
            "a UID-only miss was not memoed -- it will re-query every frame"

    def test_the_memo_short_circuits_the_next_pass(self, monkeypatch):
        import extras.AFC_BambuAMS as m
        calls = []
        monkeypatch.setattr(m, "find_spool_by_uid",
                            lambda c, u: calls.append(u) or None)
        monkeypatch.setattr(m, "_bambu_spoolman_client", lambda afc: object())
        u = self._u()
        lane = types.SimpleNamespace(name="lane1", spool_id=None)
        info = {"index": 0, "present": True, "rfid_uid": "deadbeef"}
        afcBambuAMS._spoolman_sync(u, lane, info)
        afcBambuAMS._spoolman_sync(u, lane, info)
        afcBambuAMS._spoolman_sync(u, lane, info)
        assert len(calls) == 1, f"Spoolman was asked {len(calls)} times"


class TestTheSpoolmanBindingFollowsTheTag:
    """
    "This lane is bound" and "this lane is bound to the spool physically in it"
    are not the same fact, and treating them as one wrote a measurement onto
    the wrong reel:

        02:22:21  spool REMOVED from slot 0
        02:22:21  unbinding lane23 from spool 87 -- the bay is empty
        02:22:21  matched lane23 to Spoolman spool 87 by UID 0a1882ac  <- 66ms
        02:23:56  spool INSERTED  (a different spool, tag 01D0EC0F)
        02:24:33  tag read: PLA Basic (9CDBD9) [tag 01D0EC0F]
        02:24:33  wrote 810 g to Spoolman spool 87   <- the PLA GLOW reel
    """

    def _u(self, bound_uid=None, spool_id=None):
        u = types.SimpleNamespace(
            name="HT", logger=_Logger(), afc=None,
            _bound_uid=dict(bound_uid or {}),
            _unbind_spool=lambda ln, reason="": setattr(ln, "spool_id", ""),
            _remember_bound_uid=lambda s, uid: None)
        lane = types.SimpleNamespace(name="lane23", spool_id=spool_id)
        return u, lane

    def test_an_empty_bay_never_binds(self):
        # The leftover UID must not re-bind the lane the instant it is unbound.
        u, lane = self._u()
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": False, "rfid_uid": "0a1882ac"})
        assert lane.spool_id is None            # never reached Spoolman at all

    def test_the_same_tag_on_a_bound_lane_is_a_no_op(self):
        u, lane = self._u({0: "0a1882ac"}, spool_id=87)
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": True, "rfid_uid": "0a1882ac"})
        assert lane.spool_id == 87              # left alone

    def test_a_different_tag_releases_the_old_binding(self):
        # afc is None, so the sync returns right after the unbind -- which is
        # exactly the behaviour under test: the stale link does not survive.
        u, lane = self._u({0: "0a1882ac"}, spool_id=87)
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": True, "rfid_uid": "01d0ec0f"})
        assert lane.spool_id == ""              # released for the new tag

    def test_a_binding_we_did_not_make_is_left_alone(self):
        # No recorded UID -> a manual/restored assignment. Not ours to revoke.
        u, lane = self._u({}, spool_id=42)
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": True, "rfid_uid": "01d0ec0f"})
        assert lane.spool_id == 42

    # ── AFTER A RESTART THERE IS NO MEMO, AND THE BINDING IS STILL STALE ──
    #
    # _bound_uid lives in memory and starts empty every boot, so every tagged
    # bay looks "not bound by a tag read" after a restart and the check above
    # returns -- keeping whatever spool was in the bay BEFORE the restart.
    # Measured live: lane17 read a blue PLA Matte tag (ECB61CD0) while still
    # bound to spool 87, the Glow, which had moved to a different unit; the
    # lane showed "Glow" because the binding, not the tag, names the spool.
    #
    # Spoolman's record of which UIDs a spool carries outlives the restart, so
    # it is the authority when our memo is gone.

    def test_a_restart_does_not_preserve_a_stale_binding(self):
        u, lane = self._u({}, spool_id=87)      # no memo: the restart case
        # Spoolman says 87 carries the Glow's tag, not the one in this bay.
        u._binding_contradicted = lambda sid, uid: True
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": True, "rfid_uid": "ecb61cd0"})
        assert lane.spool_id == ""              # released

    def test_a_spool_with_no_recorded_uid_still_survives_a_restart(self):
        # Nothing to contradict -> the hand-assigned spool keeps its lane.
        u, lane = self._u({}, spool_id=42)
        u._binding_contradicted = lambda sid, uid: False
        afcBambuAMS._spoolman_sync(
            u, lane, {"index": 0, "present": True, "rfid_uid": "ecb61cd0"})
        assert lane.spool_id == 42


class TestNothingIsClaimedBeforeTheUnitFinishes:
    """
    The record of a bay mid-scan is not an answer yet, and _sync_lanes is the
    one place that enforces it: while the verdict is "waiting" the lane is not
    touched at all.

    A bay's UID survives in the unit's record after the scan clears the profile
    fields, so the UID-only Spoolman bind could fire on the PREVIOUS spool's
    UID -- and did, announcing the match before the insert edge was even
    logged:

        22:12  spool REMOVED from slot 0
        22:12  unbinding lane23 from spool 137 -- the bay is empty
        22:12  matched lane23 to Spoolman spool 137 by UID 01d0ec0f
        22:12  spool INSERTED in slot 0          <- the insert is AFTER

    Comparing the record's CONTENT could not catch that: the scan clears the
    profile, so a wiped profile read as "the unit re-read" while the stale UID
    rode along underneath. Asking the unit does catch it.
    """

    def _run(self, verdict, info=None):
        lane = types.SimpleNamespace(
            hub_obj=_vhub(True), tool_loaded=False, prep_state=None,
            _load_state=None, loaded_to_hub=None, status=None,
            bambu_slot_info=None)
        surfaced, finalized = [], []
        slots = [info or {"present": True, "rfid_uid": "01d0ec0f"}]
        shim = _sync_shim({"l": 0}, {"l": lane}, slots, verdict=verdict,
                          surfaced=surfaced, finalized=finalized)
        afcBambuAMS._sync_lanes(shim)
        return shim, lane, surfaced, finalized

    def test_mid_scan_the_lane_is_not_touched(self):
        _, _, surfaced, finalized = self._run("waiting")
        assert surfaced == [] and finalized == []

    def test_a_read_surfaces_the_record(self):
        _, lane, surfaced, finalized = self._run("read")
        assert surfaced == [lane] and finalized == []

    def test_no_scan_open_surfaces_normally(self):
        _, lane, surfaced, _ = self._run("none")
        assert surfaced == [lane]

    def test_no_tag_applies_defaults_instead_of_the_record(self):
        shim, lane, surfaced, finalized = self._run("notag")
        assert finalized == [0], "the lane must get its defaults"
        assert surfaced == [], "the bay's leftover profile must NOT surface"
        assert shim._scan_notag[0] is True

    def test_no_tag_does_not_reuse_the_old_spools_remain(self):
        # THE UNIT WILL NOT MEASURE A SPOOL IT DID NOT READ A BAMBU TAG ON --
        # its firmware, not ours; we open the capacity window on every insert
        # and it declines. So a remain% still sitting in the bay's record came
        # off the PREVIOUS spool's tag, exactly like the rest of the profile,
        # and there is no weight to let through on this branch.
        weighed = []
        lane = types.SimpleNamespace(
            hub_obj=_vhub(True), tool_loaded=False, prep_state=None,
            _load_state=None, loaded_to_hub=None, status=None)
        shim = _sync_shim({"l": 0}, {"l": lane},
                          [{"present": True, "index": 0, "remain_pct": 80}],
                          verdict="notag")
        shim._measured_remain = {0: 80}      # cannot happen; proves it is unused
        shim._apply_remain_weight = lambda ln, info: weighed.append(info)
        afcBambuAMS._sync_lanes(shim)
        assert weighed == []

    def test_the_defaults_are_applied_once_not_per_frame(self):
        lane = types.SimpleNamespace(
            hub_obj=_vhub(True), tool_loaded=False, prep_state=None,
            _load_state=None, loaded_to_hub=None, status=None,
            bambu_slot_info=None)
        surfaced, finalized = [], []
        shim = _sync_shim({"l": 0}, {"l": lane}, [{"present": True}],
                          verdict="notag", surfaced=surfaced,
                          finalized=finalized)
        for _ in range(5):
            afcBambuAMS._sync_lanes(shim)
        assert finalized == [0], "defaults re-applied on every status frame"


class TestWeightWriteSaysWhereTheNumberCameFrom:
    """
    A tag record announced as a physical measurement is the machine claiming
    work it did not do -- seen as "wrote 1000 g remaining ... (physical AMS
    measurement)" during an insert in which no measurement ran at all.
    """

    def _shim(self):
        pushed = []
        return types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            _measured_remain={0: 64},
            _push_measured_to_spoolman=lambda ln, g, src="": pushed.append(
                (g, src))), pushed

    def _lane(self):
        return types.SimpleNamespace(name="lane23", weight=0,
                                     tool_loaded=False)

    def test_a_real_measurement_says_so(self):
        shim, pushed = self._shim()
        afcBambuAMS._apply_remain_weight(
            shim, self._lane(), {"index": 0, "remain_pct": 100, "weight": 1000})
        assert pushed == [(640, "physical AMS measurement")]

    def test_a_tag_record_is_never_written_at_all(self):
        # It used to be written and merely LABELLED as coming from the tag.
        # It is not written now: the tag's stored remain does not track
        # printing and nothing updates it, so on a reel holding 230 g it
        # still read 80% and pushed 800 g into the operator's inventory.
        shim, pushed = self._shim()
        shim._measured_remain = {}
        afcBambuAMS._apply_remain_weight(
            shim, self._lane(), {"index": 0, "remain_pct": 100, "weight": 1000})
        assert pushed == []

    def test_a_measurement_is_written_and_named(self):
        shim, pushed = self._shim()
        shim._measured_remain = {0: 23}
        afcBambuAMS._apply_remain_weight(
            shim, self._lane(), {"index": 0, "remain_pct": 80, "weight": 1000})
        assert pushed == [(230, "physical AMS measurement")]


class TestReadSuccessInEveryDialect:
    """
    A successful tag read must be recognisable from EVERY unit type.

    These are verbatim lines from AFC_BambuAMS.log, one complete successful
    read per model. The AMS HT ones are the regression: ``_RFID_READ_OK_RE``
    required a literal ``STEP:`` -- the boxed punctuation -- so on an HT it
    could not match anything, ever. ``rfid_read_succeeded_since()`` was
    therefore hard-wired False for that model, and ``_finalize_scan`` took the
    "no readable tag, apply defaults / keep the leftover record" path on every
    HT insert, including a re-insert of the same spool.
    """

    HT_OK = [
        # 17:49:02  0x1800 -- authenticated and committed to its own flash
        "[AMS_RFID] STEP3,auth card successful [RF] tray0: info write to "
        "flash [AMS_RFID] STEP3,save to flash ,card info valid",
        # 17:49:04  0x1800 -- and said the read landed
        "[AMS_RFID] STEP3,feed with rfid success [AMS_RFID] STEP3,read "
        "success ,goto Cali",
    ]
    BOXED_OK = [
        "[AMS_DEV] STEP:read success,valid",       # AMS 1  (space, colon)
        "[AMS_RFID]STEP:read success,valid",       # AMS 2  (no space, colon)
        "[AMS_DEV] STEP:read_done=1",
    ]
    NOT_OK = [
        # Mid-cycle steps and the failure end of the window. A read that RUNS
        # is not a read that LANDS.
        "[AMS_RFID] STEP2,search 0 card",
        "[AMS_RFID] STEP3,empty to read,feed with rfid",
        "[AMS_DEV] STEP5:no card in RF",
        "[AMS_DEV] STEP:search finished, found 0 card",
        "[AMS_DEV] STEP:tray pull over 790 mm, but no card detected",
        "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1",
    ]

    @pytest.mark.parametrize("line", HT_OK + BOXED_OK)
    def test_every_dialect_reports_a_landed_read(self, line):
        from extras.AFC_BambuAMS_bridge import _RFID_READ_OK_RE
        assert _RFID_READ_OK_RE.search(line) is not None

    @pytest.mark.parametrize("line", NOT_OK)
    def test_running_or_failed_is_not_a_landed_read(self, line):
        from extras.AFC_BambuAMS_bridge import _RFID_READ_OK_RE
        assert _RFID_READ_OK_RE.search(line) is None

    @pytest.mark.parametrize("line", HT_OK)
    def test_ht_steps_also_register_as_in_flight(self, line):
        # The HT's comma punctuation defeated the in-flight pattern too; only
        # the "[RF] trayN:" alternative was carrying it.
        from extras.AFC_BambuAMS_bridge import _RFID_INFLIGHT_RE
        assert _RFID_INFLIGHT_RE.search(line) is not None

    def test_ht_cycle_end_still_matches(self):
        from extras.AFC_BambuAMS_bridge import _RFID_CYCLE_END_RE
        assert _RFID_CYCLE_END_RE.search("[AMS_RFID] STEP4,Calibration rst:0")

    def test_bridge_credits_a_read_to_the_device_that_said_it(self):
        # An AMS 1 (0x0700) narrating a successful read while an HT (0x1800)
        # is mid-scan must not hand the HT a read it never made -- which is
        # not hypothetical: an AMS 1 insert opened 3s into an HT scan window.
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        t0 = reactor.monotonic()
        bridge.handle_line(
            '{"evt":"amsdbg","addr":1792,'
            '"text":"[AMS_DEV] STEP:read success,valid"}')
        assert bridge.rfid_read_succeeded_since(t0, addr=0x0700) is True
        assert bridge.rfid_read_succeeded_since(t0, addr=0x1800) is False
        # Bridge-wide (no addr) keeps its old, deliberately loose behaviour.
        assert bridge.rfid_read_succeeded_since(t0) is True

    def test_ht_read_is_credited_to_the_ht(self):
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        t0 = reactor.monotonic()
        bridge.handle_line(
            '{"evt":"amsdbg","addr":6144,'
            '"text":"[AMS_RFID] STEP3,save to flash ,card info valid"}')
        assert bridge.rfid_read_succeeded_since(t0, addr=0x1800) is True
        assert bridge.rfid_read_succeeded_since(t0, addr=0x0700) is False

    def test_unattributed_narration_still_answers_for_any_address(self):
        # Firmware that reports no address must not read as "this unit has
        # gone silent" -- that is the mistake a dead counter already caused
        # once.
        bridge, reactor, logger, _seen = _bridge()
        bridge.reactor = reactor
        t0 = reactor.monotonic()
        bridge.handle_line(
            '{"evt":"amsdbg","text":"[AMS_DEV] STEP:read success,valid"}')
        assert bridge.rfid_read_succeeded_since(t0, addr=0x1800) is True

    def test_scan_echoes_are_not_unhandled_events(self):
        from extras.AFC_BambuAMS_bridge import _BRIDGE_EVENTS_KNOWN
        for evt in ("scan", "reid", "reread", "prime"):
            assert evt in _BRIDGE_EVENTS_KNOWN


class TestChamberTelemetryAttribution:
    """Chamber telemetry must reach only the unit that produced it.

    It arrives on the log drain, which is addressed to 0x0700 -- a bus-wide
    address, not a unit -- and the text carries no unit id, so it was stored
    on the BRIDGE and read by every unit sharing it. A drying HT therefore
    published its chamber temperature as an idle AMS 2's as well.
    """

    class _Bridge:
        def __init__(self, temp=52.0, target=55.0, state=2, seen=100.0):
            self._chmb_temp = temp
            self._chmb_target = target
            self._chmb_state = state
            self._chmb_t_seen = seen

    class _Reactor:
        def __init__(self, now=100.0):
            self._now = now

        def monotonic(self):
            return self._now

    def _unit(self, name, bridge, drying, printer, reactor, heater=True):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.name = name
        u._bridge = bridge
        u._drying = drying
        u.has_heater = heater
        u.reactor = reactor
        u.printer = printer
        u.logger = MagicMock()
        return u

    def _wire(self, units):
        printer = MagicMock()
        printer.lookup_objects.return_value = [("AFC_BambuAMS " + u.name, u)
                                               for u in units]
        for u in units:
            u.printer = printer
        return printer

    def test_the_only_drying_unit_owns_it(self):
        r, b = self._Reactor(), self._Bridge()
        ht = self._unit("HT", b, True, None, r)
        ams2 = self._unit("AMS2", b, False, None, r)
        self._wire([ht, ams2])
        assert ht._owns_chamber_telemetry() is True

    def test_an_idle_unit_on_the_same_bridge_does_not(self):
        r, b = self._Reactor(), self._Bridge()
        ht = self._unit("HT", b, True, None, r)
        ams2 = self._unit("AMS2", b, False, None, r)
        self._wire([ht, ams2])
        # AMS2 must not report the HT's chamber temperature.
        assert ams2._owns_chamber_telemetry() is False

    def test_two_drying_units_share_a_bridge_so_neither_claims_it(self):
        r, b = self._Reactor(), self._Bridge()
        ht = self._unit("HT", b, True, None, r)
        ams2 = self._unit("AMS2", b, True, None, r)
        self._wire([ht, ams2])
        assert ht._owns_chamber_telemetry() is False
        assert ams2._owns_chamber_telemetry() is False

    def test_ambiguity_is_reported_once_not_every_poll(self):
        r, b = self._Reactor(), self._Bridge()
        ht = self._unit("HT", b, True, None, r)
        ams2 = self._unit("AMS2", b, True, None, r)
        self._wire([ht, ams2])
        for _ in range(5):
            ht._owns_chamber_telemetry()
        assert ht.logger.info.call_count == 1

    def test_units_on_separate_bridges_do_not_block_each_other(self):
        r = self._Reactor()
        ht = self._unit("HT", self._Bridge(), True, None, r)
        ams2 = self._unit("AMS2", self._Bridge(), True, None, r)
        self._wire([ht, ams2])
        assert ht._owns_chamber_telemetry() is True
        assert ams2._owns_chamber_telemetry() is True

    def test_a_sole_heater_claims_it_even_when_the_flag_says_idle(self):
        # _drying is host state and is wrong after a Klipper restart: the AMS
        # keeps drying. With one heater on the bridge nothing else can be
        # narrating, so the reading is unambiguously this unit's and the panel
        # can catch up to a cycle it did not start.
        r, b = self._Reactor(), self._Bridge()
        only = self._unit("HT", b, False, None, r)
        self._wire([only])
        assert only._owns_chamber_telemetry() is True

    def test_an_idle_unit_does_not_claim_it_when_another_heater_shares_the_bus(self):
        r, b = self._Reactor(), self._Bridge()
        idle = self._unit("AMS2", b, False, None, r)
        other = self._unit("HT", b, True, None, r)
        self._wire([idle, other])
        assert idle._owns_chamber_telemetry() is False

    def test_a_heaterless_unit_never_claims_it(self):
        r, b = self._Reactor(), self._Bridge()
        ams1 = self._unit("AMS1", b, False, None, r, heater=False)
        self._wire([ams1])
        assert ams1._owns_chamber_telemetry() is False

    def test_freshness_window(self):
        r = self._Reactor(now=100.0)
        fresh = self._unit("a", self._Bridge(seen=50.0), True, None, r)
        stale = self._unit("b", self._Bridge(seen=-100.0), True, None, r)
        self._wire([fresh]); self._wire([stale])
        assert fresh._chamber_telemetry_fresh() is True
        assert stale._chamber_telemetry_fresh() is False


class TestChamberTelemetryByAddress:
    """Attribution by the device address the text frame carried.

    The AMS's 1A/02 text frames carry the sending address at bytes [7:8], and
    it is per-model -- 0x0700 on an AMS 2 Pro, 0x1800 on an HT -- which is
    exactly the unit's own dry_dev_addr. With the firmware reporting it, two
    units on one bridge can dry at once and keep their chambers apart.
    """

    class _Bridge:
        def __init__(self, by_addr=None):
            self._chmb_by_addr = by_addr or {}
            self._chmb_temp = None
            self._chmb_target = None
            self._chmb_state = None
            self._chmb_t_seen = 0.0

    class _Reactor:
        def monotonic(self):
            return 100.0

    def _unit(self, name, bridge, addr, drying=True):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.name, u._bridge, u._drying = name, bridge, drying
        u.has_heater = True
        u.dry_dev_addr = addr
        u.reactor = self._Reactor()
        u.logger = MagicMock()
        u.printer = MagicMock()
        u.printer.lookup_objects.return_value = []
        return u

    def _rec(self, temp, target, seen=100.0, state=2):
        return {"temp": temp, "target": target, "state": state, "seen": seen}

    def test_each_unit_reads_its_own_address(self):
        b = self._Bridge({0x0700: self._rec(41.0, 55.0),
                          0x1800: self._rec(78.0, 85.0)})
        ams2 = self._unit("AMS2", b, 0x0700)
        ht = self._unit("HT", b, 0x1800)
        assert ams2._chamber_record()["temp"] == 41.0
        assert ht._chamber_record()["temp"] == 78.0

    def test_both_can_dry_at_once_without_crosstalk(self):
        # Both drying at once: each unit reports its own chamber, not nothing.
        b = self._Bridge({0x0700: self._rec(41.0, 55.0),
                          0x1800: self._rec(78.0, 85.0)})
        ht = self._unit("HT", b, 0x1800)
        assert ht._chamber_record()["target"] == 85.0

    def test_a_unit_with_no_record_gets_none(self):
        b = self._Bridge({0x1800: self._rec(78.0, 85.0)})
        ams2 = self._unit("AMS2", b, 0x0700)
        assert ams2._chamber_record() is None

    def test_stale_record_is_rejected(self):
        b = self._Bridge({0x1800: self._rec(78.0, 85.0, seen=-100.0)})
        ht = self._unit("HT", b, 0x1800)
        assert ht._record_fresh(ht._chamber_record()) is False

    def test_fresh_record_is_accepted(self):
        b = self._Bridge({0x1800: self._rec(78.0, 85.0, seen=50.0)})
        ht = self._unit("HT", b, 0x1800)
        assert ht._record_fresh(ht._chamber_record()) is True

    def test_old_firmware_without_addresses_falls_back(self):
        # No _chmb_by_addr entries: the shared value is still used, gated on
        # this unit being the only one drying.
        b = self._Bridge({})
        ht = self._unit("HT", b, 0x1800)
        assert ht._chamber_record() is None
        assert ht._owns_chamber_telemetry() is True


class TestBridgeStoresByAddress:
    def _bridge(self):
        br = BambuBridge.__new__(BambuBridge)
        br.name = "b"
        br.logger = MagicMock()
        br.reactor = MagicMock()
        br._chmb_temp = br._chmb_target = br._chmb_state = None
        br._chmb_t_seen = 0.0
        br._chmb_by_addr = {}
        br._last_chmb_t = 0.0
        br._last_human = None
        br._last_human_t = 0.0
        return br

    LINE = ("[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100"
            "|ad:2|wd:0000|fa:98|t:70")

    def test_address_keyed_record_is_stored(self):
        br = self._bridge()
        br._narrate_human(self.LINE, 100.0, 0x1800)
        assert br._chmb_by_addr[0x1800]["temp"] == 44.0
        assert br._chmb_by_addr[0x1800]["target"] == 55.0

    def test_two_addresses_do_not_overwrite_each_other(self):
        br = self._bridge()
        br._narrate_human(self.LINE, 100.0, 0x1800)
        br._narrate_human(self.LINE.replace("vt:44.0", "vt:22.5")
                          .replace("rf:55", "rf:65"), 101.0, 0x0700)
        assert br._chmb_by_addr[0x1800]["temp"] == 44.0
        assert br._chmb_by_addr[0x0700]["temp"] == 22.5

    def test_no_address_still_updates_the_shared_value(self):
        br = self._bridge()
        br._narrate_human(self.LINE, 100.0, None)
        assert br._chmb_temp == 44.0
        assert br._chmb_by_addr == {}


class TestDryingCatchUp:
    """The panel must catch up to a cycle it did not start.

    `drying` is host state set by AFC_BAMBU_HEATER_START, so a Klipper restart
    mid-cycle left the panel showing Idle beside a physically hot dryer --
    with the Start/Stop button offering Start. Chamber telemetry only streams
    WHILE a cycle runs, so its presence is direct evidence from the unit.
    """

    class _Bridge:
        def __init__(self, temp=52.0, seen=100.0, by_addr=None):
            self._chmb_temp = temp
            self._chmb_target = 55.0
            self._chmb_state = 2
            self._chmb_t_seen = seen
            self._chmb_by_addr = by_addr or {}

    def _unit(self, bridge, drying=False, addr=0x1800):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.name, u._bridge, u._drying = "HT", bridge, drying
        u.has_heater = True
        u.dry_dev_addr = addr
        u.reactor = MagicMock()
        u.reactor.monotonic.return_value = 100.0
        u.logger = MagicMock()
        u.printer = MagicMock()
        u.printer.lookup_objects.return_value = []
        return u

    def test_live_telemetry_means_drying_even_if_the_flag_is_false(self):
        u = self._unit(self._Bridge(seen=95.0))
        u.printer.lookup_objects.return_value = [("AFC_BambuAMS HT", u)]
        assert u._owns_chamber_telemetry() is True
        assert u._chamber_telemetry_fresh() is True

    def test_stale_telemetry_does_not_resurrect_a_finished_cycle(self):
        u = self._unit(self._Bridge(seen=-200.0))
        u.printer.lookup_objects.return_value = [("AFC_BambuAMS HT", u)]
        assert u._chamber_telemetry_fresh() is False

    def test_address_keyed_record_also_proves_it(self):
        rec = {"temp": 52.0, "target": 55.0, "state": 2, "seen": 95.0}
        u = self._unit(self._Bridge(by_addr={0x1800: rec}))
        assert u._record_fresh(u._chamber_record()) is True


class TestDryingStopSticks:
    """Stop must actually stop -- adoption may not undo it.

    Adoption re-set `_drying` from chamber telemetry up to 120s old, i.e. from
    the cycle just ended, on the very next status poll. On the printer the
    panel could not be made to go Idle by pressing Stop: every click was
    reverted within a second, and only one that happened to land after the
    staleness window expired stuck. Reported as "the panel doesn't update when
    they stop" after the units had physically switched off.
    """

    class _Bridge:
        def __init__(self, by_addr=None, seen=100.0, temp=52.0):
            self._chmb_temp = temp
            self._chmb_target = 55.0
            self._chmb_state = 2
            self._chmb_t_seen = seen
            self._chmb_by_addr = by_addr or {}
            self.sent = []

        def send(self, obj):
            self.sent.append(obj)

        def latest_status(self):
            return None

    def _unit(self, bridge, now=200.0, drying=True, seen_live=True,
              stop_t=0.0, addr=0x1800):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.name, u._bridge, u._drying = "HT", bridge, drying
        u.has_heater = True
        u.ams_model = "ht"
        u.ams_index = 0
        u.dry_dev_addr = addr
        u.dry_ams_id = 0
        u.dry_max_temp = 65
        u._dry_adopt_after = stop_t
        u._dry_seen_live = seen_live
        u._slots = [None] * 4
        u._following_lane = None
        u.reactor = MagicMock()
        u.reactor.monotonic.return_value = now
        u.logger = MagicMock()
        u.printer = MagicMock()
        u.printer.lookup_objects.return_value = []
        return u

    def _rec(self, seen):
        return {"temp": 52.0, "target": 55.0, "state": 2, "seen": seen}

    def test_telemetry_from_before_the_stop_does_not_readopt(self):
        # Last reading 30s ago -- still inside the 120s freshness window, but
        # older than the stop. It describes the cycle we just ended.
        br = self._Bridge(by_addr={0x1800: self._rec(170.0)})
        u = self._unit(br, now=200.0, drying=False, stop_t=180.0)
        live, attributable, seen_t = u._chamber_live()
        assert live is True                      # the record IS still fresh...
        assert seen_t < u._dry_adopt_after            # ...but predates the stop
        assert u._drying is False

    def test_telemetry_after_a_restart_is_still_adopted(self):
        # _dry_adopt_after is 0.0 at boot, so a cycle already running when Klipper
        # starts must still be picked up -- that is the point of adoption.
        br = self._Bridge(by_addr={0x1800: self._rec(195.0)})
        u = self._unit(br, now=200.0, drying=False, seen_live=False,
                       stop_t=0.0)
        live, _attributable, seen_t = u._chamber_live()
        assert live is True and seen_t > u._dry_adopt_after

    def test_silence_after_reporting_releases_the_cycle(self):
        # The unit reported for this cycle, then went quiet past the window:
        # a dryer that finished its own timer must not leave the panel
        # asserting "drying" forever.
        br = self._Bridge(by_addr={0x1800: self._rec(10.0)})
        u = self._unit(br, now=200.0, drying=True, seen_live=True)
        live, attributable, _seen = u._chamber_live()
        assert live is False and attributable is True

    def test_a_freshly_started_cycle_is_not_released_before_it_reports(self):
        # No telemetry yet ("Starting -- waiting for the unit to report").
        # _dry_seen_live is False, so silence must NOT be read as "finished".
        br = self._Bridge(by_addr={})
        u = self._unit(br, now=200.0, drying=True, seen_live=False,
                       stop_t=199.0)
        u.printer.lookup_objects.return_value = [("AFC_BambuAMS HT", u)]
        assert u._dry_seen_live is False

    def test_unattributable_telemetry_neither_adopts_nor_releases(self):
        # Two same-address units drying at once share one record and cannot be
        # told apart, so absence of telemetry means nothing for either.
        br = self._Bridge(by_addr={})
        u = self._unit(br, now=200.0, drying=True)
        other = self._unit(br, now=200.0, drying=True)
        other.name = "AMS2"
        u.printer.lookup_objects.return_value = [
            ("AFC_BambuAMS HT", u), ("AFC_BambuAMS AMS2", other)]
        other.printer = u.printer
        live, attributable, _seen = u._chamber_live()
        assert attributable is False
        assert live is False

    def test_stop_stamps_the_transition(self):
        br = self._Bridge(by_addr={0x1800: self._rec(170.0)})
        u = self._unit(br, now=200.0, drying=True)
        gcmd = MagicMock()
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_STOP(u, gcmd)
        assert u._drying is False
        # Stamped forward by the grace period: an AMS keeps narrating for a
        # moment after being told to stop, and those lines must not re-adopt
        # the cycle -- that is why Stop had to be pressed twice on hardware.
        assert u._dry_adopt_after == 200.0 + afcBambuAMS.DRY_STOP_GRACE
        assert u._dry_seen_live is False
        assert br.sent and br.sent[-1]["on"] == 0

    def test_start_clears_the_previous_cycles_evidence(self):
        br = self._Bridge(by_addr={0x1800: self._rec(50.0)})
        u = self._unit(br, now=200.0, drying=False, seen_live=True,
                       stop_t=10.0)
        u._tool_loaded_lane = lambda: None
        u.lanes = {}
        gcmd = MagicMock()
        gcmd.get_int.side_effect = lambda k, d=None, **kw: {
            "TEMP": 55, "TIME": 480, "ROTATE": 0,
            "AMSID": u.dry_ams_id, "ADDR": u.dry_dev_addr, "FORCE": 0}.get(k, d)
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(u, gcmd)
        assert u._drying is True
        assert u._dry_adopt_after == 200.0
        assert u._dry_seen_live is False


class TestDbgAddressByteOffsets:
    """The device address in an AMS text frame, pinned to real captures.

    A first cut read bytes [7:8] and shipped: it arrived on hardware as a
    constant 0x0003 and identified nothing, because [7:8] is the SENDER (the
    MC) and every unit's narration carries the same value. The device is at
    [9:10], little-endian.
    """

    # Verbatim from docs/captures.
    AMS2 = "3D0000001500F4000300071A020000"
    HT = "3D0000001500F4000300181A028000"

    def _addr(self, frame_hex):
        b = [int(frame_hex[i:i + 2], 16) for i in range(0, len(frame_hex), 2)]
        return (b[10] << 8) | b[9]          # what the firmware computes

    def _sender(self, frame_hex):
        b = [int(frame_hex[i:i + 2], 16) for i in range(0, len(frame_hex), 2)]
        return (b[7] << 8) | b[8]

    def test_ams2_frame_yields_its_dry_dev_addr(self):
        assert self._addr(self.AMS2) == 0x0700

    def test_ht_frame_yields_its_dry_dev_addr(self):
        assert self._addr(self.HT) == 0x1800

    def test_the_two_models_are_distinguishable(self):
        assert self._addr(self.AMS2) != self._addr(self.HT)

    def test_the_sender_field_is_not_a_discriminator(self):
        # Both carry 0x0003 there -- the bug this class exists to prevent.
        assert self._sender(self.AMS2) == self._sender(self.HT) == 0x0003

    def test_addresses_match_the_models_configured_dry_addr(self):
        from extras.AFC_BambuAMS import _AMS_MODELS
        assert self._addr(self.AMS2) == _AMS_MODELS["ams2"][1]
        assert self._addr(self.HT) == _AMS_MODELS["ht"][1]


class TestMcAddressing:
    """MC poll addressing, from the single-unit printer captures.

    Our replayed frames are all addressed to 0x0700 with payload 0x01. A real
    printer addresses every poll to the unit's OWN device with the unit's OWN
    id: 0x1800/0x00 for an HT, 0x0700/<chain index> for a boxed AMS. On an HT
    bus the whole poll set was going to a device that is not present.

    The HT's payload is 0x00, swept on hardware: 0x80 drew 0 replies from 352
    polls, 0x01 drew 0 from 134, 0x00 drew 35 from 70 with 19 carrying text.
    """

    def _shim(self, model, index=0, dev=None, amsid=-1):
        from extras.AFC_BambuAMS import _MC_ADDRESSING
        sent = []
        shim = types.SimpleNamespace(
            ams_index=index,
            _bridge=types.SimpleNamespace(send=lambda o: sent.append(o)))
        mc = _MC_ADDRESSING.get(model, (0x0700, 0x00))
        shim.mc_dev_addr = dev if dev is not None else mc[0]
        shim.mc_id_base = mc[1]
        shim.mc_ams_id = amsid
        shim._send_mc_addr = afcBambuAMS._send_mc_addr.__get__(shim)
        return shim, sent

    def test_ht_uses_its_own_device_and_id(self):
        shim, sent = self._shim("ht")
        shim._send_mc_addr(shim._bridge)
        assert sent == [{"cmd": "mcaddr", "unit": 0, "addr": 0x1800,
                         "pay": 0x00}]

    def test_the_ht_payload_is_the_one_it_actually_answers(self):
        # This asserted 0x80 for weeks and 0x80 is the value that draws no
        # reply at all -- the whole narration outage on an HT bus. Swept
        # against the physical unit; 0x00 is the only value it answers.
        from extras.AFC_BambuAMS import _MC_ADDRESSING
        assert _MC_ADDRESSING["ht"][1] == 0x00
        assert _MC_ADDRESSING["amsht"][1] == 0x00

    def test_boxed_ams_uses_0x0700_and_the_chain_index(self):
        shim, sent = self._shim("ams2")
        shim._send_mc_addr(shim._bridge)
        assert sent == [{"cmd": "mcaddr", "unit": 0, "addr": 0x0700,
                         "pay": 0}]

    def test_second_ht_would_take_its_chain_index(self):
        # The id is class base | chain position. With the HT's base measured
        # at 0x00 that makes a second HT 0x01 -- which now collides with a
        # second boxed AMS's id, distinguished only by the device address.
        # Not verified against two physical HTs.
        shim, sent = self._shim("ht", index=1)
        shim._send_mc_addr(shim._bridge)
        assert sent[0]["pay"] == 0x01

    def test_second_boxed_unit_takes_id_1(self):
        # The id identifies WHICH unit on the wire -- this is the case that
        # produced the old "payload 0x01 answered 115/115" measurement.
        shim, sent = self._shim("ams2", index=1)
        shim._send_mc_addr(shim._bridge)
        assert sent[0]["pay"] == 1

    def test_ams1_is_indistinguishable_from_ams2_by_address(self):
        # Confirmed on hardware: both answer at 0x0700 id 0x00 when alone.
        # This is why the model comes from config and is never probed.
        a, sa = self._shim("ams1")
        b, sb = self._shim("ams2")
        a._send_mc_addr(a._bridge); b._send_mc_addr(b._bridge)
        assert sa[0]["addr"] == sb[0]["addr"] == 0x0700
        assert sa[0]["pay"] == sb[0]["pay"] == 0

    def test_explicit_config_overrides_the_model_default(self):
        shim, sent = self._shim("ams2", dev=0x1800, amsid=0x80)
        shim._send_mc_addr(shim._bridge)
        assert sent[0]["addr"] == 0x1800 and sent[0]["pay"] == 0x80

    def test_no_bridge_is_a_no_op(self):
        shim, sent = self._shim("ht")
        shim._send_mc_addr(None)
        assert sent == []


class TestAmsDevNarration:
    """A regular AMS narrates as [AMS_DEV]/[RF], not [AMS_RFID]/[AMS_TRAY].

    None of the human-narration rules fired on an AMS 1 before these existed:
    they matched "[AMS_RFID]STEP:" while it emits "[AMS_DEV] STEP:" -- a
    different prefix AND a space after the bracket.
    """

    # Verbatim from captures/ams1_alone_insert_timestamped.txt.
    #
    # "read success,valid" and "feed with rfid success" are NOT here: they are
    # deliberately absent from the console table, because an HT emits both on
    # an attempt that then FAILS and retries, so narrating them announced a
    # read that had not happened. The auth and flash lines mark the real one.
    LINES = [
        "[AMS_DEV] STEP,first detected",
        "[AMS_DEV] STEP:card auth success!",
        "[RF] tray0: info write to flash",
    ]

    def _narrate(self, text):
        from extras.AFC_BambuAMS import _AMS_HUMAN
        for rx, fn in _AMS_HUMAN:
            m = rx.search(text)
            if m:
                return fn(m)
        return None

    @pytest.mark.parametrize("line", LINES)
    def test_every_real_ams1_line_narrates(self, line):
        assert self._narrate(line) is not None, line

    def test_flash_cache_line_names_the_bay_one_based(self):
        out = self._narrate("[RF] tray0: info write to flash")
        assert "bay 1" in out and "flash" in out

    def test_the_ams2_rules_still_work(self):
        # The AMS_DEV rules are inserted ahead of the AMS_RFID ones; make sure
        # they did not shadow them.
        assert self._narrate("[AMS_RFID]STEP:card auth success!") is not None

    @pytest.mark.parametrize("line", [
        "[AMS_DEV] STEP:read success,valid",
        "[AMS_DEV] STEP:feed with rfid success",
        "[AMS_RFID] STEP3,read success ,goto Cali",
        "[AMS_RFID] STEP3,feed with rfid success",
    ])
    def test_a_mid_cycle_read_claim_is_not_narrated(self, line):
        """These fire on an attempt that FAILS and retries.

        Captured on the HT: 'feed with rfid success' + 'read success ,goto
        Cali' at 04:56:57, then info_valid 0, then a retry, and only at
        04:57:07 the auth/flash pair that marked the read which actually
        landed. Narrating the first pair told the operator a spool had been
        read ten seconds before it was."""
        assert self._narrate(line) is None

    def test_the_authentication_is_narrated_in_every_dialect(self):
        for line in ("[AMS_DEV] STEP:card auth success!",
                     "[AMS_RFID]STEP:card auth success!",
                     "[AMS_RFID] STEP3,auth card successful"):
            assert self._narrate(line) is not None, line

# ── the AMS's own PTFE measurement ─────────────────────────────────────────────

class TestTubeLenFromNarration:
    """The unit self-calibrates its filament path from consecutive feeds and
    narrates the result. That is the real distance on THIS machine, so it beats
    any configured estimate -- but only once it has calibrated, and only for
    the unit that said it."""

    def _dbg(self, bridge, text, addr=None):
        obj = {"evt": "amsdbg", "text": text}
        if addr is not None:
            obj["addr"] = addr
        bridge.handle_line(json.dumps(obj))

    def test_the_mm_form_is_captured(self):
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm, list:3491,3472,0 "
                          "mm, err:19 mm", addr=0x0700)
        assert bridge.tube_len(0x0700) == 3481.0

    def test_the_metre_form_is_captured_and_converted(self):
        # This form appears on a STALL line, which is exactly when knowing the
        # calibrated length matters most.
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m, "
                          "tube_len:2.186 m", addr=0x1800)
        assert bridge.tube_len(0x1800) == 2186.0

    def test_zero_is_not_adopted(self):
        # The unit reports 0 until it has enough samples. Adopting that would
        # set every derived deadline to zero.
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:0 mm, list:3491,0,0 mm, "
                          "err:3491 mm", addr=0x0700)
        assert bridge.tube_len(0x0700) is None

    def test_zero_in_metres_is_not_adopted(self):
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m, "
                          "tube_len:0.000 m", addr=0x1800)
        assert bridge.tube_len(0x1800) is None

    def test_two_units_do_not_share_a_measurement(self):
        # The exact cross-unit attribution bug the chamber telemetry already
        # had: an HT's path length must never be served to an AMS 2.
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:1693 mm", addr=0x1800)
        assert bridge.tube_len(0x0700) == 3481.0
        assert bridge.tube_len(0x1800) == 1693.0

    def test_an_unaddressed_query_is_refused_when_two_units_reported(self):
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:1693 mm", addr=0x1800)
        assert bridge.tube_len(None) is None

    def test_an_unaddressed_query_answers_when_only_one_unit_reported(self):
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        assert bridge.tube_len(None) == 3481.0

    def test_narration_without_an_address_is_not_stored(self):
        # Firmware older than 1.0.7.0 does not say who narrated; storing that
        # against a guessed unit is worse than not storing it.
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm")
        assert bridge.tube_len(0x0700) is None
        assert bridge.tube_len(None) is None

    def test_a_later_measurement_replaces_the_earlier_one(self):
        bridge, _r, _l, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3502 mm", addr=0x0700)
        assert bridge.tube_len(0x0700) == 3502.0

    def test_the_first_adoption_is_announced_once(self):
        # Worth one console line -- it changes how every move timeout is
        # sized -- but not on every load.
        bridge, _r, logger, _s = _bridge()
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        self._dbg(bridge, "[AMS_SWITCH]new tube_len:3502 mm", addr=0x0700)
        said = [m for lvl, m in logger.messages
                if lvl == "info" and "measured filament path" in m]
        assert len(said) == 1, said
        assert "3481mm" in said[0]

    def test_an_unknown_unit_reports_nothing(self):
        bridge, _r, _l, _s = _bridge()
        assert bridge.tube_len(0x0700) is None


class TestNarrationSurvivesWithoutAnExternalName:
    """Regression: BambuBridge.name was never assigned by __init__, so every
    line matching _AMS_HUMAN raised AttributeError straight into handle_line's
    bare `except Exception: pass`. The say-it-in-English narration was dead in
    production and no test caught it, because the test shims all set `name`
    themselves. These build the bridge exactly as production does."""

    def test_a_freshly_constructed_bridge_has_a_name(self):
        bridge, _r, _l, _s = _bridge()
        assert getattr(bridge, "name", None)

    def test_a_matching_line_actually_reaches_the_console(self):
        bridge, reactor, logger, _s = _bridge()
        # Past the 1 s console floor: _last_human_t starts at 0.0, so a
        # reactor clock still at 0.0 would suppress this for a reason that has
        # nothing to do with the bug under test.
        reactor.advance(5.0)
        bridge.handle_line(json.dumps(
            {"evt": "amsdbg", "text": "[AMS_CHMB]set state CTC_STATE_HEATING"}))
        assert any("now heating" in m for lvl, m in logger.messages
                   if lvl == "info")

    def test_a_matching_line_does_not_block_later_parsing(self):
        # The failure mode that made this expensive: _AMS_HUMAN runs FIRST, so
        # an exception there skipped the tube_len and chamber blocks below it
        # for exactly the lines that carry them.
        bridge, _r, _l, _s = _bridge()
        bridge.handle_line(json.dumps(
            {"evt": "amsdbg", "addr": 0x0700,
             "text": "[AMS_SWITCH]new tube_len:3481 mm, list:3491,3472,0 mm, "
                     "err:19 mm"}))
        assert bridge.tube_len(0x0700) == 3481.0


class TestMoveSpeedsAreConstantsNotConfig:
    """feed_speed / retract_speed / max_speed were config options whose names
    promised control they never had: the AMS meters its own moves and both the
    firmware and _wait_move convert mm + mm/s into a runaway deadline. They are
    module constants now. These lock in that the collapse changed the config
    surface and NOT the behaviour."""

    def test_the_constants_are_the_old_defaults(self):
        assert afcBambuAMS_mod.NOMINAL_MMPS == 20.0
        assert afcBambuAMS_mod.MAX_MMPS == 30.0

    def test_the_unit_no_longer_carries_the_removed_attributes(self):
        # Anything still reading unit.feed_speed would now get an
        # AttributeError rather than a stale default.
        shim, _order = _eject_shim()
        for gone in ("feed_speed", "retract_speed", "max_speed"):
            assert not hasattr(shim, gone)

    def test_wait_move_sizes_its_deadline_from_the_measured_rate(self):
        # No longer the old feed_speed=20 basis: DEADLINE_MMPS is measured.
        calls = []

        class _R:
            def __init__(self):
                self.t = 0.0

            def monotonic(self):
                return self.t

            def pause(self, until):
                self.t = until

        shim = types.SimpleNamespace(
            afc=types.SimpleNamespace(reactor=_R()),
            _ams_mode=lambda: None,          # no status frame -> deadline only
            _bridge=types.SimpleNamespace(
                last_finish=lambda: (calls.append(1), (0, False, ""))[1]))
        # 200mm at DEADLINE_MMPS (60) -> 3.3s -> 2x + 5 = 11.7s of polling.
        # Returns False because no completion ever arrives.
        assert afcBambuAMS_mod.afcBambuAMS._wait_move(shim, 200.0) is False
        assert shim.afc.reactor.t == pytest.approx(11.7, abs=0.3)

    def test_an_explicit_speed_is_still_honoured_and_clamped(self):
        # feed()/retract() still take mmps for callers that pass one; it is
        # clamped to MAX_MMPS rather than to a per-unit config value.
        assert afcBambuAMS_mod.clamp_speed(999.0, afcBambuAMS_mod.MAX_MMPS) == 30.0
        assert afcBambuAMS_mod.clamp_speed(5.0, afcBambuAMS_mod.MAX_MMPS) == 5.0


class TestAdoptMeasuredPath:
    """The unit measures its own PTFE path and narrates it. That figure is
    better than anything an operator measures by hand, so it is adopted and
    written back through AFC's normal ConfigRewrite path -- the same way every
    other AFC calibration persists."""

    def _u(self, measured, bowden=3000.0, unload=None):
        writes = []
        u = types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            full_name=["AFC_BambuAMS", "AMS"],
            afc_bowden_length=bowden,
            afc_unload_bowden_length=bowden if unload is None else unload,
            measured_path_mm=lambda: measured,
            # A unit whose only source is tube_len -- the chain must fall
            # through the other two and still adopt.
            _measure_path_from_odom=lambda: None,
            _dw_len_mm=lambda: None,
            afc=types.SimpleNamespace(function=types.SimpleNamespace(
                ConfigRewrite=lambda sec, key, val, msg="":
                    writes.append((sec, key, val)))))
        u._path_measurement = lambda: afcBambuAMS._path_measurement(u)
        return u, writes

    def test_the_measurement_is_adopted(self):
        u, _w = self._u(2186.0)
        afcBambuAMS._adopt_measured_path(u)
        assert u.afc_bowden_length == 2186.0

    def test_it_is_written_to_the_config(self):
        u, writes = self._u(2186.0)
        afcBambuAMS._adopt_measured_path(u)
        assert ("AFC_BambuAMS AMS", "afc_bowden_length", 2186.0) in writes

    def test_the_unload_length_follows_when_it_was_defaulted(self):
        u, writes = self._u(2186.0)                 # unload == bowden
        afcBambuAMS._adopt_measured_path(u)
        assert u.afc_unload_bowden_length == 2186.0
        assert ("AFC_BambuAMS AMS", "afc_unload_bowden_length", 2186.0) in writes

    def test_a_deliberate_unload_length_is_left_alone(self):
        u, writes = self._u(2186.0, unload=800.0)
        afcBambuAMS._adopt_measured_path(u)
        assert u.afc_unload_bowden_length == 800.0
        assert not any(k == "afc_unload_bowden_length" for _s, k, _v in writes)

    def test_an_uncalibrated_unit_does_nothing_and_stays_retryable(self):
        # tube_len is 0 until two consistent samples, and an RFID read resets
        # the odometer -- "not yet" is the normal state and must not latch off.
        u, writes = self._u(None)
        afcBambuAMS._adopt_measured_path(u)
        assert writes == []
        assert getattr(u, "_path_adopted", False) is False

    def test_a_value_already_close_enough_is_not_rewritten(self):
        u, writes = self._u(3010.0)                 # within tolerance of 3000
        afcBambuAMS._adopt_measured_path(u)
        assert writes == []
        assert u.afc_bowden_length == 3000.0

    def test_it_only_writes_once_per_session(self):
        u, writes = self._u(2186.0)
        afcBambuAMS._adopt_measured_path(u)
        afcBambuAMS._adopt_measured_path(u)
        assert len(writes) == 2                     # bowden + unload, once each

    def test_a_failed_save_still_adopts_and_warns(self):
        # The value is live before the write is attempted, so a config that
        # cannot be written costs persistence, not the print in progress.
        u, _w = self._u(2186.0)

        def _boom(*a, **k):
            raise OSError("read-only filesystem")
        u.afc.function.ConfigRewrite = _boom
        afcBambuAMS._adopt_measured_path(u)
        assert u.afc_bowden_length == 2186.0
        assert any(lvl == "warning" and "Could not save" in m
                   for lvl, m in u.logger.messages)


class TestDistHubIsFixed:
    def test_it_is_a_constant_not_a_config_option(self):
        # The hub here is virtual -- the AMS multiplexes internally and there
        # is no switch to reach -- so there is nothing to measure and nothing
        # for an operator to set.
        assert afcBambuAMS_mod.DIST_HUB_MM == 250.0

    def test_the_bowden_default_is_long_enough_for_a_first_load(self):
        # A short default would abort the very load during which the unit
        # measures its path, so it could never calibrate.
        assert afcBambuAMS_mod.DEFAULT_BOWDEN_MM == 3000.0


class TestFollowerGatedOnAnEnergisedExtruder:
    """The post-restart pulsing. Klipper comes up with steppers de-energised
    while the loaded state is restored from saved vars, so the follower was
    engaged against an extruder gripping nothing: the AMS fed to refill its
    buffer, the filament just moved, and it kept poking. Homing ended it --
    because homing energises the motors, NOT because it changed which extruder
    is selected (that read "extruder" on both sides of the home)."""

    def _u(self, enabled, name="extruder", raises=False):
        line = types.SimpleNamespace(is_motor_enabled=lambda: enabled)

        def _lookup(obj, default=None):
            if raises:
                raise RuntimeError("not ready")
            if obj != "stepper_enable":
                return default
            return types.SimpleNamespace(lookup_enable=lambda n: line)
        return types.SimpleNamespace(
            extruder=name,
            printer=types.SimpleNamespace(lookup_object=_lookup))

    def test_an_energised_extruder_permits_engaging(self):
        assert afcBambuAMS._extruder_motor_enabled(self._u(True)) is True

    def test_a_dead_motor_blocks_engaging(self):
        assert afcBambuAMS._extruder_motor_enabled(self._u(False)) is False

    def test_the_lanes_extruder_wins_over_the_units(self):
        seen = []
        line = types.SimpleNamespace(is_motor_enabled=lambda: True)
        u = types.SimpleNamespace(
            extruder="extruder",
            printer=types.SimpleNamespace(lookup_object=lambda o, d=None:
                types.SimpleNamespace(
                    lookup_enable=lambda n: (seen.append(n), line)[1])))
        afcBambuAMS._extruder_motor_enabled(
            u, types.SimpleNamespace(extruder="extruder4"))
        assert seen == ["extruder4"]

    def test_it_fails_open_with_no_stepper_enable_object(self):
        u = self._u(False)
        u.printer = types.SimpleNamespace(lookup_object=lambda o, d=None: None)
        assert afcBambuAMS._extruder_motor_enabled(u) is True

    def test_it_fails_open_when_the_lookup_raises(self):
        # Runs during startup, where not every object exists yet.
        assert afcBambuAMS._extruder_motor_enabled(
            self._u(False, raises=True)) is True

    def test_it_fails_open_with_no_extruder_configured(self):
        assert afcBambuAMS._extruder_motor_enabled(
            self._u(False, name=None)) is True

    def test_it_fails_open_when_the_name_is_unknown(self):
        u = self._u(False)
        u.printer = types.SimpleNamespace(
            lookup_object=lambda o, d=None: types.SimpleNamespace(
                lookup_enable=lambda n: None))
        assert afcBambuAMS._extruder_motor_enabled(u) is True


class TestStartupRestoreSkipsADeadMotor:
    def _shim(self, enabled):
        engaged = []
        lane = types.SimpleNamespace(name="lane15", tool_loaded=True)
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), _bridge=object(),
            lanes={"lane15": lane},
            _ready_to_follow=lambda ln=None: enabled,
            _engage_follower=lambda ln: engaged.append(ln.name))
        return shim, engaged

    def test_it_engages_when_the_motor_is_live(self):
        shim, engaged = self._shim(True)
        afcBambuAMS._startup_restore_loaded(shim)
        assert engaged == ["lane15"]

    def test_it_skips_when_the_motor_is_dead(self):
        shim, engaged = self._shim(False)
        afcBambuAMS._startup_restore_loaded(shim)
        assert engaged == []

    def test_the_skip_is_explained_in_the_log(self):
        # Silently not engaging would look identical to the follower being
        # broken, which is the harder fault to chase.
        shim, _e = self._shim(False)
        afcBambuAMS._startup_restore_loaded(shim)
        assert any("de-energised" in m for _lvl, m in shim.logger.messages)

    def test_a_lane_that_is_not_tool_loaded_is_untouched(self):
        shim, engaged = self._shim(True)
        shim.lanes["lane15"].tool_loaded = False
        afcBambuAMS._startup_restore_loaded(shim)
        assert engaged == []


class TestReadyToFollow:
    """The gate is gone: a loaded tray is followed whenever the printer is on.

    It used to require the extruder motor to be energised. That is a
    divergence from the printer we emulate -- a real Bambu streams the hold
    continuously and never asks about steppers -- and it cost an evening:
    idle timeout dropped the steppers, the gate stood the follower down, and
    filament pulled by hand was never recovered.
    """

    def test_always_ready(self):
        shim = types.SimpleNamespace()
        assert afcBambuAMS._ready_to_follow(shim) is True

    def test_a_dead_extruder_motor_no_longer_blocks(self):
        # The exact case that broke it live: homed, loaded, steppers dropped
        # by idle timeout. The follower must still hold.
        shim = types.SimpleNamespace(
            _extruder_motor_enabled=lambda lane=None: False)
        assert afcBambuAMS._ready_to_follow(shim, object()) is True

class TestToolheadHomedIsGone:
    """_toolhead_homed existed only as the second half of _ready_to_follow.
    That half was removed after it armed the follower on a rebooted machine
    that reported homed_axes "xyz" with every extruder disabled, so the helper
    goes with it rather than lingering as an unused input."""

    def test_the_helper_is_removed(self):
        assert not hasattr(afcBambuAMS, "_toolhead_homed")

class TestTheGateDoesNotBlockDockedTools:
    """Async loading into a DOCKED tool is planned, and a lane being loaded
    while its tool is parked needs its follower exactly as much as one on the
    shuttle. An earlier version of this gate also required the lane to be on
    the active tool, which would have blocked that outright. Whether a tool is
    docked is not evidence about whether filament is moving."""

    def test_a_live_motor_arms_regardless_of_which_tool_is_active(self):
        # No active-tool input exists: the decision is made from motor state
        # alone, so a lane whose tool is parked arms exactly like one on the
        # shuttle.
        u = types.SimpleNamespace(
            _extruder_motor_enabled=lambda lane=None: True)
        docked_lane = types.SimpleNamespace(name="lane23", extruder="extruder4")
        assert afcBambuAMS._ready_to_follow(u, docked_lane) is True

    def test_a_docked_tool_with_a_live_motor_arms_unhomed_too(self):
        u = types.SimpleNamespace(
            _extruder_motor_enabled=lambda lane=None: True,
            _toolhead_homed=lambda: False)
        docked_lane = types.SimpleNamespace(name="lane23", extruder="extruder4")
        assert afcBambuAMS._ready_to_follow(u, docked_lane) is True

    def test_the_gate_takes_no_active_tool_argument_at_all(self):
        # Guards against reintroducing the coupling: if someone adds an
        # active-tool check back into the signature this fails loudly.
        import inspect
        params = list(inspect.signature(
            afcBambuAMS._ready_to_follow).parameters)
        assert params == ["self", "lane"], params


class TestPrepPostLoadUsesTheSameGate:
    """prep_post_load is a THIRD automatic engage path, and it runs on every
    boot. Gating only _startup_restore_loaded left this one re-arming the
    follower against a de-energised extruder, which would have reintroduced
    the post-restart pulsing by the back door."""

    def _shim(self, ready, tool_loaded=True):
        engaged = []
        return types.SimpleNamespace(
            name="AMS", logger=_Logger(), _bridge=object(),
            _slots=[{"present": True}],
            _slot_of=lambda ln: 0,
            _ready_to_follow=lambda ln=None: ready,
            _engage_follower=lambda ln: engaged.append(ln.name)), engaged

    def _lane(self, tool_loaded=True):
        return types.SimpleNamespace(name="lane15", tool_loaded=tool_loaded,
                                     loaded_to_hub=False)

    def test_it_engages_when_ready(self):
        shim, engaged = self._shim(True)
        afcBambuAMS.prep_post_load(shim, self._lane())
        assert engaged == ["lane15"]

    def test_it_defers_on_a_cold_machine(self):
        shim, engaged = self._shim(False)
        afcBambuAMS.prep_post_load(shim, self._lane())
        assert engaged == []

    def test_the_deferral_is_explained(self):
        shim, _e = self._shim(False)
        afcBambuAMS.prep_post_load(shim, self._lane())
        assert any("deferring the follower" in m
                   for _lvl, m in shim.logger.messages)

    def test_the_hub_latch_still_happens_when_deferring(self):
        # The follower is deferred; the staged-at-hub bookkeeping is not, or
        # the lane would look unstaged for the rest of the session.
        shim, _e = self._shim(False)
        lane = self._lane()
        afcBambuAMS.prep_post_load(shim, lane)
        assert lane.loaded_to_hub is True

    def test_a_lane_not_tool_loaded_is_untouched(self):
        shim, engaged = self._shim(True)
        afcBambuAMS.prep_post_load(shim, self._lane(tool_loaded=False))
        assert engaged == []


class TestBambuTagFillsTheSpoolFields:
    """A Bambu tag says "PLA Matte" and the whole string went into
    lane.material, so the variant was lost inside the material field and
    spool_vendor / sub_type / filament_name stayed blank. Every surface that
    shows a spool -- the dryer panel, Spoolman, the RFID notifications -- then
    had no vendor and no variant to show."""

    def _u(self):
        return types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            _slot_map={"lane15": 0},
            _spoolman_sync=lambda lane, info: None,
            _apply_remain_weight=lambda lane, info: None,
            _save_lane_vars=lambda: None,
            lanes={})

    def _lane(self, **kw):
        lane = types.SimpleNamespace(name="lane15", spool_id=None,
                                     material=None, color=None, sub_type=None,
                                     spool_vendor=None, filament_name=None,
                                     weight=0, extruder_temp=None)
        for k, v in kw.items():
            setattr(lane, k, v)
        return lane

    def _apply(self, lane, info):
        afcBambuAMS._surface_slot_info(self._u(), lane, info)

    def test_the_variant_is_split_out_of_the_material(self):
        lane = self._lane()
        self._apply(lane, {"material": "PLA Matte"})
        assert lane.material == "PLA"
        assert lane.sub_type == "Matte"

    def test_the_vendor_is_set(self):
        # The AMS reader only answers Bambu tags, so the brand is known even
        # though it never appears on the wire.
        lane = self._lane()
        self._apply(lane, {"material": "PLA Basic"})
        assert lane.spool_vendor == "Bambu"

    def test_the_display_name_matches_the_ace_path(self):
        lane = self._lane()
        self._apply(lane, {"material": "PLA Matte"})
        assert lane.filament_name == "Bambu PLA Matte"

    def test_a_bare_material_leaves_no_variant(self):
        lane = self._lane()
        self._apply(lane, {"material": "ABS"})
        assert lane.material == "ABS" and lane.sub_type == ""
        assert lane.filament_name == "Bambu ABS"

    def test_a_hyphenated_composite_is_not_split(self):
        # "PLA-CF" is ONE material. Splitting on the hyphen would give a
        # material of "PLA" and a sub_type of "CF", which is a different
        # filament with a different density.
        lane = self._lane()
        self._apply(lane, {"material": "PLA-CF"})
        assert lane.material == "PLA-CF" and lane.sub_type == ""

    def test_a_multiword_variant_survives(self):
        lane = self._lane()
        self._apply(lane, {"material": "PETG HF Translucent"})
        assert lane.material == "PETG"
        assert lane.sub_type == "HF Translucent"

    def test_a_spoolman_linked_lane_still_gets_the_tag(self):
        # The bound lane is the one the delay was reported on. Its spool_id
        # never stopped the tag being READ -- it stopped the tag being APPLIED.
        lane = self._lane(spool_id=42, material="PETG", sub_type="")
        self._apply(lane, {"material": "PLA Matte"})
        assert lane.material == "PLA" and lane.sub_type == "Matte"
        assert lane.spool_vendor == "Bambu"
        assert lane.spool_id == 42


class TestSplitBambuMaterial:
    def test_material_and_variant(self):
        assert afcBambuAMS_mod._split_bambu_material("PLA Matte") == ("PLA", "Matte")

    def test_material_only(self):
        assert afcBambuAMS_mod._split_bambu_material("ABS") == ("ABS", "")

    def test_composites_stay_whole(self):
        assert afcBambuAMS_mod._split_bambu_material("PA6-CF") == ("PA6-CF", "")

    def test_empty_is_safe(self):
        assert afcBambuAMS_mod._split_bambu_material("") == ("", "")
        assert afcBambuAMS_mod._split_bambu_material(None) == ("", "")

    def test_extra_whitespace_does_not_create_empty_parts(self):
        assert afcBambuAMS_mod._split_bambu_material("  PLA   Matte  ") == ("PLA", "Matte")


class TestMoveDeadlineIsCapped:
    """The deadline is derived from distance / NOMINAL_MMPS, so raising the
    bowden default 500 -> 3000 multiplied every fallback wait by six. A
    toolhead unload that physically finished in seconds left AFC in "Tool
    Unloading" for 330 s -- timed on hardware, exactly the computed value.

    It is a runaway guard, not a schedule: the AMS meters its own move and
    normally ends it by reporting a completion, so the wait only matters when
    that report never comes. Scaling it without limit serves nothing."""

    class _R:
        def __init__(self):
            self.t = 0.0

        def monotonic(self):
            return self.t

        def pause(self, until):
            self.t = until

    def _shim(self, mode=None):
        return types.SimpleNamespace(
            afc=types.SimpleNamespace(reactor=self._R()),
            _ams_mode=lambda: mode,          # None -> no status frame
            _bridge=types.SimpleNamespace(last_finish=lambda: (0, False, "")))

    def test_a_long_move_is_capped(self):
        shim = self._shim()
        afcBambuAMS._wait_move(shim, 3250.0)          # would be 330 s uncapped
        assert shim.afc.reactor.t <= afcBambuAMS_mod.MOVE_DEADLINE_MAX_S + 0.5

    def test_a_short_move_uses_the_measured_rate(self):
        # 200mm at DEADLINE_MMPS (60) -> 3.3s -> *2 + 5 = 11.7s.
        shim = self._shim()
        afcBambuAMS._wait_move(shim, 200.0)
        assert shim.afc.reactor.t == pytest.approx(11.7, abs=0.3)

    def test_a_full_bowden_move_is_held_to_the_cap(self):
        # 3250mm at 60mm/s computes 113s; the cap holds it to 60s, which is
        # what a 560mm path produced before the bowden default was raised --
        # so no worse than the pre-change behaviour.
        shim = self._shim()
        afcBambuAMS._wait_move(shim, 3250.0)
        assert shim.afc.reactor.t <= 35.5

    def test_the_deadline_rate_is_separate_from_the_wire_speed(self):
        # Changing the watchdog basis must not change what we transmit.
        assert afcBambuAMS_mod.DEADLINE_MMPS == 60.0
        assert afcBambuAMS_mod.NOMINAL_MMPS == 20.0

    def test_the_cap_matches_the_pre_change_worst_case(self):
        # Deliberately the old 560mm-at-20mm/s figure: while the narration
        # drain is down the fallback is hit every time, so this must not be
        # worse than what was there before.
        assert afcBambuAMS_mod.MOVE_DEADLINE_MAX_S == 35.0


class TestLoadTimeoutUsesTheMeasuredRate:
    """The load give-up timeout had the same 20 mm/s error _wait_move did, and
    was missed when that one was fixed. feed_dist / 20 on a 3250 mm path is
    162 s before AFC would even consider the load finished -- reported from the
    machine as a load that "took forever to notify AFC it was done"."""

    def test_the_timeout_is_sized_from_the_measured_rate(self):
        d = 3250.0
        old = d / afcBambuAMS_mod.NOMINAL_MMPS          # 162.5 s
        new = d / afcBambuAMS_mod.DEADLINE_MMPS         # 54.2 s
        assert old > 160 and new < 60

    def test_it_is_NOT_capped_like_the_move_watchdog(self):
        # The move watchdog bounds hearing about ONE move; this bounds the AMS
        # completing a load INCLUDING its own feed/stall/retract/retry cycles.
        # Clamped to 35 s the give-up sat below load_retry_timeout (40 s) on
        # its own, so every attempt was cut off mid-cycle.
        assert afcBambuAMS_mod.LOAD_SENSOR_MAX_S > \
            afcBambuAMS_mod.MOVE_DEADLINE_MAX_S

    def test_the_window_clears_the_bulk_feed_plus_the_retry_budget(self):
        # 3250 mm at DEADLINE_MMPS is 54 s of pure travel; a window that does
        # not clear that cannot distinguish "still feeding" from "stalled".
        bulk = 3250.0 / afcBambuAMS_mod.DEADLINE_MMPS
        assert afcBambuAMS_mod.LOAD_SENSOR_MAX_S > bulk + 40.0

    def test_the_ams_gets_room_for_several_of_its_own_retry_cycles(self):
        # The unit's routine is feed -> stall -> retract -> retry, ~10-20 s a
        # cycle. Interrupting it with our re-home turns a load it finishes
        # unaided into three truncated attempts (166 s, measured).
        bulk = 3250.0 / afcBambuAMS_mod.DEADLINE_MMPS
        assert (afcBambuAMS_mod.LOAD_SENSOR_MAX_S - bulk) / 20.0 >= 4


class TestAnnounceSendsAreIndependent:
    """The four announce sends shared one try/except: pass, so a failure in any
    of the first three silently skipped _send_mc_addr. Without a per-unit MC
    address the firmware's log drain falls back to the captured 0x0700 pair,
    which never asks an HT at 0x1800 -- so every HT load, unload, stall and
    measured length was discarded while the bus looked healthy. Measured: the
    HT narrated the instant the drain was forced to 0x1800, and went silent
    again after a Pico reboot re-ran this announce."""

    def _shim(self, fail=None):
        sent = []

        class _B:
            def send(self, obj):
                if fail == "units":
                    raise RuntimeError("boom")
                sent.append(obj)
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), _bridge=_B(), ams_index=0,
            # The announce carries the unit's UID now, so the firmware can pin
            # it to a chain index. None means "not configured", which is the
            # ordinary case these tests cover.
            unit_uid=None, _id_resolved=True,
            _send_ht_flag=lambda b: (_ for _ in ()).throw(RuntimeError("boom"))
            if fail == "ht" else sent.append({"cmd": "htunit"}),
            _send_mc_addr=lambda b: sent.append({"cmd": "mcaddr"}))
        return shim, sent

    def test_all_of_them_are_sent_normally(self):
        # "model" joins the announce set: the firmware keys every
        # model-specific decision on it, and until it arrives a unit is
        # judged by AMS 1's vocabulary -- which is the cross-model confusion
        # the per-model split exists to end. Like the others it is re-sent on
        # every announce so it survives a Pico reboot.
        shim, sent = self._shim()
        afcBambuAMS._announce_unit(shim)
        assert [o["cmd"] for o in sent] == ["units", "htunit", "model",
                                            "mcaddr", "armms"]

    def test_the_arm_cadence_is_re_applied_on_every_announce(self):
        # It is a firmware runtime override, so a Pico reboot drops it back to
        # the built-in 520 ms. Re-sending it here is what makes it survive
        # without a reflash.
        shim, sent = self._shim()
        afcBambuAMS._announce_unit(shim)
        arm = [o for o in sent if o["cmd"] == "armms"]
        assert arm and arm[0]["ms"] == int(afcBambuAMS_mod.FOLLOW_ARM_MS)

    def test_the_cadence_is_the_printers_own(self):
        # REVERSED from "far slower than the firmware default". The hourly
        # override reasoned from "following holds without it" -- true, and
        # beside the point: 11/04 is the bus-wide liveness keep-alive, and
        # the full-bus printer reel (ams3_fullbus_tagged_and_rescans) streams
        # it to EVERY unit at full cadence straight through its measuring
        # AMS 2 rescan (0411@0700 3.98/s + 0411@1800 1.99/s). Under the
        # hourly override our TX carried ZERO 0411 in every echo ever
        # diffed -- the largest single deviation from the reference reels.
        # 600000 = effectively off, the MEASURED-GOOD regime (2026-08-10):
        # at the printer's 520 ms our blocking master's arm lands
        # mid-calibration and the second odometer edge dies; silenced, the
        # same cycles measured P:97/74/67. The printer survives its own
        # stream because its drive never blocks. Do not re-enable the sweep
        # without a live measure PASSING under it.
        assert afcBambuAMS_mod.FOLLOW_ARM_MS == 600000

    def test_a_failing_ht_flag_does_not_skip_the_mc_address(self):
        # The exact regression: mcaddr is what keeps the log drain per-unit.
        shim, sent = self._shim(fail="ht")
        afcBambuAMS._announce_unit(shim)
        assert any(o["cmd"] == "mcaddr" for o in sent), sent

    def test_a_failing_units_send_does_not_skip_the_rest(self):
        shim, sent = self._shim(fail="units")
        afcBambuAMS._announce_unit(shim)
        assert any(o["cmd"] == "mcaddr" for o in sent), sent

    def test_the_failure_is_named_not_swallowed(self):
        shim, _sent = self._shim(fail="ht")
        afcBambuAMS._announce_unit(shim)
        warns = [m for lvl, m in shim.logger.messages if lvl == "warning"]
        assert any("ht flag" in w for w in warns), warns


class TestModeCompletionWasRemoved:
    """`fstate` was once a SECOND completion signal in _wait_move, added on the
    theory that a typed status field cannot go quiet the way narration had.

    It never fired. Traced across a full unload and load on both units with
    every CHANGE written to the log: fstate read 4 from before the move to
    after it, and the trace recorded nothing at all. Whatever that field
    tracks, it is not motion on this firmware.

    A fallback that has never fired reads like a safety net and is not one, so
    it was removed. These tests exist so it is not re-added from the same
    reasoning: the silence it was written against was the drain payload (an HT
    answers 0x00, not 0x80) plus each dialect's own words, and those are what
    fixed it.
    """

    class _R:
        def __init__(self):
            self.t = 0.0

        def monotonic(self):
            return self.t

        def pause(self, until):
            self.t = until

    def test_the_accessor_is_gone(self):
        assert not hasattr(afcBambuAMS, "_ams_mode")

    def test_the_settled_set_is_gone(self):
        assert not hasattr(afcBambuAMS_mod, "_AMS_MODES_SETTLED")

    def test_the_mode_names_are_kept_as_documentation(self):
        # Still worth naming: IDLE is used by the follower re-assert and
        # FOLLOWING is the mode a load leaves the unit in, while the rest
        # document what the field means to anyone reading a status frame.
        # All five are pinned so the documented vocabulary cannot drift or be
        # dropped as unused.
        assert afcBambuAMS_mod.AMS_MODE_IDLE == 0
        assert afcBambuAMS_mod.AMS_MODE_ASSIST == 1
        assert afcBambuAMS_mod.AMS_MODE_MOVING == 2
        assert afcBambuAMS_mod.AMS_MODE_DONE == 3
        assert afcBambuAMS_mod.AMS_MODE_FOLLOWING == 4

    def test_narration_still_completes_the_wait(self):
        seq = {"n": 0}

        def finish():
            seq["n"] += 1
            return (seq["n"], True, "[AMS_SWITCH]feed finish, buff_pos:1.28")
        shim = types.SimpleNamespace(
            afc=types.SimpleNamespace(reactor=self._R()),
            _bridge=types.SimpleNamespace(last_finish=finish))
        assert afcBambuAMS._wait_move(shim, 3250.0) is True

    def test_a_move_with_no_completion_uses_the_deadline(self):
        # What removing it actually costs, stated plainly: with nothing said,
        # the wait runs to the deadline -- which is what the mode path was
        # supposed to spare us and never once did.
        shim = types.SimpleNamespace(
            afc=types.SimpleNamespace(reactor=self._R()),
            _bridge=types.SimpleNamespace(last_finish=lambda: (0, False, "")))
        assert afcBambuAMS._wait_move(shim, 200.0) is False
        assert shim.afc.reactor.t > 0.0


class TestOnlyTheToolheadSignalCompletesALoad:
    """The load loop kicks until the toolhead signal says filament arrived, and
    nothing else ends it early.

    A fallback to the AMS's own arrival report was added and removed. It was
    asked for so a lane with no toolhead sensor could still load -- but that
    lane does not exist: tool_start is always either a real PIN or "buffer",
    and both are authorities. So the branch was unreachable while looking like
    a safety net.

    Before it was gated on the sensor it DID fire, and reported a load
    complete with no filament at the toolhead: the AMS said
    "feed finish, dw_len:3.532 m" -- it had reached the end of ITS OWN measured
    tube -- while the filament was stuck in the PTFE. Relieving the friction by
    hand let it feed through and trip the sensor during the next purge. The
    kicks at the end of the path are what seat it, and stopping early removed
    them."""

    def _lane(self):
        return types.SimpleNamespace(
            name="lane19",
            extruder_obj=types.SimpleNamespace(tool_start="PA7"))

    def test_the_sensor_completes_a_load(self):
        shim, calls, _ = _load_shim(sensor_after=2)
        assert afcBambuAMS._feed_until_sensor(shim, self._lane(), 5.0) is True
        assert calls["stop"] == 1

    def test_an_untripped_sensor_fails_however_the_ams_reports(self):
        # An untripped sensor is a NEGATIVE signal, not a missing one.
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.4,
                                    arrivals=[(9, True)] * 40)
        assert afcBambuAMS._feed_until_sensor(
            shim, self._lane(), 0.4) is False

    def test_it_keeps_kicking_until_the_sensor_trips(self):
        shim, calls, _ = _load_shim(sensor_after=6, timeout=5.0,
                                    arrivals=[(9, True)] * 40)
        shim.load_retry_interval = 0.0
        assert afcBambuAMS._feed_until_sensor(shim, self._lane(), 5.0) is True
        assert len(calls["feed"]) >= 2

    def test_the_arrival_config_option_is_gone(self):
        # Removed rather than defaulted off: an option that cannot change
        # behaviour is worse than no option.
        assert not hasattr(afcBambuAMS, "_has_toolhead_sensor")
        assert not hasattr(afcBambuAMS, "_finish_since")

class TestDryPreCheckIsHtOnly:
    """An AMS HT will not heat with filament out of its bay -- confirmed on
    hardware, both hub-staged and toolhead-loaded. An ACE and an ACE 2 heat
    while printing, so this is a property of the UNIT, and whether an AMS 2 Pro
    shares it is untested. Warning on every model would state as fact something
    known for one."""

    def _shim(self, is_ht, lanes):
        said = []
        shim = types.SimpleNamespace(
            lanes={l.name: l for l in lanes},
            _is_ht=lambda: is_ht)
        shim._committed_lanes = afcBambuAMS._committed_lanes.__get__(shim)
        return shim, said

    def _lane(self, name, hub=False, tool=False):
        return types.SimpleNamespace(name=name, loaded_to_hub=hub,
                                     tool_loaded=tool)

    def test_a_hub_staged_lane_is_committed(self):
        shim, _ = self._shim(True, [self._lane("l1", hub=True)])
        assert [l.name for l in shim._committed_lanes()] == ["l1"]

    def test_a_toolhead_loaded_lane_is_committed_too(self):
        # The case the first version of this check wrongly excluded.
        shim, _ = self._shim(True, [self._lane("l1", tool=True)])
        assert [l.name for l in shim._committed_lanes()] == ["l1"]

    def test_a_lane_in_its_bay_is_not_committed(self):
        shim, _ = self._shim(True, [self._lane("l1")])
        assert shim._committed_lanes() == []

    def test_the_ht_predicate_is_what_gates_the_warning(self):
        # Pins the scoping decision itself: the warning is HT-only, so a
        # non-HT unit must not be told something only measured on an HT.
        ht = types.SimpleNamespace(has_heater=True, dry_dev_addr=0x1800)
        ams2 = types.SimpleNamespace(has_heater=True, dry_dev_addr=0x0700)
        ams1 = types.SimpleNamespace(has_heater=False, dry_dev_addr=0x0700)
        assert afcBambuAMS._is_ht(ht) is True
        assert afcBambuAMS._is_ht(ams2) is False
        assert afcBambuAMS._is_ht(ams1) is False


class TestDryPreCheckGatesOnToolLoadedOnly:
    """tool_loaded is the one state in which filament is genuinely OUT of the
    unit -- threaded through the hub into the toolhead -- and that is what an
    HT's interlock objects to.

    NOT loaded_to_hub: that is a STAGING state ("parked near the hub for a
    fast reload"), an intent this module sets and clears itself, with the
    filament still inside the unit. The first version of this check included
    it and warned about lane23 while the HT heated perfectly well."""

    def _lane(self, name, **kw):
        return types.SimpleNamespace(name=name, tool_loaded=False,
                                     loaded_to_hub=False, **kw)

    def _run(self, dev_addr, lane):
        shim, sent, _ = _heater_shim({"a": lane}, dev_addr=dev_addr)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        return sent, gcmd.info

    def test_a_toolhead_loaded_lane_warns(self):
        lane = self._lane("a"); lane.tool_loaded = True
        _sent, info = self._run(0x1800, lane)
        assert any("loaded to the toolhead" in m for m in info)

    def test_a_hub_STAGED_lane_does_NOT_warn(self):
        # The regression that prompted this: filament is still in the unit.
        lane = self._lane("a"); lane.loaded_to_hub = True
        _sent, info = self._run(0x1800, lane)
        assert not any("loaded to the toolhead" in m for m in info)

    def test_a_lane_in_its_bay_does_not_warn(self):
        _sent, info = self._run(0x1800, self._lane("a"))
        assert not any("loaded to the toolhead" in m for m in info)

    def test_a_boxed_unit_is_not_warned_even_when_loaded(self):
        # An ACE/ACE2 heats while printing; the AMS 2 Pro is untested, and its
        # own refusal would surface anyway.
        lane = self._lane("a"); lane.tool_loaded = True
        _sent, info = self._run(0x0700, lane)
        assert not any("loaded to the toolhead" in m for m in info)

    def test_the_command_is_still_sent(self):
        # WARN, not block -- a unit whose interlock differs must stay dryable.
        lane = self._lane("a"); lane.tool_loaded = True
        sent, _info = self._run(0x1800, lane)
        assert sent and sent[0]["temp"] == 55

    def test_rotate_gating_still_uses_committed_lanes(self):
        # Unaffected and must stay: spinning a spool whose filament is in the
        # path fights it, which is mechanical fact rather than an interlock
        # guess -- and THAT one does care about hub staging.
        lane = self._lane("a"); lane.loaded_to_hub = True
        shim, sent, _ = _heater_shim({"a": lane}, dev_addr=0x1800)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 1})
        afcBambuAMS.cmd_AFC_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 0
        assert any("ROTATE disabled" in m for m in gcmd.info)


class TestFaultHoldCannotLatchForever:
    """The hold suppresses the follower auto-arm after a stall, and releases
    when the operator resumes -- which is them saying the jam is cleared.

    Outside a print there is no pause and therefore no resume, so waiting for
    one latched the follower off for the life of the object. Seen on hardware
    as a follower that "stopped working" with no way back short of
    AFC_BAMBU_FOLLOWER ENABLE=1 or a fresh load, and easily mistaken for state
    surviving a restart -- it is not saved anywhere, it was simply being re-set
    each time the follower armed into de-energised gears and stalled again."""

    def _u(self, paused=False, printing=False, saw_pause=False):
        return types.SimpleNamespace(
            name="BambuAMS_1", logger=_Logger(),
            _follow_fault_hold=True,
            _follow_fault_saw_pause=saw_pause,
            afc=types.SimpleNamespace(function=types.SimpleNamespace(
                is_paused=lambda: paused, in_print=lambda: printing)))

    def test_no_hold_set_is_not_active(self):
        u = self._u()
        u._follow_fault_hold = False
        assert afcBambuAMS._fault_hold_active(u) is False

    def test_a_fault_while_idle_releases_instead_of_latching(self):
        # THE bug: not printing, so no pause can ever arrive.
        u = self._u(paused=False, printing=False)
        assert afcBambuAMS._fault_hold_active(u) is False
        assert u._follow_fault_hold is False

    def test_a_fault_mid_print_still_waits_for_the_pause(self):
        # Unchanged where the original reasoning holds: AFC_error queues the
        # pause, so releasing before it lands would re-arm into the jam.
        u = self._u(paused=False, printing=True)
        assert afcBambuAMS._fault_hold_active(u) is True
        assert u._follow_fault_hold is True

    def test_while_paused_it_holds_and_remembers(self):
        u = self._u(paused=True, printing=True)
        assert afcBambuAMS._fault_hold_active(u) is True
        assert u._follow_fault_saw_pause is True

    def test_resume_after_a_seen_pause_releases(self):
        u = self._u(paused=False, printing=True, saw_pause=True)
        assert afcBambuAMS._fault_hold_active(u) is False
        assert u._follow_fault_hold is False

    def test_an_unknown_print_state_keeps_holding(self):
        # Fail safe: if we cannot tell whether a print is running, the held
        # behaviour is the conservative one.
        def boom():
            raise RuntimeError("no such object")
        u = self._u()
        u.afc.function.in_print = boom
        assert afcBambuAMS._fault_hold_active(u) is True

    def test_the_release_says_which_case_it_was(self):
        u = self._u(paused=False, printing=False)
        afcBambuAMS._fault_hold_active(u)
        assert any("no print to resume" in m for _l, m in u.logger.messages)


class TestTheFollowerIsNotPerModel:
    """
    Every unit is held the same way -- op-04 07/7F at 148 ms, the cadence a
    real printer uses -- with no buffer deadband to tune.

    There is no per-model distinction to configure: measured on a three-unit
    bus, a regular AMS sits at 0.56-0.59 on the virtual FPS, indistinguishable
    from an HT.

    Guards against self_centres / follow_always returning as config reads, and
    against the announce sending a {"cmd":"selfc"} the firmware has no reader
    for.
    """

    def test_the_options_are_gone_from_the_module(self):
        import inspect
        from extras import AFC_BambuAMS as m
        src = inspect.getsource(m)
        for dead in ('config.getboolean("self_centres"',
                     'config.getboolean("follow_always"',
                     "_send_selfcentre_flag"):
            assert dead not in src, f"{dead} is still read"

    def test_the_announce_no_longer_sends_selfc(self):
        from extras.AFC_BambuAMS_bridge import _BRIDGE_EVENTS_KNOWN
        assert "selfc" not in _BRIDGE_EVENTS_KNOWN

class TestPathAdoptionNeedsOnlyOneLoad:
    """The unit narrates tube_len at the END of a load, so an adoption that
    only runs at the START can never see the load it is part of -- it adopts
    the PREVIOUS one. That made it need two loads in a single Klipper session,
    and the measurement lives only in the bridge's memory, so a restart in
    between reset it.

    Observed: an AMS 2 reported tube_len:3532 mm on two consecutive loads and
    stayed on the 3000 mm default, because a deploy landed between them."""

    def _shim(self, measured, bowden=3000.0):
        writes = []
        shim = types.SimpleNamespace(
            name="BambuAMS_2", logger=_Logger(),
            afc_bowden_length=bowden, afc_unload_bowden_length=bowden,
            full_name=("AFC_BambuAMS", "BambuAMS_2"),
            measured_path_mm=lambda: measured,
            _measure_path_from_odom=lambda: None,
            _dw_len_mm=lambda: None,
            afc=types.SimpleNamespace(function=types.SimpleNamespace(
                ConfigRewrite=lambda sec, key, val, msg=None:
                    writes.append((key, val)))))
        shim._path_measurement = lambda: afcBambuAMS._path_measurement(shim)
        return shim, writes

    def test_a_fresh_measurement_is_adopted(self):
        shim, writes = self._shim(3532.0)
        afcBambuAMS._adopt_measured_path(shim)
        assert ("afc_bowden_length", 3532.0) in writes
        assert shim.afc_bowden_length == 3532.0

    def test_no_measurement_yet_writes_nothing(self):
        # What the start-of-load call sees on a fresh Klipper session.
        shim, writes = self._shim(None)
        afcBambuAMS._adopt_measured_path(shim)
        assert writes == []
        assert shim.afc_bowden_length == 3000.0

    def test_it_is_safe_to_call_twice(self):
        # It now runs at both ends of a load; the second must be a no-op.
        shim, writes = self._shim(3532.0)
        afcBambuAMS._adopt_measured_path(shim)
        afcBambuAMS._adopt_measured_path(shim)
        assert len(writes) == 2          # bowden + unload bowden, once each

    def test_a_figure_within_tolerance_does_not_rewrite(self):
        # The unit's measurement moves a few mm between calibrations.
        shim, writes = self._shim(3005.0, bowden=3000.0)
        afcBambuAMS._adopt_measured_path(shim)
        assert writes == []

    def test_the_unload_length_follows_when_it_was_defaulted(self):
        shim, writes = self._shim(3532.0)
        afcBambuAMS._adopt_measured_path(shim)
        assert shim.afc_unload_bowden_length == 3532.0


class TestBufferChipIsPerBusMaster:
    """The virtual buffer ADC is one chip per BUS MASTER, not per printer.

    Units on a Pico share one buffer and one extruder, so they share the chip.
    A second Pico is a second buffer feeding a second extruder and must read
    its own -- registering a single printer-wide chip gave every
    `bambu_buffer:` pin whichever unit initialised first, which under
    `tool_start: buffer` is the toolhead authority reading the wrong bus."""

    def _printer(self):
        chips = {}
        pins = types.SimpleNamespace(
            register_chip=lambda name, chip: chips.setdefault(name, chip))
        return types.SimpleNamespace(
            lookup_object=lambda n, d=None: pins,
            register_event_handler=lambda *a: None,
            config_error=RuntimeError), chips

    def _unit(self, printer, name, buff=0.5):
        return types.SimpleNamespace(
            printer=printer, buffer_chip_name=name,
            fps_buffer_value=lambda: buff)

    def test_units_sharing_a_master_share_one_chip(self):
        pr, chips = self._printer()
        a = self._unit(pr, "bambu_buffer")
        b = self._unit(pr, "bambu_buffer")
        afcBambuAMS_mod._register_bambu_buffer_chip(a)
        afcBambuAMS_mod._register_bambu_buffer_chip(b)
        assert list(chips) == ["bambu_buffer"]
        assert len(pr._bambu_buffer_chips) == 1

    def test_a_second_master_gets_its_own_chip(self):
        pr, chips = self._printer()
        afcBambuAMS_mod._register_bambu_buffer_chip(
            self._unit(pr, "bambu_buffer", buff=0.2))
        afcBambuAMS_mod._register_bambu_buffer_chip(
            self._unit(pr, "bambu_buffer_2", buff=0.8))
        assert sorted(chips) == ["bambu_buffer", "bambu_buffer_2"]

    def test_each_chip_reads_its_own_unit(self):
        # The failure this prevents: a second bus reporting the first's value.
        pr, chips = self._printer()
        afcBambuAMS_mod._register_bambu_buffer_chip(
            self._unit(pr, "bambu_buffer", buff=0.2))
        afcBambuAMS_mod._register_bambu_buffer_chip(
            self._unit(pr, "bambu_buffer_2", buff=0.8))
        assert chips["bambu_buffer"]._unit.fps_buffer_value() == 0.2
        assert chips["bambu_buffer_2"]._unit.fps_buffer_value() == 0.8

    def test_the_default_name_is_unchanged(self):
        # Existing configs say `adc_pin: bambu_buffer:fps` and must keep working.
        assert afcBambuAMS_mod._BUFFER_CHIP_NAME == "bambu_buffer"


class TestFollowArmAcked:
    """
    The per-unit receipt for the follower arm.

    The arm frame is never answered, so the bridge infers acknowledgement from
    the unit narrating ``state:4`` and reports it as an ``armack`` bitmask. The
    thing worth testing on the host side is that we read OUR bit and that we do
    not turn "the firmware never said" into "not acknowledged" -- a silent arm
    and an old firmware look identical once both collapse to False.
    """

    def _unit(self, index):
        return types.SimpleNamespace(
            ams_index=index,
            _follow_arm_acked=afcBambuAMS_mod.afcBambuAMS._follow_arm_acked)

    def _acked(self, index, latest):
        u = self._unit(index)
        return u._follow_arm_acked(u, latest)

    def test_reads_this_units_bit(self):
        assert self._acked(0, {"armack": 0b0001}) is True
        assert self._acked(1, {"armack": 0b0001}) is False
        assert self._acked(1, {"armack": 0b0010}) is True
        assert self._acked(2, {"armack": 0b0110}) is True

    def test_unknown_when_the_firmware_does_not_report(self):
        # Not False: an arm that never lands and a firmware that cannot say so
        # are different problems, and only one of them is a bug to chase.
        assert self._acked(0, {}) is None
        assert self._acked(0, {"armack": None}) is None
        assert self._acked(0, {"armack": "3"}) is None

    def test_unknown_with_no_status_frame(self):
        assert self._acked(0, None) is None
        assert self._acked(0, {}) is None


class TestMeasureOnInsertToggle:
    """measure_on_insert is pushed to the FIRMWARE, not branched on here.

    cap_open() is the single door into the measurement window for every unit
    type: a boxed unit reaches it from bb_do_capscan_ex, an AMS HT from
    ht_scan_arm() on the insert edge -- where the module is not involved at
    all. Gating in the module would therefore cover boxed units only, which is
    exactly the bug this replaced.
    """

    def _unit(self, measure):
        sent = []
        u = types.SimpleNamespace(
            name="u", ams_index=2, measure_on_insert=measure,
            ht_0f_hold=False, has_heater=False, dry_dev_addr=0x0700,
            _is_ht=lambda: False,
        )
        return u, types.SimpleNamespace(send=lambda o: sent.append(o)), sent

    def test_on_pushes_capen_1(self):
        u, bridge, sent = self._unit(True)
        afcBambuAMS._send_ht_flag(u, bridge)
        assert {"cmd": "capen", "unit": 2, "on": 1} in sent

    def test_off_pushes_capen_0(self):
        u, bridge, sent = self._unit(False)
        afcBambuAMS._send_ht_flag(u, bridge)
        assert {"cmd": "capen", "unit": 2, "on": 0} in sent

    def test_an_ht_gets_the_flag_too(self):
        """The whole point: an HT is gated the same way as a boxed unit."""
        u, bridge, sent = self._unit(False)
        u._is_ht = lambda: True
        u.has_heater, u.dry_dev_addr = True, 0x1800
        afcBambuAMS._send_ht_flag(u, bridge)
        assert {"cmd": "capen", "unit": 2, "on": 0} in sent

    def test_default_is_on(self):
        u, bridge, sent = self._unit(True)
        del u.measure_on_insert          # an object predating the option
        afcBambuAMS._send_ht_flag(u, bridge)
        assert {"cmd": "capen", "unit": 2, "on": 1} in sent


class TestUnitStateFaultDetector:
    """op-04 reply byte[19] is the one fault signal every unit sends.

    Measured 2026-08-05 by stuck-spooling each unit alone on the wire: all
    three set it to 0x07, and on the two that also narrate it reads 0x07 at
    exactly the moment they print "state:7". An AMS 1 sets it while emitting no
    fault text at all -- so narration alone leaves that unit undetected.
    """

    def _unit(self, ustate, idx=1):
        raised = []
        units = [{"n": 0, "ustate": 4}]
        if ustate is not None:
            units.append({"n": idx, "ustate": ustate})
        u = types.SimpleNamespace(
            name="u", ams_index=idx, fault_detect=True,
            _unload_in_progress=False, _drying=False, _stalled_seen=False,
            _bridge=types.SimpleNamespace(latest_status=lambda: {"units": units}),
            afc=types.SimpleNamespace(
                function=types.SimpleNamespace(in_print=lambda: False)),
            _raise_ams_fault=lambda lane, msg: raised.append(msg),
        )
        u._unit_state = afcBambuAMS._unit_state.__get__(u)
        u.AMS_STATE_STALLED = afcBambuAMS.AMS_STATE_STALLED
        return u, raised

    def _run(self, u):
        return afcBambuAMS._check_unit_stalled(u, types.SimpleNamespace(name="lane1"))

    def test_stalled_state_raises(self):
        u, raised = self._unit(0x07)
        assert self._run(u) is True
        assert "STALLED" in raised[0] and "lane1" in raised[0]

    def test_healthy_state_does_not(self):
        u, raised = self._unit(0x04)
        assert self._run(u) is False and raised == []

    def test_transitional_states_do_not(self):
        """02/00 churn while the printer retries -- not a fault."""
        for st in (0x00, 0x02, 0x03):
            u, raised = self._unit(st)
            assert self._run(u) is False, st
            assert raised == []

    def test_not_heard_from_is_not_healthy_and_is_not_a_fault(self):
        """255 means the firmware has committed no state. Judge nothing."""
        u, raised = self._unit(0xFF)
        assert self._run(u) is False and raised == []
        assert u._unit_state(u._bridge.latest_status()) is None

    def test_only_this_units_state_is_read(self):
        """Unit 0 stalling must not fault unit 1."""
        u, raised = self._unit(0x04, idx=1)
        u._bridge.latest_status = lambda: {
            "units": [{"n": 0, "ustate": 0x07}, {"n": 1, "ustate": 0x04}]}
        assert self._run(u) is False and raised == []

    def test_one_fault_per_stall_not_one_per_tick(self):
        u, raised = self._unit(0x07)
        self._run(u); self._run(u); self._run(u)
        assert len(raised) == 1

    def test_it_rearms_after_recovery(self):
        u, raised = self._unit(0x07)
        assert self._run(u) is True
        u._bridge.latest_status = lambda: {"units": [{"n": 1, "ustate": 0x04}]}
        assert self._run(u) is False          # clears the latch
        u._bridge.latest_status = lambda: {"units": [{"n": 1, "ustate": 0x07}]}
        assert self._run(u) is True           # a NEW stall reports again
        assert len(raised) == 2

    def test_stands_down_during_unload_and_drying(self):
        for attr in ("_unload_in_progress", "_drying"):
            u, raised = self._unit(0x07)
            setattr(u, attr, True)
            assert self._run(u) is False, attr
            assert raised == []

    def test_respects_fault_detect_off(self):
        u, raised = self._unit(0x07)
        u.fault_detect = False
        assert self._run(u) is False and raised == []


class TestHTInsertMarksThePendingSlot:
    """An HT insert must mark the slot, or its measurement is discarded.

    The HT's scan AND capacity window are both armed in firmware on the insert
    edge (ht_scan_arm -> cap_open), so the module sends nothing -- and it was
    therefore never setting _cap_pending_slot, which the entire apply path is
    gated on. Observed live: the unit reported "odom C:0.522,R:0.083,P:102%"
    and "Calibration rst:0" while the lane never changed.
    """

    def _unit(self, is_ht):
        u = types.SimpleNamespace(
            name="u", _is_ht=lambda: is_ht,
            _cap_pending_slot=None, _cap_pending_t0=0.0,
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: 123.0)),
            logger=types.SimpleNamespace(debug=lambda *a, **k: None),
        )
        return u

    def test_ht_insert_marks_the_slot(self):
        u = self._unit(True)
        _ht_insert_branch(u, 0)
        assert u._cap_pending_slot == 0
        assert u._cap_pending_t0 == 123.0

    def test_a_bad_clock_does_not_break_the_marker(self):
        u = self._unit(True)
        u.afc.reactor.monotonic = lambda: (_ for _ in ()).throw(RuntimeError())
        _ht_insert_branch(u, 0)
        assert u._cap_pending_slot == 0      # still marked
        assert u._cap_pending_t0 == 0.0


def _ht_insert_branch(unit, slot):
    """The HT half of the insert edge, lifted from _start_tag_scan."""
    if unit._is_ht():
        unit._cap_pending_slot = slot
        try:
            unit._cap_pending_t0 = unit.afc.reactor.monotonic()
        except Exception:
            unit._cap_pending_t0 = 0.0
        unit.logger.debug("ht")


class TestChainResolveIsQuietUntilItIsStuck:
    """Holding registrations until the UID resolves is CORRECT, not an error.

    It happens at every boot -- the chain map has not arrived yet -- and the
    hold is the entire point of resolving by UID rather than filing
    registrations against the config default. Warning about it every start
    trains the operator to ignore the log. What deserves a warning is a UID
    that NEVER resolves, because then the unit is not on the bus at all and
    its registrations were never sent.
    """

    def _unit(self, deferred=True, resolved=False, t0=100.0):
        warns, debugs = [], []
        u = types.SimpleNamespace(
            name="u", unit_uid="A9CD393238310D0030383131",
            _announce_deferred=deferred, _id_resolved=resolved,
            _announce_defer_t0=t0, _announce_defer_warned=False,
            CHAIN_RESOLVE_WARN_S=afcBambuAMS.CHAIN_RESOLVE_WARN_S,
            logger=types.SimpleNamespace(
                warning=lambda m: warns.append(m),
                debug=lambda *a, **k: debugs.append(a)),
        )
        return u, warns

    def _run(self, u, now):
        afcBambuAMS._check_chain_resolve(u, now)

    def test_silent_while_it_is_still_early(self):
        u, warns = self._unit()
        self._run(u, 100.5)          # half a second in
        self._run(u, 120.0)          # 20s in
        assert warns == []

    def test_warns_once_when_genuinely_stuck(self):
        u, warns = self._unit()
        self._run(u, 131.0)
        assert len(warns) == 1 and "not answering the bus" in warns[0]
        self._run(u, 200.0)          # and never again
        self._run(u, 999.0)
        assert len(warns) == 1

    def test_silent_once_resolved(self):
        u, warns = self._unit(deferred=False, resolved=True)
        self._run(u, 999.0)
        assert warns == []

    def test_silent_with_no_hold_recorded(self):
        """No start time means nothing to measure against -- judge nothing."""
        u, warns = self._unit(t0=0.0)
        self._run(u, 999.0)
        assert warns == []


class TestScanDoesNotRetriggerItself:
    """A scan moves filament off the bay switch -- that is not a removal.

    Observed live on an AMS 2 bay 3: scan -> filament retracts past the switch
    -> "spool REMOVED" -> filament returns -> "spool INSERTED" -> scan again,
    repeating every ~30s indefinitely. The scan was causing the edge that
    started the next scan.
    """

    def _unit(self, started=None, now=100.0):
        logs = []
        u = types.SimpleNamespace(
            name="u", SCAN_FALLBACK_CAP=afcBambuAMS.SCAN_FALLBACK_CAP,
            _scan_t0=[None]*4, _scan_motion_t0=[started, None, None, None],
            # THE REAL CLASS CONSTANTS, not stand-ins. Hardcoding them here is
            # what let SCAN_MOTION_QUIET_S be referenced by _scan_in_flight and
            # never defined on the class: the shim supplied it, the tests
            # passed, and on hardware every status frame for a unit whose scan
            # outlived its narration died on an AttributeError inside
            # _sync_lanes -- no slot data, no lane sync, for the whole unit.
            SCAN_MOTION_QUIET_S=afcBambuAMS.SCAN_MOTION_QUIET_S,
            _prev_present=[True, False, False, False],
            unit_slots=4, _scan_primed=True, _auto_scanned=[False]*4,
            _scan_notag=[False]*4,
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: now)),
            logger=types.SimpleNamespace(info=lambda m: logs.append(m),
                                         debug=lambda *a, **k: None),
            # the removal path touches these
            _lane_for_slot=lambda s: None,
            _clear_lane_filament=lambda ln: None,
            _measured_remain={}, _scan_defer=[False]*4,
            _release_scan_hold=lambda s: None,
            _bridge=types.SimpleNamespace(last_scan_end=lambda: None),
        )
        u._scan_in_flight = afcBambuAMS._scan_in_flight.__get__(u)
        return u, logs

    def test_mid_scan_presence_drop_is_not_a_removal(self):
        u, logs = self._unit(started=90.0, now=100.0)      # 10s into a scan
        assert u._scan_in_flight(0) is True
        afcBambuAMS._maybe_auto_scan(u, 0, False, {})
        assert logs == [], "a scan's own retract must not log a removal"
        assert u._prev_present[0] is False                 # still tracked

    def test_the_window_expires_so_a_real_removal_still_lands(self):
        u, logs = self._unit(started=1.0, now=200.0)       # 199s -- long over
        assert u._scan_in_flight(0) is False
        afcBambuAMS._maybe_auto_scan(u, 0, False, {})
        assert any("REMOVED" in m for m in logs)

    def test_no_scan_running_behaves_normally(self):
        u, logs = self._unit(started=None)
        assert u._scan_in_flight(0) is False
        afcBambuAMS._maybe_auto_scan(u, 0, False, {})
        assert any("REMOVED" in m for m in logs)

    def test_other_slots_are_unaffected_by_this_slots_scan(self):
        u, logs = self._unit(started=90.0, now=100.0)
        assert u._scan_in_flight(1) is False    # only slot 0 is scanning
        u._prev_present[1] = True
        afcBambuAMS._maybe_auto_scan(u, 1, False, {})
        assert any("REMOVED" in m and "slot 1" in m for m in logs)

    def test_a_successful_read_does_not_end_the_motion_guard(self):
        """The bug this replaced.

        _release_scan_hold clears _scan_t0 the moment the tag reads -- which is
        BEFORE the unit retracts the filament. Hanging the guard on that
        timestamp left the retract unguarded, so the scan's own pull-back read
        as a removal and started the next scan.
        """
        u, logs = self._unit(started=90.0, now=100.0)
        u._scan_t0[0] = None                    # read succeeded, hold released
        assert u._scan_in_flight(0) is True, "motion guard must outlive the read"
        afcBambuAMS._maybe_auto_scan(u, 0, False, {})
        assert logs == []

    def test_the_units_own_end_marker_releases_the_guard(self):
        """The unit announces its cycle end -- that ends the guard, not a timer.

        "Calibration rst:0" (HT), "odom calib success exit 0" (AMS 1),
        "STEP7:cali end" (AMS 2). Waiting for the announcement is exact; a
        timer is wrong short (the scan retriggers itself) or wrong long (a real
        removal goes unnoticed).
        """
        u, logs = self._unit(started=90.0, now=100.0)
        assert u._scan_in_flight(0) is True          # mid-cycle, still guarding
        u._bridge.last_scan_end = lambda: 95.0       # unit says it finished
        assert u._scan_in_flight(0) is False
        afcBambuAMS._maybe_auto_scan(u, 0, False, {})
        assert any("REMOVED" in m for m in logs), "a real removal must land now"

    def test_an_end_from_BEFORE_this_scan_does_not_release_it(self):
        """A previous cycle's end must not end the current one."""
        u, logs = self._unit(started=90.0, now=100.0)
        u._bridge.last_scan_end = lambda: 50.0       # older than this scan
        assert u._scan_in_flight(0) is True

    def test_a_broken_clock_does_not_wedge_it(self):
        u, logs = self._unit(started=90.0)
        u.afc.reactor.monotonic = lambda: (_ for _ in ()).throw(RuntimeError())
        assert u._scan_in_flight(0) is False    # fail open, edges keep working


class TestSlotsWaitForIdentity:
    """A unit must not apply slot data before it knows which slots are its own.

    With unit_uid configured, ams_index is the config default (0) until the
    chain map resolves the UID. Every unit on the bus therefore matches unit
    0's slots. Observed live: one HT tag applied to lane15, lane19 and lane23
    in the same instant, because all three units were still at index 0.
    """

    def _unit(self, uid, resolved, idx=0):
        applied = []
        u = types.SimpleNamespace(
            name="u", unit_uid=uid, _id_resolved=resolved, ams_index=idx,
            SLOTS_PER_UNIT=4, _slots=[None]*4,
            _sync_lanes=lambda: applied.append("synced"),
            _status_err_last=None,
            logger=types.SimpleNamespace(warning=lambda m: None,
                                         debug=lambda m: None),
        )
        return u, applied

    FRAME = {"slots": [{"unit": 0, "i": 0, "material": "PLA"}]}

    def test_unresolved_uid_applies_nothing(self):
        u, applied = self._unit("A9CD39", resolved=False)
        afcBambuAMS._on_status(u, self.FRAME)
        assert applied == [], "must not adopt another unit's slots"

    def test_resolved_uid_applies_normally(self):
        u, applied = self._unit("A9CD39", resolved=True)
        afcBambuAMS._on_status(u, self.FRAME)
        assert applied == ["synced"]

    def test_no_uid_configured_is_unaffected(self):
        """Without unit_uid the index is authoritative from the start."""
        u, applied = self._unit(None, resolved=False)
        afcBambuAMS._on_status(u, self.FRAME)
        assert applied == ["synced"]


class TestEmptyBaysDoNotKeepFilament:
    """A bay the unit reports EMPTY must not leave data on its lane.

    Two ways it used to survive, both seen live:
      - restored from saved vars at boot, with no insert/removal edge to fix it
        (AMS 1 bays 2 and 4 showed filament, one bound to spool 130)
      - a Spoolman-linked lane was skipped on removal as "authoritative", so it
        kept claiming filament AND blocked the next real tag from applying
        (lane23 showed another unit's colour while bound to spool 124)
    """

    def _unit(self, slots, lanes):
        cleared, unbound, logs = [], [], []
        u = types.SimpleNamespace(
            name="u", unit_slots=4, SLOTS_PER_UNIT=4,
            _prev_present=[False]*4, _slots=slots,
            _lane_for_slot=lambda s: lanes.get(s),
            _clear_lane_filament=lambda ln: cleared.append(ln.name),
            logger=types.SimpleNamespace(info=lambda m: logs.append(m),
                                         debug=lambda *a, **k: None),
        )
        u._unbind_spool = afcBambuAMS._unbind_spool.__get__(u)
        u._reconcile_empty_bays = afcBambuAMS._reconcile_empty_bays.__get__(u)
        return u, cleared, logs

    def _lane(self, name, material=None, spool=None):
        return types.SimpleNamespace(name=name, material=material,
                                     spool_id=spool)

    def test_empty_bay_with_stale_material_is_cleared(self):
        lanes = {1: self._lane("lane16", material="PLA")}
        u, cleared, _ = self._unit([None]*4, lanes)
        u._reconcile_empty_bays()
        assert cleared == ["lane16"]

    def test_empty_bay_still_bound_to_spoolman_is_unbound(self):
        lanes = {3: self._lane("lane18", material="PLA", spool=130)}
        u, cleared, _ = self._unit([None]*4, lanes)
        u._reconcile_empty_bays()
        assert cleared == ["lane18"]
        assert lanes[3].spool_id == ''

    def test_a_present_bay_is_left_alone(self):
        slots = [{"present": True}, None, None, None]
        lanes = {0: self._lane("lane15", material="PLA", spool=124)}
        u, cleared, _ = self._unit(slots, lanes)
        u._reconcile_empty_bays()
        assert cleared == [] and lanes[0].spool_id == 124

    def test_an_already_clean_empty_bay_says_nothing(self):
        lanes = {2: self._lane("lane17")}
        u, cleared, logs = self._unit([None]*4, lanes)
        u._reconcile_empty_bays()
        assert cleared == [] and logs == []

    def test_unbind_is_a_noop_when_there_is_no_binding(self):
        lane = self._lane("lane17")
        u, _, _ = self._unit([None]*4, {})
        u._unbind_spool(lane)
        assert lane.spool_id is None      # untouched, not blanked

    def test_phantom_bays_beyond_unit_slots_are_ignored(self):
        """A 1-slot HT must not have bays 2-4 reconciled."""
        lanes = {s: self._lane("lane%d" % s, material="PLA") for s in range(4)}
        u, cleared, _ = self._unit([None]*4, lanes)
        u.unit_slots = 1
        u._reconcile_empty_bays()
        assert cleared == ["lane0"]


class TestSpoolmanMissIsNotRetriedForever:
    """A UID Spoolman does not know is a permanent answer, not a retry.

    This path runs on every status pass. Without a memo, a spool whose tag has
    no Spoolman entry re-queries Spoolman at 1 Hz forever -- a blocking HTTP
    call on the reactor. Observed live: ~20 minutes of "no Spoolman spool
    matches UID ECB61CD0", 1061 "Resetting prediction variance" events as the
    host lost its MCU clock, then "MCU 'mcu' shutdown: Timer too close" and
    every MCU down. The lookup did not fail -- it succeeded, and the answer was
    "no match".
    """

    def _unit(self):
        calls = []
        u = types.SimpleNamespace(
            name="u", SLOTS_PER_UNIT=4,
            _slots=[{"uid": "ECB61CD0"}, None, None, None],
            _spoolman_no_match=set(),
        )
        u._forget_spoolman_miss = afcBambuAMS._forget_spoolman_miss.__get__(u)
        return u, calls

    def test_a_miss_is_remembered(self):
        u, _ = self._unit()
        u._spoolman_no_match.add("ECB61CD0")
        assert "ECB61CD0" in u._spoolman_no_match

    def test_removal_forgets_the_miss(self):
        """The next spool -- or this one after being added -- must re-check."""
        u, _ = self._unit()
        u._spoolman_no_match.add("ECB61CD0")
        u._forget_spoolman_miss(0)
        assert "ECB61CD0" not in u._spoolman_no_match

    def test_forgetting_clears_the_whole_set(self):
        # Deliberately the WHOLE set, not this slot's UID. The per-UID variant
        # read info["uid"] -- a key the normalized dict never has (it is
        # "rfid_uid") -- so it cleared nothing for the entire time it existed;
        # and a power-cycled unit blanks the record, leaving a memoized UID
        # nobody can name anymore. A whole-set clear on a physical edge costs
        # one memoized re-lookup per still-unmatched bay and is what lets
        # "bind it in Spoolman, re-insert it" actually work.
        u, _ = self._unit()
        u._spoolman_no_match.add("ECB61CD0")
        u._forget_spoolman_miss(1)                 # ANY slot's edge clears
        assert not u._spoolman_no_match

    def test_a_different_uid_is_not_suppressed(self):
        """Keyed by UID, so another spool still gets its own lookup."""
        u, _ = self._unit()
        u._spoolman_no_match.add("ECB61CD0")
        assert "13F56D32" not in u._spoolman_no_match

    def test_forget_survives_a_missing_memo(self):
        u, _ = self._unit()
        del u._spoolman_no_match
        u._forget_spoolman_miss(0)                 # must not raise


class TestNoUndefinedNames:
    """A NameError in __init__ halts Klipper at connect, and nothing else here
    catches it.

    Every test in this file calls module functions against hand-built shims, so
    the constructor is never executed and an undefined local in it is invisible:
    on 2026-08-06 a comment-block edit deleted the line computing `_is_ht` and
    the suite reported the same 25 failures before, during and after. The
    printer halted with "name '_is_ht' is not defined" and had to be recovered
    by hand, because a halted Klippy cannot run the g-code that repairs it.

    Constructing the real class needs most of Klipper mocked, so this checks the
    same class of defect statically instead. It is not a substitute for a
    construction test; it is the cheap guard that would have caught this one.
    """

    def _check(self, path):
        import subprocess
        import sys
        out = subprocess.run([sys.executable, "-m", "pyflakes", path],
                             capture_output=True, text=True)
        return [ln for ln in (out.stdout + out.stderr).splitlines()
                if "undefined name" in ln]

    def test_the_unit_module_has_no_undefined_names(self):
        import os
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        assert self._check(os.path.join(here, "extras/AFC_BambuAMS.py")) == []

    def test_the_bridge_module_has_no_undefined_names(self):
        import os
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        assert self._check(
            os.path.join(here, "extras/AFC_BambuAMS_bridge.py")) == []



class _GErr(Exception):
    pass


class TestReloadBeforeResume:
    """The ordinary resume button reloads a lane a Bambu fault emptied -- and
    refuses to continue if the reload does not take.

    Two failures this encodes, both of which happened:
      * resuming into an empty toolhead after a fault-park (the print carried
        on extruding nothing)
      * a recovery that reported success without checking
    """

    def _shim(self, *, paused=True, target=True, loaded=False,
              loads_ok=True):
        scripts, said, logged = [], [], []
        lane = types.SimpleNamespace(name="lane15", tool_loaded=loaded)
        unit = types.SimpleNamespace(name="BambuAMS_1",
                                     _resume_needs_reload=True,
                                     _auto_recover_armed=True)

        def _run(script):
            scripts.append(script)
            if loads_ok:
                lane.tool_loaded = True

        shim = types.SimpleNamespace(
            name="BambuAMS_1",
            printer=types.SimpleNamespace(command_error=_GErr),
            gcode=types.SimpleNamespace(run_script_from_command=_run),
            afc=types.SimpleNamespace(
                function=types.SimpleNamespace(is_paused=lambda: paused)),
            logger=types.SimpleNamespace(warning=lambda m: logged.append(m),
                                         debug=lambda m: None),
            _resume_reload_target=lambda: (
                (unit, lane) if target else (None, None)))
        gcmd = types.SimpleNamespace(
            respond_info=lambda m: said.append(m),
            error=lambda m: _GErr(m),
            get_raw_command_parameters=lambda: "")
        return shim, gcmd, lane, unit, scripts, said

    def test_an_ordinary_resume_is_untouched(self):
        # Not paused: this runs on EVERY resume on the printer, including ones
        # with nothing to do with an AMS. Silence is the contract.
        shim, gcmd, _l, _u, scripts, said = self._shim(paused=False)
        afcBambuAMS._reload_before_resume(shim, gcmd)
        assert scripts == [] and said == []

    def test_no_pending_fault_is_untouched(self):
        shim, gcmd, _l, _u, scripts, said = self._shim(target=False)
        afcBambuAMS._reload_before_resume(shim, gcmd)
        assert scripts == [] and said == []

    def test_an_already_loaded_lane_is_not_reloaded(self):
        shim, gcmd, _l, unit, scripts, _s = self._shim(loaded=True)
        afcBambuAMS._reload_before_resume(shim, gcmd)
        assert scripts == []
        assert unit._resume_needs_reload is False

    def test_a_faulted_lane_is_reloaded_before_the_resume(self):
        shim, gcmd, _l, unit, scripts, said = self._shim()
        afcBambuAMS._reload_before_resume(shim, gcmd)
        assert scripts == ["CHANGE_TOOL LANE=lane15"]
        assert unit._resume_needs_reload is False
        assert any("lane15" in m for m in said)

    def test_a_reload_that_does_not_take_refuses_to_resume(self):
        shim, gcmd, _l, unit, scripts, _s = self._shim(loads_ok=False)
        with pytest.raises(_GErr):
            afcBambuAMS._reload_before_resume(shim, gcmd)
        assert scripts == ["CHANGE_TOOL LANE=lane15"]
        # Still owed, so a second press tries again rather than giving up.
        assert unit._resume_needs_reload is True


class TestWrappedResumeDelegates:
    """Whatever happens in our half, RESUME must still resume -- unless we
    deliberately refused. A broken resume button cannot be recovered without
    restarting Klipper; a missed reload can always be done by hand."""

    def _shim(self, reload_impl):
        scripts, logged = [], []
        shim = types.SimpleNamespace(
            name="BambuAMS_1",
            printer=types.SimpleNamespace(command_error=_GErr),
            gcode=types.SimpleNamespace(
                run_script_from_command=lambda s: scripts.append(s)),
            logger=types.SimpleNamespace(warning=lambda m: logged.append(m),
                                         debug=lambda m: None),
            _reload_before_resume=reload_impl)
        gcmd = types.SimpleNamespace(
            respond_info=lambda m: None, error=lambda m: _GErr(m),
            get_raw_command_parameters=lambda: "")
        return shim, gcmd, scripts, logged

    def test_the_previous_handler_is_called(self):
        shim, gcmd, scripts, _l = self._shim(lambda g: None)
        afcBambuAMS.cmd_AFC_BAMBU_WRAPPED_RESUME(shim, gcmd)
        assert scripts == ["_AFC_BAMBU_RENAMED_RESUME_ "]

    def test_an_unexpected_error_still_resumes(self):
        def _boom(g):
            raise ValueError("bookkeeping went wrong")
        shim, gcmd, scripts, logged = self._shim(_boom)
        afcBambuAMS.cmd_AFC_BAMBU_WRAPPED_RESUME(shim, gcmd)
        assert scripts == ["_AFC_BAMBU_RENAMED_RESUME_ "]
        assert any("resuming anyway" in m for m in logged)

    def test_a_deliberate_refusal_does_not_resume(self):
        def _refuse(g):
            raise _GErr("did NOT reload")
        shim, gcmd, scripts, _l = self._shim(_refuse)
        with pytest.raises(_GErr):
            afcBambuAMS.cmd_AFC_BAMBU_WRAPPED_RESUME(shim, gcmd)
        assert scripts == []            # the print stays PAUSED


class TestAutoRecoveryNeverResumes:
    """Auto recovery restores the FILAMENT. Only a human restores the PRINT.

    This resumed on its own TWICE on hardware. The first time it was
    unconditional and restarted the print with nothing in the toolhead. That
    was made conditional on the lane coming back tool_loaded -- and the second
    time it resumed a correctly loaded lane, which was still wrong:

        57672.9  lane15 reached the toolhead sensor after 7 kick(s)
        57705.0  AFC_RESUME                       <- us, nobody asked

    "Verified before resuming" was never the requirement. The machine does not
    get to decide to restart somebody's print, and a real printer does not
    either -- it holds at the fault until a human presses continue.
    """

    def _shim(self, *, loaded, paused=True, declared=False):
        scripts, logged = [], []
        lane = types.SimpleNamespace(name="lane15", tool_loaded=loaded)
        reactor = types.SimpleNamespace(
            NEVER=float("inf"), monotonic=lambda: 0.0,
            register_callback=lambda cb, t: cb(0.0))
        shim = types.SimpleNamespace(
            name="BambuAMS_1", auto_error_recovery=True,
            _auto_recover_armed=False, _resume_needs_reload=True,
            lanes={"lane15": lane},
            AMS_STATE_STALLED=0x07,
            # The LATCH, not a fresh sample -- 0x07 shows up in only 11% of
            # frames during the park, so a single read misses it eight times
            # in nine. _on_status sets this; _raise_ams_fault clears it.
            _declared_since_fault=declared,
            _jam_location=lambda: "",
            _unit_state=lambda latest: 0x01,
            _bridge=types.SimpleNamespace(latest_status=lambda: {}),
            gcode=types.SimpleNamespace(
                run_script=lambda s: scripts.append(s)),
            afc=types.SimpleNamespace(
                reactor=reactor,
                function=types.SimpleNamespace(is_paused=lambda: paused)),
            logger=types.SimpleNamespace(info=lambda m: logged.append(m),
                                         warning=lambda m: logged.append(m),
                                         debug=lambda m: None))
        return shim, lane, scripts, logged

    def test_a_successful_reload_does_not_resume(self):
        shim, _lane, scripts, logged = self._shim(loaded=True)
        afcBambuAMS._maybe_auto_recover(shim, shim.lanes["lane15"])
        assert not any("RESUME" in s for s in scripts)
        assert any("STILL PAUSED" in m for m in logged)

    def test_it_still_runs_the_unload_and_reload(self):
        shim, _lane, scripts, _l = self._shim(loaded=True)
        afcBambuAMS._maybe_auto_recover(shim, shim.lanes["lane15"])
        assert scripts == ["TOOL_UNLOAD LANE=lane15\nCHANGE_TOOL LANE=lane15"]

    def test_a_successful_reload_clears_the_resume_debt(self):
        # The lane is fed, so the resume wrap must not reload it a second time.
        shim, _lane, _s, _l = self._shim(loaded=True)
        afcBambuAMS._maybe_auto_recover(shim, shim.lanes["lane15"])
        assert shim._resume_needs_reload is False

    def test_a_declared_unit_parks_without_resuming(self):
        shim, _lane, scripts, logged = self._shim(loaded=False, declared=True)
        afcBambuAMS._maybe_auto_recover(shim, shim.lanes["lane15"])
        assert not any("RESUME" in s for s in scripts)
        assert any("given up" in m for m in logged)


class TestAutoRecoveryIsOneAttempt:
    """The AMS retries the load by itself. By the time it gives up it is
    already held in error, and from that point it will not move again until it
    is TOLD to load -- which is what a human pressing resume sends.

    So there is nothing out here to retry. This was built as an unbounded 5 s
    loop and watched on hardware cycling unload/reload for over two minutes at
    a latched unit; it only ever "worked" at the moment a human freed the jam
    by hand. It was also a retry around a retry -- unit_load_lane already kicks
    23 times over two rounds, about 90 s, before reporting failure.
    """

    def _run_once(self, *, loaded, paused=True):
        rescheduled, logged = [], []
        lane = types.SimpleNamespace(name="lane15", tool_loaded=loaded)
        reactor = types.SimpleNamespace(
            NEVER=float("inf"), monotonic=lambda: 0.0,
            register_callback=lambda cb, t: rescheduled.append(cb(0.0)))
        shim = types.SimpleNamespace(
            name="BambuAMS_1", auto_error_recovery=True,
            _auto_recover_armed=False, _resume_needs_reload=True,
            lanes={"lane15": lane}, AMS_STATE_STALLED=0x07,
            _jam_location=lambda: "",
            _unit_state=lambda latest: 0x01,
            _bridge=types.SimpleNamespace(latest_status=lambda: {}),
            gcode=types.SimpleNamespace(run_script=lambda s: None),
            afc=types.SimpleNamespace(
                reactor=reactor,
                function=types.SimpleNamespace(is_paused=lambda: paused)),
            logger=types.SimpleNamespace(info=lambda m: logged.append(m),
                                         warning=lambda m: logged.append(m),
                                         debug=lambda m: None))
        afcBambuAMS._maybe_auto_recover(shim, lane)
        return rescheduled, logged

    def test_a_failed_reload_does_not_reschedule(self):
        rescheduled, _l = self._run_once(loaded=False)
        assert rescheduled == [float("inf")]     # NEVER, not eventtime + 5

    def test_a_failed_reload_says_the_unit_is_held(self):
        _r, logged = self._run_once(loaded=False)
        assert any("HELD IN ERROR" in m and "press resume" in m
                   for m in logged)

    def test_a_successful_reload_does_not_reschedule_either(self):
        rescheduled, _l = self._run_once(loaded=True)
        assert rescheduled == [float("inf")]


class TestDeclaredLatch:
    """byte[19] == 0x07 is the park signal and it is INTERMITTENT.

    Counted in the AMS 1 fault capture (ams1_print_fault_2026-08-05), by phase:

        HOLD (printing)   1333 frames    0 x 0x07
        RETRY             2686 frames   12 x 0x07   0.4%
        PARK              2523 frames  278 x 0x07  11.0%
        HOLD (after)      1333 frames    0 x 0x07

    It appears ONLY in the park, so the signal is sound -- but reading the
    CURRENT frame once, at the end of a ~90 s recovery attempt, misses it eight
    times in nine. That is what happened on hardware: the operator was looking
    at a unit latched red and our check never fired.
    """

    def _shim(self):
        return types.SimpleNamespace(
            name="BambuAMS_1", ams_index=0, unit_uid=None, _id_resolved=True,
            AMS_STATE_STALLED=0x07, _declared_since_fault=False,
            _follow_fault_hold=False, _odom_lo=None, _odom_hi=None,
            _track_odom=lambda o: None,
            SLOTS_PER_UNIT=4, _slots=[{} for _ in range(4)],
            _status_err_last=None,
            _unit_state=lambda o: afcBambuAMS._unit_state(
                types.SimpleNamespace(ams_index=0), o),
            _sync_lanes=lambda: None,
            logger=types.SimpleNamespace(warning=lambda m: None,
                                         debug=lambda m: None))

    @staticmethod
    def _frame(ustate):
        return {"units": [{"n": 0, "ustate": ustate}], "slots": []}

    def test_one_declaring_frame_in_a_run_of_quiet_ones_latches(self):
        # The measured shape: 8 quiet frames, 1 carrying 0x07, 8 more quiet.
        # Sampling the last frame would report "not declared".
        shim = self._shim()
        for st in [0x00] * 8 + [0x07] + [0x00] * 8:
            afcBambuAMS._on_status(shim, self._frame(st))
        assert shim._declared_since_fault is True

    def test_a_healthy_run_never_latches(self):
        shim = self._shim()
        for st in (0x00, 0x01, 0x02, 0x03, 0x04, 0x05):
            afcBambuAMS._on_status(shim, self._frame(st))
        assert shim._declared_since_fault is False

    def test_the_not_heard_from_sentinel_is_not_a_state(self):
        # 0xFF is the firmware saying "no reply yet". It must never latch.
        shim = self._shim()
        for _ in range(20):
            afcBambuAMS._on_status(shim, self._frame(0xFF))
        assert shim._declared_since_fault is False

    def test_another_units_declaration_does_not_latch_ours(self):
        shim = self._shim()
        afcBambuAMS._on_status(
            shim, {"units": [{"n": 1, "ustate": 0x07}], "slots": []})
        assert shim._declared_since_fault is False

    def test_a_new_fault_clears_the_latch(self):
        # "Declared" must mean "since THIS fault", never a leftover.
        shim = types.SimpleNamespace(
            name="BambuAMS_1", fault_pause=True, _follow_fault_hold=False,
            _follow_fault_saw_pause=False, _starved_since=1.0,
            _declared_since_fault=True, _fault_lane=None,
            _odom_lo=1.0, _odom_hi=9.0,
            _fault_floor_seen=True, _fault_recover_since=5.0,
            _fault_recover_reads=3, _resume_needs_reload=False,
            set_feed_assist=lambda ln, on: None,
            logger=types.SimpleNamespace(warning=lambda m: None,
                                         debug=lambda m: None),
            afc=types.SimpleNamespace(
                function=types.SimpleNamespace(in_print=lambda: False),
                error=types.SimpleNamespace(
                    AFC_error=lambda m, pause=True: None)))
        lane = types.SimpleNamespace(name="lane15")
        afcBambuAMS._raise_ams_fault(shim, lane, "jam")
        assert shim._declared_since_fault is False

    def test_the_latch_cannot_break_status_mirroring(self):
        # It shares _on_status's single except with the slot apply, so a throw
        # here would abandon the whole frame -- silently, for every frame.
        shim = self._shim()
        shim._unit_state = lambda o: (_ for _ in ()).throw(RuntimeError("x"))
        applied = []
        shim._sync_lanes = lambda: applied.append(True)
        afcBambuAMS._on_status(shim, {"units": [], "slots": []})
        assert applied == [True]


class TestJamLocation:
    """Say WHERE the jam is, from how far the AMS moved filament.

    Measured in the AMS 1 fault capture -- the odometer is a POSITION (0 = home
    in the AMS, ~1.86 m = at the toolhead), not a consumption counter:

        HOLD (printing)   frozen at 1.864 m     spread 0.000
        RETRY (jammed)    -0.009 .. 1.830       spread 1.839
        PARK              0.000 .. 0.002        spread 0.002

    The AMS swung the FULL tube during its retry and the filament still never
    reached the toolhead -- so the AMS was working and the blockage was
    downstream. Our stall message currently hedges ("the spool is likely
    tangled or the path jammed") because we had no way to tell, and the two
    need opposite responses.

    THERE IS DELIBERATELY NO CLOG DETECTOR HERE. During a print the value is
    pinned at tube length however much filament is consumed, and the drive
    channel is nearly silent anyway -- 49 replies in a 6.5 minute hold, all in
    the final minute, 0.13/s. There is no signal to watch.
    """

    def _shim(self, *, fault=True):
        return types.SimpleNamespace(
            name="BambuAMS_1", ams_index=0, unit_uid=None, _id_resolved=True,
            AMS_STATE_STALLED=0x07, _declared_since_fault=False,
            ODOM_MOVED_MM=200.0, _follow_fault_hold=fault,
            _odom_lo=None, _odom_hi=None,
            SLOTS_PER_UNIT=4, _slots=[{} for _ in range(4)],
            _status_err_last=None, _sync_lanes=lambda: None,
            _unit_state=lambda o: None,
            logger=types.SimpleNamespace(warning=lambda m: None,
                                         debug=lambda m: None))

    def _shimmed(self, **kw):
        sh = self._shim(**kw)
        # _jam_location calls _odom_span_mm on self; bind the real one.
        sh._odom_span_mm = lambda: afcBambuAMS._odom_span_mm(sh)
        return sh

    @staticmethod
    def _f(odom):
        return {"units": [{"n": 0, "odom": odom}], "slots": []}

    def _feed(self, shim, values):
        for v in values:
            afcBambuAMS._track_odom(shim, self._f(v))

    def test_a_working_ams_points_downstream(self):
        # The measured retry: swings the full tube, never reaches the toolhead.
        shim = self._shimmed()
        self._feed(shim, [-9, 1830, 12, 1810, 0])
        assert afcBambuAMS._odom_span_mm(shim) == 1839.0
        msg = afcBambuAMS._jam_location(shim)
        assert "DOWNSTREAM OF THE AMS" in msg and "not the spool" in msg

    def test_a_stuck_ams_points_at_the_spool(self):
        shim = self._shimmed()
        self._feed(shim, [1500, 1512, 1498, 1505])
        assert afcBambuAMS._odom_span_mm(shim) == 14.0
        assert "AT THE AMS" in afcBambuAMS._jam_location(shim)

    def test_it_says_nothing_when_it_cannot_tell(self):
        # Fewer than two readings is genuinely unknown; hedging beats guessing.
        shim = self._shimmed()
        assert afcBambuAMS._odom_span_mm(shim) is None
        assert afcBambuAMS._jam_location(shim) == ""

    def test_it_does_not_track_outside_a_fault(self):
        # The operator's constraint: this must never DECIDE anything during a
        # normal load, where the AMS legitimately slows and fights resistance
        # just before the toolhead sensor.
        #
        # AMENDED, and worth stating plainly. The fault range is still
        # fault-only -- that is what this asserts. A failed LOAD now records a
        # SEPARATE range (_load_odom_lo/hi), because the unit's odometer is the
        # only thing that knew a load had put five metres of filament on the
        # floor. The constraint survives because nothing reads that range until
        # the load has ALREADY failed: it is evidence for the error message,
        # never an input to a jam decision, and it cannot end or alter a load
        # that is working.
        shim = self._shimmed(fault=False)
        self._feed(shim, [0, 900, 1800])
        assert shim._odom_lo is None and shim._odom_hi is None
        assert afcBambuAMS._jam_location(shim) == ""

    def test_a_load_records_its_own_range_without_touching_the_faults(self):
        shim = self._shimmed(fault=False)
        shim._load_in_progress = True
        shim._load_odom_lo = shim._load_odom_hi = None
        self._feed(shim, [0, 2600, 5004, 4900])
        # The fault range stays untouched -- no fault is running.
        assert shim._odom_lo is None and shim._odom_hi is None
        assert afcBambuAMS._load_odom_span_mm(shim) == 5004.0

    def test_a_fault_during_a_load_cannot_wipe_the_loads_evidence(self):
        # _raise_ams_fault resets the FAULT range on purpose, so the two must
        # not share storage: a fault raised partway through a load would
        # otherwise erase the very measurement the failure needs.
        shim = self._shimmed(fault=True)
        shim._load_in_progress = True
        shim._load_odom_lo = shim._load_odom_hi = None
        self._feed(shim, [0, 5004])
        shim._odom_lo = shim._odom_hi = None          # what _raise_ams_fault does
        assert afcBambuAMS._load_odom_span_mm(shim) == 5004.0

    def test_the_load_span_reads_a_swing_not_a_net_move(self):
        # THE ODOMETER IS A POSITION. A unit that swings the whole tube and
        # comes back has a net delta of zero having moved 1.8m twice; a
        # start-vs-end reading would call the busiest failure we have "never
        # moved".
        shim = self._shimmed(fault=False)
        shim._load_in_progress = True
        shim._load_odom_lo = shim._load_odom_hi = None
        self._feed(shim, [0, 1830, 0])
        assert afcBambuAMS._load_odom_span_mm(shim) == 1830.0

    def test_the_load_span_drives_the_same_verdict(self):
        # Same question, different window -- so _jam_location takes the span
        # rather than owning one.
        shim = self._shimmed()
        assert "DOWNSTREAM OF THE AMS" in afcBambuAMS._jam_location(shim, 5004.0)
        assert "AT THE AMS" in afcBambuAMS._jam_location(shim, 14.0)
        assert afcBambuAMS._jam_location(shim, None) == ""       # falls back

    def test_the_unknown_sentinel_is_not_a_reading(self):
        # -1 is the firmware saying "no odometer yet". Treating it as a
        # position would fake a full-tube span out of nothing.
        shim = self._shimmed()
        self._feed(shim, [-1, -1, -1])
        assert afcBambuAMS._odom_span_mm(shim) is None

    def test_another_units_odometer_is_ignored(self):
        shim = self._shimmed()
        afcBambuAMS._track_odom(shim, {"units": [{"n": 1, "odom": 1800}]})
        assert shim._odom_lo is None

    def test_the_boundary_reads_as_moved(self):
        shim = self._shimmed()
        self._feed(shim, [0, 200])
        assert "DOWNSTREAM" in afcBambuAMS._jam_location(shim)


class TestRecoveryCannotRetriggerItself:
    """The attempt must not reset its own one-shot guard.

    unit_load_lane clears _auto_recover_armed on every load -- correctly, so a
    new fault after a normal load can recover. But auto recovery DRIVES a load,
    so its own CHANGE_TOOL landed there and cleared the guard it was holding.
    The next fault then armed another attempt. Measured on hardware:

        12:40:49  fault -> auto error recovery armed
        12:45:03  lane21 reached the toolhead sensor    (the reload worked)
        12:45:14  the AMS reports STALLED (state 7)
        12:45:14  auto error recovery armed AGAIN       <- same second
        12:45:35  the unit has given up. Parked.

    Four and a half minutes, which is exactly what "one attempt" was meant to
    prevent. The operator watched it and said "something not quite right".
    """

    def _shim(self, *, in_recovery):
        return types.SimpleNamespace(
            name="BambuAMS_2", _in_auto_recover=in_recovery,
            _auto_recover_armed=True, _follow_manual_off=True,
            _follow_fault_hold=True, _follow_fault_saw_pause=True)

    def test_a_load_during_recovery_keeps_the_guard(self):
        shim = self._shim(in_recovery=True)
        # The line inside unit_load_lane, in isolation.
        if not getattr(shim, "_in_auto_recover", False):
            shim._auto_recover_armed = False
        assert shim._auto_recover_armed is True

    def test_an_ordinary_load_still_clears_the_guard(self):
        # The guard must not become permanent -- that turns a self-retriggering
        # bug into a never-triggering one.
        shim = self._shim(in_recovery=False)
        if not getattr(shim, "_in_auto_recover", False):
            shim._auto_recover_armed = False
        assert shim._auto_recover_armed is False

    def test_every_exit_from_the_attempt_clears_the_flag(self):
        # Including the failures. A stuck flag suppresses the legitimate
        # re-arm on the NEXT fault.
        import inspect
        src = inspect.getsource(afcBambuAMS._maybe_auto_recover)
        body = src[src.index("def _run("):]
        assert "return self.afc.reactor.NEVER" not in body, (
            "an exit from _run bypasses _done() and leaks _in_auto_recover")
        assert body.count("_done(self.afc.reactor.NEVER)") >= 4


class TestResumeWrapWaitsForTheRealHandler:
    """Wrap when RESUME actually IS AFC's handler -- not when a flag says prep
    has started.

    The first version polled AFC_prep.rename_occurred, which is a race:

        if not self.rename_occurred:
            self.rename_occurred = True            <- set FIRST
            self.afc.function._rename(RESUME...)   <- rename AFTER

    The flag is set before the rename it announces, so the wrap could land in
    the gap and PREP would overwrite it. Observed live: the log said "RESUME
    wrapped" at 12:52:43 and RESUME was AFC's handler afterwards -- the wrapper
    was gone. It had worked three times before that on timing alone.
    """

    @staticmethod
    def _gate(command_names):
        """The precondition, in isolation: has AFC's rename COMPLETED?

        _AFC_RENAMED_RESUME_ exists if and only if it has, because AFC creates
        it as part of doing the rename. The effect, not a flag beside it.
        """
        return "_AFC_RENAMED_RESUME_" in command_names

    def test_it_waits_before_afc_has_renamed(self):
        assert not self._gate({"RESUME": "the printer's own", "PAUSE": ""})

    def test_it_fires_once_the_rename_has_completed(self):
        assert self._gate({"RESUME": "", "_AFC_RENAMED_RESUME_": ""})

    def test_an_empty_table_is_not_a_go_signal(self):
        assert not self._gate({})

    def test_it_does_not_reach_into_private_klipper_attributes(self):
        # The second failed attempt read gcode.ready_gcode_handlers directly.
        # This Klipper does not present it the way that assumed, so the gate
        # never matched and timed out after 120s with the wrap inactive.
        import inspect
        src = inspect.getsource(afcBambuAMS._arm_resume_wrap)
        code = "\n".join(l for l in src.split("\n")
                          if not l.strip().startswith("#"))
        assert "ready_gcode_handlers" not in code
        assert "get_command_help" in code

    def test_the_gate_is_not_a_flag_read(self):
        # Guards the actual regression: nothing in the arm may consult
        # rename_occurred, because it is set before the rename it announces.
        import inspect
        src = inspect.getsource(afcBambuAMS._arm_resume_wrap)
        code = "\n".join(l for l in src.split("\n")
                         if not l.strip().startswith("#"))
        assert "rename_occurred" not in code


class TestMeasuredRemainIsCapped:
    """A spool cannot hold more than its own nominal weight.

    Measured on a real printer with our hardware entirely off the bus (the Pico
    ran the listen-only sniff build and Klipper was stopped) -- the SAME HT
    spool, eight minutes apart:

        16:47:13  C:0.531  R:0.084  P:107%  od:1.132
        16:55:28  C:0.551  R:0.088  P:119%  od:1.143

    The radius went UP by 3.5mm while filament was being consumed, which cannot
    happen, so at least one is wrong by 12 points. The AMS derives R from a
    circumference sampled over od/C = ~2.1 SPOOL REVOLUTIONS.

    We cannot improve the unit's arithmetic. We can decline to publish 1190g of
    filament on a 1kg spool as a measured weight.
    """

    def _shim(self):
        pushed = []
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            _push_measured_to_spoolman=lambda ln, g, src="": pushed.append((ln.name, g)))
        return shim, pushed

    def _lane(self):
        return types.SimpleNamespace(name="lane15", weight=0,
                                     tool_loaded=False)

    def test_the_measured_119_percent_becomes_1000g_not_1190g(self):
        shim, pushed = self._shim()
        shim._measured_remain = {2: 119}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 119, "weight": 1000})
        assert lane.weight == 1000
        assert pushed == [("lane15", 1000)]

    def test_the_measured_107_percent_is_capped_too(self):
        shim, _p = self._shim()
        shim._measured_remain = {2: 107}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 107, "weight": 1000})
        assert lane.weight == 1000

    def test_an_ordinary_reading_is_untouched(self):
        # An ordinary MEASURED reading passes through uncapped and unrounded.
        shim, _p = self._shim()
        shim._measured_remain = {2: 80}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 80, "weight": 1000})
        assert lane.weight == 800

    # ── the measurement beats the tag record, HERE too ──────────────────────
    #
    # This function runs on every status frame off the RAW bridge slot, so it
    # was re-applying the tag's number seconds after a fresh measurement landed
    # -- get_status overrides remain_pct with _measured_remain, and this path
    # never saw that. Captured on an AMS 1 insert, spool #87, three writes in
    # 106 ms:
    #
    #   14:03:27  wrote 700 g            (tag record, 70%)
    #   14:03:40  measured ... 69% (~690 g) [capscan]
    #   14:03:40  wrote 690 g            (the measurement)
    #   14:03:40  wrote 700 g            (this function, tag record again)
    #
    # The operator saw the stale figure and asked whether it had come from
    # Spoolman rather than the read. It had -- and then went back.

    def test_a_fresh_measurement_wins_over_the_tag_record(self):
        shim, pushed = self._shim()
        shim._measured_remain = {2: 69}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 70, "weight": 1000})
        assert lane.weight == 690
        assert pushed == [("lane15", 690)]

    def test_it_is_the_right_slots_measurement(self):
        # Keyed by slot: bay 3's measurement must not describe bay 2's spool.
        # Bay 2 has none of its own, so it falls back to ITS OWN tag (70% of
        # 1000 = 700, below the lane's 1000) -- never to bay 3's 69%.
        shim, pushed = self._shim()
        shim._measured_remain = {3: 69}
        lane = self._lane()
        lane.weight = 1000
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 70, "weight": 1000})
        assert lane.weight == 700         # its own tag, not 690
        assert pushed == [("lane15", 700)]

    def test_no_measurement_takes_the_tag_only_when_it_is_LOWER(self):
        # FILAMENT ONLY GOES DOWN, so with no measurement the smaller of the
        # tag and what the lane already tracks is the more recent truth. Tag
        # 70% = 700 g against a lane carrying 1000 -> the tag saw usage the
        # tracking missed, so it wins and the correction reaches Spoolman.
        #
        # Live case: lane21 orange PLA, fast-pathed so never measured, tag
        # 60% = 600 g while Spoolman spool 97 held 800 -- the console promised
        # 600 and the lane showed 800.
        shim, pushed = self._shim()
        lane = self._lane()
        lane.weight = 1000
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 70, "weight": 1000})
        assert lane.weight == 700
        assert pushed == [("lane15", 700)]

    def test_a_tag_reading_HIGHER_than_the_lane_is_refused(self):
        # The floor is what makes the fallback safe, and this is the reel the
        # old prohibition was written for: read off the wire at one moment the
        # tag said 80% (~800 g) while the AMS measured 23% (~230 g) and the
        # scale agreed at 230. Nothing writes a measurement back to the tag,
        # so its figure is whatever the last calibration left there and it can
        # sit far too high. A lane already down at 230 must not be inflated to
        # 800, and that fiction must never reach Spoolman.
        shim, pushed = self._shim()
        lane = self._lane()
        lane.weight = 230
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 80, "weight": 1000})
        assert lane.weight == 230         # untouched
        assert pushed == []               # and nothing reached Spoolman

    def test_a_zero_remain_tag_is_never_a_weight(self):
        # 0 on a Bambu tag means "never measured", not "empty" -- it must not
        # zero a lane just because it is lower than everything.
        shim, pushed = self._shim()
        lane = self._lane()
        lane.weight = 1000
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 0, "weight": 1000})
        assert lane.weight == 1000
        assert pushed == []

    def test_a_measurement_over_100_is_still_capped(self):
        # The measurement winning must not smuggle 1190 g past the cap.
        shim, pushed = self._shim()
        shim._measured_remain = {2: 119}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 70, "weight": 1000})
        assert lane.weight == 1000

    # ── all three unit types ────────────────────────────────────────────────
    #
    # The fix is dialect-free by construction: it keys on a SLOT INDEX and an
    # integer percent and never touches text, so there is no per-model wording
    # for it to get wrong. What varies between the types is the bay layout --
    # the HT has exactly one bay (index 0) where the boxed units have four --
    # and _measured_remain is a per-unit-object dict, so an HT's slot 0 and an
    # AMS 2's slot 0 cannot reach each other.
    #
    # (Parsing the measurement is the part that IS dialect-specific, and that
    # is pinned separately in test_AFC_BambuAMS_bridge.py::
    # test_capacity_line_parses_on_all_three_units. Today's AMS 1 insert line,
    # "STEP:odom C:0.461,R:0.073,P:69%, od:0.852", is that same shape.)

    def test_the_measurement_wins_on_every_bay_of_a_boxed_unit(self):
        for bay in (0, 1, 2, 3):
            shim, pushed = self._shim()
            shim._measured_remain = {bay: 69}
            lane = self._lane()
            afcBambuAMS._apply_remain_weight(
                shim, lane, {"index": bay, "remain_pct": 70, "weight": 1000})
            assert lane.weight == 690, f"bay {bay}"
            assert pushed == [("lane15", 690)], f"bay {bay}"

    def test_the_ht_single_bay_behaves_the_same(self):
        # An HT only ever has index 0, and a 1kg spool on it must correct the
        # same way a boxed bay does.
        shim, pushed = self._shim()
        shim._measured_remain = {0: 69}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 0, "remain_pct": 70, "weight": 1000})
        assert lane.weight == 690
        assert pushed == [("lane15", 690)]

    def test_the_percent_is_applied_against_each_units_own_nominal(self):
        # 250 g sample spools and 1 kg reels both exist on this rig; the tag's
        # nominal is what the percent is applied to, per unit and per bay.
        for nominal, expect in ((1000, 690), (250, 172), (750, 517)):
            shim, pushed = self._shim()
            shim._measured_remain = {0: 69}
            lane = self._lane()
            afcBambuAMS._apply_remain_weight(
                shim, lane, {"index": 0, "remain_pct": 70, "weight": nominal})
            assert lane.weight == expect, nominal

    def test_the_cap_follows_the_tags_nominal_weight(self):
        # A 250g sample spool caps at 250g, not 1000g. Driven by a MEASURED
        # 119% now that the tag's stored figure is not a weight source.
        shim, _p = self._shim()
        shim._measured_remain = {2: 119}
        lane = self._lane()
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"index": 2, "remain_pct": 119, "weight": 250})
        assert lane.weight == 250

    def test_a_loaded_lane_is_left_alone(self):
        # Its weight is being decremented by extrusion.
        shim, pushed = self._shim()
        lane = self._lane()
        lane.tool_loaded = True
        afcBambuAMS._apply_remain_weight(
            shim, lane, {"remain_pct": 119, "weight": 1000})
        assert lane.weight == 0 and pushed == []


class TestFaultReasonIsReadable:
    """The reason, without the bus dump around it.

    What went to the operator, verbatim, at a real stall:

        [AMS_COMMON]en:1,mode:3,idx:0,ref:0 [AMS_COMMON]en:1,mode:3,idx:2,
        ref:127 [AMS_COMMON]en:1,mode:3,idx:0,ref:0 [AMS_SWITCH]timeout,
        assist finish stall! pos:0.1

    Three of those four fragments are link keep-alive, present at every instant
    of every print. Exactly one is the reason.
    """

    RAW = ("[AMS_COMMON]en:1,mode:3,idx:0,ref:0 "
           "[AMS_COMMON]en:1,mode:3,idx:2,ref:127 "
           "[AMS_COMMON]en:1,mode:3,idx:0,ref:0 "
           "[AMS_SWITCH]timeout, assist finish stall! pos:0.1")

    def test_the_reason_survives_and_the_chatter_does_not(self):
        out = _fault_reason(self.RAW)
        assert out == "[AMS_SWITCH]timeout, assist finish stall! pos:0.1"

    def test_an_unknown_dialect_is_kept(self):
        # NOT a whitelist of known stall wording. The three unit types phrase
        # it three ways and one says nothing at all, so matching on known
        # phrases would silently swallow the dialect nobody has seen yet.
        out = _fault_reason("[AMS_COMMON]en:1 [AMS_WHATEVER]something new")
        assert out == "[AMS_WHATEVER]something new"

    def test_several_reasons_are_all_kept(self):
        out = _fault_reason("[AMS_SWITCH]stall [AMS_COMMON]en:1 "
                            "[AMS_DEV] STEP:odom tray_id error 255")
        assert "[AMS_SWITCH]stall" in out
        assert "[AMS_DEV] STEP:odom tray_id error 255" in out
        assert "AMS_COMMON" not in out

    def test_all_chatter_falls_back_to_the_raw_text(self):
        # Better a dump than an empty explanation -- if filtering leaves
        # nothing, the operator still gets what the unit said.
        raw = "[AMS_COMMON]en:1,mode:3 [AMS_LINK]err_code:0x00->0x16"
        assert _fault_reason(raw) == raw

    def test_empty_stays_empty(self):
        assert _fault_reason("") == ""
        assert _fault_reason(None) is None

    def test_text_with_no_markers_is_untouched(self):
        assert _fault_reason("something went wrong") == "something went wrong"

    # ── the AMS 1's verdict rides on chatter tags ────────────────────────────
    #
    # NOT because it is silent. It is not, and treating it as silent is the
    # mistake this project has made twice. Its give-up buffer is full of
    # [AMS_DEV] odometry, which is not chatter and survives the filter fine.
    # What does not survive is the part that says it gave up: state:6 on
    # [AMS_COMMON] and en:0,mode:7 on [AMS_LINK], both keep-alive tags. So the
    # operator got a real message full of real fragments, all of them hunting
    # odometry, with the verdict filtered out of it.
    #
    # Verbatim from the failing lane15 capture (THE_LOAD.md), tags included --
    # a test written from memory of the wire format is worth nothing.

    AMS1_GIVE_UP = ("[AMS_DEV] STEP:odom search, odo 0.516 ... 1.185 "
                    "[AMS_DEV] STEP:odom reset tray 0 "
                    "[AMS_IDLE]set ams state switch "
                    "[AMS_COMMON]state:0,tray_now:255,tray_exit:6 "
                    "[AMS_COMMON]state:6,tray_now:255,tray_exit:6 "
                    "[AMS_LINK]en:0,mode:7,idx:255,ref:0")

    def test_the_ams1_give_up_survives_the_chatter_filter(self):
        out = _fault_reason(self.AMS1_GIVE_UP)
        assert "state:6" in out
        assert "en:0,mode:7,idx:255" in out

    def test_the_ams1_odometry_was_never_the_problem(self):
        # [AMS_DEV] is not chatter and always survived. Pinned so nobody
        # "fixes" the tag list on the theory that the AMS 1 says nothing.
        out = _fault_reason(self.AMS1_GIVE_UP)
        assert "STEP:odom search" in out

    def test_the_keep_alive_beside_the_verdict_still_goes(self):
        # state:0 is the same tag as state:6 and means nothing here. The rescue
        # keys on the signature, so only the fragment that IS the reason stays.
        out = _fault_reason(self.AMS1_GIVE_UP)
        assert "state:0" not in out
        assert "set ams state switch" not in out      # [AMS_IDLE], chatter

    def test_the_ht_stall_state_is_kept_beside_its_words(self):
        # Captured on the HT, lane23: it says BOTH, and the state was being
        # dropped from its own error message because only state:6 was listed.
        out = _fault_reason("[AMS_COMMON]state:7,tray_now:0,tray_exit:1 "
                            "[AMS_LED]TIMEOUT error 0")
        assert "state:7" in out and "TIMEOUT error 0" in out

    def test_state_zero_is_not_a_give_up(self):
        # The states either side of them are ordinary. 6 and 7 are the list.
        assert "state:0" not in _fault_reason(
            "[AMS_COMMON]state:0,tray_now:0 [AMS_LED]TIMEOUT error 0")

    def test_en0_mode7_is_kept_off_ams_link(self):
        # The tag it actually arrives on. An earlier version of this test said
        # [AMS_COMMON] from memory; the capture says [AMS_LINK].
        out = _fault_reason("[AMS_LINK]en:1,mode:3,idx:0,ref:0 "
                            "[AMS_LINK]en:0,mode:7,idx:255,ref:0")
        assert out == "[AMS_LINK]en:0,mode:7,idx:255,ref:0"

    def test_a_chatter_tagged_stall_is_still_a_reason(self):
        # Whichever tag it lands on. The rescue keys on the signature, not on
        # which unit happened to emit it.
        out = _fault_reason("[AMS_COMMON]en:1,mode:3 "
                            "[AMS_COMMON]pull err,bdc stall")
        assert out == "[AMS_COMMON]pull err,bdc stall"

    def test_ordinary_chatter_is_still_dropped(self):
        # The rescue must not become a second fallback: a keep-alive with no
        # fault signature is dropped.
        out = _fault_reason("[AMS_COMMON]en:1,mode:3,idx:0,ref:0 "
                            "[AMS_SWITCH]feed finish, buff_pos:1.28")
        assert out == "[AMS_SWITCH]feed finish, buff_pos:1.28"


# The load path uses the bare stop(). See the note above _feed_until_sensor.



class TestTxEchoIsDiffable:
    """The TX echo emits the SAME line shape as a capture, on purpose.

    The one thing nobody can see on this bus is our own output: the sniff build
    is listen-only and we are the master. Three wrong calls in one evening came
    from inferring what we transmit -- a phase that could not be reached, an
    enrollment branch that could not be reached, and an op-03 byte that
    correlated with the ref across 32,000 captured frames and faulted a unit at
    1.39A when we sent it.

    Emitting the capture's own shape means a TX log diffs against
    ht_clean_load with the tools that already exist, rather than new ones
    written to read a new format.
    """

    def test_a_tx_line_parses_as_a_capture_line(self):
        import json as _json
        line = ('{"evt":"tx","us":10322878545,"n":13,'
                '"hex":"3DC50DF10400077F03000211BC"}')
        obj = _json.loads(line)
        assert obj["evt"] == "tx"
        # Every field the capture tooling reads off a sniff line.
        assert isinstance(obj["us"], int)
        assert obj["n"] == 13
        assert bytes.fromhex(obj["hex"])[0] == 0x3D

    def test_the_length_field_is_the_REAL_length(self):
        # The ring stores at most 32 bytes but records the true length, so a
        # diff can tell "this frame was 44 bytes" from "we kept 32 of it".
        # A capture that silently truncates is a capture that lies.
        import json as _json
        obj = _json.loads('{"evt":"tx","us":1,"n":44,"hex":"' + "AB" * 32 + '"}')
        assert obj["n"] == 44
        assert len(bytes.fromhex(obj["hex"])) == 32
        assert obj["n"] > len(bytes.fromhex(obj["hex"]))   # truncation visible

    def test_the_event_is_known_to_the_bridge(self):
        # Unknown events hit the catch-all above the per-event branches and are
        # swallowed -- which is exactly how the rc/rollcall events went missing
        # once. tx is consumed by its own branch, so it must not be treated as
        # unhandled either.
        from extras.AFC_BambuAMS_bridge import _BRIDGE_EVENTS_KNOWN
        assert "txecho" in _BRIDGE_EVENTS_KNOWN     # the command echo


class TestTheSensorReadIsThePin:
    """A false arrival does not just mis-report -- it DISABLES THE RETRY.

    unit_load_lane's recovery (stop -> re-home -> feed again,
    load_recover_attempts times) is gated on `if not loaded`. If
    _feed_until_sensor returns True on filament that never arrived, the load is
    declared good and the retry the operator expects never runs. That is the
    difference between "it retried by unloading and reloading" and tonight's
    "it continued like it thinks it did"."""

    def _lane(self, pin, cache):
        return types.SimpleNamespace(
            name="lane21",
            extruder_obj=types.SimpleNamespace(
                fila_tool_start=types.SimpleNamespace(
                    runout_helper=types.SimpleNamespace(
                        filament_present=pin))),
            get_toolhead_pre_sensor_state=lambda: cache)

    def test_the_pin_beats_a_stale_cache(self):
        shim = types.SimpleNamespace(name="AMS", logger=_Logger())
        # Cache says loaded, pin says no filament. The pin wins, so the load
        # fails and the retry gets its chance.
        assert afcBambuAMS._toolhead_sensor_triggered(
            shim, self._lane(pin=False, cache=True)) is False
        assert afcBambuAMS._toolhead_sensor_triggered(
            shim, self._lane(pin=True, cache=False)) is True

    def test_no_switch_falls_back_to_the_lane(self):
        # tool_start = "buffer" has no pin to read; the lane accessor is right.
        shim = types.SimpleNamespace(name="AMS", logger=_Logger())
        lane = types.SimpleNamespace(
            name="lane21",
            extruder_obj=types.SimpleNamespace(fila_tool_start=None),
            get_toolhead_pre_sensor_state=lambda: True)
        assert afcBambuAMS._toolhead_sensor_triggered(shim, lane) is True


class TestBiteThenLetItPull:
    """The AMS pulls the tray back on the mode change into mode:4. That is
    native -- it is in every working load. The damage is that the extruder is
    mid-advance when it happens, so the gears drive forward while the unit
    reels back and the filament is fought from both ends.

    Bite a little, get out of the way, then advance the rest."""

    def _rig(self, tool_stn=40.0, bite=2.0, settle=1.5):
        order = []
        clock = _Clock()
        # The pull "completes" as soon as it is waited on, unless a test says
        # otherwise -- the interesting cases are ordering and the ceiling.
        pulls = {"n": 0}
        lane = types.SimpleNamespace(
            name="lane21",
            activate_toolhead_extruder=lambda: order.append("activate"))
        ext = types.SimpleNamespace(tool_stn=tool_stn, tool_load_speed=7.0)
        afc = types.SimpleNamespace(
            reactor=clock,
            move_e_pos=lambda d, s, label: order.append((label, round(d, 2))))
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), afc=afc,
            tool_bite_mm=bite, pull_settle_s=settle,
            pull_push_dwell_s=0.0, arrival_select=True,
            arrival_assist_delay_s=0.0,
            select_lane=lambda ln: order.append("select"),
            set_feed_assist=lambda ln, on: order.append(("assist", on)),
            stop=lambda: order.append("stop"),
            # The unit reports its pull; the wait is on THAT, not a timer.
            _pull_seq_now=lambda: pulls["n"],
            _assist_seq_now=lambda: pulls["n"],
            _bridge=types.SimpleNamespace(last_pull=lambda: pulls["n"],
                                          last_assist_done=lambda: pulls["n"]))
        shim._wait_for_pull = (
            lambda s0, a0=0: afcBambuAMS._wait_for_pull(shim, s0, a0))
        return shim, lane, ext, afc, order, clock

    def test_the_bite_lands_before_the_assist_and_the_rest_after(self):
        shim, lane, ext, afc, order, _ = self._rig()
        afcBambuAMS._advance_into_extruder(shim, lane, ext)
        assert order == [
            "activate",
            ("tool bite", 2.0),      # gears grip, nothing else moving
            "select",
            ("assist", True),        # the unit's pull happens here
            ("tool stn", 38.0),      # the remainder, into a settled path
        ]

    def test_it_waits_while_the_unit_pulls(self):
        shim, lane, ext, afc, order, clock = self._rig(settle=1.5)
        t0 = clock.monotonic()
        afcBambuAMS._advance_into_extruder(shim, lane, ext)
        assert clock.monotonic() - t0 >= 1.5

    def test_bite_zero_advances_in_one_go(self):
        shim, lane, ext, afc, order, _ = self._rig(bite=0.0, settle=0.0)
        afcBambuAMS._advance_into_extruder(shim, lane, ext)
        assert ("tool stn", 40.0) in order
        assert not any(o[0] == "tool bite" for o in order
                       if isinstance(o, tuple))

    def test_a_bite_bigger_than_tool_stn_does_not_overshoot(self):
        shim, lane, ext, afc, order, _ = self._rig(tool_stn=1.0, bite=2.0)
        afcBambuAMS._advance_into_extruder(shim, lane, ext)
        moves = [o for o in order if isinstance(o, tuple) and "tool" in str(o[0])]
        assert sum(d for _, d in moves) == 1.0


class TestEnrollmentEchoesAreKnown:
    """`bind` and `htuid` are emitted once per known unit on EVERY status round.
    Absent from _BRIDGE_EVENTS_KNOWN they take the "unhandled bridge event"
    path, which with three units on the wire measured 69 log lines per second
    inside Klipper's process -- enough to starve the reactor until the CAN
    toolhead missed a scheduled pin event:

        MCU 'EBBT0' shutdown: Missed scheduling of next digital out event

    A per-round event that is not in this set is a logging storm waiting for a
    second unit to be plugged in."""

    def test_the_per_round_echoes_are_known(self):
        from extras.AFC_BambuAMS_bridge import _BRIDGE_EVENTS_KNOWN
        for evt in ("bind", "htuid"):
            assert evt in _BRIDGE_EVENTS_KNOWN, (
                f"{evt} is emitted every status round; unknown means it is "
                f"logged every status round")

    def test_every_command_echo_we_send_is_known(self):
        # The same trap, generalised: anything the firmware echoes per round
        # and we do not recognise becomes per-round log traffic.
        from extras.AFC_BambuAMS_bridge import _BRIDGE_EVENTS_KNOWN
        for evt in ("chain", "status", "ack", "units", "htunit"):
            assert evt in _BRIDGE_EVENTS_KNOWN


class TestBiteIsRuntimeToggleable:
    """The bite is the A/B knob for "does the extruder advance at the sensor
    help or hurt". Making that test cost a config edit and a restart is how it
    does not get run."""

    def _shim(self):
        said = []
        return types.SimpleNamespace(
            name="AMS", tool_bite_mm=2.0, pull_settle_s=2.0,
            pull_push_dwell_s=3.0), said

    def _gcmd(self, **kw):
        return types.SimpleNamespace(
            get_float=lambda k, d, minval=None, maxval=None: kw.get(k, d),
            respond_info=lambda m: kw.setdefault("_said", []).append(m))

    def test_mm_zero_turns_the_bite_off(self):
        shim, _ = self._shim()
        afcBambuAMS.cmd_AFC_BAMBU_BITE(shim, self._gcmd(MM=0.0))
        assert shim.tool_bite_mm == 0.0
        assert shim.pull_settle_s == 2.0      # settle untouched

    def test_mm_sets_the_bite(self):
        shim, _ = self._shim()
        afcBambuAMS.cmd_AFC_BAMBU_BITE(shim, self._gcmd(MM=3.5))
        assert shim.tool_bite_mm == 3.5

    def test_the_settle_has_its_own_command(self):
        # Separate mechanisms, separate commands. The bite is about the gears
        # having hold; the settle is about staying out of the way while the AMS
        # pulls. One knob with two numbers is how they kept being reasoned
        # about as a single thing.
        shim, _ = self._shim()
        afcBambuAMS.cmd_AFC_BAMBU_SETTLE(shim, self._gcmd(S=4.0))
        assert shim.pull_settle_s == 4.0
        assert shim.tool_bite_mm == 2.0        # bite untouched

    def test_the_bite_does_not_touch_the_settle(self):
        shim, _ = self._shim()
        afcBambuAMS.cmd_AFC_BAMBU_BITE(shim, self._gcmd(MM=0.0))
        assert shim.pull_settle_s == 2.0

    def test_no_args_reports_without_changing(self):
        shim, _ = self._shim()
        afcBambuAMS.cmd_AFC_BAMBU_BITE(shim, self._gcmd())
        assert (shim.tool_bite_mm, shim.pull_settle_s) == (2.0, 2.0)


class TestTheFollowerIsHeldOffDuringALoad:
    """cur_lane.status only becomes TOOL_LOADED at the END of a load, so
    throughout the arrival _tool_loaded_lane() answers None. Without a guard the
    follower tick reads that as "nothing loaded here anymore", drops the assist
    the load path just armed, and re-arms it when the status lands -- three mode
    changes in two seconds at a unit that was loading correctly:

        03:52:24  ack select, ack assist          (load path)
        03:52:24  standing the follower down for lane23
        03:52:26  ack select, ack assist          (re-armed)

    The unload already had this guard. The load never did."""

    def test_the_guard_is_set_during_the_load_and_cleared_after(self):
        seen = []
        shim = types.SimpleNamespace(_load_in_progress=False)
        shim._unit_load_lane = lambda ln, ext: (
            seen.append(shim._load_in_progress), True)[1]
        assert afcBambuAMS.unit_load_lane(shim, object(), object()) is True
        assert seen == [True]                 # set while the load runs
        assert shim._load_in_progress is False   # and cleared after

    def test_it_is_cleared_even_when_the_load_raises(self):
        # Five returns and it can raise. A load that leaves the guard set would
        # silence the follower for the rest of the session.
        shim = types.SimpleNamespace(_load_in_progress=False)

        def boom(ln, ext):
            raise RuntimeError("bridge went away")
        shim._unit_load_lane = boom
        with pytest.raises(RuntimeError):
            afcBambuAMS.unit_load_lane(shim, object(), object())
        assert shim._load_in_progress is False

    def test_a_failed_load_also_clears_it(self):
        shim = types.SimpleNamespace(_load_in_progress=False)
        shim._unit_load_lane = lambda ln, ext: False
        assert afcBambuAMS.unit_load_lane(shim, object(), object()) is False
        assert shim._load_in_progress is False


class TestAnHtHasOneBay:
    """An HT is a single-spool unit. The internal slot arrays stay
    SLOTS_PER_UNIT wide on every unit type on purpose -- the bridge indexes
    them by slot number and a short array would fault on a stray frame naming
    slot 3 -- but PUBLISHING four made an HT look like a four-bay unit:

        BambuAMS_HT  online=True idx=2 slots=4 present=1

    unit_slots already carried the truth; it was not applied on the way out."""

    def _shim(self, unit_slots):
        return types.SimpleNamespace(
            unit_slots=unit_slots,
            _slots=[{"i": 0}, {"i": 1}, {"i": 2}, {"i": 3}])

    def test_an_ht_publishes_one(self):
        shim = self._shim(1)
        out = afcBambuAMS._published_slots(shim)
        assert len(out) == 1
        assert out[0]["i"] == 0

    def test_a_boxed_unit_publishes_four(self):
        shim = self._shim(4)
        assert len(afcBambuAMS._published_slots(shim)) == 4

    def test_the_storage_is_not_narrowed(self):
        # The trim must be on the way OUT only: a stray frame naming slot 3 at
        # an HT must still land somewhere rather than raising.
        shim = self._shim(1)
        afcBambuAMS._published_slots(shim)
        assert len(shim._slots) == 4


class TestWaitForTheUnitsPull:
    """Measured on hardware: the AMS's native mode:4 pull ENDED 2.02s after the
    assist was armed, while the blind settle was 2.00s -- the extruder advance
    began 20ms before the unit finished pulling, and the captures range the
    pull 0.5-2.2s. A timer cannot win that. Wait for the unit to say it is
    done, and keep the timer only as the ceiling."""

    def _shim(self, pulls, settle=2.0):
        clock = _Clock()
        return types.SimpleNamespace(
            name="AMS", logger=_Logger(), pull_settle_s=settle,
            pull_push_dwell_s=0.0, arrival_select=True,
            afc=types.SimpleNamespace(reactor=clock),
            _pull_seq_now=lambda: pulls["n"],
            _assist_seq_now=lambda: pulls["n"]), clock

    def test_it_returns_as_soon_as_the_unit_reports_the_pull(self):
        pulls = {"n": 7}
        shim, clock = self._shim(pulls)
        # The pull lands on the first poll.
        orig = shim._pull_seq_now
        def seq():
            pulls["n"] = 8      # pull AND push-forward both reported
            return orig()
        shim._pull_seq_now = seq
        shim._assist_seq_now = seq
        t0 = clock.monotonic()
        assert afcBambuAMS._wait_for_pull(shim, 7) is True
        # Returned well inside the 2s ceiling -- only the post-pull grace.
        assert clock.monotonic() - t0 < 2.0

    def test_it_gives_up_on_the_ceiling_if_the_unit_never_reports(self):
        # A dialect that does not narrate the pull must still proceed, on
        # exactly the wait it had before.
        pulls = {"n": 3}
        shim, clock = self._shim(pulls, settle=1.0)
        t0 = clock.monotonic()
        assert afcBambuAMS._wait_for_pull(shim, 3) is False
        assert clock.monotonic() - t0 >= 1.0

    def test_settle_zero_does_not_wait_at_all(self):
        pulls = {"n": 0}
        shim, clock = self._shim(pulls, settle=0.0)
        assert afcBambuAMS._wait_for_pull(shim, 0) is False
        assert clock.monotonic() == 0.0

    def test_the_sequence_is_read_before_the_mode_change(self):
        # _advance_into_extruder must sample the counter BEFORE select+assist,
        # or a pull that completes quickly is missed between the two and we
        # wait the full ceiling into an already-finished pull.
        import inspect
        src = inspect.getsource(afcBambuAMS._advance_into_extruder)
        assert src.index("seq0 = self._pull_seq_now()") < src.index("select_lane")
        assert src.index("select_lane") < src.index("_wait_for_pull")


class TestNoSwitchMeansNoPullToWaitFor:
    """The pull is caused by the mode-09 select. With arrival_select off we
    never command the switch, the unit finishes into mode:4 by itself as the
    printer's does, and no pull happens -- so waiting for one burns the entire
    ceiling on an event that cannot arrive. Stacked on the arrival assist
    delay that was ten seconds of dead time before the advance."""

    def test_it_does_not_wait_when_the_select_is_off(self):
        clock = _Clock()
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), pull_settle_s=6.0,
            pull_push_dwell_s=3.0, arrival_select=False,
            afc=types.SimpleNamespace(reactor=clock),
            _pull_seq_now=lambda: 0, _assist_seq_now=lambda: 0)
        assert afcBambuAMS._wait_for_pull(shim, 0, 0) is False
        assert clock.monotonic() == 0.0        # not one second spent

    def test_it_still_waits_when_the_select_is_on(self):
        clock = _Clock()
        shim = types.SimpleNamespace(
            name="AMS", logger=_Logger(), pull_settle_s=1.0,
            pull_push_dwell_s=0.0, arrival_select=True,
            afc=types.SimpleNamespace(reactor=clock),
            _pull_seq_now=lambda: 0, _assist_seq_now=lambda: 0)
        assert afcBambuAMS._wait_for_pull(shim, 0, 0) is False
        assert clock.monotonic() >= 1.0        # rode out the ceiling


class TestALatchedUnitStopsTheLoadLoop:
    """A FAULT IS NOT A REASON TO STOP ASKING.

    This class used to assert the opposite -- that a declared fault broke the
    kick loop, because a latched AMS "has already given up". The captures say
    otherwise, and the real printer is the authority. Counted in
    ams1_print_fault_2026-08-05, from the `stall` narration to the end of the
    capture:

        op-03 drive frames AFTER the fault ....... 12,556
        op-04 polls .............................. 8,060
        op-20 heartbeats ......................... 8,410

    The printer does not flinch. The AMS runs its own recovery UNDERNEATH the
    continued request -- unlatching, re-homing, re-feeding -- so withdrawing
    the request is what makes a fault terminal. Live cost of the old rule: an
    HT declared a fault, we quit kicking one interval later, and it died
    mid-recovery.
    """

    def test_a_declared_fault_keeps_the_loop_running(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=5.0)
        faults = {"n": 0}

        def declared():
            faults["n"] += 1
            return faults["n"] >= 2      # faults from the second look onward
        shim._ams_declared_fault = declared
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is False
        # Rode out the window feeding, exactly as the printer does.
        assert len(calls["feed"]) >= 2, calls["feed"]

    def test_the_sensor_still_ends_it_even_while_faulted(self):
        # Recovery succeeding mid-fault must still stop the moment the
        # filament arrives -- continuing to ask is not the same as ignoring
        # the answer.
        shim, calls, _ = _load_shim(sensor_after=2, timeout=5.0)
        shim._ams_declared_fault = lambda: True
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True

    def test_no_fault_still_rides_the_window(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.4)
        shim._ams_declared_fault = lambda: False
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 0.4) is False


class TestTheVerdictSurvivesBeingConsumed:
    """_ams_declared_fault CONSUMES the bridge's sequence -- that is what stops
    a second consumer raising the same event twice. It also threw away the only
    account of WHY, and everything downstream then had to guess.

    Measured on the HT, lane23:

        10:46:05.661  AMS: ...state:7... [AMS_LED]TIMEOUT error 0
        10:46:05.668  the AMS reported a fault during the load; stopping
        10:49:34      "Filament did not reach the toolhead sensor within 101s
                       ... Check the filament path and afc_bowden_length
                       calibration."

    The break worked, in 7ms. The operator was still sent to measure a bowden
    that had nothing to do with it.
    """

    def _shim(self, seq=4, text="[AMS_LED]TIMEOUT error 0"):
        shim = types.SimpleNamespace(
            _bridge=types.SimpleNamespace(
                last_fault=lambda: (seq, text, 0.0)),
            _fault_seen=0, _declared_fault_text=None)
        # The real accessor, so the filter under test is the shipped one.
        shim._ams_fault_since = (
            lambda mark, consume=True:
                afcBambuAMS._ams_fault_since(shim, mark, consume))
        return shim

    def test_the_words_are_kept(self):
        shim = self._shim()
        assert afcBambuAMS._ams_declared_fault(shim) is True
        assert "TIMEOUT error 0" in shim._declared_fault_text

    def test_the_sequence_is_still_consumed(self):
        shim = self._shim(seq=4)
        afcBambuAMS._ams_declared_fault(shim)
        assert shim._fault_seen == 4
        # ...and a second look is not a second fault.
        assert afcBambuAMS._ams_declared_fault(shim) is False

    def test_nothing_new_leaves_the_text_alone(self):
        shim = self._shim(seq=0, text="")
        assert afcBambuAMS._ams_declared_fault(shim) is False
        assert shim._declared_fault_text is None

    def test_a_benign_bump_is_not_a_verdict(self):
        # A scan ending its pull-in advances the bridge's sequence and means
        # nothing; it must not become the reason a load is reported to have
        # failed.
        shim = self._shim(seq=9, text="[AMS_SWITCH]bldc stall exit")
        assert afcBambuAMS._ams_declared_fault(shim) is False
        assert shim._declared_fault_text is None


class TestAms1FaultReachesEveryErrorPath:
    """AMS 1 answers in STATE, not words. Every fault path in the unit reads
    the same bridge accessor, so covering it at the source covers them all --
    this pins that they still do, and that none grows its own word list."""

    def test_all_four_paths_read_last_fault(self):
        # Either directly, or through _ams_fault_since -- which reads it and is
        # pinned to below, so the chain still ends at the bridge. The unload
        # paths reach it the same way.
        import inspect
        for name in ("get_status", "_check_ams_fault",
                     "_ams_declared_fault", "_ack_faults",
                     "unit_unload_lane", "eject_lane"):
            src = inspect.getsource(getattr(afcBambuAMS, name))
            assert "last_fault" in src or "_ams_fault_s" in src, (
                f"{name} must take its faults from the bridge, or the AMS 1's "
                f"state-only give-up will be invisible to it")

    def test_the_shared_accessor_reads_the_bridge(self):
        import inspect
        for name in ("_ams_fault_seq", "_ams_fault_since"):
            assert "last_fault" in inspect.getsource(
                getattr(afcBambuAMS, name))

    def test_no_path_matches_fault_words_itself(self):
        # A path with its own "stall"/"timeout" list would silently exclude the
        # AMS 1 again, because AMS 1 never uses those words.
        import inspect
        for name in ("_check_ams_fault", "_ams_declared_fault", "_ack_faults"):
            src = inspect.getsource(getattr(afcBambuAMS, name))
            for word in ('"stall"', "'stall'", '"timeout error"'):
                assert word not in src, (
                    f"{name} matches fault text itself; the AMS 1 says none of "
                    f"those words -- keep detection in the bridge")


class TestTheFirmwareVerdictOutranksNarration:
    """
    fw >= 1.9.3.0 publishes a per-bay scan verdict in the status frame:
    scan_seq advances when the firmware's scan window resolves for THAT bay,
    scan_res says how (1 read / 2 foreign / 3 no tag).

    Why it exists, measured live with two boxed units: the narration stamps
    are bridge-wide (per address class at best) and both boxed units narrate
    as 0x0700 -- so a sibling's "read success" answered for this bay, and ANY
    cycle-end (including this unit's OWN insert-preload, which runs before
    the commanded scan) finalised this bay as no-tag. Back-to-back inserts on
    two units finalised the second as "no readable tag profile" with the
    tag's UID sitting in the same log line.
    """

    def _u(self, seq=None, res=None, base=None, read_ok=False, ended=False,
           t0=100.0, now=101.0):
        b = MagicMock()
        b.rfid_read_succeeded_since = lambda since, addr=None: read_ok
        b.rfid_cycle_ended_since = lambda since, addr=None: ended
        info = {"index": 0, "present": True}
        if seq is not None:
            info["scan_seq"] = seq
            info["scan_res"] = res
        u = types.SimpleNamespace(
            _bridge=b, _scan_t0=[t0], dry_dev_addr=0x0700,
            SCAN_FALLBACK_CAP=afcBambuAMS.SCAN_FALLBACK_CAP,
            SCAN_VERDICT_CAP=afcBambuAMS.SCAN_VERDICT_CAP,
            _slots=[info],
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: now)))
        if base is not None:
            u._scan_seq0 = {0: base}
        return u

    def test_a_resolved_read_is_read(self):
        u = self._u(seq=2, res=1, base=1)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_a_resolved_no_tag_is_notag(self):
        u = self._u(seq=2, res=3, base=1)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_a_foreign_tag_finalizes_to_defaults(self):
        # res 2 = a chip answered anticollision but auth failed. The verdict
        # is "notag" -- _finalize_scan's operator message is what tells a
        # third-party spool from an empty reader, not the verdict.
        u = self._u(seq=2, res=2, base=1)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_a_siblings_read_success_cannot_answer_for_this_bay(self):
        # THE cross-credit pin. The narration says a read succeeded (it was
        # the OTHER unit's), but this bay's firmware verdict has not resolved:
        # the answer is still "waiting", never "read".
        u = self._u(seq=1, res=1, base=1, read_ok=True)
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"

    def test_a_siblings_cycle_end_cannot_finalize_this_bay(self):
        # The mirror image: any unit's cycle-end used to finalize this bay as
        # no-tag -- the live "no readable tag profile" failure on back-to-back
        # inserts. With an unresolved firmware verdict it stays "waiting".
        u = self._u(seq=1, res=None, base=1, ended=True)
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"

    def test_reinserting_the_same_spool_still_reads(self):
        # Same spool back in the bay: the record bytes are identical, but the
        # firmware still bumps the seq when its window resolves -- the case
        # that sank every record-content comparison scheme.
        u = self._u(seq=2, res=1, base=1)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_first_resolution_after_boot_counts(self):
        # No baseline recorded (scan opened before any verdict existed):
        # seq 1 vs baseline None is an advance.
        u = self._u(seq=1, res=1)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_the_silence_backstop_survives_the_seq_path(self):
        # Firmware frames stopped mid-scan: the seq can never advance, and the
        # verdict must still fall back to no-tag rather than waiting forever.
        u = self._u(seq=1, res=None, base=1,
                    now=100.0 + afcBambuAMS.SCAN_VERDICT_CAP + 1.0)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_the_short_cap_does_not_preempt_the_firmware(self):
        # The 45 s narration-mode cap must NOT fire on the seq path: measured
        # live, a presence flap opened the hold early and the short cap
        # finalized defaults mid-preload -- a timer answering instead of the
        # unit. Past 45 s but inside SCAN_VERDICT_CAP the answer is still
        # "waiting".
        u = self._u(seq=1, res=None, base=1,
                    now=100.0 + afcBambuAMS.SCAN_FALLBACK_CAP + 5.0)
        u.SCAN_VERDICT_CAP = afcBambuAMS.SCAN_VERDICT_CAP
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"

    def test_old_firmware_falls_back_to_narration(self):
        # No scan_seq in the record (fw < 1.9.3.0): the narration stamps
        # decide, exactly as before.
        u = self._u(read_ok=True)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_open_scan_baselines_the_seq(self):
        u = self._u(seq=7, res=1)
        u._scan_notag = [False]
        u._scan_seq0 = {}
        afcBambuAMS._open_scan(u, 0)
        assert u._scan_seq0[0] == 7
        # A verdict resolved BEFORE this scan opened must not answer it.
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"


class TestMemoLifecycleOnPhysicalEdges:
    """
    The Spoolman memos are per-OCCUPANCY, not per-session: a spool physically
    moving is the only event that can change their answers, so the edges clear
    them. Live failure this pins: UID 0A1882AC stayed memoized as "no
    Spoolman match" while spool 87 verifiably carried it -- the operator's
    bind-then-reinsert workflow could never recover without a restart.
    """

    def test_forget_spares_the_binding_check(self):
        # The miss memo clears on physical edges; the binding-check memo does
        # NOT. Clearing it per edge re-probed Spoolman for every bound lane on
        # the next pass, and scan motion flaps edges in bursts -- blocking
        # HTTP on the reactor, the "Timer too close" class. It resets only
        # with the connection.
        u = types.SimpleNamespace(
            _spoolman_no_match={"AABBCCDD"},
            _binding_check={("87", "AABBCCDD"): False},
            _slots=[{}], SLOTS_PER_UNIT=4)
        afcBambuAMS._forget_spoolman_miss(u, 0)
        assert not u._spoolman_no_match
        assert u._binding_check == {("87", "AABBCCDD"): False}

    def test_a_binding_lookup_failure_is_not_memoized(self):
        # get_spool raising must not settle the (spool, uid) verdict: the next
        # status pass retries once Spoolman is back. Only real answers are
        # memoized.
        client = MagicMock()
        client.get_spool.side_effect = RuntimeError("down")
        import extras.AFC_BambuAMS as mod
        u = types.SimpleNamespace(afc=object())
        orig = mod._bambu_spoolman_client
        mod._bambu_spoolman_client = lambda afc: client
        try:
            assert afcBambuAMS._binding_contradicted(u, 87, "AABBCCDD") is False
            assert not getattr(u, "_binding_check", {})
            # Spoolman comes back with a real contradiction: the verdict now
            # lands (and memoizes) instead of being masked by the failed try.
            client.get_spool.side_effect = None
            client.get_spool.return_value = {
                "extra": {"card_uids": "\"11223344\""}}
            assert afcBambuAMS._binding_contradicted(u, 87, "AABBCCDD") is True
            assert u._binding_check[("87", "AABBCCDD")] is True
        finally:
            mod._bambu_spoolman_client = orig


class TestALateTagBeatsANoTagVerdict:
    """The bay's record can improve AFTER the unit has answered.

    "Two outcomes, never three" assumed it could not. Measured live on AMS 2
    bay 1: the scan resolved no-tag (its narration never reached the window),
    the background fill then re-read the bay and the record filled in with
    PLA Matte / A3D8E1 / uid ecb61cd0 / remain 100 -- while lane19 sat on
    PLA/1000 g defaults, because the no-tag latch blocks a bay from surfacing
    until the spool is physically pulled. The lane and the record disagreed,
    and the record was right.
    """

    def _u(self, info):
        lane = types.SimpleNamespace(name="lane19", status=None, spool_id=None)
        u = types.SimpleNamespace(
            name="BambuAMS_2", _slots=[info], _slot_map={"lane19": 0},
            lanes={"lane19": lane}, _scan_notag=[False], _afc_owned=set(), _prep_seen=True,
            _measure_settled=lambda slot, info: True,
            _scan_t0=[100.0], _prev_present=[True], SLOTS_PER_UNIT=4,
            unit_slots=4, logger=_Logger(), surfaced=[], finalized=[],
            _ACTIVE_STATES=afcBambuAMS._ACTIVE_STATES,
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: 200.0)))
        u._scan_verdict = lambda s: "notag"
        u._maybe_auto_scan = lambda s, p, i: None
        u._is_virtual_hub = lambda l: False
        u.lane_loaded = lambda l: None
        u.lane_not_ready = lambda l: None
        u.lane_illuminate_spool = lambda l: None
        u._drain_spool_summary = lambda s: None
        u._release_scan_hold = lambda s: u._scan_t0.__setitem__(s, None)
        u._finalize_scan = lambda s: u.finalized.append(s)
        u._surface_slot_info = lambda l, i: u.surfaced.append(i)
        return u, lane

    def test_a_record_with_a_tag_is_surfaced_despite_the_notag_verdict(self):
        u, lane = self._u({"index": 0, "present": True, "material": "PLA Matte",
                           "rfid_uid": "ecb61cd0", "color": "A3D8E1"})
        afcBambuAMS._sync_lanes(u)
        assert u.surfaced, "a real tag must reach the lane"
        assert not u.finalized, "defaults must not be applied over a real tag"
        assert u._scan_notag[0] is False   # latch cleared, not stuck

    def test_a_genuinely_empty_record_still_gets_defaults(self):
        u, lane = self._u({"index": 0, "present": True,
                           "material": "", "rfid_uid": None})
        afcBambuAMS._sync_lanes(u)
        assert u.finalized == [0]
        assert not u.surfaced

    def test_a_uid_without_a_profile_is_not_enough(self):
        # A chip that anticollided but never decoded is not a tag read; the
        # no-tag path owns that case and its message names it.
        u, lane = self._u({"index": 0, "present": True,
                           "material": "", "rfid_uid": "d34e4e39"})
        afcBambuAMS._sync_lanes(u)
        assert u.finalized == [0]
        assert not u.surfaced


class TestTheMeasurementIsAttributedByTheWindow:
    """AMS 1's measurement must never land on AMS 2's spool.

    Measured live: "[AMS_DEV] odom C:0.360,R:0.057,P:23%" from AMS 1 was
    adopted by AMS 2 -- which had a capacity window pending -- and written to
    Spoolman spool 109 as 230 g, recording a ~900 g reel as nearly empty. The
    narration arrives on device 0x0700 and BOTH boxed units answer there, so
    the address cannot attribute it; the firmware's capacity window can,
    because it was opened for a specific bay.
    """

    def _u(self, info):
        lane = types.SimpleNamespace(name="lane19", status=None, spool_id=109)
        u = types.SimpleNamespace(
            name="BambuAMS_2", _slots=[info], _slot_map={"lane19": 0},
            lanes={"lane19": lane}, _scan_notag=[False], _scan_t0=[None],
            _afc_owned=set(), _prep_seen=True,
            _measure_settled=lambda slot, info: True,
            _prev_present=[True], SLOTS_PER_UNIT=4, unit_slots=4,
            logger=_Logger(), adopted=[],
            _ACTIVE_STATES=afcBambuAMS._ACTIVE_STATES,
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: 200.0)))
        u._scan_verdict = lambda s: "none"
        u._maybe_auto_scan = lambda s, p, i: None
        u._is_virtual_hub = lambda l: False
        u.lane_loaded = lambda l: None
        u.lane_not_ready = lambda l: None
        u.lane_illuminate_spool = lambda l: None
        u._drain_spool_summary = lambda s: None
        u._surface_slot_info = lambda l, i: None
        u._adopt_measured_remain = lambda s, p, src="": u.adopted.append((s, p, src))
        return u

    def _info(self, **kw):
        base = {"index": 0, "present": True, "material": "PLA Matte",
                "rfid_uid": "ecb61cd0"}
        base.update(kw)
        return base

    def test_a_measurement_on_this_bay_is_adopted_once(self):
        u = self._u(self._info(meas_pct=23, meas_seq=1))
        afcBambuAMS._sync_lanes(u)
        afcBambuAMS._sync_lanes(u)          # same seq: already counted
        assert u.adopted == [(0, 23, "physical AMS measurement")]

    def test_a_fresh_measurement_of_the_same_value_is_new_news(self):
        info = self._info(meas_pct=23, meas_seq=1)
        u = self._u(info)
        afcBambuAMS._sync_lanes(u)
        info["meas_seq"] = 2                # measured again, same percent
        afcBambuAMS._sync_lanes(u)
        assert len(u.adopted) == 2

    def test_no_measurement_means_nothing_is_adopted(self):
        u = self._u(self._info(meas_pct=None, meas_seq=0))
        afcBambuAMS._sync_lanes(u)
        assert u.adopted == []

    def test_older_firmware_does_not_adopt_from_this_path(self):
        # No mpct/mseq in the frame at all: the narration path stays in charge.
        u = self._u(self._info())
        afcBambuAMS._sync_lanes(u)
        assert u.adopted == []


class TestEachModelOwnsItsBehaviour:
    """AMS 1, AMS 2 and the HT are three machines, not one with an if.

    Every assertion here is a regression that actually shipped tonight
    because a shared branch decided something model-specific.
    """

    def _u(self, model):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.ams_model = model
        return u

    def test_every_model_has_a_profile(self):
        for m in ("ams1", "ams2", "ht"):
            assert self._u(m).profile["fw_model"] in (0, 1, 2)

    def test_the_three_models_are_distinct_to_the_firmware(self):
        seen = {self._u(m).profile["fw_model"] for m in ("ams1", "ams2", "ht")}
        assert seen == {0, 1, 2}, "AMS 1 and AMS 2 must not share a row"

    def test_only_the_ht_is_ht(self):
        assert self._u("ht")._is_ht() is True
        assert self._u("ams1")._is_ht() is False
        assert self._u("ams2")._is_ht() is False

    def test_the_ht_is_never_pre_read(self):
        # A plain 0x211 to an HT answers instantly from its FLASH CACHE --
        # the previous spool on a swap. The boxed units answer for the bay
        # they were asked about.
        assert self._u("ht").profile["pre_read_safe"] is False
        assert self._u("ams1").profile["pre_read_safe"] is True
        assert self._u("ams2").profile["pre_read_safe"] is True

    def test_the_ht_scan_is_firmware_armed(self):
        # The module must NOT command an HT scan: its window is armed on the
        # insert edge in firmware, and a commanded read would collect the
        # flash cache before the unit has touched the new tag.
        assert self._u("ht").profile["commands_scan"] is False
        assert self._u("ams1").profile["commands_scan"] is True
        assert self._u("ams2").profile["commands_scan"] is True

    def test_slot_counts_are_per_model(self):
        assert self._u("ht").profile["slots"] == 1
        assert self._u("ams1").profile["slots"] == 4
        assert self._u("ams2").profile["slots"] == 4

    def test_an_unknown_model_falls_back_without_raising(self):
        # A typo in the config must not take a unit's identity with it.
        assert self._u("nonsense").profile["fw_model"] in (0, 1, 2)


class TestPrepKeepsWhatAFCRestored:
    """PREP reads sensors. AFC owns the filament data.

    Surfacing the bay's record at PREP is how one unit's tag reached
    another's lane at startup -- uid 01d0ec0f applied to BOTH units' slot 2,
    overwriting what AFC had correctly restored from vars. Every other unit
    type refuses that job: OpenAMS's prep sets prep_state from its sensors
    and touches no filament field.
    """

    def _lane(self, **kw):
        lane = types.SimpleNamespace(
            name="lane21", material="", color="", weight=0, spool_id=None,
            prep_state=False, loaded_to_hub=False, status=None,
            tool_loaded=False, index=3, map="T1")
        for k, v in kw.items():
            setattr(lane, k, v)
        return lane

    def _unit(self, lane, info):
        u = types.SimpleNamespace(
            name="BambuAMS_2", _slots=[info], surfaced=[],
            lanes={lane.name: lane}, logger=_Logger(),
            _bridge=MagicMock(), SLOTS_PER_UNIT=4, _afc_owned=set())
        u._slot_of = lambda l: 0
        u._unit_online = lambda latest: True
        u._is_virtual_hub = lambda l: False
        u.lane_loaded = lambda l: None
        u.lane_not_ready = lambda l: None
        u.lane_illuminate_spool = lambda l: None
        u._surface_slot_info = lambda l, i: u.surfaced.append(i)
        u._restore_sub_type = lambda l: None
        u.afc = types.SimpleNamespace(
            function=types.SimpleNamespace(TcmdAssign=lambda l: None))
        return u

    INFO = {"index": 0, "present": True, "material": "PLA Basic",
            "rfid_uid": "01d0ec0f", "color": "9CDBD9"}

    def test_a_lane_afc_restored_is_left_alone(self):
        lane = self._lane(material="PLA", spool_id=109, color="#A3D8E1")
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert u.surfaced == [], "PREP must not overwrite restored lane data"
        assert lane.spool_id == 109 and lane.color == "#A3D8E1"

    def test_a_spoolman_linked_lane_with_no_material_is_still_left_alone(self):
        # The binding is AFC's, re-hydrated from Spoolman at boot.
        lane = self._lane(spool_id=137)
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert u.surfaced == []
        assert lane.spool_id == 137

    def test_an_empty_lane_still_gets_filled(self):
        # Nothing to preserve, so a fresh install is not left blank.
        lane = self._lane()
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert u.surfaced == [self.INFO]

    def test_prep_still_sets_the_sensor_state(self):
        lane = self._lane(material="PLA", spool_id=109)
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert lane.prep_state is True      # the sensor half still runs

    def test_prep_marks_the_bay_so_the_status_path_defers_too(self):
        # Gating PREP alone moves the overwrite by one poll, no further.
        lane = self._lane(material="PLA", spool_id=109)
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert u._afc_owned == {0}

    def test_prep_marks_nothing_for_a_lane_it_filled_itself(self):
        lane = self._lane()
        u = self._unit(lane, self.INFO)
        afcBambuAMS.system_Test(u, lane, 0.0, False, False)
        assert u._afc_owned == set()


class TestTheStatusPathDefersToAFCUntilTheUnitAnswers:
    """The other half of the boot rule.

    PREP declining to surface only helps if the poll a second later declines
    too: with no scan open the verdict is "none" and the bay just reports its
    cached record, which is exactly the value PREP refused.
    """

    def _shim(self, verdict, owned, info=None, lane=None):
        lane = lane or types.SimpleNamespace(
            hub_obj=None, tool_loaded=False, prep_state=None,
            _load_state=None, loaded_to_hub=None, status=None,
            material="PLA", spool_id=109)
        info = info if info is not None else {
            "present": True, "material": "PLA Basic", "rfid_uid": "01d0ec0f"}
        s = _sync_shim({"l": 0}, {"l": lane}, [info], verdict=verdict)
        s.surfaced = []
        s._surface_slot_info = lambda ln, i: s.surfaced.append(i)
        s._afc_owned = set(owned)
        s._fill_missing_variant = lambda ln, i: None
        s._measure_settled = lambda slot, info: True
        s.logger = _Logger()
        s.name = "BambuAMS_2"
        s.lane = lane
        return s

    def test_a_marked_bay_is_not_surfaced_with_no_scan_open(self):
        s = self._shim("none", {0})
        afcBambuAMS._sync_lanes(s)
        assert s.surfaced == []

    def test_an_unmarked_bay_is_surfaced_as_before(self):
        s = self._shim("none", set())
        afcBambuAMS._sync_lanes(s)
        assert len(s.surfaced) == 1

    def test_a_read_outranks_the_mark_and_clears_it(self):
        # The unit answering for this bay beats anything restored from vars.
        s = self._shim("read", {0})
        s._log_tag_readout = lambda ln, i, force=False: None
        afcBambuAMS._sync_lanes(s)
        assert len(s.surfaced) == 1
        assert s._afc_owned == set()

    def test_pulling_the_spool_clears_the_mark(self):
        # AFC's data described a spool that is no longer there.
        s = self._shim("none", {0}, info={"present": False})
        afcBambuAMS._sync_lanes(s)
        assert s._afc_owned == set()

    def test_a_notag_verdict_also_clears_the_mark(self):
        s = self._shim("notag", {0},
                       info={"present": True, "material": "", "rfid_uid": ""})
        afcBambuAMS._sync_lanes(s)
        assert s._afc_owned == set()

    def test_nothing_is_written_before_prep_has_run(self):
        # Measured on the printer: "applied tag to lane21" landed five
        # seconds BEFORE "BambuAMS_2 Prepping lanes". AFC_prep restores the
        # var file and then calls system_Test, so these early polls read a
        # blank lane and would leave prep nothing to protect.
        s = self._shim("none", set())
        s._prep_seen = False
        afcBambuAMS._sync_lanes(s)
        assert s.surfaced == []

    def test_the_sensor_half_still_runs_before_prep(self):
        s = self._shim("none", set())
        s._prep_seen = False
        afcBambuAMS._sync_lanes(s)
        assert s.lane.prep_state is True


class TestTheVariantComesBackFromTheBay:
    """The card renders material + sub_type itself.

    Joining them in the lane's status produced "PLA Glow Glow" on the actual
    panel -- four Bambu lanes reading a bare "PLA" is a MISSING sub_type.
    Nothing outside this module can supply it: AFC's prep restores material,
    colour, weight and the Spoolman link and not the variant, and Spoolman
    has no sub_type field. The bay still has the whole string.
    """

    def _lane(self, **kw):
        lane = types.SimpleNamespace(
            name="lane15", material="PLA", sub_type="", spool_vendor="",
            filament_name="", send_lane_data=lambda: None)
        for k, v in kw.items():
            setattr(lane, k, v)
        return lane

    def _u(self):
        u = types.SimpleNamespace(name="BambuAMS_1", logger=_Logger(),
                                  saved=[])
        u._save_lane_vars = lambda: u.saved.append(True)
        u._uid_claimed_elsewhere = lambda uid: False
        return u

    def _fill(self, lane, material="PLA Sparkle"):
        afcBambuAMS._fill_missing_variant(self._u(), lane, {"material": material})
        return lane

    def test_the_variant_is_taken_from_the_record(self):
        assert self._fill(self._lane()).sub_type == "Sparkle"

    def test_the_material_is_not_touched(self):
        # The card joins the two; writing "PLA Sparkle" here doubles it.
        assert self._fill(self._lane()).material == "PLA"

    def test_a_variant_already_on_the_lane_wins(self):
        lane = self._fill(self._lane(sub_type="Matte"))
        assert lane.sub_type == "Matte"

    def test_a_different_material_cannot_decorate_the_lane(self):
        # A bay reporting PETG has nothing to say about a PLA lane.
        lane = self._fill(self._lane(), material="PETG HF")
        assert lane.sub_type == ""

    def test_a_record_with_no_variant_changes_nothing(self):
        assert self._fill(self._lane(), material="PLA").sub_type == ""

    def test_an_unknown_record_changes_nothing(self):
        assert self._fill(self._lane(), material="unknown").sub_type == ""

    def test_a_blank_record_changes_nothing(self):
        assert self._fill(self._lane(), material="").sub_type == ""

    def test_the_vendor_and_display_name_are_filled_in_too(self):
        lane = self._fill(self._lane())
        assert lane.spool_vendor == "Bambu"
        assert lane.filament_name == "Bambu PLA Sparkle"

    def test_an_existing_display_name_is_left_alone(self):
        lane = self._fill(self._lane(filament_name="My Custom Name"))
        assert lane.filament_name == "My Custom Name"

    def test_the_change_is_persisted_once(self):
        u = self._u()
        lane = self._lane()
        info = {"material": "PLA Sparkle"}
        afcBambuAMS._fill_missing_variant(u, lane, info)
        afcBambuAMS._fill_missing_variant(u, lane, info)   # sub_type is set
        assert u.saved == [True]


class TestABootDeferredBayStillGetsItsVariant:
    """The fill has to reach the lanes the boot rule is protecting -- those
    are exactly the ones AFC restored without a variant."""

    def test_a_marked_bay_is_filled_without_being_surfaced(self):
        lane = types.SimpleNamespace(
            name="lane15", hub_obj=None, tool_loaded=False, prep_state=None,
            _load_state=None, loaded_to_hub=None, status=None, material="PLA",
            spool_id=109, sub_type="", spool_vendor="", filament_name="",
            send_lane_data=lambda: None)
        info = {"present": True, "material": "PLA Sparkle",
                "rfid_uid": "04c07001"}
        s = _sync_shim({"l": 0}, {"l": lane}, [info], verdict="none")
        s.surfaced = []
        s._surface_slot_info = lambda ln, i: s.surfaced.append(i)
        s._afc_owned = {0}
        s._prep_seen = True
        s.logger = _Logger()
        s.name = "BambuAMS_1"
        s._save_lane_vars = lambda: None
        s._uid_claimed_elsewhere = lambda uid: False
        s._fill_missing_variant = afcBambuAMS._fill_missing_variant.__get__(s)
        afcBambuAMS._sync_lanes(s)
        assert s.surfaced == []          # the boot rule still holds
        assert lane.sub_type == "Sparkle"
        assert lane.material == "PLA"    # and only the variant moved


class TestAMS2WaitsForItsMeasurement:
    """AMS 2 announces the read BEFORE it calibrates, and applying the tag
    right then stops the calibration.

    Two cycles, same unit, same spool, read line by line:

      07:00 SUCCEEDED -- the card was already in range so the read was silent
      ("cali read tray 1", no read phrase), the module never fired an apply,
      and nothing but our capacity enables sat between "first detected"
      (07:00:33) and "second detected ... P:87%" (07:00:40).

      12:03 STALLED -- "read success,valid" at 12:03:15, apply and Spoolman
      round-trip at 12:03:15.141, calibration started 12:03:18, and mid-window
      at 12:03:35 "STEP:rfid pull 0" (which in the good cycle came AFTER
      "odom calib success"). Five seconds later "cali end", no percentage.
    """

    def _u(self, model, mseq0=None, mseq=None, ended=False):
        u = types.SimpleNamespace(
            name="BambuAMS_2", ams_model=model, logger=_Logger(),
            _slot_map={"lane19": 0}, _scan_t0=[100.0], dry_dev_addr=0x0700,
            _meas_seq0={0: mseq0}, _measure_wait_said=set(),
            _bridge=types.SimpleNamespace(
                rfid_cycle_ended_since=lambda since, addr=None: ended))
        u.profile = afcBambuAMS._PROFILES[model]
        return u

    def _settled(self, u, mseq=None):
        return afcBambuAMS._measure_settled(u, 0, {"meas_seq": mseq})

    def test_ams2_holds_the_lane_after_a_read(self):
        assert self._settled(self._u("ams2")) is False

    def test_ams2_applies_once_the_measurement_lands(self):
        # meas_seq moved past what _open_scan baselined.
        assert self._settled(self._u("ams2", mseq0=4), mseq=5) is True

    def test_a_measurement_that_has_not_moved_is_not_this_scans(self):
        assert self._settled(self._u("ams2", mseq0=4), mseq=4) is False

    def test_ams2_applies_when_the_unit_ends_the_cycle_without_one(self):
        # "cali end" with no percentage: finished, nothing coming.
        assert self._settled(self._u("ams2", ended=True)) is True

    def test_ams1_never_waits(self):
        # Nine clean measurements in the same log with the apply going out
        # mid-cycle. It does not need the wait, so it does not take it.
        assert self._settled(self._u("ams1")) is True

    def test_the_ht_never_waits(self):
        assert self._settled(self._u("ht")) is True

    def test_the_wait_is_announced_once_not_per_status_frame(self):
        u = self._u("ams2")
        for _ in range(5):
            self._settled(u)
        said = [m for k, m in u.logger.messages
                if k == "info" and "holding the lane" in m]
        assert len(said) == 1

    def test_the_wait_names_the_lane(self):
        u = self._u("ams2")
        self._settled(u)
        assert any("lane19" in m for _, m in u.logger.messages)

    def test_a_broken_bridge_helper_never_blocks_the_lane(self):
        u = self._u("ams2")
        u._bridge.rfid_cycle_ended_since = MagicMock(side_effect=RuntimeError)
        assert self._settled(u) is True

    def test_a_new_scan_gets_a_fresh_announcement(self):
        u = self._u("ams2")
        self._settled(u)
        u._scan_t0 = [None]
        u._slots = [{}]
        u._scan_notag = [True]
        u._bridge = None
        u.afc = types.SimpleNamespace(
            reactor=types.SimpleNamespace(monotonic=lambda: 200.0,
                                          register_timer=lambda *a, **k: None,
                                          NEVER=9e99, update_timer=lambda *a: None))
        try:
            afcBambuAMS._open_scan(u, 0)
        except Exception:
            pass
        assert 0 not in u._measure_wait_said


class TestADuplicatedUIDIsNotEvidence:
    """A tag is one chip in one bay. Two units cannot both be reading it.

    Measured after a reflash: the bridge's chain map came back "index 0:
    A9CD... <- slot0=PLA Matte, slot2=PLA Matte" and "index 1: 68273B... <-
    slot0=PLA Matte, slot2=PLA Matte" -- identical records for two units,
    while AMS 1 physically held Sparkle and Basic. The UID pin was right and
    the records behind it were not, and the variant fill wrote "Matte" over
    lane15's Sparkle on the strength of it.
    """

    def _pair(self, other_slots):
        a = afcBambuAMS.__new__(afcBambuAMS)
        b = afcBambuAMS.__new__(afcBambuAMS)
        b._slots = other_slots
        a.afc = types.SimpleNamespace(units={"BambuAMS_1": a, "BambuAMS_2": b})
        return a

    def test_a_uid_another_unit_also_claims_is_refused(self):
        a = self._pair([{"present": True, "rfid_uid": "13f56d32"}])
        assert a._uid_claimed_elsewhere("13f56d32") is True

    def test_case_and_padding_do_not_hide_the_duplicate(self):
        a = self._pair([{"present": True, "rfid_uid": " 13F56D32 "}])
        assert a._uid_claimed_elsewhere("13f56d32") is True

    def test_a_uid_only_this_unit_claims_is_fine(self):
        a = self._pair([{"present": True, "rfid_uid": "ecb61cd0"}])
        assert a._uid_claimed_elsewhere("04c07001") is False

    def test_an_empty_bay_elsewhere_does_not_count(self):
        # A stale record on a bay with nothing in it claims nothing.
        a = self._pair([{"present": False, "rfid_uid": "04c07001"}])
        assert a._uid_claimed_elsewhere("04c07001") is False

    def test_no_uid_is_never_a_duplicate(self):
        a = self._pair([{"present": True, "rfid_uid": "04c07001"}])
        assert a._uid_claimed_elsewhere("") is False
        assert a._uid_claimed_elsewhere(None) is False

    def test_non_bambu_units_are_ignored(self):
        a = afcBambuAMS.__new__(afcBambuAMS)
        ace = types.SimpleNamespace(_slots=[{"present": True,
                                             "rfid_uid": "04c07001"}])
        a.afc = types.SimpleNamespace(units={"a": a, "Ace2_1": ace})
        assert a._uid_claimed_elsewhere("04c07001") is False

    def test_the_fill_refuses_a_duplicated_record(self):
        u = afcBambuAMS.__new__(afcBambuAMS)
        other = afcBambuAMS.__new__(afcBambuAMS)
        other._slots = [{"present": True, "rfid_uid": "13f56d32"}]
        u.afc = types.SimpleNamespace(units={"a": u, "b": other})
        u.name = "BambuAMS_1"
        u.logger = _Logger()
        u._save_lane_vars = lambda: None
        lane = types.SimpleNamespace(name="lane15", material="PLA",
                                     sub_type="", spool_vendor="",
                                     filament_name="",
                                     send_lane_data=lambda: None)
        u._fill_missing_variant(lane, {"material": "PLA Matte",
                                       "rfid_uid": "13f56d32"})
        assert lane.sub_type == ""      # Sparkle is not overwritten by Matte


class TestAConfirmationIsNotANoOpTheFirstTime:
    """The chain order is not stable across reboots.

    Both boxed units answer on 0x0700, so enrollment order is whoever
    replies first, and the log has it flipping: A9CD=1/68273B=0 at 07:22,
    flipped at 07:37, back for ten boots from 11:09, flipped again at 12:13.

    Before resolution every unit sits on its config ams_index (default 0),
    so both boxed units consume index 0's frames. The one that pins
    elsewhere clears its cache; the one that CONFIRMS the index it was
    already assuming used to keep it -- and on a flip boot that cache is the
    other unit's. That is the 12:13 boot, where both units reported
    slot0=PLA Matte / slot2=PLA Matte while AMS 1 held Sparkle and Basic.
    """

    def _shim(self, resolved):
        sent = []
        u = types.SimpleNamespace(
            name="BambuAMS_1", ams_index=0, unit_uid="A9CD", SLOTS_PER_UNIT=4,
            _dry_id_follows_index=False, _id_resolved=resolved,
            _announce_deferred=False, _announce_defer_warned=False,
            _announce_unit=lambda: None, ams_model="ams1",
            logger=types.SimpleNamespace(info=lambda m: None,
                                         debug=lambda m: None,
                                         warning=lambda m: None),
            _bridge=types.SimpleNamespace(send=lambda o: sent.append(o)))
        u._slots = [{"present": True, "rfid_uid": "ecb61cd0",
                     "material": "PLA Matte"}, {}, {}, {}]
        u._send_ht_flag = lambda b: None
        u._send_mc_addr = lambda b: None
        u._is_ht = lambda: False
        return u, sent

    def test_the_first_confirmation_drops_the_unverified_cache(self):
        u, sent = self._shim(resolved=False)
        afcBambuAMS._adopt_index(u, 0)          # same index it was assuming
        assert u._slots == [{}, {}, {}, {}]

    def test_the_first_confirmation_re_seeds_from_the_verified_index(self):
        u, sent = self._shim(resolved=False)
        afcBambuAMS._adopt_index(u, 0)
        assert {"cmd": "status"} in sent

    def test_a_later_confirmation_keeps_the_cache(self):
        # Re-confirmations happen at every reconnect; the index was already
        # verified, so the records under it are this unit's.
        u, sent = self._shim(resolved=True)
        afcBambuAMS._adopt_index(u, 0)
        assert u._slots[0]["rfid_uid"] == "ecb61cd0"

    def test_the_index_is_marked_resolved_either_way(self):
        u, _ = self._shim(resolved=False)
        afcBambuAMS._adopt_index(u, 0)
        assert u._id_resolved is True

    def test_a_changed_pin_still_clears_as_before(self):
        u, sent = self._shim(resolved=True)
        afcBambuAMS._adopt_index(u, 1)
        assert u._slots == [{}, {}, {}, {}]
        assert u.ams_index == 1


class TestTheIdentityTableIsPushedAndPersisted:
    """Class, order and MODEL all arrive when Klipper connects, and the Pico
    enumerates the chain at power-up -- so the first enrollment knows none of
    them. Keyed by UID they are knowable before a unit has an index, which is
    what lets the firmware keep them in flash and be right first time.
    """

    def _unit(self, name, uid, model, is_ht=False):
        u = types.SimpleNamespace(unit_uid=uid, ams_model=model)
        u._is_ht = lambda: is_ht
        u.profile = afcBambuAMS._PROFILES[model]
        return (name, u)

    def _send(self, units):
        sent = []
        shim = types.SimpleNamespace(
            printer=types.SimpleNamespace(lookup_objects=lambda k: units))
        bridge = types.SimpleNamespace(send=lambda o: sent.append(o))
        afcBambuAMS._send_bindings(shim, bridge)
        return sent

    ALL = None

    def _all(self):
        return [self._unit("BambuAMS_1", "A" * 24, "ams1"),
                self._unit("BambuAMS_2", "B" * 24, "ams2"),
                self._unit("BambuAMS_HT", "C" * 24, "ht", is_ht=True)]

    def test_every_unit_is_bound_with_its_model(self):
        binds = [o for o in self._send(self._all()) if o.get("cmd") == "bind"]
        assert [(b["idx"], b["m"]) for b in binds] == [(0, 0), (1, 1), (2, 2)]

    def test_boxed_units_come_before_hts(self):
        binds = [o for o in self._send(self._all()) if o.get("cmd") == "bind"]
        assert binds[-1]["uid"] == "C" * 24      # the HT is last

    def test_the_save_is_requested_once_after_the_binds(self):
        sent = self._send(self._all())
        assert [o.get("cmd") for o in sent].count("idsave") == 1
        assert sent[-1] == {"cmd": "idsave"}     # after every bind

    def test_an_unknown_model_still_binds_without_raising(self):
        # A typo in ams_model must not cost the unit its place in the chain.
        name, u = self._unit("BambuAMS_1", "A" * 24, "ams1")
        u.profile = {}                            # no fw_model key
        binds = [o for o in self._send([(name, u)]) if o.get("cmd") == "bind"]
        assert binds[0]["m"] in (0, 1, 2)

    def test_a_unit_without_a_uid_is_skipped(self):
        units = [self._unit("BambuAMS_1", "", "ams1")] + self._all()[1:]
        binds = [o for o in self._send(units) if o.get("cmd") == "bind"]
        assert all(b["uid"] for b in binds)
        assert len(binds) == 2

    def test_nothing_is_sent_when_no_unit_has_a_uid(self):
        units = [self._unit("BambuAMS_1", "", "ams1")]
        assert self._send(units) == []


class TestTheVariantComesBackFromTheVarFile:
    """get_status writes sub_type on every save and AFC_prep never reads it
    back, so every restart returns a lane with its variant blank -- lane23
    came back "PLA" instead of "PLA Glow".

    _fill_missing_variant covers this from the bay's record, but only when
    there IS one: the HT stopped re-scanning on boot (a reboot is not an
    insert), so nothing refreshed its record and the variant stayed lost. The
    value was in the var file the whole time.
    """

    def _u(self, tmp_path, saved):
        import json as _json
        var = str(tmp_path / "AFC")
        with open(var + ".unit", "w") as fh:
            _json.dump({"BambuAMS_HT": {"lane23": saved}}, fh)
        u = types.SimpleNamespace(
            name="BambuAMS_HT",
            afc=types.SimpleNamespace(VarFile=var))
        return u

    def _lane(self, sub=""):
        return types.SimpleNamespace(name="lane23", sub_type=sub)

    def test_the_variant_is_restored(self, tmp_path):
        u = self._u(tmp_path, {"material": "PLA", "sub_type": "Glow"})
        lane = self._lane()
        afcBambuAMS._restore_sub_type(u, lane)
        assert lane.sub_type == "Glow"

    def test_a_variant_already_on_the_lane_wins(self, tmp_path):
        # It came from a tag this session; a stored string is not closer.
        u = self._u(tmp_path, {"sub_type": "Glow"})
        lane = self._lane(sub="Matte")
        afcBambuAMS._restore_sub_type(u, lane)
        assert lane.sub_type == "Matte"

    def test_a_blank_stored_value_changes_nothing(self, tmp_path):
        u = self._u(tmp_path, {"material": "PLA", "sub_type": ""})
        lane = self._lane()
        afcBambuAMS._restore_sub_type(u, lane)
        assert lane.sub_type == ""

    def test_a_missing_lane_entry_changes_nothing(self, tmp_path):
        u = self._u(tmp_path, {"sub_type": "Glow"})
        lane = types.SimpleNamespace(name="lane99", sub_type="")
        afcBambuAMS._restore_sub_type(u, lane)
        assert lane.sub_type == ""

    def test_a_missing_var_file_never_raises(self):
        u = types.SimpleNamespace(
            name="BambuAMS_HT",
            afc=types.SimpleNamespace(VarFile="/nonexistent/AFC"))
        lane = self._lane()
        afcBambuAMS._restore_sub_type(u, lane)      # no raise
        assert lane.sub_type == ""

    def test_bad_json_never_raises(self, tmp_path):
        var = str(tmp_path / "AFC")
        open(var + ".unit", "w").write("{not json")
        u = types.SimpleNamespace(
            name="BambuAMS_HT", afc=types.SimpleNamespace(VarFile=var))
        lane = self._lane()
        afcBambuAMS._restore_sub_type(u, lane)      # no raise
        assert lane.sub_type == ""


class TestAMeasurementOutlivesTheSession:
    """lane.weight was set and never persisted. A Spoolman-bound lane survived
    a restart through re-hydration -- lane19's 900 g came back -- so this was
    invisible there, and not on lane15, whose measured 250 g reverted to the
    stored 220 the moment Klipper restarted.
    """

    def _u(self, lane):
        u = types.SimpleNamespace(
            name="BambuAMS_1", logger=_Logger(), saves=[],
            SLOTS_PER_UNIT=4, _slot_map={"lane15": 0},
            lanes={"lane15": lane}, _meas_pct={}, _spoolman_push=True)
        u._save_lane_vars = lambda: u.saves.append(True)
        u._queue_spool_summary = lambda *a: None
        u._push_measured_to_spoolman = lambda l, g: None
        u._lane_for_slot = lambda s: lane
        return u

    def test_a_measured_weight_is_persisted(self):
        lane = types.SimpleNamespace(name="lane15", weight=220.0,
                                     spool_id=None)
        u = self._u(lane)
        u._slots = [{"weight": 1000}]
        ok = afcBambuAMS._adopt_measured_remain(u, 0, 25, "test")
        if ok:                                  # the path that sets weight
            assert lane.weight == 250
            assert u.saves == [True], "a measurement must outlive the session"


class TestTheTagsStoredRemainNeverSetsTheWeight:
    """A MEASUREMENT OUTRANKS THE TAG. The tag still beats nothing at all.

    _surface_slot_info scaled the nominal by info["remain_pct"] -- the
    percentage written on the tag by whatever last wrote it, not this spool's
    current contents. It overwrote lane15's MEASURED 250 g with the tag's 80%
    of 1000 = 800 g, and once measurements began persisting properly that
    wrong number reached the var file instead of evaporating at restart.
    That case is still asserted below and must never come back.

    The fix over-corrected, though: it barred the tag from the weight
    ENTIRELY. An AMS 2 fast-paths any card it recognises -- "odom calib
    success exit 0", its stored per-tray calibration confirmed on one edge,
    no percent published, which is the unit's design and what a real printer
    does too. On that path no measurement ever arrives, so a 60%-remaining
    PETG reel sat on its lane as a full 1000 g indefinitely while the console
    printed the honest 60% two lines earlier.

    So the rule is precedence, not prohibition: a measurement wins whenever
    there is one (_measured_remain), and the tag seeds the weight only when
    there is not. A measurement landing later still overwrites.
    """

    def _apply(self, lane_weight, remain_pct, nominal=1000):
        lane = types.SimpleNamespace(
            name="lane15", material="", sub_type="", color="", spool_vendor="",
            filament_name="", weight=lane_weight, spool_id=None,
            extruder_temp=None, multi_color=[], bambu_sku="")
        u = types.SimpleNamespace(name="BambuAMS_1", logger=_Logger())
        u._save_lane_vars = lambda: None
        u._spoolman_sync = lambda *a, **k: None
        u._log_tag_readout = lambda *a, **k: None
        u._apply_remain_weight = lambda l, i: None
        afcBambuAMS._surface_slot_info(u, lane, {
            "material": "PLA Sparkle", "color": "2D2B28", "weight": nominal,
            "remain_pct": remain_pct, "rfid_uid": "04c07001"})
        return lane

    def test_a_measured_weight_is_not_overwritten_by_the_tag(self):
        # The exact lane15 case: 250 g measured, tag says 80%.
        assert self._apply(250, 80).weight == 250

    def test_a_lane_on_the_nominal_takes_the_tag_percent_when_unmeasured(self):
        # A lane sitting at exactly the nominal is a lane carrying a DEFAULT,
        # not a claim. With no measurement for this bay the tag's 80% is the
        # best thing known about the spool, and reporting a part-used reel as
        # full is its own wrong answer.
        assert self._apply(1000, 80).weight == 800

    def test_an_empty_lane_takes_the_tag_percent_when_unmeasured(self):
        assert self._apply(0, 80).weight == 800

    def test_a_measurement_still_outranks_the_tag_on_a_nominal_lane(self):
        # The precedence that matters: same nominal lane, same 80% tag, but
        # this bay HAS an adopted measurement -- the tag must not touch it.
        lane = types.SimpleNamespace(
            name="lane15", material="", sub_type="", color="", spool_vendor="",
            filament_name="", weight=1000, spool_id=None,
            extruder_temp=None, multi_color=[], bambu_sku="")
        u = types.SimpleNamespace(name="BambuAMS_1", logger=_Logger())
        u._save_lane_vars = lambda: None
        u._spoolman_sync = lambda *a, **k: None
        u._log_tag_readout = lambda *a, **k: None
        u._apply_remain_weight = lambda l, i: None
        u._measured_remain = {2: 25}          # 25% measured for this slot
        afcBambuAMS._surface_slot_info(u, lane, {
            "material": "PLA Sparkle", "color": "2D2B28", "weight": 1000,
            "remain_pct": 80, "rfid_uid": "04c07001", "index": 2})
        assert lane.weight == 1000            # untouched by the tag

    def test_a_zero_remain_tag_still_seeds_the_nominal(self):
        assert self._apply(0, 0).weight == 1000

    def test_the_profile_is_still_applied_to_an_unlinked_lane(self):
        # No Spoolman match is not a reason to leave the lane blank -- the tag
        # is what we know about the spool, so all of it goes on.
        lane = self._apply(0, 80)
        assert lane.material == "PLA"
        assert lane.sub_type == "Sparkle"
        assert lane.color == "#2D2B28"
        assert lane.spool_vendor == "Bambu"
        assert lane.filament_name == "Bambu PLA Sparkle"


class TestTheTareBelongsToTheSpoolNotTheLane:
    """
    ``empty_spool_weight`` is written from Spoolman's ``spool_weight`` when a
    lane binds (AFC_spool), so dropping the link has to put the CONFIGURED
    value back -- otherwise the lane keeps a departed spool's tare, and since
    the var file persists it, "keeps" means across every future restart.

    Measured on this printer: several Spoolman entries carry
    ``spool_weight: 1000`` -- the net filament weight typed into the empty-spool
    field -- so lane15, bound to nothing, measured 220 g and reported a total
    of 220 + 1000 g. It had held one of those spools once.
    """

    def _lane(self, tare=1000.0, configured=190):
        cfg = types.SimpleNamespace(
            getfloat=lambda k, d=None, **kw: configured)
        return types.SimpleNamespace(
            name="lane15", spool_id=87, empty_spool_weight=tare, _config=cfg)

    def _u(self):
        u = types.SimpleNamespace(
            name="BambuAMS_1",
            logger=types.SimpleNamespace(debug=lambda *a, **k: None))
        u._restore_config_tare = afcBambuAMS._restore_config_tare.__get__(u)
        u._unbind_spool = afcBambuAMS._unbind_spool.__get__(u)
        u._clear_lane_filament = afcBambuAMS._clear_lane_filament.__get__(u)
        return u

    def test_unbinding_gives_the_configured_tare_back(self):
        u, lane = self._u(), self._lane()
        u._unbind_spool(lane, "the bay is empty")
        assert lane.spool_id == ''
        assert lane.empty_spool_weight == 190

    def test_clearing_a_lane_does_it_too(self):
        # The no-tag path blanks the profile without going through _unbind.
        u, lane = self._u(), self._lane()
        u._clear_lane_filament(lane)
        assert lane.empty_spool_weight == 190

    def test_a_lane_already_bound_to_nothing_is_left_alone(self):
        # _unbind_spool returns early, so nothing is rewritten -- a lane the
        # operator set by hand keeps what they set.
        u, lane = self._u(), self._lane()
        lane.spool_id = None
        lane.empty_spool_weight = 250.0
        u._unbind_spool(lane)
        assert lane.empty_spool_weight == 250.0

    def test_a_lane_config_that_sets_its_own_tare_wins(self):
        u = self._u()
        lane = self._lane(configured=420)
        u._unbind_spool(lane)
        assert lane.empty_spool_weight == 420

    def test_a_lane_with_no_config_is_survived(self):
        # Duck-typed stand-ins have no _config; leaving the value alone is the
        # old behaviour and must not raise.
        u = self._u()
        lane = types.SimpleNamespace(name="lane15", spool_id=87,
                                     empty_spool_weight=1000.0)
        u._unbind_spool(lane)
        assert lane.spool_id == ''
        assert lane.empty_spool_weight == 1000.0


class TestAnUnlinkedLaneKeepsItsNameAcrossARestart:
    """
    _fill_missing_variant used to return the moment ``sub_type`` was set, and
    _restore_sub_type -- added later, at prep -- sets it from the var file. So
    on an UNLINKED lane the second fix switched the first one off:

        lane15  material 'PLA'  sub_type 'Sparkle'  filament_name ''
        lane23  material 'PLA'  sub_type 'Glow'     filament_name 'Bambu Glow'

    lane23 looked fine only because it is BOUND: set_spoolID re-hydrates it
    from Spoolman, which has no sub_type column, so its sub_type came back
    blank after prep and the fill still ran. lane15 has no spool to re-hydrate
    from, so nothing wiped its sub_type and the early return took the other two
    fields with it. filament_name is what the card displays.
    """

    def _u(self):
        u = types.SimpleNamespace(
            name="BambuAMS_1",
            logger=types.SimpleNamespace(info=lambda *a, **k: None,
                                         debug=lambda *a, **k: None),
            _uid_claimed_elsewhere=lambda uid: False,
            _save_lane_vars=lambda *a, **k: None)
        u._fill_missing_variant = afcBambuAMS._fill_missing_variant.__get__(u)
        return u

    def _info(self, material="PLA Sparkle", uid="04c07001"):
        return {"material": material, "rfid_uid": uid, "index": 0}

    def test_a_restored_sub_type_no_longer_blocks_the_name(self):
        u = self._u()
        lane = types.SimpleNamespace(name="lane15", material="PLA",
                                     sub_type="Sparkle", spool_vendor="",
                                     filament_name="")
        u._fill_missing_variant(lane, self._info())
        assert lane.filament_name, "the card's field was left blank"
        assert "Sparkle" in lane.filament_name
        assert lane.spool_vendor

    def test_a_lane_with_everything_is_left_alone(self):
        u = self._u()
        lane = types.SimpleNamespace(name="lane15", material="PLA",
                                     sub_type="Sparkle",
                                     spool_vendor="Somebody Else",
                                     filament_name="A Name I Chose")
        u._fill_missing_variant(lane, self._info())
        assert lane.filament_name == "A Name I Chose"
        assert lane.spool_vendor == "Somebody Else"

    def test_a_blank_lane_still_gets_the_variant(self):
        u = self._u()
        lane = types.SimpleNamespace(name="lane23", material="PLA",
                                     sub_type="", spool_vendor="",
                                     filament_name="")
        u._fill_missing_variant(lane, self._info("PLA Glow", "0a1882ac"))
        assert lane.sub_type == "Glow"
        assert lane.filament_name

    def test_a_bay_holding_another_material_decorates_nothing(self):
        # The guard that was already there, and must survive the wider gate.
        u = self._u()
        lane = types.SimpleNamespace(name="lane15", material="PLA",
                                     sub_type="Sparkle", spool_vendor="",
                                     filament_name="")
        u._fill_missing_variant(lane, self._info("PETG Basic"))
        assert lane.filament_name == ""

    def test_a_uid_two_units_both_claim_decorates_nothing(self):
        u = self._u()
        u._uid_claimed_elsewhere = lambda uid: True
        lane = types.SimpleNamespace(name="lane15", material="PLA",
                                     sub_type="Sparkle", spool_vendor="",
                                     filament_name="")
        u._fill_missing_variant(lane, self._info())
        assert lane.filament_name == ""


class TestTheSummaryWaitsForTheSpoolmanAnswer:
    """
    The measured-spool summary names the Spoolman binding, and _bind_by_uid_bg
    does that lookup off-reactor. So the bay's record can be complete while the
    fact the sentence depends on is still a round-trip away -- and 170 ms was
    enough to print the opposite of the truth:

        17:20:11.781  ... not linked to a Spoolman spool, so this is kept on
                      the lane only -- Spoolman has no spool carrying ECB61CD0
        17:20:11.950  matched lane19 to Spoolman spool 109 by UID ecb61cd0

    _spoolman_inflight holds exactly the UIDs whose lookup is running.
    """

    def _u(self, inflight=(), deadline=200.0, verdict="read"):
        # deadline in the FUTURE by default (reactor clock is 100.0), so these
        # exercise the hold rather than the backstop.
        said = []
        u = types.SimpleNamespace(
            name="BambuAMS_2",
            _slots=[{"index": 0, "material": "PLA Matte", "rfid_uid": "ecb61cd0"}],
            _pending_summary={0: (91, 910, 1000, deadline)},
            _spoolman_inflight=set(inflight),
            _scan_verdict=lambda s: verdict,
            _lane_for_slot=lambda s: types.SimpleNamespace(name="lane19"),
            afc=types.SimpleNamespace(
                reactor=types.SimpleNamespace(monotonic=lambda: 100.0)),
        )
        u._say_spool_summary = lambda *a, **k: said.append(a)
        u._drain_spool_summary = \
            afcBambuAMS._drain_spool_summary.__get__(u)
        return u, said

    def test_it_holds_while_the_lookup_is_running(self):
        u, said = self._u(inflight=["ecb61cd0"])
        u._drain_spool_summary(0)
        assert said == [], "spoke while the Spoolman answer was in flight"
        assert 0 in u._pending_summary, "dropped the summary instead of holding"

    def test_it_speaks_once_the_lookup_finishes(self):
        u, said = self._u(inflight=["ecb61cd0"])
        u._drain_spool_summary(0)
        u._spoolman_inflight.clear()          # the lookup came back
        u._drain_spool_summary(0)
        assert len(said) == 1
        assert 0 not in u._pending_summary

    def test_case_does_not_defeat_the_check(self):
        # The record carries lower case; the in-flight set is seeded from the
        # tag, which is upper. Matching literally would have missed every time.
        u, said = self._u(inflight=["ECB61CD0"])
        u._drain_spool_summary(0)
        assert said == []

    def test_an_unrelated_lookup_does_not_hold_this_bay(self):
        u, said = self._u(inflight=["0a1882ac"])
        u._drain_spool_summary(0)
        assert len(said) == 1

    def test_the_backstop_still_wins(self):
        # A Spoolman that never answers must not silence the line for ever.
        u, said = self._u(inflight=["ecb61cd0"], deadline=50.0)  # already past
        u._drain_spool_summary(0)
        assert len(said) == 1, "the backstop stopped working"

    def test_no_uid_on_the_record_is_not_held(self):
        u, said = self._u(inflight=["ecb61cd0"])
        u._slots = [{"index": 0, "material": "PLA Matte", "rfid_uid": ""}]
        u._drain_spool_summary(0)
        assert len(said) == 1


class TestNoScanWhileAnythingIsMovingFilament:
    """A scan started beside a load shut the toolhead mcu down.

        23:38:09  Loading lane21                     (AMS 2, slot 2)
        23:38:28  spool INSERTED in slot 0 (BambuAMS_1)
        23:38:28  new spool detected in slot 0, scanning tag
        23:38:44  MCU 'EBBT0' shutdown: Timer too close

    A tag scan holds the bus for tens of seconds; run beside a load it
    starves Klipper's reactor until the toolhead misses its timers. The two
    guards in place both had a hole: in_print() (a bare TOOL_LOAD is not a
    print) and _unit_tool_loaded (asks about THIS unit -- the load was on the
    other one). The hazard is any filament moving anywhere.
    """

    def _unit(self, lane_states=(), in_toolchange=False):
        lanes = {f"lane{i}": types.SimpleNamespace(status=s)
                 for i, s in enumerate(lane_states)}
        afc = types.SimpleNamespace(in_toolchange=in_toolchange, lanes=lanes)
        return types.SimpleNamespace(afc=afc,
                                     _AFC_BUSY_STATES=afcBambuAMS._AFC_BUSY_STATES)

    def test_idle_lanes_are_not_busy(self):
        u = self._unit(("None", "Loaded", "Tool Loaded"))
        assert afcBambuAMS._afc_motion_busy(u) is False

    def test_a_load_on_ANOTHER_unit_is_busy(self):
        # The crash: BambuAMS_1's insert while BambuAMS_2's lane21 loaded.
        u = self._unit(("None", "Tool Loading"))
        assert afcBambuAMS._afc_motion_busy(u) is True

    def test_an_unload_is_busy(self):
        assert afcBambuAMS._afc_motion_busy(self._unit(("Tool Unloading",))) is True

    def test_a_toolchange_is_busy_even_with_quiet_lanes(self):
        u = self._unit(("None",), in_toolchange=True)
        assert afcBambuAMS._afc_motion_busy(u) is True

    def test_an_unanswerable_afc_is_treated_as_busy(self):
        # A false busy costs a few seconds; a false idle costs the print.
        class Boom:
            @property
            def in_toolchange(self):
                raise RuntimeError("AFC not answering")
        u = types.SimpleNamespace(afc=Boom(),
                                  _AFC_BUSY_STATES=afcBambuAMS._AFC_BUSY_STATES)
        assert afcBambuAMS._afc_motion_busy(u) is True

    def test_no_afc_at_all_is_not_busy(self):
        u = types.SimpleNamespace(afc=None,
                                  _AFC_BUSY_STATES=afcBambuAMS._AFC_BUSY_STATES)
        assert afcBambuAMS._afc_motion_busy(u) is False


from extras import AFC_BambuAMS_bridge as _brgmod  # noqa: E402


class TestTheConsoleReadsLikeEnglish:
    """The operator's console during a print was register dumps.

    Pasted verbatim off a live print -- these were essentially every line
    shown for minutes at a time, none of it actionable:

        [AMS_PMSM]mode:0->2                       assist motor cycling
        [AMS_LED]tray 1 loading                   bay LED restating itself
        [AMS_SWITCH]BUFF,pos:0.10->0.73,i:0.635A  buffer telemetry
        [AMS_COMMON]state:4,tray_now:1            "feeding", every frame

    All console-only: AFC_BambuAMS.log still keeps every line verbatim and
    every parser runs before this is decided.
    """

    NOISE = (
        "l [AMS_COMMON]state:4,tray_now:1,tray_exit:15 [AMS_LED]tray 1 loading"
        " [AMS_PMSM]mode:0->2 [AMS_PMSM]mode:2->0",
        "[AMS_PMSM]mode:0->2",
        "C [AMS_LED]tray 1 loading",
        "n [AMS_SWITCH]BUFF,pos:0.10->0.73,det:20mm,i:0.635A [AMS_PMSM]mode:2->0",
        "[AMS_COMMON]en:0,mode:0,idx:1,ref:0 [AMS_COMMON]preload_disable:1,"
        " tmpr:22.0, cd:0 [AMS_COMMON]state:0,tray_now:1,tray_exit:15",
    )

    # A fault, a load in progress, a loaded unit, a measurement and a stall.
    # Suppressing any of these is the failure mode that matters.
    MUST_SHOW = (
        "[AMS_COMMON]state:6,tray_now:255,tray_exit:15",
        "[AMS_COMMON]state:1,tray_now:1,tray_exit:15",
        "[AMS_COMMON]state:7,tray_now:1,tray_exit:15",
        "[AMS_RFID]STEP:odom C:0.478,R:0.076,P:79%, od:0.491",
        "[AMS_SWITCH]feed finish -1, stall, len_det:1.620 m, tube_len:3.506 m",
    )

    def test_print_time_chatter_is_console_suppressed(self):
        for line in self.NOISE:
            assert _brgmod._ams_is_noise(line), line

    def test_faults_and_results_are_never_suppressed(self):
        for line in self.MUST_SHOW:
            assert not _brgmod._ams_is_noise(line), line

    def _render(self, text):
        for pat, render in _brgmod._AMS_HUMAN:
            m = pat.search(text)
            if m:
                return render(m)
        return None

    def test_a_measurement_reads_as_a_sentence(self):
        out = self._render("[AMS_RFID]STEP:odom C:0.478,R:0.076,P:79%, od:0.491")
        assert "79% left" in out

    def test_the_stored_flash_value_is_surfaced(self):
        # The only place the unit says what it remembers -- at power-up.
        out = self._render("[AMS_RFID] STEP,odom load from flash 0,R:0.075,P:75")
        assert "bay 1" in out and "75%" in out

    def test_a_stall_names_both_distances(self):
        out = self._render(
            "[AMS_SWITCH]feed finish -1, stall, len_det:1.620 m, tube_len:3.506 m")
        assert "1.62 m" in out and "3.51 m" in out and "STALLED" in out

    def test_an_error_code_is_named_not_dumped(self):
        assert "0x17" in self._render("[AMS_LINK]err_code:0x00->0x17")
        assert "cleared" in self._render("[AMS_LINK]err_code:0x17->0x00")

    def test_chatter_carrying_a_heartbeat_is_still_chatter(self):
        # The AMS bundles its 10s "[DBG] ams time" liveness into whatever
        # frame is going out. The noise test used to run on the RAW text, so
        # any pure-chatter line that happened to carry a heartbeat matched no
        # rule and reached the console. Replaying the live log, that single
        # technicality accounted for 2,916 console lines -- more than every
        # other survivor combined.
        beat = " [DBG] ams time: now=42044054ms diff=10005ms"
        for line in self.NOISE:
            stripped = _brgmod._DBG_AMSTIME_RE.sub("", line + beat).strip()
            assert _brgmod._ams_is_noise(stripped), line

    def test_bays_are_one_based_for_humans(self):
        # The wire is 0-based; an operator counts bays from 1.
        assert "bay 2" in self._render("[AMS_RFID]STEP:odom save tray:1, R:0.0765")
        assert "bay 1" in self._render("[AMS_IDLE]tray 0 out,clear magic_num")
