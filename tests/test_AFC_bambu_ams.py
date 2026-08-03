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
            "temp_max": 240, "weight": 1000, "rfid_uid": None}

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
        # listeners. They used to go to python logging.debug, which klipper runs
        # at INFO, so they were discarded rather than merely kept off screen.
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
        shim.self_centres = True
        shim._send_selfcentre_flag = (
            afcBambuAMS._send_selfcentre_flag.__get__(shim))
        # MC poll addressing is pushed alongside the other per-unit config.
        shim.mc_dev_addr = 0x0700
        shim.mc_id_base = 0x00
        shim.mc_ams_id = -1              # -1 -> derive from base|index
        shim._send_mc_addr = afcBambuAMS._send_mc_addr.__get__(shim)
        shim._is_ht = afcBambuAMS._is_ht.__get__(shim)
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
    """Demand-gated follower re-engage: re-select only on real extrusion."""

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
            follow_min_extrude=0.4, follow_always=False,
            follow_when_loaded=False, follow_idle_ping=False,
            _follow_manual_off=False, _unload_in_progress=False,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            follow_poll_interval=0.3, ams_index=0, _slot_map={"lane1": 0},
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_first_sample_sets_baseline_no_ping(self):
        shim, state = self._shim(e_start=100.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == []              # baseline only
        assert shim._follow_last_e == 100.0

    def test_extrusion_beyond_threshold_pings(self):
        shim, state = self._shim(e_start=100.0)
        afcBambuAMS._follow_tick(shim, 1.0)     # baseline 100
        state["e"] = 100.5                       # extruded 0.5mm (> 0.4)
        afcBambuAMS._follow_tick(shim, 1.3)
        assert state["sent"] == [{"cmd": "follow"}]   # window refreshed
        assert shim._follow_last_e == 100.5

    def test_idle_does_not_ping(self):
        shim, state = self._shim(e_start=100.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        afcBambuAMS._follow_tick(shim, 1.3)     # no extrusion
        afcBambuAMS._follow_tick(shim, 1.6)
        assert state["sent"] == []               # silent at idle

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
            follow_always=False, follow_when_loaded=True,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            follow_idle_ping=False,      # exercise the demand-gated path
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

    def test_engages_when_tool_loaded_but_does_not_ping_without_extrusion(self):
        # Auto-arm still puts the tray in mode:4 so it is ready, but with no
        # extrusion the firmware window must stay shut. Pinging here is what
        # held the window open permanently and made the HT tick at ~20Hz.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=None)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["engaged"] == ["lane1"]           # auto-engaged
        assert state["sent"] == []                     # but NOT pinged
        assert shim._following_lane is lane

    def test_pings_once_the_extruder_advances(self):
        lane = types.SimpleNamespace(name="lane1")
        # Baseline already established below the new position, so this tick
        # sees real extrusion (1.0mm >= follow_min_extrude).
        shim, state = self._shim(loaded_lane=lane, following=lane, e=1.0,
                                 last_e=0.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]
        assert shim._follow_last_e == 1.0

    def test_retract_resets_baseline_without_pinging(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, e=2.0,
                                 last_e=5.0)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == []
        assert shim._follow_last_e == 2.0

    def test_follow_always_pings_regardless_of_extrusion(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane)
        shim.follow_always = True
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]

    def test_reasserts_mode4_when_dropped(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        afcBambuAMS._follow_tick(shim, 10.0)
        assert state["assist"] == [("lane1", True)]     # re-asserted mode:4
        assert state["sent"] == []                      # no idle ping

    def test_idle_state_0_does_not_reassert(self):
        # state:0 is the AMS RESTING, not a fault -- it arms, finishes its
        # assist, and sits at 0 until something wants filament. Re-arming on
        # state alone loops forever at ~2s, each one an LED flash and a motor
        # nudge, and it floods the console with assist acks.
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        shim._follow_last_demand = 0.0          # no recent extrusion
        shim.follow_rearm_window = 3.0
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["assist"] == []

    def test_state_0_with_recent_demand_does_reassert(self):
        lane = types.SimpleNamespace(name="lane1")
        shim, state = self._shim(loaded_lane=lane, following=lane, fstate=0)
        shim._follow_last_demand = 99.0         # extruded a moment ago
        shim.follow_rearm_window = 3.0
        afcBambuAMS._follow_tick(shim, 100.0)
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

    def test_no_active_answer_falls_back_to_shuttle(self):
        docked = self._lane("lane1", on_shuttle=False)
        u = self._unit([docked], current=None)
        assert afcBambuAMS._tool_loaded_lane(u) is None

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
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 1
        assert shim._drying is True

    def test_rotate_gated_when_lane_committed(self):
        lanes = {"a": self._lane("a", tool_loaded=True)}
        shim, sent, _ = _heater_shim(lanes)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 1})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 0
        assert any("ROTATE disabled" in m for m in gcmd.info)

    def test_temp_clamped_to_ceiling(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes)
        gcmd = _HeaterGcmd({"TEMP": 85, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == _mod.MAX_DRY_TEMP_C
        assert any("clamping" in m for m in gcmd.info)

    def test_ht_ceiling_allows_85(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3")
        shim.dry_max_temp = 85                              # AMS HT
        gcmd = _HeaterGcmd({"TEMP": 85, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == 85                        # not clamped
        assert not any("clamping" in m for m in gcmd.info)

    def test_ht_addressing_sent(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3", ams_index=2)
        shim.dry_dev_addr = 0x1800                          # AMS HT device addr
        shim.dry_ams_id = 2                                 # HT id = chain index
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["addr"] == 0x1800
        assert sent[0]["amsid"] == 2

    def test_ams2pro_addressing_default(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_2", ams_index=1)
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
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
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["temp"] == 85
        assert any("clamping" in m for m in gcmd.info)

    def test_stop_carries_ht_addressing(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_3", ams_index=2)
        shim.dry_dev_addr = 0x1800
        shim.dry_ams_id = 2
        shim._drying = True
        gcmd = _HeaterGcmd({})
        afcBambuAMS.cmd_BAMBU_HEATER_STOP(shim, gcmd)
        assert sent[0]["on"] == 0
        assert sent[0]["addr"] == 0x1800                    # HT hears the stop
        assert sent[0]["amsid"] == 2
        assert shim._drying is False

    def test_ignored_on_heaterless_unit(self):
        lanes = {"a": self._lane("a")}
        shim, sent, _ = _heater_shim(lanes, name="BambuAMS_1")
        shim.has_heater = False
        gcmd = _HeaterGcmd({"TEMP": 55, "TIME": 480, "ROTATE": 0})
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent == []                                   # nothing sent
        assert shim._drying is False
        assert any("no drying heater" in m for m in gcmd.info)


def _vhub(virtual=True):
    return types.SimpleNamespace(is_virtual_pin=lambda: virtual)


def _sync_shim(slot_map, lanes, slots):
    """Duck-typed self for afcBambuAMS._sync_lanes (no Klipper needed)."""
    return types.SimpleNamespace(
        _slot_map=slot_map, lanes=lanes, _slots=slots,
        _ACTIVE_STATES=afcBambuAMS._ACTIVE_STATES,
        _is_virtual_hub=afcBambuAMS._is_virtual_hub,
        _maybe_auto_scan=lambda slot, present, info: None,
        lane_loaded=lambda lane: None,
        lane_not_ready=lambda lane: None,
        lane_illuminate_spool=lambda lane: None,
        _surface_slot_info=lambda lane, info: None)


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
        _bridge=object(),
        logger=_Logger(),
        afc=types.SimpleNamespace(
            function=types.SimpleNamespace(in_print=lambda: in_print),
            reactor=reactor),
        _finalize_scan=lambda s: None,
        scan=lambda slot: scans.append(slot))
    shim._is_ht = afcBambuAMS._is_ht.__get__(shim)
    shim._lane_for_slot = afcBambuAMS._lane_for_slot.__get__(shim)
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
    calls = {"stop": 0, "feed": [], "sensor": 0}

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
        _finish_seq_now=lambda: 0,
        _finish_since=finish_since,
        stop=lambda: calls.__setitem__("stop", calls["stop"] + 1),
        feed=lambda lane, mm: calls["feed"].append(mm))
    return shim, calls, clock


_LANE = types.SimpleNamespace(name="lane1")


class TestRecover:
    """BAMBU_RECOVER / eject-based recovery of a stuck load."""

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
        assert cmd == "BAMBU_RECOVER UNIT=BambuAMS_1 LANE=lane3"


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
    def test_already_at_sensor_stops_and_returns(self):
        shim, calls, _ = _load_shim(sensor_after=0)      # triggers immediately
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True
        assert calls["stop"] == 1                        # halted right away
        assert calls["feed"] == []                       # no feed needed

    def test_stops_the_instant_sensor_triggers(self):
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
        shim._adopt_measured_path = (
            lambda: afcBambuAMS._adopt_measured_path(shim))
        lane = types.SimpleNamespace(name="lane1", loaded_to_hub=False)
        return shim, lane, calls

    def test_stalled_load_rehomes_then_fails(self):
        shim, lane, calls = self._shim(attempts=2)
        assert afcBambuAMS.unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 2                # one re-home per recover attempt
        assert len(calls["feed"]) == 3            # initial feed + 2 re-feeds
        assert calls["fail"] == 1                 # reported once, after exhausting

    def test_recover_disabled_fails_immediately(self):
        shim, lane, calls = self._shim(attempts=0)
        assert afcBambuAMS.unit_load_lane(shim, lane) is False
        assert calls["rehome"] == 0               # no re-home when disabled
        assert len(calls["feed"]) == 1           # just the initial feed
        assert calls["fail"] == 1


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
        _wait_move=lambda d, s=None: (order.append(("wait", d)), True)[1],
        measured_path_mm=lambda: None,
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


def _surface_self():
    return types.SimpleNamespace(name="AMS", logger=_Logger(), afc=None)


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
        # the AMS tag is authoritative for the bay -- it wins over a value we
        # previously auto-set (e.g. an AFC default), which was the bug.
        lane = types.SimpleNamespace(name="lane1", material="ABS",
                                     color="#FF0000", extruder_temp=250.0)
        info = bridge_slot_to_info({
            "i": 0, "present": True, "material": "PLA", "sku": "GFA00",
            "color": "00ae42ff", "tmin": 210, "tmax": 230})
        afcBambuAMS._surface_slot_info(_surface_self(), lane, info)
        assert lane.material == "PLA"                # tag wins
        assert lane.color == "#00AE42"
        assert lane.extruder_temp == 210.0

    def test_spoolman_linked_lane_not_overridden(self):
        lane = types.SimpleNamespace(name="lane1", material="ABS",
                                     color="#FF0000", extruder_temp=250.0,
                                     spool_id=42)
        info = bridge_slot_to_info({
            "i": 0, "present": True, "material": "PLA", "color": "00ae42ff",
            "tmin": 210, "tmax": 230})
        afcBambuAMS._surface_slot_info(_surface_self(), lane, info)
        assert lane.material == "ABS"                # Spoolman is authoritative
        assert lane.color == "#FF0000"

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
    """BAMBU_FOLLOWER ENABLE=0 must survive the auto-arm, which otherwise
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
            follow_always=False, follow_when_loaded=True,
            follow_idle_ping=False,
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
        # BAMBU_FOLLOWER ENABLE=0 clears _following_lane. Fault detection used
        # to hang off that, so stopping the follower silently stopped stall
        # detection -- in the state most likely to starve the buffer.
        lane = types.SimpleNamespace(name="lane1")
        checked = []
        shim, state = self._shim(lane, manual_off=True)
        shim._check_ams_fault = lambda ln: checked.append(ln.name)
        shim._check_buffer_starved = lambda ln, et: checked.append("buff")
        afcBambuAMS._follow_tick(shim, 100.0)
        assert state["engaged"] == []           # still does not re-arm
        assert checked == ["lane1", "buff"]     # but still watches for a stall

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


class TestFollowIdlePing:
    """follow_idle_ping holds the firmware's feed window open while a lane is
    loaded. Default True: it is the only setting measured to actually feed."""

    def _shim(self, idle_ping):
        state = {"sent": []}
        lane = types.SimpleNamespace(name="lane1")
        toolhead = types.SimpleNamespace(get_position=lambda: [0, 0, 0, 0.0])
        shim = types.SimpleNamespace(
            _following_lane=lane, _follow_last_e=0.0, follow_min_extrude=0.05,
            _follow_manual_off=False, _unload_in_progress=False,
            _follow_last_demand=99.0, follow_rearm_window=3.0,
            _bridge=types.SimpleNamespace(
                send=lambda o: state["sent"].append(o),
                latest_status=lambda: {"fstate": 4, "buff": 0}),
            follow_always=False, follow_when_loaded=True,
            follow_idle_ping=idle_ping,
            follow_debug_interval=0.0, _follow_last_log=0.0,
            follow_poll_interval=0.1, ams_index=0,
            _tool_loaded_lane=lambda: lane,
            _check_ams_fault=lambda ln: None,
            _fault_hold_active=lambda: False,
            _ready_to_follow=lambda lane=None: True,
            _engage_follower=lambda ln: None,
            set_feed_assist=lambda ln, on: None,
            afc=types.SimpleNamespace(toolhead=toolhead))
        return shim, state

    def test_default_is_off(self):
        # The default is what ships; assert it explicitly so a flip is a
        # deliberate change rather than a silent one.
        import inspect
        src = inspect.getsource(afcBambuAMS.__init__)
        assert 'config.getboolean("follow_idle_ping", False)' in src

    def test_on_pings_with_no_extrusion(self):
        shim, state = self._shim(True)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == [{"cmd": "follow"}]

    def test_off_stays_silent_with_no_extrusion(self):
        # Same shim, flag cleared -> proves the assertion above is the flag
        # working rather than the tick pinging unconditionally.
        shim, state = self._shim(False)
        afcBambuAMS._follow_tick(shim, 1.0)
        assert state["sent"] == []


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
        raised, warned, assist = [], [], []
        shim = types.SimpleNamespace(
            name="BambuAMS_2", fault_detect=detect, fault_pause=pause,
            _fault_seen=seen, _unload_in_progress=unloading, _drying=drying,
            _follow_fault_hold=False, _follow_fault_saw_pause=False,
            _starved_since=5.0,
            _bridge=types.SimpleNamespace(
                last_fault=lambda: (1, fault_text, 1.6)),
            set_feed_assist=lambda ln, on: assist.append((ln.name, on)),
            logger=types.SimpleNamespace(warning=lambda m: warned.append(m),
                                         debug=lambda m: None),
            afc=types.SimpleNamespace(error=types.SimpleNamespace(
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
            follow_min_extrude=0.05, follow_always=False,
            follow_when_loaded=True, follow_idle_ping=False,
            follow_debug_interval=0.0, _follow_last_log=0.0,
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


class TestBufferStarved:
    """The buffer-based stall detector, for units that never narrate."""

    def _shim(self, *, detect=True, pause=True):
        raised = []
        state = {"buff": 3, "buffn": 0, "e": 100.0}
        shim = types.SimpleNamespace(
            name="BambuAMS_2", fault_detect=detect, fault_pause=pause,
            fault_starved_below=25, fault_starved_seconds=2.0,
            follow_min_extrude=0.1,
            _unload_in_progress=False, _drying=False,
            _starved_since=0.0, _starved_e=0.0, _starved_reads=None,
            _follow_fault_hold=False, _follow_fault_saw_pause=False,
            _bridge=types.SimpleNamespace(
                latest_status=lambda: {"buff": state["buff"],
                                       "buffn": state["buffn"]}),
            set_feed_assist=lambda ln, on: None,
            logger=types.SimpleNamespace(warning=lambda m: raised.append(m),
                                         debug=lambda m: None),
            afc=types.SimpleNamespace(
                in_toolchange=False,
                toolhead=types.SimpleNamespace(
                    get_position=lambda: [0, 0, 0, state["e"]]),
                error=types.SimpleNamespace(
                    AFC_error=lambda m, pause=True: raised.append(m))))
        shim._raise_ams_fault = (
            lambda ln, m: afcBambuAMS._raise_ams_fault(shim, ln, m))
        return shim, state, raised

    def _run(self, shim, state, lane, seconds, *, mm_per_s=1.0, buffn_hz=2.0):
        """Drive the check at the real ~100ms tick rate for `seconds`.

        Deliberately realistic: per tick the extruder moves far less than
        follow_min_extrude and the buffer counter only ticks a couple of times a
        second. The original implementation reset its window on both of those
        and so could never fire.
        """
        ticks = int(seconds / 0.1)
        for i in range(ticks):
            t = 100.0 + i * 0.1
            state["e"] += mm_per_s * 0.1
            state["buffn"] = int(t * buffn_hz)
            afcBambuAMS._check_buffer_starved(shim, lane, t)

    def test_sustained_starvation_while_extruding_raises(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        self._run(shim, state, lane, 4.0)
        assert len(raised) == 1 and "buffer has been empty" in raised[0]
        assert shim._follow_fault_hold is True

    def test_short_starvation_does_not_raise(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        self._run(shim, state, lane, 1.5)
        assert raised == []

    def test_healthy_buffer_never_raises(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        state["buff"] = 58
        self._run(shim, state, lane, 30.0)
        assert raised == []

    def test_recovery_clears_the_window(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        self._run(shim, state, lane, 1.5)
        state["buff"] = 58                      # AMS caught up
        self._run(shim, state, lane, 2.0)
        state["buff"] = 3                       # starved again, from zero
        self._run(shim, state, lane, 1.5)
        assert raised == []

    def test_idle_printer_sitting_starved_does_not_raise(self):
        # Not a fault: nothing is asking the AMS for filament.
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        self._run(shim, state, lane, 30.0, mm_per_s=0.0)
        assert raised == []

    def test_frozen_telemetry_never_raises(self):
        # A stuck buffer counter reads exactly like an empty buffer. It must
        # not be able to manufacture a fault out of nothing.
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        self._run(shim, state, lane, 30.0, buffn_hz=0.0)
        assert raised == []

    def test_unload_does_not_raise(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        shim._unload_in_progress = True
        self._run(shim, state, lane, 30.0)
        assert raised == []

    def test_toolchange_does_not_raise(self):
        # A purge bottoms the buffer out while the extruder moves -- the fault
        # signature exactly. With a 2s window this guard is load-bearing.
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim()
        shim.afc.in_toolchange = True
        self._run(shim, state, lane, 30.0)
        assert raised == []

    def test_detect_disabled_does_nothing(self):
        lane = types.SimpleNamespace(name="lane22")
        shim, state, raised = self._shim(detect=False)
        self._run(shim, state, lane, 30.0)
        assert raised == []


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
        # This is the bug: AMS2 used to report the HT's chamber temperature.
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
        # The case that previously had to report nothing for either unit.
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

    `drying` is host state set by BAMBU_HEATER_START, so a Klipper restart
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
        afcBambuAMS.cmd_BAMBU_HEATER_STOP(u, gcmd)
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
        afcBambuAMS.cmd_BAMBU_HEATER_START(u, gcmd)
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

    # Verbatim from captures/ams1_alone_insert_timestamped.txt
    LINES = [
        "[AMS_DEV] STEP,first detected",
        "[AMS_DEV] STEP:card auth success!",
        "[RF] tray0: info write to flash",
        "[AMS_DEV] STEP:read success,valid",
        "[AMS_DEV] STEP:feed with rfid success",
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
        assert self._narrate("[AMS_RFID]STEP:read success") is not None

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
        return types.SimpleNamespace(
            name="AMS", logger=_Logger(),
            full_name=["AFC_BambuAMS", "AMS"],
            afc_bowden_length=bowden,
            afc_unload_bowden_length=bowden if unload is None else unload,
            measured_path_mm=lambda: measured,
            afc=types.SimpleNamespace(function=types.SimpleNamespace(
                ConfigRewrite=lambda sec, key, val, msg="":
                    writes.append((sec, key, val))))), writes

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
    """Motor-state alone was too tight. G28 does not necessarily energise the
    EXTRUDER -- measured on hardware: homed_axes "xyz" with the extruder
    stepper still disabled -- so gating purely on the motor left the follower
    disarmed on a machine that was plainly in use, and the AMS would not hold
    buffer pressure. Homing alone would be too loose, since it stays true
    forever after. Either-of-two leaves exactly one blocking case: unhomed AND
    de-energised, which is what a cold restart looks like."""

    def _u(self, motor, homed):
        return types.SimpleNamespace(
            _extruder_motor_enabled=lambda lane=None: motor,
            _toolhead_homed=lambda: homed)

    def test_a_live_motor_qualifies_even_unhomed(self):
        assert afcBambuAMS._ready_to_follow(self._u(True, False)) is True

    def test_a_homed_machine_qualifies_even_with_a_dead_motor(self):
        # The case that motivated widening it: homed, extruder still disabled.
        assert afcBambuAMS._ready_to_follow(self._u(False, True)) is True

    def test_both_qualifies(self):
        assert afcBambuAMS._ready_to_follow(self._u(True, True)) is True

    def test_only_a_cold_restart_blocks(self):
        assert afcBambuAMS._ready_to_follow(self._u(False, False)) is False


class TestToolheadHomed:
    def _u(self, axes, th=True, raises=False):
        def _lookup(name, default=None):
            if raises:
                raise RuntimeError("not ready")
            return types.SimpleNamespace(
                get_status=lambda t: {"homed_axes": axes}) if th else None
        return types.SimpleNamespace(
            printer=types.SimpleNamespace(lookup_object=_lookup),
            reactor=types.SimpleNamespace(monotonic=lambda: 1.0))

    def test_all_three_axes(self):
        assert afcBambuAMS._toolhead_homed(self._u("xyz")) is True

    def test_a_partial_home_does_not_count(self):
        assert afcBambuAMS._toolhead_homed(self._u("xy")) is False

    def test_nothing_homed(self):
        assert afcBambuAMS._toolhead_homed(self._u("")) is False

    def test_it_fails_open_with_no_toolhead(self):
        assert afcBambuAMS._toolhead_homed(self._u("", th=False)) is True

    def test_it_fails_open_when_the_lookup_raises(self):
        assert afcBambuAMS._toolhead_homed(self._u("", raises=True)) is True




class TestTheGateDoesNotBlockDockedTools:
    """Async loading into a DOCKED tool is planned, and a lane being loaded
    while its tool is parked needs its follower exactly as much as one on the
    shuttle. An earlier version of this gate also required the lane to be on
    the active tool, which would have blocked that outright. Whether a tool is
    docked is not evidence about whether filament is moving."""

    def test_a_homed_machine_arms_regardless_of_which_tool_is_active(self):
        # No active-tool input exists any more: the decision is made from
        # motor state and homing alone.
        u = types.SimpleNamespace(
            _extruder_motor_enabled=lambda lane=None: False,
            _toolhead_homed=lambda: True)
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

    def test_a_spoolman_linked_lane_is_left_alone(self):
        # spool_id set means Spoolman is authoritative; the tag must not
        # overwrite what the operator linked.
        lane = self._lane(spool_id=42, material="PETG", sub_type="")
        self._apply(lane, {"material": "PLA Matte"})
        assert lane.material == "PETG" and lane.spool_vendor is None


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
        # This test used to assert the opposite, and that was the regression.
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
            _send_ht_flag=lambda b: (_ for _ in ()).throw(RuntimeError("boom"))
            if fail == "ht" else sent.append({"cmd": "htunit"}),
            _send_selfcentre_flag=lambda b: sent.append({"cmd": "selfc"}),
            _send_mc_addr=lambda b: sent.append({"cmd": "mcaddr"}))
        return shim, sent

    def test_all_four_are_sent_normally(self):
        shim, sent = self._shim()
        afcBambuAMS._announce_unit(shim)
        assert [o["cmd"] for o in sent] == ["units", "htunit", "selfc", "mcaddr"]

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


class TestAmsArrivalCompletesLoad:
    """The unit knows it got there without our sensor -- an HT feeds to the end
    of its measured PTFE and says so, a boxed AMS zeroes the tray odometer. The
    toolhead sensor stays FIRST by AFC design; this is what makes a lane with
    no sensor loadable at all, and what turns a failed sensor into a completion
    instead of a silent timeout."""

    def test_the_sensor_still_wins_when_both_would_fire(self):
        # Measured on both units once calibrated: the sensor triggers 1-2 s
        # ahead, so on a sensored lane this path must never be the one used.
        shim, calls, _ = _load_shim(sensor_after=1, arrivals=[(9, True)] * 20)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True
        assert not any("own arrival" in m for _l, m in shim.logger.messages)

    def test_arrival_completes_when_the_sensor_never_triggers(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9,
                                    arrivals=[(0, False), (9, True)])
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0) is True
        assert any("own arrival" in m for _l, m in shim.logger.messages)
        assert calls["stop"] == 1               # never left it feeding

    def test_a_stalled_short_arrival_does_NOT_complete(self):
        # ok=False is the bridge's distance judgement: it stopped, but short.
        # Accepting that would report a load that is stuck mid-bowden.
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.3,
                                    arrivals=[(9, False)] * 20)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 0.3) is False

    def test_a_stale_completion_is_not_this_move(self):
        # Sequence unchanged from the one captured before the feed: that is
        # the PREVIOUS move's completion still sitting there.
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.3,
                                    arrivals=[(0, True)] * 20)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 0.3) is False

    def test_it_can_be_turned_off(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9, timeout=0.3,
                                    ams_arrival=False,
                                    arrivals=[(9, True)] * 20)
        assert afcBambuAMS._feed_until_sensor(shim, _LANE, 0.3) is False

    def test_the_kick_count_is_reported_with_the_arrival(self):
        shim, calls, _ = _load_shim(sensor_after=10 ** 9,
                                    arrivals=[(0, False)] * 6 + [(9, True)])
        afcBambuAMS._feed_until_sensor(shim, _LANE, 5.0)
        assert any("feed kick(s)" in m for _l, m in shim.logger.messages)


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
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
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
        afcBambuAMS.cmd_BAMBU_HEATER_START(shim, gcmd)
        assert sent[0]["rotate"] == 0
        assert any("ROTATE disabled" in m for m in gcmd.info)
