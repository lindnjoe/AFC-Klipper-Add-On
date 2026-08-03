"""
Broad branch-coverage tests for extras/AFC_ACE.py (afcACE V1 unit) covering the
large previously-uncovered method blocks: module logo/CRC helpers, RFID slot
cache (_store_slot_rfid), the diagnostic gcode commands (DRY/FAN/FEED_INFO/
RFID_DUMP/CMD/LANE_RESET/CALIBRATE), stuck-spool detection, feed/unwind waiters,
feed-assist start/stop retry paths, load/unload transports and the calibration
routines.

Style: typed fakes (tests/ace_helpers.py), __new__ construction like the sibling
ACE tests, one test class per method under test, exact log-list assertions.
"""

from __future__ import annotations

import types

import pytest

import extras.AFC_ACE as ace_mod
from extras.AFC_ACE import (
    MODE_COMBINED,
    MODE_DIRECT,
    ACETimeoutError,
    _ams_box_logo,
    _ams_box_logo_error,
    afcACE,
    crc16_ccitt_reflected,
)
from extras.AFC_lane import AFCLaneState

from tests.ace_helpers import (
    FakeAFC,
    FakeAce,
    FakeExtruderObj,
    FakeGcmd,
    FakeHub,
    FakeLane,
    FakeLogger,
    FakeToolheadPrinter,
    Recorder,
)


# ── Shared fakes / builders ───────────────────────────────────────────────────

class Gcmd(FakeGcmd):
    """FakeGcmd plus get_float (the dryer / feed-test commands need it)."""

    def get_float(self, name, default=None, minval=None, maxval=None):
        val = self._params.get(name, default)
        return float(val) if val is not None else None


def _ace(connected=True, status=None):
    """A V1 FakeAce with the extra transport methods the unit calls."""
    ace = FakeAce(connected=connected)
    ready = {"status": "ready", "slots": [{"status": "ready"}] * 4}
    ace.get_status = Recorder(result=status if status is not None else ready)
    ace.feed_filament = Recorder()
    ace.unwind_filament = Recorder()
    ace.stop_feed_assist_sync = Recorder()
    ace.connect = Recorder()
    ace.status_callback = None
    ace.reconnect_callback = None
    ace.start_drying = Recorder()
    ace.stop_drying = Recorder()
    ace.send_command = Recorder(result={})
    ace.get_filament_info = Recorder(result={})
    ace.enable_rfid = Recorder()
    return ace


def make_unit(**kw):
    unit = afcACE.__new__(afcACE)
    unit.name = "ACE_1"
    unit.logger = FakeLogger()
    unit.logger.raw = Recorder()  # afcUnit logger has .raw; FakeLogger doesn't
    unit.afc = FakeAFC()
    unit.gcode = types.SimpleNamespace(run_script_from_command=Recorder())
    unit.printer = types.SimpleNamespace(
        send_event=Recorder(),
        register_event_handler=Recorder(),
        lookup_object=lambda *a, **k: None,
        start_args={})
    unit.lanes = {}
    unit._ace = None
    unit._slot_map = {}
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    unit._feed_assist_active = set()
    unit._assist_suppressed = set()
    unit._assist_watchdog = True
    unit._operation_active = False
    unit._prev_states_stale = False
    unit._prev_slot_states = {}
    unit._hub_load_suppressed = set()
    unit._stuck_detection = False
    unit._stuck_time = 4.0
    unit._stuck_tripped = False
    unit._current_action = ""
    unit._cached_hw_status = {}
    unit._cached_temp_info = {}
    unit._uses_firmware_rfid = True
    unit._preloads_to_hub_on_insert = True
    unit._unit_load_to_hub = None
    unit._default_feed_assist = True
    unit.feed_speed = 100.0
    unit.retract_speed = 100.0
    unit.max_feed_speed = 100.0
    unit.calibration_step = 50.0
    unit.eject_buffer = 475.0
    unit.max_dryer_temperature = 55.0
    unit.feed_departure_timeout = 3.0
    unit.load_retry_pulse = 100.0
    unit.load_retry_timeout = 10.0
    unit.prep_ready_timeout = 90.0
    unit.serial_port = "/dev/ttyACM0"
    unit.mode = MODE_COMBINED
    unit.auto_spoolman_create = False
    unit.baud_rate = 115200
    for k, v in kw.items():
        setattr(unit, k, v)
    return unit


# ── _ams_box_logo ─────────────────────────────────────────────────────────────

class TestAmsBoxLogo:
    def test_short_title_uses_min_bay_width(self):
        logo = _ams_box_logo("ACE", 4, "myace")
        # 4 bays * 3 wide + 3 separators = 15-char inner border.
        assert "+" + "-" * 15 + "+" in logo
        assert logo.endswith("   myace\n")
        assert "success--text" in logo

    def test_long_title_grows_bay_width(self):
        logo = _ams_box_logo("SUPERLONGTITLE", 1, "u")
        # 1 bay must widen until it fits the 14-char title: bay_w=14, inner=14.
        assert "+" + "-" * 14 + "+" in logo
        assert "SUPERLONGTITLE" in logo

    def test_zero_slots_defaults_to_one(self):
        logo = _ams_box_logo("X", 0, "u")
        assert "+" + "-" * 3 + "+" in logo


class TestAmsBoxLogoError:
    def test_error_banner_and_min_width(self):
        logo = _ams_box_logo_error("ACE", 4, "myace")
        assert "error--text" in logo
        assert "X ERROR" in logo
        assert "+" + "-" * 15 + "+" in logo

    def test_error_width_floors_at_error_banner(self):
        # n=1, short title: inner floors at len("X ERROR") == 7.
        logo = _ams_box_logo_error("A", 1, "u")
        assert "+" + "-" * 7 + "+" in logo


class TestCrc16CcittReflected:
    def test_empty_is_init_value(self):
        assert crc16_ccitt_reflected(b"") == 0xFFFF

    def test_known_check_vector(self):
        # CRC-16/MCRF4XX check value for "123456789" is 0x6F91 (independent).
        assert crc16_ccitt_reflected(b"123456789") == 0x6F91


# ── _get_bowden_length ────────────────────────────────────────────────────────

class TestGetBowdenLength:
    def test_adds_hub_bowden_length(self):
        unit = make_unit()
        lane = FakeLane("l0")
        lane.dist_hub = 100.0
        lane.hub_obj = types.SimpleNamespace(afc_bowden_length=900.0)
        assert unit._get_bowden_length(lane) == 1000.0

    def test_without_hub_is_dist_hub_only(self):
        unit = make_unit()
        lane = FakeLane("l0", hub_obj=None)
        lane.dist_hub = 120.0
        assert unit._get_bowden_length(lane) == 120.0


# ── on_filament_remove ────────────────────────────────────────────────────────

class TestOnFilamentRemove:
    def test_clears_inventory_and_hub_state(self):
        unit = make_unit(_slot_map={"l0": 1})
        unit._slot_inventory[1]["material"] = "PLA"
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.loaded_to_hub = True
        lane.tool_loaded = False
        unit._hub_load_suppressed = {"l0"}

        unit.on_filament_remove(lane)

        assert unit._slot_inventory[1]["material"] == ""
        assert unit._slot_inventory[1]["color"] == [0, 0, 0]
        assert lane.loaded_to_hub is False
        assert lane._load_state is False
        assert "l0" not in unit._hub_load_suppressed


# ── _store_slot_rfid ──────────────────────────────────────────────────────────

class TestStoreSlotRfid:
    def test_full_payload_derives_temps_and_logs_read(self):
        unit = make_unit()
        info = {
            "sku": "HPL-1", "brand": "AC", "type": "PLA", "rfid": 2,
            "color": [1, 2, 3], "diameter": 1.75,
            "extruder_temp": {"min": 200, "max": 220},
            "hotbed_temp": {"min": 50, "max": 60},
        }
        unit._store_slot_rfid(0, info)

        inv = unit._slot_inventory[0]
        assert inv["material"] == "PLA"
        assert inv["extruder_temp"] == (200 + 220) // 2
        assert inv["bed_temp"] == (50 + 60) // 2
        # changed + recognized -> both a debug echo and an info read line.
        assert unit.logger.lines["info"] == [
            f"ACE ACE_1: slot 0 RFID read — sku='HPL-1' brand='AC' "
            f"type='PLA' rfid=2 nozzle=200-220C"]

    def test_max_only_temps_and_no_change_no_log(self):
        unit = make_unit()
        info = {"type": "PETG", "extruder_temp": {"max": 240},
                "hotbed_temp": {"max": 70}, "rfid": 0}
        unit._slot_inventory[0]["raw"] = info  # same object -> unchanged

        unit._store_slot_rfid(0, info)

        inv = unit._slot_inventory[0]
        assert inv["extruder_temp"] == 240
        assert inv["bed_temp"] == 70
        assert unit.logger.lines["info"] == []
        assert unit.logger.lines["debug"] == []

    def test_non_dict_temps_yield_none(self):
        unit = make_unit()
        unit._store_slot_rfid(0, {"extruder_temp": "bad", "hotbed_temp": 5})

        inv = unit._slot_inventory[0]
        assert inv["extruder_temp"] is None
        assert inv["extruder_temp_min"] is None
        assert inv["bed_temp"] is None
        assert inv["bed_temp_max"] is None

    def test_min_without_max_temp_is_none(self):
        unit = make_unit()
        unit._store_slot_rfid(0, {"extruder_temp": {"min": 200},
                                  "hotbed_temp": {"min": 50}})
        assert unit._slot_inventory[0]["extruder_temp"] is None
        assert unit._slot_inventory[0]["bed_temp"] is None


# ── _refresh_slot_inventory ───────────────────────────────────────────────────

class TestRefreshSlotInventory:
    def test_disconnected_noop(self):
        unit = make_unit(_ace=None)
        unit._refresh_slot_inventory(0)
        assert unit._slot_inventory[0] == {}

    def test_out_of_range_noop(self):
        unit = make_unit(_ace=_ace())
        unit._refresh_slot_inventory(9)
        assert unit._slot_inventory[0] == {}

    def test_success_stores(self):
        ace = _ace()
        ace.get_filament_info = Recorder(result={"sku": "S1"})
        unit = make_unit(_ace=ace)
        unit._refresh_slot_inventory(2)
        assert unit._slot_inventory[2]["sku"] == "S1"

    def test_exception_logged_debug(self):
        ace = _ace()
        ace.get_filament_info = Recorder(raises=RuntimeError("boom"))
        unit = make_unit(_ace=ace)
        unit._refresh_slot_inventory(0)
        assert unit.logger.lines["debug"] == [
            "ACE ACE_1: slot 0 RFID refresh failed: boom"]


# ── _clear_slot_inventory ─────────────────────────────────────────────────────

class TestClearSlotInventory:
    def test_clears_fields_and_drops_uid(self):
        unit = make_unit()
        unit._slot_inventory[1].update(
            {"material": "PLA", "color": [1, 2, 3], "uid": "AA"})
        unit._clear_slot_inventory(1)
        assert unit._slot_inventory[1]["material"] == ""
        assert unit._slot_inventory[1]["color"] == [0, 0, 0]
        assert "uid" not in unit._slot_inventory[1]

    def test_out_of_range_noop(self):
        unit = make_unit()
        unit._clear_slot_inventory(99)  # must not raise
        assert unit._slot_inventory[0] == {}


# ── _sync_slot_loaded_state ───────────────────────────────────────────────────

class TestSyncSlotLoadedState:
    def test_disconnected_noop(self):
        unit = make_unit(_ace=None, lanes={"l0": FakeLane("l0")})
        unit._sync_slot_loaded_state()
        assert unit.lanes["l0"].prep_state is False

    def test_ready_slot_marks_prep_state(self):
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        unit = make_unit(_ace=_ace(), lanes={"l0": lane},
                         _slot_map={"l0": 0})
        unit._slot_inventory[0]["status"] = "ready"
        unit._sync_slot_loaded_state()
        assert lane.prep_state is True

    def test_empty_slot_clears_hub_state(self):
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.loaded_to_hub = True
        unit = make_unit(_ace=_ace(), lanes={"l0": lane},
                         _slot_map={"l0": 0})
        unit._slot_inventory[0]["status"] = "empty"
        unit._sync_slot_loaded_state()
        assert lane.prep_state is False
        assert lane.loaded_to_hub is False


# ── _get_auto_spoolman_create ─────────────────────────────────────────────────

class TestGetAutoSpoolmanCreate:
    def test_fallback_when_helper_absent(self, monkeypatch):
        monkeypatch.setattr(ace_mod, "get_auto_spoolman_create", None)
        unit = make_unit(auto_spoolman_create=True)
        assert unit._get_auto_spoolman_create(FakeLane("l0")) is True

    def test_delegates_to_helper_when_present(self, monkeypatch):
        seen = {}

        def helper(lane, unit_default):
            seen["args"] = (lane.name, unit_default)
            return "delegated"
        monkeypatch.setattr(ace_mod, "get_auto_spoolman_create", helper)
        unit = make_unit(auto_spoolman_create=False)
        assert unit._get_auto_spoolman_create(FakeLane("l0")) == "delegated"
        assert seen["args"] == ("l0", False)


# ── get_lane_reset_command ────────────────────────────────────────────────────

class TestGetLaneResetCommand:
    def test_builds_command(self):
        unit = make_unit()
        cmd = unit.get_lane_reset_command(FakeLane("l0"), 0.0)
        assert cmd == "ACE_LANE_RESET UNIT=ACE_1 LANE=l0"


# ── prep_capture_td1 ──────────────────────────────────────────────────────────

class TestPrepCaptureTd1:
    def test_returns_handled_when_when_loaded_and_staged(self):
        unit = make_unit()
        lane = FakeLane("l0")
        lane.td1_when_loaded = True
        lane.loaded_to_hub = True
        assert unit.prep_capture_td1(lane) == (
            True, "TD-1 capture handled by prep_post_load")

    def test_returns_none_when_not_staged(self):
        unit = make_unit()
        lane = FakeLane("l0")
        lane.td1_when_loaded = True
        lane.loaded_to_hub = False
        assert unit.prep_capture_td1(lane) is None

    def test_returns_none_when_not_when_loaded(self):
        unit = make_unit()
        lane = FakeLane("l0")
        lane.td1_when_loaded = False
        lane.loaded_to_hub = True
        assert unit.prep_capture_td1(lane) is None


# ── cmd_ACE_DRY ───────────────────────────────────────────────────────────────

class TestCmdAceDry:
    def test_temp_capped_to_max(self):
        ace = _ace()
        unit = make_unit(_ace=ace, max_dryer_temperature=55.0)
        gcmd = Gcmd(TEMP=80.0, DURATION=90.0, FAN=7000)
        unit.cmd_ACE_DRY(gcmd)
        assert ace.start_drying.last_args == (55.0, 7000, 90.0)
        assert gcmd.responses == [
            "ACE dryer: TEMP 80°C capped to max_dryer_temperature 55°C",
            "ACE dryer started: 55.0°C for 90.0 min"]

    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd(TEMP=40.0)
        unit.cmd_ACE_DRY(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_success_uncapped(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(TEMP=40.0, DURATION=30.0, FAN=7000)
        unit.cmd_ACE_DRY(gcmd)
        assert ace.start_drying.last_args == (40.0, 7000, 30.0)
        assert gcmd.responses == ["ACE dryer started: 40.0°C for 30.0 min"]

    def test_error_surfaced(self):
        ace = _ace()
        ace.start_drying = Recorder(raises=RuntimeError("nope"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(TEMP=40.0)
        unit.cmd_ACE_DRY(gcmd)
        assert gcmd.responses == ["Error starting dryer: nope"]


class TestCmdAceDryStop:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd()
        unit.cmd_ACE_DRY_STOP(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_success(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        gcmd = Gcmd()
        unit.cmd_ACE_DRY_STOP(gcmd)
        assert ace.stop_drying.call_count == 1
        assert gcmd.responses == ["ACE dryer stopped"]

    def test_error(self):
        ace = _ace()
        ace.stop_drying = Recorder(raises=RuntimeError("x"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd()
        unit.cmd_ACE_DRY_STOP(gcmd)
        assert gcmd.responses == ["Error stopping dryer: x"]


# ── cmd_ACE_FAN ───────────────────────────────────────────────────────────────

class TestCmdAceFan:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd(SPEED=50)
        unit.cmd_ACE_FAN(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_success_sends_both_keys(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(SPEED=40)
        unit.cmd_ACE_FAN(gcmd)
        assert ace.send_command.last_args == ("set_fan_speed",)
        assert ace.send_command.last_kwargs == {
            "params": {"speed": 40, "fan_speed": 40}}
        assert gcmd.responses == ["ACE fan set to 40%"]

    def test_error(self):
        ace = _ace()
        ace.send_command = Recorder(raises=RuntimeError("f"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(SPEED=40)
        unit.cmd_ACE_FAN(gcmd)
        assert gcmd.responses == ["Error setting fan: f"]


# ── cmd_ACE_LANE_RESET ────────────────────────────────────────────────────────

class TestCmdAceLaneReset:
    def test_usage_on_unknown_lane(self):
        unit = make_unit()
        unit.afc.lanes = {}
        gcmd = Gcmd(LANE="ghost")
        unit.cmd_ACE_LANE_RESET(gcmd)
        assert gcmd.responses == ["Usage: ACE_LANE_RESET LANE=<lane_name>"]

    def test_ejects_known_lane(self):
        unit = make_unit()
        lane = FakeLane("l0")
        unit.afc.lanes = {"l0": lane}
        unit.eject_lane = Recorder()
        gcmd = Gcmd(LANE="l0")
        unit.cmd_ACE_LANE_RESET(gcmd)
        assert unit.eject_lane.calls == [((lane,), {})]
        assert gcmd.responses == ["Lane l0 reset"]


# ── cmd_ACE_FEED_INFO ─────────────────────────────────────────────────────────

class TestCmdAceFeedInfo:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd()
        unit.cmd_ACE_FEED_INFO(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_query_error(self):
        ace = _ace()
        ace.send_command = Recorder(raises=RuntimeError("bad"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd()
        unit.cmd_ACE_FEED_INFO(gcmd)
        assert gcmd.responses == ["Error querying ACE feed info: bad"]

    def test_no_data(self):
        ace = _ace()
        ace.send_command = Recorder(result={"raw_fields": {"a": 1}})
        unit = make_unit(_ace=ace)
        gcmd = Gcmd()
        unit.cmd_ACE_FEED_INFO(gcmd)
        assert gcmd.responses == [
            "ACE feed info: no data (unsupported on this firmware, or "
            "nothing fed yet). raw_fields={'a': 1}"]

    def test_reports_ratio(self):
        ace = _ace()
        ace.send_command = Recorder(result={
            "feed_info": [{"steps": 10, "length": 100, "decoder": 123}]})
        unit = make_unit(_ace=ace)
        gcmd = Gcmd()
        unit.cmd_ACE_FEED_INFO(gcmd)
        assert gcmd.responses == [
            "ACE feed info (slot: steps / length_mm / encoder_mm / ratio):\n"
            "  slot 0: 10 / 100 / 123 / 1.230"]


# ── cmd_ACE_RFID_DUMP ─────────────────────────────────────────────────────────

class TestCmdAceRfidDump:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd()
        unit.cmd_ACE_RFID_DUMP(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_slot_from_lane_and_raw_present(self):
        ace = _ace()
        ace.send_command = Recorder(result={"sku": "S1", "raw": {"1": "x"}})
        unit = make_unit(_ace=ace, _slot_map={"l0": 2})
        unit.afc.lanes = {"l0": FakeLane("l0")}
        gcmd = Gcmd(LANE="l0")
        unit.cmd_ACE_RFID_DUMP(gcmd)
        assert ace.send_command.last_args == ("get_filament_info", {"index": 2})
        assert gcmd.responses == [
            "ACE RFID slot 2:\n  parsed: {'sku': 'S1'}\n"
            "  raw protobuf fields (field#: value): {'1': 'x'}"]

    def test_default_slot_zero_no_raw(self):
        ace = _ace()
        ace.send_command = Recorder(result={"sku": ""})
        unit = make_unit(_ace=ace)
        unit.afc.lanes = {}
        gcmd = Gcmd()
        unit.cmd_ACE_RFID_DUMP(gcmd)
        assert ace.send_command.last_args == ("get_filament_info", {"index": 0})
        assert gcmd.responses == [
            "ACE RFID slot 0:\n  parsed: {'sku': ''}\n"
            "  (no raw field map — V1 ACE Pro or empty read)"]

    def test_error(self):
        ace = _ace()
        ace.send_command = Recorder(raises=RuntimeError("z"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(SLOT=1)
        unit.cmd_ACE_RFID_DUMP(gcmd)
        assert gcmd.responses == ["Error reading ACE RFID slot 1: z"]


# ── cmd_ACE_CMD ───────────────────────────────────────────────────────────────

class TestCmdAceCmd:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd(METHOD="get_status")
        unit.cmd_ACE_CMD(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_requires_method(self):
        unit = make_unit(_ace=_ace())
        gcmd = Gcmd(METHOD="")
        unit.cmd_ACE_CMD(gcmd)
        assert gcmd.responses == ["ACE_CMD: METHOD=<method> required"]

    def test_success_with_params(self):
        ace = _ace()
        ace.send_command = Recorder(result={"code": 0})
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(METHOD="set_fan_speed", PARAMS="{fan_speed:7000}")
        unit.cmd_ACE_CMD(gcmd)
        assert ace.send_command.last_args == ("set_fan_speed",)
        assert ace.send_command.last_kwargs == {"params": {"fan_speed": 7000}}
        assert gcmd.responses == ["ACE_CMD set_fan_speed: OK -> {'code': 0}"]

    def test_command_error(self):
        ace = _ace()
        ace.send_command = Recorder(raises=RuntimeError("400"))
        unit = make_unit(_ace=ace)
        gcmd = Gcmd(METHOD="bad", PARAMS="")
        unit.cmd_ACE_CMD(gcmd)
        assert gcmd.responses == ["ACE_CMD bad: 400"]


# ── cmd_ACE_STUCK_SPOOL_DETECTION ─────────────────────────────────────────────

class TestCmdAceStuckSpoolDetection:
    def test_enable_on(self):
        unit = make_unit(_stuck_detection=False)
        gcmd = Gcmd(ENABLE=1)
        unit.cmd_ACE_STUCK_SPOOL_DETECTION(gcmd)
        assert unit._stuck_detection is True
        assert unit.logger.lines["info"] == [
            "ACE stuck spool detection ON: stuck_time=4.0s"]

    def test_disable_clears_latch(self):
        unit = make_unit(_stuck_detection=True, _stuck_tripped=True)
        gcmd = Gcmd(ENABLE=0)
        unit.cmd_ACE_STUCK_SPOOL_DETECTION(gcmd)
        assert unit._stuck_detection is False
        assert unit._stuck_tripped is False
        assert unit.logger.lines["info"] == [
            "ACE stuck spool detection OFF: stuck_time=4.0s"]

    def test_sets_stuck_time_only(self):
        unit = make_unit(_stuck_detection=True)
        gcmd = Gcmd(STUCK_TIME=6.0)
        unit.cmd_ACE_STUCK_SPOOL_DETECTION(gcmd)
        assert unit._stuck_time == 6.0
        assert unit.logger.lines["info"] == [
            "ACE stuck spool detection ON: stuck_time=6.0s"]


# ── _check_stuck ──────────────────────────────────────────────────────────────

class TestCheckStuck:
    def _unit(self, **kw):
        params = {"_stuck_detection": True, "_stuck_time": 4.0,
                  "_feed_assist_active": {0}}
        params.update(kw)
        unit = make_unit(**params)
        unit.afc.function = types.SimpleNamespace(
            in_print=lambda: True, is_paused=lambda: False)
        return unit

    def test_disabled_returns_early(self):
        unit = self._unit(_stuck_detection=False)
        unit._check_stuck({"cont_assist_time": 99})
        assert unit._stuck_tripped is False
        assert not unit.afc.reactor.register_callback.called

    def test_not_printing_clears_latch(self):
        unit = self._unit(_stuck_tripped=True)
        unit.afc.function.in_print = lambda: False
        unit._check_stuck({"cont_assist_time": 99})
        assert unit._stuck_tripped is False
        assert not unit.afc.reactor.register_callback.called

    def test_no_cont_field_returns(self):
        unit = self._unit()
        unit._check_stuck({})
        assert unit._stuck_tripped is False

    def test_non_numeric_cont_returns(self):
        unit = self._unit()
        unit._check_stuck({"cont_assist_time": "bad"})
        assert unit._stuck_tripped is False

    def test_below_threshold_clears_latch(self):
        unit = self._unit(_stuck_tripped=True)
        unit._check_stuck({"cont_assist_time": 1.0})
        assert unit._stuck_tripped is False

    def test_trips_once_and_defers(self):
        unit = self._unit()
        unit._check_stuck({"cont_assist_time": 5.0})
        assert unit._stuck_tripped is True
        assert unit.afc.reactor.register_callback.call_count == 1

    def test_already_tripped_does_not_re_defer(self):
        unit = self._unit(_stuck_tripped=True)
        unit._check_stuck({"cont_assist_time": 5.0})
        assert not unit.afc.reactor.register_callback.called


# ── _handle_stuck ─────────────────────────────────────────────────────────────

class TestHandleStuck:
    def test_stops_assist_and_pauses_via_afc(self):
        lane = FakeLane("l0", extruder_obj=FakeExtruderObj(lane_loaded="l0"),
                        tool_loaded=True)
        unit = make_unit(lanes={"l0": lane}, _slot_map={"l0": 0})
        unit.afc.lanes = {"l0": lane}
        unit.printer = FakeToolheadPrinter(active_extruder="extruder")
        unit._stop_feed_assist = Recorder()

        unit._handle_stuck(7.5)

        assert unit._stop_feed_assist.calls == [((0,), {})]
        assert unit.afc.error.AFC_error.call_count == 1
        msg = unit.afc.error.AFC_error.last_args[0]
        assert "lane l0" in msg
        assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}

    def test_fallback_to_gcode_pause_when_afc_raises(self):
        unit = make_unit()
        unit.afc.lanes = {}
        unit.printer = FakeToolheadPrinter(active_extruder=None)  # no lane
        unit.afc.error.AFC_error = Recorder(raises=RuntimeError("nope"))

        unit._handle_stuck(9.0)

        assert unit.gcode.run_script_from_command.calls == [(("PAUSE",), {})]
        assert len(unit.logger.lines["error"]) == 1


# ── _handle_extruder_activated ────────────────────────────────────────────────

class TestHandleExtruderActivated:
    def test_no_active_lane_noop(self):
        unit = make_unit()
        unit.printer = FakeToolheadPrinter(active_extruder=None)
        unit._handle_extruder_activated()
        assert not unit.afc.reactor.register_callback.called

    def test_schedules_reconcile_for_active_lane(self):
        lane = FakeLane("l0", extruder_obj=FakeExtruderObj(lane_loaded="l0"),
                        tool_loaded=True)
        unit = make_unit(lanes={"l0": lane})
        unit.afc.lanes = {"l0": lane}
        unit.printer = FakeToolheadPrinter(active_extruder="extruder")
        unit._use_feed_assist = Recorder(result=True)
        unit._handle_extruder_activated()
        assert unit.afc.reactor.register_callback.call_count == 1

    def test_no_schedule_when_assist_disabled(self):
        lane = FakeLane("l0", extruder_obj=FakeExtruderObj(lane_loaded="l0"),
                        tool_loaded=True)
        unit = make_unit(lanes={"l0": lane})
        unit.afc.lanes = {"l0": lane}
        unit.printer = FakeToolheadPrinter(active_extruder="extruder")
        unit._use_feed_assist = Recorder(result=False)
        unit._handle_extruder_activated()
        assert not unit.afc.reactor.register_callback.called


# ── _on_ace_reconnect ─────────────────────────────────────────────────────────

class TestOnAceReconnect:
    def test_logs_and_clears_when_assist_active(self):
        unit = make_unit(_feed_assist_active={1, 2})
        unit._on_ace_reconnect()
        assert unit._feed_assist_active == set()
        assert unit._prev_states_stale is True
        assert unit.logger.lines["info"] == [
            "ACE reconnected — re-establishing feed assist for the active lane"]
        assert unit.afc.reactor.register_callback.call_count == 1

    def test_no_log_when_no_assist(self):
        unit = make_unit(_feed_assist_active=set())
        unit._on_ace_reconnect()
        assert unit.logger.lines["info"] == []
        assert unit._prev_states_stale is True
        assert unit.afc.reactor.register_callback.call_count == 1


# ── _resync_assist_after_reconnect ────────────────────────────────────────────

class TestResyncAssistAfterReconnect:
    def test_operation_active_only_reapplies_feed_check(self):
        unit = make_unit(_operation_active=True, _feed_assist_active={0})
        unit._apply_feed_check = Recorder()
        unit._maybe_assist_watchdog = Recorder()
        unit._resync_assist_after_reconnect(0.0)
        assert not unit._maybe_assist_watchdog.called
        assert unit._feed_assist_active == {0}  # untouched
        assert unit._apply_feed_check.call_count == 1

    def test_clears_all_slots_then_reconciles(self):
        ace = _ace()
        unit = make_unit(_operation_active=False, _ace=ace,
                         _feed_assist_active={0, 1})
        unit._apply_feed_check = Recorder()
        unit._maybe_assist_watchdog = Recorder()
        unit._resync_assist_after_reconnect(0.0)
        assert ace.stop_feed_assist_sync.call_count == afcACE.SLOTS_PER_UNIT
        assert unit._feed_assist_active == set()
        assert unit._maybe_assist_watchdog.call_count == 1
        assert unit._apply_feed_check.call_count == 1


# ── _handle_tool_loaded ───────────────────────────────────────────────────────

class TestHandleToolLoaded:
    def test_combined_ignores_unknown_lane(self):
        unit = make_unit(mode=MODE_COMBINED, _slot_map={"l0": 0})
        unit._handle_tool_loaded(types.SimpleNamespace(name="other"))
        assert not unit.afc.reactor.register_callback.called

    def test_combined_schedules_for_our_lane(self):
        unit = make_unit(mode=MODE_COMBINED, _slot_map={"l0": 0})
        unit._handle_tool_loaded(types.SimpleNamespace(name="l0"))
        assert unit.afc.reactor.register_callback.call_count == 1

    def test_direct_falls_back_to_extruder_lane_loaded(self):
        unit = make_unit(mode=MODE_DIRECT, _slot_map={"l0": 0})
        payload = types.SimpleNamespace(name="extruder", lane_loaded="l0")
        unit._handle_tool_loaded(payload)
        assert unit.afc.reactor.register_callback.call_count == 1

    def test_direct_falls_back_to_afc_current(self):
        unit = make_unit(mode=MODE_DIRECT, _slot_map={"l0": 0})
        unit.afc.current = "l0"
        payload = types.SimpleNamespace(name="extruder", lane_loaded=None)
        unit._handle_tool_loaded(payload)
        assert unit.afc.reactor.register_callback.call_count == 1


# ── _pick_test_slot ───────────────────────────────────────────────────────────

class TestPickTestSlot:
    def test_returns_first_recognized_slot(self):
        unit = make_unit()
        unit._slot_inventory[2]["rfid"] = 2
        assert unit._pick_test_slot() == 2

    def test_returns_slot_with_sku(self):
        unit = make_unit()
        unit._slot_inventory[1]["sku"] = "S1"
        assert unit._pick_test_slot() == 1

    def test_none_when_empty(self):
        unit = make_unit()
        assert unit._pick_test_slot() is None


# ── _poll_until_status ────────────────────────────────────────────────────────

class TestPollUntilStatus:
    def test_matches_ready(self):
        ace = _ace(status={"status": "ready"})
        unit = make_unit(_ace=ace)
        assert unit._poll_until_status(True, timeout=1.0) is True

    def test_times_out_when_never_matches(self):
        ace = _ace(status={"status": "busy"})
        unit = make_unit(_ace=ace)
        assert unit._poll_until_status(True, timeout=0.4) is False


# ── _slot_is_moving / _slot_in_error ──────────────────────────────────────────

class TestSlotIsMoving:
    def test_non_dict_false(self):
        assert make_unit()._slot_is_moving("x", 0) is False

    def test_unit_busy_true(self):
        assert make_unit()._slot_is_moving({"status": "busy"}, 0) is True

    def test_slot_status_field_moving(self):
        unit = make_unit()
        hw = {"status": "ready", "slots": [{"status": "feeding"}]}
        assert unit._slot_is_moving(hw, 0) is True

    def test_slot_status_key_moving(self):
        unit = make_unit()
        hw = {"status": "ready", "slots": [{"status": "ready",
                                            "slot_status": "rollback"}]}
        assert unit._slot_is_moving(hw, 0) is True

    def test_idle_slot_false(self):
        unit = make_unit()
        hw = {"status": "ready", "slots": [{"status": "ready"}]}
        assert unit._slot_is_moving(hw, 0) is False


class TestSlotInError:
    def test_non_dict_false(self):
        assert make_unit()._slot_in_error(None, 0) is False

    def test_status_error_true(self):
        unit = make_unit()
        hw = {"slots": [{"status": "feed_error"}]}
        assert unit._slot_in_error(hw, 0) is True

    def test_no_error_false(self):
        unit = make_unit()
        hw = {"slots": [{"status": "ready", "slot_status": "ready"}]}
        assert unit._slot_in_error(hw, 0) is False


# ── _wait_for_ace_ready ───────────────────────────────────────────────────────

class TestWaitForAceReady:
    def test_disconnected_false(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        assert unit._wait_for_ace_ready() is False

    def test_ready_immediately_true(self):
        unit = make_unit(_ace=_ace(status={"status": "ready"}))
        assert unit._wait_for_ace_ready() is True

    def test_timeout_warns_and_returns_false(self):
        unit = make_unit(_ace=_ace(status={"status": "busy"}))
        assert unit._wait_for_ace_ready(timeout=1.0) is False
        assert unit.logger.lines["warning"] == [
            "ACE: did not become ready within 1s, proceeding anyway"]


# ── _wait_for_feed_complete ───────────────────────────────────────────────────

class TestWaitForFeedComplete:
    def test_disconnected_false(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        assert unit._wait_for_feed_complete(0, 100.0, 100.0) is False

    def test_lane_sensor_early_return(self):
        unit = make_unit(_ace=_ace())
        lane = FakeLane("l0")
        unit._toolhead_sensor_triggered = lambda l: True
        assert unit._wait_for_feed_complete(0, 100.0, 100.0, lane) is True

    def test_short_move_completed_before_motion(self):
        # feed_departure_timeout=0 -> departure loop never runs (no motion seen),
        # and a short expected move in a healthy state is treated as done.
        ace = _ace(status={"status": "ready", "slots": [{"status": "ready"}]})
        unit = make_unit(_ace=ace, feed_departure_timeout=0.0)
        assert unit._wait_for_feed_complete(0, 1.0, 100.0) is True
        assert unit.logger.lines["debug"] == [
            "ACE wait: slot 0 short move (1mm) completed before motion was "
            "observed — treating as done"]

    def test_genuine_no_start_returns_false(self):
        ace = _ace(status={"status": "ready", "slots": [{"status": "ready"}]})
        unit = make_unit(_ace=ace, feed_departure_timeout=0.0)
        assert unit._wait_for_feed_complete(0, 500.0, 100.0) is False
        assert unit.logger.lines["debug"] == [
            "ACE wait: slot 0 never reported motion after feed/unwind "
            "command — motor may not have started"]

    def test_moves_then_completes(self):
        # First poll shows moving (phase 0 breaks), then two idle reads complete.
        seq = iter([
            {"status": "busy"},
            {"status": "ready", "slots": [{"status": "ready"}]},
            {"status": "ready", "slots": [{"status": "ready"}]},
        ])
        ace = _ace()
        ace.get_status = lambda *a, **k: next(seq)
        unit = make_unit(_ace=ace)
        assert unit._wait_for_feed_complete(0, 100.0, 100.0) is True


# ── _smart_load_retry ─────────────────────────────────────────────────────────

class TestSmartLoadRetry:
    def test_succeeds_when_sensor_triggers(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._toolhead_sensor_triggered = lambda l: True
        lane = FakeLane("l0")
        assert unit._smart_load_retry(lane, 0, 100.0) is True
        assert ace.feed_filament.call_count == 1

    def test_exhausts_retries(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._toolhead_sensor_triggered = lambda l: False
        lane = FakeLane("l0")
        assert unit._smart_load_retry(lane, 0, 100.0, max_retries=2) is False
        assert ace.feed_filament.call_count == 2


# ── _stop_feed_assist ─────────────────────────────────────────────────────────

class TestStopFeedAssist:
    def test_not_tracked_noop(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _feed_assist_active=set())
        unit._stop_feed_assist(0)
        assert ace.stop_feed_assist_sync.call_count == 0

    def test_disconnected_noop(self):
        ace = _ace(connected=False)
        unit = make_unit(_ace=ace, _feed_assist_active={0})
        unit._stop_feed_assist(0)
        assert ace.stop_feed_assist_sync.call_count == 0

    def test_success_discards(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _feed_assist_active={0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._stop_feed_assist(0)
        assert unit._feed_assist_active == set()

    def test_timeout_retries_then_errors(self):
        ace = _ace()
        ace.stop_feed_assist_sync = Recorder(raises=ACETimeoutError("to"))
        unit = make_unit(_ace=ace, _feed_assist_active={0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._stop_feed_assist(0)
        assert ace.stop_feed_assist_sync.call_count == 3
        assert unit.logger.lines["error"] == [
            "Failed to stop feed assist slot 0 after 3 attempts: to"]

    def test_generic_error_logged_once(self):
        ace = _ace()
        ace.stop_feed_assist_sync = Recorder(raises=RuntimeError("boom"))
        unit = make_unit(_ace=ace, _feed_assist_active={0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._stop_feed_assist(0)
        assert ace.stop_feed_assist_sync.call_count == 1
        assert unit.logger.lines["error"] == [
            "Failed to stop feed assist slot 0: boom"]


# ── _start_feed_assist (uncovered error/retry paths) ──────────────────────────

class TestStartFeedAssist:
    def test_timeout_retries_then_errors(self):
        ace = _ace()
        ace.start_feed_assist = Recorder(raises=ACETimeoutError("to"))
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._start_feed_assist(0)
        assert ace.start_feed_assist.call_count == 3
        assert 0 not in unit._feed_assist_active
        assert unit.logger.lines["error"] == [
            "Failed to start feed assist slot 0 after 3 attempts: to"]

    def test_error_2_already_assisting_marks_tracked(self):
        ace = _ace()
        ace.start_feed_assist = Recorder(
            raises=RuntimeError("code=2, msg=error_2"))
        unit = make_unit(_ace=ace, _cached_hw_status={
            "slots": [{"slot_status": "assisting"}]})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._start_feed_assist(0)
        assert 0 in unit._feed_assist_active
        assert unit.logger.lines["debug"] == [
            "Feed assist slot 0 already assisting (error_2); marked tracked"]

    def test_forbidden_logged_debug(self):
        ace = _ace()
        ace.start_feed_assist = Recorder(raises=RuntimeError("FORBIDDEN"))
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._start_feed_assist(0)
        assert 0 not in unit._feed_assist_active
        assert unit.logger.lines["error"] == []
        assert any("FORBIDDEN" in m for m in unit.logger.lines["debug"])


# ── eject_lane ────────────────────────────────────────────────────────────────

class TestEjectLane:
    def test_disconnected_noop(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        lane = FakeLane("l0")
        lane.loaded_to_hub = True
        unit.eject_lane(lane)
        assert lane.loaded_to_hub is True  # untouched

    def test_success_clears_staging_and_suppresses(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.dist_hub = 300.0
        lane.loaded_to_hub = True
        lane.tool_loaded = False
        unit._stop_feed_assist = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True

        unit.eject_lane(lane)

        assert ace.unwind_filament.last_args == (0, 300.0 + 475.0, 100.0)
        assert lane.loaded_to_hub is False
        assert "l0" in unit._hub_load_suppressed
        assert unit.logger.lines["info"] == [
            "ACE eject l0: unwinding 775mm (dist_hub=300mm)"]

    def test_exception_logged(self):
        ace = _ace()
        ace.unwind_filament = Recorder(raises=RuntimeError("bad"))
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.dist_hub = 300.0
        lane.tool_loaded = False
        unit._stop_feed_assist = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True

        unit.eject_lane(lane)

        assert unit.logger.lines["error"] == ["ACE eject failed for l0: bad"]


# ── lane_move ─────────────────────────────────────────────────────────────────

class TestLaneMove:
    def test_disconnected_logs_error(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        unit.lane_move(FakeLane("l0"), 100.0, None)
        assert unit.logger.lines["error"] == ["ACE not connected for lane_move"]

    def test_positive_feeds(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.lane_move(FakeLane("l0"), 50.0, None)
        assert ace.feed_filament.last_args == (0, 50.0, 100.0)
        assert ace.unwind_filament.call_count == 0

    def test_negative_unwinds(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.lane_move(FakeLane("l0"), -50.0, None)
        assert ace.unwind_filament.last_args == (0, 50.0, 100.0)
        assert ace.feed_filament.call_count == 0

    def test_exception_logged(self):
        ace = _ace()
        ace.feed_filament = Recorder(raises=RuntimeError("x"))
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit.lane_move(FakeLane("l0"), 50.0, None)
        assert unit.logger.lines["error"] == ["ACE lane_move failed: x"]


# ── lane_unload ───────────────────────────────────────────────────────────────

class TestLaneUnload:
    def test_disconnected_returns_true(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        assert unit.lane_unload(FakeLane("l0")) is True

    def test_success_clears_hub_state(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True), tool_loaded=True)
        lane.dist_hub = 100.0
        lane.hub_obj = FakeHub(virtual=True)
        lane.loaded_to_hub = True
        unit._stop_feed_assist = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._get_eject_length = lambda l: 800.0

        assert unit.lane_unload(lane) is True
        assert ace.unwind_filament.last_args == (0, 800.0, 100.0)
        assert lane.loaded_to_hub is False


# ── prepare_unload / lane_unloading ───────────────────────────────────────────

class TestPrepareUnload:
    def test_sets_operation_active_and_stops_assist(self):
        unit = make_unit(_slot_map={"l0": 0})
        unit._stop_feed_assist = Recorder()
        unit.prepare_unload(FakeLane("l0"), None, None)
        assert unit._operation_active is True
        assert unit._stop_feed_assist.calls == [((0,), {})]


class TestLaneUnloading:
    def test_calls_prepare_unload(self):
        unit = make_unit(_slot_map={"l0": 0})
        unit.afc.function = types.SimpleNamespace(afc_led=Recorder())
        lane = FakeLane("l0")
        lane.led_unloading = "u"
        lane.led_index = 1
        unit.prepare_unload = Recorder()
        unit.lane_unloading(lane)
        assert unit.prepare_unload.call_count == 1

    def test_swallows_prepare_error(self):
        unit = make_unit(_slot_map={"l0": 0})
        unit.afc.function = types.SimpleNamespace(afc_led=Recorder())
        lane = FakeLane("l0")
        lane.led_unloading = "u"
        lane.led_index = 1
        unit.prepare_unload = Recorder(raises=RuntimeError("boom"))
        unit.lane_unloading(lane)
        assert unit.logger.lines["warning"] == [
            "ACE: lane_unloading assist-stop error for l0: boom"]


# ── unit_load_lane / unit_unload_lane ─────────────────────────────────────────

class TestUnitLoadLane:
    def test_failure_returns_false(self):
        unit = make_unit()
        unit._ace_load_sequence = Recorder(result=False)
        lane = FakeLane("l0")
        assert unit.unit_load_lane(lane, types.SimpleNamespace()) is False
        assert not unit.afc.save_vars.called

    def test_success_sets_status(self):
        unit = make_unit()
        unit._ace_load_sequence = Recorder(result=True)
        lane = FakeLane("l0")
        assert unit.unit_load_lane(lane, types.SimpleNamespace()) is True
        assert lane.status == AFCLaneState.TOOL_LOADED
        assert unit.afc.save_vars.call_count == 1


class TestUnitUnloadLane:
    def _extruder(self):
        return types.SimpleNamespace(tool_unload_speed=50)

    def _lane(self):
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.disable_buffer = Recorder()
        lane.select_lane = Recorder()
        return lane

    def test_success_full_sequence(self):
        unit = make_unit()
        unit.afc.move_e_pos = Recorder()
        unit.afc.do_tool_cut_tip_form = Recorder()
        unit.afc.post_unload_macro = None
        unit._ace_unload_sequence = Recorder(result=True)
        lane = self._lane()
        ext = self._extruder()

        assert unit.unit_unload_lane(lane, ext) is True
        assert lane.status == AFCLaneState.NONE
        assert lane.set_tool_unloaded.call_count == 1
        assert unit._operation_active is False
        assert unit._prev_states_stale is True

    def test_unload_sequence_failure_returns_false(self):
        unit = make_unit()
        unit.afc.move_e_pos = Recorder()
        unit.afc.do_tool_cut_tip_form = Recorder()
        unit._ace_unload_sequence = Recorder(result=False)
        lane = self._lane()
        ext = self._extruder()

        assert unit.unit_unload_lane(lane, ext) is False
        assert unit._operation_active is False

    def test_cut_exception_still_clears_operation_flag(self):
        unit = make_unit(_operation_active=True)
        unit.afc.move_e_pos = Recorder(raises=RuntimeError("cut boom"))
        lane = FakeLane("l0")
        ext = self._extruder()

        with pytest.raises(RuntimeError, match="cut boom"):
            unit.unit_unload_lane(lane, ext)
        assert unit._operation_active is False
        assert unit._prev_states_stale is True


# ── _clear_stale_sensor_state ─────────────────────────────────────────────────

class TestClearStaleSensorState:
    def test_clears_sensor_and_buffer_latch(self):
        unit = make_unit()
        sensor = types.SimpleNamespace(
            runout_helper=types.SimpleNamespace(filament_present=True))
        buffer = types.SimpleNamespace(clear_advance_latch=Recorder())
        ext = types.SimpleNamespace(
            filament_sensor_obj=sensor, tool_start_state=True)
        lane = FakeLane("l0", extruder_obj=ext)
        lane.buffer_obj = buffer

        unit._clear_stale_sensor_state(lane)

        assert sensor.runout_helper.filament_present is False
        assert ext.tool_start_state is False
        assert buffer.clear_advance_latch.call_count == 1

    def test_handles_missing_objects(self):
        unit = make_unit()
        ext = types.SimpleNamespace(filament_sensor_obj=None,
                                    fila_tool_start=None)
        lane = FakeLane("l0", extruder_obj=ext)
        lane.buffer_obj = None
        unit._clear_stale_sensor_state(lane)  # must not raise


# ── _feed_until_sensor ────────────────────────────────────────────────────────

class TestFeedUntilSensor:
    def test_triggers_on_first_step(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._toolhead_sensor_triggered = lambda l: True
        lane = FakeLane("l0")
        fed, triggered = unit._feed_until_sensor(0, lane, 100.0, step_size=50.0)
        assert (fed, triggered) == (50.0, True)
        assert unit.logger.lines["info"] == [
            "ACE calibration: sensor triggered at 50.0mm"]

    def test_reaches_max_without_trigger(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._toolhead_sensor_triggered = lambda l: False
        lane = FakeLane("l0")
        fed, triggered = unit._feed_until_sensor(0, lane, 100.0, step_size=50.0)
        assert (fed, triggered) == (100.0, False)

    def test_feed_exception_retries(self):
        ace = _ace()
        calls = {"n": 0}

        def feed(slot, step, speed):
            calls["n"] += 1
            if calls["n"] == 1:
                raise RuntimeError("first fails")
        ace.feed_filament = feed
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._toolhead_sensor_triggered = lambda l: True
        lane = FakeLane("l0")
        fed, triggered = unit._feed_until_sensor(0, lane, 100.0, step_size=50.0)
        assert (fed, triggered) == (50.0, True)
        assert calls["n"] == 2  # retried the failed step
        assert unit.logger.lines["warning"] == [
            "ACE calibration: feed failed at 0mm, retrying: first fails"]


# ── system_Test ───────────────────────────────────────────────────────────────

class TestSystemTest:
    def _base_unit(self):
        unit = make_unit(_slot_map={"l0": 0})
        unit.lane_not_ready = Recorder()
        unit.lane_fault = Recorder()
        unit.lane_loaded = Recorder()
        unit.lane_illuminate_spool = Recorder()
        unit.lane_tool_loaded = Recorder()
        unit.lane_tool_loaded_idle = Recorder()
        unit.afc.function = types.SimpleNamespace(TcmdAssign=Recorder())
        return unit

    def test_not_connected_reports_error(self):
        unit = self._base_unit()
        unit._ace = FakeAce(connected=False)
        lane = FakeLane("l0")
        lane.raw_load_state = False
        lane.map = "T0"
        lane.send_lane_data = Recorder()
        lane.do_enable = Recorder()
        lane.set_afc_prep_done = Recorder()

        ok = unit.system_Test(lane, 0.0, True, False)  # assignTcmd=True

        assert ok is False
        # Not connected -> succeeded=False before the prep-state block, so the
        # lane_* helpers are skipped; only the tail (Tcmd/prep-done) runs.
        assert not unit.lane_not_ready.called
        assert not unit.lane_loaded.called
        assert unit.afc.function.TcmdAssign.call_count == 1
        assert lane.set_afc_prep_done.call_count == 1

    def test_empty_ready_for_spool(self):
        unit = self._base_unit()
        ace = _ace(status={"status": "ready", "slots": [{"status": "empty"}]})
        unit._ace = ace
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.raw_load_state = False
        lane.map = "T0"
        lane.send_lane_data = Recorder()
        lane.do_enable = Recorder()
        lane.set_afc_prep_done = Recorder()

        ok = unit.system_Test(lane, 0.0, False, False)

        assert ok is True
        assert lane.prep_state is False
        assert unit.lane_not_ready.call_count == 1
        assert not unit.lane_loaded.called


# ── _ace_load_inner (not-connected + pre-feed guard) ──────────────────────────

class TestAceLoadInner:
    def test_not_connected_fails(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(in_print=lambda: False)
        lane = FakeLane("l0")
        assert unit._ace_load_inner(lane, types.SimpleNamespace()) is False
        assert unit.afc.error.handle_lane_failure.call_count == 1

    def test_pre_feed_sensor_triggered_fails(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(in_print=lambda: True)
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.loaded_to_hub = False
        lane.buffer_obj = None
        unit._get_bowden_length = lambda l: 100.0
        unit._toolhead_sensor_triggered = lambda l: True

        assert unit._ace_load_inner(lane, types.SimpleNamespace()) is False
        msg = unit.afc.error.handle_lane_failure.last_args[1]
        assert "detects filament before ACE feed" in msg


# ── _ace_unload_inner (not connected) ─────────────────────────────────────────

class TestAceUnloadInner:
    def test_not_connected_fails(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(in_print=lambda: False)
        lane = FakeLane("l0")
        assert unit._ace_unload_inner(lane, types.SimpleNamespace()) is False
        assert unit.afc.error.handle_lane_failure.call_count == 1


# ── _calibrate_hub_inner ──────────────────────────────────────────────────────

class TestCalibrateHubInner:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        lane = FakeLane("l0")
        ok, msg, dist = unit._calibrate_hub_inner(lane)
        assert (ok, msg, dist) == (False, "ACE not connected", 0)

    def test_virtual_hub_rejected(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        ok, msg, dist = unit._calibrate_hub_inner(lane)
        assert (ok, dist) == (False, 0)
        assert "Physical hub sensor required" in msg

    def test_hub_already_triggered(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        hub = types.SimpleNamespace(is_virtual_pin=lambda: False, state=True)
        lane = FakeLane("l0", hub_obj=hub)
        ok, msg, dist = unit._calibrate_hub_inner(lane)
        assert (ok, dist) == (False, 0)
        assert "already triggered" in msg


# ── calibrate_bowden ──────────────────────────────────────────────────────────

class TestCalibrateBowden:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        assert unit.calibrate_bowden(lane, 50.0, 1.0) == (
            False, "ACE not connected", 0)

    def test_no_hub(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=None)
        assert unit.calibrate_bowden(lane, 50.0, 1.0) == (
            False, "Lane has no hub configured", 0)

    def test_delegates_to_inner(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        unit._calibrate_bowden_inner = Recorder(result=(True, "done", 5.0))
        assert unit.calibrate_bowden(lane, 50.0, 1.0) == (True, "done", 5.0)
        assert unit._operation_active is False


# ── calibrate_lane ────────────────────────────────────────────────────────────

class TestCalibrateLane:
    def test_wraps_hub_inner(self):
        unit = make_unit()
        unit._calibrate_hub_inner = Recorder(result=(True, "ok", 3.0))
        assert unit.calibrate_lane(FakeLane("l0"), 1.0) == (True, "ok", 3.0)
        assert unit._operation_active is False
        assert unit._prev_states_stale is True


# ── calibrate_td1 / capture_td1_data guards ───────────────────────────────────

class TestCalibrateTd1:
    def test_wraps_inner_and_clears_flag(self):
        unit = make_unit()
        unit._calibrate_td1_inner = Recorder(result=(True, "ok", 9.0))
        assert unit.calibrate_td1(FakeLane("l0"), 50.0, 1.0) == (True, "ok", 9.0)
        assert unit._operation_active is False
        assert unit._prev_states_stale is True


class TestCaptureTd1Data:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False), _slot_map={"l0": 0})
        lane = FakeLane("l0")
        assert unit.capture_td1_data(lane) == (False, "ACE not connected")

    def test_missing_device_id(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        lane = FakeLane("l0")
        lane.td1_device_id = None
        assert unit.capture_td1_data(lane) == (
            False, "td1_device_id not set for lane")

    def test_missing_bowden_length(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        lane = FakeLane("l0")
        lane.td1_device_id = "td1"
        lane.td1_bowden_length = None
        assert unit.capture_td1_data(lane) == (
            False, "td1_bowden_length not set — run TD-1 calibration first")

    def test_invalid_td1_id_surfaced(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        unit.afc.function = types.SimpleNamespace(
            check_for_td1_id=lambda i: (False, "bad td1"))
        lane = FakeLane("l0")
        lane.td1_device_id = "td1"
        lane.td1_bowden_length = 300.0
        assert unit.capture_td1_data(lane) == (False, "bad td1")


# ── cmd_ACE_FEED_TEST (guard branches) ────────────────────────────────────────

class TestCmdAceFeedTest:
    def test_not_connected(self):
        unit = make_unit(_ace=FakeAce(connected=False))
        gcmd = Gcmd()
        unit.cmd_ACE_FEED_TEST(gcmd)
        assert gcmd.responses == ["ACE not connected"]

    def test_no_loaded_slot(self):
        unit = make_unit(_ace=_ace())
        gcmd = Gcmd(SLOT=-1, LENGTH=100.0, START=10, END=250, STEP=20)
        unit._pick_test_slot = lambda: None
        unit.cmd_ACE_FEED_TEST(gcmd)
        assert gcmd.responses == [
            "ACE_FEED_TEST: no loaded slot detected — pass SLOT=<n>"]


# ── cmd_ACE_CALIBRATE / cmd_ACE_CALIBRATE_HUB ─────────────────────────────────

class TestCmdAceCalibrate:
    def test_usage_on_unknown_lane(self):
        unit = make_unit()
        unit.afc.lanes = {}
        gcmd = Gcmd(LANE="ghost")
        unit.cmd_ACE_CALIBRATE(gcmd)
        assert gcmd.responses == ["Usage: ACE_CALIBRATE LANE=<lane_name>"]

    def test_runs_calibration(self):
        unit = make_unit()
        lane = FakeLane("l0")
        unit.afc.lanes = {"l0": lane}
        unit.calibrate_bowden = Recorder(result=(True, "done 5mm", 5.0))
        gcmd = Gcmd(LANE="l0")
        unit.cmd_ACE_CALIBRATE(gcmd)
        assert gcmd.responses == ["done 5mm"]


class TestCmdAceCalibrateHub:
    def test_usage_on_unknown_lane(self):
        unit = make_unit()
        unit.afc.lanes = {}
        gcmd = Gcmd(LANE="ghost")
        unit.cmd_ACE_CALIBRATE_HUB(gcmd)
        assert gcmd.responses == ["Usage: ACE_CALIBRATE_HUB LANE=<lane_name>"]

    def test_rejects_virtual_hub(self):
        unit = make_unit()
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        unit.afc.lanes = {"l0": lane}
        gcmd = Gcmd(LANE="l0")
        unit.cmd_ACE_CALIBRATE_HUB(gcmd)
        assert gcmd.responses == [
            "Hub calibration requires a physical hub sensor, not virtual"]

    def test_runs_hub_calibration(self):
        unit = make_unit()
        hub = types.SimpleNamespace(is_virtual_pin=lambda: False)
        lane = FakeLane("l0", hub_obj=hub)
        unit.afc.lanes = {"l0": lane}
        unit._calibrate_hub_inner = Recorder(result=(True, "hub 3mm", 3.0))
        gcmd = Gcmd(LANE="l0")
        unit.cmd_ACE_CALIBRATE_HUB(gcmd)
        assert gcmd.responses == ["hub 3mm"]


# ── _deferred_ace_connect ─────────────────────────────────────────────────────

class TestDeferredAceConnect:
    def test_success_first_try(self):
        ace = _ace()
        unit = make_unit(lanes={})
        unit._create_serial_logger = lambda: None
        unit._make_connection = Recorder(result=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._apply_feed_check = Recorder()
        unit._sync_inventory = Recorder()
        unit._sync_slot_loaded_state = Recorder()

        unit._deferred_ace_connect(0.0)

        assert unit._ace is ace
        assert ace.enable_rfid.call_count == 1
        assert unit._prev_states_stale is True
        assert ace.status_callback == unit._on_hw_status_callback
        assert any("connected, mode=" in m for m in unit.logger.lines["info"])

    def test_all_attempts_fail_logs_error(self):
        unit = make_unit()
        unit._create_serial_logger = lambda: None
        unit._make_connection = Recorder(raises=RuntimeError("no port"))

        unit._deferred_ace_connect(0.0)

        assert unit._ace is None
        assert len(unit.logger.lines["error"]) == 1
        assert "failed to connect" in unit.logger.lines["error"][0]


# ══════════════════════════════════════════════════════════════════════════════
# ACEConnection serial transport
# ══════════════════════════════════════════════════════════════════════════════

import sys  # noqa: E402

from extras.AFC_ACE import (  # noqa: E402
    ACEConnection,
    ACESerialError,
    HEARTBEAT_INTERVAL,
    crc16_ccitt_reflected as _crc,
    load_config_prefix,
)


class FakeCompletion:
    def __init__(self, result=None):
        self._result = result
        self.completed = "unset"

    def wait(self, deadline):
        return self._result

    def complete(self, value):
        self.completed = value


class ConnReactor:
    NEVER = 9e18

    def __init__(self):
        self.mono = 100.0
        self.completion_result = None
        self.timers = []
        self.unregistered = []
        self.fds = []

    def monotonic(self):
        return self.mono

    def completion(self):
        return FakeCompletion(self.completion_result)

    def register_timer(self, cb, when):
        handle = ("timer", len(self.timers))
        self.timers.append((cb, when))
        return handle

    def unregister_timer(self, handle):
        self.unregistered.append(handle)

    def register_fd(self, fd, cb):
        self.fds.append((fd, cb))
        return ("fd", fd)

    def unregister_fd(self, handle):
        pass

    def pause(self, until):
        pass


class FakeSerial:
    def __init__(self, read_data=b"", write_raises=None):
        self._read_data = read_data
        self.write_raises = write_raises
        self.written = []
        self.flushed = 0
        self.closed = False

    def write(self, data):
        if self.write_raises is not None:
            raise self.write_raises
        self.written.append(data)

    def flush(self):
        self.flushed += 1

    def read(self, n):
        d, self._read_data = self._read_data, b""
        return d

    def fileno(self):
        return 3

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.closed = True


def _conn(connected=True, serial=None):
    reactor = ConnReactor()
    conn = ACEConnection(reactor=reactor, serial_port="/dev/ttyACM0",
                         logger=FakeLogger(), baud_rate=115200)
    if connected:
        conn._connected = True
        conn._serial = serial if serial is not None else FakeSerial()
    return conn


# ── connected / __init__ ──────────────────────────────────────────────────────

class TestConnectionConnected:
    def test_default_disconnected(self):
        conn = _conn(connected=False)
        assert conn.connected is False
        assert conn.slot_count == 4

    def test_connected_true(self):
        assert _conn().connected is True


# ── _build_frame ──────────────────────────────────────────────────────────────

class TestBuildFrame:
    def test_frame_layout(self):
        conn = _conn(connected=False)
        payload = b"hi"
        frame = conn._build_frame(payload)
        assert frame[:2] == b"\xff\xaa"
        assert frame[-1:] == b"\xfe"
        # length little-endian, payload, then CRC little-endian.
        assert frame[2:4] == (2).to_bytes(2, "little")
        assert frame[4:6] == payload
        assert frame[6:8] == _crc(payload).to_bytes(2, "little")


# ── send_command ──────────────────────────────────────────────────────────────

class TestSendCommand:
    def test_not_connected_raises(self):
        conn = _conn(connected=False)
        with pytest.raises(ACESerialError, match="ACE not connected"):
            conn.send_command("get_status")

    def test_payload_too_large_raises(self):
        conn = _conn()
        big = "x" * 2000
        with pytest.raises(ACESerialError, match="payload too large"):
            conn.send_command("m", params={"big": big})

    def test_success_returns_result(self):
        conn = _conn()
        conn._reactor.completion_result = {"id": 0, "result": {"ok": 1}}
        assert conn.send_command("get_status") == {"ok": 1}
        assert conn._serial.written  # frame was written
        assert 0 not in conn._pending  # pending cleared

    def test_error_code_raises(self):
        conn = _conn()
        conn._reactor.completion_result = {"id": 0, "code": 2, "msg": "error_2"}
        with pytest.raises(ACESerialError, match="code=2, msg=error_2"):
            conn.send_command("start_feed_assist")

    def test_timeout_raises(self):
        conn = _conn()
        conn._reactor.completion_result = None
        with pytest.raises(ACETimeoutError, match="timed out"):
            conn.send_command("get_status")
        assert len(conn._timeout_timestamps) == 1

    def test_write_failure_reconnects(self):
        conn = _conn(serial=FakeSerial(write_raises=OSError("cable")))
        with pytest.raises(ACESerialError, match="write failed"):
            conn.send_command("get_status")
        assert conn._connected is False  # reconnect() disconnected


# ── send_command_async ────────────────────────────────────────────────────────

class TestSendCommandAsync:
    def test_not_connected_noop(self):
        conn = _conn(connected=False)
        conn.send_command_async("get_status")  # no raise
        assert conn._next_request_id == 0

    def test_success_records_async_id(self):
        conn = _conn()
        conn.send_command_async("get_status")
        assert list(conn._async_ids) == [0]
        assert conn._serial.written

    def test_write_failure_reconnects(self):
        conn = _conn(serial=FakeSerial(write_raises=OSError("x")))
        conn.send_command_async("get_status")
        assert conn._connected is False


# ── disconnect ────────────────────────────────────────────────────────────────

class TestDisconnect:
    def test_closes_and_fails_pending(self):
        conn = _conn()
        pending = FakeCompletion()
        conn._pending[5] = pending
        conn._fd_handle = ("fd", 3)
        serial = conn._serial

        conn.disconnect()

        assert conn._connected is False
        assert serial.closed is True
        assert conn._pending == {}
        assert pending.completed is None
        assert conn._logger.lines["info"] == ["ACE serial disconnected"]


# ── reconnect / _quick_reconnect ──────────────────────────────────────────────

class TestReconnect:
    def test_disabled_noop(self):
        conn = _conn()
        conn._reconnect_enabled = False
        conn.reconnect()
        assert conn._connected is True  # untouched

    def test_schedules_timer_with_backoff(self):
        conn = _conn()
        conn.reconnect()
        assert conn._connected is False
        assert len(conn._reactor.timers) == 1
        assert conn._reconnect_backoff > 5.0  # advanced from min

    def test_quick_reconnect_schedules_fast_timer(self):
        conn = _conn()
        conn._quick_reconnect()
        assert conn._connected is False
        cb, when = conn._reactor.timers[-1]
        assert when == conn._reactor.monotonic() + 0.5


# ── _start_heartbeat / _stop_heartbeat / _heartbeat_tick ──────────────────────

class TestHeartbeat:
    def test_start_registers_once(self):
        conn = _conn()
        conn._start_heartbeat()
        first = conn._heartbeat_timer
        conn._start_heartbeat()  # already running -> no new timer
        assert conn._heartbeat_timer is first

    def test_stop_unregisters(self):
        conn = _conn()
        conn._heartbeat_timer = ("timer", 0)
        conn._stop_heartbeat()
        assert conn._heartbeat_timer is None

    def test_tick_sends_and_reschedules(self):
        conn = _conn()
        conn._last_rx_time = conn._reactor.monotonic()
        nxt = conn._heartbeat_tick(conn._reactor.monotonic())
        assert nxt == conn._reactor.monotonic() + HEARTBEAT_INTERVAL
        assert conn._serial.written  # get_status async sent

    def test_tick_reconnects_on_silence(self):
        conn = _conn()
        conn._last_rx_time = 0.0  # very old vs monotonic 100
        nxt = conn._heartbeat_tick(conn._reactor.monotonic())
        assert nxt == conn._reactor.NEVER
        assert conn._connected is False

    def test_tick_disconnected_returns_never(self):
        conn = _conn(connected=False)
        assert conn._heartbeat_tick(0.0) == conn._reactor.NEVER


# ── health supervision ────────────────────────────────────────────────────────

class TestSupervision:
    def test_track_timeout_prunes_old(self):
        conn = _conn()
        conn._reactor.mono = 100.0
        conn._timeout_timestamps = [10.0]  # older than window
        conn._track_timeout()
        assert conn._timeout_timestamps == [100.0]

    def test_supervision_forces_reconnect_when_unhealthy(self):
        conn = _conn()
        conn._last_supervision_check = 0.0
        conn._timeout_timestamps = [99.0] * 15
        conn._unsolicited_timestamps = [99.0] * 15
        conn._supervision_check()
        assert conn._connected is False

    def test_supervision_skips_when_recent(self):
        conn = _conn()
        conn._last_supervision_check = conn._reactor.monotonic()
        conn._supervision_check()
        assert conn._connected is True


# ── _parse_frames / _handle_read ──────────────────────────────────────────────

class TestParseFrames:
    def test_no_header_clears_buffer(self):
        conn = _conn()
        conn._read_buffer = b"garbage"
        conn._parse_frames()
        assert conn._read_buffer == b""

    def test_partial_frame_waits(self):
        conn = _conn()
        conn._read_buffer = b"\xff\xaa\x02"  # header but < 7 bytes
        conn._parse_frames()
        assert conn._read_buffer == b"\xff\xaa\x02"

    def test_valid_frame_routes_to_callback(self):
        conn = _conn()
        received = []
        conn.status_callback = lambda r: received.append(r)
        payload = b'{"result":1}'
        conn._read_buffer = conn._build_frame(payload)
        conn._parse_frames()
        assert received == [{"result": 1}]
        assert conn._read_buffer == b""

    def test_footer_mismatch_rescans(self):
        conn = _conn()
        frame = bytearray(conn._build_frame(b'{"a":1}'))
        frame[-1] = 0x00  # corrupt footer
        conn._read_buffer = bytes(frame)
        conn._parse_frames()
        assert conn._read_buffer == b""  # scanned to exhaustion

    def test_oversized_length_skips_false_header(self):
        conn = _conn()
        conn._read_buffer = b"\xff\xaa\xff\xff" + b"\x00" * 4
        conn._parse_frames()
        assert conn._read_buffer == b""

    def test_bad_json_warns(self):
        conn = _conn()
        conn._read_buffer = conn._build_frame(b"not json")
        conn._parse_frames()
        assert any("JSON parse error" in m
                   for m in conn._logger.lines["warning"])


class TestHandleRead:
    def test_no_data_returns(self):
        conn = _conn(serial=FakeSerial(read_data=b""))
        conn._handle_read(101.0)
        assert conn._read_buffer == b""

    def test_data_buffered(self):
        conn = _conn(serial=FakeSerial(read_data=b"\xff\xaa\x02"))
        conn._handle_read(101.0)
        assert conn._last_rx_time == 101.0

    def test_read_error_reconnects(self):
        class BadSerial(FakeSerial):
            def read(self, n):
                raise OSError("device error")
        conn = _conn(serial=BadSerial())
        conn._handle_read(101.0)
        assert conn._connected is False


# ── _handle_response ──────────────────────────────────────────────────────────

class TestHandleResponse:
    def test_unsolicited_forwarded(self):
        conn = _conn()
        seen = []
        conn.status_callback = lambda r: seen.append(r)
        conn._handle_response({"result": 1})  # no id
        assert seen == [{"result": 1}]
        assert len(conn._unsolicited_timestamps) == 1

    def test_pending_completed(self):
        conn = _conn()
        comp = FakeCompletion()
        conn._pending[7] = comp
        conn._handle_response({"id": 7, "result": 1})
        assert comp.completed == {"id": 7, "result": 1}

    def test_async_id_not_counted_unsolicited(self):
        conn = _conn()
        conn._async_ids.append(9)
        seen = []
        conn.status_callback = lambda r: seen.append(r)
        conn._handle_response({"id": 9, "result": 2})
        assert seen == [{"id": 9, "result": 2}]
        assert conn._unsolicited_timestamps == []  # async -> not counted

    def test_unknown_id_counted_and_logged(self):
        conn = _conn()
        conn._handle_response({"id": 999})
        assert len(conn._unsolicited_timestamps) == 1


class TestResponseMatchesPending:
    def test_base_always_true(self):
        conn = _conn()
        assert conn._response_matches_pending(1, {"id": 1}) is True


# ── connect ───────────────────────────────────────────────────────────────────

class TestConnect:
    def test_already_connected_noop(self):
        conn = _conn()
        conn.connect()  # returns immediately
        assert conn._serial is not None

    def test_missing_pyserial_raises(self, monkeypatch):
        conn = _conn(connected=False)
        monkeypatch.setitem(sys.modules, "serial", None)
        with pytest.raises(ACESerialError, match="pyserial not installed"):
            conn.connect()

    def test_success_opens_and_starts_heartbeat(self, monkeypatch):
        conn = _conn(connected=False)
        fake_serial = FakeSerial()
        fake_mod = types.SimpleNamespace(Serial=lambda **kw: fake_serial)
        monkeypatch.setitem(sys.modules, "serial", fake_mod)
        conn._reactor.completion_result = {"id": 0, "result": {"fw": "1.0"}}

        conn.connect()

        assert conn._connected is True
        assert conn.device_info == {"fw": "1.0"}
        assert conn._heartbeat_timer is not None
        assert conn._fd_handle is not None


# ── High-level command wrappers ───────────────────────────────────────────────

class TestConnectionCommandWrappers:
    def _conn_stub(self):
        conn = _conn()
        conn.send_command = Recorder(result="R")
        conn.send_command_async = Recorder()
        return conn

    def test_get_status(self):
        conn = self._conn_stub()
        assert conn.get_status(timeout=5.0) == "R"
        assert conn.send_command.last_args == ("get_status",)
        assert conn.send_command.last_kwargs == {"timeout": 5.0}

    def test_get_temp(self):
        conn = self._conn_stub()
        conn.get_temp()
        assert conn.send_command.last_args == ("get_temp",)

    def test_get_filament_info(self):
        conn = self._conn_stub()
        conn.get_filament_info(2)
        assert conn.send_command.last_kwargs["params"] == {"index": 2}

    def test_get_material_info(self):
        conn = self._conn_stub()
        conn.get_material_info(1)
        assert conn.send_command.last_args == ("get_material_info",)
        assert conn.send_command.last_kwargs["params"] == {"index": 1}

    def test_set_material_name(self):
        conn = self._conn_stub()
        conn.set_material_name(0, "PLA")
        assert conn.send_command.last_kwargs["params"] == {
            "index": 0, "name": "PLA"}

    def test_get_sensor_state(self):
        conn = self._conn_stub()
        conn.get_sensor_state()
        assert conn.send_command.last_args == ("get_sensor_state",)

    def test_feed_filament(self):
        conn = self._conn_stub()
        conn.feed_filament(0, 100.0, 50.0)
        assert conn.send_command.last_kwargs["params"] == {
            "index": 0, "length": 100.0, "speed": 50.0}

    def test_stop_feed_filament(self):
        conn = self._conn_stub()
        conn.stop_feed_filament(0)
        assert conn.send_command_async.last_args == ("stop_feed_filament",)

    def test_unwind_filament(self):
        conn = self._conn_stub()
        conn.unwind_filament(0, 100.0, 50.0)
        assert conn.send_command.last_kwargs["params"] == {
            "index": 0, "length": 100.0, "speed": 50.0, "mode": "normal"}

    def test_stop_unwind_filament(self):
        conn = self._conn_stub()
        conn.stop_unwind_filament(1)
        assert conn.send_command_async.last_args == ("stop_unwind_filament",)

    def test_start_feed_assist(self):
        conn = self._conn_stub()
        conn.start_feed_assist(3)
        assert conn.send_command.last_kwargs["params"] == {"index": 3}

    def test_stop_feed_assist(self):
        conn = self._conn_stub()
        conn.stop_feed_assist(3)
        assert conn.send_command_async.last_kwargs["params"] == {"index": 3}

    def test_stop_feed_assist_sync(self):
        conn = self._conn_stub()
        conn.stop_feed_assist_sync(3)
        assert conn.send_command.last_args == ("stop_feed_assist",)

    def test_update_feeding_speed(self):
        conn = self._conn_stub()
        conn.update_feeding_speed(0, 80.0)
        assert conn.send_command_async.last_kwargs["params"] == {
            "index": 0, "speed": 80.0}

    def test_update_unwinding_speed(self):
        conn = self._conn_stub()
        conn.update_unwinding_speed(0, 80.0)
        assert conn.send_command_async.last_args == ("update_feeding_speed",)

    def test_start_drying(self):
        conn = self._conn_stub()
        conn.start_drying(50.0, 7000, 90.0)
        assert conn.send_command.last_kwargs["params"] == {
            "temp": 50.0, "fan_speed": 7000, "duration": 90.0}

    def test_stop_drying(self):
        conn = self._conn_stub()
        conn.stop_drying()
        assert conn.send_command.last_args == ("drying_stop",)

    def test_enable_rfid(self):
        conn = self._conn_stub()
        conn.enable_rfid()
        assert conn.send_command.last_args == ("enable_rfid",)

    def test_disable_rfid(self):
        conn = self._conn_stub()
        conn.disable_rfid()
        assert conn.send_command.last_args == ("disable_rfid",)

    def test_set_filament_info(self):
        conn = self._conn_stub()
        conn.set_filament_info(0, "PLA", [1, 2, 3])
        assert conn.send_command.last_kwargs["params"] == {
            "index": 0, "type": "PLA", "color": [1, 2, 3]}


# ── module helpers ────────────────────────────────────────────────────────────

class TestPreInfoHandshake:
    def test_base_is_noop(self):
        conn = _conn(connected=False)
        assert conn._pre_info_handshake() is None


class TestPollExtras:
    def test_base_is_noop(self):
        conn = _conn(connected=False)
        assert conn._poll_extras() is None


# ══════════════════════════════════════════════════════════════════════════════
# Calibration inner routines, load/unload transports and remaining big blocks
# ══════════════════════════════════════════════════════════════════════════════

def _seq_sensor(values):
    """A _toolhead_sensor_triggered stub returning successive booleans."""
    it = iter(values)
    return lambda lane: next(it)


# ── _calibrate_bowden_inner ───────────────────────────────────────────────────

class TestCalibrateBowdenInner:
    def _unit(self):
        unit = make_unit(_ace=_ace(), _slot_map={"l0": 0})
        unit._clear_stale_sensor_state = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.afc.function = types.SimpleNamespace(ConfigRewrite=Recorder())
        return unit

    def _hub(self):
        return types.SimpleNamespace(afc_bowden_length=0,
                                     afc_unload_bowden_length=0,
                                     fullname="AFC_hub hub")

    def test_already_triggered_bails(self):
        unit = self._unit()
        unit._toolhead_sensor_triggered = lambda lane: True
        lane = FakeLane("l0")
        assert unit._calibrate_bowden_inner(lane, self._hub(), 0) == (
            False, "Toolhead sensor already triggered — unload first", 0)

    def test_success_writes_config(self):
        unit = self._unit()
        unit._toolhead_sensor_triggered = lambda lane: False
        unit._feed_until_sensor = Recorder(result=(500.0, True))
        hub = self._hub()
        lane = FakeLane("l0")
        lane.loaded_to_hub = False

        ok, msg, dist = unit._calibrate_bowden_inner(lane, hub, 0)

        assert (ok, dist) == (True, 500.0)
        assert msg == "afc_bowden_length calibration: 500.0mm (was 0mm)"
        assert hub.afc_bowden_length == 500.0
        assert hub.afc_unload_bowden_length == 500.0
        assert unit.afc.function.ConfigRewrite.call_count == 2

    def test_no_trigger_retracts_and_fails(self):
        unit = self._unit()
        unit._toolhead_sensor_triggered = lambda lane: False
        unit._feed_until_sensor = Recorder(result=(300.0, False))
        lane = FakeLane("l0")
        lane.loaded_to_hub = False

        ok, msg, dist = unit._calibrate_bowden_inner(lane, self._hub(), 0)

        assert (ok, dist) == (False, 300.0)
        assert "did not trigger after 300mm" in msg


# ── _calibrate_hub_inner (success / no-trigger bodies) ────────────────────────

class TestCalibrateHubInnerBody:
    def test_success_first_step(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0}, calibration_step=50.0)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.afc.function = types.SimpleNamespace(ConfigRewrite=Recorder())
        hub = types.SimpleNamespace(is_virtual_pin=lambda: False, state=False,
                                    fullname="AFC_hub hub")

        def feed(slot, step, speed):
            hub.state = True  # hub sensor trips after the first feed
        ace.feed_filament = feed
        lane = FakeLane("l0", hub_obj=hub)
        lane.dist_hub = 10.0
        lane.fullname = "AFC_stepper l0"

        ok, msg, dist = unit._calibrate_hub_inner(lane)

        assert (ok, dist) == (True, 50.0)
        assert msg == "dist_hub calibration: 50.0mm (was 10.0mm)"
        assert lane.dist_hub == 50.0
        assert unit.afc.function.ConfigRewrite.call_count == 1

    def test_no_trigger_fails(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0}, calibration_step=4000.0)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        hub = types.SimpleNamespace(is_virtual_pin=lambda: False, state=False,
                                    fullname="AFC_hub hub")
        lane = FakeLane("l0", hub_obj=hub)
        lane.dist_hub = 10.0

        ok, msg, dist = unit._calibrate_hub_inner(lane)

        assert (ok, dist) == (False, 4000.0)
        assert "did not trigger after 4000mm" in msg


# ── _ace_load_inner (full success body) ───────────────────────────────────────

class TestAceLoadInnerSuccess:
    def test_success_marks_loaded_and_returns_true(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(in_print=lambda: False)
        unit._get_bowden_length = lambda l: 500.0
        unit._set_hub_state = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit._use_feed_assist = lambda l: False
        # pre-feed sensor False (don't bail), post-feed sensor True (reached).
        unit._toolhead_sensor_triggered = _seq_sensor([False, True])
        lane = FakeLane("l0")
        lane.loaded_to_hub = False
        lane.buffer_obj = None
        ext = types.SimpleNamespace(tool_stn=0)

        assert unit._ace_load_inner(lane, ext) is True
        assert lane.loaded_to_hub is True
        assert ace.feed_filament.last_args == (0, 500.0, 100.0)
        assert not unit.afc.error.handle_lane_failure.called


# ── _ace_unload_inner (full success body) ─────────────────────────────────────

class TestAceUnloadInnerSuccess:
    def test_success_stages_at_hub(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(
            in_print=lambda: False, log_toolhead_pos=Recorder())
        unit.afc.move_e_pos = Recorder()
        unit._set_hub_state = Recorder()
        unit._stop_feed_assist = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.lane_tool_unloaded = Recorder()
        hub = types.SimpleNamespace(afc_unload_bowden_length=900.0)
        lane = FakeLane("l0", hub_obj=hub, tool_loaded=True)
        ext = types.SimpleNamespace(tool_stn_unload=5.0, tool_unload_speed=50)

        assert unit._ace_unload_inner(lane, ext) is True
        assert ace.unwind_filament.last_args == (0, 900.0, 100.0)
        assert lane.loaded_to_hub is True
        assert "l0" in unit._hub_load_suppressed
        assert unit.lane_tool_unloaded.call_count == 1


# ── system_Test (LOADED and TOOLED paths) ─────────────────────────────────────

class TestSystemTestLoaded:
    def _unit(self):
        unit = make_unit(_slot_map={"l0": 0}, _ace=_ace(
            status={"status": "ready", "slots": [{"status": "ready"}]}))
        unit.lane_not_ready = Recorder()
        unit.lane_fault = Recorder()
        unit.lane_loaded = Recorder()
        unit.lane_illuminate_spool = Recorder()
        unit.lane_tool_loaded = Recorder()
        unit.lane_tool_loaded_idle = Recorder()
        unit.afc.function = types.SimpleNamespace(TcmdAssign=Recorder())
        return unit

    def _lane(self, **kw):
        lane = FakeLane("l0", hub_obj=FakeHub(virtual=True))
        lane.raw_load_state = False
        lane.map = "T0"
        lane.load_to_hub = False
        lane.spool_id = 7
        lane.send_lane_data = Recorder()
        lane.do_enable = Recorder()
        lane.set_afc_prep_done = Recorder()
        for k, v in kw.items():
            setattr(lane, k, v)
        return lane

    def test_present_spool_marks_loaded(self):
        unit = self._unit()
        lane = self._lane(tool_loaded=False)

        ok = unit.system_Test(lane, 0.0, False, False)

        assert ok is True
        assert lane.prep_state is True
        assert lane.status == AFCLaneState.LOADED
        assert unit.lane_loaded.call_count == 1
        assert not unit.lane_tool_loaded.called

    def test_tooled_current_lane_sets_active_spool(self):
        unit = self._unit()
        unit.afc.current = "l0"
        unit.afc.spool = types.SimpleNamespace(set_active_spool=Recorder())
        unit._use_feed_assist = lambda l: False
        lane = self._lane(
            tool_loaded=True,
            extruder_obj=FakeExtruderObj(lane_loaded="l0"))

        ok = unit.system_Test(lane, 0.0, False, False)

        assert ok is True
        assert lane.status == AFCLaneState.TOOLED
        assert unit.afc.spool.set_active_spool.calls == [((7,), {})]
        assert unit.lane_tool_loaded.call_count == 1
        assert lane.sync_to_extruder.call_count == 1


# ── cmd_ACE_FEED_TEST (sweep body) ────────────────────────────────────────────

class TestCmdAceFeedTestSweep:
    def test_runs_sweep_and_reports_verdict(self):
        ace = _ace()
        unit = make_unit(_ace=ace)
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._poll_until_status = lambda *a, **k: True
        gcmd = Gcmd(SLOT=0, LENGTH=100.0, START=10, END=30, STEP=20)
        unit.cmd_ACE_FEED_TEST(gcmd)
        # two speeds swept -> feed+unwind each, plus the summary verdict line.
        assert ace.feed_filament.call_count == 2
        assert ace.unwind_filament.call_count == 2
        assert any("ACE_FEED_TEST done." in r for r in gcmd.responses)


# ── reconnect timer callbacks ─────────────────────────────────────────────────

class TestReconnectCallbacks:
    def test_reconnect_callback_success_fires_hook(self, monkeypatch):
        conn = _conn()
        hook = Recorder()
        conn.reconnect_callback = hook
        fake_serial = FakeSerial()
        monkeypatch.setitem(sys.modules, "serial",
                            types.SimpleNamespace(Serial=lambda **kw: fake_serial))
        conn._reactor.completion_result = {"id": 0, "result": {}}

        conn.reconnect()
        cb, _when = conn._reactor.timers[-1]
        assert cb(conn._reactor.monotonic()) == conn._reactor.NEVER
        assert conn._connected is True
        assert hook.call_count == 1

    def test_reconnect_callback_failure_reschedules(self, monkeypatch):
        conn = _conn()
        monkeypatch.setitem(sys.modules, "serial", None)  # import fails
        conn.reconnect()
        cb, _when = conn._reactor.timers[-1]
        result = cb(conn._reactor.monotonic())
        assert result > conn._reactor.monotonic()  # rescheduled, not NEVER

    def test_quick_reconnect_callback_success(self, monkeypatch):
        conn = _conn()
        fake_serial = FakeSerial()
        monkeypatch.setitem(sys.modules, "serial",
                            types.SimpleNamespace(Serial=lambda **kw: fake_serial))
        conn._reactor.completion_result = {"id": 0, "result": {}}

        conn._quick_reconnect()
        cb, _when = conn._reactor.timers[-1]
        assert cb(conn._reactor.monotonic()) == conn._reactor.NEVER
        assert conn._connected is True


# ── _handle_read autosuspend ──────────────────────────────────────────────────

class TestHandleReadAutosuspend:
    def test_usb_autosuspend_quick_reconnects(self):
        class IdleSerial(FakeSerial):
            def read(self, n):
                raise OSError("device reports readiness to read but returned "
                              "no data")
        conn = _conn(serial=IdleSerial())
        conn._handle_read(101.0)
        assert conn._connected is False
        # quick reconnect resets backoff to the minimum
        assert conn._reconnect_backoff == 5.0


# ── _create_serial_logger ─────────────────────────────────────────────────────

class TestCreateSerialLogger:
    def test_returns_none_without_queue_support(self, monkeypatch):
        monkeypatch.setattr(ace_mod, "AFC_QueueListener", None)
        monkeypatch.setattr(ace_mod, "QueueHandler", None)
        unit = make_unit()
        assert unit._create_serial_logger() is None


# ── _capture_td1_data_inner ───────────────────────────────────────────────────

class TestCaptureTd1DataInner:
    def test_captures_when_get_td1_succeeds(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.get_td1_data = Recorder(result=True)
        lane = FakeLane("l0")
        lane.dist_hub = 100.0
        lane.loaded_to_hub = True
        lane.td1_bowden_length = 300.0

        ok, msg = unit._capture_td1_data_inner(lane, 0)

        assert ok is True
        assert msg == "TD-1 data captured for l0"
        assert unit.get_td1_data.call_count == 1


# ── _calibrate_td1_inner ──────────────────────────────────────────────────────

class TestCalibrateTd1Inner:
    def test_success_at_hub_writes_config(self):
        ace = _ace()
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit._wait_for_ace_ready = lambda *a, **k: True
        unit._wait_for_feed_complete = lambda *a, **k: True
        unit.get_td1_data = Recorder(result=True)
        unit.afc.function = types.SimpleNamespace(
            check_for_td1_id=lambda i: (True, ""), ConfigRewrite=Recorder())
        lane = FakeLane("l0", hub_obj=None)
        lane.dist_hub = 100.0
        lane.loaded_to_hub = True
        lane.td1_device_id = "td1"
        lane.td1_bowden_length = None
        lane.fullname = "AFC_stepper l0"

        ok, msg, dist = unit._calibrate_td1_inner(lane, 50.0, 1.0)

        assert (ok, dist) == (True, 0.0)
        assert lane.td1_bowden_length == 0.0
        assert unit.afc.function.ConfigRewrite.call_count == 1
        assert unit.afc.save_vars.call_count == 1


# ── on_filament_insert ────────────────────────────────────────────────────────

class TestOnFilamentInsert:
    def test_restores_saved_spool_and_stages(self):
        unit = make_unit(_slot_map={"l0": 0}, _uses_firmware_rfid=True)
        lane = FakeLane("l0")
        lane.spool_id = 5
        lane._afc_staged_spool_id = None
        lane.remember_spool = True

        def clear_values(lz):
            lz.spool_id = None  # simulate the real clear
        unit.afc.spool = types.SimpleNamespace(
            clear_values=clear_values, set_spoolID=Recorder())
        unit._refresh_slot_inventory = Recorder()
        unit.lane_illuminate_spool = Recorder()
        unit.prep_post_load = Recorder()

        unit.on_filament_insert(lane)

        assert unit.afc.spool.set_spoolID.calls == [((lane, 5), {})]
        assert unit.afc.save_vars.call_count == 1
        assert unit.prep_post_load.call_count == 1
        # base handler + ACE post_insert event both fire.
        assert unit.printer.send_event.call_count == 2

    def test_prep_post_load_error_logged(self):
        unit = make_unit(_slot_map={"l0": 0}, _uses_firmware_rfid=False)
        lane = FakeLane("l0")
        lane.spool_id = None
        lane._afc_staged_spool_id = None
        lane.remember_spool = False
        unit.afc.spool = types.SimpleNamespace(
            clear_values=Recorder(), set_spoolID=Recorder())
        unit.lane_illuminate_spool = Recorder()
        unit.prep_post_load = Recorder(raises=RuntimeError("stage boom"))

        unit.on_filament_insert(lane)

        assert unit.logger.lines["error"] == [
            "ACE on_filament_insert: prep_post_load error for l0: stage boom"]


# ══════════════════════════════════════════════════════════════════════════════
# Small helpers, sequence wrappers and remaining branch/exception paths
# ══════════════════════════════════════════════════════════════════════════════

class TestUseFeedAssist:
    def test_per_lane_disabled_returns_false(self):
        unit = make_unit()
        lane = FakeLane("l0")
        lane.use_feed_assist = False
        assert unit._use_feed_assist(lane) is False

    def test_default_used_when_lane_unset_and_active(self):
        unit = make_unit(_default_feed_assist=True)
        unit.afc.current = None  # active tool -> _lane_is_active_tool True
        lane = FakeLane("l0", extruder_obj=None)
        lane.use_feed_assist = None
        assert unit._use_feed_assist(lane) is True

    def test_enabled_but_not_active_tool_returns_false(self):
        unit = make_unit()
        unit.afc.current = "other"  # a different lane is the active tool
        lane = FakeLane("l0", extruder_obj=None)
        lane.use_feed_assist = True
        assert unit._use_feed_assist(lane) is False


class TestLaneIsActiveTool:
    def test_toolchanger_on_shuttle_true(self):
        unit = make_unit()
        ext = types.SimpleNamespace(tc_unit_name="tc", on_shuttle=lambda: True)
        lane = FakeLane("l0", extruder_obj=ext)
        assert unit._lane_is_active_tool(lane) is True

    def test_toolchanger_off_shuttle_false(self):
        unit = make_unit()
        ext = types.SimpleNamespace(tc_unit_name="tc", on_shuttle=lambda: False)
        lane = FakeLane("l0", extruder_obj=ext)
        assert unit._lane_is_active_tool(lane) is False

    def test_toolchanger_on_shuttle_not_callable_true(self):
        unit = make_unit()
        ext = types.SimpleNamespace(tc_unit_name="tc", on_shuttle=None)
        lane = FakeLane("l0", extruder_obj=ext)
        assert unit._lane_is_active_tool(lane) is True

    def test_combined_matches_current(self):
        unit = make_unit()
        unit.afc.current = "l0"
        lane = FakeLane("l0", extruder_obj=None)
        assert unit._lane_is_active_tool(lane) is True

    def test_combined_other_current_false(self):
        unit = make_unit()
        unit.afc.current = "other"
        lane = FakeLane("l0", extruder_obj=None)
        assert unit._lane_is_active_tool(lane) is False


class TestPrepLoad:
    def test_is_noop(self):
        unit = make_unit()
        assert unit.prep_load(FakeLane("l0")) is None


class TestAceLoadSequence:
    def test_wraps_inner_and_clears_flag(self):
        unit = make_unit()
        unit._ace_load_inner = Recorder(result=True)
        assert unit._ace_load_sequence(FakeLane("l0"), None) is True
        assert unit._operation_active is False
        assert unit._prev_states_stale is True


class TestAceUnloadSequence:
    def test_wraps_inner_and_clears_flag(self):
        unit = make_unit()
        unit._ace_unload_inner = Recorder(result=True)
        assert unit._ace_unload_sequence(FakeLane("l0"), None) is True
        assert unit._operation_active is False
        assert unit._prev_states_stale is True


class TestAceUnloadInnerException:
    def test_unwind_failure_returns_false(self):
        ace = _ace()
        ace.unwind_filament = Recorder(raises=RuntimeError("wind boom"))
        unit = make_unit(_ace=ace, _slot_map={"l0": 0})
        unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
        unit.afc.function = types.SimpleNamespace(in_print=lambda: False)
        unit.afc.move_e_pos = Recorder()
        unit._stop_feed_assist = Recorder()
        unit._wait_for_ace_ready = lambda *a, **k: True
        hub = types.SimpleNamespace(afc_unload_bowden_length=900.0)
        lane = FakeLane("l0", hub_obj=hub, tool_loaded=True)
        ext = types.SimpleNamespace(tool_stn_unload=0, tool_unload_speed=50)

        assert unit._ace_unload_inner(lane, ext) is False
        msg = unit.afc.error.handle_lane_failure.last_args[1]
        assert "ACE unwind failed for l0: wind boom" == msg


class TestGetStatusAgeException:
    def test_monotonic_exception_leaves_age_none(self):
        unit = make_unit()
        unit.lanes = {}
        unit._hw_status_time = 50.0
        unit._ace = FakeAce(connected=True)

        def boom():
            raise RuntimeError("clock")
        unit.afc.reactor = types.SimpleNamespace(monotonic=boom)

        st = unit.get_status()

        assert st["ace_status_age"] is None
        assert st["ace_status_stale"] is False


class TestQuickReconnectCallbackFailure:
    def test_failure_falls_back_to_backoff_reconnect(self, monkeypatch):
        conn = _conn()
        monkeypatch.setitem(sys.modules, "serial", None)  # connect fails
        conn._quick_reconnect()
        cb, _when = conn._reactor.timers[-1]
        assert cb(conn._reactor.monotonic()) == conn._reactor.NEVER
        assert conn._connected is False
        assert any("quick reconnect failed" in m
                   for m in conn._logger.lines["warning"])
