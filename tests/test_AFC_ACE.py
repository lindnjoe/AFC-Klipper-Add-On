"""
Tests for the ACE V1 unit, extras/AFC_ACE.py.

Everything this suite knows about an afcACE lives here: the module's logo/CRC
helpers, RFID slot handling and the shared-reader ambiguity guard, feed assist,
slot-state sync, the stage-probe feed, the startup inventory sweep, the
diagnostic gcode handlers, current-action surfacing, sensor and temperature
helpers, the overlapping unload retract, and eject's reload suppression.

Consolidated from twelve files. Where two of them defined a module-level helper
under the same name, the helper carries its old file's tag -- _make_unit_assist,
_make_unit_slot_sync and so on -- because those were different implementations
that happened to share a name, and merging them would have silently shadowed
one with the other. Section banners below name the file each block came from.
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
import sys  # noqa: E402
from extras.AFC_ACE import (  # noqa: E402
    ACEConnection,
    ACESerialError,
    HEARTBEAT_INTERVAL,
    crc16_ccitt_reflected as _crc,
    load_config_prefix,
)
from extras.AFC_ACE import afcACE
from tests.ace_helpers import (
    FakeAce,
    FakeAFC,
    FakeExtruderObj,
    FakeGcmd,
    FakeLane,
    FakeLogger,
    FakeToolheadPrinter,
    Recorder,
)
import types as _types
from extras.AFC_ACE import MODE_DIRECT
from tests.ace_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeHub,
    FakeLane,
    FakeLogger,
    Recorder,
)
from tests.ace_helpers import FakeAce, FakeLane, FakeLogger, Recorder
from tests.ace_helpers import (
    FakeAce2,
    FakeGcmd,
    Recorder,
)
from tests.ace_helpers import FakeLogger, FakeGcmd, FakeAce, Recorder
from extras.AFC_ACE2 import _decode_status, pb_uint32  # noqa: E402
from extras.AFC_ACE2 import afcACE2
from tests.ace_helpers import (
    FakeAce,
    FakeHub,
    FakeLane,
    FakeLogger,
    Recorder,
)
from extras.AFC_ACE import afcACE, ACEConnection
from extras.AFC_ACE2 import ACE2Connection
from tests.ace_helpers import FakeLogger, Recorder
from tests.ace_helpers import FakeLogger
from tests.ace_helpers import (
    FakeAce, FakeError, FakeFunction, FakeLane, FakeHub, FakeLogger, Recorder,
)
import contextlib
from tests.ace_helpers import FakeAce, FakeHub, FakeLane, FakeLogger, Recorder


# ── Broad branch-coverage tests for extras/AFC_ACE.py (afcACE V1 unit) covering the ───
#
# was tests/test_AFC_ACE_coverage.py
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


# ── Unit tests for afcACE feed-assist management in extras/AFC_ACE.py ─────────
#
# was tests/test_AFC_ACE_assist.py
# ── Helpers ───────────────────────────────────────────────────────────────────

def _lane_assist(name, ext_section="extruder", ext_physical=None, tool_loaded=True,
          lane_loaded=None):
    ext = FakeExtruderObj(name=ext_section, th_extruder_name=ext_physical,
                          lane_loaded=lane_loaded)
    return FakeLane(name, extruder_obj=ext, tool_loaded=tool_loaded)


def _make_unit_assist(lanes=(), slot_map=None, active_extruder="extruder"):
    unit = afcACE.__new__(afcACE)
    unit.name = "ACE_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.lanes = {}
    for lane in lanes:
        unit.lanes[lane.name] = lane
        unit.afc.lanes[lane.name] = lane
    unit._slot_map = dict(slot_map or {})
    unit._feed_assist_active = set()
    unit._assist_suppressed = set()
    unit._assist_watchdog = True
    unit._ace = None
    unit.printer = FakeToolheadPrinter(active_extruder=active_extruder)
    return unit


# ── _active_assist_lane ───────────────────────────────────────────────────────

def test_active_assist_lane_matches_section_name():
    lane = _lane_assist("lane0", ext_section="extruder", lane_loaded="lane0")
    unit = _make_unit_assist([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_matches_physical_name():
    """[AFC_extruder e0] with extruder_name: extruder — toolhead reports the
    PHYSICAL name; the lane must still resolve (SET_LANE_LOADED fix)."""
    lane = _lane_assist("lane0", ext_section="e0", ext_physical="extruder",
                 lane_loaded="lane0")
    unit = _make_unit_assist([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_toolhead_lookup_failure_returns_none():
    lane = _lane_assist("lane0", lane_loaded="lane0")
    unit = _make_unit_assist([lane], active_extruder=None)  # lookup raises
    assert unit._active_assist_lane() is None


def test_active_assist_lane_skips_lane_without_extruder():
    lane = _lane_assist("lane0", lane_loaded="lane0")
    lane.extruder_obj = None
    unit = _make_unit_assist([lane], active_extruder="extruder")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_skips_unloaded_lanes():
    lane = _lane_assist("lane0", tool_loaded=False)
    unit = _make_unit_assist([lane], active_extruder="extruder")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_ignores_other_extruders():
    lane = _lane_assist("lane0", ext_section="e0", ext_physical="extruder",
                 lane_loaded="lane0")
    unit = _make_unit_assist([lane], active_extruder="extruder4")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_fallback_when_lane_loaded_lags():
    """extruder.lane_loaded lags tool_loaded at print start — the unique
    loaded lane is still the assist target via the fallback candidate."""
    lane = _lane_assist("lane0", lane_loaded=None)
    unit = _make_unit_assist([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_exact_match_beats_candidate():
    """A lane the extruder RECORDS as loaded wins over an earlier
    tool_loaded-only candidate."""
    ext = FakeExtruderObj(name="extruder", lane_loaded="lane1")
    lane0 = FakeLane("lane0", extruder_obj=ext, tool_loaded=True)
    lane1 = FakeLane("lane1", extruder_obj=ext, tool_loaded=True)
    unit = _make_unit_assist([lane0, lane1], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane1"


# ── _maybe_assist_watchdog ────────────────────────────────────────────────────

def _watchdog_unit(**kw):
    lane = _lane_assist("lane0", lane_loaded="lane0")
    unit = _make_unit_assist([lane], slot_map={"lane0": 0},
                      active_extruder="extruder", **kw)
    unit._use_feed_assist = Recorder(result=True)
    return unit


def test_watchdog_schedules_reconcile_when_assist_missing():
    unit = _watchdog_unit()
    unit._maybe_assist_watchdog()
    assert unit.afc.reactor.register_callback.call_count == 1
    assert "enabling feed assist" in unit.logger.lines["info"][0]


def test_watchdog_fires_when_wrong_slot_assisting():
    unit = _watchdog_unit()
    unit._feed_assist_active = {3}
    unit._maybe_assist_watchdog()
    assert unit.afc.reactor.register_callback.call_count == 1


def test_watchdog_noop_when_assist_already_correct():
    unit = _watchdog_unit()
    unit._feed_assist_active = {0}
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_respects_manual_suppression():
    unit = _watchdog_unit()
    unit._assist_suppressed = {0}
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_disabled_by_config():
    unit = _watchdog_unit()
    unit._assist_watchdog = False
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_noop_without_active_lane():
    unit = _make_unit_assist(active_extruder="extruder")  # no lanes at all
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_noop_when_assist_disabled_for_lane():
    unit = _watchdog_unit()
    unit._use_feed_assist = Recorder(result=False)
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


# ── cmd_ACE_FEED_ASSIST ───────────────────────────────────────────────────────

def test_feed_assist_cmd_requires_enable():
    unit = _make_unit_assist()
    with pytest.raises(RuntimeError, match="ENABLE is required"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(LANE="lane0"))


def test_feed_assist_cmd_requires_lane_or_slot():
    unit = _make_unit_assist()
    with pytest.raises(RuntimeError, match="LANE or SLOT"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0))


def test_feed_assist_cmd_unknown_lane():
    unit = _make_unit_assist()
    with pytest.raises(RuntimeError, match="Unknown lane"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0, LANE="nope"))


def test_feed_assist_stop_suppresses_and_stops_tracked_slot():
    unit = _make_unit_assist(slot_map={"lane0": 2})
    unit._feed_assist_active = {2}
    unit._stop_feed_assist = Recorder()

    gcmd = FakeGcmd(ENABLE=0, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert unit._assist_suppressed == {2}
    assert unit._stop_feed_assist.calls == [((2,), {})]
    assert "suppressed" in gcmd.responses[0]


def test_feed_assist_stop_sends_firmware_stop_on_tracking_drift():
    """Slot not tracked as assisting, but firmware might be — the manual stop
    must still reach the hardware."""
    unit = _make_unit_assist(slot_map={"lane0": 2})
    unit._ace = FakeAce(connected=True)
    unit._stop_feed_assist = Recorder()

    unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0, LANE="lane0"))

    assert unit._assist_suppressed == {2}
    assert not unit._stop_feed_assist.called            # not tracked
    assert unit._ace.stop_feed_assist.calls == [((2,), {})]  # raw stop


def test_feed_assist_stop_untracked_without_hardware_only_suppresses():
    unit = _make_unit_assist(slot_map={"lane0": 2})  # _ace is None
    unit._stop_feed_assist = Recorder()

    gcmd = FakeGcmd(ENABLE=0, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert unit._assist_suppressed == {2}
    assert not unit._stop_feed_assist.called
    assert len(gcmd.responses) == 1


def test_feed_assist_start_stops_other_slots_first():
    """The ACE can only feed-assist one slot at a time — a manual start must
    stop the other assisting slot(s) before starting (or the ACE refuses
    with error_2)."""
    unit = _make_unit_assist(slot_map={"lane0": 0, "lane1": 1})
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist = lambda s: calls.append(("stop", s))
    unit._start_feed_assist = lambda s, explicit=False: calls.append(("start", s))

    gcmd = FakeGcmd(ENABLE=1, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert calls == [("stop", 1), ("start", 0)]
    assert "started" in gcmd.responses[0]


def test_feed_assist_start_accepts_slot_param():
    unit = _make_unit_assist()
    unit._start_feed_assist = Recorder()
    unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=1, SLOT=3))
    # A user ENABLE=1 is an EXPLICIT start (clears any manual suppression).
    assert unit._start_feed_assist.calls == [((3,), {"explicit": True})]


# ── _start_feed_assist (real method) early-outs ───────────────────────────────

def test_explicit_start_clears_suppression():
    """An EXPLICIT start ends the manual suppression — with no hardware
    connected the method early-outs right after the discard."""
    unit = _make_unit_assist()
    unit._assist_suppressed = {2}
    unit._start_feed_assist(2, explicit=True)
    assert unit._assist_suppressed == set()
    assert unit._feed_assist_active == set()  # no hardware -> not tracked


def test_non_explicit_start_respects_suppression():
    """A non-explicit (watchdog/restore/load) start must NOT clear a manual
    suppression, and bails without enabling assist — so a reconcile queued just
    before ACE_FEED_ASSIST ENABLE=0 can't re-enable it behind the user's back."""
    unit = _make_unit_assist()
    unit._ace = FakeAce(connected=True)
    unit._assist_suppressed = {2}
    unit._start_feed_assist(2)                # non-explicit
    assert unit._assist_suppressed == {2}     # suppression preserved
    assert not unit._ace.start_feed_assist.called
    assert unit._feed_assist_active == set()


def test_start_already_active_is_noop():
    unit = _make_unit_assist()
    unit._ace = FakeAce(connected=True)
    unit._feed_assist_active = {2}

    unit._start_feed_assist(2)

    assert not unit._ace.start_feed_assist.called
    assert unit._feed_assist_active == {2}


def test_start_feed_assist_stops_other_active_slot_first():
    # The ACE assists ONE slot at a time. Starting slot 0 (e.g. loading a lane)
    # must stop the previously-active slot 2 first, so we never leave two assists
    # running — the single-assist invariant the live toolchange test caught.
    unit = _make_unit_assist()
    unit._ace = FakeAce(connected=True)
    unit._wait_for_ace_ready = lambda *a, **k: True
    unit._feed_assist_active = {2}
    stopped = []
    unit._stop_feed_assist = lambda s: (stopped.append(s),
                                        unit._feed_assist_active.discard(s))

    unit._start_feed_assist(0)

    assert stopped == [2]                      # stopped the other slot first
    assert unit._ace.start_feed_assist.called  # then started the target
    assert unit._feed_assist_active == {0}     # exactly one active


def test_start_feed_assist_clears_stale_second_assist_when_already_active():
    # Even if the target slot is already tracked active, a stray second active
    # slot must be stopped (defensive: never more than one).
    unit = _make_unit_assist()
    unit._ace = FakeAce(connected=True)
    unit._wait_for_ace_ready = lambda *a, **k: True
    unit._feed_assist_active = {0, 2}
    unit._stop_feed_assist = lambda s: unit._feed_assist_active.discard(s)

    unit._start_feed_assist(0)

    assert unit._feed_assist_active == {0}


# ── _reconcile_feed_assist ────────────────────────────────────────────────────

def _reconcile_unit():
    lane = _lane_assist("lane0")
    unit = _make_unit_assist([lane], slot_map={"lane0": 0, "lane1": 1})
    unit._use_feed_assist = Recorder(result=True)
    unit._toolhead_sensor_triggered = Recorder(result=True)
    unit._stop_feed_assist = Recorder()
    unit._start_feed_assist = Recorder()
    return unit


def test_reconcile_stops_other_slots_then_starts_target():
    unit = _reconcile_unit()
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist = lambda s: calls.append(("stop", s))
    unit._start_feed_assist = lambda s: calls.append(("start", s))

    unit._reconcile_feed_assist("lane0")

    assert calls == [("stop", 1), ("start", 0)]


def test_reconcile_does_not_start_before_filament_at_toolhead():
    unit = _reconcile_unit()
    unit._toolhead_sensor_triggered = Recorder(result=False)
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_sensor_exception_treated_as_not_at_toolhead():
    unit = _reconcile_unit()
    unit._toolhead_sensor_triggered = Recorder(raises=RuntimeError("boom"))
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_respects_suppression():
    unit = _reconcile_unit()
    unit._assist_suppressed = {0}
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_already_active_does_not_restart():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called
    assert not unit._stop_feed_assist.called  # target slot is never stopped


def test_reconcile_assist_disabled_for_lane():
    unit = _reconcile_unit()
    unit._use_feed_assist = Recorder(result=False)
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_lane_on_other_unit_stops_ours():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}
    unit.afc.lanes["other_lane"] = _lane_assist("other_lane")

    unit._reconcile_feed_assist("other_lane")  # not in our _slot_map

    assert unit._stop_feed_assist.calls == [((0,), {})]
    assert not unit._start_feed_assist.called


def test_reconcile_unresolvable_name_leaves_assist_untouched():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}

    unit._reconcile_feed_assist("ghost")  # not our lane, not any afc lane

    assert not unit._stop_feed_assist.called
    assert not unit._start_feed_assist.called
    assert unit._feed_assist_active == {0}


# ── Retract distance math ─────────────────────────────────────────────────────

class _Hub:
    def __init__(self, unload=None, bowden=None):
        if unload is not None:
            self.afc_unload_bowden_length = unload
        if bowden is not None:
            self.afc_bowden_length = bowden


def _distance_unit():
    unit = _make_unit_assist()
    unit.eject_buffer = 475.0
    return unit


def test_eject_length_staged_at_hub():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    assert unit._get_eject_length(lane) == 300.0 + 475.0


def test_eject_length_tool_loaded_uses_full_unload_path():
    unit = _distance_unit()
    lane = FakeLane("lane0", tool_loaded=True, hub_obj=None)
    lane.dist_hub = 300.0
    lane.hub_obj = _Hub(unload=1000.0)
    assert unit._get_eject_length(lane) == 300.0 + 1000.0


def test_unload_length_falls_back_to_bowden_length():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    lane.hub_obj = _Hub(bowden=900.0)  # no afc_unload_bowden_length
    assert unit._get_unload_length(lane) == 300.0 + 900.0


def test_unload_length_without_hub():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    lane.hub_obj = None
    assert unit._get_unload_length(lane) == 300.0


# ── check_runout ──────────────────────────────────────────────────────────────

def test_check_runout_true_while_printing():
    unit = _make_unit_assist()
    unit.afc.function.printing = True
    assert unit.check_runout(FakeLane("lane0")) is True


def test_check_runout_false_when_idle():
    unit = _make_unit_assist()
    unit.afc.function.printing = False
    assert unit.check_runout(FakeLane("lane0")) is False


def test_check_runout_false_on_exception():
    unit = _make_unit_assist()
    unit.afc.function.raise_on_is_printing = RuntimeError("boom")
    assert unit.check_runout(FakeLane("lane0")) is False


# ── _start_feed_assist error handling ─────────────────────────────────────────

def test_start_feed_assist_error_2_logged_debug_not_error():
    """error_2 = the ACE momentarily refusing assist (concurrent-assist limit /
    slot state settling); the watchdog retries, so it's debug, not an error."""
    unit = _make_unit_assist(slot_map={"lane0": 2})
    unit._ace = FakeAce(connected=True)
    unit._wait_for_ace_ready = Recorder()

    def _refuse(slot):
        raise Exception("ACE2 command 'start_feed_assist' failed: "
                        "code=2, msg=error_2")
    unit._ace.start_feed_assist = _refuse

    unit._start_feed_assist(2)

    assert 2 not in unit._feed_assist_active
    assert unit.logger.lines["error"] == []
    assert any("refused (error_2" in m for m in unit.logger.lines["debug"])


def test_start_feed_assist_unexpected_error_stays_error():
    unit = _make_unit_assist(slot_map={"lane0": 2})
    unit._ace = FakeAce(connected=True)
    unit._wait_for_ace_ready = Recorder()

    def _boom(slot):
        raise Exception("something genuinely unexpected")
    unit._ace.start_feed_assist = _boom

    unit._start_feed_assist(2)

    assert any("Failed to start feed assist" in m
               for m in unit.logger.lines["error"])


# ── slot map build/validation (D4) and _get_slot fallback (D2) ────────────────

def _idx_lane(name, index):
    lane = _lane_assist(name)
    lane.index = index
    return lane


def test_build_slot_map_maps_index_to_zero_based_slot():
    unit = _make_unit_assist(lanes=[_idx_lane("lane1", 1), _idx_lane("lane2", 3)])
    assert unit._build_slot_map() == {"lane1": 0, "lane2": 2}


def test_build_slot_map_rejects_duplicate_index():
    unit = _make_unit_assist(lanes=[_idx_lane("a", 1), _idx_lane("b", 1)])
    with pytest.raises(Exception):
        unit._build_slot_map()


def test_build_slot_map_rejects_out_of_range_index():
    unit = _make_unit_assist(lanes=[_idx_lane("a", afcACE.SLOTS_PER_UNIT + 1)])
    with pytest.raises(Exception):
        unit._build_slot_map()
    unit0 = _make_unit_assist(lanes=[_idx_lane("z", 0)])   # 0 is not a valid 1-based idx
    with pytest.raises(Exception):
        unit0._build_slot_map()


def test_get_slot_returns_mapped_slot():
    unit = _make_unit_assist(slot_map={"lane1": 2})
    assert unit._get_slot("lane1") == 2


def test_get_slot_unknown_lane_defaults_zero_and_warns():
    unit = _make_unit_assist(slot_map={"lane1": 2})
    assert unit._get_slot("ghost") == 0                 # safe fallback
    assert any("not in this unit's slot map" in m
               for m in unit.logger.lines["warning"])
    # warned once per lane — a second lookup doesn't re-log
    unit.logger.lines["warning"].clear()
    assert unit._get_slot("ghost") == 0
    assert unit.logger.lines["warning"] == []


# ── load feed stops other assists first (all modes) + _log_delta guard ────────



def test_ace_load_stops_other_assist_before_feed_in_direct_mode():
    # The ACE can't feed one slot while another assists. _ace_load_inner must
    # stop other active-assist slots before feeding in ALL modes (was gated to
    # combined mode only, so a toolchanger/direct unit fed into a live assist and
    # the feed timed out — the live incident). We bail at the pre-feed sensor
    # check right after the stop, so no real feed is needed.
    lane = _lane_assist("lane0", tool_loaded=False)
    lane.loaded_to_hub = False
    lane.buffer_obj = None
    unit = _make_unit_assist([lane], slot_map={"lane0": 0, "lane2": 2})
    unit.mode = MODE_DIRECT                       # NOT combined
    unit._ace = FakeAce(connected=True)
    unit._hub_load_suppressed = set()
    unit._feed_assist_active = {2}                # another slot still assisting
    stopped = []
    unit._stop_feed_assist = lambda s: (stopped.append(s),
                                        unit._feed_assist_active.discard(s))
    unit._get_bowden_length = lambda l: 100.0
    unit._set_hub_state = lambda l, s: None
    unit._toolhead_sensor_triggered = lambda l: True   # bail right after the stop
    unit.afc.function = _types.SimpleNamespace(in_print=lambda: False)
    unit.afc.error = _types.SimpleNamespace(handle_lane_failure=Recorder())

    ok = unit._ace_load_inner(lane, _types.SimpleNamespace())

    assert ok is False                            # bailed at the pre-feed check
    assert stopped == [2]                          # but stopped slot 2 FIRST


def test_log_delta_starts_clock_when_unstarted():
    unit = _make_unit_assist()

    class DT:
        start_time = None
        started = False
        logged = None
        def set_start_time(self):
            self.start_time = "now"; self.started = True
        def log_with_time(self, m, debug=True):
            self.logged = m
    dt = DT()
    unit.afc = _types.SimpleNamespace(afcDeltaTime=dt)
    unit._log_delta("hello")
    assert dt.started is True and dt.logged == "hello"


def test_log_delta_swallows_upstream_error():
    unit = _make_unit_assist()

    class DT:
        start_time = "x"
        def log_with_time(self, m, debug=True):
            raise TypeError("unsupported operand type(s) for -: datetime vs None")
    unit.afc = _types.SimpleNamespace(afcDeltaTime=DT())
    unit._log_delta("hello")   # must NOT raise


# ── Unit tests for afcACE._sync_slot_states in extras/AFC_ACE.py ──────────────
#
# was tests/test_AFC_ACE_slot_sync.py
# ── Helpers ───────────────────────────────────────────────────────────────────

def _lane_slot_sync(name="lane0", prev_ready=None, tool_loaded=False,
          status=AFCLaneState.NONE, lane_loaded=None, hub=None):
    lane = FakeLane(name, extruder_obj=FakeExtruderObj("extruder",
                                                       lane_loaded=lane_loaded),
                    hub_obj=hub, tool_loaded=tool_loaded, status=status)
    lane.prep_state = bool(prev_ready)
    lane.loaded_to_hub = bool(prev_ready)
    lane.load_to_hub = True
    return lane


def _make_unit_slot_sync(lane, prev_ready=None, preloads=False, stale=False,
               current=None):
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.current = current
    unit.lanes = {lane.name: lane}
    unit._slot_map = {lane.name: 0}
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    unit._prev_slot_states = {} if prev_ready is None else {lane.name: prev_ready}
    unit._prev_states_stale = stale
    unit._hub_load_suppressed = set()
    unit._preloads_to_hub_on_insert = preloads
    unit._use_feed_assist = Recorder(result=False)
    unit._start_feed_assist = Recorder()
    unit.lane_tool_loaded = Recorder()
    unit.lane_tool_loaded_idle = Recorder()
    # afc collaborators the TOOLED-restore path touches
    unit.afc.spool = Recorder()
    unit.afc.spool.set_active_spool = Recorder()
    lane.spool_id = 42
    return unit


def _hw(slot_status, unit_status="ready"):
    return {"status": unit_status, "slots": [{"status": slot_status}]}


# ── Inventory + malformed input ───────────────────────────────────────────────

def test_inventory_status_copied_per_slot():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready",
                            "slots": [{"status": "ready"}, {"status": "empty"}]})

    assert unit._slot_inventory[0]["status"] == "ready"
    assert unit._slot_inventory[1]["status"] == "empty"


def test_lane_beyond_reported_slots_skipped():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready", "slots": []})

    assert lane.prep_state is True            # untouched
    assert not lane.handle_load_runout.called


def test_malformed_slot_entry_skipped():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready", "slots": ["garbage"]})

    assert lane.prep_state is True
    assert not lane.handle_load_runout.called


# ── Virtual hub refresh ───────────────────────────────────────────────────────

def test_virtual_hub_refreshed_from_tool_loaded():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 hub=FakeHub(virtual=True))
    unit = _make_unit_slot_sync(lane, prev_ready=True)
    lane._load_state = False  # stale

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_state is True  # derived from tool_loaded every poll


def test_real_hub_load_state_untouched():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 hub=None)
    unit = _make_unit_slot_sync(lane, prev_ready=True)
    lane._load_state = "sentinel"

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_state == "sentinel"


# ── Transient statuses ────────────────────────────────────────────────────────

def test_transient_status_leaves_everything_alone():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    for status in ("shifting", "feeding", "unwinding"):
        unit._sync_slot_states(_hw(status))
        assert lane.prep_state is True                  # untouched
        assert lane.loaded_to_hub is True               # untouched
        assert unit._prev_slot_states["lane0"] is True  # snapshot untouched
        assert not lane.handle_load_runout.called


# ── Removal / runout ──────────────────────────────────────────────────────────

def test_ready_to_empty_fires_runout_and_clears_staging():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty"))

    assert lane.prep_state is False
    assert lane.loaded_to_hub is False
    assert lane.handle_load_runout.call_count == 1
    eventtime, state = lane.handle_load_runout.last_args
    assert state is False
    assert unit._prev_slot_states["lane0"] is False


def test_unit_busy_suppresses_removal():
    """ACE2 flickers slots 'empty' while its own cycles run — a unit-level
    'busy' must not fire a false runout or drop the staged state."""
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty", unit_status="busy"))

    assert not lane.handle_load_runout.called
    assert lane.loaded_to_hub is True         # staged state survives
    assert lane.prep_state is False           # live prep still tracks the slot


def test_stale_prev_states_resync_without_events():
    lane = _lane_slot_sync(prev_ready=True)
    unit = _make_unit_slot_sync(lane, prev_ready=True, stale=True)

    unit._sync_slot_states(_hw("empty"))

    assert not lane.handle_load_runout.called
    assert unit._prev_slot_states["lane0"] is False  # re-synced
    assert unit._prev_states_stale is False          # consumed
    assert lane.loaded_to_hub is True                # not cleared on resync


def test_empty_stays_empty_no_event():
    lane = _lane_slot_sync(prev_ready=False)
    unit = _make_unit_slot_sync(lane, prev_ready=False)

    unit._sync_slot_states(_hw("empty"))

    assert not lane.handle_load_runout.called
    assert unit._prev_slot_states["lane0"] is False


# ── Insert / staging ──────────────────────────────────────────────────────────

def test_fresh_insert_v1_preloads_to_hub():
    """V1 ACE preloads filament to the hub on insert: empty -> ready stages
    the lane (honoring load_to_hub)."""
    lane = _lane_slot_sync(prev_ready=False)
    unit = _make_unit_slot_sync(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True


def test_fresh_insert_respects_load_to_hub_off():
    lane = _lane_slot_sync(prev_ready=False)
    lane.load_to_hub = False
    unit = _make_unit_slot_sync(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is False


def test_fresh_insert_ace2_does_not_preload():
    """ACE2 stages via prep_post_load's real dist_hub feed instead."""
    lane = _lane_slot_sync(prev_ready=False)
    unit = _make_unit_slot_sync(lane, prev_ready=False, preloads=False)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is False


def test_tool_loaded_lane_never_preloaded():
    lane = _lane_slot_sync(prev_ready=False, tool_loaded=True, status=AFCLaneState.TOOLED)
    lane.loaded_to_hub = False
    unit = _make_unit_slot_sync(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.loaded_to_hub is False


# ── Insert path for an un-tooled NONE lane ────────────────────────────────────

def test_ready_untooled_lane_fires_insert_path():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.handle_load_runout.call_count == 1
    _, state = lane.handle_load_runout.last_args
    assert state is True
    assert lane._load_suppressed is False


def test_ready_untooled_suppressed_marks_lane():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    unit = _make_unit_slot_sync(lane, prev_ready=True)
    unit._hub_load_suppressed = {"lane0"}

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_suppressed is True
    assert lane.handle_load_runout.call_count == 1


def test_prep_not_done_skips_insert_path():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    lane._afc_prep_done = False
    unit = _make_unit_slot_sync(lane, prev_ready=True)

    unit._sync_slot_states(_hw("ready"))

    assert not lane.handle_load_runout.called


# ── TOOLED restore ────────────────────────────────────────────────────────────

def test_ready_tooled_current_lane_restores_full_state():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=True, status=AFCLaneState.NONE,
                 lane_loaded="lane0")
    unit = _make_unit_slot_sync(lane, prev_ready=True, current="lane0")
    unit._use_feed_assist = Recorder(result=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.loaded_to_hub is True
    assert lane.sync_to_extruder.call_count == 1
    assert lane.status == AFCLaneState.TOOLED
    assert unit.afc.spool.set_active_spool.calls == [((42,), {})]
    assert unit.lane_tool_loaded.calls == [((lane,), {})]
    assert not unit.lane_tool_loaded_idle.called
    assert lane.enable_buffer.call_count == 1
    assert unit._start_feed_assist.calls == [((0,), {})]
    assert not lane.handle_load_runout.called


def test_ready_tooled_idle_lane_restores_idle_state():
    """A tool-loaded lane on a NOT-current tool restores as idle-tooled (and
    is marked TOOLED so the restore doesn't re-fire every poll)."""
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=True, status=AFCLaneState.NONE,
                 lane_loaded="lane0")
    unit = _make_unit_slot_sync(lane, prev_ready=True, current="other_lane")

    unit._sync_slot_states(_hw("ready"))

    assert unit.lane_tool_loaded_idle.calls == [((lane,), {})]
    assert not unit.lane_tool_loaded.called
    assert not unit.afc.spool.set_active_spool.called
    assert lane.status == AFCLaneState.TOOLED
    assert lane.enable_buffer.call_count == 1


def test_restore_does_not_refire_once_tooled():
    lane = _lane_slot_sync(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 lane_loaded="lane0")
    unit = _make_unit_slot_sync(lane, prev_ready=True, current="lane0")

    unit._sync_slot_states(_hw("ready"))

    assert not lane.sync_to_extruder.called
    assert not unit.lane_tool_loaded.called


# ── Tests for the RFID stage-probe feed in extras/AFC_ACE.py (_feed_to_hub_probing) ───
#
# was tests/test_AFC_ACE_stage_probe.py
def _make_unit_stage_probe(feeds, unwinds, handlers):
    unit = afcACE.__new__(afcACE)
    ace = FakeAce(connected=True)
    ace.feed_filament = lambda slot, d, sp: feeds.append(round(d, 3))
    ace.unwind_filament = lambda slot, d, sp: unwinds.append(round(d, 3))
    unit._ace = ace
    unit.logger = FakeLogger()
    unit.feed_speed = 50.0
    unit.retract_speed = 50.0
    unit.prep_ready_timeout = 5.0
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._slot_reports_empty = lambda slot: False       # present unless overridden

    class _Printer:
        def send_event(self, name, *a):
            h = handlers.get(name)
            if h is not None:
                h(*a)
    unit.printer = _Printer()
    return unit


def _handlers(fed=0.0, initial=100.0, done=False, removed=False):
    """Stage-read listener: the scan (run inside begin) reports how far it fed
    (``fed``), the mandatory ``initial`` load, whether it read a tag (``done``)
    and whether the spool was pulled mid-scan (``removed``)."""
    def begin(lane, ctx):
        ctx["active"] = True
        ctx["initial"] = initial
        ctx["fed"] = fed
        if done:
            ctx["done"] = True
        if removed:
            ctx["removed"] = True

    return {
        "afc_ace:stage_probe_begin": begin,
        "afc_ace:stage_probe_end": lambda lane, ctx: None,
    }


def test_no_listener_is_plain_dist_hub_feed():
    feeds, unwinds = [], []
    unit = _make_unit_stage_probe(feeds, unwinds, {})              # no listener -> fed=0
    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [200.0]                             # one move, dist_hub only
    assert unwinds == []


def test_inactive_with_initial_is_one_plain_feed():
    # disable_rfid path: the listener requests the initial load but the scan
    # fed nothing -> feed initial + dist_hub in ONE move.
    feeds, unwinds = [], []

    def begin(lane, ctx):
        ctx["initial"] = 100.0                         # load set, scan fed nothing

    handlers = {
        "afc_ace:stage_probe_begin": begin,
        "afc_ace:stage_probe_end": lambda lane, ctx: None,
    }
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)
    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [300.0]                             # initial 100 + dist_hub 200
    assert unwinds == []


def test_read_during_scan_feeds_remainder_in_one_move():
    # Scan fed 150mm and read the tag -> remainder (100+200 - 150) in one move.
    # The smooth scan doesn't trip the stuck-spool latch, so no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=150.0, initial=100.0, done=True)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [150.0]                             # 300 total - 150 already fed
    assert unwinds == []                               # no recovery unwind needed


def test_tagless_feeds_remainder_after_full_scan():
    # Tagless spool: scan fed the whole 200mm window, no read -> feed the
    # remainder (300-200) in one move, no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=200.0, initial=100.0, done=False)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [100.0]                             # 300 total - 200 scanned
    assert sum(feeds) + 200.0 == 300.0                 # net = initial + dist_hub
    assert unwinds == []


def test_scan_fed_full_load_needs_no_remainder():
    # The scan already fed the entire initial+dist_hub -> no remainder move.
    feeds, unwinds = [], []
    handlers = _handlers(fed=300.0, initial=100.0, done=True)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == []                                  # nothing left to stage
    assert unwinds == []                               # no recovery unwind needed


def test_dist_hub_zero_still_does_initial_load():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0, done=False)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=0.0)

    assert feeds == [100.0]                             # just the initial load
    assert unwinds == []


def test_completed_stage_returns_true():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0, done=False)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is True
    assert feeds == [300.0]


def test_stage_aborts_when_scan_reports_removed():
    # The scan detected the spool pulled mid-feed (ctx['removed']) -> abort, no
    # remainder feed, no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=120.0, initial=100.0, removed=True)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is False
    assert feeds == []
    assert unwinds == []
    assert any("filament removed during staging" in m
               for m in unit.logger.lines["debug"])


def test_stage_aborts_when_slot_already_empty():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)
    unit._slot_reports_empty = lambda slot: True       # gone before the remainder

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is False
    assert feeds == []                                 # never fed an empty slot
    assert unwinds == []


def test_recovery_unwind_skipped_when_no_read():
    # No read (done False) -> no stuck-spool clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=200.0, initial=100.0, done=False)
    unit = _make_unit_stage_probe(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert unwinds == []


# ── prep_post_load must NOT short-circuit at dist_hub=0 ───────────────────────

def _prep_unit():
    """Minimal unit wired for prep_post_load: it should run the stage probe even
    when dist_hub=0 (the stage scan / initial load are what pull the filament
    into the unit; dist_hub is additive on top)."""
    import contextlib
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit._unit_load_to_hub = None
    unit._ace = FakeAce(connected=True)
    unit._get_slot = lambda name: 3
    unit.prep_ready_timeout = 5.0
    unit._wait_for_ace_ready = Recorder()
    unit._set_hub_state = lambda lane, state: None
    unit._operation = lambda: contextlib.nullcontext()
    unit.afc = types.SimpleNamespace(
        save_vars=lambda: None, td1_present=False)
    return unit


def test_prep_post_load_runs_probe_when_dist_hub_zero():
    unit = _prep_unit()
    calls = []
    unit._feed_to_hub_probing = lambda lane, slot, dist: (
        calls.append((slot, dist)) or True)

    lane = FakeLane("lane3")
    lane.load_to_hub = True
    lane.loaded_to_hub = False
    lane.prep_state = True
    lane.dist_hub = 0.0
    lane.td1_when_loaded = False
    lane.td1_device_id = None

    unit.prep_post_load(lane)

    assert calls == [(3, 0.0)]          # probe ran even with dist_hub=0
    assert lane.loaded_to_hub is True   # staged once the probe returned True


# ── _slot_reports_empty helper ─────────────────────────────────────────────────

class _StatusAce:
    def __init__(self, status=None, raises=False, connected=True, sequence=None):
        self.connected = connected
        self._status = status
        self._raises = raises
        self._sequence = list(sequence) if sequence else None

    def get_status(self, timeout=2.0):
        if self._raises:
            raise RuntimeError("status timeout")
        if self._sequence is not None:
            return self._sequence.pop(0) if self._sequence else self._status
        return self._status


class _NoWaitReactor:
    def monotonic(self):
        return 0.0

    def pause(self, when):
        pass


def _unit_with_ace(ace):
    unit = afcACE.__new__(afcACE)
    unit._ace = ace
    unit.logger = FakeLogger()
    unit.afc = types.SimpleNamespace(reactor=_NoWaitReactor())
    return unit


def _empty(slot="empty"):
    return {"status": "ready", "slots": [{"status": slot}]}


def test_slot_reports_empty_true_when_stably_empty():
    ace = _StatusAce(_empty())                          # empty on every poll
    assert _unit_with_ace(ace)._slot_reports_empty(0) is True


def test_slot_reports_empty_false_on_single_flicker():
    # empty once, then present: a one-poll flicker must NOT abort a live stage.
    ace = _StatusAce(sequence=[_empty(), _empty("ready")])
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_slot_ready():
    ace = _StatusAce(_empty("ready"))
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_unit_busy():
    # A busy unit's slots can flicker 'empty' mid-motion; never abort on that.
    ace = _StatusAce({"status": "busy", "slots": [{"status": "empty"}]})
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_query_fails():
    assert _unit_with_ace(_StatusAce(raises=True))._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_disconnected():
    ace = _StatusAce(_empty(), connected=False)
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_no_ace():
    assert _unit_with_ace(None)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_slot_index_out_of_range():
    ace = _StatusAce(_empty())
    assert _unit_with_ace(ace)._slot_reports_empty(5) is False


# ── Unit tests for the ACE diagnostic gcode handlers in extras/AFC_ACE.py: ────
#
# was tests/test_AFC_ACE_diag_cmds.py
def _unit(ace):
    unit = afcACE.__new__(afcACE)
    unit._ace = ace
    return unit


# ── cmd_ACE_TEMP_INFO ─────────────────────────────────────────────────────────

def test_temp_info_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_TEMP_INFO(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_temp_info_get_temp_raises():
    ace = FakeAce2()
    ace.get_temp = Recorder(raises=RuntimeError("unsupported"))
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert len(gcmd.responses) == 1
    assert "ACE_TEMP_INFO" in gcmd.responses[0]
    assert "unsupported" in gcmd.responses[0]


def test_temp_info_non_dict_reply():
    ace = FakeAce2()
    ace.get_temp = Recorder(result=None)
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert len(gcmd.responses) == 1
    assert "unexpected reply" in gcmd.responses[0]


def test_temp_info_success_formats_all_channels():
    ace = FakeAce2()
    ace.get_temp = Recorder(result={
        'box1_temp': 30.5, 'box2_temp': 31.0,
        'ptc1_temp': 55.0, 'ptc2_temp': 60.0,
        'env_temp': 24.0, 'env_humidity': 41.0,
    })
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert ace.get_temp.call_count == 1
    assert len(gcmd.responses) == 1
    msg = gcmd.responses[0]
    for token in ("box1=30.5", "box2=31.0", "ptc1=55.0",
                  "ptc2=60.0", "env=24.0", "humidity=41.0"):
        assert token in msg


def test_temp_info_missing_channels_render_na():
    ace = FakeAce2()
    ace.get_temp = Recorder(result={'box1_temp': 30.5})  # rest absent
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    msg = gcmd.responses[0]
    assert "box1=30.5" in msg
    assert "humidity=n/a" in msg


# ── cmd_ACE_MATERIAL_INFO ─────────────────────────────────────────────────────

def test_material_info_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_MATERIAL_INFO(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_material_info_default_slot_zero():
    ace = FakeAce2()
    ace.get_material_info = Recorder(result={
        'index': 0, 'material_name': 'S0395MB251230046650C3', 'status': 0})
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert ace.get_material_info.last_args == (0,)
    msg = gcmd.responses[0]
    assert "slot 0" in msg
    assert "S0395MB251230046650C3" in msg


def test_material_info_explicit_slot():
    ace = FakeAce2()
    ace.get_material_info = Recorder(result={
        'index': 3, 'material_name': 'PETG', 'status': 1})
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=3)

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert ace.get_material_info.last_args == (3,)
    assert "slot 3" in gcmd.responses[0]
    assert "PETG" in gcmd.responses[0]


def test_material_info_error_surfaced():
    ace = FakeAce2()
    ace.get_material_info = Recorder(raises=RuntimeError("timeout"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0)

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert "ACE_MATERIAL_INFO" in gcmd.responses[0]
    assert "timeout" in gcmd.responses[0]


# ── cmd_ACE_SET_MATERIAL ──────────────────────────────────────────────────────

def test_set_material_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd(SLOT=0, NAME="X")
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_set_material_requires_slot():
    ace = FakeAce2()
    unit = _unit(ace)
    gcmd = FakeGcmd(NAME="X")  # no SLOT
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert "SLOT=<n> required" in gcmd.responses[0]
    assert ace.set_material_name.call_count == 0


def test_set_material_requires_name():
    ace = FakeAce2()
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0)  # no NAME
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert "NAME=<text> required" in gcmd.responses[0]
    assert ace.set_material_name.call_count == 0


def test_set_material_writes_and_reads_back():
    ace = FakeAce2()
    ace.set_material_name = Recorder(result={})
    ace.get_material_info = Recorder(result={'index': 2, 'material_name': 'PLA_X'})
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=2, NAME="PLA_X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    assert ace.set_material_name.last_args == (2, "PLA_X")
    assert ace.get_material_info.last_args == (2,)     # read-back
    assert "slot 2" in gcmd.responses[0]
    assert "PLA_X" in gcmd.responses[0]


def test_set_material_write_error_surfaced():
    ace = FakeAce2()
    ace.set_material_name = Recorder(raises=RuntimeError("boom"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0, NAME="X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    assert "ACE_SET_MATERIAL" in gcmd.responses[0]
    assert "boom" in gcmd.responses[0]
    assert ace.get_material_info.call_count == 0       # never reached read-back


def test_set_material_readback_failure_still_reports_write():
    ace = FakeAce2()
    ace.set_material_name = Recorder(result={})
    ace.get_material_info = Recorder(raises=RuntimeError("readfail"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0, NAME="X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    msg = gcmd.responses[0]
    assert "wrote 'X'" in msg
    assert "read-back failed" in msg


# ── cmd_ACE_SENSOR_STATE ──────────────────────────────────────────────────────

def test_sensor_state_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_SENSOR_STATE(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_sensor_state_reports_mask_and_triggered():
    ace = FakeAce2()
    sensors = [bool(0x11 & (1 << i)) for i in range(17)]  # bits 0 and 4
    ace.get_sensor_state = Recorder(result={
        'sensor_bitmask': 0x11, 'sensors': sensors})
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_SENSOR_STATE(gcmd)

    msg = gcmd.responses[0]
    assert "0x11" in msg
    assert "[0, 4]" in msg


def test_sensor_state_error_surfaced():
    ace = FakeAce2()
    ace.get_sensor_state = Recorder(raises=RuntimeError("nope"))
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_SENSOR_STATE(gcmd)

    assert "ACE_SENSOR_STATE" in gcmd.responses[0]
    assert "nope" in gcmd.responses[0]


# ── Unit tests for the ACE "current action" surfacing (extras/AFC_ACE.py): ────
#
# was tests/test_AFC_ACE_action.py
V1_BUSY = {"status": "busy", "action": "preload",
           "slots": [{"index": 0, "status": "preload"},
                     {"index": 1, "status": "ready"}]}
V1_IDLE = {"status": "ready", "slots": [{"index": 0, "status": "ready"}]}
ACE2_BUSY = {"status": "busy",
             "slots": [{"index": 0, "slot_status": "feeding", "status": "ready"},
                       {"index": 1, "slot_status": "ready", "status": "ready"}]}
ACE2_IDLE = {"status": "ready",
             "slots": [{"index": 0, "slot_status": "ready", "status": "ready"}]}


# ── _derive_action ────────────────────────────────────────────────────────────

def test_derive_action_v1_busy_slot_tagged():
    assert afcACE._derive_action(V1_BUSY) == "preload(slot 0)"


def test_derive_action_ace2_busy_slot_uses_slot_status():
    assert afcACE._derive_action(ACE2_BUSY) == "feeding(slot 0)"


def test_derive_action_top_level_fallback_when_no_busy_slot():
    r = {"action": "drying", "slots": [{"index": 0, "status": "ready"}]}
    assert afcACE._derive_action(r) == "drying"


def test_derive_action_idle_returns_empty():
    assert afcACE._derive_action(V1_IDLE) == ""
    assert afcACE._derive_action(ACE2_IDLE) == ""


def test_derive_action_no_slot_index_untagged():
    assert afcACE._derive_action({"slots": [{"slot_status": "rollback"}]}) == "rollback"


def test_derive_action_non_dict_and_empty():
    assert afcACE._derive_action(None) == ""
    assert afcACE._derive_action("nope") == ""
    assert afcACE._derive_action({}) == ""


# ── _on_hw_status_callback transition logging ─────────────────────────────────

def _make_unit_action(operation_active=False):
    unit = afcACE.__new__(afcACE)
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit._cached_hw_status = {}
    unit._cached_temp_info = {}
    unit._current_action = ""
    unit._operation_active = operation_active
    unit._sync_slot_states = Recorder()
    unit._maybe_assist_watchdog = Recorder()
    unit._check_stuck = Recorder()
    return unit


def test_callback_logs_transition_and_tracks_action():
    unit = _make_unit_action()

    unit._on_hw_status_callback({"result": V1_BUSY})
    assert unit._current_action == "preload(slot 0)"
    assert any("Ace_1: idle -> preload(slot 0)" in m
               for m in unit.logger.lines["info"])

    unit._on_hw_status_callback({"result": V1_IDLE})
    assert unit._current_action == ""
    assert any("preload(slot 0) -> idle" in m for m in unit.logger.lines["info"])


def test_callback_no_duplicate_log_when_action_unchanged():
    unit = _make_unit_action()
    unit._on_hw_status_callback({"result": ACE2_BUSY})
    n = len(unit.logger.lines["info"])
    unit._on_hw_status_callback({"result": ACE2_BUSY})   # same action
    assert len(unit.logger.lines["info"]) == n           # no new transition line


def test_callback_tracks_action_even_during_operation():
    unit = _make_unit_action(operation_active=True)

    unit._on_hw_status_callback({"result": ACE2_BUSY})

    assert unit._current_action == "feeding(slot 0)"     # logged despite op-active
    assert unit._sync_slot_states.call_count == 0        # but sync still skipped


def test_callback_temp_reply_does_not_touch_action():
    unit = _make_unit_action()
    unit._current_action = "feeding(slot 0)"
    # A get_temp reply (no slots) routes to the temp cache and returns early.
    unit._on_hw_status_callback({"result": {"ptc1_temp": 55.0}})
    assert unit._current_action == "feeding(slot 0)"     # unchanged


# ── cmd_ACE_STATUS ────────────────────────────────────────────────────────────

def _status_unit(status_result):
    unit = afcACE.__new__(afcACE)
    ace = FakeAce()
    ace.get_status = Recorder(result=status_result)
    unit._ace = ace
    return unit


def test_cmd_ace_status_reports_busy_action():
    unit = _status_unit(ACE2_BUSY)
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert any("action: feeding(slot 0)" in r for r in gcmd.responses)


def test_cmd_ace_status_reports_idle():
    unit = _status_unit(V1_IDLE)
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert any("action: idle" in r for r in gcmd.responses)


def test_cmd_ace_status_not_connected():
    unit = _status_unit(V1_IDLE)
    unit._ace.connected = False
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert gcmd.responses == ["ACE not connected"]


# ── get_status (Moonraker-queryable unit state) ───────────────────────────────

def _status_obj_unit(hw=None, action="", inventory=None, connected=True):
    unit = afcACE.__new__(afcACE)
    unit.lanes = {}          # empty -> base get_status returns empty aggregates
    unit._cached_hw_status = hw or {}
    unit._cached_temp_info = {}
    unit._hw_status_time = None      # no heartbeat yet -> age/stale unset
    unit._current_action = action
    unit._ace = FakeAce(connected=connected)
    unit._slot_inventory = (inventory if inventory is not None
                            else [{} for _ in range(afcACE.SLOTS_PER_UNIT)])
    return unit


def test_get_status_adds_ace_state():
    hw = {"status": "busy", "temp": 28, "dryer_status": {"status": "stop"}}
    inv = ([{"status": "ready", "rfid": 2, "sku": "HPL19-107",
             "material": "PLA", "uid": "BB2613B0102474", "color": [137, 168, 79]}]
           + [{} for _ in range(afcACE.SLOTS_PER_UNIT - 1)])
    unit = _status_obj_unit(hw=hw, action="feeding(slot 0)", inventory=inv)

    st = unit.get_status()

    # base structure preserved
    assert st["lanes"] == [] and "hubs" in st and "buffers" in st
    # ACE live state
    assert st["ace_connected"] is True
    assert st["ace_status"] == "busy"
    assert st["ace_action"] == "feeding(slot 0)"
    assert st["ace_temp"] == 28
    assert st["ace_dryer"] == "stop"
    assert len(st["ace_slots"]) == afcACE.SLOTS_PER_UNIT
    s0 = st["ace_slots"][0]
    assert (s0["sku"], s0["material"], s0["uid"], s0["rfid"]) == \
        ("HPL19-107", "PLA", "BB2613B0102474", 2)
    assert s0["color"] == [137, 168, 79]


def test_get_status_humidity_only_when_present():
    ace2 = _status_obj_unit(hw={"status": "ready", "temp": 26, "humidity": 31})
    assert ace2.get_status()["ace_humidity"] == 31
    v1 = _status_obj_unit(hw={"status": "ready", "temp": 26})  # V1 omits humidity
    assert "ace_humidity" not in v1.get_status()


def test_get_status_disconnected_and_empty():
    unit = _status_obj_unit(connected=False)
    st = unit.get_status()
    assert st["ace_connected"] is False
    assert st["ace_action"] == ""
    assert st["ace_temp"] is None
    assert len(st["ace_slots"]) == afcACE.SLOTS_PER_UNIT


def test_get_status_falls_back_to_temp_cache_for_ace2():
    # ACE2's get_status payload has no temp/humidity (they arrive via get_temp
    # into _cached_temp_info); env channels must still surface.
    unit = _status_obj_unit(hw={"status": "ready"})   # no temp/humidity in status
    unit._cached_temp_info = {"env_temp": 24.5, "env_humidity": 38}
    st = unit.get_status()
    assert st["ace_temp"] == 24.5
    assert st["ace_humidity"] == 38


def test_get_status_reports_stale_when_cache_ages():
    unit = _status_obj_unit(hw={"status": "ready"})
    unit.afc = types.SimpleNamespace(
        reactor=types.SimpleNamespace(monotonic=lambda: 100.0))
    unit._hw_status_time = 100.0                       # fresh -> not stale
    assert unit.get_status()["ace_status_stale"] is False
    unit._hw_status_time = 100.0 - 20.0                # 20s old (> 3 heartbeats)
    st = unit.get_status()
    assert st["ace_status_stale"] is True
    assert st["ace_status_age"] >= 20.0


# ── ACE2 _decode_status now indexes slots (so _derive_action can tag them) ─────



def test_ace2_decode_status_indexes_and_tags_busy_slot():
    # One real slot: slot_state=1 (feeding), filament_state=1 (present); padded.
    slot0 = pb_uint32(1, 1) + pb_uint32(2, 1)
    status = _decode_status({9: [(2, slot0)]})
    assert [s["index"] for s in status["slots"]] == [0, 1, 2, 3]
    assert status["slots"][0]["slot_status"] == "feeding"
    # The whole point: the ACE2 action is now slot-tagged like the V1's.
    assert afcACE._derive_action(status) == "feeding(slot 0)"


def test_ace2_decode_status_pads_slots_with_index():
    status = _decode_status({})          # no field-9 slots -> 4 padded
    assert [s["index"] for s in status["slots"]] == [0, 1, 2, 3]


# ── Unit tests for ACE sensor/state helpers in extras/AFC_ACE.py and the ACE2 ───
#
# was tests/test_AFC_ACE_sensors.py
# ── Fakes ─────────────────────────────────────────────────────────────────────

class _U1Sensor:
    """A U1 motion sensor: exposes the raw physical switch state."""

    def __init__(self, buttun_state):
        self.runout_buttun_state = buttun_state


class _PlainSensor:
    """A plain switch sensor: no runout_buttun_state attribute."""


class _Extruder_sensors:
    """AFC_extruder stand-in with the two possible sensor attributes; omit
    either by passing the _MISSING sentinel."""

    _MISSING = object()

    def __init__(self, filament_sensor_obj=None, fila_tool_start=None):
        if filament_sensor_obj is not self._MISSING:
            self.filament_sensor_obj = filament_sensor_obj
        if fila_tool_start is not self._MISSING:
            self.fila_tool_start = fila_tool_start


def _make_unit_sensors():
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    return unit


def _sensor_lane(ext, pre_sensor=False):
    lane = FakeLane("lane0", extruder_obj=ext)
    lane.get_toolhead_pre_sensor_state = Recorder(result=pre_sensor)
    return lane


# ── _toolhead_sensor_triggered ────────────────────────────────────────────────

def test_toolhead_sensor_uses_u1_button_state_true():
    unit = _make_unit_sensors()
    lane = _sensor_lane(_Extruder_sensors(fila_tool_start=_U1Sensor(1)))

    assert unit._toolhead_sensor_triggered(lane) is True
    assert not lane.get_toolhead_pre_sensor_state.called  # no fallback


def test_toolhead_sensor_uses_u1_button_state_false():
    unit = _make_unit_sensors()
    lane = _sensor_lane(_Extruder_sensors(fila_tool_start=_U1Sensor(0)))

    assert unit._toolhead_sensor_triggered(lane) is False
    assert not lane.get_toolhead_pre_sensor_state.called


def test_toolhead_sensor_filament_sensor_obj_takes_priority():
    unit = _make_unit_sensors()
    ext = _Extruder_sensors(filament_sensor_obj=_U1Sensor(1),
                    fila_tool_start=_U1Sensor(0))
    lane = _sensor_lane(ext)

    assert unit._toolhead_sensor_triggered(lane) is True


def test_toolhead_sensor_plain_sensor_falls_back():
    """A sensor without runout_buttun_state -> normal pre-sensor read."""
    unit = _make_unit_sensors()
    lane = _sensor_lane(_Extruder_sensors(fila_tool_start=_PlainSensor()),
                        pre_sensor=True)

    assert unit._toolhead_sensor_triggered(lane) is True
    assert lane.get_toolhead_pre_sensor_state.call_count == 1


def test_toolhead_sensor_no_sensor_objects_falls_back():
    unit = _make_unit_sensors()
    lane = _sensor_lane(_Extruder_sensors(filament_sensor_obj=_Extruder_sensors._MISSING,
                                  fila_tool_start=_Extruder_sensors._MISSING),
                        pre_sensor=False)

    assert unit._toolhead_sensor_triggered(lane) is False
    assert lane.get_toolhead_pre_sensor_state.call_count == 1


# ── Virtual hub semantics ─────────────────────────────────────────────────────

def test_is_virtual_hub_all_branches():
    unit = _make_unit_sensors()

    lane = FakeLane("lane0", hub_obj=None)
    assert unit._is_virtual_hub(lane) is False

    class _RealHub:
        pass  # no is_virtual_pin attribute

    lane = FakeLane("lane0", hub_obj=_RealHub())
    assert unit._is_virtual_hub(lane) is False

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True))
    assert unit._is_virtual_hub(lane) is True

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=False))
    assert unit._is_virtual_hub(lane) is False


def test_virtual_hub_occupancy_derives_from_tool_loaded():
    """The virtual hub reads 'loaded' only while filament is threaded THROUGH
    it to a toolhead — a staged-but-not-loaded lane stays clear so its own
    load doesn't trip 'Hub not clear'."""
    unit = _make_unit_sensors()

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True), tool_loaded=False)
    unit._set_hub_state(lane, True)   # the staged flag is NOT the live signal
    assert lane._load_state is False

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True), tool_loaded=True)
    unit._set_hub_state(lane, False)
    assert lane._load_state is True


def test_real_hub_left_alone():
    unit = _make_unit_sensors()
    lane = FakeLane("lane0", hub_obj=None)
    lane._load_state = "sentinel"

    unit._set_hub_state(lane, True)

    assert lane._load_state == "sentinel"


# ── _parse_ace_params ─────────────────────────────────────────────────────────

def test_parse_params_real_json():
    assert afcACE._parse_ace_params('{"index": 0, "type": "PLA"}') == {
        "index": 0, "type": "PLA"}


def test_parse_params_console_stripped_quotes():
    """The gcode console strips JSON double-quotes — the quote-less form must
    parse with int/float/bool/string inference."""
    result = afcACE._parse_ace_params('{index:0,length:50.5,type:PLA,dry:true}')
    assert result == {"index": 0, "length": 50.5, "type": "PLA", "dry": True}


def test_parse_params_false_bool():
    assert afcACE._parse_ace_params('{dry:false}') == {"dry": False}


def test_parse_params_list_values():
    result = afcACE._parse_ace_params('{index:0,color:[255,0,0]}')
    assert result == {"index": 0, "color": [255, 0, 0]}


def test_parse_params_empty_returns_none():
    assert afcACE._parse_ace_params("") is None
    assert afcACE._parse_ace_params(None) is None
    assert afcACE._parse_ace_params("   ") is None


# ── ACE2 _apply_feed_check ────────────────────────────────────────────────────

def _make_ace2(ace=None):
    unit = afcACE2.__new__(afcACE2)
    unit.name = "ACE_1"
    unit.logger = FakeLogger()
    unit.feed_check_length = 200
    unit.feed_error_length = 190
    unit._ace = ace
    return unit


def test_feed_check_pushes_configured_window():
    unit = _make_ace2(ace=FakeAce())
    unit._apply_feed_check()
    assert unit._ace.send_command_async.calls == [(
        ("set_feed_check", {"check_length": 200, "error_length": 190}), {})]
    assert len(unit.logger.lines["info"]) == 1


def test_feed_check_noop_without_connection():
    unit = _make_ace2(ace=None)
    unit._apply_feed_check()  # must not raise
    assert unit.logger.lines["info"] == []


def test_feed_check_error_is_nonfatal():
    ace = FakeAce()
    ace.send_command_async = Recorder(raises=RuntimeError("serial down"))
    unit = _make_ace2(ace=ace)

    unit._apply_feed_check()  # swallowed with a warning

    assert len(unit.logger.lines["warning"]) == 1


# ── Unit tests for the ACE get_temp caching path (extras/AFC_ACE.py + AFC_ACE2.py): ───
#
# was tests/test_AFC_ACE_temp_cache.py
# ── _on_hw_status_callback routing ────────────────────────────────────────────

def _make_unit_temp_cache(operation_active=False):
    unit = afcACE.__new__(afcACE)
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit._cached_hw_status = {}
    unit._cached_temp_info = {}
    unit._current_action = ""
    unit._operation_active = operation_active
    unit._sync_slot_states = Recorder()
    unit._maybe_assist_watchdog = Recorder()
    unit._check_stuck = Recorder()
    return unit


def test_status_reply_updates_status_cache_and_syncs():
    unit = _make_unit_temp_cache()
    status = {"status": "ready", "slots": [{"status": "ready"}]}

    unit._on_hw_status_callback({"result": status})

    assert unit._cached_hw_status == status
    assert unit._cached_temp_info == {}          # untouched
    assert unit._sync_slot_states.call_count == 1
    assert unit._sync_slot_states.last_args == (status,)
    assert unit._maybe_assist_watchdog.call_count == 1
    assert unit._check_stuck.call_count == 1


def test_temp_reply_updates_temp_cache_only():
    unit = _make_unit_temp_cache()
    unit._cached_hw_status = {"status": "ready", "slots": []}
    prev_status = unit._cached_hw_status
    temp = {"box1_temp": 0.0, "ptc1_temp": 55.0, "env_temp": 27.0,
            "env_humidity": 30.0}

    unit._on_hw_status_callback({"result": temp})

    assert unit._cached_temp_info == temp
    assert unit._cached_hw_status is prev_status  # status cache NOT overwritten
    # No slot-state work runs on a thermal reply.
    assert unit._sync_slot_states.call_count == 0
    assert unit._maybe_assist_watchdog.call_count == 0
    assert unit._check_stuck.call_count == 0


def test_temp_reply_detected_by_box_or_env_only():
    # A reply with box1_temp but no ptc/env still routes to the temp cache.
    unit = _make_unit_temp_cache()
    unit._on_hw_status_callback({"result": {"box1_temp": 24.0}})
    assert unit._cached_temp_info == {"box1_temp": 24.0}
    assert unit._sync_slot_states.call_count == 0


def test_status_reply_with_operation_active_caches_but_skips_sync():
    unit = _make_unit_temp_cache(operation_active=True)
    status = {"status": "busy", "slots": []}

    unit._on_hw_status_callback({"result": status})

    assert unit._cached_hw_status == status      # still cached
    assert unit._sync_slot_states.call_count == 0  # but sync suppressed


def test_non_dict_response_ignored():
    unit = _make_unit_temp_cache()
    unit._on_hw_status_callback("not a dict")
    unit._on_hw_status_callback({"result": "not a dict"})
    assert unit._cached_hw_status == {}
    assert unit._cached_temp_info == {}
    assert unit._sync_slot_states.call_count == 0


def test_bare_status_without_result_wrapper():
    # Some callers pass the status dict directly (no 'result' envelope).
    unit = _make_unit_temp_cache()
    status = {"slots": [], "status": "ready"}
    unit._on_hw_status_callback(status)
    assert unit._cached_hw_status == status
    assert unit._sync_slot_states.call_count == 1


# ── _poll_extras ──────────────────────────────────────────────────────────────

def test_base_poll_extras_is_noop():
    conn = ACEConnection.__new__(ACEConnection)
    conn.send_command_async = Recorder()
    # Should not raise and should not send anything (V1 has no get_temp).
    assert conn._poll_extras() is None
    assert conn.send_command_async.call_count == 0


def test_ace2_poll_extras_sends_get_temp():
    conn = ACE2Connection.__new__(ACE2Connection)
    conn.send_command_async = Recorder()

    conn._poll_extras()

    # Polls both the thermal channels and the per-lane buffer/sensor state.
    assert conn.send_command_async.call_count == 2
    methods = [c[0][0] for c in conn.send_command_async.calls]
    assert methods == ["get_temp", "get_sensor_state"]


# ── Unit tests for the startup-prep RFID inventory sweep gating (extras/AFC_ACE.py ───
#
# was tests/test_AFC_ACE_sync_inventory.py
class _FakeConn:
    """ACE serial connection fake that records each get_filament_info call and
    returns a canned per-slot payload."""

    def __init__(self, connected=True, payloads=None):
        self.connected = connected
        self._payloads = payloads or {}
        self.filament_calls = []

    def get_filament_info(self, slot):
        self.filament_calls.append(slot)
        return self._payloads.get(slot, {"index": slot})


def _make_v1(conn):
    unit = afcACE.__new__(afcACE)
    unit._ace = conn
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    # _store_slot_rfid runs for real; FakeLogger absorbs its debug/info lines.
    return unit


def _make_v2(conn):
    unit = afcACE2.__new__(afcACE2)
    unit._ace = conn
    unit.name = "Ace2_1"
    unit.logger = FakeLogger()
    unit._slot_inventory = [{} for _ in range(afcACE2.SLOTS_PER_UNIT)]
    return unit


# ── class flag ────────────────────────────────────────────────────────────────

def test_v1_uses_firmware_rfid_true():
    assert afcACE._uses_firmware_rfid is True


def test_v2_uses_firmware_rfid_false():
    assert afcACE2._uses_firmware_rfid is False


# ── V1 sweeps the firmware ──────────────────────────────────────────────────────

def test_v1_sync_inventory_reads_every_slot():
    conn = _FakeConn(connected=True)
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert conn.filament_calls == list(range(afcACE.SLOTS_PER_UNIT))


def test_v1_sync_inventory_stores_payload():
    conn = _FakeConn(
        connected=True,
        payloads={0: {"index": 0, "sku": "HPL19-107", "type": "PLA"}})
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert unit._slot_inventory[0]["sku"] == "HPL19-107"
    assert unit._slot_inventory[0]["material"] == "PLA"


def test_v1_sync_inventory_skips_when_disconnected():
    conn = _FakeConn(connected=False)
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert conn.filament_calls == []


def test_v1_sync_inventory_skips_when_no_conn():
    unit = _make_v1(_FakeConn())
    unit._ace = None

    unit._sync_inventory()  # must not raise


# ── V2 never touches the firmware ───────────────────────────────────────────────

def test_v2_sync_inventory_skips_firmware_even_when_connected():
    conn = _FakeConn(connected=True,
                     payloads={0: {"index": 0, "sku": "HPL19-107"}})
    unit = _make_v2(conn)

    unit._sync_inventory()

    # The whole point: no firmware get_filament_info sweep on the ACE 2.
    assert conn.filament_calls == []
    # And the slot cache is left untouched by the (skipped) sweep.
    assert unit._slot_inventory[0] == {}


# ── Tests for the concurrent (overlapping) ACE unload retract in extras/AFC_ACE.py ───
#
# was tests/test_AFC_ACE_unload_overlap.py
class _Extruder_unload_overlap:
    def __init__(self, tool_stn_unload, tool_unload_speed=25.0):
        self.tool_stn_unload = tool_stn_unload
        self.tool_unload_speed = tool_unload_speed


class _DeltaTime:
    def log_with_time(self, msg, debug=True):
        pass


class _AFC:
    def __init__(self, events):
        self._events = events
        self.error = FakeError()
        self.error.handle_lane_failure = Recorder()
        self.function = FakeFunction()
        self.function.log_toolhead_pos = Recorder()
        self.afcDeltaTime = _DeltaTime()
        self.move_calls = []

    def move_e_pos(self, e_amount, speed, log_string="", wait_tool=False):
        self.move_calls.append(
            {"e_amount": e_amount, "speed": speed, "wait_tool": wait_tool})
        self._events.append(("move", wait_tool))


def _make_ace_unload_overlap(events, tool_stn_unload=60.0):
    unit = afcACE.__new__(afcACE)
    unit.afc = _AFC(events)
    unit.logger = FakeLogger()
    unit.serial_port = "/dev/ttyACM0"
    unit.retract_speed = 50.0
    ace = FakeAce(connected=True)

    def _unwind(*a, **k):
        events.append(("unwind",))
    ace.unwind_filament = _unwind
    unit._ace = ace
    unit._hub_load_suppressed = set()
    unit._get_slot = Recorder(result=3)
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._set_hub_state = Recorder()
    unit.lane_tool_unloaded = Recorder()

    def _stop(slot):
        events.append(("stop_assist", slot))
    unit._stop_feed_assist = _stop
    return unit, _Extruder_unload_overlap(tool_stn_unload)


def _idx(events, tag):
    return next(i for i, e in enumerate(events) if e[0] == tag)


def test_two_retracts_first_blocks_second_overlaps_unwind():
    events = []
    unit, ext = _make_ace_unload_overlap(events, tool_stn_unload=60.0)
    lane = FakeLane("lane3", hub_obj=FakeHub())

    assert unit._ace_unload_inner(lane, ext) is True

    # assist stopped, THEN two extruder retracts, THEN the ACE rollback — both
    # retracts precede the rollback so the second (async) one overlaps it.
    moves = [i for i, e in enumerate(events) if e[0] == "move"]
    assert len(moves) == 2
    assert _idx(events, "stop_assist") < moves[0] < moves[1] < _idx(events, "unwind")
    assert len(unit.afc.move_calls) == 2
    first, second = unit.afc.move_calls
    assert first["wait_tool"] is True            # first retract blocks (freed slack)
    assert first["e_amount"] == -30.0            # -tool_stn_unload / 2
    assert first["speed"] == ext.tool_unload_speed
    assert second["wait_tool"] is False          # second retract overlaps the unwind
    assert second["e_amount"] == -60.0           # full -tool_stn_unload
    assert second["speed"] == ext.tool_unload_speed


def test_no_retract_move_when_tool_stn_unload_zero_but_still_unwinds():
    events = []
    unit, ext = _make_ace_unload_overlap(events, tool_stn_unload=0.0)
    assert unit._ace_unload_inner(FakeLane("lane3", hub_obj=FakeHub()), ext) is True

    # No extruder move when disabled, but the ACE rollback still runs.
    assert unit.afc.move_calls == []
    assert any(e[0] == "unwind" for e in events)


# ── Tests for AFC_ACE.eject_lane's reload-suppression (extras/AFC_ACE.py) ─────
#
# was tests/test_AFC_ACE_eject.py
def _make_ace_eject(connected=True):
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit.retract_speed = 50.0
    unit.eject_buffer = 475.0
    unit._ace = FakeAce(connected=connected)
    unit._ace.unwind_filament = Recorder()
    unit._hub_load_suppressed = set()
    unit._get_slot = Recorder(result=3)
    unit._operation = lambda: contextlib.nullcontext()
    unit._stop_feed_assist = Recorder()
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._set_hub_state = Recorder()
    return unit


def _hub_staged_lane():
    lane = FakeLane("lane3", hub_obj=FakeHub())
    lane.dist_hub = 100.0
    lane.tool_loaded = False
    lane.loaded_to_hub = True
    return lane


def test_eject_suppresses_auto_reload():
    unit = _make_ace_eject()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # The fix: the ejected lane is suppressed so the ready-slot sync won't
    # immediately pull the filament back in while the spool is still present.
    assert "lane3" in unit._hub_load_suppressed


def test_eject_clears_hub_state():
    unit = _make_ace_eject()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    assert lane.loaded_to_hub is False
    # _set_hub_state(lane, False) was called — hub signal cleared.
    assert unit._set_hub_state.calls
    assert unit._set_hub_state.last_args[1] is False


def test_eject_retracts_hub_stage_distance():
    unit = _make_ace_eject()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # Hub-staged (not tool-loaded): dist_hub (100) + eject_buffer (475) at
    # retract_speed (50).
    assert unit._ace.unwind_filament.calls
    args = unit._ace.unwind_filament.last_args
    assert args[1] == 575.0
    assert args[2] == 50.0


def test_eject_noop_when_disconnected():
    unit = _make_ace_eject(connected=False)
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # Nothing happens when the ACE isn't connected — no suppression, no retract,
    # and the caller's loaded_to_hub is left untouched.
    assert "lane3" not in unit._hub_load_suppressed
    assert not unit._ace.unwind_filament.calls
    assert lane.loaded_to_hub is True


# ── Tests for the shared-reader RFID ambiguity guard in extras/AFC_ACE.py / ───
#
# was tests/test_AFC_ACE_shared_reader.py
def _inv(cls):
    return [{} for _ in range(cls.SLOTS_PER_UNIT)]


def test_base_ace_has_no_shared_reader_sibling():
    unit = afcACE.__new__(afcACE)
    assert unit._reader_sibling_slot(0) is None
    unit._slot_inventory = _inv(afcACE)
    unit._slot_inventory[0] = {"sku": "X"}
    unit._slot_inventory[1] = {"sku": "X"}
    assert unit._shared_rfid_ambiguous(0) is False   # per-slot reader: never shared


def test_ace2_reader_sibling_pairs():
    unit = afcACE2.__new__(afcACE2)
    assert unit._reader_sibling_slot(0) == 1
    assert unit._reader_sibling_slot(1) == 0
    assert unit._reader_sibling_slot(2) == 3
    assert unit._reader_sibling_slot(3) == 2


def test_ace2_ambiguous_when_sibling_reports_same_sku():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[0] = {"sku": "HPL19-107"}
    unit._slot_inventory[1] = {"sku": "HPL19-107"}   # slot 1 read slot 0's tag
    assert unit._shared_rfid_ambiguous(1) is True
    assert unit._shared_rfid_ambiguous(0) is True


def test_ace2_ambiguous_when_sibling_reports_same_uid():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[2] = {"sku": "", "uid": "deadbeef"}
    unit._slot_inventory[3] = {"sku": "", "uid": "deadbeef"}
    assert unit._shared_rfid_ambiguous(2) is True


def test_ace2_not_ambiguous_when_sibling_differs():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[2] = {"sku": "BAMBU-A", "uid": "aa"}
    unit._slot_inventory[3] = {"sku": "BAMBU-B", "uid": "bb"}
    assert unit._shared_rfid_ambiguous(2) is False
    assert unit._shared_rfid_ambiguous(3) is False


def test_ace2_not_ambiguous_when_sibling_empty():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[0] = {"sku": "HPL19-107"}
    unit._slot_inventory[1] = {}                       # empty sibling
    assert unit._shared_rfid_ambiguous(0) is False

