"""
Unit tests for extras/AFC_OpenAMS.py

Covers:
  - AMSEventBus: singleton, subscribe / publish, priority ordering,
    graceful handling of failing subscribers
  - LaneInfo: dataclass-style attribute population
  - LaneRegistry: register / lookup by lane / spool / extruder / token
  - AMSHardwareService: latest_status / update_lane_snapshot / latest_lane_snapshot
  - OpenAMSStateMutation + EVENT_POLICY
  - afcAMS: inheritance, _is_openams_unit, get_lane_reset_command,
    unit hook signatures
"""

from __future__ import annotations

from unittest.mock import MagicMock
import pytest

from extras.AFC_OpenAMS import (
    AMSEventBus,
    AMSHardwareService,
    EVENT_POLICY,
    LaneInfo,
    LaneRegistry,
    OpenAMSStateMutation,
    afcAMS,
)
from extras.AFC_unit import afcUnit


# ── AMSEventBus ──────────────────────────────────────────────────────────────

class TestAMSEventBus:
    def setup_method(self):
        # Reset singleton each test to avoid cross-test contamination
        AMSEventBus._instance = None

    def test_get_instance_returns_singleton(self):
        bus1 = AMSEventBus.get_instance()
        bus2 = AMSEventBus.get_instance()
        assert bus1 is bus2

    def test_logger_set_on_first_call(self):
        logger = MagicMock()
        bus = AMSEventBus.get_instance(logger=logger)
        assert bus.logger is logger

    def test_logger_not_overridden_when_already_set(self):
        first_logger = MagicMock()
        second_logger = MagicMock()
        AMSEventBus.get_instance(logger=first_logger)
        bus = AMSEventBus.get_instance(logger=second_logger)
        assert bus.logger is first_logger

    def test_publish_with_no_subscribers_returns_zero(self):
        bus = AMSEventBus.get_instance()
        assert bus.publish("nobody_home") == 0

    def test_subscribe_and_publish_invokes_callback(self):
        bus = AMSEventBus.get_instance()
        calls = []
        bus.subscribe("tick", lambda **kw: calls.append(kw))
        bus.publish("tick", value=1, eventtime=0.1)
        assert len(calls) == 1
        assert calls[0]["value"] == 1
        assert calls[0]["event_type"] == "tick"

    def test_publish_returns_success_count(self):
        bus = AMSEventBus.get_instance()
        bus.subscribe("t", lambda **kw: None)
        bus.subscribe("t", lambda **kw: None)
        assert bus.publish("t") == 2

    def test_priority_higher_fires_first(self):
        bus = AMSEventBus.get_instance()
        order = []
        bus.subscribe("p", lambda **kw: order.append("low"), priority=0)
        bus.subscribe("p", lambda **kw: order.append("high"), priority=100)
        bus.publish("p")
        assert order == ["high", "low"]

    def test_failing_callback_does_not_stop_others(self):
        bus = AMSEventBus.get_instance(logger=MagicMock())

        def boom(**kw):
            raise RuntimeError("bad")

        good_calls = []
        bus.subscribe("err", boom)
        bus.subscribe("err", lambda **kw: good_calls.append(1))
        count = bus.publish("err")
        # Only the good one counted
        assert count == 1
        assert good_calls == [1]

    def test_publish_records_history(self):
        bus = AMSEventBus.get_instance()
        bus.publish("history_test", n=5)
        # Find our event in history
        assert any(ev[0] == "history_test" for ev in bus._event_history)


# ── LaneInfo ──────────────────────────────────────────────────────────────────

class TestLaneInfo:
    def test_minimum_fields(self):
        info = LaneInfo("lane1", "unit1", 0, "extruder")
        assert info.lane_name == "lane1"
        assert info.unit_name == "unit1"
        assert info.spool_index == 0
        assert info.extruder == "extruder"
        assert info.fps_name is None
        assert info.hub_name is None
        assert info.led_index is None

    def test_optional_fields(self):
        info = LaneInfo(
            "l", "u", 2, "e",
            fps_name="fps1", hub_name="hub1", led_index=3,
            custom_load_cmd="LOAD", custom_unload_cmd="UNLOAD",
        )
        assert info.fps_name == "fps1"
        assert info.hub_name == "hub1"
        assert info.led_index == 3
        assert info.custom_load_cmd == "LOAD"
        assert info.custom_unload_cmd == "UNLOAD"


# ── LaneRegistry ──────────────────────────────────────────────────────────────

class TestLaneRegistry:
    def _make(self):
        LaneRegistry._instances = {}
        return LaneRegistry(MagicMock(), logger=MagicMock())

    def test_for_printer_returns_same_instance_per_printer(self):
        LaneRegistry._instances = {}
        printer = MagicMock()
        a = LaneRegistry.for_printer(printer)
        b = LaneRegistry.for_printer(printer)
        assert a is b

    def test_for_printer_differs_per_printer(self):
        LaneRegistry._instances = {}
        a = LaneRegistry.for_printer(MagicMock())
        b = LaneRegistry.for_printer(MagicMock())
        assert a is not b

    def test_register_lane_returns_info(self):
        reg = self._make()
        info = reg.register_lane("lane1", "unitA", 1, "extruder_a")
        assert isinstance(info, LaneInfo)
        assert info.lane_name == "lane1"

    def test_get_by_lane(self):
        reg = self._make()
        reg.register_lane("lane1", "unitA", 1, "ex")
        assert reg.get_by_lane("lane1").unit_name == "unitA"
        assert reg.get_by_lane("missing") is None

    def test_get_by_spool(self):
        reg = self._make()
        reg.register_lane("lane1", "unitA", 2, "ex")
        assert reg.get_by_spool("unitA", 2).lane_name == "lane1"
        assert reg.get_by_spool("unitA", 99) is None

    def test_resolve_lane_token_case_insensitive(self):
        reg = self._make()
        reg.register_lane("Lane_Alpha", "u", 0, "e")
        assert reg.resolve_lane_token("lane_alpha").lane_name == "Lane_Alpha"
        assert reg.resolve_lane_token("LANE_ALPHA").lane_name == "Lane_Alpha"

    def test_resolve_lane_name(self):
        reg = self._make()
        reg.register_lane("laneX", "U", 1, "e")
        assert reg.resolve_lane_name("U", 1) == "laneX"
        assert reg.resolve_lane_name("U", 0) is None

    def test_resolve_extruder(self):
        reg = self._make()
        reg.register_lane("lane_q", "U", 0, "extruder_q")
        assert reg.resolve_extruder("lane_q") == "extruder_q"
        assert reg.resolve_extruder("missing") is None

    def test_re_register_replaces_entry(self):
        reg = self._make()
        reg.register_lane("l1", "U", 0, "ex1")
        reg.register_lane("l1", "U", 0, "ex2")
        assert reg.resolve_extruder("l1") == "ex2"


# ── AMSHardwareService ───────────────────────────────────────────────────────

class TestAMSHardwareService:
    def _make(self):
        AMSHardwareService._instances = {}
        printer = MagicMock()
        afc = MagicMock()
        afc.logger = MagicMock()
        printer.lookup_object = MagicMock(return_value=afc)
        # Isolate the LaneRegistry instance
        LaneRegistry._instances = {}
        return AMSHardwareService(printer, "default", logger=MagicMock())

    def test_latest_status_returns_dict(self):
        svc = self._make()
        svc._status = {"k": 1}
        assert svc.latest_status() == {"k": 1}
        # Mutation of returned dict must not leak
        out = svc.latest_status()
        out["k"] = 99
        assert svc.latest_status() == {"k": 1}

    def test_for_printer_returns_same_instance_per_key(self):
        AMSHardwareService._instances = {}
        printer = MagicMock()
        printer.lookup_object = MagicMock(return_value=MagicMock(logger=MagicMock()))
        a = AMSHardwareService.for_printer(printer, name="unit1")
        b = AMSHardwareService.for_printer(printer, name="unit1")
        assert a is b

    def test_for_printer_differs_by_name(self):
        AMSHardwareService._instances = {}
        printer = MagicMock()
        printer.lookup_object = MagicMock(return_value=MagicMock(logger=MagicMock()))
        a = AMSHardwareService.for_printer(printer, name="unit1")
        b = AMSHardwareService.for_printer(printer, name="unit2")
        assert a is not b

    def test_update_lane_snapshot_stores_basic_fields(self):
        svc = self._make()
        svc.event_bus = MagicMock()
        svc.update_lane_snapshot(
            unit_name="U", lane_name="L", lane_state=True,
            hub_state=False, eventtime=42.0,
        )
        snap = svc.latest_lane_snapshot("U", "L")
        assert snap["unit"] == "U"
        assert snap["lane"] == "L"
        assert snap["lane_state"] is True
        assert snap["hub_state"] is False
        assert snap["timestamp"] == 42.0

    def test_latest_lane_snapshot_returns_none_for_missing(self):
        svc = self._make()
        # Missing key -> returns None
        snap = svc.latest_lane_snapshot("unknown", "lane")
        assert snap is None

    def test_update_lane_snapshot_emits_spool_event_on_change(self):
        svc = self._make()
        bus = MagicMock()
        svc.event_bus = bus
        # First update seeds state; the code emits spool events only when the
        # old state existed and differs from the new state.
        svc.update_lane_snapshot(
            unit_name="U", lane_name="L", lane_state=False,
            hub_state=None, eventtime=1.0, spool_index=0,
        )
        bus.publish.assert_not_called()
        # Now change lane_state: should publish spool_loaded
        svc.update_lane_snapshot(
            unit_name="U", lane_name="L", lane_state=True,
            hub_state=None, eventtime=2.0, spool_index=0,
        )
        # spool_loaded is published with expected args
        calls = [c for c in bus.publish.call_args_list
                 if c.args and c.args[0] == "spool_loaded"]
        assert len(calls) == 1
        assert calls[0].kwargs["unit_name"] == "U"
        assert calls[0].kwargs["lane_name"] == "L"


# ── OpenAMSStateMutation / EVENT_POLICY ──────────────────────────────────────

class TestStateMutation:
    def test_values(self):
        assert OpenAMSStateMutation.SENSOR.value == "sensor"
        assert OpenAMSStateMutation.TOOL.value == "tool"
        assert OpenAMSStateMutation.RUNOUT.value == "runout"

    def test_sensor_policy_allows_everything(self):
        p = EVENT_POLICY[OpenAMSStateMutation.SENSOR.value]
        assert p["allow_spool_events"] is True
        assert p["allow_full_unload"] is True

    def test_tool_policy_blocks_spool_and_unload(self):
        p = EVENT_POLICY[OpenAMSStateMutation.TOOL.value]
        assert p["allow_spool_events"] is False
        assert p["allow_full_unload"] is False

    def test_runout_policy_allows_everything(self):
        p = EVENT_POLICY[OpenAMSStateMutation.RUNOUT.value]
        assert p["allow_spool_events"] is True
        assert p["allow_full_unload"] is True


# ── afcAMS unit ──────────────────────────────────────────────────────────────

def _make_ams(name="OpenAMS_1"):
    """Build an afcAMS bypassing __init__ for logic method tests."""
    unit = afcAMS.__new__(afcAMS)

    from tests.conftest import MockAFC, MockPrinter, MockLogger, MockReactor

    afc = MockAFC()
    reactor = MockReactor()
    afc.reactor = reactor
    afc.logger = MockLogger()
    printer = MockPrinter(afc=afc)

    unit.printer = printer
    unit.afc = afc
    unit.logger = afc.logger
    unit.reactor = reactor
    unit.name = name
    unit.full_name = ["AFC_OpenAMS", name]
    unit.type = "OpenAMS"
    unit.gcode = afc.gcode

    unit.lanes = {}
    unit.hub_obj = None
    unit.extruder_obj = None
    unit.buffer_obj = None
    unit.hub = None
    unit.extruder = None
    unit.buffer_name = None

    unit.oams_name = "oams1"
    unit.oams = None
    unit.registry = None
    unit.event_bus = None
    unit.stuck_spool_auto_recovery = False
    unit.stuck_spool_load_grace = 8.0
    unit.stuck_spool_pressure_threshold = 0.08
    unit._engagement_pressure_max = 0.6
    unit._engagement_pressure_min = 0.25
    unit.clog_sensitivity = "medium"

    return unit


class TestAfcAMSInheritance:
    def test_is_subclass_of_afc_unit(self):
        assert issubclass(afcAMS, afcUnit)

    def test_instance_is_afc_unit(self):
        unit = _make_ams()
        assert isinstance(unit, afcUnit)

    def test_type_attribute(self):
        unit = _make_ams()
        assert unit.type == "OpenAMS"


class TestIsOpenAmsUnit:
    def test_false_when_oams_none(self):
        unit = _make_ams()
        unit.oams = None
        assert unit._is_openams_unit() is False

    def test_true_when_oams_set(self):
        unit = _make_ams()
        unit.oams = MagicMock()
        assert unit._is_openams_unit() is True


class TestGetLaneResetCommand:
    def test_returns_tool_unload(self):
        unit = _make_ams()
        lane = MagicMock()
        lane.name = "laneX"
        cmd = unit.get_lane_reset_command(lane, 50)
        assert cmd == "TOOL_UNLOAD LANE=laneX"

    def test_ignores_distance_arg(self):
        unit = _make_ams()
        lane = MagicMock()
        lane.name = "laneY"
        cmd_none = unit.get_lane_reset_command(lane, None)
        cmd_50 = unit.get_lane_reset_command(lane, 50)
        assert cmd_none == cmd_50 == "TOOL_UNLOAD LANE=laneY"


class TestAfcAMSHookSignatures:
    """Verify all unit-level hook overrides exist and are callable."""

    def test_load_sequence(self):
        unit = _make_ams()
        assert callable(getattr(unit, "load_sequence"))

    def test_unload_sequence(self):
        unit = _make_ams()
        assert callable(getattr(unit, "unload_sequence"))

    def test_lane_unload(self):
        unit = _make_ams()
        assert callable(getattr(unit, "lane_unload"))

    def test_get_lane_reset_command(self):
        unit = _make_ams()
        assert callable(getattr(unit, "get_lane_reset_command"))

    def test_eject_lane(self):
        unit = _make_ams()
        assert callable(getattr(unit, "eject_lane"))

    def test_prep_load(self):
        unit = _make_ams()
        assert callable(getattr(unit, "prep_load"))

    def test_prep_post_load(self):
        unit = _make_ams()
        assert callable(getattr(unit, "prep_post_load"))
