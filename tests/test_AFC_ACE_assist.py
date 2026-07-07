"""
Unit tests for afcACE feed-assist management in extras/AFC_ACE.py

Covers:
  - _active_assist_lane: resolves the loaded lane on the active extruder,
    matching EITHER the [AFC_extruder] section name or th_extruder_name
    (the physical Klipper extruder name) — the SET_LANE_LOADED watchdog fix
  - _maybe_assist_watchdog: schedules a reconcile only on a genuine
    discrepancy; respects the ACE_FEED_ASSIST suppression set
  - cmd_ACE_FEED_ASSIST: ENABLE=0 suppresses + stops (incl. firmware stop on
    tracking drift); ENABLE=1 stops other slots first then starts; LANE/SLOT
    resolution and validation
  - _start_feed_assist: an explicit start clears the manual suppression
  - _reconcile_feed_assist: stop-others-first protocol, at_toolhead gate,
    suppression veto
  - _get_eject_length / _get_unload_length: retract distance math
  - check_runout: gated on is_printing
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from extras.AFC_ACE import afcACE


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_lane(name, ext_section="e0", ext_physical="extruder",
               tool_loaded=True, lane_loaded=None):
    lane = MagicMock()
    lane.name = name
    lane.tool_loaded = tool_loaded
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = ext_section
    lane.extruder_obj.th_extruder_name = ext_physical
    lane.extruder_obj.lane_loaded = lane_loaded
    return lane


def _make_unit(lanes=None, slot_map=None, active_extruder="extruder"):
    unit = afcACE.__new__(afcACE)
    unit.name = "ACE_1"
    unit.logger = MagicMock()
    unit.afc = MagicMock()
    unit.afc.lanes = dict(lanes or {})
    unit.lanes = dict(lanes or {})
    unit._slot_map = dict(slot_map or {})
    unit._feed_assist_active = set()
    unit._assist_suppressed = set()
    unit._assist_watchdog = True
    unit._ace = None

    unit.printer = MagicMock()
    toolhead = MagicMock()
    toolhead.get_extruder.return_value.get_name.return_value = active_extruder
    unit.printer.lookup_object.return_value = toolhead
    return unit


class _FakeGcmd:
    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get_int(self, name, default=None):
        val = self._params.get(name, default)
        return int(val) if val is not None else None

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


# ── _active_assist_lane ───────────────────────────────────────────────────────

def test_active_assist_lane_matches_section_name():
    lane = _make_lane("lane0", ext_section="extruder", ext_physical="extruder",
                      lane_loaded="lane0")
    unit = _make_unit({"lane0": lane}, active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_matches_physical_name():
    """[AFC_extruder e0] with extruder_name: extruder — toolhead reports the
    PHYSICAL name; the lane must still resolve (SET_LANE_LOADED fix)."""
    lane = _make_lane("lane0", ext_section="e0", ext_physical="extruder",
                      lane_loaded="lane0")
    unit = _make_unit({"lane0": lane}, active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_ignores_other_extruders():
    lane = _make_lane("lane0", ext_section="e0", ext_physical="extruder",
                      lane_loaded="lane0")
    unit = _make_unit({"lane0": lane}, active_extruder="extruder4")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_fallback_when_lane_loaded_lags():
    """At print start extruder.lane_loaded can lag behind tool_loaded — the
    unique loaded lane on the active extruder is still the assist target."""
    lane = _make_lane("lane0", ext_section="e0", ext_physical="extruder",
                      lane_loaded=None)
    unit = _make_unit({"lane0": lane}, active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_skips_unloaded_lanes():
    lane = _make_lane("lane0", tool_loaded=False)
    unit = _make_unit({"lane0": lane}, active_extruder="extruder")
    assert unit._active_assist_lane() is None


# ── _maybe_assist_watchdog ────────────────────────────────────────────────────

def _watchdog_unit():
    lane = _make_lane("lane0", ext_section="extruder", ext_physical="extruder",
                      lane_loaded="lane0")
    unit = _make_unit({"lane0": lane}, slot_map={"lane0": 0},
                      active_extruder="extruder")
    unit._use_feed_assist = MagicMock(return_value=True)
    return unit


def test_watchdog_schedules_reconcile_when_assist_missing():
    unit = _watchdog_unit()
    unit._maybe_assist_watchdog()
    unit.afc.reactor.register_callback.assert_called_once()


def test_watchdog_noop_when_assist_already_correct():
    unit = _watchdog_unit()
    unit._feed_assist_active = {0}
    unit._maybe_assist_watchdog()
    unit.afc.reactor.register_callback.assert_not_called()


def test_watchdog_respects_manual_suppression():
    unit = _watchdog_unit()
    unit._assist_suppressed = {0}
    unit._maybe_assist_watchdog()
    unit.afc.reactor.register_callback.assert_not_called()


def test_watchdog_disabled_by_config():
    unit = _watchdog_unit()
    unit._assist_watchdog = False
    unit._maybe_assist_watchdog()
    unit.afc.reactor.register_callback.assert_not_called()


# ── cmd_ACE_FEED_ASSIST ───────────────────────────────────────────────────────

def test_feed_assist_cmd_requires_enable():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="ENABLE is required"):
        unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(LANE="lane0"))


def test_feed_assist_cmd_requires_lane_or_slot():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="LANE or SLOT"):
        unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=0))


def test_feed_assist_cmd_unknown_lane():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="Unknown lane"):
        unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=0, LANE="nope"))


def test_feed_assist_stop_suppresses_and_stops_tracked_slot():
    unit = _make_unit(slot_map={"lane0": 2})
    unit._feed_assist_active = {2}
    unit._stop_feed_assist = MagicMock()

    unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=0, LANE="lane0"))

    assert 2 in unit._assist_suppressed
    unit._stop_feed_assist.assert_called_once_with(2)


def test_feed_assist_stop_sends_firmware_stop_on_tracking_drift():
    """Slot not tracked as assisting, but the firmware might be — the manual
    stop must still reach the hardware."""
    unit = _make_unit(slot_map={"lane0": 2})
    unit._ace = MagicMock()
    unit._ace.connected = True
    unit._stop_feed_assist = MagicMock()

    unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=0, LANE="lane0"))

    assert 2 in unit._assist_suppressed
    unit._stop_feed_assist.assert_not_called()          # not tracked
    unit._ace.stop_feed_assist.assert_called_once_with(2)  # raw firmware stop


def test_feed_assist_start_stops_other_slots_first():
    """The ACE can only assist one slot at a time — a manual start must stop
    the other assisting slot(s) before starting (or the ACE refuses with
    error_2)."""
    unit = _make_unit(slot_map={"lane0": 0, "lane1": 1})
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist = MagicMock(side_effect=lambda s: calls.append(("stop", s)))
    unit._start_feed_assist = MagicMock(side_effect=lambda s: calls.append(("start", s)))

    unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=1, LANE="lane0"))

    assert calls == [("stop", 1), ("start", 0)]


def test_feed_assist_start_accepts_slot_param():
    unit = _make_unit()
    unit._start_feed_assist = MagicMock()
    unit.cmd_ACE_FEED_ASSIST(_FakeGcmd(ENABLE=1, SLOT=3))
    unit._start_feed_assist.assert_called_once_with(3)


def test_explicit_start_clears_suppression():
    """Any explicit _start_feed_assist (command or load path) ends the manual
    ACE_FEED_ASSIST suppression — with no hardware connected it early-outs
    after the discard."""
    unit = _make_unit()
    unit._assist_suppressed = {2}
    unit._start_feed_assist(2)  # real method; _ace is None -> returns early
    assert 2 not in unit._assist_suppressed


# ── _reconcile_feed_assist ────────────────────────────────────────────────────

def _reconcile_unit():
    lane = _make_lane("lane0")
    unit = _make_unit({"lane0": lane}, slot_map={"lane0": 0, "lane1": 1})
    unit._use_feed_assist = MagicMock(return_value=True)
    unit._toolhead_sensor_triggered = MagicMock(return_value=True)
    unit._stop_feed_assist = MagicMock()
    unit._start_feed_assist = MagicMock()
    return unit


def test_reconcile_stops_other_slots_then_starts_target():
    unit = _reconcile_unit()
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist.side_effect = lambda s: calls.append(("stop", s))
    unit._start_feed_assist.side_effect = lambda s: calls.append(("start", s))

    unit._reconcile_feed_assist("lane0")

    assert calls == [("stop", 1), ("start", 0)]


def test_reconcile_does_not_start_before_filament_at_toolhead():
    unit = _reconcile_unit()
    unit._toolhead_sensor_triggered.return_value = False
    unit._reconcile_feed_assist("lane0")
    unit._start_feed_assist.assert_not_called()


def test_reconcile_respects_suppression():
    unit = _reconcile_unit()
    unit._assist_suppressed = {0}
    unit._reconcile_feed_assist("lane0")
    unit._start_feed_assist.assert_not_called()


def test_reconcile_lane_on_other_unit_stops_ours():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}
    unit.afc.lanes["other_lane"] = MagicMock()

    unit._reconcile_feed_assist("other_lane")  # not in our _slot_map

    unit._stop_feed_assist.assert_called_once_with(0)
    unit._start_feed_assist.assert_not_called()


# ── Retract distance math ─────────────────────────────────────────────────────

def _distance_unit():
    unit = _make_unit()
    unit.eject_buffer = 475.0
    return unit


def test_eject_length_staged_at_hub():
    unit = _distance_unit()
    lane = MagicMock()
    lane.tool_loaded = False
    lane.dist_hub = 300.0
    assert unit._get_eject_length(lane) == 300.0 + 475.0


def test_eject_length_tool_loaded_uses_full_unload_path():
    unit = _distance_unit()
    lane = MagicMock()
    lane.tool_loaded = True
    lane.dist_hub = 300.0
    lane.hub_obj = MagicMock()
    lane.hub_obj.afc_unload_bowden_length = 1000.0
    assert unit._get_eject_length(lane) == 300.0 + 1000.0


def test_unload_length_without_hub():
    unit = _distance_unit()
    lane = MagicMock()
    lane.dist_hub = 300.0
    lane.hub_obj = None
    assert unit._get_unload_length(lane) == 300.0


# ── check_runout ──────────────────────────────────────────────────────────────

def test_check_runout_true_while_printing():
    unit = _make_unit()
    unit.afc.function.is_printing.return_value = True
    assert unit.check_runout(MagicMock()) is True


def test_check_runout_false_when_idle():
    unit = _make_unit()
    unit.afc.function.is_printing.return_value = False
    assert unit.check_runout(MagicMock()) is False
