"""
Unit tests for afcAMS sensor polling and load finishing in
extras/AFC_OpenAMS.py

Covers:
  - _poll_oams_sensors: F1S -> prep_state, F1S loss clears loaded_to_hub and
    fires handle_load_runout on a genuine transition, resync-after-operation
    suppresses false events, suppressed loads mark _load_suppressed, hub HES
    drives raw _load_state, operation-active and no-hardware early-outs
  - _advance_tool_stn_to_nozzle: sensor->nozzle distance math incl. the
    already-advanced (engagement) credit
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_OpenAMS import afcAMS


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_lane(name):
    lane = MagicMock()
    lane.name = name
    lane.prep_state = False
    lane.loaded_to_hub = True
    lane._load_state = False
    lane._load_suppressed = False
    lane._oams_runout_detected = False
    return lane


def _make_unit(lane, f1s, hub, last_f1s=None, stale=False, op_active=False):
    unit = afcAMS.__new__(afcAMS)
    unit.logger = MagicMock()
    unit.afc = MagicMock()
    unit.afc.reactor.NEVER = 9e9
    unit.lanes = {lane.name: lane}
    unit._spool_map = {lane.name: 0}
    unit._operation_active = op_active
    unit._prev_states_stale = stale
    unit._last_f1s = [None] * 4 if last_f1s is None else list(last_f1s)
    unit._last_hub = [None] * 4
    unit._hub_load_suppressed = set()
    unit.oams = MagicMock()
    unit.oams.f1s_hes_value = f1s
    unit.oams.hub_hes_value = hub
    return unit


# ── _poll_oams_sensors ────────────────────────────────────────────────────────

def test_f1s_drives_prep_state():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True  # untouched while filament present
    lane.handle_load_runout.assert_not_called()


def test_f1s_loss_fires_runout_and_clears_staging():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is False
    assert lane.loaded_to_hub is False  # spool gone -> can't be staged
    lane.handle_load_runout.assert_called_once_with(100.0, False)
    assert unit._last_f1s[0] is False


def test_f1s_insert_fires_load():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])

    unit._poll_oams_sensors(100.0)

    lane.handle_load_runout.assert_called_once_with(100.0, True)


def test_resync_after_operation_suppresses_events():
    """First poll after a load/unload: the previous snapshot is stale — just
    re-sync it without firing insert/remove."""
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None], stale=True)

    unit._poll_oams_sensors(100.0)

    lane.handle_load_runout.assert_not_called()
    assert unit._last_f1s[0] is False       # re-synced
    assert unit._prev_states_stale is False  # consumed


def test_suppressed_load_marks_lane_suppressed():
    """A load event the unit itself caused (hub feed) is marked suppressed so
    handle_load_runout can tell it apart from a user insert."""
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])
    unit._hub_load_suppressed = {"lane0"}

    unit._poll_oams_sensors(100.0)

    assert lane._load_suppressed is True
    assert "lane0" not in unit._hub_load_suppressed  # consumed
    lane.handle_load_runout.assert_called_once_with(100.0, True)


def test_hub_hes_drives_raw_load_state():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[1, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane._load_state == 1


def test_poll_skipped_during_operation():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None], op_active=True)

    result = unit._poll_oams_sensors(100.0)

    assert result == 102.0  # re-schedules without touching state
    lane.handle_load_runout.assert_not_called()
    assert lane.loaded_to_hub is True


def test_poll_stops_without_hardware():
    lane = _make_lane("lane0")
    unit = _make_unit(lane, f1s=[], hub=[])
    unit.oams = None

    assert unit._poll_oams_sensors(100.0) == unit.afc.reactor.NEVER


# ── _advance_tool_stn_to_nozzle ───────────────────────────────────────────────

def _advance_unit():
    unit = afcAMS.__new__(afcAMS)
    unit.logger = MagicMock()
    unit.afc = MagicMock()
    unit._oams_extrude = MagicMock()
    return unit


def _advance_lane(tool_stn, load_speed=25.0):
    lane = MagicMock()
    lane.name = "lane0"
    lane.extruder_obj.tool_stn = tool_stn
    lane.extruder_obj.tool_load_speed = load_speed
    return lane


def test_tool_stn_advance_full_distance():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(100.0))
    unit._oams_extrude.assert_called_once_with(
        100.0, 25.0 * 60.0, "tool_stn_to_nozzle")


def test_tool_stn_advance_credits_engagement():
    """Engagement verification already pushed some filament past the sensor —
    only the remainder is extruded."""
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(100.0), already_advanced=30.0)
    assert unit._oams_extrude.call_args[0][0] == 70.0


def test_tool_stn_advance_noop_when_fully_covered():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(50.0), already_advanced=60.0)
    unit._oams_extrude.assert_not_called()


def test_tool_stn_advance_noop_without_tool_stn():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(0))
    unit._oams_extrude.assert_not_called()
