"""
Unit tests for afcACE._dispatch_load_runout in extras/AFC_ACE.py

It exists because of a real failure on a printing ACE unit. A runout ran
TOOL_UNLOAD, whose check_absolute_mode flipped gcode_move.absolute_extrude to
True, and an in-flight relative E value was then read as an absolute target --
a -517.780mm move that tripped max_extrude_only_distance and cancelled the
print. The cancel unhomed the axes, so the unload's own first move raised
"Must home X axis first", and THAT exception disappeared into the serial
transport's catch-all, leaving the lane latched TOOL_UNLOADING with nothing
logged anywhere a user would look.

The move itself is fixed at source in AFC_functions.check_absolute_mode (see
tests/test_AFC_functions.py); what is tested here is the second half -- that a
runout handler which fails can no longer fail silently.

Covered here:
  - the load and runout directions both reach handle_load_runout
  - a handler that raises is reported through AFC's logger AND AFC_error
  - a broken error path still cannot re-raise into the transport
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE
from extras.AFC_lane import AFCLaneState

from tests.ace_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeLane,
    FakeLogger,
    Recorder,
)


def _unit(printing=False):
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.function.printing = printing
    return unit


def _lane(name="lane0"):
    return FakeLane(name, extruder_obj=FakeExtruderObj("extruder"),
                    status=AFCLaneState.TOOLED)


def test_insert_reaches_the_lane_handler():
    unit, lane = _unit(printing=True), _lane()

    unit._dispatch_load_runout(lane, 12.0, True)

    assert lane.handle_load_runout.call_count == 1
    assert lane.handle_load_runout.last_args == (12.0, True)
    assert not unit.afc.error.AFC_error.called


def test_runout_reaches_the_lane_handler():
    # No gating on the print stream: AFC's own _perform_pause_runout issues the
    # pause as its first action, and the E-origin re-zero in
    # check_absolute_mode is what makes the unload safe either way.
    unit, lane = _unit(printing=True), _lane()

    unit._dispatch_load_runout(lane, 12.0, False)

    assert lane.handle_load_runout.call_count == 1
    assert lane.handle_load_runout.last_args == (12.0, False)
    assert not unit.afc.error.AFC_error.called


def test_handler_exception_is_logged_and_raised_as_an_afc_error():
    unit, lane = _unit(printing=False), _lane()
    lane.handle_load_runout = Recorder(
        raises=RuntimeError("Must home X axis first"))

    unit._dispatch_load_runout(lane, 12.0, False)

    # Logged with a traceback through AFC's logger...
    assert unit.logger.lines["error"], "the failure must reach AFC's logger"
    logged = unit.logger.lines["error"][-1]
    assert "lane0" in logged
    assert "Must home X axis first" in logged
    # ...and surfaced to the user as an AFC error naming the toolhead risk.
    assert unit.afc.error.AFC_error.call_count == 1
    msg = unit.afc.error.AFC_error.last_args[0]
    assert "Must home X axis first" in msg
    assert "toolhead" in msg


def test_insert_failure_is_reported_as_a_load_not_a_runout():
    unit, lane = _unit(printing=False), _lane()
    lane.handle_load_runout = Recorder(raises=RuntimeError("boom"))

    unit._dispatch_load_runout(lane, 12.0, True)

    assert "load handling failed" in unit.logger.lines["error"][-1]


def test_handler_exception_does_not_escape_to_the_transport():
    unit, lane = _unit(printing=False), _lane()
    lane.handle_load_runout = Recorder(raises=RuntimeError("boom"))
    unit.afc.error.AFC_error = Recorder(raises=RuntimeError("no error obj"))

    # Even a broken error path must not re-raise into _handle_response, which
    # would put us right back to a silently swallowed failure.
    unit._dispatch_load_runout(lane, 12.0, False)

    assert unit.logger.lines["error"]
