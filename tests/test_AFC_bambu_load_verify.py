# A load may only claim TOOL_LOADED on evidence that survives the advance.
#
# The status was set unconditionally after _advance_into_extruder, so a load
# reported success on the sensor trigger from BEFORE the advance -- up to a
# minute stale. Anything undoing the load during the advance was invisible.
#
# Seen on bridgebox with the link cut mid-feed: the sensor tripped, the advance
# ran with set_feed_assist dropped by the dead link, the extruder alone could
# not hold ~2.8m of bowden, the filament went back, and AFC still logged
# "lane28 is now loaded in toolhead". The operator found the toolhead empty.
from __future__ import annotations

import types

from extras.AFC_BambuAMS import afcBambuAMS


class _Recorder:
    def __init__(self): self.calls = []
    def __call__(self, *a, **k): self.calls.append((a, k))


def _lane(name="lane28"):
    return types.SimpleNamespace(name=name, loaded_to_hub=False, status=None,
                                 tool_loaded=False)


def _unit(sensor_after: bool, connected: bool = True):
    """A unit whose post-advance sensor reads `sensor_after`."""
    u = afcBambuAMS.__new__(afcBambuAMS)
    u.name = "Bambu_AMS_HT_1"
    u.logger = types.SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None,
                                     warning=lambda *a, **k: None)
    u._bridge = types.SimpleNamespace(is_connected=lambda: connected)
    u.stop = lambda *a, **k: None
    u.bridge_finish = lambda *a, **k: None
    u._ack_faults = lambda *a, **k: None
    u._advance_into_extruder = lambda *a, **k: None
    u._toolhead_sensor_triggered = lambda lane: sensor_after
    u._fault_lane = None
    u._resume_needs_reload = False
    return u


def _run_tail(u, lane, extruder, afc):
    """Drive just the epilogue under test: advance -> verify -> mark loaded."""
    u.afc = afc
    u.bridge_finish(lane)
    u._advance_into_extruder(lane, extruder)
    if not u._toolhead_sensor_triggered(lane):
        cause = "lost the sensor during the advance"
        u.stop()
        if afc.function.in_print():
            u._fault_lane = lane
            u._resume_needs_reload = True
        afc.error.handle_lane_failure(lane, cause, pause=afc.function.in_print())
        return False
    lane.loaded_to_hub = True
    lane.status = "TOOL_LOADED"
    return True


def _afc(in_print=False):
    return types.SimpleNamespace(
        function=types.SimpleNamespace(in_print=lambda: in_print),
        error=types.SimpleNamespace(handle_lane_failure=_Recorder()))


def test_the_real_module_verifies_the_sensor_after_the_advance():
    # Guards the actual source, not the harness: the check and its failure
    # path must be present in _unit_load_lane's epilogue.
    import inspect
    src = inspect.getsource(afcBambuAMS._unit_load_lane)
    i_adv = src.index("_advance_into_extruder(cur_lane")
    i_chk = src.index("_toolhead_sensor_triggered(cur_lane)", i_adv)
    i_set = src.index("AFCLaneState.TOOL_LOADED", i_adv)
    assert i_adv < i_chk < i_set, "the sensor must be re-read between the " \
                                  "advance and the TOOL_LOADED claim"


def test_a_lost_sensor_fails_the_load_instead_of_claiming_it():
    lane, afc = _lane(), _afc()
    u = _unit(sensor_after=False)
    assert _run_tail(u, lane, types.SimpleNamespace(tool_stn=50), afc) is False
    assert lane.status != "TOOL_LOADED"
    assert lane.loaded_to_hub is False
    assert afc.error.handle_lane_failure.calls


def test_a_held_sensor_still_completes_the_load():
    lane, afc = _lane(), _afc()
    u = _unit(sensor_after=True)
    assert _run_tail(u, lane, types.SimpleNamespace(tool_stn=50), afc) is True
    assert lane.status == "TOOL_LOADED"
    assert not afc.error.handle_lane_failure.calls


def test_a_failure_mid_print_marks_the_lane_for_reload():
    lane, afc = _lane(), _afc(in_print=True)
    u = _unit(sensor_after=False)
    assert _run_tail(u, lane, types.SimpleNamespace(tool_stn=50), afc) is False
    assert u._fault_lane is lane and u._resume_needs_reload is True
    assert afc.error.handle_lane_failure.calls[0][1]["pause"] is True
