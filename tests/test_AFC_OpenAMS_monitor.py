"""
Unit tests for the OAMSMonitor clog detection in extras/AFC_OpenAMS.py

Covers:
  - _check_clog fires on a genuine clog (extruder advances, encoder still,
    pressure at target, dwell elapsed) on a single extruder
  - _check_clog restarts its window when the active extruder changes
    mid-window (toolchange), instead of comparing position counters across
    two different extruder objects (phantom-advance false positive)
  - clog detection still works on the new extruder after a swap
  - cumulative encoder progress re-baselines the window (and records the
    extruder object it re-baselined on)
  - pressure outside the deadband resets clog tracking
  - post-load grace suppresses detection
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_OpenAMS import (
    OAMSMonitor,
    CLOG_PRESSURE_TARGET,
    CLOG_EXTRUSION_WINDOW,
    CLOG_DWELL,
    CLOG_ENCODER_SLACK,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

class _FakeExtruder:
    """Stands in for a Klipper extruder: identity + its own position counter."""

    def __init__(self, name, last_position=0.0):
        self.name = name
        self.last_position = last_position


class _FakeFps:
    """Minimal fps object: fps_value + an `extruder` attribute the test can
    swap to emulate a toolchange (the real property resolves the ACTIVE
    toolhead extruder)."""

    def __init__(self, extruder):
        self.extruder = extruder
        self.fps_value = CLOG_PRESSURE_TARGET


def _make_monitor(fps, on_clog=None):
    monitor = OAMSMonitor(
        fps_name="FPS_test",
        fps_obj=fps,
        reactor=MagicMock(),
        logger=MagicMock(),
        on_clog=on_clog,
        clog_sensitivity="medium",
        is_printing_fn=lambda: True,
    )
    # _check_clog reads st.last_encoder (normally fed by the timer loop)
    monitor.state.last_encoder = 0
    return monitor


def _run_clog_window(monitor, start_time=100.0, encoder_delta=0,
                     pressure=CLOG_PRESSURE_TARGET):
    """Drive _check_clog twice: once to open the window, once after the dwell
    with the extrusion window satisfied. Returns the second call's eventtime."""
    monitor._check_clog(start_time, encoder_delta, pressure)
    fire_time = start_time + CLOG_DWELL + 1.0
    monitor._check_clog(fire_time, encoder_delta, pressure)
    return fire_time


# ── Genuine clog on a single extruder ─────────────────────────────────────────

def test_clog_fires_same_extruder():
    """Extruder advances past the window, encoder still, pressure at target,
    dwell elapsed -> clog callback fires."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)  # opens window
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    on_clog.assert_called_once()
    assert monitor.state.clog_active is True
    msg = on_clog.call_args[0][1]
    assert "Clog detected" in msg


def test_clog_does_not_fire_below_extrusion_window():
    """Dwell elapsed but extruder advanced less than the window -> no clog."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW / 2
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    on_clog.assert_not_called()
    assert monitor.state.clog_active is False


# ── Extruder swap mid-window (the toolchange false positive) ─────────────────

def test_extruder_swap_restarts_window():
    """A toolchange mid-window swaps fps.extruder to a different object whose
    position counter differs arbitrarily. The window must restart on the new
    extruder instead of firing off the cross-extruder position delta."""
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1060.9)
    fps = _FakeFps(extruder_a)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    # Window opens while extruder A is active
    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_time is not None
    assert monitor.state.clog_start_extruder_obj is extruder_a

    # Toolchange: active extruder becomes B (counters differ by 60.9mm)
    fps.extruder = extruder_b
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    # No false clog; the window restarted on B
    on_clog.assert_not_called()
    assert monitor.state.clog_active is False
    assert monitor.state.clog_start_extruder_obj is extruder_b
    assert monitor.state.clog_start_extruder == extruder_b.last_position


def test_clog_fires_on_new_extruder_after_swap():
    """After a swap restarts the window, a genuine clog on the NEW extruder
    must still be detected."""
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=0.0)
    fps = _FakeFps(extruder_a)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    fps.extruder = extruder_b
    monitor._check_clog(110.0, 0, CLOG_PRESSURE_TARGET)   # restart on B
    on_clog.assert_not_called()

    # Now B genuinely clogs: advances past the window with encoder still
    extruder_b.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(110.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    on_clog.assert_called_once()
    assert monitor.state.clog_active is True


def test_swap_back_and_forth_never_fires_without_advance():
    """Repeated toolchanges with no real extrusion never accumulate into a
    clog, no matter how the two counters differ."""
    extruder_a = _FakeExtruder("extruder", last_position=500.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1234.5)
    fps = _FakeFps(extruder_a)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    eventtime = 100.0
    for _ in range(5):
        monitor._check_clog(eventtime, 0, CLOG_PRESSURE_TARGET)
        eventtime += CLOG_DWELL + 1.0
        fps.extruder = extruder_b if fps.extruder is extruder_a else extruder_a

    on_clog.assert_not_called()
    assert monitor.state.clog_active is False


# ── Window re-baselining and resets ───────────────────────────────────────────

def test_cumulative_encoder_progress_restarts_window():
    """Cumulative encoder movement past the slack means filament IS flowing:
    the window re-baselines (including the extruder object) instead of firing."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    baseline_time = monitor.state.clog_start_time

    # Encoder made real cumulative progress since the window opened; each
    # per-check delta stays within slack (burst feeding) so the condition
    # branch is still entered.
    monitor.state.last_encoder = CLOG_ENCODER_SLACK * 3
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    fire_time = 100.0 + CLOG_DWELL + 1.0
    monitor._check_clog(fire_time, 0, CLOG_PRESSURE_TARGET)

    on_clog.assert_not_called()
    assert monitor.state.clog_start_time == fire_time  # re-baselined
    assert monitor.state.clog_start_time != baseline_time
    assert monitor.state.clog_start_extruder_obj is extruder
    assert monitor.state.clog_start_encoder == monitor.state.last_encoder


def test_pressure_out_of_band_resets_tracking():
    """Pressure away from target means the extruder is not pushing against a
    blockage -> clog tracking resets."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_time is not None

    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, 0.2)  # tension, not target

    on_clog.assert_not_called()
    assert monitor.state.clog_start_time is None


def test_encoder_moving_resets_tracking():
    """A per-check encoder delta above the slack means filament is flowing
    -> clog tracking resets."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0,
                        CLOG_ENCODER_SLACK + 1, CLOG_PRESSURE_TARGET)

    on_clog.assert_not_called()
    assert monitor.state.clog_start_time is None


def test_post_load_grace_suppresses_detection():
    """Within clog_post_load_grace of a lane change, no window is opened."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    on_clog = MagicMock()
    monitor = _make_monitor(fps, on_clog=on_clog)

    monitor.state.last_lane_change_time = 100.0
    monitor._check_clog(100.0 + monitor.clog_post_load_grace - 1.0,
                        0, CLOG_PRESSURE_TARGET)

    assert monitor.state.clog_start_time is None
    on_clog.assert_not_called()


def test_state_reset_clears_extruder_obj():
    """FPSState.reset() clears the baselined extruder object with the rest of
    the clog window state."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    fps = _FakeFps(extruder)
    monitor = _make_monitor(fps)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_extruder_obj is extruder

    monitor.state.reset()
    assert monitor.state.clog_start_extruder_obj is None
    assert monitor.state.clog_start_time is None


# ── Stuck spool detection ─────────────────────────────────────────────────────

from extras.AFC_OpenAMS import (  # noqa: E402
    STUCK_PRESSURE_LOW,
    STUCK_PRESSURE_CLEAR,
    STUCK_DWELL,
    STUCK_MIN_ENCODER,
)


def _make_stuck_monitor():
    fps = _FakeFps(_FakeExtruder("extruder"))
    on_stuck = MagicMock()
    on_cleared = MagicMock()
    monitor = _make_monitor(fps)
    monitor._on_stuck_spool = on_stuck
    monitor._on_stuck_cleared = on_cleared
    return monitor, on_stuck, on_cleared


def test_stuck_spool_fires_after_dwell():
    monitor, on_stuck, _ = _make_stuck_monitor()
    low = STUCK_PRESSURE_LOW - 0.01

    monitor._check_stuck_spool(100.0, 0, low)                  # opens dwell
    on_stuck.assert_not_called()
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, low)

    on_stuck.assert_called_once()
    assert monitor.state.stuck_active is True
    assert "Stuck spool" in on_stuck.call_args[0][1]


def test_stuck_spool_no_fire_when_encoder_moving():
    monitor, on_stuck, _ = _make_stuck_monitor()
    low = STUCK_PRESSURE_LOW - 0.01

    monitor._check_stuck_spool(100.0, STUCK_MIN_ENCODER, low)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, STUCK_MIN_ENCODER, low)

    on_stuck.assert_not_called()


def test_stuck_spool_no_fire_when_pressure_ok():
    monitor, on_stuck, _ = _make_stuck_monitor()

    monitor._check_stuck_spool(100.0, 0, STUCK_PRESSURE_LOW + 0.05)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0,
                               STUCK_PRESSURE_LOW + 0.05)

    on_stuck.assert_not_called()


def test_stuck_spool_clears_on_pressure_recovery():
    monitor, on_stuck, on_cleared = _make_stuck_monitor()
    low = STUCK_PRESSURE_LOW - 0.01

    monitor._check_stuck_spool(100.0, 0, low)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, low)
    assert monitor.state.stuck_active is True

    monitor._check_stuck_spool(110.0, 0, STUCK_PRESSURE_CLEAR + 0.01)

    on_cleared.assert_called_once()
    assert monitor.state.stuck_active is False
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_engagement_grace_suppresses():
    monitor, on_stuck, _ = _make_stuck_monitor()
    monitor.state.engagement_checked_at = 100.0
    low = STUCK_PRESSURE_LOW - 0.01

    monitor._check_stuck_spool(101.0, 0, low)  # within 6s grace

    assert monitor.state.stuck_start_time is None
    on_stuck.assert_not_called()
