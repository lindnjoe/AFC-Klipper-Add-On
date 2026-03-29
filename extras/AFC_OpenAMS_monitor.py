# AFC OpenAMS Monitor — Real-time filament monitoring for OpenAMS units
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Monitors encoder movement and FPS pressure during printing to detect
stuck spools, clogs, and runout conditions. Owned by AFC_OpenAMS — not
a standalone klipper module.

Design principles:
- AFC is master control. The monitor reports problems to AFC, doesn't
  make policy decisions (no direct PAUSE, no state writes).
- Reads from hardware (encoder, FPS, hub sensors) via oams.py objects.
- Reports via callbacks to AFC_OpenAMS which decides what to do.
"""

from __future__ import annotations
from typing import Any, Callable, Dict, Optional


# ---- Constants ----

MONITOR_INTERVAL = 2.0          # Seconds between monitoring checks
MONITOR_INTERVAL_IDLE = 4.0     # Slower interval when not printing

# Stuck spool detection
STUCK_PRESSURE_LOW = 0.08       # FPS below this + encoder stopped = stuck
STUCK_PRESSURE_CLEAR = 0.12     # FPS must rise above this to clear stuck
STUCK_DWELL = 2.0               # Seconds condition must persist before firing
STUCK_LOAD_GRACE = 8.0          # Grace period after load completes
STUCK_MIN_ENCODER = 3           # Minimum encoder clicks per check to be "moving"

# Clog detection
CLOG_PRESSURE_TARGET = 0.50     # Expected FPS pressure during normal printing
CLOG_PRESSURE_BAND = 0.06       # Deadband around target (no clog if pressure varies)
CLOG_EXTRUSION_WINDOW = 24.0    # mm of extrusion before checking for clog
CLOG_DWELL = 10.0               # Seconds clog condition must persist
CLOG_POST_LOAD_GRACE = 12.0     # Grace after load before clog detection starts
CLOG_ENCODER_SLACK = 8          # Encoder clicks of slack allowed

# Sensitivity presets
CLOG_SENSITIVITY = {
    'off': None,
    'low': 1.5,
    'medium': 1.0,
    'high': 0.7,
}


class FPSLoadState:
    """Load state for an FPS buffer channel."""
    UNLOADED = 0
    LOADED = 1
    LOADING = 2
    UNLOADING = 3


class FPSState:
    """Tracking state for one FPS buffer channel."""
    def __init__(self):
        # Core state
        self.state = FPSLoadState.UNLOADED
        self.current_lane = None
        self.current_oams = None
        self.current_spool_idx = None
        self.since = None

        # Encoder tracking
        self.last_encoder = None

        # Stuck spool detection
        self.stuck_active = False
        self.stuck_start_time = None

        # Clog detection
        self.clog_active = False
        self.clog_start_time = None
        self.clog_start_extruder = None
        self.clog_start_encoder = None

        # Engagement tracking (suppress detection during engagement)
        self.engagement_in_progress = False
        self.engagement_checked_at = None

        # Lane change tracking
        self.last_lane_change_time = None

    def reset(self):
        """Reset to unloaded state."""
        self.state = FPSLoadState.UNLOADED
        self.current_lane = None
        self.current_oams = None
        self.current_spool_idx = None
        self.since = None
        self.stuck_active = False
        self.stuck_start_time = None
        self.clog_active = False
        self.clog_start_time = None
        self.clog_start_extruder = None
        self.clog_start_encoder = None
        self.engagement_in_progress = False

    def clear_encoder_samples(self):
        self.last_encoder = None


class OAMSMonitor:
    """Real-time filament monitoring for a single OAMS/FPS channel.

    Runs on a reactor timer during printing. Checks encoder movement
    and FPS pressure to detect stuck spools and clogs.

    Reports problems via callbacks — does NOT pause or modify AFC state.
    """

    def __init__(self, fps_name, fps_obj, reactor, logger,
                 on_stuck_spool=None, on_clog=None, on_stuck_cleared=None,
                 clog_sensitivity='medium', is_printing_fn=None,
                 is_lane_loaded_fn=None):
        """
        :param fps_name: FPS buffer name (e.g. 'FPS_buffer1')
        :param fps_obj: AFC_FPS buffer object (for ADC readings)
        :param reactor: Klipper reactor for timers
        :param logger: AFC logger
        :param on_stuck_spool: Callback(fps_name, message) when stuck detected
        :param on_clog: Callback(fps_name, message) when clog detected
        :param on_stuck_cleared: Callback(fps_name) when stuck condition clears
        :param clog_sensitivity: 'off', 'low', 'medium', 'high'
        :param is_printing_fn: Callable returning True when printer is actively printing
        :param is_lane_loaded_fn: Callable returning True when a lane is loaded to toolhead
        """
        self.fps_name = fps_name
        self.fps = fps_obj
        self.reactor = reactor
        self.logger = logger

        self._on_stuck_spool = on_stuck_spool
        self._on_clog = on_clog
        self._on_stuck_cleared = on_stuck_cleared
        self._is_printing = is_printing_fn
        self._is_lane_loaded = is_lane_loaded_fn

        self.clog_multiplier = CLOG_SENSITIVITY.get(clog_sensitivity, 1.0)
        self.enable_clog = self.clog_multiplier is not None

        self.state = FPSState()
        self._timer = None
        self._running = False
        self._oams = None  # Set when lane loads

        # Configurable thresholds (can be overridden)
        self.stuck_pressure_low = STUCK_PRESSURE_LOW
        self.stuck_pressure_clear = STUCK_PRESSURE_CLEAR
        self.stuck_dwell = STUCK_DWELL
        self.stuck_load_grace = STUCK_LOAD_GRACE
        self.stuck_min_encoder = STUCK_MIN_ENCODER
        self.clog_dwell = CLOG_DWELL * (self.clog_multiplier or 1.0)
        self.clog_extrusion_window = CLOG_EXTRUSION_WINDOW
        self.clog_post_load_grace = CLOG_POST_LOAD_GRACE

    # ---- Lifecycle ----

    def start(self, oams_obj):
        """Start monitoring for the given OAMS hardware object."""
        self._oams = oams_obj
        self._running = True
        # Reset detection state to prevent stale triggers
        self.state.stuck_start_time = None
        self.state.clog_start_time = None
        self.state.clog_start_extruder = None
        self.state.clog_start_encoder = None
        self.state.clear_encoder_samples()
        if self._timer is None:
            self._timer = self.reactor.register_timer(
                self._monitor_tick, self.reactor.NOW + MONITOR_INTERVAL)
        self.logger.debug(f"Monitor started for {self.fps_name}")

    def stop(self):
        """Stop monitoring."""
        self._running = False
        if self._timer is not None:
            self.reactor.unregister_timer(self._timer)
            self._timer = None
        self.logger.debug(f"Monitor stopped for {self.fps_name}")

    def notify_load_complete(self, lane_name, oams_name, spool_idx):
        """Called by AFC_OpenAMS after successful load."""
        self.state.state = FPSLoadState.LOADED
        self.state.current_lane = lane_name
        self.state.current_oams = oams_name
        self.state.current_spool_idx = spool_idx
        self.state.since = self.reactor.monotonic()
        self.state.last_lane_change_time = self.state.since
        self.state.stuck_active = False
        self.state.clog_active = False
        self.state.clear_encoder_samples()

    def notify_unload_complete(self):
        """Called by AFC_OpenAMS after successful unload."""
        self.state.reset()

    def notify_engagement_start(self):
        """Suppress detection during engagement verification."""
        self.state.engagement_in_progress = True

    def notify_engagement_end(self):
        """Resume detection after engagement verification."""
        self.state.engagement_in_progress = False
        self.state.engagement_checked_at = self.reactor.monotonic()

    # ---- Main monitoring loop ----

    def _monitor_tick(self, eventtime):
        """Reactor timer callback — runs every MONITOR_INTERVAL."""
        if not self._running or self._oams is None:
            return self.reactor.NEVER

        st = self.state
        if st.state != FPSLoadState.LOADED:
            return eventtime + MONITOR_INTERVAL_IDLE

        # Verify a lane is ACTUALLY loaded to toolhead. If state got
        # out of sync (e.g. unload didn't go through _oams_unload),
        # auto-stop the monitor to prevent false detections.
        if self._is_lane_loaded and not self._is_lane_loaded():
            self.logger.debug(
                f"{self.fps_name}: no lane loaded to toolhead, stopping monitor")
            self._running = False
            self.state.reset()
            return self.reactor.NEVER

        # Only run detection while actively printing — skip during
        # idle, PRINT_START, homing, probing, manual moves, etc.
        if self._is_printing and not self._is_printing():
            # Reset clog tracking so it doesn't accumulate during non-print
            st.clog_start_time = None
            st.clog_start_extruder = None
            return eventtime + MONITOR_INTERVAL_IDLE

        # Don't monitor during engagement verification
        if st.engagement_in_progress:
            return eventtime + MONITOR_INTERVAL

        # Grace period after load
        if st.since and (eventtime - st.since) < self.stuck_load_grace:
            return eventtime + MONITOR_INTERVAL

        try:
            encoder = self._oams.encoder_clicks
            pressure = float(getattr(self.fps, 'fps_value', 0.5))
            hub_values = getattr(self._oams, 'hub_hes_value', None)

            # Calculate encoder delta
            encoder_delta = 0
            if st.last_encoder is not None:
                encoder_delta = abs(encoder - st.last_encoder)
            st.last_encoder = encoder

            # Run detection checks
            self._check_stuck_spool(eventtime, encoder_delta, pressure)

            if self.enable_clog:
                self._check_clog(eventtime, encoder_delta, pressure)

        except Exception as e:
            self.logger.error(f"Monitor error on {self.fps_name}: {e}")

        return eventtime + MONITOR_INTERVAL

    # ---- Stuck spool detection ----

    def _check_stuck_spool(self, eventtime, encoder_delta, pressure):
        """Detect stuck spool: encoder stopped + FPS pressure low.

        When the spool is jammed, the follower can't push filament so
        encoder stops and FPS pressure drops (no tension in buffer).
        """
        st = self.state

        # Grace after engagement check
        if st.engagement_checked_at and (eventtime - st.engagement_checked_at) < 6.0:
            return

        encoder_moving = encoder_delta >= self.stuck_min_encoder
        pressure_low = pressure <= self.stuck_pressure_low

        if not encoder_moving and pressure_low:
            # Condition met — start or continue dwell timer
            if not st.stuck_active:
                if st.stuck_start_time is None:
                    st.stuck_start_time = eventtime
                    self.logger.debug(
                        f"{self.fps_name}: stuck spool timer started "
                        f"(pressure={pressure:.2f}, encoder_delta={encoder_delta})")
                elif (eventtime - st.stuck_start_time) >= self.stuck_dwell:
                    # Dwell exceeded — fire callback
                    st.stuck_active = True
                    msg = (
                        f"Stuck spool on {self.fps_name}: encoder stopped "
                        f"({encoder_delta} clicks), FPS pressure {pressure:.2f}")
                    self.logger.info(msg)
                    if self._on_stuck_spool:
                        self._on_stuck_spool(self.fps_name, msg)
        else:
            # Condition cleared
            if st.stuck_start_time is not None or st.stuck_active:
                if pressure >= self.stuck_pressure_clear or encoder_moving:
                    if st.stuck_active and self._on_stuck_cleared:
                        self._on_stuck_cleared(self.fps_name)
                    st.stuck_active = False
                    st.stuck_start_time = None

    # ---- Clog detection ----

    def _check_clog(self, eventtime, encoder_delta, pressure):
        """Detect clog: extruder advancing but encoder not tracking.

        When filament is clogged between the extruder and spool, the
        extruder keeps pushing (pressure stays at target) but the encoder
        doesn't move (filament isn't flowing through the buffer).
        """
        st = self.state

        # Grace after load
        if st.last_lane_change_time and \
           (eventtime - st.last_lane_change_time) < self.clog_post_load_grace:
            return

        # Need extruder position to detect clog
        try:
            extruder_pos = self.fps.extruder.last_position if hasattr(self.fps, 'extruder') else None
        except Exception:
            extruder_pos = None
        if extruder_pos is None:
            return

        # Pressure near target (extruder is pushing normally)
        pressure_near_target = abs(pressure - CLOG_PRESSURE_TARGET) <= CLOG_PRESSURE_BAND

        # Encoder not moving despite extrusion
        encoder_stuck = encoder_delta <= CLOG_ENCODER_SLACK

        if pressure_near_target and encoder_stuck:
            if st.clog_start_time is None:
                st.clog_start_time = eventtime
                st.clog_start_extruder = extruder_pos
                st.clog_start_encoder = st.last_encoder
            else:
                elapsed = eventtime - st.clog_start_time
                extrusion_delta = abs(extruder_pos - st.clog_start_extruder)

                if elapsed >= self.clog_dwell and extrusion_delta >= self.clog_extrusion_window:
                    # Clog confirmed
                    if not st.clog_active:
                        st.clog_active = True
                        encoder_total = abs((st.last_encoder or 0) - (st.clog_start_encoder or 0))
                        msg = (
                            f"Clog detected on {self.fps_name}: "
                            f"extruder advanced {extrusion_delta:.1f}mm, "
                            f"encoder moved {encoder_total} clicks, "
                            f"FPS pressure {pressure:.2f} "
                            f"(dwell {elapsed:.1f}s)")
                        self.logger.info(msg)
                        if self._on_clog:
                            self._on_clog(self.fps_name, msg)
        else:
            # Reset clog tracking
            st.clog_start_time = None
            st.clog_start_extruder = None
            st.clog_start_encoder = None
            if st.clog_active:
                st.clog_active = False
