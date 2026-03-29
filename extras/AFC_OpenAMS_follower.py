# AFC OpenAMS Follower Controller
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Follower motor control, LED state, and rate-limited MCU command queue
for OpenAMS hardware units. Used by AFC_OpenAMS to manage the OAMS
follower motor that maintains filament tension in the buffer tube."""

from __future__ import annotations
from typing import Any, Dict, Optional


class FollowerState:
    """Track follower motor state for a single OAMS unit."""
    def __init__(self):
        self.coasting   = False
        self.last_state = None  # (enable, direction) to avoid redundant MCU commands


class FollowerController:
    """Controls OAMS follower motors, LED errors, and rate-limited MCU commands.

    Created and owned by AFC_OpenAMS. Operates on OAMS hardware objects
    directly — no AFC policy decisions, just hardware control.
    """

    def __init__(self, oams_dict, reactor, logger):
        """
        :param oams_dict: Dict of OAMS hardware objects keyed by name
        :param reactor: Klipper reactor for timers
        :param logger: AFC logger instance
        """
        self.oams = oams_dict
        self.reactor = reactor
        self.logger = logger
        self.follower_state: Dict[str, FollowerState] = {}
        self._mcu_command_queue: Dict[str, list] = {}
        self._mcu_command_in_flight: Dict[str, bool] = {}
        self._mcu_command_poll_timers: Dict[str, Any] = {}
        self._led_error_state: Dict[str, int] = {}

    def get_follower_state(self, oams_name):
        """Get or create follower state tracking for an OAMS unit."""
        if oams_name not in self.follower_state:
            self.follower_state[oams_name] = FollowerState()
        return self.follower_state[oams_name]

    def get_oams(self, oams_name):
        """Look up OAMS hardware object by name."""
        return self.oams.get(oams_name)

    def is_mcu_ready(self, oams):
        """Check if OAMS MCU is ready for commands."""
        if oams is None:
            return False
        try:
            mcu = getattr(oams, 'mcu', None)
            if mcu is None:
                return False
            if hasattr(mcu, 'is_shutdown'):
                return not mcu.is_shutdown()
            reactor = getattr(mcu, '_reactor', None) or self.reactor
            if hasattr(mcu, 'get_last_clock'):
                last_clock = mcu.get_last_clock()
                return last_clock is not None and last_clock > 0
            return True
        except Exception:
            return False

    # ---- Follower motor control ----

    def enable_follower(self, fps_state, oams, direction, context, force=False):
        """Enable follower motor in the given direction."""
        if oams is None:
            return
        direction = direction if direction in (0, 1) else 1
        oams_name = getattr(oams, 'name', None)
        if oams_name:
            self._set_follower_if_changed(oams_name, oams, 1, direction, context, force=force)

    def set_follower_state(self, fps_state, oams, enable, direction, context, force=False):
        """Set follower state directly."""
        if oams is None:
            return
        oams_name = getattr(oams, 'name', None)
        if not oams_name:
            return
        direction = direction if direction in (0, 1) else 1
        self._set_follower_if_changed(oams_name, oams, enable, direction, context, force=force)

    def _set_follower_if_changed(self, oams_name, oams, enable, direction, context, force=False):
        """Send follower MCU command only if state changed (or force=True)."""
        state = self.get_follower_state(oams_name)
        new_state = (enable, direction)

        if not force and state.last_state == new_state:
            return

        self.logger.debug(
            f"Follower command for {oams_name}: enable={enable} direction={direction} "
            f"context={context or 'no context'} forced={force}")

        try:
            oams.set_oams_follower(enable, direction)
            state.last_state = new_state
            self.logger.debug(f"Follower {'enabled' if enable else 'disabled'} for {oams_name} ({context})")
        except Exception as e:
            self.logger.error(f"Failed to set follower on {oams_name}: {e}")

    # ---- LED error control ----

    def set_led_error_if_changed(self, oams, oams_name, spool_idx, error_state, context=""):
        """Set LED error state on hardware, deduplicating repeated calls."""
        key = f"{oams_name}_{spool_idx}"
        current = self._led_error_state.get(key)
        if current == error_state:
            return
        self._led_error_state[key] = error_state
        try:
            self.rate_limited_mcu_command(oams_name, oams.set_led_error, spool_idx, error_state)
            self.logger.debug(
                f"LED error {'set' if error_state else 'cleared'} for {oams_name} "
                f"spool {spool_idx} ({context})")
        except Exception as e:
            self.logger.error(f"Failed to set LED error on {oams_name}: {e}")

    def clear_error_led(self, oams, oams_name, spool_idx, context=""):
        """Clear LED error state."""
        self.set_led_error_if_changed(oams, oams_name, spool_idx, 0, context)

    # ---- Rate-limited MCU command queue ----

    def rate_limited_mcu_command(self, oams_name, command_fn, *args, **kwargs):
        """Queue an MCU command with completion-aware rate limiting."""
        oams = self.oams.get(oams_name)
        if oams is None or not self.is_mcu_ready(oams):
            return

        if oams_name not in self._mcu_command_queue:
            self._mcu_command_queue[oams_name] = []
            self._mcu_command_in_flight[oams_name] = False

        self._mcu_command_queue[oams_name].append((command_fn, args, kwargs))
        self._process_mcu_command_queue(oams_name)

    def _process_mcu_command_queue(self, oams_name):
        """Process queued MCU commands, one at a time."""
        oams = self.oams.get(oams_name)
        if oams is None:
            return
        if self._mcu_command_in_flight.get(oams_name, False):
            return

        queue = self._mcu_command_queue.get(oams_name, [])
        if not queue:
            return

        if getattr(oams, "action_status", None) is not None:
            def _retry(eventtime):
                self._mcu_command_in_flight[oams_name] = False
                self._process_mcu_command_queue(oams_name)
                return self.reactor.NEVER

            if oams_name in self._mcu_command_poll_timers:
                try:
                    self.reactor.unregister_timer(self._mcu_command_poll_timers[oams_name])
                except Exception:
                    pass
            self._mcu_command_poll_timers[oams_name] = self.reactor.register_timer(
                _retry, self.reactor.NOW + 0.1)
            return

        command_fn, args, kwargs = queue.pop(0)
        self._mcu_command_in_flight[oams_name] = True
        try:
            command_fn(*args, **kwargs)
        except Exception as e:
            self.logger.error(f"MCU command failed for {oams_name}: {e}")

        def _done(eventtime):
            self._mcu_command_in_flight[oams_name] = False
            self._process_mcu_command_queue(oams_name)
            return self.reactor.NEVER
        self.reactor.register_timer(_done, self.reactor.NOW + 0.05)

    def cleanup(self):
        """Clean up timers on disconnect."""
        for oams_name, timer in list(self._mcu_command_poll_timers.items()):
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                pass
        self._mcu_command_poll_timers.clear()
        self._mcu_command_queue.clear()
        self._mcu_command_in_flight.clear()
