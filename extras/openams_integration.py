# Armored Turtle Automated Filament Changer (OPTIMIZED)
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Shared OpenAMS integration helpers used by AFC and OpenAMS."""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple


class AMSHardwareService:
    """Centralised interface for accessing OpenAMS hardware from AFC.

    The service tracks the underlying ``OAMS`` firmware object created by
    ``klipper_openams`` and exposes high level helpers so AFC can interact with
    the same hardware instance without reimplementing any low level MCU
    messaging.
    """

    _instances: Dict[Tuple[int, str], "AMSHardwareService"] = {}

    def __init__(self, printer, name: str, logger: Optional[logging.Logger] = None):
        self.printer = printer
        self.name = name
        self.logger = logger or logging.getLogger(f"AFC.AMS.{name}")
        self._controller = None
        self._lock = threading.RLock()
        self._status: Dict[str, Any] = {}
        self._lane_snapshots: Dict[str, Dict[str, Any]] = {}
        self._status_callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._lanes_by_spool: Dict[Tuple[str, int], str] = {}
        
        # Cache reactor reference
        self._reactor = None

    @classmethod
    def for_printer(cls, printer, name: str = "default", logger: Optional[logging.Logger] = None) -> "AMSHardwareService":
        """Return the singleton service for the provided printer/name pair."""
        key = (id(printer), name)
        try:
            service = cls._instances[key]
        except KeyError:
            service = cls(printer, name, logger)
            cls._instances[key] = service
        else:
            if logger is not None:
                service.logger = logger
        return service

    def attach_controller(self, controller: Any) -> None:
        """Attach the low level ``OAMS`` controller to this service."""
        with self._lock:
            self._controller = controller
        if controller is not None:
            try:
                status = controller.get_status(self._monotonic())
            except Exception:
                status = None
            if status:
                self._update_status(status)
            self.logger.debug("Attached OAMS controller %s", controller)

    def resolve_controller(self) -> Optional[Any]:
        """Return the currently attached controller, attempting lookup if needed."""
        with self._lock:
            controller = self._controller
        if controller is not None:
            return controller

        lookup_name = f"oams {self.name}"
        try:
            controller = self.printer.lookup_object(lookup_name, None)
        except Exception:
            controller = None
        if controller is not None:
            self.attach_controller(controller)
        return controller

    def _monotonic(self) -> float:
        """Cache reactor reference for faster lookups."""
        if self._reactor is None:
            reactor = getattr(self.printer, "get_reactor", None)
            if callable(reactor):
                try:
                    self._reactor = reactor()
                except Exception:
                    pass
            if self._reactor is None:
                try:
                    self._reactor = self.printer.get_reactor()
                except Exception:
                    return 0.0
        
        try:
            return self._reactor.monotonic()
        except Exception:
            return 0.0

    def poll_status(self) -> Optional[Dict[str, Any]]:
        """Query the controller for its latest status snapshot."""
        controller = self.resolve_controller()
        if controller is None:
            return None

        eventtime = self._monotonic()
        try:
            status = controller.get_status(eventtime)
        except Exception:
            #  Direct attribute access fallback
            status = {
                "current_spool": getattr(controller, "current_spool", None),
                "f1s_hes_value": list(getattr(controller, "f1s_hes_value", []) or []),
                "hub_hes_value": list(getattr(controller, "hub_hes_value", []) or []),
                "fps_value": getattr(controller, "fps_value", 0.0),
            }
            encoder = getattr(controller, "encoder_clicks", None)
            if encoder is not None:
                status["encoder_clicks"] = encoder
        self._update_status(status)
        return status

    def _update_status(self, status: Dict[str, Any]) -> None:
        """Update cached status and notify observers."""
        with self._lock:
            self._status = dict(status)
            callbacks = list(self._status_callbacks)
        
        # Only call callbacks if there are any registered
        if callbacks:
            status_copy = dict(status)
            for callback in callbacks:
                try:
                    callback(status_copy)
                except Exception:
                    self.logger.exception("AMS status observer failed for %s", self.name)

    def latest_status(self) -> Dict[str, Any]:
        """Return the most recently cached status snapshot."""
        with self._lock:
            return dict(self._status)

    def register_status_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Register a callback to be notified of status updates."""
        with self._lock:
            if callback not in self._status_callbacks:
                self._status_callbacks.append(callback)

    def unregister_status_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Unregister a previously registered status callback."""
        with self._lock:
            if callback in self._status_callbacks:
                self._status_callbacks.remove(callback)

    def update_lane_snapshot(self, unit_name: str, lane_name: str, lane_state: bool, hub_state: Optional[bool], eventtime: float, *, spool_index: Optional[int] = None, tool_state: Optional[bool] = None) -> None:
        """Update the cached state snapshot for a specific lane."""
        key = f"{unit_name}:{lane_name}"
        with self._lock:
            self._lane_snapshots[key] = {
                "unit": unit_name,
                "lane": lane_name,
                "lane_state": bool(lane_state),
                "hub_state": None if hub_state is None else bool(hub_state),
                "timestamp": eventtime,
            }
            if spool_index is not None:
                try:
                    normalized_index = int(spool_index)
                except (TypeError, ValueError):
                    normalized_index = None
                else:
                    self._lane_snapshots[key]["spool_index"] = normalized_index
                    self._lanes_by_spool[(unit_name, normalized_index)] = lane_name
            if tool_state is not None:
                self._lane_snapshots[key]["tool_state"] = bool(tool_state)

    def latest_lane_snapshot(self, unit_name: str, lane_name: str) -> Optional[Dict[str, Any]]:
        """Return the most recent state snapshot for a specific lane."""
        key = f"{unit_name}:{lane_name}"
        with self._lock:
            snapshot = self._lane_snapshots.get(key)
        return dict(snapshot) if snapshot else None

    def resolve_lane_for_spool(self, unit_name: str, spool_index: Optional[int]) -> Optional[str]:
        """Map a spool index to its corresponding lane name."""
        if spool_index is None:
            return None
        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None
        key = (unit_name, normalized)
        with self._lock:
            return self._lanes_by_spool.get(key)

    def latest_lane_snapshot_for_spool(self, unit_name: str, spool_index: Optional[int]) -> Optional[Dict[str, Any]]:
        """Return the most recent state snapshot for a spool by its index."""
        lane_name = self.resolve_lane_for_spool(unit_name, spool_index)
        if lane_name is None:
            return None
        return self.latest_lane_snapshot(unit_name, lane_name)

    def _require_controller(self):
        """Ensure a controller is available, raising if not."""
        controller = self.resolve_controller()
        if controller is None:
            raise RuntimeError(f"OpenAMS controller '{self.name}' is not ready")
        return controller

    def load_spool(self, spool_index: int) -> None:
        """Command the OAMS to load a specific spool."""
        controller = self._require_controller()
        controller.oams_load_spool_cmd.send([spool_index])

    def unload_spool(self) -> None:
        """Command the OAMS to unload the current spool."""
        controller = self._require_controller()
        controller.oams_unload_spool_cmd.send([])

    def set_follower(self, enable: bool, direction: int) -> None:
        """Enable or disable the OAMS follower motor."""
        controller = self._require_controller()
        controller.oams_follower_cmd.send([1 if enable else 0, direction])

    def set_led_error(self, idx: int, value: int) -> None:
        """Set the error LED state for a specific spool bay."""
        controller = self._require_controller()
        controller.set_led_error(idx, value)


class AMSRunoutCoordinator:
    """Coordinates runout events between OpenAMS and AFC """

    _units: Dict[Tuple[int, str], List[Any]] = {}
    _monitors: Dict[Tuple[int, str], List[Any]] = {}
    _lock = threading.RLock()

    @classmethod
    def _key(cls, printer, name: str) -> Tuple[int, str]:
        """Generate a unique key for printer/name combinations."""
        return (id(printer), name)

    @classmethod
    def register_afc_unit(cls, unit) -> AMSHardwareService:
        """Register an ``afcAMS`` unit as participating in AMS integration."""
        service = AMSHardwareService.for_printer(unit.printer, unit.oams_name, unit.logger)
        key = cls._key(unit.printer, unit.oams_name)
        with cls._lock:
            cls._units.setdefault(key, [])
            if unit not in cls._units[key]:
                cls._units[key].append(unit)
        return service

    @classmethod
    def register_runout_monitor(cls, monitor) -> AMSHardwareService:
        """Register an OpenAMS runout monitor and return the hardware service."""
        printer = getattr(monitor, "printer", None)
        state = getattr(monitor, "fps_state", None)
        oams_name = getattr(state, "current_oams", None) if state else None
        if not oams_name:
            oams_name = getattr(monitor, "fps_name", "default")
        key = cls._key(printer, oams_name)
        with cls._lock:
            cls._monitors.setdefault(key, [])
            if monitor not in cls._monitors[key]:
                cls._monitors[key].append(monitor)
        return AMSHardwareService.for_printer(printer, oams_name)

    @classmethod
    def notify_runout_detected(cls, monitor, spool_index: Optional[int], *, lane_name: Optional[str] = None) -> None:
        """Forward runout detection from OpenAMS to any registered AFC units."""
        printer = getattr(monitor, "printer", None)
        state = getattr(monitor, "fps_state", None)
        oams_name = getattr(state, "current_oams", None) if state else None
        if not oams_name:
            oams_name = getattr(monitor, "fps_name", "default")
        key = cls._key(printer, oams_name)
        with cls._lock:
            units = list(cls._units.get(key, ()))
        lane_hint = lane_name or getattr(monitor, "latest_lane_name", None)
        for unit in units:
            try:
                unit.handle_runout_detected(spool_index, monitor, lane_name=lane_hint)
            except Exception:
                unit.logger.exception("Failed to propagate OpenAMS runout to AFC unit %s", unit.name)

    @classmethod
    def notify_afc_error(cls, printer, name: str, message: str, *, pause: bool = False) -> None:
        """Deliver an OpenAMS pause/error message to any registered AFC units."""
        key = cls._key(printer, name)
        with cls._lock:
            units = list(cls._units.get(key, ()))

        for unit in units:
            afc = getattr(unit, "afc", None)
            if afc is None:
                continue

            error_obj = getattr(afc, "error", None)
            if error_obj is None:
                continue

            try:
                error_obj.AFC_error(message, pause=pause, level=3)
            except Exception:
                logger = getattr(unit, "logger", None)
                if logger is None:
                    logger = logging.getLogger(__name__)
                logger.exception("Failed to deliver OpenAMS error '%s' to AFC unit %s", message, unit)

    @classmethod
    def notify_lane_tool_state(cls, printer, name: str, lane_name: str, *, loaded: bool, spool_index: Optional[int] = None, eventtime: Optional[float] = None) -> bool:
        """Propagate lane tool state changes from OpenAMS into AFC."""
        key = cls._key(printer, name)
        with cls._lock:
            units = list(cls._units.get(key, ()))

        if not units:
            return False

        if eventtime is None:
            try:
                eventtime = printer.get_reactor().monotonic()
            except Exception:
                eventtime = None

        handled = False
        for unit in units:
            try:
                if unit.handle_openams_lane_tool_state(lane_name, loaded, spool_index=spool_index, eventtime=eventtime):
                    handled = True
            except Exception:
                unit.logger.exception("Failed to update AFC lane %s from OpenAMS tool state", lane_name)
        return handled

    @classmethod
    def active_units(cls, printer, name: str) -> Iterable[Any]:
        """Return all AFC units registered for a specific OpenAMS instance."""
        key = cls._key(printer, name)
        with cls._lock:
            return tuple(cls._units.get(key, ()))

    @classmethod
    def active_monitors(cls, printer, name: str) -> Iterable[Any]:
        """Return all runout monitors registered for a specific OpenAMS instance."""
        key = cls._key(printer, name)
        with cls._lock:
            return tuple(cls._monitors.get(key, ()))
