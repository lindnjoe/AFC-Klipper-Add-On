# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
import os
import re
import time
import threading
import traceback
from datetime import datetime
from types import MethodType
from enum import Enum

from configparser import Error as ConfigError


def _raise_import_error(import_lib: str, *, template: Optional[str] = None) -> None:
    trace = traceback.format_exc()
    if template is None:
        raise ConfigError(f"Error when trying to import {import_lib}\n{trace}")
    raise ConfigError(template.format(import_lib=import_lib, trace=trace))


try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    _raise_import_error("AFC_utils.ERROR_STR")

try:
    from extras.AFC_unit import afcUnit
except Exception:
    _raise_import_error("AFC_unit", template=ERROR_STR)

try:
    from extras.AFC_lane import AFCLane, AFCLaneState
except Exception:
    _raise_import_error("AFC_lane", template=ERROR_STR)

try:
    from extras.AFC_utils import add_filament_switch
except Exception:
    _raise_import_error("AFC_utils", template=ERROR_STR)

try:
    from extras.AFC_respond import AFCprompt
except Exception:
    _raise_import_error("AFC_respond", template=ERROR_STR)

# -- OpenAMS integration classes ------

OPENAMS_VERSION = "0.0.3"


def normalize_extruder_name(name):
    if not name or not isinstance(name, str):
        return None
    normalized = name.strip()
    if not normalized:
        return None
    return normalized.lower() or None



class AMSEventBus:
    _instance = None
    _lock = threading.RLock()
    _MAX_HISTORY = 500
    _HISTORY_TTL = 3600.0

    def __init__(self):
        self._subscribers = {}
        self._event_history = []
        self.logger = None

    @classmethod
    def get_instance(cls, logger=None):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            if logger is not None and cls._instance.logger is None:
                cls._instance.logger = logger
            return cls._instance

    def subscribe(self, event_type, callback, *, priority=0):
        with self._lock:
            if event_type not in self._subscribers:
                self._subscribers[event_type] = []
            subscribers = self._subscribers[event_type]
            insert_idx = 0
            for i, (existing_callback, existing_priority) in enumerate(subscribers):
                if priority > existing_priority:
                    insert_idx = i
                    break
                insert_idx = i + 1
            subscribers.insert(insert_idx, (callback, priority))
            if self.logger is not None:
                self.logger.debug(f"Subscribed to '{event_type}' (priority={priority}, total={len(subscribers)})")

    def publish(self, event_type, **kwargs):
        eventtime = kwargs.get('eventtime', time.time())
        with self._lock:
            self._event_history.append((event_type, eventtime, dict(kwargs)))
            if len(self._event_history) > self._MAX_HISTORY:
                cutoff = time.monotonic() - self._HISTORY_TTL
                self._event_history = [e for e in self._event_history if e[1] > cutoff]
                if len(self._event_history) > self._MAX_HISTORY:
                    self._event_history = self._event_history[-self._MAX_HISTORY:]
            subscribers = list(self._subscribers.get(event_type, []))
        if not subscribers:
            return 0
        success_count = 0
        for callback, priority in subscribers:
            try:
                callback(event_type=event_type, **kwargs)
                success_count += 1
            except Exception as e:
                if self.logger is not None:
                    self.logger.error(f"Event handler failed for '{event_type}' (priority={priority}): {e}")
        return success_count


class LaneInfo:
    def __init__(self, lane_name, unit_name, spool_index, extruder,
                 fps_name=None, hub_name=None, led_index=None,
                 custom_load_cmd=None, custom_unload_cmd=None):
        self.lane_name = lane_name
        self.unit_name = unit_name
        self.spool_index = spool_index
        self.extruder = extruder
        self.fps_name = fps_name
        self.hub_name = hub_name
        self.led_index = led_index
        self.custom_load_cmd = custom_load_cmd
        self.custom_unload_cmd = custom_unload_cmd


class LaneRegistry:
    _instances = {}
    _lock = threading.RLock()

    def __init__(self, printer, logger=None):
        self.printer = printer
        self.logger = logger
        self._lanes = []
        self._by_lane_name = {}
        self._by_lane_name_lower = {}
        self._by_spool = {}
        self._by_extruder = {}
        self.event_bus = AMSEventBus.get_instance(logger=self.logger)

    @classmethod
    def for_printer(cls, printer, logger=None):
        with cls._lock:
            key = id(printer)
            if key not in cls._instances:
                cls._instances[key] = cls(printer, logger=logger)
            elif logger is not None:
                cls._instances[key].logger = logger
            return cls._instances[key]

    def register_lane(self, lane_name, unit_name, spool_index, extruder, *,
                      fps_name=None, hub_name=None, led_index=None,
                      custom_load_cmd=None, custom_unload_cmd=None):
        with self._lock:
            existing = self._by_lane_name.get(lane_name)
            if existing is not None:
                self.logger.warning(f"Lane '{lane_name}' already registered, updating")
                self._unregister_lane(existing)
            info = LaneInfo(
                lane_name=lane_name, unit_name=unit_name,
                spool_index=spool_index, extruder=extruder,
                fps_name=fps_name, hub_name=hub_name, led_index=led_index,
                custom_load_cmd=custom_load_cmd, custom_unload_cmd=custom_unload_cmd,
            )
            self._lanes.append(info)
            self._by_lane_name[lane_name] = info
            self._by_lane_name_lower[lane_name.lower()] = info
            self._by_spool[(unit_name, spool_index)] = info
            if extruder not in self._by_extruder:
                self._by_extruder[extruder] = []
            self._by_extruder[extruder].append(info)
            self.logger.info(f"Registered lane: {lane_name} - {unit_name}[{spool_index}] (extruder={extruder}, fps={fps_name})")
            return info

    def _unregister_lane(self, info):
        if info in self._lanes:
            self._lanes.remove(info)
        self._by_lane_name.pop(info.lane_name, None)
        self._by_lane_name_lower.pop(info.lane_name.lower(), None)
        self._by_spool.pop((info.unit_name, info.spool_index), None)
        extruder_lanes = self._by_extruder.get(info.extruder, [])
        if info in extruder_lanes:
            extruder_lanes.remove(info)
            if not extruder_lanes:
                self._by_extruder.pop(info.extruder, None)

    def get_by_lane(self, lane_name):
        with self._lock:
            return self._by_lane_name.get(lane_name)

    def get_by_spool(self, unit_name, spool_index):
        with self._lock:
            return self._by_spool.get((unit_name, spool_index))

    def resolve_lane_token(self, token):
        with self._lock:
            return self._by_lane_name_lower.get(token.lower())

    def resolve_lane_name(self, unit_name, spool_index):
        info = self.get_by_spool(unit_name, spool_index)
        return info.lane_name if info else None

    def resolve_extruder(self, lane_name):
        info = self.get_by_lane(lane_name)
        return info.extruder if info else None


class AMSHardwareService:
    _instances = {}

    def __init__(self, printer, name, logger=None):
        self.printer = printer
        self.name = name
        if logger is not None:
            self.logger = logger
        else:
            self.logger = printer.lookup_object("AFC").logger
        self._controller = None
        self._lock = threading.RLock()
        self._status = {}
        self._lane_snapshots = {}
        self._status_callbacks = []
        self.registry = LaneRegistry.for_printer(printer, logger=self.logger)
        self.event_bus = AMSEventBus.get_instance(logger=self.logger)
        self._reactor = None
        self._polling_timer = None
        self._polling_interval = 2.0
        self._polling_interval_idle = 4.0
        self._consecutive_idle_polls = 0
        self._idle_poll_threshold = 3
        self._last_encoder_clicks = None
        self._last_f1s_hes = [None, None, None, None]
        self._last_hub_hes = [None, None, None, None]
        self._last_fps_value = None
        self._polling_enabled = False

    @classmethod
    def for_printer(cls, printer, name="default", logger=None):
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

    def attach_controller(self, controller):
        with self._lock:
            self._controller = controller
        if controller is not None:
            try:
                status = controller.get_status(self._monotonic())
            except Exception:
                status = None
            if status:
                self._update_status(status)
            self.logger.debug(f"Attached OAMS controller {controller}")

    def resolve_controller(self):
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

    def _monotonic(self):
        if self._reactor is None:
            self._reactor = self.printer.get_reactor()
        return self._reactor.monotonic()

    def start_polling(self):
        if self._polling_timer is not None:
            self.logger.warning(f"Polling already started for {self.name}")
            return
        if self._reactor is None:
            self._monotonic()
        if self._reactor is None:
            self.logger.error("Cannot start polling: reactor not available")
            return
        self._polling_enabled = True
        self._polling_timer = self._reactor.register_timer(
            self._polling_callback, self._reactor.NOW + 1.0
        )
        self.logger.info(f"Started unified hardware polling for {self.name} (offset by 1s)")

    def stop_polling(self):
        self._polling_enabled = False
        if self._polling_timer is not None and self._reactor is not None:
            self._reactor.unregister_timer(self._polling_timer)
            self._polling_timer = None

    def _polling_callback(self, eventtime):
        if not self._polling_enabled:
            return self._reactor.NEVER
        try:
            status = self.poll_status()
            if not status:
                return eventtime + self._polling_interval_idle
            encoder_changed = False
            encoder_clicks = status.get("encoder_clicks")
            if encoder_clicks is not None:
                if self._last_encoder_clicks is not None:
                    if encoder_clicks != self._last_encoder_clicks:
                        encoder_changed = True
                        self._consecutive_idle_polls = 0
                self._last_encoder_clicks = encoder_clicks
            f1s_values = status.get("f1s_hes_value", [])
            for bay in range(min(len(f1s_values), 4)):
                new_val = bool(f1s_values[bay])
                old_val = self._last_f1s_hes[bay]
                if old_val is None or new_val != old_val:
                    self.event_bus.publish(
                        "f1s_changed", unit_name=self.name, bay=bay,
                        value=new_val, eventtime=eventtime
                    )
                    if old_val is None:
                        self.logger.info(f"f1s[{bay}] initial state: {new_val}")
                    else:
                        self.logger.debug(f"f1s[{bay}] changed: {old_val} -> {new_val}")
                self._last_f1s_hes[bay] = new_val
            hub_values = status.get("hub_hes_value", [])
            for bay in range(min(len(hub_values), 4)):
                new_val = bool(hub_values[bay])
                old_val = self._last_hub_hes[bay]
                if old_val is None or new_val != old_val:
                    self.event_bus.publish(
                        "hub_changed", unit_name=self.name, bay=bay,
                        value=new_val, eventtime=eventtime
                    )
                    if old_val is None:
                        self.logger.info(f"hub[{bay}] initial state: {new_val}")
                    else:
                        self.logger.debug(f"hub[{bay}] changed: {old_val} -> {new_val}")
                self._last_hub_hes[bay] = new_val
            fps_value = status.get("fps_value")
            if fps_value is not None:
                old_fps = self._last_fps_value
                if old_fps is not None and abs(fps_value - old_fps) > 0.05:
                    self.event_bus.publish(
                        "fps_changed", unit_name=self.name, value=fps_value,
                        old_value=old_fps, eventtime=eventtime
                    )
                self._last_fps_value = fps_value
            if encoder_changed:
                return eventtime + self._polling_interval
            self._consecutive_idle_polls += 1
            if self._consecutive_idle_polls > self._idle_poll_threshold:
                return eventtime + self._polling_interval_idle
            return eventtime + self._polling_interval
        except Exception as e:
            self.logger.error(f"Error in unified polling callback for {self.name}: {e}\n{traceback.format_exc()}")
            return eventtime + self._polling_interval_idle

    def poll_status(self):
        controller = self.resolve_controller()
        if controller is None:
            return None
        eventtime = self._monotonic()
        try:
            status = controller.get_status(eventtime)
        except Exception:
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

    def _update_status(self, status):
        with self._lock:
            self._status = dict(status)

    def latest_status(self):
        with self._lock:
            return dict(self._status)

    def update_lane_snapshot(self, unit_name: str, lane_name: str, lane_state: bool,
                           hub_state: Optional[bool], eventtime: float, *,
                           spool_index: Optional[int] = None,
                           tool_state: Optional[bool] = None,
                           emit_spool_event: bool = True) -> None:
        """Update the cached state snapshot for a specific lane."""
        key = f"{unit_name}:{lane_name}"

        normalized_index: Optional[int]
        if spool_index is not None:
            try:
                normalized_index = int(spool_index)
            except (TypeError, ValueError):
                normalized_index = None
            else:
                if normalized_index < 0:
                    normalized_index = None
        else:
            normalized_index = None

        with self._lock:
            old_snapshot = self._lane_snapshots.get(key, {})

            self._lane_snapshots[key] = {
                "unit": unit_name,
                "lane": lane_name,
                "lane_state": bool(lane_state),
                "hub_state": None if hub_state is None else bool(hub_state),
                "timestamp": eventtime,
            }
            if normalized_index is not None:
                self._lane_snapshots[key]["spool_index"] = normalized_index
            elif "spool_index" in old_snapshot:
                self._lane_snapshots[key]["spool_index"] = old_snapshot["spool_index"]
            if tool_state is not None:
                self._lane_snapshots[key]["tool_state"] = bool(tool_state)

        event_spool_index = normalized_index
        if event_spool_index is None:
            event_spool_index = old_snapshot.get("spool_index")

        old_lane_state = old_snapshot.get("lane_state")
        new_lane_state = bool(lane_state)

        if emit_spool_event and old_lane_state is not None and old_lane_state != new_lane_state and event_spool_index is not None:
            event_type = "spool_loaded" if new_lane_state else "spool_unloaded"
            self.event_bus.publish(
                event_type,
                unit_name=unit_name,
                lane_name=lane_name,
                spool_index=event_spool_index,
                eventtime=eventtime,
            )

        old_hub_state = old_snapshot.get("hub_state")
        new_hub_state = hub_state

        if old_hub_state is not None and new_hub_state is not None:
            if old_hub_state != new_hub_state:
                event_type = "lane_hub_loaded" if new_hub_state else "lane_hub_unloaded"
                self.event_bus.publish(
                    event_type,
                    unit_name=unit_name,
                    lane_name=lane_name,
                    spool_index=spool_index,
                    eventtime=eventtime
                )

        if tool_state is not None:
            old_tool_state = old_snapshot.get("tool_state")
            if old_tool_state is not None and old_tool_state != tool_state:
                event_type = "lane_tool_loaded" if tool_state else "lane_tool_unloaded"
                self.event_bus.publish(
                    event_type,
                    unit_name=unit_name,
                    lane_name=lane_name,
                    spool_index=spool_index,
                    eventtime=eventtime
                )

    def latest_lane_snapshot(self, unit_name: str, lane_name: str):
        """Return the most recent state snapshot for a specific lane."""
        key = f"{unit_name}:{lane_name}"
        with self._lock:
            snapshot = self._lane_snapshots.get(key)
        return dict(snapshot) if snapshot else None

    def resolve_lane_for_spool(self, unit_name, spool_index):
        if spool_index is None:
            return None
        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None
        return self.registry.resolve_lane_name(unit_name, normalized)

    def resolve_lane_for_spool_with_afc(self, unit_name, spool_index):
        lane_name = self.resolve_lane_for_spool(unit_name, spool_index)
        if lane_name is not None:
            return lane_name
        return self._resolve_lane_name_from_afc(unit_name, spool_index)

    def _resolve_lane_name_from_afc(self, unit_name, spool_index):
        if spool_index is None:
            return None
        try:
            normalized_index = int(spool_index)
        except (TypeError, ValueError):
            return None
        try:
            afc = self.printer.lookup_object("AFC", None)
        except Exception:
            afc = None
        if afc is None or not hasattr(afc, 'units'):
            return None
        for unit_obj in afc.units.values():
            if not hasattr(unit_obj, 'oams_name'):
                continue
            if unit_obj.oams_name != unit_name:
                continue
            target_slot = normalized_index + 1
            for lane_name, lane_obj in getattr(unit_obj, 'lanes', {}).items():
                lane_unit = getattr(lane_obj, 'unit', None)
                if lane_unit and ':' in lane_unit:
                    try:
                        slot = int(lane_unit.split(':', 1)[1])
                    except (ValueError, IndexError):
                        continue
                    if slot == target_slot:
                        return lane_name
        return None


class AMSRunoutCoordinator:
    """Coordinates runout events between OpenAMS and AFC."""
    _units = {}
    _monitors = {}
    _lock = threading.RLock()

    @classmethod
    def _key(cls, printer, name):
        return (id(printer), name)

    @classmethod
    def register_afc_unit(cls, unit):
        service = AMSHardwareService.for_printer(unit.printer, unit.oams_name, unit.logger)
        key = cls._key(unit.printer, unit.oams_name)
        with cls._lock:
            cls._units.setdefault(key, [])
            if unit not in cls._units[key]:
                cls._units[key].append(unit)
        return service

    @classmethod
    def register_runout_monitor(cls, monitor):
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
    def notify_runout_detected(cls, monitor, spool_index, *, lane_name=None):
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
            except Exception as e:
                unit.logger.error(f"Failed to propagate OpenAMS runout to AFC unit {unit.name}: {e}")

    @classmethod
    def notify_afc_error(cls, printer, name, message, *, pause=False):
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
                error_obj.AFC_error(message, pause=pause, stack_name="notify_afc_error")
            except Exception as e:
                logger = getattr(unit, "logger", None)
                if logger is not None:
                    logger.error(f"Failed to deliver OpenAMS error '{message}' to AFC unit {unit}: {e}")

    @classmethod
    def notify_lane_tool_state(cls, printer, name, lane_name, *, loaded, spool_index=None, eventtime=None):
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
            except Exception as e:
                unit.logger.error(f"Failed to update AFC lane {lane_name} from OpenAMS tool state: {e}")
        return handled

def _is_openams_unit(unit_obj) -> bool:
    """Return True if *unit_obj* represents an OpenAMS unit."""
    if unit_obj is None:
        return False
    if isinstance(unit_obj, afcAMS):
        return True
    if getattr(unit_obj, "type", None) == "OpenAMS":
        return True
    if hasattr(unit_obj, "oams_name"):
        return True
    return False


class OpenAMSStateMutation(str, Enum):
    """Authoritative state mutation channels for OpenAMS/AFC integration."""
    SENSOR = "sensor"
    TOOL = "tool"
    RUNOUT = "runout"


EVENT_POLICY: Dict[str, Dict[str, bool]] = {
    # Hardware sensor channels own load/prep and hub transitions.
    OpenAMSStateMutation.SENSOR.value: {
        "allow_spool_events": True,
        "allow_full_unload": True,
    },
    # Tool-only channels must not emit spool transitions.
    OpenAMSStateMutation.TOOL.value: {
        "allow_spool_events": False,
        "allow_full_unload": False,
    },
    # Runout flow can force full state cleanup.
    OpenAMSStateMutation.RUNOUT.value: {
        "allow_spool_events": True,
        "allow_full_unload": True,
    },
}


class afcAMS(afcUnit):
    """AFC unit subclass that synchronises state with OpenAMS"""

    _sync_command_registered = False
    _sync_instances: Dict[str, "afcAMS"] = {}
    _hydrated_extruders: Set[str] = set()

    def __init__(self, config):
        super().__init__(config)
        self.type = "OpenAMS"

        # AMS units don't have physical TurtleNeck buffers.
        # However, AFC_FPS buffers ARE allowed — they provide the FPS ADC reading
        # and don't try to adjust stepper rotation distance (no stepper on OpenAMS).
        # If user configured an AFC_FPS buffer, keep it; otherwise force None.
        if self.buffer_obj is not None and not hasattr(self.buffer_obj, 'get_fps_value'):
            self.buffer_obj = None

        # LED attributes are inherited from afcUnit.__init__ via super().__init__.

        self.oams_name = config.get("oams", "oams1")

        # When True, a stuck spool detected during printing triggers an automatic
        # unload + reload + resume cycle instead of just pausing for user intervention.
        # Defaults to False (pause-only) so the behaviour is opt-in.
        self.stuck_spool_auto_recovery = config.getboolean("stuck_spool_auto_recovery", False)

        # Detection thresholds (configurable per unit)
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", 8.0, minval=0.0)
        self.stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", 0.08, minval=0.0)
        self._engagement_pressure_max = config.getfloat("engagement_pressure_threshold", 0.6, minval=0.0)
        self._engagement_pressure_min = config.getfloat("engagement_min_pressure", 0.25, minval=0.0)
        self.clog_sensitivity = config.get("clog_sensitivity", "medium").lower()

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        # Lane registry integration
        self.registry = None
        if LaneRegistry is not None:
            try:
                self.registry = LaneRegistry.for_printer(self.printer)
            except Exception as e:
                self.logger.error(f"Failed to initialize LaneRegistry: {e}")

        # Event bus subscription for spool changes
        self.event_bus = None
        if AMSEventBus is not None:
            try:
                self.event_bus = AMSEventBus.get_instance()
                self.event_bus.subscribe("spool_loaded", self._handle_spool_loaded_event, priority=10)
                self.event_bus.subscribe("spool_unloaded", self._handle_spool_unloaded_event, priority=10)
            except Exception as e:
                self.logger.error(f"Failed to subscribe to AMS events: {e}")

        # NOTE: This class uses AFC native state exclusively (no duplicate tracking):
        # - extruder.lane_loaded - which lane is loaded to toolhead
        # - lane.load_state - filament at F1S sensor
        # - lane.loaded_to_hub - filament at hub sensor
        # - lane.tool_loaded - filament loaded to extruder
        # Previous versions maintained local caches, now removed for single source of truth

        self._saved_unit_cache: Optional[Dict[str, Any]] = None
        self._saved_unit_mtime: Optional[float] = None
        # Keep _last_hub_hes_values for HES calibration (not an AFC responsibility)
        self._last_hub_hes_values: Optional[List[float]] = None

        self._cached_sensor_helper = None
        self._cached_gcode = None
        self._cached_extruder_objects: Dict[str, Any] = {}
        self._cached_lane_objects: Dict[str, Any] = {}
        self._cached_oams_index: Optional[int] = None
        # Subsystem instances created in handle_ready() after hardware resolution

        # Track pending TD-1 capture timers (delayed after spool insertion)
        self._pending_spool_loaded_timers: Dict[str, Any] = {}
        # Track last synced lane_tool_loaded signature per lane to avoid re-running
        self._last_lane_tool_loaded_sync: Dict[str, tuple] = {}

        self.oams = None
        self.hardware_service = None

        # Owned subsystem instances (created in handle_ready when hardware is available)
        self._follower = None   # FollowerController — follower motor + LED + MCU queue
        self._monitor = None    # OAMSMonitor — stuck spool + clog detection

        if AMSRunoutCoordinator is not None:
            self.hardware_service = AMSRunoutCoordinator.register_afc_unit(self)
        elif AMSHardwareService is not None:
            self.hardware_service = AMSHardwareService.for_printer(self.printer, self.oams_name, self.logger)

        self._register_sync_dispatcher()

        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_HUB_HES", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_HUB_HES, desc="calibrate the OpenAMS HUB HES value for a specific lane")
        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_HUB_HES_ALL", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL, desc="calibrate the OpenAMS HUB HES value for every loaded lane")
        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_PTFE", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_PTFE, desc="Calibrate PTFE length for this OpenAMS unit (any lane can be used — distance to hub is identical for all bays)")
        self.gcode.register_mux_command("UNIT_PTFE_CALIBRATION", "UNIT", self.name, self.cmd_UNIT_PTFE_CALIBRATION, desc="show OpenAMS PTFE calibration menu")
        self.gcode.register_mux_command("AFC_OAMS_CLEAR_ERRORS", "UNIT", self.name, self.cmd_AFC_OAMS_CLEAR_ERRORS, desc="Clear OpenAMS errors, LEDs, and resync state")
        self.gcode.register_mux_command("AFC_OAMS_STATUS", "UNIT", self.name, self.cmd_AFC_OAMS_STATUS, desc="Show OpenAMS unit status")

    def _is_openams_unit(self):
        """Check if this unit has OpenAMS hardware available."""
        return self.oams is not None

    def get_lane_reset_command(self, lane, dis) -> str:
        """Return the GCode command used when a user requests a lane reset.

        OpenAMS units don't support stepper-based reset-to-hub. Uses the
        OAMS hardware command when an FPS ID is resolvable; otherwise falls
        back to TOOL_UNLOAD so the filament is correctly retracted via AFC.
        This hook is called by AFC_functions._lane_reset_command() so reset
        prompts and AFC_LANE_RESET both produce the correct action.
        """
        return f"TOOL_UNLOAD LANE={lane.name}"

    # Load/unload, monitoring, follower control all handled by owned subsystems

    # ---- Dock purge (AFC-owned, same pattern as ACE) ----

    def _dock_purge_dropoff(self):
        """Drop off current tool at dock for dock purging.

        Enters docking mode and runs the toolchanger's dropoff gcode so the
        nozzle rests on the dock pad while filament is loaded and purged.
        """
        cur_extruder = self.afc.function.get_current_extruder_obj()
        tc = cur_extruder.tc_unit_obj if cur_extruder else None
        if not tc or not tc.active_tool:
            self.logger.warning("OAMS dock purge: no active tool, skipping dropoff")
            return
        tool = tc.active_tool

        self.afc.gcode.run_script_from_command("ENTER_DOCKING_MODE")

        gcode_pos = list(tc.gcode_move.get_status()['gcode_position'])
        start_pos = tc._position_with_tool_offset(gcode_pos, None)
        self._dock_purge_context = {
            'dropoff_tool': tool.name,
            'pickup_tool': tool.name,
            'dock_purge': True,
            'start_position': tc._position_to_xyz(start_pos, 'xyz'),
            'restore_position': tc._position_to_xyz(start_pos, 'XYZ'),
        }

        tc._run_gcode('tool.dropoff_gcode', tool.dropoff_gcode, self._dock_purge_context)
        self.logger.info("OAMS dock purge: tool dropped off at dock")

    def _dock_purge_pickup(self):
        """Pick up tool from dock after purging.

        Runs the toolchanger's pickup gcode and exits docking mode.
        If pickup verification fails, fires the toolchanger's error_gcode
        (e.g. M112) to prevent dock damage.
        """
        cur_extruder = self.afc.function.get_current_extruder_obj()
        tc = cur_extruder.tc_unit_obj if cur_extruder else None
        if not tc or not tc.active_tool or not hasattr(self, '_dock_purge_context') or self._dock_purge_context is None:
            self.logger.warning("OAMS dock purge: no context for pickup, skipping")
            return
        tool = tc.active_tool

        try:
            tc._run_gcode('tool.pickup_gcode', tool.pickup_gcode, self._dock_purge_context)
        except Exception as e:
            self.logger.error(f"OAMS dock purge: pickup failed: {e}")
            # If error_gcode (M112) didn't already fire via process_error,
            # fire it now to prevent dock damage
            if tc.error_gcode and tc.status != 'error':
                try:
                    tc.process_error(None, f"Dock purge pickup failed: {e}")
                except Exception:
                    pass
            self._dock_purge_context = None
            raise
        self.afc.gcode.run_script_from_command("EXIT_DOCKING_MODE")
        self._dock_purge_context = None
        self.logger.info("OAMS dock purge: tool picked up from dock")

    def _is_dock_purge_enabled(self):
        """Check if dock purge is enabled on the OAMS hardware."""
        return self.oams is not None and getattr(self.oams, 'dock_load', False)

    # ---- Direct hardware load/unload ----

    def _wait_oams_idle(self, oams, timeout=10.0, context="operation"):
        """Wait for OAMS MCU to finish any in-flight action.

        The MCU responds to commands asynchronously. If we send a new command
        while it's still processing the previous one, we get ERROR_BUSY.
        This polls action_status until None (idle) or timeout.
        """
        if oams.action_status is None:
            return True

        self.logger.debug(
            f"Waiting for OAMS MCU to finish {oams.action_status} before {context}")
        deadline = self.afc.reactor.monotonic() + timeout
        while oams.action_status is not None:
            if self.afc.reactor.monotonic() > deadline:
                self.logger.warning(
                    f"OAMS MCU still busy after {timeout}s, forcing clear for {context}")
                oams.action_status = None
                oams.action_status_code = None
                return False
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)

        self.logger.debug(f"OAMS MCU idle, proceeding with {context}")
        return True

    def _oams_load(self, cur_lane, max_retries=None):
        """Load filament via OAMS hardware directly.

        Calls oams.py load_spool_with_retry(), verifies engagement,
        enables follower.

        :param cur_lane: Lane object to load
        :param max_retries: Override for engagement retry count
        :return: (success, message) tuple
        """
        oams = self.oams
        if oams is None:
            return False, "OAMS hardware not available"

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            return False, f"Cannot resolve spool index for {cur_lane.name}"

        # Check bay is ready
        if not oams.is_bay_ready(spool_index):
            return False, f"Bay {spool_index} not ready (no spool detected)"

        if max_retries is None:
            max_retries = self.afc.tool_max_unload_attempts

        # Wait for MCU to be idle before starting load
        self._wait_oams_idle(oams, context=f"load {cur_lane.name}")

        # Suspend monitor during load to prevent false stuck spool detection
        if self._monitor is not None:
            self._monitor.stop()

        # Enable follower forward before load
        if self._follower is not None:
            fps_state = self._get_monitor_state()
            if fps_state:
                self._follower.enable_follower(fps_state, oams, 1, "before load", force=True)

        # Load with retries
        for attempt in range(max_retries):
            self.logger.debug(f"Load attempt {attempt + 1}/{max_retries} for {cur_lane.name}")

            try:
                result = oams.load_spool_with_retry(spool_index)
            except Exception as e:
                self.logger.error(f"Load failed for {cur_lane.name}: {e}")
                continue

            # load_spool_with_retry returns (success_bool, message_str)
            if isinstance(result, tuple):
                success = bool(result[0])
            else:
                success = bool(result)

            if not success:
                self.logger.warning(f"Load attempt {attempt + 1} failed for {cur_lane.name}")
                continue

            # Load succeeded — verify engagement
            engaged = self._verify_engagement(cur_lane)
            if engaged:
                # Enable follower forward after successful engagement
                if self._follower is not None:
                    fps_state = self._get_monitor_state()
                    if fps_state:
                        self._follower.enable_follower(fps_state, oams, 1, "load complete", force=True)
                # Notify monitor and resume
                if self._monitor is not None:
                    self._monitor.notify_load_complete(
                        cur_lane.name, self.oams_name, spool_index)
                    self._monitor.start(oams)
                return True, f"Loaded {cur_lane.name}"

            # Engagement failed — fully reset MCU state before retry.
            # Without abort + wait, the MCU thinks the spool is still loaded
            # and the next load_spool returns "success" in 0.2s without
            # physically moving filament.
            self.logger.warning(f"Engagement failed for {cur_lane.name}, cleaning up for retry")

            # Set follower reverse before retract so it assists the pull
            if self._follower is not None:
                fps_state = self._get_monitor_state()
                if fps_state:
                    self._follower.set_follower_state(
                        fps_state, oams, 1, 0, "engagement cleanup", force=True)
                    self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)

            # Retract full tool_stn_unload + 10mm to clear extruder gears
            # before OAMS hardware unload. engagement_length alone isn't enough
            # because post-engagement extrusion pushed filament further in.
            unload_length, unload_speed = self.get_unload_params(cur_lane.name)
            if unload_length and unload_length > 0:
                unload_length += 10.0
                retract_speed = unload_speed if unload_speed else 25.0 * 60.0
                self.logger.debug(
                    f"Retracting {unload_length:.1f}mm from extruder before cleanup unload")
                try:
                    self._oams_extrude(-unload_length, retract_speed, "engagement_cleanup_retract")
                except Exception as e:
                    self.logger.warning(f"Cleanup retract failed: {e}")

            # Abort any in-flight MCU action and wait for it to clear
            oams.abort_current_action(wait=True)
            self._wait_oams_idle(oams, context="post-abort settle")

            # Full hardware unload — pulls filament back to spool bay
            try:
                oams.unload_spool_with_retry()
            except Exception as e:
                self.logger.error(f"Cleanup unload failed: {e}")
                return False, f"Engagement cleanup failed for {cur_lane.name}: {e}"

            # Wait for MCU to fully settle before next load attempt
            self._wait_oams_idle(oams, context="post-cleanup settle")
            # Extra delay for MCU firmware to reset its internal state
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.0)

        # All attempts failed — do NOT restart monitor here.
        # The caller (load_sequence) manages monitor lifecycle around
        # dock purge. Restarting here causes false stuck spool detection
        # during the dock purge pickup that follows a load failure.
        return False, f"All {max_retries} load attempts failed for {cur_lane.name}"

    def _verify_engagement(self, cur_lane):
        """Verify filament engaged the extruder after OAMS load.

        Extrudes engagement_length and checks:
        - Encoder moved (follower tracked filament being pulled)
        - FPS pressure in valid range (not too high, not too low)

        :param cur_lane: Lane being loaded
        :return: True if engaged
        """
        oams = self.oams
        if oams is None:
            return True  # Assume success if no hardware

        engagement_length, engagement_speed = self.get_engagement_params(cur_lane.name)
        if engagement_length is None:
            return True  # No params = skip verification

        self.logger.debug(
            f"Verifying engagement for {cur_lane.name}: "
            f"{engagement_length:.1f}mm at {engagement_speed:.0f}mm/min")

        # Notify monitor to suppress detection during engagement
        if self._monitor is not None:
            self._monitor.notify_engagement_start()

        try:
            # Wait for FPS pressure to settle after OAMS load push.
            # The OAMS just pushed filament through 2000mm+ of PTFE,
            # so FPS pressure is maxed out. Give it time to normalize
            # before checking engagement pressure.
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.5)

            # Record encoder before
            encoder_before = oams.encoder_clicks

            # Two-phase extrusion: short prime then remaining
            prime_length = 5.0
            remaining = max(0.0, engagement_length - prime_length)

            self._oams_extrude(prime_length, engagement_speed, "engagement_prime")
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            if remaining > 0:
                self._oams_extrude(remaining, engagement_speed, "engagement_main")

            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

            # Check encoder movement
            encoder_after = oams.encoder_clicks
            encoder_delta = abs(encoder_after - encoder_before)
            expected = max(1.0, remaining * 0.5)
            min_movement = max(1.0, expected - 3.0)

            fps_pressure = oams.fps_value

            # Primary check: encoder moved enough = filament is engaged.
            # Encoder movement is the strongest signal — if the OAMS encoder
            # is tracking, the extruder is pulling filament through the buffer.
            # Pressure is secondary (FPS pressure can be high right after load).
            engagement_min = getattr(self, '_engagement_pressure_min', 0.25)
            if encoder_delta >= min_movement:
                self.logger.debug(
                    f"Engagement verified for {cur_lane.name} "
                    f"(encoder={encoder_delta}, pressure={fps_pressure:.2f})")
                # Complete the load with remaining stn distance
                reload_length, reload_speed = self.get_reload_params(cur_lane.name)
                if reload_length and reload_speed and reload_length > 0:
                    self._oams_extrude(reload_length, reload_speed, "post_engagement")
                return True

            # Encoder didn't move enough — retry with short delay
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
            encoder_retry = oams.encoder_clicks
            encoder_delta = abs(encoder_retry - encoder_before)

            if encoder_delta >= min_movement:
                self.logger.debug(
                    f"Engagement verified on retry for {cur_lane.name} "
                    f"(encoder={encoder_delta})")
                reload_length, reload_speed = self.get_reload_params(cur_lane.name)
                if reload_length and reload_speed and reload_length > 0:
                    self._oams_extrude(reload_length, reload_speed, "post_engagement")
                return True

            self.logger.info(
                f"Engagement failed for {cur_lane.name}: encoder only moved "
                f"{encoder_delta} clicks (need >={min_movement:.1f})")
            return False

        finally:
            if self._monitor is not None:
                self._monitor.notify_engagement_end()

    def _oams_extrude(self, length, speed, context="oams_move"):
        """Extrude filament via gcode (for engagement verification).

        :param length: Distance in mm
        :param speed: Speed in mm/min
        :param context: Logging context
        """
        gcode = self.afc.gcode
        gcode.run_script_from_command(f"SAVE_GCODE_STATE NAME={context}")
        try:
            gcode.run_script_from_command("M83")
            gcode.run_script_from_command(f"G1 E{length:.2f} F{speed:.0f}")
            gcode.run_script_from_command("M400")
        finally:
            gcode.run_script_from_command(f"RESTORE_GCODE_STATE NAME={context} MOVE=0")

    def _oams_unload(self, cur_lane):
        """Unload filament via OAMS hardware directly.

        Retracts filament from extruder gears first, then tells OAMS
        hardware to pull it back to the spool bay.

        :param cur_lane: Lane to unload
        :return: (success, message) tuple
        """
        oams = self.oams
        if oams is None:
            return False, "OAMS hardware not available"

        # Wait for MCU to be idle before starting unload
        self._wait_oams_idle(oams, context=f"unload {cur_lane.name}")

        # Suspend monitor during unload to prevent false stuck spool detection
        if self._monitor is not None:
            self._monitor.stop()

        # Retract filament from extruder gears before OAMS hardware unload.
        # The OAMS firmware handles follower direction internally during unload.
        unload_length, unload_speed = self.get_unload_params(cur_lane.name)
        if unload_length and unload_length > 0:
            unload_length += 10.0  # Extra margin to fully clear extruder gears
            retract_speed = unload_speed if unload_speed else 25.0 * 60.0
            self.logger.debug(
                f"Retracting {unload_length:.1f}mm from extruder before OAMS unload")
            try:
                self._oams_extrude(-unload_length, retract_speed, "pre_oams_unload_retract")
            except Exception as e:
                self.logger.warning(f"Extruder retract before OAMS unload failed: {e}")

        try:
            # Queue a 20mm extruder retract WITHOUT waiting — it runs
            # concurrently with the OAMS hardware unload so the extruder
            # helps pull filament back as the spool motor rewinds.
            try:
                gcode = self.afc.gcode
                gcode.run_script_from_command("M83")
                gcode.run_script_from_command("G1 E-20.00 F1500")
                # No M400 — let it run in parallel with unload_spool
            except Exception as e:
                self.logger.warning(f"Concurrent retract failed: {e}")

            result = oams.unload_spool_with_retry()
            success = bool(result) if not isinstance(result, tuple) else bool(result[0])

            # Wait for MCU to fully settle after unload
            self._wait_oams_idle(oams, context="post-unload settle")

            # Notify monitor of unload completion — do NOT restart.
            # No filament is loaded after unload, so monitoring is pointless
            # and could cause false detections.
            if self._monitor is not None:
                self._monitor.notify_unload_complete()
            if success:
                return True, f"Unloaded {cur_lane.name}"
            msg = result[1] if isinstance(result, tuple) and len(result) > 1 else f"OAMS unload failed for {cur_lane.name}"
            return False, msg
        except Exception as e:
            return False, f"OAMS unload error for {cur_lane.name}: {e}"

    def _get_monitor_state(self):
        """Get the FPSState from the monitor if available."""
        if self._monitor is not None:
            return self._monitor.state
        return None

    def _init_follower_and_monitor(self):
        """Create and wire FollowerController and OAMSMonitor.

        Called from handle_ready() after OAMS hardware is resolved.
        The follower controls the OAMS follower motor; the monitor watches
        encoder + FPS pressure for stuck spool and clog conditions.
        """
        oams = self.oams
        if oams is None:
            return

        # ---- FollowerController ----
        try:
            from extras.AFC_OpenAMS_follower import FollowerController
            self._follower = FollowerController(
                {self.oams_name: oams}, self.reactor, self.logger)
            self.logger.debug(f"FollowerController created for {self.oams_name}")
        except Exception as e:
            self.logger.error(f"Failed to create FollowerController: {e}")

        # ---- OAMSMonitor ----
        fps_name = self._get_fps_name()
        fps_obj = self.buffer_obj
        if fps_name and fps_obj:
            try:
                from extras.AFC_OpenAMS_monitor import OAMSMonitor
                self._monitor = OAMSMonitor(
                    fps_name, fps_obj, self.reactor, self.logger,
                    on_stuck_spool=self._on_stuck_spool_detected,
                    on_clog=self._on_clog_detected,
                    on_stuck_cleared=self._on_stuck_spool_cleared,
                    clog_sensitivity=self.clog_sensitivity,
                    is_printing_fn=lambda: self.afc.function.in_print(),
                    is_lane_loaded_fn=lambda: any(
                        getattr(l, 'tool_loaded', False)
                        for l in self.lanes.values()),
                )
                # Apply config thresholds
                self._monitor.stuck_pressure_low = self.stuck_spool_pressure_threshold
                self._monitor.stuck_load_grace = self.stuck_spool_load_grace
                # Do NOT start monitor here — it will be started after a
                # successful load in _oams_load(). Starting at boot causes
                # false clog detection during PRINT_START (homing, mesh, etc.)
                # because the extruder position advances without any filament loaded.
                self.logger.debug(f"OAMSMonitor created for {fps_name} (starts on load)")
            except Exception as e:
                self.logger.error(f"Failed to create OAMSMonitor: {e}")
        else:
            self.logger.debug(
                f"No FPS buffer for {self.name}, monitor not started "
                f"(fps_name={fps_name}, buffer_obj={fps_obj})")

    def _get_fps_name(self):
        """Get the FPS buffer name for this unit."""
        if self.buffer_name:
            return self.buffer_name
        # Try to derive from first lane's extruder config
        for lane in self.lanes.values():
            extruder = getattr(lane, 'extruder_obj', None)
            if extruder:
                buf = getattr(extruder, 'buffer_name', None)
                if buf:
                    return buf
        return None

    def _on_stuck_spool_detected(self, fps_name, message):
        """Callback from OAMSMonitor when stuck spool is detected."""
        self.logger.info(f"Monitor: {message}")
        st = self._get_monitor_state()
        lane_name = st.current_lane if st else None

        # Set error LED on hardware
        if st and self._follower and st.current_spool_idx is not None:
            self._follower.set_led_error_if_changed(
                self.oams, self.oams_name, st.current_spool_idx, 1,
                "stuck spool detected")

        # Auto-recovery or pause
        if self.stuck_spool_auto_recovery and lane_name:
            self._on_stuck_spool_recovery_needed(fps_name, lane_name)
        else:
            self.request_pause(message, lane_name=lane_name)

    def _on_clog_detected(self, fps_name, message):
        """Callback from OAMSMonitor when clog is detected."""
        self.logger.info(f"Monitor: {message}")
        st = self._get_monitor_state()
        lane_name = st.current_lane if st else None
        self.request_pause(message, lane_name=lane_name)

    def _on_stuck_spool_cleared(self, fps_name):
        """Callback from OAMSMonitor when stuck spool condition clears."""
        self.logger.debug(f"Monitor: stuck spool cleared on {fps_name}")
        st = self._get_monitor_state()
        if st and self._follower and st.current_spool_idx is not None:
            self._follower.clear_error_led(
                self.oams, self.oams_name, st.current_spool_idx,
                "stuck spool cleared")

    def _handle_disconnect(self):
        """Stop monitor on klipper disconnect/shutdown."""
        if self._monitor is not None:
            self._monitor.stop()

    # ---- Parameter getters (AFC owns extruder config) ----

    def get_engagement_params(self, lane_name):
        """Get engagement extrusion length and speed from AFC extruder config.

        :param lane_name: Lane name to get params for
        :return: (engagement_length_mm, engagement_speed_mm_per_min) or (None, None)
        """
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return None, None
        extruder = getattr(lane, 'extruder_obj', None)
        if extruder is None:
            return None, None
        tool_stn = getattr(extruder, 'tool_stn', None)
        if tool_stn is None:
            return None, None
        engagement_length = tool_stn / 2.0
        engagement_speed = getattr(extruder, 'tool_load_speed', 25.0) * 60.0
        return engagement_length, engagement_speed

    def get_reload_params(self, lane_name):
        """Get post-engagement reload length and speed from AFC extruder config.

        The reload length is the remaining distance after engagement verification:
        (tool_stn / 2) + tool_sensor_after_extruder + retract_length + hotend_compensation

        :param lane_name: Lane name to get params for
        :return: (reload_length_mm, reload_speed_mm_per_min) or (None, None)
        """
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return None, None
        extruder = getattr(lane, 'extruder_obj', None)
        if extruder is None:
            return None, None
        tool_stn = getattr(extruder, 'tool_stn', 0.0)
        tool_sensor_after = getattr(extruder, 'tool_sensor_after_extruder', 0.0)
        tool_load_speed = getattr(extruder, 'tool_load_speed', 25.0)

        # Get additional components from macro variables
        try:
            macro_vars = self.printer.lookup_object('gcode_macro _oams_macro_variables', None)
            hotend_compensation = getattr(macro_vars, 'hotend_meltzone_compensation', 0.0) if macro_vars else 0.0
            cut_tip_vars = self.printer.lookup_object('gcode_macro _AFC_CUT_TIP_VARS', None)
            retract_length = getattr(cut_tip_vars, 'retract_length', 0.0) if cut_tip_vars else 0.0
        except Exception:
            hotend_compensation = 0.0
            retract_length = 0.0

        reload_length = (tool_stn / 2.0) + tool_sensor_after + retract_length + hotend_compensation
        reload_speed = tool_load_speed * 60.0
        return reload_length, reload_speed

    def get_unload_params(self, lane_name):
        """Get unload retract length and speed from AFC extruder config.

        :param lane_name: Lane name to get params for
        :return: (unload_length_mm, unload_speed_mm_per_min) or (None, None)
        """
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return None, None
        extruder = getattr(lane, 'extruder_obj', None)
        if extruder is None:
            return None, None
        unload_length = getattr(extruder, 'tool_stn_unload', None)
        if unload_length is None or unload_length <= 0:
            unload_length = getattr(extruder, 'tool_stn', None)
        unload_speed = getattr(extruder, 'tool_unload_speed', None)
        unload_speed = unload_speed * 60.0 if unload_speed is not None else None
        return unload_length, unload_speed

    # ---- State queries (AFC is source of truth) ----

    def get_loaded_lane_for_extruder(self, extruder_name):
        """Return the lane name currently loaded on the given extruder, per AFC.

        This is the authoritative answer.

        :param extruder_name: Extruder name (e.g. 'extruder4')
        :return: Lane name (e.g. 'lane5') or None
        """
        tool = self.afc.tools.get(extruder_name)
        if tool is not None:
            return tool.lane_loaded
        return None

    def is_lane_loaded(self, lane_name):
        """Check if a specific lane is loaded to its extruder, per AFC.

        :param lane_name: Lane name to check
        :return: True if lane is loaded to toolhead
        """
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return False
        return (lane.extruder_obj.lane_loaded == lane_name)

    def request_pause(self, message, lane_name=None):
        """Request AFC to pause the print with a message.

        Called by OAMSMonitor callbacks when a problem is detected
        (clog, stuck spool, runout without target lane). AFC owns the
        pause decision.

        :param message: Error message to display
        :param lane_name: Optional lane name for lane-specific error handling
        """
        try:
            if lane_name:
                lane = self.afc.lanes.get(lane_name)
                if lane:
                    self.afc.error.handle_lane_failure(lane, message)
                    return
            self.afc.error.AFC_error(message, pause=True)
        except Exception as e:
            self.logger.error(f"Failed to request AFC pause: {e}")
            # Last resort — direct pause if AFC error handler fails
            try:
                self.afc.gcode.run_script_from_command("PAUSE")
            except Exception:
                pass

    def handle_same_fps_reload(self, source_lane_name, target_lane_name):
        """Handle same-FPS infinite runout reload — AFC is master control.

        Called after the old filament has
        coasted through the shared PTFE and cleared the path. The new spool's
        filament will be pushed forward by the OAMS to meet the old filament
        at the extruder gears and get pulled in seamlessly.

        No pause, no unload, no tool swap — printing continues uninterrupted.

        :param source_lane_name: Lane that ran out (e.g. 'lane4')
        :param target_lane_name: Lane to reload from (e.g. 'lane5')
        :return: True if reload succeeded
        """
        afc = self.afc
        source_lane = afc.lanes.get(source_lane_name)
        target_lane = afc.lanes.get(target_lane_name)

        if not source_lane or not target_lane:
            self.logger.error(
                f"Same-FPS reload failed: source={source_lane_name} "
                f"target={target_lane_name} — lane not found")
            return False

        self.logger.info(
            f"Same-FPS infinite runout: {source_lane_name} -> {target_lane_name}")

        # Mark source lane as empty — filament has coasted out
        source_lane.status = AFCLaneState.NONE
        self.lane_not_ready(source_lane)

        # Tell OAMS hardware to load the new spool
        try:
            success, message = self._oams_load(target_lane)
            if not success:
                self.logger.error(f"Same-FPS reload failed: {message}")
                self.request_pause(
                    f"Same-FPS reload failed for {target_lane_name}: {message}",
                    lane_name=target_lane_name)
                return False
        except Exception as e:
            self.logger.error(f"Same-FPS reload exception: {e}")
            self.request_pause(
                f"Same-FPS reload failed for {target_lane_name}: {e}",
                lane_name=target_lane_name)
            return False

        # Remap T# so the slicer's tool command now points to the new lane
        source_map = source_lane.map
        if source_map:
            afc.gcode.run_script_from_command(
                f'SET_MAP LANE={target_lane_name} MAP={source_map}')
            self.logger.info(
                f"Remapped {source_map} from {source_lane_name} to {target_lane_name}")

        # Update AFC state — new lane is loaded
        target_lane.set_tool_loaded()
        self.lane_tool_loaded(target_lane)
        afc.current = target_lane_name
        afc.save_vars()

        self.logger.info(
            f"Same-FPS reload complete: {target_lane_name} now active")
        return True

    def load_sequence(self, cur_lane, cur_hub, cur_extruder):
        """OpenAMS load sequence — AFC-owned orchestration.

        Handles dock purge wrapper, temperature management, and delegates
        the hardware load to OAMS directly. AFC is master control: dock purge
        happens here, same pattern as ACE.

        :param cur_lane: The lane object to be loaded.
        :param cur_hub: The hub object associated with the lane (unused for OpenAMS).
        :param cur_extruder: The extruder object associated with the lane.
        :return bool: True if load was successful, False on error.
        """
        afc = self.afc

        # Check if this lane is already loaded to toolhead — sync state and skip
        if cur_lane.get_toolhead_pre_sensor_state() and hasattr(cur_lane, 'tool_loaded') and cur_lane.tool_loaded:
            self.logger.debug(f"Lane {cur_lane.name} already loaded to toolhead, skipping load")
            cur_lane.set_tool_loaded()
            afc.save_vars()
            return True

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        if afc.afcDeltaTime.start_time is None:
            afc.afcDeltaTime.set_start_time()
        else:
            now = datetime.now()
            afc.afcDeltaTime.major_delta_time = now
            afc.afcDeltaTime.last_time = now

        # Pre-load check: if a DIFFERENT lane is loaded on this extruder,
        # auto-unload it first. AFC owns this decision.
        existing_lane = cur_extruder.lane_loaded
        if existing_lane and existing_lane != cur_lane.name:
            self.logger.info(
                f"Auto-unloading {existing_lane} before loading {cur_lane.name} "
                f"(AFC pre-load cleanup)")
            try:
                afc.gcode.run_script_from_command(f"TOOL_UNLOAD LANE={existing_lane}")
                afc.toolhead.wait_moves()
            except Exception as e:
                afc.error.handle_lane_failure(
                    cur_lane,
                    f"Failed to auto-unload {existing_lane} before loading {cur_lane.name}: {e}")
                return False

        # Dock purge phase 1: drop off tool before feeding filament
        dock_dropped_off = False
        if self._is_dock_purge_enabled():
            self.logger.info("OAMS dock purge: dropping tool off at dock before feed")
            self._dock_purge_dropoff()
            dock_dropped_off = True
            afc.afcDeltaTime.log_with_time("OAMS: After dock purge dropoff")

        # Suspend monitor for the entire dock purge cycle (load + purge + pickup).
        # Without this, clog detection fires during dock purge extrusion
        # (extruder pushes but encoder doesn't move = false clog).
        if dock_dropped_off and self._monitor is not None:
            self._monitor.stop()

        # Wrap the load so tool is always picked back up, even on failure
        load_result = False
        try:
            afc._suppress_tool_swap_timer = True
            self.logger.debug(
                f"OpenAMS load: loading {cur_lane.name} via OAMS hardware"
            )

            success, message = self._oams_load(cur_lane)
            if not success:
                message = message or f"OpenAMS load failed for {cur_lane.name}"
                afc.error.handle_lane_failure(cur_lane, message)
                return False

            load_result = True
        except Exception as e:
            message = "OpenAMS load failed for {}: {}".format(cur_lane.name, str(e))
            afc.error.handle_lane_failure(cur_lane, message)
            return False
        finally:
            afc._suppress_tool_swap_timer = False
            if dock_dropped_off:
                # Always pick up tool — even on failure
                if load_result:
                    # Success: purge in dock, then pick up
                    purge_length = getattr(self.oams, 'post_load_purge', 0.0) or 0.0
                    if purge_length > 0:
                        purge_speed = getattr(cur_extruder, 'tool_load_speed', 7.0)
                        self.logger.info(
                            f"OAMS dock purge: extruding {purge_length:.1f}mm "
                            f"@ {purge_speed}mm/s in dock, then picking up"
                        )
                        afc.move_e_pos(purge_length, purge_speed, "dock purge extrude")
                else:
                    self.logger.info("OAMS dock purge: picking up tool after load failure")
                self._dock_purge_pickup()
                afc.afcDeltaTime.log_with_time("OAMS: After dock purge pickup")
                # Monitor was started inside _oams_load on success (with
                # notify_load_complete). Don't restart here — on failure
                # no filament is loaded so monitoring would be false.

        if not cur_lane.get_toolhead_pre_sensor_state() and not cur_lane.extruder_obj.on_shuttle():
            message = (
                "OpenAMS load did not trigger pre extruder gear toolhead sensor, CHECK FILAMENT PATH\n"
                "||=====||====||==>--||\nTRG   LOAD   HUB   TOOL"
            )
            message += "\nTo resolve set lane loaded with `SET_LANE_LOADED LANE={}` macro.".format(cur_lane.name)
            if afc.function.in_print():
                message += "\nOnce filament is fully loaded click resume to continue printing"
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        cur_lane.set_tool_loaded()
        # Enable FPS buffer for Mainsail display and fault detection
        cur_lane.enable_buffer(disable_fault=True)
        afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, cur_hub, cur_extruder):
        """OpenAMS unload sequence - uses shared toolhead steps then delegates to OAMSManager.

        The toolhead steps (cut, form tip, park) are shared with stock AFC since they
        happen at the toolhead. After those, retraction is delegated to OAMSManager
        instead of using stepper-based retraction.

        Called by AFC's upstream delegation hook:
            if hasattr(cur_lane.unit_obj, 'unload_sequence'):
                return cur_lane.unit_obj.unload_sequence(...)

        :param cur_lane: The lane object to be unloaded.
        :param cur_hub: The hub object associated with the lane (unused for OpenAMS).
        :param cur_extruder: The extruder object associated with the lane.
        :return bool: True if unload was successful, False on error.
        """
        afc = self.afc

        cur_lane.status = AFCLaneState.TOOL_UNLOADING

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Set follower reverse before cut so it assists all retract phases
        if self._follower is not None and self.oams is not None:
            fps_state = self._get_monitor_state()
            if fps_state:
                self._follower.set_follower_state(
                    fps_state, self.oams, 1, 0, "before unload cut", force=True)

        # Quick pull to prevent oozing
        afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False)
        afc.function.log_toolhead_pos("TOOL_UNLOAD quick pull: ")
        self.lane_unloading(cur_lane)
        cur_lane.sync_to_extruder()
        cur_lane.do_enable(True)
        cur_lane.select_lane()

        # Shared toolhead steps: cut, park, form tip
        if afc.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            afc.gcode.run_script_from_command(afc.tool_cut_cmd)
            afc.afcDeltaTime.log_with_time("TOOL_UNLOAD: After cut")
            afc.function.log_toolhead_pos()

            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)
                afc.afcDeltaTime.log_with_time("TOOL_UNLOAD: After park")
                afc.function.log_toolhead_pos()

        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)
                afc.afcDeltaTime.log_with_time("TOOL_UNLOAD: After form tip park")
                afc.function.log_toolhead_pos()

            if afc.form_tip_cmd == "AFC":
                afc.tip.tip_form()
                afc.afcDeltaTime.log_with_time("TOOL_UNLOAD: After afc form tip")
                afc.function.log_toolhead_pos()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)
                afc.afcDeltaTime.log_with_time("TOOL_UNLOAD: After custom form tip")
                afc.function.log_toolhead_pos()

        try:
            # Unsync from extruder before OpenAMS unload
            # After cut/form_tip, lane is synced to extruder. Must unsync before
            # AFC hardware unload can control the spool independently.
            cur_lane.unsync_to_extruder()

            self.logger.debug(
                f"OpenAMS unload: unloading {cur_lane.name} via OAMS hardware"
            )
            success, message = self._oams_unload(cur_lane)
            if not success:
                message = message or "OpenAMS unload failed for {}".format(cur_lane.name)
                afc.error.handle_lane_failure(cur_lane, message)
                return False

            # After unload, read the actual hub sensor to determine if filament
            # is still at the hub.  The OAMS unload retracts filament back to the
            # spool bay (past the hub sensor), so hub_hes_value will typically be 0.
            # Hardcoding True here previously caused stale hub status on swaps.
            hub_loaded = False
            try:
                spool_idx = self._get_openams_spool_index(cur_lane)
                hub_values = getattr(self.oams, "hub_hes_value", None)
                if hub_values is not None and spool_idx is not None and 0 <= spool_idx < len(hub_values):
                    hub_loaded = bool(hub_values[spool_idx])
            except Exception:
                hub_loaded = False
            cur_lane.loaded_to_hub = hub_loaded
            # Clear the virtual hub sensor so Mainsail/status reflects
            # that the hub is no longer occupied after unload.
            hub_obj = getattr(cur_lane, "hub_obj", None)
            if hub_obj is not None and hasattr(hub_obj, "switch_pin_callback"):
                hub_obj.switch_pin_callback(
                    self.reactor.monotonic(), hub_loaded
                )
            cur_lane.set_tool_unloaded()
            cur_lane.status = AFCLaneState.LOADED
            self.lane_tool_unloaded(cur_lane)
            afc.save_vars()
        except Exception as e:
            message = "OpenAMS unload failed for {}: {}".format(cur_lane.name, str(e))
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        return True

    def lane_unload(self, cur_lane):
        """Block manual LANE_UNLOAD for OpenAMS lanes.

        OpenAMS units have no AFC stepper path for lane ejection. Users should
        remove spools physically or use TOOL_UNLOAD for toolhead-side unloads.

        Also handles cross-extruder runout FPS state cleanup when triggered
        during infinite spool scenarios.
        """
        lane_name = getattr(cur_lane, "name", "unknown")

        # Handle cross-extruder runout FPS state cleanup
        if getattr(cur_lane, '_oams_cross_extruder_runout', False):
            try:
                # Clear our own monitor state for this lane
                if self._monitor is not None:
                    self._monitor.notify_unload_complete()
                spool_index = self._get_openams_spool_index(cur_lane)
                fps_name = self._get_fps_name()
                if fps_name:
                    self.logger.debug(f"Cross-Extruder: Cleared FPS state for {fps_name} (was spool {spool_index})")
                    try:
                        AMSRunoutCoordinator.notify_lane_tool_state(
                            self.printer, self.oams_name, lane_name,
                            loaded=False, spool_index=spool_index,
                            eventtime=self.reactor.monotonic())
                    except Exception as e:
                        self.logger.error(f"Failed to notify AFC about lane {lane_name} unload: {e}")
            except Exception as e:
                self.logger.error(f"Failed to clear FPS state for lane {lane_name}: {e}")
            cur_lane._oams_cross_extruder_runout = False

        message = (
            f"LANE_UNLOAD is not supported for OpenAMS lane {lane_name}. "
            "OpenAMS units handle filament automatically - just remove the spool physically. "
            "Use TOOL_UNLOAD if you need to unload from the toolhead."
        )
        self.logger.info(message)
        return True

    def on_lane_unset_loaded(self, lane, extruder_name):
        """Update internal state when a lane is manually unset from the toolhead."""
        lane_name = getattr(lane, "name", None)
        if not lane_name:
            return
        try:
            # Clear monitor FPS state
            if self._monitor is not None:
                self._monitor.notify_unload_complete()
            # Clear current_spool on hardware
            if self.oams is not None:
                self.oams.current_spool = None
            # Update follower based on remaining hub state
            if self._follower is not None and self.oams is not None:
                oams_name = self.oams_name
                hub_values = getattr(self.oams, 'hub_hes_value', None)
                any_hub = any(hub_values) if hub_values else False
                if any_hub:
                    # Keep follower on — other bays still have filament
                    pass
                else:
                    fps_state = self._get_monitor_state()
                    if fps_state:
                        self._follower.set_follower_state(
                            fps_state, self.oams, 0, 0, "lane unset", force=True)
            self.logger.debug(f"Updated state for lane {lane_name} unset from {extruder_name}")
        except Exception as e:
            self.logger.error(f"Failed to update state during unset_lane_loaded: {e}")

    def eject_lane(self, lane):
        """Block eject for OpenAMS lanes.

        OpenAMS units have no AFC stepper path for lane ejection. Users should
        remove spools physically or use TOOL_UNLOAD for toolhead-side unloads.
        """
        lane_name = getattr(lane, "name", "unknown")
        message = (
            f"Eject is not supported for OpenAMS lane {lane_name}. "
            "OpenAMS units handle filament automatically - just remove the spool physically. "
            "Use TOOL_UNLOAD if you need to unload from the toolhead."
        )
        self.logger.info(message)

    def abort_load(self, cur_lane):
        """Cancel an in-progress OpenAMS load operation.

        Sends load_spool_cancel() to stop the hardware motor, waits for
        the firmware ACK, and syncs Klipper-side state so that a subsequent
        unload_spool() can proceed correctly.
        """
        try:
            spool_index = self._get_openams_spool_index(cur_lane)
            if spool_index is not None:
                self.logger.info(
                    "Aborting OpenAMS load for %s (spool %d)"
                    % (cur_lane.name, spool_index))
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            else:
                self.logger.warning(
                    "abort_load: could not resolve spool index for %s"
                    % cur_lane.name)
        except Exception as e:
            self.logger.error(
                "abort_load failed for %s: %s" % (cur_lane.name, e))

    def prep_load(self, lane):
        """No-op for OpenAMS: hardware drives filament to the load sensor directly."""

    def prep_post_load(self, lane):
        """No-op for OpenAMS: hardware handles hub loading internally."""

    def _get_fps_id_for_lane(self, lane_name: str) -> Optional[str]:
        """Get FPS buffer name for a lane via AFC's extruder config."""
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return None
        extruder = getattr(lane, 'extruder_obj', None)
        if extruder is None:
            return None
        return getattr(extruder, 'buffer_name', None)

    def _format_openams_calibration_command(self, base_command, lane):
        if base_command not in {"OAMS_CALIBRATE_HUB_HES", "OAMS_CALIBRATE_PTFE_LENGTH"}:
            return None

        oams_index = self._get_openams_index()
        spool_index = self._get_openams_spool_index(lane)

        if oams_index is None or spool_index is None:
            lane_name = getattr(lane, "name", lane)
            self.logger.warning(f"Unable to format OpenAMS calibration command for lane {lane_name} on unit {self.name}")
            return None

        if base_command == "OAMS_CALIBRATE_HUB_HES":
            return f"AFC_OAMS_CALIBRATE_HUB_HES UNIT={self.name} SPOOL={spool_index}"

        return f"AFC_OAMS_CALIBRATE_PTFE UNIT={self.name} SPOOL={spool_index}"

    def cmd_UNIT_CALIBRATION(self, gcmd):
        """Override base calibration menu to show OpenAMS-specific options."""
        if not self._is_openams_unit():
            super().cmd_UNIT_CALIBRATION(gcmd)
            return

        prompt = AFCprompt(gcmd, self.logger)
        title = f"{self.name} Calibration"
        text = "Select OpenAMS calibration type"
        buttons = []

        # HUB HES calibration button
        buttons.append(("Calibrate HUB HES", f"UNIT_LANE_CALIBRATION UNIT={self.name}", "primary"))

        # PTFE calibration button
        buttons.append(("Calibrate PTFE Length", f"UNIT_PTFE_CALIBRATION UNIT={self.name}", "secondary"))

        any_lane_has_td1_ids = any(lane.td1_device_id for lane in self.lanes.values())
        if self.afc.td1_defined and (self.td1_device_id or any_lane_has_td1_ids):
            buttons.append(("Calibrate TD-1 Length", f"AFC_UNIT_TD_ONE_CALIBRATION UNIT={self.name}", "secondary"))

        # Back button
        back = [("Back", "AFC_CALIBRATION", "info")]

        prompt.create_custom_p(title, text, None, True, [buttons], back)

    def cmd_UNIT_PTFE_CALIBRATION(self, gcmd):
        """Show PTFE calibration menu with buttons for each loaded lane."""
        if not self._is_openams_unit():
            self.logger.info("PTFE calibration is only available for OpenAMS units.")
            return

        # Check if any lane on THIS UNIT is loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                self.logger.info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"{self.name} PTFE Length Calibration"
        text = (
            "Select any loaded lane from {} to calibrate PTFE length. "
            "Only one lane needs to be calibrated — each bay has an equal "
            "length path to the hub."
        ).format(self.name)

        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue

            button_command = self._format_openams_calibration_command(
                "OAMS_CALIBRATE_PTFE_LENGTH", lane
            )
            if button_command is None:
                continue

            button_label = f"{lane}"
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, button_command, button_style))

            index += 1
            if index % 2 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]

        prompt.create_custom_p(title, text, None, True, buttons, back)

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        """Override base prompt to expose an all-lane HUB HES calibration action."""
        if not self._is_openams_unit():
            super().cmd_UNIT_LANE_CALIBRATION(gcmd)
            return

        # Check if any lane on THIS UNIT is loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                self.logger.info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"{self.name} Lane Calibration"
        text = (
            "Select a loaded lane from {} to calibrate HUB HES using OpenAMS. "
            "Command: OAMS_CALIBRATE_HUB_HES"
        ).format(self.name)

        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue

            button_command = self._format_openams_calibration_command(
                "OAMS_CALIBRATE_HUB_HES", lane
            )
            if button_command is None:
                continue

            button_label = f"{lane}"
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, button_command, button_style))

            index += 1
            if index % 2 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        all_lanes = None
        if total_buttons > 1:
            all_lanes = [
                (
                    "Calibrate All HUB HES",
                    f"AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={self.name}",
                    "default",
                )
            ]

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]

        prompt.create_custom_p(title, text, all_lanes, True, buttons, back)

    def handle_connect(self):
        """Initialise the AMS unit and configure custom logos."""
        super().handle_connect()
        # Pre-warm object caches for faster runtime access
        if self._cached_gcode is None:
            try:
                self._cached_gcode = self.printer.lookup_object("gcode")
            except Exception:
                pass

        # Pre-cache OAMS index
        if self.oams is not None and self._cached_oams_index is None:
            self._cached_oams_index = getattr(self.oams, "oams_idx", None)


        # Register internal gcode command for stuck spool auto-recovery.
        # Multiple afcAMS units share the same printer gcode namespace so ignore
        # the AlreadyRegistered error if another unit registered it first.
        try:
            self.gcode.register_command(
                '_AFC_OAMS_STUCK_RECOVERY',
                self._cmd_stuck_spool_recovery,
                desc="Internal: auto-recover from stuck spool via unload+reload",
            )
        except Exception:
            pass

        try:
            self.gcode.register_command(
                'AFC_OAMS_TEST_CANCEL',
                self._cmd_test_cancel,
                desc="Test: load lane, cancel after 500mm, unload, reload",
            )
        except Exception:
            pass

        # Strip non-FPS buffers from AMS lanes.  AMS units don't have physical
        # TurtleNeck buffers, but AFC_FPS buffers ARE valid (they provide the ADC
        # reading for proportional control).  Keep any buffer that exposes
        # get_fps_value(); remove the rest.
        for lane in self.lanes.values():
            if lane.buffer_obj is not None and not hasattr(lane.buffer_obj, 'get_fps_value'):
                self.logger.warning(
                    f"Lane {lane.name} had non-FPS buffer '{lane.buffer_obj.name}' configured, "
                    f"but OpenAMS units only support FPS buffers. Removing buffer assignment."
                )
                lane.buffer_obj = None

        #  Register each lane with the shared registry
        for lane in self.lanes.values():
            lane.prep_state = False
            lane._load_state = False
            lane.loaded_to_hub = False
            lane.status = AFCLaneState.NONE
            lane.fps_share_prep_load = getattr(lane, "load", None) is None
            # Initialize OpenAMS runout tracking attributes so downstream code
            # can use direct access instead of hasattr()/getattr() guards.
            lane._oams_runout_detected = False
            lane._oams_cross_extruder_runout = False

            idx = getattr(lane, "index", 0) - 1
            if idx >= 0 and self.registry is not None:
                lane_name = getattr(lane, "name", None)
                unit_name = self.oams_name or self.name
                extruder_name = getattr(lane, "extruder_name", None) or getattr(self, "extruder", None)

                if lane_name and extruder_name:
                    try:
                        self.registry.register_lane(
                            lane_name=lane_name,
                            unit_name=unit_name,
                            spool_index=idx,
                            extruder=extruder_name,
                            fps_name=None,
                            hub_name=getattr(lane, "hub", None),
                            led_index=getattr(lane, "led_index", None),
                            custom_load_cmd=getattr(lane, "custom_load_cmd", None),
                            custom_unload_cmd=getattr(lane, "custom_unload_cmd", None),
                        )
                    except Exception as e:
                        self.logger.error(f"Failed to register lane {lane_name} with registry: {e}")
        self.logo  = '<span class=success--text>R  __________________\n'
        self.logo += 'E | Open  |   AMS    |\n'
        self.logo += 'A | (o) (o) (o) (o)  |\n'
        self.logo += 'D |__________________|\n'
        self.logo += 'Y |_1__|_2__|_3__|_4_|\n'
        self.logo += '  ' + self.name + '</span>\n'

        self.logo_error  = '<span class=error--text>E  __________________\n'
        self.logo_error += 'R | Open  |   AMS    |\n'
        self.logo_error += 'R | (X) (X) (X) (X)  |\n'
        self.logo_error += 'O |__________________|\n'
        self.logo_error += 'R |_X__|_X__|_X__|_X_|\n'
        self.logo_error += '  ' + self.name + '</span>\n'

    def _lane_matches_extruder(self, lane) -> bool:
        """Return True if the lane is mapped to this AMS unit's extruder."""
        extruder_name = getattr(self, "extruder", None)
        unit_extruder_obj = getattr(self, "extruder_obj", None)
        if not extruder_name:
            return False

        lane_extruder = getattr(lane, "extruder_name", None)
        if lane_extruder is None:
            lane_extruder_obj = getattr(lane, "extruder_obj", None)
            lane_extruder = getattr(lane_extruder_obj, "name", None)
        else:
            lane_extruder_obj = getattr(lane, "extruder_obj", None)

        if unit_extruder_obj is not None and lane_extruder_obj is unit_extruder_obj:
            return True

        if lane_extruder == extruder_name:
            return True

        normalized_lane = normalize_extruder_name(lane_extruder)
        normalized_unit = normalize_extruder_name(extruder_name)

        if normalized_lane and normalized_unit and normalized_lane == normalized_unit:
            return True

        return False

    def _lane_reports_tool_filament(self, lane, sync_only: bool = False) -> Optional[bool]:
        """Return the best-known tool filament state for a lane.

        Args:
            lane: The lane to check
            sync_only: If True, only trust extruder.lane_loaded (for post-reboot sync).
                      If False, fall through to lane state for in-progress loads.

        Checks if the EXTRUDER thinks this lane is loaded, not just if the
        lane thinks it's loaded (to avoid stale state after reboot).
        """
        if lane is None:
            return None

        lane_name = getattr(lane, "name", None)
        lane_unit = getattr(lane, "unit", None)

        # Only apply AMS virtual sensor logic to lanes that belong to any OpenAMS unit
        if lane_unit:
            sync_units = self.__class__._sync_instances
            if lane_unit not in sync_units:
                return None
            if lane_unit != getattr(self, "name", None):
                return None

        # Check if the extruder thinks THIS lane is loaded (authoritative)
        extruder = getattr(lane, "extruder_obj", None)
        if extruder is not None:
            lane_loaded = getattr(extruder, "lane_loaded", None)
            if lane_loaded == lane_name:
                # Extruder confirms this lane is loaded
                return True
            elif lane_loaded is not None and lane_loaded != lane_name:
                # Extruder has a different lane loaded.
                # If the loaded lane doesn't belong to this unit, return None
                # (no opinion) so we don't override another unit's sensor state.
                if lane_loaded not in self.lanes:
                    return None
                return False
            elif lane_loaded is None:
                # Extruder lost track (e.g., during BT_PREP system test). Rehydrate from saved state, then lane state
                lane_has_filament = bool(getattr(lane, "tool_loaded", False) and getattr(lane, "load_state", False))
                if not lane_has_filament:
                    # Fall back to live OAMS sensors for this lane to avoid toolchanger prep clearing LEDs
                    try:
                        sensor_snapshot = self._get_oams_sensor_snapshot({lane_name: lane}, require_hub=True)
                        lane_has_filament = bool(sensor_snapshot.get(lane_name, False) and getattr(lane, "tool_loaded", False))
                    except Exception:
                        lane_has_filament = False

                saved_lane_loaded = self._get_saved_extruder_lane_loaded(getattr(extruder, "name", None))
                canonical_saved = self._canonical_lane_name(saved_lane_loaded)
                canonical_lane = self._canonical_lane_name(lane_name)

                if lane_has_filament and canonical_saved is not None:
                    if canonical_saved == canonical_lane:
                        try:
                            extruder.lane_loaded = lane_name
                            self.logger.debug(f"Restored extruder.lane_loaded to {lane_name} from AFC.var.unit during sync")
                            return True
                        except Exception as e:
                            self.logger.debug(f"Failed to restore extruder.lane_loaded from AFC.var.unit during sync: {e}")
                    else:
                        return False

                if lane_has_filament:
                    try:
                        extruder.lane_loaded = lane_name
                        self.logger.debug(f"Restored extruder.lane_loaded to {lane_name} based on lane state during sync")
                        return True
                    except Exception as e:
                        self.logger.debug(f"Failed to restore extruder.lane_loaded during sync: {e}")
                if sync_only:
                    # Post-reboot sync: extruder says nothing loaded, trust it over stale lane state
                    return False
            # else: lane_loaded is None but sync_only=False, fall through to check lane state
            # This allows in-progress loads to work (lane.load_state changes before lane_loaded)

        # Fallback: check lane's own state
        # During normal operation: reflects in-progress loads (hub sensor detects filament)
        # During sync_only: should not reach here (extruder is not None for AMS lanes)
        load_state = getattr(lane, "load_state", None)
        if load_state is not None:
            # Only trust load_state when lane.tool_loaded is also True.
            # Prevents stale saved load_state from re-marking lanes as loaded at startup.
            if getattr(lane, "tool_loaded", False):
                return bool(load_state)
            return False if sync_only else None

        if getattr(lane, "tool_loaded", False):
            return True

        return None

    def lane_tool_loaded(self, lane):
        """Update the virtual tool sensor when a lane loads into the tool."""
        lane_name = getattr(lane, "name", None)
        extruder_obj = getattr(lane, "extruder_obj", None)
        super().lane_tool_loaded(lane)

        # When a new lane loads to toolhead, clear tool_loaded on any OTHER lanes from this unit
        # that are on the SAME FPS/extruder (each FPS can have its own lane loaded)
        # This handles cross-Extruder runout where AFC switches from OpenAMS lane to different Extruder/FPS lane
        lane_extruder = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        if lane_extruder:
            for other_lane in self.lanes.values():
                if other_lane.name == lane.name:
                    continue
                if not getattr(other_lane, 'tool_loaded', False):
                    continue
                # Check if other lane is on same extruder/FPS
                other_extruder = getattr(other_lane.extruder_obj, "name", None) if hasattr(other_lane, "extruder_obj") else None

                if other_extruder == lane_extruder:
                    # PHASE 1 REFACTOR: Use AFC native set_tool_unloaded() instead of direct assignment
                    # This ensures proper state cleanup and callbacks
                    other_lane.set_tool_unloaded()
                    other_lane._oams_runout_detected = False
                    self.logger.debug(f"Cleared tool_loaded for {other_lane.name} on same FPS (new lane {lane.name} loaded)")

        # Sync OAMS MCU current_spool only when this callback aligns with AFC's
        # active load transition (current_loading) or the extruder mapping already
        # confirms the lane. This prevents stray/late duplicate callbacks from
        # clobbering active spool tracking.
        if self.oams is not None:
            try:
                current_extruder_lane = getattr(extruder_obj, "lane_loaded", None) if extruder_obj is not None else None
                afc_loading_lane = getattr(self.afc, "current_loading", None)
                lane_confirmed = current_extruder_lane == lane_name
                lane_is_loading = afc_loading_lane == lane_name
                should_update_current_spool = lane_confirmed or lane_is_loading

                if should_update_current_spool:
                    spool_index = getattr(lane, 'index', None)
                    if spool_index is not None:
                        new_spool = spool_index - 1
                        # Skip if already set to this spool (avoid redundant MCU commands)
                        if self.oams.current_spool != new_spool:
                            self.oams.current_spool = new_spool
                            self.logger.debug(f"Set OAMS current_spool to {new_spool} for {getattr(lane, 'name', None)}")
                    if lane_name is not None:
                        self._last_lane_tool_loaded_sync[lane_name] = (
                            bool(getattr(lane, "tool_loaded", False)),
                            current_extruder_lane,
                        )
                else:
                    self.logger.debug(
                        f"Ignoring lane_tool_loaded current_spool update for {lane_name}: "
                        f"lane_loaded={current_extruder_lane}, current_loading={afc_loading_lane}"
                    )
            except Exception as e:
                self.logger.error(f"Failed to update OAMS current_spool for {getattr(lane, 'name', None)}: {e}")

        if not self._lane_matches_extruder(lane):
            return

        # Wait for all moves to complete to prevent "Timer too close" errors
        try:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.wait_moves()
            # Add a small delay to allow the MCU to catch up
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        except Exception:
            pass

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)

    def lane_tool_unloaded(self, lane):
        """Update the virtual tool sensor when a lane unloads from the tool."""
        super().lane_tool_unloaded(lane)

        # PHASE 1 REFACTOR: Removed redundant lane.tool_loaded = False
        # Parent's lane_tool_unloaded() already calls lane.set_tool_unloaded() which sets tool_loaded = False

        # Clear runout flag if set
        lane._oams_runout_detected = False

        if not self._lane_matches_extruder(lane):
            return

        # Wait for all moves to complete to prevent "Timer too close" errors
        try:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.wait_moves()
            # Add a small delay to allow the MCU to catch up
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        except Exception:
            pass

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)

    def _unit_matches(self, unit_value: Optional[str]) -> bool:
        """Return True when a mux UNIT value targets this AMS instance."""
        if not unit_value:
            return True

        normalized = unit_value.strip().strip('"').strip("'")
        if not normalized:
            return True

        if normalized == self.name:
            return True

        lowered = normalized.lower()
        if lowered == self.name.lower():
            return True

        parts = normalized.replace("_", " ").replace("-", " ").split()
        return any(part.lower() == self.name.lower() for part in parts)

    def _normalize_lane_alias(self, alias: Optional[str]) -> Optional[str]:
        """Return a trimmed lane alias token for comparison."""
        if not alias or not isinstance(alias, str):
            return None

        normalized = alias.strip()
        if not normalized:
            return None

        if " " in normalized:
            normalized = normalized.split()[-1]

        return normalized

    def _resolve_lane_alias(self, identifier: Optional[str]) -> Optional[str]:
        """Map common aliases (fps names, case variants) to lane objects."""
        if not identifier:
            return None

        lookup = identifier.strip()
        if not lookup:
            return None

        lane = self.lanes.get(lookup)
        if lane is not None:
            return lane.name

        lowered = lookup.lower()
        normalized_lookup = self._normalize_lane_alias(lookup)
        for lane in self.lanes.values():
            if lane.name.lower() == lowered:
                return lane.name

            lane_map = getattr(lane, "map", None)
            if isinstance(lane_map, str) and lane_map.lower() == lowered:
                return lane.name

            canonical_map = self._normalize_lane_alias(lane_map)
            if (canonical_map is not None and normalized_lookup is not None and canonical_map.lower() == normalized_lookup.lower()):
                return lane.name

        return None

    def _canonical_lane_name(self, lane_name: Optional[str]) -> Optional[str]:
        """Return a consistent lane identifier for latch/feed tracking."""
        if lane_name is None:
            return None

        lookup = lane_name.strip() if isinstance(lane_name, str) else str(lane_name).strip()
        if not lookup:
            return None

        resolved = self._resolve_lane_alias(lookup)
        if resolved:
            return resolved

        return lookup

    def record_load(self, extruder: Optional[str] = None, lane_name: Optional[str] = None) -> Optional[str]:
        canonical = self._canonical_lane_name(lane_name)
        return canonical

    def _get_extruder_object(self, extruder_name: Optional[str]):
        if not extruder_name:
            return None
        cached = self._cached_extruder_objects.get(extruder_name)
        if cached is not None:
            return cached
        extruder = self.printer.lookup_object(f"AFC_extruder {extruder_name}", None)
        if extruder is not None:
            self._cached_extruder_objects[extruder_name] = extruder
        return extruder

    def _current_lane_for_extruder(self, extruder_name: Optional[str]) -> Optional[str]:
        extruder = self._get_extruder_object(extruder_name)
        lane_name = getattr(extruder, "lane_loaded", None) if extruder else None
        return self._canonical_lane_name(lane_name)

    def _get_lane_object(self, lane_name: Optional[str]):
        canonical = self._canonical_lane_name(lane_name)
        if canonical is None:
            return None
        # Check local lanes dict first
        lane = self.lanes.get(canonical)
        if lane is not None:
            return lane
        cached = self._cached_lane_objects.get(canonical)
        if cached is not None:
            return cached
        lane = self.printer.lookup_object(f"AFC_lane {canonical}", None)
        if lane is not None:
            self._cached_lane_objects[canonical] = lane
        return lane

    def _saved_unit_file_path(self) -> Optional[str]:
        base_path = getattr(self.afc, "VarFile", None)
        if not base_path:
            return None
        return os.path.expanduser(str(base_path) + ".unit")

    def _load_saved_unit_snapshot(self) -> Optional[Dict[str, Any]]:
        filename = self._saved_unit_file_path()
        if not filename:
            return None

        try:
            mtime = os.path.getmtime(filename)
        except OSError:
            self._saved_unit_cache = None
            self._saved_unit_mtime = None
            return None

        if self._saved_unit_cache is not None and self._saved_unit_mtime == mtime:
            return self._saved_unit_cache

        try:
            with open(filename, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            self.logger.debug(f"Failed to read saved AFC unit data from {filename}")
            self._saved_unit_cache = None
        else:
            self._saved_unit_cache = data if isinstance(data, dict) else None

        self._saved_unit_mtime = mtime
        return self._saved_unit_cache

    def _get_unit_snapshot(self) -> Optional[Dict[str, Any]]:
        """Get AFC.var.unit snapshot from file.

        AFC doesn't have a .var object attribute - it saves to VarFile + '.unit' file.
        Always load from the saved file which is kept up-to-date by AFC's save_vars().
        """
        return self._load_saved_unit_snapshot()

    def _find_lane_snapshot(self, lane_name: Optional[str], unit_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        canonical = self._canonical_lane_name(lane_name)
        if canonical is None:
            return None

        snapshot = self._get_unit_snapshot()
        if not isinstance(snapshot, dict):
            return None

        unit_key = unit_name
        if isinstance(unit_key, str) and ":" in unit_key:
            unit_key = unit_key.split(":", 1)[0]

        if unit_key:
            unit_data = snapshot.get(unit_key)
            if isinstance(unit_data, dict):
                lane_data = unit_data.get(canonical)
                if isinstance(lane_data, dict):
                    return lane_data

        for unit_key, unit_data in snapshot.items():
            if unit_key in ("system", "Tools") or not isinstance(unit_data, dict):
                continue
            lane_data = unit_data.get(canonical)
            if isinstance(lane_data, dict):
                return lane_data

        return None

    def _get_saved_lane_field(self, lane_name: Optional[str], field: str) -> Optional[Any]:
        """Fetch a field for a lane from the AFC.var.unit snapshot."""

        unit_key = None
        lane_obj = self._get_lane_object(lane_name)
        if lane_obj is not None:
            unit_key = getattr(lane_obj, "unit", None)
        if unit_key is None:
            unit_key = getattr(self, "name", None)
        lane_data = self._find_lane_snapshot(lane_name, unit_name=unit_key)
        if not isinstance(lane_data, dict):
            return None

        return lane_data.get(field)

    def _get_saved_extruder_lane_loaded(self, extruder_name: Optional[str]) -> Optional[str]:
        """Return the persisted lane_loaded value for an extruder from AFC.var.unit."""
        if not extruder_name:
            return None

        snapshot = self._get_unit_snapshot()
        system = snapshot.get("system") if isinstance(snapshot, dict) else None
        if not isinstance(system, dict):
            return None

        extruders = system.get("extruders")
        if not isinstance(extruders, dict):
            return None

        data = extruders.get(extruder_name)
        if not isinstance(data, dict):
            return None

        return data.get("lane_loaded")

    def _get_openams_unit_names(self) -> Set[str]:
        """Return the set of unit names that correspond to OpenAMS units."""
        units: Set[str] = set(self.__class__._sync_instances.keys())

        afc = getattr(self, "afc", None)
        afc_units = getattr(afc, "units", {}) if afc else {}
        for unit_name, unit_obj in afc_units.items():
            if _is_openams_unit(unit_obj):
                units.add(getattr(unit_obj, "name", unit_name))

        return {unit for unit in units if unit}

    def _get_oams_sensor_snapshot(self, lanes: Dict[str, Any], *, require_hub: bool = False) -> Dict[str, bool]:
        """Return a lane->filament-present map using live OAMS sensor data.

        Reads directly from OAMS hardware objects.
        """
        snapshot: Dict[str, bool] = {}
        for lane_obj in lanes.values():
            lane_name = getattr(lane_obj, "name", None)
            lane_index = getattr(lane_obj, "index", None)
            lane_unit = getattr(lane_obj, "unit", None)
            if lane_name is None or lane_index is None or lane_unit is None:
                continue

            base_unit_name = lane_unit.split(":", 1)[0]
            unit_obj = getattr(lane_obj, "unit_obj", None)
            if unit_obj is None:
                afc = getattr(self, "afc", None)
                units = getattr(afc, "units", {}) if afc else {}
                unit_obj = units.get(base_unit_name)

            if not _is_openams_unit(unit_obj):
                continue

            # Use the unit's own oams hardware object directly
            oam = getattr(unit_obj, "oams", None)
            if oam is None:
                continue

            bay_index = int(lane_index) - 1
            if bay_index < 0:
                continue

            f1s_values = getattr(oam, "f1s_hes_value", None)
            hub_values = getattr(oam, "hub_hes_value", None)

            hub_present = False
            f1s_present = False
            try:
                if hub_values and bay_index < len(hub_values):
                    hub_present = bool(hub_values[bay_index])
                if f1s_values and bay_index < len(f1s_values):
                    f1s_present = bool(f1s_values[bay_index])
            except Exception:
                pass

            has_filament = bool(hub_present or (f1s_present and not require_hub))
            snapshot[lane_name] = has_filament

        return snapshot

    def _hydrate_from_saved_state(self) -> None:
        """Restore extruder.lane_loaded from saved state when sensors confirm filament at toolhead."""
        if getattr(self, "_hydrated_from_saved", False):
            return

        afc = getattr(self, "afc", None)
        if afc is None:
            return

        lanes = getattr(afc, "lanes", {})
        tools = getattr(afc, "tools", {})

        openams_units = self._get_openams_unit_names()
        if not openams_units:
            return

        sensor_snapshot = self._get_oams_sensor_snapshot(lanes, require_hub=True)

        for extruder_name, extruder_obj in tools.items():
            if extruder_name in self.__class__._hydrated_extruders:
                continue

            saved_lane = self._get_saved_extruder_lane_loaded(extruder_name)
            current_lane = getattr(extruder_obj, "lane_loaded", None)

            lane_obj = None
            canonical_lane = None
            lane_unit = None

            for candidate in (current_lane, saved_lane):
                canonical = self._canonical_lane_name(candidate)
                if not canonical:
                    continue

                lane_obj = lanes.get(candidate) or lanes.get(canonical)
                if lane_obj is not None:
                    lane_unit = getattr(lane_obj, "unit", None) or self._get_saved_lane_field(canonical, "unit")
                    canonical_lane = getattr(lane_obj, "name", None) or canonical
                    break

                lane_unit = self._get_saved_lane_field(canonical, "unit")
                if lane_unit:
                    canonical_lane = canonical
                    break

            if not canonical_lane:
                continue

            if lane_unit and lane_unit not in openams_units:
                continue

            unit_obj = getattr(lane_obj, "unit_obj", None)
            if unit_obj is not None and not _is_openams_unit(unit_obj):
                continue

            hub_reports_filament = bool(sensor_snapshot.get(canonical_lane, False))
            lane_filament_present = False
            if lane_obj is not None:
                lane_filament_present = bool(getattr(lane_obj, "tool_loaded", False) and getattr(lane_obj, "load_state", False) and hub_reports_filament)
            else:
                saved_tool_loaded = bool(self._get_saved_lane_field(canonical_lane, "tool_loaded"))
                saved_load_state = bool(self._get_saved_lane_field(canonical_lane, "load_state"))
                lane_filament_present = bool(hub_reports_filament and saved_tool_loaded and saved_load_state)

            # Only hydrate when sensors (virtual or hardware) indicate filament at toolhead
            if not lane_filament_present:
                continue

            try:
                extruder_obj.lane_loaded = canonical_lane
                self.__class__._hydrated_extruders.add(extruder_name)
                self.logger.debug(f"Hydrated {extruder_name}.lane_loaded from saved state: {canonical_lane}")
            except Exception:
                self.logger.debug(f"Failed to hydrate {extruder_name} lane from saved state")

        self._hydrated_from_saved = True

    def _get_saved_lane_runout_target(self, lane_name: Optional[str]) -> Optional[str]:
        """Return the saved runout target for a lane from AFC.var.unit, if available."""

        saved_value = self._get_saved_lane_field(lane_name, "runout_lane")
        return saved_value

    cmd_SYNC_TOOL_SENSOR_help = "Synchronise the AMS virtual tool-start sensor with the assigned lane."
    def cmd_SYNC_TOOL_SENSOR(self, gcmd):
        cls = self.__class__

        unit_value = gcmd.get("UNIT", None)
        if not unit_value:
            unit_value = cls._extract_raw_param(gcmd.get_commandline(), "UNIT")

        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            lane_name = gcmd.get("FPS", None)

        if lane_name is None:
            commandline = gcmd.get_commandline()
            lane_name = cls._extract_raw_param(commandline, "LANE")
            if lane_name is None:
                lane_name = cls._extract_raw_param(commandline, "FPS")

        for instance in cls._sync_instances.values():
            if not instance._unit_matches(unit_value):
                continue

            resolved_lane = instance._resolve_lane_alias(lane_name)
            eventtime = instance.reactor.monotonic()

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate AMS lane state without attempting any motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                # BoxTurtle-style logic: lane isn't truly empty until hub is clear too
                # Check hub sensor before declaring empty (handles runout buffer scenarios)
                hub_loaded = cur_lane.hub_obj and cur_lane.hub_obj.state
                if not hub_loaded:
                    # Truly empty - both spool (F1S) and hub are clear
                    self.lane_not_ready(cur_lane)
                    msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
                else:
                    # Hub still has filament (runout in progress, repositioning, or residual)
                    self.lane_fault(cur_lane)
                    msg += '<span class=warning--text>Filament in hub</span>'
                    succeeded = False
            else:
                self.lane_fault(cur_lane)
                msg += '<span class=error--text> NOT READY</span>'
                cur_lane.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False
        else:
            self.lane_loaded(cur_lane)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.lane_not_ready(cur_lane)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.lane_illuminate_spool(cur_lane)

                # If tool_loaded was not saved (e.g. power cycle / crash) but the hub
                # sensor still shows filament for exactly one bay on this unit, infer
                # that this lane is loaded to the toolhead so the unload sequence can handle it.
                if not cur_lane.tool_loaded:
                    if self._infer_tool_loaded_from_hub(cur_lane):
                        msg += '<span class=primary--text> (hub-detected: auto-set as in ToolHead)</span>'

                if cur_lane.tool_loaded:
                    tool_ready = (cur_lane.get_toolhead_pre_sensor_state() or cur_lane.extruder_obj.tool_start == "buffer" or cur_lane.extruder_obj.tool_end_state or cur_lane.extruder_obj.on_shuttle())
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        try:
                            if cur_lane.extruder_obj.tc_unit_obj:
                                on_shuttle = " and toolhead on shuttle" if cur_lane.extruder_obj.on_shuttle() else ""
                        except Exception:
                            pass

                        msg += f"<span class=primary--text> in ToolHead{on_shuttle}</span>"
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>'
                        # Filament is in the toolhead so it has passed through the hub
                        if not cur_lane.loaded_to_hub:
                            cur_lane.loaded_to_hub = True
                        hub_obj = getattr(cur_lane, "hub_obj", None)
                        if hub_obj is not None and hasattr(hub_obj, "switch_pin_callback"):
                            hub_obj.switch_pin_callback(self.reactor.monotonic(), True)
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        else:
                            cur_lane.unit_obj.lane_tool_loaded_idle(cur_lane)
                    elif tool_ready:
                        msg += '<span class=error--text> error in ToolHead. Lane identified as loaded but not identified as loaded in extruder</span>'
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()

        # Trigger OpenAMS sensor reconciliation after PREP completes for this lane.
        # Keep this in AFC_OpenAMS (not AFC_prep) so OpenAMS owns its recovery behavior.
        try:
            self.afc.reactor.register_callback(
                lambda et: self.sync_openams_sensors(
                    et,
                    sync_hub=True,
                    sync_f1s=True,
                    allow_lane_clear=True,
                ),
                self.afc.reactor.monotonic() + 0.2,
            )
        except Exception as e:
            self.logger.error(f"Failed to schedule OpenAMS post-PREP sensor sync: {e}")

        return succeeded

    def _infer_tool_loaded_from_hub(self, cur_lane) -> bool:
        """During PREP: if the hub sensor is active for this lane and it is the only
        hub-loaded bay on this unit, infer that the lane is loaded to the toolhead.

        This covers power-cycle / crash-recovery scenarios where tool_loaded was not
        persisted but the physical filament is still sitting past the hub.

        Returns True if tool_loaded was inferred and set_tool_loaded() was called.
        """
        if self.oams is None:
            return False

        hub_values = getattr(self.oams, "hub_hes_value", None)
        if hub_values is None:
            return False

        spool_idx = self._get_openams_spool_index(cur_lane)
        if spool_idx is None or not (0 <= spool_idx < len(hub_values)):
            return False

        if not bool(hub_values[spool_idx]):
            return False  # This lane's hub sensor is not active

        # Safety: if any other bay on this unit also shows hub active we cannot
        # unambiguously attribute the toolhead filament to cur_lane.
        other_hub_active = any(
            bool(hub_values[i]) for i in range(len(hub_values)) if i != spool_idx
        )
        if other_hub_active:
            self.logger.debug(
                f"PREP hub inference: {cur_lane.name} hub active but other bays on "
                f"{self.name} also show hub - cannot infer tool_loaded unambiguously"
            )
            return False

        self.logger.info(
            f"PREP: {cur_lane.name} is the only hub-loaded lane on {self.name} "
            f"(spool index {spool_idx}) - auto-setting as loaded to toolhead"
        )
        cur_lane.set_tool_loaded()
        return True

    def calibrate_bowden(self, cur_lane, dis, tol):
        """OpenAMS units use different calibration commands."""
        msg = (
            "OpenAMS units do not support standard AFC bowden calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_PTFE UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={}"
        ).format(self.name, self.name, self.name)
        self.logger.info(msg)
        return False, msg, 0

    def _cancel_and_mark_loaded(self, spool_index, lane_name=None):
        """Cancel an in-progress load and mark the spool as loaded.

        The firmware cancel command stops the follower motor and considers the
        spool loaded at its current position.  We mirror that on the Klipper
        side by setting current_spool and updating the monitor fps_state
        to LOADED so that a subsequent unload_spool() works correctly.
        """
        self.oams.load_spool_cancel()
        # Wait for firmware CANCEL response to clear action_status
        deadline = self.afc.reactor.monotonic() + 5.0
        while self.oams.action_status is not None:
            if self.afc.reactor.monotonic() > deadline:
                self.logger.warning("Cancel response timeout - forcing action_status clear")
                self.oams.action_status = None
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
        # Firmware considers this spool loaded after cancel
        self.oams.current_spool = spool_index
        # Update monitor state if available
        monitor_state = self._get_monitor_state()
        if monitor_state is not None:
            from extras.AFC_OpenAMS_monitor import FPSLoadState
            monitor_state.state = FPSLoadState.LOADED

    def _cmd_test_cancel(self, gcmd):
        """Test: load a lane, cancel after 500mm, unload, then reload."""
        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            raise gcmd.error("LANE is required (e.g. AFC_OAMS_TEST_CANCEL LANE=lane11)")

        cur_lane = self.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Lane '{lane_name}' not found in {self.name}")

        if self.oams is None:
            raise gcmd.error("OpenAMS hardware not available")

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            raise gcmd.error(f"Unable to resolve spool index for {lane_name}")

        from extras.oams import OAMSStatus

        target_clicks = gcmd.get_int("CLICKS", 500)
        self.logger.info(f"Step 1: Loading {lane_name} (spool {spool_index})...")

        # --- LOAD ---
        self.oams.action_status = OAMSStatus.LOADING
        self.oams.oams_load_spool_cmd.send([spool_index])

        try:
            encoder_start = int(self.oams.encoder_clicks)
        except Exception:
            encoder_start = 0

        # Wait for encoder to reach target clicks
        deadline = self.afc.reactor.monotonic() + 30.0
        while self.afc.reactor.monotonic() < deadline:
            try:
                clicks = abs(int(self.oams.encoder_clicks) - encoder_start)
            except Exception:
                clicks = 0
            if clicks >= target_clicks:
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        try:
            final_clicks = abs(int(self.oams.encoder_clicks) - encoder_start)
        except Exception:
            final_clicks = 0
        self.logger.info(f"Step 2: Cancelling load at {final_clicks} clicks...")

        # --- CANCEL (marks spool as loaded so unload works) ---
        try:
            self._cancel_and_mark_loaded(spool_index, lane_name)
        except Exception as e:
            self.logger.info(f"  cancel error: {e}")

        self.logger.info("Step 3: Unloading...")

        # --- UNLOAD ---
        try:
            success, msg = self.oams.unload_spool()
            self.logger.info(f"  unload result: success={success} msg={msg}")
        except Exception as e:
            self.logger.info(f"  unload exception: {e}")

        try:
            self.oams.clear_errors()
        except Exception:
            pass
        self._clear_lane_state_after_td1(cur_lane)

        self.logger.info("Step 4: Reloading...")

        # --- RELOAD ---
        try:
            code, msg = self.oams.load_spool(spool_index)
            self.logger.info(f"  reload result: code={code} msg={msg}")
        except Exception as e:
            self.logger.info(f"  reload exception: {e}")

        try:
            self.oams.clear_errors()
        except Exception:
            pass
        self._clear_lane_state_after_td1(cur_lane)

        self.logger.info(f"Test complete for {lane_name}")

    def _clear_lane_state_after_td1(self, cur_lane):
        """Clear AFC-level lane loaded state that background detection may have set
        during a temporary TD-1 load, and persist the clean state."""
        try:
            if getattr(cur_lane, "tool_loaded", False):
                cur_lane.set_tool_unloaded()
                self.logger.debug(f"Cleared tool_loaded state for {cur_lane.name} after TD-1")
            elif getattr(cur_lane, "extruder_obj", None) is not None:
                if getattr(cur_lane.extruder_obj, "lane_loaded", None) == cur_lane.name:
                    cur_lane.extruder_obj.lane_loaded = None
                    self.logger.debug(f"Cleared extruder lane_loaded for {cur_lane.name} after TD-1")
        except Exception as e:
            self.logger.debug(f"Failed to clear lane state after TD-1 for {cur_lane.name}: {e}")
        self.afc.save_vars()

    def _unload_after_td1(self, cur_lane, spool_index, fps_id):
        """
        Unload filament after TD-1 operation using the proper firmware unload_spool().
        Uses the high-level unload_spool() so the firmware properly clears its internal
        state, then re-syncs current_spool with hardware via clear_errors().
        """
        # Use the proper unload_spool() which tracks action_status, waits for
        # firmware response, and clears current_spool on success
        success = False
        for attempt in range(3):
            try:
                success, msg = self.oams.unload_spool()
            except Exception as e:
                self.logger.error(f"Unload exception for {cur_lane.name}: {e}")
                success = False
                msg = str(e)
            if success:
                self.logger.info(f"TD-1 unload completed for {cur_lane.name}")
                break
            self.logger.debug(
                f"Unload attempt {attempt + 1} failed for {cur_lane.name}: {msg}"
            )

        # Always re-sync firmware state after TD-1 operations.
        # clear_errors() resets action_status/code/value and queries the firmware
        # for current_spool, ensuring Klipper and firmware agree on what's loaded.
        try:
            self.oams.clear_errors()
        except Exception as e:
            self.logger.debug(f"Failed to clear_errors after TD-1 unload for {cur_lane.name}: {e}")

        # Clear AFC-level lane loaded state that background state detection may
        # have set during our temporary TD-1 load.
        self._clear_lane_state_after_td1(cur_lane)

        if not success:
            self.logger.warning(f"TD-1 unload did not fully clear for {cur_lane.name}")

    def calibrate_td1(self, cur_lane, dis, tol):
        """
        Calibration function for automatically determining td1_bowden_length.
        Uses OAMS load/unload commands and TD-1 data detection to bracket distance.
        """
        if cur_lane.td1_device_id is None:
            msg = f"Cannot calibrate TD-1 for {cur_lane.name}, td1_device_id is a required "
            msg += "field in AFC_hub or per AFC_lane"
            return False, msg, 0

        if self.oams is None:
            msg = "OpenAMS hardware not available; cannot calibrate TD-1."
            self.logger.error(msg)
            return False, msg, 0

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            msg = f"Unable to resolve spool index for {cur_lane.name}"
            return False, msg, 0

        # Verify TD-1 is still connected before trying to get data
        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            msg = f"TD-1 device(SN: {cur_lane.td1_device_id}) not detected anymore, "
            msg += "please check before continuing to calibrate TD-1 bowden length"
            return valid, msg, 0

        self.logger.raw(f"Calibrating bowden length to TD-1 device with {cur_lane.name}")
        fps_id = self._get_fps_id_for_lane(cur_lane.name)
        if fps_id is None:
            msg = f"Unable to resolve FPS for {cur_lane.name}"
            self.logger.error(msg)
            return False, msg, 0

        # Load the spool before starting TD-1 calibration
        # The load command is needed to move filament from spool bay to hub motor
        # Set action_status so firmware responses are handled correctly
        from extras.oams import OAMSStatus
        self.oams.action_status = OAMSStatus.LOADING
        try:
            self.oams.oams_load_spool_cmd.send([spool_index])
        except Exception as e:
            self.oams.action_status = None
            self.logger.error(f"Failed to start spool load for TD-1 calibration on {cur_lane.name}: {e}")
            return False, "Failed to start spool load", 0

        # Wait for hub sensor to trigger while loading
        # IMPORTANT: Always use OAMS hardware sensor, not hub_obj (which doesn't update in real-time)
        hub_timeout = self.afc.reactor.monotonic() + 10.0
        hub_detected = False
        encoder_before = None
        check_count = 0

        self.logger.debug(f"TD-1 calibration: using OAMS hardware sensor hub_hes_value[{spool_index}] for {cur_lane.name}")

        while self.afc.reactor.monotonic() < hub_timeout:
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            check_count += 1

            hub_loaded = None
            try:
                hub_loaded = bool(self.oams.hub_hes_value[spool_index])
                if check_count % 10 == 0:
                    self.logger.debug(f"TD-1 calibration check #{check_count}: hub_hes_value[{spool_index}] = {self.oams.hub_hes_value[spool_index]}")
            except Exception as e:
                if check_count % 10 == 0:
                    self.logger.debug(f"TD-1 calibration check #{check_count}: Exception reading hub_hes_value: {e}")
                hub_loaded = None

            if hub_loaded:
                hub_detected = True
                # Read encoder immediately at hub trigger as our reference point
                try:
                    encoder_before = int(self.oams.encoder_clicks)
                except Exception:
                    encoder_before = None
                self.logger.info(
                    f"Hub sensor triggered for {cur_lane.name} after {check_count} checks "
                    f"(~{check_count * 0.1:.1f}s), encoder reference={encoder_before}"
                )
                break

        if not hub_detected:
            try:
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            except Exception:
                pass
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception as e:
                self.logger.error(f"Failed to disable follower for {cur_lane.name}: {e}")
            try:
                self.oams.unload_spool()
            except Exception:
                pass
            try:
                self.oams.clear_errors()
            except Exception:
                pass
            self._clear_lane_state_after_td1(cur_lane)
            msg = f"Hub sensor did not trigger during TD-1 calibration for {cur_lane.name}"
            self.logger.error(msg)
            return False, msg, 0

        if encoder_before is None:
            msg = "Unable to read encoder at hub trigger for TD-1 calibration"
            self.logger.error(msg)
            try:
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            except Exception:
                pass
            self._unload_after_td1(cur_lane, spool_index, fps_id)
            return False, msg, 0

        # Hub triggered and encoder reference captured - let the load keep feeding
        # and poll for TD-1 detection. No need to cancel/restart with follower.
        compare_time = datetime.now()
        td1_timeout = self.afc.reactor.monotonic() + 180.0
        td1_detected = False
        next_td1_poll = self.afc.reactor.monotonic()
        last_clicks_moved = 0
        last_progress_time = self.afc.reactor.monotonic()
        stall_logged = False
        td1_relaxed_ready = self.afc.reactor.monotonic() + 6.0
        last_scan_times = getattr(self, "_td1_last_scan_time_calibration", None)
        if last_scan_times is None:
            last_scan_times = {}
            self._td1_last_scan_time_calibration = last_scan_times

        def _capture_td1_if_fresh():
            """Returns (detected, scan_time) tuple."""
            td1_data = self.afc.moonraker.get_td1_data()
            if not td1_data or cur_lane.td1_device_id not in td1_data:
                return False, None
            data = td1_data[cur_lane.td1_device_id]
            scan_time = data.get("scan_time")
            if scan_time is None:
                return False, None
            if scan_time.endswith("+00:00Z"):
                scan_time = scan_time[:-1]
            else:
                scan_time = scan_time[:-1] + "+00:00"
            try:
                scan_time = datetime.fromisoformat(scan_time).astimezone()
            except (AttributeError, ValueError):
                return False, None
            if scan_time <= compare_time.astimezone():
                return False, None
            last_scan_time = last_scan_times.get(cur_lane.td1_device_id)
            if last_scan_time is not None and scan_time <= last_scan_time:
                return False, None
            if data.get("td") is None or data.get("color") is None:
                return False, None
            last_scan_times[cur_lane.td1_device_id] = scan_time
            return True, scan_time

        def _capture_td1_relaxed():
            """Returns (detected, scan_time) tuple."""
            td1_data = self.afc.moonraker.get_td1_data()
            if not td1_data or cur_lane.td1_device_id not in td1_data:
                return False, None
            data = td1_data[cur_lane.td1_device_id]
            if data.get("td") is None or data.get("color") is None:
                return False, None
            scan_time = datetime.now().astimezone()
            last_scan_times[cur_lane.td1_device_id] = scan_time
            return True, scan_time

        self.logger.debug(f"Hub loaded, letting load continue feeding for TD-1 detection on {cur_lane.name}")

        while self.afc.reactor.monotonic() < td1_timeout:
            try:
                encoder_now = int(self.oams.encoder_clicks)
            except Exception:
                encoder_now = encoder_before

            clicks_moved = abs(encoder_now - encoder_before)
            if clicks_moved != last_clicks_moved:
                last_clicks_moved = clicks_moved
                last_progress_time = self.afc.reactor.monotonic()
            elif (self.afc.reactor.monotonic() - last_progress_time) > 4.2:
                if not stall_logged:
                    self.logger.info(
                        f"TD-1 calibration: load stalled after {clicks_moved} clicks on {cur_lane.name}"
                    )
                    stall_logged = True

            if self.afc.reactor.monotonic() >= next_td1_poll:
                next_td1_poll = self.afc.reactor.monotonic() + 2.0
                detected, scan_time = _capture_td1_if_fresh()
                if detected:
                    td1_detected = True
                    self.logger.debug(f"TD-1 data detected for {cur_lane.name} at scan_time={scan_time}")
                    break
                if self.afc.reactor.monotonic() >= td1_relaxed_ready:
                    detected, scan_time = _capture_td1_relaxed()
                    if detected:
                        td1_detected = True
                        self.logger.debug(
                            f"TD-1 data detected (relaxed) for {cur_lane.name} at scan_time={scan_time}"
                        )
                        break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        # TD-1 detected (or timeout) - cancel the load to stop filament movement
        self.logger.debug(f"Cancelling load for {cur_lane.name} after TD-1 {'detection' if td1_detected else 'timeout'}")
        try:
            self._cancel_and_mark_loaded(spool_index, cur_lane.name)
        except Exception:
            pass

        if not td1_detected:
            self._unload_after_td1(cur_lane, spool_index, fps_id)
            msg = f"TD-1 failed to detect filament for {cur_lane.name}"
            return False, msg, 0

        # Read encoder now that load is cancelled and filament is stopped
        # Delta from hub trigger = TD-1 bowden length
        try:
            encoder_after = int(self.oams.encoder_clicks)
        except Exception:
            encoder_after = None

        if encoder_after is None:
            msg = "Unable to read encoder after TD-1 detection"
            self.logger.error(msg)
            self._unload_after_td1(cur_lane, spool_index, fps_id)
            return False, msg, 0

        encoder_delta = abs(encoder_after - encoder_before)
        self.logger.info(
            f"TD-1 calibration for {cur_lane.name}: encoder at hub={encoder_before}, "
            f"encoder at TD-1={encoder_after}, bowden length={encoder_delta}"
        )

        # Unload filament after successful TD-1 calibration
        self._unload_after_td1(cur_lane, spool_index, fps_id)

        cal_msg = f"\n td1_bowden_length: New: {encoder_delta} Old: {cur_lane.td1_bowden_length}"
        cur_lane.td1_bowden_length = encoder_delta
        fullname = cur_lane.fullname
        self.afc.function.ConfigRewrite(fullname, "td1_bowden_length", encoder_delta, cal_msg)
        cur_lane.do_enable(False)
        cur_lane.unit_obj.return_to_home()
        self.afc.save_vars()

        return True, "td1_bowden_length calibration successful", encoder_delta

    def _capture_td1_with_oams(
        self,
        cur_lane,
        *,
        require_loaded: bool,
        require_enabled: bool,
    ) -> Tuple[bool, str]:
        last_capture_time = getattr(self, "_td1_last_capture_time", None)
        if last_capture_time is not None:
            settle_delay = 4.2 - (self.afc.reactor.monotonic() - last_capture_time)
            if settle_delay > 0:
                self.afc.reactor.pause(self.afc.reactor.monotonic() + settle_delay)
        if require_enabled and not cur_lane.td1_when_loaded:
            return False, "TD-1 capture disabled"
        if cur_lane.td1_device_id is None:
            return False, "TD-1 device ID not configured"

        # Check if toolhead actually has filament loaded (not just hub)
        # tool_loaded indicates filament is loaded all the way to the toolhead
        # This is different from the lane being "current" which just means it's in the hub
        if getattr(cur_lane, "tool_loaded", False):
            self.logger.info(
                f"Cannot get TD-1 data for {cur_lane.name}, toolhead is loaded"
            )
            return False, "Toolhead is loaded"

        # Fix hub state logic - only check if hub exists and is loaded
        hub_state = False
        if cur_lane.hub_obj is not None:
            hub_state = bool(cur_lane.hub_obj.state)

        if cur_lane.td1_bowden_length is None:
            self.logger.info(
                f"td1_bowden_length not set for {cur_lane.name}, skipping TD-1 capture"
            )
            return False, "td1_bowden_length not set"
        if require_loaded and not (cur_lane.load_state or cur_lane.prep_state):
            return False, "Lane is not loaded"
        if self.oams is None:
            self.logger.error("OpenAMS hardware not available; skipping TD-1 capture")
            return False, "OpenAMS hardware not available"

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            self.logger.error(f"Unable to resolve spool index for {cur_lane.name}")
            return False, "Unable to resolve spool index"

        # Check for conflicts with other OpenAMS units
        hub_values = getattr(self.oams, "hub_hes_value", None)
        current_hub_loaded = False
        other_hub_loaded = False
        try:
            if hub_values and spool_index < len(hub_values):
                current_hub_loaded = bool(hub_values[spool_index])
                other_hub_loaded = any(
                    bool(value) for idx, value in enumerate(hub_values) if idx != spool_index
                )
        except Exception:
            current_hub_loaded = False
            other_hub_loaded = False

        if other_hub_loaded and last_capture_time is not None:
            settle_deadline = self.afc.reactor.monotonic() + 5.0
            while self.afc.reactor.monotonic() < settle_deadline:
                try:
                    hub_values = getattr(self.oams, "hub_hes_value", None)
                    if hub_values and all(not bool(value) for value in hub_values):
                        other_hub_loaded = False
                        break
                except Exception:
                    break
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        if other_hub_loaded:
            self.logger.info(
                "Skipping TD-1 capture for %s because another OpenAMS hub is already loaded",
                cur_lane.name,
            )
            return False, "Another OpenAMS hub already loaded"

        if last_capture_time is not None:
            settle_deadline = self.afc.reactor.monotonic() + 5.0
            while self.afc.reactor.monotonic() < settle_deadline:
                try:
                    hub_values = getattr(self.oams, "hub_hes_value", None)
                    if hub_values and all(not bool(value) for value in hub_values):
                        break
                except Exception:
                    break
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        if hub_state and not current_hub_loaded:
            self.logger.info(
                f"Cannot get TD-1 data for {cur_lane.name}, hub shows filament in path"
            )
            return False, "Hub shows filament in path"

        # Load the spool before starting TD-1 capture
        # The load command is needed to move filament from spool bay to follower (hub) motor
        # Set action_status so firmware responses are handled correctly
        from extras.oams import OAMSStatus
        self.oams.action_status = OAMSStatus.LOADING
        try:
            self.oams.oams_load_spool_cmd.send([spool_index])
        except Exception as e:
            self.oams.action_status = None
            self.logger.error(f"Failed to start spool load for TD-1 capture on {cur_lane.name}: {e}")
            return False, "Failed to start spool load"

        fps_id = self._get_fps_id_for_lane(cur_lane.name)
        if fps_id is None:
            self.logger.error(f"Unable to resolve FPS for {cur_lane.name}")
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                pass
            # Cancel the in-progress load (marks spool as loaded), then unload + clean up
            try:
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            except Exception:
                pass
            try:
                self.oams.unload_spool()
            except Exception:
                pass
            try:
                self.oams.clear_errors()
            except Exception:
                pass
            self._clear_lane_state_after_td1(cur_lane)
            return False, "Unable to resolve FPS"

        # Wait for hub to load (should happen within a few seconds)
        # Use OAMS hardware sensor, not hub_obj
        hub_timeout = self.afc.reactor.monotonic() + 10.0
        hub_detected = False
        self.logger.debug(f"TD-1 capture: waiting for hub sensor on {cur_lane.name}")

        while self.afc.reactor.monotonic() < hub_timeout:
            try:
                hub_detected = bool(self.oams.hub_hes_value[spool_index])
            except Exception:
                hub_detected = False
            if hub_detected:
                self.logger.info(f"Hub sensor triggered for TD-1 capture on {cur_lane.name}")
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        if not hub_detected:
            # Cancel the in-progress load (marks spool as loaded), then unload + clean up
            try:
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            except Exception:
                pass
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                pass
            try:
                self.oams.unload_spool()
            except Exception:
                pass
            try:
                self.oams.clear_errors()
            except Exception:
                pass
            self._clear_lane_state_after_td1(cur_lane)
            self.logger.error(
                f"Hub sensor did not trigger during TD-1 capture for {cur_lane.name}"
            )
            return False, "Hub sensor did not trigger"

        # Hub detected - use OAMS load flow and encoder tracking
        try:
            encoder_before = int(self.oams.encoder_clicks)
        except Exception:
            encoder_before = None

        if encoder_before is None:
            self.logger.error(
                f"Unable to read encoder before TD-1 capture for {cur_lane.name}"
            )
            # Cancel the in-progress load (marks spool as loaded), then unload + clean up
            try:
                self._cancel_and_mark_loaded(spool_index, cur_lane.name)
            except Exception:
                pass
            try:
                self.oams.unload_spool()
            except Exception:
                pass
            try:
                self.oams.clear_errors()
            except Exception:
                pass
            self._clear_lane_state_after_td1(cur_lane)
            return False, "Unable to read encoder before capture"

        target_clicks = max(0, int(cur_lane.td1_bowden_length))
        compare_time = datetime.now()
        td1_timeout = self.afc.reactor.monotonic() + 30.0
        last_scan_times = getattr(self, "_td1_last_scan_time_capture", None)
        if last_scan_times is None:
            last_scan_times = {}
            self._td1_last_scan_time_capture = last_scan_times

        def _capture_td1_if_fresh() -> bool:
            td1_data = self.afc.moonraker.get_td1_data()
            if not td1_data:
                return False
            if cur_lane.td1_device_id not in td1_data:
                return False
            data = td1_data[cur_lane.td1_device_id]
            scan_time = data.get("scan_time")
            if scan_time is None:
                return False
            if scan_time.endswith("+00:00Z"):
                scan_time = scan_time[:-1]
            else:
                scan_time = scan_time[:-1] + "+00:00"
            try:
                scan_time = datetime.fromisoformat(scan_time).astimezone()
            except (AttributeError, ValueError):
                return False
            if scan_time <= compare_time.astimezone():
                return False
            last_scan_time = last_scan_times.get(cur_lane.td1_device_id)
            if last_scan_time is not None and scan_time <= last_scan_time:
                return False
            if data.get("td") is None or data.get("color") is None:
                return False
            cur_lane.td1_data = data
            last_scan_times[cur_lane.td1_device_id] = scan_time
            self.logger.info(
                f"{cur_lane.name} TD-1 data captured: td={data.get('td')} color={data.get('color')}"
            )
            self.afc.save_vars()
            return True

        self.logger.debug(
            f"TD-1 capture: waiting for encoder to reach {target_clicks} clicks on {cur_lane.name}"
        )
        while self.afc.reactor.monotonic() < td1_timeout:
            try:
                encoder_now = int(self.oams.encoder_clicks)
            except Exception:
                encoder_now = encoder_before
            clicks_moved = abs(encoder_now - encoder_before)
            if clicks_moved >= target_clicks:
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        # Read TD-1 data while filament is at/passing the sensor
        td1_detected = _capture_td1_if_fresh()
        if not td1_detected:
            td1_wait_deadline = self.afc.reactor.monotonic() + 3.5
            while self.afc.reactor.monotonic() < td1_wait_deadline:
                if _capture_td1_if_fresh():
                    td1_detected = True
                    break
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        # Cancel the load now that we have TD-1 data (or timed out), mark as loaded, then unload
        try:
            self._cancel_and_mark_loaded(spool_index, cur_lane.name)
        except Exception:
            pass

        # Always unload after capture attempt, regardless of TD-1 read result
        self._unload_after_td1(cur_lane, spool_index, fps_id)
        self._td1_last_capture_time = self.afc.reactor.monotonic()

        if not td1_detected:
            self.logger.debug(f"TD-1 data not captured for {cur_lane.name}")
            return False, "TD-1 data not captured (unload completed)"

        return True, "TD-1 data captured"

    def prep_capture_td1(self, cur_lane):
        # require_enabled=False - capture decision is made by capture_td1_data in AFC_prep
        return self._capture_td1_with_oams(
            cur_lane,
            require_loaded=True,
            require_enabled=False,
        )

    def capture_td1_data(self, cur_lane):
        return self._capture_td1_with_oams(
            cur_lane,
            require_loaded=True,
            require_enabled=False,
        )

    def calibrate_hub(self, cur_lane, tol):
        """OpenAMS units use different calibration commands."""
        msg = (
            "OpenAMS units do not support standard AFC hub calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={}"
        ).format(self.name, self.name)
        self.logger.info(msg)
        return False, msg, 0

    def _sync_afc_from_hardware_at_startup(self):
        """Align AFC state with OpenAMS hardware sensors.

        This reconciles AFC.var.unit and AFC live state with actual hardware sensors.
        Can be called at startup or on-demand (e.g., AFC_OAMS_CLEAR_ERRORS).

        Priority hierarchy:
        1. Tool sensor (F1S/virtual) = Highest priority (actual filament in toolhead)
        2. Hub sensor = Secondary (filament present in AMS)
        3. AFC.var.unit = Lowest priority (might be stale)
        """
        if not hasattr(self, 'afc') or self.afc is None:
            self.logger.debug("State sync: AFC not available, skipping")
            return

        if not hasattr(self, 'lanes') or not self.lanes:
            self.logger.debug("State sync: No lanes configured, skipping")
            return

        # First reconcile lane-level AFC booleans from live OpenAMS sensors.
        # This is critical after crashes/restarts where AFC.var.unit persisted values
        # (load_state/prep_state/loaded_to_hub) can be stale.
        try:
            eventtime = self.reactor.monotonic()
            self.sync_openams_sensors(
                eventtime,
                sync_hub=True,
                sync_f1s=True,
                allow_lane_clear=True,
            )
        except Exception as e:
            self.logger.debug(f"State sync: initial lane sensor reconciliation failed: {e}")

        self.logger.info(f"State sync: Reconciling AFC state with {self.name} hardware sensors")

        synced_count = 0
        cleared_count = 0
        skipped_count = 0
        conflict_count = 0

        # Track which extruders have lanes claiming to be loaded
        extruder_loaded_lanes = {}  # {extruder_name: [lane_names]}

        for lane in self.lanes.values():
            try:
                lane_name = getattr(lane, 'name', None)
                if not lane_name:
                    skipped_count += 1
                    continue

                extruder_obj = getattr(lane, 'extruder_obj', None)
                if extruder_obj is None:
                    skipped_count += 1
                    continue

                extruder_name = getattr(extruder_obj, 'name', 'unknown')

                # Read ACTUAL hardware state
                # Tool sensor is the ground truth - if it shows loaded, filament is in toolhead
                tool_loaded = False
                try:
                    tool_loaded = self._lane_reports_tool_filament(lane, sync_only=False)
                except Exception:
                    pass

                # Hub sensor shows filament in AMS (but not necessarily in toolhead)
                # Read ACTUAL hardware sensor, not AFC state!
                hub_loaded = False
                if self.oams is not None:
                    try:
                        spool_index = self._get_openams_spool_index(lane)
                        if spool_index is not None:
                            hub_loaded = bool(self.oams.hub_hes_value[spool_index])
                    except Exception:
                        pass

                # Read what AFC THINKS
                afc_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                afc_thinks_this_lane = (afc_lane_loaded == lane_name)

                # Track lanes that have tool filament for conflict detection
                if tool_loaded:
                    if extruder_name not in extruder_loaded_lanes:
                        extruder_loaded_lanes[extruder_name] = []
                    extruder_loaded_lanes[extruder_name].append(lane_name)

                # RECONCILE: Tool sensor wins (it's the authoritative truth)
                if tool_loaded and not afc_thinks_this_lane:
                    # Hardware shows loaded but AFC doesn't know
                    # Only update if we're sure (no conflicts)
                    if extruder_name in extruder_loaded_lanes and len(extruder_loaded_lanes[extruder_name]) > 1:
                        # Multiple lanes show loaded for same extruder - don't auto-fix
                        self.logger.error(
                            f"State sync: Multiple lanes show tool filament for {extruder_name}: "
                            f"{extruder_loaded_lanes[extruder_name]}. Manual intervention required."
                        )
                        conflict_count += 1
                        continue

                    # Check if AFC thinks a non-AMS lane is loaded - don't override non-AMS lanes!
                    if afc_lane_loaded is not None:
                        # Look up what AFC thinks is loaded and check if it's an AMS lane
                        loaded_lane_obj = self.afc.lanes.get(afc_lane_loaded) if hasattr(self.afc, 'lanes') else None
                        if loaded_lane_obj is not None:
                            loaded_unit_obj = getattr(loaded_lane_obj, 'unit_obj', None)

                            if not _is_openams_unit(loaded_unit_obj):
                                # AFC thinks a non-AMS lane (Box Turtle, etc.) is loaded
                                # Don't auto-correct based on AMS sensors!
                                loaded_unit_type = getattr(loaded_unit_obj, 'type', None)
                                self.logger.debug(
                                    f"State sync: AMS sensors show {lane_name} on {extruder_name}, "
                                    f"but AFC thinks non-AMS lane '{afc_lane_loaded}' (type={loaded_unit_type}) is loaded. "
                                    f"Skipping auto-correction to preserve non-AMS state."
                                )
                                skipped_count += 1
                                continue

                    self.logger.info(
                        f"State sync: Sensors show {lane_name} loaded to {extruder_name}, "
                        f"but AFC thinks '{afc_lane_loaded}' is loaded. Updating AFC to match hardware."
                    )
                    try:
                        extruder_obj.lane_loaded = lane_name
                        synced_count += 1
                        # Also ensure AFC saves this state
                        if hasattr(self.afc, 'save_vars'):
                            self.afc.save_vars()
                    except Exception as e:
                        self.logger.error(f"State sync: Failed to update AFC for {lane_name}: {e}")

                elif not tool_loaded and afc_thinks_this_lane:
                    # AFC thinks loaded but no filament detected
                    if not hub_loaded:
                        # Really empty - clear stale AFC state
                        self.logger.info(
                            f"State sync: No filament detected for {lane_name}, but AFC thinks it's loaded. "
                            f"Clearing stale AFC state."
                        )
                        try:
                            extruder_obj.lane_loaded = None
                            cleared_count += 1
                            # Save the cleared state
                            if hasattr(self.afc, 'save_vars'):
                                self.afc.save_vars()
                        except Exception as e:
                            self.logger.error(f"State sync: Failed to clear AFC state for {lane_name}: {e}")
                    else:
                        # Filament in hub but not tool - this is a mid-load state, don't change it
                        self.logger.debug(
                            f"State sync: {lane_name} has filament in hub but not tool. "
                            f"Leaving AFC state as-is (mid-load state)."
                        )
                        skipped_count += 1

                elif tool_loaded and afc_thinks_this_lane:
                    # Hardware and AFC agree - all good
                    self.logger.debug(f"State sync: {lane_name} state matches between hardware and AFC")

                elif not tool_loaded and not afc_thinks_this_lane:
                    # Both agree it's not loaded - all good
                    self.logger.debug(f"State sync: {lane_name} correctly shows as unloaded in both hardware and AFC")

            except Exception as e:
                self.logger.error(f"State sync: Failed to process lane {getattr(lane, 'name', 'unknown')}: {e}")
                skipped_count += 1

        # Summary log
        self.logger.info(
            f"State sync complete for {self.name}: "
            f"{synced_count} lanes synced to AFC, "
            f"{cleared_count} stale states cleared, "
            f"{skipped_count} skipped, "
            f"{conflict_count} conflicts requiring manual resolution"
        )

        if conflict_count > 0:
            self.logger.error(
                f"State sync: {conflict_count} conflicts detected. "
                f"Multiple lanes show loaded for the same extruder. "
                f"Use TOOL_UNLOAD and load the correct lane manually."
            )

    def handle_ready(self):
        """Resolve the OpenAMS object once Klippy is ready."""
        # Strip non-FPS buffers that AFC_lane._handle_ready() may have assigned.
        # AFC_FPS buffers are allowed (they provide ADC reading for proportional
        # control); only remove traditional TurtleNeck-style buffers.
        for lane in self.lanes.values():
            if lane.buffer_obj is not None and not hasattr(lane.buffer_obj, 'get_fps_value'):
                buffer_name = getattr(lane.buffer_obj, 'name', 'unknown')
                self.logger.warning(
                    f"Lane {lane.name} was assigned non-FPS buffer '{buffer_name}' during initialization, "
                    f"but OpenAMS units only support FPS buffers. Removing buffer assignment."
                )
                lane.buffer_obj = None

        # First check if ANY OpenAMS hardware exists in the system
        if not _has_openams_hardware(self.printer):
            self.logger.info(
                "No OpenAMS hardware found in configuration. "
                f"Skipping OpenAMS integration for AFC unit '{self.name}'. "
                "This is normal if you are using Box Turtle or other non-AMS hardware."
            )
            # Skip all OpenAMS initialization if no hardware is present
            return

        if self.hardware_service is not None:
            self.oams = self.hardware_service.resolve_controller()
        else:
            self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
            if AMSHardwareService is not None and self.oams is not None:
                try:
                    self.hardware_service = AMSHardwareService.for_printer(self.printer, self.oams_name, self.logger)
                    self.hardware_service.attach_controller(self.oams)
                except Exception as e:
                    self.logger.error(f"Failed to attach AMSHardwareService for {self.oams_name}: {e}")
        # Check if OAMS hardware was found for THIS specific unit
        if self.oams is None:
            self.logger.warning(
                "OpenAMS hardware '[oams "
                f"{self.oams_name}]' not found for AFC unit '{self.name}'. "
                "If you are using Box Turtle or other non-AMS hardware, "
                f"remove the '[afc_openams {self.name}]' section from your config."
            )
            # Don't start polling if no OAMS hardware
            return

        # Subscribe to hardware sensor events (requires AMSHardwareService)
        if self.hardware_service is None:
            self.logger.error(
                f"AMSHardwareService not available for {self.name}. "
                "Sensor updates will not work!"
            )
            return

        # Subscribe to sensor change events
        self.event_bus.subscribe("f1s_changed", self._on_f1s_changed, priority=5)
        self.event_bus.subscribe("hub_changed", self._on_hub_changed, priority=5)

        # Start unified polling in AMSHardwareService
        try:
            self.hardware_service.start_polling()
        except Exception as e:
            self.logger.error(f"Failed to start unified polling for {self.oams_name}: {e}")
        # Initialize owned subsystems now that hardware is resolved
        self._init_follower_and_monitor()

        # Sync AFC state with hardware sensors at startup
        # This should run after all initialization is complete and sensors are stable
        # Delay slightly to ensure sensors have had time to stabilize
        self.reactor.register_callback(lambda et: self._sync_afc_from_hardware_at_startup())

        # Some OpenAMS controllers report late initial sensor updates right after boot.
        # Run a second delayed reconciliation so PREP/load checks use true hub/F1S state.
        self.reactor.register_callback(
            lambda et: self.sync_openams_sensors(
                et,
                sync_hub=True,
                sync_f1s=True,
                allow_lane_clear=True,
            ),
            self.reactor.monotonic() + 1.0,
        )

    def _sync_lane_virtual_f1s_sensors(self, lane, eventtime, state) -> None:
        """Mirror F1S state into lane virtual prep/load filament-switch helpers."""
        state_val = bool(state)
        for attr in ("fila_load", "fila_prep"):
            sensor = getattr(lane, attr, None)
            if sensor is None:
                continue
            helper = getattr(sensor, "runout_helper", None)
            if helper is None:
                continue
            try:
                helper.note_filament_present(eventtime, state_val)
            except TypeError:
                try:
                    helper.note_filament_present(is_filament_present=state_val)
                except Exception:
                    pass
            except Exception:
                pass

    def _on_f1s_changed(self, event_type, unit_name, bay, value, eventtime, **kwargs):
        """Handle F1S sensor change events from AMSHardwareService.

        Event-driven architecture: sensor updates published by AMSHardwareService
        when hardware detects state changes, eliminating need for polling.
        """
        if unit_name != self.oams_name:
            return  # Not our unit

        lane = self._lane_for_spool_index(bay)
        if lane is None:
            return

        lane_val = bool(value)
        prev_val = getattr(lane, "load_state", False)
        self._sync_lane_virtual_f1s_sensors(lane, eventtime, lane_val)
        self.logger.debug(f"_on_f1s_changed: lane={lane.name} value={lane_val} prev={prev_val} fps_share_prep_load={getattr(lane, 'fps_share_prep_load', False)}")

        # Update lane state based on sensor FIRST
        if getattr(lane, "fps_share_prep_load", False):
            self._update_shared_lane(lane, lane_val, eventtime)
        elif lane_val != prev_val:
            lane.load_callback(eventtime, lane_val)
            lane.prep_callback(eventtime, lane_val)

            # Publish spool_loaded/spool_unloaded event for non-shared lanes
            # Pass previous_loaded state since lane.load_state is already updated by callbacks above
            if self.event_bus is not None:
                try:
                    spool_index = self._get_openams_spool_index(lane)
                    event_type_name = "spool_loaded" if lane_val else "spool_unloaded"
                    self.event_bus.publish(event_type_name,
                        unit_name=self.name,
                        spool_index=spool_index,
                        eventtime=eventtime,
                        previous_loaded=prev_val,
                    )
                except Exception as e:
                    self.logger.error(f"Failed to publish {event_type_name} event for {lane.name}: {e}")

        # Detect F1S sensor going False (spool empty) - trigger runout detection AFTER sensor update
        # Only trigger if printer is actively printing (not during filament insertion/removal)
        if prev_val and not lane_val:
            try:
                is_printing = self.afc.function.is_printing()
            except Exception:
                is_printing = False

            if is_printing:
                # Only trigger runout detection if THIS lane is the one loaded to its extruder
                # Skip runout detection on inactive lanes (e.g., when changing spools on different lane)
                # In multi-extruder setups, check the specific extruder this lane is associated with
                skip_runout = False
                try:
                    extruder_obj = getattr(lane, 'extruder_obj', None)
                    if extruder_obj is not None:
                        lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                        if lane_loaded is not None and lane_loaded != lane.name:
                            # This lane is NOT the one loaded to its extruder - skip runout detection
                            self.logger.debug(
                                f"F1S sensor False for {lane.name} but lane {lane_loaded} is loaded to "
                                f"{getattr(extruder_obj, 'name', 'extruder')} - skipping runout detection on inactive lane"
                            )
                            skip_runout = True
                except Exception:
                    pass

                if not skip_runout:
                    self.logger.info("F1S sensor False for {} (spool empty, printing), triggering runout detection".format(lane.name))
                    try:
                        self.handle_runout_detected(bay, None, lane_name=lane.name)
                    except Exception as e:
                        self.logger.error(
                            f"Failed to handle runout detection for {lane.name} "
                            f"(spool_index={bay}, runout_lane={getattr(lane, 'runout_lane', None)}): {e}",
                            traceback=traceback.format_exc(),
                        )
            else:
                self.logger.debug("F1S sensor False for {} but not printing - skipping runout detection (likely filament insertion/removal)".format(lane.name))

        # Update hardware service snapshot
        if self.hardware_service is not None:
            hub_obj = getattr(lane, "hub_obj", None)
            hub_state = getattr(lane, "loaded_to_hub", False) if hub_obj else None
            tool_state = self._lane_reports_tool_filament(lane)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name, lane.name, lane_val, hub_state, eventtime,
                    spool_index=bay, tool_state=tool_state
                )
            except Exception as e:
                self.logger.error(f"Failed to update lane snapshot for {lane.name}: {e}")
        # Sync virtual tool sensor

    def _on_hub_changed(self, event_type, unit_name, bay, value, eventtime, **kwargs):
        """Handle hub sensor change events from AMSHardwareService.

        Event-driven architecture: sensor updates published by AMSHardwareService
        when hardware detects state changes, eliminating need for polling.
        """
        if unit_name != self.oams_name:
            return  # Not our unit

        lane = self._lane_for_spool_index(bay)
        if lane is None:
            return

        hub = getattr(lane, "hub_obj", None)
        hub_val = bool(value)
        if hub is None:
            lane.loaded_to_hub = hub_val
            if self.hardware_service is not None:
                lane_state = getattr(lane, "load_state", False)
                tool_state = self._lane_reports_tool_filament(lane)
                try:
                    self.hardware_service.update_lane_snapshot(
                        self.oams_name, lane.name, lane_state, hub_val, eventtime,
                        spool_index=bay, tool_state=tool_state
                    )
                except Exception as e:
                    self.logger.error(f"Failed to update lane snapshot for {lane.name}: {e}")
            return

        if hub_val != getattr(lane, "loaded_to_hub", False):
            hub.switch_pin_callback(eventtime, hub_val)
            # Update lane.loaded_to_hub to match hub sensor state
            # This field is reported to Mainsail via lane.get_status()
            # Without this, Mainsail shows stale hub status even when hardware sensor is correct
            lane.loaded_to_hub = hub_val
            fila = getattr(hub, "fila", None)
            if fila is not None:
                fila.runout_helper.note_filament_present(eventtime, hub_val)

        # Update hardware service snapshot
        if self.hardware_service is not None:
            lane_state = getattr(lane, "load_state", False)
            tool_state = self._lane_reports_tool_filament(lane)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name, lane.name, lane_state, hub_val, eventtime,
                    spool_index=bay, tool_state=tool_state
                )
            except Exception as e:
                self.logger.error(f"Failed to update lane snapshot for {lane.name}: {e}")

    def sync_openams_sensors(self, eventtime, *, sync_hub=True, sync_f1s=False, allow_lane_clear=True):
        """Periodic level-based sync of OAMS hardware sensors to AFC lane state.

        Called periodically as a level-based sync complement to edge-triggered callbacks.
        Unlike _on_hub_changed/_on_f1s_changed (which are edge-triggered and only fire
        on sensor value *changes*), this method re-reads the actual hardware values and
        corrects any drift -- e.g., when set_unloaded() clears loaded_to_hub but the hub
        sensor is still True.
        """
        if self.oams is None:
            return

        hub_values = getattr(self.oams, "hub_hes_value", None)
        f1s_values = getattr(self.oams, "f1s_hes_value", None)

        for lane in self.lanes.values():
            try:
                spool_idx = self._get_openams_spool_index(lane)
                if spool_idx is None or spool_idx < 0:
                    continue

                # Sync hub sensor -> loaded_to_hub + virtual hub sensor objects.
                # Must mirror what _on_hub_changed does: update the lane attribute AND
                # drive switch_pin_callback + runout_helper so the virtual hub sensor
                # AFC reads for hub-runout detection stays in sync with hardware.
                if sync_hub and hub_values is not None and spool_idx < len(hub_values):
                    hw_hub = bool(hub_values[spool_idx])
                    current = getattr(lane, "loaded_to_hub", False)
                    if hw_hub != current:
                        lane.loaded_to_hub = hw_hub
                        hub_obj = getattr(lane, "hub_obj", None)
                        if hub_obj is not None:
                            try:
                                if hasattr(hub_obj, "switch_pin_callback"):
                                    hub_obj.switch_pin_callback(eventtime, hw_hub)
                                fila = getattr(hub_obj, "fila", None)
                                if fila is not None and hasattr(fila, "runout_helper"):
                                    fila.runout_helper.note_filament_present(eventtime, hw_hub)
                            except Exception as hub_e:
                                self.logger.debug(
                                    f"sync_openams_sensors: failed to update virtual hub sensor "
                                    f"for {lane.name}: {hub_e}"
                                )
                        self.logger.debug(
                            f"sync_openams_sensors: corrected loaded_to_hub "
                            f"{current}->{hw_hub} for {lane.name}"
                        )

                # Sync F1S sensor -> load_state/prep_state (only when allowed)
                if sync_f1s and f1s_values is not None and spool_idx < len(f1s_values):
                    hw_f1s = bool(f1s_values[spool_idx])
                    self._sync_lane_virtual_f1s_sensors(lane, eventtime, hw_f1s)
                    current_load = getattr(lane, "load_state", False)
                    if hw_f1s != current_load:
                        if hw_f1s or allow_lane_clear:
                            share = getattr(lane, "fps_share_prep_load", False)
                            if share:
                                self._update_shared_lane(
                                    lane, hw_f1s, eventtime,
                                    allow_clear=allow_lane_clear,
                                )
                            else:
                                lane.load_callback(eventtime, hw_f1s)
                                lane.prep_callback(eventtime, hw_f1s)
            except Exception as e:
                self.logger.debug(f"sync_openams_sensors: error syncing {getattr(lane, 'name', '?')}: {e}")

    def _should_block_sensor_update_for_runout(self, lane, lane_val):
        """Check if sensor update should be blocked due to active runout.

        Returns True if the update should be blocked, False otherwise.
        Automatically clears the runout flag if runout handling is complete.
        """
        # Cross-extruder runouts should not block shared sensor transitions; let AFC handle
        # them like Box Turtle units. Only same-extruder runouts set _oams_runout_detected.
        if lane._oams_cross_extruder_runout:
            # Make sure any stale blocking flag is cleared for cross-extruder scenarios
            lane._oams_runout_detected = False
            return False

        if not lane._oams_runout_detected:
            return False

        should_block = False
        try:
            is_printing = self.afc.function.is_printing()
            is_tool_loaded = getattr(lane, 'tool_loaded', False)
            lane_status = getattr(lane, 'status', None)
            # Only block if actively printing with this lane loaded and in runout state
            if is_printing and is_tool_loaded and lane_status in (AFCLaneState.INFINITE_RUNOUT, AFCLaneState.TOOL_UNLOADING):
                should_block = True
            else:
                # Clear the flag - runout handling is complete
                lane._oams_runout_detected = False
                self.logger.debug(f"Clearing runout flag for lane {getattr(lane, 'name', 'unknown')} - runout handling complete")
        except Exception:
            # On error, clear the flag to be safe
            lane._oams_runout_detected = False

        # Block only if conditions met and trying to set sensors to True
        if should_block and lane_val:
            return True
        # Sensor confirms empty - always clear flag
        elif not lane_val:
            lane._oams_runout_detected = False
            self.logger.debug(f"Sensor confirmed empty state for lane {getattr(lane, 'name', 'unknown')} - clearing runout flag")
        return False

    def _trigger_td1_capture_delayed(self, lane_name):
        """
        Timer callback to trigger TD-1 capture after 4.2-second delay.
        This allows the AMS auto-load sequence to complete (pushes to hub sensor -> retracts -> settles).

        CRITICAL: This is called from timer context - must not use reactor.pause() or wait=True.
        """
        def _timer_callback(eventtime):
            try:
                # Clean up pending timer tracking
                if lane_name in self._pending_spool_loaded_timers:
                    del self._pending_spool_loaded_timers[lane_name]

                # Find the lane and trigger TD-1 capture
                lane = self.afc.lanes.get(lane_name)
                if lane is not None:
                    try:
                        self.logger.info(f"Triggering delayed TD-1 capture for {lane_name} (AMS settled)")
                        lane._prep_capture_td1()
                    except Exception as e:
                        self.logger.error(f"Failed to trigger TD-1 capture for {lane_name}: {e}")
                else:
                    self.logger.error(f"Lane {lane_name} not found for delayed TD-1 capture")
            except Exception as e:
                self.logger.error(f"Error in _trigger_td1_capture_delayed timer callback for {lane_name}: {e}")

            # Return NEVER to stop the timer from repeating
            return self.reactor.NEVER

        return _timer_callback

    def _should_force_full_unload_for_shared_lane(self, lane, lane_val_bool: bool, eventtime: float, *, allow_clear: bool = True) -> bool:
        """Gate destructive shared-lane unloads to confirmed sensor/removal conditions."""
        if not allow_clear or lane_val_bool:
            return False

        if bool(
            getattr(lane, "_oams_same_fps_runout", False)
            or getattr(lane, "_oams_runout_detected", False)
            or getattr(lane, "_oams_cross_extruder_runout", False)
        ):
            return True

        if bool(getattr(lane, "tool_loaded", False)):
            return False

        hub_state = getattr(lane, "loaded_to_hub", False)
        if self.hardware_service is not None:
            try:
                snapshot = self.hardware_service.latest_lane_snapshot(self.oams_name, lane.name)
                if snapshot is not None:
                    hub_state = snapshot.get("hub_state", hub_state)
            except Exception:
                pass

        return not bool(hub_state)

    def _update_shared_lane(self, lane, lane_val, eventtime, *, allow_clear: bool = True):
        """Synchronise shared prep/load sensor lanes without triggering errors."""
        # Check if runout handling requires blocking this sensor update
        if self._should_block_sensor_update_for_runout(lane, lane_val):
            self.logger.debug(f"_update_shared_lane: blocked by runout handling for {lane.name}")
            return

        previous = getattr(lane, "load_state", False)
        lane_val_bool = bool(lane_val)
        prep_state = getattr(lane, "prep_state", None)
        load_state = getattr(lane, "load_state", None)

        self.logger.debug(f"_update_shared_lane: lane={lane.name} lane_val={lane_val_bool} previous={previous} prep_state={prep_state} load_state={load_state}")

        if (
            previous is not None
            and bool(previous) == lane_val_bool
            and (prep_state is None or bool(prep_state) == lane_val_bool)
            and (load_state is None or bool(load_state) == lane_val_bool)
        ):
            # prep/load is shared for OpenAMS lanes, but hub sensing is per-lane.
            # Even when prep/load is unchanged, reconcile this lane's own hub state.
            hub_state = None
            lane_index = None
            try:
                lane_index = int(getattr(lane, "index", 0)) - 1
            except Exception:
                lane_index = None
            if self.oams is not None and lane_index is not None and lane_index >= 0:
                try:
                    hub_values = getattr(self.oams, "hub_hes_value", None)
                    if hub_values is not None and lane_index < len(hub_values):
                        hub_state = bool(hub_values[lane_index])
                except Exception:
                    hub_state = None
            if hub_state is None and self.hardware_service is not None:
                try:
                    snapshot = self.hardware_service.latest_lane_snapshot(self.oams_name, lane.name)
                except Exception:
                    snapshot = None
                if isinstance(snapshot, dict) and "hub_state" in snapshot:
                    snap_hub_state = snapshot.get("hub_state")
                    if snap_hub_state is not None:
                        hub_state = bool(snap_hub_state)
            if hub_state is not None and bool(getattr(lane, "loaded_to_hub", False)) != hub_state:
                try:
                    lane.loaded_to_hub = hub_state
                except Exception:
                    pass
                try:
                    hub_obj = getattr(lane, "hub_obj", None)
                    if hub_obj is not None:
                        if hasattr(hub_obj, "switch_pin_callback"):
                            hub_obj.switch_pin_callback(eventtime, hub_state)
                        fila = getattr(hub_obj, "fila", None)
                        if fila is not None and hasattr(fila, "runout_helper"):
                            fila.runout_helper.note_filament_present(eventtime, hub_state)
                except Exception as e:
                    self.logger.debug(
                        f"_update_shared_lane: failed to reconcile per-lane hub state for {lane.name}: {e}"
                    )
            self.logger.debug(f"_update_shared_lane: early return - state unchanged for {lane.name}")
            return

        if lane_val_bool:
            # Defer metadata application (material, spoolman IDs, colors, etc.) to
            # AFC's callbacks. The callbacks will update prep/load state and apply lane data consistently for both
            # single- and shared-sensor lanes.
            try:
                lane.prep_callback(eventtime, True)
            finally:
                lane.load_callback(eventtime, True)


            # Publish spool_loaded event immediately (TD-1 capture delay happens in event handler)
            # Pass previous_loaded state since lane.load_state is already updated by callbacks above
            if self.event_bus is not None:
                try:
                    spool_index = self._get_openams_spool_index(lane)
                    self.event_bus.publish("spool_loaded",
                        unit_name=self.name,
                        spool_index=spool_index,
                        eventtime=eventtime,
                        previous_loaded=previous,
                    )
                except Exception as e:
                    self.logger.error(f"Failed to publish spool_loaded event for {lane.name}: {e}")
        else:
            # Sensor False - filament left spool bay
            # Update sensor state but don't aggressively clear everything (align with Box Turtle behavior)
            lane.load_callback(eventtime, False)
            lane.prep_callback(eventtime, False)


            # Cancel any pending TD-1 capture timer since filament was removed
            lane_name = lane.name
            if lane_name in self._pending_spool_loaded_timers:
                try:
                    timer = self._pending_spool_loaded_timers[lane_name]
                    self.reactor.unregister_timer(timer)
                    del self._pending_spool_loaded_timers[lane_name]
                    self.logger.debug(f"Cancelled pending TD-1 capture timer for {lane_name} (filament removed)")
                except Exception as e:
                    self.logger.error(f"Error cancelling TD-1 capture timer for {lane_name}: {e}")

            # Shared prep/load sensors stay in sync for AMS lanes; treat False as fully unloaded
            try:
                # PHASE 1 REFACTOR: Remove redundant manual state assignments
                # lane.set_unloaded() already handles tool_loaded and loaded_to_hub
                # Only call set_unloaded if lane isn't already in NONE state
                # This prevents errors when sensor reports empty for an already-empty lane
                force_full_unload = self._should_force_full_unload_for_shared_lane(
                    lane,
                    lane_val_bool,
                    eventtime,
                    allow_clear=allow_clear and EVENT_POLICY[OpenAMSStateMutation.SENSOR.value]["allow_full_unload"],
                )
                if force_full_unload and lane.status != AFCLaneState.NONE:
                    lane.set_unloaded()
                    if hasattr(lane, "_afc_prep_done"):
                        lane._afc_prep_done = False
                lane.prep_state = lane_val_bool
                lane._load_state = lane_val_bool
                lane.loaded_to_hub = lane_val_bool
            except Exception as e:
                # This is often benign (lane already cleared), log at debug level
                self.logger.debug(f"Could not fully clear shared lane {lane.name} after sensor cleared (lane may already be empty): {e}")
            # For same-FPS runouts: blocking mechanism in _should_block_sensor_update_for_runout()
            # prevents us from reaching here during active runout
            # For cross-extruder runouts: AFC's LANE_UNLOAD wrapper handles cleanup
            # For manual unloads: AFC's LANE_UNLOAD command handles cleanup

            # Only unsync from extruder if not in active cross-extruder runout or tool operation
            try:
                is_printing = self.afc.function.is_printing()
            except Exception:
                is_printing = False
            is_cross_extruder_runout = lane._oams_cross_extruder_runout and is_printing

            # Check if this is a shared extruder (multiple lanes on same extruder)
            # For shared extruders, NEVER clear lane_loaded from sensor callback
            # EXCEPT during runouts when filament actually left the toolhead
            # Sensor can go False during tool parking/swapping without filament leaving,
            # but during runouts the sensor goes False because filament ran out and must be cleared
            is_shared_extruder = False
            is_same_fps_runout = False
            try:
                if hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None:
                    # no_lanes=True means standalone toolhead, no_lanes=False means shared extruder
                    is_shared_extruder = not getattr(lane.extruder_obj, 'no_lanes', True)

                    # Check if we're in a same-FPS runout (sensor went False during print, filament actually ran out)
                    # This is different from tool changes where sensor might briefly go False
                    if is_shared_extruder and is_printing:
                        # During runout, the runout monitor sets a flag to indicate filament actually ran out
                        # We should clear lane_loaded in this case even for shared extruders
                        is_same_fps_runout = getattr(lane, '_oams_same_fps_runout', False)
            except Exception:
                pass

            if not is_cross_extruder_runout and not (is_shared_extruder and not is_same_fps_runout):
                try:
                    if hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None:
                        if lane.extruder_obj.lane_loaded == lane.name:
                            lane.unsync_to_extruder()
                            lane.extruder_obj.lane_loaded = None
                            self.logger.debug(f"Unsynced lane {lane.name} and cleared extruder.lane_loaded when sensor went False")
                except Exception as e:
                    self.logger.error(
                        f"Failed to unsync lane {lane.name} from extruder when sensor cleared: {e}",
                        traceback=traceback.format_exc(),
                    )
            elif is_shared_extruder and not is_same_fps_runout:
                self.logger.debug(f"Skipping lane_loaded clear for {lane.name} - shared extruder (only UNLOAD/RUNOUT commands should clear)")
            else:
                self.logger.debug(f"Skipping extruder unsync for {lane.name} - cross-extruder runout (AFC will handle via LANE_UNLOAD)")
        lane.prep_state = lane_val_bool
        lane._load_state = lane_val_bool
        lane.loaded_to_hub = lane_val_bool
        lane.afc.save_vars()

    def _apply_lane_sensor_state(self, lane, lane_val, eventtime):
        """Apply a boolean lane sensor value using existing AFC callbacks."""
        # Check if runout handling requires blocking this sensor update
        if self._should_block_sensor_update_for_runout(lane, lane_val):
            self.logger.debug(f"Ignoring sensor update for lane {getattr(lane, 'name', 'unknown')} - runout in progress")
            return

        try:
            share = getattr(lane, "fps_share_prep_load", False)
        except Exception:
            share = False

        if share:
            self._update_shared_lane(lane, lane_val, eventtime)
            return

        previous = getattr(lane, "load_state", False)

        if previous is not None and bool(previous) == bool(lane_val):
            return

        try:
            lane.load_callback(eventtime, lane_val)
        except TypeError:
            lane.load_callback(eventtime, load_state=lane_val)
        except Exception as e:
            self.logger.error(f"Failed to update load sensor for lane {lane}: {e}")
        try:
            lane.prep_callback(eventtime, lane_val)
        except TypeError:
            lane.prep_callback(eventtime, prep_state=lane_val)
        except Exception as e:
            self.logger.error(f"Failed to update prep sensor for lane {lane}: {e}")
        # When sensor goes False (empty), only clear tool/hub loaded flags
        # Let AFC's normal flow handle status and cleanup (align with Box Turtle)
        if not lane_val:
            lane.tool_loaded = False
            lane.loaded_to_hub = False


    def _lane_for_spool_index(self, spool_index: Optional[int]):
        """Use indexed lookup instead of iteration."""
        if spool_index is None:
            return None

        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None

        registry_unit = self.oams_name or self.name
        if self.registry is not None:
            lane_info = self.registry.get_by_spool(registry_unit, normalized)
            if lane_info is not None:
                lane = self.lanes.get(lane_info.lane_name)
                if lane is not None:
                    return lane

        if normalized < 0 or normalized >= 4:
            return None

        return self._lane_by_local_index(normalized)

    def _resolve_lane_reference(self, lane_name: Optional[str]):
        """Return a lane object by name (or alias), case-insensitively."""
        lane, _trace = self._resolve_lane_reference_with_trace(lane_name)
        return lane

    def _resolve_lane_reference_with_trace(self, lane_name: Optional[str]):
        """Return a lane object and the resolution path for debugging."""
        trace: List[str] = []
        if not lane_name:
            trace.append("no name provided")
            return None, trace

        def _normalize_tool_token(token: Optional[str]) -> Optional[str]:
            if not isinstance(token, str):
                return None
            cleaned = token.strip().lower()
            if cleaned.startswith("t"):
                cleaned = cleaned[1:]
            return cleaned or None

        resolved_name = self._resolve_lane_alias(lane_name)
        registry = self.registry

        def _registry_lookup_by_name(name: str):
            if registry is None:
                return None
            info = registry.get_by_lane(name)
            if info is None:
                return None
            lane_obj = self._get_lane_object(info.lane_name)
            if lane_obj is not None:
                trace.append(
                    f"registry lane {info.lane_name} (unit {info.unit_name}, extruder {info.extruder})"
                )
                return lane_obj
            trace.append(f"registry lane {info.lane_name} unresolved in printer objects")
            return None

        if resolved_name:
            trace.append(f"alias -> {resolved_name}")
            lane = self.lanes.get(resolved_name)
            if lane is not None:
                trace.append(f"found lane {lane.name} via alias")
                return lane, trace

            lane = _registry_lookup_by_name(resolved_name)
            if lane is not None:
                return lane, trace
        else:
            resolved_name = lane_name
            trace.append(f"no alias, using raw '{resolved_name}'")

        lane = self.lanes.get(resolved_name)
        if lane is not None:
            trace.append(f"matched direct lane {lane.name}")
            return lane, trace

        lowered = resolved_name.lower()
        for candidate_name, candidate in self.lanes.items():
            if candidate_name.lower() == lowered:
                trace.append(f"matched case-insensitive lane {candidate_name}")
                return candidate, trace

        registry_lane = _registry_lookup_by_name(resolved_name)
        if registry_lane is not None:
            return registry_lane, trace

        normalized_tool = _normalize_tool_token(resolved_name)
        trace.append(f"normalized tool token '{normalized_tool}'" if normalized_tool else "no tool token available")
        if normalized_tool:
            for candidate in self.lanes.values():
                candidate_tool = _normalize_tool_token(getattr(candidate, "map", None))
                if candidate_tool and candidate_tool == normalized_tool:
                    trace.append(
                        f"matched tool mapping '{getattr(candidate, 'map', None)}' -> lane {candidate.name}"
                    )
                    return candidate, trace

            if registry is not None:
                token = resolved_name if resolved_name.lower().startswith("t") else f"T{normalized_tool}"
                info = registry.resolve_lane_token(token)
                if info is not None:
                    lane = self._get_lane_object(info.lane_name)
                    if lane is not None:
                        trace.append(
                            f"registry token '{token}' -> lane {info.lane_name} (unit {info.unit_name})"
                        )
                        return lane, trace
                    trace.append(
                        f"registry token '{token}' found lane {info.lane_name} but object missing"
                    )

        trace.append("no lane resolved")
        return None, trace

    def _get_snapshot_lane_extruder(self, lane_name: str) -> Optional[str]:
        unit_name = None
        lane_obj = self._get_lane_object(lane_name)
        if lane_obj is not None:
            unit_name = getattr(lane_obj, "unit", None)

        lane_data = self._find_lane_snapshot(lane_name, unit_name=unit_name)
        if isinstance(lane_data, dict):
            return lane_data.get("extruder")
        return None

    def handle_runout_detected(self, spool_index: Optional[int], monitor=None, *, lane_name: Optional[str] = None) -> None:
        """Handle runout notifications coming from OpenAMS monitors."""
        lane = None
        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is None:
                lowered = lane_name.lower()
                lane = next((candidate for name, candidate in self.lanes.items() if name.lower() == lowered), None)
        if lane is None:
            lane = self._lane_for_spool_index(spool_index)
        if lane is None:
            return

        eventtime = self.reactor.monotonic()

        # Only handle the AMS same-extruder case here. Any other runout should fall back to
        # AFC's normal handling without OpenAMS interjection.
        runout_lane_name = self._canonical_lane_name(getattr(lane, "runout_lane", None))
        saved_runout_lane = self._get_saved_lane_runout_target(lane.name)
        runout_from_saved = False

        if not runout_lane_name and saved_runout_lane:
            runout_lane_name = saved_runout_lane
            runout_from_saved = True
            try:
                lane.runout_lane = runout_lane_name
            except Exception:
                self.logger.debug(f"Unable to write saved runout lane {runout_lane_name} onto {lane.name}")
        target_lane, handoff_trace = self._resolve_lane_reference_with_trace(runout_lane_name) if runout_lane_name else (None, [])
        same_extruder_handoff = False

        source_extruder = normalize_extruder_name(self._get_snapshot_lane_extruder(lane.name))
        target_extruder = None
        if target_lane:
            target_extruder = normalize_extruder_name(self._get_snapshot_lane_extruder(target_lane.name))

        if not source_extruder or (target_lane and not target_extruder):
            self.logger.error(
                f"Runout classification failed for {lane.name}: AFC.var.unit missing extruder data "
                f"(source={source_extruder}, target={target_extruder}); pausing for user intervention"
            )
            try:
                self.gcode.run_script_from_command("PAUSE")
            except Exception as e:
                self.logger.error(f"Failed to issue PAUSE after runout classification failure: {e}")
            self.logger.info(
                f"Runout classification failed for {lane.name}: AFC.var.unit missing extruder data. "
                "Please check AFC.var.unit and lane mappings."
            )
            return

        self.logger.debug(
            f"Runout classification for {lane.name}: spool_index={spool_index}, "
            f"runout_lane={runout_lane_name} -> "
            f"{getattr(target_lane, 'name', None) if target_lane else None}, "
            f"source_extruder={source_extruder}, target_extruder={target_extruder}"
        )
        if runout_lane_name:
            self.logger.debug(
                f"Runout handoff resolution trace for {lane.name}: "
                f"{' > '.join(handoff_trace) if handoff_trace else '(no trace)'}"
            )

        if runout_lane_name and not target_lane and saved_runout_lane and saved_runout_lane != runout_lane_name:
            self.logger.info(
                f"Runout lane {runout_lane_name} for {lane.name} could not be resolved; "
                f"falling back to saved AFC state {saved_runout_lane}"
            )
            runout_lane_name = saved_runout_lane
            runout_from_saved = True
            target_lane, handoff_trace = self._resolve_lane_reference_with_trace(runout_lane_name)
            if target_lane:
                try:
                    lane.runout_lane = runout_lane_name
                except Exception:
                    self.logger.debug(f"Unable to persist saved runout lane {runout_lane_name} on {lane.name}")
        if runout_from_saved:
            self.logger.info(f"Resolved runout lane for {lane.name} from saved AFC.var.unit state: {runout_lane_name}")
        if target_lane:
            if source_extruder and target_extruder:
                if source_extruder == target_extruder:
                    same_extruder_handoff = True

        if not same_extruder_handoff:
            # Only block shared sensor transitions for same-FPS handoffs. For any other runout, keep
            # sensors flowing so AFC can perform its own infinite-runout logic just like Box Turtle
            # units.
            try:
                lane._oams_runout_detected = False
            except Exception:
                pass

            if runout_lane_name and not target_lane:
                self.logger.warning(
                    f"Runout handoff lane {runout_lane_name} not found for {lane.name}; "
                    "deferring to AFC's native runout handling"
                )
            elif target_lane:
                # Box Turtle infinite-runout behavior: when a handoff lane is configured on a
                # different extruder, trigger the infinite spool swap directly and let AFC resume
                # afterward instead of treating it as a same-FPS handoff.
                try:
                    lane._oams_cross_extruder_runout = True
                except Exception:
                    pass

                resolved_name = getattr(target_lane, "name", None)
                if resolved_name and resolved_name != getattr(lane, "runout_lane", None):
                    try:
                        lane.runout_lane = resolved_name
                    except Exception:
                        self.logger.debug(f"Could not set resolved runout lane on {lane.name}")
                perform_infinite = getattr(lane, "_perform_infinite_runout", None)
                if callable(perform_infinite):
                    try:
                        current_lane = getattr(self.afc, "current", None)
                        if current_lane != lane.name:
                            if current_lane not in getattr(self.afc, "lanes", {}):
                                self.logger.info(
                                    f"Cross-extruder runout: AFC current lane {current_lane} invalid; "
                                    f"overriding to {lane.name}"
                                )
                            try:
                                self.afc.current = lane.name
                            except Exception:
                                self.logger.debug(f"Unable to set AFC current lane to {lane.name} before infinite runout")
                        self.logger.info(
                            f"Cross-extruder runout: invoking infinite spool handoff from {lane.name} to {resolved_name}"
                        )
                        perform_infinite()
                        return
                    except Exception as e:
                        self.logger.error(
                            f"Failed infinite spool handoff for {lane.name} -> {resolved_name}; "
                            f"falling back to AFC runout handler: {e}",
                            traceback=traceback.format_exc(),
                        )
            else:
                try:
                    lane._oams_cross_extruder_runout = False
                except Exception:
                    pass

                self.logger.info(
                    f"Runout for {lane.name} does not target same extruder; letting AFC handle it normally"
                )

            # Allow AFC's built-in runout handler to pause or swap as configured
            extruder = getattr(lane, "extruder_obj", None)
            runout_cb = getattr(extruder, "handle_start_runout", None) if extruder is not None else None
            if callable(runout_cb):
                try:
                    runout_cb(eventtime)
                except TypeError:
                    runout_cb(eventtime=eventtime)
                except Exception as e:
                    self.logger.error(
                        f"Extruder runout handler failed for {lane.name} "
                        f"(spool_index={spool_index}, runout_lane={runout_lane_name}): {e}",
                        traceback=traceback.format_exc(),
                    )
            else:
                self.logger.warning(
                    f"Runout detected for {lane.name} but no extruder runout handler is available; "
                    "AFC must handle downstream"
                )
            return

        # Same-extruder runout: set the runout flag so shared sensor updates don't bounce the state.
        # The runout monitor will detect the F1S=False via its own polling
        # and handle the reload sequence.
        try:
            lane._oams_runout_detected = True
            lane._oams_cross_extruder_runout = False
            self.logger.info(
                f"Same-extruder runout: Marked lane {lane.name} for runout "
                "(monitor will handle reload)"
            )
        except Exception as e:
            self.logger.error(f"Failed to mark lane {lane.name} for runout tracking: {e}")

    # ------------------------------------------------------------------
    # Stuck spool auto-recovery (post-engagement, during printing)
    # ------------------------------------------------------------------

    def _on_stuck_spool_recovery_needed(self, fps_name, lane_name):
        """Trigger stuck spool recovery (unload + reload + resume).

        Called from reactor timer context (non-blocking). Schedules the recovery
        gcode command which runs in the gcode thread where blocking is safe.
        """
        self.logger.info(
            f"Stuck spool recovery scheduled: fps={fps_name}, lane={lane_name}"
        )
        try:
            # fps_name is the AFC_FPS buffer name (e.g., "FPS_buffer1")
            self.gcode.run_script_from_command(
                f"_AFC_OAMS_STUCK_RECOVERY LANE={lane_name} FPS={fps_name or ''}"
            )
        except Exception as e:
            self.logger.error(
                f"Failed to schedule stuck spool recovery gcode for {lane_name}: {e}"
            )
            # Recovery command failed - pause the print so the spool issue is not ignored
            try:
                self.gcode.run_script_from_command("PAUSE")
            except Exception as pause_e:
                self.logger.error(f"Failed to pause print after recovery command failure: {pause_e}")

    def _cmd_stuck_spool_recovery(self, gcmd):
        """Internal gcode command: full unload + reload + resume for a stuck spool.

        Runs in the gcode thread so blocking operations (TOOL_UNLOAD, TOOL_LOAD) are safe.
        Triggered by OAMSMonitor when stuck spool is detected post-engagement during printing.
        """
        lane_name = gcmd.get('LANE', None)
        # FPS is the AFC_FPS buffer name (e.g., "FPS_buffer1")
        fps_name  = gcmd.get('FPS', None)

        # Use the global AFC lane registry so this works regardless of which
        # AFC_OpenAMS unit registered the gcode command first.
        lane = self.afc.lanes.get(lane_name) if lane_name else None
        if lane is None:
            self.logger.error(
                f"Stuck spool recovery: lane '{lane_name}' not found, falling back to pause"
            )
            self._stuck_spool_recovery_fallback(fps_name, lane_name, "lane not found")
            return

        self.logger.info(
            f"Stuck spool auto-recovery starting: unload then reload for {lane_name}"
        )

        # Save toolhead position so AFC_RESUME can restore it afterwards
        self.afc.save_pos()

        # Pause the print queue immediately so no more gcode feeds from sdcard
        pause_resume = self.printer.lookup_object("pause_resume", None)
        if pause_resume is not None and not self.afc.function.is_paused():
            pause_resume.send_pause_command()

        # Z-hop to clear the part
        try:
            pos = self.afc.gcode_move.last_position
            self.afc.move_z_pos(pos[2] + self.afc.z_hop, "stuck_spool_recovery_zhop")
        except Exception as e:
            self.logger.warning(f"Stuck spool recovery: Z-hop failed: {e}")

        try:
            self.logger.info(f"Stuck spool recovery: unloading {lane_name} (with cut)")
            unload_ok = self.afc.TOOL_UNLOAD(lane, set_start_time=True)
            if not unload_ok:
                raise RuntimeError(f"TOOL_UNLOAD returned False for {lane_name}")

            # Wait for the hub sensor to settle before reloading.
            # The AFC unload homing move retracts filament past the hub sensor,
            # which clears it before TOOL_UNLOAD returns True.  However the
            # filament can continue to move slightly after stopping (inertia /
            # bowden spring-back) and briefly re-trigger the hub sensor.
            # TOOL_LOAD checks hub.state at the very start; if it reads True it
            # immediately returns False with "Hub not clear".  Polling here until
            # the sensor actually reads clear (up to HUB_CLEAR_TIMEOUT seconds)
            # absorbs that transient without requiring user intervention.
            HUB_CLEAR_TIMEOUT  = 5.0   # seconds; far more than any real settle time
            HUB_POLL_INTERVAL  = 0.1   # seconds between polls
            hub = getattr(lane, 'hub_obj', None)
            if hub is not None and hub.state:
                reactor = self.printer.get_reactor()
                waited  = 0.0
                while hub.state and waited < HUB_CLEAR_TIMEOUT:
                    reactor.pause(reactor.monotonic() + HUB_POLL_INTERVAL)
                    waited += HUB_POLL_INTERVAL
                if hub.state:
                    self.logger.warning(
                        f"Stuck spool recovery: hub sensor still active {waited:.1f}s "
                        f"after unload of {lane_name} - proceeding with TOOL_LOAD anyway"
                    )
                else:
                    self.logger.info(
                        f"Stuck spool recovery: hub cleared after {waited:.1f}s settle "
                        f"post-unload for {lane_name}"
                    )

            self.logger.info(f"Stuck spool recovery: reloading {lane_name}")
            load_ok = self.afc.TOOL_LOAD(lane, set_start_time=True)
            if not load_ok:
                raise RuntimeError(f"TOOL_LOAD returned False for {lane_name}")

        except Exception as e:
            self.logger.error(f"Stuck spool auto-recovery FAILED for {lane_name}: {e}")
            self._stuck_spool_recovery_fallback(fps_name, lane_name, str(e))
            return

        # Recovery succeeded - clear OAMS error state then resume
        self.logger.info(
            f"Stuck spool auto-recovery SUCCEEDED for {lane_name}, resuming print"
        )
        self._stuck_spool_recovery_clear_oams_state(fps_name, lane_name)

        try:
            self.afc.error.reset_failure()

            # AFC_RESUME guards on is_paused() == True before it will restore
            # the toolhead position and restart the SD card.  During recovery
            # the pause flag can be cleared without the SD card being properly
            # restarted (e.g. if a Tx toolchange command in the still-running
            # print file fires AFC_RESUME mid-recovery before we reach here).
            # Force the flag back so AFC_RESUME takes the full resume path.
            if not self.afc.function.is_paused():
                self.logger.info(
                    "Stuck spool recovery: pause state was cleared during recovery; "
                    "restoring it so AFC_RESUME can properly restart the print"
                )
                pause_resume_obj = self.printer.lookup_object("pause_resume", None)
                if pause_resume_obj is not None:
                    pause_resume_obj.is_paused = True

            self.gcode.run_script_from_command("AFC_RESUME")
        except Exception as e:
            self.logger.error(
                f"Stuck spool recovery: AFC_RESUME failed after successful reload: {e}"
            )

    def _stuck_spool_recovery_fallback(self, fps_name, lane_name, reason):
        """Fall back to pausing when auto-recovery is not possible or fails."""
        msg = (
            f"Stuck spool auto-recovery failed for {lane_name or fps_name}: {reason}.\n"
            f"Please manually correct the issue, then run:\n"
            f"  SET_LANE_LOADED LANE={lane_name}\n"
            f"followed by AFC_OAMS_CLEAR_ERRORS"
        )
        self.logger.error(msg)
        try:
            self.afc.error.AFC_error(msg, pause=True)
        except Exception as e:
            self.logger.error(f"Failed to issue AFC error for stuck spool fallback: {e}")
            try:
                self.gcode.run_script_from_command("PAUSE")
            except Exception:
                pass

    def _stuck_spool_recovery_clear_oams_state(self, fps_name, lane_name):
        """Clear the stuck spool error state after a successful auto-recovery."""
        try:
            if not fps_name and lane_name:
                fps_name = self._get_fps_id_for_lane(lane_name) or self._get_fps_name()
            if not fps_name:
                return

            # Clear error LED via owned follower
            st = self._get_monitor_state()
            if st and st.current_spool_idx is not None and self._follower:
                self._follower.clear_error_led(
                    self.oams, self.oams_name, st.current_spool_idx,
                    "stuck spool auto-recovery succeeded")

            # Reset monitor stuck state
            if st:
                st.stuck_active = False
                st.stuck_start_time = None

            self.logger.debug(
                f"Cleared stuck spool state for {fps_name} after auto-recovery"
            )
        except Exception as e:
            self.logger.warning(
                f"Failed to clear stuck spool state for {fps_name}: {e}"
            )

    def handle_openams_lane_tool_state(self, lane_name: str, loaded: bool, *, spool_index: Optional[int] = None, eventtime: Optional[float] = None) -> bool:
        """Update lane/tool state in response to OpenAMS hardware events."""
        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(f"OpenAMS reported lane {lane_name} but AFC unit {self.name} cannot resolve it")
            return False

        if eventtime is None:
            try:
                eventtime = self.reactor.monotonic()
            except Exception:
                eventtime = 0.0

        lane_state = bool(loaded)
        if lane_state:
            try:
                self._apply_lane_sensor_state(lane, lane_state, eventtime)
            except Exception as e:
                self.logger.error(f"Failed to mirror OpenAMS lane sensor state for {lane.name}: {e}")
        if self.hardware_service is not None:
            hub_state = getattr(lane, "loaded_to_hub", None)
            tool_state = getattr(lane, "tool_loaded", None)
            sensor_lane_state = bool(getattr(lane, "load_state", False) or getattr(lane, "prep_state", False))
            mapped_spool = spool_index
            if mapped_spool is None:
                try:
                    mapped_spool = int(getattr(lane, "index", 0)) - 1
                except (TypeError, ValueError):
                    mapped_spool = None
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    sensor_lane_state,
                    hub_state if hub_state is not None else None,
                    eventtime,
                    spool_index=mapped_spool,
                    tool_state=tool_state if tool_state is not None else lane_state,
                    emit_spool_event=False,
                )
            except Exception as e:
                self.logger.error(f"Failed to update shared lane snapshot for {lane.name}: {e}")
        afc_function = getattr(self.afc, "function", None)

        def _clear_lane_virtual_hub_sensor(target_lane) -> None:
            """Clear cached hub state + virtual hub sensor for a lane."""
            if target_lane is None:
                return
            try:
                target_lane.loaded_to_hub = False
            except Exception:
                pass

            try:
                hub_obj = getattr(target_lane, "hub_obj", None)
            except Exception:
                hub_obj = None

            if hub_obj is None:
                return

            try:
                if hasattr(hub_obj, "switch_pin_callback"):
                    hub_obj.switch_pin_callback(eventtime, False)
            except Exception as e:
                self.logger.debug(
                    f"Failed to clear virtual hub switch state for {getattr(target_lane, 'name', '<unknown>')}: {e}"
                )

            try:
                fila = getattr(hub_obj, "fila", None)
                if fila is not None and hasattr(fila, "runout_helper"):
                    fila.runout_helper.note_filament_present(eventtime, False)
            except Exception as e:
                self.logger.debug(
                    f"Failed to clear virtual hub runout helper for {getattr(target_lane, 'name', '<unknown>')}: {e}"
                )

        def _lane_hardware_hub_has_filament(target_lane) -> bool:
            """Return True when hardware hub sensor still reports filament for lane."""
            if target_lane is None or self.oams is None:
                return False
            try:
                spool_idx = self._get_openams_spool_index(target_lane)
            except Exception:
                spool_idx = None

            if spool_idx is None or spool_idx < 0:
                return False

            hub_values = getattr(self.oams, "hub_hes_value", None)
            if not hub_values or spool_idx >= len(hub_values):
                return False

            return bool(hub_values[spool_idx])

        if lane_state:
            # Filament at toolhead must have passed through hub - ensure hub indicator is correct.
            # hub_changed events are edge-triggered and can miss transitions when loaded_to_hub
            # was cleared by software (e.g. set_unloaded during retry cleanup) but hardware
            # stayed True between polls, so no event fires to re-set it.
            lane.loaded_to_hub = True
            if afc_function is not None:
                previous_lane = None
                try:
                    previous_lane = afc_function.get_current_lane_obj()
                except Exception:
                    previous_lane = None

                previous_lane_name = getattr(previous_lane, "name", None) if previous_lane is not None else None
                current_lane_name = getattr(lane, "name", None)
                if previous_lane_name and previous_lane_name != current_lane_name:
                    try:
                        afc_function.unset_lane_loaded()
                    except Exception as e:
                        self.logger.error(f"Failed to unset previously loaded lane {previous_lane_name}: {e}")

                    clear_previous_hub = bool(
                        getattr(previous_lane, "_oams_same_fps_runout", False)
                        or getattr(previous_lane, "_oams_runout_detected", False)
                        or getattr(previous_lane, "_oams_cross_extruder_runout", False)
                    )
                    if not clear_previous_hub:
                        clear_previous_hub = not _lane_hardware_hub_has_filament(previous_lane)
                    if clear_previous_hub:
                        _clear_lane_virtual_hub_sensor(previous_lane)
                elif previous_lane_name == current_lane_name:
                    self.logger.debug(
                        f"OpenAMS lane tool-state load for {current_lane_name} received while lane is already active; skipping unset_lane_loaded self-clear"
                    )
            try:
                # Call set_tool_loaded() instead of set_loaded() since filament is loaded to toolhead
                # This properly sets extruder.lane_loaded which is needed for lane tracking
                lane.set_tool_loaded()
            except Exception as e:
                self.logger.error(f"Failed to mark lane {lane.name} as loaded: {e}")
            try:
                lane.sync_to_extruder()
                # Do not block on toolhead.wait_moves() here.
                # This callback can run mid-print during same-FPS runout handoff;
                # waiting for move queue drain can defer state updates until print end.
            except Exception as e:
                self.logger.error(f"Failed to sync lane {lane.name} to extruder: {e}")
            if afc_function is not None:
                try:
                    current_lane_obj = afc_function.get_current_lane_obj()
                    current_lane_name = getattr(current_lane_obj, "name", None) if current_lane_obj is not None else None
                    extruder_lane_name = getattr(lane.extruder_obj, "lane_loaded", None)
                    needs_activate_sync = (
                        current_lane_name != lane.name
                        or extruder_lane_name != lane.name
                    )

                    if needs_activate_sync:
                        afc_function.handle_activate_extruder()
                except Exception as e:
                    self.logger.error(f"Failed to activate extruder after loading lane {lane.name}: {e}")
            try:
                self.afc.save_vars()
            except Exception as e:
                self.logger.error(f"Failed to persist AFC state after lane load: {e}")
            try:
                self.select_lane(lane)
            except Exception:
                self.logger.debug(f"Unable to select lane {lane.name} during OpenAMS load")
            if self._lane_matches_extruder(lane):
                try:
                    force_update = True
                    if lane:
                        force_update = (getattr(lane, "tool_loaded", False) is not False)
                except Exception as e:
                    self.logger.error(f"Failed to mirror tool sensor state for loaded lane {lane.name}: {e}")
            return True

        current_lane = None
        if afc_function is not None:
            try:
                current_lane = afc_function.get_current_lane_obj()
            except Exception:
                current_lane = None

        if current_lane is lane and afc_function is not None:
            try:
                afc_function.unset_lane_loaded()
            except Exception as e:
                self.logger.error(f"Failed to unset currently loaded lane {lane.name}: {e}")
            return True

        if getattr(lane, "tool_loaded", False):
            try:
                lane.unsync_to_extruder()
                # Do not block on toolhead.wait_moves() in tool-state callbacks.
                # During active prints this can stall unload state propagation until idle.
            except Exception as e:
                self.logger.error(f"Failed to unsync lane {lane.name} from extruder: {e}")
            try:
                lane.set_tool_unloaded()
            except Exception as e:
                self.logger.error(f"Failed to mark lane {lane.name} as tool-unloaded: {e}")
            try:
                self.afc.save_vars()
            except Exception as e:
                self.logger.error(f"Failed to persist AFC state after unloading lane {lane.name}: {e}")

        # Ensure extruder tracking is cleared even if lane.tool_loaded was already false
        extruder_obj = getattr(lane, "extruder_obj", None)
        try:
            extruder_lane = getattr(extruder_obj, "lane_loaded", None)
        except Exception:
            extruder_lane = None
        if extruder_obj is not None and extruder_lane == getattr(lane, "name", None):
            try:
                extruder_obj.lane_loaded = None
                self.logger.debug(f"Cleared extruder.lane_loaded for {getattr(lane, 'name', None)} during unload cleanup")
            except Exception:
                self.logger.debug(f"Failed to clear extruder tracking for {getattr(lane, 'name', None)} during unload cleanup")
        return True

    def mark_cross_extruder_runout(self, lane_name: str) -> bool:
        """Mark a lane as cross-extruder for shared sensor handling."""

        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(
                f"Requested cross-extruder runout mark for {lane_name} but AFC unit {self.name} cannot resolve it"
            )
            return False

        try:
            lane._oams_runout_detected = False
            lane._oams_cross_extruder_runout = True
            self.logger.info(
                f"Marked lane {lane.name} as cross-extruder runout participant for shared sensor bypass"
            )
            return True
        except Exception as e:
            self.logger.error(f"Failed to mark lane {lane.name} for cross-extruder runout: {e}")
            return False

    def _is_event_for_unit(self, unit_name: Optional[str]) -> bool:
        """Check whether an event payload targets this unit."""
        if not unit_name:
            return False

        candidates = {str(self.name).lower()}
        if getattr(self, "oams_name", None):
            candidates.add(str(self.oams_name).lower())

        return str(unit_name).lower() in candidates

    def _handle_spool_loaded_event(self, *, event_type=None, **kwargs):
        """Update local state in response to a spool_loaded event."""
        unit_name = kwargs.get("unit_name")
        if not self._is_event_for_unit(unit_name):
            return

        spool_index = kwargs.get("spool_index")
        try:
            normalized_index = int(spool_index) if spool_index is not None else None
        except (TypeError, ValueError):
            normalized_index = None

        lane = self._find_lane_by_spool(normalized_index)
        if lane is None:
            self.logger.debug(f"_handle_spool_loaded_event: lane not found for spool_index={spool_index}")
            return

        # Use previous_loaded from event payload (passed before callbacks updated state)
        # Fall back to lane.load_state for backwards compatibility
        previous_loaded = kwargs.get("previous_loaded")
        if previous_loaded is None:
            previous_loaded = bool(getattr(lane, "load_state", False))
        else:
            previous_loaded = bool(previous_loaded)

        # Check global capture_td1_data setting from AFC_prep config
        capture_td1_data = False
        try:
            prep_obj = self.printer.lookup_object('AFC_prep', None)
            if prep_obj is not None:
                capture_td1_data = getattr(prep_obj, "get_td1_data", False) and self.afc.td1_present
        except Exception:
            pass
        self.logger.debug(f"_handle_spool_loaded_event: lane={lane.name} previous_loaded={previous_loaded} capture_td1_data={capture_td1_data}")

        eventtime = kwargs.get("eventtime", 0.0)

        # Clear stale spool metadata from a previously loaded spool before
        # set_loaded() applies defaults / next_spool_id.  AFC's _set_values()
        # only resets material and weight; spool_id, color, and temps would
        # otherwise persist across spool swaps.
        if not previous_loaded and not getattr(lane, "remember_spool", False):
            try:
                self.afc.spool.clear_values(lane)
            except Exception:
                pass

        # PHASE 1 REFACTOR: Use AFC native set_loaded() instead of direct state assignment
        # lane.set_loaded() handles:
        # - Sets lane.status = AFCLaneState.LOADED
        # - Calls self.unit_obj.lane_loaded(self) for LED updates
        # - Calls self.afc.spool._set_values(self) for spool metadata
        # This eliminates manual state management and ensures proper state transitions
        try:
            lane.set_loaded()
        except AttributeError:
            # moonraker not ready yet during startup; state vars already set
            pass

        # Schedule TD-1 capture with 4.2-second delay if capture_td1_data is enabled in AFC_prep
        # The delay allows AMS auto-load sequence to complete (pushes to hub -> retracts -> settles)
        # Check lane-level td1_device_id first, fall back to unit-level
        td1_device = getattr(lane, "td1_device_id", None) or getattr(self, "td1_device_id", None)

        # During PREP, skip event-based TD-1 capture - PREP's _td1_prep handles sequential capture
        # This prevents all lanes from trying to capture simultaneously when their timers fire
        in_prep = not getattr(self.afc, "prep_done", True)

        should_capture = (
            not previous_loaded
            and capture_td1_data
            and td1_device
            and not in_prep  # Only capture via events after PREP is done
        )
        self.logger.debug(f"TD-1 capture decision: previous_loaded={previous_loaded} capture_td1_data={capture_td1_data} td1_device={td1_device} in_prep={in_prep} should_capture={should_capture}")
        if should_capture:
            lane_name = lane.name
            try:
                # Cancel any existing pending timer for this lane
                if lane_name in self._pending_spool_loaded_timers:
                    try:
                        old_timer = self._pending_spool_loaded_timers[lane_name]
                        self.reactor.unregister_timer(old_timer)
                    except Exception:
                        pass  # Timer may have already fired

                # Register new timer with 4.2-second delay for TD-1 capture
                # AMS pushes filament to hub sensor then retracts - need time for this sequence
                timer_callback = self._trigger_td1_capture_delayed(lane_name)
                timer = self.reactor.register_timer(timer_callback, self.reactor.monotonic() + 4.2)
                self._pending_spool_loaded_timers[lane_name] = timer

                self.logger.info(f"Scheduled TD-1 capture for {lane_name} in 4.2 seconds (allowing AMS to settle)")
            except Exception as e:
                self.logger.error(f"Failed to schedule TD-1 capture for {lane_name}: {e}")
        extruder_name = getattr(lane, "extruder_name", None)
        if extruder_name is None and self.registry is not None:
            try:
                extruder_name = self.registry.resolve_extruder(lane.name)
            except Exception:
                self.logger.debug(
                    f"_handle_spool_loaded_event: unable to resolve extruder for {lane.name}; recording load without extruder"
                )
                extruder_name = None

        self.record_load(extruder=extruder_name, lane_name=lane.name)

        if self.hardware_service is not None and normalized_index is not None:
            hub_state = getattr(lane, "loaded_to_hub", None)
            tool_state = getattr(lane, "tool_loaded", None)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    True,
                    hub_state if hub_state is not None else None,
                    eventtime,
                    spool_index=normalized_index,
                    tool_state=tool_state if tool_state is not None else None,
                    emit_spool_event=False,
                )
            except Exception as e:
                self.logger.error(f"Failed to mirror spool load event for {lane.name}: {e}")
    def _handle_spool_unloaded_event(self, *, event_type=None, **kwargs):
        """Update local state in response to a spool_unloaded event."""
        unit_name = kwargs.get("unit_name")
        if not self._is_event_for_unit(unit_name):
            return

        spool_index = kwargs.get("spool_index")
        try:
            normalized_index = int(spool_index) if spool_index is not None else None
        except (TypeError, ValueError):
            normalized_index = None

        lane = self._find_lane_by_spool(normalized_index)
        if lane is None:
            return

        # Ignore initial startup "unloaded" baselines during PREP/bring-up.
        # These can be emitted before AFC/Moonraker is fully initialized and
        # should not be treated as real unload transitions.
        previous_loaded = kwargs.get("previous_loaded")
        if previous_loaded is not None:
            previous_loaded = bool(previous_loaded)
        in_prep = not getattr(self.afc, "prep_done", True)
        if in_prep and previous_loaded is False:
            self.logger.debug(
                "Skipping spool_unloaded baseline for %s during PREP (spool_index=%s)",
                lane.name,
                normalized_index,
            )
            return

        # PHASE 1 REFACTOR: Use AFC native set_unloaded() instead of direct state assignments
        # lane.set_unloaded() handles:
        # - Sets lane.tool_loaded = False
        # - Sets lane.status = AFCLaneState.NONE
        # - Sets lane.loaded_to_hub = False
        # - Clears lane.td1_data = {}
        # - Calls self.afc.spool.clear_values(self)
        # - Calls self.unit_obj.lane_unloaded(self)
        # This eliminates manual state management and ensures proper cleanup
        #
        # PHASE 2 REFACTOR: Removed dictionary updates
        # AFC native lane.set_unloaded() handles all state cleanup
        try:
            lane.set_unloaded()
        except AttributeError:
            # moonraker not ready yet during startup; state vars already set
            pass

        eventtime = kwargs.get("eventtime", 0.0)
        if self.hardware_service is not None and normalized_index is not None:
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    False,
                    False,
                    eventtime,
                    spool_index=normalized_index,
                    tool_state=False,
                    emit_spool_event=False,
                )
            except Exception as e:
                self.logger.error(f"Failed to mirror spool unload event for {lane.name}: {e}")
    # ---- AFC_OAMS_CLEAR_ERRORS / AFC_OAMS_STATUS gcode commands ----

    def cmd_AFC_OAMS_CLEAR_ERRORS(self, gcmd):
        """Clear OpenAMS errors, LED states, and resync AFC state with hardware."""
        oams = self.oams
        if oams is None:
            gcmd.respond_info(f"No OAMS hardware available for {self.name}")
            return

        # Stop monitor during clear
        if self._monitor is not None:
            self._monitor.stop()

        try:
            # Abort any in-flight action
            oams.abort_current_action()
            self.reactor.pause(self.reactor.monotonic() + 0.1)

            # Clear hardware errors (clears all LED errors too)
            oams.clear_errors()
            oams.current_spool = None
            self.reactor.pause(self.reactor.monotonic() + 0.2)

            # Clear LED tracking in follower
            if self._follower is not None:
                self._follower._led_error_state.clear()
                # Reset follower tracking
                fstate = self._follower.get_follower_state(self.oams_name)
                fstate.last_state = None
                fstate.coasting = False
                fstate.coast_start_pos = 0.0
                fstate.had_filament = False

            # Reset monitor state
            if self._monitor is not None:
                self._monitor.state.reset()

            # Clear AFC lane_loaded state
            try:
                afc_fn = getattr(self.afc, "function", None)
                unset_fn = getattr(afc_fn, "unset_lane_loaded", None)
                if unset_fn:
                    unset_fn()
            except Exception as e:
                self.logger.warning(f"Failed to clear AFC lane_loaded state: {e}")

            # Reconcile from hardware sensors
            try:
                self._sync_afc_from_hardware_at_startup()
            except Exception as e:
                self.logger.warning(f"State reconciliation failed: {e}")

            # Restore LEDs based on reconciled state
            for lane_name, lane in self.lanes.items():
                try:
                    if getattr(lane, 'tool_loaded', False):
                        self.lane_tool_loaded(lane)
                    elif getattr(lane, 'load_state', False):
                        self.lane_loaded(lane)
                    else:
                        self.lane_unloaded(lane)
                except Exception:
                    pass

            # Force followers on
            if self._follower is not None:
                fps_state = self._get_monitor_state()
                if fps_state:
                    self._follower.set_follower_state(
                        fps_state, oams, 1, 1, "clear_errors", force=True)

        finally:
            # Only restart monitor if a lane is loaded to toolhead
            # (otherwise false clog detection fires during idle)
            if self._monitor is not None:
                has_loaded = any(
                    getattr(lane, 'tool_loaded', False)
                    for lane in self.lanes.values())
                if has_loaded:
                    self._monitor.start(oams)

        # Build summary
        loaded = []
        for lane_name, lane in self.lanes.items():
            if getattr(lane, 'tool_loaded', False):
                loaded.append(lane_name)
        summary = f"[OK] {self.name} cleared and ready"
        if loaded:
            summary += f" | Loaded: {', '.join(loaded)}"
        else:
            summary += " | No lanes loaded"
        gcmd.respond_info(summary)

    def cmd_AFC_OAMS_STATUS(self, gcmd):
        """Show OpenAMS unit status."""
        oams = self.oams
        parts = [f"=== {self.name} ({self.oams_name}) ==="]

        if oams is None:
            parts.append("Hardware: NOT AVAILABLE")
            gcmd.respond_info("\n".join(parts))
            return

        parts.append(f"Hardware: OK (current_spool={oams.current_spool})")

        # FPS / Monitor state
        st = self._get_monitor_state()
        if st is not None:
            from extras.AFC_OpenAMS_monitor import FPSLoadState
            state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
            parts.append(
                f"Monitor: state={state_names.get(st.state, st.state)} "
                f"lane={st.current_lane} stuck={st.stuck_active} clog={st.clog_active}")
        else:
            parts.append("Monitor: not running")

        # Follower state
        if self._follower:
            fstate = self._follower.get_follower_state(self.oams_name)
            parts.append(
                f"Follower: last_state={fstate.last_state} "
                f"coasting={fstate.coasting}")
        else:
            parts.append("Follower: not available")

        # Lane summary
        for lane_name, lane in self.lanes.items():
            tool = "TOOL_LOADED" if getattr(lane, 'tool_loaded', False) else ""
            hub = "HUB" if getattr(lane, 'loaded_to_hub', False) else ""
            f1s = "F1S" if getattr(lane, 'load_state', False) else ""
            status = " | ".join(filter(None, [tool, hub, f1s])) or "empty"
            parts.append(f"  {lane_name}: {status}")

        gcmd.respond_info("\n".join(parts))

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        """Run the OpenAMS HUB HES calibration for a specific lane."""
        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            self.logger.info("SPOOL parameter is required for OpenAMS HUB HES calibration.")
            return

        lane = self._find_lane_by_spool(spool_index)
        if lane is None:
            self.logger.info(f"Could not find lane for spool index {spool_index}.")
            return

        # Check if this lane's extruder has something loaded to toolhead
        extruder_name = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        loaded_lane = self._check_toolhead_loaded(extruder_name)
        if loaded_lane:
            self.logger.info(f"Cannot run OpenAMS calibration while {loaded_lane} is loaded to the toolhead on this extruder. Please unload the tool and try again.")
            return

        lane_name = getattr(lane, "name", None)
        self._calibrate_hub_hes_spool(spool_index, gcmd, lane_name=lane_name)

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL(self, gcmd):
        """Calibrate HUB HES for every loaded OpenAMS lane in this unit."""
        # Check if any lane on THIS UNIT has something loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                self.logger.info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        prompt.p_end()

        calibrations = []
        skipped = []
        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue
            spool_index = self._get_openams_spool_index(lane)
            if spool_index is None:
                skipped.append(getattr(lane, "name", str(lane)))
                continue
            calibrations.append((lane, spool_index))

        if not calibrations:
            self.logger.info("No loaded OpenAMS lanes were found to calibrate HUB HES values.")
            return

        successful = 0
        for lane, spool_index in calibrations:
            if self._calibrate_hub_hes_spool(spool_index, gcmd, lane_name=getattr(lane, "name", None)):
                successful += 1

        self.logger.info(f"Completed HUB HES calibration for {successful} OpenAMS lane(s).")

        if skipped:
            skipped_lanes = ", ".join(skipped)
            self.logger.info("Skipped HUB HES calibration for lanes lacking OpenAMS mapping: {}.".format(skipped_lanes))

    def cmd_AFC_OAMS_CALIBRATE_PTFE(self, gcmd):
        """Run the OpenAMS PTFE calibration for this unit.

        Any lane/spool on the unit can be used — the PTFE distance from
        each bay to the hub is identical.  The result is saved to the
        [oams ...] section so the firmware reads it on next startup.
        """
        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            self.logger.info("SPOOL parameter is required for OpenAMS PTFE calibration.")
            return

        lane = self._find_lane_by_spool(spool_index)
        if lane is None:
            self.logger.info(f"Could not find lane for spool index {spool_index}.")
            return

        # Check if this lane's extruder has something loaded to toolhead
        extruder_name = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        loaded_lane = self._check_toolhead_loaded(extruder_name)
        if loaded_lane:
            self.logger.info(f"Cannot run OpenAMS calibration while {loaded_lane} is loaded to the toolhead on this extruder. Please unload the tool and try again.")
            return

        lane_name = getattr(lane, "name", None)
        self._calibrate_ptfe_spool(spool_index, gcmd, lane=lane, lane_name=lane_name)

    def _calibrate_hub_hes_spool(self, spool_index, gcmd, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            self.logger.info("Unable to determine OpenAMS index for HUB HES calibration.")
            return False

        command = f"OAMS_CALIBRATE_HUB_HES OAMS={oams_index} SPOOL={spool_index}"
        lane_label = lane_name or f"spool {spool_index}"
        self.logger.info(f"Running HUB HES calibration for {lane_label} with '{command}'.")

        try:
            messages = self._run_command_with_capture(command)
        except Exception as e:
            self.logger.error(f"Failed to execute OpenAMS HUB HES calibration for spool {spool_index}: {e}")
            self.logger.info(f"Failed to execute HUB HES calibration for {lane_label}. See logs.")
            return False

        hub_values = self._parse_hub_hes_messages(messages)
        if not hub_values:
            self.logger.info(f"Completed {command} but no HUB HES value was reported. Check OpenAMS status logs.")
            return False

        config_values = self._read_config_sequence("hub_hes_on")
        if not config_values:
            config_values = self._last_hub_hes_values or []

        if not config_values:
            self.logger.info("Could not find hub_hes_on in your cfg; update the value manually.")
            return False

        values = list(config_values)
        max_length = len(values)
        updated_indices = []
        for index, parsed_value in sorted(hub_values.items()):
            if index >= max_length:
                self.logger.info("HUB HES calibration reported index {} but your cfg only defines {} value(s); update the remaining entries manually.".format(index, max_length))
                continue
            values[index] = parsed_value
            updated_indices.append(index)

        if not updated_indices:
            self.logger.info("Completed {} but no HUB HES value was stored; check your cfg.".format(command))
            return False

        formatted = self._format_sequence(values)
        if not formatted:
            self.logger.info("Unable to format HUB HES calibration values.")
            return False

        if not self._write_config_value("hub_hes_on", formatted):
            self.logger.info("Failed to update hub_hes_on in your cfg; please update it manually.")
            return False

        self._last_hub_hes_values = values

        if updated_indices:
            if len(updated_indices) == 1:
                index_text = f"index {updated_indices[0]}"
            else:
                index_text = "indices " + ", ".join(str(i) for i in updated_indices)
            self.logger.info(f"Stored OpenAMS hub_hes_on {formatted} in your cfg (updated {index_text}).")
        else:
            self.logger.info(f"Stored OpenAMS hub_hes_on {formatted} in your cfg.")
        return True

    def _calibrate_ptfe_spool(self, spool_index, gcmd, lane=None, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            self.logger.info("Unable to determine OpenAMS index for PTFE calibration.")
            return False

        command = f"OAMS_CALIBRATE_PTFE_LENGTH OAMS={oams_index} SPOOL={spool_index}"
        lane_label = lane_name or f"spool {spool_index}"
        self.logger.info(f"Running PTFE calibration for {lane_label} with '{command}'.")

        try:
            messages = self._run_command_with_capture(command)
        except Exception as e:
            self.logger.error(f"Failed to execute OpenAMS PTFE calibration for spool {spool_index}: {e}")
            self.logger.info(f"Failed to execute PTFE calibration for {lane_label}. See logs.")
            return False

        captured = self._parse_ptfe_messages(messages)
        value = None
        if captured:
            if 0 <= spool_index < len(captured):
                value = captured[spool_index]
            elif len(captured) == 1:
                value = captured[0]

        if value is None:
            self.logger.info(f"Completed {command} but no PTFE length was reported. Check OpenAMS status logs.")
            return False

        formatted_value = self._format_numeric(value)
        if formatted_value is None:
            self.logger.info("Unable to format PTFE calibration value for config storage.")
            return False

        # All lanes on the same OAMS unit share an identical PTFE path to
        # the hub, so calibration from any lane applies to the whole unit.
        # Save to the [oams ...] section where the firmware reads it.
        if self.oams is None:
            self.logger.info("OAMS object not available, cannot save ptfe_length.")
            return False

        section = self.oams.config_name
        cal_msg = f"\n{section} ptfe_length: {formatted_value}"
        try:
            self.function.ConfigRewrite(section, "ptfe_length", formatted_value, cal_msg)
        except Exception as e:
            self.logger.error(f"Failed to persist ptfe_length for {section}: {e}")
            self.logger.info("Failed to update ptfe_length in your cfg; please update it manually.")
            return False

        self.logger.info(
            f"PTFE calibration complete for {lane_label}: ptfe_length {formatted_value} "
            f"saved to [{section}]. Any lane on this unit can be used to calibrate — "
            f"the PTFE distance to the hub is identical for all bays."
        )
        return True

    def _run_command_with_capture(self, command):
        captured: List[str] = []
        original = getattr(self.gcode, "respond_info", None)

        if original is None:
            self.gcode.run_script_from_command(command)
            return captured

        def _capture(this, message, *args, **kwargs):
            if isinstance(message, str):
                captured.append(message)
            return original(message, *args, **kwargs)

        self.gcode.respond_info = MethodType(_capture, self.gcode)
        try:
            self.gcode.run_script_from_command(command)
        finally:
            self.gcode.respond_info = original

        return captured

    def _parse_hub_hes_messages(self, messages):
        results = {}
        pattern = re.compile(r"HES\s*([0-9]+)\D+(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

        for message in messages or []:
            if not isinstance(message, str):
                continue
            for match in pattern.finditer(message):
                try:
                    index = int(match.group(1))
                    value = float(match.group(2))
                except (TypeError, ValueError):
                    continue
                results[index] = value

        return results

    def _parse_ptfe_messages(self, messages):
        values = []
        pattern = re.compile(r"(?:ptfe|bowden)[^0-9\-]*(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

        for message in messages or []:
            if not isinstance(message, str):
                continue
            for match in pattern.finditer(message):
                try:
                    values.append(float(match.group(1)))
                except (TypeError, ValueError):
                    continue

        return values

    def _format_numeric(self, value):
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None

        if abs(number - round(number)) <= 1e-6:
            return str(int(round(number)))

        return f"{number:.6f}".rstrip("0").rstrip(".")

    def _format_sequence(self, values):
        if values is None:
            return None

        formatted = []
        for value in list(values):
            formatted_value = self._format_numeric(value)
            if formatted_value is None:
                continue
            formatted.append(formatted_value)

        return ", ".join(formatted) if formatted else None

    def _config_section_name(self):
        name = getattr(self, "oams_name", None)
        if not name:
            return None
        return f"oams {name}"

    def _read_config_sequence(self, key):
        section = self._config_section_name()
        config_dir = getattr(self.afc, "cfgloc", None)
        if not section or not config_dir:
            return None

        header = f"[{section}]".strip().lower()
        key_pattern = re.compile(rf"^{re.escape(key)}\s*:\s*(.+)$", re.IGNORECASE)

        try:
            filenames = sorted(filename for filename in os.listdir(config_dir) if filename.lower().endswith(".cfg"))
        except OSError:
            return None

        for filename in filenames:
            path = os.path.join(config_dir, filename)
            try:
                with open(path, "r", encoding="utf-8") as cfg_file:
                    in_section = False
                    for line in cfg_file:
                        stripped = line.strip()
                        if not stripped:
                            continue
                        if stripped.startswith("[") and stripped.endswith("]"):
                            in_section = stripped.lower() == header
                            continue
                        if not in_section:
                            continue
                        match = key_pattern.match(stripped)
                        if not match:
                            continue
                        value_part = match.group(1)
                        if "#" in value_part:
                            value_part = value_part.split("#", 1)[0]
                        raw_value = value_part.strip()
                        return self._parse_sequence_string(raw_value)
            except OSError:
                continue

        return None

    def _parse_sequence_string(self, raw_value):
        if raw_value is None:
            return []

        values = []
        for token in raw_value.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                values.append(float(token))
            except (TypeError, ValueError):
                continue

        return values

    def _write_config_value(self, key, value):
        section = self._config_section_name()
        if not section:
            return False
        msg = f"\n{self.name} {key}: Saved {value}"
        try:
            self.function.ConfigRewrite(section, key, value, msg)
        except Exception as e:
            self.logger.error(f"Failed to persist {key} for OpenAMS unit {self.name}: {e}")
            return False
        return True

    def _check_toolhead_loaded(self, extruder_name=None):
        """Check if a lane is currently loaded to the specified extruder's toolhead.

        Args:
            extruder_name: Optional extruder name to check. If None, checks all extruders.

        Returns: Lane name if loaded, None otherwise.
        """
        # If specific extruder provided, only check lanes on that extruder
        if extruder_name:
            for lane_name, lane in self.afc.lanes.items():
                if getattr(lane, "tool_loaded", False):
                    lane_extruder = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
                    if lane_extruder == extruder_name:
                        return lane_name
        else:
            # Check all lanes across all AFC units
            for lane_name, lane in self.afc.lanes.items():
                if getattr(lane, "tool_loaded", False):
                    return lane_name
        return None

    def _find_lane_by_spool(self, spool_index):
        """Resolve lane by spool index using registry when available."""
        if spool_index is None:
            return None

        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None

        registry_unit = self.oams_name or self.name
        if self.registry is not None:
            lane_info = self.registry.get_by_spool(registry_unit, normalized)
            if lane_info is not None:
                lane = self.lanes.get(lane_info.lane_name)
                if lane is not None:
                    return lane

        return self._lane_by_local_index(normalized)

    def _lane_by_local_index(self, normalized: int):
        for candidate in self.lanes.values():
            lane_index = getattr(candidate, "index", None)
            try:
                lane_index = int(lane_index) - 1
            except (TypeError, ValueError):
                continue

            if lane_index == normalized:
                return candidate

        return None

    def _get_openams_index(self):
        """Helper to extract OAMS index (OPTIMIZED with caching)."""
        # Cache OAMS index after first lookup
        if self._cached_oams_index is not None:
            return self._cached_oams_index

        if self.oams is not None:
            oams_idx = getattr(self.oams, "oams_idx", None)
            if oams_idx is not None:
                self._cached_oams_index = oams_idx
                return oams_idx
        return None

    def _get_openams_spool_index(self, lane):
        """Helper to extract spool index from lane."""
        try:
            return int(getattr(lane, "index", 0)) - 1
        except (TypeError, ValueError):
            return None

    def check_runout(self, lane=None):
        if lane is None:
            return False
        if getattr(lane, "unit_obj", None) is not self:
            return False
        try:
            is_printing = self.afc.function.is_printing()
        except Exception:
            is_printing = False
        return bool(is_printing)

    def _register_sync_dispatcher(self) -> None:
        """Ensure the shared sync command is available for all OpenAMS units."""
        cls = self.__class__
        if not cls._sync_command_registered:
            self.gcode.register_command(
                "AFC_FPS_SYNC_TOOL_SENSOR",
                cls._dispatch_sync_tool_sensor,
                desc=self.cmd_SYNC_TOOL_SENSOR_help,
            )
            cls._sync_command_registered = True

        cls._sync_instances[self.name] = self

    @classmethod
    def _extract_raw_param(cls, commandline: str, key: str) -> Optional[str]:
        """Recover multi-word parameter values from the raw command line."""
        if not commandline:
            return None

        key_upper = key.upper() + "="
        command_upper = commandline.upper()
        start = command_upper.find(key_upper)
        if start == -1:
            return None

        start += len(key_upper)
        remainder = commandline[start:]
        match = re.search(r"\s[A-Z0-9_]+=|;", remainder)
        end = start + match.start() if match else len(commandline)

        value = commandline[start:end].strip()
        if not value:
            return None

        if value[0] in ('\'', '"') and value[-1] == value[0]:
            value = value[1:-1]

        return value

    @classmethod
    def _dispatch_sync_tool_sensor(cls, gcmd):
        """Route sync requests to the correct AMS instance, tolerating spaces."""
        unit_value = gcmd.get("UNIT", None)
        if not unit_value:
            unit_value = cls._extract_raw_param(gcmd.get_commandline(), "UNIT")

        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            lane_name = gcmd.get("FPS", None)

        if lane_name is None:
            commandline = gcmd.get_commandline()
            lane_name = cls._extract_raw_param(commandline, "LANE")
            if lane_name is None:
                lane_name = cls._extract_raw_param(commandline, "FPS")

        for instance in cls._sync_instances.values():
            if not instance._unit_matches(unit_value):
                continue

            resolved_lane = instance._resolve_lane_alias(lane_name)
            eventtime = instance.reactor.monotonic()

def _has_openams_hardware(printer):
    """Check if any OpenAMS hardware is configured in the system.

    Returns True if any [oams ...] sections are found in the configuration.
    This prevents unnecessary OpenAMS initialization for users with other unit types.
    """
    try:
        # Try to find any OAMS objects in the printer
        # This will work after all configs have been loaded during handle_ready
        for obj_name in printer.objects:
            if obj_name.startswith('oams '):
                return True
        return False
    except Exception:
        # If we can't check, assume OAMS might be present to avoid breaking existing setups
        return True


def load_config_prefix(config):
    """Load OpenAMS integration - actual hardware check happens at handle_ready."""
    # Note: We can't reliably check for OAMS sections during config load because
    # they may be in [include] files that haven't been processed yet.
    # The actual OAMS hardware check happens in handle_ready() after all configs load.
    return afcAMS(config)