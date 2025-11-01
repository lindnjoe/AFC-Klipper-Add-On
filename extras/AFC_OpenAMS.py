# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AMS integration helpers for Armored Turtle AFC."""

from __future__ import annotations

import math
import os
import re
import traceback
from textwrap import dedent
from typing import Dict, Optional, List, Tuple
from types import MethodType

from configparser import Error as ConfigError
try: from extras.AFC_utils import ERROR_STR
except: raise ConfigError("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLane, AFCLaneState
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))
try: from extras.AFC_utils import add_filament_switch
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_utils", trace=traceback.format_exc()))
try: from extras.AFC_respond import AFCprompt
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))
try: import extras.AFC_extruder as _afc_extruder_mod
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_extruder", trace=traceback.format_exc()))

try:  # pragma: no cover - optional at config parse time
    from extras.openams_integration import AMSHardwareService, AMSRunoutCoordinator
except Exception:  # pragma: no cover - integration may be absent during import
    AMSHardwareService = None
    AMSRunoutCoordinator = None

SYNC_INTERVAL = 2.0
_MISSING = object()
_ORIGINAL_LANE_PRE_SENSOR = getattr(AFCLane, "get_toolhead_pre_sensor_state", None)

class _VirtualRunoutHelper:
    """Minimal runout helper used by AMS-managed virtual sensors."""

    def __init__(self, printer, name, runout_cb=None, enable_runout=False):
        self.printer = printer
        self._reactor = printer.get_reactor()
        self.name = name
        self.runout_callback = runout_cb
        self.sensor_enabled = bool(enable_runout)
        self.filament_present = False
        self.insert_gcode = None
        self.runout_gcode = None
        self.event_delay = 0.0
        self.min_event_systime = self._reactor.NEVER

    def note_filament_present(self, eventtime=None, is_filament_present=False, **_kwargs):
        if eventtime is None:
            eventtime = self._reactor.monotonic()

        new_state = bool(is_filament_present)
        if new_state == self.filament_present:
            return

        self.filament_present = new_state

        if (
            not new_state
            and self.sensor_enabled
            and callable(self.runout_callback)
        ):
            try:
                self.runout_callback(eventtime)
            except TypeError:
                self.runout_callback(eventtime=eventtime)

    def get_status(self, _eventtime=None):
        return {
            "filament_detected": bool(self.filament_present),
            "enabled": bool(self.sensor_enabled),
        }

class _VirtualFilamentSensor:
    """Lightweight filament sensor placeholder for AMS virtual pins."""

    QUERY_HELP = "Query the status of the Filament Sensor"
    SET_HELP = "Sets the filament sensor on/off"

    def __init__(self, printer, name, show_in_gui=True, runout_cb=None, enable_runout=False):
        self.printer = printer
        self.name = name
        self._object_name = f"filament_switch_sensor {name}"
        self.runout_helper = _VirtualRunoutHelper(
            printer,
            name,
            runout_cb=runout_cb,
            enable_runout=enable_runout,
        )

        objects = getattr(printer, "objects", None)
        if isinstance(objects, dict):
            objects.setdefault(self._object_name, self)
            if not show_in_gui:
                hidden_key = "_" + self._object_name
                objects[hidden_key] = objects.pop(self._object_name)

        gcode = printer.lookup_object("gcode")
        try:
            gcode.register_mux_command(
                "QUERY_FILAMENT_SENSOR",
                "SENSOR",
                name,
                self.cmd_QUERY_FILAMENT_SENSOR,
                desc=self.QUERY_HELP,
            )
        except Exception:
            pass

        try:
            gcode.register_mux_command(
                "SET_FILAMENT_SENSOR",
                "SENSOR",
                name,
                self.cmd_SET_FILAMENT_SENSOR,
                desc=self.SET_HELP,
            )
        except Exception:
            pass

    def get_status(self, eventtime):
        return self.runout_helper.get_status(eventtime)

    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        status = self.runout_helper.get_status(None)
        if status["filament_detected"]:
            msg = f"Filament Sensor {self.name}: filament detected"
        else:
            msg = f"Filament Sensor {self.name}: filament not detected"
        gcmd.respond_info(msg)

    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.runout_helper.sensor_enabled = bool(gcmd.get_int("ENABLE", 1))

def _normalize_extruder_name(name: Optional[str]) -> Optional[str]:
    """Return a case-insensitive token for comparing extruder aliases."""

    if not name or not isinstance(name, str):
        return None

    normalized = name.strip()
    if not normalized:
        return None

    lowered = normalized.lower()
    if lowered.startswith("ams_"):
        lowered = lowered[4:]

    return lowered or None

def _normalize_ams_pin_value(pin_value) -> Optional[str]:
    """Return the cleaned AMS_* token stripped of comments and modifiers."""

    if not isinstance(pin_value, str):
        return None

    cleaned = pin_value.strip()
    if not cleaned:
        return None

    # Remove any inline comments commonly used to preserve original pins.
    for comment_char in ("#", ";"):
        idx = cleaned.find(comment_char)
        if idx != -1:
            cleaned = cleaned[:idx].strip()
    if not cleaned:
        return None

    while cleaned and cleaned[0] in "!^":
        cleaned = cleaned[1:]

    return cleaned or None

def _patch_extruder_for_virtual_ams() -> None:
    """Patch AFC extruders so AMS_* tool pins avoid config-time errors."""

    extruder_cls = getattr(_afc_extruder_mod, "AFCExtruder", None)
    if extruder_cls is None or getattr(extruder_cls, "_ams_virtual_tool_patched", False):
        return

    base_init = extruder_cls.__init__

    class _ProxyConfig:
        def __init__(self, original):
            self._original = original

        def get(self, key, *args, **kwargs):
            if key == "pin_tool_start":
                return "buffer"
            return self._original.get(key, *args, **kwargs)

        def __getattr__(self, item):
            return getattr(self._original, item)

    def _patched_init(self, config):
        try:
            pin_value = config.get("pin_tool_start", None)
        except Exception:
            pin_value = None

        normalized = _normalize_ams_pin_value(pin_value)
        proxy_config = config

        if normalized:
            if normalized.upper().startswith("AMS_"):
                proxy_config = _ProxyConfig(config)
            else:
                normalized = None

        base_init(self, proxy_config)

        if not normalized:
            return

        show_sensor = getattr(self, "enable_sensors_in_gui", True)
        enable_runout = getattr(self, "enable_runout", False)
        runout_cb = getattr(self, "handle_start_runout", None)

        setattr(self, "_ams_virtual_tool_name", normalized)

        # OpenAMS is responsible for dispatching runout events for AMS tool
        # start sensors, so keep the virtual Klipper sensor purely for state
        # reporting and disable the local runout callback wiring.
        virtual = _VirtualFilamentSensor(
            self.printer,
            normalized,
            show_in_gui=show_sensor,
            runout_cb=runout_cb,
            enable_runout=enable_runout,
        )

        self.tool_start = pin_value
        self.fila_tool_start = virtual
        self.tool_start_state = bool(virtual.runout_helper.filament_present)

    extruder_cls.__init__ = _patched_init
    extruder_cls._ams_virtual_tool_patched = True

class afcAMS(afcUnit):
    """AFC unit subclass that synchronises state with OpenAMS."""

    _sync_command_registered = False
    _sync_instances: Dict[str, "afcAMS"] = {}
    _ptfe_command_registered = False
    _ptfe_instances: Dict[int, "afcAMS"] = {}

    def __init__(self, config):
        super().__init__(config)
        self.type = "OpenAMS"

        # OpenAMS specific options
        self.oams_name = config.get("oams", "oams1")
        self.interval = config.getfloat("interval", SYNC_INTERVAL, above=0.0)

        self.reactor = self.printer.get_reactor()
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Track previous sensor state to only forward changes
        self._last_lane_states: Dict[str, bool] = {}
        self._last_hub_states: Dict[str, bool] = {}
        self._virtual_tool_sensor = None
        self._last_virtual_tool_state: Optional[bool] = None
        self._lane_tool_latches: Dict[str, bool] = {}
        self._lane_tool_latches_by_lane: Dict[object, bool] = {}
        self._lane_feed_activity: Dict[str, bool] = {}
        self._lane_feed_activity_by_lane: Dict[object, bool] = {}
        self._last_encoder_clicks: Optional[int] = None
        self._last_hub_hes_string: Optional[str] = None
        self._last_ptfe_string: Optional[str] = None
        self._last_lane_ptfe_lengths: Dict[str, float] = {}
        self.oams = None
        self.hardware_service = None

        if AMSRunoutCoordinator is not None:
            self.hardware_service = AMSRunoutCoordinator.register_afc_unit(self)
        elif AMSHardwareService is not None:
            self.hardware_service = AMSHardwareService.for_printer(
                self.printer, self.oams_name, self.logger
            )

        self.gcode.register_mux_command(
            "AFC_OAMS_CALIBRATE_HUB_HES_ALL",
            "UNIT",
            self.name,
            self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL,
            desc=self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL_help,
        )

        self._register_sync_dispatcher()
        self._register_ptfe_calibration_dispatcher()

    cmd_UNIT_CALIBRATION_help = (
        "open prompt to calibrate OpenAMS HUB HES or PTFE length values"
    )

    def cmd_UNIT_CALIBRATION(self, gcmd):
        """Open the OpenAMS calibration prompt with customised buttons."""

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        title = f"{self.name} Calibration"
        text = "Select to calibrate the hub_hes value or oams ptfe length."

        buttons.append(
            (
                "Calibrate Lane HUB HES",
                "UNIT_LANE_CALIBRATION UNIT={}".format(self.name),
                "primary",
            )
        )

        direct_hubs = any(lane.is_direct_hub() for lane in self.afc.lanes.values())
        lanes_loaded = any(
            lane.load_state and not lane.is_direct_hub()
            for lane in self.afc.lanes.values()
        )

        if not direct_hubs or lanes_loaded:
            buttons.append(
                (
                    "Calibrate OAMS PTFE Length",
                    "UNIT_BOW_CALIBRATION UNIT={}".format(self.name),
                    "secondary",
                )
            )

        if self.afc.td1_defined:
            buttons.append(
                (
                    "Calibrate TD-1 Length",
                    "AFC_UNIT_TD_ONE_CALIBRATION UNIT={}".format(self.name),
                    "primary",
                )
            )

        back = [("Back to unit selection", "AFC_CALIBRATION", "info")]

        prompt.create_custom_p(title, text, buttons, True, None, back)

    cmd_UNIT_LANE_CALIBRATION_help = (
        "open prompt to calibrate OpenAMS hub_hes values for each lane"
    )

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        """Prompt for calibrating HUB HES values via OpenAMS macros."""

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
            if not lane.load_state:
                continue

            command = self._format_hub_hes_calibration_command(lane)
            if not command:
                continue

            button_label = "{}".format(lane)
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, command, button_style))

            index += 1
            if index % 4 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"
            all_lanes = None
        else:
            all_lanes = [
                (
                    "All lanes",
                    f"AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={self.name}",
                    "default",
                )
            ]

        back = [("Back", "UNIT_CALIBRATION UNIT={}".format(self.name), "info")]

        prompt.create_custom_p(title, text, all_lanes, True, buttons, back)

    cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL_help = (
        "calibrate the OpenAMS HUB HES value for all loaded lanes in the unit"
    )

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL(self, gcmd):
        """Iterate HUB HES calibration across every loaded OpenAMS lane."""

        prompt = AFCprompt(gcmd, self.logger)
        prompt.p_end()

        active_lane = getattr(self.afc, "current", None)
        active_lane_obj = None
        active_lane_name = None

        if active_lane is not None:
            if hasattr(active_lane, "unit_obj"):
                active_lane_obj = active_lane
                active_lane_name = getattr(active_lane_obj, "name", str(active_lane_obj))
            else:
                active_lane_name = str(active_lane)
                afc_lanes = getattr(self.afc, "lanes", None)
                if isinstance(afc_lanes, dict):
                    active_lane_obj = afc_lanes.get(active_lane_name)

        if (
            active_lane_obj is not None
            and getattr(active_lane_obj, "unit_obj", None) is self
            and getattr(active_lane_obj, "tool_loaded", False)
        ):
            lane_label = active_lane_name or getattr(active_lane_obj, "name", "current lane")
            gcmd.respond_info(
                "Cannot calibrate all OpenAMS lanes while {} is active on this unit. "
                "Please unload the current tool and try again.".format(lane_label)
            )
            return

        calibrations: List[Tuple[object, str]] = []
        skipped: List[str] = []

        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue

            command = self._format_hub_hes_calibration_command(lane)
            if not command:
                lane_name = getattr(lane, "name", str(lane))
                skipped.append(lane_name)
                continue

            calibrations.append((lane, command))

        if not calibrations:
            gcmd.respond_info(
                "No loaded OpenAMS lanes were found to calibrate HUB HES values."
            )
            return

        successful = 0

        for lane, command in calibrations:
            lane_name = getattr(lane, "name", str(lane))
            gcmd.respond_info(
                f"Running HUB HES calibration for {lane_name} with '{command}'."
            )
            try:
                self.gcode.run_script_from_command(command)
            except Exception:
                self.logger.exception(
                    "Failed to execute OpenAMS HUB HES calibration for lane %s", lane
                )
                gcmd.respond_info(
                    f"Failed to execute HUB HES calibration for {lane_name}. See logs."
                )
                continue

            successful += 1

        gcmd.respond_info(
            f"Completed HUB HES calibration for {successful} OpenAMS lane(s)."
        )

        if skipped:
            skipped_lanes = ", ".join(skipped)
            gcmd.respond_info(
                "Skipped HUB HES calibration for lanes lacking OpenAMS mapping: "
                f"{skipped_lanes}."
            )

    def _get_openams_index(self):
        oams_name = getattr(self, "oams_name", None)
        if not oams_name:
            return None

        match = re.search(r"(\d+)$", str(oams_name))
        if match:
            try:
                return int(match.group(1))
            except (TypeError, ValueError):
                return None
        return None

    def _get_openams_spool_index(self, lane):
        try:
            index = int(getattr(lane, "index", 0)) - 1
        except (TypeError, ValueError):
            return None

        return index if index >= 0 else None

    def _format_openams_calibration_command(self, base_command, lane):
        oams_index = self._get_openams_index()
        spool_index = self._get_openams_spool_index(lane)

        if oams_index is None or spool_index is None:
            lane_name = getattr(lane, "name", lane)
            self.logger.warning(
                "Unable to format OpenAMS calibration command for lane %s on unit %s",
                lane_name,
                self.name,
            )
            return None

        return f"{base_command} OAMS={oams_index} SPOOL={spool_index}"

    def _format_hub_hes_calibration_command(self, lane):
        return self._format_openams_calibration_command("OAMS_CALIBRATE_HUB_HES", lane)

    def _format_ptfe_length_calibration_command(self, lane):
        command = self._format_openams_calibration_command(
            "AFC_OAMS_CALIBRATE_PTFE", lane
        )
        return command

    def _values_look_binary(self, values):
        if not values:
            return False
        tolerance = 1e-3
        for value in values:
            if value is None:
                continue
            if abs(value) <= tolerance or abs(value - 1.0) <= tolerance:
                continue
            return False
        return True

    def _normalize_numeric_sequence(self, raw_values):
        if raw_values is None:
            return None
        if isinstance(raw_values, (int, float)):
            return (float(raw_values),)
        if isinstance(raw_values, str):
            candidate = raw_values.strip()
            if not candidate:
                return None
            if "," in candidate:
                parts = [part.strip() for part in candidate.split(",")]
                numbers = []
                for part in parts:
                    if not part:
                        continue
                    try:
                        numbers.append(float(part))
                    except (TypeError, ValueError):
                        try:
                            match_numbers = re.findall(r"[-+]?[0-9]*\.?[0-9]+", part)
                        except re.error:
                            match_numbers = []
                        if not match_numbers:
                            return None
                        for item in match_numbers:
                            try:
                                numbers.append(float(item))
                            except (TypeError, ValueError):
                                return None
                return tuple(numbers) if numbers else None
            try:
                return (float(candidate),)
            except (TypeError, ValueError):
                try:
                    match_numbers = re.findall(r"[-+]?[0-9]*\.?[0-9]+", candidate)
                except re.error:
                    match_numbers = []
                if not match_numbers:
                    return None
                numbers = []
                for item in match_numbers:
                    try:
                        numbers.append(float(item))
                    except (TypeError, ValueError):
                        return None
                return tuple(numbers) if numbers else None
        if isinstance(raw_values, (list, tuple)):
            numbers = []
            for item in raw_values:
                normalized = self._normalize_numeric_sequence(item)
                if not normalized:
                    continue
                numbers.extend(normalized)
            return tuple(numbers) if numbers else None
        if isinstance(raw_values, dict):
            items = []
            numeric_keys = True
            keys = list(raw_values.keys())
            for key in keys:
                try:
                    int(key)
                except (TypeError, ValueError):
                    numeric_keys = False
                    break
            if numeric_keys:
                for key in sorted(keys, key=lambda item: int(item)):
                    items.append(raw_values[key])
            else:
                items.extend(raw_values.values())
            numbers = []
            for item in items:
                normalized = self._normalize_numeric_sequence(item)
                if not normalized:
                    continue
                numbers.extend(normalized)
            return tuple(numbers) if numbers else None
        return None

    def _extract_hub_hes_calibration_values(self, status):
        candidates = []
        if status:
            for key in ("hub_hes_on", "hub_hes_value", "hub_hes_values"):
                value = status.get(key)
                if value is not None:
                    candidates.append(value)
            calibration = status.get("calibration")
            if isinstance(calibration, dict):
                for key in ("hub_hes_on", "hub_hes_value", "hub_hes_values"):
                    value = calibration.get(key)
                    if value is not None:
                        candidates.append(value)

        controller = None
        if self.hardware_service is not None:
            try:
                controller = self.hardware_service.resolve_controller()
            except Exception:
                controller = None
        if controller is None:
            controller = self.oams

        if controller is not None:
            for attr in ("hub_hes_on", "hub_hes_value", "hub_hes_values"):
                try:
                    value = getattr(controller, attr)
                except Exception:
                    value = None
                if value is not None:
                    candidates.append(value)

        for candidate in candidates:
            normalized = self._normalize_numeric_sequence(candidate)
            if not normalized:
                continue
            if self._values_look_binary(normalized):
                continue
            return normalized
        return None

    def _collect_status_values_for_keys(self, status, keys):
        if not status:
            return []

        stack = [status]
        seen = set()
        collected = []
        target_keys = {key.lower() for key in keys if isinstance(key, str)}
        while stack:
            current = stack.pop()
            ident = id(current)
            if ident in seen:
                continue
            seen.add(ident)

            if isinstance(current, dict):
                alias = None
                for key, value in current.items():
                    key_name = key.lower() if isinstance(key, str) else key
                    if key_name in target_keys:
                        collected.append(value)
                    if isinstance(value, (dict, list, tuple, set)):
                        stack.append(value)
                    if (
                        alias is None
                        and isinstance(value, str)
                        and value.lower() in target_keys
                    ):
                        alias = value
                if alias is not None:
                    for candidate_key in ("value", "values", "data", "result", "results"):
                        candidate = current.get(candidate_key)
                        if candidate is not None:
                            collected.append(candidate)
            elif isinstance(current, (list, tuple, set)):
                for item in current:
                    if isinstance(item, (dict, list, tuple, set)):
                        stack.append(item)

        return collected

    def _extract_ptfe_calibration_values(self, status):
        candidates = []
        ptfe_keys = (
            "ptfe_length",
            "ptfe_lengths",
            "ptfe_length_value",
            "ptfe_length_values",
            "ptfe_length_mm",
            "ptfe_lengths_mm",
            "bowden_length",
            "bowden_lengths",
        )

        candidates.extend(self._collect_status_values_for_keys(status, ptfe_keys))

        controller = None
        if self.hardware_service is not None:
            try:
                controller = self.hardware_service.resolve_controller()
            except Exception:
                controller = None
        if controller is None:
            controller = self.oams

        if controller is not None:
            for attr in ptfe_keys:
                try:
                    value = getattr(controller, attr)
                except Exception:
                    value = None
                if value is not None:
                    candidates.append(value)

        for candidate in candidates:
            normalized = self._normalize_numeric_sequence(candidate)
            if not normalized:
                continue
            if self._values_look_binary(normalized):
                continue
            return normalized
        return None

    def _extract_ptfe_length_from_messages(self, messages: List[str]) -> Optional[List[float]]:
        if not messages:
            return None

        pattern = re.compile(
            r"(?:ptfe|bowden)[^0-9\-]*(-?[0-9]+(?:\.[0-9]+)?)",
            re.IGNORECASE,
        )

        extracted: List[float] = []
        for message in messages:
            if not isinstance(message, str):
                continue
            for match in pattern.finditer(message):
                try:
                    extracted.append(float(match.group(1)))
                except (TypeError, ValueError):
                    continue

        return extracted or None

    def _format_numeric_values(self, values):
        if not values:
            return None
        if not isinstance(values, (list, tuple)):
            values = (values,)
        formatted = []
        for value in values:
            if value is None:
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if abs(number - round(number)) <= 1e-6:
                formatted.append(str(int(round(number))))
            else:
                formatted.append(f"{number:.6f}".rstrip("0").rstrip("."))
        return ", ".join(formatted) if formatted else None

    def _persist_lane_ptfe_length(self, lane, new_length) -> bool:
        """Persist a calibrated PTFE length to the lane's hub configuration."""

        if lane is None:
            return False

        hub = getattr(lane, "hub_obj", None)
        if hub is None:
            return False

        try:
            numeric_length = float(new_length)
        except (TypeError, ValueError):
            return False

        if not math.isfinite(numeric_length) or numeric_length <= 0.0:
            return False

        lane_key = getattr(lane, "name", None) or str(lane)
        tolerance = 1e-3
        last_saved = self._last_lane_ptfe_lengths.get(lane_key)
        if last_saved is not None and abs(last_saved - numeric_length) <= tolerance:
            return False

        previous_unload = getattr(hub, "afc_unload_bowden_length", None)
        previous_bowden = getattr(hub, "afc_bowden_length", None)

        try:
            hub.afc_unload_bowden_length = numeric_length
            if hasattr(hub, "afc_bowden_length"):
                hub.afc_bowden_length = numeric_length
        except Exception:
            self.logger.exception(
                "Failed to update hub bowden lengths for lane %s", lane_key
            )

        hub_fullname = getattr(hub, "fullname", None)
        afc_function = getattr(self.afc, "function", None)
        config_writer = getattr(afc_function, "ConfigRewrite", None)

        formatted_new = self._format_numeric_values([numeric_length]) or str(
            numeric_length
        )
        unload_msg = f"\n afc_unload_bowden_length: New: {formatted_new}"
        if previous_unload is not None:
            formatted_previous_unload = self._format_numeric_values(
                [previous_unload]
            )
            if formatted_previous_unload:
                unload_msg += f" Old: {formatted_previous_unload}"

        bowden_msg = f"\n afc_bowden_length: New: {formatted_new}"
        if previous_bowden is not None:
            formatted_previous_bowden = self._format_numeric_values([previous_bowden])
            if formatted_previous_bowden:
                bowden_msg += f" Old: {formatted_previous_bowden}"

        persisted = False
        if callable(config_writer) and hub_fullname:
            try:
                config_writer(hub_fullname, "afc_bowden_length", numeric_length, bowden_msg)
                config_writer(
                    hub_fullname,
                    "afc_unload_bowden_length",
                    numeric_length,
                    unload_msg,
                )
                persisted = True
            except Exception:
                self.logger.exception(
                    "Failed to persist PTFE length for lane %s", lane_key
                )

        if persisted:
            self._last_lane_ptfe_lengths[lane_key] = numeric_length

        return persisted

    def _rewrite_oams_config_file(
        self, section_name: str, key: str, value: str, msg: str
    ) -> bool:
        config_dir = getattr(self.afc, "cfgloc", None)
        if not config_dir:
            return False

        try:
            config_files = sorted(
                os.path.join(config_dir, filename)
                for filename in os.listdir(config_dir)
                if filename.lower().endswith(".cfg")
            )
        except Exception:
            return False

        header = f"[{section_name}]".strip().lower()
        key_pattern = re.compile(rf"\s*{re.escape(key)}\s*:")

        for config_path in config_files:
            if not os.path.isfile(config_path):
                continue

            try:
                with open(config_path, "r", encoding="utf-8") as cfg_file:
                    lines = cfg_file.readlines()
            except Exception:
                continue

            in_section = False
            updated = False
            output_lines = []

            for raw_line in lines:
                line = raw_line.rstrip("\n")
                stripped = line.strip()

                if stripped.startswith("[") and stripped.endswith("]"):
                    if in_section and not updated:
                        output_lines.append(f"{key}:{value}")
                        updated = True
                    in_section = stripped.lower() == header
                    output_lines.append(line)
                    continue

                if in_section and key_pattern.match(line):
                    comment = ""
                    if "#" in line:
                        comment_index = line.index("#")
                        comment = line[comment_index:].strip()
                    new_line = f"{key}:{value}"
                    if comment:
                        new_line = f"{new_line} {comment}"
                    output_lines.append(new_line)
                    updated = True
                    continue

                output_lines.append(line)

            if in_section and not updated:
                output_lines.append(f"{key}:{value}")
                updated = True

            if not updated:
                continue

            try:
                with open(config_path, "w", encoding="utf-8") as cfg_file:
                    cfg_file.write("\n".join(output_lines) + "\n")
            except Exception:
                continue

            self.logger.info(msg)
            return True

        return False

    def _write_openams_config_value(self, key, value, previous_string=None):
        if not value:
            return
        section_name = f"oams {self.oams_name}" if self.oams_name else None
        afc_function = getattr(self.afc, "function", None)
        if not section_name or afc_function is None:
            return

        msg = f"\n{self.name} {key}: New: {value}"
        if previous_string:
            msg += f" Old: {previous_string}"

        if self._rewrite_oams_config_file(section_name, key, value, msg):
            return

        config_writer = getattr(afc_function, "ConfigRewrite", None)
        if not callable(config_writer):
            return

        try:
            config_writer(section_name, key, value, msg)
        except Exception:
            self.logger.exception(
                "Failed to persist %s to oams config for unit %s", key, self.name
            )

    def _persist_openams_calibration_results(self, status):
        hub_values = self._extract_hub_hes_calibration_values(status)
        if hub_values:
            formatted = self._format_numeric_values(hub_values)
            if formatted and formatted != self._last_hub_hes_string:
                self._write_openams_config_value(
                    "hub_hes_on", formatted, previous_string=self._last_hub_hes_string
                )
            if formatted:
                self._last_hub_hes_string = formatted

        ptfe_values = self._extract_ptfe_calibration_values(status)
        if ptfe_values:
            formatted = self._format_numeric_values(ptfe_values)
            if formatted and formatted != self._last_ptfe_string:
                self._write_openams_config_value(
                    "ptfe_length", formatted, previous_string=self._last_ptfe_string
                )
            if formatted:
                self._last_ptfe_string = formatted
            for lane in self.lanes.values():
                spool_index = self._get_openams_spool_index(lane)
                if spool_index is None:
                    continue
                try:
                    lane_value = ptfe_values[spool_index]
                except (IndexError, TypeError):
                    continue
                if lane_value is None:
                    continue
                self._persist_lane_ptfe_length(lane, lane_value)

    cmd_UNIT_BOW_CALIBRATION_help = (
        "open prompt to calibrate OpenAMS PTFE lengths"
    )

    def cmd_UNIT_BOW_CALIBRATION(self, gcmd):
        """Prompt for calibrating PTFE length for OpenAMS lanes."""

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"OAMS PTFE Calibration {self.name}"
        text = (
            "Select a loaded lane from {} to calibrate PTFE length using OpenAMS. "
            "Results will be saved to oamsc.cfg when values change. "
            "Command: OAMS_CALIBRATE_PTFE_LENGTH"
        ).format(self.name)

        for lane in self.lanes.values():
            if not lane.load_state:
                continue

            command = self._format_ptfe_length_calibration_command(lane)
            if not command:
                continue

            button_label = "{}".format(lane)
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, command, button_style))

            index += 1
            if index % 4 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        if index == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", "UNIT_CALIBRATION UNIT={}".format(self.name), "info")]

        prompt.create_custom_p(title, text, None, True, buttons, back)

    def handle_connect(self):
        """Initialise the AMS unit and configure custom logos."""
        super().handle_connect()

        self._ensure_virtual_tool_sensor()

        # AMS lanes report their state via OpenAMS so default them until the
        # first poll comes back.
        for lane in self.lanes.values():
            lane.prep_state = False
            lane.load_state = False
            lane.status = AFCLaneState.NONE
            lane.ams_share_prep_load = getattr(lane, "load", None) is None

        first_leg = (
            "<span class=warning--text>|</span>"
            "<span class=error--text>_</span>"
        )
        second_leg = f"{first_leg}<span class=warning--text>|</span>"
        self.logo = dedent(
            """\
            <span class=success--text>R  _____     ____
            E /      \\  |  </span><span class=info--text>o</span><span class=success--text> |
            A |       |/ ___/
            D |_________/
            Y {first}{second} {first}{second}
              {name}
            </span>
            """
        ).format(first=first_leg, second=second_leg, name=self.name)

        self.logo_error = dedent(
            """\
            <span class=error--text>E  _ _   _ _
            R |_|_|_|_|_|
            R |         \\____
            O |              \\
            R |          |\\ <span class=secondary--text>X</span> |
            ! \\_________/ |___|
              {name}
            </span>
            """
        ).format(name=self.name)

    def _ensure_virtual_tool_sensor(self) -> bool:
        """Resolve or create the virtual tool-start sensor for AMS extruders."""

        if self._virtual_tool_sensor is not None:
            return True

        extruder = getattr(self, "extruder_obj", None)
        if extruder is None:
            return False

        tool_pin = getattr(extruder, "tool_start", None)
        normalized = _normalize_ams_pin_value(tool_pin)
        if normalized is None:
            normalized = getattr(extruder, "_ams_virtual_tool_name", None)

        if not normalized or normalized.lower() in {"buffer", "none", "unknown"}:
            return False

        original_pin = tool_pin
        if not normalized.upper().startswith("AMS_"):
            return False

        sensor = getattr(extruder, "fila_tool_start", None)
        if sensor is None:
            sensor = self.printer.lookup_object(
                f"filament_switch_sensor {normalized}", None
            )

        if sensor is None:
            pins = self.printer.lookup_object("pins")
            if not getattr(self.afc, "_virtual_ams_chip_registered", False):
                try:
                    pins.register_chip("afc_virtual_ams", self.afc)
                except Exception:
                    return False
                else:
                    self.afc._virtual_ams_chip_registered = True

            enable_gui = getattr(extruder, "enable_sensors_in_gui", True)
            runout_cb = getattr(extruder, "handle_start_runout", None)
            enable_runout = getattr(extruder, "enable_runout", False)
            debounce = getattr(extruder, "debounce_delay", 0.0)

            try:
                created = add_filament_switch(
                    normalized,
                    f"afc_virtual_ams:{normalized}",
                    self.printer,
                    enable_gui,
                    runout_cb,
                    enable_runout,
                    debounce,
                )
            except TypeError:
                try:
                    created = add_filament_switch(
                        normalized,
                        f"afc_virtual_ams:{normalized}",
                        self.printer,
                        enable_gui,
                    )
                except Exception:
                    return False
            except Exception:
                return False

            sensor = created[0] if isinstance(created, tuple) else created

        helper = getattr(sensor, "runout_helper", None)
        if helper is None:
            return False

        # Ensure OpenAMS owns the runout handling by disabling any
        # automatically wired callbacks on the virtual sensor helper.
        helper.runout_callback = None
        helper.sensor_enabled = False

        filament_present = getattr(helper, "filament_present", None)
        if filament_present is not None:
            self._last_virtual_tool_state = bool(filament_present)

        if getattr(extruder, "fila_tool_start", None) is None:
            extruder.fila_tool_start = sensor

        extruder.tool_start = original_pin
        self._virtual_tool_sensor = sensor

        # Maintain legacy filament sensor aliases so existing macros keep
        # functioning even though the physical switch has been replaced by an
        # OpenAMS-provided virtual sensor.
        alias_token = None
        try:
            alias_token = f"{extruder.name}_tool_start"
        except Exception:
            alias_token = None

        if alias_token:
            alias_object = f"filament_switch_sensor {alias_token}"
            objects = getattr(self.printer, "objects", None)
            if isinstance(objects, dict):
                previous = objects.get(alias_object)
                if previous is None or previous is sensor:
                    objects[alias_object] = sensor

            try:
                gcode = self.printer.lookup_object("gcode")
            except Exception:
                gcode = None

            if gcode is not None:
                for command, handler, desc in (
                    (
                        "QUERY_FILAMENT_SENSOR",
                        sensor.cmd_QUERY_FILAMENT_SENSOR,
                        sensor.QUERY_HELP,
                    ),
                    (
                        "SET_FILAMENT_SENSOR",
                        sensor.cmd_SET_FILAMENT_SENSOR,
                        sensor.SET_HELP,
                    ),
                ):
                    try:
                        gcode.register_mux_command(
                            command,
                            "SENSOR",
                            alias_token,
                            handler,
                            desc=desc,
                        )
                    except Exception:
                        pass

        return True

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

        normalized_lane = _normalize_extruder_name(lane_extruder)
        normalized_unit = _normalize_extruder_name(extruder_name)

        if normalized_lane and normalized_unit and normalized_lane == normalized_unit:
            return True

        return False

    def _lane_reports_tool_filament(self, lane) -> Optional[bool]:
        """Return the best-known tool filament state for a lane."""

        if lane is None:
            return None

        load_state = getattr(lane, "load_state", None)
        if load_state is not None:
            return bool(load_state)

        if getattr(lane, "tool_loaded", False):
            return True

        return None

    def _set_virtual_tool_sensor_state(
        self,
        filament_present: bool,
        eventtime: float,
        lane_name: Optional[str] = None,
        *,
        force: bool = False,
        lane_obj=None,
    ) -> None:
        """Update the cached virtual sensor and extruder state."""

        if not self._ensure_virtual_tool_sensor():
            return

        new_state = bool(filament_present)

        canonical_lane = self._canonical_lane_name(lane_name)
        if canonical_lane is None and lane_obj is not None:
            canonical_lane = self._canonical_lane_name(getattr(lane_obj, "name", None))

        if new_state and not force:
            if canonical_lane and self._lane_tool_latches.get(canonical_lane) is False:
                return

        sensor = self._virtual_tool_sensor
        helper = getattr(sensor, "runout_helper", None)
        if helper is None:
            return

        try:
            helper.note_filament_present(eventtime, filament_present)
        except TypeError:
            helper.note_filament_present(is_filament_present=filament_present)

        setattr(sensor, "filament_present", filament_present)

        extruder = getattr(self, "extruder_obj", None)
        if extruder is not None:
            extruder.tool_start_state = filament_present

        self._last_virtual_tool_state = new_state

        if canonical_lane:
            self._lane_tool_latches[canonical_lane] = new_state
            self._lane_feed_activity[canonical_lane] = new_state

    def lane_tool_loaded(self, lane):
        """Update the virtual tool sensor when a lane loads into the tool."""

        super().lane_tool_loaded(lane)

        if not self._lane_matches_extruder(lane):
            return

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(
            True, eventtime, lane_name, force=True, lane_obj=lane
        )

    def lane_tool_unloaded(self, lane):
        """Update the virtual tool sensor when a lane unloads from the tool."""

        super().lane_tool_unloaded(lane)

        if not self._lane_matches_extruder(lane):
            return

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(
            False, eventtime, lane_name, lane_obj=lane
        )

    def _mirror_lane_to_virtual_sensor(self, lane, eventtime: float) -> None:
        """Mirror a lane's load state into the AMS virtual tool sensor."""

        if not self._lane_matches_extruder(lane):
            return

        desired_state = self._lane_reports_tool_filament(lane)
        if desired_state is None:
            return

        if desired_state == self._last_virtual_tool_state:
            return

        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(
            desired_state, eventtime, lane_name, lane_obj=lane
        )

    def _sync_virtual_tool_sensor(
        self, eventtime: float, lane_name: Optional[str] = None
    ) -> None:
        """Align the AMS virtual tool sensor with the mapped lane state."""

        if not self._ensure_virtual_tool_sensor():
            return

        desired_state: Optional[bool] = None
        desired_lane: Optional[str] = None
        desired_lane_obj = None

        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is not None and self._lane_matches_extruder(lane):
                result = self._lane_reports_tool_filament(lane)
                if result is not None:
                    desired_state = result
                    desired_lane = getattr(lane, "name", None)
                    desired_lane_obj = lane

        if desired_state is None:
            pending_false = None
            for lane in self.lanes.values():
                if not self._lane_matches_extruder(lane):
                    continue

                result = self._lane_reports_tool_filament(lane)
                if result is None:
                    continue

                lane_id = getattr(lane, "name", None)
                if result:
                    desired_state = True
                    desired_lane = lane_id
                    desired_lane_obj = lane
                    break

                if pending_false is None:
                    pending_false = (False, lane_id, lane)

            if desired_state is None and pending_false is not None:
                desired_state, desired_lane, desired_lane_obj = pending_false

        if desired_state is None or desired_state == self._last_virtual_tool_state:
            return

        self._set_virtual_tool_sensor_state(
            desired_state, eventtime, desired_lane, lane_obj=desired_lane_obj
        )

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

        # Accept config-style names that include the unit type prefix.
        parts = normalized.replace("_", " ").replace("-", " ").split()
        return any(part.lower() == self.name.lower() for part in parts)

    def _normalize_group_name(self, group: Optional[str]) -> Optional[str]:
        """Return a trimmed filament group token for alias comparison."""

        if not group or not isinstance(group, str):
            return None

        normalized = group.strip()
        if not normalized:
            return None

        # Filament group identifiers are often prefixed (e.g. "Group T4"), so
        # prefer the final token which corresponds to the configured AFC map.
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
        normalized_lookup = self._normalize_group_name(lookup)
        for lane in self.lanes.values():
            if lane.name.lower() == lowered:
                return lane.name

            lane_map = getattr(lane, "map", None)
            if isinstance(lane_map, str) and lane_map.lower() == lowered:
                return lane.name

            canonical_map = self._normalize_group_name(lane_map)
            if (
                canonical_map is not None
                and normalized_lookup is not None
                and canonical_map.lower() == normalized_lookup.lower()
            ):
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

    cmd_SYNC_TOOL_SENSOR_help = (
        "Synchronise the AMS virtual tool-start sensor with the assigned lane."
    )
    def cmd_SYNC_TOOL_SENSOR(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            lane_name = gcmd.get("FPS", None)

        lane_name = self._resolve_lane_alias(lane_name)
        eventtime = self.reactor.monotonic()
        self._sync_virtual_tool_sensor(eventtime, lane_name)

    # ------------------------------------------------------------------
    # Global sync command dispatching
    # ------------------------------------------------------------------

    def _register_sync_dispatcher(self) -> None:
        """Ensure the shared sync command is available for all AMS units."""

        cls = self.__class__
        if not cls._sync_command_registered:
            self.gcode.register_command(
                "AFC_AMS_SYNC_TOOL_SENSOR",
                cls._dispatch_sync_tool_sensor,
                desc=self.cmd_SYNC_TOOL_SENSOR_help,
            )
            cls._sync_command_registered = True

        cls._sync_instances[self.name] = self

    def _register_ptfe_calibration_dispatcher(self) -> None:
        """Register the shared PTFE calibration wrapper command."""

        cls = self.__class__
        if not cls._ptfe_command_registered:
            self.gcode.register_command(
                "AFC_OAMS_CALIBRATE_PTFE",
                cls._dispatch_ptfe_calibration_command,
                desc="Run OAMS_CALIBRATE_PTFE_LENGTH and persist results",
            )
            cls._ptfe_command_registered = True

        index = self._get_openams_index()
        if index is not None:
            cls._ptfe_instances[index] = self

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

        if value[0] in "'\"" and value[-1] == value[0]:
            value = value[1:-1]

        return value

    @classmethod
    def _dispatch_ptfe_calibration_command(cls, gcmd):
        """Route PTFE calibration requests to the matching AMS unit."""

        try:
            oams_index = gcmd.get_int("OAMS")
        except Exception:
            gcmd.respond_info("AFC_OAMS_CALIBRATE_PTFE requires an OAMS parameter")
            return

        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            gcmd.respond_info(
                "AFC_OAMS_CALIBRATE_PTFE requires a SPOOL parameter between 0-3"
            )
            return

        instance = cls._ptfe_instances.get(oams_index)
        if instance is None:
            gcmd.respond_info(
                f"No OpenAMS unit is registered for OAMS index {oams_index}"
            )
            return

        instance._execute_ptfe_calibration_command(gcmd, oams_index, spool_index)

    def _run_command_with_capture(self, command: str) -> List[str]:
        """Execute a command while capturing respond_info messages."""

        captured: List[str] = []
        respond_info = getattr(self.gcode, "respond_info", None)
        if not callable(respond_info):
            self.gcode.run_script_from_command(command)
            return captured

        original_attribute = self.gcode.__dict__.get("respond_info", _MISSING)

        def _capture_messages(this, message, *args, **kwargs):
            if isinstance(message, str):
                captured.append(message)
            return respond_info(message, *args, **kwargs)

        self.gcode.respond_info = MethodType(_capture_messages, self.gcode)

        try:
            self.gcode.run_script_from_command(command)
        finally:
            if original_attribute is _MISSING:
                self.gcode.__dict__.pop("respond_info", None)
            else:
                self.gcode.respond_info = original_attribute

        return captured

    def _execute_ptfe_calibration_command(self, gcmd, oams_index: int, spool_index: int):
        """Run the PTFE calibration macro then persist the reported value."""

        lane = self._lane_for_spool_index(spool_index)
        lane_name = getattr(lane, "name", None)
        raw_command = f"OAMS_CALIBRATE_PTFE_LENGTH OAMS={oams_index} SPOOL={spool_index}"
        captured_messages = self._run_command_with_capture(raw_command)
        captured_values = self._extract_ptfe_length_from_messages(captured_messages)
        lane_length_value = None
        lane_persisted = False
        if lane is not None and captured_values:
            if 0 <= spool_index < len(captured_values):
                lane_length_value = captured_values[spool_index]
            elif len(captured_values) == 1:
                lane_length_value = captured_values[0]
            if lane_length_value is not None:
                lane_persisted = self._persist_lane_ptfe_length(lane, lane_length_value)

        formatted_capture = None
        if captured_values:
            formatted_capture = self._format_numeric_values(captured_values)

        if formatted_capture:
            if formatted_capture != self._last_ptfe_string:
                self._write_openams_config_value(
                    "ptfe_length",
                    formatted_capture,
                    previous_string=self._last_ptfe_string,
                )
            self._last_ptfe_string = formatted_capture
            lane_suffix = f" for {lane_name}" if lane_name else ""
            gcmd.respond_info(
                f"Stored OpenAMS PTFE length {formatted_capture}{lane_suffix} in oamsc.cfg."
            )
            if lane_persisted and lane_length_value is not None:
                hub_name = getattr(getattr(lane, "hub_obj", None), "name", None)
                target_name = hub_name or lane_name or "hub"
                formatted_lane = self._format_numeric_values([lane_length_value]) or str(
                    lane_length_value
                )
                gcmd.respond_info(
                    f"Saved bowden lengths of {formatted_lane}mm for {target_name} in configuration."
                )
            return

        if self._last_ptfe_string:
            gcmd.respond_info(
                "Completed {}. Latest known PTFE length: {}.".format(
                    raw_command, self._last_ptfe_string
                )
            )
        else:
            gcmd.respond_info(
                "Completed {} but no PTFE length was reported. Check OpenAMS status logs.".format(
                    raw_command
                )
            )

        if lane_persisted and lane_length_value is not None and not formatted_capture:
            hub_name = getattr(getattr(lane, "hub_obj", None), "name", None)
            target_name = hub_name or lane_name or "hub"
            formatted_lane = self._format_numeric_values([lane_length_value]) or str(
                lane_length_value
            )
            gcmd.respond_info(
                f"Saved bowden lengths of {formatted_lane}mm for {target_name} in configuration."
            )

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
            instance._sync_virtual_tool_sensor(eventtime, resolved_lane)

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate AMS lane state without attempting any motion."""

        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
            else:
                self.afc.function.afc_led(cur_lane.led_fault, cur_lane.led_index)
                msg += '<span class=error--text> NOT READY</span>'
                cur_lane.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False
        else:
            self.afc.function.afc_led(cur_lane.led_ready, cur_lane.led_index)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.afc.function.afc_led(cur_lane.led_spool_illum, cur_lane.led_spool_index)

                if cur_lane.tool_loaded:
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                    )
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        msg += '<span class=primary--text> in ToolHead</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += (
                                '<span class=warning--text>'
                                ' Ram sensor enabled, confirm tool is loaded</span>'
                            )
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        cur_lane.enable_buffer()
                    elif tool_ready:
                        msg += (
                            '<span class=error--text> error in ToolHead. '
                            'Lane identified as loaded '
                            'but not identified as loaded in extruder</span>'
                        )
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info(
            '{lane_name} tool cmd: {tcmd:3} {msg}'.format(
                lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg
            )
        )
        cur_lane.set_afc_prep_done()
        return succeeded

    def handle_ready(self):
        """Resolve the OpenAMS object once Klippy is ready."""

        if self.hardware_service is not None:
            self.oams = self.hardware_service.resolve_controller()
        else:
            self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
            if AMSHardwareService is not None and self.oams is not None:
                try:
                    self.hardware_service = AMSHardwareService.for_printer(
                        self.printer, self.oams_name, self.logger
                    )
                    self.hardware_service.attach_controller(self.oams)
                except Exception:
                    self.logger.exception(
                        "Failed to attach AMSHardwareService for %s", self.oams_name
                    )
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _update_shared_lane(self, lane, lane_val, eventtime):
        """Synchronise shared prep/load sensor lanes without triggering errors."""

        if lane_val == self._last_lane_states.get(lane.name):
            return

        if lane_val:
            lane.load_state = False
            try:
                lane.prep_callback(eventtime, True)
            finally:
                lane.load_callback(eventtime, True)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            if (
                lane.prep_state
                and lane.load_state
                and lane.printer.state_message == "Printer is ready"
                and getattr(lane, "_afc_prep_done", False)
            ):
                lane.status = AFCLaneState.LOADED
                lane.unit_obj.lane_loaded(lane)
                lane.afc.spool._set_values(lane)
                lane._prep_capture_td1()
        else:
            lane.load_callback(eventtime, False)
            lane.prep_callback(eventtime, False)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            lane.tool_loaded = False
            lane.loaded_to_hub = False
            lane.status = AFCLaneState.NONE
            lane.unit_obj.lane_unloaded(lane)
            lane.td1_data = {}
            lane.afc.spool.clear_values(lane)

        lane.afc.save_vars()
        self._last_lane_states[lane.name] = lane_val

    def _apply_lane_sensor_state(self, lane, lane_val, eventtime):
        """Apply a boolean lane sensor value using existing AFC callbacks."""

        try:
            share = getattr(lane, "ams_share_prep_load", False)
        except Exception:
            share = False

        if share:
            self._update_shared_lane(lane, lane_val, eventtime)
            return

        previous = self._last_lane_states.get(getattr(lane, "name", ""))

        if previous is not None and bool(previous) == bool(lane_val):
            return

        try:
            lane.load_callback(eventtime, lane_val)
        except TypeError:
            lane.load_callback(eventtime, load_state=lane_val)
        except Exception:
            self.logger.exception("Failed to update load sensor for lane %s", lane)

        try:
            lane.prep_callback(eventtime, lane_val)
        except TypeError:
            lane.prep_callback(eventtime, prep_state=lane_val)
        except Exception:
            self.logger.exception("Failed to update prep sensor for lane %s", lane)

        self._mirror_lane_to_virtual_sensor(lane, eventtime)
        lane_name = getattr(lane, "name", None)
        if lane_name:
            self._last_lane_states[lane_name] = bool(lane_val)

    def _sync_event(self, eventtime):
        """Poll OpenAMS for state updates and propagate to lanes/hubs."""

        try:
            status = None
            if self.hardware_service is not None:
                status = self.hardware_service.poll_status()
                if status is None:
                    self.oams = self.hardware_service.resolve_controller()
            elif self.oams is not None:
                status = {
                    "encoder_clicks": getattr(self.oams, "encoder_clicks", None),
                    "f1s_hes_value": getattr(self.oams, "f1s_hes_value", None),
                    "hub_hes_value": getattr(self.oams, "hub_hes_value", None),
                }

            if not status:
                return eventtime + self.interval

            try:
                self._persist_openams_calibration_results(status)
            except Exception:
                self.logger.exception(
                    "Failed to persist OpenAMS calibration data for %s", self.name
                )

            encoder_clicks = status.get("encoder_clicks")
            try:
                encoder_clicks = int(encoder_clicks)
            except Exception:
                encoder_clicks = None

            lane_values = status.get("f1s_hes_value")
            hub_values = status.get("hub_hes_value")

            active_lane_name = None
            if encoder_clicks is not None:
                last_clicks = self._last_encoder_clicks
                if last_clicks is not None and encoder_clicks != last_clicks:
                    current_loading = getattr(self.afc, "current_loading", None)
                    if current_loading:
                        lane = self.lanes.get(current_loading)
                        if lane is not None and self._lane_matches_extruder(lane):
                            active_lane_name = getattr(lane, "name", None)
                    if active_lane_name is None:
                        for lane in self.lanes.values():
                            if self._lane_matches_extruder(lane) and getattr(lane, "status", None) == AFCLaneState.TOOL_LOADING:
                                active_lane_name = getattr(lane, "name", None)
                                break
                    if active_lane_name:
                        canonical_lane = self._canonical_lane_name(active_lane_name)
                        if canonical_lane:
                            self._lane_feed_activity[canonical_lane] = True
                self._last_encoder_clicks = encoder_clicks
            elif encoder_clicks is None:
                self._last_encoder_clicks = None

            for lane in list(self.lanes.values()):
                idx = getattr(lane, "index", 0) - 1
                if idx < 0:
                    continue

                if lane_values is None or idx >= len(lane_values):
                    continue

                lane_val = bool(lane_values[idx])
                if getattr(lane, "ams_share_prep_load", False):
                    self._update_shared_lane(lane, lane_val, eventtime)
                elif lane_val != self._last_lane_states.get(lane.name):
                    lane.load_callback(eventtime, lane_val)
                    lane.prep_callback(eventtime, lane_val)
                    self._mirror_lane_to_virtual_sensor(lane, eventtime)
                    self._last_lane_states[lane.name] = lane_val

                if self.hardware_service is not None:
                    hub_state = None
                    if hub_values is not None and idx < len(hub_values):
                        hub_state = bool(hub_values[idx])
                    tool_state = self._lane_reports_tool_filament(lane)
                    self.hardware_service.update_lane_snapshot(
                        self.oams_name,
                        lane.name,
                        lane_val,
                        hub_state,
                        eventtime,
                        spool_index=idx,
                        tool_state=tool_state,
                    )

                hub = getattr(lane, "hub_obj", None)
                if hub is None or hub_values is None or idx >= len(hub_values):
                    continue

                hub_val = bool(hub_values[idx])
                if hub_val != self._last_hub_states.get(hub.name):
                    hub.switch_pin_callback(eventtime, hub_val)
                    fila = getattr(hub, "fila", None)
                    if fila is not None:
                        fila.runout_helper.note_filament_present(eventtime, hub_val)
                    self._last_hub_states[hub.name] = hub_val
            self._sync_virtual_tool_sensor(eventtime)
        except Exception:
            # Avoid stopping the reactor loop if OpenAMS query fails.
            pass

        return eventtime + self.interval

    def _lane_for_spool_index(self, spool_index: Optional[int]):
        if spool_index is None:
            return None
        for lane in self.lanes.values():
            try:
                idx = int(getattr(lane, "index", 0)) - 1
            except Exception:
                idx = -1
            if idx == spool_index:
                return lane
        return None

    def _resolve_lane_reference(self, lane_name: Optional[str]):
        """Return a lane object by name (or alias), case-insensitively."""

        if not lane_name:
            return None

        # Allow callers to pass aliases such as FPS names or filament group
        # identifiers by normalising them back to the canonical lane name.
        resolved_name = self._resolve_lane_alias(lane_name)
        if resolved_name:
            lane = self.lanes.get(resolved_name)
            if lane is not None:
                return lane
        else:
            resolved_name = lane_name

        lane = self.lanes.get(resolved_name)
        if lane is not None:
            return lane

        lowered = resolved_name.lower()
        for candidate_name, candidate in self.lanes.items():
            if candidate_name.lower() == lowered:
                return candidate
        return None

    def handle_runout_detected(
        self,
        spool_index: Optional[int],
        monitor=None,
        *,
        lane_name: Optional[str] = None,
    ) -> None:
        """Handle runout notifications coming from OpenAMS monitors."""

        lane = None
        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is None:
                lowered = lane_name.lower()
                lane = next(
                    (
                        candidate
                        for name, candidate in self.lanes.items()
                        if name.lower() == lowered
                    ),
                    None,
                )
        if lane is None:
            lane = self._lane_for_spool_index(spool_index)
        if lane is None:
            return

        eventtime = self.reactor.monotonic()
        try:
            lane.handle_load_runout(eventtime, False)
        except TypeError:
            lane.handle_load_runout(eventtime, load_state=False)
        except AttributeError:
            self.logger.warning(
                "Lane %s is missing load runout handler for AMS integration", lane
            )

    def handle_openams_lane_tool_state(
        self,
        lane_name: str,
        loaded: bool,
        *,
        spool_index: Optional[int] = None,
        eventtime: Optional[float] = None,
    ) -> bool:
        """Update lane/tool state in response to OpenAMS hardware events."""

        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(
                "OpenAMS reported lane %s but AFC unit %s cannot resolve it",
                lane_name,
                self.name,
            )
            return False

        if eventtime is None:
            try:
                eventtime = self.reactor.monotonic()
            except Exception:
                eventtime = 0.0

        lane_state = bool(loaded)
        try:
            self._apply_lane_sensor_state(lane, lane_state, eventtime)
        except Exception:
            self.logger.exception(
                "Failed to mirror OpenAMS lane sensor state for %s", lane.name
            )

        if self.hardware_service is not None:
            hub_state = getattr(lane, "loaded_to_hub", None)
            tool_state = getattr(lane, "tool_loaded", None)
            mapped_spool = spool_index
            if mapped_spool is None:
                try:
                    mapped_spool = int(getattr(lane, "index", 0)) - 1
                except Exception:
                    mapped_spool = None
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    lane_state,
                    hub_state if hub_state is not None else None,
                    eventtime,
                    spool_index=mapped_spool,
                    tool_state=tool_state if tool_state is not None else lane_state,
                )
            except Exception:
                self.logger.exception(
                    "Failed to update shared lane snapshot for %s", lane.name
                )

        afc_function = getattr(self.afc, "function", None)

        if lane_state:
            if afc_function is not None:
                try:
                    afc_function.unset_lane_loaded()
                except Exception:
                    self.logger.exception("Failed to unset previously loaded lane")
            try:
                lane.set_loaded()
            except Exception:
                self.logger.exception("Failed to mark lane %s as loaded", lane.name)
            try:
                lane.sync_to_extruder()
            except Exception:
                self.logger.exception("Failed to sync lane %s to extruder", lane.name)
            if afc_function is not None:
                try:
                    afc_function.handle_activate_extruder()
                except Exception:
                    self.logger.exception(
                        "Failed to activate extruder after loading lane %s",
                        lane.name,
                    )
            try:
                self.afc.save_vars()
            except Exception:
                self.logger.exception("Failed to persist AFC state after lane load")
            try:
                self.select_lane(lane)
            except Exception:
                self.logger.debug(
                    "Unable to select lane %s during OpenAMS load", lane.name
                )
            if self._lane_matches_extruder(lane):
                try:
                    canonical_lane = self._canonical_lane_name(lane.name)
                    force_update = True
                    if canonical_lane:
                        force_update = (
                            self._lane_tool_latches.get(canonical_lane) is not False
                        )
                    self._set_virtual_tool_sensor_state(
                        True,
                        eventtime,
                        lane.name,
                        force=force_update,
                        lane_obj=lane,
                    )
                except Exception:
                    self.logger.exception(
                        "Failed to mirror tool sensor state for loaded lane %s",
                        lane.name,
                    )
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
            except Exception:
                self.logger.exception(
                    "Failed to unset currently loaded lane %s", lane.name
                )
            return True

        if getattr(lane, "tool_loaded", False):
            try:
                lane.unsync_to_extruder()
            except Exception:
                self.logger.exception(
                    "Failed to unsync lane %s from extruder", lane.name
                )
            try:
                lane.set_unloaded()
            except Exception:
                self.logger.exception("Failed to mark lane %s as unloaded", lane.name)
            try:
                self.afc.save_vars()
            except Exception:
                self.logger.exception(
                    "Failed to persist AFC state after unloading lane %s",
                    lane.name,
                )
        if self._lane_matches_extruder(lane):
            try:
                self._set_virtual_tool_sensor_state(
                    False, eventtime, lane.name, lane_obj=lane
                )
            except Exception:
                self.logger.exception(
                    "Failed to mirror tool sensor state for unloaded lane %s",
                    lane.name,
                )
        return True

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

def _patch_lane_pre_sensor_for_ams() -> None:
    """Patch AFCLane.get_toolhead_pre_sensor_state for AMS virtual sensors."""

    if _ORIGINAL_LANE_PRE_SENSOR is None:
        return

    if getattr(AFCLane, "_ams_pre_sensor_patched", False):
        return

    def _ams_get_toolhead_pre_sensor_state(self, *args, **kwargs):
        unit = getattr(self, "unit_obj", None)
        if not isinstance(unit, afcAMS):
            return _ORIGINAL_LANE_PRE_SENSOR(self, *args, **kwargs)

        reactor = getattr(unit, "reactor", None)
        eventtime = None
        if reactor is not None:
            try:
                eventtime = reactor.monotonic()
            except Exception:
                eventtime = None

        if eventtime is not None:
            try:
                unit._sync_event(eventtime)
            except Exception:
                pass
        else:
            eventtime = 0.0

        try:
            unit._sync_virtual_tool_sensor(eventtime, self.name)
        except Exception:
            pass

        result = _ORIGINAL_LANE_PRE_SENSOR(self, *args, **kwargs)

        if result:
            return bool(result)

        desired_state = unit._lane_reports_tool_filament(self)
        if desired_state is None:
            desired_state = False

        if desired_state:
            try:
                unit._set_virtual_tool_sensor_state(
                    desired_state,
                    eventtime,
                    getattr(self, "name", None),
                    lane_obj=self,
                )
            except Exception:
                pass
            return True

        return bool(result)

    AFCLane.get_toolhead_pre_sensor_state = _ams_get_toolhead_pre_sensor_state
    AFCLane._ams_pre_sensor_patched = True

def load_config_prefix(config):
    _patch_lane_pre_sensor_for_ams()
    _patch_extruder_for_virtual_ams()
    return afcAMS(config)

