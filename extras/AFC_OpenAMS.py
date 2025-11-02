# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AMS integration helpers for Armored Turtle AFC."""

from __future__ import annotations

import os
import re
import traceback
from textwrap import dedent
from types import MethodType
from typing import Dict, Optional, List
from collections.abc import Mapping

from configparser import Error as ConfigError
try: from extras.AFC_utils import ERROR_STR
except: raise ConfigError("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLane, AFCLaneState
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))
try: from extras.AFC_utils import add_filament_switch
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_utils", trace=traceback.format_exc()))
try: import extras.AFC_extruder as _afc_extruder_mod
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_extruder", trace=traceback.format_exc()))
try: from extras.AFC_respond import AFCprompt
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try:  # pragma: no cover - optional at config parse time
    from extras.openams_integration import AMSHardwareService, AMSRunoutCoordinator
except Exception:  # pragma: no cover - integration may be absent during import
    AMSHardwareService = None
    AMSRunoutCoordinator = None

SYNC_INTERVAL = 2.0
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
        self._last_hub_hes_values: Optional[List[float]] = None
        self._last_ptfe_value: Optional[float] = None
        self.oams = None
        self.hardware_service = None

        if AMSRunoutCoordinator is not None:
            self.hardware_service = AMSRunoutCoordinator.register_afc_unit(self)
        elif AMSHardwareService is not None:
            self.hardware_service = AMSHardwareService.for_printer(
                self.printer, self.oams_name, self.logger
            )

        self.gcode.register_mux_command(
            "AFC_OAMS_CALIBRATE_HUB_HES",
            "UNIT",
            self.name,
            self.cmd_AFC_OAMS_CALIBRATE_HUB_HES,
            desc="calibrate the OpenAMS HUB HES value for a specific lane",
        )
        self.gcode.register_mux_command(
            "AFC_OAMS_CALIBRATE_HUB_HES_ALL",
            "UNIT",
            self.name,
            self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL,
            desc="calibrate the OpenAMS HUB HES value for every loaded lane",
        )
        self.gcode.register_mux_command(
            "AFC_OAMS_CALIBRATE_PTFE",
            "UNIT",
            self.name,
            self.cmd_AFC_OAMS_CALIBRATE_PTFE,
            desc="calibrate the OpenAMS PTFE length for a specific lane",
        )

        self._register_sync_dispatcher()

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        """Prompt for calibrating HUB HES values via OpenAMS macros."""

        prompt = AFCprompt(gcmd, self.logger)
        title = f"{self.name} Lane Calibration"
        text = (
            "Select a loaded lane from {} to calibrate HUB HES using OpenAMS. "
            "Command: OAMS_CALIBRATE_HUB_HES"
        ).format(self.name)

        groups, count = self._build_openams_lane_button_groups(
            "OAMS_CALIBRATE_HUB_HES", "HUB HES calibration"
        )

        if count == 0:
            text = "No lanes are loaded, please load before calibration"
        elif count > 1:
            groups.insert(
                0,
                [
                    (
                        "All lanes",
                        f"AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={self.name}",
                        "default",
                    )
                ],
            )

        back = [("Back", "UNIT_CALIBRATION UNIT={}".format(self.name), "info")]

        prompt.create_custom_p(
            title,
            text,
            buttons=None,
            cancel=True,
            groups=groups if groups else None,
            footer_buttons=back,
        )

    def cmd_UNIT_BOW_CALIBRATION(self, gcmd):
        """Prompt for calibrating PTFE length for OpenAMS lanes."""

        prompt = AFCprompt(gcmd, self.logger)
        title = f"OAMS PTFE Calibration {self.name}"
        text = (
            "Select a loaded lane from {} to calibrate PTFE length using OpenAMS. "
            "Results will be saved to your cfg when values change. "
            "Command: OAMS_CALIBRATE_PTFE_LENGTH"
        ).format(self.name)

        groups, count = self._build_openams_lane_button_groups(
            "OAMS_CALIBRATE_PTFE_LENGTH", "PTFE calibration"
        )

        if count == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", "UNIT_CALIBRATION UNIT={}".format(self.name), "info")]

        prompt.create_custom_p(
            title,
            text,
            buttons=None,
            cancel=True,
            groups=groups if groups else None,
            footer_buttons=back,
        )

    def _format_openams_calibration_command(self, base_command, lane):
        if base_command not in {
            "OAMS_CALIBRATE_HUB_HES",
            "OAMS_CALIBRATE_PTFE_LENGTH",
        }:
            return super()._format_openams_calibration_command(base_command, lane)

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

        if base_command == "OAMS_CALIBRATE_HUB_HES":
            return f"AFC_OAMS_CALIBRATE_HUB_HES UNIT={self.name} SPOOL={spool_index}"

        return f"AFC_OAMS_CALIBRATE_PTFE UNIT={self.name} SPOOL={spool_index}"

    def _build_openams_lane_button_groups(
        self, base_command, context
    ) -> tuple[List[List[tuple[str, str, str]]], int]:
        lane_values = self._openams_lane_values(context)

        groups: List[List[tuple[str, str, str]]] = []
        current_group: List[tuple[str, str, str]] = []
        index = 0

        for lane in lane_values:
            try:
                if not getattr(lane, "load_state", False):
                    continue
            except Exception:
                continue

            command = self._format_openams_calibration_command(base_command, lane)
            if not command:
                continue

            button_label = "{}".format(lane)
            button_style = "primary" if index % 2 == 0 else "secondary"
            current_group.append((button_label, command, button_style))
            index += 1

            if index % 4 == 0:
                groups.append(current_group)
                current_group = []

        if current_group:
            groups.append(current_group)

        return groups, index

    def _openams_lane_values(self, context) -> List[AFCLane]:
        lane_container = getattr(self, "lanes", None)
        if isinstance(lane_container, Mapping):
            return list(lane_container.values())
        if hasattr(lane_container, "values"):
            try:
                return list(lane_container.values())
            except Exception:
                self.logger.exception(
                    "Failed to enumerate OpenAMS lanes for %s prompt", context
                )
                return []
        if isinstance(lane_container, (list, tuple)):
            return list(lane_container)
        return []

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

            encoder_clicks = status.get("encoder_clicks")
            try:
                encoder_clicks = int(encoder_clicks)
            except Exception:
                encoder_clicks = None

            lane_values = status.get("f1s_hes_value")
            hub_values = status.get("hub_hes_value")
            ptfe_values = status.get("ptfe_length")

            if isinstance(hub_values, (list, tuple)):
                try:
                    parsed_hub_values = [float(value) for value in hub_values]
                except (TypeError, ValueError):
                    parsed_hub_values = None
                if parsed_hub_values:
                    self._last_hub_hes_values = parsed_hub_values

            new_ptfe_value = None
            if isinstance(ptfe_values, (list, tuple)):
                for entry in ptfe_values:
                    try:
                        new_ptfe_value = float(entry)
                        break
                    except (TypeError, ValueError):
                        continue
            elif ptfe_values is not None:
                try:
                    new_ptfe_value = float(ptfe_values)
                except (TypeError, ValueError):
                    new_ptfe_value = None

            if new_ptfe_value is not None:
                self._last_ptfe_value = new_ptfe_value

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

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        """Run the OpenAMS HUB HES calibration for a specific lane."""

        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            gcmd.respond_info(
                "SPOOL parameter is required for OpenAMS HUB HES calibration."
            )
            return

        lane = self._find_lane_by_spool(spool_index)
        lane_name = getattr(lane, "name", None)
        self._calibrate_hub_hes_spool(spool_index, gcmd, lane_name=lane_name)

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL(self, gcmd):
        """Calibrate HUB HES for every loaded OpenAMS lane in this unit."""

        prompt = AFCprompt(gcmd, self.logger)
        prompt.p_end()

        active_lane = getattr(self.afc, "current", None)
        if isinstance(active_lane, AFCLane):
            active_lane_obj = active_lane
        elif isinstance(active_lane, str):
            active_lane_obj = self.lanes.get(active_lane)
        else:
            active_lane_obj = None

        if (
            isinstance(active_lane_obj, AFCLane)
            and getattr(active_lane_obj, "unit_obj", None) is self
            and getattr(active_lane_obj, "tool_loaded", False)
        ):
            lane_label = getattr(active_lane_obj, "name", "current lane")
            gcmd.respond_info(
                "Cannot calibrate all OpenAMS lanes while {} is active on this unit. "
                "Please unload the current tool and try again.".format(lane_label)
            )
            return

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
            gcmd.respond_info(
                "No loaded OpenAMS lanes were found to calibrate HUB HES values."
            )
            return

        successful = 0
        for lane, spool_index in calibrations:
            if self._calibrate_hub_hes_spool(
                spool_index, gcmd, lane_name=getattr(lane, "name", None)
            ):
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

    def cmd_AFC_OAMS_CALIBRATE_PTFE(self, gcmd):
        """Run the OpenAMS PTFE calibration for a specific lane."""

        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            gcmd.respond_info(
                "SPOOL parameter is required for OpenAMS PTFE calibration."
            )
            return

        lane = self._find_lane_by_spool(spool_index)
        lane_name = getattr(lane, "name", None)
        self._calibrate_ptfe_spool(spool_index, gcmd, lane_name=lane_name)

    def _calibrate_hub_hes_spool(self, spool_index, gcmd, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            gcmd.respond_info(
                "Unable to determine OpenAMS index for HUB HES calibration."
            )
            return False

        command = f"OAMS_CALIBRATE_HUB_HES OAMS={oams_index} SPOOL={spool_index}"
        lane_label = lane_name or f"spool {spool_index}"
        gcmd.respond_info(
            f"Running HUB HES calibration for {lane_label} with '{command}'."
        )

        try:
            messages = self._run_command_with_capture(command)
        except Exception:
            self.logger.exception(
                "Failed to execute OpenAMS HUB HES calibration for spool %s", spool_index
            )
            gcmd.respond_info(
                f"Failed to execute HUB HES calibration for {lane_label}. See logs."
            )
            return False

        hub_values = self._parse_hub_hes_messages(messages)
        if not hub_values:
            gcmd.respond_info(
                f"Completed {command} but no HUB HES value was reported. Check OpenAMS status logs."
            )
            return False

        config_values = self._read_config_sequence("hub_hes_on")
        if not config_values:
            config_values = self._last_hub_hes_values or []

        if not config_values:
            gcmd.respond_info(
                "Could not find hub_hes_on in your cfg; update the value manually."
            )
            return False

        values = list(config_values)
        max_length = len(values)
        updated_indices = []
        for index, parsed_value in sorted(hub_values.items()):
            if index >= max_length:
                gcmd.respond_info(
                    "HUB HES calibration reported index {} but your cfg only "
                    "defines {} value(s); update the remaining entries manually."
                    .format(index, max_length)
                )
                continue
            values[index] = parsed_value
            updated_indices.append(index)

        if not updated_indices:
            gcmd.respond_info(
                "Completed {} but no HUB HES value was stored; check your cfg."
                .format(command)
            )
            return False

        formatted = self._format_sequence(values)
        if not formatted:
            gcmd.respond_info("Unable to format HUB HES calibration values.")
            return False

        if not self._write_config_value("hub_hes_on", formatted):
            gcmd.respond_info(
                "Failed to update hub_hes_on in your cfg; please update it manually."
            )
            return False

        self._last_hub_hes_values = values

        if updated_indices:
            if len(updated_indices) == 1:
                index_text = f"index {updated_indices[0]}"
            else:
                index_text = "indices " + ", ".join(str(i) for i in updated_indices)
            gcmd.respond_info(
                f"Stored OpenAMS hub_hes_on {formatted} in your cfg (updated {index_text})."
            )
        else:
            gcmd.respond_info(
                f"Stored OpenAMS hub_hes_on {formatted} in your cfg."
            )
        return True

    def _calibrate_ptfe_spool(self, spool_index, gcmd, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            gcmd.respond_info(
                "Unable to determine OpenAMS index for PTFE calibration."
            )
            return False

        command = (
            f"OAMS_CALIBRATE_PTFE_LENGTH OAMS={oams_index} SPOOL={spool_index}"
        )
        lane_label = lane_name or f"spool {spool_index}"
        gcmd.respond_info(
            f"Running PTFE calibration for {lane_label} with '{command}'."
        )

        try:
            messages = self._run_command_with_capture(command)
        except Exception:
            self.logger.exception(
                "Failed to execute OpenAMS PTFE calibration for spool %s", spool_index
            )
            gcmd.respond_info(
                f"Failed to execute PTFE calibration for {lane_label}. See logs."
            )
            return False

        captured = self._parse_ptfe_messages(messages)
        value = None
        if captured:
            if 0 <= spool_index < len(captured):
                value = captured[spool_index]
            elif len(captured) == 1:
                value = captured[0]

        if value is None:
            gcmd.respond_info(
                f"Completed {command} but no PTFE length was reported. Check OpenAMS status logs."
            )
            return False

        formatted_value = self._format_numeric(value)
        if formatted_value is None:
            gcmd.respond_info(
                "Unable to format PTFE calibration value for config storage."
            )
            return False

        if not self._write_config_value("ptfe_length", formatted_value):
            gcmd.respond_info(
                "Failed to update ptfe_length in your cfg; please update it manually."
            )
            return False

        self._last_ptfe_value = value
        target_name = lane_label
        gcmd.respond_info(
            f"Stored OpenAMS ptfe_length {formatted_value} for {target_name} in your cfg."
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
        pattern = re.compile(
            r"HES\s*([0-9]+)\D+(-?[0-9]+(?:\.[0-9]+)?)",
            re.IGNORECASE,
        )

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
        pattern = re.compile(
            r"(?:ptfe|bowden)[^0-9\-]*(-?[0-9]+(?:\.[0-9]+)?)",
            re.IGNORECASE,
        )

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
            filenames = sorted(
                filename
                for filename in os.listdir(config_dir)
                if filename.lower().endswith(".cfg")
            )
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
        afc_function = getattr(self.afc, "function", None)
        if not section or afc_function is None:
            return False

        rewrite = getattr(afc_function, "ConfigRewrite", None)
        if not callable(rewrite):
            return False

        msg = f"\n{self.name} {key}: Saved {value}"
        try:
            rewrite(section, key, value, msg)
        except Exception:
            self.logger.exception(
                "Failed to persist %s for OpenAMS unit %s", key, self.name
            )
            return False

        return True

    def _find_lane_by_spool(self, spool_index):
        for lane in self.lanes.values():
            if self._get_openams_spool_index(lane) == spool_index:
                return lane
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
