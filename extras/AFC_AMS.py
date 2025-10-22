"""AMS integration helpers for Armored Turtle AFC."""

from __future__ import annotations

import traceback
from pathlib import Path
from textwrap import dedent
from typing import Dict

from configparser import ConfigParser, Error as ConfigError

try:  # pragma: no cover - defensive guard for runtime import errors
    from extras.AFC_unit import afcUnit
except Exception as exc:  # pragma: no cover - defensive guard
    raise ConfigError(
        "Error when trying to import AFC_unit\n{trace}".format(
            trace=traceback.format_exc()
        )
    ) from exc

try:  # pragma: no cover - defensive guard for runtime import errors
    from extras.AFC_lane import AFCLaneState
except Exception as exc:  # pragma: no cover - defensive guard
    raise ConfigError(
        "Error when trying to import AFC_lane\n{trace}".format(
            trace=traceback.format_exc()
        )
    ) from exc

try:  # pragma: no cover - defensive guard for runtime import errors
    from extras.AFC_utils import add_filament_switch
except Exception as exc:  # pragma: no cover - defensive guard
    raise ConfigError(
        "Error when trying to import AFC_utils\n{trace}".format(
            trace=traceback.format_exc()
        )
    ) from exc


SYNC_INTERVAL = 2.0


def _normalize_ams_tool_pin(config) -> str | None:
    """Ensure the assigned extruder uses the virtual bypass chip."""

    try:
        extruder_name = config.get("extruder", None, note_valid=False)
    except Exception:
        extruder_name = None

    if not extruder_name:
        return None

    parser = getattr(config, "fileconfig", None)
    if parser is None:
        return None

    section = f"AFC_extruder {extruder_name}"
    if not parser.has_section(section):
        return None

    try:
        raw_value = parser.get(section, "pin_tool_start")
    except Exception:
        return None

    if raw_value is None:
        return None

    pin_value = raw_value.strip()
    if not pin_value or pin_value.lower() == "none":
        return None

    if pin_value.startswith("afc_virtual_bypass:"):
        base_pin = pin_value.split(":", 1)[1].strip()
        if base_pin.startswith("AMS_"):
            return base_pin
        return None

    if not pin_value.startswith("AMS_"):
        return None

    parser.set(section, "pin_tool_start", f"afc_virtual_bypass:{pin_value}")
    return pin_value


class afcAMS(afcUnit):
    """AFC unit subclass that synchronises state with OpenAMS."""

    def __init__(self, config):
        initial_virtual_tool_pin = getattr(
            config, "_afc_ams_virtual_tool_pin", None
        )

        super().__init__(config)
        self.type = "AMS"

        # OpenAMS specific options
        self.oams_name = config.get("oams", "oams1")
        self.interval = config.getfloat("interval", SYNC_INTERVAL, above=0.0)

        self.reactor = self.printer.get_reactor()
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Track previous sensor state to only forward changes
        self._last_lane_states: Dict[str, bool] = {}
        self._last_hub_states: Dict[str, bool] = {}
        self._virtual_tool_switch_created = False
        self._cached_virtual_tool_pin: str | None = initial_virtual_tool_pin
        self.oams = None

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

    def _ensure_virtual_tool_sensor(self):
        """Create a virtual AMS tool sensor when configured for this unit."""

        if self._virtual_tool_switch_created:
            return

        base_pin = self._get_virtual_tool_pin()
        if base_pin is None:
            return

        try:
            self.printer.lookup_object(f"filament_switch_sensor {base_pin}")
        except Exception:
            sensor_exists = False
        else:
            sensor_exists = True

        if sensor_exists:
            self._virtual_tool_switch_created = True
            return

        virtual_pin = f"afc_virtual_bypass:{base_pin}"

        try:
            add_filament_switch(
                base_pin,
                virtual_pin,
                self.printer,
                show_sensor=self.afc.enable_sensors_in_gui,
            )
        except Exception:
            return

        self._virtual_tool_switch_created = True
        self.logger.info(
            f"Registered virtual AMS filament sensor '{base_pin}' using pin {virtual_pin}"
        )

    def _get_virtual_tool_pin(self) -> str | None:
        """Return the AMS pin name tied to this unit's extruder, if any."""

        if self._cached_virtual_tool_pin is not None:
            return self._cached_virtual_tool_pin

        extruder_name = getattr(self, "extruder", None)
        if not extruder_name:
            self._cached_virtual_tool_pin = None
            return None

        cfg_dir = getattr(self.afc, "cfgloc", None)
        if not cfg_dir:
            self._cached_virtual_tool_pin = None
            return None

        cfg_path = Path(cfg_dir) / "AFC-hardware.cfg"
        if not cfg_path.exists():
            self._cached_virtual_tool_pin = None
            return None

        parser = ConfigParser()
        parser.optionxform = str

        try:
            with cfg_path.open("r", encoding="utf-8") as cfg_file:
                parser.read_file(cfg_file)
        except Exception:
            self._cached_virtual_tool_pin = None
            return None

        section = f"AFC_extruder {extruder_name}"
        if not parser.has_section(section):
            self._cached_virtual_tool_pin = None
            return None

        tool_pin = parser.get(section, "pin_tool_start", fallback=None)
        if tool_pin is None:
            self._cached_virtual_tool_pin = None
            return None

        tool_pin = tool_pin.strip()
        if not tool_pin or tool_pin.lower() == "none":
            self._cached_virtual_tool_pin = None
            return None

        if tool_pin.startswith("afc_virtual_bypass:"):
            tool_pin = tool_pin.split(":", 1)[1].strip()

        if not tool_pin.startswith("AMS_"):
            self._cached_virtual_tool_pin = None
            return None

        self._cached_virtual_tool_pin = tool_pin
        return tool_pin

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

        self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
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

            lane.tool_loaded = False
            lane.loaded_to_hub = False
            lane.status = AFCLaneState.NONE
            lane.unit_obj.lane_unloaded(lane)
            lane.td1_data = {}
            lane.afc.spool.clear_values(lane)

        lane.afc.save_vars()
        self._last_lane_states[lane.name] = lane_val

    def _sync_event(self, eventtime):
        """Poll OpenAMS for state updates and propagate to lanes/hubs."""

        try:
            if self.oams is None:
                return eventtime + self.interval

            lane_values = getattr(self.oams, "f1s_hes_value", None)
            hub_values = getattr(self.oams, "hub_hes_value", None)

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
                    self._last_lane_states[lane.name] = lane_val

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
        except Exception:
            # Avoid stopping the reactor loop if OpenAMS query fails.
            pass

        return eventtime + self.interval


def load_config_prefix(config):
    base_pin = _normalize_ams_tool_pin(config)
    if base_pin is not None:
        setattr(config, "_afc_ams_virtual_tool_pin", base_pin)
    return afcAMS(config)
