# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AFC unit driver for Anycubic ACE PRO filament changers."""

from __future__ import annotations

import logging
import traceback
from configparser import Error as error
from datetime import datetime
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState, SpeedMode, AssistActive
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_ACE_serial import ACEConnection, ACESerialError, ACETimeoutError
except: raise error(ERROR_STR.format(import_lib="AFC_ACE_serial", trace=traceback.format_exc()))

try:
    from extras.AFC_RFID import (
        rgb_array_to_hex, color_name, apply_filament_defaults,
        sync_rfid_to_spoolman, get_auto_spoolman_create,
        log_new_filament, log_new_spool,
    )
except Exception:
    # AFC_RFID is OPTIONAL. The ACE has its own RFID routine and must still
    # apply filament info to the lane (and work at all) on printers that don't
    # deploy / want AFC's RFID+Spoolman integration. Provide self-contained
    # fallbacks only for the lane-info path; the Spoolman-specific helpers stay
    # None so those code paths (all guarded) simply no-op — without AFC_RFID
    # there's no Spoolman matching/creation, and _get_auto_spoolman_create()
    # already falls back to its own self.auto_spoolman_create.
    sync_rfid_to_spoolman = None
    get_auto_spoolman_create = None
    log_new_filament = None
    log_new_spool = None

    def color_name(hex_str):
        return ""

    def rgb_array_to_hex(color_array):
        """[r, g, b] -> '#rrggbb' (fallback when AFC_RFID isn't present)."""
        if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
            try:
                r, g, b = (int(color_array[0]), int(color_array[1]),
                           int(color_array[2]))
                return "#%02x%02x%02x" % (r & 0xFF, g & 0xFF, b & 0xFF)
            except (TypeError, ValueError):
                return ""
        return ""

    def apply_filament_defaults(lane, slot_info, color_converter=None,
                                afc_defaults=None):
        """Apply the ACE's RFID material/color/temps to a lane when not already
        set — a dependency-free copy of AFC_RFID.apply_filament_defaults so the
        ACE can populate the lane (shows in Mainsail) with no AFC_RFID present."""
        info = slot_info or {}
        has_material = getattr(lane, "material", None) not in (None, "")
        has_color = getattr(lane, "color", None) not in (None, "", "#000000")
        has_ext_temp = getattr(lane, "extruder_temp", None) is not None
        has_bed_temp = getattr(lane, "bed_temp", None) is not None

        material = (info.get("material", "") or "")
        if material.lower() == "unknown":
            material = ""
        color_hex = info.get("color_hex", "") or ""
        if not color_hex and color_converter is not None:
            raw = info.get("color", [0, 0, 0])
            if raw != [0, 0, 0]:
                color_hex = color_converter(raw)

        if not has_material and material:
            lane.material = material
        if not has_color and color_hex:
            lane.color = color_hex if color_hex.startswith("#") else "#" + color_hex
        if not has_ext_temp and info.get("extruder_temp") is not None:
            try:
                lane.extruder_temp = float(info.get("extruder_temp"))
            except (TypeError, ValueError):
                pass
        if not has_bed_temp and info.get("bed_temp") is not None:
            try:
                lane.bed_temp = float(info.get("bed_temp"))
            except (TypeError, ValueError):
                pass

        if afc_defaults is not None:
            if not getattr(lane, "material", None):
                dm = afc_defaults.get("default_material_type")
                if dm:
                    lane.material = dm
            if not getattr(lane, "color", None):
                dc = afc_defaults.get("default_color")
                if dc:
                    lane.color = dc

try: from extras.AFC_logger import AFC_QueueListener
except: AFC_QueueListener = None

try: from queuelogger import QueueHandler
except: QueueHandler = None


MODE_COMBINED = "combined"
MODE_DIRECT = "direct"


# ── Compatibility shims for the frozen upstream core (see AFC_compat.py) ──
from extras.AFC_compat import apply_compat_patches
apply_compat_patches()


class afcACE(afcUnit):
    SLOTS_PER_UNIT = 4

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get('type', 'ACE')

        self.serial_port = config.get("serial_port")
        mode = config.get("mode", MODE_COMBINED).lower().strip()
        if mode not in (MODE_COMBINED, MODE_DIRECT):
            raise error(
                f"[{config.get_name()}] invalid mode '{mode}'. "
                f"Use '{MODE_COMBINED}' or '{MODE_DIRECT}'.")
        self.mode = mode

        self.feed_speed = config.getfloat("feed_speed", 60.0)
        self.retract_speed = config.getfloat("retract_speed", 50.0)
        self.unload_preretract = config.getfloat("unload_preretract", 50.0)
        self._unit_load_to_hub = config.getboolean("load_to_hub", None)
        self._default_feed_assist = config.getboolean("use_feed_assist", True)
        # Watchdog: on each heartbeat (when idle) make sure the lane on the
        # active tool actually has its feed assist running, in case a pickup /
        # tool-change event didn't enable it.
        self._assist_watchdog = config.getboolean("assist_watchdog", True)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)
        self.sensor_approach_margin = config.getfloat("sensor_approach_margin", 30.0)
        self.sensor_step = config.getfloat("sensor_step", 40.0)
        self.calibration_step = config.getfloat("calibration_step", 50.0)
        self.max_feed_overshoot = config.getfloat("max_feed_overshoot", 100.0)
        self.fps_threshold = config.getfloat("fps_threshold", 0.9)
        self.fps_load_threshold = config.getfloat("fps_load_threshold", 0.65)
        self.fps_delta_threshold = config.getfloat("fps_delta_threshold", 0.15)
        self.fps_confirm_count = config.getint("fps_confirm_count", 3, minval=1)
        self.baud_rate = config.getint("baud_rate", 115200)

        self.auto_spoolman_create = config.getboolean("auto_spoolman_create", False)

        self._ace: Optional[ACEConnection] = None
        self._slot_map: dict[str, int] = {}
        self._feed_assist_active: set[int] = set()
        self._slot_inventory: list[dict] = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._operation_active = False
        self._cached_hw_status = {}
        self._prev_states_stale = False
        self._prev_slot_states: dict[str, bool] = {}
        self._hub_load_suppressed: set[str] = set()

        self.gcode = self.printer.lookup_object('gcode')
        unit_suffix = self.name.upper().replace(" ", "_")
        self._custom_load_cmd_name = f'_ACE_CUSTOM_LOAD_{unit_suffix}'
        self._custom_unload_cmd_name = f'_ACE_CUSTOM_UNLOAD_{unit_suffix}'
        self.gcode.register_command(
            self._custom_load_cmd_name, self._cmd_ace_custom_load,
            desc=f"ACE internal load command ({self.name})")
        self.gcode.register_command(
            self._custom_unload_cmd_name, self._cmd_ace_custom_unload,
            desc=f"ACE internal unload command ({self.name})")
        self.gcode.register_command(
            f'ACE_CALIBRATE_{unit_suffix}', self.cmd_ACE_CALIBRATE,
            desc=f"Calibrate ACE feed distance to toolhead ({self.name})")
        self.gcode.register_command(
            f'ACE_CALIBRATE_HUB_{unit_suffix}', self.cmd_ACE_CALIBRATE_HUB,
            desc=f"Calibrate ACE feed distance to hub ({self.name})")
        self.gcode.register_command(
            f'ACE_STATUS_{unit_suffix}', self.cmd_ACE_STATUS,
            desc=f"Query ACE hardware status ({self.name})")
        self.gcode.register_command(
            f'ACE_DRY_{unit_suffix}', self.cmd_ACE_DRY,
            desc=f"Start ACE filament dryer ({self.name})")
        self.gcode.register_command(
            f'ACE_DRY_STOP_{unit_suffix}', self.cmd_ACE_DRY_STOP,
            desc=f"Stop ACE filament dryer ({self.name})")
        self.gcode.register_command(
            f'ACE_LANE_RESET_{unit_suffix}', self.cmd_ACE_LANE_RESET,
            desc=f"Retract ACE lane filament back into unit ({self.name})")
        # Register base commands (first unit wins for single-unit setups)
        for cmd, handler, desc in [
            ('ACE_CALIBRATE', self.cmd_ACE_CALIBRATE, "Calibrate ACE feed distance to toolhead"),
            ('ACE_CALIBRATE_HUB', self.cmd_ACE_CALIBRATE_HUB, "Calibrate ACE feed distance to hub"),
            ('ACE_STATUS', self.cmd_ACE_STATUS, "Query ACE hardware status"),
            ('ACE_DRY', self.cmd_ACE_DRY, "Start ACE filament dryer"),
            ('ACE_DRY_STOP', self.cmd_ACE_DRY_STOP, "Stop ACE filament dryer"),
            ('ACE_LANE_RESET', self.cmd_ACE_LANE_RESET, "Retract ACE lane filament back into unit"),
        ]:
            try:
                self.gcode.register_command(cmd, handler, desc=desc)
            except Exception:
                pass

        # Register temperature_ace sensor factory for [temperature_sensor]
        # sections that use sensor_type: temperature_ace.  This is a
        # convenience — prefer [temperature_ace <name>] sections which
        # bypass the factory mechanism and avoid config-ordering issues.
        try:
            from extras.temperature_ace import TemperatureACE
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_ace", TemperatureACE)
        except Exception as e:
            logging.info(
                "AFC_ACE: temperature_ace factory not registered (%s); "
                "use [temperature_ace <name>] config sections instead", e)

    def handle_connect(self):
        super().handle_connect()

        self.logo = '<span class=success--text>R  ACE PRO\n'
        self.logo += 'E  ========\n'
        self.logo += 'A  |      |\n'
        self.logo += 'D  | {slots} |\n'.format(slots=''.join(['[{}]'.format(i) for i in range(len(self.lanes))]))
        self.logo += 'Y  ========</span>\n'
        self.logo += '  ' + self.name + '\n'

        self.logo_error = '<span class=error--text>E  ACE PRO\n'
        self.logo_error += 'R  ========\n'
        self.logo_error += 'R  | ERR  |\n'
        self.logo_error += 'O  |  <span class=secondary--text>X</span>   |\n'
        self.logo_error += 'R  ========</span>\n'
        self.logo_error += '  ' + self.name + '\n'

        # Build slot map (1-based config index → 0-based ACE slot)
        # and set custom load/unload commands
        for lane_name, lane in self.lanes.items():
            idx = getattr(lane, 'index', 0)
            slot = max(0, idx - 1)
            self._slot_map[lane_name] = slot
            lane.custom_load_cmd = f"{self._custom_load_cmd_name} UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"{self._custom_unload_cmd_name} UNIT={self.name} LANE={lane_name}"

        # Defer serial connection until reactor is running (klippy:ready)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # Reconcile feed assist to the active lane on every tool load.
        self.printer.register_event_handler("afc:tool_loaded", self._handle_tool_loaded)
        # A tool pickup activates that tool's extruder without necessarily
        # firing afc:tool_loaded (the lane is already loaded), so also reconcile
        # assist whenever the active extruder changes — works in both modes.
        self.printer.register_event_handler(
            "extruder:activate_extruder", self._handle_extruder_activated)

    def _handle_tool_loaded(self, lane):
        # Resolve the active lane name. The payload is usually the loaded
        # lane, but at print start / with a tool already on the shuttle the
        # toolchanger fires this with the extruder object (name "extruder"):
        # use the lane loaded on that extruder, then fall back to AFC's
        # current lane.
        name = getattr(lane, 'name', None)
        if self.mode == MODE_DIRECT:
            if name not in self._slot_map:
                loaded = getattr(lane, 'lane_loaded', None)  # extruder payload
                if loaded in self._slot_map:
                    name = loaded
                else:
                    name = getattr(self.afc, 'current', None)
        else:
            # Combined mode: one shared toolhead, so only act when the event
            # is directly for one of our lanes (a lane load or a manual
            # SET_LANE_LOADED recovery). Don't fall back to afc.current.
            if name not in self._slot_map:
                return
        # Defer the ACE handshake off the toolhead's gcode greenlet so it
        # does not hold position over the wipe tower (and ooze) while the
        # ACE acks the feed-assist start.
        self.afc.reactor.register_callback(
            lambda et, n=name: self._reconcile_feed_assist(n))

    def _reconcile_feed_assist(self, name):
        """Switch ACE feed assist to the active lane (background callback).

        Stop assist on the previously-active slot(s) and wait for the ACE to
        ack, THEN start assist on the new slot. Both run in this background
        reactor callback — off the toolhead's gcode greenlet — so the acks
        never hold the toolhead over the wipe tower.
        """
        active_slot = self._slot_map.get(name)
        if active_slot is not None:
            # Stop the previously-active slot(s) first (stop_feed_assist_sync
            # waits for the ACE ack), then start the new slot, so the ACE is
            # never asked to feed two slots at once.
            for slot in list(self._feed_assist_active):
                if slot != active_slot:
                    self._stop_feed_assist(slot)
            active_lane = self.afc.lanes.get(name)
            if (active_lane is not None and self._use_feed_assist(active_lane)
                    and active_slot not in self._feed_assist_active):
                self._start_feed_assist(active_slot)
        elif name is not None and name in self.afc.lanes:
            # The active tool is a real lane on ANOTHER unit — stop ours.
            for slot in list(self._feed_assist_active):
                self._stop_feed_assist(slot)
        # else: couldn't resolve the active tool — leave assist untouched
        # rather than risk stopping a valid one.

    _CONNECT_MAX_RETRIES = 5
    _CONNECT_RETRY_DELAYS = [2.0, 3.0, 5.0, 8.0, 10.0]
    _SERIAL_LOG_MAX_BYTES = 5 * 1024 * 1024

    def _create_serial_logger(self):
        """Create a dedicated logger for ACE serial comms (writes to AFC_ACE_serial.log)."""
        if AFC_QueueListener is None or QueueHandler is None:
            return None
        logger = logging.getLogger("AFC_ACE_serial_file")
        if logger.handlers:
            return logger
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        try:
            log_path = self.printer.start_args.get("log_file", None)
            if log_path:
                log_file = Path(log_path).parent / "AFC_ACE_serial.log"
                # Rotate oversized log on startup
                try:
                    lp = Path(log_file)
                    if lp.exists() and lp.stat().st_size > self._SERIAL_LOG_MAX_BYTES:
                        backup = lp.with_suffix('.log.1')
                        if backup.exists():
                            backup.unlink()
                        lp.rename(backup)
                except Exception:
                    pass
                ql = AFC_QueueListener(log_file)
                ql.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                handler = QueueHandler(ql.bg_queue)
                logger.addHandler(handler)
                self._serial_ql = ql
                return logger
        except Exception:
            pass
        return None

    def _handle_ready(self):
        # Seed each lane's virtual hub with its real loaded state up front.
        # Loadless serial lanes default _load_state=True upstream, so a virtual
        # hub (state = any(raw_load_state)) reads "loaded" until something drives
        # it — and if the ACE never connects (empty/offline unit) nothing ever
        # does. Driving it here sets _state_driven so the hub reports its real
        # state regardless of the hardware connection. No-op for physical hubs
        # (their real switch_pin drives the state), so both setups work.
        for lane in self.lanes.values():
            self._set_hub_state(lane, lane.loaded_to_hub)
        self.afc.reactor.register_callback(self._deferred_ace_connect)

    def _deferred_ace_connect(self, eventtime):
        """Connect to ACE hardware after reactor is fully running."""
        serial_logger = self._create_serial_logger() or self.logger
        last_err = None
        for attempt in range(self._CONNECT_MAX_RETRIES):
            try:
                self._ace = ACEConnection(
                    reactor=self.afc.reactor,
                    serial_port=self.serial_port,
                    logger=serial_logger,
                    baud_rate=self.baud_rate,
                )
                self._ace.connect()
                if attempt > 0:
                    self.logger.info(
                        f"ACE {self.name}: connected on attempt {attempt + 1}")
                break
            except Exception as e:
                last_err = e
                self._ace = None
                if attempt < self._CONNECT_MAX_RETRIES - 1:
                    delay = self._CONNECT_RETRY_DELAYS[attempt]
                    self.logger.info(
                        f"ACE {self.name}: serial port not available, "
                        f"retrying in {delay:.0f}s "
                        f"(attempt {attempt + 1}/{self._CONNECT_MAX_RETRIES})")
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + delay)
        else:
            self.logger.error(
                f"ACE {self.name}: failed to connect at "
                f"{self.serial_port} after {self._CONNECT_MAX_RETRIES} "
                f"attempts: {last_err}")
            return

        self.logger.info(
            f"ACE {self.name}: connected, mode={self.mode}, "
            f"port={self.serial_port}, slots={self.SLOTS_PER_UNIT}")

        # Enable RFID reader so get_filament_info returns spool data
        try:
            self._ace.enable_rfid()
            self.logger.debug(f"ACE {self.name}: RFID enabled")
        except Exception as e:
            self.logger.warning(
                f"ACE {self.name}: enable_rfid failed (non-fatal): {e}")

        # Seed slot status from get_status
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                self._cached_hw_status = hw_status
                for i, slot_data in enumerate(hw_status.get("slots", [])):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        self._slot_inventory[i]["status"] = slot_data.get("status", "")
        except Exception as e:
            self.logger.debug(f"ACE initial get_status failed: {e}")

        # Sync RFID data and lane loaded states
        self._sync_inventory()
        self._sync_slot_loaded_state()

        # Register callback for heartbeat status updates
        self._ace.status_callback = self._on_hw_status_callback
        # Re-establish feed assist after a USB drop/reconnect (common on this
        # hardware). The ACE firmware resets on reconnect and forgets any
        # running feed assist, so resync it back to the active lane.
        self._ace.reconnect_callback = self._on_ace_reconnect

    def _on_ace_reconnect(self):
        # The ACE forgot its feed-assist state across the reset. Drop our
        # stale tracking (otherwise the watchdog's "already active" guard would
        # never re-issue it) and immediately re-send assist for whatever lane
        # is currently loaded on the active toolhead — derived from live state
        # so it stays correct even if a tool change happened during the drop.
        if self._feed_assist_active:
            self.logger.info(
                "ACE reconnected — re-establishing feed assist for the "
                "active lane")
        self._feed_assist_active.clear()
        # Defer off the reconnect/serial path so the ACE acks don't block it.
        self.afc.reactor.register_callback(self._resync_assist_after_reconnect)

    def _resync_assist_after_reconnect(self, eventtime):
        # Don't fight an in-flight load/unload — it sets assist itself when it
        # finishes. Otherwise let the watchdog re-issue assist for the active
        # lane right now (its target is empty again, so it will reconcile).
        if not self._operation_active:
            self._maybe_assist_watchdog()

    def _on_hw_status_callback(self, response):
        """Process heartbeat status from ACE — keep lane states in sync."""
        if not isinstance(response, dict):
            return
        result = response.get("result", response)
        if not isinstance(result, dict):
            return
        self._cached_hw_status = result
        if self._operation_active:
            return
        self._sync_slot_states(result)
        self._maybe_assist_watchdog()

    def _is_virtual_hub(self, lane) -> bool:
        hub = lane.hub_obj
        return (hub is not None
                and hasattr(hub, 'is_virtual_pin')
                and hub.is_virtual_pin())

    def _set_hub_state(self, lane, state: bool):
        """Set the virtual hub sensor state for an ACE lane."""
        hub = lane.hub_obj
        if hub and hasattr(hub, 'is_virtual_pin') and hub.is_virtual_pin():
            try:
                eventtime = self.afc.reactor.monotonic()
                hub.switch_pin_callback(eventtime, state)
            except Exception:
                pass

    def _sync_slot_states(self, hw_status):
        """Sync lane prep/load state from ACE hardware slot status.

        Uses _prev_slot_states (per-lane dict) and _prev_states_stale to
        avoid false insert/remove detection after load/unload operations.
        """
        slots = hw_status.get("slots", [])

        resync_prev = self._prev_states_stale
        if resync_prev:
            self._prev_states_stale = False

        for i, slot_data in enumerate(slots):
            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                self._slot_inventory[i]["status"] = slot_data.get("status", "")

        for lane in self.lanes.values():
            slot = self._get_slot(lane.name)
            if slot >= len(slots) or not isinstance(slots[slot], dict):
                continue

            slot_status = slots[slot].get("status", "")
            slot_ready = slot_status == "ready"
            slot_transient = slot_status in ("shifting", "feeding", "unwinding")

            if not slot_transient:
                lane._load_state = slot_ready
                lane.prep_state = slot_ready

            prep_done = getattr(lane, '_afc_prep_done', False)

            # Slot ready but lane stuck in NONE — restore loaded state.
            if slot_ready and not slot_transient and prep_done and lane.status == AFCLaneState.NONE:
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE: {lane.name} restoring TOOLED state from saved vars")
                    lane.loaded_to_hub = True
                    self._set_hub_state(lane, True)
                    lane.sync_to_extruder()
                    if self.afc.current == lane.name:
                        self.afc.spool.set_active_spool(lane.spool_id)
                        self.lane_tool_loaded(lane)
                        lane.status = AFCLaneState.TOOLED
                    else:
                        self.lane_tool_loaded_idle(lane)
                        # Mark idle tool-loaded lanes TOOLED too, otherwise
                        # status stays NONE and this restore block re-fires on
                        # every hardware poll (log spam / wasted work).
                        lane.status = AFCLaneState.TOOLED
                    lane.enable_buffer()
                    if self._use_feed_assist(lane):
                        self._start_feed_assist(slot)
                else:
                    if lane.name in self._hub_load_suppressed:
                        lane._load_suppressed = True
                    eventtime = self.afc.reactor.monotonic()
                    lane.handle_load_runout(eventtime, True)

            # Filament removed — skip on first callback after operation
            # to avoid false triggers from stale _prev_slot_states.
            if not slot_ready and not slot_transient and not resync_prev:
                self._hub_load_suppressed.discard(lane.name)
                prev_ready = self._prev_slot_states.get(lane.name)
                if prev_ready:
                    eventtime = self.afc.reactor.monotonic()
                    lane.handle_load_runout(eventtime, False)

            if not slot_transient:
                self._prev_slot_states[lane.name] = slot_ready

    # ── Lane parameter helpers ──────────────────────────────────────

    def _get_bowden_length(self, cur_lane) -> float:
        """Total feed distance: dist_hub (lane→hub) + afc_bowden_length (hub→toolhead)."""
        dist = cur_lane.dist_hub
        hub = cur_lane.hub_obj
        if hub is not None:
            dist += getattr(hub, 'afc_bowden_length', 0)
        return dist

    def _get_unload_length(self, cur_lane) -> float:
        """Total retract distance: dist_hub + afc_unload_bowden_length."""
        dist = cur_lane.dist_hub
        hub = cur_lane.hub_obj
        if hub is not None:
            dist += getattr(hub, 'afc_unload_bowden_length', getattr(hub, 'afc_bowden_length', 0))
        return dist

    def _use_feed_assist(self, cur_lane) -> bool:
        fa = getattr(cur_lane, 'use_feed_assist', None)
        if fa is not None:
            return fa
        return self._default_feed_assist

    def _get_slot(self, lane_name: str) -> int:
        return self._slot_map.get(lane_name, 0)

    # ── Unit interface overrides ────────────────────────────────────

    def prep_load(self, lane: AFCLane):
        pass

    def check_runout(self, cur_lane):
        """ACE supports runout detection during printing."""
        try:
            return self.afc.function.is_printing()
        except Exception:
            return False

    def on_filament_insert(self, lane):
        """ACE-specific insertion: refresh RFID inventory, apply spool defaults, sync to spoolman."""
        slot = self._get_slot(lane.name)
        if slot >= self.SLOTS_PER_UNIT:
            return
        saved_spool_id = lane.spool_id
        # A spool_id present at insert came from set_loaded()->_set_values:
        # either a freshly-staged scan (external scanner -> next_spool_id) or a
        # remembered spool. The compat load handler records the staged id so we
        # can keep a fresh scan even when remember_spool is off.
        staged_spool_id = getattr(lane, '_afc_staged_spool_id', None)
        lane._afc_staged_spool_id = None
        self.afc.spool.clear_values(lane)
        self._refresh_slot_inventory(slot)
        slot_info = self._slot_inventory[slot]
        if apply_filament_defaults is not None:
            apply_filament_defaults(
                lane, slot_info,
                color_converter=rgb_array_to_hex,
                afc_defaults={
                    "default_material_type": getattr(self.afc, "default_material_type", None),
                    "default_color": getattr(self.afc, "default_color", None),
                })
        if sync_rfid_to_spoolman is not None:
            self._sync_rfid_to_spoolman(lane, slot_info)
        # If the ACE slot had no RFID identity of its own (lane.spool_id still
        # unset after the inventory read), restore the spool assigned before
        # insertion when it was freshly scanned/staged or remember_spool is on.
        if (saved_spool_id and not lane.spool_id
                and (staged_spool_id is not None or lane.remember_spool)):
            self.afc.spool.set_spoolID(lane, saved_spool_id)
        self.lane_illuminate_spool(lane)
        self._hub_load_suppressed.discard(lane.name)
        self.afc.save_vars()
        try:
            self.prep_post_load(lane)
        except Exception as e:
            self.logger.error(f"ACE on_filament_insert: prep_post_load error for {lane.name}: {e}")
        super().on_filament_insert(lane)

    def on_filament_remove(self, lane):
        """ACE-specific removal: clear slot inventory and hub state."""
        slot = self._get_slot(lane.name)
        if slot < self.SLOTS_PER_UNIT:
            self._clear_slot_inventory(slot)
        self._hub_load_suppressed.discard(lane.name)
        lane.loaded_to_hub = False
        self._set_hub_state(lane, False)

    # ── TD-1 support ────────────────────────────────────────────────

    def calibrate_td1(self, cur_lane, dis, tol):
        """Calibrate TD-1 bowden length by feeding until TD-1 device detects filament."""
        self._operation_active = True
        try:
            return self._calibrate_td1_inner(cur_lane, dis, tol)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def prep_capture_td1(self, cur_lane):
        """ACE TD-1 capture is triggered from prep_post_load (after hub feed).

        Return non-None to prevent the base _prep_capture_td1 from running
        the stepper-based path.
        """
        if cur_lane.td1_when_loaded and cur_lane.loaded_to_hub:
            return True, "TD-1 capture handled by prep_post_load"
        return None

    def capture_td1_data(self, cur_lane):
        """ACE TD-1 data capture using feed_filament instead of stepper moves."""
        if self._ace is None or not self._ace.connected:
            return None
        if cur_lane.td1_device_id is None:
            return None
        if cur_lane.td1_bowden_length is None:
            return False, "td1_bowden_length not set — run TD-1 calibration first"

        slot = self._get_slot(cur_lane.name)
        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            return False, msg

        self._operation_active = True
        try:
            return self._capture_td1_data_inner(cur_lane, slot)
        finally:
            self._operation_active = False

    def _capture_td1_data_inner(self, cur_lane, slot):
        dist_hub = cur_lane.dist_hub

        if not cur_lane.loaded_to_hub:
            self.logger.info(f"ACE TD-1 capture: feeding {dist_hub}mm to hub for {cur_lane.name}")
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, dist_hub, self.feed_speed)
            self._wait_for_ace_ready()

        feed_dist = cur_lane.td1_bowden_length
        self.logger.info(
            f"ACE TD-1 capture: feeding {feed_dist}mm to TD-1 for {cur_lane.name}")

        compare_time = datetime.now()
        self._wait_for_ace_ready()
        self._ace.feed_filament(slot, feed_dist, self.feed_speed)
        self._wait_for_ace_ready()

        self.afc.reactor.pause(self.afc.reactor.monotonic() + 5.0)

        success = self.get_td1_data(cur_lane, compare_time)
        if not success:
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 3.0)
            success = self.get_td1_data(cur_lane, compare_time)

        retract_dist = feed_dist + (0 if cur_lane.loaded_to_hub else dist_hub)
        self.logger.info(f"ACE TD-1 capture: retracting {retract_dist}mm for {cur_lane.name}")
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
        except Exception as e:
            self.logger.error(f"ACE TD-1 capture retract failed: {e}")

        if success:
            return True, f"TD-1 data captured for {cur_lane.name}"
        return False, "TD-1 data not captured (unload completed)"

    def _calibrate_td1_inner(self, cur_lane, dis, tol):
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        if cur_lane.td1_device_id is None:
            return False, (
                f"Cannot calibrate TD-1 for {cur_lane.name}, td1_device_id "
                "is a required field in AFC_hub or per AFC_lane"), 0

        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            return False, msg, 0

        slot = self._get_slot(cur_lane.name)
        step_size = dis if dis > 0 else 50.0
        max_bowden_length = 6000
        cur_hub = cur_lane.hub_obj
        dist_hub = cur_lane.dist_hub

        self.logger.info(
            f"ACE calibrate_td1: feeding slot {slot} in {step_size}mm steps, "
            f"max {max_bowden_length}mm, TD-1 device={cur_lane.td1_device_id}")

        self._wait_for_ace_ready()

        # Phase 1: Get filament to hub position
        has_real_hub = (cur_hub is not None
                        and hasattr(cur_hub, 'switch_pin')
                        and getattr(cur_hub, 'switch_pin', 'virtual').lower() != 'virtual')
        if cur_lane.loaded_to_hub:
            self.logger.info("ACE calibrate_td1: filament already at hub, skipping hub feed")
        elif has_real_hub:
            self.logger.info("ACE calibrate_td1: feeding to hub sensor")
            self._ace.feed_filament(slot, dist_hub + 200, self.feed_speed)
            hub_timeout = self.afc.reactor.monotonic() + 15.0
            while self.afc.reactor.monotonic() < hub_timeout:
                if cur_hub.state:
                    break
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            self._wait_for_ace_ready()
            if not cur_hub.state:
                self._ace.unwind_filament(slot, dist_hub + 200, self.retract_speed)
                self._wait_for_ace_ready()
                return False, f"Hub sensor did not trigger for {cur_lane.name}", 0
        elif dist_hub > 0:
            self.logger.info(f"ACE calibrate_td1: feeding {dist_hub}mm to virtual hub")
            self._ace.feed_filament(slot, dist_hub, self.feed_speed)
            self._wait_for_ace_ready()

        # Phase 2: Feed incrementally from hub, checking TD-1 after each step
        bow_pos = 0.0
        compare_time = datetime.now()
        while not self.get_td1_data(cur_lane, compare_time):
            if bow_pos > max_bowden_length:
                retract = bow_pos + (0 if cur_lane.loaded_to_hub else dist_hub)
                self._ace.unwind_filament(slot, retract, self.retract_speed)
                self._wait_for_ace_ready()
                return False, f"TD-1 failed to detect filament after {bow_pos:.0f}mm", bow_pos

            compare_time = datetime.now()
            bow_pos += step_size
            self._ace.feed_filament(slot, step_size, self.feed_speed)
            self._wait_for_ace_ready()
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 5.0)

        self.logger.info(f"ACE calibrate_td1: TD-1 detected filament at {bow_pos:.1f}mm")

        # Retract back
        if cur_lane.loaded_to_hub:
            retract_dist = bow_pos
        else:
            retract_dist = bow_pos + dist_hub
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
        except Exception as e:
            self.logger.error(f"ACE calibrate_td1: retract failed: {e}")

        # Save td1_bowden_length
        old_td1 = getattr(cur_lane, "td1_bowden_length", None)
        cur_lane.td1_bowden_length = bow_pos
        cal_msg = f"\n td1_bowden_length: New: {bow_pos} Old: {old_td1}"
        self.afc.function.ConfigRewrite(
            cur_lane.fullname, "td1_bowden_length", bow_pos, cal_msg)
        self.afc.save_vars()

        return True, (
            f"ACE TD-1 calibration: filament detected at {bow_pos:.1f}mm.\n"
            f"td1_bowden_length: {bow_pos:.0f} (was {old_td1})\n"
            f"Value saved to config."), bow_pos

    # ── Prep and post-load ─────────────────────────────────────────

    def prep_post_load(self, lane: AFCLane):
        """Stage filament to hub via ACE serial feed."""
        if self._unit_load_to_hub is not None:
            load_to_hub = self._unit_load_to_hub
        else:
            load_to_hub = getattr(lane, 'load_to_hub',
                                  getattr(self.afc, 'load_to_hub', False))
        if not load_to_hub or lane.loaded_to_hub:
            return
        if not lane.prep_state:
            return
        slot = self._get_slot(lane.name)
        if not self._ace or not self._ace.connected:
            return

        dist_hub = lane.dist_hub
        if dist_hub <= 0:
            # No feed needed (hub effectively at the load point): the filament
            # is already at the hub the moment it's detected, so mark it staged
            # instead of leaving the lane "detected but not loaded".
            lane.loaded_to_hub = True
            self._set_hub_state(lane, True)
            self.afc.save_vars()
            self.logger.info(
                f"ACE prep_post_load: {lane.name} staged at hub "
                f"(dist_hub=0, no feed needed)")
            return

        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self._wait_for_ace_ready(timeout=30.0)
                self._ace.feed_filament(slot, dist_hub, self.feed_speed)
                self._wait_for_feed_complete(slot, dist_hub, self.feed_speed)
                lane.loaded_to_hub = True
                self.afc.save_vars()
                self.logger.info(
                    f"ACE prep_post_load: {lane.name} staged at hub "
                    f"(dist_hub={dist_hub:.0f}mm)")
                # Capture TD-1 data now that filament is at hub
                if (lane.td1_when_loaded
                    and lane.td1_device_id
                    and self.afc.td1_present):
                    self.logger.info(f"ACE prep_post_load: capturing TD-1 data for {lane.name}")
                    self.capture_td1_data(lane)
                return
            except Exception as e:
                if attempt < max_attempts - 1:
                    wait = 5.0 * (attempt + 1)
                    self.logger.warning(
                        f"ACE prep_post_load: attempt {attempt + 1}/{max_attempts} "
                        f"failed, retrying in {wait:.0f}s: {e}")
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + wait)
                    continue
                self.logger.error(
                    f"ACE prep_post_load failed for {lane.name} after "
                    f"{max_attempts} attempts: {e}")

    def eject_lane(self, lane: AFCLane):
        """Retract filament back into ACE unit."""
        slot = self._get_slot(lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                self._wait_for_ace_ready()
                dist = self._get_unload_length(lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_feed_complete(slot, dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE eject failed for {lane.name}: {e}")

    def lane_move(self, cur_lane, distance, speed_mode):
        """Move filament via ACE serial instead of stepper."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            self.logger.error("ACE not connected for lane_move")
            return
        try:
            self._wait_for_ace_ready()
            if distance > 0:
                self._ace.feed_filament(slot, abs(distance), self.feed_speed)
            else:
                self._ace.unwind_filament(slot, abs(distance), self.retract_speed)
            self._wait_for_feed_complete(slot, abs(distance),
                                         self.feed_speed if distance > 0 else self.retract_speed)
        except Exception as e:
            self.logger.error(f"ACE lane_move failed: {e}")

    def lane_unload(self, cur_lane):
        """Custom lane unload — retract via ACE serial."""
        slot = self._get_slot(cur_lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                self._wait_for_ace_ready()
                dist = self._get_unload_length(cur_lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_feed_complete(slot, dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE lane_unload failed for {cur_lane.name}: {e}")
        return True

    def abort_load(self, cur_lane):
        """Cancel in-progress ACE feed."""
        if self._ace and self._ace.connected:
            try:
                self._ace.abort_current_action()
            except Exception:
                pass

    def get_lane_reset_command(self, lane, dis):
        return f"ACE_LANE_RESET UNIT={self.name} LANE={lane.name}"

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """ACE system test — query hardware slot status instead of physical switches."""
        msg = ''
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        # ACE has no physical prep/load switches — determine state from hardware
        if self._ace is not None and self._ace.connected:
            slot = self._get_slot(cur_lane.name)
            try:
                hw_status = self._ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot < len(slots) and isinstance(slots[slot], dict):
                        slot_ready = slots[slot].get("status", "") == "ready"
                        cur_lane._load_state = slot_ready
                        cur_lane.prep_state = slot_ready
                        if not slot_ready:
                            cur_lane.loaded_to_hub = False
                            self._set_hub_state(cur_lane, False)
            except Exception as e:
                self.logger.debug(f"ACE get_status failed during PREP: {e}")
        else:
            msg = '<span class=error--text>ACE NOT CONNECTED</span>'
            succeeded = False

        if succeeded:
            if not cur_lane.prep_state:
                if not cur_lane.raw_load_state:
                    self.lane_not_ready(cur_lane)
                    msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
                else:
                    self.lane_fault(cur_lane)
                    msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                    succeeded = False
            else:
                self.lane_loaded(cur_lane)
                msg += "<span class=success--text>LOCKED</span>"
                if not cur_lane.raw_load_state:
                    msg += "<span class=error--text> NOT LOADED</span>"
                    self.lane_not_ready(cur_lane)
                    succeeded = False
                else:
                    cur_lane.status = AFCLaneState.LOADED
                    msg += "<span class=success--text> AND LOADED</span>"
                    self.lane_illuminate_spool(cur_lane)

                    # Apply RFID data if available and not already set
                    if apply_filament_defaults is not None:
                        slot_info = self._slot_inventory[slot] if slot < self.SLOTS_PER_UNIT else {}
                        apply_filament_defaults(
                            cur_lane, slot_info,
                            color_converter=rgb_array_to_hex,
                            afc_defaults={
                                "default_material_type": getattr(self.afc, "default_material_type", None),
                                "default_color": getattr(self.afc, "default_color", None),
                            })

                    # Assume filament staged at hub on startup
                    if not cur_lane.tool_loaded and not cur_lane.loaded_to_hub:
                        load_to_hub = getattr(cur_lane, 'load_to_hub',
                                              getattr(self.afc, 'load_to_hub', False))
                        if load_to_hub:
                            cur_lane.loaded_to_hub = True

                    if (cur_lane.tool_loaded
                        and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                        cur_lane.loaded_to_hub = True
                        self._set_hub_state(cur_lane, True)
                        cur_lane.sync_to_extruder()
                        msg += "<span class=primary--text> in ToolHead</span>"

                        if self.afc.current == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            self.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                            self.printer.send_event("afc:tool_loaded", cur_lane)
                        else:
                            self.lane_tool_loaded_idle(cur_lane)
                        cur_lane.enable_buffer()

                        # Start feed assist for any lane confirmed in the
                        # toolhead — don't gate on self.afc.current which
                        # may be None on toolchangers during prep.
                        if self._use_feed_assist(cur_lane):
                            self._start_feed_assist(slot)

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.send_lane_data()
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    def _clear_stale_sensor_state(self, cur_lane):
        """Clear stale tool_start_state / filament_present / FPS latch so
        calibration reads real-time sensor values."""
        sensor_obj = getattr(cur_lane.extruder_obj, 'filament_sensor_obj', None)
        if sensor_obj is not None:
            sensor_obj.runout_helper.filament_present = False
        if hasattr(cur_lane.extruder_obj, 'tool_start_state'):
            cur_lane.extruder_obj.tool_start_state = False
        buffer_obj = getattr(cur_lane, 'buffer_obj', None)
        if buffer_obj is not None and hasattr(buffer_obj, 'clear_advance_latch'):
            buffer_obj.clear_advance_latch()
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)

    def _toolhead_sensor_triggered(self, cur_lane):
        """Check if the toolhead sensor is triggered, using the raw hardware
        button state for U1 motion sensors (which need encoder rotation for
        filament_present but have a physical switch for static detection)."""
        sensor_obj = getattr(cur_lane.extruder_obj, 'filament_sensor_obj', None)
        if sensor_obj is not None and hasattr(sensor_obj, 'runout_buttun_state'):
            return bool(sensor_obj.runout_buttun_state)
        return cur_lane.get_toolhead_pre_sensor_state()

    def _feed_until_sensor(self, slot, cur_lane, max_distance, step_size=None):
        """Feed filament in steps until toolhead sensor triggers.

        Returns (distance_fed, sensor_triggered).
        """
        if step_size is None:
            step_size = self.calibration_step
        total_fed = 0.0

        while total_fed < max_distance:
            step = min(step_size, max_distance - total_fed)
            self._wait_for_ace_ready()
            try:
                self._ace.feed_filament(slot, step, self.feed_speed)
            except Exception as e:
                self.logger.warning(
                    f"ACE calibration: feed failed at {total_fed:.0f}mm, retrying: {e}")
                self._wait_for_ace_ready(timeout=15.0)
                self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete(slot, step, self.feed_speed)
            total_fed += step

            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
            if self._toolhead_sensor_triggered(cur_lane):
                self.logger.info(f"ACE calibration: sensor triggered at {total_fed:.1f}mm")
                return total_fed, True

        return total_fed, False

    def calibrate_bowden(self, cur_lane, dis, tol):
        """Calibrate afc_bowden_length (hub→toolhead) by feeding until
        the toolhead sensor triggers."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            return False, "ACE not connected", 0

        cur_hub = cur_lane.hub_obj
        if cur_hub is None:
            return False, "Lane has no hub configured", 0

        self._operation_active = True
        try:
            return self._calibrate_bowden_inner(cur_lane, cur_hub, slot)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _calibrate_bowden_inner(self, cur_lane, cur_hub, slot):
        self._clear_stale_sensor_state(cur_lane)

        if self._toolhead_sensor_triggered(cur_lane):
            return False, "Toolhead sensor already triggered — unload first", 0

        old_bowden = getattr(cur_hub, 'afc_bowden_length', 0)
        max_distance = 6000

        self.logger.raw(
            f'Calibrating afc_bowden_length for {cur_lane.name} '
            f'(max {max_distance}mm in {self.calibration_step}mm steps)')

        # Coarse pass
        coarse_distance, triggered = self._feed_until_sensor(
            slot, cur_lane, max_distance)

        if not triggered:
            retract_dist = coarse_distance
            self.logger.info(f"ACE calibrate: retracting {retract_dist:.0f}mm")
            try:
                self._wait_for_ace_ready()
                self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
                self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
                self._wait_for_ace_ready(timeout=15.0)
            except Exception as e:
                self.logger.error(f"ACE calibrate: retract failed: {e}")
            self._clear_stale_sensor_state(cur_lane)
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.0)
            return False, (
                f"Toolhead sensor did not trigger after {coarse_distance:.0f}mm. "
                "Check filament path and sensor wiring."), coarse_distance

        # First detection only: use the trigger distance directly. The fine
        # back-off/re-feed pass was too small a move to reliably trip the
        # sensor and only added error — keep it simple.
        distance = coarse_distance
        self.logger.info(f"ACE calibrate: trigger at {distance:.1f}mm")

        # Retract
        retract_dist = distance + 5
        self.logger.info(f"ACE calibrate: retracting {retract_dist:.0f}mm")
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
            self._wait_for_ace_ready(timeout=15.0)
        except Exception as e:
            self.logger.error(f"ACE calibrate: retract failed: {e}")

        # Clear sensor state after retract
        self._clear_stale_sensor_state(cur_lane)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.0)

        # Account for filament already at hub
        if cur_lane.loaded_to_hub:
            dist_hub = cur_lane.dist_hub
            if dist_hub > 0:
                self.logger.info(
                    f"ACE calibrate: filament at hub, adding dist_hub="
                    f"{dist_hub:.0f}mm to measured {distance:.1f}mm")
                distance += dist_hub

        bowden_dist = round(distance, 2)
        cal_msg = f'\n afc_bowden_length: New: {bowden_dist} Old: {old_bowden}'
        cur_hub.afc_bowden_length = bowden_dist
        cur_hub.afc_unload_bowden_length = bowden_dist
        self.afc.function.ConfigRewrite(
            cur_hub.fullname, "afc_bowden_length", bowden_dist, cal_msg)
        self.afc.function.ConfigRewrite(
            cur_hub.fullname, "afc_unload_bowden_length", bowden_dist,
            f'\n afc_unload_bowden_length: {bowden_dist}')

        return True, f"afc_bowden_length calibration: {bowden_dist}mm (was {old_bowden}mm)", bowden_dist

    def calibrate_lane(self, cur_lane, tol):
        """Calibrate dist_hub from ACE slot to hub sensor."""
        self._operation_active = True
        try:
            return self._calibrate_hub_inner(cur_lane)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_ace_custom_load(self, gcmd):
        """Handle _ACE_CUSTOM_LOAD — filament transport to toolhead area."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_load_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE load failed for {lane_name}")

    def _cmd_ace_custom_unload(self, gcmd):
        """Handle _ACE_CUSTOM_UNLOAD — filament transport from toolhead."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_unload_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE unload failed for {lane_name}")

    def _ace_load_sequence(self, cur_lane, cur_extruder) -> bool:
        """ACE load transport: serial feed to toolhead area + feed assist."""
        self._operation_active = True
        try:
            return self._ace_load_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _ace_load_inner(self, cur_lane, cur_extruder) -> bool:
        """ACE custom load — filament transport only.

        AFC's load_sequence handles the shared toolhead engagement
        (sync_to_extruder, tool_end, tool_stn, sensor verification,
        buffer ram) after this returns via custom_load_cmd.
        """
        afc = self.afc
        slot = self._get_slot(cur_lane.name)

        if not self._ace or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE not connected ({self.serial_port})")
            return False

        # In combined mode, stop feed assist on other slots
        if self.mode == MODE_COMBINED:
            for other_name, other_slot in self._slot_map.items():
                if other_slot != slot and other_slot in self._feed_assist_active:
                    self._stop_feed_assist(other_slot)

        # Calculate feed distance
        if cur_lane.loaded_to_hub:
            hub = cur_lane.hub_obj
            feed_dist = getattr(hub, 'afc_bowden_length', 0) if hub else 0
            feed_dist = max(feed_dist, 50.0)
        else:
            feed_dist = self._get_bowden_length(cur_lane)

        self._set_hub_state(cur_lane, True)
        self._hub_load_suppressed.discard(cur_lane.name)

        # Enable FPS advance latch so the toolhead sensor stays triggered
        # even when pressure drops briefly after ACE feed completes.
        buffer_obj = getattr(cur_lane, 'buffer_obj', None)
        if buffer_obj is not None and hasattr(buffer_obj, 'enable_advance_latch'):
            buffer_obj.enable_advance_latch()

        # Pre-feed check: if the toolhead sensor detects filament before
        # we start feeding, there's stuck filament from a previous load.
        # Don't push more filament into an occupied toolhead.
        if self._toolhead_sensor_triggered(cur_lane):
            message = (
                f"Toolhead sensor detects filament before ACE feed for "
                f"{cur_lane.name}.\nFilament may be stuck from a previous "
                f"load — clear the toolhead before loading.\n"
                f"To resolve, manually retract filament from the toolhead "
                f"or run AFC_RESET for {cur_lane.name}."
            )
            if afc.function.in_print():
                message += "\nOnce cleared, click resume to continue printing"
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # ACE serial feed to toolhead area
        try:
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, feed_dist, self.feed_speed)
            success = self._wait_for_feed_complete(slot, feed_dist, self.feed_speed, cur_lane)

            if not success:
                if self._toolhead_sensor_triggered(cur_lane):
                    self.logger.info("Feed error but sensor triggered — continuing")
                else:
                    success = self._smart_load_retry(cur_lane, slot, feed_dist)
                    if not success:
                        afc.error.handle_lane_failure(
                            cur_lane, f"ACE feed failed for {cur_lane.name}")
                        return False
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE load feed error: {e}")
            return False

        # Post-feed sensor check: feed completed but filament may not have
        # reached the toolhead sensor yet.  Pulse 100mm until the sensor
        # triggers or 10 seconds elapse — ACE internal buffer handles
        # back-pressure at the extruder gears so larger pulses are safe.
        if not self._toolhead_sensor_triggered(cur_lane):
            deadline = self.afc.reactor.monotonic() + 10.0
            pulse_num = 0
            reached = False
            while self.afc.reactor.monotonic() < deadline:
                pulse_num += 1
                self.logger.info(
                    f"Sensor not triggered after feed for {cur_lane.name}, "
                    f"retry pulse {pulse_num} (100mm)")
                try:
                    self._wait_for_ace_ready()
                    self._ace.feed_filament(slot, 100.0, self.feed_speed)
                    self._wait_for_feed_complete(slot, 100.0, self.feed_speed, cur_lane)
                except Exception:
                    pass
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
                if self._toolhead_sensor_triggered(cur_lane):
                    self.logger.info(
                        f"Sensor triggered on retry pulse {pulse_num} for {cur_lane.name}")
                    reached = True
                    break
            if not reached:
                afc.error.handle_lane_failure(
                    cur_lane,
                    f"Filament did not reach toolhead sensor after feed + "
                    f"{pulse_num} retry pulses (10s timeout) for {cur_lane.name}.\n"
                    f"Check filament path and bowden length calibration.")
                return False

        # Set loaded_to_hub AFTER successful feed
        cur_lane.loaded_to_hub = True

        # Enable feed assist
        if self._use_feed_assist(cur_lane):
            self._start_feed_assist(slot)

        afc.afcDeltaTime.log_with_time("ACE load transport complete")
        return True

    def lane_unloading(self, lane):
        """Unload-start hook the upstream core actually calls.

        Upstream's unload sequence invokes ``lane_unloading`` (for LEDs) but
        never ``prepare_unload``, so we trigger the ACE feed-assist stop here as
        well. Keeps the watchdog from fighting the retract on the upstream core.
        """
        super().lane_unloading(lane)
        try:
            self.prepare_unload(lane, getattr(lane, 'hub_obj', None),
                                getattr(lane, 'extruder_obj', None))
        except Exception as e:
            self.logger.warning(
                "ACE: lane_unloading assist-stop error for %s: %s"
                % (getattr(lane, 'name', '?'), e))

    def prepare_unload(self, cur_lane, cur_hub, cur_extruder):
        """Stop ACE feed assist during filament tip forming.

        Assist should stop before cut/park/tip since it only conflicts with
        the serial unwind. This prevents inadvertent assist state on resume
        if an error during tip-forming pauses the print.
        """
        # Suppress the assist watchdog for the whole unload. AFC's quick-pull /
        # cut / tip-forming runs between here and the custom unload sequence
        # (which is the only thing that sets _operation_active). Without this,
        # the watchdog re-issues start_feed_assist on its ~2s tick while the
        # lane is still loaded, and the ACE feeds forward against the retract.
        # _ace_unload_sequence clears _operation_active in its finally; a failed
        # tip-form self-heals on the next load (which also resets it).
        self._operation_active = True
        slot = self._get_slot(cur_lane.name)
        self._stop_feed_assist(slot)

    def _ace_unload_sequence(self, cur_lane, cur_extruder) -> bool:
        """ACE unload transport — ACE serial unwind back to hub."""
        self._operation_active = True
        try:
            return self._ace_unload_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _ace_unload_inner(self, cur_lane, cur_extruder) -> bool:
        """ACE custom unload — filament transport only.

        AFC's unload_sequence handles the shared toolhead operations
        (LED, heat, quick pull, buffer disable, sync, cut/park/tip)
        then calls this via custom_unload_cmd.  The lane is still
        synced to the extruder on entry so we can do extruder moves.
        """
        afc = self.afc
        slot = self._get_slot(cur_lane.name)

        if not self._ace or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE not connected ({self.serial_port})")
            return False

        # Retract filament out of the toolhead/extruder gears before
        # the ACE serial unwind.  The lane is still synced to the
        # extruder at this point.
        if cur_extruder.tool_stn_unload > 0:
            afc.afcDeltaTime.log_with_time(
                'ACE unload: retracting tool_stn_unload from toolhead')
            afc.move_e_pos(
                cur_extruder.tool_stn_unload * -1,
                cur_extruder.tool_unload_speed, "ACE STN unload")
            afc.function.log_toolhead_pos("ACE STN unload after ")

        cur_lane.unsync_to_extruder()

        # ACE serial unwind — retract to hub staging point
        hub = cur_lane.hub_obj
        bowden = getattr(hub, 'afc_unload_bowden_length', getattr(hub, 'afc_bowden_length', 0)) if hub else 0
        retract_dist = bowden - cur_lane.dist_hub
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE unwind failed for {cur_lane.name}: {e}")
            return False

        afc.afcDeltaTime.log_with_time("ACE unwind complete")

        # Filament staged near hub, ready for fast reload.
        self._set_hub_state(cur_lane, False)
        cur_lane.loaded_to_hub = True
        self.lane_tool_unloaded(cur_lane)
        self._hub_load_suppressed.add(cur_lane.name)

        return True

    # ── Calibration commands ────────────────────────────────────────

    cmd_ACE_CALIBRATE_options = {
        "UNIT": {"type": "string", "default": ""},
        "LANE": {"type": "string", "default": ""},
    }

    def cmd_ACE_CALIBRATE(self, gcmd):
        """Calibrate ACE feed distance to toolhead for a lane."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_CALIBRATE LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        success, msg, dist = self.calibrate_bowden(cur_lane, self.calibration_step, 1.0)
        gcmd.respond_info(msg)

    cmd_ACE_CALIBRATE_HUB_options = {
        "UNIT": {"type": "string", "default": ""},
        "LANE": {"type": "string", "default": ""},
    }

    def cmd_ACE_CALIBRATE_HUB(self, gcmd):
        """Calibrate ACE feed distance to hub sensor."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_CALIBRATE_HUB LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        if cur_lane.hub_obj and cur_lane.hub_obj.is_virtual_pin():
            gcmd.respond_info("Hub calibration requires a physical hub sensor, not virtual")
            return
        success, msg, dist = self._calibrate_hub_inner(cur_lane)
        gcmd.respond_info(msg)

    def cmd_ACE_STATUS(self, gcmd):
        """Query and display ACE hardware status."""
        if not self._ace or not self._ace.connected:
            gcmd.respond_info("ACE not connected")
            return
        try:
            status = self._ace.get_status()
            gcmd.respond_info(f"ACE Status: {status}")
        except Exception as e:
            gcmd.respond_info(f"Error querying ACE: {e}")

    cmd_ACE_DRY_options = {
        "UNIT": {"type": "string", "default": ""},
        "TEMP": {"type": "float", "default": 50.0},
        "DURATION": {"type": "float", "default": 90.0},
        "FAN": {"type": "int", "default": 800},
    }

    def cmd_ACE_DRY(self, gcmd):
        """Start ACE filament dryer."""
        temp = gcmd.get_float('TEMP', 50.0)
        duration = gcmd.get_float('DURATION', 90.0)
        fan = gcmd.get_int('FAN', 800)
        if not self._ace or not self._ace.connected:
            gcmd.respond_info("ACE not connected")
            return
        try:
            self._ace.start_drying(temp, fan, duration)
            gcmd.respond_info(f"ACE dryer started: {temp}°C for {duration} min")
        except Exception as e:
            gcmd.respond_info(f"Error starting dryer: {e}")

    def cmd_ACE_DRY_STOP(self, gcmd):
        """Stop ACE filament dryer."""
        if not self._ace or not self._ace.connected:
            gcmd.respond_info("ACE not connected")
            return
        try:
            self._ace.stop_drying()
            gcmd.respond_info("ACE dryer stopped")
        except Exception as e:
            gcmd.respond_info(f"Error stopping dryer: {e}")

    def cmd_ACE_LANE_RESET(self, gcmd):
        """Retract lane filament back into ACE unit."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_LANE_RESET LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        self.eject_lane(cur_lane)
        gcmd.respond_info(f"Lane {lane_name} reset")

    # ── Hardware interaction helpers ────────────────────────────────

    def _start_feed_assist(self, slot: int):
        if slot in self._feed_assist_active:
            return
        if not (self._ace and self._ace.connected):
            return
        # The ACE is single-threaded; a momentary firmware/serial hiccup can
        # push the ACK past the 2s command timeout. start_feed_assist is
        # idempotent (re-enabling an already-on slot is a no-op), so retry on
        # timeout instead of silently leaving the loaded lane with assist OFF.
        attempts = 3
        for attempt in range(1, attempts + 1):
            try:
                self._wait_for_ace_ready()
                self._ace.start_feed_assist(slot)
                self._feed_assist_active.add(slot)
                return
            except ACETimeoutError as e:
                if attempt < attempts:
                    self.logger.debug(
                        f"start feed assist slot {slot} timed out "
                        f"(attempt {attempt}/{attempts}), retrying: {e}")
                    continue
                self.logger.error(
                    f"Failed to start feed assist slot {slot} after "
                    f"{attempts} attempts: {e}")
            except Exception as e:
                # FORBIDDEN means the ACE refused assist for a moment (slot
                # state still settling, e.g. right after boot). The heartbeat
                # watchdog re-issues it on the next status tick and it succeeds,
                # so log at debug — it's transient and self-healing, not a fault.
                if 'FORBIDDEN' in str(e).upper():
                    self.logger.debug(
                        f"Feed assist slot {slot} not permitted yet "
                        f"(FORBIDDEN); watchdog will retry: {e}")
                else:
                    self.logger.error(
                        f"Failed to start feed assist slot {slot}: {e}")
                return

    def _stop_feed_assist(self, slot: int):
        if slot not in self._feed_assist_active:
            return
        if not (self._ace and self._ace.connected):
            return
        # Retry on timeout for the same reason as start: a late ACK shouldn't
        # leave us believing assist is still running on a slot we meant to
        # stop. stop_feed_assist is idempotent too.
        attempts = 3
        for attempt in range(1, attempts + 1):
            try:
                self._wait_for_ace_ready()
                self._ace.stop_feed_assist_sync(slot)
                self._feed_assist_active.discard(slot)
                return
            except ACETimeoutError as e:
                if attempt < attempts:
                    self.logger.debug(
                        f"stop feed assist slot {slot} timed out "
                        f"(attempt {attempt}/{attempts}), retrying: {e}")
                    continue
                self.logger.error(
                    f"Failed to stop feed assist slot {slot} after "
                    f"{attempts} attempts: {e}")
            except Exception as e:
                self.logger.error(f"Failed to stop feed assist slot {slot}: {e}")
                return

    def _active_assist_lane(self):
        # Name of the lane that SHOULD have feed assist: the one loaded on the
        # toolhead extruder currently on the shuttle. None if there isn't one.
        try:
            active_ext = self.printer.lookup_object(
                'toolhead').get_extruder().get_name()
        except Exception:
            return None
        # A lane is the assist target when it is loaded (tool_loaded) AND its
        # extruder is the one on the shuttle. Prefer the lane the extruder
        # records as loaded, but fall back to the (unique, since only one lane
        # per extruder is tool_loaded at a time) loaded lane on that extruder.
        # The fallback matters at print start when a lane is "already loaded":
        # the load sequence is skipped so extruder.lane_loaded can lag behind
        # tool_loaded, which used to make this return None and leave assist off.
        candidate = None
        for lane in self.lanes.values():
            ext_obj = getattr(lane, 'extruder_obj', None)
            if (ext_obj is None
                    or getattr(ext_obj, 'name', None) != active_ext
                    or not getattr(lane, 'tool_loaded', False)):
                continue
            if getattr(ext_obj, 'lane_loaded', None) == lane.name:
                return lane.name
            if candidate is None:
                candidate = lane.name
        return candidate

    def _maybe_assist_watchdog(self):
        # Heartbeat watchdog: if the active tool's lane should be assisted but
        # isn't (or the wrong slot is), reconcile. Idempotent — a no-op once
        # assist is correct, so it only acts on a genuine discrepancy.
        if not self._assist_watchdog:
            return
        name = self._active_assist_lane()
        if name is None:
            return
        slot = self._slot_map.get(name)
        lane = self.afc.lanes.get(name)
        if (slot is not None and lane is not None
                and self._use_feed_assist(lane)
                and self._feed_assist_active != {slot}):
            self.logger.info(
                "ACE assist watchdog: enabling feed assist for %s (slot %d)"
                % (name, slot))
            self.afc.reactor.register_callback(
                lambda et, n=name: self._reconcile_feed_assist(n))

    def _handle_extruder_activated(self):
        # A tool pickup (e.g. grabbing the probe tool during print start)
        # activates that tool's extruder. If one of our lanes is loaded on the
        # now-active extruder, switch feed assist to it. Mode-agnostic.
        name = self._active_assist_lane()
        if name is None:
            return
        lane = self.afc.lanes.get(name)
        if lane is not None and self._use_feed_assist(lane):
            self.afc.reactor.register_callback(
                lambda et, n=name: self._reconcile_feed_assist(n))

    def _wait_for_ace_ready(self, timeout=30.0):
        """Wait for the overall ACE status to be 'ready' before sending commands.

        After a retract/feed completes, the slot may report 'ready'/'empty'
        before the ACE controller finishes internal housekeeping. Sending
        a new feed_filament while the ACE is still busy returns FORBIDDEN.
        """
        ace = self._ace
        if ace is None or not ace.connected:
            return
        poll_interval = 0.5
        elapsed = 0.0
        while elapsed < timeout:
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    if hw_status.get("status", "") == "ready":
                        return
                    self.logger.debug(
                        f"ACE: waiting for ready "
                        f"(status={hw_status.get('status', '?')}, "
                        f"{elapsed:.1f}s/{timeout:.0f}s)")
            except Exception:
                pass
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval)
            elapsed += poll_interval
        self.logger.warning(
            f"ACE: did not become ready within {timeout:.0f}s, proceeding anyway")

    def _wait_for_feed_complete(self, slot_index, length_mm, speed_mm_s,
                                 lane=None, poll_interval=0.5) -> bool:
        """Wait for ACE feed/unwind movement to complete by polling slot status."""
        ace = self._ace
        if ace is None or not ace.connected:
            return False

        max_wait = (length_mm / max(speed_mm_s, 1)) + 10.0
        deadline = self.afc.reactor.monotonic() + max_wait

        # Phase 0: Wait for slot to leave ready/empty (motor starting)
        departure_deadline = self.afc.reactor.monotonic() + 3.0
        motor_started = False
        while self.afc.reactor.monotonic() < departure_deadline:
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            if status not in ("ready", "empty", ""):
                                motor_started = True
                                break
            except Exception:
                pass
        if not motor_started:
            self.logger.debug(
                f"ACE wait: slot {slot_index} never left ready state "
                f"after feed command — motor may not have started")
            return False

        # Phase 1: Wait for slot to return to ready/empty (motor done)
        while self.afc.reactor.monotonic() < deadline:
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval)

            if lane is not None and self._toolhead_sensor_triggered(lane):
                self.logger.debug(
                    f"ACE wait: toolhead sensor triggered for slot {slot_index}")
                return True

            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            if status in ("ready", "empty"):
                                return True
            except Exception:
                pass

        self.logger.debug(
            f"ACE wait: timeout waiting for slot {slot_index} "
            f"movement ({max_wait:.1f}s)")
        return False

    def _smart_load_retry(self, cur_lane, slot, feed_dist, max_retries: int = 3) -> bool:
        """Resend the full feed command when the initial feed stalls or times out."""
        for attempt in range(max_retries):
            self.logger.info(
                f"Feed retry {attempt + 1}/{max_retries} for {cur_lane.name} "
                f"({feed_dist:.0f}mm)")
            try:
                self._wait_for_ace_ready()
                self._ace.feed_filament(slot, feed_dist, self.feed_speed)
                self._wait_for_feed_complete(slot, feed_dist, self.feed_speed, cur_lane)
                if self._toolhead_sensor_triggered(cur_lane):
                    return True
            except Exception:
                pass
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.0)
        return False

    def _calibrate_hub_inner(self, cur_lane) -> tuple:
        """Calibrate dist_hub by incrementally feeding until the hub sensor
        triggers; the first-detection distance is the measurement."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            return False, "ACE not connected", 0

        hub = cur_lane.hub_obj
        if hub is None or hub.is_virtual_pin():
            return False, "Physical hub sensor required for calibration", 0

        if hub.state:
            return False, "Hub sensor already triggered — clear the hub first", 0

        max_distance = 4000
        step = self.calibration_step

        self.logger.raw(
            f'Calibrating dist_hub for {cur_lane.name} '
            f'(max {max_distance}mm in {step}mm steps)')

        # Coarse pass
        coarse_fed = 0.0
        triggered = False
        while coarse_fed < max_distance:
            feed_step = min(step, max_distance - coarse_fed)
            try:
                self._wait_for_ace_ready()
                self._ace.feed_filament(slot, feed_step, self.feed_speed)
                self._wait_for_feed_complete(slot, feed_step, self.feed_speed)
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: feed failed: {e}")
                break
            coarse_fed += feed_step

            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            if hub.state:
                self.logger.info(
                    f"ACE hub calibrate: coarse trigger at {coarse_fed:.1f}mm")
                triggered = True
                break

        if not triggered:
            if coarse_fed > 0:
                try:
                    self._wait_for_ace_ready()
                    self._ace.unwind_filament(slot, coarse_fed, self.retract_speed)
                    self._wait_for_feed_complete(slot, coarse_fed, self.retract_speed)
                except Exception as e:
                    self.logger.error(f"ACE hub calibrate: retract failed: {e}")
            return False, (
                f"Hub sensor did not trigger after {coarse_fed:.0f}mm. "
                "Check filament path and hub sensor wiring."), coarse_fed

        # First detection only: use the trigger distance directly. The fine
        # back-off/re-feed pass was too small a move to reliably trip the
        # sensor and only added error — keep it simple.
        distance = coarse_fed
        self.logger.info(f"ACE hub calibrate: trigger at {distance:.1f}mm")

        # Retract
        retract_dist = max(distance - 50, 0)
        if retract_dist > 0:
            try:
                self._wait_for_ace_ready()
                self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
                self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: retract failed: {e}")

        old_dist = cur_lane.dist_hub
        new_dist = round(distance, 2)
        cal_msg = f'\n dist_hub: New: {new_dist} Old: {old_dist}'
        cur_lane.dist_hub = new_dist
        self.afc.function.ConfigRewrite(cur_lane.fullname, "dist_hub", new_dist, cal_msg)

        return True, f"dist_hub calibration: {new_dist}mm (was {old_dist}mm)", new_dist


    # ── RFID / Spoolman helpers ─────────────────────────────────────

    def _sync_inventory(self):
        """Pull RFID/NFC filament data from ACE hardware into slot cache."""
        if self._ace is None or not self._ace.connected:
            return
        for slot in range(self.SLOTS_PER_UNIT):
            try:
                info = self._ace.get_filament_info(slot)
                if isinstance(info, dict):
                    self._store_slot_rfid(slot, info)
            except Exception as e:
                self.logger.debug(
                    f"ACE {self.name}: slot {slot} inventory query failed: {e}")

    def _store_slot_rfid(self, slot, info):
        """Store RFID fields from a get_filament_info response into slot cache."""
        inv = self._slot_inventory[slot]
        inv["material"] = info.get("material", info.get("type", ""))
        inv["color"] = info.get("color", [0, 0, 0])
        inv["sku"] = info.get("sku", "")
        inv["brand"] = info.get("brand", "")
        inv["diameter"] = info.get("diameter", 1.75)
        inv["total_weight"] = info.get("total", 0)
        inv["current_weight"] = info.get("current", 0)
        ext_temp = info.get("extruder_temp", {})
        bed_temp = info.get("hotbed_temp", {})
        if isinstance(ext_temp, dict) and ext_temp.get("min") and ext_temp.get("max"):
            inv["extruder_temp"] = (ext_temp["min"] + ext_temp["max"]) // 2
        elif isinstance(ext_temp, dict) and ext_temp.get("max"):
            inv["extruder_temp"] = ext_temp["max"]
        else:
            inv["extruder_temp"] = None
        inv["bed_temp"] = bed_temp.get("max") if isinstance(bed_temp, dict) else None

    def _refresh_slot_inventory(self, slot):
        """Fetch fresh RFID data for a single slot from ACE hardware."""
        if self._ace is None or not self._ace.connected:
            return
        if slot < 0 or slot >= self.SLOTS_PER_UNIT:
            return
        try:
            info = self._ace.get_filament_info(slot)
            if isinstance(info, dict):
                self._store_slot_rfid(slot, info)
        except Exception as e:
            self.logger.debug(
                f"ACE {self.name}: slot {slot} RFID refresh failed: {e}")

    def _clear_slot_inventory(self, slot):
        """Clear cached RFID data for a slot."""
        if 0 <= slot < self.SLOTS_PER_UNIT:
            self._slot_inventory[slot]["material"] = ""
            self._slot_inventory[slot]["color"] = [0, 0, 0]

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane load/prep state at startup."""
        if self._ace is None or not self._ace.connected:
            return
        for lane in self.lanes.values():
            slot = self._get_slot(lane.name)
            if 0 <= slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[slot]
                slot_loaded = bool(
                    slot_info and slot_info.get("status", "") == "ready")
                lane._load_state = slot_loaded
                lane.prep_state = slot_loaded

                if apply_filament_defaults is not None:
                    apply_filament_defaults(
                        lane, slot_info,
                        color_converter=rgb_array_to_hex,
                        afc_defaults={
                            "default_material_type": getattr(self.afc, "default_material_type", None),
                            "default_color": getattr(self.afc, "default_color", None),
                        })

                if slot_loaded and sync_rfid_to_spoolman is not None:
                    self._sync_rfid_to_spoolman(lane, slot_info)

    def _sync_rfid_to_spoolman(self, lane, slot_info):
        """Sync ACE RFID tag data to Spoolman."""
        if sync_rfid_to_spoolman is None:
            return
        if self.afc.spoolman is None or self.afc.moonraker is None:
            return
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            return
        sku = slot_info.get("sku", "")
        if not sku:
            return

        color_rgb = slot_info.get("color", [0, 0, 0])
        color_hex = ""
        if rgb_array_to_hex is not None and color_rgb != [0, 0, 0]:
            color_hex = rgb_array_to_hex(color_rgb).lstrip("#")

        normalized = dict(slot_info)
        normalized["color_hex"] = color_hex

        spool_wt = slot_info.get("total_weight", 0)
        allow_create = self._get_auto_spoolman_create(lane)
        sync_rfid_to_spoolman(
            self.afc, lane, normalized, self.logger, "ACE RFID",
            allow_create=allow_create,
            spool_weight=spool_wt if spool_wt > 0 else None)

    def _get_auto_spoolman_create(self, lane):
        """Check if auto Spoolman spool creation is enabled for a lane."""
        if get_auto_spoolman_create is not None:
            return get_auto_spoolman_create(lane, self.auto_spoolman_create)
        return self.auto_spoolman_create


def load_config_prefix(config):
    return afcACE(config)
