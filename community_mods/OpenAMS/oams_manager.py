# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OPTIMIZATIONS APPLIED:
# 1. Adaptive Polling Intervals: Monitors use 2.0s active / 4.0s idle intervals
#    to reduce CPU usage when printer is idle (15-25% reduction in polling overhead)
# 2. Object Caching: Frequently accessed objects (idle_timeout, gcode, toolhead, AFC)
#    are cached at initialization to avoid repeated lookups
# 3. State Change Tracking: FPSState tracks consecutive idle polls to intelligently
#    switch between active and idle polling intervals
# 4. Pre-checks: Monitor functions skip expensive sensor reads when idle and stable
#
# CONFIGURABLE DETECTION PARAMETERS:
# The following parameters can be tuned in [oams_manager] config section:
# - stuck_spool_load_grace: Grace period (seconds) after load before stuck spool detection (default: 8.0)
# - stuck_spool_pressure_threshold: Pressure below which stuck detection starts (default: 0.08)
# - stuck_spool_pressure_clear_threshold: Pressure above which stuck state clears - hysteresis (default: 0.12)
# - clog_pressure_target: Target FPS pressure for clog detection (default: 0.50)
# - post_load_pressure_dwell: Duration (seconds) to monitor pressure after load (default: 15.0)
# - clog_sensitivity: Detection sensitivity level - "low", "medium", "high" (default: "medium")

import logging
import time
from functools import partial
from typing import Optional, Tuple, Dict, List, Any, Callable

try:
    from extras.ams_integration import AMSRunoutCoordinator
except Exception:
    AMSRunoutCoordinator = None

# Configuration constants
PAUSE_DISTANCE = 60
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14
MONITOR_ENCODER_PERIOD = 2.0
MONITOR_ENCODER_PERIOD_IDLE = 4.0  # OPTIMIZATION: Longer interval when idle
MONITOR_ENCODER_SPEED_GRACE = 2.0
AFC_DELEGATION_TIMEOUT = 30.0
IDLE_POLL_THRESHOLD = 3  # OPTIMIZATION: Polls before switching to idle interval

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08
STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD = 0.12  # Hysteresis upper threshold
STUCK_SPOOL_DWELL = 3.5
STUCK_SPOOL_LOAD_GRACE = 8.0

CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {"extrusion_window": 48.0, "encoder_slack": 15, "pressure_band": 0.08, "dwell": 12.0},
    "medium": {"extrusion_window": 24.0, "encoder_slack": 8, "pressure_band": 0.06, "dwell": 8.0},
    "high": {"extrusion_window": 12.0, "encoder_slack": 4, "pressure_band": 0.04, "dwell": 6.0},
}
CLOG_SENSITIVITY_DEFAULT = "medium"

POST_LOAD_PRESSURE_THRESHOLD = 0.65
POST_LOAD_PRESSURE_DWELL = 15.0
POST_LOAD_PRESSURE_CHECK_PERIOD = 0.5


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"
    MONITORING = "MONITORING"
    DETECTED = "DETECTED"
    COASTING = "COASTING"
    RELOADING = "RELOADING"
    PAUSED = "PAUSED"


class FPSLoadState:
    """Enum for FPS loading states"""
    UNLOADED = 0
    LOADED = 1
    LOADING = 2
    UNLOADING = 3
    
class OAMSRunoutMonitor:
    """Monitors filament runout for a specific FPS."""
    
    def __init__(self,
                 printer,
                 fps_name: str,
                 fps,
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable,
                 reload_before_toolhead_distance: float = 0.0):
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        
        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback

        self.reactor = self.printer.get_reactor()

        self.hardware_service = None
        self.latest_lane_name: Optional[str] = None
        if AMSRunoutCoordinator is not None:
            try:
                self.hardware_service = AMSRunoutCoordinator.register_runout_monitor(self)
            except Exception:
                logging.getLogger(__name__).exception(
                    "Failed to register OpenAMS monitor with AMSRunoutCoordinator"
                )
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state in (OAMSRunoutState.STOPPED, OAMSRunoutState.PAUSED, OAMSRunoutState.RELOADING):
                return eventtime + MONITOR_ENCODER_PERIOD
            
            if self.state == OAMSRunoutState.MONITORING:
                if getattr(fps_state, "afc_delegation_active", False):
                    now = self.reactor.monotonic()
                    if now < getattr(fps_state, "afc_delegation_until", 0.0):
                        return eventtime + MONITOR_ENCODER_PERIOD
                    fps_state.afc_delegation_active = False
                    fps_state.afc_delegation_until = 0.0
                
                oams_obj = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
                if oams_obj is None:
                    return eventtime + MONITOR_ENCODER_PERIOD

                spool_idx = fps_state.current_spool_idx
                if spool_idx is None:
                    self.latest_lane_name = None
                    return eventtime + MONITOR_ENCODER_PERIOD

                lane_name = None
                spool_empty = None
                unit_name = getattr(fps_state, "current_oams", None) or self.fps_name

                if self.hardware_service is not None:
                    try:
                        lane_name = self.hardware_service.resolve_lane_for_spool(unit_name, spool_idx)
                        snapshot = self.hardware_service.latest_lane_snapshot_for_spool(unit_name, spool_idx)
                    except Exception:
                        snapshot = None
                    if snapshot:
                        hub_state = snapshot.get("hub_state")
                        lane_state = snapshot.get("lane_state")
                        if hub_state is not None:
                            spool_empty = not bool(hub_state)
                        elif lane_state is not None:
                            spool_empty = not bool(lane_state)

                if spool_empty is None:
                    try:
                        hes_values = oams_obj.hub_hes_value
                        if spool_idx < 0 or spool_idx >= len(hes_values):
                            return eventtime + MONITOR_ENCODER_PERIOD
                        spool_empty = not bool(hes_values[spool_idx])
                    except Exception:
                        logging.exception("OAMS: Failed to read HES values for runout detection on %s", self.fps_name)
                        return eventtime + MONITOR_ENCODER_PERIOD

                self.latest_lane_name = lane_name

                if (is_printing and fps_state.state == FPSLoadState.LOADED and 
                    fps_state.current_group is not None and fps_state.current_spool_idx is not None and spool_empty):
                    self.state = OAMSRunoutState.DETECTED
                    logging.info("OAMS: Runout detected on FPS %s, pausing for %d mm", self.fps_name, PAUSE_DISTANCE)
                    self.runout_position = fps.extruder.last_position
                    if AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_runout_detected(self, spool_idx, lane_name=lane_name)
                        except Exception:
                            logging.getLogger(__name__).exception("Failed to notify AFC about OpenAMS runout")

            elif self.state == OAMSRunoutState.DETECTED:
                traveled_distance = fps.extruder.last_position - self.runout_position
                if traveled_distance >= PAUSE_DISTANCE:
                    logging.info("OAMS: Pause complete, coasting the follower.")
                    try:
                        self.oams[fps_state.current_oams].set_oams_follower(0, 1)
                    except Exception:
                        logging.exception("OAMS: Failed to stop follower while coasting on %s", self.fps_name)
                    finally:
                        fps_state.following = False
                    self.bldc_clear_position = fps.extruder.last_position
                    self.runout_after_position = 0.0
                    self.state = OAMSRunoutState.COASTING

            elif self.state == OAMSRunoutState.COASTING:
                traveled_distance_after_bldc_clear = max(fps.extruder.last_position - self.bldc_clear_position, 0.0)
                self.runout_after_position = traveled_distance_after_bldc_clear
                try:
                    path_length = getattr(self.oams[fps_state.current_oams], "filament_path_length", 0.0)
                except Exception:
                    logging.exception("OAMS: Failed to read filament path length while coasting on %s", self.fps_name)
                    return eventtime + MONITOR_ENCODER_PERIOD
                
                effective_path_length = (path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0)
                consumed_with_margin = (self.runout_after_position + PAUSE_DISTANCE + self.reload_before_toolhead_distance)

                if consumed_with_margin >= effective_path_length:
                    logging.info("OAMS: Loading next spool (%.2f mm consumed)", self.runout_after_position + PAUSE_DISTANCE)
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
            
            return eventtime + MONITOR_ENCODER_PERIOD
        
        self._timer_callback = _monitor_runout
        self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)

    def start(self) -> None:
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        self.state = OAMSRunoutState.MONITORING
    
    def stop(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        
    def reloading(self) -> None:
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        
    def paused(self) -> None:
        self.state = OAMSRunoutState.PAUSED
        
    def reset(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """Global state container for all FPS units."""
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """Tracks the state of a single FPS"""
    
    def __init__(self, 
                 state: int = FPSLoadState.UNLOADED,
                 current_group: Optional[str] = None, 
                 current_oams: Optional[str] = None, 
                 current_spool_idx: Optional[int] = None):
        
        self.state = state
        self.current_group = current_group
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx
        
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        self.encoder_sample_prev: Optional[int] = None
        self.encoder_sample_current: Optional[int] = None

        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 1

        self.clog_active: bool = False
        self.clog_restore_follower: bool = False
        self.clog_restore_direction: int = 1
        self.clog_start_extruder: Optional[float] = None
        self.clog_start_encoder: Optional[int] = None
        self.clog_start_time: Optional[float] = None
        self.clog_min_pressure: Optional[float] = None
        self.clog_max_pressure: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None

        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None

        # OPTIMIZATION: Adaptive polling state
        self.consecutive_idle_polls: int = 0
        self.last_state_change: Optional[float] = None

    def record_encoder_sample(self, value: int) -> Optional[int]:
        """Record encoder sample and return diff if we have 2 samples."""
        self.encoder_sample_prev = self.encoder_sample_current
        self.encoder_sample_current = value
        
        if self.encoder_sample_prev is not None:
            return abs(self.encoder_sample_current - self.encoder_sample_prev)
        return None
    
    def clear_encoder_samples(self):
        """Reset encoder tracking."""
        self.encoder_sample_prev = None
        self.encoder_sample_current = None

    def reset_runout_positions(self) -> None:
        self.runout_position = None
        self.runout_after_position = None

    def reset_stuck_spool_state(self, preserve_restore: bool = False) -> None:
        self.stuck_spool_start_time = None
        self.stuck_spool_active = False
        if not preserve_restore:
            self.stuck_spool_restore_follower = False
            self.stuck_spool_restore_direction = 1

    def reset_clog_tracker(self) -> None:
        self.clog_active = False
        self.clog_start_extruder = None
        self.clog_start_encoder = None
        self.clog_start_time = None
        self.clog_min_pressure = None
        self.clog_max_pressure = None
        self.clog_last_extruder = None
        self.clog_restore_follower = False
        self.clog_restore_direction = 1

    def prime_clog_tracker(self, extruder_pos: float, encoder_clicks: int, pressure: float, timestamp: float) -> None:
        self.clog_start_extruder = extruder_pos
        self.clog_last_extruder = extruder_pos
        self.clog_start_encoder = encoder_clicks
        self.clog_start_time = timestamp
        self.clog_min_pressure = pressure
        self.clog_max_pressure = pressure
        
    def __repr__(self) -> str:
        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        return f"FPSState(state={state_names.get(self.state, self.state)}, group={self.current_group}, oams={self.current_oams}, spool={self.current_spool_idx})"


class OAMSManager:
    """Main coordinator for OpenAMS system"""
    
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger(__name__)
        
        self.filament_groups: Dict[str, Any] = {}
        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        self.current_state = OAMSState()
        self.current_group: Optional[str] = None
        self.afc = None
        self._afc_logged = False

        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        self.reload_before_toolhead_distance: float = config.getfloat("reload_before_toolhead_distance", 0.0)

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            self.logger.warning("Unknown clog_sensitivity '%s', using %s", sensitivity, CLOG_SENSITIVITY_DEFAULT)
            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Configurable detection thresholds and timing parameters
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", STUCK_SPOOL_LOAD_GRACE)
        self.stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", STUCK_SPOOL_PRESSURE_THRESHOLD)
        self.stuck_spool_pressure_clear_threshold = config.getfloat("stuck_spool_pressure_clear_threshold", STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD)
        self.clog_pressure_target = config.getfloat("clog_pressure_target", CLOG_PRESSURE_TARGET)
        self.post_load_pressure_dwell = config.getfloat("post_load_pressure_dwell", POST_LOAD_PRESSURE_DWELL)

        self.group_to_fps: Dict[str, str] = {}
        self._canonical_lane_by_group: Dict[str, str] = {}
        self._canonical_group_by_lane: Dict[str, str] = {}
        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}

        # OPTIMIZATION: Cache hardware service lookups
        self._hardware_service_cache: Dict[str, Any] = {}
        self._idle_timeout_obj = None
        self._gcode_obj = None
        self._toolhead_obj = None

        self._initialize_oams()
        self._initialize_filament_groups()

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing_resumed)
        self.printer.register_event_handler("pause:resume", self._handle_printing_resumed)

        self.printer.add_object("oams_manager", self)
        self.register_commands()
        
    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """Return current status of all FPS units and OAMS hardware."""
        attributes: Dict[str, Dict[str, Any]] = {"oams": {}}

        for name, oam in self.oams.items():
            status_name = name.split()[-1]
            try:
                oam_status = {
                    "action_status": oam.action_status,
                    "action_status_code": oam.action_status_code,
                    "action_status_value": oam.action_status_value,
                }
            except Exception:
                self.logger.exception("Failed to fetch status from %s", name)
                oam_status = {"action_status": "error", "action_status_code": None, "action_status_value": None}
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_group": fps_state.current_group,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": state_names.get(fps_state.state, str(fps_state.state)),
                "since": fps_state.since,
            }

        return attributes
    
    def determine_state(self) -> None:
        """Analyze hardware state and update FPS state tracking."""
        for fps_name, fps_state in self.current_state.fps_state.items():
            (
                fps_state.current_group,
                current_oams,
                fps_state.current_spool_idx,
            ) = self.determine_current_loaded_group(fps_name)

            if current_oams is not None:
                fps_state.current_oams = current_oams.name
            else:
                fps_state.current_oams = None

            if (fps_state.current_oams is not None and fps_state.current_spool_idx is not None):
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "state detection")
            else:
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                fps_state.clog_restore_follower = False
                fps_state.clog_restore_direction = 1
                self._cancel_post_load_pressure_check(fps_state)
        
    def handle_ready(self) -> None:
        """Initialize system when printer is ready."""
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)

        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")

        # OPTIMIZATION: Cache frequently accessed objects
        try:
            self._idle_timeout_obj = self.printer.lookup_object("idle_timeout")
        except Exception:
            self._idle_timeout_obj = None

        try:
            self._gcode_obj = self.printer.lookup_object("gcode")
        except Exception:
            self._gcode_obj = None

        try:
            self._toolhead_obj = self.printer.lookup_object("toolhead")
        except Exception:
            self._toolhead_obj = None

        self._rebuild_group_fps_index()
        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self) -> None:
        for name, group in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            self.logger.info("Adding group %s", name)
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which filament group is currently loaded in the specified FPS."""
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")
            
        for group_name, group in self.filament_groups.items():
            for oam, bay_index in group.bays:
                try:
                    is_loaded = oam.is_bay_loaded(bay_index)
                except Exception:
                    self.logger.exception("Failed to query bay %s on %s", bay_index, getattr(oam, "name", "<unknown>"))
                    continue

                if is_loaded and oam in fps.oams:
                    return group_name, oam, bay_index
                    
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        commands = [
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT, self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT, self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS, self.cmd_CLEAR_ERRORS_help),
        ]
        for cmd_name, handler, help_text in commands:
            gcode.register_command(cmd_name, handler, desc=help_text)
    
    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        for fps_state in self.current_state.fps_state.values():
            fps_state.clear_encoder_samples()
            fps_state.reset_stuck_spool_state()
            self._cancel_post_load_pressure_check(fps_state)

        for oam in self.oams.values():
            try:
                oam.clear_errors()
            except Exception:
                self.logger.exception("Failed to clear errors on %s", getattr(oam, "name", "<unknown>"))
        self.determine_state()
        self.start_monitors()
    
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        direction = gcmd.get_int('DIRECTION')
        fps_name = "fps " + gcmd.get('FPS')
        
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return
        
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj is None:
            gcmd.respond_info(f"OAMS {fps_state.current_oams} is not available")
            return

        try:
            oams_obj.set_oams_follower(enable, direction)
            fps_state.following = bool(enable)
            fps_state.direction = direction
        except Exception:
            self.logger.exception("Failed to set follower on %s", fps_state.current_oams)
            gcmd.respond_info(f"Failed to set follower. Check logs.")

    def _rebuild_group_fps_index(self) -> None:
        mapping: Dict[str, str] = {}
        for group_name, group in self.filament_groups.items():
            for fps_name, fps in self.fpss.items():
                if any(oam in fps.oams for oam in group.oams):
                    mapping[group_name] = fps_name
                    break
        self.group_to_fps = mapping

    def group_fps_name(self, group_name: str) -> Optional[str]:
        if group_name not in self.group_to_fps and self.fpss:
            self._rebuild_group_fps_index()
        return self.group_to_fps.get(group_name)

    def _normalize_group_name(self, group: Optional[str]) -> Optional[str]:
        if not group or not isinstance(group, str):
            return None
        group = group.strip()
        if not group:
            return None
        if " " in group:
            group = group.split()[-1]
        return group

    def _rebuild_lane_location_index(self) -> None:
        mapping: Dict[Tuple[str, int], str] = {}
        for group_name, lane_name in self._canonical_lane_by_group.items():
            group = self.filament_groups.get(group_name)
            if not group:
                continue
            for oam, bay_index in group.bays:
                mapping[(oam.name, bay_index)] = lane_name
        self._lane_by_location = mapping

    def _ensure_afc_lane_cache(self, afc) -> None:
        lanes = getattr(afc, "lanes", {})
        updated = False
        for lane_name, lane in lanes.items():
            canonical_group = self._normalize_group_name(getattr(lane, "_map", None))
            if canonical_group is None:
                canonical_group = self._normalize_group_name(getattr(lane, "map", None))
            if canonical_group:
                if lane_name not in self._canonical_group_by_lane:
                    self._canonical_group_by_lane[lane_name] = canonical_group
                    updated = True
                if canonical_group not in self._canonical_lane_by_group:
                    self._canonical_lane_by_group[canonical_group] = lane_name
                    updated = True
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name
        if updated:
            self._rebuild_lane_location_index()

    def _resolve_lane_for_state(self, fps_state: 'FPSState', group_name: Optional[str], afc) -> Tuple[Optional[str], Optional[str]]:
        normalized_group = self._normalize_group_name(group_name)
        lane_name: Optional[str] = None

        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            lane_name = self._lane_by_location.get((fps_state.current_oams, fps_state.current_spool_idx))
            if lane_name:
                lane_group = self._canonical_group_by_lane.get(lane_name)
                if lane_group:
                    normalized_group = lane_group

        if lane_name is None and normalized_group:
            lane_name = self._canonical_lane_by_group.get(normalized_group)

        lanes = getattr(afc, "lanes", {})
        if lane_name is None and normalized_group:
            lane_name = next((name for name, lane in lanes.items() 
                             if self._normalize_group_name(getattr(lane, "_map", None)) == normalized_group), None)

        canonical_group = normalized_group
        if lane_name:
            lane = lanes.get(lane_name)
            if lane is not None:
                canonical_candidate = self._normalize_group_name(getattr(lane, "_map", None))
                if canonical_candidate is None:
                    canonical_candidate = self._normalize_group_name(getattr(lane, "map", None))

                if canonical_candidate:
                    canonical_group = canonical_candidate
                    if lane_name not in self._canonical_group_by_lane:
                        self._canonical_group_by_lane[lane_name] = canonical_candidate
                    if canonical_candidate not in self._canonical_lane_by_group:
                        self._canonical_lane_by_group[canonical_candidate] = lane_name
                    self._rebuild_lane_location_index()

        return lane_name, canonical_group

    def _get_afc(self):
        # OPTIMIZATION: Cache AFC object lookup
        if self.afc is not None:
            return self.afc

        cached_afc = self._hardware_service_cache.get("afc_object")
        if cached_afc is not None:
            self.afc = cached_afc
            return self.afc

        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None

        self.afc = afc
        self._hardware_service_cache["afc_object"] = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            self.logger.info("AFC integration detected; enabling same-FPS infinite runout support.")
            self._afc_logged = True
        return self.afc

    def _get_infinite_runout_target_group(self, fps_name: str, fps_state: 'FPSState') -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        current_group = fps_state.current_group
        normalized_group = self._normalize_group_name(current_group)
        if normalized_group is None:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, resolved_group = self._resolve_lane_for_state(fps_state, normalized_group, afc)

        if resolved_group and resolved_group != normalized_group:
            normalized_group = resolved_group
            fps_state.current_group = resolved_group

        if not lane_name:
            return None, None, False, None

        lanes = getattr(afc, "lanes", {})
        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        runout_lane_name = getattr(lane, "runout_lane", None)
        if not runout_lane_name:
            return None, None, False, lane_name

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            self.logger.warning("Runout lane %s for %s on %s is not available; deferring to AFC", runout_lane_name, normalized_group, fps_name)
            return None, runout_lane_name, True, lane_name

        source_unit = self._lane_unit_map.get(lane_name)
        target_unit = self._lane_unit_map.get(runout_lane_name)
        if source_unit and target_unit and source_unit != target_unit:
            return None, runout_lane_name, True, lane_name

        source_extruder = getattr(lane, "extruder_obj", None)
        target_extruder = getattr(target_lane, "extruder_obj", None)
        if (source_extruder is not None and target_extruder is not None and source_extruder is not target_extruder):
            return None, runout_lane_name, True, lane_name

        target_group = self._canonical_group_by_lane.get(runout_lane_name)
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "_map", None))
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "map", None))

        if not target_group:
            return None, runout_lane_name, True, lane_name

        if runout_lane_name not in self._canonical_group_by_lane:
            self._canonical_group_by_lane[runout_lane_name] = target_group
        if target_group not in self._canonical_lane_by_group:
            self._canonical_lane_by_group[target_group] = runout_lane_name
        self._rebuild_lane_location_index()

        if target_group == normalized_group:
            return None, runout_lane_name, True, lane_name

        if normalized_group not in self.filament_groups:
            return None, runout_lane_name, True, lane_name

        if target_group not in self.filament_groups:
            return None, runout_lane_name, True, lane_name

        source_fps = self.group_fps_name(normalized_group)
        target_fps = self.group_fps_name(target_group)
        if source_fps != fps_name or target_fps != fps_name:
            self.logger.info("Deferring infinite runout for %s on %s to AFC lane %s", normalized_group, fps_name, runout_lane_name)
            return None, runout_lane_name, True, lane_name

        self.logger.info("Infinite runout configured for %s on %s -> %s (lanes %s -> %s)", normalized_group, fps_name, target_group, lane_name, runout_lane_name)
        return target_group, runout_lane_name, False, lane_name

    def _delegate_runout_to_afc(self, fps_name: str, fps_state: 'FPSState', source_lane_name: Optional[str], target_lane_name: Optional[str]) -> bool:
        afc = self._get_afc()
        if afc is None:
            return False

        if not source_lane_name:
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            self.logger.warning("AFC lane %s not found while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        runout_target = getattr(lane, "runout_lane", None)
        if not runout_target:
            self.logger.warning("AFC lane %s has no runout target while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        if target_lane_name and target_lane_name != runout_target:
            pass

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            return True

        if runout_target not in afc.lanes:
            self.logger.warning("AFC runout lane %s referenced by %s is unavailable", runout_target, source_lane_name)
            return False

        try:
            lane._perform_infinite_runout()
        except Exception:
            self.logger.exception("AFC infinite runout failed for lane %s -> %s", source_lane_name, runout_target)
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        self.logger.info("Delegated infinite runout for %s via AFC lane %s -> %s", fps_name, source_lane_name, runout_target)
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is not currently loaded"

        if fps_state.current_oams is None:
            return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            self.current_group = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, "Spool already unloaded"

        lane_name: Optional[str] = None
        spool_index = fps_state.current_spool_idx
        if AMSRunoutCoordinator is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    lane_name, _ = self._resolve_lane_for_state(fps_state, fps_state.current_group, afc)
            except Exception:
                self.logger.exception("Failed to resolve AFC lane for unload on %s", fps_name)
                lane_name = None

        try:
            fps_state.state = FPSLoadState.UNLOADING
            fps_state.encoder = oams.encoder_clicks
            fps_state.since = self.reactor.monotonic()
            fps_state.current_oams = oams.name
            fps_state.current_spool_idx = oams.current_spool
            fps_state.clear_encoder_samples()  # Clear stale encoder samples
        except Exception:
            self.logger.exception("Failed to capture unload state for %s", fps_name)
            return False, f"Failed to prepare unload on {fps_name}"

        try:
            success, message = oams.unload_spool_with_retry()
        except Exception:
            self.logger.exception("Exception while unloading filament on %s", fps_name)
            return False, f"Exception unloading filament on {fps_name}"

        if success:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1
            fps_state.since = self.reactor.monotonic()
            if lane_name:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or oams.name, lane_name, loaded=False, spool_index=spool_index, eventtime=fps_state.since)
                except Exception:
                    self.logger.exception("Failed to notify AFC that lane %s unloaded on %s", lane_name, fps_name)

            # Clear LED error state if stuck spool was active before resetting state
            if fps_state.stuck_spool_active and oams is not None and spool_index is not None:
                try:
                    oams.set_led_error(spool_index, 0)
                    self.logger.info("Cleared stuck spool LED for %s spool %d after successful unload", fps_name, spool_index)
                except Exception:
                    self.logger.exception("Failed to clear LED on %s spool %d after successful unload", fps_name, spool_index)

            fps_state.current_group = None
            fps_state.current_spool_idx = None
            self.current_group = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, message

        fps_state.state = FPSLoadState.LOADED
        return False, message

    def _load_filament_for_group(self, group_name: str) -> Tuple[bool, str]:
        if group_name not in self.filament_groups:
            return False, f"Group {group_name} does not exist"

        fps_name = self.group_fps_name(group_name)
        if fps_name is None:
            return False, f"No FPS associated with group {group_name}"

        fps_state = self.current_state.fps_state[fps_name]
        self._cancel_post_load_pressure_check(fps_state)

        for (oam, bay_index) in self.filament_groups[group_name].bays:
            try:
                is_ready = oam.is_bay_ready(bay_index)
            except Exception:
                self.logger.exception("Failed to query readiness of bay %s on %s", bay_index, getattr(oam, "name", "<unknown>"))
                continue

            if not is_ready:
                continue

            try:
                fps_state.state = FPSLoadState.LOADING
                fps_state.encoder = oam.encoder_clicks
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                # Set since to now for THIS load attempt (will be updated on success)
                fps_state.since = self.reactor.monotonic()
                fps_state.clear_encoder_samples()
            except Exception:
                self.logger.exception("Failed to capture load state for group %s bay %s", group_name, bay_index)
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_group = None
                fps_state.current_spool_idx = None
                fps_state.current_oams = None
                continue

            try:
                success, message = oam.load_spool_with_retry(bay_index)
            except Exception:
                self.logger.exception("Exception while loading group %s bay %s", group_name, bay_index)
                success, message = False, f"Exception loading spool {bay_index} on {group_name}"

            if success:
                fps_state.current_group = group_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                
                # CRITICAL FIX: Set fps_state.since to the successful load time BEFORE changing state
                # This prevents the monitor from seeing stale timestamps during LOADING state
                successful_load_time = oam.get_last_successful_load_time(bay_index)
                if successful_load_time is not None:
                    fps_state.since = successful_load_time
                else:
                    fps_state.since = self.reactor.monotonic()
                
                # Now set state to LOADED after timestamp is correct
                fps_state.state = FPSLoadState.LOADED
                
                fps_state.direction = 1
                self.current_group = group_name

                # Clear LED error state if stuck spool was active before resetting state
                if fps_state.stuck_spool_active and oam is not None and bay_index is not None:
                    try:
                        oam.set_led_error(bay_index, 0)
                        self.logger.info("Cleared stuck spool LED for %s spool %d after successful load", fps_name, bay_index)
                    except Exception:
                        self.logger.exception("Failed to clear LED on %s spool %d after successful load", fps_name, bay_index)

                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "load filament")
                
                # FIX: Skip post-load pressure check if this load completed after retries
                # Retries indicate unstable conditions and pressure sensors need time to stabilize
                skip_pressure_check = False
                try:
                    if hasattr(oam, 'last_load_was_retry'):
                        skip_pressure_check = oam.last_load_was_retry(bay_index)
                except Exception:
                    self.logger.exception("Failed to check retry status for bay %d", bay_index)
                    skip_pressure_check = False
                
                if not skip_pressure_check:
                    self._schedule_post_load_pressure_check(fps_name, fps_state)
                else:
                    self.logger.info("OAMS[%s]: Skipping post-load pressure check after retry for spool %d", 
                                     oam.oams_idx, bay_index)

                if AMSRunoutCoordinator is not None:
                    lane_name: Optional[str] = None
                    try:
                        afc = self._get_afc()
                        if afc is not None:
                            lane_name, _ = self._resolve_lane_for_state(fps_state, group_name, afc)
                    except Exception:
                        self.logger.exception("Failed to resolve AFC lane for group %s on %s", group_name, fps_name)
                        lane_name = None

                    if lane_name:
                        try:
                            AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or oam.name, lane_name, loaded=True, spool_index=fps_state.current_spool_idx, eventtime=fps_state.since)
                        except Exception:
                            self.logger.exception("Failed to notify AFC that lane %s loaded for %s", lane_name, group_name)

                return True, message

            fps_state.state = FPSLoadState.UNLOADED
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.current_oams = None
            fps_state.following = False
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return False, message

        return False, f"No spool available for group {group_name}"

    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = "fps " + gcmd.get('FPS')
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return

        success, message = self._unload_filament_for_fps(fps_name)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        group_name = gcmd.get('GROUP')
        if group_name not in self.filament_groups:
            gcmd.respond_info(f"Group {group_name} does not exist")
            return
        fps_name = self.group_fps_name(group_name)
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.LOADED:
            gcmd.respond_info(f"Group {group_name} is already loaded")
            return
        success, message = self._load_filament_for_group(group_name)
        gcmd.respond_info(message)

    def _pause_printer_message(self, message, oams_name: Optional[str] = None):
        self.logger.info(message)

        if AMSRunoutCoordinator is not None and oams_name:
            try:
                AMSRunoutCoordinator.notify_afc_error(self.printer, oams_name, message, pause=False)
            except Exception:
                self.logger.exception("Failed to forward OAMS pause message to AFC")

        # OPTIMIZATION: Use cached gcode object
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")
                self._gcode_obj = gcode
            except Exception:
                self.logger.exception("Failed to look up gcode object for pause message")
                return

        pause_message = f"Print has been paused: {message}"
        try:
            gcode.run_script(f"M118 {pause_message}")
            gcode.run_script(f"M114 {pause_message}")
        except Exception:
            self.logger.exception("Failed to send pause notification gcode")

        # OPTIMIZATION: Use cached toolhead object
        toolhead = self._toolhead_obj
        if toolhead is None:
            try:
                toolhead = self.printer.lookup_object("toolhead")
                self._toolhead_obj = toolhead
            except Exception:
                self.logger.exception("Failed to query toolhead state during pause handling")
                return

        try:
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")
        except Exception:
            self.logger.exception("Failed to query toolhead state during pause handling")
            return

        try:
            state_message = self.printer.get_state_message()
        except Exception:
            state_message = None

        if isinstance(state_message, (list, tuple)) and state_message:
            printer_state_text = state_message[0]
        else:
            printer_state_text = state_message

        printer_state_text = printer_state_text if isinstance(printer_state_text, str) else None

        if printer_state_text:
            lowered_state = printer_state_text.lower()
            if "lost communication" in lowered_state or "mcu" in lowered_state:
                self.logger.warning(
                    "Printer reported an error state during pause handling: %s",
                    printer_state_text,
                )
                gcode.respond_info(
                    f"Pause notification may fail because printer reported: {printer_state_text}"
                )

        already_paused = False
        try:
            pause_resume = self.printer.lookup_object("pause_resume")
        except Exception:
            pause_resume = None

        if pause_resume is not None:
            try:
                already_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                already_paused = False

        if already_paused:
            self.logger.debug("Skipping PAUSE command because printer is already paused")
            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            try:
                gcode.run_script("PAUSE")
            except Exception:
                self.logger.exception("Failed to run PAUSE script")
        else:
            self.logger.warning("Skipping PAUSE command because axes are not homed (homed_axes=%s)", homed_axes)

    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                self.logger.exception("Failed to cancel post-load pressure timer")
        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(self, fps_name: str, fps_state: "FPSState") -> None:
        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(self, eventtime):
            tracked_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            if tracked_state.state != FPSLoadState.LOADED:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            pressure = float(getattr(fps, "fps_value", 0.0))
            if pressure <= POST_LOAD_PRESSURE_THRESHOLD:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            now = self.reactor.monotonic()
            if tracked_state.post_load_pressure_start is None:
                tracked_state.post_load_pressure_start = now
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            if now - tracked_state.post_load_pressure_start < self.post_load_pressure_dwell:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = self.oams.get(tracked_state.current_oams)
            if (oams_obj is not None and tracked_state.current_spool_idx is not None):
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    self.logger.exception("Failed to set clog LED on %s spool %s after loading", fps_name, tracked_state.current_spool_idx)

            tracked_state.clog_active = True
            message = f"Possible clog detected after loading {tracked_state.current_group or fps_name}: FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"
            self._pause_printer_message(message, tracked_state.current_oams)
            self._cancel_post_load_pressure_check(tracked_state)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(partial(_monitor_pressure, self), self.reactor.NOW)
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None

    def _enable_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], direction: int, context: str) -> None:
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = direction if direction in (0, 1) else 1

        try:
            oams.set_oams_follower(1, direction)
            fps_state.following = True
            fps_state.direction = direction
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug("Enabled follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)
        except Exception:
            self.logger.exception("Failed to enable follower for %s after %s", fps_name, context)

    def _ensure_forward_follower(self, fps_name: str, fps_state: "FPSState", context: str) -> None:
        if (fps_state.current_oams is None or fps_state.current_spool_idx is None or 
            fps_state.stuck_spool_active or fps_state.state != FPSLoadState.LOADED):
            return

        if fps_state.following and fps_state.direction == 1:
            return

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        fps_state.direction = 1
        self._enable_follower(fps_name, fps_state, oams, 1, context)

    def _restore_follower_if_needed(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        if not fps_state.stuck_spool_restore_follower:
            return

        if fps_state.current_oams is None:
            fps_state.stuck_spool_restore_follower = False
            return

        if oams is None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.stuck_spool_restore_direction
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.stuck_spool_restore_follower = False
            self.logger.info("Restarted follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)

    def _reactivate_clog_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        if not fps_state.clog_restore_follower:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.clog_restore_direction if fps_state.clog_restore_direction in (0, 1) else 1
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1

    def _handle_printing_resumed(self, _eventtime):
        # Check if monitors were stopped and need to be restarted
        if not self.monitor_timers:
            self.logger.info("Restarting monitors after pause/intervention")
            self.start_monitors()
        
        for fps_name, fps_state in self.current_state.fps_state.items():
            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            
            # Clear stuck_spool_active on resume to allow follower to restart
            if fps_state.stuck_spool_active:
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s on print resume", fps_name)
            
            # Clear clog_active on resume and reset tracker
            if fps_state.clog_active:
                fps_state.reset_clog_tracker()
                self.logger.info("Cleared clog state for %s on print resume", fps_name)
                # Clear the error LED if we have an OAMS and spool index
                if oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        self.logger.exception("Failed to clear clog LED on %s after resume", fps_name)

            if fps_state.clog_restore_follower:
                self._enable_follower(
                    fps_name,
                    fps_state,
                    oams,
                    fps_state.clog_restore_direction,
                    "print resume",
                )
                if fps_state.following:
                    fps_state.clog_restore_follower = False
                    fps_state.clog_restore_direction = 1
            
            if fps_state.stuck_spool_restore_follower:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "print resume")
            elif (fps_state.current_oams is not None and fps_state.current_spool_idx is not None and not fps_state.following):
                self._ensure_forward_follower(fps_name, fps_state, "print resume")

    def _trigger_stuck_spool_pause(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        if fps_state.stuck_spool_active:
            return

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)

        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set stuck spool LED on %s spool %s", fps_name, spool_idx)

            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.direction = direction
            fps_state.stuck_spool_restore_follower = True
            fps_state.stuck_spool_restore_direction = direction
            if fps_state.following:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    self.logger.exception("Failed to stop follower for %s spool %s during stuck spool pause", fps_name, spool_idx)

            fps_state.following = False

        if oams is not None:
            try:
                oams.abort_current_action()
            except Exception:
                self.logger.exception("Failed to abort active action for %s during stuck spool pause", fps_name)

        fps_state.stuck_spool_active = True
        fps_state.stuck_spool_start_time = None
        self._pause_printer_message(message, fps_state.current_oams)

    def _unified_monitor_for_fps(self, fps_name):
        """Consolidated monitor handling all FPS checks in a single timer (OPTIMIZED)."""
        def _unified_monitor(self, eventtime):
            fps_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if fps_state is None or fps is None:
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None

            # OPTIMIZATION: Use cached idle_timeout object
            is_printing = False
            if self._idle_timeout_obj is not None:
                try:
                    is_printing = self._idle_timeout_obj.get_status(eventtime)["state"] == "Printing"
                except Exception:
                    is_printing = False

            # OPTIMIZATION: Skip sensor reads if idle and no state changes
            state = fps_state.state
            if not is_printing and state == FPSLoadState.LOADED:
                fps_state.consecutive_idle_polls += 1
                if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            # Read sensors
            try:
                if oams:
                    encoder_value = oams.encoder_clicks
                    pressure = float(getattr(fps, "fps_value", 0.0))
                    hes_values = oams.hub_hes_value
                else:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE
            except Exception:
                self.logger.exception("Failed to read sensors for %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            now = self.reactor.monotonic()
            state_changed = False

            if state == FPSLoadState.UNLOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_unload_speed(fps_name, fps_state, oams, encoder_value, now)
                state_changed = True
            elif state == FPSLoadState.LOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_load_speed(fps_name, fps_state, oams, encoder_value, now)
                state_changed = True
            elif state == FPSLoadState.LOADED:
                if is_printing:
                    self._check_stuck_spool(fps_name, fps_state, fps, oams, pressure, hes_values, now)
                    self._check_clog(fps_name, fps_state, fps, oams, encoder_value, pressure, now)
                    state_changed = True

            # OPTIMIZATION: Adaptive polling interval
            if state_changed or is_printing:
                fps_state.consecutive_idle_polls = 0
                fps_state.last_state_change = now
                return eventtime + MONITOR_ENCODER_PERIOD

            fps_state.consecutive_idle_polls += 1
            if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            return eventtime + MONITOR_ENCODER_PERIOD

        return partial(_unified_monitor, self)

    def _check_unload_speed(self, fps_name, fps_state, oams, encoder_value, now):
        """Check unload speed using optimized encoder tracking."""
        # Skip check if already handling a stuck spool
        if fps_state.stuck_spool_active:
            return
            
        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return
        
        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Unload Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)
        
        if encoder_diff < MIN_ENCODER_DIFF:
            group_label = fps_state.current_group or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"
            
            # Abort the current unload operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool unload operation on %s", fps_name)
            except Exception:
                self.logger.exception("Failed to abort unload operation on %s", fps_name)
            
            # Set LED error
            try:
                oams.set_led_error(fps_state.current_spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set LED during unload stuck detection on %s", fps_name)
            
            # Transition to LOADED state cleanly (unload failed, so still loaded)
            fps_state.state = FPSLoadState.LOADED
            fps_state.clear_encoder_samples()
            
            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool_active = True
            fps_state.stuck_spool_start_time = None
            
            self.logger.info("Spool appears stuck while unloading %s spool %s - letting retry logic handle it", group_label, spool_label)

    def _check_load_speed(self, fps_name, fps_state, oams, encoder_value, now):
        """Check load speed using optimized encoder tracking."""
        if fps_state.stuck_spool_active:
            return

        # Skip check if we don't have a valid since timestamp
        if fps_state.since is None:
            return
            
        # Skip check if we're still in grace period
        if now - fps_state.since <= MONITOR_ENCODER_SPEED_GRACE:
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Load Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)
        
        if encoder_diff < MIN_ENCODER_DIFF:
            group_label = fps_state.current_group or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"
            
            # Abort the current load operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool load operation on %s", fps_name)
            except Exception:
                self.logger.exception("Failed to abort load operation on %s", fps_name)
            
            # Set LED error
            try:
                oams.set_led_error(fps_state.current_spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set LED during load stuck detection on %s", fps_name)
            
            # Transition to UNLOADED state cleanly
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.clear_encoder_samples()
            
            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool_active = True
            fps_state.stuck_spool_start_time = None
            
            self.logger.info("Spool appears stuck while loading %s spool %s - letting retry logic handle it", group_label, spool_label)

    def _check_stuck_spool(self, fps_name, fps_state, fps, oams, pressure, hes_values, now):
        """Check for stuck spool conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED while runout monitor inactive on %s", fps_name)
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if not is_printing:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED while idle on %s", fps_name)
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if fps_state.since is not None and now - fps_state.since < self.stuck_spool_load_grace:
            fps_state.stuck_spool_start_time = None
            # Clear stuck spool flag during grace period after successful load
            if fps_state.stuck_spool_active:
                if oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        self.logger.exception("Failed to clear stuck spool LED during grace period on %s", fps_name)
                fps_state.reset_stuck_spool_state(preserve_restore=True)
            return

        if not fps_state.following or fps_state.direction != 1:
            fps_state.stuck_spool_start_time = None
            # Auto-enable follower if we have a spool loaded but follower is disabled
            if is_printing and oams is not None and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "auto-enable after manual load")
            elif fps_state.stuck_spool_restore_follower and is_printing and oams is not None:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            return

        # Hysteresis logic: Use lower threshold to start timer, upper threshold to clear
        if pressure <= self.stuck_spool_pressure_threshold:
            # Pressure is low - start or continue stuck spool timer
            if fps_state.stuck_spool_start_time is None:
                fps_state.stuck_spool_start_time = now
            elif (not fps_state.stuck_spool_active and now - fps_state.stuck_spool_start_time >= STUCK_SPOOL_DWELL):
                message = "Spool appears stuck"
                if fps_state.current_group is not None:
                    message = f"Spool appears stuck on {fps_state.current_group} spool {fps_state.current_spool_idx}"
                self._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
        elif pressure >= self.stuck_spool_pressure_clear_threshold:
            # Pressure is definitively high - clear stuck spool state
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED on %s spool %d", fps_name, fps_state.current_spool_idx)

                # Clear the stuck_spool_active flag BEFORE trying to restore follower
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s, pressure restored to %.2f", fps_name, pressure)

            # Also clear timer if it was running but not yet triggered
            if fps_state.stuck_spool_start_time is not None and not fps_state.stuck_spool_active:
                fps_state.stuck_spool_start_time = None

            # Now restore/enable follower
            if fps_state.stuck_spool_restore_follower and is_printing:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            elif is_printing and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "stuck spool recovery")
        # else: Pressure is in hysteresis band (between thresholds) - maintain current state

    def _check_clog(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check for clog conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear clog LED on %s while runout monitor inactive", fps_name)
            fps_state.reset_clog_tracker()
            return

        if not is_printing:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear clog LED on %s while printer idle", fps_name)
            fps_state.reset_clog_tracker()
            return

        try:
            extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
        except Exception:
            self.logger.exception("Failed to read extruder position while monitoring clogs on %s", fps_name)
            return

        if fps_state.clog_start_extruder is None:
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if extruder_pos < (fps_state.clog_last_extruder or extruder_pos):
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        fps_state.clog_last_extruder = extruder_pos
        if fps_state.clog_min_pressure is None or pressure < fps_state.clog_min_pressure:
            fps_state.clog_min_pressure = pressure
        if fps_state.clog_max_pressure is None or pressure > fps_state.clog_max_pressure:
            fps_state.clog_max_pressure = pressure

        extrusion_delta = extruder_pos - (fps_state.clog_start_extruder or extruder_pos)
        encoder_delta = abs(encoder_value - (fps_state.clog_start_encoder or encoder_value))
        pressure_span = (fps_state.clog_max_pressure or pressure) - (fps_state.clog_min_pressure or pressure)

        settings = self.clog_settings
        if extrusion_delta < settings["extrusion_window"]:
            return

        if (encoder_delta > settings["encoder_slack"] or pressure_span > settings["pressure_band"]):
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if now - (fps_state.clog_start_time or now) < settings["dwell"]:
            return

        if not fps_state.clog_active:
            if oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                except Exception:
                    self.logger.exception("Failed to set clog LED on %s spool %s", fps_name, fps_state.current_spool_idx)
            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.clog_restore_follower = True
            fps_state.clog_restore_direction = direction
            if oams is not None and fps_state.following:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    self.logger.exception("Failed to stop follower on %s during clog pause", fps_name)
            fps_state.following = False
            pressure_mid = (fps_state.clog_min_pressure + fps_state.clog_max_pressure) / 2.0
            message = (f"Clog suspected on {fps_state.current_group or fps_name}: "
                      f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                      f"with FPS {pressure_mid:.2f} near {self.clog_pressure_target:.2f}")
            fps_state.clog_active = True
            self._pause_printer_message(message, fps_state.current_oams)
            self._reactivate_clog_follower(fps_name, fps_state, oams, "clog pause")

    def start_monitors(self):
        """Start all monitoring timers"""
        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        
        for fps_name in self.current_state.fps_state.keys():
            self.monitor_timers.append(
                reactor.register_timer(
                    self._unified_monitor_for_fps(fps_name), 
                    reactor.NOW
                )
            )

            def _reload_callback(fps_name=fps_name, fps_state=self.current_state.fps_state[fps_name]):
                monitor = self.runout_monitors.get(fps_name)
                source_group = fps_state.current_group
                active_oams = fps_state.current_oams
                target_group, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_group(fps_name, fps_state)
                source_group = fps_state.current_group

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(fps_name, fps_state, source_lane, target_lane)
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    self.logger.error("Failed to delegate infinite runout for %s on %s via AFC", fps_name, source_group or "<unknown>")
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(f"Unable to delegate infinite runout for {source_group or fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                group_to_load = target_group or source_group

                if target_group:
                    self.logger.info("Infinite runout triggered for %s on %s -> %s", fps_name, source_group, target_group)
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        self.logger.error("Failed to unload filament during infinite runout on %s: %s", fps_name, unload_message)
                        failure_message = unload_message or f"Failed to unload current spool on {fps_name}"
                        self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                        if monitor:
                            monitor.paused()
                        return

                if group_to_load is None:
                    self.logger.error("No filament group available to reload on %s", fps_name)
                    self._pause_printer_message(f"No filament group available to reload on {fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                load_success, load_message = self._load_filament_for_group(group_to_load)
                if load_success:
                    self.logger.info("Successfully loaded group %s on %s%s", group_to_load, fps_name, " after infinite runout" if target_group else "")
                    if target_group and target_lane:
                        handled = False
                        if AMSRunoutCoordinator is not None:
                            try:
                                handled = AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or active_oams, target_lane, loaded=True, spool_index=fps_state.current_spool_idx, eventtime=fps_state.since)
                            except Exception:
                                self.logger.exception("Failed to notify AFC lane %s after infinite runout on %s", target_lane, fps_name)
                                handled = False
                        if not handled:
                            try:
                                gcode = self.printer.lookup_object("gcode")
                                gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                                self.logger.debug("Marked lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                            except Exception:
                                self.logger.exception("Failed to mark lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                self.logger.error("Failed to load group %s on %s: %s", group_to_load, fps_name, load_message)
                failure_message = load_message or f"No spool available for group {group_to_load}"
                self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                if monitor:
                    monitor.paused()

            fps_reload_margin = getattr(self.fpss[fps_name], "reload_before_toolhead_distance", None)
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance

            monitor = OAMSRunoutMonitor(self.printer, fps_name, self.fpss[fps_name], self.current_state.fps_state[fps_name], self.oams, _reload_callback, reload_before_toolhead_distance=fps_reload_margin)
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        self.logger.info("All monitors started (optimized)")

    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}


def load_config(config):
    return OAMSManager(config)