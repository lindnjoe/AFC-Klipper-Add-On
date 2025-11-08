# OpenAMS Mainboard (OPTIMIZED) - FIXED VERSION
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# FIX: Added last_load_was_retry() method to prevent false positive clog detection
#      after successful load retries

import logging
import mcu
import struct
from math import pi
from typing import Tuple, List, Optional, Any, Dict

try:  # pragma: no cover - optional dependency during unit tests
    from extras.ams_integration import AMSHardwareService
except Exception:  # pragma: no cover - best-effort integration only
    AMSHardwareService = None

# Pre-compiled struct formats for float conversions 
_FLOAT_STRUCT = struct.Struct("f")
_U32_STRUCT = struct.Struct("I")

# OAMS Hardware Status Constants
class OAMSStatus:
    """Hardware status codes reported by OAMS firmware."""
    LOADING = 0
    UNLOADING = 1
    FORWARD_FOLLOWING = 2
    REVERSE_FOLLOWING = 3
    COASTING = 4
    STOPPED = 5
    CALIBRATING = 6
    ERROR = 7

# OAMS Operation Result Codes  
class OAMSOpCode:
    """Operation result codes from OAMS firmware."""
    SUCCESS = 0
    ERROR_UNSPECIFIED = 1
    ERROR_BUSY = 2
    SPOOL_ALREADY_IN_BAY = 3
    NO_SPOOL_IN_BAY = 4
    ERROR_KLIPPER_CALL = 5


class OAMS:
    """
    OpenAMS hardware controller for managing filament spools.
    
    Hardware Interface:
    - Controls 4 filament bays (indexed 0-3)
    - Monitors Hall Effect Sensors (HES) for spool detection
    - Manages BLDC motor for filament feeding
    - Tracks encoder position for motion feedback
    
    Key State Variables:
    - current_spool: Index (0-3) of currently loaded spool, None if unloaded
    - f1s_hes_value: Array of filament sensor readings [bay0, bay1, bay2, bay3]
    - hub_hes_value: Array of hub sensor readings [bay0, bay1, bay2, bay3] 
    - fps_value: Current pressure sensor reading
    - encoder_clicks: Current encoder position
    """
    
    def __init__(self, config):
        # Core printer interface
        self.printer = config.get_printer()
        self.section_name = config.get_name().split()[-1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.reactor = self.printer.get_reactor()

        # OPTIMIZATION: Cache gcode object
        self._cached_gcode = None
        
        # Hardware configuration - Pressure sensor thresholds
        self.fps_upper_threshold: float = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold: float = config.getfloat("fps_lower_threshold") 
        self.fps_is_reversed: bool = config.getboolean("fps_is_reversed")
        
        # Current state variables
        self.current_spool: Optional[int] = None
        self.encoder_clicks: int = 0
        self.i_value: float = 0.0
        
        # Sensor configuration - Hall Effect Sensor thresholds
        self.f1s_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("f1s_hes_on").split(","))
        )
        self.f1s_hes_is_above: bool = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("hub_hes_on").split(","))
        )
        self.hub_hes_is_above: bool = config.getboolean("hub_hes_is_above")
        
        # Physical configuration
        self.filament_path_length: float = config.getfloat("ptfe_length")
        self.oams_idx: int = config.getint("oams_idx")

        # PID control parameters for pressure
        self.kd: float = config.getfloat("kd", 0.0)
        self.ki: float = config.getfloat("ki", 0.0)
        self.kp: float = config.getfloat("kp", 6.0)

        # PID control parameters for current
        self.current_kp: float = config.getfloat("current_kp", 0.375)
        self.current_ki: float = config.getfloat("current_ki", 0.0)
        self.current_kd: float = config.getfloat("current_kd", 0.0)

        # Target values
        self.fps_target: float = config.getfloat(
            "fps_target",
            0.5,
            minval=0.0,
            maxval=1.0,
            above=self.fps_lower_threshold,
            below=self.fps_upper_threshold,
        )
        self.current_target: float = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4
        )
        
        # Hardware state arrays (updated by firmware)
        self.fps_value: float = 0
        self.f1s_hes_value: List[int] = [0, 0, 0, 0]
        self.hub_hes_value: List[int] = [0, 0, 0, 0]
        
        # Action status tracking
        self.action_status: Optional[int] = None
        self.action_status_code: Optional[int] = None
        self.action_status_value: Optional[int] = None
        
        # Setup MCU communication
        self.mcu.register_response(self._oams_action_status, "oams_action_status")
        self.mcu.register_response(self._oams_cmd_stats, "oams_cmd_stats")
        self.mcu.register_response(self._oams_cmd_current_stats, "oams_cmd_current_status")
        self.mcu.register_config_callback(self._build_config)
        
        # Register commands and event handlers
        self.name = config.get_name()
        self.register_commands(self.name.split()[-1])

        # Retry configuration
        self.load_retry_max: int = config.getint("load_retry_max", 3, minval=1, maxval=5)
        self.unload_retry_max: int = config.getint("unload_retry_max", 2, minval=1, maxval=3)
        self.retry_backoff_base: float = config.getfloat("retry_backoff_base", 1.0, above=0.0)
        self.retry_backoff_max: float = config.getfloat("retry_backoff_max", 5.0, above=0.0)
        self.auto_unload_on_failed_load: bool = config.getboolean(
            "auto_unload_on_failed_load", True
        )

        # Retry state tracking
        self._load_retry_count: Dict[int, int] = {}
        self._unload_retry_count: int = 0
        self._last_load_attempt: Dict[int, float] = {}
        self._last_unload_attempt: float = 0.0
        self._last_successful_load: Dict[int, float] = {}
        # FIX: Track whether the last successful load was after retries
        self._last_load_was_retry: Dict[int, bool] = {}

        # Retry failure statistics for monitoring
        self._load_retry_failures: int = 0
        self._unload_retry_failures: int = 0
        self._last_load_failure_time: Optional[float] = None
        self._last_unload_failure_time: Optional[float] = None

        # Expose the underlying hardware controller to AFC when available
        if AMSHardwareService is not None:
            try:
                service = AMSHardwareService.for_printer(
                    self.printer, self.section_name
                )
                service.attach_controller(self)
            except Exception:
                logging.getLogger(__name__).exception(
                    "Failed to register OAMS controller with AMSHardwareService"
                )
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def get_status(self, eventtime: float) -> dict:
        """Return current hardware status for monitoring."""
        return {
            "current_spool": self.current_spool,
            "f1s_hes_value": list(self.f1s_hes_value),
            "hub_hes_value": list(self.hub_hes_value),
            "fps_value": self.fps_value,
            # Retry failure statistics
            "load_retry_failures": self._load_retry_failures,
            "unload_retry_failures": self._unload_retry_failures,
            "last_load_failure_time": self._last_load_failure_time,
            "last_unload_failure_time": self._last_unload_failure_time,
        }
    
    def is_bay_ready(self, bay_index: int) -> bool:
        """Check if a spool bay has filament ready to load."""
        return bool(self.f1s_hes_value[bay_index])
    
    def is_bay_loaded(self, bay_index: int) -> bool:
        """Check if a spool bay has filament loaded into the hub."""
        return bool(self.hub_hes_value[bay_index])
    
    def stats(self, eventtime):
        return (
            False,
            """
OAMS[%s]: current_spool=%s fps_value=%s f1s_hes_value_0=%d f1s_hes_value_1=%d f1s_hes_value_2=%d f1s_hes_value_3=%d hub_hes_value_0=%d hub_hes_value_1=%d hub_hes_value_2=%d hub_hes_value_3=%d kp=%d ki=%d kd=%d encoder_clicks=%d i_value=%.2f
"""
            % ( self.oams_idx,
                self.current_spool,
                self.fps_value,
                self.f1s_hes_value[0],
                self.f1s_hes_value[1],
                self.f1s_hes_value[2],
                self.f1s_hes_value[3],
                self.hub_hes_value[0],
                self.hub_hes_value[1],
                self.hub_hes_value[2],
                self.hub_hes_value[3],
                self.kp,
                self.ki,
                self.kd,
                self.encoder_clicks,
                self.i_value,
            ),
        )

    def handle_connect(self):
        """Initialize MCU commands """
        # Define all commands in a structured way
        command_defs = {
            'load_spool': "oams_cmd_load_spool spool=%c",
            'unload_spool': "oams_cmd_unload_spool",
            'follower': "oams_cmd_follower enable=%c direction=%c",
            'calibrate_ptfe_length': "oams_cmd_calibrate_ptfe_length spool=%c",
            'calibrate_hub_hes': "oams_cmd_calibrate_hub_hes spool=%c",
            'pid': "oams_cmd_pid kp=%u ki=%u kd=%u target=%u",
            'set_led_error': "oams_set_led_error idx=%c value=%c",
        }
        
        try:
            # Batch command lookup 
            for cmd_name, cmd_string in command_defs.items():
                cmd_obj = self.mcu.lookup_command(cmd_string)
                setattr(self, f'oams_{cmd_name}_cmd', cmd_obj)
            
            # Query command with dedicated queue
            cmd_queue = self.mcu.alloc_command_queue()
            self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                "oams_cmd_query_spool",
                "oams_query_response_spool spool=%u",
                cq=cmd_queue,
            )
            
            self.clear_errors()
            
        except Exception as e:
            logging.error("Failed to initialize OAMS commands: %s", e)

    def get_spool_status(self, bay_index):
        return self.f1s_hes_value[bay_index]
            
    def clear_errors(self):
        for i in range(4):
            self.set_led_error(i, 0)
        self.current_spool = self.determine_current_spool()
            
    def set_led_error(self, idx, value):
        if logging.getLogger(__name__).isEnabledFor(logging.DEBUG):
            logging.debug("Setting LED %d to %d", idx, value)
        self.oams_set_led_error_cmd.send([idx, value])
        
    def determine_current_spool(self):
        params = self.oams_spool_query_spool_cmd.send()
        if params is not None and "spool" in params:
            spool_val = params["spool"]
            if 0 <= spool_val <= 3:
                return spool_val
        return None
        

    def register_commands(self, name):
        id = str(self.oams_idx)
        # OPTIMIZATION: Cache gcode object lookup
        if self._cached_gcode is None:
            self._cached_gcode = self.printer.lookup_object("gcode")
        gcode = self._cached_gcode

        # Register all mux commands
        commands = [
            ("OAMS_LOAD_SPOOL", self.cmd_OAMS_LOAD_SPOOL, self.cmd_OAMS_LOAD_SPOOL_help),
            ("OAMS_UNLOAD_SPOOL", self.cmd_OAMS_UNLOAD_SPOOL, self.cmd_OAMS_UNLOAD_SPOOL_help),
            ("OAMS_FOLLOWER", self.cmd_OAMS_FOLLOWER, self.cmd_OAMS_FOLLOWER_help),
            ("OAMS_CALIBRATE_PTFE_LENGTH", self.cmd_OAMS_CALIBRATE_PTFE_LENGTH, self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help),
            ("OAMS_CALIBRATE_HUB_HES", self.cmd_OAMS_CALIBRATE_HUB_HES, self.cmd_OAMS_CALIBRATE_HUB_HES_help),
            ("OAMS_PID_AUTOTUNE", self.cmd_OAMS_PID_AUTOTUNE, self.cmd_OAMS_PID_AUTOTUNE_help),
            ("OAMS_PID_SET", self.cmd_OAMS_PID_SET, self.cmd_OAMS_PID_SET_help),
            ("OAMS_CURRENT_PID_SET", self.cmd_OAMS_CURRENT_PID_SET, self.cmd_OAMS_CURRENT_PID_SET_help),
            ("OAMS_RETRY_STATUS", self.cmd_OAMS_RETRY_STATUS, self.cmd_OAMS_RETRY_STATUS_help),
            (
                "OAMS_RESET_RETRY_COUNTS",
                self.cmd_OAMS_RESET_RETRY_COUNTS,
                self.cmd_OAMS_RESET_RETRY_COUNTS_help,
            ),
        ]

        for cmd_name, handler, help_text in commands:
            gcode.register_mux_command(cmd_name, "OAMS", id, handler, desc=help_text)

    cmd_OAMS_RETRY_STATUS_help = "Display retry configuration and state"

    def cmd_OAMS_RETRY_STATUS(self, gcmd):
        msg_lines = [
            f"OAMS[{self.oams_idx}] Retry Status:",
            f"  Load retry max: {self.load_retry_max}",
            f"  Unload retry max: {self.unload_retry_max}",
            f"  Backoff: {self.retry_backoff_base:.1f}s (max {self.retry_backoff_max:.1f}s)",
            f"  Auto-unload on failed load: {self.auto_unload_on_failed_load}",
            f"  Current unload retry count: {self._unload_retry_count}",
        ]

        if self._load_retry_count:
            msg_lines.append("  Load retry counts:")
            for spool_idx, count in sorted(self._load_retry_count.items()):
                msg_lines.append(
                    f"    Spool {spool_idx}: {count}/{self.load_retry_max}"
                )
        else:
            msg_lines.append("  No active load retries")

        gcmd.respond_info("\n".join(msg_lines))

    cmd_OAMS_RESET_RETRY_COUNTS_help = "Reset retry counters"

    def cmd_OAMS_RESET_RETRY_COUNTS(self, gcmd):
        self._load_retry_count.clear()
        self._unload_retry_count = 0
        self._last_load_attempt.clear()
        self._last_unload_attempt = 0.0
        self._last_successful_load.clear()
        # FIX: Also clear the retry flag tracking
        self._last_load_was_retry.clear()
        gcmd.respond_info(f"OAMS[{self.oams_idx}]: Reset all retry counters")

    def _calculate_retry_delay(self, attempt_number: int) -> float:
        """Calculate exponential backoff delay with max cap."""
        delay = self.retry_backoff_base * (2 ** max(attempt_number - 1, 0))
        return min(delay, self.retry_backoff_max)

    def _reset_load_retry_count(self, spool_idx: int) -> None:
        """Clear retry tracking for a specific spool."""
        self._load_retry_count.pop(spool_idx, None)
        self._last_load_attempt.pop(spool_idx, None)
        # FIX: Also clear the retry flag to prevent stale state
        self._last_load_was_retry.pop(spool_idx, None)

    def _reset_unload_retry_count(self) -> None:
        """Clear unload retry tracking."""
        self._unload_retry_count = 0
        self._last_unload_attempt = 0.0

    def load_spool_with_retry(self, spool_idx: int) -> Tuple[bool, str]:
        """Load spool with automatic retry on failure."""
        retry_count = self._load_retry_count.get(spool_idx, 0)

        # Use a loop instead of recursion to prevent monitor state issues
        while retry_count < self.load_retry_max:
            if retry_count > 0:
                delay = self._calculate_retry_delay(retry_count)
                logging.info(
                    "OAMS[%d]: Load retry %d/%d for spool %d, waiting %.1fs",
                    self.oams_idx,
                    retry_count + 1,
                    self.load_retry_max,
                    spool_idx,
                    delay,
                )
                self.reactor.pause(self.reactor.monotonic() + delay)

            self._load_retry_count[spool_idx] = retry_count + 1
            self._last_load_attempt[spool_idx] = self.reactor.monotonic()

            success, message = self.load_spool(spool_idx)

            if success:
                # Record successful load timestamp for stuck spool detection
                self._last_successful_load[spool_idx] = self.reactor.monotonic()
                # FIX: Track whether this load completed after retries
                if retry_count > 0:
                    self._last_load_was_retry[spool_idx] = True
                else:
                    self._last_load_was_retry[spool_idx] = False
                self._reset_load_retry_count(spool_idx)
                logging.info(
                    "OAMS[%d]: Successfully loaded spool %d on attempt %d",
                    self.oams_idx,
                    spool_idx,
                    retry_count + 1,
                )
                return True, message

            # Load failed
            if retry_count + 1 < self.load_retry_max:
                logging.warning(
                    "OAMS[%d]: Load failed for spool %d: %s. Attempt %d/%d",
                    self.oams_idx,
                    spool_idx,
                    message,
                    retry_count + 1,
                    self.load_retry_max,
                )

                if self.auto_unload_on_failed_load:
                    logging.info("OAMS[%d]: Auto-unloading before retry", self.oams_idx)
                    unload_success, unload_msg = self.unload_spool_with_retry()
                    if not unload_success:
                        logging.error(
                            "OAMS[%d]: Failed to unload before retry: %s",
                            self.oams_idx,
                            unload_msg,
                        )
                        # FIX: Abort load retry sequence if auto-unload fails
                        # Filament is likely stuck in the tube - continuing load attempts will fail
                        self._reset_load_retry_count(spool_idx)
                        # Record retry failure for monitoring
                        self._load_retry_failures += 1
                        self._last_load_failure_time = self.reactor.monotonic()
                        return False, (
                            f"Failed to unload spool {spool_idx} back to AMS before retry. "
                            f"Load aborted after {retry_count + 1} attempts. {unload_msg}"
                        )

                # Increment and continue loop
                retry_count += 1
            else:
                # Max retries reached
                break

        self._reset_load_retry_count(spool_idx)
        # Record retry failure for monitoring
        self._load_retry_failures += 1
        self._last_load_failure_time = self.reactor.monotonic()
        return False, (
            f"Failed to load spool {spool_idx} after {self.load_retry_max} attempts: {message}"
        )

    def get_last_load_attempt_time(self, spool_idx: int) -> Optional[float]:
        """Return timestamp of the most recent load attempt for spool."""
        return self._last_load_attempt.get(spool_idx)

    def get_last_successful_load_time(self, spool_idx: int) -> Optional[float]:
        """Return timestamp of the most recent successful load for spool."""
        return self._last_successful_load.get(spool_idx)

    # FIX: New method to check if last load was after retries
    def last_load_was_retry(self, spool_idx: int) -> bool:
        """
        Check if the last successful load for this spool was after retries.
        
        This is used by the manager to skip post-load pressure checks after retries,
        since pressure may not have stabilized yet and could cause false positives.
        """
        return self._last_load_was_retry.get(spool_idx, False)

    def unload_spool_with_retry(self) -> Tuple[bool, str]:
        """Unload spool with automatic retry on failure."""
        # Use a loop instead of recursion to prevent monitor state issues
        while self._unload_retry_count < self.unload_retry_max:
            if self._unload_retry_count > 0:
                delay = self._calculate_retry_delay(self._unload_retry_count)
                logging.info(
                    "OAMS[%d]: Unload retry %d/%d, waiting %.1fs",
                    self.oams_idx,
                    self._unload_retry_count + 1,
                    self.unload_retry_max,
                    delay,
                )
                self.reactor.pause(self.reactor.monotonic() + delay)

            self._unload_retry_count += 1
            attempt_number = self._unload_retry_count
            self._last_unload_attempt = self.reactor.monotonic()

            success, message = self.unload_spool()

            if success:
                self._reset_unload_retry_count()
                logging.info(
                    "OAMS[%d]: Successfully unloaded spool on attempt %d",
                    self.oams_idx,
                    attempt_number,
                )
                return True, message

            # Unload failed
            if self._unload_retry_count < self.unload_retry_max:
                logging.warning(
                    "OAMS[%d]: Unload failed: %s. Attempt %d/%d",
                    self.oams_idx,
                    message,
                    self._unload_retry_count,
                    self.unload_retry_max,
                )
                # Continue loop for retry
            else:
                # Max retries reached
                break

        self._reset_unload_retry_count()
        # Record retry failure for monitoring
        self._unload_retry_failures += 1
        self._last_unload_failure_time = self.reactor.monotonic()
        return False, f"Failed to unload after {self.unload_retry_max} attempts: {message}"

    cmd_OAMS_CURRENT_PID_SET_help = "Set the PID values for the current sensor"
    def cmd_OAMS_CURRENT_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.current_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.current_kp = p
        self.current_ki = i
        self.current_kd = d
        self.current_target = t
        gcmd.respond_info(
            "Current PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t)
        )

    cmd_OAMS_PID_SET_help = "Set the PID values for the OAMS"
    def cmd_OAMS_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.fps_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.kp = p
        self.ki = i
        self.kd = d
        self.fps_target = t
        gcmd.respond_info("PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t))

    cmd_OAMS_PID_AUTOTUNE_help = "Run PID autotune"
    def cmd_OAMS_PID_AUTOTUNE(self, gcmd):
        target_flow = gcmd.get_float("TARGET_FLOW", None)
        target_temp = gcmd.get_float("TARGET_TEMP", None)

        if target_flow is None:
            raise gcmd.error("TARGET flowrate in mm^3/s is required")
        if target_temp is None:
            raise gcmd.error("TARGET temperature in degrees C is required")

        extrusion_speed_per_min = (60 * target_flow / (pi * (1.75 / 2) ** 2))
        extrusion_length = (extrusion_speed_per_min / 60 * 30)

        gcode = self.printer.lookup_object("gcode")
        gcode.send("M104 S%f" % target_temp)
        gcode.send("G1 E%f F%f" % (extrusion_length, extrusion_speed_per_min))

    cmd_OAMS_CALIBRATE_HUB_HES_help = "Calibrate the range of a single hub HES"
    def cmd_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        self.action_status = OAMSStatus.CALIBRATING
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")
        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")
        self.oams_calibrate_hub_hes_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            value = self.u32_to_float(self.action_status_value)
            gcmd.respond_info("Calibrated HES %d to %f threshold" % (spool_idx, value))
            configfile = self.printer.lookup_object("configfile")
            self.hub_hes_on[spool_idx] = value
            values = ",".join(map(str, self.hub_hes_on))
            configfile.set(self.name, "hub_hes_on", "%s" % (values,))
            gcmd.respond_info("Done calibrating HES; please note this value, and update parameter hub_hes_on for index %d in the configuration" % (spool_idx,))
        else:
            gcmd.error("Calibration of HES %d failed" % spool_idx)

    cmd_OAMS_CALIBRATE_PTFE_LENGTH_help = "Calibrate the length of the PTFE tube"
    def cmd_OAMS_CALIBRATE_PTFE_LENGTH(self, gcmd):
        self.action_status = OAMSStatus.CALIBRATING
        spool = gcmd.get_int("SPOOL", None)
        if spool is None:
            raise gcmd.error("SPOOL index is required")
        self.oams_calibrate_ptfe_length_cmd.send([spool])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            gcmd.respond_info("Calibrated PTFE length to %d" % self.action_status_value)
            configfile = self.printer.lookup_object("configfile")
            configfile.set(self.name, "ptfe_length", "%d" % (self.action_status_value,))
            gcmd.respond_info("Done calibrating clicks, please note this value and update parameter ptfe_length in the configuration")
        else:
            gcmd.error("Calibration of PTFE length failed")

    def load_spool(self, spool_idx):
        self.action_status = OAMSStatus.LOADING
        self.oams_load_spool_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = spool_idx
            return True, "Spool loaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool loading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS with code %d" % self.action_status_code

    cmd_OAMS_LOAD_SPOOL_help = "Load a new spool of filament"
    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        self.action_status = OAMSStatus.LOADING
        self.oams_spool_query_spool_cmd.send()
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")
        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")
        
        success, message = self.load_spool_with_retry(spool_idx)
        
        if success:
            gcmd.respond_info(message)
        else:
            gcmd.error(message)
            
    def unload_spool(self):
        self.action_status = OAMSStatus.UNLOADING
        self.oams_unload_spool_cmd.send()
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = None
            return True, "Spool unloaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool unloading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS"

    cmd_OAMS_UNLOAD_SPOOL_help = "Unload a spool of filament"
    def cmd_OAMS_UNLOAD_SPOOL(self, gcmd):
        success, message = self.unload_spool_with_retry()
        if success:
            gcmd.respond_info(message)
        else:
            gcmd.error(message)

    def set_oams_follower(self, enable, direction):
        self.oams_follower_cmd.send([enable, direction])

    def abort_current_action(self, code: int = OAMSOpCode.ERROR_KLIPPER_CALL) -> None:
        """Abort any in-flight hardware action initiated by Klipper helpers."""
        if self.action_status is None:
            return

        logging.warning(
            "OAMS[%d]: Aborting current action %s with code %d",
            self.oams_idx,
            self.action_status,
            code,
        )
        self.action_status_code = code
        self.action_status_value = None
        self.action_status = None

    cmd_OAMS_FOLLOWER_help = "Enable or disable follower and set its direction"
    def cmd_OAMS_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", None)
        if enable is None:
            raise gcmd.error("ENABLE is required")
        direction = gcmd.get_int("DIRECTION", None)
        if direction is None:
            raise gcmd.error("DIRECTION is required")
        self.oams_follower_cmd.send([enable, direction])
        if enable == 1 and direction == 0:
            gcmd.respond_info("Follower enable in reverse direction")
        elif enable == 1 and direction == 1:
            gcmd.respond_info("Follower enable in forward direction")
        elif enable == 0:
            gcmd.respond_info("Follower disabled")

    def _oams_cmd_stats(self, params):
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value[0] = params["f1s_hes_value_0"]
        self.f1s_hes_value[1] = params["f1s_hes_value_1"]
        self.f1s_hes_value[2] = params["f1s_hes_value_2"]
        self.f1s_hes_value[3] = params["f1s_hes_value_3"]
        self.hub_hes_value[0] = params["hub_hes_value_0"]
        self.hub_hes_value[1] = params["hub_hes_value_1"]
        self.hub_hes_value[2] = params["hub_hes_value_2"]
        self.hub_hes_value[3] = params["hub_hes_value_3"]
        self.encoder_clicks = params["encoder_clicks"]
        
    def _oams_cmd_current_stats(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def get_current(self):
        return self.i_value

    def _oams_action_status(self, params):
        if logging.getLogger(__name__).isEnabledFor(logging.DEBUG):
            logging.debug("OAMS status received")
        
        action = params["action"]
        code = params["code"]
        
        if action in (OAMSStatus.LOADING, OAMSStatus.UNLOADING, OAMSStatus.ERROR):
            self.action_status = None
            self.action_status_code = code
        elif action == OAMSStatus.CALIBRATING:
            self.action_status = None
            self.action_status_code = code
            self.action_status_value = params["value"]
        elif code == OAMSOpCode.ERROR_KLIPPER_CALL:
            self.action_status = None
            self.action_status_code = code
        else:
            logging.error(
                "Spurious response from AMS with code %d and action %d",
                code,
                action,
            )

    def float_to_u32(self, f: float) -> int:
        """Convert float to u32 using pre-compiled struct"""
        return _U32_STRUCT.unpack(_FLOAT_STRUCT.pack(f))[0]

    def u32_to_float(self, i: int) -> float:
        """Convert u32 to float using pre-compiled struct"""
        return _FLOAT_STRUCT.unpack(_U32_STRUCT.pack(i))[0]

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_oams_buffer upper=%u lower=%u is_reversed=%u"
            % (
                self.float_to_u32(self.fps_upper_threshold),
                self.float_to_u32(self.fps_lower_threshold),
                self.fps_is_reversed,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (
                self.float_to_u32(self.f1s_hes_on[0]),
                self.float_to_u32(self.f1s_hes_on[1]),
                self.float_to_u32(self.f1s_hes_on[2]),
                self.float_to_u32(self.f1s_hes_on[3]),
                self.f1s_hes_is_above,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (
                self.float_to_u32(self.hub_hes_on[0]),
                self.float_to_u32(self.hub_hes_on[1]),
                self.float_to_u32(self.hub_hes_on[2]),
                self.float_to_u32(self.hub_hes_on[3]),
                self.hub_hes_is_above,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_pid kp=%u ki=%u kd=%u target=%u"
            % (
                self.float_to_u32(self.kp),
                self.float_to_u32(self.ki),
                self.float_to_u32(self.kd),
                self.float_to_u32(self.fps_target),
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_ptfe length=%u" % (self.filament_path_length)
        )

        self.mcu.add_config_cmd(
            "config_oams_current_pid kp=%u ki=%u kd=%u target=%u"
            % (
                self.float_to_u32(self.current_kp),
                self.float_to_u32(self.current_ki),
                self.float_to_u32(self.current_kd),
                self.float_to_u32(self.current_target),
            )
        )

        self.mcu.add_config_cmd("config_oams_logger idx=%u" % (self.oams_idx))


def load_config_prefix(config):
    return OAMS(config)