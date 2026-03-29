# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# FPS (Filament Position Sensor) Buffer Driver
#
# Uses an analog FPS sensor to provide proportional buffer control for any
# AFC unit type.  Instead of the TurtleNeck's bang-bang approach (two
# mechanical switches toggling between high/low multipliers), this driver
# reads a continuous 0.0-1.0 value from an ADC pin and applies a smooth,
# proportional rotation-distance adjustment to keep the buffer at its
# set_point (default 0.5).
#
# FPS reading semantics:
#   0.1 (low)  -> buffer is stretched / not feeding fast enough -> increase feed
#   0.5 (mid)  -> buffer is centered / ideal state
#   0.9 (high) -> buffer is compressed / pushing too much -> decrease feed
#
# The driver is unit-agnostic and registers itself as an AFC buffer so any
# lane can reference it via  buffer: <name>  in its config.
#
# For units WITHOUT stepper motors (OpenAMS, etc.), the driver simply
# provides the ADC reading.  The unit's own code (AFC_OpenAMS) manages
# tool_start_state and virtual sensors exactly as it already does — this
# driver just acts as the FPS hardware interface and config entry point.
# Set  pin_tool_start: buffer  and  buffer: <FPS_buffer_name>  on
# the extruder so Klipper treats it as a buffer, not a GPIO pin.

from __future__ import annotations

from configparser import Error as error
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from extras.AFC_stepper import AFCExtruderStepper
    from gcode import GCodeCommand

TRAILING_STATE_NAME = "Trailing"
ADVANCING_STATE_NAME = "Advancing"
NEUTRAL_STATE_NAME = "Neutral"
CHECK_RUNOUT_TIMEOUT = 0.5
FPS_ENDSTOP_POLL_TIME = 0.01  # 10ms poll interval for software endstop


class FPSEndstopWrapper:
    """Software endstop that triggers based on FPS reading threshold.

    Implements the MCU endstop interface so klipper's homing/drip_move system
    can use the FPS analog reading as a buffer endstop — just like a turtleneck
    switch, but triggered at a configurable threshold.

    For advance endstop: triggers when smoothed_fps >= high_point (buffer compressed)
    For trailing endstop: triggers when smoothed_fps <= low_point (buffer stretched)
    """

    def __init__(self, fps_buffer, trigger_func):
        self._fps_buffer = fps_buffer
        self._reactor = fps_buffer.reactor
        self._trigger_func = trigger_func
        self._steppers = []
        self._trigger_time = 0.
        self._completion = None
        self._poll_timer = None

    def get_mcu(self):
        return self._fps_buffer.adc.get_mcu()

    def add_stepper(self, stepper):
        self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        self._trigger_time = 0.
        self._completion = self._reactor.completion()
        # Check if already triggered before starting poll timer
        if self._trigger_func() == triggered:
            mcu = self.get_mcu()
            self._trigger_time = mcu.estimated_print_time(
                self._reactor.monotonic())
            self._completion.complete(True)
        else:
            self._poll_timer = self._reactor.register_timer(
                self._poll_fps, self._reactor.NOW)
        return self._completion

    def _poll_fps(self, eventtime):
        if self._trigger_func():
            mcu = self.get_mcu()
            self._trigger_time = mcu.estimated_print_time(eventtime)
            self._completion.complete(True)
            return self._reactor.NEVER
        return eventtime + FPS_ENDSTOP_POLL_TIME

    def home_wait(self, home_end_time):
        if self._poll_timer is not None:
            self._reactor.unregister_timer(self._poll_timer)
            self._poll_timer = None
        return self._trigger_time

    def query_endstop(self, print_time):
        return 1 if self._trigger_func() else 0


class AFCFPSBuffer:
    """
    FPS-based buffer driver for AFC.

    Reads an analog filament position sensor and proportionally adjusts the
    lane's stepper rotation distance to keep the FPS reading at the configured
    set_point.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.afc = self.printer.load_object(config, 'AFC')
        self.reactor = self.afc.reactor
        self.gcode = self.afc.gcode
        self.logger = self.afc.logger

        self.name = config.get_name().split(' ')[-1]
        self.lanes = {}
        self.last_state = "Unknown"
        self.enable = False
        self.current_lane: Optional[AFCLane | AFCExtruderStepper] = None
        self.advance_state = False
        self.trailing_state = False
        self._advance_latched = False
        self._latch_enabled = False  # Only latch during active loads
        self.toolhead = None  # Set in _handle_ready

        self.debug = config.getboolean("debug", False)

        # ---- ADC / FPS sensor configuration ----
        self.ppins = self.printer.lookup_object('pins')
        adc_pin = config.get('adc_pin')
        self.adc = self.ppins.setup_pin('adc', adc_pin)

        self.sample_count: int = config.getint('sample_count', 5)
        self.sample_time: float = config.getfloat('sample_time', 0.005)
        self.report_time: float = config.getfloat('report_time', 0.100)
        self.reversed: bool = config.getboolean('reversed', False)

        # Setup ADC sampling — handle both Klipper and Kalico APIs
        try:
            self.adc.setup_adc_sample(self.sample_time, self.sample_count)
        except Exception:
            self.adc.setup_minmax(self.sample_time, self.sample_count)

        # Register ADC callback
        try:
            self.adc.setup_adc_callback(self.report_time, self._adc_callback)
        except TypeError:
            self.adc.setup_adc_callback(self._adc_callback)

        # ---- Buffer tuning parameters ----
        self.fps_value: float = 0.0
        self.set_point: float = config.getfloat('set_point', 0.5, minval=0.1, maxval=0.9)
        self.low_point: float = config.getfloat('low_point', 0.1, minval=0.0, maxval=0.5)
        self.high_point: float = config.getfloat('high_point', 0.9, minval=0.5, maxval=1.0)

        # Multiplier range — how aggressively the buffer corrects
        self.multiplier_high: float = config.getfloat('multiplier_high', 1.1, minval=1.0)
        self.multiplier_low: float = config.getfloat('multiplier_low', 0.9, minval=0.0, maxval=1.0)

        # Deadband — total width of the neutral window centered on set_point
        # No correction applied when FPS is within this range.
        # Gives headroom for fast retractions and tool changes without fighting.
        # Default 0.30 → window from .35 to .65 (set_point ± deadband/2)
        self.deadband: float = config.getfloat('deadband', 0.30, minval=0.0, maxval=0.6)

        # Validate that low_point < set_point < high_point
        if self.low_point >= self.set_point:
            raise error(
                "AFC_FPS {}: low_point ({}) must be less than set_point ({})".format(
                    self.name, self.low_point, self.set_point))
        if self.high_point <= self.set_point:
            raise error(
                "AFC_FPS {}: high_point ({}) must be greater than set_point ({})".format(
                    self.name, self.high_point, self.set_point))

        # Validate deadband doesn't exceed the available range
        half_db = self.deadband / 2.0
        if self.set_point - half_db <= self.low_point:
            raise error(
                "AFC_FPS {}: deadband ({}) too wide — neutral_low ({:.3f}) "
                "must be above low_point ({})".format(
                    self.name, self.deadband,
                    self.set_point - half_db, self.low_point))
        if self.set_point + half_db >= self.high_point:
            raise error(
                "AFC_FPS {}: deadband ({}) too wide — neutral_high ({:.3f}) "
                "must be below high_point ({})".format(
                    self.name, self.deadband,
                    self.set_point + half_db, self.high_point))

        # Smoothing factor for exponential moving average (0 = no smoothing, 1 = max)
        self.smoothing: float = config.getfloat('smoothing', 0.3, minval=0.0, maxval=0.95)
        self.smoothed_fps: float = 0.5

        # Timer interval for applying corrections
        self.update_interval: float = config.getfloat('update_interval', 0.25, minval=0.05)

        # ---- Fault detection ----
        self.error_sensitivity: float = config.getfloat('filament_error_sensitivity', 0, minval=0, maxval=10)
        self.fault_sensitivity: float = self.get_fault_sensitivity(self.error_sensitivity)
        self.filament_error_pos = None
        self.past_extruder_position = None
        self.extruder_pos_timer = None
        self.fault_timer = None
        self.min_event_systime = self.reactor.NEVER

        # ---- LED settings ----
        self.led = False
        self.led_index = config.get('led_index', None)
        self.led_advancing = config.get('led_buffer_advancing', '0,0,1,0')
        self.led_trailing = config.get('led_buffer_trailing', '0,1,0,0')
        self.led_neutral = config.get('led_buffer_neutral', '0,0.5,0.5,0')
        self.led_buffer_disabled = config.get('led_buffer_disable', '0,0,0,0.25')

        if self.led_index is not None:
            self.led = True

        # Enable sensors in GUI
        self.enable_sensors_in_gui = config.getboolean(
            "enable_sensors_in_gui", self.afc.enable_sensors_in_gui
        )

        # ---- Register virtual filament sensors for GUI display ----
        # Same pattern as TurtleNeck buffer which registers advance/trailing sensors.
        # These let Mainsail show buffer state (grey = ramming mode) instead of
        # red (no sensor). The VirtualFilamentSensor tracks advance/trailing state
        # that gets updated in _adc_callback.
        self.adv_filament_switch_name = f"{self.name}_expanded"
        self.fila_adv = VirtualFilamentSensor(
            self.printer, self.adv_filament_switch_name,
            show_in_gui=self.enable_sensors_in_gui)
        self.trail_filament_switch_name = f"{self.name}_compressed"
        self.fila_trail = VirtualFilamentSensor(
            self.printer, self.trail_filament_switch_name,
            show_in_gui=self.enable_sensors_in_gui)

        # ---- Correction timer ----
        self.correction_timer = self.reactor.register_timer(self._correction_event)

        # ---- Register event handlers ----
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # ---- G-code commands ----
        self.function = self.printer.load_object(config, 'AFC_functions')
        self.show_macros = self.afc.show_macros

        self.function.register_mux_command(
            self.show_macros, "QUERY_BUFFER", "BUFFER", self.name,
            self.cmd_QUERY_BUFFER,
            self.cmd_QUERY_BUFFER_help, self.cmd_QUERY_BUFFER_options
        )
        self.gcode.register_mux_command("ENABLE_BUFFER", "BUFFER", self.name, self.cmd_ENABLE_BUFFER)
        self.gcode.register_mux_command("DISABLE_BUFFER", "BUFFER", self.name, self.cmd_DISABLE_BUFFER)
        self.gcode.register_mux_command(
            "AFC_SET_ERROR_SENSITIVITY", "BUFFER", self.name,
            self.cmd_AFC_SET_ERROR_SENSITIVITY,
            desc=self.cmd_AFC_SET_ERROR_SENSITIVITY_help
        )
        self.gcode.register_mux_command(
            "SET_ROTATION_FACTOR", "BUFFER", self.name,
            self.cmd_SET_ROTATION_FACTOR, desc=self.cmd_LANE_ROT_FACTOR_help
        )
        self.gcode.register_mux_command(
            "SET_BUFFER_MULTIPLIER", "BUFFER", self.name,
            self.cmd_SET_BUFFER_MULTIPLIER, desc=self.cmd_SET_BUFFER_MULTIPLIER_help
        )
        self.gcode.register_mux_command(
            "SET_FPS_SET_POINT", "BUFFER", self.name,
            self.cmd_SET_FPS_SET_POINT, desc=self.cmd_SET_FPS_SET_POINT_help
        )

        # Software endstop wrappers — provide the MCU endstop interface so
        # klipper's homing system can use FPS thresholds like turtleneck switches.
        # Advance endstop: triggers at high_point (buffer compressed, filament loaded)
        # Trailing endstop: triggers at low_point (buffer stretched, spool stuck)
        self.fps_endstop = FPSEndstopWrapper(self, lambda: self.buffer_triggered)
        self.fps_trailing_endstop = FPSEndstopWrapper(
            self, lambda: self.buffer_trailing_triggered)

        # Register with AFC buffer registry
        self.afc.buffers[self.name] = self

        # Also register under the AFC_buffer namespace so the upstream
        # AFC_unit buffer lookup (printer.lookup_object('AFC_buffer <name>'))
        # finds us without needing changes to AFC_unit.py.
        self.printer.add_object('AFC_buffer {}'.format(self.name), self)

    def __str__(self):
        return self.name

    # ------------------------------------------------------------------
    # Klipper ready
    # ------------------------------------------------------------------
    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.
        self.toolhead = self.printer.lookup_object('toolhead')

        if self.error_sensitivity > 0:
            self.setup_fault_timer()

        if self.led_index is not None:
            error_string, led = self.afc.function.verify_led_object(self.led_index)
            if led is None:
                raise error(error_string)

    @property
    def extruder(self):
        """Return the active toolhead extruder.

        The native fps.py stored a direct extruder reference.  OAMSMonitor
        accesses ``fps.extruder.last_position`` for clog detection.
        This property provides the same interface without requiring an
        extruder config option — it simply returns whatever extruder is
        currently active on the toolhead.
        """
        if self.toolhead is not None:
            return self.toolhead.get_extruder()
        return None

    def get_fps_value(self) -> float:
        """Get current FPS pressure value (0.0-1.0)."""
        return self.fps_value

    @property
    def buffer_triggered(self) -> bool:
        """True when FPS reading indicates the buffer is compressed (at high_point).

        This is the FPS equivalent of the turtleneck advance switch being
        triggered — used by tool_loaded_check to verify filament is loaded
        into the toolhead without requiring a hardware endstop.
        """
        return self.smoothed_fps >= self.high_point

    @property
    def buffer_trailing_triggered(self) -> bool:
        """True when FPS reading indicates the buffer is stretched (at low_point).

        This is the FPS equivalent of the turtleneck trailing switch being
        triggered — indicates the spool is not feeding fast enough or is stuck.
        """
        return self.smoothed_fps <= self.low_point

    # ------------------------------------------------------------------
    # ADC callback — runs at report_time intervals
    # ------------------------------------------------------------------
    def _adc_callback(self, read_time, read_value=None):
        """Process ADC reading from FPS sensor."""
        if isinstance(read_time, list):
            if not read_time:
                return
            read_time, read_value = read_time[-1]
        elif read_value is None:
            read_value = read_time
            read_time = self.reactor.monotonic()

        if self.reversed:
            read_value = 1.0 - read_value

        self.fps_value = read_value

        # Apply exponential moving average
        self.smoothed_fps = (
            self.smoothing * self.smoothed_fps
            + (1.0 - self.smoothing) * read_value
        )

        # Keep last_state and advance/trailing booleans current even when
        # the correction loop isn't running (buffer disabled during
        # loading / calibration).  Without this, get_toolhead_pre_sensor_state()
        # returns stale False and ACE load / calibration can't detect
        # filament arrival at the toolhead.
        if not self.enable:
            half_db = self.deadband / 2.0
            if self.smoothed_fps > self.set_point + half_db:
                self.last_state = ADVANCING_STATE_NAME
                self.advance_state = True
                if self._latch_enabled:
                    self._advance_latched = True
                self.trailing_state = False
            elif self._advance_latched:
                # Latched during load: keep advance_state True even if
                # pressure drops briefly between ACE motor pulses.
                self.advance_state = True
            elif self.smoothed_fps < self.set_point - half_db:
                self.last_state = TRAILING_STATE_NAME
                self.advance_state = False
                self.trailing_state = True
            else:
                self.last_state = NEUTRAL_STATE_NAME
                self.advance_state = False
                self.trailing_state = False

        # Update virtual filament sensors for GUI display
        self._update_virtual_sensors(read_time)

    # ------------------------------------------------------------------
    # Correction timer — proportional adjustment loop
    # ------------------------------------------------------------------
    def _update_virtual_sensors(self, eventtime):
        """Push buffer state into virtual filament sensors for GUI display.

        The advance sensor reports filament present whenever the FPS reads
        above low_point (any meaningful pressure = filament exists in buffer).
        This prevents the indicator from going red during neutral state when
        filament IS loaded but pressure is balanced.
        """
        # Filament is present if FPS reads above the low threshold
        filament_present = self.smoothed_fps > self.low_point
        try:
            if hasattr(self, 'fila_adv') and self.fila_adv is not None:
                self.fila_adv.runout_helper.note_filament_present(
                    eventtime, filament_present)
        except TypeError:
            self.fila_adv.runout_helper.note_filament_present(filament_present)
        except Exception:
            pass
        try:
            if hasattr(self, 'fila_trail') and self.fila_trail is not None:
                self.fila_trail.runout_helper.note_filament_present(
                    eventtime, self.trailing_state)
        except TypeError:
            self.fila_trail.runout_helper.note_filament_present(self.trailing_state)
        except Exception:
            pass

    def _correction_event(self, eventtime):
        """Periodically adjust rotation distance based on FPS reading."""
        if not self.enable or self.current_lane is None:
            return self.reactor.NEVER
        # Non-stepper lanes (ACE/OpenAMS) don't need rotation correction
        if getattr(self.current_lane, 'extruder_stepper', None) is None:
            return self.reactor.NEVER

        reading = self.smoothed_fps

        half_db = self.deadband / 2.0
        neutral_low = self.set_point - half_db
        neutral_high = self.set_point + half_db

        # Inside deadband window (.35-.65 default) — no correction.
        # Gives the buffer room for fast retractions and tool changes
        # without the correction loop fighting back.
        if neutral_low <= reading <= neutral_high:
            self.set_multiplier(1.0)
            self.last_state = NEUTRAL_STATE_NAME
            self.advance_state = False
            self.trailing_state = False
            if self.led:
                self.afc.function.afc_led(self.led_neutral, self.led_index)
            return eventtime + self.update_interval

        if reading > neutral_high:
            # FPS reading is HIGH (buffer compressed / filament at toolhead)
            # Need to slow down feeding → multiplier < 1
            # Scale from 1.0 at neutral_high to multiplier_low at high_point
            range_size = self.high_point - neutral_high
            if range_size > 0:
                fraction = min((reading - neutral_high) / range_size, 1.0)
            else:
                fraction = 1.0
            multiplier = 1.0 - fraction * (1.0 - self.multiplier_low)
            self.last_state = ADVANCING_STATE_NAME
            self.advance_state = True
            self.trailing_state = False
            if self.led:
                self.afc.function.afc_led(self.led_advancing, self.led_index)
        else:
            # FPS reading is LOW (buffer stretched / not feeding fast enough)
            # Need to speed up feeding → multiplier > 1
            # Scale from 1.0 at neutral_low to multiplier_high at low_point
            range_size = neutral_low - self.low_point
            if range_size > 0:
                fraction = min((neutral_low - reading) / range_size, 1.0)
            else:
                fraction = 1.0
            multiplier = 1.0 + fraction * (self.multiplier_high - 1.0)
            self.last_state = TRAILING_STATE_NAME
            self.advance_state = False
            self.trailing_state = True
            if self.led:
                self.afc.function.afc_led(self.led_trailing, self.led_index)

        self.set_multiplier(multiplier)

        if self.debug:
            self.logger.debug(
                "FPS_buffer {}: fps={:.3f} smoothed={:.3f} "
                "multiplier={:.4f} state={}".format(
                    self.name, self.fps_value, self.smoothed_fps,
                    multiplier, self.last_state
                )
            )

        self._update_virtual_sensors(eventtime)
        return eventtime + self.update_interval

    # ------------------------------------------------------------------
    # Buffer enable / disable  (interface expected by AFCLane)
    # ------------------------------------------------------------------
    def enable_advance_latch(self):
        """Enable latching so advance_state stays True once triggered.

        Call before a load sequence. The latch prevents brief pressure
        drops between ACE motor pulses from clearing the sensor state.
        """
        self._latch_enabled = True
        self._advance_latched = False

    def clear_advance_latch(self):
        """Disable latching and reset advance_state to real-time pressure."""
        self._latch_enabled = False
        self._advance_latched = False

    def enable_buffer(self, lane):
        """Enable the FPS buffer for the given lane.

        For stepper-based units (BoxTurtle, etc.) this also starts the
        proportional correction timer that adjusts rotation distance.
        For non-stepper units (OpenAMS, ACE) the correction loop is skipped
        but the buffer is still marked as enabled/active.
        """
        self.current_lane = lane
        self.enable = True
        self._latch_enabled = False
        self._advance_latched = False
        has_stepper = getattr(lane, 'extruder_stepper', None) is not None

        if self.led:
            self.afc.function.afc_led(self.led_neutral, self.led_index)

        # Reset smoothed value to current reading
        self.smoothed_fps = self.fps_value

        if has_stepper:
            # Start the proportional correction timer
            self.reactor.update_timer(self.correction_timer, self.reactor.NOW)

            # Start fault detection if configured
            if self.fault_detection_enabled():
                self.start_fault_detection(0, 1.0)

        self.logger.debug(f"{self.name} FPS buffer enabled for {self.current_lane.name} (correction={'active' if has_stepper else 'off/adc-only'})")

    def disable_buffer(self):
        """Disable the FPS buffer, reset multiplier, stop timers."""
        self.enable = False
        self._latch_enabled = False
        self._advance_latched = False
        if self.current_lane is None:
            return

        self.logger.debug(f"{self.name} FPS buffer disabled for {self.current_lane.name}")

        if self.led:
            self.afc.function.afc_led(self.led_buffer_disabled, self.led_index)

        self.reset_multiplier()

        # Stop correction timer
        self.reactor.update_timer(self.correction_timer, self.reactor.NEVER)

        # Stop fault detection
        if self.error_sensitivity > 0 and self.extruder_pos_timer is not None:
            eventtime = self.reactor.monotonic()
            self.stop_fault_timer(eventtime)

        self.current_lane = None

    # ------------------------------------------------------------------
    # Multiplier control  (same interface as AFCTrigger)
    # ------------------------------------------------------------------
    def set_multiplier(self, multiplier):
        """Apply rotation distance multiplier to current lane's stepper."""
        if not self.enable:
            return
        if self.current_lane is None:
            return
        if self.current_lane.extruder_stepper is None:
            return

        self.current_lane.update_rotation_distance(multiplier)

    def reset_multiplier(self):
        """Reset rotation distance back to base value."""
        if self.current_lane is None:
            return
        if self.current_lane.extruder_stepper is None:
            return

        self.current_lane.update_rotation_distance(1)
        self.logger.debug("FPS buffer multiplier reset for {}".format(self.current_lane.name))

    # ------------------------------------------------------------------
    # Fault detection  (same interface as AFCTrigger)
    # ------------------------------------------------------------------
    def get_fault_sensitivity(self, sensitivity):
        if sensitivity > 0:
            return (11 - sensitivity) * 10
        return 0

    def disable_fault_sensitivity(self):
        self.fault_sensitivity = 0

    def restore_fault_sensitivity(self):
        self.fault_sensitivity = self.get_fault_sensitivity(self.error_sensitivity)

    def setup_fault_timer(self):
        self.update_filament_error_pos()
        if self.extruder_pos_timer is None:
            self.extruder_pos_timer = self.reactor.register_timer(self.extruder_pos_update_event)

    def start_fault_timer(self, print_time):
        self.fault_timer = "Running"
        self.reactor.update_timer(self.extruder_pos_timer, self.reactor.NOW)

    def stop_fault_timer(self, print_time):
        self.fault_timer = "Stopped"
        self.reactor.update_timer(self.extruder_pos_timer, self.reactor.NEVER)

    def fault_detection_enabled(self):
        return self.fault_sensitivity > 0

    def start_fault_detection(self, eventtime, multiplier):
        self.set_multiplier(multiplier)
        self.update_filament_error_pos()
        self.start_fault_timer(eventtime)

    def get_extruder_pos(self):
        current_pos = self.afc.function.get_extruder_pos(
            past_extruder_position=self.past_extruder_position
        )
        if current_pos is not None:
            self.past_extruder_position = current_pos
        return current_pos

    def update_filament_error_pos(self):
        current_pos = self.get_extruder_pos()
        if current_pos is not None:
            self.filament_error_pos = current_pos + self.fault_sensitivity

    def extruder_pos_update_event(self, eventtime):
        cur_lane = self.current_lane
        if cur_lane is not None and getattr(cur_lane, 'extruder_stepper', None) is None:
            return eventtime + CHECK_RUNOUT_TIMEOUT

        if cur_lane is not None:
            active_extruder = self.afc.toolhead.get_extruder()
            lane_extruder_name = getattr(cur_lane, 'extruder_name', None)
            if (lane_extruder_name
                    and hasattr(active_extruder, 'name')
                    and active_extruder.name != lane_extruder_name):
                return eventtime + CHECK_RUNOUT_TIMEOUT

        extruder_pos = self.get_extruder_pos()
        if (self.afc.function.is_printing(check_movement=True)
                and extruder_pos is not None
                and self.filament_error_pos is not None):
            if extruder_pos > self.filament_error_pos:
                msg = "AFC FPS buffer filament fault detected! Take necessary action."
                self.pause_on_error(msg, True)
                self.update_filament_error_pos()

        return eventtime + CHECK_RUNOUT_TIMEOUT

    def pause_on_error(self, msg, pause=False):
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime or not self.enable or self.afc.function.is_paused():
            return
        if pause:
            if self.last_state == ADVANCING_STATE_NAME:
                msg += '\nCLOG DETECTED'
            if self.last_state == TRAILING_STATE_NAME:
                msg += '\nAFC NOT FEEDING'
            self.afc.error.AFC_error(msg, True)

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def buffer_status(self):
        return self.last_state

    def get_status(self, eventtime=None):
        response = {}
        response['state'] = self.last_state
        response['lanes'] = [lane.name for lane in self.lanes.values()]
        response['enabled'] = self.enable
        response['fps_value'] = round(self.fps_value, 3)
        response['smoothed_fps'] = round(self.smoothed_fps, 3)
        response['set_point'] = self.set_point

        if self.enable and self.current_lane is not None:
            if (self.current_lane.extruder_stepper is not None
                    and self.current_lane.extruder_stepper.stepper is not None):
                stepper = self.current_lane.extruder_stepper.stepper
                response['rotation_distance'] = stepper.get_rotation_distance()[0]
            response['active_lane'] = self.current_lane.name
        else:
            response['rotation_distance'] = None
            response['active_lane'] = None

        response['fault_detection_enabled'] = self.error_sensitivity > 0
        response['error_sensitivity'] = self.error_sensitivity
        response['fault_timer'] = self.fault_timer

        if self.error_sensitivity > 0 and self.filament_error_pos is not None:
            current_pos = self.get_extruder_pos()
            if current_pos is not None:
                response['distance_to_fault'] = self.filament_error_pos - current_pos
                response['filament_error_pos'] = self.filament_error_pos
                response['current_pos'] = current_pos
            else:
                response['distance_to_fault'] = None
        else:
            response['distance_to_fault'] = None

        return response

    # ------------------------------------------------------------------
    # G-code commands
    # ------------------------------------------------------------------
    cmd_QUERY_BUFFER_help = "Report FPS buffer sensor state"
    cmd_QUERY_BUFFER_options = {"BUFFER": {"type": "string", "default": ""}}

    def cmd_QUERY_BUFFER(self, gcmd):
        """
        Reports the current state of the FPS buffer sensor including the
        current FPS reading, smoothed value, and rotation distance.

        Usage: ``QUERY_BUFFER BUFFER=<buffer_name>``
        """
        state_mapping = {
            ADVANCING_STATE_NAME: ' (buffer compressed - filament loaded)',
            TRAILING_STATE_NAME: ' (buffer stretched - not feeding enough)',
            NEUTRAL_STATE_NAME: ' (buffer is centered)',
        }

        buffer_status = self.buffer_status()
        state_info = "{}{}".format(buffer_status, state_mapping.get(buffer_status, ''))
        state_info += "\nFPS raw: {:.3f}  smoothed: {:.3f}  set_point: {:.2f}".format(
            self.fps_value, self.smoothed_fps, self.set_point
        )

        if self.enable and self.current_lane is not None:
            if self.current_lane.extruder_stepper is not None:
                stepper = self.current_lane.extruder_stepper.stepper
                rotation_dist = stepper.get_rotation_distance()[0]
                state_info += "\n{} Rotation distance: {:.4f}".format(
                    self.current_lane.name, rotation_dist
                )

        if self.error_sensitivity > 0:
            state_info += "\nFault detection enabled, sensitivity {}".format(
                self.error_sensitivity
            )

        self.logger.info("{} : {}".format(self.name, state_info))

    cmd_AFC_SET_ERROR_SENSITIVITY_help = "Set filament error sensitivity (0-10, 0=disabled)"

    def cmd_AFC_SET_ERROR_SENSITIVITY(self, gcmd):
        """
        Sets the filament error sensitivity for fault detection.

        Usage: ``AFC_SET_ERROR_SENSITIVITY BUFFER=<name> SENSITIVITY=<0-10>``
        """
        sensitivity = gcmd.get_float('SENSITIVITY', minval=0, maxval=10)
        old_sensitivity = self.error_sensitivity
        self.error_sensitivity = sensitivity
        self.fault_sensitivity = self.get_fault_sensitivity(self.error_sensitivity)

        if old_sensitivity == 0 and sensitivity > 0:
            if self.fault_detection_enabled():
                self.setup_fault_timer()
                self.start_fault_detection(0, 1.0)
        elif old_sensitivity > 0 and sensitivity == 0:
            eventtime = self.reactor.monotonic()
            self.stop_fault_timer(eventtime)
        elif sensitivity > 0:
            self.update_filament_error_pos()

        self.logger.info("Error sensitivity set to {} (fault sensitivity: {})".format(
            self.error_sensitivity, self.fault_sensitivity
        ))

    cmd_SET_BUFFER_MULTIPLIER_help = "Live adjust FPS buffer high and low multiplier range"

    def cmd_SET_BUFFER_MULTIPLIER(self, gcmd):
        """
        Adjust the buffer multiplier range.

        Usage: ``SET_BUFFER_MULTIPLIER BUFFER=<name> MULTIPLIER=<HIGH/LOW> FACTOR=<value>``
        """
        if self.current_lane is not None and self.enable:
            chg_multiplier = gcmd.get('MULTIPLIER', None)
            if chg_multiplier is None:
                self.logger.info("Multiplier must be provided, HIGH or LOW")
                return
            chg_factor = gcmd.get_float('FACTOR')
            if chg_factor <= 0:
                self.logger.info("FACTOR must be greater than 0")
                return
            if chg_multiplier == "HIGH" and chg_factor > 1:
                self.multiplier_high = chg_factor
                self.logger.info("multiplier_high set to {}".format(chg_factor))
                self.logger.info(
                    'multiplier_high: {} MUST be updated under buffer config for value to be saved'.format(chg_factor)
                )
            elif chg_multiplier == "LOW" and chg_factor < 1:
                self.multiplier_low = chg_factor
                self.logger.info("multiplier_low set to {}".format(chg_factor))
                self.logger.info(
                    'multiplier_low: {} MUST be updated under buffer config for value to be saved'.format(chg_factor)
                )
            else:
                self.logger.info(
                    'multiplier_high must be greater than 1, multiplier_low must be less than 1'
                )

    cmd_LANE_ROT_FACTOR_help = "Change rotation distance by factor specified"

    def cmd_SET_ROTATION_FACTOR(self, gcmd):
        """
        Manually override the rotation distance factor.

        Usage: ``SET_ROTATION_FACTOR BUFFER=<name> FACTOR=<value>``
        """
        if self.current_lane is not None and self.enable:
            change_factor = gcmd.get_float('FACTOR', 1.0)
            if change_factor <= 0:
                self.logger.info("FACTOR must be greater than 0")
                return
            elif change_factor == 1.0:
                self.set_multiplier(1)
                self.logger.info("Rotation distance reset to base value")
            else:
                self.set_multiplier(change_factor)
        else:
            self.logger.info("BUFFER {} NOT ENABLED".format(self.name))

    cmd_SET_FPS_SET_POINT_help = "Live adjust FPS buffer set point target"

    def cmd_SET_FPS_SET_POINT(self, gcmd):
        """
        Adjust the FPS target set point and deadband while running.

        Usage: ``SET_FPS_SET_POINT BUFFER=<name> SET_POINT=<0.1-0.9> [DEADBAND=<0.0-0.6>]``
        """
        new_set_point = gcmd.get_float('SET_POINT', self.set_point, minval=0.1, maxval=0.9)
        new_deadband = gcmd.get_float('DEADBAND', self.deadband, minval=0.0, maxval=0.6)

        self.set_point = new_set_point
        self.deadband = new_deadband
        half_db = self.deadband / 2.0
        self.logger.info("FPS set_point={:.2f} deadband={:.2f} (neutral window {:.2f}-{:.2f})".format(
            self.set_point, self.deadband,
            self.set_point - half_db, self.set_point + half_db
        ))

    def cmd_ENABLE_BUFFER(self, gcmd: GCodeCommand):
        """
        Manually enables the FPS buffer for a lane.

        Usage: ``ENABLE_BUFFER BUFFER=<name> LANE=<lane_name>``
        """
        lane = gcmd.get("LANE")
        lane_obj = self.lanes.get(lane, None)
        if not lane_obj:
            raise gcmd.error(f"{lane} not assigned to {self.name} buffer")
        self.enable_buffer(lane_obj)

    def cmd_DISABLE_BUFFER(self, gcmd):
        """
        Manually disables the FPS buffer.

        Usage: ``DISABLE_BUFFER BUFFER=<name>``
        """
        self.disable_buffer()


# ---------------------------------------------------------------------------
#  Virtual filament sensor & extruder patch
#
#  When pin_tool_start references an FPS buffer name (e.g. "FPS_buffer1"),
#  AFCExtruder.__init__ would try to register it as a GPIO pin and fail.
#  The patch below temporarily rewrites the config value to "buffer" (which
#  the base init harmlessly skips), then creates a lightweight virtual
#  filament sensor so the extruder has a valid fila_tool_start object.
# ---------------------------------------------------------------------------


class VirtualRunoutHelper:
    """Minimal runout helper used by FPS virtual sensors."""

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

        if (not new_state and self.sensor_enabled and callable(self.runout_callback)):
            try:
                self.runout_callback(eventtime)
            except TypeError:
                self.runout_callback(eventtime=eventtime)

    def get_status(self, _eventtime=None):
        return {
            "filament_detected": bool(self.filament_present),
            "enabled": bool(self.sensor_enabled),
        }


class VirtualFilamentSensor:
    """Lightweight filament sensor placeholder for FPS virtual pins."""

    QUERY_HELP = "Query the status of the Filament Sensor"
    SET_HELP = "Sets the filament sensor on/off"

    def __init__(self, printer, name, show_in_gui=True, runout_cb=None, enable_runout=False):
        self.printer = printer
        self.name = name
        self._object_name = f"filament_switch_sensor {name}"
        self.runout_helper = VirtualRunoutHelper(printer, name, runout_cb=runout_cb, enable_runout=enable_runout)

        try:
            printer.add_object(self._object_name, self)
            if not show_in_gui:
                # Hide from GUI by prefixing with underscore
                objects = getattr(printer, "objects", {})
                if self._object_name in objects:
                    objects["_" + self._object_name] = objects.pop(self._object_name)
        except Exception:
            # Fallback: direct dict registration
            objects = getattr(printer, "objects", None)
            if isinstance(objects, dict):
                objects.setdefault(self._object_name, self)

        gcode = printer.lookup_object("gcode", None)
        if gcode is None:
            return
        try:
            gcode.register_mux_command("QUERY_FILAMENT_SENSOR", "SENSOR", name, self.cmd_QUERY_FILAMENT_SENSOR, desc=self.QUERY_HELP)
        except Exception:
            pass
        try:
            gcode.register_mux_command("SET_FILAMENT_SENSOR", "SENSOR", name, self.cmd_SET_FILAMENT_SENSOR, desc=self.SET_HELP)
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
        self.logger.info(msg)

    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.runout_helper.sensor_enabled = bool(gcmd.get_int("ENABLE", 1))


def load_config_prefix(config):
    return AFCFPSBuffer(config)
