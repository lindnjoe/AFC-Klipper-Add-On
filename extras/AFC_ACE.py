# Armored Turtle Automated Filament Changer
#
# ACE Unit Type - Direct ACE PRO hardware integration without ACEPRO/DuckACE
#
# Supports two operational modes:
#   combined: Multiple ACE slots share one toolhead path (combiner/splitter).
#             Must retract current slot before feeding new one.
#   direct:   Each ACE slot feeds its own extruder independently.
#             No global retract-before-feed constraint.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import traceback

from configfile import error as config_error
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from configfile import ConfigWrapper

try: from extras.AFC_utils import ERROR_STR
except: raise config_error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise config_error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLane, AFCLaneState
except: raise config_error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_respond import AFCprompt
except: raise config_error(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try: from extras.AFC_ACE_serial import ACEConnection, ACESerialError, ACETimeoutError
except: raise config_error(ERROR_STR.format(import_lib="AFC_ACE_serial", trace=traceback.format_exc()))

try: from extras.AFC_logger import AFC_QueueListener
except: pass  # Fallback: serial logger will use module-level default

from queuelogger import QueueHandler

# Operational modes
MODE_COMBINED = "combined"  # Multiple slots -> one toolhead (retract before feed)
MODE_DIRECT = "direct"      # Each slot -> its own extruder (independent operation)



class afcACE(afcUnit):
    """AFC unit that talks directly to Anycubic ACE PRO hardware.

    Owns the serial communication to ACE PRO hardware and implements both
    combined (shared toolhead) and direct (per-extruder) operational modes.

    Hub config (required - virtual sensor since ACE has no physical hub sensor):
        [AFC_hub ace_hub1]
        switch_pin: virtual

    Config example:
        [AFC_ACE Ace_1]
        serial_port: /dev/ttyACM0
        hub: ace_hub1
        extruder: extruder
        mode: combined          # or "direct" for multi-extruder
        feed_speed: 60          # mm/s (ACE firmware native unit)
        retract_speed: 50       # mm/s (ACE firmware native unit)
        feed_length: 500        # mm - distance from ACE to toolhead
        retract_length: 500     # mm - distance to retract back to ACE

    Lane config:
        [AFC_lane lane1]
        unit: Ace_1:1            # Unit:Slot (1-based in config, 0-based internal)
        hub: ace_hub1
        extruder: extruder
        feed_length: 2800        # (optional) per-lane override
        retract_length: 2500     # (optional) per-lane override
        use_feed_assist: True    # (optional) per-lane override
    """

    SLOTS_PER_UNIT = 4

    def __init__(self, config: ConfigWrapper):
        super().__init__(config)
        self.type = "ACE"

        # Serial port configuration
        self.serial_port = config.get("serial_port")

        # Operational mode
        mode = config.get("mode", MODE_COMBINED).lower().strip()
        if mode not in (MODE_COMBINED, MODE_DIRECT):
            raise config_error(
                f"[{config.get_name()}] invalid mode '{mode}'. "
                f"Must be '{MODE_COMBINED}' or '{MODE_DIRECT}'"
            )
        self.mode = mode

        # Feed/retract parameters — speeds are in mm/s (matches ACE firmware expectation)
        self.feed_speed = config.getfloat("feed_speed", 60.0)             # mm/s — ACE firmware expects mm/s
        self.retract_speed = config.getfloat("retract_speed", 50.0)       # mm/s — ACE firmware expects mm/s
        self.feed_length = config.getfloat("feed_length", 500.0)        # mm
        self.retract_length = config.getfloat("retract_length", 500.0)  # mm
        self.unload_preretract = config.getfloat("unload_preretract", 50.0) # mm - ACE rewind before cut to tighten spool

        # Hub distance: slot-to-hub/combiner distance.  Used for two-phase
        # loading (prep_post_load feeds to hub, load_sequence feeds hub-to-toolhead).
        # Default 200mm is a safe starting point; calibrate with ACE_CALIBRATE_HUB.
        self.dist_hub = config.getfloat("dist_hub", 200.0)             # mm

        # load_to_hub: unit-level override.  Inherits from AFC global if not set.
        # Can also be overridden per-lane in [AFC_lane] sections.
        self._unit_load_to_hub = config.getboolean("load_to_hub", None)

        # Feed assist: default enable/disable for all slots
        self._default_feed_assist = config.getboolean("use_feed_assist", True)

        # Per-slot feed assist overrides (populated at runtime)
        # None = use default, True/False = explicit override
        self._slot_feed_assist: Dict[int, Optional[bool]] = {}

        # Per-lane overrides for feed/retract/dist_hub parameters.
        # Keyed by lane name (e.g. "lane12").  None values omitted.
        self._lane_feed_length: Dict[str, float] = {}
        self._lane_retract_length: Dict[str, float] = {}
        self._lane_dist_hub: Dict[str, float] = {}
        self._lane_feed_assist: Dict[str, bool] = {}
        self._lane_auto_spoolman_create: Dict[str, bool] = {}
        self._parse_lane_overrides(config)

        # Extruder assist length: how far to advance with extruder motor
        # during feed assist (after filament reaches toolhead sensor area)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)  # mm
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)   # mm/min

        # Sensor-based feeding: feed in increments near the toolhead sensor
        # instead of blindly feeding a fixed distance. This enables calibration.
        self.sensor_approach_margin = config.getfloat("sensor_approach_margin", 30.0)  # mm before expected sensor to switch to incremental
        self.sensor_step = config.getfloat("sensor_step", 40.0)                       # mm per check during sensor approach
        self.calibration_step = config.getfloat("calibration_step", 50.0)              # mm per check during calibration
        self.max_feed_overshoot = config.getfloat("max_feed_overshoot", 100.0)         # mm extra to try past feed_length before giving up

        # Dock purge: drop tool at dock, load filament, purge in dock, then pick up
        self.dock_purge = config.getboolean("dock_purge", False)                       # Set to True to enable dock purge during load
        self.dock_purge_length = config.getfloat("dock_purge_length", 50.0)            # mm of filament to extrude while docked for purging
        self.dock_purge_speed = config.getfloat("dock_purge_speed", 5.0)               # mm/s extrude speed during dock purge

        # Auto Spoolman: when True, automatically create filaments and spools
        # in Spoolman from RFID data. When False, RFID data still matches to
        # existing filaments/spools but won't create new ones.
        self.auto_spoolman_create = config.getboolean("auto_spoolman_create", False)


        # FPS (Filament Pressure Sensor) integration: when the extruder uses
        # an AFC_FPS buffer as pin_tool_start, the FPS ADC value is used as
        # the toolhead sensor.  fps_value 0-1, where 1 means filament is
        # fully compressed against extruder gears.
        self.fps_threshold = config.getfloat("fps_threshold", 0.9)
        # Lower threshold used during active operations (loading/calibration).
        # The latch prevents false clears, so we can trigger earlier to catch
        # brief pressure spikes when filament engages the extruder gears.
        self.fps_load_threshold = config.getfloat("fps_load_threshold", 0.65)
        # Minimum jump between consecutive FPS readings to count as filament
        # engagement during active operations.  Triggers even if the absolute
        # value stays below fps_load_threshold.
        self.fps_delta_threshold = config.getfloat("fps_delta_threshold", 0.15)
        # Number of consecutive ADC readings above fps_threshold required
        # before the sensor is considered triggered during calibration/feeding.
        # ACE pulsed feeding can cause brief spikes above threshold as filament
        # grazes the sensor on the rising edge — requiring multiple consecutive
        # readings filters these out.  At ~100ms per ADC callback, 3 readings
        # adds ~200ms of confirmation delay.
        self.fps_confirm_count = config.getint("fps_confirm_count", 3, minval=1)
        self._prev_fps_value = 0.0  # previous ADC reading for delta calc
        self._fps_obj = None       # resolved AFC_FPS buffer object
        self._fps_extruder = None  # the extruder object associated with FPS
        # Latch: during active operations (calibration/feed), once the FPS
        # crosses the threshold we latch so ACE knows filament reached extruder.
        # This is ACE-internal state — FPS buffer advance_state handles AFC's sensor.
        self._fps_latched = False
        self._fps_consecutive_hits = 0  # consecutive readings above threshold
        self._fps_poll_timer = None  # timer for polling AFC_FPS buffers

        # Sensor polling interval for status/runout monitoring
        self.poll_interval = config.getfloat("poll_interval", 1.0)

        # Baud rate (ACE default is 115200)
        self.baud_rate = config.getint("baud_rate", 115200)

        # Serial connection (created on ready)
        self._ace: Optional[ACEConnection] = None

        # Slot inventory cache: [{status, material, color}, ...]
        self._slot_inventory = [{} for _ in range(self.SLOTS_PER_UNIT)]

        # Cached hardware status (refreshed by _poll_slot_status, never by get_status)
        self._cached_hw_status = {}

        # For combined mode: track which slot is currently in the toolhead
        # -1 means nothing loaded
        self._current_loaded_slot = -1

        # Previous slot states for runout detection
        self._prev_slot_states: Dict[str, bool] = {}

        # Lanes where load-to-hub is suppressed after an explicit eject.
        # Cleared when the slot goes empty (spool removed), so reinsertion
        # triggers a fresh load-to-hub.
        self._hub_load_suppressed: set = set()

        # Set True during active operations (load, unload, calibration) to
        # prevent heartbeat/poll callbacks from modifying lane state
        self._operation_active = False
        # Set True when _operation_active transitions to False so the
        # first status callback re-syncs _prev_slot_states instead of
        # acting on stale ready→not-ready transitions.
        self._prev_states_stale = False

        # Set True while ACE dryer is running so slot status polling
        # skips runout detection (drying can cause transient slot changes)
        self._drying_active = False

        # Track which slots have feed assist actively running on hardware.
        # Used to restore feed assist after ACE reconnection.
        self._feed_assist_active: set = set()

        # Periodic feed assist refresh: re-send start_feed_assist every
        # ~30 seconds to guard against the ACE firmware silently dropping
        # the assist motor (internal timeout, brief busy state, etc.).
        self._feed_assist_refresh_counter = 0
        self._FEED_ASSIST_REFRESH_INTERVAL = 7  # heartbeats (~15s at 2s interval)

        # When True, the next successful heartbeat response will restore
        # feed assist for all tracked slots before clearing the flag.
        self._pending_feed_assist_restore = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Register temperature_ace sensor factory early enough for
        # [temperature_sensor] sections to resolve sensor_type: temperature_ace.
        # Must happen during config parsing (before sensors are instantiated).
        try:
            from extras.temperature_ace import TemperatureACE
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_ace", TemperatureACE)
        except Exception:
            pass

    def _parse_lane_overrides(self, config: ConfigWrapper):
        """Scan [AFC_lane] sections for per-lane feed/retract/dist_hub overrides."""
        unit_name = self.name
        for section in config.fileconfig.sections():
            if not section.startswith("AFC_lane "):
                continue
            lane_cfg = config.getsection(section)
            unit_val = lane_cfg.get("unit", "")
            if unit_val.split(":")[0] != unit_name:
                continue
            lane_name = section.split()[-1]
            feed_len = lane_cfg.getfloat("feed_length", None)
            retract_len = lane_cfg.getfloat("retract_length", None)
            dist_hub = lane_cfg.getfloat("dist_hub", None)
            feed_assist = lane_cfg.getboolean("use_feed_assist", None)
            if feed_len is not None:
                self._lane_feed_length[lane_name] = feed_len
            if retract_len is not None:
                self._lane_retract_length[lane_name] = retract_len
            if dist_hub is not None:
                self._lane_dist_hub[lane_name] = dist_hub
            if feed_assist is not None:
                self._lane_feed_assist[lane_name] = feed_assist
            auto_spoolman = lane_cfg.getboolean("auto_spoolman_create", None)
            if auto_spoolman is not None:
                self._lane_auto_spoolman_create[lane_name] = auto_spoolman

    def _handle_ready(self):
        """Schedule deferred init - reactor pause is disabled during klippy:ready."""
        self.afc.reactor.register_callback(self._deferred_init)

    # Maximum ACE serial log size before rotation (10 MB).
    _SERIAL_LOG_MAX_BYTES = 10 * 1024 * 1024

    def _create_serial_logger(self):
        """Create a dedicated logger for ACE serial comms (writes to AFC_ACE_serial.log)."""
        logger = logging.getLogger("AFC_ACE_serial_file")
        if logger.handlers:
            return logger  # Already set up (e.g. multiple ACE units)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        try:
            log_path = self.printer.start_args.get("log_file", None)
            if log_path:
                log_file = Path(log_path).parent / "AFC_ACE_serial.log"
                # Rotate oversized log on startup to prevent unbounded growth.
                self._rotate_log_if_needed(log_file)
                ql = AFC_QueueListener(log_file)
                ql.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"
                ))
                handler = QueueHandler(ql.bg_queue)
                logger.addHandler(handler)
                # Keep a reference so the QueueListener (and its
                # background writer thread) is not garbage-collected.
                self._serial_ql = ql
                return logger
        except Exception:
            pass
        return None  # Caller will fall back to main AFC logger

    def _rotate_log_if_needed(self, log_file):
        """Rotate log file on startup if it exceeds the size limit."""
        try:
            log_path = Path(log_file)
            if log_path.exists() and log_path.stat().st_size > self._SERIAL_LOG_MAX_BYTES:
                backup = log_path.with_suffix('.log.1')
                if backup.exists():
                    backup.unlink()
                log_path.rename(backup)
        except Exception:
            pass

    # USB devices may not be enumerated yet at Klipper startup.
    # Retry with backoff so a full reboot doesn't require a manual
    # FIRMWARE_RESTART just to bring the ACE online.
    _CONNECT_MAX_RETRIES = 5
    _CONNECT_RETRY_DELAYS = [2.0, 3.0, 5.0, 8.0, 10.0]

    def _deferred_init(self, eventtime):
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
                        f"ACE {self.name}: connected on attempt "
                        f"{attempt + 1}"
                    )
                break
            except Exception as e:
                last_err = e
                self._ace = None
                delay = self._CONNECT_RETRY_DELAYS[attempt]
                if attempt < self._CONNECT_MAX_RETRIES - 1:
                    self.logger.info(
                        f"ACE {self.name}: serial port not available, "
                        f"retrying in {delay:.0f}s "
                        f"(attempt {attempt + 1}/{self._CONNECT_MAX_RETRIES})"
                    )
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + delay
                    )
        else:
            self.logger.error(
                f"ACE {self.name}: failed to connect to ACE at "
                f"{self.serial_port} after {self._CONNECT_MAX_RETRIES} "
                f"attempts: {last_err}\n{traceback.format_exc()}"
            )
            return

        # Enable RFID reader so get_filament_info returns spool data
        try:
            self._ace.enable_rfid()
            self.logger.debug(f"ACE {self.name}: RFID enabled")
        except Exception as e:
            self.logger.warning(
                f"ACE {self.name}: enable_rfid failed (non-fatal): {e}"
            )

        # Seed slot status from get_status (needed before _sync_slot_loaded_state)
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                self._cached_hw_status = hw_status
                for i, slot_data in enumerate(hw_status.get("slots", [])):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        self._slot_inventory[i]["status"] = slot_data.get("status", "")
        except Exception as e:
            self.logger.debug(f"ACE {self.name}: initial get_status failed: {e}")

        # Sync RFID data and lane loaded states
        self._sync_inventory()
        self._sync_slot_loaded_state()

        # Register callbacks for heartbeat status updates and reconnection
        self._ace.status_callback = self._on_hw_status_callback
        self._ace.reconnect_callback = self._on_ace_reconnected

        # Resolve FPS sensor for this unit's extruder (if one exists)
        self._resolve_fps_sensor()

        # If linked to an AFC_FPS buffer (not native fps), start polling timer
        # so _fps_adc_callback gets called with current FPS values
        if self._fps_obj is not None and hasattr(self._fps_obj, 'get_fps_value'):
            self._fps_poll_timer = self.reactor.register_timer(
                self._fps_poll_event, self.reactor.NOW
            )

        # Hydrate the virtual tool sensor for lanes that were tool-loaded

        # Start runout detection polling
        self._start_slot_status_monitor()

        self.logger.info(
            f"ACE {self.name}: connected, mode={self.mode}, "
            f"port={self.serial_port}, slots={self.SLOTS_PER_UNIT}"
        )

    def _get_feed_assist_for_slot(self, slot_index) -> bool:
        """Check if feed assist is enabled for a specific slot."""
        override = self._slot_feed_assist.get(slot_index)
        if override is not None:
            return override
        return self._default_feed_assist

    def _get_feed_length(self, lane=None) -> float:
        """Get effective feed_length, checking lane override first."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_feed_length:
                return self._lane_feed_length[name]
        return self.feed_length

    def _get_retract_length(self, lane=None) -> float:
        """Get effective retract_length, checking lane override first."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_retract_length:
                return self._lane_retract_length[name]
        return self.retract_length

    def _get_dist_hub(self, lane=None) -> float:
        """Get effective dist_hub, checking lane override first."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_dist_hub:
                return self._lane_dist_hub[name]
        return self.dist_hub

    def _get_feed_assist(self, slot_index, lane=None) -> bool:
        """Get effective feed assist: lane override > slot override > unit default."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_feed_assist:
                return self._lane_feed_assist[name]
        return self._get_feed_assist_for_slot(slot_index)

    def _quiet_speed(self, speed):
        """Return *speed* halved when AFC quiet mode is active, else unchanged."""
        if self.afc._get_quiet_mode():
            return speed * 0.5
        return speed

    # ---- FPS (Filament Pressure Sensor) Integration ----

    def _resolve_fps_sensor(self):
        """Find the AFC_FPS buffer that feeds into this unit's extruder.

        Activates when the extruder's ``pin_tool_start`` references an
        AFC_FPS buffer name (e.g. ``FPS_buffer1``).

        Looks up the AFC_FPS buffer object from the AFC buffer registry
        and registers an ADC callback so ``extruder.tool_start_state`` is
        updated in real-time as the FPS value crosses the threshold.
        """
        extruder_name = getattr(self, "extruder", None)
        if not extruder_name:
            return

        try:
            extruder_obj = self.printer.lookup_object(
                "AFC_extruder {}".format(extruder_name), None
            )
        except Exception:
            extruder_obj = None

        if extruder_obj is None:
            return

        # Check if the extruder's buffer is an AFC_FPS buffer
        buffer_name = getattr(extruder_obj, "buffer_name", None)
        if not buffer_name or not isinstance(buffer_name, str):
            return

        fps_obj = self.afc.buffers.get(buffer_name, None)
        if fps_obj is not None and hasattr(fps_obj, 'get_fps_value'):
            self._fps_obj = fps_obj
            self._fps_extruder = extruder_obj

            fila = getattr(extruder_obj, "fila_tool_start", None)
            helper = getattr(fila, "runout_helper", None) if fila else None
            self._fps_runout_helper = helper

            self.logger.info(
                "ACE {}: linked to AFC_FPS buffer '{}' "
                "(extruder={}, threshold={})".format(
                    self.name, buffer_name, extruder_name, self.fps_threshold
                )
            )
            return

        self.logger.debug(
            "ACE {}: extruder '{}' buffer='{}' is not an AFC_FPS buffer".format(
                self.name, extruder_name, buffer_name
            )
        )

    def _fps_poll_event(self, eventtime):
        """Poll the AFC_FPS buffer's fps_value and feed it to _fps_adc_callback.

        Used when the FPS sensor is an AFC_FPS buffer (which doesn't support
        add_callback like native fps objects do).  Runs at ~100ms intervals
        matching the ADC report rate.
        """
        fps_obj = self._fps_obj
        if fps_obj is None:
            return self.reactor.NEVER
        fps_value = fps_obj.get_fps_value()
        self._fps_adc_callback(eventtime, fps_value)
        return eventtime + 0.1

    def _fps_adc_callback(self, read_time, fps_value):
        """ADC callback from the FPS sensor for ACE load detection.

        Called every ~100 ms with the current pressure reading (0.0-1.0).
        During active operations (feeding/calibration), latches when pressure
        crosses threshold so ACE knows filament reached the extruder.

        NOTE: This does NOT set extruder.tool_start_state — the FPS buffer's
        advance_state is the authoritative sensor for AFC's load/ramming logic
        (same as BoxTurtle with buffer). This callback only manages ACE's
        internal _fps_latched flag for its own feed detection.
        When the value crosses the threshold, updates the extruder's
        tool_start_state so that ``lane.get_toolhead_pre_sensor_state()``
        reflects filament presence at the extruder gears.  Also updates
        the virtual filament sensor (fila_tool_start) so Klipper's
        filament sensor system tracks the state and can trigger runout.

        During active operations (calibration/feeding), the state is
        latched: once triggered it stays True until the operation clears
        the latch.  This prevents pulsed ACE feeding from briefly
        dropping the FPS value between motor pulses and clearing the
        triggered state before the calibration loop can see it.
        """
        extruder = self._fps_extruder
        if extruder is None:
            return

        # Skip if another unit (e.g. OAMS) is actively using this shared FPS.
        # Only process FPS readings when this ACE unit owns the currently loaded
        # lane or is actively performing an operation.
        if not self._operation_active:
            loaded_lane = getattr(extruder, 'lane_loaded', None)
            if loaded_lane and loaded_lane not in self.lanes:
                return

        triggered = fps_value >= self.fps_threshold

        if self._operation_active:
            if self._fps_latched:
                # Already confirmed — stay latched, don't clear.
                self._prev_fps_value = fps_value
                return
            # Use the lower load threshold OR a rapid rate-of-change to
            # catch pressure spikes when filament engages extruder gears.
            delta = fps_value - self._prev_fps_value
            self._prev_fps_value = fps_value
            load_triggered = fps_value >= self.fps_load_threshold
            delta_triggered = (fps_value >= 0.5
                               and delta >= self.fps_delta_threshold)
            if load_triggered or delta_triggered:
                self._fps_consecutive_hits += 1
                if self._fps_consecutive_hits >= self.fps_confirm_count:
                    reason = "threshold" if load_triggered else f"delta={delta:.3f}"
                    self._fps_latched = True
                    self.logger.debug(
                        f"ACE FPS: LATCHED True "
                        f"(fps_value={fps_value:.3f}, {reason}, "
                        f"confirmed after {self._fps_consecutive_hits} consecutive reads)"
                    )
                else:
                    self.logger.debug(
                        f"ACE FPS: above threshold ({fps_value:.3f}), "
                        f"confirming {self._fps_consecutive_hits}/{self.fps_confirm_count}"
                    )
            else:
                # Reading dropped below threshold — reset the counter.
                if self._fps_consecutive_hits > 0:
                    self.logger.debug(
                        f"ACE FPS: dropped below threshold ({fps_value:.3f}), "
                        f"resetting confirm counter from {self._fps_consecutive_hits}"
                    )
                self._fps_consecutive_hits = 0
            return

        self._prev_fps_value = fps_value
        # Normal mode: FPS buffer's advance_state handles sensor state for AFC.
        # No tool_start_state writes needed — AFC reads advance_state via
        # get_toolhead_pre_sensor_state().

    def get_fps_value(self):
        """Return the current FPS pressure value, or None if no FPS linked."""
        if self._fps_obj is not None:
            # Works for both AFC_FPS buffers (get_fps_value()) and native fps (fps_value)
            if hasattr(self._fps_obj, 'get_fps_value'):
                return self._fps_obj.get_fps_value()
            return self._fps_obj.fps_value
        return None

    # ---- Slot / Lane Mapping ----

    def _get_local_slot_for_lane(self, lane) -> int:
        """Map an AFC lane to a local ACE slot index (0-3).

        AFC_lane stores the slot index as lane.index (1-based from config
        'unit: ace1:1'). We convert to 0-based for ACE hardware.
        """
        index = getattr(lane, "index", 0)
        if isinstance(index, int) and 1 <= index <= self.SLOTS_PER_UNIT:
            return index - 1  # Config is 1-based, ACE is 0-based
        return -1

    def _disable_extruder_buffers(self, cur_extruder, cur_lane):
        """Disable any active buffers on the shared extruder from other units.

        When an ACE lane shares an extruder with a stepper-based unit (e.g. Turtle),
        the other unit's buffer may still be active and its fault detection timer
        can trigger 'AFC NOT FEEDING' errors during ACE operations. This disables
        those buffers to prevent cross-lane fault detection.
        """
        for lane_name, lane in cur_extruder.lanes.items():
            if lane_name == cur_lane.name:
                continue
            buf = getattr(lane, "buffer_obj", None)
            if buf is not None and getattr(buf, "enable", False):
                self.logger.info(
                    f"ACE: disabling buffer '{buf.name}' on shared extruder "
                    f"lane {lane_name} to prevent cross-lane fault detection"
                )
                buf.disable_buffer()

    def _dock_purge_dropoff(self):
        """Drop off current tool at dock for dock purging.

        Enters docking mode and runs the toolchanger's dropoff gcode so the
        nozzle rests on the dock pad while filament is loaded and purged.
        Uses the native AFC toolchanger engine.
        """
        # Find the AFC_Toolchanger unit for the current extruder
        cur_extruder = self.afc.function.get_current_extruder_obj()
        tc = cur_extruder.tc_unit_obj if cur_extruder else None
        if not tc or not tc.active_tool:
            self.logger.warning("ACE dock purge: no active tool, skipping dropoff")
            return
        tool = tc.active_tool

        # If the toolchanger is in error (e.g. detection failure during a
        # prior tool swap), re-initialize before attempting docking mode.
        if tc.status == 'error':
            self.logger.warning(
                "ACE dock purge: toolchanger in error state (%s), "
                "re-initializing" % tc.error_message)
            tc.initialize(tc.active_tool)

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

    def _dock_purge_pickup(self):
        """Pick up tool from dock after purging.

        Runs the toolchanger's pickup gcode (nozzle wipes on pad during pickup)
        and exits docking mode to restore normal operation.
        Uses the native AFC toolchanger engine.
        """
        cur_extruder = self.afc.function.get_current_extruder_obj()
        tc = cur_extruder.tc_unit_obj if cur_extruder else None
        if not tc or not tc.active_tool or not hasattr(self, '_dock_purge_context'):
            self.logger.warning("ACE dock purge: no context for pickup, skipping")
            return
        tool = tc.active_tool

        tc._run_gcode('tool.pickup_gcode', tool.pickup_gcode, self._dock_purge_context)
        self.afc.gcode.run_script_from_command("EXIT_DOCKING_MODE")
        self._dock_purge_context = None

    # ---- Inventory / State Sync ----

    def _sync_inventory(self):
        """Pull RFID/NFC filament data from ACE hardware into local cache."""
        if self._ace is None or not self._ace.connected:
            return

        for slot in range(self.SLOTS_PER_UNIT):
            try:
                info = self._ace.get_filament_info(slot)
                if isinstance(info, dict):
                    self._store_slot_rfid(slot, info)
            except Exception as e:
                self.logger.debug(
                    f"ACE {self.name}: slot {slot} inventory query failed: {e}"
                )

    def _clear_slot_inventory(self, slot):
        """Clear cached RFID data for a slot so stale info isn't reapplied."""
        if 0 <= slot < self.SLOTS_PER_UNIT:
            self._slot_inventory[slot]["material"] = ""
            self._slot_inventory[slot]["color"] = [0, 0, 0]

    def _store_slot_rfid(self, slot, info):
        """Store all RFID fields from a get_filament_info response into the slot cache."""
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
        inv["extruder_temp_max"] = ext_temp.get("max") if isinstance(ext_temp, dict) else None
        inv["bed_temp_max"] = bed_temp.get("max") if isinstance(bed_temp, dict) else None

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
                f"ACE {self.name}: slot {slot} RFID refresh failed: {e}"
            )

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane load/prep state."""
        if self._ace is None or not self._ace.connected:
            return

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[local_slot]
                slot_loaded = bool(
                    slot_info and slot_info.get("status", "") == "ready"
                )
                lane._load_state = slot_loaded
                lane.prep_state = slot_loaded

                # Only set material/color if the lane doesn't already have
                # values (from Spoolman, manual set, or previous RFID read).
                # Never overwrite existing assignments.
                self._apply_slot_filament_defaults(lane, slot_info)

                # On startup, sync RFID to Spoolman for lanes without a spool_id
                if slot_loaded:
                    self._sync_rfid_to_spoolman(lane, slot_info)

    def _apply_slot_filament_defaults(self, lane, slot_info):
        """Apply filament material/color to a lane if not already set.

        Priority: existing lane values > Spoolman next_spool_id > ACE RFID > AFC defaults.
        Never overwrites values that are already set.
        """
        # If lane already has material and color set, don't touch them
        has_material = getattr(lane, "material", None) not in (None, "")
        has_color = getattr(lane, "color", None) not in (None, "", "#000000")

        if has_material and has_color:
            return

        # Try ACE RFID data from slot inventory
        rfid_material = slot_info.get("material", "") if slot_info else ""
        rfid_color = slot_info.get("color", [0, 0, 0]) if slot_info else [0, 0, 0]

        # Treat "unknown" material from 3rd-party spools as empty
        if rfid_material and rfid_material.lower() == "unknown":
            rfid_material = ""
        rfid_has_data = bool(rfid_material) or rfid_color != [0, 0, 0]

        if rfid_has_data:
            if not has_material and rfid_material:
                lane.material = rfid_material
            if not has_color and rfid_color != [0, 0, 0]:
                lane.color = self._ace_color_to_hex(rfid_color)

        # Apply AFC defaults for anything still missing
        if not has_material and not getattr(lane, "material", None):
            default_mat = getattr(self.afc, "default_material_type", None)
            if default_mat:
                lane.material = default_mat
        if not has_color and not getattr(lane, "color", None):
            default_color = getattr(self.afc, "default_color", None)
            if default_color:
                lane.color = default_color
        if not getattr(lane, "weight", 0):
            lane.weight = 1000

    def _get_auto_spoolman_create(self, lane):
        """Check if auto Spoolman creation is enabled for a lane.

        Lane-level override takes priority, then unit-level default.
        """
        lane_name = getattr(lane, "name", None)
        if lane_name and lane_name in self._lane_auto_spoolman_create:
            return self._lane_auto_spoolman_create[lane_name]
        return self.auto_spoolman_create

    def _sync_rfid_to_spoolman(self, lane, slot_info):
        """Sync ACE RFID tag data to Spoolman: match existing or create new filament + spool.

        Always searches for existing filaments (by SKU) and spools to match.
        Only creates new filaments/spools when auto_spoolman_create is enabled
        (unit-level or lane-level override).

        Silently skips if Spoolman is not configured.
        """
        # Skip silently if Spoolman is not configured — no error for users without it
        if self.afc.spoolman is None or self.afc.moonraker is None:
            return
        # Skip if lane already has a spool assigned
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            return
        sku = slot_info.get("sku", "")
        if not sku:
            return

        moonraker = self.afc.moonraker
        allow_create = self._get_auto_spoolman_create(lane)
        brand = slot_info.get("brand", "")
        material = slot_info.get("material", "")
        color_rgb = slot_info.get("color", [0, 0, 0])
        color_hex = self._ace_color_to_hex(color_rgb).lstrip("#") if color_rgb != [0, 0, 0] else None
        diameter = slot_info.get("diameter", 1.75)
        # RFID 'total' is the empty spool weight (tare), not filament weight
        spool_weight = slot_info.get("total_weight", 0)
        ext_temp = slot_info.get("extruder_temp_max")
        bed_temp = slot_info.get("bed_temp_max")
        # Default filament weight for new spools
        default_filament_weight = 1000

        try:
            # Search for existing filament by SKU (article_number)
            filaments = moonraker.search_filaments(article_number=sku)
            filament = None
            for f in filaments:
                if f.get("article_number", "") == sku:
                    filament = f
                    break

            if filament is None:
                if not allow_create:
                    self.logger.debug(
                        f"No Spoolman filament for SKU {sku} and "
                        f"auto_spoolman_create is disabled for {lane.name}")
                    return

                # Resolve or create vendor
                vendor_id = None
                if brand:
                    vendor = moonraker.get_or_create_vendor(brand)
                    if vendor:
                        vendor_id = vendor.get("id")

                # Create new filament
                filament_name = f"{material} {sku}" if material else sku
                filament = moonraker.create_filament(
                    name=filament_name,
                    vendor_id=vendor_id,
                    material=material or None,
                    density=1.24,
                    diameter=diameter,
                    color_hex=color_hex,
                    settings_extruder_temp=ext_temp,
                    settings_bed_temp=bed_temp,
                    weight=default_filament_weight,
                    spool_weight=spool_weight if spool_weight > 0 else None,
                    article_number=sku,
                )
                if filament is None:
                    self.logger.error(
                        f"Failed to create filament in Spoolman for SKU {sku}")
                    return
                self.logger.info(
                    f"Created Spoolman filament #{filament['id']}: "
                    f"{filament_name} ({material})")

            filament_id = filament.get("id")
            if filament_id is None:
                return

            # Search for an existing spool using this filament before creating a new one.
            # Pick the spool with the most remaining weight (most likely a fresh/partial spool
            # that was previously assigned by this same RFID flow).
            spool = None
            existing_spools = moonraker.search_spools(filament_id=filament_id)
            if existing_spools:
                # Prefer spools with remaining weight, sorted descending
                best = None
                for s in existing_spools:
                    remaining = s.get("remaining_weight") or 0
                    if best is None or remaining > (best.get("remaining_weight") or 0):
                        best = s
                if best is not None:
                    spool = best
                    self.logger.info(
                        f"Found existing Spoolman spool #{spool['id']} for {lane.name} "
                        f"(SKU: {sku}, filament #{filament_id}, "
                        f"remaining: {spool.get('remaining_weight', '?')}g)")

            # Create a new spool only if no existing one was found
            if spool is None:
                if not allow_create:
                    self.logger.debug(
                        f"No existing Spoolman spool for filament #{filament_id} and "
                        f"auto_spoolman_create is disabled for {lane.name}")
                    return

                spool = moonraker.create_spool(
                    filament_id=filament_id,
                    initial_weight=default_filament_weight,
                    remaining_weight=default_filament_weight,
                    spool_weight=spool_weight if spool_weight > 0 else None,
                )
                if spool is None:
                    self.logger.error(
                        f"Failed to create spool in Spoolman for filament #{filament_id}")
                    return
                self.logger.info(
                    f"Created Spoolman spool #{spool['id']} for {lane.name} "
                    f"(SKU: {sku}, filament #{filament_id})")

            spool_id = spool.get("id")

            # Assign spool_id to lane through AFC's normal flow
            self.afc.spool.set_spoolID(lane, spool_id)
            lane.send_lane_data()

        except Exception as e:
            self.logger.error(
                f"RFID-to-Spoolman sync failed for {lane.name}: {e}")

    def _restore_tool_loaded_state(self, lane):
        """Restore a lane to TOOLED state after klipper restart.

        Called when the heartbeat/poll detects a slot is ready and the lane's
        saved state indicates it was previously loaded to the toolhead.  This
        covers the timing gap where _deferred_init completes after PREP has
        already restored saved vars but system_Test couldn't reach the ACE.
        """
        lane.loaded_to_hub = True
        lane.tool_loaded = True
        lane.status = AFCLaneState.TOOLED
        lane.extruder_obj.lane_loaded = lane.name
        lane.sync_to_extruder()
        self.afc.spool.set_active_spool(lane.spool_id)

        # Filament is in the hub path to the toolhead -- set virtual hub sensor
        self._set_hub_state(lane, True)

        self.lane_tool_loaded(lane)

        # Start feed assist immediately
        local_slot = self._get_local_slot_for_lane(lane)
        if (local_slot >= 0
                and self._get_feed_assist(local_slot, lane)
                and self._ace is not None):
            try:
                self._ace.start_feed_assist(local_slot)
                self._feed_assist_active.add(local_slot)
            except Exception:
                pass

        # Restore combined mode slot tracking
        if self.mode == MODE_COMBINED and local_slot >= 0:
            self._current_loaded_slot = local_slot

        self.afc.save_vars()
        self.logger.info(
            f"ACE: restored TOOLED state for {lane.name}"
        )

    def _on_hw_status_callback(self, response):
        """Process slot status from any ACE response (including heartbeat).

        Called from the serial layer's reactor thread for every response that
        doesn't match a pending synchronous request - primarily the heartbeat
        get_status responses that fire every 2 seconds.
        """
        if not isinstance(response, dict):
            return

        # Restore feed assist after reconnection - the first successful
        # heartbeat response confirms the connection is stable.
        if self._pending_feed_assist_restore:
            self._restore_feed_assist()
            self._feed_assist_refresh_counter = 0

        # Periodic feed assist enforcement: any lane that is tool_loaded on
        # its active extruder should have feed assist running if configured.
        # This catches cases where _feed_assist_active lost track of a slot
        # (e.g. startup recovery, manual SET_LANE_LOADED, firmware drop).
        # Also re-sends the command every ~15s to guard against the ACE
        # firmware silently disabling the assist motor.
        if not self._operation_active and self._ace is not None:
            self._feed_assist_refresh_counter += 1
            if self._feed_assist_refresh_counter >= self._FEED_ASSIST_REFRESH_INTERVAL:
                self._feed_assist_refresh_counter = 0
                for lane in self.lanes.values():
                    if not getattr(lane, "tool_loaded", False):
                        continue
                    local_slot = self._get_local_slot_for_lane(lane)
                    if local_slot < 0:
                        continue
                    if not self._get_feed_assist(local_slot, lane):
                        continue
                    try:
                        self._ace.start_feed_assist(local_slot)
                        self._feed_assist_active.add(local_slot)
                    except Exception:
                        pass

        # Skip lane state updates during active operations (load/unload/calibration)
        if self._operation_active:
            return

        # Extract result payload (JSON-RPC wraps it)
        result = response.get("result", response)
        if not isinstance(result, dict):
            return

        slots = result.get("slots")
        if not slots or not isinstance(slots, list):
            return

        # After an operation (load/unload/calibration) completes,
        # _prev_slot_states is stale because callbacks were skipped while
        # _operation_active was True.  Re-sync it to current slot reality
        # so we don't misinterpret a stale ready→not-ready edge as
        # filament removal (which would wipe the lane state).
        resync_prev = self._prev_states_stale
        if resync_prev:
            self._prev_states_stale = False

        # Update cached hardware status and slot inventory
        self._cached_hw_status = result
        for i, slot_data in enumerate(slots):
            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                self._slot_inventory[i]["status"] = slot_data.get("status", "")

        # Sync slot states and ensure lane consistency
        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = self._slot_inventory[local_slot]
            slot_status = slot_info.get("status", "") if slot_info else ""
            slot_ready = slot_status == "ready"
            slot_transient = slot_status in ("shifting", "feeding", "unwinding")

            # Keep load/prep state in sync with slot status, but skip
            # transient motor states so the lane doesn't briefly show empty.
            if not slot_transient:
                lane._load_state = slot_ready
                lane.prep_state = slot_ready

            prep_done = getattr(lane, '_afc_prep_done', False)

            # State consistency: if hardware says ready but lane is stuck
            # in NONE, fix it regardless of transition detection
            if slot_ready and prep_done and lane.status == AFCLaneState.NONE:
                # Check if this lane was tool-loaded before restart -
                # restore TOOLED state instead of just LOADED.
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE callback: {lane.name} slot {local_slot} ready, "
                        f"restoring TOOLED state from saved vars"
                    )
                    self._restore_tool_loaded_state(lane)
                elif lane.name in self._hub_load_suppressed:
                    # Lane was explicitly ejected -- slot is still ready but
                    # we must NOT treat this as new filament.  Restore LOADED
                    # state so the lane isn't stuck in NONE, but keep the
                    # suppression flag so prep_post_load won't re-feed to hub.
                    self.logger.info(
                        f"ACE callback: {lane.name} slot {local_slot} ready "
                        f"but hub-load suppressed (ejected), setting loaded "
                        f"without re-feed"
                    )
                    lane.set_loaded()
                    self.afc.save_vars()
                else:
                    self.logger.info(
                        f"ACE callback: {lane.name} slot {local_slot} ready, "
                        f"setting loaded"
                    )
                    # New filament insertion: clear old data, apply fresh.
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
                    self._refresh_slot_inventory(local_slot)
                    slot_info = self._slot_inventory[local_slot]
                    self._apply_slot_filament_defaults(lane, slot_info)
                    # Auto-create/assign Spoolman spool from RFID data
                    self._sync_rfid_to_spoolman(lane, slot_info)
                    self.lane_illuminate_spool(lane)
                    # New filament clears any previous suppression
                    self._hub_load_suppressed.discard(lane.name)
                    self.afc.save_vars()
                    # Feed filament to hub if load_to_hub is enabled
                    try:
                        self.prep_post_load(lane)
                    except Exception as e:
                        self.logger.error(
                            f"ACE callback: prep_post_load error for "
                            f"{lane.name}: {e}"
                        )

            # When a slot goes empty, handle runout for TOOLED lanes
            # or transition LOADED lanes to unloaded.  Skip transition
            # detection on the first callback after an operation ends
            # (_prev_slot_states is stale and would cause false triggers).
            if not slot_ready and not slot_transient and not resync_prev:
                prev_ready_cb = self._prev_slot_states.get(lane.name)
                if prev_ready_cb:
                    self._hub_load_suppressed.discard(lane.name)
                    is_printing = False
                    try:
                        is_printing = self.afc.function.is_printing()
                    except Exception:
                        pass
                    if (not self._drying_active
                            and is_printing
                            and lane.status == AFCLaneState.TOOLED):
                        self.logger.info(
                            f"ACE runout detected on {lane.name} "
                            f"(slot {local_slot})"
                        )
                        self._clear_slot_inventory(local_slot)
                        lane.loaded_to_hub = False
                        self._set_hub_state(lane, False)
                        if lane.runout_lane:
                            try:
                                lane._perform_infinite_runout()
                            except Exception as e:
                                self.logger.error(
                                    f"ACE infinite spool failed for "
                                    f"{lane.name}: {e}\n"
                                    f"{traceback.format_exc()}"
                                )
                                lane._perform_pause_runout()
                            finally:
                                lane.loaded_to_hub = False
                        else:
                            self._ace_pause_runout(lane)
                    elif lane.status == AFCLaneState.LOADED:
                        self._clear_slot_inventory(local_slot)
                        lane.set_unloaded()
                        self.lane_not_ready(lane)
                        self.afc.save_vars()

            # Don't update prev state during transient states to
            # avoid false runout triggers in _poll_slot_status.
            if not slot_transient:
                self._prev_slot_states[lane.name] = slot_ready

    def _on_ace_reconnected(self):
        """Called after ACE reconnects (power cycle or USB disconnect).

        Defers feed assist restoration until the first successful heartbeat
        to ensure the connection is stable before sending commands.
        """
        if self._feed_assist_active:
            self.logger.info(
                f"ACE reconnect: deferring feed assist restore for "
                f"slots {sorted(self._feed_assist_active)} until first heartbeat"
            )
            self._pending_feed_assist_restore = True
        else:
            self.logger.debug("ACE reconnect: no feed assist to restore")

    def _restore_feed_assist(self):
        """Restore feed assist for all tracked active slots after reconnect."""
        self._pending_feed_assist_restore = False
        if not self._feed_assist_active or self._ace is None:
            return

        for slot_index in sorted(self._feed_assist_active):
            try:
                self._ace.start_feed_assist(slot_index)
                self.logger.info(
                    f"ACE reconnect: feed assist restored for slot {slot_index}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE reconnect: failed to restore feed assist "
                    f"for slot {slot_index}: {e}"
                )

    def get_slot_info(self, local_slot):
        """Return cached slot info dict for a slot index."""
        if 0 <= local_slot < self.SLOTS_PER_UNIT:
            return self._slot_inventory[local_slot]
        return {}

    # ---- AFC Unit Interface Implementation ----

    def handle_connect(self):
        super().handle_connect()
        self._register_gcode_commands()

        self.logo = '<span class=success--text>R  _____ _____ _____\n'
        self.logo += 'E | AFC | ACE |     |\n'
        self.logo += 'A |  P  |  R  |  O  |\n'
        self.logo += 'D |_____|_____|_____|\n'
        self.logo += 'Y |_1_|_2_|_3_|_4_|\n'
        self.logo += '  ' + self.name + ' (' + self.mode + ')\n'

        self.logo_error = '<span class=error--text>E  _____ _____ _____\n'
        self.logo_error += 'R | AFC | ACE |     |\n'
        self.logo_error += 'R | ERR | ERR | ERR |\n'
        self.logo_error += 'O |_____|_____|_____|\n'
        self.logo_error += 'R |_X_|_X_|_X_|_X_|\n'
        self.logo_error += '  ' + self.name + '</span>\n'

    def get_status(self, eventtime=None):
        """Return status dict including cached ACE hardware state.

        IMPORTANT: This must NOT send serial commands - Klipper calls this
        multiple times per second for UI updates. Use cached data only.
        The cache is refreshed by _poll_slot_status at poll_interval.
        """
        response = super().get_status(eventtime)

        response["ace_mode"] = self.mode
        response["ace_connected"] = (
            self._ace is not None and self._ace.connected
        )
        response["ace_serial_port"] = self.serial_port
        response["ace_status"] = self._cached_hw_status
        response["ace_drying"] = self._drying_active

        # Hardware temps/humidity from cached get_status response
        hw = self._cached_hw_status
        response["ace_temp"] = hw.get("temp")
        response["ace_humidity"] = hw.get("humidity")

        # Feed assist state: list of slot indices with active feed assist
        response["ace_feed_assist_slots"] = sorted(self._feed_assist_active)

        return response

    def lane_tool_loaded(self, lane):
        """Set the virtual tool sensor when an ACE lane loads into the toolhead.

        The base class only updates LEDs.
        """
        super().lane_tool_loaded(lane)
        extruder = self._fps_extruder
        if extruder is None:
            return
        eventtime = self.afc.reactor.monotonic()

    def lane_tool_unloaded(self, lane):
        """Clear the virtual tool sensor when an ACE lane unloads.

        Mirrors lane_tool_loaded: explicitly clears the sensor so the extruder
        knows the toolhead is empty.
        """
        super().lane_tool_unloaded(lane)
        extruder = self._fps_extruder
        if extruder is None:
            return
        eventtime = self.afc.reactor.monotonic()

    def load_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Load filament from ACE slot into the toolhead.

        In combined mode: retracts the currently loaded slot first.
        In direct mode: feeds independently (each lane has its own extruder).
        """
        afc = self.afc
        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._load_sequence_inner(cur_lane, cur_hub, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _load_sequence_inner(self, cur_lane, cur_hub, cur_extruder):
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE load failed: ACE not connected ({self.serial_port})",
            )
            return False

        # Disable any active buffers on the shared extruder from other units.
        # This prevents cross-lane buffer fault detection (e.g. a Turtle buffer
        # on the same extruder) from triggering "AFC NOT FEEDING" during ACE ops.
        self._disable_extruder_buffers(cur_extruder, cur_lane)

        # Check if this lane is already loaded
        if (cur_lane.get_toolhead_pre_sensor_state()
                and hasattr(cur_lane, "tool_loaded") and cur_lane.tool_loaded):
            self.logger.debug(
                f"Lane {cur_lane.name} already loaded to toolhead, skipping"
            )
            cur_lane.set_tool_loaded()
            self.afc.save_vars()
            return True

        # Check if a different lane is loaded on this extruder and unload it
        # first.  This handles the case where CHANGE_TOOL's normal unload was
        # skipped (e.g. shuttle was empty at startup so self.current was None).
        loaded_lane_name = cur_extruder.lane_loaded
        if loaded_lane_name is not None and loaded_lane_name != cur_lane.name:
            loaded_lane = afc.lanes.get(loaded_lane_name)
            if loaded_lane is not None:
                self.logger.info(
                    f"ACE load: {loaded_lane_name} still loaded on "
                    f"{cur_extruder.name}, unloading first"
                )
                if not afc.TOOL_UNLOAD(loaded_lane, set_start_time=False):
                    afc.error.handle_lane_failure(
                        cur_lane,
                        f"ACE load failed: could not unload {loaded_lane_name} "
                        f"from {cur_extruder.name}",
                    )
                    return False

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Dock purge phase 1: drop off tool at dock before feeding filament
        dock_dropped_off = False
        if self.dock_purge:
            self.logger.info("ACE dock purge: dropping tool off at dock before feed")
            self._dock_purge_dropoff()
            dock_dropped_off = True
            afc.afcDeltaTime.log_with_time("ACE: After dock purge dropoff")

        # Wrap the rest of the load in try/finally so the tool is always
        # picked back up from the dock even if the load fails.
        load_result = False
        try:
            load_result = self._load_sequence_feed_and_verify(
                cur_lane, cur_hub, cur_extruder, afc
            )
        finally:
            if dock_dropped_off:
                # Always pick up the tool -even on failure
                if load_result and self.dock_purge:
                    # Success path: purge in dock, then pick up
                    purge_spd = self._quiet_speed(self.dock_purge_speed)
                    self.logger.info(
                        f"ACE dock purge: extruding {self.dock_purge_length}mm "
                        f"@ {purge_spd}mm/s in dock, then picking up"
                    )
                    afc.move_e_pos(
                        self.dock_purge_length, purge_spd,
                        "dock purge extrude"
                    )
                else:
                    self.logger.info(
                        "ACE dock purge: picking up tool after load failure"
                    )
                self._dock_purge_pickup()
                afc.afcDeltaTime.log_with_time("ACE: After dock purge pickup")

        return load_result

    def _load_sequence_feed_and_verify(self, cur_lane, cur_hub,
                                       cur_extruder, afc):
        """Feed filament and verify it reached the toolhead.

        Separated from _load_sequence_inner so dock purge pickup always
        runs via the try/finally wrapper even on failure.
        """
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE load failed: cannot determine slot for {cur_lane.name}",
            )
            return False

        try:
            # Combined mode: retract current slot first
            if self.mode == MODE_COMBINED and self._current_loaded_slot >= 0:
                if self._current_loaded_slot != local_slot:
                    self.logger.info(
                        f"ACE combined mode: retracting slot "
                        f"{self._current_loaded_slot} before loading slot {local_slot}"
                    )
                    # Stop feed assist on old slot before retracting
                    try:
                        self._wait_for_ace_ready()
                        self._ace.stop_feed_assist(self._current_loaded_slot)
                        self._feed_assist_active.discard(self._current_loaded_slot)
                    except Exception:
                        pass
                    self._retract_slot(self._current_loaded_slot)
                    self._current_loaded_slot = -1

            # Calculate effective feed distance.  If filament is already
            # staged at the hub (via prep_post_load), only feed the
            # remaining hub-to-toolhead distance.
            full_feed = self._get_feed_length(cur_lane)
            dist_hub = self._get_dist_hub(cur_lane)
            pre_fed = cur_lane.loaded_to_hub
            if pre_fed and dist_hub > 0:
                feed_distance = max(full_feed - dist_hub, 0)
                self.logger.info(
                    f"ACE load: filament at hub, feeding {feed_distance:.0f}mm "
                    f"(full={full_feed:.0f} - hub={dist_hub:.0f}) "
                    f"slot {local_slot} for lane {cur_lane.name}"
                )
            else:
                feed_distance = full_feed
                self.logger.info(
                    f"ACE load: feeding full {feed_distance:.0f}mm "
                    f"slot {local_slot} for lane {cur_lane.name} "
                    f"(mode={self.mode})"
                )

            # Filament is about to feed through the hub -- set virtual hub
            # sensor so downstream checks see it as occupied.
            self._set_hub_state(cur_lane, True)

            # Enable FPS advance latch so the toolhead sensor stays triggered
            # even when ACE briefly pauses between motor pulses during feed.
            buffer_obj = getattr(cur_lane, 'buffer_obj', None)
            if buffer_obj is not None and hasattr(buffer_obj, 'enable_advance_latch'):
                buffer_obj.enable_advance_latch()

            _, _, early_engage = self._feed_slot(local_slot, lane=cur_lane, feed_distance=feed_distance)

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = local_slot

            # Filament has been fed through the hub into the bowden
            cur_lane.loaded_to_hub = True

        except Exception as e:
            # Load failed -- clear virtual hub since filament may not be in path
            self._set_hub_state(cur_lane, False)
            message = f"ACE load failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Verify toolhead sensor triggered.
        # ACE lanes with FPS buffers skip the traditional buffer decompress
        # check — the FPS manages filament tension itself and its
        # advance_state reflects real-time pressure, not a physical switch
        # that needs to be cleared by retracting.
        if cur_extruder.tool_start:
            # If the FPS already latched during the feed, the filament is
            # confirmed present — skip the smart-load retry loop.  The FPS
            # is pressure-based: once the ACE stops feeding, pressure drops
            # and advance_state / get_toolhead_pre_sensor_state() goes False
            # even though filament is still there.  Trusting the latch
            # avoids unnecessary 220mm retry feeds.
            if self._fps_latched:
                self.logger.info(
                    "ACE smart load: skipping — FPS already latched during feed"
                )
            # Standard toolhead sensor verification with retry.
            # When home_to_tool is active, scale retries using
            # tool_homing_distance so the ACE searches the same range
            # a BoxTurtle stepper would during endstop homing.
            elif not cur_lane.get_toolhead_pre_sensor_state():
                homing_active = (getattr(afc, 'homing_enabled', False)
                                 and getattr(afc, 'home_to_tool', False))
                if homing_active:
                    smart_load_dist = getattr(afc, 'tool_homing_distance', 200.0) + 20.0
                else:
                    smart_load_dist = 200.0 + 20.0

                smart_load_retries = 3
                smart_spd = self._quiet_speed(self.feed_speed)
                triggered = False

                for attempt in range(1, smart_load_retries + 1):
                    self.logger.info(
                        f"ACE smart load: sensor not triggered, "
                        f"feeding {smart_load_dist:.0f}mm continuous "
                        f"(attempt {attempt}/{smart_load_retries}, "
                        f"home_to_tool={'active' if homing_active else 'off'})"
                    )

                    # Ensure feed assist is running so ACE pushes filament
                    if (local_slot >= 0
                            and self._get_feed_assist(local_slot, cur_lane)
                            and local_slot not in self._feed_assist_active):
                        try:
                            self._wait_for_ace_ready()
                            self._ace.start_feed_assist(local_slot)
                            self._feed_assist_active.add(local_slot)
                        except Exception:
                            pass

                    self._wait_for_ace_ready()
                    try:
                        self._ace.feed_filament(local_slot, smart_load_dist, smart_spd)
                    except (ACESerialError, ACETimeoutError) as e:
                        self.logger.warning(
                            f"ACE smart load: feed_filament failed on "
                            f"attempt {attempt}: {e}"
                        )
                        afc.reactor.pause(afc.reactor.monotonic() + 1.0)
                        continue
                    sensor_hit = self._wait_for_feed_complete(
                        local_slot, smart_load_dist, smart_spd,
                        lane=cur_lane, poll_interval=0.1,
                    )
                    afc.reactor.pause(afc.reactor.monotonic() + 0.2)

                    if sensor_hit or cur_lane.get_toolhead_pre_sensor_state():
                        self.logger.info(
                            f"ACE smart load: sensor triggered on "
                            f"attempt {attempt}"
                        )
                        # Re-enable feed assist in case stop_feed_filament
                        # disabled it on the ACE firmware side.
                        if (local_slot >= 0
                                and self._get_feed_assist(local_slot, cur_lane)):
                            try:
                                self._ace.start_feed_assist(local_slot)
                                self._feed_assist_active.add(local_slot)
                            except Exception:
                                pass
                        triggered = True
                        break

                if not triggered:
                    total_searched = smart_load_retries * smart_load_dist
                    message = (
                        f"ACE load did not trigger toolhead sensor after "
                        f"{total_searched:.0f}mm of extra feeding "
                        f"({smart_load_retries} attempts). "
                        f"CHECK FILAMENT PATH\n"
                        "To resolve, set lane loaded with "
                        f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                    )
                    if homing_active:
                        message += (
                            f"\nFilament can also be reset back to hub by "
                            f"running AFC_RESET command then select "
                            f"{cur_lane.name} to reset back to hub. Once lane "
                            f"is reset try reload lane with {cur_lane.map} macro."
                        )
                    if afc.function.in_print():
                        message += (
                            "\nOnce filament is fully loaded click resume to continue printing"
                        )
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False

        # ── Early extruder engagement (FPS buffer + home_to_tool) ────
        # When the FPS latched during the home_to_tool feed, sync the
        # extruder and start feed_assist immediately, then advance
        # tool_stn.  Skips the normal feed_assist_count verification.
        if early_engage:
            self.logger.info(
                "ACE load: FPS early engage — starting extruder immediately"
            )
            cur_lane.status = AFCLaneState.TOOL_LOADED
            self.afc.save_vars()
            cur_lane.sync_to_extruder()

            # Start feed assist right away so ACE pushes while extruder
            # pulls — this keeps tension during engagement.
            if local_slot >= 0 and self._get_feed_assist(local_slot, cur_lane):
                try:
                    self._wait_for_ace_ready()
                    self._ace.start_feed_assist(local_slot)
                    self._feed_assist_active.add(local_slot)
                    self.logger.info(
                        f"ACE load: early engage — feed assist started "
                        f"for slot {local_slot}"
                    )
                except Exception as e:
                    self.logger.warning(
                        f"ACE load: early engage — start_feed_assist failed: {e}"
                    )

            # If tool_end sensor exists, feed until it triggers
            if cur_extruder.tool_end:
                tool_attempts = 0
                while not cur_extruder.tool_end_state:
                    tool_attempts += 1
                    afc.move_e_pos(
                        cur_lane.short_move_dis, cur_extruder.tool_load_speed,
                        "Tool end", wait_tool=True
                    )
                    if tool_attempts > 20:
                        message = (
                            "ACE load: filament failed to trigger post-extruder sensor.\n"
                            "To resolve, set lane loaded with "
                            f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                        )
                        afc.error.handle_lane_failure(cur_lane, message)
                        return False

            # Advance into nozzle — extruder pulls, feed assist pushes
            if cur_extruder.tool_stn:
                self.logger.info(
                    f"ACE load: early engage — advancing {cur_extruder.tool_stn}mm "
                    f"into nozzle @ {cur_extruder.tool_load_speed}mm/s"
                )
                afc.move_e_pos(
                    cur_extruder.tool_stn, cur_extruder.tool_load_speed, "tool stn"
                )

            cur_lane.set_tool_loaded()
            cur_lane.enable_buffer(disable_fault=True)
            self.afc.save_vars()
            return True

        # ── Normal engagement path ───────────────────────────────────
        # Sync to extruder and load filament into the nozzle using tool_stn
        cur_lane.status = AFCLaneState.TOOL_LOADED
        self.afc.save_vars()
        cur_lane.sync_to_extruder()

        # Snapshot feed_assist_count now that the extruder is synced and
        # about to pull filament.  The assist motor should engage once the
        # extruder gears start pulling, so we compare after tool_stn.
        pre_assist_count = None
        try:
            self._wait_for_ace_ready()
            pre_status = self._ace.get_status()
            pre_assist_count = pre_status.get("feed_assist_count")
        except Exception:
            pass

        # If tool_end sensor exists, feed until it triggers
        if cur_extruder.tool_end:
            tool_attempts = 0
            while not cur_extruder.tool_end_state:
                tool_attempts += 1
                afc.move_e_pos(
                    cur_lane.short_move_dis, cur_extruder.tool_load_speed,
                    "Tool end", wait_tool=True
                )
                if tool_attempts > 20:
                    message = (
                        "ACE load: filament failed to trigger post-extruder sensor.\n"
                        "To resolve, set lane loaded with "
                        f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                    )
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False

        # Push filament into the nozzle using tool_stn distance
        if cur_extruder.tool_stn:
            self.logger.info(
                f"ACE load: advancing {cur_extruder.tool_stn}mm into nozzle "
                f"@ {cur_extruder.tool_load_speed}mm/s"
            )
            afc.move_e_pos(
                cur_extruder.tool_stn, cur_extruder.tool_load_speed, "tool stn"
            )

        # Verify feed assist count increased — confirms the ACE assist
        # motor pushed filament during the load, meaning the extruder
        # gears actually engaged and pulled filament.
        # If the count did not increase, retract 20mm via ACE and retry
        # once to re-seat the filament against the extruder gears.
        if pre_assist_count is not None:
            try:
                self._wait_for_ace_ready()
                post_status = self._ace.get_status()
                post_assist_count = post_status.get("feed_assist_count")
                if post_assist_count is not None:
                    delta = post_assist_count - pre_assist_count
                    if delta > 0:
                        self.logger.info(
                            f"ACE load: feed assist engaged {delta} time(s) "
                            f"during load (count {pre_assist_count} -> {post_assist_count})"
                        )
                    else:
                        self.logger.warning(
                            f"ACE load: feed_assist_count did not increase "
                            f"during load ({pre_assist_count} -> {post_assist_count}). "
                            f"Retracting tool_stn + 20mm and retrying to engage extruder gears."
                        )
                        retry_dist = 20.0
                        retry_spd = self._quiet_speed(self.retract_speed)
                        feed_spd = self._quiet_speed(self.feed_speed)
                        # 1. Retract extruder by tool_stn to pull filament
                        #    back out of the nozzle area
                        if cur_extruder.tool_stn:
                            self.logger.info(
                                f"ACE load retry: retracting extruder "
                                f"{cur_extruder.tool_stn}mm "
                                f"@ {cur_extruder.tool_load_speed}mm/s"
                            )
                            afc.move_e_pos(
                                -cur_extruder.tool_stn,
                                cur_extruder.tool_load_speed,
                                "tool stn retract",
                            )
                        # 2. ACE unwinds 20mm to pull filament back further
                        self._wait_for_ace_ready()
                        self._ace.unwind_filament(local_slot, retry_dist, retry_spd)
                        self._wait_for_feed_complete(
                            local_slot, retry_dist, retry_spd
                        )
                        # 3. ACE re-feeds 20mm to push filament back in
                        self._wait_for_ace_ready()
                        self._ace.feed_filament(local_slot, retry_dist, feed_spd)
                        self._wait_for_feed_complete(
                            local_slot, retry_dist, feed_spd
                        )
                        # 4. Re-advance extruder by tool_stn into nozzle
                        if cur_extruder.tool_stn:
                            self.logger.info(
                                f"ACE load retry: advancing {cur_extruder.tool_stn}mm "
                                f"into nozzle @ {cur_extruder.tool_load_speed}mm/s"
                            )
                            afc.move_e_pos(
                                cur_extruder.tool_stn,
                                cur_extruder.tool_load_speed,
                                "tool stn retry",
                            )
                        # Re-check feed_assist_count after retry
                        self._wait_for_ace_ready()
                        retry_status = self._ace.get_status()
                        retry_count = retry_status.get("feed_assist_count")
                        if retry_count is not None:
                            retry_delta = retry_count - post_assist_count
                            if retry_delta > 0:
                                self.logger.info(
                                    f"ACE load retry: feed assist engaged {retry_delta} "
                                    f"time(s) after retry — extruder gears engaged."
                                )
                            else:
                                self.logger.warning(
                                    f"ACE load: feed_assist_count still did not increase "
                                    f"after retry ({post_assist_count} -> {retry_count}). "
                                    f"Filament may not be engaged with extruder gears."
                                )
            except Exception as e:
                self.logger.debug(f"ACE load: could not verify feed_assist_count: {e}")

        cur_lane.set_tool_loaded()
        cur_lane.enable_buffer(disable_fault=True)

        # Ensure feed assist is running while filament is loaded.
        # _feed_slot starts it during phase 3, but if the sensor triggered
        # early (during bulk feed) phase 3 may not have run.
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot >= 0 and self._get_feed_assist(local_slot, cur_lane):
            try:
                self._wait_for_ace_ready()
                self._ace.start_feed_assist(local_slot)
                self._feed_assist_active.add(local_slot)
                self.logger.debug(
                    f"ACE load: feed assist enabled for slot {local_slot}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE load: failed to start feed assist for slot {local_slot}: {e}"
                )

        self.afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Unload filament: shared toolhead steps then ACE retraction."""
        afc = self.afc
        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._unload_sequence_inner(cur_lane, cur_hub, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _unload_sequence_inner(self, cur_lane, cur_hub, cur_extruder):
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE unload failed: ACE not connected ({self.serial_port})",
            )
            return False

        # Disable any active buffers on the shared extruder from other units
        self._disable_extruder_buffers(cur_extruder, cur_lane)

        cur_lane.status = AFCLaneState.TOOL_UNLOADING

        # Stop feed assist before unloading - it was kept running while loaded
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot >= 0 and self._ace is not None:
            try:
                self._wait_for_ace_ready()
                self._ace.stop_feed_assist(local_slot)
                self._feed_assist_active.discard(local_slot)
                self.logger.debug(
                    f"ACE unload: feed assist stopped for slot {local_slot}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE unload: failed to stop feed assist for slot {local_slot}: {e}"
                )

        # Disable buffer before unloading (safe no-op if no buffer)
        cur_lane.disable_buffer()

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Quick pull to prevent oozing
        quick_pull_dist = 2  # mm
        afc.move_e_pos(
            -quick_pull_dist, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False
        )

        # Tighten the spool loop: command the ACE to rewind before the cut
        # so the filament rewinds cleanly onto the spool.
        if self.unload_preretract > 0:
            try:
                self._wait_for_ace_ready()
                self._ace.unwind_filament(
                    local_slot, self.unload_preretract,
                    self._quiet_speed(self.retract_speed),
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE unload: pre-cut spool tighten failed for slot {local_slot}: {e}"
                )

        self.lane_unloading(cur_lane)
        cur_lane.sync_to_extruder()
        cur_lane.do_enable(True)
        cur_lane.select_lane()

        # Shared toolhead steps: cut, park, form tip
        if afc.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            afc.gcode.run_script_from_command(
                f"{afc.tool_cut_cmd} EXTRUDER={cur_extruder.name}"
            )

            if afc.park:
                afc.gcode.run_script_from_command(
                    f"{afc.park_cmd} EXTRUDER={cur_extruder.name}"
                )

        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(
                    f"{afc.park_cmd} EXTRUDER={cur_extruder.name}"
                )

            if afc.form_tip_cmd == "AFC":
                afc.tip.tip_form()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)

        # ACE lanes have no lane stepper, so all retract moves use move_e_pos
        # (extruder motor).  Start the ACE unwind first, then do the full
        # extruder retract concurrently — both motors pull together from the
        # start, keeping constant spool tension during unload.
        local_slot = self._get_local_slot_for_lane(cur_lane)

        # Calculate full extruder retract distance
        retract_distance = cur_extruder.tool_stn_unload + 10  # +10mm margin

        # Calculate ACE unwind distance (from extruder back through hub).
        # Path: ACE → hub (dist_hub) → bowden → extruder (retract_length total)
        # Unwind = bowden + hub_clear to pull filament back through the hub combiner.
        # For real hub pins, the hub clearing loop handles the last part.
        # For virtual hubs, add hub_clear_move_dis to ensure filament clears the combiner.
        full_retract = self._get_retract_length(cur_lane)
        dist_hub = self._get_dist_hub(cur_lane)
        has_real_hub_pin = cur_hub.switch_pin.lower() != "virtual"
        if dist_hub > 0 and dist_hub < full_retract:
            retract_length = full_retract - dist_hub  # bowden length (hub to extruder)
            if not has_real_hub_pin:
                retract_length += cur_hub.hub_clear_move_dis  # pull past hub combiner
        else:
            retract_length = full_retract

        # Step 1: Start ACE unwind FIRST so it's already pulling when
        # the extruder retract begins — both motors work together from the start.
        self.logger.info(
            f"ACE unload: starting ACE unwind slot {local_slot} "
            f"{retract_length:.0f}mm (to hub) for lane {cur_lane.name}"
        )
        unload_spd = self._quiet_speed(self.retract_speed)
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(
                local_slot, retract_length, unload_spd
            )
        except Exception as e:
            message = f"ACE unload: failed to start ACE unwind for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Brief pause to let the ACE motor start pulling before the extruder
        # pushes back — prevents slack buildup at the start of the retract.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.0)

        # Step 2: Full extruder retract (stn_unload + 10mm) concurrent with ACE unwind.
        self.logger.info(
            f"ACE unload: extruder retract {retract_distance:.0f}mm "
            f"@ {cur_extruder.tool_unload_speed}mm/s (concurrent with ACE unwind)"
        )
        afc.move_e_pos(
            retract_distance * -1,
            cur_extruder.tool_unload_speed,
            "ACE nozzle retract", wait_tool=True
        )

        # Move past the sensor-after-extruder if configured
        if cur_extruder.tool_sensor_after_extruder > 0:
            afc.move_e_pos(
                cur_extruder.tool_sensor_after_extruder * -1,
                cur_extruder.tool_unload_speed, "After extruder"
            )

        # Unsync from extruder now that filament is clear of the gears
        cur_lane.unsync_to_extruder()

        try:
            self.logger.info(
                f"ACE unload: waiting for ACE unwind to complete "
                f"slot {local_slot} for lane {cur_lane.name}"
            )
            self._wait_for_feed_complete(
                local_slot, retract_length, unload_spd
            )

            # If hub has a physical sensor, retract in small steps until
            # the hub sensor clears, then retract hub_clear_move_dis extra
            # to ensure filament is fully out of the hub exit path.
            has_real_hub_pin = (
                cur_hub.switch_pin.lower() != "virtual"
            )
            if has_real_hub_pin:
                hub_clear_step = 10  # mm per check
                max_hub_clear_tries = 30
                num_tries = 0
                while cur_hub.state:
                    num_tries += 1
                    if num_tries > max_hub_clear_tries:
                        message = (
                            f"Hub sensor did not clear after "
                            f"{num_tries * hub_clear_step}mm retract "
                            f"for {cur_lane.name}. Filament may be stuck."
                        )
                        afc.error.handle_lane_failure(cur_lane, message)
                        return False
                    self.logger.debug(
                        f"ACE unload: hub still triggered, retracting "
                        f"{hub_clear_step}mm (attempt {num_tries})"
                    )
                    self._wait_for_ace_ready()
                    self._ace.unwind_filament(
                        local_slot, hub_clear_step, unload_spd
                    )
                    self._wait_for_feed_complete(
                        local_slot, hub_clear_step, unload_spd
                    )
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + 0.1
                    )

                # Hub sensor clear -- retract extra to fully clear the path
                clear_dis = cur_hub.hub_clear_move_dis
                self.logger.info(
                    f"ACE unload: hub cleared, retracting "
                    f"{clear_dis:.0f}mm hub_clear_move_dis"
                )
                self._wait_for_ace_ready()
                self._ace.unwind_filament(
                    local_slot, clear_dis, unload_spd
                )
                self._wait_for_feed_complete(
                    local_slot, clear_dis, unload_spd
                )

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = -1

            # Filament is staged near hub, ready for fast reload.
            # Clear virtual hub sensor -- filament is no longer in the
            # hub path, just nearby for convenience.
            self._set_hub_state(cur_lane, False)
            cur_lane.loaded_to_hub = True
            cur_lane.set_tool_unloaded()
            cur_lane.status = AFCLaneState.LOADED
            self.lane_tool_unloaded(cur_lane)
            self.afc.save_vars()

        except Exception as e:
            message = f"ACE unload failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        return True

    # ---- Low-Level Slot Operations ----

    def _set_hub_state(self, lane, state: bool):
        """Set the virtual hub sensor state for an ACE lane.

        Uses switch_pin_callback consistent with how OpenAMS drives
        virtual hub state, so the hub sensor and any associated
        runout_helper stay in sync.
        """
        hub = getattr(lane, "hub_obj", None)
        if hub is None:
            return
        eventtime = self.afc.reactor.monotonic()
        hub.switch_pin_callback(eventtime, state)
        fila = getattr(hub, "fila", None)
        if fila is not None:
            helper = getattr(fila, "runout_helper", None)
            if helper is not None:
                try:
                    helper.note_filament_present(eventtime, state)
                except TypeError:
                    helper.note_filament_present(state)

    def _wait_for_ace_ready(self, timeout=30.0):
        """Wait for the overall ACE status to be 'ready' before sending commands.

        After a retract/feed completes, the slot may report 'ready'/'empty'
        before the ACE controller finishes its internal housekeeping.  Sending
        a new feed_filament while the ACE is still 'busy' returns FORBIDDEN.
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
                        f"ACE: waiting for ACE ready "
                        f"(status={hw_status.get('status', '?')}, "
                        f"{elapsed:.1f}s/{timeout:.0f}s)"
                    )
            except Exception:
                pass
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval
            )
            elapsed += poll_interval
        self.logger.warning(
            f"ACE: ACE did not become ready within {timeout:.0f}s, proceeding anyway"
        )

    def _feed_slot(self, slot_index, lane=None, feed_distance=None):
        """Feed filament from an ACE slot through to the toolhead.

        Uses a three-phase approach:
        1. Bulk feed: advance most of the bowden distance in one shot (fast)
        2. Sensor approach: feed in small increments, checking the toolhead
           sensor between each step (precise, stops when sensor triggers)
        3. Feed assist + extruder assist for the final stretch into the hotend

        If no lane is provided (no sensor to check), falls back to fixed-distance feed.

        :param feed_distance: Override the total feed distance. When filament
            is already at the hub, pass only the hub-to-toolhead distance.

        Returns (total_fed, sensor_triggered, early_engage).  *early_engage*
        is True when the FPS buffer latched during a home_to_tool feed,
        signalling the caller to sync the extruder and start feed_assist
        immediately, then advance tool_stn — skipping Phase 3 here.
        """
        ace = self._ace
        feed_length = feed_distance if feed_distance is not None else self._get_feed_length(lane)

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        # Determine homing behaviour from AFC global config.  When
        # home_to_tool and homing_enabled are both True we send one
        # single continuous feed for the full distance + tool_homing_distance
        # + 20mm safety margin and poll the sensor at 100ms.  No splitting
        # into bulk + approach phases — just feed and watch.
        afc = self.afc
        homing_active = getattr(afc, 'homing_enabled', False) and getattr(afc, 'home_to_tool', False)

        feed_spd = self._quiet_speed(self.feed_speed)
        self.logger.debug(
            f"ACE feed: slot {slot_index}, "
            f"length={feed_length}mm @ {feed_spd}mm/s"
        )

        # early_engage: when True the caller should sync the extruder,
        # start feed_assist, and run tool_stn immediately — skipping
        # Phase 3 and the normal engagement path.
        early_engage = False

        if homing_active and lane is not None:
            # ── home_to_tool: single continuous feed ──────────────────
            homing_dist = getattr(afc, 'tool_homing_distance', 200.0)
            total_distance = feed_length + homing_dist + 20.0
            self.logger.info(
                f"ACE feed: home_to_tool active — single feed "
                f"{total_distance:.0f}mm (feed={feed_length:.0f} + "
                f"homing={homing_dist:.0f} + 20mm margin)"
            )

            ace.feed_filament(slot_index, total_distance, feed_spd)
            sensor_hit = self._wait_for_feed_complete(
                slot_index, total_distance, feed_spd, lane=lane,
                poll_interval=0.1
            )

            sensor_triggered = sensor_hit or lane.get_toolhead_pre_sensor_state()
            total_fed = total_distance

            if sensor_triggered:
                self.logger.info(
                    f"ACE feed: toolhead sensor triggered during "
                    f"home_to_tool feed"
                )
                # When using an FPS buffer, signal early extruder engagement
                # so the caller starts the extruder immediately without
                # sending any ACE commands first.  Feed assist will be
                # started after the extruder has engaged.
                if self._fps_obj is not None:
                    early_engage = True
                    self.logger.info(
                        "ACE feed: FPS buffer — early extruder engagement"
                    )
                else:
                    # Non-FPS sensor: re-enable feed assist now as before.
                    if self._get_feed_assist(slot_index, lane):
                        try:
                            ace.start_feed_assist(slot_index)
                            self._feed_assist_active.add(slot_index)
                        except Exception:
                            pass

        elif lane is not None:
            # ── Normal (non-homing): bulk feed then sensor approach ───
            approach_margin = self.sensor_approach_margin
            overshoot = self.max_feed_overshoot

            # Phase 1: Bulk feed (skip the last approach_margin mm)
            bulk_distance = max(0, feed_length - approach_margin)
            if bulk_distance > 0:
                ace.feed_filament(slot_index, bulk_distance, feed_spd)
                sensor_triggered_early = self._wait_for_feed_complete(
                    slot_index, bulk_distance, feed_spd, lane=lane
                )
            else:
                sensor_triggered_early = False

            total_fed = bulk_distance
            sensor_triggered = sensor_triggered_early

            if not sensor_triggered:
                # Check if sensor triggered after bulk feed settled
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
                if lane.get_toolhead_pre_sensor_state():
                    sensor_triggered = True
                    self.logger.info(
                        f"ACE feed: toolhead sensor triggered during bulk feed "
                        f"at ~{total_fed:.0f}mm"
                    )

            # Phase 2: Single continuous feed for the remaining distance
            # while polling the toolhead sensor.
            max_total = feed_length + overshoot
            remaining = max_total - total_fed
            if not sensor_triggered and remaining > 0:
                ace.feed_filament(slot_index, remaining, feed_spd)
                sensor_hit = self._wait_for_feed_complete(
                    slot_index, remaining, feed_spd, lane=lane,
                    poll_interval=0.1
                )
                total_fed = max_total

                if sensor_hit or lane.get_toolhead_pre_sensor_state():
                    sensor_triggered = True
                    self.logger.info(
                        f"ACE feed: toolhead sensor triggered at ~{total_fed:.0f}mm"
                    )
                    # Re-enable feed assist in case stop_feed_filament
                    # disabled it on the ACE firmware side.
                    if self._get_feed_assist(slot_index, lane):
                        try:
                            ace.start_feed_assist(slot_index)
                            self._feed_assist_active.add(slot_index)
                        except Exception:
                            pass

        else:
            # No lane/sensor - just feed the fixed distance
            ace.feed_filament(slot_index, feed_length, feed_spd)
            self._wait_for_feed_complete(
                slot_index, feed_length, feed_spd
            )
            total_fed = feed_length
            sensor_triggered = False

        # Phase 3: Feed assist + extruder assist for the last stretch.
        # Feed assist stays enabled after loading to maintain filament tension
        # during printing. It is stopped in _unload_sequence_inner before retraction.
        # Skip when early_engage is set — the caller will engage the extruder
        # first and start feed assist afterwards.
        if not early_engage and self._get_feed_assist(slot_index, lane):
            try:
                self._wait_for_ace_ready()
                ace.start_feed_assist(slot_index)
                self._feed_assist_active.add(slot_index)
            except Exception as e:
                self.logger.warning(
                    f"ACE feed: start_feed_assist failed for slot "
                    f"{slot_index}: {e}"
                )

            # Use extruder motor to pull filament into hotend
            if self.extruder_assist_length > 0:
                ext_spd = self._quiet_speed(self.extruder_assist_speed)
                self.afc.gcode.run_script_from_command(
                    f"G92 E0\n"
                    f"G1 E{self.extruder_assist_length} F{ext_spd}"
                )

        return total_fed, sensor_triggered, early_engage

    def _wait_for_feed_complete(self, slot_index, length_mm, speed_mm_s,
                                 lane=None, poll_interval=0.5):
        """Wait for the ACE hardware to finish a feed/unwind movement.

        The ACE acknowledges feed_filament/unwind_filament commands immediately
        via JSON-RPC, but the physical motor movement continues asynchronously.
        This method polls get_status until the slot is no longer busy, or until
        the calculated maximum movement time expires.

        If a lane with a toolhead sensor is provided, also checks the sensor
        each poll iteration and returns early if triggered.

        :param speed_mm_s: Speed in mm/s (ACE firmware native unit)
        Returns True if the sensor triggered during the wait, False otherwise.
        """
        ace = self._ace
        if ace is None or not ace.connected:
            return False

        # Calculate max wait: movement time (mm / mm_s = seconds) + generous buffer
        max_wait = (length_mm / max(speed_mm_s, 1)) + 5.0
        deadline = self.afc.reactor.monotonic() + max_wait
        sensor_triggered = False

        # Initial delay: give the ACE time to start the motor and update
        # its slot status before we begin polling. Without this, the first
        # poll can see the slot still in "ready" state (or missing status)
        # and exit immediately before the motor has begun moving.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.0)

        while self.afc.reactor.monotonic() < deadline:
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval
            )

            # Check toolhead sensor if available
            if lane is not None and lane.get_toolhead_pre_sensor_state():
                sensor_triggered = True
                self.logger.debug(
                    f"ACE wait: toolhead sensor triggered for slot {slot_index}"
                )
                # Stop the ACE feed -the motor is still running for the
                # full requested distance but we don't need any more filament.
                # Without this, the ACE stays "busy/feeding" for minutes,
                # blocking feed_assist and causing RFID reads to return empty.
                try:
                    ace.stop_feed_filament(slot_index)
                    self.logger.debug(
                        f"ACE wait: stopped feed for slot {slot_index} "
                        "(sensor triggered early)"
                    )
                except Exception:
                    pass
                return True

            # Poll ACE status to check if movement is done
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            # If slot is back to "ready" or "empty",
                            # movement is done. Empty/missing status is
                            # NOT treated as complete -it likely means the
                            # ACE hasn't updated yet.
                            if status in ("ready", "empty"):
                                self.logger.debug(
                                    f"ACE wait: slot {slot_index} movement "
                                    f"complete (status={status})"
                                )
                                return sensor_triggered
            except Exception:
                # Status poll failed, just keep waiting
                pass

        self.logger.debug(
            f"ACE wait: timeout waiting for slot {slot_index} "
            f"movement ({max_wait:.1f}s)"
        )
        return sensor_triggered

    def _feed_until_sensor(self, slot_index, lane, max_length, step_size=None):
        """Feed filament incrementally until the toolhead sensor triggers.

        Used for calibration. Feeds in small steps, checking the sensor
        between each step. Returns the total distance fed.

        Returns (distance_fed, sensor_triggered).
        """
        if step_size is None:
            step_size = self.calibration_step

        ace = self._ace
        total_fed = 0.0

        while total_fed < max_length:
            step = min(step_size, max_length - total_fed)
            # Ensure ACE is ready before each step - after the previous
            # step completes, the ACE overall status may briefly stay
            # "busy" for internal housekeeping.  Sending feed_filament
            # while busy returns FORBIDDEN and kills the calibration.
            self._wait_for_ace_ready()
            try:
                ace.feed_filament(slot_index, step, self.feed_speed)
            except Exception as e:
                self.logger.warning(
                    f"ACE calibration: feed_filament failed at "
                    f"{total_fed:.0f}mm, retrying: {e}"
                )
                self._wait_for_ace_ready(timeout=15.0)
                ace.feed_filament(slot_index, step, self.feed_speed)
            # Wait for ACE to physically complete this step
            sensor_hit = self._wait_for_feed_complete(
                slot_index, step, self.feed_speed, lane=lane,
                poll_interval=0.3
            )
            total_fed += step

            if sensor_hit or lane.get_toolhead_pre_sensor_state():
                self.logger.info(
                    f"ACE calibration: sensor triggered at {total_fed:.1f}mm"
                )
                return total_fed, True

        return total_fed, False

    def _retract_slot(self, slot_index, lane=None, to_hub=True):
        """Retract filament back into the ACE unit.

        :param to_hub: If True, only retract from toolhead to hub (leaving
            filament staged at the hub). If False, retract the full distance
            back to the spool.
        """
        ace = self._ace
        full_retract = self._get_retract_length(lane)
        dist_hub = self._get_dist_hub(lane)

        if to_hub and dist_hub > 0 and dist_hub < full_retract:
            retract_length = full_retract - dist_hub
        else:
            retract_length = full_retract

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        retract_spd = self._quiet_speed(self.retract_speed)
        self.logger.debug(
            f"ACE retract: slot {slot_index}, "
            f"length={retract_length}mm (to_hub={to_hub}) "
            f"@ {retract_spd}mm/s"
        )
        ace.unwind_filament(slot_index, retract_length, retract_spd)
        # Wait for ACE to physically complete the retraction
        self._wait_for_feed_complete(
            slot_index, retract_length, retract_spd
        )

    # ---- Abort / Cleanup ----

    def abort_load(self, cur_lane):
        """Cancel any in-progress ACE feed operation and stop feed assist.

        Sends stop commands to the ACE serial driver so that motors don't
        remain active after a load failure.  Without this the ACE stays
        'busy/feeding' and blocks subsequent operations.
        """
        ace = self._ace
        if ace is None or not ace.connected:
            return
        slot = self._get_local_slot_for_lane(cur_lane)
        if slot < 0:
            return
        try:
            self.logger.info(
                "Aborting ACE load for %s (slot %d)" % (cur_lane.name, slot))
            ace.stop_feed_filament(slot)
            ace.stop_feed_assist(slot)
            self._feed_assist_active.discard(slot)
        except Exception as e:
            self.logger.error(
                "abort_load failed for %s: %s" % (cur_lane.name, e))

    def lane_move(self, cur_lane, distance, speed_mode):
        """Move filament via ACE serial driver.

        Positive distance = feed forward, negative = rewind/retract.
        Uses the ACE feed_filament/unwind_filament serial commands.

        :param cur_lane: Lane object to move
        :param distance: Distance in mm (positive = forward, negative = retract)
        :param speed_mode: SpeedMode enum (unused — ACE uses its own feed/retract speeds)
        """
        ace = self._ace
        if ace is None or not ace.connected:
            self.logger.error("ACE not connected, cannot move %s" % cur_lane.name)
            return
        slot = self._get_local_slot_for_lane(cur_lane)
        if slot < 0:
            self.logger.error("Cannot resolve slot for %s" % cur_lane.name)
            return

        abs_distance = abs(distance)
        try:
            self._wait_for_ace_ready()
            if distance > 0:
                spd = self._quiet_speed(self.feed_speed)
                self.logger.info(
                    f"ACE lane move: feeding {cur_lane.name} slot {slot} "
                    f"{abs_distance}mm @ {spd}mm/s")
                ace.feed_filament(slot, abs_distance, spd)
            else:
                spd = self._quiet_speed(self.retract_speed)
                self.logger.info(
                    f"ACE lane move: retracting {cur_lane.name} slot {slot} "
                    f"{abs_distance}mm @ {spd}mm/s")
                ace.unwind_filament(slot, abs_distance, spd)
            self._wait_for_feed_complete(slot, abs_distance, spd, lane=cur_lane)
        except Exception as e:
            self.logger.error(
                f"ACE lane move failed for {cur_lane.name}: {e}")

    # ---- No-Op / Unsupported Operations ----

    def prep_load(self, lane):
        """No-op: ACE hardware manages filament to sensors directly."""
        pass

    def prep_post_load(self, lane):
        """Feed filament to hub after spool is detected in ACE slot.

        Mirrors BoxTurtle's prep_post_load: feeds dist_hub mm from the
        ACE slot to the hub/combiner so filament is staged for faster
        tool changes.  Only runs if load_to_hub is enabled, the lane
        is not already at the hub, and the slot shows filament ready.
        """
        # Resolve load_to_hub: unit override > lane (which defaults from
        # global) > AFC global.  Unit-level takes priority because the lane
        # config always has a value (it defaults from the AFC global, not
        # from the unit), so the lane value can never fall through to unit.
        if self._unit_load_to_hub is not None:
            load_to_hub = self._unit_load_to_hub
        else:
            load_to_hub = getattr(lane, 'load_to_hub',
                                  getattr(self.afc, 'load_to_hub', False))
        if not load_to_hub:
            return
        if lane.loaded_to_hub:
            return
        if getattr(lane, 'name', '') in self._hub_load_suppressed:
            return
        if not lane.prep_state:
            return
        if self._ace is None or not self._ace.connected:
            return

        local_slot = self._get_local_slot_for_lane(lane)
        if local_slot < 0:
            return

        dist_hub = self._get_dist_hub(lane)
        if dist_hub <= 0:
            return

        # When home_to_hub is enabled, simulate BoxTurtle behaviour:
        # 1. Bulk-feed dist_hub - hub_clear_move_dis (fast, most of the way)
        # 2. Incremental feeds checking the hub sensor between each step
        #    (if the hub has a physical sensor, not virtual)
        # 3. Once sensor triggers (or distance exhausted), retract
        #    hub_clear_move_dis so filament clears the hub exit
        afc = self.afc
        homing_hub = (getattr(afc, 'homing_enabled', False)
                      and getattr(afc, 'home_to_hub', False))
        hub = getattr(lane, 'hub_obj', None)
        hub_clear = getattr(hub, 'hub_clear_move_dis', 0) if hub else 0
        has_physical_hub_sensor = (
            hub is not None
            and getattr(hub, 'switch_pin', 'virtual').lower() != 'virtual'
        )

        if homing_hub and hub_clear > 0 and has_physical_hub_sensor:
            bulk_dist = max(0, dist_hub - hub_clear)
            approach_dist = hub_clear
            overshoot = hub_clear
            self.logger.info(
                f"ACE prep_post_load: home_to_hub active — bulk "
                f"{bulk_dist:.0f}mm + approach {approach_dist:.0f}mm "
                f"(+{overshoot:.0f}mm overshoot) then backoff "
                f"{hub_clear:.0f}mm for {lane.name} (physical hub sensor)"
            )
        elif homing_hub and hub_clear > 0:
            # Virtual hub with homing: single move of dist_hub - hub_clear.
            # No incremental approach / backoff since there is no physical
            # sensor to trigger — the net position is the same but we skip
            # the pointless feed-then-retract dance.
            bulk_dist = max(0, dist_hub - hub_clear)
            approach_dist = 0
            overshoot = 0
            self.logger.info(
                f"ACE prep_post_load: feeding slot {local_slot} "
                f"{bulk_dist:.0f}mm to hub for {lane.name} (virtual hub)"
            )
        else:
            bulk_dist = dist_hub
            approach_dist = 0
            overshoot = 0
            self.logger.info(
                f"ACE prep_post_load: feeding slot {local_slot} "
                f"{dist_hub:.0f}mm to hub for {lane.name}"
            )

        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self._wait_for_ace_ready(timeout=30.0)
                hub_feed_spd = self._quiet_speed(self.feed_speed)

                # Phase 1: Bulk feed (most of dist_hub)
                if bulk_dist > 0:
                    self._ace.feed_filament(local_slot, bulk_dist, hub_feed_spd)
                    self._wait_for_feed_complete(
                        local_slot, bulk_dist, hub_feed_spd
                    )

                # Phase 2: Incremental approach checking hub sensor
                if homing_hub and (approach_dist + overshoot) > 0:
                    step = self.sensor_step  # reuse sensor_step (40mm default)
                    total_approach = 0
                    max_approach = approach_dist + overshoot
                    hub_triggered = False

                    while total_approach < max_approach:
                        cur_step = min(step, max_approach - total_approach)
                        self._wait_for_ace_ready()
                        self._ace.feed_filament(
                            local_slot, cur_step, hub_feed_spd
                        )
                        self._wait_for_feed_complete(
                            local_slot, cur_step, hub_feed_spd,
                            poll_interval=0.3
                        )
                        total_approach += cur_step
                        afc.reactor.pause(afc.reactor.monotonic() + 0.2)

                        # Check physical hub sensor
                        if has_physical_hub_sensor and hub.state:
                            hub_triggered = True
                            self.logger.info(
                                f"ACE prep_post_load: hub sensor triggered "
                                f"at {bulk_dist + total_approach:.0f}mm"
                            )
                            break

                    if not hub_triggered and has_physical_hub_sensor:
                        self.logger.warning(
                            f"ACE prep_post_load: hub sensor did not trigger "
                            f"after {bulk_dist + total_approach:.0f}mm for "
                            f"{lane.name}"
                        )

                    # Phase 3: Backoff hub_clear_move_dis so filament
                    # clears the hub exit, matching BoxTurtle positioning
                    self._wait_for_ace_ready()
                    retract_spd = self._quiet_speed(self.feed_speed)
                    self._ace.unwind_filament(
                        local_slot, hub_clear, retract_spd
                    )
                    self._wait_for_feed_complete(
                        local_slot, hub_clear, retract_spd
                    )
                    self.logger.info(
                        f"ACE prep_post_load: retracted {hub_clear:.0f}mm "
                        f"hub_clear backoff for {lane.name}"
                    )
                elif not homing_hub:
                    # No homing: just feed the remaining distance
                    remaining = dist_hub - bulk_dist
                    if remaining > 0:
                        self._wait_for_ace_ready()
                        self._ace.feed_filament(
                            local_slot, remaining, hub_feed_spd
                        )
                        self._wait_for_feed_complete(
                            local_slot, remaining, hub_feed_spd
                        )

                lane.loaded_to_hub = True
                self.afc.save_vars()
                self.logger.info(
                    f"ACE prep_post_load: {lane.name} staged at hub "
                    f"(dist_hub={dist_hub:.0f}mm)"
                )
                return
            except Exception as e:
                if attempt < max_attempts - 1:
                    wait = 5.0 * (attempt + 1)
                    self.logger.warning(
                        f"ACE prep_post_load: failed on attempt "
                        f"{attempt + 1}/{max_attempts}, retrying in "
                        f"{wait:.0f}s: {e}"
                    )
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + wait
                    )
                    continue
                self.logger.error(
                    f"ACE prep_post_load failed for {lane.name} after "
                    f"{max_attempts} attempts: {e}"
                )

    def eject_lane(self, lane):
        """Retract filament from hub back to spool for ACE lane.

        Retracts dist_hub distance to pull filament back into the ACE slot.
        If filament is at the toolhead, use TOOL_UNLOAD instead.
        """
        lane_name = getattr(lane, "name", "unknown")

        if getattr(lane, "tool_loaded", False):
            message = (
                f"ACE lane {lane_name} is loaded to toolhead. "
                "Use TOOL_UNLOAD to unload from toolhead first."
            )
            self.logger.info(message)
            return

        if self._ace is None or not self._ace.connected:
            self.logger.error(f"ACE eject_lane: ACE not connected for {lane_name}")
            return

        local_slot = self._get_local_slot_for_lane(lane)
        if local_slot < 0:
            return

        # Check if slot has filament -- don't gate on loaded_to_hub alone
        # since that flag can get out of sync with the physical state.
        slot_info = self._slot_inventory[local_slot] if local_slot < len(self._slot_inventory) else {}
        slot_ready = slot_info.get("status", "") == "ready"
        if not lane.loaded_to_hub and not slot_ready:
            self.logger.info(f"ACE lane {lane_name} is not loaded to hub and slot is not ready, nothing to eject.")
            return

        if not lane.loaded_to_hub:
            self.logger.warning(
                f"ACE eject_lane: {lane_name} loaded_to_hub is False but slot "
                f"is ready -- retracting anyway"
            )

        dist_hub = self._get_dist_hub(lane) + 400
        self.logger.info(
            f"ACE eject_lane: retracting slot {local_slot} "
            f"{dist_hub:.0f}mm from hub for {lane_name}"
        )
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(local_slot, dist_hub, self.retract_speed)
            self._wait_for_feed_complete(
                local_slot, dist_hub, self.retract_speed
            )
            lane.loaded_to_hub = False
            self._set_hub_state(lane, False)
            self._hub_load_suppressed.add(lane_name)
            self.afc.save_vars()
            self.logger.info(f"ACE lane {lane_name} retracted from hub to spool")
        except Exception as e:
            self.logger.error(f"ACE eject_lane failed for {lane_name}: {e}")

    def lane_unload(self, cur_lane):
        """Full ACE lane unload: retract from hub and clean up state.

        Returns True to prevent the generic LANE_UNLOAD path from also
        running (which would double-call eject_lane and apply BoxTurtle
        cleanup logic that doesn't apply to ACE).
        """
        lane_name = getattr(cur_lane, "name", "unknown")

        cur_lane.status = AFCLaneState.EJECTING
        self.afc.save_vars()

        self.eject_lane(cur_lane)

        cur_lane.loaded_to_hub = False
        cur_lane.status = AFCLaneState.NONE
        self.afc.save_vars()

        # Remove spool association since it was ejected
        self.afc.spool.set_spoolID(cur_lane, None)
        self.logger.info(f"LANE {lane_name} eject done")
        self.lane_not_ready(cur_lane)

        return True

    def get_lane_reset_command(self, lane, dis) -> str:
        """ACE lanes retract via ACE hardware, bypassing TOOL_UNLOAD."""
        return f"ACE_LANE_RESET UNIT={self.name} LANE={lane.name}"

    def get_current_lane_fallback(self, tool_obj):
        """Return lane_loaded even when the shuttle is empty.

        ACE's on_shuttle() returns False when no tool is detected (e.g. after a
        power cycle with the shuttle parked).  This causes CHANGE_TOOL to skip
        TOOL_UNLOAD, leaving stale filament in the hub/bowden path.  Returning
        lane_loaded here lets CHANGE_TOOL properly trigger the unload sequence.
        """
        return tool_obj.lane_loaded

    # ---- System Test / PREP ----

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate ACE lane state during PREP without attempting motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        # Query slot status directly from hardware (cache may not be populated yet)
        if self._ace is not None and self._ace.connected:
            local_slot = self._get_local_slot_for_lane(cur_lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                try:
                    hw_status = self._ace.get_status(timeout=2.0)
                    if isinstance(hw_status, dict):
                        slots = hw_status.get("slots", [])
                        for i, slot_data in enumerate(slots):
                            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                                self._slot_inventory[i]["status"] = slot_data.get("status", "")
                except Exception as e:
                    self.logger.debug(f"ACE {self.name}: get_status failed during PREP: {e}")

                slot_info = self._slot_inventory[local_slot]
                slot_ready = slot_info.get("status", "") == "ready"
                cur_lane._load_state = slot_ready
                cur_lane.prep_state = slot_ready

                # If slot is empty, filament can't be at the hub.
                # If slot is ready, preserve the persisted loaded_to_hub
                # value  -  filament may be staged at the hub from
                # prep_post_load or a prior tool change.
                if not slot_ready:
                    cur_lane.loaded_to_hub = False

                # Apply filament defaults if lane doesn't have values set
                self._apply_slot_filament_defaults(cur_lane, slot_info)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.lane_not_ready(cur_lane)
                msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
            else:
                self.lane_fault(cur_lane)
                msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                cur_lane.do_enable(False)
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

                # If load_to_hub is enabled, assume filament is already
                # at the hub on startup so prep_post_load doesn't re-feed
                # and push filament further on every restart.
                if not cur_lane.tool_loaded and not cur_lane.loaded_to_hub:
                    if self._unit_load_to_hub is not None:
                        load_to_hub = self._unit_load_to_hub
                    else:
                        load_to_hub = getattr(cur_lane, 'load_to_hub',
                                              getattr(self.afc, 'load_to_hub', False))
                    if load_to_hub:
                        cur_lane.loaded_to_hub = True

                if cur_lane.tool_loaded:
                    # Filament is in the toolhead, so it's also in the hub path
                    cur_lane.loaded_to_hub = True
                    # Set virtual hub sensor -- filament is actively through hub
                    self._set_hub_state(cur_lane, True)
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                        or cur_lane.extruder_obj.on_shuttle()
                    )
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        if cur_lane.extruder_obj.tc_unit_obj:
                            on_shuttle = " and toolhead on shuttle" if cur_lane.extruder_obj.on_shuttle() else ""
                        msg += f'<span class=primary--text> in ToolHead{on_shuttle}</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>'

                        # Restore combined mode tracking regardless of shuttle
                        # state so ACE knows which slot is loaded for the next
                        # tool change even when the shuttle starts empty.
                        if self.mode == MODE_COMBINED:
                            local_slot = self._get_local_slot_for_lane(cur_lane)
                            if local_slot >= 0:
                                self._current_loaded_slot = local_slot

                        # Use persisted lane_loaded to determine current
                        # lane rather than get_current_lane() which depends
                        # on klipper's active extruder — unreliable during
                        # prep before the toolchanger initializes.
                        if cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED

                            # Start feed assist so ACE pushes filament
                            # immediately  -  don't wait for the periodic
                            # refresh (~15 s) which could starve the
                            # extruder if a print resumes right away.
                            fa_slot = self._get_local_slot_for_lane(cur_lane)
                            if (fa_slot >= 0
                                    and self._get_feed_assist_for_slot(fa_slot)
                                    and self._ace is not None):
                                try:
                                    self._ace.start_feed_assist(fa_slot)
                                    self._feed_assist_active.add(fa_slot)
                                    self.logger.info(
                                        f"PREP: feed assist started for "
                                        f"slot {fa_slot} ({cur_lane.name})"
                                    )
                                except Exception as e:
                                    self.logger.debug(
                                        f"PREP: feed assist start failed "
                                        f"for slot {fa_slot}: {e}"
                                    )
                        else:
                            cur_lane.unit_obj.lane_tool_loaded_idle(cur_lane)

                        cur_lane.enable_buffer()
                    elif tool_ready:
                        msg += (
                            '<span class=error--text> error in ToolHead. '
                            'Lane identified as loaded but not identified as loaded in extruder</span>'
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
        # After startup prep, feed loaded lanes to hub if configured.
        # Must happen after set_afc_prep_done so the lane is fully ready.
        if succeeded and cur_lane.prep_state and not cur_lane.tool_loaded:
            try:
                self.prep_post_load(cur_lane)
            except Exception as e:
                self.logger.error(
                    f"PREP: prep_post_load error for {cur_lane.name}: {e}"
                )
        return succeeded

    # ---- Calibration UI overrides ----

    def _hub_is_virtual(self):
        """Return True when the unit's hub uses a virtual sensor."""
        if self.hub_obj is None:
            return True
        return getattr(self.hub_obj, 'switch_pin', 'virtual').lower() == 'virtual'

    def cmd_UNIT_CALIBRATION(self, gcmd):
        """Override base calibration menu to show ACE-specific options."""
        prompt = AFCprompt(gcmd, self.logger)
        title = f"{self.name} Calibration"
        text = "Select ACE calibration type"
        buttons = []

        # Per-lane full PTFE length (slot to toolhead)
        buttons.append(("Calibrate Lanes", f"UNIT_LANE_CALIBRATION UNIT={self.name}", "primary"))

        # Unit-level bowden length (fallback when per-lane not set)
        lanes_loaded = any(lane.load_state and not lane.is_direct_hub() for lane in self.lanes.values())
        direct_hubs = any(lane.is_direct_hub() for lane in self.lanes.values())
        if not direct_hubs or lanes_loaded:
            buttons.append(("Calibrate Bowden Length", f"UNIT_BOW_CALIBRATION UNIT={self.name}", "secondary"))

        # Hub calibration (slot to hub sensor) - only if physical hub
        if not self._hub_is_virtual():
            buttons.append(("Calibrate Hub", f"ACE_HUB_CALIBRATION UNIT={self.name}", "secondary"))

        # TD-1 calibration
        any_lane_has_td1_ids = any(lane.td1_device_id for lane in self.lanes.values())
        if self.afc.td1_defined and any_lane_has_td1_ids:
            buttons.append(("Calibrate TD-1 Length", f"AFC_UNIT_TD_ONE_CALIBRATION UNIT={self.name}", "secondary"))

        back = [("Back to unit selection", "AFC_CALIBRATION", "info")]
        prompt.create_custom_p(title, text, None, True, [buttons], back)

    def cmd_UNIT_BOW_CALIBRATION(self, gcmd):
        """Override base bowden calibration prompt for ACE units.

        ACE stores per-lane feed_length/retract_length.  The unit-level bowden
        calibration is kept as a fallback when per-lane values are not set.
        """
        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"Bowden Calibration {self.name}"
        text = (
            f"Select a loaded lane from {self.name} to measure Bowden length. "
            "This sets the unit-level feed_length / retract_length used when "
            "per-lane values are not configured."
        )

        for lane in self.lanes.values():
            if lane.load_state and not lane.is_direct_hub():
                button_label = f"{lane}"
                button_command = f"CALIBRATE_AFC BOWDEN={lane}"
                button_style = "primary" if index % 2 == 0 else "secondary"
                group_buttons.append((button_label, button_command, button_style))

                if (index + 1) % 2 == 0 or index == len(self.lanes) - 1:
                    buttons.append(list(group_buttons))
                    group_buttons = []
                index += 1

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
        prompt.create_custom_p(title, text, None, True, buttons, back)

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        """Override base lane calibration prompt for ACE units.

        On ACE, lane calibration measures the full PTFE length (slot to
        toolhead) and saves per-lane feed_length / retract_length.
        """
        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"{self.name} Lane Calibration"
        text = (
            f"Select a loaded lane from {self.name} to calibrate. "
            "Each lane is measured individually (slot to toolhead). "
            "Config option: feed_length / retract_length"
        )

        for lane in self.lanes.values():
            if lane.load_state and not lane.tool_loaded:
                button_label = f"{lane}"
                button_command = f"CALIBRATE_AFC LANE={lane}"
                button_style = "primary" if index % 2 == 0 else "secondary"
                group_buttons.append((button_label, button_command, button_style))

                if (index + 1) % 2 == 0 or index == len(self.lanes) - 1:
                    buttons.append(list(group_buttons))
                    group_buttons = []
                index += 1

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons > 1:
            all_lanes = [("All lanes", f"CALIBRATE_AFC LANE=all UNIT={self.name}", "default")]
        else:
            all_lanes = None
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
        prompt.create_custom_p(title, text, all_lanes, True, buttons, back)

    def cmd_ACE_HUB_CALIBRATION(self, gcmd):
        """Prompt to calibrate dist_hub (slot to hub sensor) per lane.

        Only available when the unit has a physical hub sensor.
        """
        if self._hub_is_virtual():
            prompt = AFCprompt(gcmd, self.logger)
            title = f"Hub Calibration {self.name}"
            text = (
                f"{self.name} has a virtual hub. Hub calibration requires a physical hub sensor. "
                "Set dist_hub manually in config."
            )
            back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
            prompt.create_custom_p(title, text, None, True, None, back)
            return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"Hub Calibration {self.name}"
        text = (
            f"Select a loaded lane from {self.name} to measure distance from slot to hub. "
            "Each lane is calibrated individually. "
            "Config option: dist_hub"
        )

        for lane in self.lanes.values():
            if lane.load_state and not lane.tool_loaded:
                button_label = f"{lane}"
                button_command = f"ACE_CALIBRATE_HUB UNIT={self.name} LANE={lane.name}"
                button_style = "primary" if index % 2 == 0 else "secondary"
                group_buttons.append((button_label, button_command, button_style))

                if (index + 1) % 2 == 0 or index == len(self.lanes) - 1:
                    buttons.append(list(group_buttons))
                    group_buttons = []
                index += 1

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
        prompt.create_custom_p(title, text, None, True, buttons, back)

    def cmd_AFC_UNIT_TD_ONE_CALIBRATION(self, gcmd):
        """Override base TD-1 calibration prompt for ACE units.

        Each lane is calibrated individually.  Virtual hubs block calibration.
        """
        if self._hub_is_virtual():
            prompt = AFCprompt(gcmd, self.logger)
            title = f"TD-1 Bowden Calibration {self.name}"
            text = (
                f"{self.name} has a virtual hub. TD-1 calibration requires a physical hub sensor. "
                "Set td1_bowden_length manually in config."
            )
            back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
            prompt.create_custom_p(title, text, None, True, None, back)
            return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"TD-1 Bowden Calibration {self.name}"
        text = (
            f"Select a loaded lane from {self.name} to measure Bowden length to your TD-1 Device. "
            "Each lane is calibrated individually. "
            "WARNING: This could take some time to complete. "
            "Config option: td1_bowden_length"
        )

        for lane in self.lanes.values():
            if lane.td1_device_id and lane.load_state:
                button_label = f"{lane}"
                button_command = f"CALIBRATE_AFC TD1={lane} DISTANCE=50"
                button_style = "primary" if index % 2 == 0 else "secondary"
                group_buttons.append((button_label, button_command, button_style))

                if (index + 1) % 2 == 0 or index == len(self.lanes) - 1:
                    buttons.append(list(group_buttons))
                    group_buttons = []
                index += 1

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]
        prompt.create_custom_p(title, text, None, True, buttons, back)

    # ---- Calibration ----

    def calibrate_bowden(self, cur_lane, dis, tol):
        """Calibrate bowden length by feeding until toolhead sensor triggers."""
        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._calibrate_bowden_inner(cur_lane, dis, tol)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _measure_bowden_distance(self, cur_lane):
        """Feed filament until toolhead sensor triggers and retract.

        Returns (success, distance, new_feed_length, new_retract_length).
        On failure, distance is how far was fed before giving up.
        """
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Clear FPS advance latch from any previous operation so the
        # sensor check below reads real-time pressure, not a stale latch.
        buffer_obj = getattr(cur_lane, 'buffer_obj', None)
        if buffer_obj is not None and hasattr(buffer_obj, 'clear_advance_latch'):
            buffer_obj.clear_advance_latch()
            # Brief pause for ADC to update with current pressure
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)

        # Don't calibrate if sensor is already triggered
        if cur_lane.get_toolhead_pre_sensor_state():
            return False, "Toolhead sensor already triggered - unload first", 0

        # ACE always needs the full range -the 'dis' parameter from
        # CALIBRATE_AFC DISTANCE=N is designed for stepper-based units and
        # is far too small (default 25mm) for bowden-length calibration.
        max_distance = 6000

        self.logger.info(
            f"ACE calibrate: feeding slot {local_slot} "
            f"in {self.calibration_step}mm steps, max {max_distance}mm"
        )

        distance, triggered = self._feed_until_sensor(
            local_slot, cur_lane, max_distance, step_size=self.calibration_step
        )

        # Retract the full detected distance plus a small margin to ensure
        # filament is fully clear of the bowden path after calibration.
        retract_dist = distance + 5 if triggered else distance
        self.logger.info(
            f"ACE calibrate: retracting {retract_dist:.0f}mm "
            f"@ {self.retract_speed}mm/s"
        )
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(local_slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(
                local_slot, retract_dist, self.retract_speed
            )
            self.logger.info("ACE calibrate: retract complete")
            # Wait for ACE to finish internal rewind/housekeeping so the hub
            # is clear before the next lane's calibration starts.
            self._wait_for_ace_ready(timeout=15.0)

            # Clear the FPS latch and tool_start_state so the next lane's
            # calibration doesn't see a stale "sensor triggered" from this
            # lane's filament.  Then pause to let the FPS ADC callback run
            # at least one cycle (~100ms) with no filament present so the
            # sensor state reflects reality.
            self._fps_latched = False
            self._fps_consecutive_hits = 0
            # Allow sensor polling to catch up and hub to physically clear
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + 3.0
            )
            self.logger.info("ACE calibrate: post-retract settle complete")
        except Exception as e:
            self.logger.error(f"ACE calibrate: retract failed: {e}")

        if not triggered:
            msg = (
                f"Toolhead sensor did not trigger after {distance:.0f}mm. "
                "Check filament path and sensor wiring."
            )
            return False, msg, distance

        # If filament was already staged at the hub, the measurement only
        # covers hub→toolhead.  Add dist_hub back so the returned distance
        # represents the full slot→toolhead path.  This prevents the loading
        # code (which subtracts dist_hub when loaded_to_hub is True) from
        # double-subtracting.
        if cur_lane.loaded_to_hub:
            dist_hub = self._get_dist_hub(cur_lane)
            if dist_hub > 0:
                self.logger.info(
                    f"ACE calibrate: filament was pre-loaded to hub, "
                    f"adding dist_hub={dist_hub:.0f}mm to measured "
                    f"{distance:.1f}mm"
                )
                distance += dist_hub

        return True, "", distance

    def _calibrate_bowden_inner(self, cur_lane, dis, tol):
        """Calibrate bowden and save to the UNIT config section."""
        success, msg, distance = self._measure_bowden_distance(cur_lane)
        if not success:
            return False, msg, distance

        new_feed_length = round(distance, 0)
        new_retract_length = round(distance - 20, 0)

        # Update in-memory unit-level values
        old_feed = self.feed_length
        old_retract = self.retract_length
        self.feed_length = new_feed_length
        self.retract_length = new_retract_length

        # Write to the unit config section [AFC_ACE Ace_1]
        unit_section = " ".join(self.full_name)
        cal_msg = f"\n feed_length: New: {new_feed_length} Old: {old_feed}"
        self.afc.function.ConfigRewrite(
            unit_section, "feed_length", new_feed_length, cal_msg
        )
        cal_msg = f"\n retract_length: New: {new_retract_length} Old: {old_retract}"
        self.afc.function.ConfigRewrite(
            unit_section, "retract_length", new_retract_length, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE bowden calibration: toolhead sensor triggered at {distance:.1f}mm.\n"
            f"feed_length: {new_feed_length:.0f} (was {old_feed:.0f})\n"
            f"retract_length: {new_retract_length:.0f} (was {old_retract:.0f})\n"
            f"Values saved to unit config [{unit_section}]."
        )
        return True, msg, distance

    def calibrate_hub(self, cur_lane, tol):
        """Calibrate dist_hub by feeding until hub sensor triggers.

        Requires a physical hub sensor (not virtual). Feeds filament in
        calibration_step increments, checking the hub sensor after each
        step. Saves the measured distance as dist_hub in the lane config.
        """
        hub_obj = getattr(cur_lane, 'hub_obj', None)
        if hub_obj is None:
            return False, "No hub object for this lane", 0

        # Virtual hubs can't be calibrated  -  user must set dist_hub manually
        if getattr(hub_obj, 'switch_pin', 'virtual').lower() == 'virtual':
            return False, (
                "Hub calibration requires a physical hub sensor. "
                "For virtual hubs, set dist_hub manually in config."
            ), 0

        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._calibrate_hub_inner(cur_lane, hub_obj)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _calibrate_hub_inner(self, cur_lane, hub_obj):
        """Feed until hub sensor triggers and save dist_hub."""
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Don't calibrate if hub sensor is already triggered
        if hub_obj.state:
            return False, "Hub sensor already triggered - clear the hub first", 0

        max_distance = 3000  # max reasonable slot-to-hub distance
        step = self.calibration_step
        total_fed = 0.0
        sensor_triggered = False

        self.logger.info(
            f"ACE hub calibrate: feeding slot {local_slot} "
            f"in {step}mm steps, max {max_distance}mm"
        )

        while total_fed < max_distance:
            feed_step = min(step, max_distance - total_fed)
            try:
                self._wait_for_ace_ready()
                self._ace.feed_filament(local_slot, feed_step, self.feed_speed)
                self._wait_for_feed_complete(
                    local_slot, feed_step, self.feed_speed
                )
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: feed failed: {e}")
                break
            total_fed += feed_step

            # Check hub sensor
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            if hub_obj.state:
                sensor_triggered = True
                self.logger.info(
                    f"ACE hub calibrate: hub sensor triggered at {total_fed:.1f}mm"
                )
                break

        # Retract what we fed (minus a small amount so filament stays near hub)
        retract_dist = total_fed - 50 if sensor_triggered else total_fed
        if retract_dist > 0:
            try:
                self._wait_for_ace_ready()
                self._ace.unwind_filament(local_slot, retract_dist, self.retract_speed)
                self._wait_for_feed_complete(
                    local_slot, retract_dist, self.retract_speed
                )
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: retract failed: {e}")

        if not sensor_triggered:
            return False, (
                f"Hub sensor did not trigger after {total_fed:.0f}mm. "
                "Check filament path and hub sensor wiring."
            ), total_fed

        # Save calibrated dist_hub with 50mm safety margin so
        # load-to-hub feeds don't jam filament into/past the hub sensor.
        lane_name = cur_lane.name
        hub_clearance = 50
        new_dist_hub = round(total_fed - hub_clearance, 0)
        old_dist_hub = self._lane_dist_hub.get(lane_name, self.dist_hub)

        # Update in-memory per-lane override
        self._lane_dist_hub[lane_name] = new_dist_hub

        # Write to lane config
        lane_section = f"AFC_lane {lane_name}"
        cal_msg = f"\n dist_hub: New: {new_dist_hub} Old: {old_dist_hub}"
        self.afc.function.ConfigRewrite(
            lane_section, "dist_hub", new_dist_hub, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE hub calibration: hub sensor triggered at {total_fed:.1f}mm.\n"
            f"dist_hub: {new_dist_hub:.0f} ({total_fed:.0f} - {hub_clearance}mm clearance)"
            f" (was {old_dist_hub:.0f})\n"
            f"Value saved to lane config [{lane_section}]."
        )
        return True, msg, total_fed

    def calibrate_lane(self, cur_lane, tol):
        """Calibrate per-lane bowden length and save to the lane config section.

        On stepper units, calibrate_lane measures extruder-to-hub distance.
        On ACE, it measures ACE slot to toolhead and writes the result to
        the [AFC_lane] section so each lane can have its own feed/retract length.
        """
        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._calibrate_lane_inner(cur_lane, tol)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _calibrate_lane_inner(self, cur_lane, tol):
        success, msg, distance = self._measure_bowden_distance(cur_lane)
        if not success:
            return False, msg, distance

        lane_name = cur_lane.name

        # _measure_bowden_distance already adds dist_hub when filament
        # starts at the hub, so 'distance' is the full ACE-to-extruder path.
        new_feed_length = round(distance, 0)
        new_retract_length = round(distance - 20, 0)

        # Update in-memory per-lane overrides
        old_feed = self._lane_feed_length.get(lane_name, self.feed_length)
        old_retract = self._lane_retract_length.get(lane_name, self.retract_length)
        self._lane_feed_length[lane_name] = new_feed_length
        self._lane_retract_length[lane_name] = new_retract_length

        # Write calibrated values to the lane config section
        lane_section = f"AFC_lane {lane_name}"
        cal_msg = f"\n feed_length: New: {new_feed_length} Old: {old_feed}"
        self.afc.function.ConfigRewrite(
            lane_section, "feed_length", new_feed_length, cal_msg
        )
        cal_msg = f"\n retract_length: New: {new_retract_length} Old: {old_retract}"
        self.afc.function.ConfigRewrite(
            lane_section, "retract_length", new_retract_length, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE lane calibration: toolhead sensor triggered at {distance:.1f}mm.\n"
            f"feed_length: {new_feed_length:.0f} (was {old_feed:.0f})\n"
            f"retract_length: {new_retract_length:.0f} (was {old_retract:.0f})\n"
            f"Values saved to lane config [{lane_section}]."
        )
        return True, msg, distance

    def calibrate_td1(self, cur_lane, dis, tol):
        """Calibrate TD-1 bowden length by feeding until TD-1 device detects filament."""
        self._operation_active = True
        self._fps_latched = False
        self._fps_consecutive_hits = 0
        try:
            return self._calibrate_td1_inner(cur_lane, dis, tol)
        finally:
            self._operation_active = False
            self._prev_states_stale = True
            self._fps_latched = False
            self._fps_consecutive_hits = 0

    def _calibrate_td1_inner(self, cur_lane, dis, tol):
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        # Validate TD-1 device ID
        if cur_lane.td1_device_id is None:
            msg = (
                f"Cannot calibrate TD-1 for {cur_lane.name}, td1_device_id is a required "
                "field in AFC_hub or per AFC_lane"
            )
            return False, msg, 0

        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            msg = (
                f"TD-1 device(SN: {cur_lane.td1_device_id}) not detected anymore, "
                "please check before continuing to calibrate TD-1 bowden length"
            )
            return False, msg, 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Use calibration_step if dis is not specified
        step_size = dis if dis > 0 else self.calibration_step
        max_bowden_length = 6000

        self.logger.info(
            f"ACE calibrate_td1: feeding slot {local_slot} in {step_size}mm steps, "
            f"max {max_bowden_length}mm, TD-1 device={cur_lane.td1_device_id}"
        )

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        # Feed incrementally until TD-1 detects filament
        bow_pos = 0.0
        compare_time = datetime.now()
        while not self.get_td1_data(cur_lane, compare_time):
            if bow_pos > max_bowden_length:
                # Retract what we fed
                self._retract_slot(local_slot, lane=cur_lane)
                msg = f"TD-1 failed to detect filament after moving {bow_pos:.0f}mm"
                return False, msg, bow_pos

            compare_time = datetime.now()
            bow_pos += step_size
            self._ace.feed_filament(local_slot, step_size, self.feed_speed)

            # Brief pause for TD-1 to register
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        self.logger.info(
            f"ACE calibrate_td1: TD-1 detected filament at {bow_pos:.1f}mm"
        )

        # Retract back to ACE unit using measured distance
        retract_dist = bow_pos + 50
        self.logger.info(
            f"ACE calibrate_td1: retracting {retract_dist:.0f}mm "
            f"@ {self.retract_speed}mm/s"
        )
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(local_slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(
                local_slot, retract_dist, self.retract_speed
            )
            self.logger.info("ACE calibrate_td1: retract complete")
        except Exception as e:
            self.logger.error(f"ACE calibrate_td1: retract failed: {e}")

        # Save td1_bowden_length to the lane's config section
        old_td1 = getattr(cur_lane, "td1_bowden_length", None)
        cur_lane.td1_bowden_length = bow_pos
        fullname = cur_lane.fullname
        cal_msg = f"\n td1_bowden_length: New: {bow_pos} Old: {old_td1}"
        self.afc.function.ConfigRewrite(
            fullname, "td1_bowden_length", bow_pos, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE TD-1 calibration: filament detected at {bow_pos:.1f}mm.\n"
            f"td1_bowden_length: {bow_pos:.0f} (was {old_td1})\n"
            f"Value saved to config."
        )
        return True, msg, bow_pos

    # ---- Runout Detection ----

    def check_runout(self, cur_lane):
        """ACE supports runout detection during printing."""
        try:
            return self.afc.function.is_printing()
        except Exception:
            return False

    def _ace_pause_runout(self, lane):
        """ACE-specific pause runout when no runout lane is configured.

        When unload_on_runout is enabled, automatically runs TOOL_UNLOAD
        to retract filament from the toolhead and then LANE_UNLOAD to
        eject filament from the hub back to the ACE slot.

        When unload_on_runout is disabled, pauses the print and tells
        the user which commands to run to clear the filament path.
        """
        lane_name = getattr(lane, "name", "unknown")

        if self.unload_on_runout:
            self.logger.info(
                f"ACE runout on {lane_name}: auto-unloading from "
                f"toolhead and ejecting lane"
            )
            self.afc.error.pause_resume.send_pause_command()
            self.afc.save_pos()
            self.afc.TOOL_UNLOAD(lane)
            if not self.afc.error_state:
                self.afc.LANE_UNLOAD(lane)
        else:
            self.logger.info(
                f"ACE runout on {lane_name}: pausing print "
                f"(unload_on_runout disabled)"
            )

        lane.status = AFCLaneState.NONE
        self.lane_not_ready(lane)

        if self.unload_on_runout:
            msg = (
                f"Runout detected on {lane_name}. "
                f"Filament has been automatically unloaded.\n"
                f"Please load a new spool and resume the print."
            )
        else:
            msg = (
                f"Runout detected on {lane_name} and no runout lane "
                f"is configured.\n"
                f"To clear the filament path, run:\n"
                f"  TOOL_UNLOAD LANE={lane_name}\n"
                f"Then manually remove the remaining filament, "
                f"load a new spool, and resume the print."
            )
        self.afc.error.AFC_error(msg)

    def _start_slot_status_monitor(self):
        """Start periodic polling for slot status changes (runout detection)."""
        self._prev_slot_states = {}
        # Track when we last did a full inventory sync (RFID data)
        self._last_inventory_sync = 0.0
        self._inventory_sync_interval = 30.0  # Full RFID sync every 30s
        self._slot_monitor_timer = self.afc.reactor.register_timer(
            self._poll_slot_status,
            self.afc.reactor.monotonic() + 5.0,
        )
        self.logger.info(
            f"ACE {self.name}: started slot status monitor "
            f"(interval={self.poll_interval}s)"
        )

    def _poll_slot_status(self, eventtime):
        """Periodic callback to detect slot status changes during printing.

        Uses a single get_status call per poll to check slot states.
        Full inventory sync (4x get_filament_info) only runs infrequently.
        """
        if self._ace is None or not self._ace.connected:
            return eventtime + self.poll_interval * 4

        # Skip during active operations (load/unload/calibration)
        if self._operation_active:
            return eventtime + self.poll_interval

        # After an operation completes, _prev_slot_states is stale.
        # Re-sync on the first poll so we don't false-trigger runout
        # or lane clearing from a stale ready→not-ready edge.
        resync_prev = self._prev_states_stale
        if resync_prev:
            self._prev_states_stale = False

        is_printing = False
        try:
            is_printing = self.afc.function.is_printing()
        except Exception:
            pass

        # Single get_status call - refreshes cached hw status AND slot states
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                self._cached_hw_status = hw_status
                # Parse slot states from get_status response
                slots = hw_status.get("slots", [])
                for i, slot_data in enumerate(slots):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        status = slot_data.get("status", "")
                        # Update inventory cache with status from get_status
                        self._slot_inventory[i]["status"] = status
        except Exception:
            pass

        # Full RFID inventory sync only infrequently (material, color data)
        if eventtime - self._last_inventory_sync > self._inventory_sync_interval:
            self._last_inventory_sync = eventtime
            try:
                self._sync_inventory()
            except Exception:
                pass

        # Sync loaded states and detect runout
        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = self._slot_inventory[local_slot]
            slot_status = slot_info.get("status", "") if slot_info else ""
            slot_ready = slot_status == "ready"

            # "shifting" is a transient state during feed_assist start -
            # don't treat it as not-ready or it will false-trigger runout.
            # "feeding" and "unwinding" are motor operations (prep_post_load,
            # eject, calibration) - don't clear load/prep state or update
            # prev_slot_states or the lane will briefly show empty in the UI
            # and then re-trigger set_loaded + prep_post_load when the slot
            # returns to "ready".
            slot_transient = slot_status in ("shifting", "feeding", "unwinding")

            # Always sync load/prep state so status is accurate,
            # but skip during transient motor operations.
            if not slot_transient:
                lane._load_state = slot_ready
                lane.prep_state = slot_ready

            prev_ready = self._prev_slot_states.get(lane.name)
            # Don't update prev state during transient states - wait
            # for a definitive ready/empty to avoid false runout triggers.
            if not slot_transient:
                self._prev_slot_states[lane.name] = slot_ready

            # State consistency: if hardware says ready but lane is stuck
            # in NONE, fix it (covers first poll, missed transitions, etc.)
            if slot_ready and lane._afc_prep_done and lane.status == AFCLaneState.NONE:
                # Check if this lane was tool-loaded before restart -
                # restore TOOLED state instead of just LOADED.
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE poll: {lane.name} slot {local_slot} ready, "
                        f"restoring TOOLED state from saved vars"
                    )
                    self._restore_tool_loaded_state(lane)
                elif lane.name in self._hub_load_suppressed:
                    # Lane was explicitly ejected -- slot is still ready but
                    # we must NOT treat this as new filament.  Restore LOADED
                    # state so the lane isn't stuck in NONE, but keep the
                    # suppression flag so prep_post_load won't re-feed to hub.
                    self.logger.info(
                        f"ACE poll: {lane.name} slot {local_slot} ready "
                        f"but hub-load suppressed (ejected), setting loaded "
                        f"without re-feed"
                    )
                    lane.set_loaded()
                    self.afc.save_vars()
                else:
                    self.logger.info(
                        f"ACE poll: {lane.name} slot {local_slot} ready, "
                        f"setting loaded"
                    )
                    # New filament insertion: clear old data, apply fresh.
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
                    self._refresh_slot_inventory(local_slot)
                    slot_info = self._slot_inventory[local_slot]
                    self._apply_slot_filament_defaults(lane, slot_info)
                    # Auto-create/assign Spoolman spool from RFID data
                    self._sync_rfid_to_spoolman(lane, slot_info)
                    self.lane_illuminate_spool(lane)
                    # New filament clears any previous suppression
                    self._hub_load_suppressed.discard(lane.name)
                    self.afc.save_vars()
                    # Feed filament to hub if load_to_hub is enabled
                    try:
                        self.prep_post_load(lane)
                    except Exception as e:
                        self.logger.error(
                            f"ACE poll: prep_post_load error for "
                            f"{lane.name}: {e}"
                        )

            # Detect ready -> not-ready transition (filament runout)
            # Skip runout detection while dryer is running - drying can
            # cause transient slot state changes that aren't real runouts.
            # Also skip on first poll after an operation ends (stale prev).
            elif (not self._drying_active and not resync_prev
                    and prev_ready and not slot_ready and not slot_transient):
                self._hub_load_suppressed.discard(lane.name)
                if is_printing and lane.status == AFCLaneState.TOOLED:
                    self.logger.info(
                        f"ACE runout detected on {lane.name} (slot {local_slot})"
                    )
                    lane.loaded_to_hub = False
                    self._set_hub_state(lane, False)

                    if lane.runout_lane:
                        try:
                            lane._perform_infinite_runout()
                        except Exception as e:
                            self.logger.error(
                                f"ACE infinite spool failed for {lane.name}: "
                                f"{e}\n{traceback.format_exc()}"
                            )
                            lane._perform_pause_runout()
                        finally:
                            lane.loaded_to_hub = False
                    else:
                        self._ace_pause_runout(lane)
                elif lane.status == AFCLaneState.LOADED:
                    # Slot went empty - transition to unloaded so new
                    # filament insertion is properly detected.
                    self._clear_slot_inventory(local_slot)
                    lane.set_unloaded()
                    self.lane_not_ready(lane)
                    self.afc.save_vars()
                else:
                    # Filament physically removed after eject or while in
                    # NONE/EJECTING state — clear stale inventory so a
                    # fresh spool is properly detected on next insertion.
                    self._clear_slot_inventory(local_slot)
                    self.lane_not_ready(lane)
                    self.afc.save_vars()

        # Polling rates: 2s when printing (runout detection), 5s when idle
        if is_printing:
            return eventtime + max(self.poll_interval, 2.0)
        return eventtime + 5.0

    # ---- Inventory Sync ----

    def sync_ace_inventory(self):
        """Public method to force-sync RFID/spool data from ACE hardware."""
        self._sync_inventory()
        self._sync_slot_loaded_state()

    # ---- GCode Commands ----

    def _register_gcode_commands(self):
        """Register ACE-specific GCode commands."""
        self.gcode.register_mux_command(
            "ACE_STATUS", "UNIT", self.name,
            self.cmd_ACE_STATUS,
            desc="Query ACE hardware status",
        )
        self.gcode.register_mux_command(
            "ACE_DRY", "UNIT", self.name,
            self.cmd_ACE_DRY,
            desc="Start ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "ACE_DRY_STOP", "UNIT", self.name,
            self.cmd_ACE_DRY_STOP,
            desc="Stop ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "ACE_FEED_ASSIST", "UNIT", self.name,
            self.cmd_ACE_FEED_ASSIST,
            desc="Enable/disable feed assist for ACE unit",
        )
        self.gcode.register_mux_command(
            "ACE_SYNC_INVENTORY", "UNIT", self.name,
            self.cmd_ACE_SYNC_INVENTORY,
            desc="Refresh RFID/spool inventory from ACE hardware",
        )
        self.gcode.register_mux_command(
            "ACE_CALIBRATE", "UNIT", self.name,
            self.cmd_ACE_CALIBRATE,
            desc="Calibrate ACE per-lane bowden length by feeding until toolhead sensor triggers",
        )
        self.gcode.register_mux_command(
            "ACE_CALIBRATE_HUB", "UNIT", self.name,
            self.cmd_ACE_CALIBRATE_HUB,
            desc="Calibrate ACE dist_hub by feeding until hub sensor triggers",
        )
        self.gcode.register_mux_command(
            "ACE_HUB_CALIBRATION", "UNIT", self.name,
            self.cmd_ACE_HUB_CALIBRATION,
            desc="Show ACE hub calibration menu",
        )
        self.gcode.register_mux_command(
            "ACE_LANE_RESET", "UNIT", self.name,
            self.cmd_ACE_LANE_RESET,
            desc="Retract ACE lane filament back into the unit",
        )

    def cmd_ACE_STATUS(self, gcmd):
        """Query and display ACE hardware status.

        Usage: ACE_STATUS UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        try:
            status = self._ace.get_status()
            self.logger.info(f"ACE {self.name} status: {status}")
        except Exception as e:
            self.logger.info(f"ACE {self.name} status query failed: {e}")

    def cmd_ACE_DRY(self, gcmd):
        """Start ACE filament dryer.

        Usage: ACE_DRY UNIT=<name> TEMP=<celsius> DURATION=<minutes> [FAN=<rpm>]
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        temp = gcmd.get_int("TEMP", 50)
        duration = gcmd.get_int("DURATION", 240)
        fan = gcmd.get_int("FAN", 800)

        try:
            self._ace.start_drying(temp, fan, duration)
            self._drying_active = True
            self.logger.info(
                f"ACE {self.name}: drying started "
                f"({temp}C, {duration}min, fan={fan}rpm)"
            )
        except Exception as e:
            self.logger.info(f"ACE {self.name}: drying failed: {e}")

    def cmd_ACE_DRY_STOP(self, gcmd):
        """Stop ACE filament dryer.

        Usage: ACE_DRY_STOP UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        try:
            self._ace.stop_drying()
            self._drying_active = False
            self.logger.info(f"ACE {self.name}: drying stopped")
        except Exception as e:
            self._drying_active = False
            self.logger.info(f"ACE {self.name}: stop drying failed: {e}")

    def cmd_ACE_FEED_ASSIST(self, gcmd):
        """Enable or disable feed assist, globally or per-slot.

        Usage:
            ACE_FEED_ASSIST UNIT=<name>                    # query all slots
            ACE_FEED_ASSIST UNIT=<name> ENABLE=<0|1>       # set default for all
            ACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=<0|1>  # set per-slot
            ACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=default # clear per-slot override
        """
        slot_num = gcmd.get_int("SLOT", default=None)
        enable_str = gcmd.get("ENABLE", default=None)

        # Query mode: no ENABLE param
        if enable_str is None:
            lines = [f"ACE {self.name} feed assist:"]
            default_str = "enabled" if self._default_feed_assist else "disabled"
            lines.append(f"  Default: {default_str}")
            for s in range(self.SLOTS_PER_UNIT):
                override = self._slot_feed_assist.get(s)
                effective = self._get_feed_assist_for_slot(s)
                eff_str = "enabled" if effective else "disabled"
                if override is not None:
                    lines.append(f"  Slot {s + 1}: {eff_str} (override)")
                else:
                    lines.append(f"  Slot {s + 1}: {eff_str} (default)")
            self.logger.info("\n".join(lines))
            return

        # Per-slot override
        if slot_num is not None:
            slot_index = slot_num - 1  # Config is 1-based
            if slot_index < 0 or slot_index >= self.SLOTS_PER_UNIT:
                self.logger.info(
                    f"ACE {self.name}: invalid slot {slot_num} (must be 1-{self.SLOTS_PER_UNIT})"
                )
                return

            if enable_str.lower() == "default":
                # Clear per-slot override
                self._slot_feed_assist.pop(slot_index, None)
                effective = self._get_feed_assist_for_slot(slot_index)
                state = "enabled" if effective else "disabled"
                self.logger.info(
                    f"ACE {self.name}: slot {slot_num} feed assist reset to default ({state})"
                )
            else:
                enable = bool(int(enable_str))
                self._slot_feed_assist[slot_index] = enable
                state = "enabled" if enable else "disabled"
                self.logger.info(
                    f"ACE {self.name}: slot {slot_num} feed assist {state}"
                )
            return

        # Global default
        enable = bool(int(enable_str))
        self._default_feed_assist = enable
        state = "enabled" if enable else "disabled"
        self.logger.info(f"ACE {self.name}: feed assist default {state}")

    def cmd_ACE_SYNC_INVENTORY(self, gcmd):
        """Refresh RFID/spool inventory from ACE hardware and sync to lanes.

        Usage: ACE_SYNC_INVENTORY UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        try:
            # Re-enable RFID in case it was disabled
            self._ace.enable_rfid()
            # Pull fresh inventory
            self._sync_inventory()
            self._sync_slot_loaded_state()

            # Report what we found
            lines = [f"ACE {self.name} inventory:"]
            for slot in range(self.SLOTS_PER_UNIT):
                info = self._slot_inventory[slot]
                status = info.get("status", "unknown")
                material = info.get("material", "")
                color = info.get("color", [0, 0, 0])
                hex_color = self._ace_color_to_hex(color)
                lines.append(
                    f"  Slot {slot + 1}: {status} | {material} | {hex_color}"
                )
            self.logger.info("\n".join(lines))
        except Exception as e:
            self.logger.info(
                f"ACE {self.name}: inventory sync failed: {e}"
            )

    def cmd_ACE_LANE_RESET(self, gcmd):
        """Retract ACE lane filament back into the unit.

        Unlike TOOL_UNLOAD this skips homing, z-hop, tip forming and other
        toolhead operations.  It simply tells the ACE hardware to retract
        the slot and updates lane state.

        Usage: ACE_LANE_RESET UNIT=<unit> LANE=<lane>
        """
        lane_name = gcmd.get("LANE", None)
        if lane_name is None or lane_name not in self.afc.lanes:
            self.logger.info(f"ACE_LANE_RESET: invalid lane '{lane_name}'")
            return

        cur_lane = self.afc.lanes[lane_name]
        if cur_lane.unit_obj is not self:
            self.logger.info(
                f"ACE_LANE_RESET: {lane_name} does not belong to {self.name}"
            )
            return

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
            self.logger.info(
                f"ACE_LANE_RESET: {lane_name} has no valid slot mapping"
            )
            return

        if self._ace is None or not self._ace.connected:
            self.logger.info(
                f"ACE_LANE_RESET: ACE {self.name} not connected"
            )
            return

        self.logger.info(f"ACE lane reset: retracting {lane_name} slot {local_slot}")

        # Stop feed assist if running
        try:
            self._wait_for_ace_ready()
            self._ace.stop_feed_assist(local_slot)
            self._feed_assist_active.discard(local_slot)
        except Exception as e:
            self.logger.warning(
                f"ACE lane reset: failed to stop feed assist: {e}"
            )

        # Retract filament back into the ACE
        try:
            self._retract_slot(local_slot, lane=cur_lane)
        except Exception as e:
            self.logger.error(f"ACE lane reset failed for {lane_name}: {e}")
            self.logger.info(f"ACE_LANE_RESET: retract failed for {lane_name}: {e}")
            return

        # Update lane state
        if self.mode == MODE_COMBINED:
            self._current_loaded_slot = -1
        cur_lane.loaded_to_hub = False
        self._set_hub_state(cur_lane, False)
        self._hub_load_suppressed.add(lane_name)
        cur_lane.set_tool_unloaded()
        cur_lane.status = AFCLaneState.LOADED
        self.lane_tool_unloaded(cur_lane)

        # Clear extruder's lane_loaded if this lane was loaded
        cur_extruder = cur_lane.extruder_obj
        if cur_extruder is not None and cur_extruder.lane_loaded == lane_name:
            cur_extruder.lane_loaded = None

        self.afc.save_vars()
        self.logger.info(f"ACE lane reset: {lane_name} retracted successfully")

    def cmd_ACE_CALIBRATE(self, gcmd):
        """Calibrate per-lane bowden length by feeding until toolhead sensor triggers.

        Feeds filament from the specified lane's ACE slot in small increments,
        checking the toolhead sensor between each step. Saves the measured
        feed_length and retract_length to the lane's config section.

        Usage: ACE_CALIBRATE UNIT=<name> LANE=<lane_name>
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        lane_name = gcmd.get("LANE", default=None)
        if lane_name is None:
            self.logger.info("ACE_CALIBRATE requires LANE=<lane_name>")
            return

        cur_lane = self.lanes.get(lane_name)
        if cur_lane is None:
            # Try looking up from AFC global lanes
            cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            self.logger.info(f"Lane '{lane_name}' not found")
            return

        self.logger.info(
            f"ACE {self.name}: starting lane calibration for {lane_name}...\n"
            f"Feeding in {self.calibration_step}mm steps"
        )

        success, msg, distance = self.calibrate_lane(cur_lane, 0)
        self.logger.info(msg)

    def cmd_ACE_CALIBRATE_HUB(self, gcmd):
        """Calibrate dist_hub by feeding until hub sensor triggers.

        Requires a physical hub sensor. Feeds filament in small increments,
        checking the hub sensor after each step. Saves the measured
        dist_hub to the lane's config section.

        Usage: ACE_CALIBRATE_HUB UNIT=<name> LANE=<lane_name>
        """
        if self._ace is None or not self._ace.connected:
            self.logger.info(f"ACE {self.name}: not connected")
            return

        lane_name = gcmd.get("LANE", default=None)
        if lane_name is None:
            self.logger.info("ACE_CALIBRATE_HUB requires LANE=<lane_name>")
            return

        cur_lane = self.lanes.get(lane_name)
        if cur_lane is None:
            cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            self.logger.info(f"Lane '{lane_name}' not found")
            return

        self.logger.info(
            f"ACE {self.name}: starting hub calibration for {lane_name}...\n"
            f"Feeding in {self.calibration_step}mm steps"
        )

        success, msg, distance = self.calibrate_hub(cur_lane, 0)
        self.logger.info(msg)

    # ---- Utilities ----

    @staticmethod
    def _ace_color_to_hex(color_array):
        """Convert ACE RGB array [r, g, b] to hex color string."""
        if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
            r, g, b = int(color_array[0]), int(color_array[1]), int(color_array[2])
            return f"#{r:02x}{g:02x}{b:02x}"
        return "#000000"


def load_config_prefix(config):
    return afcACE(config)