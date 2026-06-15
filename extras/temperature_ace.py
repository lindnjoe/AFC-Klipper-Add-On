# Optional ACE temperature sensor for Klipper.
# Reads the ACE device temperature from an AFC_ACE unit's cached
# hardware status and exposes it as a standard temperature_sensor.
#
# Adapted from Kobra-S1/ACEPRO temperature_ace.py for the AFC_ACE
# driver used in this project.
#
# Configuration example (place AFTER your [AFC_ACE ...] section):
#
#   [temperature_sensor ace_temp]
#   sensor_type: temperature_ace
#   ace_unit: Ace1           # Name of your AFC_ACE unit (default: Ace1)
#   min_temp: 0
#   max_temp: 70

import logging

ACE_REPORT_TIME = 1.0  # seconds between samples
_REGISTERED = False

# Fallback logger for early init before AFC logger is available
_fallback_logger = logging.getLogger("temperature_ace")


class TemperatureACE:
    """Temperature sensor that reads ACE device temperature from an
    AFC_ACE unit's cached hardware status."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.ace_unit_name = config.get("ace_unit", "Ace1")

        # Temperature state
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 70.0
        self.measured_min = float("inf")
        self.measured_max = 0.0

        # AFC_ACE reference resolved on ready
        self._ace_unit = None
        # AFC logger reference (resolved in handle_ready)
        self._logger = None
        self._sample_error_logged = False

        # Klipper temperature callback
        self._callback = None

        # Register object
        self.printer.add_object("temperature_ace " + self.name, self)

        # Skip timers in debug mode
        if self.printer.get_start_args().get("debugoutput") is not None:
            return

        self.sample_timer = self.reactor.register_timer(
            self._sample_ace_temperature
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def _log(self):
        """Return AFC logger if available, otherwise fallback."""
        if self._logger is not None:
            return self._logger
        return _fallback_logger

    def handle_ready(self):
        """Resolve AFC_ACE unit reference once Klipper is ready."""
        self._ace_unit = self._resolve_unit()

        # Try to grab AFC's logger for consistent log output
        try:
            afc = self.printer.lookup_object("AFC")
            self._logger = afc.logger
        except Exception:
            pass

        if self._ace_unit:
            self._log().info(
                f"temperature_ace: linked to AFC_ACE unit '{self.ace_unit_name}'"
            )
        else:
            self._log().warning(
                f"temperature_ace: AFC_ACE unit '{self.ace_unit_name}' not found; "
                "reporting 0C"
            )

        if hasattr(self, "sample_timer"):
            self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return ACE_REPORT_TIME

    def _resolve_unit(self):
        """Look up the AFC_ACE unit object via AFC's units dict."""
        try:
            afc = self.printer.lookup_object("AFC")
            units = getattr(afc, "units", {})
            unit = units.get(self.ace_unit_name)
            if unit is not None:
                return unit
        except Exception:
            pass

        # Fallback: try direct printer object lookup
        try:
            return self.printer.lookup_object(
                f"AFC_ACE {self.ace_unit_name}", None
            )
        except Exception:
            return None

    def _sample_ace_temperature(self, eventtime):
        """Timer callback to read ACE temperature and feed the heaters system."""
        try:
            if not self._ace_unit:
                self._ace_unit = self._resolve_unit()

            if self._ace_unit:
                hw_status = getattr(self._ace_unit, "_cached_hw_status", {})
                ace_temp = float(hw_status.get("temp", 0.0) or 0.0)

                self.temp = ace_temp

                if self.temp > 0:
                    self.measured_min = min(self.measured_min, self.temp)
                    self.measured_max = max(self.measured_max, self.temp)

                if self.temp > 0 and self.temp < self.min_temp:
                    self.printer.invoke_shutdown(
                        "ACE temperature %.1f below minimum of %.1f"
                        % (self.temp, self.min_temp)
                    )
                if self.temp > self.max_temp:
                    self.printer.invoke_shutdown(
                        "ACE temperature %.1f above maximum of %.1f"
                        % (self.temp, self.max_temp)
                    )
            else:
                self.temp = 0.0
        except Exception as e:
            if not self._sample_error_logged:
                self._log().error(f"temperature_ace: error sampling ACE temperature: {e}")
                self._sample_error_logged = True
            self.temp = 0.0

        if self._callback:
            mcu = self.printer.lookup_object("mcu")
            measured_time = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(measured_time), self.temp)

        return eventtime + ACE_REPORT_TIME

    def get_temp(self, eventtime):
        return self.temp, 0.0

    def stats(self, eventtime):
        return False, "temperature_ace %s: temp=%.1f" % (self.name, self.temp)

    def get_status(self, eventtime):
        return {
            "temperature": round(self.temp, 2),
            "measured_min_temp": round(self.measured_min, 2)
            if self.measured_min != float("inf")
            else 0.0,
            "measured_max_temp": round(self.measured_max, 2),
            "ace_unit": self.ace_unit_name,
        }


def load_config(config):
    """Register temperature_ace sensor factory with Klipper (config hook)."""
    _register_sensor_factory(config.get_printer())


def _register_sensor_factory(printer):
    """Idempotently register the temperature_ace sensor factory."""
    global _REGISTERED
    if _REGISTERED:
        return
    try:
        heaters = printer.lookup_object("heaters")
    except Exception:
        try:
            heaters = printer.load_object(printer, "heaters")
        except Exception as e:
            _fallback_logger.warning(f"temperature_ace: failed to load heaters: {e}")
            return

    heaters.add_sensor_factory("temperature_ace", TemperatureACE)
    _REGISTERED = True