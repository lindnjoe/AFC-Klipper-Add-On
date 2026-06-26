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
        """Initialize the ACE temperature sensor.

        :param config: Klipper config wrapper for this ``[temperature_sensor]``
            section. Reads ``ace_unit`` — the AFC_ACE unit name whose cached
            hardware status supplies the temperature (default ``Ace1``).
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.ace_unit_name = config.get("ace_unit", "Ace1")
        # Register under a Mainsail-recognized sensor name so the web UI shows
        # temperature AND humidity. Mainsail reads humidity only from sensors it
        # knows (e.g. aht3x); a plain temperature_ace object surfaces temp only.
        # Mirrors temperature_oams' simulate_supported_sensor_mainsail option.
        self.simulate_aht3x = config.getboolean(
            "simulate_supported_sensor_mainsail", True)

        # Temperature state
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 70.0
        self.measured_min = float("inf")
        self.measured_max = 0.0
        # Humidity (%RH) from the dryer sensor — only ACE 2 reports it (V1 ACE
        # omits the key). Surfaced in get_status when present so the UI shows it
        # alongside temp, like the OpenAMS temperature sensor.
        self.humidity = 0.0
        self._has_humidity = False

        # AFC_ACE reference resolved on ready
        self._ace_unit = None
        # AFC logger reference (resolved in handle_ready)
        self._logger = None
        self._sample_error_logged = False

        # Klipper temperature callback
        self._callback = None

        # Register object. Registering as "aht3x <name>" makes Mainsail treat
        # this as a humidity-capable sensor and display the humidity field
        # (ACE 2); falls back to the plain name when simulation is disabled.
        if self.simulate_aht3x:
            self.printer.add_object("aht3x " + self.name, self)
        else:
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
            # Decide humidity support now (ACE 2 reports it, V1 ACE doesn't) so
            # the field is present in get_status from the first query — Mainsail
            # registers a sensor's humidity field early, before the first sample
            # would otherwise set it.
            if getattr(self._ace_unit, "type", "") == "ACE2":
                self._has_humidity = True
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
        """Set the allowed temperature range (Klipper sensor interface).

        :param min_temp: minimum allowed temperature in C; a reading below it
            (when temp > 0) triggers a printer shutdown.
        :param max_temp: maximum allowed temperature in C; a reading above it
            triggers a printer shutdown.
        """
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        """Register the heaters callback that receives temperature samples.

        :param cb: callable invoked as ``cb(measured_time, temp)`` each sample.
        """
        self._callback = cb

    def get_report_time_delta(self):
        """Return the sampling interval (Klipper sensor interface).

        :return float: seconds between temperature reports (ACE_REPORT_TIME).
        """
        return ACE_REPORT_TIME

    def _resolve_unit(self):
        """Look up the AFC_ACE unit object via AFC's units dict.

        :return: the AFC_ACE unit object for ``ace_unit_name``, or None if AFC
            or the unit is not (yet) available.
        """
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
        """Timer callback: read ACE temperature and feed the heaters system.

        Reads ``temp`` from the AFC_ACE unit's cached hardware status, tracks
        measured min/max, enforces the min/max limits (shutdown on breach), and
        invokes the registered heaters callback with the sample.

        :param eventtime: reactor event time of this firing.
        :return float: the next reactor time to fire (eventtime + report time).
        """
        try:
            if not self._ace_unit:
                self._ace_unit = self._resolve_unit()

            if self._ace_unit:
                hw_status = getattr(self._ace_unit, "_cached_hw_status", {})
                ace_temp = float(hw_status.get("temp", 0.0) or 0.0)

                self.temp = ace_temp
                # Only ACE 2 reports humidity; V1 ACE omits the key entirely.
                if "humidity" in hw_status:
                    self._has_humidity = True
                    self.humidity = float(hw_status.get("humidity", 0.0) or 0.0)

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
        """Return the current temperature (Klipper sensor interface).

        :param eventtime: reactor event time (unused).
        :return tuple: (current temperature in C, measured error 0.0).
        """
        return self.temp, 0.0

    def stats(self, eventtime):
        """Return a stats line for the sensor (Klipper sensor interface).

        :param eventtime: reactor event time (unused).
        :return tuple: (False, status string carrying the current temperature).
        """
        return False, "temperature_ace %s: temp=%.1f" % (self.name, self.temp)

    def get_status(self, eventtime):
        """Return sensor status for Moonraker / status queries.

        :param eventtime: reactor event time (unused).
        :return dict: ``temperature`` plus ``measured_min_temp`` /
            ``measured_max_temp``, the source ``ace_unit`` name, and
            ``humidity`` when the unit reports it (ACE 2 only).
        """
        # Mirror the OpenAMS sensor's status shape so Mainsail's card shows just
        # temperature (plus humidity for ACE 2). measured_min/max were extra
        # numeric fields Mainsail rendered as unlabeled values that cluttered the
        # card. measured_min/max stay tracked internally for the shutdown limits.
        status = {"temperature": round(self.temp, 2)}
        # Only present for ACE 2 (V1 ACE has no humidity sensor), matching how
        # the OpenAMS temperature sensor surfaces humidity.
        if self._has_humidity:
            status["humidity"] = round(self.humidity, 2)
        return status


def load_config(config):
    """Register the temperature_ace sensor factory with Klipper (config hook).

    :param config: Klipper config wrapper (used only to reach the printer).
    """
    _register_sensor_factory(config.get_printer())


def _register_sensor_factory(printer):
    """Idempotently register the temperature_ace sensor factory.

    :param printer: the Klipper printer object whose ``heaters`` object the
        ``temperature_ace`` sensor factory is added to.
    """
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