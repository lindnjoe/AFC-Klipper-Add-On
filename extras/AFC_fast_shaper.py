# Fast input shaper calibration using narrow frequency band around current settings
class AFCFastShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.delta_freq = config.getfloat('delta_freq', 10.0, minval=5.0)
        self.min_freq = config.getfloat('min_freq', 5.0)
        self.max_freq = config.getfloat('max_freq', 200.0)
        self.gcode.register_command(
            'FAST_SHAPER_CALIBRATE', self.cmd_FAST_SHAPER_CALIBRATE,
            desc="Run narrow-band shaper calibration around current frequencies")

    def _get_shaper_params_from_tool(self):
        """Read shaper params from the active AFC_extruder's tool params."""
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            return None
        ext_name = toolhead.get_extruder().get_name()
        afc_ext = self.printer.lookup_object('AFC_extruder %s' % ext_name, None)
        if afc_ext is None:
            return None
        params = getattr(afc_ext, 'params', {})
        freq_x = params.get('params_input_shaper_freq_x')
        freq_y = params.get('params_input_shaper_freq_y')
        type_x = params.get('params_input_shaper_type_x', 'mzv')
        type_y = params.get('params_input_shaper_type_y', 'mzv')
        if freq_x is None or freq_y is None:
            return None
        if isinstance(type_x, str):
            type_x = type_x.strip("'\"")
        if isinstance(type_y, str):
            type_y = type_y.strip("'\"")
        return {
            'x': (float(freq_x), str(type_x)),
            'y': (float(freq_y), str(type_y)),
        }

    def cmd_FAST_SHAPER_CALIBRATE(self, gcmd):
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is None:
            raise gcmd.error(
                "[input_shaper] not found. "
                "Add [input_shaper] to your config first.")

        delta = gcmd.get_float('DELTA_FREQ', self.delta_freq, minval=5.0)

        tool_params = self._get_shaper_params_from_tool()

        for axis_idx, axis_name in enumerate(['x', 'y']):
            shaper = input_shaper.shapers[axis_idx]
            current_freq = shaper.params.shaper_freq
            shaper_type = shaper.params.shaper_type

            if current_freq == 0 and tool_params is not None:
                current_freq, shaper_type = tool_params[axis_name]

            if current_freq == 0:
                raise gcmd.error(
                    "%s axis shaper_freq is 0. "
                    "Run full SHAPER_CALIBRATE first." % axis_name.upper())

            freq_start = current_freq - abs(delta)
            if freq_start < self.min_freq:
                freq_start = self.min_freq
            freq_end = current_freq + abs(delta)
            if freq_end > self.max_freq:
                freq_end = self.max_freq

            gcmd.respond_info(
                "FAST_SHAPER_CALIBRATE: axis=%s type=%s freq_range=%.1f-%.1f"
                % (axis_name.upper(), shaper_type, freq_start, freq_end))

            command = ("SHAPER_CALIBRATE AXIS=%s SHAPER_TYPES=%s "
                       "FREQ_START=%d FREQ_END=%d"
                       % (axis_name, shaper_type, freq_start, freq_end))
            self.gcode.run_script_from_command(command)

def load_config(config):
    return AFCFastShaper(config)
