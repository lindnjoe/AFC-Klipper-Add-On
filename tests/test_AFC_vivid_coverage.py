"""
Branch-coverage unit tests for extras/AFC_vivid.py.

These complement tests/test_AFC_vivid.py by covering paths that file leaves
uncovered: real ``__init__`` construction (and ``load_config_prefix``), the
``tool_loaded`` short-circuit in ``_move_lane``, the selector-calibration-move
skip in ``select_lane``, and the RFID stage-read event error handlers in
``_stage_and_load``.

Construction goes through the real AFC_vivid.__init__ with only the afcBoxTurtle
(afcUnit) base constructor stubbed, so the AFC_vivid-specific body runs for real.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from extras.AFC_vivid import AFC_vivid, load_config_prefix
from extras.AFC_BoxTurtle import afcBoxTurtle
from extras.AFC_unit import afcUnit
from tests.conftest import MockAFC, MockConfig, MockPrinter


# ── Helpers ───────────────────────────────────────────────────────────────────

def _install_fake_super(self, cfg):
    """Minimal stand-in for the afcUnit/afcBoxTurtle base __init__.

    Sets only the attributes AFC_vivid.__init__ relies on after its
    ``super().__init__`` call, so the AFC_vivid-specific body runs against
    mocked Klipper dependencies. ``_lookup_objects`` is stubbed separately, so
    the drive/selector stepper objects are supplied here.
    """
    printer = cfg.get_printer()
    self.printer = printer
    self.afc = printer.lookup_object("AFC")
    self.function = self.afc.function
    self.logger = self.afc.logger
    self.reactor = printer.get_reactor()
    self.full_name = cfg.get_name().split()
    self.name = self.full_name[-1]
    self.drive_stepper_obj = MagicMock()
    self.selector_stepper_obj = MagicMock()


def _make_vivid_real(name="ViViD_1", values=None):
    """Construct an AFC_vivid through its real __init__ (base stubbed).

    The afcUnit/afcBoxTurtle base constructor and ``_lookup_objects`` (both in
    other modules) are stubbed so only AFC_vivid's own __init__ body runs.
    """
    afc = MockAFC()
    printer = MockPrinter(afc=afc)
    vals = {"drive_stepper": "drive", "selector_stepper": "selector"}
    if values:
        vals.update(values)
    config = MockConfig(name=f"AFC_vivid {name}", printer=printer, values=vals)
    with patch.object(afcBoxTurtle, "__init__", _install_fake_super), \
            patch.object(afcUnit, "_lookup_objects", lambda self, cfg: None):
        unit = AFC_vivid(config)
    return unit


# ── __init__ ──────────────────────────────────────────────────────────────────

class TestInit:
    def test_sets_vivid_specific_attributes(self):
        unit = _make_vivid_real()
        assert unit.type == "ViViD"
        assert unit.drive_stepper == "drive"
        assert unit.selector_stepper == "selector"
        assert unit.current_selected_lane is None
        assert unit.home_state is False
        assert unit.prep_homed is False
        assert unit.failed_to_home is False
        assert unit.selector_homing_speed == 150
        assert unit.selector_homing_accel == 150
        assert unit.max_selector_movement == 800
        assert unit._eject_to_calibrate is True

    def test_enable_sensors_in_gui_defaults_to_afc_value(self):
        unit = _make_vivid_real()
        # MockAFC.enable_sensors_in_gui is False and no override is provided.
        assert unit.enable_sensors_in_gui is False

    def test_type_override_from_config(self):
        unit = _make_vivid_real(values={"type": "CustomViViD"})
        assert unit.type == "CustomViViD"

    def test_custom_homing_parameters(self):
        unit = _make_vivid_real(values={
            "selector_homing_speed": 200,
            "selector_homing_accel": 250,
            "max_selector_movement": 900,
        })
        assert unit.selector_homing_speed == 200
        assert unit.selector_homing_accel == 250
        assert unit.max_selector_movement == 900

    def test_lookup_objects_populated_and_command_registered(self):
        unit = _make_vivid_real()
        assert unit.drive_stepper_obj is not None
        assert unit.selector_stepper_obj is not None
        unit.function.register_mux_command.assert_called_once()


# ── load_config_prefix ────────────────────────────────────────────────────────

class TestLoadConfigPrefix:
    def test_returns_afc_vivid_instance(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        config = MockConfig(
            name="AFC_vivid my_vivid", printer=printer,
            values={"drive_stepper": "drive", "selector_stepper": "selector"})

        with patch.object(afcBoxTurtle, "__init__", _install_fake_super), \
                patch.object(afcUnit, "_lookup_objects", lambda self, cfg: None):
            instance = load_config_prefix(config)

        assert isinstance(instance, AFC_vivid)
        assert instance.type == "ViViD"


# ── _move_lane (tool_loaded short-circuit) ────────────────────────────────────

class TestMoveLaneToolLoaded:
    def test_tool_loaded_skips_retract_and_reset(self):
        # prep_state True and homing succeeds, but tool_loaded True -> the
        # hub-clear retract (lane.move_to) must be skipped, and because homed is
        # True with loaded_to_hub set the reset block is skipped too.
        unit = _make_vivid_real()
        unit.move_to_load = MagicMock(return_value=(True, 0.0, None))
        unit.lane_unloaded = MagicMock()
        lane = MagicMock()
        lane.prep_state = True
        lane.tool_loaded = True
        lane.loaded_to_hub = False
        lane.hub_obj.hub_clear_move_dis = 65

        result = unit._move_lane(lane, 1, True)

        assert result is True
        assert lane.loaded_to_hub is True
        lane.move_to.assert_not_called()
        unit.lane_unloaded.assert_not_called()
        unit.move_to_load.assert_called_once()


# ── select_lane (selector calibration move) ───────────────────────────────────

class TestSelectLaneCalibrationMove:
    def _lane(self, cal_dis):
        lane = MagicMock()
        lane.name = "lane1"
        lane.selector_endstop_name = "lane1_selector"
        lane.selector_cal_dis = cal_dis
        lane.short_moves_speed = 50
        lane.short_moves_accel = 400
        lane.fila_selector.get_status.return_value = {"filament_detected": False}
        return lane

    def _prep_unit(self):
        unit = _make_vivid_real()
        unit.printer._objects = {}  # no stepper_enable -> selector disabled
        unit.selector_stepper_obj.do_homing_move.return_value = (True, 15.0)
        return unit

    def test_calibration_move_runs_when_cal_dis_nonzero(self):
        unit = self._prep_unit()
        lane = self._lane(5.0)

        homed, dist = unit.select_lane(lane)

        assert (homed, dist) == (True, 15.0)
        unit.selector_stepper_obj.move.assert_called_once_with(
            5.0, lane.short_moves_speed, lane.short_moves_accel, False)
        assert unit.logger.messages == [
            ("debug", "ViViD: Selecting lane1"),
            ("debug", "ViViD: Homing done, success:True, distance:15.0")]

    def test_calibration_move_skipped_when_cal_dis_none(self):
        # First sub-condition (cal_dis is not None) is False -> skip the move.
        unit = self._prep_unit()
        lane = self._lane(None)

        homed, dist = unit.select_lane(lane)

        assert (homed, dist) == (True, 15.0)
        unit.selector_stepper_obj.move.assert_not_called()

    def test_calibration_move_skipped_when_cal_dis_zero(self):
        # Second sub-condition (cal_dis != 0.0) is False -> skip the move.
        unit = self._prep_unit()
        lane = self._lane(0.0)

        homed, dist = unit.select_lane(lane)

        assert (homed, dist) == (True, 15.0)
        unit.selector_stepper_obj.move.assert_not_called()


# ── _stage_and_load (RFID event error handlers) ───────────────────────────────

class TestStageAndLoadEventErrors:
    def _lane(self):
        lane = MagicMock()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.prep_state = True
        lane.raw_load_state = True  # already at sensor -> feed loop skipped
        return lane

    def test_stage_read_begin_error_is_logged(self):
        unit = _make_vivid_real()
        lane = self._lane()

        def send_event(name, arg=None):
            if name == "afc_vivid:stage_read_begin":
                raise RuntimeError("boom")

        unit.printer.send_event = send_event

        homed, total = unit._stage_and_load(lane)

        assert homed is True
        assert total == 0.0
        assert unit.logger.messages == [
            ("error", "ViViD stage read begin error: boom")]

    def test_stage_read_end_error_is_logged(self):
        unit = _make_vivid_real()
        lane = self._lane()

        def send_event(name, arg=None):
            if name == "afc_vivid:stage_read_end":
                raise RuntimeError("bang")

        unit.printer.send_event = send_event

        homed, total = unit._stage_and_load(lane)

        assert homed is True
        assert total == 0.0
        assert unit.logger.messages == [
            ("error", "ViViD stage read end error: bang")]
