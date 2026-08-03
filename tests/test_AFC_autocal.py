"""
Unit tests for extras/AFC_autocal.py — per-spool flow calibration (K).

Instances are built through the real __init__ with local fakes for the Klipper
config/printer/reactor/gcode plumbing (no __new__ bypass). One test class per
method, per the project test rules.
"""

from __future__ import annotations

import threading
import types

import extras.AFC_autocal as autocal_mod
from extras.AFC_autocal import AFC_autocal, load_config


# ── fakes ─────────────────────────────────────────────────────────────────────

class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _Reactor:
    def __init__(self):
        self.now = 1000.0
        self.callbacks = []          # (cb, waketime)
        self.async_callbacks = []

    def monotonic(self):
        return self.now

    def register_callback(self, cb, waketime=None):
        self.callbacks.append((cb, waketime))

    def register_async_callback(self, cb):
        self.async_callbacks.append(cb)

    def run_pending(self):
        """Invoke every queued callback once (draining the queues)."""
        cbs = [cb for cb, _ in self.callbacks] + self.async_callbacks
        self.callbacks = []
        self.async_callbacks = []
        for cb in cbs:
            cb(self.now)


class _Gcode:
    def __init__(self):
        self.commands = {}
        self.scripts = []            # run_script calls
        self.script_cmds = []        # run_script_from_command calls
        self.mux_commands = {}       # name -> (key, {value: handler})

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def register_mux_command(self, name, key, value, func, desc=None):
        entry = self.mux_commands.setdefault(name, (key, {}))
        entry[1][value] = func

    def run_script(self, script):
        self.scripts.append(script)

    def run_script_from_command(self, script):
        self.script_cmds.append(script)


class _Printer:
    def __init__(self):
        self.reactor = _Reactor()
        self.gcode = _Gcode()
        self.objects = {"gcode": self.gcode}
        self.events = []             # (event, handler)
        self.sent_events = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default="__raise__"):
        if name in self.objects:
            return self.objects[name]
        if default == "__raise__":
            raise Exception(f"unknown object {name}")
        return default

    def register_event_handler(self, event, cb):
        self.events.append((event, cb))

    def send_event(self, event, *args):
        self.sent_events.append((event, args))


class _Config:
    def __init__(self, printer, values=None):
        self._printer = printer
        self.values = dict(values or {})

    def get_printer(self):
        return self._printer

    def getboolean(self, key, default=False):
        return self.values.get(key, default)

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getfloat(self, key, default=None, **kw):
        return self.values.get(key, default)


class _Estepper:
    """extruder_stepper fake for consumer-mode stepper apply."""

    def __init__(self, smooth_time=0.04):
        self.pressure_advance_smooth_time = smooth_time
        self.applied = []            # (k, smooth_time)

    def _set_pressure_advance(self, k, smooth_time):
        self.applied.append((k, smooth_time))


class _PrinterExtruder:
    """Klipper printer 'extruder' object fake (holds an extruder_stepper)."""

    def __init__(self, estepper=None):
        self.extruder_stepper = estepper if estepper is not None else _Estepper()


class _Extruder:
    """Active-toolhead extruder fake (Klipper extruder object)."""

    def __init__(self, name="extruder"):
        self._name = name

    def get_name(self):
        return self._name


class _Toolhead:
    def __init__(self, extruder_name="extruder"):
        self.extruder = _Extruder(extruder_name)

    def get_extruder(self):
        return self.extruder


class _FlowCal:
    def __init__(self):
        self._current_k = {}
        self.applied = []            # (extruder_obj, k)

    def _set_pressure_advance(self, ext, k):
        self.applied.append((ext, k))


class _Gcmd:
    def __init__(self):
        self.lines = []

    def respond_info(self, msg):
        self.lines.append(msg)


class _ImmediateThread:
    """threading.Thread stand-in that runs its target synchronously."""

    def __init__(self, target=None, name=None, daemon=None):
        self._target = target

    def start(self):
        self._target()


def _make_lane(name="lane1", spool_id=5, tool_loaded=False, load_state=True,
               ext_name="extruder", th_name=None, load_active=False):
    ext = types.SimpleNamespace(name=ext_name,
                                th_extruder_name=th_name or ext_name,
                                load_active=load_active)
    return types.SimpleNamespace(name=name, spool_id=spool_id,
                                 tool_loaded=tool_loaded,
                                 load_state=load_state, extruder_obj=ext)


def _make_afc(lanes=None, prep_done=True, state="State.IDLE", printing=False,
              current=None):
    func = types.SimpleNamespace(
        is_printing=lambda: printing,
        get_current_lane_obj=lambda: current)
    return types.SimpleNamespace(
        lanes=lanes or {}, prep_done=prep_done, current_state=state,
        function=func, logger=_Logger(), moonraker=None, spoolman=None)


def _make(values=None, printer=None, calibrator=False):
    printer = printer or _Printer()
    if calibrator:
        printer.objects["flow_calibrator"] = _FlowCal()
    cal = AFC_autocal(_Config(printer, values))
    cal.logger = _Logger()
    return cal, printer


def _make_ready(values=None, afc=None, grace=0.0, calibrator=True):
    """Build + run _handle_ready with an AFC present (patches skipped by
    default toggles unless enabled in values). Injects a flow_calibrator by
    default (U1 mode); pass calibrator=False for consumer-mode tests."""
    printer = _Printer()
    afc = afc or _make_afc()
    printer.objects["AFC"] = afc
    if calibrator:
        printer.objects["flow_calibrator"] = _FlowCal()
    vals = {"startup_cal_grace": grace}
    vals.update(values or {})
    cal = AFC_autocal(_Config(printer, vals))
    cal.afc = afc
    cal._ready_time = printer.reactor.monotonic() - (grace + 1.0)
    cal.logger = _Logger()
    return cal, printer, afc


# ── __init__ ──────────────────────────────────────────────────────────────────

class TestInit:
    def test_toggles_default_off(self):
        cal, _ = _make()
        assert cal.apply_stored_k is False
        assert cal.auto_calibrate is False

    def test_enabled_master_turns_both_on(self):
        cal, _ = _make({"enabled": True})
        assert cal.apply_stored_k is True
        assert cal.auto_calibrate is True

    def test_explicit_toggles_override_master(self):
        cal, _ = _make({"enabled": True, "auto_calibrate": False})
        assert cal.apply_stored_k is True
        assert cal.auto_calibrate is False

    def test_calibrate_gcode_default_and_override(self):
        cal, _ = _make()
        assert cal.calibrate_gcode == "FLOW_CALIBRATE"
        cal2, _ = _make({"calibrate_gcode": "MY_CAL"})
        assert cal2.calibrate_gcode == "MY_CAL"

    def test_commands_and_events_registered(self):
        cal, printer = _make()
        assert "AFC_APPLY_LANE_FLOW_K" in printer.gcode.commands
        assert "AFC_CALIBRATE_LANE_FLOW_K" in printer.gcode.commands
        assert [e for e, _ in printer.events] == [
            "klippy:ready", "afc:tool_loaded", "afc:spool_assigned",
            "homing:home_rails_end", "extruder:activate_extruder"]


# ── _handle_ready ─────────────────────────────────────────────────────────────

class TestHandleReady:
    def test_afc_missing_disables(self):
        cal, printer = _make()
        log = cal.logger
        cal._handle_ready()
        assert cal.afc is None
        assert log.messages == [
            ("warning", "AFC_autocal: AFC not loaded; disabled")]

    def test_sets_ready_time_and_adopts_afc_logger(self):
        cal, printer = _make()
        afc = _make_afc()
        printer.objects["AFC"] = afc
        cal._handle_ready()
        assert cal.afc is afc
        assert cal.logger is afc.logger
        assert cal._ready_time == printer.reactor.monotonic()

    def test_patches_gated_on_toggles(self, monkeypatch):
        calls = []
        cal, printer = _make({"apply_stored_k": True, "auto_calibrate": True},
                             calibrator=True)
        printer.objects["AFC"] = _make_afc()
        monkeypatch.setattr(cal, "_patch_set_tool_loaded_emit",
                            lambda: calls.append("tool"))
        monkeypatch.setattr(cal, "_patch_set_spoolid_emit",
                            lambda: calls.append("spool"))
        cal._handle_ready()
        assert calls == ["tool", "spool"]

    def test_no_patches_when_both_toggles_off(self, monkeypatch):
        calls = []
        cal, printer = _make()
        printer.objects["AFC"] = _make_afc()
        monkeypatch.setattr(cal, "_patch_set_tool_loaded_emit",
                            lambda: calls.append("tool"))
        monkeypatch.setattr(cal, "_patch_set_spoolid_emit",
                            lambda: calls.append("spool"))
        cal._handle_ready()
        assert calls == []


# ── _patch_set_tool_loaded_emit ───────────────────────────────────────────────

class TestPatchSetToolLoadedEmit:
    def test_patches_and_emits(self, monkeypatch):
        from extras.AFC_lane import AFCLane
        orig_calls = []
        monkeypatch.setattr(AFCLane, "set_tool_loaded",
                            lambda self, normal_toolchange=False:
                            orig_calls.append(normal_toolchange),
                            raising=False)
        monkeypatch.setattr(AFCLane, "_afc_autocal_emit_patched", False,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_tool_loaded_emit()
        assert AFCLane._afc_autocal_emit_patched is True
        fake_printer = _Printer()
        fake_lane = types.SimpleNamespace(printer=fake_printer)
        AFCLane.set_tool_loaded(fake_lane, normal_toolchange=True)
        assert orig_calls == [True]                    # original still runs
        assert fake_printer.sent_events == [("afc:tool_loaded", (fake_lane,))]

    def test_idempotent(self, monkeypatch):
        from extras.AFC_lane import AFCLane
        sentinel = lambda self, normal_toolchange=False: None
        monkeypatch.setattr(AFCLane, "set_tool_loaded", sentinel, raising=False)
        monkeypatch.setattr(AFCLane, "_afc_autocal_emit_patched", True,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_tool_loaded_emit()
        assert AFCLane.set_tool_loaded is sentinel     # untouched


# ── _patch_set_spoolid_emit ───────────────────────────────────────────────────

class TestPatchSetSpoolidEmit:
    def test_patches_and_emits(self, monkeypatch):
        from extras.AFC_spool import AFCSpool
        orig_calls = []
        monkeypatch.setattr(AFCSpool, "set_spoolID",
                            lambda self, cur_lane, SpoolID, save_vars=True:
                            orig_calls.append((SpoolID, save_vars)),
                            raising=False)
        monkeypatch.setattr(AFCSpool, "_afc_autocal_spoolid_patched", False,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_spoolid_emit()
        assert AFCSpool._afc_autocal_spoolid_patched is True
        fake_printer = _Printer()
        fake_spool = types.SimpleNamespace(printer=fake_printer)
        lane = _make_lane()
        AFCSpool.set_spoolID(fake_spool, lane, 7, save_vars=False)
        assert orig_calls == [(7, False)]
        assert fake_printer.sent_events == [("afc:spool_assigned", (lane,))]

    def test_idempotent(self, monkeypatch):
        from extras.AFC_spool import AFCSpool
        sentinel = lambda self, cur_lane, SpoolID, save_vars=True: None
        monkeypatch.setattr(AFCSpool, "set_spoolID", sentinel, raising=False)
        monkeypatch.setattr(AFCSpool, "_afc_autocal_spoolid_patched", True,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_spoolid_emit()
        assert AFCSpool.set_spoolID is sentinel


# ── _spoolman ─────────────────────────────────────────────────────────────────

class TestSpoolman:
    def test_none_without_afc(self):
        cal, _ = _make()
        assert cal._spoolman() is None

    def test_none_without_moonraker(self):
        cal, _ = _make()
        cal.afc = _make_afc()
        cal.afc.spoolman = object()
        assert cal._spoolman() is None

    def test_none_without_spoolman(self):
        cal, _ = _make()
        cal.afc = _make_afc()
        cal.afc.moonraker = object()
        assert cal._spoolman() is None

    def test_builds_client(self, monkeypatch):
        made = []
        monkeypatch.setattr(autocal_mod, "SpoolmanClient",
                            lambda mr: made.append(mr) or "client")
        cal, _ = _make()
        cal.afc = _make_afc()
        cal.afc.moonraker = "MR"
        cal.afc.spoolman = object()
        assert cal._spoolman() == "client"
        assert made == ["MR"]


# ── _norm_spool_id ────────────────────────────────────────────────────────────

class TestNormSpoolId:
    def test_empty_values_are_none(self):
        for sid in (None, "", 0, "0"):
            assert AFC_autocal._norm_spool_id(sid) is None

    def test_garbage_is_none(self):
        assert AFC_autocal._norm_spool_id("abc") is None
        assert AFC_autocal._norm_spool_id(object()) is None

    def test_valid_ids_normalize(self):
        assert AFC_autocal._norm_spool_id(7) == 7
        assert AFC_autocal._norm_spool_id("12") == 12


# ── _set_lane_k / _get_lane_k ─────────────────────────────────────────────────

class TestSetLaneK:
    def test_caches_keyed_to_spool(self):
        cal, _ = _make()
        lane = _make_lane(spool_id="5")
        cal._set_lane_k(lane, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}


class TestGetLaneK:
    def test_missing_is_none(self):
        cal, _ = _make()
        assert cal._get_lane_k(_make_lane()) is None

    def test_returns_cached_for_same_spool(self):
        cal, _ = _make()
        lane = _make_lane(spool_id=5)
        cal._set_lane_k(lane, 0.04)
        assert cal._get_lane_k(lane) == 0.04

    def test_spool_change_drops_entry(self):
        cal, _ = _make()
        lane = _make_lane(spool_id=5)
        cal._set_lane_k(lane, 0.04)
        lane.spool_id = 6
        assert cal._get_lane_k(lane) is None
        assert cal._lane_flow_k == {}                  # entry evicted


# ── _read_k_from_spoolman / _write_k_to_spoolman ──────────────────────────────

class TestReadKFromSpoolman:
    def test_none_without_spool(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_spoolman",
                            lambda: (_ for _ in ()).throw(AssertionError()))
        assert cal._read_k_from_spoolman(_make_lane(spool_id=None)) is None

    def test_none_without_client(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_spoolman", lambda: None)
        assert cal._read_k_from_spoolman(_make_lane(spool_id=5)) is None

    def test_reads_by_spool_id(self, monkeypatch):
        cal, _ = _make()
        client = types.SimpleNamespace(read_flow_k=lambda sid: {"sid": sid})
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        assert cal._read_k_from_spoolman(_make_lane(spool_id=5)) == {"sid": 5}


class TestWriteKToSpoolman:
    def test_noop_without_spool(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_spoolman",
                            lambda: (_ for _ in ()).throw(AssertionError()))
        cal._write_k_to_spoolman(_make_lane(spool_id=None), 0.04)

    def test_noop_without_client(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_spoolman", lambda: None)
        cal._write_k_to_spoolman(_make_lane(spool_id=5), 0.04)

    def test_writes_by_spool_id(self, monkeypatch):
        cal, _ = _make()
        writes = []
        client = types.SimpleNamespace(
            write_flow_k=lambda sid, k: writes.append((sid, k)))
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        cal._write_k_to_spoolman(_make_lane(spool_id=5), 0.04)
        assert writes == [(5, 0.04)]


# ── _apply_lane_k ─────────────────────────────────────────────────────────────

class TestApplyLaneK:
    def test_none_without_cache(self):
        cal, _ = _make()
        assert cal._apply_lane_k("lane1") is None

    def test_consumer_mode_applies_on_stepper(self):
        # No flow_calibrator -> consumer mode -> stepper apply (see
        # TestApplyKStepper); with no resolvable extruder it warns.
        cal, printer = _make()
        cal.afc = _make_afc()
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") is None
        assert cal.logger.messages == [
            ("warning", "AFC autocal: extruder extruder not found")]

    def test_applies_to_active_extruder(self):
        cal, printer = _make(calibrator=True)
        flow = printer.objects["flow_calibrator"]
        printer.objects["toolhead"] = _Toolhead("extruder")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        msg = cal._apply_lane_k("lane1")
        # re-derive the expected message independently
        assert msg == "AFC autocal: applied K=0.040000 for lane1 on extruder"
        assert flow.applied == [(printer.objects["toolhead"].extruder, 0.04)]
        assert flow._current_k == {"extruder": 0.04}
        assert cal.logger.messages == [("info", msg)]


# ── _ensure_k_loaded ──────────────────────────────────────────────────────────

class TestEnsureKLoaded:
    def test_cached_short_circuits(self, monkeypatch):
        cal, _ = _make()
        lane = _make_lane(spool_id=5)
        cal._set_lane_k(lane, 0.04)
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        assert cal._ensure_k_loaded(lane) == 0.04

    def test_reads_and_caches(self, monkeypatch):
        cal, _ = _make()
        lane = _make_lane(spool_id=5)
        monkeypatch.setattr(cal, "_read_k_from_spoolman", lambda l: 0.05)
        assert cal._ensure_k_loaded(lane) == 0.05
        assert cal._lane_flow_k == {"lane1": (5, 0.05)}

    def test_none_when_neither(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_read_k_from_spoolman", lambda l: None)
        lane = _make_lane(spool_id=5)
        assert cal._ensure_k_loaded(lane) is None
        assert cal._lane_flow_k == {}


# ── _calibrate ────────────────────────────────────────────────────────────────

class TestCalibrate:
    def test_missing_flow_calibrator_gcmd(self):
        cal, _ = _make()
        g = _Gcmd()
        assert cal._calibrate(_make_lane(), gcmd=g) is None
        assert g.lines == ["AFC_autocal: flow_calibrator not found"]
        assert cal.logger.messages == []

    def test_missing_flow_calibrator_logs(self):
        cal, _ = _make()
        assert cal._calibrate(_make_lane()) is None
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: flow_calibrator not found")]

    def test_no_new_k(self):
        cal, printer = _make()
        flow = _FlowCal()
        flow._current_k["extruder"] = 0.03
        printer.objects["flow_calibrator"] = flow
        lane = _make_lane()
        assert cal._calibrate(lane) is None            # unchanged K
        assert printer.gcode.script_cmds == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_new_k_cached_applied_persisted(self, monkeypatch):
        cal, printer = _make()
        flow = _FlowCal()
        printer.objects["flow_calibrator"] = flow
        printer.objects["toolhead"] = _Toolhead("extruder")
        lane = _make_lane(spool_id=5)
        writes = []
        monkeypatch.setattr(cal, "_write_k_to_spoolman",
                            lambda l, k: writes.append((l.name, k)))

        def runner(script):
            flow._current_k["extruder"] = 0.07         # macro produced a K
        assert cal._calibrate(lane, runner=runner) == 0.07
        assert cal._lane_flow_k == {"lane1": (5, 0.07)}
        assert writes == [("lane1", 0.07)]
        assert printer.gcode.script_cmds == []         # custom runner used
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.070000 for lane1 on extruder"),
            ("info",
             "AFC autocal: calibrated and stored K=0.070000 for lane1")]

    def test_uses_th_extruder_name_for_k_key(self):
        cal, printer = _make()
        flow = _FlowCal()
        flow._current_k["extruder2"] = 0.03            # keyed by Klipper name
        printer.objects["flow_calibrator"] = flow
        lane = _make_lane(ext_name="e2", th_name="extruder2")
        assert cal._calibrate(lane) is None            # 0.03 unchanged => none


# ── _current_lane ─────────────────────────────────────────────────────────────

class TestCurrentLane:
    def test_none_without_afc(self):
        cal, _ = _make()
        assert cal._current_lane() is None

    def test_none_on_exception(self):
        cal, _ = _make()
        cal.afc = _make_afc()
        cal.afc.function.get_current_lane_obj = \
            lambda: (_ for _ in ()).throw(RuntimeError())
        assert cal._current_lane() is None

    def test_returns_current(self):
        cal, _ = _make()
        lane = _make_lane()
        cal.afc = _make_afc(current=lane)
        assert cal._current_lane() is lane


# ── _handle_tool_loaded ───────────────────────────────────────────────────────

class TestHandleToolLoaded:
    def test_gated_on_afc_lane_and_toggles(self):
        cal, printer = _make({"apply_stored_k": True})
        cal._handle_tool_loaded(_make_lane())          # afc None
        cal.afc = _make_afc()
        cal._handle_tool_loaded(None)                  # lane None
        cal.apply_stored_k = False
        cal._handle_tool_loaded(_make_lane())          # both toggles off
        assert printer.reactor.callbacks == []

    def test_defers_to_reactor(self, monkeypatch):
        cal, printer = _make({"apply_stored_k": True})
        cal.afc = _make_afc()
        lane = _make_lane()
        seen = []
        monkeypatch.setattr(cal, "_do_tool_loaded", lambda l: seen.append(l))
        cal._handle_tool_loaded(lane)
        assert len(printer.reactor.callbacks) == 1
        printer.reactor.run_pending()
        assert seen == [lane]


# ── _do_tool_loaded ───────────────────────────────────────────────────────────

class TestDoToolLoaded:
    def test_cached_k_applied_when_active(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        lane = _make_lane(spool_id=5)
        cal._set_lane_k(lane, 0.04)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k",
                            lambda n: applied.append(n))
        fetched = []
        monkeypatch.setattr(cal, "_fetch_k_async",
                            lambda l: fetched.append(l))
        cal._do_tool_loaded(lane)
        assert applied == ["lane1"]
        assert fetched == []

    def test_cached_k_not_applied_when_inactive(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        lane = _make_lane(spool_id=5)
        cal._set_lane_k(lane, 0.04)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k",
                            lambda n: applied.append(n))
        cal._do_tool_loaded(lane)
        assert applied == []

    def test_uncached_fetches_async(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        lane = _make_lane(spool_id=5)
        fetched = []
        monkeypatch.setattr(cal, "_fetch_k_async",
                            lambda l: fetched.append(l))
        cal._do_tool_loaded(lane)
        assert fetched == [lane]

    def test_error_logged(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        monkeypatch.setattr(cal, "_get_lane_k",
                            lambda l: (_ for _ in ()).throw(RuntimeError("x")))
        cal._do_tool_loaded(_make_lane())
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: tool_loaded error: x")]


# ── _handle_spool_assigned ────────────────────────────────────────────────────

class TestHandleSpoolAssigned:
    def test_gated(self):
        cal, printer = _make({"auto_calibrate": True})
        cal._handle_spool_assigned(_make_lane())       # afc None
        cal.afc = _make_afc()
        cal._handle_spool_assigned(None)               # lane None
        cal.auto_calibrate = False
        cal._handle_spool_assigned(_make_lane())       # toggle off
        assert printer.reactor.callbacks == []

    def test_defers_to_reactor(self, monkeypatch):
        cal, printer = _make({"auto_calibrate": True}, calibrator=True)
        cal.afc = _make_afc()
        lane = _make_lane()
        seen = []
        monkeypatch.setattr(cal, "_do_spool_assigned",
                            lambda l, attempts=0: seen.append(l))
        cal._handle_spool_assigned(lane)
        printer.reactor.run_pending()
        assert seen == [lane]


# ── _do_spool_assigned ────────────────────────────────────────────────────────

class TestDoSpoolAssigned:
    def _cal(self, monkeypatch, **lane_kw):
        cal, printer, afc = _make_ready({"auto_calibrate": True})
        lane = _make_lane(**lane_kw)
        checks = []
        monkeypatch.setattr(cal, "_check_staged_k_async",
                            lambda n, s: checks.append((n, s)))
        return cal, printer, lane, checks

    def test_spool_cleared_resets_tracking(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=None)
        cal._staged_handled["lane1"] = 5
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {}
        assert cal._staged_pending == set()
        assert checks == []

    def test_tool_loaded_skips(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, tool_loaded=True)
        cal._do_spool_assigned(lane)
        assert checks == []

    def test_no_filament_skips(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, load_state=False)
        cal._do_spool_assigned(lane)
        assert checks == []

    def test_already_handled_spool_skips(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        cal._staged_handled["lane1"] = 5
        cal._do_spool_assigned(lane)
        assert checks == []

    def test_pending_retry_dedupes_new_events(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane, attempts=0)
        assert checks == []

    def test_startup_grace_skips(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        cal._startup_cal_grace = 30.0
        cal._ready_time = printer.reactor.monotonic() - 1.0    # inside grace
        cal._do_spool_assigned(lane)
        assert checks == []
        assert "lane1" not in cal._staged_pending

    def test_not_settled_retries_bounded(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        monkeypatch.setattr(cal, "_safe_to_calibrate", lambda: False)
        cal._do_spool_assigned(lane, attempts=0)
        assert "lane1" in cal._staged_pending
        assert len(printer.reactor.callbacks) == 1
        assert checks == []
        cal._do_spool_assigned(lane, attempts=30)      # bound reached
        assert "lane1" not in cal._staged_pending

    def test_happy_path_records_and_checks_k(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert checks == [("lane1", 5)]

    def test_error_logged(self, monkeypatch):
        cal, printer, lane, checks = self._cal(monkeypatch, spool_id=5)
        monkeypatch.setattr(cal, "_norm_spool_id",
                            lambda s: (_ for _ in ()).throw(RuntimeError("y")))
        cal._do_spool_assigned(lane)
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: spool_assigned error: y")]


# ── _check_staged_k_async ─────────────────────────────────────────────────────

class TestCheckStagedKAsync:
    def test_reads_off_thread_then_hops_to_reactor(self, monkeypatch):
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make()
        cal.afc = _make_afc()
        client = types.SimpleNamespace(read_flow_k=lambda sid: 0.04)
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        results = []
        monkeypatch.setattr(cal, "_staged_k_ready",
                            lambda n, s, k, ok: results.append((n, s, k, ok)))
        cal._check_staged_k_async("lane1", 5)
        assert results == []                           # not before the hop
        printer.reactor.run_pending()
        assert results == [("lane1", 5, 0.04, True)]

    def test_read_failure_reports_not_ok(self, monkeypatch):
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make()
        client = types.SimpleNamespace(
            read_flow_k=lambda sid: (_ for _ in ()).throw(RuntimeError()))
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        results = []
        monkeypatch.setattr(cal, "_staged_k_ready",
                            lambda n, s, k, ok: results.append((k, ok)))
        cal._check_staged_k_async("lane1", 5)
        printer.reactor.run_pending()
        assert results == [(None, False)]


# ── _staged_k_ready ───────────────────────────────────────────────────────────

class TestStagedKReady:
    def _cal(self, lane=None):
        cal, printer, afc = _make_ready({"auto_calibrate": True})
        if lane is not None:
            afc.lanes[lane.name] = lane
        return cal, printer

    def test_lane_gone(self):
        cal, printer = self._cal()
        cal._staged_k_ready("lane1", 5, None, True)
        assert printer.gcode.scripts == []

    def test_spool_changed(self):
        lane = _make_lane(spool_id=6)
        cal, printer = self._cal(lane)
        cal._staged_k_ready("lane1", 5, None, True)
        assert printer.gcode.scripts == []

    def test_loaded_meanwhile(self):
        lane = _make_lane(spool_id=5, tool_loaded=True)
        cal, printer = self._cal(lane)
        cal._staged_k_ready("lane1", 5, None, True)
        assert printer.gcode.scripts == []

    def test_existing_k_cached_no_load(self):
        lane = _make_lane(spool_id=5)
        cal, printer = self._cal(lane)
        cal._staged_k_ready("lane1", 5, 0.04, True)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 already has "
                     "K=0.040000 — not auto-loading")]

    def test_unreadable_k_no_load(self):
        lane = _make_lane(spool_id=5)
        cal, printer = self._cal(lane)
        cal._staged_k_ready("lane1", 5, None, False)
        assert printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 K unknown (Spoolman "
                     "read failed) — not auto-loading")]

    def test_unsafe_no_load(self, monkeypatch):
        lane = _make_lane(spool_id=5)
        cal, printer = self._cal(lane)
        monkeypatch.setattr(cal, "_safe_to_calibrate", lambda: False)
        cal._staged_k_ready("lane1", 5, None, True)
        assert printer.gcode.scripts == []

    def test_no_k_auto_loads(self):
        lane = _make_lane(spool_id=5)
        cal, printer = self._cal(lane)
        cal._staged_k_ready("lane1", 5, None, True)
        assert printer.gcode.scripts == ["CHANGE_TOOL LANE=lane1"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 has no stored K — "
                     "loading to calibrate")]


# ── _fetch_k_async ────────────────────────────────────────────────────────────

class TestFetchKAsync:
    def test_inflight_dedupes(self, monkeypatch):
        cal, printer = _make({"apply_stored_k": True})
        cal._k_fetch_inflight.add("lane1")
        monkeypatch.setattr(threading, "Thread",
                            lambda **kw: (_ for _ in ()).throw(AssertionError()))
        cal._fetch_k_async(_make_lane(spool_id=5))

    def test_no_spool_calibrates_when_enabled(self, monkeypatch):
        cal, printer = _make({"auto_calibrate": True}, calibrator=True)
        cal.afc = _make_afc()
        called = []
        monkeypatch.setattr(cal, "_calibrate_when_loaded",
                            lambda l: called.append(l.name))
        cal._fetch_k_async(_make_lane(spool_id=None))
        assert called == ["lane1"]

    def test_no_spool_no_calibrate_when_disabled(self, monkeypatch):
        cal, printer = _make({"apply_stored_k": True})
        called = []
        monkeypatch.setattr(cal, "_calibrate_when_loaded",
                            lambda l: called.append(l.name))
        cal._fetch_k_async(_make_lane(spool_id=None))
        assert called == []

    def test_apply_off_goes_straight_to_k_applied(self, monkeypatch):
        cal, printer = _make({"auto_calibrate": True})
        seen = []
        monkeypatch.setattr(cal, "_k_applied",
                            lambda n, s, k: seen.append((n, s, k)))
        cal._fetch_k_async(_make_lane(spool_id=5))
        assert seen == [("lane1", 5, None)]

    def test_reads_then_applies_on_reactor(self, monkeypatch):
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make({"apply_stored_k": True})
        client = types.SimpleNamespace(read_flow_k=lambda sid: 0.04)
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        seen = []
        monkeypatch.setattr(cal, "_k_applied",
                            lambda n, s, k: seen.append((n, s, k)))
        cal._fetch_k_async(_make_lane(spool_id=5))
        assert cal._k_fetch_inflight == {"lane1"}
        printer.reactor.run_pending()
        assert seen == [("lane1", 5, 0.04)]

    def test_worker_failure_still_hops_back(self, monkeypatch):
        # the finally guarantees the inflight flag is always cleared via
        # _k_applied even when the read raises
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make({"apply_stored_k": True})
        client = types.SimpleNamespace(
            read_flow_k=lambda sid: (_ for _ in ()).throw(RuntimeError()))
        monkeypatch.setattr(cal, "_spoolman", lambda: client)
        seen = []
        monkeypatch.setattr(cal, "_k_applied",
                            lambda n, s, k: seen.append((n, s, k)))
        cal._fetch_k_async(_make_lane(spool_id=5))
        printer.reactor.run_pending()
        assert seen == [("lane1", 5, None)]


# ── _k_applied ────────────────────────────────────────────────────────────────

class TestKApplied:
    def test_clears_inflight_even_when_lane_gone(self):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        cal._k_fetch_inflight.add("lane1")
        cal._k_applied("lane1", 5, 0.04)
        assert cal._k_fetch_inflight == set()

    def test_stale_spool_ignored(self):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        afc.lanes["lane1"] = _make_lane(spool_id=6)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {}

    def test_k_cached_and_applied_when_active(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        afc.lanes["lane1"] = _make_lane(spool_id=5)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert applied == ["lane1"]

    def test_k_cached_not_applied_when_inactive(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        afc.lanes["lane1"] = _make_lane(spool_id=5)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert applied == []

    def test_no_k_calibrates_when_enabled(self, monkeypatch):
        cal, printer, afc = _make_ready(
            {"apply_stored_k": True, "auto_calibrate": True})
        lane = _make_lane(spool_id=5)
        afc.lanes["lane1"] = lane
        called = []
        monkeypatch.setattr(cal, "_calibrate_when_loaded",
                            lambda l: called.append(l))
        cal._k_applied("lane1", 5, None)
        assert called == [lane]

    def test_no_k_no_calibrate_when_disabled(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        afc.lanes["lane1"] = _make_lane(spool_id=5)
        called = []
        monkeypatch.setattr(cal, "_calibrate_when_loaded",
                            lambda l: called.append(l))
        cal._k_applied("lane1", 5, None)
        assert called == []

    def test_error_logged(self, monkeypatch):
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        afc.lanes["lane1"] = _make_lane(spool_id=5)
        monkeypatch.setattr(cal, "_set_lane_k",
                            lambda l, k: (_ for _ in ()).throw(
                                RuntimeError("z")))
        cal._k_applied("lane1", 5, 0.04)
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: K apply error: z")]


# ── _extruder_load_in_flight ──────────────────────────────────────────────────

class TestExtruderLoadInFlight:
    def test_no_extruder_obj(self):
        cal, _ = _make()
        lane = types.SimpleNamespace(extruder_obj=None)
        assert cal._extruder_load_in_flight(lane) is False

    def test_load_active_states(self):
        cal, _ = _make()
        assert cal._extruder_load_in_flight(
            _make_lane(load_active=True)) is True
        assert cal._extruder_load_in_flight(
            _make_lane(load_active=False)) is False


# ── _is_printing ──────────────────────────────────────────────────────────────

class TestIsPrinting:
    def test_states(self):
        cal, _ = _make()
        cal.afc = _make_afc(printing=True)
        assert cal._is_printing() is True
        cal.afc = _make_afc(printing=False)
        assert cal._is_printing() is False

    def test_exception_is_false(self):
        cal, _ = _make()
        cal.afc = None                                 # attribute error inside
        assert cal._is_printing() is False


# ── _calibrate_when_loaded ────────────────────────────────────────────────────

class TestCalibrateWhenLoaded:
    def _cal(self, monkeypatch, lane=None, **afc_kw):
        cal, printer, afc = _make_ready({"auto_calibrate": True},
                                        afc=_make_afc(**afc_kw))
        lane = lane or _make_lane(spool_id=5, tool_loaded=True)
        ran = []
        monkeypatch.setattr(cal, "_calibrate",
                            lambda l, runner=None: ran.append((l, runner)))
        return cal, printer, lane, ran

    def test_dedupes_running_chain(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch)
        cal._cal_pending.add("lane1")
        cal._calibrate_when_loaded(lane, attempts=0)
        assert ran == []

    def test_printing_skips(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch, printing=True)
        cal._calibrate_when_loaded(lane)
        assert ran == []
        assert "lane1" not in cal._cal_pending
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped — printing")]

    def test_prep_not_done_hard_skips(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch, prep_done=False)
        cal._calibrate_when_loaded(lane)
        assert ran == []
        assert cal.logger.messages == [
            ("info",
             "AFC autocal: lane1 calibration skipped — prep not done")]

    def test_startup_grace_hard_skips(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch)
        cal._startup_cal_grace = 30.0
        cal._ready_time = printer.reactor.monotonic() - 1.0
        cal._calibrate_when_loaded(lane)
        assert ran == []
        assert cal.logger.messages == [
            ("info",
             "AFC autocal: lane1 calibration skipped — within startup grace")]

    def test_transient_state_retries_then_gives_up(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch,
                                            state="State.LOADING")
        cal._calibrate_when_loaded(lane, attempts=0)
        assert "lane1" in cal._cal_pending
        assert len(printer.reactor.callbacks) == 1
        cal._calibrate_when_loaded(lane, attempts=240)
        assert "lane1" not in cal._cal_pending
        assert ran == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration gave up waiting to "
                     "settle (state=LOADING (not idle))")]

    def test_load_in_flight_waits(self, monkeypatch):
        lane = _make_lane(spool_id=5, tool_loaded=True, load_active=True)
        cal, printer, lane, ran = self._cal(monkeypatch, lane=lane)
        cal._calibrate_when_loaded(lane, attempts=0)
        assert len(printer.reactor.callbacks) == 1
        assert ran == []

    def test_unloaded_while_waiting_skips(self, monkeypatch):
        lane = _make_lane(spool_id=5, tool_loaded=False)
        cal, printer, lane, ran = self._cal(monkeypatch, lane=lane)
        cal._calibrate_when_loaded(lane)
        assert ran == []

    def test_off_toolhead_skips_with_message(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)
        printer.objects["toolhead"] = _Toolhead("extruder2")
        cal._calibrate_when_loaded(lane)
        assert ran == []
        assert cal.logger.messages == [
            ("info",
             "AFC autocal: lane1 calibration skipped — its tool is not on "
             "the toolhead (active extruder=extruder2); pick up/load this "
             "lane's tool to calibrate it")]

    def test_happy_runs_with_run_script_runner(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        cal._calibrate_when_loaded(lane)
        assert ran == [(lane, printer.gcode.run_script)]
        assert "lane1" not in cal._cal_pending
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1")]

    def test_error_clears_pending_and_logs(self, monkeypatch):
        cal, printer, lane, ran = self._cal(monkeypatch)
        monkeypatch.setattr(cal, "_is_printing",
                            lambda: (_ for _ in ()).throw(RuntimeError("q")))
        cal._calibrate_when_loaded(lane)
        assert "lane1" not in cal._cal_pending
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: deferred calibrate error: q")]


# ── _lane_on_active_toolhead ──────────────────────────────────────────────────

class TestLaneOnActiveToolhead:
    def test_no_extruder_obj(self):
        cal, printer = _make()
        printer.objects["toolhead"] = _Toolhead("extruder")
        lane = types.SimpleNamespace(extruder_obj=None)
        assert cal._lane_on_active_toolhead(lane) is False

    def test_matches_section_name(self):
        cal, printer = _make()
        printer.objects["toolhead"] = _Toolhead("e1")
        assert cal._lane_on_active_toolhead(
            _make_lane(ext_name="e1", th_name="extruder1")) is True

    def test_matches_klipper_name(self):
        cal, printer = _make()
        printer.objects["toolhead"] = _Toolhead("extruder1")
        assert cal._lane_on_active_toolhead(
            _make_lane(ext_name="e1", th_name="extruder1")) is True

    def test_mismatch(self):
        cal, printer = _make()
        printer.objects["toolhead"] = _Toolhead("extruder2")
        assert cal._lane_on_active_toolhead(
            _make_lane(ext_name="e1", th_name="extruder1")) is False

    def test_lookup_failure_is_false(self):
        cal, printer = _make()                         # no toolhead object
        assert cal._lane_on_active_toolhead(_make_lane()) is False


# ── _safe_to_calibrate / _cal_block_reason ────────────────────────────────────

class TestCalBlockReason:
    def test_prep_not_done(self):
        cal, printer, afc = _make_ready(afc=_make_afc(prep_done=False))
        assert cal._cal_block_reason() == "prep not done"

    def test_within_startup_grace(self):
        cal, printer, afc = _make_ready()
        cal._startup_cal_grace = 30.0
        cal._ready_time = printer.reactor.monotonic() - 1.0
        assert cal._cal_block_reason() == "within startup grace"

    def test_not_idle(self):
        cal, printer, afc = _make_ready(afc=_make_afc(state="State.LOADING"))
        assert cal._cal_block_reason() == "state=LOADING (not idle)"

    def test_idle_is_clear(self):
        cal, printer, afc = _make_ready()
        assert cal._cal_block_reason() is None


class TestSafeToCalibrate:
    def test_mirrors_block_reason(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_cal_block_reason", lambda: None)
        assert cal._safe_to_calibrate() is True
        monkeypatch.setattr(cal, "_cal_block_reason", lambda: "busy")
        assert cal._safe_to_calibrate() is False


# ── _reapply_current_k ────────────────────────────────────────────────────────

class TestReapplyCurrentK:
    def test_gated_off_when_not_applying(self, monkeypatch):
        cal, printer, afc = _make_ready()
        cal.apply_stored_k = False
        monkeypatch.setattr(cal, "_apply_lane_k",
                            lambda n: (_ for _ in ()).throw(AssertionError()))
        cal._reapply_current_k()

    def test_gated_on_prep_and_idle(self, monkeypatch):
        applied = []
        cal, printer, afc = _make_ready({"apply_stored_k": True},
                                        afc=_make_afc(prep_done=False))
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._reapply_current_k()
        cal.afc = _make_afc(state="State.LOADING")
        cal._reapply_current_k()
        assert applied == []

    def test_no_current_or_uncached_is_noop(self, monkeypatch):
        applied = []
        cal, printer, afc = _make_ready({"apply_stored_k": True})
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._reapply_current_k()                       # no current lane
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        cal._reapply_current_k()                       # no cached K
        assert applied == []

    def test_reapplies_cached_k(self, monkeypatch):
        lane = _make_lane(spool_id=5)
        cal, printer, afc = _make_ready({"apply_stored_k": True},
                                        afc=_make_afc(current=lane))
        cal._set_lane_k(lane, 0.04)
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._reapply_current_k()
        assert applied == ["lane1"]


# ── _handle_home_rails_end / _handle_activate_extruder ────────────────────────

class TestHandleHomeRailsEnd:
    def test_delegates(self, monkeypatch):
        cal, _ = _make()
        called = []
        monkeypatch.setattr(cal, "_reapply_current_k",
                            lambda: called.append(1))
        cal._handle_home_rails_end(None, None)
        assert called == [1]

    def test_error_logged(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_reapply_current_k",
                            lambda: (_ for _ in ()).throw(RuntimeError("h")))
        cal._handle_home_rails_end(None, None)
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: home reapply error: h")]


class TestHandleActivateExtruder:
    def test_delegates(self, monkeypatch):
        cal, _ = _make()
        called = []
        monkeypatch.setattr(cal, "_reapply_current_k",
                            lambda: called.append(1))
        cal._handle_activate_extruder()
        assert called == [1]

    def test_error_logged(self, monkeypatch):
        cal, _ = _make()
        monkeypatch.setattr(cal, "_reapply_current_k",
                            lambda: (_ for _ in ()).throw(RuntimeError("a")))
        cal._handle_activate_extruder()
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: activate reapply error: a")]


# ── cmd_APPLY_LANE_FLOW_K ─────────────────────────────────────────────────────

class TestCmdApplyLaneFlowK:
    def test_no_current_lane(self):
        cal, _ = _make()
        g = _Gcmd()
        cal.cmd_APPLY_LANE_FLOW_K(g)
        assert g.lines == ["AFC_autocal: no current lane"]

    def test_refuses_when_tool_not_on_toolhead(self, monkeypatch):
        cal, _ = _make(calibrator=True)
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)
        monkeypatch.setattr(cal, "_ensure_k_loaded",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        g = _Gcmd()
        cal.cmd_APPLY_LANE_FLOW_K(g)
        assert g.lines == [
            "AFC_autocal: lane1's tool is not on the toolhead — pick up/load "
            "its tool first"]

    def test_no_stored_k(self, monkeypatch):
        cal, _ = _make()
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        monkeypatch.setattr(cal, "_ensure_k_loaded", lambda l: None)
        g = _Gcmd()
        cal.cmd_APPLY_LANE_FLOW_K(g)
        assert g.lines == ["AFC_autocal: no stored K for lane1"]

    def test_applies(self, monkeypatch):
        cal, _ = _make()
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        monkeypatch.setattr(cal, "_ensure_k_loaded", lambda l: 0.04)
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: f"applied {n}")
        g = _Gcmd()
        cal.cmd_APPLY_LANE_FLOW_K(g)
        assert g.lines == ["applied lane1"]


# ── cmd_CALIBRATE_LANE_FLOW_K ─────────────────────────────────────────────────

class TestCmdCalibrateLaneFlowK:
    def test_no_current_lane(self):
        cal, _ = _make()
        g = _Gcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(g)
        assert g.lines == ["AFC_autocal: no current lane"]

    def test_no_new_k(self, monkeypatch):
        cal, _ = _make()
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        monkeypatch.setattr(cal, "_calibrate", lambda l, gcmd=None: None)
        g = _Gcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(g)
        assert g.lines == ["AFC_autocal: calibration produced no new K"]

    def test_reports_stored_k(self, monkeypatch):
        cal, _ = _make()
        cal.afc = _make_afc(current=_make_lane(spool_id=5))
        monkeypatch.setattr(cal, "_calibrate", lambda l, gcmd=None: 0.07)
        g = _Gcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(g)
        assert g.lines == ["AFC_autocal: stored K=0.070000 for lane1"]


# ── mode detection ────────────────────────────────────────────────────────────

class TestFlowCalibrator:
    def test_present_and_absent(self):
        cal, printer = _make(calibrator=True)
        assert cal._flow_calibrator() is printer.objects["flow_calibrator"]
        cal2, _ = _make()
        assert cal2._flow_calibrator() is None


class TestCanCalibrate:
    def test_requires_toggle_and_calibrator(self):
        cal, _ = _make({"auto_calibrate": True}, calibrator=True)
        assert cal._can_calibrate() is True

    def test_false_without_calibrator(self):
        cal, _ = _make({"auto_calibrate": True})    # consumer mode
        assert cal._can_calibrate() is False

    def test_false_without_toggle(self):
        cal, _ = _make(calibrator=True)             # auto_calibrate off
        assert cal._can_calibrate() is False


class TestApplyGateOk:
    def test_consumer_mode_always_ok(self):
        cal, _ = _make()                            # no flow_calibrator
        assert cal._apply_gate_ok(_make_lane()) is True

    def test_u1_mode_defers_to_active_toolhead(self, monkeypatch):
        cal, _ = _make(calibrator=True)
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)
        assert cal._apply_gate_ok(_make_lane()) is False
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: True)
        assert cal._apply_gate_ok(_make_lane()) is True


# ── consumer-mode apply (extruder stepper) ──────────────────────────────────────

class TestApplyKStepper:
    def _cal_with_ext(self, ext_name="extruder", estepper=None):
        cal, printer = _make()
        estepper = estepper if estepper is not None else _Estepper()
        printer.objects[ext_name] = _PrinterExtruder(estepper)
        lane = _make_lane(ext_name=ext_name)
        cal.afc = _make_afc(lanes={"lane1": lane})
        return cal, printer, estepper

    def test_applies_on_lane_own_extruder(self):
        cal, printer, estepper = self._cal_with_ext("extruder1")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        msg = cal._apply_lane_k("lane1")
        assert msg == "AFC autocal: applied K=0.040000 for lane1 on extruder1"
        assert estepper.applied == [(0.04, 0.04)]
        assert cal._managed_extruders == {"extruder1"}
        assert cal.logger.messages == [("info", msg)]

    def test_prefers_config_smooth_time(self):
        estepper = _Estepper(smooth_time=0.04)
        estepper.config_smooth_time = 0.01
        cal, printer, estepper = self._cal_with_ext("extruder", estepper)
        cal._lane_flow_k["lane1"] = (5, 0.05)
        cal._apply_lane_k("lane1")
        assert estepper.applied == [(0.05, 0.01)]

    def test_extruder_not_found_warns(self):
        cal, printer = _make()
        cal.afc = _make_afc(lanes={"lane1": _make_lane(ext_name="gone")})
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") is None
        assert cal._managed_extruders == set()
        assert cal.logger.messages == [
            ("warning", "AFC autocal: extruder gone not found")]

    def test_no_stepper_warns(self):
        cal, printer = _make()
        printer.objects["extruder"] = types.SimpleNamespace(
            extruder_stepper=None)
        cal.afc = _make_afc(lanes={"lane1": _make_lane(ext_name="extruder")})
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") is None
        assert cal.logger.messages == [
            ("warning",
             "AFC autocal: extruder extruder has no extruder_stepper")]


# ── consumer-mode PA-override blocking ──────────────────────────────────────────

class TestWrapPaHandlers:
    def _setup(self, ext="extruder"):
        cal, printer = _make()
        cal.afc = _make_afc(printing=True)
        orig_calls = []
        printer.gcode.register_mux_command(
            "SET_PRESSURE_ADVANCE", "EXTRUDER", ext,
            lambda gcmd: orig_calls.append(gcmd))
        return cal, printer, orig_calls, ext

    def test_no_mux_is_noop(self):
        cal, printer = _make()
        cal._wrap_pa_handlers()                     # no SET_PRESSURE_ADVANCE
        assert cal._wrapped_extruders == set()

    def test_wraps_each_once(self):
        cal, printer, _orig, ext = self._setup()
        cal._wrap_pa_handlers()
        first = printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1][ext]
        cal._wrap_pa_handlers()                     # idempotent
        second = printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1][ext]
        assert first is second
        assert cal._wrapped_extruders == {ext}

    def test_blocks_slicer_pa_for_managed_extruder_while_printing(self):
        cal, printer, orig_calls, ext = self._setup()
        cal._managed_extruders.add(ext)
        cal._wrap_pa_handlers()
        wrapper = printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1][ext]
        g = _Gcmd()
        wrapper(g)
        assert orig_calls == []                     # original NOT called
        assert g.lines == [
            "AFC flow K active — slicer pressure advance ignored"]
        assert cal.logger.messages == [
            ("info",
             "AFC autocal: slicer PA change ignored for extruder "
             "(flow K managed)")]

    def test_passes_through_when_not_managed(self):
        cal, printer, orig_calls, ext = self._setup()
        cal._wrap_pa_handlers()                     # ext not managed
        wrapper = printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1][ext]
        g = _Gcmd()
        wrapper(g)
        assert orig_calls == [g]                     # original called
        assert g.lines == []

    def test_passes_through_when_not_printing(self):
        cal, printer, orig_calls, ext = self._setup()
        cal.afc = _make_afc(printing=False)
        cal._managed_extruders.add(ext)
        cal._wrap_pa_handlers()
        wrapper = printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1][ext]
        g = _Gcmd()
        wrapper(g)
        assert orig_calls == [g]


# ── consumer-mode startup preload ───────────────────────────────────────────────

class TestLoadAllSpoolmanK:
    def test_apply_off_noop(self, monkeypatch):
        cal, _ = _make()                             # apply_stored_k off
        afc = _make_afc(lanes={"lane1": _make_lane(spool_id=5)})
        afc.moonraker = object()
        cal.afc = afc
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {}

    def test_no_moonraker_noop(self, monkeypatch):
        cal, _ = _make({"apply_stored_k": True})
        cal.afc = _make_afc(lanes={"lane1": _make_lane()})    # moonraker None
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {}

    def test_all_lanes_loaded(self, monkeypatch):
        cal, _ = _make({"apply_stored_k": True})
        a = _make_lane(name="a", spool_id=5)
        b = _make_lane(name="b", spool_id=6)
        afc = _make_afc(lanes={"a": a, "b": b})
        afc.moonraker = object()
        cal.afc = afc
        reads = []
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: reads.append(l.name) or 0.04)
        cal._load_all_spoolman_k()
        assert sorted(reads) == ["a", "b"]
        assert cal._lane_flow_k == {"a": (5, 0.04), "b": (6, 0.04)}

    def test_already_cached_skipped(self, monkeypatch):
        cal, _ = _make({"apply_stored_k": True})
        lane = _make_lane(name="s", spool_id=5)
        afc = _make_afc(lanes={"s": lane})
        afc.moonraker = object()
        cal.afc = afc
        cal._set_lane_k(lane, 0.09)
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {"s": (5, 0.09)}

    def test_read_error_swallowed(self, monkeypatch):
        cal, _ = _make({"apply_stored_k": True})
        lane = _make_lane(name="s", spool_id=5)
        afc = _make_afc(lanes={"s": lane})
        afc.moonraker = object()
        cal.afc = afc
        monkeypatch.setattr(cal, "_read_k_from_spoolman",
                            lambda l: (_ for _ in ()).throw(RuntimeError("x")))
        cal._load_all_spoolman_k()                   # must not raise
        assert cal._lane_flow_k == {}


# ── consumer-mode wiring at ready ───────────────────────────────────────────────

class TestHandleReadyConsumerMode:
    def test_wraps_and_preloads_in_consumer_mode(self, monkeypatch):
        cal, printer = _make({"apply_stored_k": True})   # no flow_calibrator
        printer.objects["AFC"] = _make_afc()
        calls = []
        monkeypatch.setattr(cal, "_wrap_pa_handlers",
                            lambda: calls.append("wrap"))
        monkeypatch.setattr(cal, "_load_all_spoolman_k",
                            lambda: calls.append("preload"))
        cal._handle_ready()
        assert calls == ["wrap", "preload"]

    def test_u1_mode_skips_wrap_and_preload(self, monkeypatch):
        cal, printer = _make({"apply_stored_k": True}, calibrator=True)
        printer.objects["AFC"] = _make_afc()
        calls = []
        monkeypatch.setattr(cal, "_wrap_pa_handlers",
                            lambda: calls.append("wrap"))
        monkeypatch.setattr(cal, "_load_all_spoolman_k",
                            lambda: calls.append("preload"))
        cal._handle_ready()
        assert calls == []

# ── defensive-path coverage ─────────────────────────────────────────────────────

class TestPatchImportFailures:
    def test_tool_loaded_import_failure_warns(self, monkeypatch):
        import sys
        # A module without AFCLane makes `from ... import AFCLane` raise.
        monkeypatch.setitem(sys.modules, 'extras.AFC_lane',
                            types.SimpleNamespace())
        cal, _ = _make()
        cal._patch_set_tool_loaded_emit()
        assert cal.logger.messages[0][0] == "warning"
        assert "cannot patch set_tool_loaded" in cal.logger.messages[0][1]

    def test_spoolid_import_failure_warns(self, monkeypatch):
        import sys
        monkeypatch.setitem(sys.modules, 'extras.AFC_spool',
                            types.SimpleNamespace())
        cal, _ = _make()
        cal._patch_set_spoolid_emit()
        assert cal.logger.messages[0][0] == "warning"
        assert "cannot patch set_spoolID" in cal.logger.messages[0][1]


class TestPatchEmitSwallowsSendEventError:
    def test_tool_loaded_emit_swallows(self, monkeypatch):
        from extras.AFC_lane import AFCLane
        monkeypatch.setattr(AFCLane, "set_tool_loaded",
                            lambda self, normal_toolchange=False: None,
                            raising=False)
        monkeypatch.setattr(AFCLane, "_afc_autocal_emit_patched", False,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_tool_loaded_emit()

        class _P:
            def send_event(self, *a):
                raise RuntimeError("boom")
        AFCLane.set_tool_loaded(types.SimpleNamespace(printer=_P()))  # no raise

    def test_spoolid_emit_swallows(self, monkeypatch):
        from extras.AFC_spool import AFCSpool
        monkeypatch.setattr(AFCSpool, "set_spoolID",
                            lambda self, cur_lane, SpoolID, save_vars=True: None,
                            raising=False)
        monkeypatch.setattr(AFCSpool, "_afc_autocal_spoolid_patched", False,
                            raising=False)
        cal, _ = _make()
        cal._patch_set_spoolid_emit()

        class _P:
            def send_event(self, *a):
                raise RuntimeError("boom")
        AFCSpool.set_spoolID(types.SimpleNamespace(printer=_P()),
                             _make_lane(), 5)                          # no raise


class TestLoadAllSpoolmanKReadNone:
    def test_read_none_not_cached(self, monkeypatch):
        cal, _ = _make({"apply_stored_k": True})
        lane = _make_lane(name="s", spool_id=5)
        afc = _make_afc(lanes={"s": lane})
        afc.moonraker = object()
        cal.afc = afc
        monkeypatch.setattr(cal, "_read_k_from_spoolman", lambda l: None)
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {}                    # read None -> no cache


class TestDoToolLoadedGuards:
    def test_no_afc_returns(self):
        cal, _ = _make({"apply_stored_k": True})         # afc None
        cal._do_tool_loaded(_make_lane())                # no raise, returns

    def test_disabled_returns(self, monkeypatch):
        cal, _ = _make()                                 # toggles off
        cal.afc = _make_afc()
        monkeypatch.setattr(cal, "_get_lane_k",
                            lambda l: (_ for _ in ()).throw(AssertionError()))
        cal._do_tool_loaded(_make_lane())                # gate false -> returns


class TestDoSpoolAssignedGuard:
    def test_cannot_calibrate_returns(self, monkeypatch):
        cal, _ = _make({"auto_calibrate": True})         # no flow_calibrator
        cal.afc = _make_afc()
        monkeypatch.setattr(cal, "_norm_spool_id",
                            lambda s: (_ for _ in ()).throw(AssertionError()))
        cal._do_spool_assigned(_make_lane(spool_id=5))   # _can_calibrate False


class TestCheckStagedKAsyncNoClient:
    def test_no_client_reports_none(self, monkeypatch):
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make()
        monkeypatch.setattr(cal, "_spoolman", lambda: None)
        results = []
        monkeypatch.setattr(cal, "_staged_k_ready",
                            lambda n, s, k, ok: results.append((k, ok)))
        cal._check_staged_k_async("lane1", 5)
        printer.reactor.run_pending()
        assert results == [(None, False)]               # no client -> read_ok False


class TestStagedKReadyError:
    def test_error_logged(self, monkeypatch):
        cal, printer, afc = _make_ready({"auto_calibrate": True})
        afc.lanes["lane1"] = _make_lane(spool_id=5)
        monkeypatch.setattr(cal, "_safe_to_calibrate",
                            lambda: (_ for _ in ()).throw(RuntimeError("boom")))
        cal._staged_k_ready("lane1", 5, None, True)
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: staged load error: boom")]


class TestFetchKAsyncNoClient:
    def test_no_client_applies_none(self, monkeypatch):
        monkeypatch.setattr(threading, "Thread", _ImmediateThread)
        cal, printer = _make({"apply_stored_k": True})
        monkeypatch.setattr(cal, "_spoolman", lambda: None)
        seen = []
        monkeypatch.setattr(cal, "_k_applied",
                            lambda n, s, k: seen.append((n, s, k)))
        cal._fetch_k_async(_make_lane(spool_id=5))
        printer.reactor.run_pending()
        assert seen == [("lane1", 5, None)]             # client None -> k None


class TestCalibrateWhenLoadedActiveUnknown:
    def test_off_toolhead_active_unknown(self, monkeypatch):
        cal, printer, afc = _make_ready({"auto_calibrate": True})
        lane = _make_lane(spool_id=5, tool_loaded=True)
        ran = []
        monkeypatch.setattr(cal, "_calibrate",
                            lambda l, runner=None: ran.append(l))
        monkeypatch.setattr(cal, "_lane_on_active_toolhead", lambda l: False)

        class _BadTH:
            def get_extruder(self):
                raise RuntimeError()
        printer.objects["toolhead"] = _BadTH()
        cal._calibrate_when_loaded(lane)
        assert ran == []
        assert cal.logger.messages == [
            ("info",
             "AFC autocal: lane1 calibration skipped — its tool is not on "
             "the toolhead (active extruder=?); pick up/load this lane's "
             "tool to calibrate it")]


class TestLaneOnActiveToolheadNoneExtruder:
    def test_active_extruder_none(self):
        cal, printer = _make()

        class _THNone:
            def get_extruder(self):
                return None
        printer.objects["toolhead"] = _THNone()
        assert cal._lane_on_active_toolhead(_make_lane()) is False


class TestReapplyCurrentKGuards:
    def test_no_afc_returns(self):
        cal, _ = _make({"apply_stored_k": True})         # afc None
        cal._reapply_current_k()                         # no raise

    def test_apply_not_enabled_skips(self, monkeypatch):
        lane = _make_lane(spool_id=5)
        cal, printer, afc = _make_ready(afc=_make_afc(current=lane))
        cal._set_lane_k(lane, 0.04)                      # apply_stored_k off
        applied = []
        monkeypatch.setattr(cal, "_apply_lane_k", lambda n: applied.append(n))
        cal._reapply_current_k()
        assert applied == []                             # not enabled -> skip


# ── load_config ───────────────────────────────────────────────────────────────

class TestLoadConfig:
    def test_builds_instance(self):
        printer = _Printer()
        cal = load_config(_Config(printer))
        assert isinstance(cal, AFC_autocal)
