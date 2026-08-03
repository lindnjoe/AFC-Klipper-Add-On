"""
Tests for the OpenAMS RFID scan-on-insert flow (afcAMS._do_rfid_scan and the
insert-edge scheduling/latch around it).

The scan drives real filament motion, so the whole point of these tests is to
pin the ORCHESTRATION (TD-1-capture style): wait for the unit's ready signal,
take the sister-tag baseline on the shared antenna, send a verified NORMAL
load, gate reader polling on encoder movement, light-probe for a NEW uid, stop
the feed at the detection position, full-read stationary (with an
unload/re-feed-to-position retry when unreadable), and always unwind back —
with the operation guard and follower state cleaned up on every path.
"""

import sys
import types
from unittest.mock import MagicMock

import pytest

# Match the defensive stubs the sibling OpenAMS test modules install.
_mcu_stub = types.ModuleType("mcu")
_mcu_stub.get_printer_mcu = MagicMock()
sys.modules.setdefault("mcu", _mcu_stub)
_bus_stub = types.ModuleType("extras.bus")
_bus_stub.MCU_I2C_from_config = MagicMock()
sys.modules.setdefault("extras.bus", _bus_stub)

from extras.AFC_OpenAMS import afcAMS, OAMSStatus  # noqa: E402
from tests.conftest import MockAFC, MockPrinter, MockConfig  # noqa: E402


class AdvancingReactor:
    """Reactor whose pause() advances monotonic time, so timed loops terminate."""

    NEVER = 9_999_999_999.0
    NOW = 0.0

    def __init__(self, monotonic_value=100.0):
        self._monotonic = monotonic_value
        self.registered = []

    def monotonic(self):
        return self._monotonic

    def pause(self, until):
        # Callers pass monotonic()+delay; advance to it so deadlines are reached.
        self._monotonic = max(self._monotonic, until)

    def register_timer(self, callback, waketime=None):
        handle = ("timer", len(self.registered))
        self.registered.append((handle, callback, waketime))
        return handle

    def unregister_timer(self, handle):
        self.registered = [r for r in self.registered if r[0] != handle]


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args=None):
        self.sent.append(args)


class FakeController:
    """Minimal AFC_OAMS stand-in exposing exactly what the scan touches.

    The load command "moves filament": sending it bumps encoder_clicks past
    the polling gate and trips the hub HES, so engagement detection and the
    encoder gate both see motion.
    """

    def __init__(self, reactor=None):
        self.follower_calls = []
        self.oams_load_spool_cmd = FakeCmd()
        self.oams_load_spool_cmd.send = self._send_load
        self.action_status = None
        self.action_status_code = None
        # The firmware reports its motor state only in ANSWER to a command --
        # there is no periodic stream -- which is why the ready-wait probes.
        self.reactor = reactor
        self.motion_status = None
        self.motion_status_code = None
        self.motion_status_time = 0.0
        self.cancel_calls = 0
        self.unload_calls = 0
        self.clear_errors_calls = 0
        self.current_spool = 3
        self.encoder_clicks = 500          # running counter, never zero
        self.hub_hes_value = [0, 0, 0, 0]
        # Number of load attempts to reject ERROR_BUSY before accepting
        # (models the firmware's insert-staging window).
        self.busy_rejections = 0
        # Number of readiness PROBES answered ERROR_BUSY before the unit
        # reports STOPPED. Independent of busy_rejections: a load can be
        # refused for reasons that have nothing to do with the motor.
        self.staging_probes = 0

    def _send_load(self, args):
        # Model the firmware load: filament moves, hub trips, load completes.
        self.oams_load_spool_cmd.sent.append(args)
        if self.busy_rejections > 0:
            self.busy_rejections -= 1
            self.action_status = None
            self.action_status_code = 2    # OAMSOpCode.ERROR_BUSY
            return
        self.encoder_clicks += 120
        self.hub_hes_value[args[0]] = 1
        self.action_status = None          # ack: load done
        self.action_status_code = 0        # OAMSOpCode.SUCCESS

    # Motor primitives
    def set_oams_follower(self, enable, direction):
        self.follower_calls.append((enable, direction))
        # Answer like the firmware: a status comes back only when the command
        # is REFUSED or the motor state actually CHANGES. While a routine
        # (e.g. the insert auto-stage) owns the motor, every stop is refused
        # ERROR_BUSY. Once it is done, a stop sent to an already-stopped unit
        # changes nothing -- so the firmware answers with SILENCE, and silence
        # is what the ready-wait has to read as "ready".
        if self.staging_probes > 0:
            self.staging_probes -= 1
            self.motion_status = OAMSStatus.REVERSE_FOLLOWING
            self.motion_status_code = 2      # OAMSOpCode.ERROR_BUSY
            if self.reactor is not None:
                self.motion_status_time = self.reactor.monotonic()

    def load_spool_cancel(self):
        self.cancel_calls += 1
        self.action_status = None
        return "cancelled"

    def unload_spool(self):
        self.unload_calls += 1
        return True, "ok"

    def clear_errors(self):
        self.clear_errors_calls += 1

    def is_bay_ready(self, bay):
        return True


class FakeCoordinator:
    """AFC_OpenAMS_rfid stand-in with the field/read API.

    ``fields`` is a list of uid-lists, one per scan_slot_uids() call: the
    FIRST is the rest-time baseline, later entries are the per-poll field
    contents during the feed (the last entry repeats once exhausted).
    """

    def __init__(self, fields=None, full_reads=None):
        self._fields = [list(f) for f in (fields or [[]])]
        self._reads = list(full_reads or [])
        self.read_excludes = []
        self.applied = []
        self.slot_map = {"lane1": 0}

    def _get_slot(self, name):
        return self.slot_map.get(name)

    def scan_slot_uids(self, slot):
        if len(self._fields) > 1:
            return self._fields.pop(0)
        return list(self._fields[0])

    def read_slot_excluding(self, slot, exclude):
        self.read_excludes.append(set(exclude))
        if not self._reads:
            return None
        return self._reads.pop(0)

    def read_slot(self, slot):              # manual-path compat
        return self.read_slot_excluding(slot, set())

    def apply_to_lane(self, lane, tag):
        self.applied.append((lane, tag))
        return {"brand": "X", "material": "PLA"}

    def undecoded_hint(self, name):
        return ""


class FakeLane:
    def __init__(self, name="lane1"):
        self.name = name
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.send_lane_data_calls = 0

    def send_lane_data(self):
        self.send_lane_data_calls += 1


def _build_unit(values=None, coord=None):
    afc = MockAFC()
    afc.reactor = AdvancingReactor()
    printer = MockPrinter(afc=afc)
    printer.reactor = afc.reactor
    cfg_values = {"rfid_scan_on_insert": True, "rfid_scan_timeout": 2.0,
                  "rfid_scan_poll": 0.2, "rfid_scan_read_retries": 2}
    cfg_values.update(values or {})
    config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values=cfg_values)
    ams = afcAMS(config)
    ams.afc = afc
    # in_print() must return a real bool (MagicMock's default is truthy).
    afc.function.in_print = lambda: False
    ams.oams = FakeController(reactor=afc.reactor)
    ams.lanes = {"lane1": FakeLane()}
    ams._spool_map = {"lane1": 0}
    coord = coord if coord is not None else FakeCoordinator()
    ams._rfid_coord = coord
    printer._objects["AFC_OpenAMS_rfid"] = coord
    return ams, coord


TAG = {"uid": "AABB", "filament": {"material": "PLA"}, "tag_type": "MifareClassic1k"}


class TestConfigDefaults:
    def test_defaults(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values={})
        ams = afcAMS(config)
        assert ams.rfid_scan_on_insert is False
        assert ams.rfid_scan_timeout == 15.0
        assert ams.rfid_scan_read_retries == 3
        # Tuned on hardware: sweep_back must cover the re-feed overshoot
        # (~150 clicks) before it buys any pre-roll before the detect point.
        assert ams.rfid_scan_sweep_back == 240
        assert ams.rfid_scan_sweep_step == 25
        assert ams.rfid_scan_sweep_past == 200

    def test_enabled(self):
        ams, _ = _build_unit()
        assert ams.rfid_scan_on_insert is True


class TestScanTagFound:
    def _run(self):
        # Empty field at rest; the moving tag arrives on the second poll.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_true(self):
        _, _, result = self._run()
        assert result is True

    def test_applies_tag_to_lane(self):
        ams, coord, _ = self._run()
        assert len(coord.applied) == 1
        lane, tag = coord.applied[0]
        assert lane is ams.lanes["lane1"]
        assert tag is TAG

    def test_surfaces_to_mainsail(self):
        ams, _, _ = self._run()
        assert ams.lanes["lane1"].send_lane_data_calls == 1

    def test_persists_across_restart(self):
        # Without save_vars a FIRMWARE_RESTART wipes the applied data when
        # PREP rebuilds lane_data (the field-observed lane6 clear).
        ams, _, _ = self._run()
        assert ams.afc.save_vars.called

    def test_sends_the_load_once(self):
        ams, _, _ = self._run()
        assert ams.oams.oams_load_spool_cmd.sent == [[0]]

    def test_follower_stopped_at_end(self):
        ams, _, _ = self._run()
        assert (1, 1) in ams.oams.follower_calls      # pre-load forward
        assert ams.oams.follower_calls[-1] == (0, 0)  # stopped at the end

    def test_unwinds_after_engagement(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1

    def test_clears_operation_guard_and_latches(self):
        ams, _, _ = self._run()
        assert ams._operation_active is False
        assert ams._prev_states_stale is True
        assert "lane1" in ams._rfid_scanned


class TestSisterTags:
    def test_constant_sister_never_detected(self):
        # A seated neighbour's tag answers every poll — stationary = sister,
        # so a scan with ONLY it in field times out instead of detecting.
        coord = FakeCoordinator(fields=[["5157e12"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []

    def test_new_uid_beside_sister_detected_and_sister_excluded(self):
        # Sister present throughout; the moving tag arrives later — it is
        # detected and the sister is excluded from the full read.
        coord = FakeCoordinator(
            fields=[["5157e12"], ["5157e12"], ["5157e12", "aabb"]],
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert all("5157e12" in ex for ex in coord.read_excludes)

    def test_own_tag_at_rest_detected_by_motion(self):
        # The inserted spool's OWN tag rests on the antenna (in the baseline),
        # then blinks out for >= reappear_polls during the feed and returns —
        # motion brands it OURS, not a sister (the 01d0ec0f field case).
        coord = FakeCoordinator(
            fields=[["01d0ec0f"],                       # baseline (at rest)
                    ["01d0ec0f"],                        # still there
                    [], [], [], [],                      # gone 4 polls (moving)
                    ["01d0ec0f"]],                       # back in range
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        # It is the target, so it must NOT be excluded from the read.
        assert all("01d0ec0f" not in ex for ex in coord.read_excludes)


class TestUnreadableReposition:
    def _run(self):
        # Detect succeeds; the stationary reads at the stop position all fail
        # (2 retries), then the reposition read decodes.
        coord = FakeCoordinator(fields=[[], ["aabb"]],
                                full_reads=[None, None, TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_decodes_after_reposition(self):
        ams, coord, result = self._run()
        assert result is True
        assert len(coord.applied) == 1

    def test_reloads_the_lane(self):
        # One load for the scan feed + one for the reposition.
        ams, _, _ = self._run()
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2

    def test_unloads_twice(self):
        # Once before the reposition re-feed, once at the final unwind.
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 2

    def test_gives_up_cleanly_when_still_unreadable(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []
        assert ams._operation_active is False
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestSafetyGates:
    def test_lane_loaded_to_shared_toolhead_blocks_scan(self):
        # Some lane (any unit's) is loaded into the toolhead this unit's
        # lanes feed -> blocked.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        shared_ext = types.SimpleNamespace(name="extruder1",
                                           lane_loaded="lane9")
        ams.lanes["lane1"].extruder_obj = shared_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert coord.applied == []

    def test_unrelated_toolhead_does_not_block(self):
        # The unit's shared toolhead is free; other toolheads on a multi-tool
        # machine are irrelevant to this unit's scan.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        free_ext = types.SimpleNamespace(name="extruder1", lane_loaded=None)
        ams.lanes["lane1"].extruder_obj = free_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True

    def test_occupied_hub_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.hub_hes_value[2] = 1        # some other bay at the hub
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []

    def test_occupied_hub_does_not_block_scheduling(self):
        # Insert staging trips the hub HES briefly — scheduling must not be
        # blocked by it (the scan re-checks after the ready-wait).
        ams, _ = _build_unit()
        ams.oams.hub_hes_value[0] = 1
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_shared_toolhead_loaded_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams.lanes["lane1"].extruder_obj = types.SimpleNamespace(
            name="extruder1", lane_loaded="lane9")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_in_print_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.afc.function.in_print = lambda: True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []


class TestHubEngageCancel:
    def test_load_cancelled_when_hub_engages(self):
        # A load still in flight when the hub HES trips must be cancelled
        # (TD-1 style) and replaced with slow follower creep.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)

        def slow_load(args):
            # Load stays in flight; hub trips immediately.
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.encoder_clicks += 30
            ams.oams.hub_hes_value[args[0]] = 1
            ams.oams.action_status = OAMSStatus.LOADING

        ams.oams.oams_load_spool_cmd.send = slow_load
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert ams.oams.cancel_calls >= 1          # load cancelled at hub
        assert (1, 1) in ams.oams.follower_calls   # creep enabled


class TestScanTimeout:
    def _run(self):
        # Reader never sees a new tag (e.g. a spool without one).
        ams, coord = _build_unit()
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_false(self):
        _, _, result = self._run()
        assert result is False

    def test_does_not_apply(self):
        _, coord, _ = self._run()
        assert coord.applied == []

    def test_still_unwinds_and_cleans_up(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestBusyHandling:
    def test_one_retry_after_busy_rejection(self):
        # Firmware refuses the first load ERROR_BUSY (staging raced the ready
        # wait); the scan waits for ready again and sends exactly ONE more.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.busy_rejections = 1
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert len(coord.applied) == 1

    def test_aborts_after_second_busy_no_hammering(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(values={"rfid_scan_ready_timeout": 5.0},
                                 coord=coord)
        ams.oams.busy_rejections = 10_000
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        # Never more than two load attempts — we wait on ready, we don't spam.
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert coord.applied == []
        # Guard cleared even on the abort path; PTFE never touched.
        assert ams._operation_active is False

    def test_follower_left_stopped_after_busy_abort(self):
        # The pre-load dance enables the follower forward (mirroring
        # _oams_load), but an aborted scan must always leave it STOPPED.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 5.0})
        ams.oams.busy_rejections = 10_000
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestRefusedLoad:
    def test_instant_error_refusal_aborts_without_retry(self):
        # A load that instantly completes with a non-success, non-busy code
        # (e.g. "no spool in bay") is a REFUSAL — reported, not retried.
        ams, coord = _build_unit()

        def dead_send(args):
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.action_status = None
            ams.oams.action_status_code = 4    # NO_SPOOL_IN_BAY

        ams.oams.oams_load_spool_cmd.send = dead_send
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert len(ams.oams.oams_load_spool_cmd.sent) == 1
        # Nothing engaged, so no unwind noise.
        assert ams.oams.unload_calls == 0


class TestScheduling:
    def test_disabled_does_not_schedule(self):
        ams, _ = _build_unit(values={"rfid_scan_on_insert": False})
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_enabled_schedules_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_already_scanned_not_rescheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scanned.add("lane1")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_operation_active_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams._operation_active = True
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_cancel_clears_latch_and_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        ams._rfid_scanned.add("lane1")
        ams._cancel_rfid_scan("lane1")
        assert ams._rfid_scan_timers == {}
        assert "lane1" not in ams._rfid_scanned


class TestOperationActiveGuard:
    def test_scan_bails_if_operation_active(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams._operation_active = True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []


class TestUnitReadyWait:
    def test_waits_out_encoder_motion(self):
        ams, _ = _build_unit()
        ticks = {"n": 0}
        real_monotonic = ams.afc.reactor.monotonic
        # Simulate: the encoder advances during the first few poll pauses
        # (firmware auto-stage still feeding), then holds still.
        base = ams.oams.encoder_clicks

        def fake_pause(until):
            AdvancingReactor.pause(ams.afc.reactor, until)
            if ticks["n"] < 3:
                ams.oams.encoder_clicks = base + ticks["n"]
                ticks["n"] += 1

        ams.afc.reactor.pause = fake_pause
        start = real_monotonic()
        assert ams._rfid_wait_for_unit_ready(10.0, quiet_time=1.0) is True
        # It must have waited at least quiet_time past the last movement.
        assert real_monotonic() - start >= 1.0

    def test_probes_instead_of_waiting_for_an_unprompted_report(self):
        # The firmware answers with its motor state only when spoken to. An
        # idle unit that has said nothing since boot must still be declared
        # ready PROMPTLY -- waiting passively for a spontaneous STOPPED report
        # burned the whole ready timeout (30s of dead air per insert scan).
        ams, _ = _build_unit()
        reactor = ams.afc.reactor
        start = reactor.monotonic()
        assert ams.oams.motion_status is None
        assert ams._rfid_wait_for_unit_ready(30.0, fresh=True,
                                             quiet_time=1.0) is True
        assert reactor.monotonic() - start < 5.0
        # Readiness came from a probe, and the probe is a harmless stop.
        assert ams.oams.follower_calls
        assert set(ams.oams.follower_calls) == {(0, 0)}

    def test_stale_active_state_is_not_ready(self):
        # A unit mid-stage keeps answering "reverse following / busy". The old
        # code only counted a FRESH active report, so a stale one let the wait
        # return ready ~1s in and the very next command took an ERROR_BUSY.
        ams, _ = _build_unit()
        ams.oams.staging_probes = 10_000
        assert ams._rfid_wait_for_unit_ready(3.0, quiet_time=1.0) is False

    def test_ready_once_staging_finishes(self):
        ams, _ = _build_unit()
        ams.oams.staging_probes = 3
        assert ams._rfid_wait_for_unit_ready(30.0, fresh=True,
                                             quiet_time=1.0) is True

    def test_no_load_sent_while_the_unit_is_still_staging(self):
        # The scan must not even reach the load while the motor is owned by
        # the firmware's insert routine.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 3.0})
        ams.oams.staging_probes = 10_000
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert ams._operation_active is False

    def test_fresh_satisfied_by_new_stopped_report(self):
        ams, _ = _build_unit()
        reactor = ams.afc.reactor

        def report_stopped(until):
            AdvancingReactor.pause(reactor, until)
            # Firmware reports STOPPED shortly after the wait begins.
            ams.oams.motion_status = OAMSStatus.STOPPED
            ams.oams.motion_status_time = reactor.monotonic()

        # Only the first pause plants the report; later pauses advance time.
        calls = {"n": 0}

        def fake_pause(until):
            if calls["n"] == 0:
                report_stopped(until)
            else:
                AdvancingReactor.pause(reactor, until)
            calls["n"] += 1

        reactor.pause = fake_pause
        assert ams._rfid_wait_for_unit_ready(10.0, fresh=True,
                                             quiet_time=0.5) is True


class TestManualScanCommandRefusals:
    """AFC_OAMS_RFID_SCAN physically feeds the lane to rotate the spool past
    the reader. Every refusal below is a reason NOT to drive a motor, so they
    matter more than the happy path."""

    def _gcmd(self, **kw):
        g = MagicMock()
        g.get.side_effect = lambda k, d=None: kw.get(k, d)
        g.error = RuntimeError
        return g

    def test_missing_lane_argument(self):
        ams, _ = _build_unit()
        with pytest.raises(RuntimeError):
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd())

    def test_unknown_lane_names_the_unit(self):
        ams, _ = _build_unit()
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane9"))
        assert "lane9" in str(e.value)

    def test_refuses_while_another_operation_runs(self):
        ams, _ = _build_unit()
        ams._operation_active = True
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "busy" in str(e.value)

    def test_refuses_when_the_scan_is_blocked(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: "lane is tool-loaded"
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "tool-loaded" in str(e.value)

    def test_refuses_with_an_empty_bay(self):
        # Feeding an empty bay spins the motor against nothing.
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams.oams.is_bay_ready = lambda bay: False
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "no filament inserted" in str(e.value)

    def test_refuses_when_the_lane_maps_to_no_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams._spool_map = {}
        with pytest.raises(RuntimeError):
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))


class TestManualScanCommandReplies:
    def _gcmd(self, **kw):
        g = MagicMock()
        g.get.side_effect = lambda k, d=None: kw.get(k, d)
        g.error = RuntimeError
        g.said = []
        g.respond_info.side_effect = g.said.append
        return g

    def _ready(self):
        ams, coord = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams.oams.is_bay_ready = lambda bay: True
        return ams, coord

    def test_a_manual_scan_bypasses_the_once_per_insert_latch(self):
        # The latch exists to stop the AUTO scan repeating; a human asking
        # again plainly wants it to run.
        ams, _ = self._ready()
        ams._rfid_scanned.add("lane1")
        ams._do_rfid_scan = lambda lane: True
        ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "lane1" not in ams._rfid_scanned

    def test_a_decoded_tag_is_reported(self):
        ams, _ = self._ready()
        ams._do_rfid_scan = lambda lane: True
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("scanned and applied to lane1" in m for m in g.said)

    def test_no_tag_reports_the_undecoded_hint(self):
        # "a tag was seen but could not be decoded" is a different fault from
        # "no tag", and only the hint distinguishes them.
        ams, coord = self._ready()
        ams._do_rfid_scan = lambda lane: False
        coord.undecoded_hint = lambda name: " (tag seen, no key)"
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1 (tag seen, no key)" in m
                   for m in g.said)

    def test_a_throwing_hint_still_produces_a_reply(self):
        ams, coord = self._ready()
        ams._do_rfid_scan = lambda lane: False
        coord.undecoded_hint = lambda name: (_ for _ in ()).throw(
            RuntimeError("x"))
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1" in m for m in g.said)

    def test_no_coordinator_still_produces_a_reply(self):
        ams, _ = self._ready()
        ams._do_rfid_scan = lambda lane: False
        ams._rfid_coord = None
        ams._rfid_coordinator = lambda: None
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1" in m for m in g.said)


class TestScanKickoffTimer:
    """The auto-scan runs from a one-shot reactor timer: it must deregister
    itself and never let an exception reach the reactor."""

    def test_runs_once_and_never_reschedules(self):
        ams, _ = _build_unit()
        ams._rfid_scan_timers["lane1"] = object()
        calls = []
        ams._do_rfid_scan = lambda lane: calls.append(lane)
        got = ams._rfid_scan_kickoff(1.0, "lane1")
        assert got == ams.afc.reactor.NEVER
        assert len(calls) == 1
        assert "lane1" not in ams._rfid_scan_timers

    def test_a_lane_that_vanished_is_skipped(self):
        ams, _ = _build_unit()
        assert ams._rfid_scan_kickoff(1.0, "gone") == ams.afc.reactor.NEVER

    def test_a_failing_scan_is_logged_not_raised(self):
        ams, _ = _build_unit()
        ams._do_rfid_scan = lambda lane: (_ for _ in ()).throw(
            RuntimeError("boom"))
        assert ams._rfid_scan_kickoff(1.0, "lane1") == ams.afc.reactor.NEVER


class TestHubDebounce:
    """Hub state feeds the virtual hub and the 'hub clear' gate, so a
    fluttering switch reaching those checks would flap a load mid-flight.
    Live consumers read oams.hub_hes_value directly and see the raw signal."""

    def _lane(self):
        return types.SimpleNamespace(_load_state=None)

    def _ams(self, committed=None, debounce=0.5):
        ams, _ = _build_unit()
        ams._last_hub = [committed, None, None, None]
        ams._hub_pending_since = None
        ams.hub_debounce = debounce
        return ams

    def test_a_resync_pass_accepts_raw_immediately(self):
        ams, lane = self._ams(), self._lane()
        ams._update_hub_debounced(lane, 0, True, 100.0, True)
        assert lane._load_state is True and ams._last_hub[0] is True

    def test_the_first_reading_is_accepted_immediately(self):
        ams, lane = self._ams(), self._lane()
        ams._update_hub_debounced(lane, 0, True, 100.0, False)
        assert lane._load_state is True

    def test_an_unchanged_reading_clears_a_pending_change(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._hub_pending_since = [50.0, None, None, None]
        ams._update_hub_debounced(lane, 0, True, 100.0, False)
        assert lane._load_state is True
        assert ams._hub_pending_since[0] is None

    def test_a_change_does_not_commit_immediately(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        assert lane._load_state is True                 # still committed
        assert ams._hub_pending_since[0] == 100.0

    def test_a_change_that_does_not_hold_is_ignored(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, False, 100.2, False)
        assert lane._load_state is True                 # inside the window

    def test_a_change_that_holds_commits(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, False, 100.6, False)
        assert lane._load_state is False
        assert ams._last_hub[0] is False
        assert ams._hub_pending_since[0] is None

    def test_a_flutter_back_to_committed_restarts_the_window(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, True, 100.2, False)   # cancels
        ams._update_hub_debounced(lane, 0, False, 100.4, False)  # restarts
        assert lane._load_state is True


class TestScanHelpersSurviveAFailingCoordinator:
    """Every one of these is an except branch around a call into the RFID
    coordinator or the OAMS controller. The scan is mid-motion when they run,
    so an exception escaping here strands the follower on and leaves the
    operation guard set -- the lane would be stuck until a restart."""

    def test_slot_falls_back_to_the_bay_index(self):
        # The coordinator's lane_slot_map wins; without one (or with a
        # coordinator that throws) the bay index is the sane default.
        ams, coord = _build_unit()
        coord._get_slot = lambda name: (_ for _ in ()).throw(KeyError(name))
        assert ams._rfid_scan_slot(coord, ams.lanes["lane1"]) == 0

    def test_slot_is_none_when_nothing_maps(self):
        ams, coord = _build_unit()
        coord._get_slot = lambda name: None
        ams._spool_map = {}
        assert ams._rfid_scan_slot(coord, ams.lanes["lane1"]) is None

    def test_a_probe_error_is_treated_as_an_empty_field(self):
        # scan_slot_uids talks to the reader while the spool turns; a transient
        # SPI error must not abort the scan, just yield no uids this pass.
        ams, coord = _build_unit()
        coord.scan_slot_uids = lambda slot: (_ for _ in ()).throw(
            RuntimeError("spi"))
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._do_rfid_scan(ams.lanes["lane1"])       # must not raise

    def test_a_stationary_read_error_yields_no_tag(self):
        ams, coord = _build_unit()
        coord.read_slot_excluding = lambda slot, exclude: (_ for _ in ()).throw(
            RuntimeError("auth failed"))
        got = ams._rfid_read_stationary(coord, 0, set(), attempts=2)
        assert got is None


class TestBlockedReason:
    """The scan refuses whenever moving filament could collide with something
    else. Each branch names WHAT is in the way, because 'blocked' alone tells
    an operator nothing."""

    def test_a_lane_loaded_to_the_toolhead_blocks_and_names_it(self):
        ams, _ = _build_unit()
        ext = types.SimpleNamespace(lane_loaded="lane1", name="extruder")
        ams.lanes["lane1"].extruder_obj = ext
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert "loaded to this unit's toolhead" in reason
        assert "extruder" in reason

    def test_hub_filament_blocks_and_names_the_bay(self):
        ams, _ = _build_unit()
        ams.oams.hub_hes_value = [0, 1, 0, 0]
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert "hub sensor shows filament (bay 1)" == reason

    def test_an_unreadable_hub_value_does_not_block(self):
        # A controller that cannot report the hub must not veto the scan; the
        # other guards still apply.
        ams, _ = _build_unit()
        type(ams.oams).hub_hes_value = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no link")))
        try:
            assert ams._rfid_scan_blocked_reason(ams.lanes["lane1"]) is None
        finally:
            del type(ams.oams).hub_hes_value

    def test_hub_check_can_be_skipped(self):
        ams, _ = _build_unit()
        ams.oams.hub_hes_value = [1, 0, 0, 0]
        assert ams._rfid_scan_blocked_reason(
            ams.lanes["lane1"], check_hub=False) is None

    def test_printing_blocks_the_scan(self):
        ams, _ = _build_unit()
        ams.afc.function.in_print = lambda: True
        assert ams._rfid_scan_blocked_reason(
            ams.lanes["lane1"]) == "printer is printing"

    def test_an_unreadable_print_state_does_not_block(self):
        ams, _ = _build_unit()
        ams.afc.function.in_print = lambda: (_ for _ in ()).throw(
            RuntimeError("no afc"))
        assert ams._rfid_scan_blocked_reason(ams.lanes["lane1"]) is None


class TestFollowerControlFallback:
    """_rfid_set_follower prefers the shared follower object and falls back to
    driving the controller directly. Both tiers are wrapped because losing the
    follower mid-scan leaves the spool turning."""

    def test_the_follower_object_is_used_when_it_works(self):
        ams, _ = _build_unit()
        calls = []
        ams._follower = types.SimpleNamespace(
            enable_follower=lambda *a, **k: calls.append("enable"),
            set_follower_state=lambda *a, **k: calls.append("stop"))
        ams._get_monitor_state = lambda: None
        ams._rfid_set_follower(1, 1, "test")
        ams._rfid_set_follower(0, 0, "test")
        assert calls == ["enable", "stop"]

    def test_a_failing_follower_object_falls_back_to_the_controller(self):
        ams, _ = _build_unit()
        ams._follower = types.SimpleNamespace(
            enable_follower=lambda *a, **k: (_ for _ in ()).throw(
                RuntimeError("no monitor")),
            set_follower_state=lambda *a, **k: (_ for _ in ()).throw(
                RuntimeError("no monitor")))
        ams._get_monitor_state = lambda: None
        ams._rfid_set_follower(1, 1, "test")
        assert ams.oams.follower_calls, "should have driven the controller"

    def test_both_tiers_failing_is_survived(self):
        # Nothing left to try; the caller's unwind must still run.
        ams, _ = _build_unit()
        ams._follower = None
        ams.oams.set_oams_follower = lambda e, d: (_ for _ in ()).throw(
            RuntimeError("link down"))
        ams._rfid_set_follower(1, 1, "test")        # must not raise


class TestScanEntryGuards:
    """_do_rfid_scan refuses before it moves anything. Each guard is a
    different way the world can have changed between the insert edge that
    scheduled the scan and the timer that runs it."""

    def test_no_coordinator_configured(self):
        ams, _ = _build_unit()
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_no_controller(self):
        ams, _ = _build_unit()
        ams.oams = None
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_another_operation_started_since_scheduling(self):
        # A real load/unload may have begun between the insert edge and the
        # timer firing; never drive filament on top of it.
        ams, _ = _build_unit()
        ams._operation_active = True
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_a_blocked_scan_is_reported_and_refused(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: "printing"
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_an_empty_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: False
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_an_unmapped_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._spool_map = {}
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_no_rfid_slot_warns_and_refuses(self):
        # Mapped to a bay but the coordinator serves no reader for it: a
        # config mistake worth naming rather than a silent no-op.
        ams, coord = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: True
        ams._rfid_scan_slot = lambda c, lane: None
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False


class TestCoordinatorResolution:
    def test_it_is_looked_up_once_and_cached(self):
        ams, coord = _build_unit()
        ams._rfid_coord = None
        calls = []
        real = ams.printer.lookup_object

        def counting(name, default=None):
            calls.append(name)
            return real(name, default)
        ams.printer.lookup_object = counting
        assert ams._rfid_coordinator() is coord
        assert ams._rfid_coordinator() is coord
        assert calls.count("AFC_OpenAMS_rfid") == 1

    def test_a_missing_coordinator_resolves_to_none(self):
        ams, _ = _build_unit()
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        assert ams._rfid_coordinator() is None


class TestBlockedReasonToolLoaded:
    def test_the_lane_itself_being_tool_loaded_blocks(self):
        ams, _ = _build_unit()
        ams.lanes["lane1"].tool_loaded = True
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert reason == "lane1 is loaded to the toolhead"


class TestScheduleGuards:
    """The insert hook schedules the scan on a timer. It must refuse quietly
    for the same reasons, and never schedule work that will just refuse."""

    def test_a_blocked_lane_is_not_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: "printing"
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_no_coordinator_means_nothing_is_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_an_empty_bay_is_not_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: False
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}


class TestEncoderDeltaIsAlwaysSafe:
    """_enc_delta is called inside every motion wait. A controller that
    cannot report the encoder must yield 0, not raise -- the callers treat
    'no movement' as a timeout, which is recoverable."""

    def test_a_readable_encoder(self):
        ams, _ = _build_unit()
        assert ams._enc_delta(150, 100) == 50

    def test_unreadable_values_are_zero(self):
        ams, _ = _build_unit()
        assert ams._enc_delta(None, 100) == 0
        assert ams._enc_delta("x", 100) == 0


class TestCancelScanTimer:
    def test_cancelling_with_no_timer_is_a_noop(self):
        ams, _ = _build_unit()
        ams._cancel_rfid_scan("lane1")

    def test_a_failing_unregister_is_survived(self):
        ams, _ = _build_unit()
        ams._rfid_scan_timers["lane1"] = object()
        ams.afc.reactor.unregister_timer = lambda h: (_ for _ in ()).throw(
            RuntimeError("gone"))
        ams._cancel_rfid_scan("lane1")
        assert "lane1" not in ams._rfid_scan_timers


class TestRepositionAndSweep:
    """Recovery when a tag ANSWERED during the feed but would not decode where
    the feed stopped. The stop overshoots the antenna's sweet spot, so this
    unloads, re-feeds to just short of the remembered detect position, then
    creeps across it reading at each step.

    This is the machinery tuned on hardware (sweep_back 240 / step 25 /
    past 200), and every bound in it exists to stop a failed read becoming an
    endless motion loop."""

    def _ready(self, **kw):
        ams, coord = _build_unit(values=kw or None)
        ams._rfid_send_load = lambda lane, slot: True
        ams._wait_for_idle = lambda *a, **k: None
        ams._rfid_scan_stop_load = lambda: None
        ams.oams.action_status = None
        return ams, coord

    def test_a_failed_unload_aborts_without_feeding(self):
        # If the unload did not happen the filament is not where the re-feed
        # assumes; feeding anyway would drive past the reader entirely.
        ams, coord = self._ready()
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("busy"))
        fed = []
        ams._rfid_send_load = lambda lane, slot: fed.append(1) or True
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is None
        assert fed == []

    def test_a_refused_reload_aborts(self):
        ams, coord = self._ready()
        ams._rfid_send_load = lambda lane, slot: False
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is None

    def test_an_unreadable_encoder_at_the_start_is_treated_as_zero(self):
        ams, coord = self._ready()
        type(ams.oams).encoder_clicks = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no encoder")))
        try:
            got = ams._rfid_reposition_and_read(
                coord, ams.lanes["lane1"], 0, 0, set(), 500)
        finally:
            del type(ams.oams).encoder_clicks
        assert got is None

    def test_a_decode_on_the_first_sweep_read_returns_it(self):
        ams, coord = self._ready()
        tag = {"uid": "AABB", "filament": {"material": "PLA"}}
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: tag
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is tag

    def test_it_gives_up_once_past_the_window(self):
        # pos beyond detect_delta + sweep_past means the sweet spot is behind
        # us; continuing would just unspool filament.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 100000
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 100) is None

    def test_an_unreadable_encoder_mid_sweep_ends_the_sweep(self):
        # Treated as "past the window": without a position there is no way to
        # know when to stop, and creeping blind is worse than giving up.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        calls = {"n": 0}

        def clicks(self):
            calls["n"] += 1
            if calls["n"] > 2:
                raise RuntimeError("encoder lost")
            return 0
        type(ams.oams).encoder_clicks = property(clicks)
        try:
            got = ams._rfid_reposition_and_read(
                coord, ams.lanes["lane1"], 0, 0, set(), 100)
        finally:
            del type(ams.oams).encoder_clicks
        assert got is None

    def test_the_sweep_is_step_bounded_so_a_stalled_encoder_cannot_loop(self):
        # With the encoder frozen at 0 the position never advances; only the
        # step bound ends this.
        ams, coord = self._ready(rfid_scan_sweep_back=50,
                                 rfid_scan_sweep_step=25,
                                 rfid_scan_sweep_past=50)
        reads = []
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: reads.append(1)
        ams.oams.encoder_clicks = 0
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 0) is None
        # max_steps = 2 + (50 + 50) // 25 = 6
        assert len(reads) == 6

    def test_a_load_that_finishes_short_creeps_with_the_follower(self):
        # Short-PTFE override: the load completes before reaching the target,
        # so the follower has to cover the rest or the sweep starts too early.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 0
        ams.oams.action_status = None          # load already finished
        reasons = []
        ams._rfid_set_follower = lambda e, d, reason: reasons.append(reason)
        # detect_delta must exceed sweep_back (240) or target_early is 0 and
        # the re-feed wait breaks immediately without ever needing the creep.
        ams._rfid_reposition_and_read(coord, ams.lanes["lane1"], 0, 0,
                                      set(), 500)
        assert any("creep" in r for r in reasons), reasons

    def test_a_still_running_load_is_cancelled_before_the_sweep(self):
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 100000        # target reached at once
        ams.oams.action_status = "loading"      # ...but the load is still live
        stopped = []
        ams._rfid_scan_stop_load = lambda: stopped.append(1)
        ams._rfid_reposition_and_read(coord, ams.lanes["lane1"], 0, 0,
                                      set(), 100)
        assert stopped, "an in-flight load must be cancelled before sweeping"


class TestStopLoadWaitsForTheFirmware:
    """Cancelling a scan feed must WAIT for the firmware to acknowledge, or the
    unload that follows is rejected as busy -- which is how a scan used to
    leave the lane wedged."""

    def test_a_cancel_that_throws_is_survived(self):
        ams, _ = _build_unit()
        ams.oams.action_status = None
        ams.oams.load_spool_cancel = lambda: (_ for _ in ()).throw(
            RuntimeError("not loading"))
        ams._rfid_scan_stop_load()

    def test_it_waits_until_the_firmware_reports_idle(self):
        # The fake's load_spool_cancel acks immediately, which real firmware
        # does not always do -- override it so the wait loop actually runs.
        ams, _ = _build_unit()
        ams.oams.load_spool_cancel = lambda: None
        ams.oams.action_status = "loading"
        orig = ams.afc.reactor.pause

        def pause(until):
            ams.oams.action_status = None      # firmware acks on the next tick
            return orig(until)
        ams.afc.reactor.pause = pause
        ams._rfid_scan_stop_load()
        assert ams.oams.action_status is None

    def test_a_firmware_that_never_acks_is_forced_clear_after_the_deadline(self):
        # Otherwise every later operation is refused as busy forever.
        ams, _ = _build_unit()
        ams.oams.load_spool_cancel = lambda: None   # firmware ignores the cancel
        ams.oams.action_status = "loading"          # ...and never clears it
        ams._rfid_scan_stop_load()
        assert ams.oams.action_status is None       # forced clear at the deadline


class TestReadyWaitReadsSilenceAsReady:
    """The OAMS firmware only reports on a refusal or a state change, so
    SILENCE is the ready signal. The wait probes with an idempotent command
    and treats any answer -- or any encoder movement -- as 'still busy'."""

    def test_an_unreadable_encoder_at_the_start_seeds_zero(self):
        ams, _ = _build_unit()
        type(ams.oams).encoder_clicks = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no encoder")))
        try:
            ams._rfid_wait_for_unit_ready(timeout=0.5, quiet_time=0.1)
        finally:
            del type(ams.oams).encoder_clicks

    def test_a_probe_that_throws_does_not_end_the_wait(self):
        # The probe is a nicety; a controller that refuses it is still being
        # watched via its status and encoder.
        ams, _ = _build_unit()
        ams.oams.set_oams_follower = lambda e, d: (_ for _ in ()).throw(
            RuntimeError("busy"))
        assert ams._rfid_wait_for_unit_ready(
            timeout=0.5, quiet_time=0.1) in (True, False)

    def test_an_encoder_that_becomes_unreadable_holds_the_last_value(self):
        # Treating an unreadable encoder as "moved" would reset the quiet
        # window forever and the wait could never succeed.
        ams, _ = _build_unit()
        state = {"n": 0}

        def clicks(self):
            state["n"] += 1
            if state["n"] > 2:
                raise RuntimeError("encoder lost")
            return 0
        type(ams.oams).encoder_clicks = property(clicks)
        try:
            assert ams._rfid_wait_for_unit_ready(
                timeout=0.6, quiet_time=0.1) is True
        finally:
            del type(ams.oams).encoder_clicks


class TestUndecodedTagIsNamedAsAKeyProblem:
    """A tag that ANSWERS but will not decode is a key/format problem, not a
    positioning one. Saying so is the difference between 'check your keys' and
    an operator moving the antenna around for an hour."""

    def test_an_answering_but_undecodable_tag_is_reported(self):
        ams, coord = _build_unit()
        said = []
        ams.logger.info = lambda m, *a, **k: said.append(m)
        coord.read_slot_excluding = lambda slot, exclude: {
            "uid": "AABB", "tag_type": "MifareClassic1k"}      # no filament
        assert ams._rfid_read_stationary(coord, 0, set(), attempts=2) is None
        assert any("won't decode" in m and "AABB" in m for m in said)
        assert any("AFC_rfid_keys" in m for m in said)

    def test_a_silent_antenna_says_nothing_about_keys(self):
        ams, coord = _build_unit()
        said = []
        ams.logger.info = lambda m, *a, **k: said.append(m)
        coord.read_slot_excluding = lambda slot, exclude: None
        assert ams._rfid_read_stationary(coord, 0, set(), attempts=2) is None
        assert not any("won't decode" in m for m in said)


class _FlakyEncoder:
    """Controller wrapper whose encoder and/or hub reads raise.

    Every read of those two is wrapped in the scan because they are polled
    continuously while filament moves; a controller that goes briefly
    unreadable must degrade the scan, not abort it mid-motion with the
    follower still on.
    """

    def __init__(self, inner, encoder=False, hub=False):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "_bad_enc", encoder)
        object.__setattr__(self, "_bad_hub", hub)

    def __getattr__(self, name):
        if name == "encoder_clicks" and self._bad_enc:
            raise RuntimeError("encoder unreadable")
        if name == "hub_hes_value" and self._bad_hub:
            raise RuntimeError("hub unreadable")
        return getattr(self._inner, name)

    def __setattr__(self, name, value):
        setattr(self._inner, name, value)


class TestScanSurvivesADegradedController:
    """A full scan with the encoder and/or hub unreadable. These are the
    fallbacks that keep a scan recoverable rather than leaving the lane
    mid-feed: the scan is already moving filament when they fire."""

    def _run(self, **flags):
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams = _FlakyEncoder(ams.oams, **flags)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_an_unreadable_encoder_throughout_still_completes(self):
        # clicks_start, the engage check and the detect position all fall back;
        # the tag is still found and applied because detection is by UID.
        ams, coord, result = self._run(encoder=True)
        assert result is True
        assert len(coord.applied) == 1

    def test_an_unreadable_hub_falls_back_to_not_engaged(self):
        ams, coord, result = self._run(hub=True)
        assert result is True

    def test_both_unreadable_still_completes(self):
        ams, coord, result = self._run(encoder=True, hub=True)
        assert result is True


class TestScanUnwindFailures:
    """The unwind runs in the finally: it is what puts the filament back after
    a scan. If it throws, the failure must be reported and the operation guard
    still cleared -- otherwise the lane is locked out until a restart."""

    def _coord(self):
        return FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])

    def test_a_failing_unload_is_reported_and_the_guard_is_cleared(self):
        ams, coord = _build_unit(coord=self._coord())
        warned = []
        ams.logger.warning = lambda m, *a, **k: warned.append(m)
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("unload refused"))
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams._operation_active is False
        assert any("unwind" in m or "unload" in m for m in warned), warned

    # NB: there is deliberately no test for the finally's follower stop
    # throwing. _rfid_set_follower cannot raise -- both the follower-object
    # tier and the direct-controller tier are internally wrapped (see
    # TestFollowerControlFallback::test_both_tiers_failing_is_survived) -- so
    # forcing it would only exercise a path the code cannot reach.


class TestF1sDebounceCommitted:
    """The f1s (bay presence) debounce mirrors the hub one: a change commits to
    prep_state only after holding, so a bouncing insert switch cannot fire the
    insert edge -- and the insert edge is what schedules an RFID scan."""

    def _lane(self):
        return types.SimpleNamespace(prep_state=None)

    def _ams(self, committed):
        ams, _ = _build_unit()
        ams._last_f1s = [committed, None, None, None]
        ams._f1s_pending_since = [None, None, None, None]
        ams.f1s_debounce = 0.5
        return ams

    def test_an_unchanged_reading_keeps_the_committed_value(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, True, 100.0, False)
        assert lane.prep_state is True
        assert ams._f1s_pending_since[0] is None

    def test_a_change_does_not_commit_immediately(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.0, False)
        assert lane.prep_state is True
        assert ams._f1s_pending_since[0] == 100.0

    def test_a_change_inside_the_window_is_ignored(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.0, False)
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.2, False)
        assert lane.prep_state is True


class TestScanWithAFirmwareThatKeepsLoading:
    """The firmware does not always report the load finished. The scan has to
    cancel it explicitly at both exit points -- on a detection and on a timeout
    -- or the unload that follows is refused as busy and the lane is stuck."""

    def _ams(self, fields, reads):
        coord = FakeCoordinator(fields=fields, full_reads=reads)
        ams, coord = _build_unit(coord=coord)
        # Firmware that moves filament but never acks the load. NB: the fake
        # binds oams_load_spool_cmd.send at construction, so the bound command
        # is what has to be replaced -- not _send_load.
        def never_acks(args):
            ams.oams.encoder_clicks += 120
            ams.oams.hub_hes_value[args[0]] = 1
            # action_status deliberately left as LOADING
        ams.oams.oams_load_spool_cmd.send = never_acks
        # NB: do NOT preset hub_hes_value -- the scan re-checks the hub before
        # loading and a pre-tripped bay reads as "hub sensor shows filament",
        # refusing the scan. never_acks trips it during the load instead,
        # which is what makes `engaged` true and the unwind run.
        stops = []
        real_stop = ams._rfid_scan_stop_load

        def stop():
            stops.append(1)
            ams.oams.action_status = None
            return real_stop()
        ams._rfid_scan_stop_load = stop
        return ams, coord, stops

    def test_the_load_is_cancelled_when_a_tag_is_detected(self):
        ams, coord, stops = self._ams([[], [], ["aabb"]], [TAG])
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert stops, "a still-running load must be cancelled at detection"

    def test_the_load_is_cancelled_on_timeout_too(self):
        # No tag ever appears; the feed must still be stopped.
        ams, coord, stops = self._ams([[]], [])
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert stops, "a still-running load must be cancelled on timeout"

    def test_a_failing_unwind_is_reported(self):
        ams, coord, _ = self._ams([[], [], ["aabb"]], [TAG])
        warned = []
        ams.logger.warning = lambda m, *a, **k: warned.append(m)
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("bay blocked"))
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert any("unwind failed" in m for m in warned), warned
        assert ams._operation_active is False

    def test_a_failing_clear_errors_is_swallowed(self):
        # Cosmetic tidy-up at the end of a scan; it must not turn a successful
        # read into a failure.
        ams, coord, _ = self._ams([[], [], ["aabb"]], [TAG])
        ams.oams.clear_errors = lambda: (_ for _ in ()).throw(
            RuntimeError("no link"))
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is True

    def test_an_unreadable_hub_during_the_cancel_decision(self):
        # The cancel decision reads the hub to see whether the feed engaged;
        # unreadable means "not engaged", which just defers to the encoder cap.
        coord = FakeCoordinator(fields=[[]], full_reads=[])
        ams, coord = _build_unit(coord=coord)
        def never_acks(args):
            ams.oams.encoder_clicks += 120
        ams.oams.oams_load_spool_cmd.send = never_acks
        ams.oams = _FlakyEncoder(ams.oams, hub=True)
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams._operation_active is False
