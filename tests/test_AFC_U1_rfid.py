"""
Tests for the Snapmaker U1 RFID path, extras/AFC_U1_rfid.py.

The scanner that takes decoded tags from the U1's own daemon over a webhook,
and a branch-coverage sweep over the module around it.
Consolidated from two files; banners name the file each block came from.
"""

from __future__ import annotations
from unittest.mock import patch
import pytest
from configfile import error as ConfigError
from extras.AFC_U1_rfid import (
    AFC_U1_RFID, load_config, POLL_INTERVAL, _BACKOFF_INTERVAL,
    _MAX_CONSECUTIVE_FAILURES, _BACKOFF_RESET_CYCLES,
)
from tests.conftest import MockAFC, MockConfig, MockGcode, MockLogger, MockPrinter
from extras.AFC_U1_rfid import AFC_U1_RFID


# ── Branch-coverage unit tests for extras/AFC_U1_rfid.py ──────────────────────
#
# was tests/test_AFC_U1_rfid_coverage.py
# ── Shared fakes ──────────────────────────────────────────────────────────────

class _Recorder_rfid_coverage:
    def __init__(self, result=None, raises=None):
        self.calls = []
        self.result = result
        self.raises = raises

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.raises is not None:
            raise self.raises
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0

    @property
    def call_count(self):
        return len(self.calls)

    @property
    def last(self):
        return self.calls[-1]


class _Webhooks:
    def __init__(self, raises=None):
        self.raises = raises
        self.registered = []

    def register_endpoint(self, path, cb):
        if self.raises is not None:
            raise self.raises
        self.registered.append((path, cb))


class _Ext:
    def __init__(self, name=None, th_name=None, lanes=None,
                 auto_spoolman_create=False):
        self.name = name
        self.th_extruder_name = th_name
        self.lanes = lanes if lanes is not None else {}
        self.auto_spoolman_create = auto_spoolman_create


class _Lane:
    def __init__(self, name="lane1", extruder_obj=None, spool_scanner=False,
                 status="", spool_id=None, tool_loaded=False):
        self.name = name
        self.extruder_obj = extruder_obj
        self.unit_obj = None
        self.spool_scanner = spool_scanner
        self.status = status
        self.spool_id = spool_id
        self.tool_loaded = tool_loaded
        self.material = "orig-mat"
        self.color = "orig-col"
        self.send_lane_data = _Recorder_rfid_coverage()


class _LookupPrinter:
    """Minimal printer whose lookup_object is fully controllable."""

    def __init__(self, objects=None):
        self._objects = objects or {}
        self._event_handlers = {}

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def register_event_handler(self, event, cb):
        self._event_handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        for cb in self._event_handlers.get(event, []):
            cb(*args)


class _AdvancingReactor:
    NEVER = 9_999_999_999.0

    def __init__(self, times):
        self._times = list(times)
        self._i = 0
        self.register_callback = _Recorder_rfid_coverage()

    def monotonic(self):
        t = self._times[min(self._i, len(self._times) - 1)]
        self._i += 1
        return t

    def pause(self, until):
        return until


class _WebReq:
    def __init__(self, data=None, int_data=None, channel_raises=False):
        self._data = data or {}
        self._int = int_data or {}
        self._channel_raises = channel_raises

    def get_int(self, key, default=None):
        if key == "channel" and self._channel_raises:
            raise ValueError("no channel")
        if key in self._int:
            return self._int[key]
        if key in self._data:
            return int(self._data[key])
        return default

    def get(self, key, default=None):
        return self._data.get(key, default)


def _build(values=None, afc=None, objects=None, webhooks=None):
    """Construct a reader through the real __init__ with a MockLogger.

    :param values: config option overrides.
    :param afc: pre-built MockAFC (else a fresh one).
    :param objects: extra printer lookup_object entries.
    :param webhooks: webhooks object to inject (else default MagicMock).
    :return AFC_U1_RFID: the constructed reader (logger is a MockLogger).
    """
    afc = afc if afc is not None else MockAFC()
    printer = MockPrinter(afc=afc)
    if objects:
        printer._objects.update(objects)
    if webhooks is not None:
        printer._objects["webhooks"] = webhooks
    config = MockConfig(printer=printer, values=values or {})
    with patch("extras.AFC_U1_rfid.logging.getLogger",
               return_value=MockLogger()):
        reader = AFC_U1_RFID(config)
    return reader


# ── __init__ ──────────────────────────────────────────────────────────────────

class TestAFCU1RFIDInit:
    def test_defaults_empty_config(self):
        reader = _build()
        assert reader._cfg_channels == {}
        assert reader._cfg_scanner_channels == set()
        assert reader._scanner_auto_create is False
        assert reader._scanner_confirm_reads == 1
        assert reader._lane_auto_create is False
        assert reader._webhook_grace == 0.0
        assert reader.afc is None
        assert reader.logger.messages == []

    def test_lane_channels_parsed_with_blank_skipped(self):
        reader = _build(values={"lane_channels": "lane4:1, lane5:2, "})
        assert reader._cfg_channels == {"lane4": 1, "lane5": 2}

    def test_channels_alias_used_when_lane_channels_absent(self):
        reader = _build(values={"channels": "lane6:3"})
        assert reader._cfg_channels == {"lane6": 3}

    def test_lane_channels_missing_separator_raises(self):
        with pytest.raises(ConfigError):
            _build(values={"lane_channels": "lane4"})

    def test_lane_channels_bad_number_raises(self):
        with pytest.raises(ConfigError):
            _build(values={"lane_channels": "lane4:x"})

    def test_scanner_channels_parsed_with_blank_skipped(self):
        reader = _build(values={"scanner_channels": "0, 2, "})
        assert reader._cfg_scanner_channels == {0, 2}

    def test_scanner_channels_bad_value_raises(self):
        with pytest.raises(ConfigError):
            _build(values={"scanner_channels": "x"})

    def test_scanner_lanes_and_flags(self):
        reader = _build(values={
            "scanner_lanes": "lane1, lane2",
            "scanner_auto_create": True,
            "auto_spoolman_create": True,
            "scanner_confirm_reads": 3,
            "webhook_grace": 1.5,
        })
        assert reader._cfg_scanners == {"lane1", "lane2"}
        assert reader._scanner_auto_create is True
        assert reader._lane_auto_create is True
        assert reader._scanner_confirm_reads == 3
        assert reader._webhook_grace == 1.5

    def test_webhook_endpoint_registered(self):
        wh = _Webhooks()
        reader = _build(webhooks=wh)
        assert wh.registered == [("afc/u1_rfid", reader._handle_webhook_scan)]
        assert reader.logger.messages == []

    def test_webhook_registration_failure_logs_warning(self):
        wh = _Webhooks(raises=ValueError("boom"))
        reader = _build(webhooks=wh)
        assert wh.registered == []
        assert reader.logger.messages == [
            ("warning", "AFC_U1_rfid: failed to register webhook endpoint: boom")]


# ── _handle_ready ─────────────────────────────────────────────────────────────

class TestAFCU1RFIDHandleReady:
    def _prep(self, reader):
        reader.start = _Recorder_rfid_coverage()
        reader._patch_scanner_rfid_update = _Recorder_rfid_coverage()
        reader.register_lane = _Recorder_rfid_coverage()

    def test_afc_not_loaded_disables_reader(self):
        reader = _build()
        reader.printer._afc = None
        self._prep(reader)
        reader.logger = MockLogger()
        reader._handle_ready()
        assert reader.afc is None
        assert reader.start.called is False
        assert reader.logger.messages == [
            ("warning", "AFC_U1_rfid: AFC not loaded; reader disabled")]

    def test_single_lane_registers_and_starts(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)
        lane = _Lane(name="lane1")
        reader._cfg_channels = {"lane1": 1}
        reader._resolve_lane = lambda name: (lane, None)
        reader._handle_ready()
        assert reader.afc is afc
        assert reader.register_lane.last == ((lane, 1), {})
        assert reader.start.called is True
        assert reader._patch_scanner_rfid_update.called is True
        assert afc.logger.messages == []

    def test_combined_extruder_becomes_scanner_channel(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)
        ext = _Ext(name="e0", lanes={"a": 1, "b": 2})
        reader._cfg_channels = {"e0": 4}
        reader._resolve_lane = lambda name: (None, ext)
        reader._handle_ready()
        assert 4 in reader._cfg_scanner_channels
        assert reader.register_lane.called is False
        assert afc.logger.messages == [(
            "info",
            "U1 RFID: 'e0' is a combined extruder (2 lanes) — ch4 acts as a "
            "spool scanner (stages next spool)")]

    def test_unresolved_lane_warns(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)
        reader._cfg_channels = {"ghost": 5}
        reader._resolve_lane = lambda name: (None, None)
        reader._handle_ready()
        assert reader.register_lane.called is False
        assert afc.logger.messages == [(
            "warning",
            "U1 RFID: configured lane 'ghost' not found in AFC (neither a lane "
            "name nor a single-lane extruder)")]

    def test_scanner_lane_flag_set_on_lane(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)
        lane = _Lane(name="lane1", spool_scanner=False)
        reader._cfg_channels = {"lane1": 1}
        reader._cfg_scanners = {"lane1"}
        reader._resolve_lane = lambda name: (lane, None)
        reader._handle_ready()
        assert lane.spool_scanner is True
        assert reader.register_lane.last == ((lane, 1), {})

    def test_scanner_flag_set_error_is_swallowed(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)

        class _RaisingLane:
            name = "lane1"

            @property
            def spool_scanner(self):
                return False

            @spool_scanner.setter
            def spool_scanner(self, value):
                raise RuntimeError("read-only")

        lane = _RaisingLane()
        reader._cfg_channels = {"lane1": 1}
        reader._cfg_scanners = {"lane1"}
        reader._resolve_lane = lambda name: (lane, None)
        reader._handle_ready()  # must not raise
        assert reader.register_lane.last == ((lane, 1), {})

    def test_standalone_scanner_channels_registered(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        self._prep(reader)
        reader._cfg_scanner_channels = {7}
        reader._handle_ready()
        assert reader._channel_to_lane[7] is None
        assert reader._last_uid[7] is None
        assert reader._consecutive_failures[7] == 0
        assert reader.start.called is True


# ── _patch_scanner_rfid_update ────────────────────────────────────────────────

class _FakeFD:
    def __init__(self, cb_list=None):
        if cb_list is not None:
            self._notify_data_update_cb = cb_list


class _FakePTC:
    def __init__(self, cb=None):
        if cb is not None:
            self._rfid_filament_info_update_cb = cb


class TestAFCU1RFIDPatchScannerRfidUpdate:
    def _reader(self, ptc, fd, scanner_channels):
        reader = _build()
        reader.logger = MockLogger()
        reader._cfg_scanner_channels = scanner_channels
        objs = {}
        if ptc is not None:
            objs["print_task_config"] = ptc
        if fd is not None:
            objs["filament_detect"] = fd
        reader.printer = _LookupPrinter(objs)
        return reader

    def test_noop_when_ptc_missing(self):
        fd = _FakeFD(cb_list=[])
        reader = self._reader(None, fd, {0})
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == []

    def test_noop_when_fd_missing(self):
        ptc = _FakePTC(cb=lambda *a: None)
        reader = self._reader(ptc, None, {0})
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == []

    def test_noop_when_fd_lacks_notify_attr(self):
        ptc = _FakePTC(cb=lambda *a: None)
        fd = _FakeFD(cb_list=None)  # no _notify_data_update_cb
        reader = self._reader(ptc, fd, {0})
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == []

    def test_noop_when_original_cb_missing(self):
        original = None
        ptc = _FakePTC(cb=original)  # attr absent -> getattr returns None
        fd = _FakeFD(cb_list=[])
        reader = self._reader(ptc, fd, {0})
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == []

    def test_noop_when_no_scanner_channels(self):
        original = _Recorder_rfid_coverage()
        ptc = _FakePTC(cb=original)
        fd = _FakeFD(cb_list=[original])
        reader = self._reader(ptc, fd, set())
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == []
        assert fd._notify_data_update_cb == [original]

    def test_patches_callback_and_suppresses_scanner_channel(self):
        original = _Recorder_rfid_coverage()
        ptc = _FakePTC(cb=original)
        fd = _FakeFD(cb_list=[original])
        reader = self._reader(ptc, fd, {0})
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb[0] is not original
        assert reader.logger.messages == [(
            "info",
            "U1 RFID: protecting scanner channels [0] from U1 display "
            "overwrite")]
        patched = fd._notify_data_update_cb[0]
        patched(0, {"x": 1}, False)          # scanner channel: suppressed
        assert original.called is False
        patched(3, {"y": 2}, True)           # other channel: passes through
        assert original.last == ((3, {"y": 2}, True), {})

    def test_warns_when_callback_not_found(self):
        original = _Recorder_rfid_coverage()
        other = _Recorder_rfid_coverage()
        ptc = _FakePTC(cb=original)
        fd = _FakeFD(cb_list=[other])  # original not present
        reader = self._reader(ptc, fd, {0})
        reader._patch_scanner_rfid_update()
        assert reader.logger.messages == [(
            "warning",
            "U1 RFID: could not locate print_task_config RFID callback to "
            "patch")]


# ── _resolve_lane ─────────────────────────────────────────────────────────────

class TestAFCU1RFIDResolveLane:
    def _reader(self, lanes=None, tools=None):
        reader = _build()
        reader.logger = MockLogger()
        reader.afc = MockAFC()
        reader.afc.lanes = lanes or {}
        reader.afc.tools = tools or {}
        return reader

    def test_direct_lane_name(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lanes={"lane1": lane})
        assert reader._resolve_lane("lane1") == (lane, None)

    def test_single_lane_by_extruder_name(self):
        lane = _Lane(name="lane1", extruder_obj=_Ext(name="e0"))
        reader = self._reader(lanes={"lane1": lane})
        assert reader._resolve_lane("e0") == (lane, None)

    def test_single_lane_by_th_extruder_name(self):
        lane = _Lane(name="lane1", extruder_obj=_Ext(th_name="extruder1"))
        reader = self._reader(lanes={"lane1": lane})
        assert reader._resolve_lane("extruder1") == (lane, None)

    def test_multiple_lanes_share_extruder_returns_extruder(self):
        ext = _Ext(name="e0")
        l1 = _Lane(name="lane1", extruder_obj=ext)
        l2 = _Lane(name="lane2", extruder_obj=ext)
        reader = self._reader(lanes={"lane1": l1, "lane2": l2})
        assert reader._resolve_lane("e0") == (None, ext)

    def test_tools_registry_single_lane(self):
        lane = _Lane(name="lane1")
        ext = _Ext(name="e0", lanes={"lane1": lane})
        reader = self._reader(lanes={}, tools={"e0": ext})
        assert reader._resolve_lane("e0") == (lane, None)

    def test_tools_registry_multiple_lanes_returns_extruder(self):
        ext = _Ext(name="e0", lanes={"a": _Lane("a"), "b": _Lane("b")})
        reader = self._reader(lanes={}, tools={"e0": ext})
        assert reader._resolve_lane("e0") == (None, ext)

    def test_tools_registry_matched_by_extruder_attr(self):
        lane = _Lane(name="lane1")
        nonmatch = _Ext(name="zzz", lanes={})
        ext = _Ext(name="e0", lanes={"lane1": lane})
        # Non-matching entry first so the loop's false branch is exercised.
        reader = self._reader(lanes={}, tools={"aaa": nonmatch, "bbb": ext})
        assert reader._resolve_lane("e0") == (lane, None)

    def test_nothing_matched_warns_and_returns_none_none(self):
        reader = self._reader(lanes={}, tools={})
        assert reader._resolve_lane("ghost") == (None, None)
        assert reader.logger.messages == [(
            "warning",
            "U1 RFID: 'ghost' resolved to no lanes. Available lanes=[]; "
            "extruders=[]. If 'ghost' is a standalone toolhead, ensure its "
            "[AFC_stepper] has 'standalone: True' and the [AFC_extruder ghost] "
            "section exists.")]


# ── register_lane ─────────────────────────────────────────────────────────────

class TestAFCU1RFIDRegisterLane:
    def test_registers_lane_and_channel(self):
        reader = _build()
        lane = _Lane(name="lane1")
        reader.register_lane(lane, 3)
        assert reader._lane_channel_map == {"lane1": 3}
        assert reader._lane_objects == {"lane1": lane}
        assert reader._last_uid[3] is None
        assert reader._channel_to_lane == {3: "lane1"}
        assert reader._consecutive_failures[3] == 0


# ── start ─────────────────────────────────────────────────────────────────────

class TestAFCU1RFIDStart:
    def _reader(self):
        afc = MockAFC()
        reader = _build(afc=afc)
        reader.afc = afc
        reader.logger = MockLogger()
        reader._try_attach_filament_detect = _Recorder_rfid_coverage(result=True)
        return reader

    def test_early_return_when_nothing_configured(self):
        reader = self._reader()
        reader._lane_channel_map = {}
        reader._cfg_scanner_channels = set()
        reader.start()
        assert reader._poll_timer is None
        assert reader._try_attach_filament_detect.called is False
        assert reader.logger.messages == []

    def test_lane_and_scanner_channels_logged(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 1}
        reader._cfg_scanners = set()
        reader._cfg_scanner_channels = {0}
        reader.start()
        assert reader._scanner_channels == {0}
        assert reader._poll_timer is not None
        assert reader._try_attach_filament_detect.called is True
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 1 lane channel(s): lane1=ch1"),
            ("info", "U1 RFID: standalone spool scanner channel(s): ch0"),
        ]

    def test_only_scanner_channels_no_lane_log(self):
        reader = self._reader()
        reader._lane_channel_map = {}
        reader._cfg_scanners = set()
        reader._cfg_scanner_channels = {0}
        reader.start()
        assert reader._scanner_channels == {0}
        assert reader._poll_timer is not None
        assert reader.logger.messages == [
            ("info", "U1 RFID: standalone spool scanner channel(s): ch0")]

    def test_lane_attached_scanner_logged(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 1}
        reader._cfg_scanners = {"lane1"}
        reader._cfg_scanner_channels = set()
        reader.start()
        assert reader._scanner_channels == {1}
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 1 lane channel(s): lane1=ch1"),
            ("info", "U1 RFID: lane-attached scanner(s): lane1"),
        ]


# ── _try_attach_filament_detect ───────────────────────────────────────────────

class TestAFCU1RFIDTryAttach:
    def test_already_attached_returns_true(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._filament_detect = object()
        reader._register_fd_callback = _Recorder_rfid_coverage()
        assert reader._try_attach_filament_detect() is True
        assert reader._register_fd_callback.called is False
        assert reader.logger.messages == []

    def test_fd_missing_returns_false(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._filament_detect = None
        reader.printer = _LookupPrinter({})
        assert reader._try_attach_filament_detect() is False
        assert reader.logger.messages == []

    def test_attaches_and_logs_recognized_api(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._filament_detect = None
        reader._register_fd_callback = _Recorder_rfid_coverage()

        class _FD:
            def get_a_filament_info(self, ch):
                return {}

            def get_status(self):
                return {}

        fd = _FD()
        reader.printer = _LookupPrinter({"filament_detect": fd})
        assert reader._try_attach_filament_detect() is True
        assert reader._filament_detect is fd
        assert reader._register_fd_callback.last == ((fd,), {})
        assert reader.logger.messages == [(
            "info",
            "U1 RFID: filament_detect attached (api: get_a_filament_info, "
            "get_status)")]

    def test_attaches_and_logs_none_recognized(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._filament_detect = None
        reader._register_fd_callback = _Recorder_rfid_coverage()
        fd = object()
        reader.printer = _LookupPrinter({"filament_detect": fd})
        assert reader._try_attach_filament_detect() is True
        assert reader.logger.messages == [(
            "info", "U1 RFID: filament_detect attached (api: none recognized)")]


# ── _register_fd_callback ─────────────────────────────────────────────────────

class TestAFCU1RFIDRegisterFdCallback:
    def test_returns_early_when_already_registered(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._fd_cb_registered = True
        reader._register_fd_callback(object())
        assert reader.logger.messages == []

    def test_registers_via_proven_api(self):
        reader = _build()
        reader.logger = MockLogger()

        class _FD:
            def __init__(self):
                self.registered = []

            def register_cb_2_update_filament_info(self, cb):
                self.registered.append(cb)

        fd = _FD()
        reader._register_fd_callback(fd)
        assert fd.registered == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [(
            "info",
            "U1 RFID: push callback registered via "
            "register_cb_2_update_filament_info")]

    def test_falls_back_to_cb_list_on_register_error(self):
        reader = _build()
        reader.logger = MockLogger()
        cb_list = []

        class _FD:
            def register_cb_2_update_filament_info(self, cb):
                raise RuntimeError("nope")

        fd = _FD()
        fd._notify_data_update_cb = cb_list
        reader._register_fd_callback(fd)
        assert cb_list == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("warning", "U1 RFID: failed to register info callback: nope"),
            ("info", "U1 RFID: push callback registered via "
                     "_notify_data_update_cb"),
        ]

    def test_appends_to_cb_list_when_no_register_api(self):
        reader = _build()
        reader.logger = MockLogger()
        fd = _FakeFD(cb_list=[])
        reader._register_fd_callback(fd)
        assert fd._notify_data_update_cb == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [(
            "info",
            "U1 RFID: push callback registered via _notify_data_update_cb")]

    def test_does_not_double_append_when_already_in_cb_list(self):
        reader = _build()
        reader.logger = MockLogger()
        fd = _FakeFD(cb_list=[reader._on_filament_info_update])
        reader._register_fd_callback(fd)
        assert fd._notify_data_update_cb == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True

    def test_warns_when_no_push_api(self):
        reader = _build()
        reader.logger = MockLogger()
        fd = object()  # no register method, no list
        reader._register_fd_callback(fd)
        assert reader._fd_cb_registered is False
        assert reader.logger.messages == [(
            "warning",
            "U1 RFID: no recognized filament_detect push-callback API; "
            "scanner will rely on polling only")]


# ── _on_filament_info_update ──────────────────────────────────────────────────

class TestAFCU1RFIDOnFilamentInfoUpdate:
    def _reader(self, check=None):
        reader = _build()
        reader.logger = MockLogger()
        reader._check_channel = check or _Recorder_rfid_coverage()
        return reader

    def test_dispatches_registered_channel(self):
        reader = self._reader()
        reader._channel_to_lane = {1: "lane1"}
        reader._on_filament_info_update(1, {"CARD_UID": 5})
        assert reader._check_channel.last == (
            ("lane1", 1), {"info": {"CARD_UID": 5}})
        assert reader.logger.messages == []

    def test_unregistered_channel_is_ignored(self):
        reader = self._reader()
        reader._channel_to_lane = {}
        reader._lane_channel_map = {"lane1": 1}
        reader._on_filament_info_update(1, {"CARD_UID": 5})
        assert reader._check_channel.called is False

    def test_dispatch_error_logs_warning(self):
        reader = self._reader(check=_Recorder_rfid_coverage(raises=RuntimeError("bad")))
        reader._channel_to_lane = {2: "lane2"}
        reader._on_filament_info_update(2, {"CARD_UID": 5})
        assert reader.logger.messages == [(
            "warning", "U1 RFID: _on_filament_info_update error ch2: bad")]

    def test_short_args_fall_back_to_lane_loop(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 1}
        reader._on_filament_info_update(1)  # len < 2
        assert reader._check_channel.last == (("lane1", 1), {})

    def test_non_int_first_arg_falls_back_to_lane_loop(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 1}
        reader._on_filament_info_update("x", {})  # args[0] not int
        assert reader._check_channel.last == (("lane1", 1), {})

    def test_non_dict_second_arg_falls_back_to_lane_loop(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 1}
        reader._on_filament_info_update(1, "x")  # args[1] not dict
        assert reader._check_channel.last == (("lane1", 1), {})

    def test_lane_loop_error_logs_warning(self):
        reader = self._reader(check=_Recorder_rfid_coverage(raises=RuntimeError("oops")))
        reader._lane_channel_map = {"lane1": 1}
        reader._on_filament_info_update()  # no args -> loop
        assert reader.logger.messages == [(
            "warning", "U1 RFID: _on_filament_info_update error lane1: oops")]


# ── stop ──────────────────────────────────────────────────────────────────────

class TestAFCU1RFIDStop:
    def test_noop_when_no_timer(self):
        reader = _build()
        calls = []
        reader.reactor.update_timer = lambda t, w: calls.append((t, w))
        reader._poll_timer = None
        reader.stop()
        assert calls == []

    def test_updates_timer_to_never(self):
        reader = _build()
        calls = []
        reader.reactor.update_timer = lambda t, w: calls.append((t, w))
        timer = object()
        reader._poll_timer = timer
        reader.stop()
        assert calls == [(timer, reader.reactor.NEVER)]


# ── _trigger_channel_update ───────────────────────────────────────────────────

class TestAFCU1RFIDTriggerChannelUpdate:
    def _reader(self, fd=None):
        reader = _build()
        reader.logger = MockLogger()
        reader._filament_detect = fd
        reader._gcode = MockGcode()
        return reader

    def test_returns_false_when_no_fd(self):
        reader = self._reader(fd=None)
        assert reader._trigger_channel_update(0) is False

    def test_success_via_filament_dt_update(self):
        reader = self._reader(fd=object())
        assert reader._trigger_channel_update(2) is True
        reader._gcode.run_script_from_command.assert_called_once_with(
            "FILAMENT_DT_UPDATE CHANNEL=2")
        assert reader.logger.messages == []

    def test_fallback_to_update_filament_info(self):
        class _FD:
            def __init__(self):
                self.calls = []

            def update_filament_info(self, ch):
                self.calls.append(ch)

        fd = _FD()
        reader = self._reader(fd=fd)
        reader._gcode.run_script_from_command.side_effect = RuntimeError("m400")
        assert reader._trigger_channel_update(3) is True
        assert fd.calls == [3]
        assert reader.logger.messages == [(
            "warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch3: m400")]

    def test_fallback_to_request_update(self):
        class _FD:
            def __init__(self):
                self.calls = []

            def request_update(self, ch):
                self.calls.append(ch)

        fd = _FD()
        reader = self._reader(fd=fd)
        reader._gcode.run_script_from_command.side_effect = RuntimeError("m400")
        assert reader._trigger_channel_update(4) is True
        assert fd.calls == [4]

    def test_all_paths_fail_returns_false(self):
        class _FD:
            def update_filament_info(self, ch):
                raise RuntimeError("x")

            def request_update(self, ch):
                raise RuntimeError("y")

        reader = self._reader(fd=_FD())
        reader._gcode.run_script_from_command.side_effect = RuntimeError("m400")
        assert reader._trigger_channel_update(5) is False

    def test_no_fallback_api_returns_false(self):
        reader = self._reader(fd=object())  # no update/request methods
        reader._gcode.run_script_from_command.side_effect = RuntimeError("m400")
        assert reader._trigger_channel_update(6) is False
        assert reader.logger.messages == [(
            "warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch6: m400")]


# ── _poll_cb ──────────────────────────────────────────────────────────────────

class TestAFCU1RFIDPollCb:
    def _reader(self, idle=None):
        reader = _build()
        reader.logger = MockLogger()
        reader._try_attach_filament_detect = _Recorder_rfid_coverage(result=True)
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=True)
        reader._check_channel = _Recorder_rfid_coverage()
        objs = {}
        if idle is not None:
            objs["idle_timeout"] = idle
        reader.printer = _LookupPrinter(objs)
        reader._scanner_channels = set()
        reader._cfg_scanner_channels = set()
        reader._lane_channel_map = {}
        return reader

    def test_backoff_when_fd_unavailable(self):
        reader = self._reader()
        reader._try_attach_filament_detect = _Recorder_rfid_coverage(result=False)
        assert reader._poll_cb(100.0) == 100.0 + _BACKOFF_INTERVAL

    def test_deferred_while_printing(self):
        class _Idle:
            def get_status(self, et):
                return {"state": "Printing"}

        reader = self._reader(idle=_Idle())
        assert reader._poll_cb(50.0) == 50.0 + POLL_INTERVAL
        assert reader._check_channel.called is False

    def test_idle_none_proceeds(self):
        reader = self._reader(idle=None)
        reader._lane_channel_map = {"lane1": 1}
        assert reader._poll_cb(10.0) == 10.0 + POLL_INTERVAL
        assert reader._check_channel.last == (("lane1", 1), {})

    def test_idle_not_printing_proceeds(self):
        class _Idle:
            def get_status(self, et):
                return {"state": "Idle"}

        reader = self._reader(idle=_Idle())
        reader._cfg_scanner_channels = {0}
        assert reader._poll_cb(0.0) == 0.0 + POLL_INTERVAL
        assert reader._check_channel.last == ((None, 0), {})

    def test_trigger_failure_reaches_backoff_threshold(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=False)
        reader._consecutive_failures = {0: _MAX_CONSECUTIVE_FAILURES - 1}
        ret = reader._poll_cb(0.0)
        assert reader._consecutive_failures[0] == _MAX_CONSECUTIVE_FAILURES
        assert reader._backed_off is True
        assert ret == 0.0 + _BACKOFF_INTERVAL
        assert reader.logger.messages == [(
            "error", "U1 RFID: ch0 failed 5 times consecutively, backing off")]

    def test_trigger_failure_below_threshold_no_backoff(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=False)
        reader._consecutive_failures = {0: 0}
        ret = reader._poll_cb(0.0)
        assert reader._consecutive_failures[0] == 1
        assert reader._backed_off is False
        assert ret == 0.0 + POLL_INTERVAL
        assert reader.logger.messages == []

    def test_trigger_success_resets_failures(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 3}
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=True)
        reader._poll_cb(0.0)
        assert reader._consecutive_failures[0] == 0

    def test_scanner_poll_error_logs_warning(self):
        reader = self._reader()
        reader._cfg_scanner_channels = {0}
        reader._check_channel = _Recorder_rfid_coverage(raises=RuntimeError("boom"))
        reader._poll_cb(0.0)
        assert reader.logger.messages == [(
            "warning", "U1 RFID: poll error on scanner ch0: boom")]

    def test_lane_poll_error_logs_warning(self):
        reader = self._reader()
        reader._lane_channel_map = {"lane1": 2}
        reader._check_channel = _Recorder_rfid_coverage(raises=RuntimeError("bad"))
        reader._poll_cb(0.0)
        assert reader.logger.messages == [(
            "warning", "U1 RFID: poll error on lane1 ch2: bad")]

    def test_backoff_reset_after_reset_cycles(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: _MAX_CONSECUTIVE_FAILURES}
        reader._backed_off = True
        reader._backoff_cycles = _BACKOFF_RESET_CYCLES - 1
        ret = reader._poll_cb(0.0)
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert reader._consecutive_failures[0] == 0
        assert ret == 0.0 + _BACKOFF_INTERVAL
        assert reader.logger.messages == [(
            "info", "U1 RFID: backoff reset, retrying normal polling")]

    def test_backoff_clears_when_all_recovered(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 0}
        reader._backed_off = True
        reader._backoff_cycles = 1
        ret = reader._poll_cb(0.0)
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert ret == 0.0 + _BACKOFF_INTERVAL
        assert reader.logger.messages == []

    def test_backoff_persists_when_not_recovered(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: _MAX_CONSECUTIVE_FAILURES}
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=True)
        reader._backed_off = True
        reader._backoff_cycles = 2
        ret = reader._poll_cb(0.0)
        # trigger success resets ch0 to 0, so all_recovered -> clears.
        assert reader._backed_off is False
        assert ret == 0.0 + _BACKOFF_INTERVAL


# ── _send_lane_data ───────────────────────────────────────────────────────────

class TestAFCU1RFIDSendLaneData:
    def test_skips_when_moonraker_down(self):
        reader = _build()
        reader.afc = MockAFC()
        reader.afc.moonraker = None
        lane = _Lane()
        reader._send_lane_data(lane)
        assert lane.send_lane_data.called is False

    def test_pushes_lane_data(self):
        reader = _build()
        reader.afc = MockAFC()
        lane = _Lane()
        reader._send_lane_data(lane)
        assert lane.send_lane_data.called is True

    def test_push_failure_logs_debug(self):
        reader = _build()
        reader.logger = MockLogger()
        reader.afc = MockAFC()
        lane = _Lane(name="lane9")
        lane.send_lane_data = _Recorder_rfid_coverage(raises=RuntimeError("nope"))
        reader._send_lane_data(lane)
        assert reader.logger.messages == [(
            "debug", "U1 RFID: send_lane_data skipped for lane9: nope")]


# ── _handle_webhook_scan ──────────────────────────────────────────────────────

class TestAFCU1RFIDHandleWebhookScan:
    def _reader(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._check_channel = _Recorder_rfid_coverage()
        reader._channel_to_lane = {0: "lane1"}
        return reader

    def test_missing_channel_returns(self):
        reader = self._reader()
        reader._handle_webhook_scan(_WebReq(channel_raises=True))
        assert reader._check_channel.called is False

    def test_unmonitored_channel_returns(self):
        reader = self._reader()
        reader._channel_to_lane = {}
        reader._handle_webhook_scan(_WebReq(data={"channel": 0}))
        assert reader._check_channel.called is False

    def test_full_payload_builds_info(self):
        reader = self._reader()
        req = _WebReq(data={
            "channel": 0, "manufacturer": "Bambu", "type": "PLA",
            "sub_type": "Basic", "hotend_min_temp": 200, "hotend_max_temp": 220,
            "bed_temp": 60, "weight_grams": 1000,
            "colors": [0xFF112233, 0xFF445566],
            "card_uid": [1, 2, 3], "manufacturing_date": "20240101",
            "diameter_mm": 1.75, "density": 1.24, "serial_number": "S1",
            "sku": "SKU9", "drying_temp_c": 55, "drying_time_hours": 8,
        })
        reader._handle_webhook_scan(req)
        assert reader._webhook_channels_seen == {0}
        args, kwargs = reader._check_channel.last
        assert args == ("lane1", 0)
        assert kwargs["source"] == "webhook"
        info = kwargs["info"]
        assert info == {
            "VENDOR": "Bambu", "MAIN_TYPE": "PLA", "SUB_TYPE": "Basic",
            "HOTEND_MIN_TEMP": 200, "HOTEND_MAX_TEMP": 220, "BED_TEMP": 60,
            "WEIGHT": 1000, "COLOR_NUMS": 2, "CARD_UID": [1, 2, 3],
            "MF_DATE": "20240101", "DIAMETER": 1.75, "DENSITY": 1.24,
            "SERIAL": "S1", "SKU": "SKU9", "DRYING_TEMP": 55, "DRYING_TIME": 8,
            "RGB_1": 0xFF112233, "RGB_2": 0xFF445566,
        }

    def test_non_list_colors_becomes_empty(self):
        reader = self._reader()
        req = _WebReq(data={"channel": 0, "colors": "notalist"})
        reader._handle_webhook_scan(req)
        info = reader._check_channel.last[1]["info"]
        assert info["COLOR_NUMS"] == 0
        assert "RGB_1" not in info

    def test_non_int_color_skipped(self):
        reader = self._reader()
        req = _WebReq(data={"channel": 0, "colors": ["bad", 0xFF445566]})
        reader._handle_webhook_scan(req)
        info = reader._check_channel.last[1]["info"]
        assert "RGB_1" not in info
        assert info["RGB_2"] == 0xFF445566

    def test_zero_valued_optional_field_omitted(self):
        reader = self._reader()
        req = _WebReq(data={"channel": 0, "drying_temp_c": 0, "density": 1.24})
        reader._handle_webhook_scan(req)
        info = reader._check_channel.last[1]["info"]
        assert "DRYING_TEMP" not in info
        assert info["DENSITY"] == 1.24

    def test_already_seen_channel_not_readded(self):
        reader = self._reader()
        reader._webhook_channels_seen = {0}
        req = _WebReq(data={"channel": 0})
        reader._handle_webhook_scan(req)
        assert reader._webhook_channels_seen == {0}
        assert reader._check_channel.called is True

    def test_check_channel_error_logs_warning(self):
        reader = self._reader()
        reader._check_channel = _Recorder_rfid_coverage(raises=RuntimeError("x"))
        req = _WebReq(data={"channel": 0})
        reader._handle_webhook_scan(req)
        assert reader.logger.messages == [(
            "warning", "U1 RFID: webhook scan error ch0: x")]


# ── _check_channel (lane path) ────────────────────────────────────────────────

_SLOT = {
    "brand": "Test", "material": "PLA", "color_hex": "FF0000",
    "multi_color": ["FF0000"], "sub_type": "",
}


class TestAFCU1RFIDCheckChannelLane:
    def _reader(self, lane, afc=None):
        afc = afc or MockAFC()
        reader = _build(afc=afc)
        reader.afc = afc
        reader.logger = MockLogger()
        reader._filament_detect = object()
        reader._cfg_scanner_channels = set()
        reader._lane_objects = {"lane1": lane}
        reader._channel_to_lane = {0: "lane1"}
        reader._webhook_channels_seen = set()
        reader._last_uid = {}
        reader._pending_confirm = {}
        reader._pending_defer = {}
        reader._webhook_grace = 0.0
        reader._scanner_confirm_reads = 1
        reader._tag_reads = {}
        reader._lane_auto_create = False
        reader._map_to_slot_info = lambda info: dict(_SLOT)
        reader._notify_scan = _Recorder_rfid_coverage()
        reader.printer = _LookupPrinter({})
        return reader

    def _tag(self, **over):
        base = {"CARD_UID": 0x1234, "MAIN_TYPE": "PLA", "SUB_TYPE": ""}
        base.update(over)
        return base

    def test_full_lane_load(self):
        lane = _Lane(name="lane1", status="", spool_id=5, tool_loaded=True)
        reader = self._reader(lane)
        events = []
        reader.printer.register_event_handler(
            "afc:tool_loaded", lambda ln: events.append(ln))
        tag = self._tag()
        with patch("extras.AFC_U1_rfid.apply_filament_defaults") as apply_def, \
                patch("extras.AFC_U1_rfid.get_auto_spoolman_create",
                      return_value=True), \
                patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman") as sync, \
                patch("extras.AFC_U1_rfid.make_tag_record", return_value={"r": 1}):
            reader._check_channel("lane1", 0, info=tag, source="poll")
        assert reader._last_uid[0] == 0x1234
        reader.afc.spool.set_spoolID.assert_called_once_with(lane, "")
        apply_def.assert_called_once_with(lane, _SLOT)
        assert sync.call_args.kwargs.get("allow_create") is True
        assert "set_next" not in sync.call_args.kwargs
        assert reader._notify_scan.last[1] == {"lane_name": "lane1"}
        assert lane.send_lane_data.called is True
        reader.afc.save_vars.assert_called_once()
        assert events == [lane]
        assert reader._tag_reads["lane1"] == {"r": 1}
        assert reader.logger.messages == [
            ("debug", f"U1 RFID: ch0 raw tag info: {tag}"),
            ("info", "U1 RFID: tag detected on lane1 — Test PLA (#FF0000)"),
        ]

    def test_colorless_tag_desc_has_no_color_label(self):
        lane = _Lane(name="lane1", status="", spool_id=None)
        reader = self._reader(lane)
        reader._map_to_slot_info = lambda info: {
            "brand": "Test", "material": "PLA", "color_hex": "",
            "multi_color": [], "sub_type": ""}
        tag = self._tag()
        with patch("extras.AFC_U1_rfid.apply_filament_defaults"), \
                patch("extras.AFC_U1_rfid.get_auto_spoolman_create",
                      return_value=False), \
                patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman"), \
                patch("extras.AFC_U1_rfid.make_tag_record", return_value={}):
            reader._check_channel("lane1", 0, info=tag, source="poll")
        assert reader.logger.messages == [
            ("debug", f"U1 RFID: ch0 raw tag info: {tag}"),
            ("info", "U1 RFID: tag detected on lane1 — Test PLA"),
        ]

    def test_no_spoolid_skips_clear(self):
        lane = _Lane(name="lane1", status="", spool_id=None, tool_loaded=False)
        reader = self._reader(lane)
        events = []
        reader.printer.register_event_handler(
            "afc:tool_loaded", lambda ln: events.append(ln))
        with patch("extras.AFC_U1_rfid.apply_filament_defaults"), \
                patch("extras.AFC_U1_rfid.get_auto_spoolman_create",
                      return_value=False), \
                patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman"), \
                patch("extras.AFC_U1_rfid.make_tag_record", return_value={}):
            reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        reader.afc.spool.set_spoolID.assert_not_called()
        assert events == []  # tool_loaded False -> no event

    def test_info_none_and_no_fd_returns(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._filament_detect = None
        reader._check_channel("lane1", 0, info=None)
        assert reader._notify_scan.called is False
        assert 0 not in reader._last_uid

    def test_info_none_reads_live_and_returns_on_none(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._get_channel_info = _Recorder_rfid_coverage(result=None)
        reader._check_channel("lane1", 0, info=None)
        assert reader._get_channel_info.last == ((0,), {})
        assert reader._notify_scan.called is False

    def test_removal_clears_lane(self):
        lane = _Lane(name="lane1", status="")
        reader = self._reader(lane)
        reader._last_uid = {0: 0x1234}
        reader._pending_confirm = {0: (0x1234, 1)}
        reader._clear_lane = _Recorder_rfid_coverage()
        reader._check_channel("lane1", 0, info=self._tag(CARD_UID=0))
        assert reader._last_uid[0] == 0
        assert 0 not in reader._pending_confirm
        assert reader._clear_lane.last == ((lane, "lane1"), {})

    def test_removal_skips_clear_when_locked(self):
        lane = _Lane(name="lane1", status="Loaded")
        reader = self._reader(lane)
        reader._last_uid = {0: 0x1234}
        reader._clear_lane = _Recorder_rfid_coverage()
        reader._check_channel("lane1", 0, info=self._tag(CARD_UID=0))
        assert reader._last_uid[0] == 0
        assert reader._clear_lane.called is False

    def test_webhook_seen_suppresses_poll_read(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._webhook_channels_seen = {0}
        reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        assert 0 not in reader._last_uid
        assert reader._notify_scan.called is False

    def test_dedup_same_uid_returns(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._last_uid = {0: 0x1234}
        reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        assert reader._notify_scan.called is False

    def test_lane_none_returns(self):
        reader = self._reader(_Lane(name="lane1"))
        reader._lane_objects = {}  # lane_name maps to nothing
        reader._check_channel("ghost", 0, info=self._tag(), source="poll")
        assert 0 not in reader._last_uid
        assert reader._notify_scan.called is False

    def test_locked_status_returns(self):
        lane = _Lane(name="lane1", status="Loaded")
        reader = self._reader(lane)
        reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        assert 0 not in reader._last_uid
        assert reader._notify_scan.called is False

    def test_main_type_none_records_uid_only(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._check_channel("lane1", 0, info=self._tag(MAIN_TYPE="NONE"),
                              source="poll")
        assert reader._last_uid[0] == 0x1234
        assert reader._notify_scan.called is False
        tag = self._tag(MAIN_TYPE="NONE")
        assert reader.logger.messages == [
            ("debug", f"U1 RFID: ch0 raw tag info: {tag}")]

    def test_webhook_grace_defers_new_tag(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._webhook_grace = 1.0
        calls = []
        reader.reactor.register_callback = lambda cb, when=None: calls.append(when)
        reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        assert reader._pending_defer[0] == 0x1234
        assert 0 not in reader._last_uid
        assert len(calls) == 1

    def test_webhook_grace_not_rearmed_for_same_uid(self):
        lane = _Lane(name="lane1")
        reader = self._reader(lane)
        reader._webhook_grace = 1.0
        reader._pending_defer = {0: 0x1234}
        calls = []
        reader.reactor.register_callback = lambda cb, when=None: calls.append(when)
        reader._check_channel("lane1", 0, info=self._tag(), source="poll")
        assert calls == []  # already armed for this uid

    def test_poll_final_bypasses_grace(self):
        lane = _Lane(name="lane1", spool_id=None)
        reader = self._reader(lane)
        reader._webhook_grace = 1.0
        with patch("extras.AFC_U1_rfid.apply_filament_defaults"), \
                patch("extras.AFC_U1_rfid.get_auto_spoolman_create",
                      return_value=False), \
                patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman"), \
                patch("extras.AFC_U1_rfid.make_tag_record", return_value={}):
            reader._check_channel("lane1", 0, info=self._tag(),
                                  source="poll-final")
        assert reader._last_uid[0] == 0x1234
        assert reader._notify_scan.called is True


# ── _grace_expired ────────────────────────────────────────────────────────────

class TestAFCU1RFIDGraceExpired:
    def _reader(self):
        reader = _build()
        reader.logger = MockLogger()
        reader._pending_defer = {}
        reader._webhook_channels_seen = set()
        reader._check_channel = _Recorder_rfid_coverage()
        return reader

    def test_superseded_uid_skips(self):
        reader = self._reader()
        reader._pending_defer = {0: 0x9999}
        reader._grace_expired("lane1", 0, 0x1234)
        assert reader._check_channel.called is False
        assert reader._pending_defer == {0: 0x9999}

    def test_webhook_landed_skips_read(self):
        reader = self._reader()
        reader._pending_defer = {0: 0x1234}
        reader._webhook_channels_seen = {0}
        reader._grace_expired("lane1", 0, 0x1234)
        assert 0 not in reader._pending_defer
        assert reader._check_channel.called is False

    def test_processes_deferred_read(self):
        reader = self._reader()
        reader._pending_defer = {0: 0x1234}
        reader._grace_expired("lane1", 0, 0x1234)
        assert reader._check_channel.last == (
            ("lane1", 0), {"source": "poll-final"})

    def test_deferred_read_error_logs_warning(self):
        reader = self._reader()
        reader._pending_defer = {0: 0x1234}
        reader._check_channel = _Recorder_rfid_coverage(raises=RuntimeError("bad"))
        reader._grace_expired("lane1", 0, 0x1234)
        assert reader.logger.messages == [(
            "warning", "U1 RFID: deferred read error ch0: bad")]


# ── _clear_lane ───────────────────────────────────────────────────────────────

class TestAFCU1RFIDClearLane:
    def test_clears_material_color_and_spool(self):
        reader = _build()
        reader.afc = MockAFC()
        lane = _Lane(name="lane1", spool_id=7)
        reader._clear_lane(lane, "lane1")
        assert lane.material == ""
        assert lane.color == ""
        reader.afc.spool.set_spoolID.assert_called_once_with(lane, "")
        assert lane.send_lane_data.called is True
        reader.afc.save_vars.assert_called_once()

    def test_skips_spool_clear_when_unset(self):
        reader = _build()
        reader.afc = MockAFC()
        lane = _Lane(name="lane1", spool_id=None)
        reader._clear_lane(lane, "lane1")
        reader.afc.spool.set_spoolID.assert_not_called()

    def test_spool_clear_error_logs_warning(self):
        reader = _build()
        reader.logger = MockLogger()
        reader.afc = MockAFC()
        reader.afc.spool.set_spoolID.side_effect = RuntimeError("nope")
        lane = _Lane(name="lane2", spool_id=7)
        reader._clear_lane(lane, "lane2")
        assert reader.logger.messages == [(
            "warning", "U1 RFID: failed to clear spool_id on lane2: nope")]


# ── _get_channel_info ─────────────────────────────────────────────────────────

class TestAFCU1RFIDGetChannelInfo:
    def _reader(self, fd):
        reader = _build()
        reader._filament_detect = fd
        return reader

    def test_get_a_filament_info_dict(self):
        class _FD:
            def get_a_filament_info(self, ch):
                return {"CARD_UID": 1}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) == {"CARD_UID": 1}

    def test_get_a_filament_info_non_dict_falls_through(self):
        class _FD:
            def get_a_filament_info(self, ch):
                return None

            def get_all_filament_info(self):
                return [{"CARD_UID": 2}]

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) == {"CARD_UID": 2}

    def test_get_a_filament_info_error_falls_through(self):
        class _FD:
            def get_a_filament_info(self, ch):
                raise RuntimeError("x")

            def get_all_filament_info(self):
                return {"0": {"CARD_UID": 3}}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) == {"CARD_UID": 3}

    def test_get_all_dict_by_int_key(self):
        class _FD:
            def get_all_filament_info(self):
                return {0: {"CARD_UID": 4}}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) == {"CARD_UID": 4}

    def test_get_all_error_falls_through_to_status(self):
        class _FD:
            def get_all_filament_info(self):
                raise RuntimeError("x")

            def get_status(self):
                return {"info": [{"CARD_UID": 5}]}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) == {"CARD_UID": 5}

    def test_status_entry_without_uid_returns_none(self):
        class _FD:
            def get_status(self):
                return {"info": [{"NO_UID": 1}]}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_get_all_list_out_of_range_falls_through(self):
        class _FD:
            def get_all_filament_info(self):
                return [{"CARD_UID": 1}]

        reader = self._reader(_FD())
        assert reader._get_channel_info(5) is None  # channel >= len

    def test_get_all_list_entry_not_dict_falls_through(self):
        class _FD:
            def get_all_filament_info(self):
                return ["not-a-dict"]

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_get_all_dict_missing_key_falls_through(self):
        class _FD:
            def get_all_filament_info(self):
                return {"9": {"CARD_UID": 1}}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_status_raises_returns_none(self):
        class _FD:
            def get_status(self):
                raise RuntimeError("x")

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_status_non_dict_returns_none(self):
        class _FD:
            def get_status(self):
                return ["not", "a", "dict"]

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_status_channel_out_of_range_returns_none(self):
        class _FD:
            def get_status(self):
                return {"info": [{"CARD_UID": 1}]}

        reader = self._reader(_FD())
        assert reader._get_channel_info(9) is None

    def test_status_entry_not_dict_returns_none(self):
        class _FD:
            def get_status(self):
                return {"info": ["not-a-dict"]}

        reader = self._reader(_FD())
        assert reader._get_channel_info(0) is None

    def test_no_api_returns_none(self):
        reader = self._reader(object())
        assert reader._get_channel_info(0) is None


# ── _tag_color_count ──────────────────────────────────────────────────────────

class TestAFCU1RFIDTagColorCount:
    def test_reads_color_nums(self):
        reader = _build()
        assert reader._tag_color_count({"COLOR_NUMS": 2}) == 2

    def test_reads_colour_count_variant(self):
        reader = _build()
        assert reader._tag_color_count({"COLOUR_COUNT": "3"}) == 3

    def test_non_numeric_skipped(self):
        reader = _build()
        assert reader._tag_color_count({"COLOR_NUM": "abc"}) is None

    def test_zero_count_skipped(self):
        reader = _build()
        assert reader._tag_color_count({"COLOR_NUM": 0}) is None

    def test_no_matching_key_returns_none(self):
        reader = _build()
        assert reader._tag_color_count({"MAIN_TYPE": "PLA"}) is None


# ── _map_to_slot_info ─────────────────────────────────────────────────────────

class TestAFCU1RFIDMapToSlotInfo:
    def test_full_tag_with_color_count(self):
        reader = _build()
        reader.logger = MockLogger()
        info = {
            "RGB_1": 0xFF112233, "RGB_2": 0xFF445566, "COLOR_NUMS": 2,
            "MAIN_TYPE": "PLA", "HOTEND_MAX_TEMP": 220, "HOTEND_MIN_TEMP": 200,
            "BED_TEMP": 60, "VENDOR": "Bambu", "SUB_TYPE": "Basic",
            "CARD_UID": [1, 2, 3, 4], "MF_DATE": "20240101", "WEIGHT": 1000,
            "DIAMETER": 1.75, "SKU": 123, "SERIAL": "S1", "DENSITY": 1.24,
            "DRYING_TEMP": 55, "DRYING_TIME": 8,
        }
        slot = reader._map_to_slot_info(info)
        assert slot["material"] == "PLA"
        assert slot["color_hex"] == "112233"
        assert slot["multi_color"] == ["112233", "445566"]
        assert slot["is_dual_color"] is True
        assert slot["sku"] == "123"
        assert slot["brand"] == "Bambu"
        assert slot["sub_type"] == "Basic"
        assert slot["diameter"] == 1.75
        assert slot["extruder_temp"] == (220 + 200) // 2
        assert slot["bed_temp"] == 60
        assert slot["mfg_date"] == "2024-01-01"
        assert slot["uid"] == "01020304"
        assert slot["extruder_temp_min"] == 200
        assert slot["extruder_temp_max"] == 220
        assert slot["weight_g"] == 1000
        assert slot["serial"] == "S1"
        assert slot["density"] == 1.24
        assert slot["drying_temp"] == 55
        assert slot["drying_time_h"] == 8
        assert slot["color_count"] == 2
        assert reader.logger.messages == [(
            "debug",
            "U1 RFID: parsed 2 colour(s) ['112233', '445566'] from RGB slots "
            "['112233', '445566'] (tag count=2)")]

    def test_white_sentinel_heuristic_without_count(self):
        reader = _build()
        reader.logger = MockLogger()
        info = {"RGB_1": 0xAA112233, "RGB_2": 0xFFFFFFFF, "MAIN_TYPE": "PLA"}
        slot = reader._map_to_slot_info(info)
        assert slot["multi_color"] == ["112233"]
        assert slot["is_dual_color"] is False
        assert reader.logger.messages == [(
            "debug",
            "U1 RFID: parsed 1 colour(s) ['112233'] from RGB slots "
            "['112233', 'ffffff'] (no tag count field; white-sentinel "
            "heuristic)")]

    def test_duplicate_colors_deduped_with_count(self):
        reader = _build()
        reader.logger = MockLogger()
        info = {"RGB_1": 0xFF112233, "RGB_2": 0xFF112233, "COLOR_NUMS": 2}
        slot = reader._map_to_slot_info(info)
        assert slot["multi_color"] == ["112233"]
        assert slot["is_dual_color"] is False

    def test_duplicate_colors_deduped_in_heuristic(self):
        reader = _build()
        reader.logger = MockLogger()
        info = {"RGB_1": 0xAA112233, "RGB_2": 0xAA112233}
        slot = reader._map_to_slot_info(info)
        assert slot["multi_color"] == ["112233"]

    def test_skips_blank_and_bad_rgb_values(self):
        reader = _build()
        reader.logger = MockLogger()
        info = {"RGB_1": "", "RGB_2": "zz", "RGB_3": 0xFF010203,
                "MAIN_TYPE": "PLA"}
        slot = reader._map_to_slot_info(info)
        assert slot["multi_color"] == ["010203"]

    def test_extruder_temp_max_only(self):
        reader = _build()
        reader.logger = MockLogger()
        slot = reader._map_to_slot_info({"HOTEND_MAX_TEMP": 240})
        assert slot["extruder_temp"] == 240
        assert "extruder_temp_min" not in slot

    def test_extruder_temp_none_when_no_temps(self):
        reader = _build()
        reader.logger = MockLogger()
        slot = reader._map_to_slot_info({"MAIN_TYPE": "PLA"})
        assert slot["extruder_temp"] is None
        assert slot["bed_temp"] is None

    def test_bad_diameter_defaults(self):
        reader = _build()
        reader.logger = MockLogger()
        slot = reader._map_to_slot_info({"DIAMETER": "bad"})
        assert slot["diameter"] == 1.75

    def test_zero_diameter_defaults(self):
        reader = _build()
        reader.logger = MockLogger()
        slot = reader._map_to_slot_info({"DIAMETER": 0})
        assert slot["diameter"] == 1.75

    def test_vendor_none_blanked_and_bad_weight(self):
        reader = _build()
        reader.logger = MockLogger()
        slot = reader._map_to_slot_info({"VENDOR": "NONE", "WEIGHT": "bad"})
        assert slot["brand"] == ""
        assert "weight_g" not in slot


# ── _fmt_mfg_date ─────────────────────────────────────────────────────────────

class TestAFCU1RFIDFmtMfgDate:
    def test_none_when_empty(self):
        assert AFC_U1_RFID._fmt_mfg_date("") is None

    def test_none_for_epoch(self):
        assert AFC_U1_RFID._fmt_mfg_date("19700101") is None

    def test_none_for_zero_strings(self):
        assert AFC_U1_RFID._fmt_mfg_date("00000000") is None
        assert AFC_U1_RFID._fmt_mfg_date("0") is None

    def test_formats_yyyymmdd(self):
        assert AFC_U1_RFID._fmt_mfg_date("20240517") == "2024-05-17"

    def test_passthrough_iso(self):
        assert AFC_U1_RFID._fmt_mfg_date("2024-05-17") == "2024-05-17"


# ── _fmt_uid ──────────────────────────────────────────────────────────────────

class TestAFCU1RFIDFmtUid:
    def test_none_when_empty(self):
        assert AFC_U1_RFID._fmt_uid(None) is None

    def test_byte_list_to_hex(self):
        assert AFC_U1_RFID._fmt_uid([123, 240, 175, 255]) == "7BF0AFFF"

    def test_bad_byte_list_returns_none(self):
        assert AFC_U1_RFID._fmt_uid([1, "x"]) is None

    def test_string_uppercased(self):
        assert AFC_U1_RFID._fmt_uid(" 7bf0afff ") == "7BF0AFFF"


# ── _notify_scan ──────────────────────────────────────────────────────────────

class TestAFCU1RFIDNotifyScan:
    def _reader(self, afc=None):
        afc = afc or MockAFC()
        reader = _build(afc=afc)
        reader.afc = afc
        reader.logger = MockLogger()
        reader._lane_objects = {}
        reader._lane_channel_map = {}
        reader.printer = _LookupPrinter({})
        reader._responses = []
        afc.gcode.respond_info = lambda msg: reader._responses.append(msg)
        return reader

    def test_lane_load_console_only(self):
        reader = self._reader()
        slot = {"sub_type": "Basic", "extruder_temp": 210, "bed_temp": 60}
        reader._notify_scan("Bambu", "PLA", "FF0000", slot, lane_name="lane1")
        assert reader._responses == ["\n".join([
            "Spool loaded on lane1:",
            "  Name: Bambu PLA Basic",
            "  Brand: Bambu",
            "  Material: PLA",
            "  Color: #FF0000",
            "  Nozzle temp: 210°C",
            "  Bed temp: 60°C",
        ])]
        # no popup for a lane load
        assert reader.logger.messages == []

    def test_lane_load_no_name_header(self):
        reader = self._reader()
        reader._notify_scan("", "", "", {}, lane_name="")
        assert reader._responses == ["Spool loaded:"]

    def test_scanner_emits_prompt_and_exception(self):
        em = _Recorder_rfid_coverage()

        class _EM:
            def raise_exception_async(self, **kwargs):
                em(**kwargs)

        reader = self._reader()
        reader.printer = _LookupPrinter({"exception_manager": _EM()})
        reader._lane_channel_map = {"lane1": 2}
        slot = {"sub_type": "Basic"}
        reader._notify_scan("Bambu", "PLA", "FF0000", slot,
                            lane_name="lane1", is_scanner=True)
        assert reader._responses == ["\n".join([
            "Spool scanned on lane1:",
            "  Name: Bambu PLA Basic",
            "  Brand: Bambu",
            "  Material: PLA",
            "  Color: #FF0000",
        ])]
        assert reader.logger.messages == [
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text Name: Bambu PLA Basic"),
            ("raw", "// action:prompt_text Brand: Bambu"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_text Color: #FF0000"),
            ("raw", "// action:prompt_footer_button "
                    "OK|RESPOND TYPE=command MSG=action:prompt_end|info"),
            ("raw", "// action:prompt_show"),
        ]
        assert em.last[1] == {
            "id": 529, "index": 2, "code": 99,
            "message": "Spool Scanned: Bambu PLA Basic", "oneshot": 1,
            "level": 1}

    def test_scanner_prompt_includes_temps_and_spool_id(self):
        afc = MockAFC()
        reader = self._reader(afc=afc)
        lane = _Lane(name="lane1", spool_id=42)
        reader._lane_objects = {"lane1": lane}
        reader._lane_channel_map = {"lane1": 0}
        enriched = {"extruder_temp": 210, "bed_temp": 60}
        with patch("extras.AFC_U1_rfid.SpoolmanClient"), \
                patch("extras.AFC_U1_rfid.enrich_from_spool",
                      return_value=enriched):
            reader._notify_scan("Bambu", "PLA", "FF0000", {"sub_type": ""},
                                lane_name="lane1", is_scanner=True)
        assert reader._responses == ["\n".join([
            "Spool scanned on lane1:",
            "  Name: Bambu PLA",
            "  Brand: Bambu",
            "  Material: PLA",
            "  Color: #FF0000",
            "  Nozzle temp: 210°C",
            "  Bed temp: 60°C",
            "  Spoolman ID: 42",
        ])]
        assert reader.logger.messages == [
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text Name: Bambu PLA"),
            ("raw", "// action:prompt_text Brand: Bambu"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_text Color: #FF0000"),
            ("raw", "// action:prompt_text Nozzle: 210°C"),
            ("raw", "// action:prompt_text Bed: 60°C"),
            ("raw", "// action:prompt_text Spoolman ID: 42"),
            ("raw", "// action:prompt_footer_button "
                    "OK|RESPOND TYPE=command MSG=action:prompt_end|info"),
            ("raw", "// action:prompt_show"),
        ]

    def test_scanner_no_exception_manager(self):
        reader = self._reader()
        reader.printer = _LookupPrinter({})  # no exception_manager
        reader._notify_scan("", "", "", {}, lane_name="", is_scanner=True)
        assert reader._responses == ["Spool scanned:"]

    def test_notification_error_logs_warning(self):
        reader = self._reader()
        with patch("extras.AFC_U1_rfid.build_filament_name",
                   side_effect=RuntimeError("boom")):
            reader._notify_scan("Bambu", "PLA", "FF0000", {}, lane_name="lane1")
        assert reader.logger.messages == [(
            "warning", "U1 RFID: notification error: boom")]

    def test_enrichment_overlays_spool_record(self):
        afc = MockAFC()
        reader = self._reader(afc=afc)
        lane = _Lane(name="lane1", spool_id=42)
        reader._lane_objects = {"lane1": lane}
        enriched = {
            "display_name": "Stored Name", "brand": "StoredBrand",
            "material": "PETG", "extruder_temp": 230, "bed_temp": 80,
        }
        with patch("extras.AFC_U1_rfid.SpoolmanClient"), \
                patch("extras.AFC_U1_rfid.enrich_from_spool",
                      return_value=enriched):
            reader._notify_scan("Bambu", "PLA", "FF0000",
                                {"sub_type": ""}, lane_name="lane1")
        assert reader._responses == ["\n".join([
            "Spool loaded on lane1:",
            "  Name: Stored Name",
            "  Brand: StoredBrand",
            "  Material: PETG",
            "  Color: #FF0000",
            "  Nozzle temp: 230°C",
            "  Bed temp: 80°C",
            "  Spoolman ID: 42",
        ])]


# ── force_read ────────────────────────────────────────────────────────────────

class TestAFCU1RFIDForceRead:
    def _reader(self, times):
        reader = _build()
        reader.logger = MockLogger()
        reader.reactor = _AdvancingReactor(times)
        reader._lane_channel_map = {"lane1": 0}
        reader._last_uid = {0: 0x1111}
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=True)
        reader._get_channel_info = _Recorder_rfid_coverage(result=None)
        reader._check_channel = _Recorder_rfid_coverage()
        return reader

    def test_unknown_lane_returns(self):
        reader = self._reader([100.0])
        reader._lane_channel_map = {}
        reader.force_read("ghost")
        assert reader._trigger_channel_update.called is False
        assert reader._last_uid == {0: 0x1111}

    def test_trigger_failure_warns(self):
        reader = self._reader([100.0])
        reader._trigger_channel_update = _Recorder_rfid_coverage(result=False)
        reader.force_read("lane1")
        assert reader._last_uid[0] is None
        assert reader._check_channel.called is False
        assert reader.logger.messages == [(
            "warning",
            "U1 RFID: force_read failed to trigger update for lane1")]

    def test_reads_within_deadline(self):
        reader = self._reader([100.0, 100.2])
        reader._get_channel_info = _Recorder_rfid_coverage(result={"CARD_UID": 0x2222})
        reader.force_read("lane1")
        args, kwargs = reader._check_channel.last
        assert args == ("lane1", 0)
        assert kwargs == {"info": {"CARD_UID": 0x2222}}

    def test_timeout_falls_back_to_final_check(self):
        reader = self._reader([100.0, 100.2, 101.5, 101.5])
        reader.force_read("lane1")
        assert reader._check_channel.last == (("lane1", 0), {})


# ── get_status ────────────────────────────────────────────────────────────────

class TestAFCU1RFIDGetStatus:
    def test_reports_wiring_and_reads(self):
        reader = _build()
        reader._lane_channel_map = {"lane1": 1}
        reader._scanner_channels = {2, 0}
        reader._tag_reads = {"lane1": {"material": "PLA"}}
        status = reader.get_status()
        assert status == {
            "lane_channel_map": {"lane1": 1},
            "scanner_channels": [0, 2],
            "last_reads": {"lane1": {"material": "PLA"}},
        }

    def test_missing_tag_reads_defaults_empty(self):
        reader = _build()
        reader._lane_channel_map = {}
        reader._scanner_channels = set()
        del reader._tag_reads
        assert reader.get_status()["last_reads"] == {}


# ── load_config ───────────────────────────────────────────────────────────────

class TestLoadConfig:
    def test_returns_reader_instance(self):
        printer = MockPrinter(afc=MockAFC())
        config = MockConfig(printer=printer, values={})
        with patch("extras.AFC_U1_rfid.logging.getLogger",
                   return_value=MockLogger()):
            reader = load_config(config)
        assert isinstance(reader, AFC_U1_RFID)


# ── Unit tests for the U1 RFID spool-scanner stable-read gate in ──────────────
#
# was tests/test_AFC_U1_rfid_scanner.py
TAG = {
    "CARD_UID": 0x56A36AEA,
    "MAIN_TYPE": "PLA",
    "SUB_TYPE": "",
}
TAG_OTHER = dict(TAG, CARD_UID=0x26A36AEA)


# ── Typed fakes ───────────────────────────────────────────────────────────────

class _Recorder_rfid_scanner:
    def __init__(self, result=None):
        self.calls = []
        self.result = result

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0

    @property
    def call_count(self):
        return len(self.calls)


class _FakeLogger:
    def __init__(self):
        self.lines = {"info": [], "debug": [], "warning": [], "error": []}

    def info(self, msg, *a, **k):
        self.lines["info"].append(msg)

    def debug(self, msg, *a, **k):
        self.lines["debug"].append(msg)

    def warning(self, msg, *a, **k):
        self.lines["warning"].append(msg)

    def error(self, msg, *a, **k):
        self.lines["error"].append(msg)


class _FakeReactor:
    def __init__(self):
        self.register_callback = _Recorder_rfid_scanner()

    def monotonic(self):
        return 100.0


class _FakeAFC:
    def __init__(self):
        self.lanes = {}


def _make_rfid(confirm_reads=3, webhook_grace=0.0):
    rfid = AFC_U1_RFID.__new__(AFC_U1_RFID)
    rfid.logger = _FakeLogger()
    rfid.afc = _FakeAFC()
    rfid.reactor = _FakeReactor()
    rfid._filament_detect = object()
    rfid._cfg_scanner_channels = {0}
    rfid._lane_objects = {}
    rfid._lane_channel_map = {}
    rfid._scanner_confirm_reads = confirm_reads
    rfid._pending_confirm = {}
    rfid._pending_defer = {}
    rfid._last_uid = {}
    rfid._webhook_channels_seen = set()
    rfid._webhook_grace = webhook_grace
    rfid._scanner_auto_create = True
    rfid._lane_auto_create = True
    rfid._notify_scan = _Recorder_rfid_scanner()
    rfid._map_to_slot_info = _Recorder_rfid_scanner(result={
        "brand": "Test", "material": "PLA", "color_hex": "FF0000",
        "multi_color": ["FF0000"]})
    return rfid


def _scan(rfid, info=TAG, source='poll'):
    with patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman") as sync:
        rfid._check_channel("", 0, info=dict(info), source=source)
    return sync


# ── Stable-read gate ──────────────────────────────────────────────────────────

def test_scan_waits_for_n_consecutive_reads():
    rfid = _make_rfid(confirm_reads=3)

    sync1 = _scan(rfid)
    assert not sync1.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 1)
    assert 0 not in rfid._last_uid            # not yet acted

    sync2 = _scan(rfid)
    assert not sync2.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 2)

    sync3 = _scan(rfid)  # third consecutive identical read acts
    sync3.assert_called_once()
    assert rfid._notify_scan.call_count == 1
    assert 0 not in rfid._pending_confirm     # gate consumed
    assert rfid._last_uid[0] == TAG["CARD_UID"]


def test_different_uid_mid_confirmation_resets_count():
    """A corrupt/partial UID mid-stream must not accumulate — a stable clean
    read wins over transient misreads."""
    rfid = _make_rfid(confirm_reads=3)

    _scan(rfid)                       # uid A: count 1
    _scan(rfid, info=TAG_OTHER)       # uid B: count resets to 1
    assert rfid._pending_confirm[0] == (TAG_OTHER["CARD_UID"], 1)

    sync = _scan(rfid)                # uid A again: count 1, not 2
    assert not sync.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 1)


def test_confirm_reads_of_one_acts_immediately():
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid)
    sync.assert_called_once()
    assert rfid._pending_confirm == {}        # gate never engaged
    assert rfid._last_uid[0] == TAG["CARD_UID"]


def test_webhook_bypasses_gate():
    """A webhook is a full-data authoritative push — no confirmation needed."""
    rfid = _make_rfid(confirm_reads=3)
    sync = _scan(rfid, source='webhook')
    sync.assert_called_once()
    assert rfid._pending_confirm == {}
    assert rfid._notify_scan.call_count == 1


# ── Dedup / removal ───────────────────────────────────────────────────────────

def test_same_uid_never_refires():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()

    sync = _scan(rfid)  # spool still presented
    assert not sync.called
    assert rfid._notify_scan.call_count == 1


def test_new_spool_after_first_fires_again():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()
    _scan(rfid, info=TAG_OTHER).assert_called_once()
    assert rfid._last_uid[0] == TAG_OTHER["CARD_UID"]
    assert rfid._notify_scan.call_count == 2


def test_tag_removal_clears_pending_confirmation():
    rfid = _make_rfid(confirm_reads=3)
    _scan(rfid)
    assert 0 in rfid._pending_confirm

    sync = _scan(rfid, info=dict(TAG, CARD_UID=0))  # tag removed

    assert not sync.called
    assert 0 not in rfid._pending_confirm
    assert rfid._last_uid.get(0) in (None, 0)  # nothing was staged yet


def test_scanner_keeps_last_uid_after_removal():
    """Scanner channels intentionally keep _last_uid after a completed scan:
    the same spool must not re-fire while/after being presented."""
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()
    assert rfid._last_uid[0] == TAG["CARD_UID"]

    _scan(rfid, info=dict(TAG, CARD_UID=0))         # removed
    assert rfid._last_uid[0] == TAG["CARD_UID"]     # kept (scanner channel)

    sync = _scan(rfid)                              # same spool re-presented
    assert not sync.called                          # still deduped


# ── Content gates ─────────────────────────────────────────────────────────────

def test_main_type_none_records_uid_but_does_not_act():
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid, info=dict(TAG, MAIN_TYPE="NONE"))
    assert not sync.called
    assert not rfid._notify_scan.called
    assert rfid._last_uid[0] == TAG["CARD_UID"]  # recorded for dedup


def test_scanner_sets_next_spool_staging():
    """Scanner reads stage via next_spool_id (set_next=True) rather than
    assigning to a lane."""
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid)
    assert sync.call_args.kwargs.get("set_next") is True
    args = sync.call_args.args
    assert args[1] is None  # scanner channel has no lane

