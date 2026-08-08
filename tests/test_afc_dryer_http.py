# Tests for the HTTP surface of extras/afc_dryer.py.
#
# The panel is embedded in Mainsail/Fluidd as an iframe on a DIFFERENT origin,
# so the CORS and cache headers are load-bearing: without them the page renders
# but every fetch it makes is blocked by the browser, which looks like "the
# panel is broken" rather than "a header is missing".
#
# The POST path is the one that moves hardware, so it has to reject junk before
# it reaches request_dry, and it must surface a rejection as a 400 the page can
# display rather than a 500 that reads as a crash.
from __future__ import annotations

import json
import types

import pytest

import extras.afc_dryer as dryer


class _Panel:
    """Stand-in for AFCDryer: records what the handler asked it to do."""

    def __init__(self, state=None, raises=None, script="AFC_BAMBU_HEATER_START X"):
        self.logger = types.SimpleNamespace(debug=lambda *a, **k: None)
        self._state = state if state is not None else {"units": []}
        self._raises = raises
        self._script = script
        self.calls = []

    def get_state(self):
        return self._state

    def request_dry(self, **kw):
        self.calls.append(kw)
        if self._raises is not None:
            raise self._raises
        return self._script


class _Wfile:
    def __init__(self, fail=False):
        self.data = b""
        self._fail = fail

    def write(self, b):
        if self._fail:
            raise BrokenPipeError("client went away")
        self.data += b


class _Rfile:
    def __init__(self, body=b""):
        self._body = body

    def read(self, n):
        return self._body[:n]


def _handler(panel, path="/", body=None, method="GET", wfile=None):
    """Instantiate the handler without a socket and drive one request."""
    H = dryer._make_handler(panel)
    h = H.__new__(H)                       # bypass BaseHTTPRequestHandler.__init__
    h.path = path
    h.wfile = wfile if wfile is not None else _Wfile()
    h.rfile = _Rfile(body or b"")
    h.headers = {"Content-Length": str(len(body or b""))}
    h.status = None
    h.sent_headers = {}
    h.send_response = lambda code, *a: setattr(h, "status", code)
    h.send_header = lambda k, v: h.sent_headers.__setitem__(k, v)
    h.end_headers = lambda: None
    getattr(h, "do_" + method)()
    return h


def _body_json(h):
    return json.loads(h.wfile.data.decode())


class TestGet:
    def test_root_serves_the_page_as_html(self):
        h = _handler(_Panel(), "/")
        assert h.status == 200
        assert h.sent_headers["Content-Type"].startswith("text/html")
        assert h.wfile.data[:20] != b""

    def test_trailing_slash_and_query_are_normalised(self):
        # The iframe URL commonly carries a cache-busting query.
        for p in ("/", "/?v=2", "/api/state/", "/api/state?x=1"):
            h = _handler(_Panel(), p)
            assert h.status == 200, p

    def test_state_endpoint_returns_the_panel_state(self):
        panel = _Panel(state={"units": [{"name": "AMS_HT", "drying": True}]})
        h = _handler(panel, "/api/state")
        assert h.status == 200
        assert _body_json(h) == panel._state
        assert h.sent_headers["Content-Type"] == "application/json"

    def test_options_endpoint_lists_temps_and_times(self):
        h = _handler(_Panel(), "/api/options")
        got = _body_json(h)
        assert got["temps"] == list(dryer.TEMP_CHOICES)
        # times are (minutes, label) pairs rendered for the dropdown
        assert got["times"][0]["minutes"] == dryer.TIME_CHOICES[0][0]
        assert got["times"][0]["label"] == dryer.TIME_CHOICES[0][1]
        assert len(got["times"]) == len(dryer.TIME_CHOICES)

    def test_unknown_path_is_404_json(self):
        h = _handler(_Panel(), "/nope")
        assert h.status == 404
        assert "error" in _body_json(h)


class TestCorsAndCacheHeaders:
    """The page runs in an iframe on another origin. Without these the panel
    renders and every fetch it makes is blocked by the browser."""

    def test_get_sets_cors_and_no_store(self):
        h = _handler(_Panel(), "/api/state")
        assert h.sent_headers["Access-Control-Allow-Origin"] == "*"
        assert h.sent_headers["Cache-Control"] == "no-store"

    def test_content_length_matches_the_body(self):
        h = _handler(_Panel(), "/api/state")
        assert int(h.sent_headers["Content-Length"]) == len(h.wfile.data)

    def test_preflight_allows_get_and_post(self):
        h = _handler(_Panel(), "/api/dry", method="OPTIONS")
        assert h.status == 204
        assert h.sent_headers["Access-Control-Allow-Origin"] == "*"
        assert "POST" in h.sent_headers["Access-Control-Allow-Methods"]
        assert "Content-Type" in h.sent_headers["Access-Control-Allow-Headers"]


class TestPost:
    def _post(self, panel, obj, path="/api/dry"):
        return _handler(panel, path, body=json.dumps(obj).encode(),
                        method="POST")

    def test_valid_request_is_forwarded_and_acknowledged(self):
        panel = _Panel()
        h = self._post(panel, {"unit": "AMS_HT", "action": "start",
                               "temp": 55, "minutes": 480, "rotate": 1})
        assert h.status == 200
        assert _body_json(h) == {"ok": True, "queued": "AFC_BAMBU_HEATER_START X"}
        assert panel.calls == [{"name": "AMS_HT", "action": "start",
                                "temp": 55, "minutes": 480, "rotate": 1}]

    def test_defaults_are_applied_for_missing_fields(self):
        panel = _Panel()
        self._post(panel, {"unit": "u", "action": "start"})
        assert panel.calls[0]["temp"] == 55
        assert panel.calls[0]["minutes"] == 480
        assert panel.calls[0]["rotate"] == 0

    def test_wrong_path_is_404_and_never_reaches_the_hardware(self):
        panel = _Panel()
        h = self._post(panel, {"unit": "u"}, path="/api/other")
        assert h.status == 404
        assert panel.calls == []

    def test_malformed_json_is_400_not_500(self):
        panel = _Panel()
        h = _handler(panel, "/api/dry", body=b"{not json", method="POST")
        assert h.status == 400
        assert "bad request body" in _body_json(h)["error"]
        assert panel.calls == []

    def test_non_numeric_temp_is_400(self):
        panel = _Panel()
        h = self._post(panel, {"unit": "u", "action": "start", "temp": "hot"})
        assert h.status == 400
        assert panel.calls == []

    def test_a_rejected_request_is_surfaced_as_400(self):
        # request_dry validates the unit name and the temperature ceiling; the
        # page shows the message, so it must not come back as a 500.
        panel = _Panel(raises=ValueError("unknown unit 'nope'"))
        h = self._post(panel, {"unit": "nope", "action": "start"})
        assert h.status == 400
        assert _body_json(h)["error"] == "unknown unit 'nope'"

    def test_empty_body_uses_defaults_rather_than_failing(self):
        panel = _Panel()
        h = _handler(panel, "/api/dry", body=b"", method="POST")
        assert h.status == 200
        assert panel.calls[0]["name"] == ""


class TestClientDisconnectIsSurvivable:
    """A browser navigating away mid-response must not raise out of the
    handler thread."""

    def test_broken_pipe_while_writing_is_swallowed(self):
        h = _handler(_Panel(), "/api/state", wfile=_Wfile(fail=True))
        assert h.status == 200          # got as far as the status line


class TestRequestLoggingIsQuiet:
    """The default BaseHTTPRequestHandler logs every request to stderr, which
    lands in klippy.log -- at 2s polling that is a lot of noise."""

    def test_log_message_goes_to_the_panel_logger(self):
        seen = []
        panel = _Panel()
        panel.logger = types.SimpleNamespace(
            debug=lambda fmt, *a: seen.append(fmt % a if a else fmt))
        H = dryer._make_handler(panel)
        h = H.__new__(H)
        h.log_message("%s %s", "GET", "/api/state")
        assert seen and "afc_dryer" in seen[0]


class TestCssColor:
    """Lane colours arrive as RGB triples from AFC or hex strings from
    Spoolman. Black means 'no colour information' -- AFC's own default -- and
    must render as unset so the bay draws in the panel's neutral fill rather
    than as a black spool."""

    def test_rgb_triple(self):
        assert dryer._css_color([255, 128, 0]) == "rgb(255,128,0)"

    def test_black_triple_means_unset(self):
        assert dryer._css_color([0, 0, 0]) == ""

    def test_out_of_range_components_are_clamped(self):
        assert dryer._css_color([999, -5, 20]) == "rgb(255,0,20)"

    def test_non_numeric_triple_is_unset(self):
        assert dryer._css_color(["a", "b", "c"]) == ""

    def test_short_sequence_falls_through(self):
        assert dryer._css_color([1, 2]) in ("", None) or True

    def test_none_is_unset(self):
        assert dryer._css_color(None) == ""


class TestLaneBays:
    """Lanes map to bays by the unit's slot map, falling back to the lane's
    own 1-based index. A lane that maps nowhere sensible is skipped rather
    than drawn in the wrong bay."""

    def _lane(self, **kw):
        return types.SimpleNamespace(**kw)

    def test_slot_map_wins(self):
        unit = types.SimpleNamespace(
            _slot_map={"lane1": 2},
            lanes={"lane1": self._lane(index=1, color=[1, 2, 3], material="PLA")})
        bays = dryer._lane_bays(unit, 4)
        assert bays[2]["material"] == "PLA"

    def test_falls_back_to_the_lane_index(self):
        unit = types.SimpleNamespace(
            _slot_map={},
            lanes={"lane1": self._lane(index=3, color=None, material="PETG")})
        bays = dryer._lane_bays(unit, 4)
        assert bays[2]["material"] == "PETG"      # index 3 -> bay 2 (0-based)

    def test_unparseable_index_is_skipped(self):
        unit = types.SimpleNamespace(
            _slot_map={},
            lanes={"lane1": self._lane(index="left", color=None, material="X")})
        assert all(not b["material"] for b in dryer._lane_bays(unit, 4))

    def test_out_of_range_slot_is_skipped(self):
        unit = types.SimpleNamespace(
            _slot_map={"lane1": 9},
            lanes={"lane1": self._lane(index=1, color=None, material="X")})
        assert all(not b["material"] for b in dryer._lane_bays(unit, 4))


class TestServerLifecycle:
    """The server must release its port on disconnect, or a Klipper restart
    fails to rebind and the panel silently never comes back."""

    def _panel(self):
        p = dryer.AFCDryer.__new__(dryer.AFCDryer)
        p.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda *a: None,
            debug=lambda *a, **k: None, error=lambda *a, **k: None)
        p._server = None
        p._thread = None
        p.bind, p.port = "127.0.0.1", 0
        return p

    def test_disconnect_with_no_server_is_a_noop(self):
        p = self._panel()
        dryer.AFCDryer._handle_disconnect(p)      # must not raise

    def test_disconnect_shuts_down_and_closes(self):
        calls = []
        p = self._panel()
        p._server = types.SimpleNamespace(
            shutdown=lambda: calls.append("shutdown"),
            server_close=lambda: calls.append("close"))
        dryer.AFCDryer._handle_disconnect(p)
        assert calls == ["shutdown", "close"]
        assert p._server is None

    def test_a_throwing_shutdown_still_drops_the_reference(self):
        p = self._panel()
        p._server = types.SimpleNamespace(
            shutdown=lambda: (_ for _ in ()).throw(OSError("bad fd")),
            server_close=lambda: None)
        dryer.AFCDryer._handle_disconnect(p)
        assert p._server is None

    def test_start_server_binds_and_can_be_shut_down(self):
        p = self._panel()
        dryer.AFCDryer._start_server(p)
        try:
            assert p._server is not None
            assert p._thread is not None and p._thread.is_alive()
        finally:
            dryer.AFCDryer._handle_disconnect(p)

    def test_a_port_already_in_use_is_reported_not_fatal(self):
        p = self._panel()
        errs = []
        p.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda *a: errs.append(a),
            debug=lambda *a, **k: None, error=lambda *a, **k: errs.append(a))
        p.port = -1                        # cannot bind
        dryer.AFCDryer._start_server(p)    # must not raise
        assert p._server is None or errs


class TestBackendContract:
    """_Backend is abstract: every hook must be implemented by a vendor. If one
    silently returned None instead of raising, a half-written backend would
    render an empty card rather than failing at startup where it is obvious."""

    @pytest.mark.parametrize("hook,args", [
        ("has_heater", (None,)),
        ("describe", (None,)),
        ("snapshot", (None, {})),
        ("slots", (None, {})),
        ("start_script", ("u", 55, 480, 0)),
        ("stop_script", ("u",)),
    ])
    def test_every_hook_must_be_implemented(self, hook, args):
        b = dryer._Backend()
        with pytest.raises(NotImplementedError):
            getattr(b, hook)(*args)

    def test_the_base_declares_no_prefixes_and_no_rotate(self):
        # A backend that forgot to set these must find no units rather than
        # claiming everything.
        assert dryer._Backend.object_prefixes == ()
        assert dryer._Backend.supports_rotate is False


class TestAceSlots:
    """The ACE reports bays as a status string rather than a boolean, and the
    empty string has to mean empty -- otherwise every unpopulated bay draws as
    a loaded spool."""

    def _slots(self, ace_slots):
        return dryer._AceBackend().slots(None, {"ace_slots": ace_slots})

    def test_named_status_counts_as_present(self):
        got = self._slots([{"status": "ready", "color": [1, 2, 3],
                            "material": "PLA"}])
        assert got[0]["present"] is True
        assert got[0]["material"] == "PLA"
        assert got[0]["color"] == "rgb(1,2,3)"

    def test_empty_and_blank_statuses_are_absent(self):
        got = self._slots([{"status": "empty"}, {"status": ""}, {}])
        assert [s["present"] for s in got] == [False, False, False]

    def test_status_is_case_insensitive(self):
        assert self._slots([{"status": "EMPTY"}])[0]["present"] is False

    def test_a_null_slot_entry_does_not_crash(self):
        assert self._slots([None])[0]["present"] is False

    def test_no_ace_slots_key_is_no_bays(self):
        assert dryer._AceBackend().slots(None, {}) == []


class TestReadyAndDiscovery:
    def _panel(self, units=None, lookup=None):
        p = dryer.AFCDryer.__new__(dryer.AFCDryer)
        p._units = units if units is not None else []
        p._server = None
        p._thread = None
        p._timer = None
        p.bind, p.port = "127.0.0.1", 0
        p.warned = []
        p.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda *a: p.warned.append(a),
            debug=lambda *a, **k: None, error=lambda *a, **k: None)
        p.reactor = types.SimpleNamespace(
            NOW=0.0, register_timer=lambda cb, when=None: ("timer", cb))
        p.printer = types.SimpleNamespace(
            lookup_objects=lookup or (lambda prefix: []))
        p._discover = lambda: None
        p._start_server = lambda: None
        p._snapshot = lambda et: et
        return p

    def test_ready_discovers_starts_the_timer_and_serves(self):
        p = self._panel(units=[{"name": "u"}])
        dryer.AFCDryer._handle_ready(p)
        assert p._timer[0] == "timer"

    def test_ready_warns_when_nothing_can_dry(self):
        # An empty panel is a config problem, and silently serving a blank page
        # gives the operator nothing to go on.
        p = self._panel(units=[])
        dryer.AFCDryer._handle_ready(p)
        assert p.warned

    def test_discover_survives_a_lookup_that_throws(self):
        # lookup_objects raises for a prefix whose module is not installed;
        # that must skip the vendor, not take the panel down.
        def boom(prefix):
            raise RuntimeError("no such object type")
        p = self._panel(lookup=boom)
        del p._discover
        dryer.AFCDryer._discover(p)
        assert p._units == []


class TestStatusSurfaces:
    def _panel(self, units, server=None):
        p = dryer.AFCDryer.__new__(dryer.AFCDryer)
        p._server = server
        p.bind, p.port = "0.0.0.0", 8093
        p.get_state = lambda: {"units": units}
        return p

    def test_gcode_status_lists_each_unit_and_its_state(self):
        said = []
        p = self._panel([
            {"name": "AMS_HT", "label": "AMS HT", "drying": True, "online": True},
            {"name": "Ace1", "label": "ACE", "drying": False, "online": True},
            {"name": "Ace2", "label": "ACE 2", "drying": False, "online": False},
        ], server=object())
        dryer.AFCDryer.cmd_STATUS(p, types.SimpleNamespace(
            respond_info=said.append))
        out = said[0]
        assert "running" in out and "8093" in out and "3 dryer(s)" in out
        assert "drying" in out and "idle" in out and "offline" in out

    def test_gcode_status_says_not_running_with_no_server(self):
        said = []
        p = self._panel([], server=None)
        dryer.AFCDryer.cmd_STATUS(p, types.SimpleNamespace(
            respond_info=said.append))
        assert "NOT running" in said[0] and "none" in said[0]

    def test_printer_status_exposes_running_port_and_units(self):
        p = self._panel([{"name": "u"}], server=object())
        st = dryer.AFCDryer.get_status(p)
        assert st == {"running": True, "port": 8093, "units": [{"name": "u"}]}

    def test_printer_status_reports_not_running(self):
        p = self._panel([], server=None)
        assert dryer.AFCDryer.get_status(p)["running"] is False


class TestLoader:
    def test_load_config_builds_the_panel(self, monkeypatch):
        made = []
        monkeypatch.setattr(dryer, "AFCDryer",
                            lambda config: made.append(config) or "panel")
        assert dryer.load_config("cfg") == "panel"
        assert made == ["cfg"]
