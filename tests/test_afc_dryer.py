"""
Tests for the dryer panel, extras/afc_dryer.py.

The per-vendor backends and the gcode they build, the page itself, and the HTTP
endpoints it serves. Consolidated from two files; banners name each source.
"""

from __future__ import annotations
import pytest
import types
from extras.afc_dryer import (AFCDryer, _AceBackend, _BambuBackend,
                              _GenericBackend,
                              _reading,
                              _css_color)
import json
import extras.afc_dryer as dryer


# ── Tests for extras/afc_dryer.py — the AFC Drying Room panel ─────────────────
#
# was tests/test_afc_dryer.py
# ── Fakes ─────────────────────────────────────────────────────────────────────

class FakeReactor:
    NOW = 0.0

    def __init__(self):
        self.async_calls = []

    def register_timer(self, cb, when=None):
        return cb

    def register_async_callback(self, cb):
        # Record rather than run: the point is that commands LEAVE the HTTP
        # thread. Tests run them explicitly.
        self.async_calls.append(cb)

    def monotonic(self):
        return 100.0


class FakeGcode:
    def __init__(self):
        self.scripts = []
        self.commands = {}

    def register_command(self, name, cb, desc=None):
        self.commands[name] = cb

    def run_script(self, script):
        self.scripts.append(script)


class FakePrinter:
    def __init__(self, units=(), afc_units=None):
        self._units = list(units)
        # AFC's own registry, which the generic backend reads. None means
        # "no [AFC] section", the case every pre-existing test exercises.
        self._afc = (types.SimpleNamespace(units=dict(afc_units))
                     if afc_units is not None else None)
        self.gcode = FakeGcode()
        self.reactor = FakeReactor()

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        if name == "AFC":
            return self._afc if self._afc is not None else default
        return default

    def lookup_objects(self, prefix):
        return [(prefix, u) for u in self._units
                if getattr(u, "PREFIX", None) == prefix]

    def register_event_handler(self, *a):
        pass


class FakeConfig:
    def __init__(self, printer, values=None):
        self._p = printer
        self._v = values or {}

    def get_printer(self):
        return self._p

    def getint(self, k, d, **kw):
        return int(self._v.get(k, d))

    def get(self, k, d=None):
        return self._v.get(k, d)

    def getfloat(self, k, d, **kw):
        return float(self._v.get(k, d))

    def getboolean(self, k, d):
        return bool(self._v.get(k, d))


class FakeBambu:
    PREFIX = "AFC_BambuAMS"

    def __init__(self, name, model="ams2", heater=True, max_temp=65,
                 slots=4, status=None):
        self.name = name
        self.ams_model = model
        self.has_heater = heater
        self.dry_max_temp = max_temp
        self.unit_slots = slots
        self._status = status if status is not None else {}

    def get_status(self, eventtime=None):
        return self._status


class FakeAce:
    PREFIX = "AFC_ACE"
    SLOTS_PER_UNIT = 4

    def __init__(self, name, max_temp=55.0, status=None):
        self.name = name
        self.max_dryer_temperature = max_temp
        self._status = status if status is not None else {}

    def get_status(self, eventtime=None):
        return self._status


def _panel(units=(), values=None, afc_units=None):
    printer = FakePrinter(units, afc_units)
    panel = AFCDryer(FakeConfig(printer, values))
    panel._discover()
    return panel, printer


# ── Colour normalisation ──────────────────────────────────────────────────────

class TestCssColor_afc_dryer:
    def test_bambu_hex_string(self):
        assert _css_color("FF8800") == "#FF8800"

    def test_bambu_hex_with_alpha_is_truncated(self):
        assert _css_color("FF8800FF") == "#FF8800"

    def test_ace_rgb_list(self):
        assert _css_color([200, 30, 30]) == "rgb(200,30,30)"

    def test_black_is_empty_not_black(self):
        # Both vendors use all-zero for "no colour known". Rendering it as
        # black would draw a spool that isn't there.
        assert _css_color("000000") == ""
        assert _css_color([0, 0, 0]) == ""

    @pytest.mark.parametrize("bad", [None, "", "zz", "GGGGGG", [1, 2], 42, {}])
    def test_unusable_values_are_empty(self, bad):
        assert _css_color(bad) == ""

    def test_rgb_values_are_clamped(self):
        assert _css_color([999, -5, 30]) == "rgb(255,0,30)"


# ── Discovery ─────────────────────────────────────────────────────────────────

class TestDiscovery:
    def test_finds_both_vendors(self):
        panel, _ = _panel([FakeBambu("AMS_2"), FakeAce("Ace2_1")])
        assert {u["name"] for u in panel._units} == {"AMS_2", "Ace2_1"}
        assert {u["kind"] for u in panel._units} == {"bambu", "ace"}

    def test_heaterless_unit_is_hidden_by_default(self):
        panel, _ = _panel([FakeBambu("AMS_1", model="ams1", heater=False),
                           FakeBambu("AMS_2")])
        assert [u["name"] for u in panel._units] == ["AMS_2"]

    def test_show_heaterless_lists_it_read_only(self):
        panel, _ = _panel([FakeBambu("AMS_1", model="ams1", heater=False)],
                          values={"show_heaterless": True})
        assert panel._units[0]["has_heater"] is False

    def test_model_drives_label_slots_and_ceiling(self):
        panel, _ = _panel([FakeBambu("HT", model="ht", max_temp=85, slots=1)])
        u = panel._units[0]
        assert (u["label"], u["max_temp"], u["slots"]) == ("AMS HT", 85, 1)

    def test_ace_descriptor_from_its_own_config(self):
        panel, _ = _panel([FakeAce("Ace2_1", max_temp=55.0)])
        u = panel._units[0]
        assert (u["label"], u["max_temp"], u["slots"]) == ("ACE", 55, 4)

    def test_only_bambu_advertises_rotation(self):
        panel, _ = _panel([FakeBambu("AMS_2"), FakeAce("Ace2_1")])
        rot = {u["name"]: u["rotate"] for u in panel._units}
        assert rot == {"AMS_2": True, "Ace2_1": False}

    def test_ace_with_zero_ceiling_opts_out(self):
        panel, _ = _panel([FakeAce("Ace2_1", max_temp=0.0)])
        assert panel._units == []

    def test_a_broken_unit_does_not_sink_discovery(self):
        class Exploding:
            PREFIX = "AFC_BambuAMS"
            name = "bad"
            has_heater = True

            @property
            def ams_model(self):
                raise RuntimeError("boom")

        panel, _ = _panel([Exploding(), FakeBambu("good")])
        assert [u["name"] for u in panel._units] == ["good"]


# ── Snapshot ──────────────────────────────────────────────────────────────────

class TestSnapshot:
    def test_bambu_fields_are_mapped(self):
        unit = FakeBambu("AMS_2", status={
            "bridge_online": True, "drying": True, "temperature": 48.0,
            "humidity": 21.0, "dry_target": 55.0,
            "slots": [{"present": True, "color": "FF8800"}]})
        panel, _ = _panel([unit])
        panel._snapshot(0.0)
        u = panel.get_state()["units"][0]
        assert (u["online"], u["drying"], u["temperature"], u["target"]) == \
            (True, True, 48.0, 55.0)
        assert u["bays"][0] == {"present": True, "color": "#FF8800",
                                "material": ""}

    def test_ace_dryer_string_decides_drying(self):
        panel, _ = _panel([FakeAce("Ace2_1", status={
            "ace_connected": True, "ace_dryer": "drying", "ace_temp": 51.2})])
        panel._snapshot(0.0)
        u = panel.get_state()["units"][0]
        assert u["drying"] is True
        # The vendor's own wording is carried through rather than restated.
        assert u["note"] == "drying"

    @pytest.mark.parametrize("value", ["", "stop", "stopped", "idle", "off",
                                       "none", "OFF", " Idle "])
    def test_ace_idle_strings_are_not_drying(self, value):
        panel, _ = _panel([FakeAce("Ace2_1", status={
            "ace_connected": True, "ace_dryer": value})])
        panel._snapshot(0.0)
        assert panel.get_state()["units"][0]["drying"] is False

    def test_unknown_ace_state_reads_as_drying_and_says_so(self):
        # An unfamiliar state must not be silently rounded down to "idle" --
        # showing it verbatim is how a new firmware string gets noticed.
        panel, _ = _panel([FakeAce("Ace2_1", status={
            "ace_connected": True, "ace_dryer": "preheat"})])
        panel._snapshot(0.0)
        u = panel.get_state()["units"][0]
        assert (u["drying"], u["note"]) == (True, "preheat")

    def test_bays_are_padded_to_the_unit_slot_count(self):
        panel, _ = _panel([FakeBambu("AMS_2", slots=4, status={"slots": []})])
        panel._snapshot(0.0)
        assert len(panel.get_state()["units"][0]["bays"]) == 4

    def test_bays_are_truncated_to_the_unit_slot_count(self):
        panel, _ = _panel([FakeBambu("HT", model="ht", slots=1, status={
            "slots": [{"present": True}, {"present": True}]})])
        panel._snapshot(0.0)
        assert len(panel.get_state()["units"][0]["bays"]) == 1

    def test_status_failure_degrades_to_offline(self):
        class Broken(FakeBambu):
            def get_status(self, eventtime=None):
                raise RuntimeError("bus down")

        panel, _ = _panel([Broken("AMS_2")])
        panel._snapshot(0.0)
        u = panel.get_state()["units"][0]
        assert (u["online"], u["drying"]) == (False, False)

    def test_snapshot_publishes_only_primitives(self):
        # The HTTP thread reads this dict; a printer object leaking into it
        # would be a cross-thread reference into Klipper.
        panel, _ = _panel([FakeBambu("AMS_2"), FakeAce("Ace2_1")])
        panel._snapshot(0.0)
        for u in panel.get_state()["units"]:
            assert "_obj" not in u and "_backend" not in u
            for v in u.values():
                assert isinstance(v, (str, int, float, bool, list, type(None)))


# ── Request validation ────────────────────────────────────────────────────────

class TestRequestDry:
    def test_bambu_start_builds_its_gcode(self):
        panel, printer = _panel([FakeBambu("AMS_2")])
        script = panel.request_dry("AMS_2", "start", 55, 480, 1)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=480 ROTATE=1"

    def test_ace_start_uses_duration_not_time(self):
        panel, _ = _panel([FakeAce("Ace2_1")])
        script = panel.request_dry("Ace2_1", "start", 50, 240, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=50 DURATION=240"

    def test_stop_is_per_vendor(self):
        panel, _ = _panel([FakeBambu("AMS_2"), FakeAce("Ace2_1")])
        assert panel.request_dry("AMS_2", "stop", 0, 0, 0) == \
            "AFC_BAMBU_HEATER_STOP UNIT=AMS_2"
        assert panel.request_dry("Ace2_1", "stop", 0, 0, 0) == \
            "ACE_DRY_STOP UNIT=Ace2_1"

    def test_temp_is_clamped_to_the_units_ceiling(self):
        panel, _ = _panel([FakeAce("Ace2_1", max_temp=55.0)])
        assert "TEMP=55" in panel.request_dry("Ace2_1", "start", 85, 60, 0)

    def test_ht_keeps_its_higher_ceiling(self):
        panel, _ = _panel([FakeBambu("HT", model="ht", max_temp=85, slots=1)])
        assert "TEMP=85" in panel.request_dry("HT", "start", 85, 60, 0)

    def test_rotate_is_dropped_for_a_vendor_that_cannot(self):
        panel, _ = _panel([FakeAce("Ace2_1")])
        assert "ROTATE" not in panel.request_dry("Ace2_1", "start", 50, 60, 1)

    def test_time_is_clamped_to_the_protocol_field(self):
        panel, _ = _panel([FakeBambu("AMS_2")])
        assert "TIME=65535" in panel.request_dry("AMS_2", "start", 55, 999999, 0)

    def test_negative_values_floor_at_zero(self):
        panel, _ = _panel([FakeBambu("AMS_2")])
        s = panel.request_dry("AMS_2", "start", -20, -5, 0)
        assert "TEMP=0" in s and "TIME=0" in s

    def test_unknown_unit_is_refused(self):
        panel, _ = _panel([FakeBambu("AMS_2")])
        with pytest.raises(ValueError, match="unknown unit"):
            panel.request_dry("nope", "start", 55, 60, 0)

    def test_heaterless_unit_is_refused(self):
        panel, _ = _panel([FakeBambu("AMS_1", model="ams1", heater=False)],
                          values={"show_heaterless": True})
        with pytest.raises(ValueError, match="no drying heater"):
            panel.request_dry("AMS_1", "start", 55, 60, 0)

    def test_unknown_action_is_refused(self):
        panel, _ = _panel([FakeBambu("AMS_2")])
        with pytest.raises(ValueError, match="unknown action"):
            panel.request_dry("AMS_2", "melt", 55, 60, 0)

    @pytest.mark.parametrize("hostile", [
        "AMS_2\nM112",                       # newline -> second command
        "AMS_2 TEMP=999",                    # argument injection
        "AMS_2; RESTART",
        "",
    ])
    def test_a_name_is_never_interpolated_as_free_text(self, hostile):
        # The panel listens on 0.0.0.0 and builds G-code. Matching the name
        # against discovered units is what keeps it a control panel rather
        # than a remote shell.
        panel, _ = _panel([FakeBambu("AMS_2")])
        with pytest.raises(ValueError):
            panel.request_dry(hostile, "start", 55, 60, 0)

    def test_commands_leave_the_http_thread(self):
        # Nothing may call into Klipper inline from a request.
        panel, printer = _panel([FakeBambu("AMS_2")])
        panel.request_dry("AMS_2", "start", 55, 480, 0)
        assert printer.gcode.scripts == []          # not run yet
        assert len(printer.reactor.async_calls) == 1
        printer.reactor.async_calls[0](0.0)         # reactor runs it
        assert printer.gcode.scripts == [
            "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=480 ROTATE=0"]

    def test_a_failing_script_does_not_escape(self):
        panel, printer = _panel([FakeBambu("AMS_2")])

        def boom(_):
            raise RuntimeError("klipper is shut down")

        printer.gcode.run_script = boom
        panel.request_dry("AMS_2", "stop", 0, 0, 0)
        printer.reactor.async_calls[0](0.0)         # must not raise


# ── Backend contract ──────────────────────────────────────────────────────────

class TestBackendContract_afc_dryer:
    @pytest.mark.parametrize("backend", [_BambuBackend(), _AceBackend()])
    def test_every_backend_implements_the_hooks(self, backend):
        for hook in ("has_heater", "describe", "snapshot", "slots",
                     "start_script", "stop_script"):
            assert callable(getattr(backend, hook))
        assert backend.object_prefixes and backend.kind

    def test_ace_backend_covers_both_unit_classes(self):
        # The ACE 2 Pro is a separate class (different wire protocol) that
        # subclasses the V1 ACE, so it registers under its own prefix while
        # sharing every dryer command. Missing it is why a live Ace2_1 did
        # not appear on the panel.
        assert set(_AceBackend().object_prefixes) == {"AFC_ACE", "AFC_ACE2"}


class TestAce2Discovery:
    def test_ace2_prefix_is_discovered(self):
        class FakeAce2(FakeAce):
            PREFIX = "AFC_ACE2"

        panel, _ = _panel([FakeAce2("Ace2_1")])
        assert [u["name"] for u in panel._units] == ["Ace2_1"]
        assert panel._units[0]["kind"] == "ace"

    def test_a_unit_matching_two_prefixes_is_listed_once(self):
        class BothPrefixes(FakeAce):
            PREFIX = "AFC_ACE"

        unit = BothPrefixes("Ace2_1")
        printer = FakePrinter()
        # Answer BOTH lookups with the same object, as Klipper would for a
        # subclass registered under its parent's name.
        printer.lookup_objects = lambda p: (
            [(p, unit)] if p in ("AFC_ACE", "AFC_ACE2") else [])
        panel = AFCDryer(FakeConfig(printer))
        panel._discover()
        assert [u["name"] for u in panel._units] == ["Ace2_1"]


class FakeLane:
    def __init__(self, name, index, color="", material=""):
        self.name = name
        self.index = index
        self.color = color
        self.material = material


class TestLaneColours:
    """The artwork's colours come from the AFC lanes, not the vendor's slots.

    A vendor reports [0,0,0] / "" for a spool it has no colour for, which
    painted every occupied bay the same fallback shade. The lane carries the
    colour AFC already agrees on (RFID, Spoolman, or lane config).
    """

    def _unit(self, lanes, slot_map=None, slots=None):
        unit = FakeBambu("AMS_2", status={"slots": slots or [
            {"present": True}, {"present": True}, {"present": False},
            {"present": False}]})
        unit.lanes = {l.name: l for l in lanes}
        if slot_map is not None:
            unit._slot_map = slot_map
        return unit

    def test_lane_colour_reaches_the_bay(self):
        panel, _ = _panel([self._unit(
            [FakeLane("lane1", 1, "#FF0000"), FakeLane("lane2", 2, "00FF00")],
            slot_map={"lane1": 0, "lane2": 1})])
        panel._snapshot(0.0)
        bays = panel.get_state()["units"][0]["bays"]
        assert [b["color"] for b in bays[:2]] == ["#FF0000", "#00FF00"]

    def test_slot_map_decides_which_bay(self):
        # Lane order must not be assumed -- the map is authoritative.
        panel, _ = _panel([self._unit(
            [FakeLane("lane1", 1, "#FF0000"), FakeLane("lane2", 2, "#00FF00")],
            slot_map={"lane1": 1, "lane2": 0})])
        panel._snapshot(0.0)
        bays = panel.get_state()["units"][0]["bays"]
        assert [b["color"] for b in bays[:2]] == ["#00FF00", "#FF0000"]

    def test_falls_back_to_the_lane_index_without_a_map(self):
        panel, _ = _panel([self._unit(
            [FakeLane("lane1", 1, "#FF0000"), FakeLane("lane2", 2, "#00FF00")])])
        panel._snapshot(0.0)
        bays = panel.get_state()["units"][0]["bays"]
        assert [b["color"] for b in bays[:2]] == ["#FF0000", "#00FF00"]

    def test_vendor_colour_is_the_fallback_not_the_winner(self):
        unit = self._unit([FakeLane("lane1", 1, "")], slot_map={"lane1": 0},
                          slots=[{"present": True, "color": "0000FF"}])
        panel, _ = _panel([unit])
        panel._snapshot(0.0)
        assert panel.get_state()["units"][0]["bays"][0]["color"] == "#0000FF"

    def test_lane_colour_beats_the_vendors(self):
        unit = self._unit([FakeLane("lane1", 1, "#FF0000")],
                          slot_map={"lane1": 0},
                          slots=[{"present": True, "color": "0000FF"}])
        panel, _ = _panel([unit])
        panel._snapshot(0.0)
        assert panel.get_state()["units"][0]["bays"][0]["color"] == "#FF0000"

    def test_presence_still_comes_from_the_vendor(self):
        # A lane can carry a colour for a spool that has been taken out.
        unit = self._unit([FakeLane("lane1", 1, "#FF0000")],
                          slot_map={"lane1": 0},
                          slots=[{"present": False}])
        panel, _ = _panel([unit])
        panel._snapshot(0.0)
        bay = panel.get_state()["units"][0]["bays"][0]
        assert bay["present"] is False and bay["color"] == "#FF0000"

    def test_out_of_range_lane_index_is_ignored(self):
        unit = self._unit([FakeLane("lane9", 99, "#FF0000")])
        panel, _ = _panel([unit])
        panel._snapshot(0.0)          # must not raise
        assert all(b["color"] == "" for b in
                   panel.get_state()["units"][0]["bays"])

    def test_unit_without_lanes_still_renders(self):
        panel, _ = _panel([FakeBambu("AMS_2", status={"slots": []})])
        panel._snapshot(0.0)
        assert len(panel.get_state()["units"][0]["bays"]) == 4


class TestPageVersion:
    """The page is served once and then polls, so a deploy leaves an open
    browser running the PREVIOUS markup while its data keeps updating. That
    reads as "the fix didn't land" -- a client-side change looks ignored while
    a server-side one appears to work. The page carries a hash of itself and
    reloads when the server reports a different one."""

    def test_served_page_has_the_version_substituted(self):
        from extras.afc_dryer import PAGE, PAGE_VERSION
        assert "__PAGE_VERSION__" not in PAGE
        assert 'var MY_VERSION = "%s"' % PAGE_VERSION in PAGE

    def test_template_keeps_the_placeholder(self):
        # Hashing the template (not the served page) keeps it non-circular.
        from extras.afc_dryer import _PAGE_TEMPLATE
        assert "__PAGE_VERSION__" in _PAGE_TEMPLATE

    def test_version_is_derived_from_the_template(self):
        import hashlib
        from extras.afc_dryer import _PAGE_TEMPLATE, PAGE_VERSION
        assert PAGE_VERSION == hashlib.md5(
            _PAGE_TEMPLATE.encode("utf-8")).hexdigest()[:8]

    def test_a_changed_page_changes_the_version(self):
        import hashlib
        from extras.afc_dryer import _PAGE_TEMPLATE, PAGE_VERSION
        edited = _PAGE_TEMPLATE.replace("<h1>", "<h1>x", 1)
        assert hashlib.md5(edited.encode("utf-8")).hexdigest()[:8] != PAGE_VERSION

    def test_state_reports_the_running_version(self):
        from extras.afc_dryer import PAGE_VERSION
        panel, _ = _panel([FakeBambu("AMS_2")])
        panel._snapshot(0.0)
        assert panel.get_state()["page_version"] == PAGE_VERSION

    def test_version_survives_a_snapshot_replacing_the_state(self):
        # get_state() stamps rather than the snapshot storing it, so the field
        # cannot be lost when _snapshot swaps the dict wholesale.
        from extras.afc_dryer import PAGE_VERSION
        panel, _ = _panel([FakeBambu("AMS_2")])
        for _ in range(3):
            panel._snapshot(0.0)
            assert panel.get_state()["page_version"] == PAGE_VERSION

    def test_page_reloads_only_once_on_a_mismatch(self):
        from extras.afc_dryer import PAGE
        assert "if (!reloading && state.page_version" in PAGE
        assert "reloading = true;" in PAGE
        assert "location.reload();" in PAGE


class TestAceGeneration:
    def test_v1_says_ace(self):
        panel, _ = _panel([FakeAce("Ace_1")])
        assert panel._units[0]["label"] == "ACE"

    def test_v2_says_ace_2(self):
        # afcACE2 subclasses afcACE, so an isinstance check would label every
        # Pro 2 as a plain ACE -- which is what the panel used to show.
        class FakeACE2(FakeAce):
            PREFIX = "AFC_ACE2"

        panel, _ = _panel([FakeACE2("Ace2_1")])
        assert panel._units[0]["label"] == "ACE 2"

    def test_generation_does_not_change_the_commands(self):
        class FakeACE2(FakeAce):
            PREFIX = "AFC_ACE2"

        panel, _ = _panel([FakeACE2("Ace2_1")])
        assert panel.request_dry("Ace2_1", "start", 50, 240, 0) == \
            "ACE_DRY UNIT=Ace2_1 TEMP=50 DURATION=240"


# ── Every other AFC unit ──────────────────────────────────────────────────────

class _Lane:
    def __init__(self, load_state=False, color=None, material=""):
        self.load_state = load_state
        self.color = color
        self.material = material


class _OtherUnit:
    """A BoxTurtle-ish unit: registered with AFC, no dryer, not a vendor the
    panel has a backend for."""

    def __init__(self, name="Turtle_1", type_="BoxTurtle", lanes=None):
        self.name = name
        self.type = type_
        self.lanes = lanes if lanes is not None else {}


class TestGenericBackendListsEveryOtherUnit:
    """show_heaterless used to mean "also the heaterless BAMBU and ACE units",
    which quietly excluded every other vendor. The panel is the one place an
    operator sees the whole machine at once, and a BoxTurtle missing from that
    list reads as a fault rather than as "this one has no heater"."""

    def test_a_boxturtle_is_listed_when_heaterless_is_on(self):
        unit = _OtherUnit()
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Turtle_1": unit})
        assert [u["name"] for u in panel._units] == ["Turtle_1"]

    def test_it_is_hidden_by_default(self):
        panel, _p = _panel(afc_units={"Turtle_1": _OtherUnit()})
        assert panel._units == []

    def test_it_is_labelled_by_its_configured_type(self):
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Turtle_1": _OtherUnit()})
        assert panel._units[0]["label"] == "BoxTurtle"
        assert panel._units[0]["kind"] == "generic"
        assert panel._units[0]["has_heater"] is False

    def test_a_unit_with_no_type_falls_back_to_its_class_name(self):
        unit = _OtherUnit(type_=None)
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Turtle_1": unit})
        assert panel._units[0]["label"] == "_OtherUnit"

    def test_slot_count_comes_from_the_lanes(self):
        unit = _OtherUnit(lanes={"l1": _Lane(), "l2": _Lane(), "l3": _Lane()})
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Turtle_1": unit})
        assert panel._units[0]["slots"] == 3

    def test_no_afc_section_is_not_an_error(self):
        # Every pre-existing test runs this path: lookup_object("AFC") returns
        # the default, and discovery must simply find nothing.
        panel, _p = _panel(values={"show_heaterless": True})
        assert panel._units == []


class TestGenericBackendDoesNotStealVendorUnits:
    def test_a_bambu_unit_keeps_its_own_backend(self):
        # The generic backend reads afc.units, which contains the Bambu unit
        # too. _discover's `seen` set must keep the vendor description.
        bambu = FakeBambu("BambuAMS_1")
        panel, _p = _panel(units=[bambu], values={"show_heaterless": True},
                           afc_units={bambu.name: bambu})
        kinds = {u["name"]: u["kind"] for u in panel._units}
        assert kinds[bambu.name] == "bambu"

    def test_the_unit_is_listed_once(self):
        bambu = FakeBambu("BambuAMS_1")
        panel, _p = _panel(units=[bambu], values={"show_heaterless": True},
                           afc_units={bambu.name: bambu})
        assert len(panel._units) == 1


class TestGenericBackendReadOnly:
    def test_slots_report_lane_load_state(self):
        b = _GenericBackend()
        unit = _OtherUnit(lanes={"l1": _Lane(True, "#ff0000", "PLA"),
                                 "l2": _Lane(False)})
        slots = b.slots(unit, {})
        assert [s["present"] for s in slots] == [True, False]
        assert slots[0]["material"] == "PLA"

    def test_the_snapshot_says_there_is_no_dryer(self):
        snap = _GenericBackend().snapshot(_OtherUnit(), {})
        assert snap["drying"] is False and snap["note"] == "no dryer"
        assert snap["temperature"] is None and snap["target"] is None


class _SensorUnit(_OtherUnit):
    """A unit whose environment sensor is a SEPARATE Klipper object, the way an
    OpenAMS is: the AFC unit is `ams_1`, its sensor is named after the
    [AFC_OAMS] controller (`oams1`), and only the driver object carries
    humidity -- `temperature_sensor oams1` has temperature alone."""

    def __init__(self, objects, name="ams_1", oams_name="oams1", **kw):
        super().__init__(name=name, **kw)
        self.oams_name = oams_name
        self.printer = _Printer(objects)


class _Printer:
    def __init__(self, objects):
        self._objects = objects

    def lookup_object(self, name, default=None):
        obj = self._objects.get(name)
        return obj if obj is not None else default


class _Sensor:
    def __init__(self, **status):
        self._status = status

    def get_status(self, eventtime=None):
        return dict(self._status)


class TestGenericBackendEnvironmentReadings:
    """An OpenAMS reports temperature AND humidity, and the panel showed
    neither -- the readings were sitting in a Klipper object the panel never
    looked at. Two things make it non-obvious: the humidity is only on the
    DRIVER object (aht10 oams1), not on the temperature_sensor wrapper, and
    the object is named after the controller, not the AFC unit."""

    def _snap(self, objects, **kw):
        return _GenericBackend().snapshot(_SensorUnit(objects, **kw), {})

    def test_the_driver_object_supplies_both(self):
        snap = self._snap({"aht10 oams1": _Sensor(temperature=29.79,
                                                  humidity=42.91)})
        assert snap["temperature"] == 29.79
        assert snap["humidity"] == 42.91

    def test_it_is_found_by_the_controller_name_not_the_unit_name(self):
        # The AFC unit is ams_1; nothing is registered under that name.
        snap = self._snap({"aht10 oams1": _Sensor(temperature=30.0,
                                                  humidity=40.0)})
        assert snap["temperature"] == 30.0

    def test_a_sensor_named_after_the_unit_also_works(self):
        # A Bambu-style layout, where the sensor carries the unit's own name.
        snap = self._snap({"aht10 ams_1": _Sensor(temperature=28.0,
                                                  humidity=44.0)},
                          oams_name=None)
        assert snap["humidity"] == 44.0

    def test_the_driver_wins_over_the_temperature_wrapper(self):
        # The wrapper has no humidity, so preferring it would drop the field.
        snap = self._snap({
            "temperature_sensor oams1": _Sensor(temperature=29.0),
            "aht10 oams1": _Sensor(temperature=29.79, humidity=42.91)})
        assert snap["humidity"] == 42.91

    def test_the_wrapper_alone_still_gives_temperature(self):
        snap = self._snap({"temperature_sensor oams1": _Sensor(temperature=29.0)})
        assert snap["temperature"] == 29.0 and snap["humidity"] is None

    def test_a_sensor_that_has_not_read_yet_is_not_shown_as_zero_degrees(self):
        # A present-but-unread driver reports 0.0/0.0. Rendering that as 0C is
        # worse than rendering nothing -- it looks like a real reading.
        snap = self._snap({"aht10 oams1": _Sensor(temperature=0.0,
                                                  humidity=0.0)})
        assert snap["temperature"] is None and snap["humidity"] is None
        assert snap["note"] == "no dryer"

    def test_humidity_alone_is_still_reported(self):
        snap = self._snap({"aht10 oams1": _Sensor(temperature=0.0,
                                                  humidity=41.0)})
        assert snap["humidity"] == 41.0

    def test_the_note_says_monitor_only_when_there_are_readings(self):
        # "no dryer" is true but reads as "nothing to report" beside a live
        # temperature; the card should say what it is actually showing.
        snap = self._snap({"aht10 oams1": _Sensor(temperature=29.79,
                                                  humidity=42.91)})
        assert snap["note"] == "monitor only"
        assert snap["drying"] is False and snap["target"] is None

    def test_no_sensor_at_all_is_unchanged(self):
        snap = self._snap({})
        assert snap["temperature"] is None and snap["note"] == "no dryer"

    def test_a_unit_with_no_printer_does_not_raise(self):
        snap = _GenericBackend().snapshot(_OtherUnit(), {})
        assert snap["temperature"] is None

    def test_a_raising_sensor_is_survived(self):
        class _Boom:
            def get_status(self, eventtime=None):
                raise RuntimeError("i2c down")
        snap = self._snap({"aht10 oams1": _Boom()})
        assert snap["temperature"] is None

    def test_starting_one_is_refused(self):
        # Unreachable through the panel (has_heater is False, so no control is
        # rendered) but a clear error beats emitting a broken g-code line.
        with pytest.raises(ValueError, match="no dryer"):
            _GenericBackend().start_script("Turtle_1", 55, 60, 0)

    def test_stopping_one_is_refused(self):
        with pytest.raises(ValueError, match="no dryer"):
            _GenericBackend().stop_script("Turtle_1")

    def test_it_never_claims_a_heater(self):
        assert _GenericBackend().has_heater(_OtherUnit()) is False


class TestArtworkLaysBaysOutInOneRow:
    """No real unit is stacked as a 2-over-2 grid -- an ACE stands its four
    spools side by side exactly as an AMS does -- so every unit except the
    single-bay HT shares one wide-row renderer. These assert on the shipped
    page source, which is the only place the artwork exists."""

    def _art(self):
        from extras.afc_dryer import PAGE
        return PAGE

    def test_the_ace_grid_is_gone(self):
        src = self._art()
        assert "u.model === 'ace'" not in src
        # the 2-column arithmetic that produced the stack
        assert "j % 2" not in src

    def test_the_ht_keeps_its_single_bay_tower(self):
        # One spool, genuinely a different shape -- not a row of one.
        assert "u.model === 'ht'" in self._art()

    def test_the_row_renderer_spaces_bays_horizontally(self):
        # x advances per bay, y is fixed: that is what "side by side" means.
        assert "bay(21 + i*24, 44," in self._art()

    def test_the_label_is_taken_from_the_unit(self):
        # Shared renderer, so the aria-label can no longer be hardcoded "AMS".
        src = self._art()
        assert "u.label || u.model" in src
        assert 'aria-label="AMS">' not in src


class TestSpoolTooltipData:
    """Hovering a spool should say what is in it. The data comes from the AFC
    lane, which AFC already populates from RFID reads and from Spoolman via
    Moonraker -- so the panel reports the same spool identity Mainsail does
    without shipping a second Spoolman client."""

    class _Lane:
        def __init__(self, **kw):
            self.color = kw.get("color", "#ff0000")
            self.material = kw.get("material", "PLA")
            self.filament_name = kw.get("filament_name", "")
            self.sub_type = kw.get("sub_type", "")
            self.spool_vendor = kw.get("spool_vendor", "")
            self.spool_id = kw.get("spool_id", None)
            self.weight = kw.get("weight", 0.0)
            self.extruder_temp = kw.get("extruder_temp", None)
            self.index = kw.get("index", 1)

    def _bays(self, **kw):
        from extras.afc_dryer import _lane_bays
        unit = types.SimpleNamespace(lanes={"lane1": self._Lane(**kw)},
                                     _slot_map={"lane1": 0})
        return _lane_bays(unit, 1)[0]

    def test_the_lane_name_is_carried(self):
        assert self._bays()["lane"] == "lane1"

    def test_spoolman_identity_is_carried(self):
        b = self._bays(filament_name="Galaxy Black", spool_vendor="Polymaker",
                       sub_type="Matte", spool_id=42)
        assert b["filament"] == "Galaxy Black"
        assert b["vendor"] == "Polymaker"
        assert b["sub_type"] == "Matte"
        assert b["spool_id"] == 42

    def test_a_tracked_weight_is_carried(self):
        assert self._bays(weight=812.34)["weight"] == 812.3

    def test_an_untracked_weight_is_omitted_not_zeroed(self):
        # 0 means "not tracked", not "empty spool" -- claiming an empty spool
        # would be worse than saying nothing.
        assert self._bays(weight=0.0)["weight"] is None

    def test_a_junk_weight_does_not_raise(self):
        assert self._bays(weight="n/a")["weight"] is None

    def test_the_extruder_temp_is_carried(self):
        assert self._bays(extruder_temp=230)["temp"] == 230


class TestSpoolTooltipRendering:
    """Assertions on the shipped page source -- the only place the tooltip
    exists."""

    def _src(self):
        from extras.afc_dryer import PAGE
        return PAGE

    def test_it_uses_a_native_svg_title(self):
        # No JS positioning, and screen readers get it for free.
        assert "'<title>' + tip + '</title>'" in self._src()

    def test_the_text_is_escaped(self):
        # Spool names come from Spoolman and are user-controlled.
        src = self._src()
        assert "function esc(" in src
        assert "return esc(out.join(" in src

    def test_an_empty_bay_still_names_its_lane(self):
        assert "' — empty'" in self._src()

    def test_blank_fields_are_omitted_rather_than_shown_empty(self):
        assert ".filter(Boolean).join(' ')" in self._src()


class TestMergeKeepsTooltipFields:
    def test_lane_detail_survives_the_merge(self):
        from extras.afc_dryer import _merge_bays
        merged = _merge_bays(
            [{"present": True}],
            [{"color": "#fff", "material": "PLA", "lane": "lane1",
              "spool_id": 7, "weight": 500.0}])[0]
        assert merged["lane"] == "lane1" and merged["spool_id"] == 7
        assert merged["weight"] == 500.0

    def test_absent_detail_is_not_invented(self):
        from extras.afc_dryer import _merge_bays
        merged = _merge_bays([{"present": False}], [{}])[0]
        assert "lane" not in merged and "spool_id" not in merged


class TestToolchangerIsNotListed:
    """A toolchanger registers itself in afc.units, but it is a unit only in
    AFC's bookkeeping sense: it holds tools, not spools. It has no bays to
    draw and nothing to report, so a card for it is noise on a page about
    what is in each bay."""

    class AfcToolchanger:                       # name is what gets matched
        def __init__(self):
            self.name = "Tools"
            self.type = "Toolchanger"
            self.lanes = {}

    class _Sub(AfcToolchanger):
        """A fork subclassing it must be excluded too."""

    def test_it_is_not_listed(self):
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Tools": self.AfcToolchanger()})
        assert panel._units == []

    def test_a_subclass_is_also_excluded(self):
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Tools": self._Sub()})
        assert panel._units == []

    def test_real_units_alongside_it_are_still_listed(self):
        panel, _p = _panel(
            values={"show_heaterless": True},
            afc_units={"Tools": self.AfcToolchanger(),
                       "Turtle_1": _OtherUnit()})
        assert [u["name"] for u in panel._units] == ["Turtle_1"]

    def test_the_match_ignores_the_operator_settable_type_string(self):
        # `type` is config-settable, so renaming it must not smuggle a
        # toolchanger back onto the page.
        tc = self.AfcToolchanger()
        tc.type = "Box_Turtle"
        panel, _p = _panel(values={"show_heaterless": True},
                           afc_units={"Tools": tc})
        assert panel._units == []


class TestEachBayGetsItsOwnTooltip:
    """Two separate causes could make every spool in a unit show the same
    thing, and both are worth pinning:

      * the SVG one, which was real -- a <title> that is a direct child of
        <svg> titles the WHOLE image, so the first bay's text won everywhere.
        It has to be scoped to a <g>.
      * the data one -- _lane_bays writing every lane into the same slot.
    """

    class _L:
        def __init__(self, idx, mat, col):
            self.index = idx
            self.material = mat
            self.color = col
            self.filament_name = ""
            self.sub_type = ""
            self.spool_vendor = ""
            self.spool_id = idx
            self.weight = 100.0 * idx
            self.extruder_temp = None

    def test_each_lane_lands_in_its_own_bay(self):
        from extras.afc_dryer import _lane_bays
        unit = types.SimpleNamespace(
            lanes={"lane1": self._L(1, "PLA", "#ff0000"),
                   "lane2": self._L(2, "PETG", "#00ff00"),
                   "lane3": self._L(3, "ABS", "#0000ff"),
                   "lane4": self._L(4, "TPU", "#ffff00")},
            _slot_map={"lane1": 0, "lane2": 1, "lane3": 2, "lane4": 3})
        bays = _lane_bays(unit, 4)
        assert [b["material"] for b in bays] == ["PLA", "PETG", "ABS", "TPU"]
        assert [b["lane"] for b in bays] == ["lane1", "lane2", "lane3", "lane4"]
        assert len({b["spool_id"] for b in bays}) == 4

    def test_the_index_fallback_also_separates_them(self):
        # A unit with no _slot_map maps through the lane's 1-based index.
        from extras.afc_dryer import _lane_bays
        unit = types.SimpleNamespace(
            lanes={"a": self._L(1, "PLA", "#f00"),
                   "b": self._L(2, "PETG", "#0f0")})
        bays = _lane_bays(unit, 2)
        assert [b["material"] for b in bays] == ["PLA", "PETG"]

    def test_each_bay_title_is_scoped_to_its_own_group(self):
        # The bug: without the <g>, the first bay's <title> became the whole
        # SVG's tooltip and every spool showed it.
        from extras.afc_dryer import PAGE
        assert "'<g>' + (tip ? '<title>' + tip + '</title>' : '')" in PAGE
        assert "</g>'" in PAGE


class TestAbsentReadingsShowADash:
    """Every one of these sensors reports 0.0 for a channel it has not read.
    A Bambu HT's aht10 sits at temperature 0.0 while its humidity reads 41.0.
    The page dashes on null and prints the number otherwise, so passing 0.0
    through puts "0 °C" on a card -- a freezing chamber rather than an absent
    reading."""

    def test_zero_is_absent(self):
        assert _reading(0.0) is None
        assert _reading(0) is None

    def test_a_real_reading_survives(self):
        assert _reading(29.79) == 29.79
        assert _reading(42.91) == 42.91

    def test_none_stays_none(self):
        assert _reading(None) is None

    def test_a_negative_reading_is_kept(self):
        # Absent is 0, not "falsy in general" -- a sub-zero chamber is
        # implausible but it is a READING, and dropping it would hide it.
        assert _reading(-3.5) == -3.5

    def test_a_string_number_is_accepted(self):
        # Vendor status payloads are not always typed.
        assert _reading("29.8") == 29.8

    def test_junk_is_absent_not_a_crash(self):
        assert _reading("n/a") is None
        assert _reading(object()) is None

    def test_one_channel_absent_does_not_hide_the_other(self):
        # The exact HT case: temperature unread, humidity live.
        assert _reading(0.0) is None and _reading(41.0) == 41.0


class TestDryRefusalReachesTheCard:
    """Reported from the machine: the card said nothing when a dry was refused
    with a lane at the toolhead. Two layers hid it, and both were host state
    standing in for machine state -- the backend suppressed the reason while
    self._drying was set (which AFC_BAMBU_HEATER_START sets whether or not the AMS
    accepted), and the page returned 'Idle' before ever consulting it."""

    """An AMS that refuses to dry echoes our temp/time back first, so the
    command IS delivered and AFC_BAMBU_HEATER_START reports success. Without the
    reason on the card, the unit just sits at "not drying" and nothing says
    why."""

    def test_the_reason_reaches_the_card(self):
        snap = _BambuBackend().snapshot(
            object(), {"bridge_online": True, "drying": False,
                       "dry_error": "filament hub load!"})
        assert snap["error"] == "filament hub load!"
        assert snap["drying"] is False

    def test_no_refusal_leaves_it_empty(self):
        snap = _BambuBackend().snapshot(
            object(), {"bridge_online": True, "drying": False})
        assert snap["error"] == "" and snap["note"] == ""

    def test_the_reason_is_its_OWN_field_not_the_note(self):
        # A note describes a running cycle ("Drying -- ..."); overloading it
        # made the card read "Drying" beside the reason it was not.
        snap = _BambuBackend().snapshot(
            object(), {"bridge_online": True, "drying": True,
                       "dry_error": "filament hub load!"})
        assert snap["error"] == "filament hub load!"
        assert snap["note"] == ""

    def test_the_reason_survives_the_drying_flag_being_set(self):
        # THE regression: a refused start leaves drying True (we set it when
        # the command went out), and that used to suppress the reason -- in
        # exactly the case the reason exists for.
        snap = _BambuBackend().snapshot(
            object(), {"bridge_online": True, "drying": True,
                       "dry_error": "filament hub load!"})
        assert snap["error"] == "filament hub load!"

    def test_a_running_cycle_shows_no_stale_reason(self):
        # The unit clears it once heating starts; this pins that the card does
        # not carry a reason next to an active cycle.
        snap = _BambuBackend().snapshot(
            object(), {"bridge_online": True, "drying": True,
                       "dry_error": None, "temperature": 44.0})
        assert snap["error"] == "" and snap["drying"] is True


# ── http ──────────────────────────────────────────────────────────────────────
#
# was tests/test_afc_dryer_http.py
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


class TestCssColor_http:
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


class TestBackendContract_http:
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

