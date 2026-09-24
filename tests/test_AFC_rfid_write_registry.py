"""
The universal tag writer: which readers exist, and picking one.

Every unit with a host-side reader registers into one printer-scoped registry,
so AFC_RFID_WRITE works the same on a BoxTurtle, an ACE2, a ViViD or an
OpenAMS. What differs per unit is only how a link is obtained and what has to
happen around the write -- the ACE2 has to be taken off its firmware's identify
loop first -- so those are the two things a unit supplies.
"""

import types
import pytest

import extras.AFC_rfid_write as mod
import extras.AFC_rfid_readers as readers_mod
from extras.AFC_rfid_write import (
    register_reader, resolve_reader, spool_fields,
)


class _Gcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, cb, desc=""):
        self.commands[name] = cb


class _Printer:
    def __init__(self, gcode=None):
        self._gcode = _Gcode() if gcode is None else gcode

    def lookup_object(self, name, default=None):
        return self._gcode if name == "gcode" else default


class _Link:
    def __init__(self, powerable=True):
        self.events = []
        if powerable:
            self.reader_power = self._power

    def _power(self, on):
        self.events.append(("power", on))

    def reg_read(self, reg):
        return 0

    def reg_write(self, reg, val):
        pass


def _unit():
    return type("U", (), {"afc": None, "reactor": None, "logger": None})()


def _reg(printer, *names, unit=None, link=None):
    unit = unit or _unit()
    for n in names:
        register_reader(printer, n, f"label for {n}", unit,
                        lambda l=link: l if l is not None else _Link())
    return unit


# ── the registry ─────────────────────────────────────────────────────────────

class TestRegistry:
    def test_registering_creates_the_commands(self):
        p = _Printer()
        _reg(p, "bt:reader0")
        assert "AFC_RFID_WRITE" in p._gcode.commands
        assert "AFC_RFID_READERS" in p._gcode.commands

    def test_a_printer_with_no_reader_gets_no_commands(self):
        p = _Printer()
        assert p._gcode.commands == {}

    def test_the_commands_are_registered_once_for_many_readers(self):
        p = _Printer()
        calls = []
        p._gcode.register_command = lambda n, c, desc="": calls.append(n)
        _reg(p, "bt:reader0", "bt:reader1", "oams:rfid_a")
        assert calls == ["AFC_RFID_READERS", "AFC_RFID_WRITE", "AFC_RFID_READ",
                         "AFC_RFID_CLASSIC_WRITE", "AFC_RFID_ERASE",
                         "AFC_RFID_ENROLL"]

    def test_re_registering_a_reader_replaces_it(self):
        """A unit that re-probes on reconnect registers again; that must not
        pile up duplicates."""
        p = _Printer()
        _reg(p, "bt:reader0")
        _reg(p, "bt:reader0")
        assert list(mod._registry(p)) == ["bt:reader0"]

    def test_the_registry_is_per_printer_not_per_module(self):
        """Klipper builds a new Printer without re-importing extras, so module
        state would outlive the units in it and hand out dead links."""
        a, b = _Printer(), _Printer()
        _reg(a, "bt:reader0")
        assert list(mod._registry(b)) == []

    def test_registration_retries_while_there_is_no_gcode_object(self):
        """Registering before the gcode object exists must not mark the
        commands done and leave the printer without them."""
        class _Late(_Printer):
            def __init__(self):
                super().__init__()
                self.ready = False

            def lookup_object(self, name, default=None):
                if name == "gcode" and not self.ready:
                    return default
                return super().lookup_object(name, default)

        p = _Late()
        _reg(p, "bt:reader0")
        assert p._gcode.commands == {}
        p.ready = True
        _reg(p, "bt:reader1")
        assert "AFC_RFID_WRITE" in p._gcode.commands


# ── choosing one ─────────────────────────────────────────────────────────────

class TestResolveReader:
    def test_a_qualified_name_is_found(self):
        p = _Printer()
        _reg(p, "bt:reader0", "oams:rfid_a")
        target, err = resolve_reader(p, "oams:rfid_a")
        assert err is None and target.name == "oams:rfid_a"

    def test_a_bare_name_works_when_it_is_unambiguous(self):
        p = _Printer()
        _reg(p, "bt:reader0", "oams:rfid_a")
        target, err = resolve_reader(p, "rfid_a")
        assert err is None and target.name == "oams:rfid_a"

    def test_a_bare_name_shared_by_two_units_is_refused(self):
        """bt:reader0 and vivid:reader0 both exist -- picking one silently
        would write the tag at the wrong bench."""
        p = _Printer()
        _reg(p, "bt:reader0", "vivid:reader0")
        target, err = resolve_reader(p, "reader0")
        assert target is None
        assert "ambiguous" in err
        assert "bt:reader0" in err and "vivid:reader0" in err

    def test_matching_ignores_case(self):
        p = _Printer()
        _reg(p, "bt:reader0")
        assert resolve_reader(p, "BT:Reader0")[0].name == "bt:reader0"

    def test_a_lone_reader_needs_no_name(self):
        p = _Printer()
        _reg(p, "bt:reader0")
        assert resolve_reader(p, None)[0].name == "bt:reader0"

    def test_with_several_readers_a_name_is_required(self):
        p = _Printer()
        _reg(p, "bt:reader0", "bt:reader1")
        target, err = resolve_reader(p, None)
        assert target is None and "READER= is required" in err

    def test_an_unknown_name_lists_what_there_is(self):
        p = _Printer()
        _reg(p, "bt:reader0")
        target, err = resolve_reader(p, "nope")
        assert target is None
        assert "no reader called nope" in err and "bt:reader0" in err

    def test_no_readers_at_all_says_so(self):
        target, err = resolve_reader(_Printer(), "anything")
        assert target is None and "no RFID readers are registered" in err


# ── taking the reader, and giving it back ────────────────────────────────────

class TestPrepareAndRelease:
    def test_a_powerable_link_is_powered_for_the_write(self, monkeypatch):
        link = _Link()
        p = _Printer()
        _reg(p, "bt:reader0", link=link)
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        uid, err, _f, _n = mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert (uid, err) == ("04ab", None)
        assert link.events == [("power", True), ("power", False)]

    def test_a_link_with_no_power_control_is_left_alone(self, monkeypatch):
        link = _Link(powerable=False)
        p = _Printer()
        _reg(p, "vivid:reader0", link=link)
        target = resolve_reader(p, "vivid:reader0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        assert mod._write_blocking(target, 0, {"ftype": "PLA"})[1] is None

    def test_the_reader_is_given_back_even_when_the_write_fails(self,
                                                                monkeypatch):
        """Leaving the ACE2's identify loop off, or the reader powered, would
        outlast the command and break the next insert."""
        link = _Link()
        p = _Printer()
        _reg(p, "bt:reader0", link=link)
        target = resolve_reader(p, "bt:reader0")[0]

        def _boom(l, pay):
            raise RuntimeError("serial went away")
        monkeypatch.setattr(mod, "write_tag", _boom)
        with pytest.raises(RuntimeError):
            mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert link.events == [("power", True), ("power", False)]

    def test_a_units_own_prepare_and_release_are_used(self, monkeypatch):
        seen = []
        p = _Printer()
        link = _Link(powerable=False)
        register_reader(p, "ace2:slot0", "ACE2 slot 0", _unit(),
                        lambda: link,
                        prepare=lambda l: seen.append("prepare"),
                        release=lambda l: seen.append("release"))
        target = resolve_reader(p, "ace2:slot0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert seen == ["prepare", "release"]

class TestStaging:
    """A spool reader has to spin the tag into range before a write. run_write
    /run_enroll do that ON THE REACTOR via the target's stage/unstage, around
    the write, and only when a lane is named."""

    def _staged_target(self, monkeypatch, seen):
        p = _Printer()
        link = _Link(powerable=False)
        register_reader(
            p, "bt:reader0", "BoxTurtle reader0", _unit(), lambda: link,
            stage=lambda lane: seen.append(("stage", lane)) or ("tok", lane),
            unstage=lambda tok: seen.append(("unstage", tok)))
        monkeypatch.setattr(
            mod, "write_tag",
            lambda l, pay: seen.append("write") or ("04ab", None))
        return resolve_reader(p, "bt:reader0")[0]

    def test_a_named_lane_stages_around_the_write(self, monkeypatch):
        seen = []
        target = self._staged_target(monkeypatch, seen)
        mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane11")
        assert seen == [("stage", "lane11"), "write",
                        ("unstage", ("tok", "lane11"))]

    def test_no_lane_skips_staging(self, monkeypatch):
        seen = []
        target = self._staged_target(monkeypatch, seen)
        mod.run_write(target, 0, {"ftype": "PLA"})
        assert seen == ["write"]

    def test_unstage_runs_even_when_the_write_fails(self, monkeypatch):
        seen = []
        target = self._staged_target(monkeypatch, seen)

        def _boom(l, pay):
            seen.append("write")
            raise RuntimeError("serial went away")
        monkeypatch.setattr(mod, "write_tag", _boom)
        with pytest.raises(RuntimeError):
            mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane11")
        assert ("unstage", ("tok", "lane11")) in seen

    def test_a_target_without_a_stage_ignores_the_lane(self, monkeypatch):
        seen = []
        p = _Printer()
        link = _Link(powerable=False)
        register_reader(p, "ace2:slot0", "ACE2 slot 0", _unit(), lambda: link)
        monkeypatch.setattr(
            mod, "write_tag",
            lambda l, pay: seen.append("write") or ("04ab", None))
        target = resolve_reader(p, "ace2:slot0")[0]
        mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane1")
        assert seen == ["write"]

    def test_enroll_stages_too(self, monkeypatch):
        seen = []
        target = self._staged_target(monkeypatch, seen)
        monkeypatch.setattr(mod, "spool_fields",
                            lambda unit, sid: ({"ftype": "PLA"}, ""))
        monkeypatch.setattr(mod, "_cached_spoolman_client",
                            lambda afc: types.SimpleNamespace(
                                write_spool_metadata=lambda *a, **k: None))
        mod.run_enroll(target, 7, {"ftype": "PLA"}, lane="lane11")
        assert seen[0] == ("stage", "lane11")
        assert seen[-1] == ("unstage", ("tok", "lane11"))


class TestOfflineReader:
    def test_an_offline_reader_is_reported_not_written(self, monkeypatch):
        p = _Printer()
        register_reader(p, "bt:reader0", "BoxTurtle reader0", _unit(),
                        lambda: None)
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: pytest.fail("must not write"))
        uid, err, _f, _n = mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert uid is None and "offline" in err


# ── Spoolman -> tag fields ───────────────────────────────────────────────────

class _Client:
    def __init__(self, spool):
        self.spool = spool
        self.bound = []

    def get_spool(self, spool_id):
        return self.spool

    def write_spool_metadata(self, spool_id, uid=None, lot_nr=None):
        self.bound.append((spool_id, uid))


@pytest.fixture
def client(monkeypatch):
    def _install(spool):
        c = _Client(spool)
        monkeypatch.setattr(mod, "_cached_spoolman_client", lambda afc: c)
        return c
    return _install


SPOOL = {
    "id": 136,
    "initial_weight": 1000,
    "remaining_weight": 412.5,
    "filament": {
        "name": "PLA Basic Black", "material": "PLA", "color_hex": "1A2B3C",
        "density": 1.24, "diameter": 1.75, "weight": 823,
        "vendor": {"name": "Polymaker"},
        "settings_extruder_temp": 225, "settings_bed_temp": 60,
    },
}


class TestSpoolFields:
    def test_it_maps_the_record_onto_tag_fields(self, client):
        client(SPOOL)
        f, note = spool_fields(_unit(), 136)
        assert f["manufacturer"] == "Polymaker"
        assert f["ftype"] == "PLA"
        assert f["sku"] == "PLA Basic Black"
        assert f["color_argb"] == 0xFF1A2B3C
        assert f["diameter_mm"] == 1.75
        assert f["density"] == 1.24
        assert f["hotend_max_c"] == 225
        assert f["bed_temp_c"] == 60
        assert f["spool_id"] == 136
        assert note == "spool 136"

    def test_the_weight_is_the_spools_not_what_is_left_on_it(self, client):
        """The tag describes the filament. Remaining weight is a running
        figure Spoolman already tracks against the spool id we also write."""
        client(SPOOL)
        assert spool_fields(_unit(), 136)[0]["weight_g"] == 823

    def test_it_falls_back_to_the_initial_weight(self, client):
        sp = {"filament": {"material": "PLA"}, "initial_weight": 750}
        client(sp)
        assert spool_fields(_unit(), 136)[0]["weight_g"] == 750

    def test_a_missing_spool_is_reported_and_nothing_is_written(self, client):
        client(None)
        f, note = spool_fields(_unit(), 999999)
        assert f == {}
        assert "no spool 999999" in note

    def test_a_spoolman_error_is_reported_not_raised(self, monkeypatch):
        class _Broken:
            def get_spool(self, _id):
                raise RuntimeError("connection refused")
        monkeypatch.setattr(mod, "_cached_spoolman_client",
                            lambda afc: _Broken())
        f, note = spool_fields(_unit(), 136)
        assert f == {} and "connection refused" in note

    def test_the_uid_is_bound_back_to_the_spool(self, client, monkeypatch):
        c = client(SPOOL)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: ("04a1b2c3d4e5f6", None))
        _uid, err, _f, note = mod._write_blocking(target, 136, {})
        assert err is None
        assert c.bound == [(136, "04a1b2c3d4e5f6")]
        assert "uid bound" in note

    def test_nothing_is_bound_when_the_write_failed(self, client, monkeypatch):
        c = client(SPOOL)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: ("04ab", "tag did not ACK page 9"))
        mod._write_blocking(target, 136, {})
        assert c.bound == []

    def test_explicit_params_beat_the_spoolman_record(self, client,
                                                      monkeypatch):
        client(SPOOL)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        _uid, _err, fields, _n = mod._write_blocking(
            target, 136, {"manufacturer": "Overture", "weight_g": 500})
        assert fields["manufacturer"] == "Overture"
        assert fields["weight_g"] == 500
        assert fields["ftype"] == "PLA", "unset fields still come from the spool"


# ── which thread the write runs on ───────────────────────────────────────────

class TestThreading:
    def test_an_mcu_backed_reader_runs_on_the_reactor(self, monkeypatch):
        """Klipper raises "cannot switch to a different thread" and shuts the
        printer down if an MCU_SPI transfer happens on a worker. Registering
        without threaded=True must keep the write inline -- the unit here has
        no reactor at all, so touching one would blow up."""
        p = _Printer()
        register_reader(p, "oams:RFID_A", "OpenAMS RFID_A", _unit(),
                        lambda: _Link(powerable=False))
        target = resolve_reader(p, "oams:RFID_A")[0]
        assert target.threaded is False
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        uid, err, _f, _n = mod.run_write(target, 0, {"ftype": "PLA"})
        assert (uid, err) == ("04ab", None)

    def test_the_default_is_the_safe_one(self):
        p = _Printer()
        _reg(p, "vivid:reader0")
        assert resolve_reader(p, "vivid:reader0")[0].threaded is False

    def test_a_host_owned_serial_port_may_be_threaded(self, monkeypatch):
        """The BoxTurtle bridge is a plain USB-CDC port this module opened, so
        its write belongs OFF the reactor -- it is seconds of round trips."""
        import threading as _t

        class _Reactor:
            def __init__(self):
                self.waited = False

            def monotonic(self):
                return 0.0

            def completion(self):
                outer = self
                ev = _t.Event()

                class _C:
                    def complete(self, _v):
                        ev.set()

                    def wait(self, _t_deadline):
                        # The real reactor blocks here until the worker
                        # completes; model that so the result is ready on return.
                        outer.waited = True
                        ev.wait(5.0)
                return _C()

            def register_async_callback(self, cb):
                cb(0.0)

        reactor = _Reactor()
        unit = type("U", (), {"afc": None, "reactor": reactor,
                              "logger": None})()
        p = _Printer()
        register_reader(p, "bt:reader0", "BoxTurtle reader0", unit,
                        lambda: _Link(), threaded=True)
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ab", None))
        uid, err, _f, _n = mod.run_write(target, 0, {"ftype": "PLA"})
        assert (uid, err) == ("04ab", None)
        assert reactor.waited, "a threaded write must wait on the reactor"


# ── a reader this host cannot drive: the payload hand-off (the U1) ────────────

class TestWritePayloadHandoff:
    def test_the_handoff_replaces_the_link_path(self, monkeypatch):
        seen = {}
        p = _Printer()

        def handoff(payload):
            seen["payload"] = payload
            return "04deadbeef1122", None

        register_reader(p, "u1:scanner0", "U1 OpenRFID scanner", _unit(),
                        open_link=lambda: True, threaded=True,
                        write_payload=handoff)
        target = resolve_reader(p, "u1:scanner0")[0]
        # write_tag must NOT be called for a hand-off target.
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: pytest.fail("link path used"))
        uid, err, fields, _n = mod._write_blocking(
            target, 0, {"ftype": "PLA", "weight_g": 1000})
        assert (uid, err) == ("04deadbeef1122", None)
        # It got the fully-encoded tag bytes (Anycubic layout + AFC block),
        # not the raw fields.
        assert len(seen["payload"]) == 144
        assert seen["payload"][:4] == readers_mod.ANYCUBIC_MAGIC

    def test_a_handoff_target_is_always_listed_online(self):
        p = _Printer()
        register_reader(p, "u1:scanner0", "U1 OpenRFID scanner", _unit(),
                        open_link=lambda: None,      # would read as offline
                        write_payload=lambda pay: ("04", None))
        target = resolve_reader(p, "u1:scanner0")[0]
        assert mod._online(target) is True

    def test_a_handoff_error_is_passed_through(self, monkeypatch):
        p = _Printer()
        register_reader(p, "u1:scanner0", "U1 OpenRFID scanner", _unit(),
                        open_link=lambda: True,
                        write_payload=lambda pay: (None, "no answer from OpenRFID"))
        target = resolve_reader(p, "u1:scanner0")[0]
        uid, err, _f, _n = mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert uid is None and "no answer" in err

    def test_the_handoff_still_binds_the_spool_uid(self, client, monkeypatch):
        c = client(SPOOL)
        p = _Printer()
        register_reader(p, "u1:scanner0", "U1 OpenRFID scanner", _unit(),
                        open_link=lambda: True,
                        write_payload=lambda pay: ("04a1b2c3d4e5f6", None))
        target = resolve_reader(p, "u1:scanner0")[0]
        _uid, err, _f, note = mod._write_blocking(target, 136, {})
        assert err is None
        assert c.bound == [(136, "04a1b2c3d4e5f6")]
        assert "uid bound" in note
# ── the read side (AFC_RFID_READ) ────────────────────────────────────────────

class _ReadLink:
    """A link that reg_read/reg_write; read_tag is monkeypatched, so behaviour
    is irrelevant -- what matters is that it HAS reg_read (a real reader)."""
    def reg_read(self, reg): return 0
    def reg_write(self, reg, val): pass


class TestRead:
    def test_it_reads_through_the_link_and_reports(self, monkeypatch):
        p = _Printer()
        unit = _unit()
        unit.bambu_master_key = b"k"
        register_reader(p, "bt:reader0", "BoxTurtle reader0", unit,
                        lambda: _ReadLink())
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: {
            "uid": "04a1b2c3d4e5f6", "tag_type": "MifareUltralight",
            "filament": {"manufacturer": "Polymaker", "type": "PLA",
                         "weight_g": 1000, "color_argb": 0xFF1A2B3C}})
        tag, err = mod.run_read(target)
        assert err is None
        assert tag["uid"] == "04a1b2c3d4e5f6"

    def test_the_unit_keys_reach_read_tag(self, monkeypatch):
        p = _Printer()
        unit = _unit()
        unit.bambu_master_key = b"master"
        unit.creality_key = b"ck"
        unit.creality_encryption_key = b"cek"
        register_reader(p, "ace2:slot0", "ACE2 slot0", unit,
                        lambda: _ReadLink())
        target = resolve_reader(p, "ace2:slot0")[0]
        seen = {}
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: seen.update(kw) or {"uid": "04"})
        mod.run_read(target)
        assert seen["bambu_master_key"] == b"master"
        assert seen["creality_key"] == b"ck"
        assert seen["creality_encryption_key"] == b"cek"

    def test_no_tag_is_reported(self, monkeypatch):
        p = _Printer()
        register_reader(p, "bt:reader0", "r", _unit(), lambda: _ReadLink())
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: None)
        tag, err = mod.run_read(target)
        assert tag is None and "no tag" in err

    def test_an_offline_reader_is_reported(self):
        p = _Printer()
        register_reader(p, "bt:reader0", "r", _unit(), lambda: None)
        target = resolve_reader(p, "bt:reader0")[0]
        tag, err = mod.run_read(target)
        assert tag is None and "offline" in err

    def test_a_handoff_reader_says_it_scans_itself(self):
        """The U1: open_link returns a sentinel with no reg_read, so it cannot
        be read on demand through the link path."""
        p = _Printer()
        register_reader(p, "u1:scanner0", "U1 scanner", _unit(),
                        lambda: True)
        target = resolve_reader(p, "u1:scanner0")[0]
        tag, err = mod.run_read(target)
        assert tag is None and "scans on its own" in err

    def test_prepare_and_release_run_around_the_read(self, monkeypatch):
        """The ACE2's identify/power dance must bracket a read too."""
        p = _Printer()
        seen = []
        register_reader(p, "ace2:slot0", "ACE2 slot0", _unit(),
                        lambda: _ReadLink(),
                        prepare=lambda l: seen.append("prep"),
                        release=lambda l: seen.append("rel"))
        target = resolve_reader(p, "ace2:slot0")[0]
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: {"uid": "04"})
        mod.run_read(target)
        assert seen == ["prep", "rel"]

    def test_release_runs_even_if_the_read_raises(self, monkeypatch):
        p = _Printer()
        seen = []
        register_reader(p, "ace2:slot0", "ACE2 slot0", _unit(),
                        lambda: _ReadLink(),
                        prepare=lambda l: seen.append("prep"),
                        release=lambda l: seen.append("rel"))
        target = resolve_reader(p, "ace2:slot0")[0]

        def _boom(link, **kw):
            raise RuntimeError("reader wedged")
        monkeypatch.setattr(mod, "read_tag", _boom)
        with pytest.raises(RuntimeError):
            mod._read_blocking(target)
        assert seen == ["prep", "rel"]


# ── enrolling a blank tag (AFC_RFID_ENROLL) ──────────────────────────────────

class _CreatingClient(_Client):
    """A Spoolman stub that also records vendor/filament/spool creation."""
    def __init__(self, spool=None):
        super().__init__(spool)
        self.made = {}

    def get_or_create_vendor(self, name):
        self.made["vendor"] = name
        return {"id": 7, "name": name}

    def create_filament(self, name, **kw):
        self.made["filament"] = dict(name=name, **kw)
        return {"id": 42, "name": name}

    def create_spool(self, filament_id, initial_weight=None, **kw):
        self.made["spool"] = dict(filament_id=filament_id,
                                  initial_weight=initial_weight)
        # the spool the write path then reads back
        self.spool = {"id": 99, "filament": {"name": "Generic PLA",
                      "material": "PLA", "weight": initial_weight}}
        return {"id": 99}


class TestEnroll:
    def test_linking_an_existing_spool_writes_and_binds(self, client,
                                                        monkeypatch):
        c = client(SPOOL)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: ("04a1b2c3d4e5f6", None))
        uid, err, _f, note = mod.run_enroll(target, 136, {})
        assert err is None
        assert c.bound == [(136, "04a1b2c3d4e5f6")]

    def test_a_new_spool_is_created_then_written(self, monkeypatch):
        c = _CreatingClient()
        monkeypatch.setattr(mod, "_cached_spoolman_client", lambda afc: c)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "read_tag", lambda l, **kw: {"uid": "04ffee"})
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04ffee", None))
        uid, err, fields, note = mod.run_enroll(
            target, 0, {"manufacturer": "Inland", "ftype": "PETG",
                        "color_argb": 0xFF00FF00, "weight_g": 1000,
                        "diameter_mm": 1.75})
        assert err is None
        assert c.made["vendor"] == "Inland"
        assert c.made["filament"]["material"] == "PETG"
        assert c.made["filament"]["color_hex"] == "00FF00"
        assert c.made["spool"]["initial_weight"] == 1000
        assert "created spool 99" in note
        assert c.bound == [(99, "04ffee")], "the new spool gets the tag UID"

    def test_a_spoolman_create_failure_is_reported(self, monkeypatch):
        class _Broken(_CreatingClient):
            def create_filament(self, name, **kw):
                return None
        c = _Broken()
        monkeypatch.setattr(mod, "_cached_spoolman_client", lambda afc: c)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "read_tag", lambda l, **kw: {"uid": "04ffee"})
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: pytest.fail("must not write"))
        uid, err, _f, _n = mod.run_enroll(target, 0, {"ftype": "PLA"})
        assert uid is None and "filament create failed" in err

    def test_no_tag_present_creates_no_orphan_spool(self, monkeypatch):
        """The orphan guard: a missing tag must not leave a Spoolman spool."""
        c = _CreatingClient()
        monkeypatch.setattr(mod, "_cached_spoolman_client", lambda afc: c)
        monkeypatch.setattr(mod, "read_tag", lambda l, **kw: None)  # no tag
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        uid, err, _f, _n = mod.run_enroll(target, 0, {"ftype": "PLA"})
        assert uid is None and "no tag" in err
        assert c.made == {}, "nothing created in Spoolman without a tag"


# ── erasing a tag (AFC_RFID_ERASE) ───────────────────────────────────────────

class TestErase:
    def test_it_writes_zeros_over_the_record(self, monkeypatch):
        seen = {}
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag",
                            lambda l, pay: (seen.update(payload=pay)
                                            or ("04aa", None)))
        uid, err = mod.run_erase(target)
        assert (uid, err) == ("04aa", None)
        assert seen["payload"] == b"\x00" * mod._ERASE_LEN
        assert len(seen["payload"]) == 144

    def test_it_does_not_touch_spoolman(self, client, monkeypatch):
        c = client(SPOOL)
        p = _Printer()
        _reg(p, "bt:reader0")
        target = resolve_reader(p, "bt:reader0")[0]
        monkeypatch.setattr(mod, "write_tag", lambda l, pay: ("04aa", None))
        mod.run_erase(target)
        assert c.bound == [], "erase leaves the Spoolman UID binding alone"

    def test_offline_is_reported(self):
        p = _Printer()
        register_reader(p, "bt:reader0", "r", _unit(), lambda: None)
        target = resolve_reader(p, "bt:reader0")[0]
        uid, err = mod.run_erase(target)
        assert uid is None and "offline" in err

    def test_a_handoff_reader_erases_through_its_payload_path(self, monkeypatch):
        seen = {}
        p = _Printer()
        register_reader(p, "u1:scanner0", "U1 scanner", _unit(),
                        open_link=lambda: True,
                        write_payload=lambda pay: (seen.update(p=pay)
                                                   or ("04bb", None)))
        target = resolve_reader(p, "u1:scanner0")[0]
        uid, err = mod.run_erase(target)
        assert (uid, err) == ("04bb", None)
        assert seen["p"] == b"\x00" * mod._ERASE_LEN
