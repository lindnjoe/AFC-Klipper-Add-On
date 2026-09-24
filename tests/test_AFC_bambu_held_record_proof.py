"""What shows that a bay's tag is not the spool its lane's restored record
describes.

A record linked to Spoolman is dropped only on Spoolman's word: its material
and colour are Spoolman's or the user's text, which differs from the tag's
for the same spool, and a Bambu reel carries a tag on each side with its own
chip UID and one tray UID. A record with no link is not dropped on its
material's words alone while its colour and variant agree with the tag. A
lookup for a lane whose record had a link is match-only.
"""
from __future__ import annotations

import json
import types

import pytest

from extras import AFC_BambuAMS_rfid as rfid
from extras.AFC_BambuAMS import afcBambuAMS, bridge_slot_to_info
from tests.test_AFC_bridgebox_lane_records import _claimed

LINKED = {"spool_id": 71, "material": "PLA", "color": "#3A7BD5",
          "weight": 612.5, "extruder_temp": 220.0, "bed_temp": 60.0}
TAG = {"present": True, "material": "PLA Basic", "color": "3A7BD5",
       "rfid_uid": "5EED0001"}


def _primed(rec, tag):
    u, lane = _claimed({"lane15": dict(rec)}, bay=dict(tag))
    u._prime_scan_baseline()
    return u, lane


class TestALinkedRecord:
    @pytest.mark.parametrize("words,tag", [
        ("Matte", {}), ("PolyTerra", {}), ("Bambu Basic", {}),
        ("PLA+", {}), ("PETG", {}),
        ("PLA", {"color": "FF0000"}),
        ("Polycarbonate", {"material": "PC"}),
        ("Nylon", {"material": "PAHT-CF"})])
    def test_is_kept_whatever_the_tags_words(self, words, tag):
        u, lane = _primed(dict(LINKED, material=words), dict(TAG, **tag))
        assert (lane.spool_id, lane.weight, lane.extruder_temp,
                lane.bed_temp) == (71, 612.5, 220.0, 60.0)
        # The tag's material and colour win.
        assert lane.material == (tag.get("material") or "PLA").split()[0]
        assert lane.color == "#" + tag.get("color", "3A7BD5")
        assert u._afc_owned == {0}
        assert not [ln for ln in u.logger.lines if "is not the spool" in ln]

    def test_a_late_tag_keeps_it_too(self):
        u, lane = _primed(dict(LINKED, material="Matte"),
                          {"present": True})
        assert u._restored_bays == {0}
        u._settle_restored_bay(lane, dict(TAG, index=0))
        assert (lane.spool_id, lane.weight, lane.material) == (
            71, 612.5, "PLA")


class TestARecordWithNoLink:
    @pytest.mark.parametrize("words,tag", [
        ("Matte", {"material": "PLA Matte"}),
        ("Silk", {"material": "PLA Silk+"}),
        ("PolyTerra", {}),
        ("Polycarbonate", {"material": "PC"}),
        ("PETG", {}),
        ("Nylon", {"material": "PAHT-CF"})])
    def test_set_by_hand_is_kept_while_colour_and_variant_agree(self, words,
                                                                tag):
        rec = dict(LINKED, spool_id=None, material=words)
        u, lane = _primed(rec, dict(TAG, **tag))
        assert (lane.weight, lane.extruder_temp, lane.bed_temp) == (
            612.5, 220.0, 60.0)
        assert lane.material == (tag.get("material") or "PLA").split()[0]
        assert u._afc_owned == {0}

    def test_a_late_tag_keeps_it_too(self):
        u, lane = _primed(dict(LINKED, spool_id=None, material="PETG"),
                          {"present": True})
        assert u._restored_bays == {0}
        u._settle_restored_bay(lane, dict(TAG, index=0))
        assert (lane.weight, lane.extruder_temp, lane.material) == (
            612.5, 220.0, "PLA")

    @pytest.mark.parametrize("rec,tag", [
        ({"material": "Matte"}, {"color": "FF0000"}),
        ({"material": "Matte", "color": ""}, {}),
        ({"material": "Matte", "sub_type": "Silk"}, {"material": "PLA Matte"}),
        ({"material": "PETG", "color": ""}, {})])
    def test_is_dropped_when_something_else_shows_another_spool(self, rec,
                                                                tag):
        # Another colour or variant, or no colour to hold the words against.
        u, lane = _primed(dict(LINKED, spool_id=None, **rec),
                          dict(TAG, **tag))
        assert (lane.weight, lane.extruder_temp) == (0, None)
        assert any("is not the spool saved for this lane" in ln
                   for ln in u.logger.lines)


class _Client:
    def __init__(self, tray=None):
        self.tray = tray

    def get_spool(self, sid):
        extra = {} if self.tray is None else {"tray_uid": json.dumps(
            self.tray)}
        return {"id": sid, "extra": extra}


class TestTheKeptLinkCheck:
    REEL = "0123456789abcdef0123456789abcdef"

    def _check(self, monkeypatch, tray, spool_tray, chips_say_other=True):
        monkeypatch.setattr(rfid, "_bambu_spoolman_client",
                            lambda afc: _Client(spool_tray))
        tag = dict(TAG, material="PLA Matte")
        if tray is not None:
            tag["tray_uid"] = tray
        u, lane = _claimed({"lane15": dict(LINKED)}, spoolman=True, bay=tag)
        asked = []

        def check(sid, uid):
            asked.append((sid, uid))
            return chips_say_other
        u._spool = types.SimpleNamespace(_binding_contradicted=check,
                                         _spoolman_bg=lambda job: job())
        u._prime_scan_baseline()
        return u, lane, asked

    @pytest.mark.parametrize("spool_tray", [REEL, REEL.upper(), None])
    def test_the_other_side_of_the_same_reel_keeps_the_link(
            self, monkeypatch, spool_tray):
        # The chip UID facing the reader is not the one Spoolman knows; the
        # tray UID is the reel's, or Spoolman records none.
        u, lane, asked = self._check(monkeypatch, self.REEL, spool_tray)
        assert asked == []
        assert (lane.spool_id, lane.weight) == (71, 612.5)
        assert not [ln for ln in u.logger.lines if "is not the spool" in ln]

    def test_another_reel_replaces_the_record(self, monkeypatch):
        u, lane, _asked = self._check(monkeypatch, self.REEL, "ff" * 16)
        assert lane.spool_id in (None, "")
        assert any("(Spoolman records another reel for spool 71)" in ln
                   for ln in u.logger.lines)
        assert u._match_only_bays == {0}

    @pytest.mark.parametrize("tray", [None, "0" * 32])
    def test_a_tag_with_no_tray_uid_is_held_against_the_chips(
            self, monkeypatch, tray):
        u, lane, asked = self._check(monkeypatch, tray, self.REEL)
        assert asked == [(71, "5EED0001")]
        assert lane.spool_id in (None, "")
        assert any("(Spoolman records other tags for spool 71)" in ln
                   for ln in u.logger.lines)
        u, lane, asked = self._check(monkeypatch, tray, self.REEL,
                                     chips_say_other=False)
        assert lane.spool_id == 71


class TestTheLookupAfterALinkWasDropped:
    def _surface(self, match_only):
        calls = []
        u = types.SimpleNamespace(
            name="AMS", logger=types.SimpleNamespace(
                info=lambda m: None, debug=lambda *a, **k: None),
            afc=None, _match_only_bays=match_only,
            _spoolman_sync=lambda lane, info, **kw: calls.append(kw),
            _apply_remain_weight=lambda lane, info: None,
            _save_lane_vars=lambda: None)
        lane = types.SimpleNamespace(name="lane15", material="", color="",
                                     spool_id=None, weight=0)
        info = bridge_slot_to_info({"i": 0, "present": True,
                                    "material": "PLA", "color": "00ae42ff",
                                    "tmin": 210, "tmax": 230,
                                    "uid": "5EED0001"})
        afcBambuAMS._surface_slot_info(u, lane, info)
        return calls

    def test_is_match_only(self):
        assert self._surface({0}) == [{"restored": True}]

    def test_a_read_of_any_other_bay_may_create(self):
        assert self._surface(set()) == [{}]

    def test_the_bay_is_marked_when_a_linked_record_is_dropped(self):
        u, lane = _claimed({"lane15": dict(LINKED)}, bay=dict(TAG))
        afcBambuAMS._drop_held_record(u, lane, dict(TAG, index=0))
        assert u._match_only_bays == {0}
        afcBambuAMS._reset_lookup_state(u)
        assert u._match_only_bays == set()
        u, lane = _claimed({"lane15": dict(LINKED, spool_id=None)},
                           bay=dict(TAG))
        afcBambuAMS._drop_held_record(u, lane, dict(TAG, index=0))
        assert u._match_only_bays == set()


class _Spoolman:
    """Spoolman's spools, id -> (card_uids, tray_uid)."""

    def __init__(self, spools):
        self.spools, self.asked = spools, []

    def _spool(self, sid):
        chips, tray = self.spools.get(int(sid), ("", None))
        extra = {}
        if chips:
            extra["card_uids"] = json.dumps(chips)
        if tray:
            extra["tray_uid"] = json.dumps(tray)
        return {"id": int(sid), "extra": extra}

    def get_spool(self, sid):
        self.asked.append(("get_spool", int(sid)))
        return self._spool(sid)

    def search_spools(self):
        self.asked.append(("search_spools",))
        return [self._spool(sid) for sid in self.spools]


class TestAReelSpoolmanRecordsOnAnotherSpool:
    """The linked spool records nothing of the reel in its bay: Spoolman
    naming another spool for the reel, or for its tag, is the proof."""

    REEL = "0123456789abcdef0123456789abcdef"

    def _check(self, monkeypatch, spools, tray):
        sm = _Spoolman(spools)
        monkeypatch.setattr(rfid, "_bambu_spoolman_client", lambda afc: sm)
        tag = dict(TAG, material="PLA Matte")
        if tray:
            tag["tray_uid"] = tray
        u, lane = _claimed({"lane15": dict(LINKED)}, spoolman=True, bay=tag)
        bs = rfid.BambuSpoolman(u)
        u._spool = types.SimpleNamespace(
            _binding_contradicted=bs._binding_contradicted,
            _spoolman_bg=lambda job: job())
        u._prime_scan_baseline()
        return u, lane, sm

    @pytest.mark.parametrize("tray,what", [(REEL, "reel"), (None, "tag")])
    def test_drops_the_link(self, monkeypatch, tray, what):
        # Spool 71 was linked by hand; spool 200 records the reel.
        u, lane, _sm = self._check(
            monkeypatch, {71: ("", None), 200: ("5EED0001", tray)}, tray)
        assert lane.spool_id in (None, "")
        assert any(f"(Spoolman records this {what} on spool 200)" in ln
                   for ln in u.logger.lines)
        # The read's lookup that follows binds spool 200 and creates nothing.
        assert u._match_only_bays == {0}

    @pytest.mark.parametrize("spools,tray", [
        # The other side of the reel: Spoolman knows neither of its tags.
        ({71: ("A1B2C3D4", None)}, REEL),
        # Spoolman knows nothing of this reel or tag.
        ({71: ("", None), 200: ("B00B0001", "ff" * 16)}, REEL),
        ({71: ("", None), 200: ("B00B0001", "ff" * 16)}, None),
        # The linked spool records this tag: a second record of the same
        # reel does not take the lane from it.
        ({71: ("5EED0001", None), 200: ("5EED0001", REEL)}, REEL),
        ({71: ("5EED0001", None), 200: ("5EED0001", None)}, None)])
    def test_keeps_the_link_without_it(self, monkeypatch, spools, tray):
        u, lane, _sm = self._check(monkeypatch, spools, tray)
        assert (lane.spool_id, lane.weight) == (71, 612.5)
        assert not [ln for ln in u.logger.lines if "is not the spool" in ln]


class TestABayFoundEmpty:
    def test_a_spool_put_in_it_later_gets_a_reads_lookup(self):
        u, lane = _claimed({"lane15": dict(LINKED)}, bay={"present": False})
        u._forget_spoolman_miss = lambda slot: None
        u._scan_primed = True
        afcBambuAMS._reconcile_empty_bays(u)
        assert lane.spool_id in (None, "")
        # A fetch of the old link landing later is still undone.
        assert u._dropped_links == {"lane15": "71"}
        assert u._match_only_bays == set()
        # During a print no scan is opened; the unit's own read surfaces.
        u.afc.function = types.SimpleNamespace(in_print=lambda: True)
        u.auto_scan, u._bridge = True, object()
        afcBambuAMS._maybe_auto_scan(u, 0, True, {"present": True})
        calls = []
        u._spoolman_sync = lambda ln, info, **kw: calls.append(kw)
        u._apply_remain_weight = lambda ln, info: None
        info = bridge_slot_to_info({"i": 0, "present": True,
                                    "material": "PETG", "color": "ff0000ff",
                                    "tmin": 230, "tmax": 260,
                                    "uid": "0BADF00D"})
        afcBambuAMS._surface_slot_info(u, lane, info)
        assert calls == [{}]


class TestAReadOfABoundBay:
    """After a restart no binding was made from a tag, so a read of a bound
    bay checks the binding; a tag with a tray UID is checked by the reel."""

    REEL = "11112222333344445555666677778888"

    def _read(self, monkeypatch, spool_tray, tag_tray):
        sm = _Spoolman({71: ("A1B2C3D4", spool_tray)})
        monkeypatch.setattr(rfid, "_bambu_spoolman_client", lambda afc: sm)
        synced, unbound = [], []
        monkeypatch.setattr(rfid, "sync_rfid_to_spoolman",
                            lambda afc, lane, si, *a, **k: synced.append(
                                k.get("allow_create")))
        monkeypatch.setattr(rfid, "get_auto_spoolman_create",
                            lambda lane, dflt: True)
        log = types.SimpleNamespace(info=lambda *a, **k: None,
                                    debug=lambda *a, **k: None,
                                    warning=lambda *a, **k: None)
        unit = types.SimpleNamespace(
            name="AMS", auto_spoolman_create=True, logger=log,
            afc=types.SimpleNamespace(spoolman=object(), moonraker=object(),
                                      reactor=None),
            _unbind_spool=lambda lane, why=None: (
                unbound.append(why), setattr(lane, "spool_id", None)))
        unit._spool = rfid.BambuSpoolman(unit)
        lane = types.SimpleNamespace(name="lane15", spool_id=71,
                                     material="PLA", color="#3A7BD5")
        # The bay reads flange B's chip.
        info = {"present": True, "index": 0, "material": "PLA Basic",
                "color": "3A7BD5", "rfid_uid": "F1F2F3F4",
                "temp_min": 190, "temp_max": 230}
        if tag_tray:
            info["tray_uid"] = tag_tray
        afcBambuAMS._spoolman_sync(unit, lane, info)
        return lane, unbound, synced, sm.asked

    @pytest.mark.parametrize("spool_tray", [REEL, None])
    def test_the_other_side_of_the_reel_keeps_the_binding(self, monkeypatch,
                                                          spool_tray):
        lane, unbound, synced, asked = self._read(monkeypatch, spool_tray,
                                                  self.REEL)
        assert (lane.spool_id, unbound, synced) == (71, [], [])
        # The one call the check makes.
        assert asked == [("get_spool", 71)]

    def test_another_reel_is_another_spool(self, monkeypatch):
        lane, unbound, synced, _asked = self._read(monkeypatch, "ff" * 16,
                                                   self.REEL)
        assert lane.spool_id is None and len(unbound) == 1
        assert synced == [True]

    def test_a_tag_with_no_tray_uid_is_held_against_the_chips(
            self, monkeypatch):
        lane, unbound, synced, _asked = self._read(monkeypatch, self.REEL,
                                                   None)
        assert lane.spool_id is None and len(unbound) == 1
