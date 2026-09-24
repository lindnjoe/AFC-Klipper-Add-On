"""
ACE_RFID_RESCAN UNIT= LANE= (extras/AFC_ACE.py afcACE.cmd_ACE_RFID_RESCAN).

The V1 ACE's firmware owns its reader, so a rescan moves nothing: it asks the
firmware for the slot's tag and, when recognized, the tag replaces what the
lane showed. The ACE 2 hands the rescan to [AFC_ACE2_rfid], which turns the
spool past the host-driven reader (tested in test_AFC_ACE2_rfid_coverage.py).
"""

from __future__ import annotations

import types

import pytest

from extras.AFC_ACE import afcACE
from extras.AFC_ACE2 import afcACE2

from tests.ace_helpers import FakeAFC, FakeGcmd, FakeLane, FakeLogger, FakeReactor


class _Conn:
    def __init__(self, payloads):
        self.connected = True
        self._payloads = list(payloads)
        self.calls = []

    def get_filament_info(self, slot):
        self.calls.append(slot)
        return self._payloads.pop(0) if len(self._payloads) > 1 else self._payloads[0]


class _Reactor(FakeReactor):
    def pause(self, until):
        self._monotonic = until


def _unit(cls, payloads=({"rfid": 0},)):
    unit = cls.__new__(cls)
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit.reactor = _Reactor()
    unit.afc = FakeAFC()
    unit._ace = _Conn(payloads)
    unit._slot_map = {"lane1": 0, "lane2": 1}
    unit._slot_inventory = [{} for _ in range(cls.SLOTS_PER_UNIT)]
    lane = FakeLane("lane1", tool_loaded=True)
    lane.material, lane.color = "PLA", "#FFFFFF"
    lane.extruder_temp, lane.bed_temp = 210.0, 60.0
    unit.lanes = {"lane1": lane}
    unit.printer = types.SimpleNamespace(lookup_object=lambda n, d=None: d)
    return unit, lane


_PETG = {"rfid": 2, "type": "PETG", "brand": "Anycubic", "color": [255, 0, 0],
         "extruder_temp": {"min": 230, "max": 250},
         "hotbed_temp": {"min": 70, "max": 80}}


def test_v1_recognized_tag_replaces_the_lane_values_in_place():
    unit, lane = _unit(afcACE, [_PETG])
    gcmd = FakeGcmd(LANE="lane1")
    unit.cmd_ACE_RFID_RESCAN(gcmd)
    assert lane.material == "PETG"
    assert lane.color.lower() == "#ff0000"
    assert (lane.extruder_temp, lane.bed_temp) == (240.0, 75.0)
    assert unit._ace.calls == [0]
    assert len(unit.afc.save_vars.calls) == 1
    assert "read lane1's tag" in gcmd.responses[-1]


def test_v1_waits_out_recognizing():
    unit, lane = _unit(afcACE, [{"rfid": 3}, {"rfid": 3}, _PETG])
    unit.cmd_ACE_RFID_RESCAN(FakeGcmd(LANE="lane1"))
    assert unit._ace.calls == [0, 0, 0]
    assert lane.material == "PETG"


@pytest.mark.parametrize("rfid,msg", [(0, "no tag found"),
                                      (1, "tag not recognized"),
                                      (3, "still recognizing")])
def test_v1_no_read_leaves_the_lane_alone(rfid, msg):
    unit, lane = _unit(afcACE, [{"rfid": rfid}])
    gcmd = FakeGcmd(LANE="lane1")
    unit.cmd_ACE_RFID_RESCAN(gcmd)
    assert (lane.material, lane.color) == ("PLA", "#FFFFFF")
    assert msg in gcmd.responses[-1]
    assert unit.afc.save_vars.calls == []


def test_a_lane_of_another_unit_is_refused():
    unit, _ = _unit(afcACE)
    with pytest.raises(RuntimeError, match="not a lane of Ace_1"):
        unit.cmd_ACE_RFID_RESCAN(FakeGcmd(LANE="lane9"))


def test_ace2_hands_the_rescan_to_its_rfid_module():
    unit, _ = _unit(afcACE2)
    seen = []
    rfid = types.SimpleNamespace(ace2=unit,
                                 rescan_lane=lambda n, g: seen.append(n))
    unit.printer = types.SimpleNamespace(
        lookup_object=lambda n, d=None: rfid if n == "AFC_ACE2_rfid" else d)
    unit.cmd_ACE_RFID_RESCAN(FakeGcmd(LANE="lane1"))
    assert seen == ["lane1"]
    assert unit._ace.calls == []              # no firmware read on an ACE 2


def test_ace2_without_its_rfid_module_is_refused():
    unit, _ = _unit(afcACE2)
    with pytest.raises(RuntimeError, match=r"needs \[AFC_ACE2_rfid\]"):
        unit.cmd_ACE_RFID_RESCAN(FakeGcmd(LANE="lane1"))
