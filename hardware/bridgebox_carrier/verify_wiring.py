#!/usr/bin/env python3
"""
Check the FINISHED board against the wiring that was proven on hardware.

verify_netlist.py already compares KiCad's extracted netlist to NETS in
gen_project.py, which catches a label typo. This checks something different
and narrower: that the handful of pin assignments established by measurement,
not by reading a silkscreen, are the ones actually on the pads of the built
board file -- after the placement, the fill and pcbnew's own rewrite.

They are worth a check of their own because every one of them is a place
where the obvious answer is the wrong one, and where being wrong produces a
bus that is SILENT rather than broken in any way a picture would show:

  * GP0 -> the module's RXD pin and GP1 -> its TXD pin. These modules label
    their logic pins from the module's own side, so RXD is the driver INPUT
    and TXD is the receiver OUTPUT. Wiring GP0 to the pin marked TXD puts
    the Pico's output against the transceiver's output.
  * The module's A reaches the connector's B, and its B the connector's A.
  * J2's 24 V and GND, because that is the pair that destroys a transceiver
    rather than merely failing to talk.

    python3 verify_wiring.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_project as g  # noqa: E402

HERE = Path(__file__).resolve().parent
BOARD = HERE / "bridgebox_carrier.kicad_pcb"

# (ref, pad) -> net, and why this one is not self-evident.
WANT: dict[tuple[str, str], tuple[str, str]] = {
    ("U1", "1"): ("RXD", "GP0 drives the module's receive input"),
    ("U1", "2"): ("TXD", "GP1 listens to the module's transmit output"),
    ("U1", "4"): ("EN", "GP2 is the direction pin"),
    ("U2", "1"): ("EN", ""),
    ("U2", "2"): ("+3V3", "3.3 V part: 5 V here destroys it"),
    ("U2", "3"): ("RXD", "module RXD is the driver input"),
    ("U2", "4"): ("TXD", "module TXD is the receiver output"),
    ("U2", "7"): ("BUS_B", "module A crosses to the connector's B"),
    ("U2", "8"): ("BUS_A", "module B crosses to the connector's A"),
    ("J2", "1"): ("BUS_A", ""),
    ("J2", "2"): ("BUS_B", ""),
    ("J2", "3"): ("GND", ""),
    ("J2", "4"): ("+24V", "the pin that puts 24 V on the transceiver"),
}

# Column spacing of the RS-485 module's two pad rows, measured on the part.
MOD_ROW_SPACING = 15.0

PICO_PIN_NAME = {"1": "GP0", "2": "GP1", "4": "GP2"}


def footprints(text: str) -> list[tuple[str, str]]:
    """Every placed footprint as (reference, its s-expression block).

    :param text: the whole .kicad_pcb file
    :return list: (reference designator, block) for each footprint
    """
    out: list[tuple[str, str]] = []
    for m in re.finditer(r'\(footprint "', text):
        blk = g.sexp_block(text, m.start())
        ref = re.search(r'\(fp_text reference "([^"]+)"', blk)
        if ref:
            out.append((ref.group(1), blk))
    return out


def pad_nets(blocks: list[tuple[str, str]]) -> dict[tuple[str, str], str]:
    """(ref, pad number) -> net name, read from the board's own pads.

    Parsed as balanced s-expressions rather than by line: pcbnew rewrites the
    file when it fills the zone, and a pad that was one line in the generated
    board comes back spread over several. A line-based reader silently finds
    no nets at all and reports every pin as unassigned.

    :param blocks: (reference, block) pairs from footprints()
    :return dict: pad to net name
    """
    out: dict[tuple[str, str], str] = {}
    for ref, blk in blocks:
        for m in re.finditer(r"\(pad ", blk):
            pad = g.sexp_block(blk, m.start())
            num = re.match(r'\(pad (?:"([^"]*)"|(\S+))', pad)
            net = re.search(r'\(net \d+ "([^"]*)"\)', pad)
            if not (num and net):
                continue
            name = num.group(1) if num.group(1) is not None else num.group(2)
            if name:
                out[(ref, name)] = net.group(1)
    return out


def row_spacing(blocks: list[tuple[str, str]]) -> float | None:
    """Distance between the RS-485 module's two pad columns, in mm.

    :param blocks: (reference, block) pairs from footprints()
    :return float: the spacing, or None if U2 is not on the board
    """
    for ref, blk in blocks:
        if ref != "U2":
            continue
        xs = {float(m.group(1)) for m in
              re.finditer(r'\(pad "[0-9]" thru_hole \w+\s*\(at ([-\d.]+)', blk)}
        if xs:
            return max(xs) - min(xs)
    return None


def main() -> int:
    if not BOARD.exists():
        print(f"  FAIL  no board file at {BOARD}; run gen_pcb.py first")
        return 1
    blocks = footprints(BOARD.read_text())
    got = pad_nets(blocks)

    bad = 0
    for (ref, pad), (net, why) in sorted(WANT.items()):
        actual = got.get((ref, pad))
        name = PICO_PIN_NAME.get(pad, "") if ref == "U1" else ""
        shown = f"{ref}.{pad}" + (f" ({name})" if name else "")
        if actual == net:
            print(f"  ok    {shown:<12s} {net}"
                  + (f"   -- {why}" if why else ""))
            continue
        bad += 1
        print(f"  FAIL  {shown:<12s} is {actual!r}, should be {net!r}"
              + (f"   -- {why}" if why else ""))

    sp = row_spacing(blocks)
    if sp is None:
        bad += 1
        print("  FAIL  U2 is not on the board")
    elif abs(sp - MOD_ROW_SPACING) > 1e-6:
        bad += 1
        print(f"  FAIL  module pad rows are {sp} mm apart, "
              f"should be {MOD_ROW_SPACING}")
    else:
        print(f"  ok    module pad rows  {sp} mm apart")

    print("\nRESULT:", "board matches the wiring proven on hardware"
          if not bad else f"{bad} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
