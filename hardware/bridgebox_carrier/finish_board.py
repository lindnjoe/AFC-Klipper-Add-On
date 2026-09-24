#!/usr/bin/env python3
"""
Fill the ground plane and run KiCad's own DRC.

TWO THINGS THIS FIXES THAT NOTHING ELSE CAUGHT.

A KiCad zone is a *request* for copper, not copper. Until it is filled the
board file holds only its outline, and `kicad-cli pcb export gerbers` plots
what is stored -- so the B.Cu gerber came out with the pads and vias in it and
no plane at all. On this board GND is ENTIRELY the pour: no fill means no
ground anywhere, on a board that otherwise looks finished. It is invisible in
a render, because a render draws the zone outline either way.

And check_drc.py is mine, so it only knows the mistakes I thought of. pcbnew
carries the real DRC engine, which also knows about annular rings, hole-to-hole
spacing, pad-to-pad clearance, silk over mask and the rest.

    python3 finish_board.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
BOARD = HERE / "bridgebox_carrier.kicad_pcb"
REPORT = HERE / "out" / "drc.rpt"


def main() -> int:
    board = pcbnew.LoadBoard(str(BOARD))

    zones = list(board.Zones())
    print(f"filling {len(zones)} zone(s)")
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(str(BOARD), board)

    board = pcbnew.LoadBoard(str(BOARD))
    for z in board.Zones():
        area = z.GetFilledArea() / 1e12      # nm^2 -> mm^2
        print(f"  {z.GetNetname()} on {board.GetLayerName(z.GetLayer())}:"
              f" {area:.0f} mm2 filled")
        if area <= 0:
            print("  the zone did not fill -- the plane would be missing")
            return 1

    REPORT.parent.mkdir(exist_ok=True)
    pcbnew.WriteDRCReport(board, str(REPORT), pcbnew.EDA_UNITS_MILLIMETRES,
                          True)
    text = REPORT.read_text()
    print("\n--- KiCad DRC ---")
    bad = 0
    for line in text.splitlines():
        low = line.lower()
        if low.startswith(("** found", "** no")):
            print(" ", line.strip())
            if "found" in low and not low.startswith("** found 0"):
                bad += 1
        elif line.startswith("["):
            print("  ", line.strip())
    print(f"\nfull report: {REPORT}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
