#!/usr/bin/env python3
"""
Fill the ground pours and run KiCad's own DRC.

**A KiCad zone is a request for copper, not copper.** Until it is filled the
board file holds only the outline, and `kicad-cli pcb export gerbers` plots what
is stored -- so the copper gerbers come out with pads and vias in them and no
pour at all. The carrier shipped exactly that once. It is invisible in a render,
because a render draws the zone outline either way, which is why plot_board.py
in this directory refuses to draw the pour as though it were copper.

**This board has TWO pours, not one**, and the failure is correspondingly
worse: GND is entirely poured here, on both sides, and it is what joins 21 via
drops and every ground pad. Unfilled, the board has no ground anywhere.

And check_drc.py is mine, so it only knows the mistakes I thought of: clearance,
ratsnest, containment. pcbnew carries the real engine, which also knows annular
rings, hole-to-hole spacing, pad-to-pad clearance, silk over mask, and the rest.

Needs a KiCad install (`import pcbnew`), which is the one thing the rest of this
directory does not.

    python3 finish_board.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import design

HERE = Path(__file__).resolve().parent
BOARD = HERE / f"{design.NAME}.kicad_pcb"
REPORT = HERE / "out" / "drc.rpt"


def main() -> int:
    try:
        import pcbnew
    except ImportError:
        print("finish_board.py needs KiCad's pcbnew module -- run it where "
              "KiCad is installed. Nothing else here does.", file=sys.stderr)
        return 2

    board = pcbnew.LoadBoard(str(BOARD))

    zones = list(board.Zones())
    print(f"filling {len(zones)} zone(s)")
    if len(zones) != 2:
        print(f"  ! expected 2 (one pour per side) -- gen_pcb.py writes both; "
              f"{len(zones)} means the board was not regenerated",
              file=sys.stderr)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(BOARD), board)

    # Filled AREA, per zone, per layer. "Filled 2 zones" is not the claim that
    # matters -- a zone can fill to nothing if it is fenced in by clearance,
    # and it reports success either way.
    for z in board.Zones():
        layer = board.GetLayerName(z.GetLayer())
        area = z.GetFilledArea() / 1e12      # nm^2 -> mm^2
        print(f"  {layer}: {area:.0f} mm^2 filled")
        if area < 100:
            print(f"  ! {layer} filled almost nothing -- that is not a pour",
                  file=sys.stderr)

    REPORT.parent.mkdir(exist_ok=True)
    ok = pcbnew.WriteDRCReport(board, str(REPORT),
                               pcbnew.EDA_UNITS_MILLIMETRES, True)
    if not ok:
        print("DRC report could not be written", file=sys.stderr)
        return 1
    text = REPORT.read_text()
    for line in text.splitlines():
        if line.startswith(("**", "Found")):
            print(line)
    bad = sum(int(w) for ln in text.splitlines() if ln.startswith("Found")
              for w in ln.split() if w.isdigit())
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
