#!/usr/bin/env python3
"""
The BOM and the placement file, split the way an assembly house needs them.

The carrier's gen_bom.py opens by saying there is no assembly BOM any more,
because every part left on that board was through-hole. **This board is the
other way round**: 25 of its 28 parts are surface-mount, including a 41-pad
module and a 0.5 mm-pitch USB-C receptacle. Hand-soldering that is a choice,
not the default, so the assembly files are the point here rather than a
formality.

Three parts are through-hole and stay off the SMD files: the barrel jack, the
Micro-Fit, and nothing else -- the USB-C's shield tabs are through-hole on an
otherwise SMD part, which is why this works off PAD TYPE per part rather than a
hand-kept list. A part counts as through-hole only if ALL its electrical pads
are; a mixed part is SMD with some tabs, and the tabs are somebody's soldering
iron either way.

LCSC numbers are left blank on purpose, for the reason the carrier gives and
which has not changed: a wrong one is not a build error, it is the wrong
component soldered to every board in the batch. JLCPCB's picker matches on
value and footprint in the ordering UI. Fill them in there, or here once
chosen.

    python3 gen_bom.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import design
import fp_lib
from verify_design import DEFAULT_LIBS, Libs

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)
    OUT.mkdir(exist_ok=True)

    rows: list[tuple[str, str, str, bool]] = []
    for ref, part in sorted(design.PARTS.items()):
        fp = fp_lib.load(libs.fp, part["fp"])
        elec = [p for p in fp.pads if p.electrical]
        # ══ A HOLE IS NOT A PART TO PLACE. ══
        #
        # A part with NO electrical pads is a mounting hole, a fiducial or a
        # logo -- there is nothing for a machine to pick and nothing to buy.
        # The four M3 holes added for the case landed on the assembly BOM the
        # first time this ran, because "not all pads are through-hole" is
        # vacuously true of a part with no pads at all.
        if not elec:
            continue
        tht = all(p.ptype == "thru_hole" for p in elec)
        # ══ THE COMMENT COLUMN IS NOT THE SCHEMATIC LABEL. ══
        #
        # `value` is what gets printed on the schematic, and for several parts
        # that is a ROLE and not a part: D1 said "VBUS OR", J3 said "USB-C
        # device", the buttons said BOOT and RESET. Useful on a drawing, and
        # not a thing an assembly house can buy -- "VBUS OR" in a Comment
        # column either stalls the order or gets guessed at.
        #
        # So parts whose label is not a description carry an explicit `bom`,
        # and the schematic keeps the label. Everything whose value IS its
        # value (100n, 10k, MAX3485ESA) needs neither.
        rows.append((ref, part.get("bom", part["value"]),
                     part["fp"].split(":", 1)[1], tht))

    # group identical (value, footprint) pairs -- an assembly house wants one
    # line per part number with the designators on it, not one line per part.
    def write(path: Path, keep) -> int:
        groups: dict[tuple[str, str], list[str]] = {}
        for ref, value, fp_name, tht in rows:
            if not keep(tht):
                continue
            groups.setdefault((value, fp_name), []).append(ref)
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
            for (value, fp_name), refs in sorted(groups.items()):
                w.writerow([value, ",".join(sorted(refs)), fp_name, ""])
        return len(groups)

    n_smd = write(OUT / "bom-smd.csv", lambda tht: not tht)
    n_all = write(OUT / "bom-all.csv", lambda tht: True)
    tht_refs = [r for r, _v, _f, tht in rows if tht]

    print(f"bom-smd.csv: {n_smd} line(s) for "
          f"{sum(1 for r in rows if not r[3])} placed parts")
    print(f"bom-all.csv: {n_all} line(s) for {len(rows)} parts")
    print(f"through-hole, not on the SMD files: {', '.join(tht_refs) or 'none'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
