#!/usr/bin/env python3
"""
The BOM. There is no assembly BOM any more.

Every part left on this board is through-hole, so there is nothing for a
pick-and-place to do: no BOM/CPL upload, no Economic-versus-Standard question,
no setup fee. Order bare boards and solder five parts.

bom-smd.csv is still written, and is still correct -- it is empty.

The assembly BOM is in the column order JLCPCB expects (Comment, Designator,
Footprint, LCSC Part #) and covers ONLY the surface-mount parts, because the
through-hole ones -- the two connectors and the fuse clips -- are not in their
library and consigning parts costs more in fees than the parts are worth.

LCSC numbers are left blank on purpose. A wrong one is not a build error, it
is the wrong component soldered to five boards, and JLCPCB's parts picker
matches on value and footprint in the ordering UI anyway. Fill them in there,
or here once you have chosen.

    python3 gen_bom.py
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_pcb as P  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

# Anything a distributor needs that the footprint does not say.
NOTES = {
    "J1": "Barrel jack 5.5 x 2.1 mm, CUI PJ-102AH",
    "J2": "Micro-Fit 3.0 header, 2x2, right angle, Molex 43045-0400",
    "F1": "Fuse clips for 5x20 mm, Keystone 3512 -- TWO per board",
    "U1": "Raspberry Pi Pico (or Pico-form-factor RP2040 board)",
    "U2": "MAX3485 RS-485 breakout, 3.3 V, EN broken out",
    "JP1": "solder jumper, no part",
    "JP2": "solder jumper, no part",
}
EXTRA = [("--", "Fuse", "5x20 mm cartridge, TIME-DELAY (T), 250 V. Rate it at "
                        "or just below the adapter -- a 3 A fuse on a 2 A "
                        "supply never opens. e.g. T2AL250V"),
         ("--", "Headers", "2x20 female 2.54 mm for U1; 1x5 + 1x3 for U2")]


def main() -> None:
    OUT.mkdir(exist_ok=True)
    smd, tht = [], []
    for ref, _lib, val, fp, *_ in P.g.PARTS:
        is_th = "thru_hole" in P.load_fp(fp)
        (tht if is_th else smd).append((ref, val, fp.split(":")[1]))

    with (OUT / "bom-smd.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        # One line per part type is what keeps the extended-part fee down, so
        # group by identical value and footprint.
        groups: dict[tuple[str, str], list[str]] = {}
        for ref, val, fp in smd:
            if ref.startswith("JP"):
                continue          # solder jumpers are copper, not components
            groups.setdefault((val, fp), []).append(ref)
        for (val, fp), refs in sorted(groups.items()):
            w.writerow([val, ",".join(sorted(refs)), fp, ""])

    with (OUT / "bom-full.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Value", "Footprint", "Mount", "Notes"])
        for ref, val, fp in sorted(smd + tht):
            w.writerow([ref, val, fp,
                        "THT" if (ref, val, fp) in tht else "SMD",
                        NOTES.get(ref, "")])
        for ref, val, note in EXTRA:
            w.writerow([ref, val, "", "", note])

    # The SAME board as an assembly BOM covering EVERY part, through-hole
    # included. bom-smd.csv is the file to upload for a normal SMT order and
    # it is empty because there is nothing surface-mount here; this one exists
    # for the case where the parts are consigned, or a house that does
    # through-hole assembly is quoting. Nothing on this board is in JLCPCB's
    # own library, so every LCSC field is necessarily blank.
    with (OUT / "bom-jlcpcb-all.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        groups: dict[tuple[str, str], list[str]] = {}
        for ref, val, fp in smd + tht:
            if ref.startswith("JP"):
                continue
            groups.setdefault((val, fp), []).append(ref)
        for (val, fp), refs in sorted(groups.items()):
            w.writerow([val, ",".join(sorted(refs)), fp, ""])
    print(f"bom-jlcpcb-all.csv: {len(groups)} part types, THT included")

    n = len({(v, f) for _r, v, f in smd if not _r.startswith("JP")})
    print(f"bom-smd.csv : {n} unique SMD part types for assembly")
    print(f"bom-full.csv: {len(smd) + len(tht) + len(EXTRA)} lines, everything")


if __name__ == "__main__":
    main()
