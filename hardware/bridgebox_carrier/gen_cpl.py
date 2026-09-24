#!/usr/bin/env python3
"""
Turn KiCad's position file into the CPL an assembly house will accept.

kicad-cli writes `Ref,Val,Package,PosX,PosY,Rot,Side`. JLCPCB's parser looks
for `Designator,Mid X,Mid Y,Layer,Rotation` and rejects the file outright when
it cannot find them -- which reads as "failed processing the CPL file" and says
nothing about why.

The COORDINATES are already right and are left alone. KiCad negates Y in both
the position file and the gerbers, so the two agree; rewriting them to a
bottom-left origin here would break that agreement unless the gerbers moved
too.

    python3 gen_cpl.py
"""

from __future__ import annotations

import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "out" / "cpl-smd.csv"
DST = HERE / "out" / "cpl-jlcpcb.csv"
SRC_ALL = HERE / "out" / "cpl-all.csv"
DST_ALL = HERE / "out" / "cpl-jlcpcb-all.csv"


def convert(src: Path, dst: Path) -> int:
    """Rewrite one kicad-cli position file into JLCPCB's column names.

    :param src: the kicad-cli csv
    :param dst: where to write the JLCPCB-shaped file
    :return int: how many placements were written
    """
    rows = list(csv.DictReader(src.open()))
    with dst.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            w.writerow([r["Ref"],
                        f'{float(r["PosX"]):.4f}mm',
                        f'{float(r["PosY"]):.4f}mm',
                        r["Side"].capitalize(),
                        f'{float(r["Rot"]):.4f}'])
    return len(rows)


def main() -> None:
    n = convert(SRC, DST)
    print(f"wrote {DST.name}: {n} placements")
    # The all-parts twin, to go with bom-jlcpcb-all.csv. Mounting holes are
    # not in either file: kicad-cli leaves them out of the position export
    # because they have no pads, which is the right answer.
    if SRC_ALL.exists():
        rows = list(csv.DictReader(SRC_ALL.open()))
        n = convert(SRC_ALL, DST_ALL)
        print(f"wrote {DST_ALL.name}: {n} placements, THT included")
        for r in rows:
            print(f"  {r['Ref']:4s} {r['Package'][:34]:34s} "
                  f"rot {float(r['Rot']):6.1f}")


if __name__ == "__main__":
    main()
