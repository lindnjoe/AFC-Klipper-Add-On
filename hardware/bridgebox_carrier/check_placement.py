#!/usr/bin/env python3
"""
Check the placement for collisions.

Courtyards are the part of a footprint that says "nothing else here", so
overlapping them is the definition of a collision. Eyeballing a render misses
these -- a mounting hole sitting inside the Pico's outline looks like a hole
next to a lot of pads.

    python3 check_placement.py
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_pcb as p  # noqa: E402

Box = tuple[float, float, float, float]


def courtyard(text: str) -> Box | None:
    """Bounding box of the F.CrtYd graphics, in footprint coordinates."""
    xs: list[float] = []
    ys: list[float] = []
    for m in re.finditer(r"\(fp_(line|rect) \(start ([-\d.]+) ([-\d.]+)\) "
                         r"\(end ([-\d.]+) ([-\d.]+)\)", text):
        blk = p.sexp(text, m.start())
        if "F.CrtYd" not in blk:
            continue
        xs += [float(m.group(2)), float(m.group(4))]
        ys += [float(m.group(3)), float(m.group(5))]
    for m in re.finditer(r"\(fp_circle \(center ([-\d.]+) ([-\d.]+)\) "
                         r"\(end ([-\d.]+) ([-\d.]+)\)", text):
        blk = p.sexp(text, m.start())
        if "F.CrtYd" not in blk:
            continue
        cx, cy = float(m.group(1)), float(m.group(2))
        r = math.hypot(float(m.group(3)) - cx, float(m.group(4)) - cy)
        xs += [cx - r, cx + r]
        ys += [cy - r, cy + r]
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def placed(box: Box, px: float, py: float, rot: int) -> Box:
    """Footprint-space box to board space, through the placement rotation."""
    a = math.radians(rot)
    ca, sa = math.cos(a), math.sin(a)
    xs, ys = [], []
    for lx in (box[0], box[2]):
        for ly in (box[1], box[3]):
            xs.append(px + lx * ca + ly * sa)
            ys.append(py - lx * sa + ly * ca)
    return min(xs), min(ys), max(xs), max(ys)


def overlap(a: Box, b: Box) -> bool:
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def main() -> int:
    fps = {ref: fp for ref, _l, _v, fp, *_ in p.g.PARTS}
    boxes: dict[str, Box] = {}

    for ref, fp in fps.items():
        cy = courtyard(p.load_fp(fp))
        if cy is None:
            print(f"  note  {ref} ({fp}) has no courtyard, skipped")
            continue
        px, py, rot = p.PLACE[ref]
        boxes[ref] = placed(cy, px, py, rot)

    hole = courtyard(p.load_fp("MountingHole:MountingHole_3.2mm_M3"))
    for i, (hx, hy) in enumerate(p.HOLES, 1):
        boxes[f"H{i}"] = placed(hole, hx, hy, 0)

    bad = 0
    names = sorted(boxes)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if overlap(boxes[a], boxes[b]):
                bad += 1
                print(f"  COLLIDE  {a} and {b}")
                print(f"           {a}: {tuple(round(v, 2) for v in boxes[a])}")
                print(f"           {b}: {tuple(round(v, 2) for v in boxes[b])}")

    board = (0.0, 0.0, p.W, p.H)
    for ref, bx in boxes.items():
        # A connector is allowed to overhang the edge it mates through; every
        # other part crossing the outline is a mistake.
        if ref in ("J1", "J2", "U1"):
            continue
        if not (board[0] <= bx[0] and board[1] <= bx[1]
                and bx[2] <= board[2] and bx[3] <= board[3]):
            bad += 1
            print(f"  OFF-BOARD  {ref}: {tuple(round(v, 2) for v in bx)}")

    print("\nRESULT:", "placement is clear" if not bad
          else f"{bad} collision(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
