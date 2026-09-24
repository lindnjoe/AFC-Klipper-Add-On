#!/usr/bin/env python3
"""
Check the placement: courtyards, the board edge, and the antenna keepout.

Placement is the part of a layout that decides whether it is any good -- a
router can only work with what the placer left it -- and it is checkable
without a router, so it is checked first and separately.

Three things, in the order they bite:

  * COURTYARD OVERLAP. Two parts whose courtyards intersect cannot both be
    assembled. This is the one that silently survives to the fab house.
  * OFF-BOARD PADS. A pad outside Edge.Cuts is not a manufacturing error, it
    is a missing connection, and it looks fine on screen.
  * THE ANTENNA KEEPOUT. The WROOM-1 footprint declares tracks, vias, pads and
    pour all not_allowed over a 48 x 21 mm rectangle past the antenna. Any pad
    inside it is a WiFi problem that will not show up until the board is in a
    printer, next to a steel frame, ten feet from the access point.

Usage:  python3 check_placement.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import design
import fp_lib
from verify_design import DEFAULT_LIBS, Libs

# How close two courtyards may come. KiCad's own default is 0; a little air
# makes a board assemblable by hand as well as by machine.
COURTYARD_GAP = 0.15


def rect_of(fp: fp_lib.Footprint, x: float, y: float, rot: float
            ) -> tuple[float, float, float, float]:
    """A part's courtyard in board coordinates, as an axis-aligned box.

    Rotating a box and re-bounding it is conservative at angles that are not
    multiples of 90 -- it reports a slightly larger part than reality. Every
    rotation on this board is a multiple of 90, where it is exact.
    """
    x0, y0, x1, y1 = fp.extent()
    pts = [fp_lib.place(px, py, rot, x, y)
           for px, py in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def overlap(a: tuple[float, float, float, float],
            b: tuple[float, float, float, float], gap: float) -> bool:
    return not (a[2] + gap <= b[0] or b[2] + gap <= a[0]
                or a[3] + gap <= b[1] or b[3] + gap <= a[1])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)
    bad: list[str] = []

    fps = {ref: fp_lib.load(libs.fp, part["fp"])
           for ref, part in design.PARTS.items()}
    missing = set(design.PARTS) - set(design.PLACE)
    if missing:
        bad.append(f"unplaced parts: {sorted(missing)}")

    boxes: dict[str, tuple[float, float, float, float]] = {}
    for ref, (x, y, rot) in design.PLACE.items():
        boxes[ref] = rect_of(fps[ref], x, y, rot)

    # ── courtyards ──────────────────────────────────────────────────────────
    # U1's "courtyard" is mostly its antenna keepout, which is deliberately off
    # the board and must not be treated as body. Compare it on its pads only.
    refs = sorted(boxes)
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            ba, bb = boxes[a], boxes[b]
            if a == "U1" or b == "U1":
                u, o = (a, b) if a == "U1" else (b, a)
                pads = [p for p in fps[u].pads if p.electrical]
                xs, ys = [], []
                for p in pads:
                    px, py = fp_lib.place(p.x, p.y, design.PLACE[u][2],
                                          design.PLACE[u][0],
                                          design.PLACE[u][1])
                    xs += [px - p.w / 2, px + p.w / 2]
                    ys += [py - p.h / 2, py + p.h / 2]
                ba = (min(xs), min(ys), max(xs), max(ys))
                bb = boxes[o]
                if u == b:
                    ba, bb = bb, ba
            if overlap(ba, bb, COURTYARD_GAP):
                bad.append(f"courtyard overlap: {a} and {b}")

    # ── pads inside the board ───────────────────────────────────────────────
    for ref, (x, y, rot) in design.PLACE.items():
        for p in fps[ref].pads:
            if not p.electrical:
                continue
            px, py = fp_lib.place(p.x, p.y, rot, x, y)
            if not (0 <= px <= design.BOARD_W and 0 <= py <= design.BOARD_H):
                bad.append(f"{ref}.{p.number} outside the board "
                           f"at ({px:.2f}, {py:.2f})")

    # ── the antenna keepout ─────────────────────────────────────────────────
    ux, uy, urot = design.PLACE["U1"]
    kx0, ky0, kx1, ky1 = design.ANTENNA_KEEPOUT
    corners = [fp_lib.place(px, py, urot, ux, uy)
               for px, py in ((kx0, ky0), (kx1, ky0), (kx1, ky1), (kx0, ky1))]
    kb = (min(c[0] for c in corners), min(c[1] for c in corners),
          max(c[0] for c in corners), max(c[1] for c in corners))
    for ref, (x, y, rot) in design.PLACE.items():
        for p in fps[ref].pads:
            if not p.electrical:
                continue
            px, py = fp_lib.place(p.x, p.y, rot, x, y)
            if kb[0] <= px <= kb[2] and kb[1] <= py <= kb[3]:
                bad.append(f"{ref}.{p.number} is inside the antenna keepout")
    on_board = max(0.0, min(kb[3], design.BOARD_H) - max(kb[1], 0.0)) * \
        max(0.0, min(kb[2], design.BOARD_W) - max(kb[0], 0.0))

    # ── do the connectors face OUT? ─────────────────────────────────────────
    # See design.MATING. This is the check that was missing: it is not about
    # copper, so no DRC anywhere would ever have asked it.
    EDGE_TOL = 1.5          # mm a face may sit inside the outline
    for ref, (axis, may_overhang, why) in sorted(
            getattr(design, "MATING", {}).items()):
        if ref not in design.PLACE:
            bad.append(f"MATING names {ref}, which is not placed")
            continue
        fab = fps[ref].fab_extent()
        if fab is None:
            bad.append(f"{ref}: no F.Fab body outline -- cannot tell which way "
                       f"it faces")
            continue
        x, y, rot = design.PLACE[ref]
        fx0, fy0, fx1, fy1 = fab
        local = {"+x": ((fx1 + fx1) / 2, (fy0 + fy1) / 2),
                 "-x": (fx0, (fy0 + fy1) / 2),
                 "+y": ((fx0 + fx1) / 2, fy1),
                 "-y": ((fx0 + fx1) / 2, fy0)}[axis]
        if axis == "+x":
            local = (fx1, (fy0 + fy1) / 2)
        bx, by = fp_lib.place(local[0], local[1], rot, x, y)
        W, H = design.BOARD_W, design.BOARD_H
        # distance OUTSIDE each edge: positive means past it
        out = {"left": -bx, "right": bx - W, "top": -by, "bottom": by - H}
        edge = max(out, key=out.get)
        d = out[edge]
        where = f"({bx:.2f}, {by:.2f})"
        if d < -EDGE_TOL:
            bad.append(f"{ref}: mating face at {where} is {-d:.1f} mm inside "
                       f"the outline -- no cable reaches it. {why}")
        elif d > 0 and not may_overhang:
            bad.append(f"{ref}: mating face at {where} sits {d:.1f} mm past "
                       f"the {edge} edge and is not marked as overhanging -- "
                       f"it would foul a case. {why}")
        else:
            note = "protrudes" if d > 0 else "flush"
            print(f"{ref} faces the {edge} edge at {where} -- {note} "
                  f"({abs(d):.2f} mm)")

    print(f"board {design.BOARD_W:.0f} x {design.BOARD_H:.0f} mm, "
          f"{len(design.PLACE)} parts")
    print(f"antenna keepout {kb[0]:.1f},{kb[1]:.1f} to {kb[2]:.1f},{kb[3]:.1f} "
          f"-- {on_board:.0f} mm^2 of it lands on the board "
          f"({'clear' if on_board < 1 else 'MUST BE EMPTY'})")
    if bad:
        print(f"\n{len(bad)} PROBLEM(S):")
        for b in bad[:30]:
            print(f"  ! {b}")
        return 1
    print("placement OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
