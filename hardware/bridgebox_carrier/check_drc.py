#!/usr/bin/env python3
"""
Clearance check for the routed board.

kicad-cli 7 has no DRC subcommand, so this stands in for it. It is not a
general DRC -- it checks the things this board can actually get wrong:

  * a track passing too close to a pad of a DIFFERENT net
  * two tracks of different nets too close together
  * a track or via too close to the board edge
  * a via too close to a pad or a track of another net
  * a track passing through a mounting hole
  * a net that is not fully joined by its own tracks

That last one matters most. Everything else here is geometry; connectivity is
the thing a picture cannot show you, and a net split into two islands looks
exactly like a routed net until the board arrives.

    python3 check_drc.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as G  # noqa: E402
import route as R  # noqa: E402

CLEAR = 0.2          # minimum copper-to-copper, mm
EDGE = 0.3           # minimum copper-to-board-edge, mm
JOIN = 0.05          # two points this close are the same node
HOLE_R = 1.6         # M3 clearance hole, 3.2 mm drilled


def seg_point(ax: float, ay: float, bx: float, by: float,
              px: float, py: float) -> float:
    """Distance from point p to segment ab."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def seg_seg(a: tuple, b: tuple) -> float:
    """Distance between two segments, 0 if they intersect."""
    ax, ay, bx, by = a
    cx, cy, dx_, dy_ = b

    def cross(ox, oy, px, py, qx, qy):
        return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)

    d1 = cross(ax, ay, bx, by, cx, cy)
    d2 = cross(ax, ay, bx, by, dx_, dy_)
    d3 = cross(cx, cy, dx_, dy_, ax, ay)
    d4 = cross(cx, cy, dx_, dy_, bx, by)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(seg_point(ax, ay, bx, by, cx, cy),
               seg_point(ax, ay, bx, by, dx_, dy_),
               seg_point(cx, cy, dx_, dy_, ax, ay),
               seg_point(cx, cy, dx_, dy_, bx, by))


def pad_clearance(pad: G.Pad, ax: float, ay: float,
                  bx: float, by: float) -> float:
    """Gap between a segment centreline and a pad's copper."""
    x, y, w, h, _shape = pad
    # Treat the pad as a rectangle; for round pads this is pessimistic at the
    # corners, which is the safe direction.
    best = float("inf")
    for t in [i / 40 for i in range(41)]:
        px, py = ax + (bx - ax) * t, ay + (by - ay) * t
        ddx = max(abs(px - x) - w / 2, 0.0)
        ddy = max(abs(py - y) - h / 2, 0.0)
        best = min(best, math.hypot(ddx, ddy))
    return best


def main() -> int:
    segs = R.segments()
    pads = G.all_pads()
    net_of_pad = {}
    for net, pins in G.p.g.NETS.items():
        for ref, num in pins:
            net_of_pad[(ref, num)] = net

    bad = 0

    # 1. tracks vs pads of other nets
    for (ax, ay, bx, by, w, net) in segs:
        for ref, pl in pads.items():
            for num, pad in pl.items():
                if net_of_pad.get((ref, num)) == net:
                    continue
                gap = pad_clearance(pad, ax, ay, bx, by) - w / 2
                if gap < CLEAR:
                    bad += 1
                    print(f"  CLEARANCE  {net} track vs {ref}.{num}"
                          f" ({net_of_pad.get((ref, num), 'no net')}):"
                          f" {gap:.2f} mm at ({ax:.1f},{ay:.1f})-"
                          f"({bx:.1f},{by:.1f})")

    # 2. tracks vs tracks of other nets
    for i, s in enumerate(segs):
        for t in segs[i + 1:]:
            if s[5] == t[5]:
                continue
            gap = seg_seg(s[:4], t[:4]) - s[4] / 2 - t[4] / 2
            if gap < CLEAR:
                bad += 1
                print(f"  CLEARANCE  {s[5]} vs {t[5]}: {gap:.2f} mm"
                      f"  near ({s[0]:.1f},{s[1]:.1f})")

    # 3. vias
    for (vx, vy) in R.GND_VIAS:
        for ref, pl in pads.items():
            for num, pad in pl.items():
                if net_of_pad.get((ref, num)) == "GND":
                    continue
                gap = pad_clearance(pad, vx, vy, vx, vy) - R.VIA_D / 2
                if gap < CLEAR:
                    bad += 1
                    print(f"  CLEARANCE  GND via at ({vx},{vy}) vs "
                          f"{ref}.{num}: {gap:.2f} mm")
        for s in segs:
            if s[5] == "GND":
                continue
            gap = seg_point(*s[:4], vx, vy) - R.VIA_D / 2 - s[4] / 2
            if gap < CLEAR:
                bad += 1
                print(f"  CLEARANCE  GND via at ({vx},{vy}) vs {s[5]}"
                      f" track: {gap:.2f} mm")

    # 4. board edge
    for (ax, ay, bx, by, w, net) in segs:
        for x, y in ((ax, ay), (bx, by)):
            m = min(x, y, G.p.W - x, G.p.H - y)
            if m - w / 2 < EDGE:
                bad += 1
                print(f"  EDGE  {net} track at ({x:.1f},{y:.1f}) is"
                      f" {m - w / 2:.2f} mm from the outline")

    # 5. tracks vs the mounting holes
    #
    # A BLIND SPOT UNTIL IT WAS LOOKED FOR. Mounting holes are footprints
    # whose only pad is unnamed, and geometry.pads_of drops unnamed pads as
    # mechanical -- so a hole is invisible to check 1, and KiCad's own DRC
    # reports it as a library warning rather than a clearance error. A track
    # laid straight through a 3.2 mm hole would have passed everything.
    for (hx, hy) in G.p.HOLES:
        for (ax, ay, bx, by, w, net) in segs:
            gap = seg_point(ax, ay, bx, by, hx, hy) - w / 2 - HOLE_R
            if gap < CLEAR:
                bad += 1
                print(f"  HOLE  {net} track passes {gap:.2f} mm from the"
                      f" mounting hole at ({hx}, {hy})")

    # 6. connectivity -- every pad of a net reachable through its own tracks
    for net, pins in G.p.g.NETS.items():
        if net == "GND":
            continue          # poured, checked by eye in the zone
        nodes = [(round(pads[r][n][0], 3), round(pads[r][n][1], 3))
                 for r, n in pins if n in pads[r]]
        # A hop leaves F.Cu and comes back; for connectivity it joins its two
        # ends just like a track does.
        mine = [s for s in segs if s[5] == net]
        mine += [(a[0], a[1], b[0], b[1], 0.0, net)
                 for a, b, hn in R.HOPS if hn == net]
        if not mine:
            bad += 1
            print(f"  OPEN  {net} has no tracks at all")
            continue
        seen = {nodes[0]}
        changed = True
        while changed:
            changed = False
            for (ax, ay, bx, by, _w, _n) in mine:
                a, b = (round(ax, 3), round(ay, 3)), (round(bx, 3), round(by, 3))
                for u, v in ((a, b), (b, a)):
                    if any(math.dist(u, s) < JOIN for s in seen) and \
                       not any(math.dist(v, s) < JOIN for s in seen):
                        seen.add(v)
                        changed = True
        for nd in nodes:
            if not any(math.dist(nd, s) < JOIN for s in seen):
                bad += 1
                who = [f"{r}.{n}" for r, n in pins
                       if n in pads[r]
                       and round(pads[r][n][0], 3) == nd[0]
                       and round(pads[r][n][1], 3) == nd[1]]
                print(f"  OPEN  {net}: {who or nd} is not connected")

    print("\nRESULT:", "board passes" if not bad else f"{bad} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
