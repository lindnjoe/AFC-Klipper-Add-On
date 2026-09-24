#!/usr/bin/env python3
"""
Routes for the BridgeBox carrier, as explicit polylines.

Not an autorouter. Each net's path is written out below and check_drc.py then
proves the whole set is legal -- which is the only reason hand-written
waypoints are safe to trust.

EVERYTHING IS ON F.Cu, and now genuinely everything: B.Cu is the ground plane
and it is whole. The previous revision spent one via pair hopping a jumper
link; with the jumpers gone there is nothing left to hop over.

THE ONE AWKWARD NET IS EN.

The Pico presents RXD, TXD, EN left to right along its bottom row. The module
wants EN, VCC, RXD, TXD left to right along its logic row. RXD and TXD keep
their order between the two, so they nest; EN reverses against both of them,
so no lane assignment can carry all three across without a crossing.

So EN goes the other way round the board: UP into the empty gap between the
Pico's two pad rows, right along it, down the free strip past the end of the
module, and back in to its pin from BELOW. The gap and the strip are both
empty copper, so the detour costs a few millimetres and buys back the via
pair the old layout needed.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as G  # noqa: E402

W_SIG = 0.3      # signal
W_BUS = 0.4      # A/B pair, kept together and a little fatter
# The 24 V rail. On 1 oz outer copper 3 A wants about 2 mm for a 10 degree
# rise, and 2 mm is also the widest that fits the bottom channel: J2's 3 mm
# polarising peg sits at y 53.3, leaving 2.88 mm between its edge and the
# board outline. 2.5 mm would not clear it, which is what settles the width.
W_PWR = 2.0

P = G.all_pads()


def at(ref: str, pad: str) -> tuple[float, float]:
    x, y, *_ = P[ref][pad]
    return round(x, 3), round(y, 3)


# net -> list of (width, [points]) polylines
ROUTES: dict[str, list[tuple[float, list[tuple[float, float]]]]] = {

    # ── 24 V, straight through ────────────────────────────────────────────
    # Jack, fuse, connector. Nothing switches it any more.
    "+24V_IN": [(W_PWR, [at("J1", "1"), (15.0, 38.0), (19.0, 34.5),
                         at("F1", "1")])],
    # The fuse now sits directly above J2, so this is a drop rather than the
    # lap of the board it used to be.
    "+24V":    [(W_PWR, [at("F1", "2"), (35.35, 38.5), (36.0, 40.0),
                         at("J2", "4")])],

    # ── Pico to module ────────────────────────────────────────────────────
    # Three nested lanes between the two rows. Depth is assigned by where each
    # net LANDS, not where it starts: the leftmost pin on the module takes the
    # deepest lane, so no net has to climb across another's horizontal.
    "TXD": [(W_SIG, [at("U1", "2"), (7.54, 24.0), (54.62, 24.0),
                     at("U2", "4")])],
    "RXD": [(W_SIG, [at("U1", "1"), (5.0, 25.5), (52.08, 25.5),
                     at("U2", "3")])],
    # Out along the top, down the left margin past the end of the Pico's pads,
    # then in on the deepest lane. 3V3 is on the Pico's FAR row, so it has to
    # get past the near one somewhere; x = 3.2 is the only gap that is free
    # for the whole height.
    "+3V3": [(W_SIG, [at("U1", "36"), (15.16, 2.4), (3.2, 2.4), (3.2, 26.5),
                      (49.54, 26.5), at("U2", "2")])],
    # The long way round -- see the note at the top of this file.
    "EN":  [(W_SIG, [at("U1", "4"), (12.62, 19.0), (59.5, 19.0), (59.5, 31.5),
                     (47.0, 31.5), at("U2", "1")])],

    # ── the bus ───────────────────────────────────────────────────────────
    # A and B cross here rather than at a jumper: the module's A pin feeds the
    # connector's B and vice versa, which is the pairing the working board
    # proved. The crossover costs nothing in copper because the two nets keep
    # their left-to-right order at both ends -- it is only the LABELS that
    # swap.
    #
    # BUS_A goes straight in. BUS_B has to reach the pad BEHIND it, so it
    # steps out to the right of the module first and comes back along the
    # bottom; diving straight down would cut across BUS_A's approach.
    "BUS_A": [(W_BUS, [at("U2", "8"), (54.62, 45.0), at("J2", "1")])],
    "BUS_B": [(W_BUS, [at("U2", "7"), (52.08, 41.5), (57.5, 41.5),
                       (57.5, 47.0), (33.5, 47.0), (33.5, 45.5),
                       at("J2", "2")])],
}

# No ground vias any more. Every remaining part is through-hole, so its pads
# already pass through the board and meet the B.Cu pour on their own.
GND_VIAS: list[tuple[float, float]] = []
GND_STUBS: list[tuple[tuple, tuple]] = []

# NO LAYER CHANGES. The old layout needed one via pair to get A_MOD past
# BUS_A's link between the two jumpers; with the jumpers gone that link does
# not exist and the ground plane is unbroken end to end.
HOPS: list[tuple[tuple, tuple, str]] = []

# Footprints with more than one pad of the same number need those pads joined
# by copper -- see geometry.dup_pads. The fuse holder is the one here.
DUP_JOINS: list[tuple[float, tuple, tuple]] = []
for _ref in ("F1",):
    for _num, _ps in G.dup_pads(_ref).items():
        for _a, _b in zip(_ps, _ps[1:]):
            DUP_JOINS.append((W_PWR, (_a[0], _a[1]), (_b[0], _b[1])))

VIA_D, VIA_DRILL = 0.8, 0.4


def segments() -> list[tuple[float, float, float, float, float, str]]:
    """Flatten the polylines into (x1, y1, x2, y2, width, net)."""
    out = []
    items = list(ROUTES.items()) + [("GND", [(W_SIG, [a, b])
                                             for a, b in GND_STUBS])]
    items += [("+24V_IN", [(DUP_JOINS[0][0],
                            [DUP_JOINS[0][1], DUP_JOINS[0][2]])]),
              ("+24V", [(DUP_JOINS[1][0],
                         [DUP_JOINS[1][1], DUP_JOINS[1][2]])])]
    # The hop itself is on B.Cu and so cannot clash with anything on F.Cu.
    for net, polys in items:
        for width, pts in polys:
            for a, b in zip(pts, pts[1:]):
                if a == b:
                    continue
                out.append((a[0], a[1], b[0], b[1], width, net))
    return out


if __name__ == "__main__":
    segs = segments()
    total = sum(((s[2] - s[0]) ** 2 + (s[3] - s[1]) ** 2) ** 0.5 for s in segs)
    print(f"{len(segs)} segments, {total:.0f} mm of track, "
          f"{len(GND_VIAS)} ground vias")
