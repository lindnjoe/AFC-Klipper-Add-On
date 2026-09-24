#!/usr/bin/env python3
"""
Route the BridgeBox S3 board: two signal layers, GND poured on both.

A grid router, not a hand-authored track list. The carrier's route.py spells
its 32 segments out as literal polylines, which is reasonable for 5 parts and
10 nets and is not reasonable for 28 and 19 -- and a hand-placed polyline that
is 0.05 mm too close to a pad looks exactly like one that is not.

THIS STARTED AS A ONE-LAYER ROUTER AND THE BOARD REFUSED IT -- but not for the
reason first written down. Every track on F.Cu with B.Cu an unbroken plane is
the carrier's arrangement, and it got 29 of 39 connections here. The four USB
failures were recorded as geometry: "tying A6 to B6 means threading between
pads on a 0.5 mm pitch, and no placement fixes that." That was wrong twice
over. The footprint puts all sixteen contacts in ONE row, so nothing is being
threaded between anything -- each pad escapes straight out. And the escapes
were blocked by the router's own raster, not by the board.

Both of those are fixed below and both were needed:

  * the pads sat half a grid cell off the grid (`design.PLACE["J3"]`), and
  * obstacles were rasterised, which quantises a pad EDGE to ±0.05 mm when the
    margin being decided is 0.025 mm (see RectObs).

The board does still want two layers -- it went from 29 connections to 46 --
but that is congestion and fan-out, which is an ordinary reason, and the
striking claim that replaced it was never true.

The shape of the problem now:

  * two layers, 0.1 mm grid, a via to change between them. Vias cost about
    1.5 mm of detour, so the search takes one when it must and not for fun.
  * GND is not routed as tracks. Each SMD ground pad drops a via into the pour;
    through-hole pads already reach both sides by being through-hole.
  * BUS_A, BUS_B and SW are PRICED onto F.Cu, not banned from the back -- a ban
    is what made BUS_B unroutable, since it forbids a 2 mm hop as firmly as a
    40 mm one. What they actually did is reported per net at the end.
  * everything else is an A* search against an exact-distance mask. A route
    that cannot be found is reported, not fudged.

Turn cost is deliberate: without it the search wanders diagonally and produces
copper that is technically legal and looks like spaghetti. With it, tracks run
straight and turn when they must.

Two things check the result rather than trusting it. `pour_islands` asks what
the ground pours actually became, because a second signal layer buys the
connections by carving the ground up. And `check_drc.py` re-reads the finished
board at full precision -- the grid is an approximation and the tightest gap on
this board is a quarter of a cell, so the router's own answer cannot be the
last word on whether it is legal.

Run:  python3 route.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import heapq
import math
import re
import sys
from collections import deque
from pathlib import Path

import numpy as np

import design
import fp_lib
from verify_design import DEFAULT_LIBS, Libs

HERE = Path(__file__).resolve().parent
GRID = 0.1                  # mm per cell
CLEARANCE = 0.2             # mm, copper to copper
EDGE_MARGIN = 0.5           # mm, keep copper off the board edge
VIA_DIAM, VIA_DRILL = 0.6, 0.3

F_CU, B_CU = 0, 1
LAYER_NAME = {F_CU: "F.Cu", B_CU: "B.Cu"}

# A via costs this many cells of detour. Straight steps cost 1.0 and a turn
# costs 3.0, so 15 means "worth about 1.5 mm and a corner" -- dear enough that
# the router stays on one side when one side will do, cheap enough that it
# stops pretending the USB-C flip pairs are routable without one.
VIA_COST = 15.0

WIDTH = {"+24V": 2.0, "+24V_IN": 2.0, "+3V3": 0.8, "SW": 0.8,
         "USB_VBUS": 0.6}
DEFAULT_WIDTH = 0.25

# GND is the pour. Nothing else may be, and the router must never try.
PLANE_NET = "GND"

# ══ NETS THAT SHOULD STAY ON F.Cu -- AND IT IS A PRICE, NOT A BAN. ══
#
# The RS-485 pair is the entire point of the board. At 1,228,800 baud the edges
# are fast enough that the return path matters, and the return path is the pour
# directly under the track. A LONG run on B.Cu would slice that pour lengthwise
# beneath its own partner, which is the thing to avoid. SW is the buck's switch
# node -- small, loud, and kept next to its inductor for the same reason.
#
# This was a hard ban first, and the ban is what left BUS_B unroutable: it has
# to reach J2.2 past a congested corner, and forbidding the back outright
# forbids a 2 mm hop as firmly as a 40 mm one. A ban also fails silently in the
# wrong direction -- an unroutable net is a worse outcome than a short detour.
#
# So it is priced instead. At this cost a via on the pair is worth about 40 mm
# of detour, so the router exhausts every F.Cu option first and takes the back
# only where nothing else exists. What it actually did is then reported, per
# net, rather than assumed: see the "pinned" line at the end of a run.
PINNED = {"BUS_A", "BUS_B", "SW"}
VIA_COST_PINNED = 400.0


def cells(mm: float) -> int:
    return int(round(mm / GRID))


# ══ OBSTACLES ARE GEOMETRY, NOT PIXELS. ══
#
# The first version of this router rasterised every pad into the grid and then
# dilated the result by the clearance. That is the obvious thing to do and it
# is wrong here, for a reason worth keeping.
#
# The USB-C escapes clear their neighbours by 0.025 mm -- a quarter of a grid
# cell. Rasterising a pad quantises its EDGES to ±0.05 mm, which is twice the
# entire margin, so the grid's answer was decided by rounding rather than by
# geometry. Rounding down lost the escape; rounding up lost it on the other
# side; and `40.15 / 0.1` evaluating to 401.4999999999999 in binary floating
# point lost it by a whole cell in one direction only.
#
# A cell centre, on the other hand, is exactly where the track's centreline
# really goes. So the obstacles keep their true coordinates and each cell is
# tested against them at full precision. The grid quantises the ROUTE, which is
# real, and no longer quantises the OBSTACLES, which was an artefact.
class RectObs:
    """A pad. Every rotation on this board is a multiple of 90, so it is an
    axis-aligned box and the point-to-box distance is exact and closed-form."""

    __slots__ = ("x0", "y0", "x1", "y1", "net", "layers")

    def __init__(self, cx, cy, w, h, net, layers):
        self.x0, self.x1 = cx - w / 2, cx + w / 2
        self.y0, self.y1 = cy - h / 2, cy + h / 2
        self.net, self.layers = net, layers

    def bounds(self):
        return self.x0, self.y0, self.x1, self.y1

    def dist(self, X, Y):
        dx = np.maximum(np.maximum(self.x0 - X, 0.0), X - self.x1)
        dy = np.maximum(np.maximum(self.y0 - Y, 0.0), Y - self.y1)
        return np.hypot(dx, dy)


class CapObs:
    """A track segment, or a via as a zero-length one: a capsule."""

    __slots__ = ("ax", "ay", "bx", "by", "r", "net", "layers")

    def __init__(self, ax, ay, bx, by, r, net, layers):
        self.ax, self.ay, self.bx, self.by = ax, ay, bx, by
        self.r, self.net, self.layers = r, net, layers

    def bounds(self):
        return (min(self.ax, self.bx) - self.r, min(self.ay, self.by) - self.r,
                max(self.ax, self.bx) + self.r, max(self.ay, self.by) + self.r)

    def dist(self, X, Y):
        dx, dy = self.bx - self.ax, self.by - self.ay
        L = dx * dx + dy * dy
        if L <= 1e-18:
            d = np.hypot(X - self.ax, Y - self.ay)
        else:
            t = ((X - self.ax) * dx + (Y - self.ay) * dy) / L
            t = np.clip(t, 0.0, 1.0)
            d = np.hypot(X - (self.ax + t * dx), Y - (self.ay + t * dy))
        return np.maximum(d - self.r, 0.0)


def pad_layers(p: fp_lib.Pad) -> tuple[int, ...]:
    """Which copper layers a pad actually occupies.

    Every part here is top-side, so an SMD pad exists on F.Cu ONLY -- which is
    what makes B.Cu worth having. A through-hole pad (`*.Cu`) is on both and
    blocks both.
    """
    if "*.Cu" in p.layers:
        return (F_CU, B_CU)
    if "B.Cu" in p.layers:
        return (B_CU,)
    return (F_CU,)



def pad_obstacle(px, py, w, h, shape, net, layers):
    """The right SHAPE of obstacle for a pad, not a box around everything.

    Every pad here was modelled as an axis-aligned rectangle. For a circle
    that is its bounding SQUARE, which is conservative -- the square contains
    the circle, so clearance is over-estimated and no violation can hide behind
    it -- but it is still wrong, and it costs routability at exactly the places
    that need it most: the corners of a 3 mm peg hole it does not actually
    occupy, and 21 round pads on this board including the whole Micro-Fit.

    An oval becomes a capsule, which is what an oval is.
    """
    if shape in ("circle", "oval"):
        r = min(w, h) / 2.0
        if abs(w - h) < 1e-9:
            return CapObs(px, py, px, py, r, net, layers)
        if w > h:
            d = (w - h) / 2.0
            return CapObs(px - d, py, px + d, py, r, net, layers)
        d = (h - w) / 2.0
        return CapObs(px, py - d, px, py + d, r, net, layers)
    return RectObs(px, py, w, h, net, layers)


class Board:
    def __init__(self, libs: Libs) -> None:
        self.libs = libs
        self.W, self.H = design.BOARD_W, design.BOARD_H
        self.nx, self.ny = cells(self.W) + 1, cells(self.H) + 1
        self.fps = {r: fp_lib.load(libs.fp, p["fp"])
                    for r, p in design.PARTS.items()}
        import gen_pcb
        self.assign = gen_pcb.pad_nets(libs)
        self.nets = gen_pcb.net_table()

        # every pad, in board coordinates, with the layers it occupies,
        # and its obstacle in the shape it actually is
        self.pads: list[tuple] = []
        self.obs: list = []
        for ref, (x, y, rot) in design.PLACE.items():
            for p in self.fps[ref].pads:
                px, py = fp_lib.place(p.x, p.y, rot, x, y)
                w, h = (p.w, p.h) if rot % 180 == 0 else (p.h, p.w)
                net = self.assign.get((ref, p.number), "")
                lays = pad_layers(p)
                self.pads.append((ref, p.number, net, px, py, w, h, lays))
                # A non-plated hole is not copper, but it IS a drill: the
                # keepout is the HOLE, and its own net is nobody's, so it
                # blocks every net. J2's mounting peg is 3 mm of it.
                self.obs.append(pad_obstacle(px, py, w, h, p.shape, net, lays))

        self.tracks: list[tuple[float, float, float, float, float, str, int]] = []
        self.vias: list[tuple[float, float, str]] = []

    # ── masks, computed exactly ─────────────────────────────────────────────
    def _paint(self, out: np.ndarray, o, need: float) -> None:
        """Mark every cell within `need` mm of obstacle `o`, on its layers."""
        bx0, by0, bx1, by1 = o.bounds()
        x0 = max(0, int(math.floor((bx0 - need) / GRID)))
        x1 = min(self.nx - 1, int(math.ceil((bx1 + need) / GRID)))
        y0 = max(0, int(math.floor((by0 - need) / GRID)))
        y1 = min(self.ny - 1, int(math.ceil((by1 + need) / GRID)))
        if x1 < x0 or y1 < y0:
            return
        X = np.arange(x0, x1 + 1) * GRID
        Y = (np.arange(y0, y1 + 1) * GRID)[:, None]
        m = o.dist(X, Y) < need
        for l in o.layers:
            out[l, y0:y1 + 1, x0:x1 + 1] |= m

    def _edges(self, out: np.ndarray, half: float) -> None:
        m = cells(EDGE_MARGIN + half)
        out[:, :m, :] = True
        out[:, -m:, :] = True
        out[:, :, :m] = True
        out[:, :, -m:] = True

    def blocked_for(self, net: str, width: float) -> np.ndarray:
        """Cells this net's centreline may not occupy, per layer."""
        out = np.zeros((2, self.ny, self.nx), dtype=bool)
        need = width / 2 + CLEARANCE
        for o in self.obs:
            if o.net == net:
                continue
            self._paint(out, o, need)
        self._edges(out, width / 2)
        return out

    def via_blocked_for(self, net: str) -> np.ndarray:
        """Cells where a via of this net may not be centred (either layer)."""
        out = np.zeros((2, self.ny, self.nx), dtype=bool)
        need = VIA_DIAM / 2 + CLEARANCE
        for o in self.obs:
            if o.net == net:
                continue
            self._paint(out, o, need)
        self._edges(out, VIA_DIAM / 2)
        return out[F_CU] | out[B_CU]

    def add_track(self, x1, y1, x2, y2, width, net, layer) -> None:
        self.obs.append(CapObs(x1, y1, x2, y2, width / 2, net, (layer,)))

    def add_via(self, px: float, py: float, net: str) -> None:
        """A via is copper on BOTH layers, and must be an obstacle on both.

        This is the one that would have bitten silently. The first version
        appended GND vias to the output list and never added them as obstacles,
        which was harmless while B.Cu was an unbroken plane -- the via sat
        inside its own pad's copper on the only layer being routed. With signal
        on B.Cu, an unrecorded via is a 0.6 mm obstacle the router cannot see,
        and the first symptom would have been a short on a fabricated board.
        """
        self.vias.append((px, py, net))
        self.obs.append(CapObs(px, py, px, py, VIA_DIAM / 2, net,
                               (F_CU, B_CU)))

    # ── the search ──────────────────────────────────────────────────────────
    def route_one(self, blocked: np.ndarray, via_bad: np.ndarray,
                  starts: set, goals: set, goal_rects: list,
                  via_cost: float):
        """A* from any start cell to any goal, over (x, y, layer).

        The heuristic is the Chebyshev distance to the nearest goal BOUNDING
        BOX. Every move costs at least 1.0 and covers at most one cell in each
        axis, so a cell D away needs at least D moves -- the bound is admissible,
        and turn and via costs only add to the true cost. Layer is ignored by
        it: a goal reachable on either side must not be over-estimated.

        It matters more than it looks. Plain Dijkstra explored the whole board
        for every connection; with two layers that is 8 million states a net.
        """
        if not starts or not goals:
            return None
        INF = float("inf")

        def h(x: int, y: int) -> float:
            best = INF
            for x0, y0, x1, y1 in goal_rects:
                dx = max(0, x0 - x, x - x1)
                dy = max(0, y0 - y, y - y1)
                d = max(dx, dy)
                if d < best:
                    best = d
            return best

        best: dict[tuple, float] = {}
        prev: dict[tuple, tuple] = {}
        pq: list[tuple] = []
        for (cx, cy, cl) in starts:
            k = (cx, cy, cl, -1)
            best[k] = 0.0
            heapq.heappush(pq, (h(cx, cy), 0.0, cx, cy, cl, -1))
        steps = ((1, 0), (-1, 0), (0, 1), (0, -1),
                 (1, 1), (1, -1), (-1, 1), (-1, -1))
        while pq:
            _f, d, x, y, l, dirn = heapq.heappop(pq)
            if d > best.get((x, y, l, dirn), INF):
                continue
            if (x, y, l) in goals:
                path = [(x, y, l)]
                k = (x, y, l, dirn)
                while k in prev:
                    k = prev[k]
                    path.append((k[0], k[1], k[2]))
                path.reverse()
                return path
            # in-layer moves
            for i, (dx, dy) in enumerate(steps):
                nx_, ny_ = x + dx, y + dy
                if not (0 <= nx_ < self.nx and 0 <= ny_ < self.ny):
                    continue
                if blocked[l, ny_, nx_] and (nx_, ny_, l) not in goals:
                    continue
                step = 1.0 if i < 4 else math.sqrt(2)
                # A turn costs three cells. Cheap enough to take when it must,
                # dear enough that the search does not zigzag for fun.
                turn = 0.0 if dirn in (-1, i) else 3.0
                nd = d + step + turn
                key = (nx_, ny_, l, i)
                if nd < best.get(key, INF):
                    best[key] = nd
                    prev[key] = (x, y, l, dirn)
                    heapq.heappush(pq, (nd + h(nx_, ny_), nd, nx_, ny_, l, i))
            # change layer
            if not via_bad[y, x]:
                nl = 1 - l
                if not blocked[nl, y, x] or (x, y, nl) in goals:
                    nd = d + via_cost
                    key = (x, y, nl, -1)
                    if nd < best.get(key, INF):
                        best[key] = nd
                        prev[key] = (x, y, l, dirn)
                        heapq.heappush(pq, (nd + h(x, y), nd, x, y, nl, -1))
        return None

    def pads_of(self, ref: str, num: str) -> list[int]:
        """EVERY pad with this number, not the first one.

        A pad number is not unique within a footprint and assuming it is loses
        copper silently. F1 is a fuse CLIP holder: two pads numbered 1 and two
        numbered 2, one pair per clip. The USB-C receptacle has four pads named
        S1 for its shell. Returning the first match routed 24 V to one half of
        each fuse terminal and left the other half a floating rectangle -- which
        looks entirely finished on screen, passes every clearance rule, and is
        an open circuit through the fuse.
        """
        return [i for i, p in enumerate(self.pads)
                if p[0] == ref and p[1] == num]

    def _pad(self, ref: str, num: str):
        idx = self.pads_of(ref, num)
        if not idx:
            return None
        r, n, net, px, py, w, h, lays = self.pads[idx[0]]
        return px, py, w, h, lays

    def pad_cells_at(self, i: int) -> set:
        ref, num = self.pads[i][0], self.pads[i][1]
        return self._cells_for(*self.pads[i][3:])

    def pad_size_at(self, i: int) -> float:
        return min(self.pads[i][5], self.pads[i][6])

    def pad_rect_at(self, i: int):
        _r, _n, _net, px, py, w, h, _l = self.pads[i]
        return (cells(px - w / 2), cells(py - h / 2),
                cells(px + w / 2), cells(py + h / 2))

    def _cells_for(self, px, py, w, h, lays) -> set:
        hw, hh = max(w * 0.25, 0.1), max(h * 0.25, 0.1)
        x0 = max(0, int(math.ceil((px - hw) / GRID)))
        x1 = min(self.nx - 1, int(math.floor((px + hw) / GRID)))
        y0 = max(0, int(math.ceil((py - hh) / GRID)))
        y1 = min(self.ny - 1, int(math.floor((py + hh) / GRID)))
        out = {(x, y, l) for x in range(x0, x1 + 1)
               for y in range(y0, y1 + 1) for l in lays}
        if not out:
            cx, cy = cells(px), cells(py)
            out = {(cx, cy, l) for l in lays}
        return out

    def pad_cells(self, ref: str, num: str) -> set:
        """Grid cells a track may legally END on, for this pad.

        The inner half of the pad, by exact containment -- not a rasterised
        box. A 0.30 mm pad is three cells across at best and one at worst, and
        which of those it is used to depend on where the rounding fell. If the
        inner half contains no cell centre at all, the nearest single cell is
        used, so a pad is never unreachable merely for being small.
        """
        idx = self.pads_of(ref, num)
        out: set = set()
        for i in idx:
            out |= self.pad_cells_at(i)
        return out

    def commit(self, path, net: str, width: float) -> None:
        """Turn a cell path into per-layer segments, with a via at each change."""
        runs: list[list[tuple]] = []
        for node in path:
            if runs and runs[-1][-1][2] == node[2]:
                runs[-1].append(node)
            else:
                runs.append([node])
        for ri, run in enumerate(runs):
            layer = run[0][2]
            pts = [(p[0] * GRID, p[1] * GRID) for p in run]
            keep = [pts[0]]
            for i in range(1, len(pts) - 1):
                ax, ay = pts[i][0] - keep[-1][0], pts[i][1] - keep[-1][1]
                bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
                if abs(ax * by - ay * bx) > 1e-9:
                    keep.append(pts[i])
            if pts[-1] != keep[-1]:
                keep.append(pts[-1])
            for (x1, y1), (x2, y2) in zip(keep, keep[1:]):
                self.tracks.append((x1, y1, x2, y2, width, net, layer))
                self.add_track(x1, y1, x2, y2, width, net, layer)
            if ri + 1 < len(runs):
                self.add_via(run[-1][0] * GRID, run[-1][1] * GRID, net)


def pour_islands(b: Board) -> dict:
    """Is each ground pour still ONE piece, and do the two sides agree?

    Adding signal to B.Cu buys the connections at the cost of carving the
    ground up, and a pour cut into islands is worse than no pour: the island
    under a track is a floating plate, not a return path. This rasterises what
    each pour can actually fill -- board area minus foreign copper grown by the
    pour clearance -- floods it, and reports the pieces by area.

    It is a model, not KiCad's filler: same clearance, square dilation rather
    than round, so it is slightly pessimistic. Pessimistic is the right side to
    be wrong on for a question like this.
    """
    ZONE_CLEAR = 0.4
    blocked = np.zeros((2, b.ny, b.nx), dtype=bool)
    for o in b.obs:
        if o.net == PLANE_NET:
            continue
        b._paint(blocked, o, ZONE_CLEAR)
    b._edges(blocked, 0.0)
    out = {}
    for l in (F_CU, B_CU):
        free = ~blocked[l]
        # flood fill, 4-connected: copper joins edge to edge, not at a corner
        seen = np.zeros_like(free)
        pieces = []
        ys, xs = np.nonzero(free)
        for sy, sx in zip(ys, xs):
            if seen[sy, sx]:
                continue
            q = deque([(sy, sx)])
            seen[sy, sx] = True
            n = 0
            while q:
                cy, cx = q.popleft()
                n += 1
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ay, ax = cy + dy, cx + dx
                    if (0 <= ay < b.ny and 0 <= ax < b.nx
                            and free[ay, ax] and not seen[ay, ax]):
                        seen[ay, ax] = True
                        q.append((ay, ax))
            pieces.append(n * GRID * GRID)
        pieces.sort(reverse=True)
        out[l] = pieces
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    b = Board(Libs(args.libs))

    # ── GND: vias into the pour, no tracks ──────────────────────────────────
    gnd_via = 0
    for ref, num, net, px, py, w, h, lays in b.pads:
        if net != PLANE_NET:
            continue
        pad = b.fps[ref].pad(num)
        if pad is None or pad.ptype != "smd":
            continue        # through-hole already reaches both sides
        b.add_via(px, py, net)
        gnd_via += 1

    # ── everything else ─────────────────────────────────────────────────────
    # ══ ORDER MATTERS, AND ALPHABETICAL IS NOT AN ORDER. ══
    #
    # Whoever routes first gets the corridor. Sorting by name put `+24V` first
    # -- a 2 mm-wide net, the widest on the board -- so it took the channel down
    # the right-hand side to J2 before BUS_B, five letters later, ever asked for
    # it. BUS_B then had nowhere to go on F.Cu and spent 18 mm on the back,
    # under its own partner's return path.
    #
    # A person laying this out by hand routes the bus pair first and threads
    # power around it, because the pair is what the board is for and power does
    # not care about its own shape. Do that: PINNED nets first, then the rest,
    # alphabetical within each group so a run stays reproducible.
    def order(n: str) -> tuple[int, str]:
        return (0 if n in PINNED else 1, n)

    ok = failed = 0
    for net in sorted(design.NETS, key=order):
        if net == PLANE_NET:
            continue
        net_width = WIDTH.get(net, DEFAULT_WIDTH)
        groups = []
        for ref, pin in design.NETS[net]:
            from verify_design import resolve
            for cand in design.PARTS[ref]["lib"]:
                try:
                    pins = b.libs.pins(cand)
                    break
                except LookupError:
                    continue
            for num in resolve(pins, pin):
                # one group per PHYSICAL pad -- see Board.pads_of. Two pads
                # sharing a number are two pieces of copper and both have to
                # be reached.
                for i in b.pads_of(ref, num):
                    cs = b.pad_cells_at(i)
                    if cs:
                        groups.append((f"{ref}.{num}", cs, b.pad_size_at(i),
                                       b.pad_rect_at(i)))
        if len(groups) < 2:
            continue
        connected = set(groups[0][1])
        names = [groups[0][0]]
        src_min = groups[0][2]
        remaining = groups[1:]
        while remaining:
            # ══ A TRACK CANNOT BE WIDER THAN THE PAD IT LANDS ON. ══
            #
            # +24V is a 2 mm net: it carries the 3 A pass-through to the AMS.
            # It also feeds the buck's VIN, which is a 1.32 x 0.60 mm SOT-23
            # pin. Routed at one width the search reported U3.3 as having
            # "32 pad cells, 0 unblocked" -- the track was wider than the
            # target and no centreline could legally touch it.
            #
            # Real boards neck down for exactly this, so the width is capped
            # per CONNECTION by the smallest pad at either end. The trunk
            # stays fat where the current is; the branch to an IC pin narrows
            # to fit, which is both correct and what a human would draw.
            tgt_min = min(g[2] for g in remaining)
            width = max(DEFAULT_WIDTH,
                        min(net_width, 0.9 * src_min, 0.9 * tgt_min))
            blocked = b.blocked_for(net, width)
            via_bad = b.via_blocked_for(net)
            goals: set = set()
            rects = []
            for _n, cs, _s, rect in remaining:
                goals |= cs
                if rect:
                    rects.append(rect)
            path = b.route_one(blocked, via_bad, connected, goals, rects,
                               via_cost=(VIA_COST_PINNED if net in PINNED
                                         else VIA_COST))
            if path is None:
                print(f"  ! {net}: no route from {'+'.join(names)} "
                      f"to {remaining[0][0]} (w={width:.2f})")
                failed += 1
                break
            end = path[-1]
            hit = next(i for i, (_n, cs, _s, _r) in enumerate(remaining)
                       if end in cs)
            b.commit(path, net, width)
            connected |= set(path) | remaining[hit][1]
            names.append(remaining[hit][0])
            src_min = min(src_min, remaining[hit][2])
            remaining.pop(hit)
            ok += 1

    # ── write the tracks into the board ─────────────────────────────────────
    pcb = HERE / f"{design.NAME}.kicad_pcb"
    text = pcb.read_text()
    text = re.sub(r"\n  \(segment [\s\S]*?\)\n", "\n", text)
    text = re.sub(r"\n  \(via [\s\S]*?\)\n", "\n", text)
    uid = [0x900]

    def nid() -> str:
        uid[0] += 1
        return "00000000-0000-0000-0000-%012x" % uid[0]

    out = []
    for x1, y1, x2, y2, w, net, layer in b.tracks:
        out.append('  (segment (start %.3f %.3f) (end %.3f %.3f) (width %g) '
                   '(layer "%s") (net %d) (tstamp %s))'
                   % (x1, y1, x2, y2, w, LAYER_NAME[layer], b.nets[net], nid()))
    for x, y, net in b.vias:
        out.append('  (via (at %.3f %.3f) (size %g) (drill %g) '
                   '(layers "F.Cu" "B.Cu") (net %d) (tstamp %s))'
                   % (x, y, VIA_DIAM, VIA_DRILL, b.nets[net], nid()))
    idx = text.rindex("\n)")
    pcb.write_text(text[:idx] + "\n\n" + "\n".join(out) + text[idx:])

    total = sum(math.hypot(t[2] - t[0], t[3] - t[1]) for t in b.tracks)
    front = sum(1 for t in b.tracks if t[6] == F_CU)
    print(f"routed {ok} connections, {failed} unrouted")
    print(f"{len(b.tracks)} segments ({front} F.Cu, {len(b.tracks)-front} "
          f"B.Cu), {total:.0f} mm of track")
    print(f"{len(b.vias)} vias ({gnd_via} GND to the pour, "
          f"{len(b.vias)-gnd_via} layer changes)")

    # What the priced nets actually did. A via on the bus pair is a compromise
    # and compromises are reported, not left for someone to find in the gerbers.
    for net in sorted(PINNED):
        ts = [t for t in b.tracks if t[5] == net]
        if not ts:
            continue
        back = sum(math.hypot(t[2] - t[0], t[3] - t[1])
                   for t in ts if t[6] == B_CU)
        whole = sum(math.hypot(t[2] - t[0], t[3] - t[1]) for t in ts)
        print(f"pinned {net}: {whole:.0f} mm total, {back:.0f} mm on B.Cu"
              + ("  <- check the return path here" if back > 0 else ""))

    isl = pour_islands(b)
    for l in (F_CU, B_CU):
        ps = isl[l]
        big = ps[0] if ps else 0.0
        strays = [a for a in ps if a >= 1.0][1:]
        print(f"{LAYER_NAME[l]} pour: {len(ps)} piece(s), largest "
              f"{big:.0f} mm^2" +
              (f", {len(strays)} more over 1 mm^2: "
               f"{', '.join('%.0f' % a for a in strays)}" if strays else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
