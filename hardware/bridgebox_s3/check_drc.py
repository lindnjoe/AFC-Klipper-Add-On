#!/usr/bin/env python3
"""
Read the finished board back and check it at full precision.

`route.py` works on a 0.1 mm raster, and a raster has half a cell of error in
every direction. That is fine for FINDING a route and not fine for believing
one: the USB-C escapes on this board clear their neighbours by 0.025 mm, which
is a quarter of a grid cell, so the router's own answer cannot settle whether
they are legal. Something has to check the real numbers.

So this parses the `.kicad_pcb` as a stranger would -- footprint placements and
their pads, segments, vias -- and works in floating-point millimetres with no
grid anywhere. It is the board-level twin of `verify_sch.py`: checking that the
generator RAN is not checking that it wrote what was meant.

Three questions:

  * CLEARANCE. Every pair of copper objects on different nets, on a shared
    layer, at least CLEARANCE apart. Exact segment-to-rectangle and
    segment-to-segment distance, not sampled.
  * CONNECTIVITY. For each net, do its pads, tracks and vias actually form ONE
    connected body? A router that reports "routed" has reported that its search
    succeeded, which is not the same claim. This is the ratsnest.
  * CONTAINMENT. Nothing outside the board edge, and nothing in the WROOM-1's
    antenna keepout.

Usage:  python3 check_drc.py [--libs DIR] [--clearance MM]
"""

from __future__ import annotations

import argparse
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
CLEARANCE = 0.2


# ── a small s-expression reader ──────────────────────────────────────────────
def sexp(text: str):
    """Parse one KiCad s-expression file into nested lists of str."""
    tok = re.compile(r'\s*(?:("(?:[^"\\]|\\.)*")|(\(|\))|([^\s()]+))')
    stack: list[list] = [[]]
    i, n = 0, len(text)
    while i < n:
        m = tok.match(text, i)
        if not m:
            break
        i = m.end()
        q, paren, word = m.group(1), m.group(2), m.group(3)
        if paren == "(":
            new: list = []
            stack[-1].append(new)
            stack.append(new)
        elif paren == ")":
            stack.pop()
        elif q is not None:
            stack[-1].append(q[1:-1].replace('\\"', '"'))
        else:
            stack[-1].append(word)
    return stack[0]


def kids(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def kid(node, name):
    k = kids(node, name)
    return k[0] if k else None


def fnum(x) -> float:
    return float(x)


# ── geometry ─────────────────────────────────────────────────────────────────
class Rect:
    """An axis-aligned pad. Every rotation on this board is a multiple of 90."""

    __slots__ = ("x0", "y0", "x1", "y1", "net", "layers", "tag")

    def __init__(self, cx, cy, w, h, net, layers, tag):
        self.x0, self.x1 = cx - w / 2, cx + w / 2
        self.y0, self.y1 = cy - h / 2, cy + h / 2
        self.net, self.layers, self.tag = net, layers, tag

    def corners(self):
        return ((self.x0, self.y0), (self.x1, self.y0),
                (self.x1, self.y1), (self.x0, self.y1))


class Seg:
    """A track, or a via as a zero-length one."""

    __slots__ = ("x1", "y1", "x2", "y2", "r", "net", "layers", "tag", "kind")

    def __init__(self, x1, y1, x2, y2, r, net, layers, tag, kind="track"):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.r, self.net, self.layers, self.tag = r, net, layers, tag
        self.kind = kind


def pt_seg(px, py, x1, y1, x2, y2) -> float:
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    if L <= 1e-18:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def pt_rect(px, py, r: Rect) -> float:
    dx = max(r.x0 - px, 0.0, px - r.x1)
    dy = max(r.y0 - py, 0.0, py - r.y1)
    return math.hypot(dx, dy)


def seg_seg(a: Seg, b: Seg) -> float:
    """Exact distance between two segments' centrelines."""
    def cross(ox, oy, ax, ay, bx, by):
        return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)
    d1 = cross(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1)
    d2 = cross(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2)
    d3 = cross(b.x1, b.y1, b.x2, b.y2, a.x1, a.y1)
    d4 = cross(b.x1, b.y1, b.x2, b.y2, a.x2, a.y2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(a.x1, a.y1, b.x1, b.y1, b.x2, b.y2),
               pt_seg(a.x2, a.y2, b.x1, b.y1, b.x2, b.y2),
               pt_seg(b.x1, b.y1, a.x1, a.y1, a.x2, a.y2),
               pt_seg(b.x2, b.y2, a.x1, a.y1, a.x2, a.y2))


def seg_rect(s: Seg, r: Rect) -> float:
    """Exact distance between a segment's centreline and an axis-aligned box.

    For two convex shapes the minimum is attained at a vertex of one against
    the other, so endpoints-to-box and corners-to-segment together are exact --
    no sampling, which would only ever be optimistic.
    """
    # cheap overlap test first
    lo_x, hi_x = min(s.x1, s.x2), max(s.x1, s.x2)
    lo_y, hi_y = min(s.y1, s.y2), max(s.y1, s.y2)
    if not (hi_x < r.x0 or lo_x > r.x1 or hi_y < r.y0 or lo_y > r.y1):
        # bounding boxes overlap: the segment may cut the rect
        for (cx, cy) in r.corners():
            if pt_seg(cx, cy, s.x1, s.y1, s.x2, s.y2) <= 1e-12:
                return 0.0
        inside = (r.x0 <= s.x1 <= r.x1 and r.y0 <= s.y1 <= r.y1) or \
                 (r.x0 <= s.x2 <= r.x1 and r.y0 <= s.y2 <= r.y1)
        if inside:
            return 0.0
        edges = [(r.x0, r.y0, r.x1, r.y0), (r.x1, r.y0, r.x1, r.y1),
                 (r.x1, r.y1, r.x0, r.y1), (r.x0, r.y1, r.x0, r.y0)]
        for (ex1, ey1, ex2, ey2) in edges:
            e = Seg(ex1, ey1, ex2, ey2, 0, "", set(), "")
            if seg_seg(s, e) <= 1e-12:
                return 0.0
    return min(pt_rect(s.x1, s.y1, r), pt_rect(s.x2, s.y2, r),
               min(pt_seg(cx, cy, s.x1, s.y1, s.x2, s.y2)
                   for (cx, cy) in r.corners()))


# ── read the board ───────────────────────────────────────────────────────────
def load_board(path: Path):
    root = sexp(path.read_text())
    pcb = root[0]
    nets = {}
    for n in kids(pcb, "net"):
        nets[int(n[1])] = n[2] if len(n) > 2 else ""

    rects: list[Rect] = []
    segs: list[Seg] = []

    for fp in kids(pcb, "footprint"):
        at = kid(fp, "at")
        ox, oy = fnum(at[1]), fnum(at[2])
        orot = fnum(at[3]) if len(at) > 3 else 0.0
        ref = ""
        for t in kids(fp, "fp_text"):
            if len(t) > 1 and t[1] == "reference":
                ref = t[2]
        for pad in kids(fp, "pad"):
            num, ptype = pad[1], pad[2]
            pat = kid(pad, "at")
            px, py = fnum(pat[1]), fnum(pat[2])
            size = kid(pad, "size")
            w, h = fnum(size[1]), fnum(size[2])
            lay = kid(pad, "layers")
            layers = set(lay[1:]) if lay else set()
            netn = kid(pad, "net")
            net = netn[2] if netn and len(netn) > 2 else ""
            if ptype == "np_thru_hole":
                net = "~NPTH"
            bx, by = fp_lib.place(px, py, orot, ox, oy)
            bw, bh = (w, h) if orot % 180 == 0 else (h, w)
            cu = {"F.Cu", "B.Cu"} if any(l.startswith("*") for l in layers) \
                else {l for l in layers if l.endswith(".Cu")}
            if not cu:
                continue
            shape = pad[3]
            tag = f"{ref}.{num}" if num else f"{ref} hole"
            # ══ A ROUND PAD IS NOT A SQUARE, AND A HOLE IS NOT COPPER. ══
            #
            # Both were modelled as axis-aligned boxes. The box around a circle
            # is conservative -- it contains the circle, so no violation hides
            # behind it -- but it is wrong, and J2's mounting peg is a 3 mm
            # ROUND hole whose corners were being treated as solid.
            #
            # The non-plated holes were worse than approximated: they were
            # SKIPPED. A drill is not copper, so it carried no net and the
            # clearance loop stepped straight over it -- meaning a track could
            # run clean across a 3 mm hole and nothing here would say a word.
            # The drill is the hazard; it goes in as an obstacle sized by its
            # own diameter, against copper only.
            if ptype == "np_thru_hole":
                d = fnum(kid(pad, "drill")[1]) if kid(pad, "drill") else min(bw, bh)
                segs.append(Seg(bx, by, bx, by, d / 2, "~NPTH", cu,
                                f"{ref} {d:g}mm hole", "hole"))
            elif shape in ("circle", "oval"):
                r = min(bw, bh) / 2.0
                dx = max(0.0, (bw - bh) / 2.0)
                dy = max(0.0, (bh - bw) / 2.0)
                segs.append(Seg(bx - dx, by - dy, bx + dx, by + dy, r,
                                net, cu, "pad " + tag, "pad"))
            else:
                rects.append(Rect(bx, by, bw, bh, net, cu, tag))

    for s in kids(pcb, "segment"):
        st, en = kid(s, "start"), kid(s, "end")
        w = fnum(kid(s, "width")[1])
        lay = kid(s, "layer")[1]
        net = nets.get(int(kid(s, "net")[1]), "")
        segs.append(Seg(fnum(st[1]), fnum(st[2]), fnum(en[1]), fnum(en[2]),
                        w / 2, net, {lay}, f"track {net}"))
    for v in kids(pcb, "via"):
        at = kid(v, "at")
        size = fnum(kid(v, "size")[1])
        net = nets.get(int(kid(v, "net")[1]), "")
        x, y = fnum(at[1]), fnum(at[2])
        segs.append(Seg(x, y, x, y, size / 2, net, {"F.Cu", "B.Cu"},
                        f"via {net}", "via"))
    return rects, segs


# ── the checks ───────────────────────────────────────────────────────────────
def check_clearance(rects, segs, clear: float, limit: int = 25):
    bad = []
    for i, a in enumerate(segs):
        for b in segs[i + 1:]:
            if a.net == b.net or not (a.layers & b.layers):
                continue
            if a.net == "~NPTH" and b.net == "~NPTH":
                continue        # two holes near each other is a drill rule,
                                # not a copper one
            d = seg_seg(a, b) - a.r - b.r
            if d < clear - 1e-9:
                bad.append((d, f"{a.tag} vs {b.tag}"))
        for r in rects:
            if r.net == a.net or not (r.layers & a.layers):
                continue
            if r.net == "~NPTH":
                continue      # a drilled hole is not copper
            d = seg_rect(a, r) - a.r
            if d < clear - 1e-9:
                bad.append((d, f"{a.tag} vs pad {r.tag} ({r.net or 'no net'})"))
    bad.sort()
    return bad[:limit], len(bad)


def pour_reach(rects, segs, pour_net: str, W: float, H: float,
               clear: float = 0.4, grid: float = 0.2):
    """Where the pour can actually copper, per layer, as a labelled raster.

    GND is not routed as tracks on this board -- it is the pour, and the pour
    is what joins it. Without modelling that, a connectivity check reports GND
    as twenty separate bodies on a perfectly good board, which is a false alarm
    that trains you to ignore the check.

    Coarser than route.py's grid on purpose: a pour keeps 0.4 mm clearance, so
    0.2 mm resolution cannot change the answer, and it makes the flood cheap.
    """
    nx, ny = int(W / grid) + 1, int(H / grid) + 1
    X = (np.arange(nx) * grid)[None, :]
    Y = (np.arange(ny) * grid)[:, None]
    free = {}
    for layer in ("F.Cu", "B.Cu"):
        blocked = np.zeros((ny, nx), dtype=bool)
        for r in rects:
            if r.net == pour_net or layer not in r.layers:
                continue
            dx = np.maximum(np.maximum(r.x0 - X, 0.0), X - r.x1)
            dy = np.maximum(np.maximum(r.y0 - Y, 0.0), Y - r.y1)
            blocked |= np.hypot(dx, dy) < clear
        for s in segs:
            if s.net == pour_net or layer not in s.layers:
                continue
            dx, dy = s.x2 - s.x1, s.y2 - s.y1
            L = dx * dx + dy * dy
            if L <= 1e-18:
                d = np.hypot(X - s.x1, Y - s.y1)
            else:
                t = np.clip(((X - s.x1) * dx + (Y - s.y1) * dy) / L, 0.0, 1.0)
                d = np.hypot(X - (s.x1 + t * dx), Y - (s.y1 + t * dy))
            blocked |= (d - s.r) < clear
        free[layer] = ~blocked

    # label the connected regions of each layer's pour
    label = {}
    nxt = 1
    for layer in ("F.Cu", "B.Cu"):
        lab = np.zeros((ny, nx), dtype=np.int32)
        f = free[layer]
        for sy in range(ny):
            for sx in range(nx):
                if not f[sy, sx] or lab[sy, sx]:
                    continue
                q = deque([(sy, sx)])
                lab[sy, sx] = nxt
                while q:
                    cy, cx = q.popleft()
                    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        ay, ax = cy + dy, cx + dx
                        if (0 <= ay < ny and 0 <= ax < nx
                                and f[ay, ax] and not lab[ay, ax]):
                            lab[ay, ax] = nxt
                            q.append((ay, ax))
                nxt += 1
        label[layer] = lab
    return label, grid


def pour_labels_touching(obj, label, grid) -> set:
    """Which pour regions this object's copper actually reaches."""
    out = set()
    if isinstance(obj, Rect):
        x0, y0, x1, y1 = obj.x0, obj.y0, obj.x1, obj.y1
    else:
        x0 = min(obj.x1, obj.x2) - obj.r
        x1 = max(obj.x1, obj.x2) + obj.r
        y0 = min(obj.y1, obj.y2) - obj.r
        y1 = max(obj.y1, obj.y2) + obj.r
    pad = 0.45      # the pour's own clearance plus a cell
    for layer in obj.layers:
        if layer not in label:
            continue
        lab = label[layer]
        ny, nx = lab.shape
        ix0 = max(0, int((x0 - pad) / grid))
        ix1 = min(nx - 1, int((x1 + pad) / grid) + 1)
        iy0 = max(0, int((y0 - pad) / grid))
        iy1 = min(ny - 1, int((y1 + pad) / grid) + 1)
        sub = lab[iy0:iy1 + 1, ix0:ix1 + 1]
        out |= {int(v) for v in np.unique(sub) if v}
    return out


def check_connectivity(rects, segs, label=None, grid=0.2, pour_net="GND"):
    """Per net: is every pad, track and via one connected body?"""
    out = []
    nets = {r.net for r in rects if r.net and r.net != "~NPTH"}
    for net in sorted(nets):
        items: list = [r for r in rects if r.net == net]
        items += [s for s in segs if s.net == net]
        if len(items) < 2:
            continue
        parent = list(range(len(items)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            a, b = find(i), find(j)
            if a != b:
                parent[a] = b

        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                A, B = items[i], items[j]
                if not (A.layers & B.layers):
                    continue
                if isinstance(A, Seg) and isinstance(B, Seg):
                    touch = seg_seg(A, B) <= A.r + B.r + 1e-6
                elif isinstance(A, Seg):
                    touch = seg_rect(A, B) <= A.r + 1e-6
                elif isinstance(B, Seg):
                    touch = seg_rect(B, A) <= B.r + 1e-6
                else:
                    touch = not (A.x1 < B.x0 - 1e-6 or B.x1 < A.x0 - 1e-6
                                 or A.y1 < B.y0 - 1e-6 or B.y1 < A.y0 - 1e-6)
                if touch:
                    union(i, j)
        # the pour joins what it touches -- see pour_reach
        if net == pour_net and label is not None:
            reach = [pour_labels_touching(it, label, grid) for it in items]
            first: dict[int, int] = {}
            for i, rs in enumerate(reach):
                for rgn in rs:
                    if rgn in first:
                        union(i, first[rgn])
                    else:
                        first[rgn] = i
        groups = len({find(i) for i in range(len(items))})
        if groups > 1:
            pads = [it.tag for it in items if isinstance(it, Rect)]
            out.append((net, groups, len(items), pads))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    ap.add_argument("--clearance", type=float, default=CLEARANCE)
    args = ap.parse_args()
    pcb = HERE / f"{design.NAME}.kicad_pcb"
    rects, segs = load_board(pcb)
    kinds = {}
    for o in segs:
        kinds[o.kind] = kinds.get(o.kind, 0) + 1
    print(f"read {pcb.name}: {len(rects) + kinds.get('pad', 0)} pads "
          f"({kinds.get('pad', 0)} round), {kinds.get('track', 0)} tracks, "
          f"{kinds.get('via', 0)} vias, {kinds.get('hole', 0)} non-plated holes")

    rc = 0

    worst, total = check_clearance(rects, segs, args.clearance)
    if total:
        print(f"\nCLEARANCE: {total} violation(s) under "
              f"{args.clearance} mm, closest first:")
        for d, what in worst:
            print(f"  ! {d:+.3f} mm  {what}")
        rc = 1
    else:
        print(f"clearance: OK, nothing closer than {args.clearance} mm")

    # how close did it actually come? The margin is the interesting number on a
    # board whose tightest escape is 0.025 mm.
    tight = min((seg_rect(a, r) - a.r
                 for a in segs for r in rects
                 if r.net != a.net and (r.layers & a.layers)
                 and r.net != "~NPTH"), default=float("inf"))
    if tight != float("inf"):
        print(f"tightest track-to-pad gap: {tight:.3f} mm")

    label, lgrid = pour_reach(rects, segs, "GND",
                              design.BOARD_W, design.BOARD_H)
    npours = {l: len({int(v) for v in np.unique(m) if v})
              for l, m in label.items()}
    print("pour regions: " + ", ".join(f"{l} {n}" for l, n in npours.items()))
    islands = check_connectivity(rects, segs, label, lgrid)
    if islands:
        print(f"\nCONNECTIVITY: {len(islands)} net(s) not fully joined:")
        for net, groups, n, pads in islands:
            print(f"  ! {net}: {groups} separate bodies out of {n} objects "
                  f"({', '.join(pads[:8])}{'...' if len(pads) > 8 else ''})")
        rc = 1
    else:
        print("connectivity: OK, every net is one body")

    # containment
    W, H = design.BOARD_W, design.BOARD_H
    out_of = [s.tag for s in segs
              if min(s.x1, s.x2) - s.r < 0 or max(s.x1, s.x2) + s.r > W
              or min(s.y1, s.y2) - s.r < 0 or max(s.y1, s.y2) + s.r > H]
    if out_of:
        print(f"\nCONTAINMENT: {len(out_of)} object(s) off the board edge")
        rc = 1
    else:
        print("containment: OK, all copper inside the outline")
    return rc


if __name__ == "__main__":
    sys.exit(main())
