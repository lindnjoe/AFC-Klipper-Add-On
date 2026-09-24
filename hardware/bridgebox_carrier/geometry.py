#!/usr/bin/env python3
"""
Pad geometry for the placed board.

Everything downstream -- routing, the clearance check -- needs to know where a
pad physically is, which is the footprint's local pad position put through the
placement rotation. That transform lives here once so the router and the
checker cannot disagree about it.

Board-local coordinates throughout: (0, 0) is the top-left corner of the
outline, +x right, +y down, millimetres. Add (X0, Y0) for sheet coordinates.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_pcb as p  # noqa: E402

Pad = tuple[float, float, float, float, str]   # x, y, w, h, shape


def rot(lx: float, ly: float, a: int) -> tuple[float, float]:
    """Footprint-local to board-local, through a placement rotation."""
    r = math.radians(a)
    ca, sa = math.cos(r), math.sin(r)
    return lx * ca + ly * sa, -lx * sa + ly * ca


def pads_of(ref: str) -> dict[str, Pad]:
    """Every numbered pad of one placed part, in board-local coordinates."""
    fp = {r: f for r, _l, _v, f, *_ in p.g.PARTS}[ref]
    px, py, a = p.PLACE[ref]
    text = p.load_fp(fp)
    out: dict[str, Pad] = {}
    for m in re.finditer(r"\(pad ", text):
        blk = p.sexp(text, m.start())
        # Same unquoted-token trap as the reference designators: older
        # stock footprints write (pad 1 smd ...) with no quotes.
        num = re.match(r'\(pad (?:"([^"]*)"|(\S+))', blk)
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", blk)
        size = re.search(r"\(size ([\d.]+) ([\d.]+)\)", blk)
        if not (num and at and size):
            continue
        name = num.group(1) if num.group(1) is not None else num.group(2)
        shape = re.match(r'\(pad (?:"[^"]*"|\S+) \w+ (\w+)', blk).group(1)
        lx, ly = float(at.group(1)), float(at.group(2))
        w, h = float(size.group(1)), float(size.group(2))
        pad_rot = float(at.group(3) or 0)
        dx, dy = rot(lx, ly, a)
        # A pad rotated 90 within its footprint, or a footprint rotated 90,
        # swaps the pad's own width and height in board space.
        if (pad_rot + a) % 180 >= 45 and (pad_rot + a) % 180 < 135:
            w, h = h, w
        if not name or name == '""':
            continue          # unnamed pads are mechanical (pegs, holes)
        out[name] = (px + dx, py + dy, w, h, shape)
    return out


def dup_pads(ref: str) -> dict[str, list[Pad]]:
    """Pads sharing a number, which some footprints genuinely have.

    A 5x20 fuse clip carries TWO pads numbered 1 and two numbered 2 -- each
    clip has two solder points. Keeping only one per number leaves the other
    an isolated island of its own net, which KiCad calls unconnected and a
    picture calls finished.
    """
    fp = {r: f for r, _l, _v, f, *_ in p.g.PARTS}[ref]
    px, py, a = p.PLACE[ref]
    text = p.load_fp(fp)
    out: dict[str, list[Pad]] = {}
    for m in re.finditer(r"\(pad ", text):
        blk = p.sexp(text, m.start())
        num = re.match(r'\(pad (?:"([^"]*)"|(\S+))', blk)
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", blk)
        size = re.search(r"\(size ([\d.]+) ([\d.]+)\)", blk)
        if not (num and at and size):
            continue
        name = num.group(1) if num.group(1) is not None else num.group(2)
        if not name or name == '""':
            continue
        dx, dy = rot(float(at.group(1)), float(at.group(2)), a)
        out.setdefault(name, []).append(
            (px + dx, py + dy, float(size.group(1)), float(size.group(2)), ""))
    return {k: v for k, v in out.items() if len(v) > 1}


def all_pads() -> dict[str, dict[str, Pad]]:
    return {ref: pads_of(ref) for ref, *_ in p.g.PARTS}


def net_pads() -> dict[str, list[tuple[str, str, Pad]]]:
    """net name -> [(ref, pad number, pad), ...]"""
    ap = all_pads()
    out: dict[str, list[tuple[str, str, Pad]]] = {}
    for net, pins in p.g.NETS.items():
        out[net] = [(r, n, ap[r][n]) for r, n in pins if n in ap[r]]
    return out


if __name__ == "__main__":
    for net, items in sorted(net_pads().items()):
        print(f"{net}:")
        for ref, num, (x, y, w, h, shape) in items:
            print(f"   {ref}.{num:<3s} ({x:6.2f}, {y:6.2f})  "
                  f"{w:.2f}x{h:.2f} {shape}")
