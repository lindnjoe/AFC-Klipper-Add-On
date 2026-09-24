#!/usr/bin/env python3
"""
Read KiCad footprints: pads, courtyard, and the text of the whole module.

Shared by the placer, the placement checker and the router, so all three agree
on where a pad actually is. Two shapes of the same file bite here and both are
handled:

  * pad numbers are quoted OR bare -- `(pad "1" thru_hole ...)` in the Molex
    connector, `(pad 1 smd ...)` in SOIC-8. A regex for one silently finds no
    pads in the other, and a footprint with no pads places cleanly and routes
    to nothing.
  * a pad's `(at x y)` may carry a third value, its own rotation, which is
    absent on most and present on the ones that matter (USB-C shells, angled
    connector pins).

Mechanical pads -- `np_thru_hole`, and pads whose number is "" -- are returned
too but flagged, because they take space on the board and belong in a
collision check while belonging to no net.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

PAD_RE = re.compile(
    r'\(pad\s+(?:"([^"]*)"|(\S+))\s+(\S+)\s+(\S+)\s+'
    r"\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)\s+"
    r"\(size\s+([-\d.]+)\s+([-\d.]+)\)")
DRILL_RE = re.compile(r"\(drill\s+([\d.]+)")
LAYERS_RE = re.compile(r'\(layers\s+([^)]*)\)')


@dataclass
class Pad:
    number: str
    ptype: str          # smd | thru_hole | np_thru_hole
    shape: str
    x: float            # footprint-local mm, +y DOWN (KiCad board convention)
    y: float
    rot: float
    w: float
    h: float
    drill: float
    layers: str

    @property
    def electrical(self) -> bool:
        return bool(self.number) and self.ptype != "np_thru_hole"

    @property
    def on_front(self) -> bool:
        return "F.Cu" in self.layers or "*.Cu" in self.layers

    def radius(self) -> float:
        """Half-diagonal -- a cheap conservative bound for collision tests."""
        return math.hypot(self.w, self.h) / 2.0


@dataclass
class Footprint:
    fpid: str
    text: str
    pads: list[Pad]

    def pad(self, number: str) -> Pad | None:
        for p in self.pads:
            if p.number == number:
                return p
        return None

    def fab_extent(self) -> tuple[float, float, float, float] | None:
        """(minx, miny, maxx, maxy) of the F.Fab body outline, or None.

        The BODY, not the courtyard. For a connector these are different
        questions and only this one answers "where is the shell?": a
        courtyard's asymmetry is as likely to be covering a second pad row as
        it is to be mating clearance, which is exactly the misreading that put
        J2 on this board facing its own middle.
        """
        xs: list[float] = []
        ys: list[float] = []
        for m in re.finditer(
                r"\(fp_(?:line|rect)\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s+"
                r"\(end\s+([-\d.]+)\s+([-\d.]+)\)[\s\S]{0,90}?"
                r'\(layer\s+"?F\.Fab"?\)', self.text):
            xs += [float(m.group(1)), float(m.group(3))]
            ys += [float(m.group(2)), float(m.group(4))]
        if not xs:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    def extent(self) -> tuple[float, float, float, float]:
        """(minx, miny, maxx, maxy) over every pad, courtyard if it has one."""
        xs: list[float] = []
        ys: list[float] = []
        for m in re.finditer(
                r"\(fp_(?:line|rect)\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s+"
                r"\(end\s+([-\d.]+)\s+([-\d.]+)\)[\s\S]{0,80}?"
                r'\(layer\s+"F\.CrtYd"\)', self.text):
            xs += [float(m.group(1)), float(m.group(3))]
            ys += [float(m.group(2)), float(m.group(4))]
        if not xs:
            for p in self.pads:
                xs += [p.x - p.w / 2, p.x + p.w / 2]
                ys += [p.y - p.h / 2, p.y + p.h / 2]
        return min(xs), min(ys), max(xs), max(ys)


def load(libs_fp_root: Path, fpid: str) -> Footprint:
    lib, name = fpid.split(":", 1)
    path = libs_fp_root / f"{lib}.pretty" / f"{name}.kicad_mod"
    if not path.exists():
        raise LookupError(f"no footprint {fpid}")
    text = path.read_text()
    pads: list[Pad] = []
    for m in PAD_RE.finditer(text):
        num = m.group(1) if m.group(1) is not None else m.group(2)
        tail = text[m.end():m.end() + 240]
        drill = DRILL_RE.search(tail)
        layers = LAYERS_RE.search(tail)
        pads.append(Pad(
            number=num.strip('"'),
            ptype=m.group(3), shape=m.group(4),
            x=float(m.group(5)), y=float(m.group(6)),
            rot=float(m.group(7)) if m.group(7) else 0.0,
            w=float(m.group(8)), h=float(m.group(9)),
            drill=float(drill.group(1)) if drill else 0.0,
            layers=layers.group(1) if layers else "",
        ))
    if not pads:
        raise LookupError(f"{fpid}: parsed no pads -- check the pad syntax")
    return Footprint(fpid=fpid, text=text, pads=pads)


def place(px: float, py: float, rot_deg: float,
          ox: float, oy: float) -> tuple[float, float]:
    """A footprint-local point, rotated and moved to board coordinates.

    KiCad rotates footprints ANTICLOCKWISE on screen, and screen +y is down, so
    the usual anticlockwise matrix with a flipped y works out as below. Getting
    this backwards mirrors every part about its own origin, which looks almost
    right and connects nothing.
    """
    r = math.radians(rot_deg)
    c, s = math.cos(r), math.sin(r)
    return ox + px * c + py * s, oy - px * s + py * c
