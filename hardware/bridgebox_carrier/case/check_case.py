#!/usr/bin/env python3
"""
Check the case against the board it is supposed to fit.

case.scad restates the board's outline, its mounting holes and where its three
connectors sit. Restated numbers rot: someone nudges a part in gen_pcb.py, the
gerbers change, and the case quietly becomes a case for the previous board. So
nothing here trusts case.scad -- it reads the constants back out of it and
checks each one against the placement the PCB is actually generated from.

What it checks:

  * board outline and the four mounting holes match gen_pcb.py exactly
  * every wall opening fully clears its connector's courtyard, with margin
  * every screw column and tray post clears every part's courtyard
  * the columns and posts stay inside the board
  * the BOOTSEL hole lands on the Pico
  * the engraved panel clears every opening in the lid's top face, and those
    openings clear the screw counterbores -- both hull()s, so both reach half
    a diameter past the coordinates their constants name

    python3 check_case.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import check_placement as cp  # noqa: E402
import gen_pcb as p  # noqa: E402

SCAD = HERE / "case.scad"

# How much wider than the connector body an opening has to be, per side, for a
# plug -- which is always bigger than the socket -- to get in.
MARGIN = 0.5

# The Pico's USB socket, which no footprint describes because it is not a part
# on this board. Stated once, here, because both this checker and the preview
# need it -- gen_preview_parts.py imports these rather than restating them.
#
# The overhang is past the PICO's own PCB end, not this board's edge.
USB_SHELL_W = 7.0        # across the wall, y
USB_SHELL_H = 3.0        # tall
USB_SHELL_OUT = 1.2      # overhang past the Pico's PCB end


def scad_numbers(text: str) -> dict[str, float]:
    """Top-level numeric constants, expressions included.

    Several of the openings are written the way they are meant to be read --
    USB_Y = BD - 13.0, the board coordinate converted in place -- so this has
    to evaluate, not just match a literal. Only unindented lines are scanned,
    which keeps module bodies out of it; anything that is not plain arithmetic
    over constants already seen (a string, a ternary) simply does not parse and
    is skipped.
    """
    out: dict[str, float] = {}
    for line in text.splitlines():
        if not line or line[0].isspace():
            continue
        for m in re.finditer(r"(\w+)\s*=\s*([^;=]+);", line):
            expr = m.group(2).strip()
            if not re.fullmatch(r"[\w\s.+\-*/()]+", expr):
                continue
            try:
                out[m.group(1)] = float(eval(expr, {"__builtins__": {}},
                                             dict(out)))
            except Exception:
                pass
    return out


def scad_holes(text: str) -> list[tuple[float, float]]:
    m = re.search(r"^HOLES\s*=\s*\[(.*?)\]\s*;", text, re.M | re.S)
    if not m:
        raise SystemExit("case.scad has no HOLES")
    return [(float(a), float(b))
            for a, b in re.findall(r"\[\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\]",
                                   m.group(1))]


def scad_vent_banks(text: str) -> list[tuple[float, float, list[float]]]:
    """VENT_BANKS as [(x0, x1, [board y, ...]), ...]."""
    m = re.search(r"VENT_BANKS\s*=\s*\[(.*?)\n\];", text, re.S)
    if not m:
        return []
    out = []
    for row in re.finditer(r"\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*\[([^\]]*)\]",
                           m.group(1)):
        ys = [float(v) for v in row.group(3).split(",") if v.strip()]
        out.append((float(row.group(1)), float(row.group(2)), ys))
    return out


def outline(text: str, layer: str) -> cp.Box | None:
    """Bounding box of one layer's fp_line graphics, in footprint space."""
    xs: list[float] = []
    ys: list[float] = []
    for m in re.finditer(r"\(fp_line \(start ([-\d.]+) ([-\d.]+)\) "
                         r"\(end ([-\d.]+) ([-\d.]+)\)", text):
        if layer not in p.sexp(text, m.start()):
            continue
        xs += [float(m.group(1)), float(m.group(3))]
        ys += [float(m.group(2)), float(m.group(4))]
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None


def boxes(layer: str | None = None) -> dict[str, cp.Box]:
    """Placed boxes for every part: courtyards, or one named layer.

    Courtyard is the right answer for "may another part sit here". It is the
    WRONG answer for "how wide does the hole in the wall have to be" -- the
    barrel jack's courtyard runs 2 mm past its body to take in the switch pin,
    and sizing an opening to that would be sizing it to a solder pad. F.Fab is
    the part's actual body.
    """
    out: dict[str, cp.Box] = {}
    for ref, _lib, _val, fp, *_ in p.g.PARTS:
        text = p.load_fp(fp)
        box = cp.courtyard(text) if layer is None else outline(text, layer)
        if box is None:
            continue
        px, py, rot = p.PLACE[ref]
        out[ref] = cp.placed(box, px, py, rot)
    return out


def top_face_openings(bd: float, holes, vents, n) -> list:
    """
    Every opening in the lid's top face, as boxes GROWN BY THEIR OWN RADIUS.

    That growth is the whole point. The BOOTSEL slot and slot() are both
    hull()s of round ends, so each reaches half a diameter past the numbers
    its constants name -- BOOTSEL spans y 33.4 on the current board, not the
    37.4 its centres suggest. Sizing anything off the centres is how the mark
    ended up through that slot, how the intake vent ended up in a screw
    counterbore, and how the 84 x 58 lid ended up with the mark through a vent.

    :param bd: board depth, for the board-y to lid-y flip
    :param holes: mounting holes, in board coordinates
    :param vents: VENT_BANKS, in board coordinates
    :param n: case.scad's numeric constants
    :return list: (name, box) pairs
    """
    out = []
    r = n["CBORE"] / 2
    for hx, hy in holes:
        cy = bd - hy
        out.append((f"counterbore ({hx:.0f},{cy:.0f})",
                    (hx - r, cy - r, hx + r, cy + r)))
    br = n["BOOT_D"] / 2
    by = bd - 13.0                      # BOOT_Y is stated as BD - 13
    out.append(("BOOTSEL slot",
                (n["BOOT_X"] - br, by - n["BOOT_OFF"] - br,
                 n["BOOT_X"] + br, by + n["BOOT_OFF"] + br)))
    for i, (x0, x1, ys) in enumerate(vents):
        for y in ys:
            cy, vr = bd - y, 2.6 / 2
            out.append((f"vent bank {i} y{y:g}",
                        (x0 - vr, cy - vr, x1 + vr, cy + vr)))
    return out


def build_overrides(path) -> dict:
    """
    The -D overrides build.sh renders the older 84 x 58 board with.

    That variant is a second lid with its own outline, holes and vents, and
    NOTHING else checks it -- which is exactly why the mark went through one of
    its vents once the panel grew from 43 to 45 mm.

    :param path: build.sh
    :return dict: the overridden names, parsed
    """
    text = path.read_text()
    out: dict = {}
    for m in re.finditer(r"-D '(\w+)=(.+?)'", text):
        name, val = m.group(1), m.group(2).strip()
        try:
            out[name] = eval(val.replace("[", "[").replace("]", "]"),
                             {"__builtins__": {}}, {})
        except Exception:
            pass
    return out


def main() -> int:
    text = SCAD.read_text()
    n = scad_numbers(text)
    bad: list[str] = []

    def check(ok: bool, msg: str) -> None:
        print(("  ok    " if ok else "  FAIL  ") + msg)
        if not ok:
            bad.append(msg)

    print("board")
    check(n["BW"] == p.W and n["BD"] == p.H,
          f"outline {n['BW']:.0f} x {n['BD']:.0f} == gen_pcb "
          f"{p.W:.0f} x {p.H:.0f}")
    check(sorted(scad_holes(text)) == sorted(p.HOLES),
          f"mounting holes {sorted(scad_holes(text))}")

    b = boxes()
    fab = boxes("F.Fab")

    print("\nwall openings")
    # Each opening is stated as a centre and a width in the wall's own axis.
    # The connector's courtyard is the thing that has to fit inside it.
    for name, ref, cen, wide, axis in (
            ("USB", "U1", p.H - n["USB_Y"], n["USB_W"], "y"),
            ("J1 barrel", "J1", p.H - n["J1_Y"], n["J1_W"], "y"),
            ("J2 Micro-Fit", "J2", n["J2_X"], n["J2_W"], "x"),
    ):
        lo, hi = cen - wide / 2, cen + wide / 2
        if ref == "U1":
            # Not the whole Pico -- only its USB shell reaches the wall. The
            # footprint draws it as the silkscreen tab hanging off the board
            # end, 7 mm across, centred on the module.
            clo, chi = p.PLACE["U1"][1] - 3.5, p.PLACE["U1"][1] + 3.5
        elif axis == "y":
            clo, chi = fab[ref][1], fab[ref][3]
        else:
            clo, chi = fab[ref][0], fab[ref][2]
        ok = lo <= clo - MARGIN and hi >= chi + MARGIN
        check(ok, f"{name}: opening {lo:.2f}..{hi:.2f} clears "
                  f"{ref} {clo:.2f}..{chi:.2f}")

    # HEIGHT, which the plan checks above cannot see. Every other opening sits
    # on a connector that is soldered to the board, so a notch starting at the
    # split line is aligned by construction. The Pico's USB socket is the one
    # exception: the Pico rides on headers, so its socket is well up the wall
    # and the opening is a closed WINDOW centred on it rather than a slot run
    # down to the split. USB_CZ is measured on the assembly from the board's
    # underside; the window is placed relative to the split line, the board's
    # TOP face.
    print("\nUSB opening height")
    centre = n["USB_CZ"] - n["BT"]
    lo, hi = centre - n["USB_H"] / 2, centre + n["USB_H"] / 2
    check(lo > 0.5,
          f"window {lo:.2f}..{hi:.2f} sits clear above the split line "
          f"(solid wall below it, no dead space)")
    check(n["USB_H"] >= USB_SHELL_H + 2 * MARGIN,
          f"window {n['USB_H']:.2f} mm tall for a {USB_SHELL_H:.1f} mm "
          f"socket shell")
    check(hi <= n["IH"],
          f"window top {hi:.2f} mm fits under the {n['IH']:.2f} mm lid")

    # And the reason a closed window is allowed at all: the socket overhangs
    # the PICO's PCB, not this one, so the lid's wall sweeps past empty space
    # on its way down. Move the Pico towards that edge and the lid stops
    # going on.
    setback = fab["U1"][0] - USB_SHELL_OUT
    check(setback > 0.5,
          f"socket stops {setback:.2f} mm short of the board edge, so the "
          f"lid still drops straight on")

    print("\nposts and columns")
    r = max(n["COL"], n["POST"]) / 2
    for hx, hy in p.HOLES:
        box = (hx - r, hy - r, hx + r, hy + r)
        hit = [ref for ref, bx in b.items() if cp.overlap(box, bx)]
        check(not hit, f"({hx:.0f},{hy:.0f}) d{2 * r:.0f} clear of parts"
                       + (f" -- hits {', '.join(sorted(hit))}" if hit else ""))
        inside = (0 <= box[0] and 0 <= box[1]
                  and box[2] <= p.W and box[3] <= p.H)
        check(inside, f"({hx:.0f},{hy:.0f}) d{2 * r:.0f} inside the board")

    print("\nbootsel")
    bx = (n["BOOT_X"] - n["BOOT_D"] / 2, (p.H - n["BOOT_Y"]) - n["BOOT_D"] / 2,
          n["BOOT_X"] + n["BOOT_D"] / 2, (p.H - n["BOOT_Y"]) + n["BOOT_D"] / 2)
    u1 = b["U1"]
    check(u1[0] <= bx[0] and u1[1] <= bx[1] and bx[2] <= u1[2]
          and bx[3] <= u1[3],
          f"hole {tuple(round(v, 2) for v in bx)} lands on the Pico")

    # ── the engraved panel ───────────────────────────────────────────────────
    # The mark is a recess in the lid's top face, so it has to sit in skin that
    # is actually there: any opening it touches turns a line of the logo into a
    # hole, and the first sign of it is a print.
    print("\nengraved panel")
    ln = scad_numbers((SCAD.parent / "logo.scad").read_text())
    # LOGO_BOLD is an offset() applied AFTER the resize, so it grows the mark by
    # that much on every side -- the keep-out is LOGO_W plus twice it, not
    # LOGO_W. Checking the nominal width would let a boldened stroke reach into
    # a vent while the checker reported clearance, which is the same class of
    # mistake as sizing a hull() off its centres.
    bold = n.get("LOGO_BOLD", 0.0)
    lw = n["LOGO_W"]
    lh = lw * ln["LOGO_HF"]
    m = re.search(r"LOGO_AT\s*=\s*\[([-\d.]+)\s*,\s*([-\d.]+)\]", text)
    lx, ly = float(m.group(1)), float(m.group(2))
    panel = (lx - lw / 2, ly - lh / 2, lx + lw / 2, ly + lh / 2)

    # The mark is sized PER BOARD: the 84 x 58 lid is bigger and its panel
    # solves to 65 mm where the current board's only reaches 48, so it carries
    # its own LOGO_W/LOGO_AT overrides. Check each board with its own.
    ov = build_overrides(SCAD.parent / "build.sh")
    boards = [("current", n["BD"], p.HOLES, scad_vent_banks(text), lw, lx, ly)]
    if {"BD", "HOLES", "VENT_BANKS"} <= set(ov):
        boards.append(("84 x 58", float(ov["BD"]), ov["HOLES"],
                       ov["VENT_BANKS"],
                       float(ov.get("LOGO_W", lw)),
                       *(ov["LOGO_AT"] if "LOGO_AT" in ov else (lx, ly))))

    for label, bd, holes, vents, lw, lx, ly in boards:
        lh = lw * ln["LOGO_HF"]
        # LOGO_BOLD is an offset() applied AFTER the resize, so the mark on the
        # part is 2 x it wider and taller than LOGO_W says. The keep-out has to
        # be the boldened size: checking the nominal one would report clearance
        # while a thickened stroke reached into a vent -- the same class of
        # mistake as sizing a hull() off its centre points.
        pw, ph = lw + 2 * bold, lh + 2 * bold
        panel = (lx - pw / 2, ly - ph / 2, lx + pw / 2, ly + ph / 2)
        openings = top_face_openings(bd, holes, vents, n)
        for name, box in openings:
            check(not cp.overlap(panel, box),
                  f"[{label}] mark {pw:.1f} x {ph:.1f} (bold {bold:g}) at ({lx:g},{ly:g}) "
                  f"clear of {name}")
        # The same hull trap bit the vents, so check those against the bosses.
        r = n["CBORE"] / 2
        for name, box in openings:
            if not name.startswith("vent"):
                continue
            for hx, hy in holes:
                cy = bd - hy
                check(not cp.overlap(box, (hx - r, cy - r, hx + r, cy + r)),
                      f"[{label}] {name} clear of counterbore "
                      f"({hx:.0f},{cy:.0f})")

    print("\nRESULT:", "case matches the board" if not bad
          else f"{len(bad)} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
