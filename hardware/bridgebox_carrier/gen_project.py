#!/usr/bin/env python3
"""
Generate the BridgeBox carrier KiCad project.

The schematic is authored here rather than by hand because the connectivity is
the part that must not drift from docs/CARRIER_BOARD.md: NETS below is the same
net list that document describes, and everything else is mechanical.

Symbols for the stock parts are lifted out of the installed KiCad libraries and
embedded in the .kicad_sch, which is what KiCad does itself on save -- it makes
the project open standalone even where those libraries are a different version.

Run from this directory:  python3 gen_project.py
"""

from __future__ import annotations

import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAME = "bridgebox_carrier"
SYMDIR = Path("/usr/share/kicad/symbols")

# ── the parts ────────────────────────────────────────────────────────────────
# ref, lib id, value, footprint, x, y  (schematic mm)
PARTS = [
    ("J1", "Connector:Barrel_Jack_Switch", "24V 5.5x2.1",
     "Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal", 40, 40),
    ("F1", "Device:Fuse", "3A slow",
     "Fuse:Fuseholder_Clip-5x20mm_Keystone_3512_Inline_P23.62x7.27mm_"
     "D1.02x1.57mm_Horizontal", 75, 35),
    ("U1", "bridgebox:Raspberry_Pi_Pico", "Pico",
     "bridgebox:RaspberryPi_Pico_THT", 55, 130),
    ("U2", "bridgebox:MAX3485_Module", "MAX3485 3V3",
     "bridgebox:MAX3485_Module_3L5R", 130, 130),
    ("J2", "Connector_Generic:Conn_02x02_Odd_Even", "Micro-Fit 43045-0400",
     "Connector_Molex:Molex_Micro-Fit_3.0_43045-0400_2x02_P3.00mm_Horizontal",
     255, 130),
]

# ── the net list ─────────────────────────────────────────────────────────────
# net name -> [(ref, pin number), ...]
#
# This mirrors the "Net list" table in docs/CARRIER_BOARD.md. Q1 is the
# reverse-polarity P-FET: source on the input side, drain on the load side,
# gate pulled to GND through R2 and clamped to the source by D2 -- without that
# zener the gate sees the full 24 V against a +/-20 V Vgs rating.
NETS = {
    # The 24 V rail is now a pass-through: jack, fuse, connector. The
    # reverse-polarity stage came out because the supply ships with the board,
    # which removes the failure it existed for -- see CARRIER_BOARD.md.
    "+24V_IN": [("J1", "1"), ("F1", "1")],
    "+24V":    [("F1", "2"), ("J2", "4")],
    # GP1 to TXD and GP0 to RXD, which is the CROSSED reading of those two
    # labels and the one the working board uses. These modules name their
    # logic pins from the module's own side: TXD is what the module
    # transmits (the receiver output), RXD is what it receives (the driver
    # input). Wired the other way the Pico drives the module's output pin
    # and the bus never speaks -- measured, not assumed.
    "TXD":     [("U1", "2"), ("U2", "4")],
    "RXD":     [("U1", "1"), ("U2", "3")],
    "EN":      [("U1", "4"), ("U2", "1")],
    "+3V3":    [("U1", "36"), ("U2", "2")],
    # A/B cross on the way to the connector, and the swap jumpers are gone.
    # They existed to defer this decision; it is settled now, so the crossover
    # is copper: the module's A reaches the connector's B and vice versa.
    "BUS_A":   [("U2", "8"), ("J2", "1")],
    "BUS_B":   [("U2", "7"), ("J2", "2")],
    "GND":     [("J1", "2"), ("J1", "3"), ("J2", "3"),
                ("U2", "5"), ("U2", "6"),
                ("U1", "3"), ("U1", "8"), ("U1", "13"), ("U1", "18"),
                ("U1", "23"), ("U1", "28"), ("U1", "33"), ("U1", "38")],
}

# ── custom symbols ───────────────────────────────────────────────────────────
PICO_PINS = [
    ("GP0", "1"), ("GP1", "2"), ("GND", "3"), ("GP2", "4"), ("GP3", "5"),
    ("GP4", "6"), ("GP5", "7"), ("GND", "8"), ("GP6", "9"), ("GP7", "10"),
    ("GP8", "11"), ("GP9", "12"), ("GND", "13"), ("GP10", "14"),
    ("GP11", "15"), ("GP12", "16"), ("GP13", "17"), ("GND", "18"),
    ("GP14", "19"), ("GP15", "20"),
    ("GP16", "21"), ("GP17", "22"), ("GND", "23"), ("GP18", "24"),
    ("GP19", "25"), ("GP20", "26"), ("GP21", "27"), ("GND", "28"),
    ("GP22", "29"), ("RUN", "30"), ("GP26", "31"), ("GP27", "32"),
    ("AGND", "33"), ("GP28", "34"), ("ADC_VREF", "35"), ("3V3_OUT", "36"),
    ("3V3_EN", "37"), ("GND", "38"), ("VSYS", "39"), ("VBUS", "40"),
]
MOD_PINS = [
    ("EN", "1", "right"), ("VCC", "2", "right"), ("RXD", "3", "right"),
    ("TXD", "4", "right"), ("GND", "5", "right"),
    ("GND", "6", "left"), ("A", "7", "left"), ("B", "8", "left"),
]


def esc(s: str) -> str:
    return s.replace('"', r"\"")


def pico_symbol() -> str:
    """40-pin module symbol, pins down both sides in package order."""
    half = 20
    h = (half + 1) * 2.54
    out = ['  (symbol "Raspberry_Pi_Pico" (pin_names (offset 1.016)) '
           "(in_bom yes) (on_board yes)",
           '    (property "Reference" "U" (at -12.7 %.2f 0) (effects (font '
           '(size 1.27 1.27)) (justify left)))' % (h / 2 + 2.54),
           '    (property "Value" "Raspberry_Pi_Pico" (at -12.7 %.2f 0) '
           "(effects (font (size 1.27 1.27)) (justify left)))" % (h / 2),
           '    (property "Footprint" "bridgebox:RaspberryPi_Pico_THT" '
           "(at 0 0 0) (effects (font (size 1.27 1.27)) hide))",
           '    (property "Datasheet" "" (at 0 0 0) (effects (font '
           "(size 1.27 1.27)) hide))",
           '    (symbol "Raspberry_Pi_Pico_0_1"',
           "      (rectangle (start -12.7 %.2f) (end 12.7 %.2f) "
           "(stroke (width 0.254) (type default)) (fill (type background)))"
           % (h / 2, -h / 2),
           "    )",
           '    (symbol "Raspberry_Pi_Pico_1_1"']
    for i, (name, num) in enumerate(PICO_PINS):
        if i < half:                      # left side, top to bottom
            y = h / 2 - 2.54 - i * 2.54
            x, rot = -17.78, 0
        else:                             # right side, bottom to top
            y = -h / 2 + 2.54 + (i - half) * 2.54
            x, rot = 17.78, 180
        etype = "power_in" if name in ("GND", "AGND") else "bidirectional"
        out.append(
            '      (pin %s line (at %.2f %.2f %d) (length 5.08)'
            ' (name "%s" (effects (font (size 1.27 1.27))))'
            ' (number "%s" (effects (font (size 1.27 1.27)))))'
            % (etype, x, y, rot, esc(name), num))
    out += ["    )", "  )"]
    return "\n".join(out)


def module_symbol() -> str:
    """The RS-485 breakout: logic pins one side, bus pins the other."""
    out = ['  (symbol "MAX3485_Module" (pin_names (offset 1.016)) '
           "(in_bom yes) (on_board yes)",
           '    (property "Reference" "U" (at -10.16 13.97 0) (effects (font '
           "(size 1.27 1.27)) (justify left)))",
           '    (property "Value" "MAX3485_Module" (at -10.16 11.43 0) '
           "(effects (font (size 1.27 1.27)) (justify left)))",
           '    (property "Footprint" "bridgebox:MAX3485_Module_3L5R" '
           "(at 0 0 0) (effects (font (size 1.27 1.27)) hide))",
           '    (property "Datasheet" "" (at 0 0 0) (effects (font '
           "(size 1.27 1.27)) hide))",
           '    (symbol "MAX3485_Module_0_1"',
           "      (rectangle (start -10.16 10.16) (end 10.16 -10.16) "
           "(stroke (width 0.254) (type default)) (fill (type background)))",
           "    )",
           '    (symbol "MAX3485_Module_1_1"']
    ln = 0
    rn = 0
    for name, num, side in MOD_PINS:
        if side == "right":               # logic side, drawn on the left
            y = 7.62 - rn * 2.54
            rn += 1
            x, rot = -15.24, 0
        else:                             # bus side, drawn on the right
            y = 5.08 - ln * 2.54
            ln += 1
            x, rot = 15.24, 180
        etype = "power_in" if name in ("GND", "VCC") else "bidirectional"
        out.append(
            '      (pin %s line (at %.2f %.2f %d) (length 5.08)'
            ' (name "%s" (effects (font (size 1.27 1.27))))'
            ' (number "%s" (effects (font (size 1.27 1.27)))))'
            % (etype, x, y, rot, esc(name), num))
    out += ["    )", "  )"]
    return "\n".join(out)


# ── pull stock symbols out of the installed libraries ────────────────────────
def sexp_block(text: str, start: int) -> str:
    """Return the balanced parenthesised block beginning at `start`."""
    depth, i, instr, esc_next = 0, start, False, False
    while i < len(text):
        c = text[i]
        if instr:
            if esc_next:
                esc_next = False
            elif c == "\\":
                esc_next = True
            elif c == '"':
                instr = False
        elif c == '"':
            instr = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    raise ValueError("unbalanced s-expression")


def stock_symbol(libid: str) -> str:
    """One symbol from the installed KiCad library, renamed for embedding.

    Only the TOP-LEVEL symbol takes the full "Lib:Name" id. The child unit
    symbols inside it keep their bare "Name_0_1" form -- prefixing those too
    makes the whole file fail to load, which is how this was found.
    """
    lib, name = libid.split(":", 1)
    text = (SYMDIR / f"{lib}.kicad_sym").read_text()
    m = re.search(r'\(symbol "%s"[\s(]' % re.escape(name), text)
    if not m:
        raise SystemExit(f"symbol not found: {libid}")
    blk = sexp_block(text, m.start())
    blk = blk.replace('(symbol "%s"' % name, '(symbol "%s"' % libid, 1)
    return "\n".join("  " + ln for ln in blk.splitlines())


# ── pin geometry, so wire stubs land exactly on pins ─────────────────────────
def pin_positions(symbol_text: str) -> dict[str, tuple[float, float, int]]:
    """number -> (x, y, rotation) in symbol coordinates (+y up)."""
    out: dict[str, tuple[float, float, int]] = {}
    for m in re.finditer(r"\(pin\s+\S+\s+\S+\s+\(at ([-\d.]+) ([-\d.]+) "
                         r"(\d+)\)", symbol_text):
        x, y, rot = float(m.group(1)), float(m.group(2)), int(m.group(3))
        blk = sexp_block(symbol_text, m.start())
        num = re.search(r'\(number "([^"]+)"', blk)
        if num:
            out[num.group(1)] = (x, y, rot)
    return out


def stub(px: float, py: float, rot: int, sx: float, sy: float
         ) -> tuple[float, float, float, float, str]:
    """Where a wire must start and end to attach to one pin.

    Determined by probing KiCad rather than reasoning about it: a pin's
    ``(at)`` IS its connection point -- the ``length`` extends from there
    towards the body, not away from it. Offsetting by the length (the obvious
    reading) puts the wire inside the symbol and every net comes out
    unconnected, which is exactly what happened first time round.

    Symbol space has +y up and schematic space has +y down, hence the flip.
    The outward direction is the pin angle turned about: angle 0 points its
    body right, so its free end runs left.

    :return: (x1, y1, x2, y2, label justification)
    """
    import math
    ax, ay = sx + px, sy - py
    r = math.radians(rot)
    bx = ax - 2.54 * math.cos(r)
    by = ay + 2.54 * math.sin(r)
    just = "right" if bx < ax - 0.01 else "left"
    return ax, ay, round(bx, 2), round(by, 2), just


def main() -> None:
    libs: dict[str, str] = {}
    for _ref, libid, *_rest in PARTS:
        if libid.startswith("bridgebox:"):
            continue
        libs[libid] = stock_symbol(libid)

    pico = pico_symbol()
    mod = module_symbol()
    (HERE / "bridgebox.kicad_sym").write_text(
        "(kicad_symbol_lib (version 20220914) (generator bridgebox)\n"
        + pico + "\n" + mod + "\n)\n")

    # the same two, renamed for embedding in the schematic
    # Same rule: rename the top-level id only, hence count=1.
    libs["bridgebox:Raspberry_Pi_Pico"] = pico.replace(
        '"Raspberry_Pi_Pico"', '"bridgebox:Raspberry_Pi_Pico"', 1)
    libs["bridgebox:MAX3485_Module"] = mod.replace(
        '"MAX3485_Module"', '"bridgebox:MAX3485_Module"', 1)

    sheet_uuid = "11111111-2222-3333-4444-555555555555"
    sch = ['(kicad_sch (version 20230121) (generator bridgebox)',
           "  (uuid %s)" % sheet_uuid,
           "  (paper \"A3\")",
           "  (title_block",
           '    (title "BridgeBox carrier — Pico + MAX3485 → Bambu AMS buffer")',
           '    (company "Sovoron")',
           '    (comment 1 "24 V rated 3 A: buffer + AMS logic + feed motors. '
           'Dryers are on their own supply.")',
           '    (comment 2 "Two layers, 2 oz, L2 an unbroken ground plane. '
           'Nothing crosses L2.")',
           '    (comment 3 "J2 pad-to-circuit mapping MUST be checked against '
           'the Molex drawing and the metered pinout.")',
           "  )",
           "  (lib_symbols"]
    for libid in sorted(libs):
        sch.append(libs[libid])
    sch.append("  )")

    uid = 0

    def nid() -> str:
        nonlocal uid
        uid += 1
        return "00000000-0000-0000-0000-%012d" % uid

    geom: dict[str, dict[str, tuple[float, float, int]]] = {}
    for ref, libid, *_ in PARTS:
        geom[ref] = pin_positions(libs[libid])

    lookup = {ref: (libid, val, fp, x, y)
              for ref, libid, val, fp, x, y in PARTS}

    for ref, libid, val, fp, x, y in PARTS:
        sch += [
            '  (symbol (lib_id "%s") (at %.2f %.2f 0) (unit 1)' % (libid, x, y),
            "    (in_bom yes) (on_board yes) (dnp no)",
            "    (uuid %s)" % nid(),
            '    (property "Reference" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) (justify left)))" % (ref, x + 12, y - 12),
            '    (property "Value" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) (justify left)))" % (esc(val), x + 12, y - 9),
            '    (property "Footprint" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) hide))" % (esc(fp), x, y),
            '    (property "Datasheet" "" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) hide))" % (x, y),
        ]
        for num in sorted(geom[ref], key=lambda n: (len(n), n)):
            sch.append('    (pin "%s" (uuid %s))' % (num, nid()))
        # KiCad 7 keeps the reference on the INSTANCE, not the symbol; without
        # this block the file loads as unannotated at best and not at all at
        # worst, and the path must match the sheet uuid above.
        sch += ["    (instances",
                '      (project "%s"' % NAME,
                '        (path "/%s" (reference "%s") (unit 1))'
                % (sheet_uuid, ref),
                "      )", "    )", "  )"]

    # Wire stubs and labels. Connectivity is by label, so nothing has to be
    # routed across the sheet and no two nets can be joined by a stray crossing.
    for net, pins in NETS.items():
        for ref, num in pins:
            if num not in geom[ref]:
                raise SystemExit(f"{ref} has no pin {num}")
            _libid, _v, _f, sx, sy = lookup[ref]
            px, py, rot = geom[ref][num]
            ax, ay, bx, by, just = stub(px, py, rot, sx, sy)
            sch += [
                "  (wire (pts (xy %.2f %.2f) (xy %.2f %.2f))" % (ax, ay, bx, by),
                "    (stroke (width 0) (type default)) (uuid %s))" % nid(),
                '  (label "%s" (at %.2f %.2f 0) (fields_autoplaced)'
                % (net, bx, by),
                "    (effects (font (size 1.27 1.27)) (justify %s bottom))"
                % just,
                "    (uuid %s))" % nid(),
            ]

    sch += ["  (sheet_instances", '    (path "/" (page "1"))', "  )", ")"]
    (HERE / f"{NAME}.kicad_sch").write_text("\n".join(sch) + "\n")
    print("wrote", NAME + ".kicad_sch", "and bridgebox.kicad_sym")


if __name__ == "__main__":
    main()
