#!/usr/bin/env python3
"""
The BridgeBox S3 design, as data.

Everything downstream -- schematic, netlist check, BOM -- reads this and only
this, so the board cannot drift from docs/S3_BOARD.md without this file
changing first.

PINS ARE NAMED, NOT NUMBERED. The carrier's generator hardcodes pin numbers,
which is fine when you can see the symbol library while writing it. These were
written where that library was not available, and a wrong pin NUMBER produces a
netlist that generates cleanly and is wrong -- the worst failure available. A
wrong pin NAME cannot: gen_project.py resolves names against the real symbol at
generation time and stops if one is missing.
"""

from __future__ import annotations

NAME = "bridgebox_s3"

# ── parts ────────────────────────────────────────────────────────────────────
# ref: (lib id, value, footprint, schematic x, y)
#
# Candidate symbol ids are given where the exact library name varies between
# KiCad releases; gen_project.py tries them in order and names the ones it
# could not find. That is deliberate -- guessing a symbol id silently is how
# you get a board built around a part that is not the part.
PARTS: dict[str, dict] = {
    "U1": dict(lib=["RF_Module:ESP32-S3-WROOM-1"],
               value="ESP32-S3-WROOM-1-N8",
               fp="RF_Module:ESP32-S3-WROOM-1",
               at=(120, 90)),
    "U2": dict(lib=["Interface_UART:MAX3485"],
               value="MAX3485ESA",
               fp="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
               at=(215, 105)),
    # TPS54202DDC, SOT-23-6: BOOT(1) VIN(2) EN(3) GND(4) FB(5) SW(6).
    # A buck is not a three-terminal regulator -- L1, the feedback divider and
    # the bootstrap cap are part of the part, and leaving them out gives a
    # netlist that generates and a board that does not regulate.
    "U3": dict(lib=["Regulator_Switching:TPS54202DDC"],
               value="TPS54202DDC 24V->3V3",
               fp="Package_TO_SOT_SMD:SOT-23-6",
               at=(60, 45)),
    "L1":  dict(lib=["Device:L"], value="10u 1A SRN6028",
                fp="Inductor_SMD:L_Bourns-SRN6028", at=(80, 45)),
    "C5":  dict(lib=["Device:C"], value="100n BOOT",
                fp="Capacitor_SMD:C_0402_1005Metric", at=(70, 38)),
    "C6":  dict(lib=["Device:C"], value="4u7 VIN",
                fp="Capacitor_SMD:C_0805_2012Metric", at=(48, 52)),
    "C7":  dict(lib=["Device:C"], value="22u VOUT",
                fp="Capacitor_SMD:C_0805_2012Metric", at=(92, 52)),
    # FB divider for 3.3 V: Vfb = 0.596 V (TPS54202) -> R6/R7 = (3.3/0.596)-1
    # = 4.537. 100k / 22k1 gives 3.296 V, one E96 pair, 33 uA through the leg.
    "R6":  dict(lib=["Device:R"], value="100k",
                fp="Resistor_SMD:R_0402_1005Metric", at=(100, 45)),
    "R7":  dict(lib=["Device:R"], value="22k1",
                fp="Resistor_SMD:R_0402_1005Metric", at=(100, 55)),
    "J1": dict(lib=["Connector:Barrel_Jack_Switch"], value="24V 5.5x2.1",
               fp="Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal",
               at=(25, 45)),
    "J2": dict(lib=["Connector_Generic:Conn_02x02_Odd_Even"],
               value="Micro-Fit 43045-0400",
               fp="Connector_Molex:Molex_Micro-Fit_3.0_43045-0400_2x02_"
                  "P3.00mm_Horizontal",
               at=(275, 105)),
    "J3": dict(lib=["Connector:USB_C_Receptacle_USB2.0_16P"], value="USB-C device",
               bom="USB-C receptacle 16P SMD (HRO TYPE-C-31-M-12)",
               fp="Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
               at=(25, 120)),
    # ══ RESETTABLE, AND IT IS A TRADE. ══
    #
    # Was a 5x20 mm cartridge in a Keystone clip holder: 25.6 mm of board for
    # one part, four pads (two per clip, both numbered), and a glass tube
    # somebody has to keep spares of. A 2920 PPTC is 7.4 x 5.1 mm, two pads,
    # and resets itself.
    #
    # What is given up is real and should not be glossed:
    #
    #  * a PPTC does not blow, it goes high-resistance and stays warm until the
    #    fault clears. Trip is seconds, not milliseconds -- overcurrent
    #    protection, not short-circuit interruption.
    #  * hold current derates hard with ambient. A 2.6 A part holds about 1.9 A
    #    at 60 C, which is a plausible temperature inside a printer enclosure.
    #  * "size the fuse against the ADAPTER" was the rule and it no longer
    #    applies -- the value is fixed at build time instead of chosen per
    #    installation.
    #
    # 30 V rating, not 24 V: a 24 V adapter at +5% is 25.2 V, so a 24 V-rated
    # part is outside its spec on a good day.
    #
    # 2.6 A hold against the board's 2 A budget is the right relationship --
    # hold ABOVE load, not at it. What that leaves open is not the current, it
    # is the AMBIENT: a PPTC derates, and 2.6 A at 25 C is about 2.2 A at 40 C
    # and 1.9 A at 60 C. At 60 C a healthy 2 A load opens the fuse, which
    # presents as the AMS dropping out mid-print with nothing wrong with it.
    # So the measurement that matters here is a thermometer where the board
    # sits, not a clamp meter on the rail. See docs/S3_BOARD.md.
    # ══ MOUNTING HOLES. THE BOARD HAD NONE. ══
    #
    # Which was fine while it was only a board, and is not fine the moment
    # anything has to hold it. Four M3, and the positions are what the parts
    # left rather than what symmetry would like: the bottom CORNERS are taken,
    # by the barrel jack on the left and the Micro-Fit on the right, so the
    # lower pair sits up the sides instead.
    #
    # That leaves the bottom edge -- the one both cables pull on -- held by the
    # tray rather than by a screw. Deliberate: a tray supports the whole board
    # on its floor and standoffs, and the screws only stop it lifting. The case
    # carries an extra post under that edge, with no screw through it.
    "H1": dict(lib=["Mechanical:MountingHole"], value="M3",
               fp="MountingHole:MountingHole_3.2mm_M3", at=(20, 20)),
    "H2": dict(lib=["Mechanical:MountingHole"], value="M3",
               fp="MountingHole:MountingHole_3.2mm_M3", at=(20, 30)),
    "H3": dict(lib=["Mechanical:MountingHole"], value="M3",
               fp="MountingHole:MountingHole_3.2mm_M3", at=(20, 40)),
    "H4": dict(lib=["Mechanical:MountingHole"], value="M3",
               fp="MountingHole:MountingHole_3.2mm_M3", at=(20, 50)),
    "F1": dict(lib=["Device:Polyfuse"], value="PPTC 2.6A hold 30V",
               fp="Fuse:Fuse_2920_7451Metric",
               at=(45, 40)),
    "D1": dict(lib=["Device:D_Schottky"], value="VBUS OR",
               bom="Schottky 30V 1A SOD-123 (BAT60A / SS13)",
               fp="Diode_SMD:D_SOD-123", at=(60, 120)),
    # ══ THREE PINS, NOT TWO. ══
    #
    # This was `Device:D_TVS`, a generic two-pin bidirectional TVS, in a
    # THREE-pad SOT-23. The SM712 is an RS-485 clamp: two asymmetric diodes
    # sharing a `common` pin that is where the surge is supposed to GO. Drawn
    # with the two-pin symbol, both pins resolved, every net checked out, the
    # board routed and passed DRC -- and pad 3 sat with no net on it, so the
    # clamp had no path to ground and could not do the one thing it is for.
    #
    # verify_design.py now checks that every electrical pad carries a net,
    # which is the check that would have caught it. `Diode:SM712_SOT23` is the
    # real part and names its pins A1 / A2 / common.
    "D2": dict(lib=["Diode:SM712_SOT23"], value="SM712 A/B clamp",
               fp="Package_TO_SOT_SMD:SOT-23", at=(245, 125)),
    # decoupling / bulk
    "C1": dict(lib=["Device:C"], value="22u", fp="Capacitor_SMD:C_0805_2012Metric",
               at=(100, 60)),
    "C2": dict(lib=["Device:C"], value="100n", fp="Capacitor_SMD:C_0402_1005Metric",
               at=(110, 60)),
    "C3": dict(lib=["Device:C"], value="100n", fp="Capacitor_SMD:C_0402_1005Metric",
               at=(205, 85)),
    # USB-C CC pull-downs: ONE EACH. A single shared resistor is the classic
    # error and makes C-to-C cables refuse to enumerate.
    "R1": dict(lib=["Device:R"], value="5k1", fp="Resistor_SMD:R_0402_1005Metric",
               at=(45, 130)),
    "R2": dict(lib=["Device:R"], value="5k1", fp="Resistor_SMD:R_0402_1005Metric",
               at=(52, 130)),
    # ══ BIAS AND TERMINATION, BOTH FITTED, FROM THE MODULE IN SERVICE. ══
    #
    # Read off the breakout actually driving the bus (photo, 2026-09-11):
    # 472 / 121 / 472 -- two 4k7 biasing A up and B down, AND a 120R
    # termination between them. All three populated.
    #
    # An earlier note here called that combination a failure, on this sum:
    #
    #     idle differential = 3V3 * 120 / (4700 + 120 + 4700) = 42 mV
    #
    # against the MAX3485's +/-200 mV receiver threshold. The sum is right and
    # the conclusion was wrong: +/-200 mV is the GUARANTEED threshold over
    # process and temperature, not the typical one, which sits near zero with
    # a few tens of mV of hysteresis. 42 mV is outside the datasheet's promise,
    # not outside what the part does -- and the hardware has been reliable on
    # exactly this for weeks. Copy what works.
    #
    # The margin is still thin and it is worth knowing where the lever is: at
    # 120R fitted, bias resistors of ~820R put the idle differential at 225 mV,
    # inside the guarantee, for 1.9 mA of extra DC load on the drivers -- which
    # nothing on this bus would notice. Change it only if a unit ever proves
    # marginal; do not change it speculatively, because the current values are
    # the ones with evidence behind them.
    # ══ 4k7 BECAUSE THE MODULE THAT WORKS READS 4k7. ══
    #
    # Fail-safe bias: R1B pulls A up to 3V3, R2B pulls B down to GND. With the
    # 120 R termination fitted these put the idle differential at 42 mV, which
    # is under the MAX3485's +/-200 mV GUARANTEED threshold -- the level below
    # which a receiver is permitted to be undecided, not one at which it fails.
    # Typical offset is near zero, and the breakout reading 472/121/472 has run
    # this bus reliably for weeks.
    #
    # 820 R would give 225 mV and is the obvious "follow the datasheet" answer.
    # It is also conditional on there being exactly ONE termination on the bus:
    # with one at each end it gives 116 mV and misses the same guarantee, after
    # changing the DC loading and throwing away the only values with evidence
    # behind them. docs/S3_BOARD.md carries the whole table.
    #
    # These are 0402 in either case, so the value is a BOM line and not a
    # layout decision. It gets made after somebody meters A-B at idle on the
    # live bus, not from a number remembered out of a document.
    "R1B": dict(lib=["Device:R"], value="4k7 bias A",
                fp="Resistor_SMD:R_0402_1005Metric", at=(240, 96)),
    "R2B": dict(lib=["Device:R"], value="4k7 bias B",
                fp="Resistor_SMD:R_0402_1005Metric", at=(240, 118)),
    "R3": dict(lib=["Device:R"], value="120R", fp="Resistor_SMD:R_0805_2012Metric",
               at=(245, 110)),
    # EN reset RC
    "R4": dict(lib=["Device:R"], value="10k", fp="Resistor_SMD:R_0402_1005Metric",
               at=(95, 75)),
    "C4": dict(lib=["Device:C"], value="1u", fp="Capacitor_SMD:C_0402_1005Metric",
               at=(95, 82)),
    # GPIO0 boot strap
    "R5": dict(lib=["Device:R"], value="10k", fp="Resistor_SMD:R_0402_1005Metric",
               at=(95, 100)),
    "SW1": dict(lib=["Switch:SW_Push"], value="BOOT", bom="tactile SPST 4.5x4.5mm SMD (B3U-1000P)",
                fp="Button_Switch_SMD:SW_SPST_B3U-1000P", at=(88, 107)),
    "SW2": dict(lib=["Switch:SW_Push"], value="RESET", bom="tactile SPST 4.5x4.5mm SMD (B3U-1000P)",
                fp="Button_Switch_SMD:SW_SPST_B3U-1000P", at=(88, 70)),
}

# ── nets ─────────────────────────────────────────────────────────────────────
# net -> [(ref, PIN NAME), ...]   mirrors the table in docs/S3_BOARD.md
NETS: dict[str, list[tuple[str, str]]] = {
    "+24V_IN": [("J1", "1"), ("F1", "1")],
    "+24V":    [("F1", "2"), ("J2", "4"), ("U3", "VIN"), ("U3", "EN"),
                ("C6", "1")],
    "SW":      [("U3", "SW"), ("L1", "1"), ("C5", "2")],
    "BOOT_C":  [("U3", "BOOT"), ("C5", "1")],
    "FB":      [("U3", "FB"), ("R6", "2"), ("R7", "1")],
    "+3V3":    [("L1", "2"), ("C7", "1"), ("R6", "1"),
                ("U1", "3V3"), ("U2", "VCC"),
                ("C1", "1"), ("C2", "1"), ("C3", "1"), ("R4", "1"),
                ("R5", "1"), ("D1", "K"), ("R1B", "1")],
    "GND":     [("J1", "2"), ("J2", "3"), ("J3", "GND"), ("U1", "GND"),
                ("U2", "GND"), ("U3", "GND"), ("C1", "2"), ("C2", "2"),
                ("C3", "2"), ("C4", "2"), ("R1", "2"), ("R2", "2"),
                ("C6", "2"), ("C7", "2"), ("R7", "2"),
                ("SW1", "2"), ("SW2", "2"),
                # ── three that were missing, all found by the pad-coverage
                # check in verify_design.py, none visible any other way ──
                #
                # R2B is the fail-safe bias PULL-DOWN. S3_BOARD.md says it in
                # as many words -- "biasing A up to VCC and B down to GND" --
                # and the down half was not connected to anything. Without it
                # there is no bias at all: an idle bus floats instead of
                # holding a known differential, which is the failure mode the
                # resistors exist to prevent.
                ("R2B", "2"),
                # the SM712's common pin: see D2 above.
                ("D2", "common"),
                # the USB-C shell. Unconnected it is an antenna with a
                # connector on the end -- no ESD path off the plug, and the
                # cable braid terminated nowhere. Straight to GND is the
                # ordinary device-side choice; an RC to chassis is the other
                # one, and there is no chassis here.
                ("J3", "SHIELD")],
    # the bus, and the crossover done in copper -- see docs/S3_BOARD.md
    "TXD":     [("U1", "IO17"), ("U2", "DI")],
    "RXD":     [("U1", "IO18"), ("U2", "RO")],
    "DE":      [("U1", "IO16"), ("U2", "DE"), ("U2", "~{RE}")],
    "BUS_A":   [("U2", "B"), ("J2", "1"), ("R3", "1"), ("D2", "A1"),
                ("R1B", "2")],
    "BUS_B":   [("U2", "A"), ("J2", "2"), ("R3", "2"), ("D2", "A2"),
                ("R2B", "1")],
    # USB-C: the host link
    "USB_DP":  [("J3", "D+"), ("U1", "IO20")],
    "USB_DM":  [("J3", "D-"), ("U1", "IO19")],
    "USB_VBUS": [("J3", "VBUS"), ("D1", "A")],
    "USB_CC1": [("J3", "CC1"), ("R1", "1")],
    "USB_CC2": [("J3", "CC2"), ("R2", "1")],
    # straps and buttons
    "EN":      [("U1", "EN"), ("R4", "2"), ("C4", "1"), ("SW2", "1")],
    "BOOT":    [("U1", "IO0"), ("R5", "2"), ("SW1", "1")],
}

# ── things a checker can enforce without KiCad ───────────────────────────────
# ESP32-S3 pins that must never carry a signal. See docs/S3_BOARD.md.
FORBIDDEN_IO = (
    [f"IO{n}" for n in range(26, 33)]                       # SPI flash
    + [f"IO{n}" for n in range(33, 38)]                     # octal PSRAM (-R8)
)
STRAPPING_IO = ["IO0", "IO3", "IO45", "IO46"]
USB_IO = ["IO19", "IO20"]


# ── board placement ──────────────────────────────────────────────────────────
# ref -> (x, y, rotation) in BOARD mm. KiCad board space: +x right, +y DOWN.
#
# THE ANTENNA SETS EVERYTHING ELSE. The WROOM-1's own footprint carries a
# keepout zone -- tracks, vias, pads and copper pour all not_allowed -- over
# x[-24,24] y[-27.75,-6.75] local, a 48 x 21 mm rectangle beyond the antenna
# end. It cannot be satisfied on a board this size, so the antenna HANGS OFF
# the edge: U1 sits at y = +6.75, which puts the keepout's lower boundary
# exactly on the board's top edge and the whole of it in free air.
#
# That is not a trick to silence a checker. It is what the module wants: any
# copper under a PCB antenna detunes it, and a board that works on the bench
# and drops WiFi inside a printer is the usual way of finding out.
# ══ THE BOARD IS AS BIG AS ITS TWO CONNECTORS SAY. ══
#
# Not as big as its parts: the interior is sparse and the fuse swap freed a
# 25 x 3 mm strip along the bottom. Neither dimension follows from that.
#
#   HEIGHT is the barrel jack. J1's pads end at y = 64.30 and its BODY
#   deliberately overhangs the bottom edge by 6.2 mm, because that is how a
#   horizontal jack presents its opening. Move it up and the plug no longer
#   reaches. 65 is not slack, it is the part.
#
#   WIDTH was read off the Micro-Fit's courtyard, and that was reading the
#   wrong axis. J2 faces along y, not x; its courtyard's x extent is the plain
#   0.5 mm margin and never meant anything. Turned to face the bottom edge as
#   it must, J2's courtyard reaches x = 74.08, and nothing on the board goes
#   further right. 80 -> 77.1 -> 74.1, and only the last number was measured
#   against the question that decides it.
#
# Shrinking further means RE-PLACING, not trimming, and re-placing re-opens the
# routing. It is available and it is not free.
BOARD_W, BOARD_H = 74.1, 65.0
ANTENNA_KEEPOUT = (-24.0, -27.75, 24.0, -6.75)     # local to U1


# ── DELIBERATELY UNCONNECTED ─────────────────────────────────────────────────
#
# verify_design.py checks that every electrical pad on every footprint carries
# a net. Three real faults came out of that the first time it ran -- a TVS with
# no path to ground, a bias resistor with no pull-down, a USB shell left
# floating -- and so did the pins below, which are genuinely meant to be open.
#
# They are DECLARED rather than filtered. An undeclared floating pad is a
# question nobody asked; a declared one is a decision with a reason attached,
# and the check goes quiet only for pads somebody has actually thought about.
NO_CONNECT: dict[str, tuple[str, set[str]]] = {
    "J1": ("the PJ-102AH's switch contact -- it breaks when a plug goes in, "
           "and nothing here needs to know that",
           {"3"}),
    "J3": ("SBU1/SBU2 carry alternate modes. A USB 2.0 DEVICE port has none, "
           "and the spec leaves them open",
           {"A8", "B8"}),
    "U1": ("unused GPIO. Listed rather than wildcarded, so adding a feature "
           "shows up here as a pin leaving the free list",
           {
              "4", "5", "6", "7", "8", "12", "15", "16", "17",
              "18", "19", "20", "21", "22", "23", "24", "25",
              "26", "28", "29", "30", "31", "32", "33", "34",
              "35", "36", "37", "38", "39"
           }),
}


# ── CONNECTORS, AND WHICH WAY THEY FACE ──────────────────────────────────────
#
# A connector whose opening does not reach a board edge takes no cable, and
# NOTHING ELSE HERE CAN SEE THAT. DRC checks copper. The netlist checks nets.
# The plot looks finished. J2 sat at rot 0 for the whole of this board's life,
# which aimed its opening at board y = 43.08 with 43 mm of PCB in front of it,
# and every check in this directory passed.
#
# So the direction is DECLARED, per connector, as the local axis its mating
# face lies on, and check_placement.py maps that through the placement rotation
# and insists it lands on an edge. The face comes from the F.Fab BODY outline,
# not the courtyard: a courtyard's asymmetry is as often a second pad row as it
# is mating clearance, and reading it as the latter is what hid this.
#
# `overhang` says whether the body is allowed past the edge. A barrel jack is
# MEANT to protrude -- that is how it meets a panel -- while a flush connector
# sitting proud of the outline would foul a case.
MATING: dict[str, tuple[str, bool, str]] = {
    "J1": ("+y", True,
           "barrel jack: the body passes through the panel, so it protrudes"),
    "J2": ("-y", False,
           "Micro-Fit: shell is on the -y side of the pads, flush to the edge"),
    "J3": ("+y", False,
           "USB-C: the shell's outer face, flush so the plug seats on the edge"),
}

PLACE: dict[str, tuple[float, float, float]] = {
    # the module, antenna overhanging the top edge
    "U1":  (40.0,  6.75,   0.0),
    # its decoupling, close in on the 3V3 pins
    "C1":  (33.0, 23.0,    0.0),
    "C2":  (37.0, 23.0,    0.0),
    # EN reset RC and the boot strap, left of the module
    "R4":  (25.0, 22.0,   90.0),
    "C4":  (25.0, 26.0,   90.0),
    "R5":  (21.0, 22.0,   90.0),
    "SW2": (14.0, 22.0,    0.0),   # RESET
    "SW1": (14.0, 28.0,    0.0),   # BOOT
    # USB-C on the left edge, opening outward.
    #
    # ══ THE .05 IS NOT A TYPO AND IT IS NOT COSMETIC. ══
    #
    # This footprint puts all sixteen contacts in ONE row on a 0.5 mm pitch, so
    # every pad centre is an odd multiple of 0.25 mm from the part origin. At
    # y = 40.0 that lands them on 39.25 / 39.75 / 40.25 / 40.75 -- exactly half
    # of route.py's 0.1 mm grid cell away from any grid line.
    #
    # Escaping one of these pads clears its neighbour by 0.025 mm when the
    # track is perfectly centred (0.5 pitch, 0.15 pad half-height, 0.325 of
    # track half-width plus clearance). Half a cell of offset eats that twice
    # over, so the router could not leave A5/A6/A7/B6/B7 AT ALL and reported
    # four USB nets as unroutable. That got written up as a geometric fact
    # about USB-C needing a second layer. It is not: it is a placement that the
    # grid cannot represent.
    #
    # 0.05 mm puts the pads on 39.30 / 39.80 / 40.30 / 40.80, on grid, and the
    # escapes exist. Snapping a connector so its pads land on the routing grid
    # is ordinary practice; the alternative was a 0.05 mm grid, which is four
    # times the cells everywhere to fix one part.
    #
    # x was 6.0, which left the receptacle's courtyard starting at x = 1.85 --
    # the shell 1.85 mm INBOARD of the board edge it is supposed to be flush
    # with. A horizontal USB-C receptacle is meant to sit at the edge so the
    # plug seats against the board and the shell is supported by it; set back,
    # the opening is in the middle of nothing. 4.15 puts the courtyard at
    # exactly x = 0.
    "J3":  ( 4.15, 40.05, 270.0),
    "R1":  (14.0, 36.0,   90.0),
    "R2":  (14.0, 39.0,   90.0),
    "D1":  (14.0, 44.0,   90.0),
    # mounting holes: top pair in the corners, bottom pair up the sides
    # because J1 and J2 own the bottom corners
    "H1":  ( 4.0,  6.0,   0.0),
    "H2":  (70.0,  6.0,   0.0),
    "H3":  ( 4.0, 50.0,   0.0),
    "H4":  (70.0, 46.0,   0.0),
    # 24 V in, bottom left. The fuse sits immediately after it on the same
    # rail rather than spanning the bottom of the board -- it is a 2920 chip
    # now, not a 25.6 mm cartridge holder, so it goes where the current goes.
    "J1":  (10.0, 57.0,    0.0),
    "F1":  (23.0, 58.0,    0.0),
    # the buck, middle of the board, switch node kept tight
    "C6":  (24.0, 48.0,    0.0),
    "U3":  (30.0, 48.0,    0.0),
    "C5":  (30.0, 44.0,    0.0),
    "L1":  (37.0, 48.0,    0.0),
    "C7":  (44.0, 48.0,    0.0),
    "R6":  (48.0, 44.0,   90.0),
    "R7":  (48.0, 51.0,   90.0),
    # the transceiver and the bus, right side, near J2
    "U2":  (58.0, 33.0,    0.0),
    "C3":  (58.0, 27.0,    0.0),
    "R1B": (52.0, 40.0,   90.0),
    "R2B": (56.0, 40.0,   90.0),
    "R3":  (60.0, 40.0,   90.0),
    "D2":  (64.0, 40.0,   90.0),
    # ══ ROT 180 IS THE WHOLE POINT: IT IS WHICH WAY THE CABLE GOES IN. ══
    #
    # This sat at rot 0, and at rot 0 this footprint's mating face points at
    # local -y. Placed at y = 52 that put the opening at board y = 43.08,
    # aimed INTO the board, with 43 mm of PCB in front of it. Routed fine,
    # passed DRC, plotted like a finished board, and no cable could ever have
    # been plugged into it.
    #
    # Nothing in this directory could see that: DRC checks copper, not whether
    # a connector faces out. The courtyard does not say either -- its +y
    # overhang is covering the second pad row, not mating clearance, which is
    # what made it look like the x extent set the board width. The BODY is what
    # says: F.Fab spans y -8.92..0.99 with the pads at y 0 and 3, so the shell
    # is on the -y side of the pads and the opening is its far end.
    #
    # rot 180 turns that to +y, and y = BOARD_H - 8.92 puts the face exactly on
    # the bottom edge. Which is what the carrier does -- J2 at (39, 45, 180) on
    # a 54 mm board, face at 53.92 -- and that board exists and works.
    #
    # It shares the bottom edge with J1 now. J1's barrel deliberately overhangs
    # by 5.7 mm to pass through a panel; this sits flush behind a cutout. One
    # face, both cables, which is what a case wants.
    "J2":  (70.0, 56.08, 180.0),
}
