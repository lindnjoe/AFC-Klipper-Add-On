/*
 * BridgeBox S3 -- two-piece printed case.  All units mm.
 *
 *   openscad -o out/tray.stl -D 'part="tray"' case.scad
 *   openscad -o out/lid.stl  -D 'part="lid"'  case.scad
 *
 * IT SPLITS AT THE BOARD PLANE, and that is inherited from the carrier's case
 * for the same reason it was right there. This project has no 3D models, so
 * every component height is a datasheet number nobody here can check against
 * the part in your hand. Put the split at the TOP FACE OF THE PCB and the
 * problem goes away: the tray holds the board and ends there, the lid covers
 * everything above, and every connector opening becomes a notch in the lid
 * wall whose BOTTOM EDGE IS THE BOARD SURFACE -- which is exactly where every
 * connector's own bottom edge is, because they all sit on the board. A guessed
 * height can then only make a notch too tall, never misaligned.
 *
 * ══ WHAT IS DIFFERENT ON THIS BOARD: THE ANTENNA, AND IT DECIDES THE SHAPE ══
 *
 * The ESP32-S3-WROOM-1 is placed so its antenna end hangs off the top edge.
 * Two consequences the carrier never had:
 *
 *   1. THE MODULE PHYSICALLY OVERHANGS by 6.0 mm. Its body runs to board
 *      y = -6.0, which is outside the outline. A wall at the board edge would
 *      not be tight, it would be through the module. So the cavity extends
 *      ANT_OVER past the top edge, and the board's top edge is NOT a wall.
 *
 *   2. THE KEEPOUT IS 48 x 21 mm OF FREE AIR past that edge, and what it
 *      forbids is COPPER AND METAL -- that is what the footprint's keepout
 *      declares and what Espressif's integration guide is about. **Plastic is
 *      allowed.** So the case may close over the antenna, and this one does,
 *      under two rules that are not negotiable:
 *
 *        * no fastener, insert, nut or any other metal within the keepout;
 *        * the wall over it is thin and unribbed -- ANT_WALL, not WALL.
 *
 *      The top pair of mounting holes sit at board x = 4 and x = 70, and the
 *      keepout spans x = 16..64. That is not a coincidence to be relied on
 *      silently: check_case.py asserts it, because a screw column drifting
 *      into that span is a WiFi fault that no print inspection would show.
 *
 * Board y runs DOWN the KiCad sheet and case y runs UP, so board (x, y)
 * becomes case (x, BD - y) throughout. Every constant below is in BOARD
 * coordinates and converted at the point of use, so they can be read straight
 * off design.py without anyone doing arithmetic in their head.
 */

part = "both";                  // "tray" | "lid" | "both"
insert = false;                 // true: bore the posts for M3 heat-set inserts

$fn = 64;
E = 0.01;

// ── the board, from design.py ────────────────────────────────────────────────
// check_case.py reads these back out of this file and fails if they drift from
// design.BOARD_W / BOARD_H / PLACE.
BW = 74.1;
BD = 65.0;
BT = 1.6;
HOLES = [[4, 6], [70, 6], [4, 50], [70, 46]];       // board coords

// ── the antenna ──────────────────────────────────────────────────────────────
ANT_X    = [16, 64];            // keepout span in board x
ANT_DEEP = 21.0;                // how far it reaches past the top edge
ANT_OVER = 8.0;                 // cavity past the top edge: clears the
                                // module's 6.0 mm of overhang with 2 to spare
ANT_WALL = 1.2;                 // thin, unribbed, over the keepout

// ── fit and wall ─────────────────────────────────────────────────────────────
GAP      = 0.4;                 // board edge to cavity wall, per side
WALL     = 2.4;
FLOOR    = 2.5;
CEIL     = 3.0;
STANDOFF = 5.0;                 // board underside above the tray floor
IH       = 14.0;                // clear height above the board. The tallest
                                // thing on top is the Micro-Fit at ~8.9 and
                                // the barrel jack at ~11; both are in walls,
                                // not under the ceiling.
CR       = 3.0;                 // outside corner radius
SKIRT    = 3.0;                 // how far the lid laps down over the tray
SKIRT_T  = 1.2;
FIT      = 0.15;

// ── fasteners ────────────────────────────────────────────────────────────────
POST    = 7.0;
COL     = 8.0;
PILOT   = insert ? 4.0 : 2.6;
PILOT_D = insert ? 5.5 : 6.5;
CLEAR   = 3.4;
CBORE   = 6.4;
CBORE_D = 1.5;

// ── openings, in BOARD coordinates ───────────────────────────────────────────
// Each is [centre along the wall, clear width]. The height is not given: every
// one is a notch from the board plane up, which is the whole point of the
// split. See check_case.py -- it takes these from here and proves each one
// clears its connector's courtyard with margin.
J1_X  = 10.75; J1_W = 14.0;     // barrel jack, BOTTOM wall. Protrudes 5.7 mm
                                // past the board edge, so this is a hole the
                                // body passes THROUGH, not a notch it peeks at.
J1_H  = 12.0;                   // ... and it is the one opening with a height,
                                // because the jack is a cylinder, not a
                                // board-hugging connector.
J2_X  = 68.5;  J2_W = 14.0;     // Micro-Fit 3.0, BOTTOM wall, face flush
J3_Y  = 40.05; J3_W = 13.0;     // USB-C, LEFT wall, face flush
BTN   = [[14, 22], [14, 28]];   // RESET, BOOT -- lid holes
BTN_D = 5.0;

// ── vents ────────────────────────────────────────────────────────────────────
// Over the buck and the transceiver, which is where the heat is. Nothing over
// the antenna span: see the header.
VENT_W = 2.0; VENT_L = 14.0;
VENTS  = [[24, 44], [24, 52], [52, 26], [52, 34]];  // board coords, centres

// ═════════════════════════════════════════════════════════════════════════════

CW = BW + 2 * GAP;              // cavity width
CD = BD + 2 * GAP + ANT_OVER;   // cavity depth: the board plus the overhang
CY0 = -GAP;                     // cavity y0 in case coords (bottom edge)
OW = CW + 2 * WALL;             // outside width
OD = CD + 2 * WALL;

function by(y) = BD - y;        // board y -> case y

module rrect(w, d, h, r) {
    hull() for (x = [r, w - r], y = [r, d - r])
        translate([x, y, 0]) cylinder(r = r, h = h);
}

// the outer shell, in case coordinates with the origin at the board's
// bottom-left corner minus GAP and WALL
module shell(h) {
    translate([-GAP - WALL, CY0 - WALL, 0]) rrect(OW, OD, h, CR);
}

module cavity(h) {
    translate([-GAP, CY0, 0]) rrect(CW, CD, h, max(0.1, CR - WALL));
}

// ── openings ─────────────────────────────────────────────────────────────────
// All cut generously deep in the wall-normal direction; the shell is what
// bounds them.
module opening_bottom(cx, w, z0, h) {
    translate([cx - w / 2, CY0 - WALL - 1, z0])
        cube([w, WALL + 2, h]);
}

module opening_left(cy, w, z0, h) {
    translate([-GAP - WALL - 1, cy - w / 2, z0])
        cube([WALL + 2, w, h]);
}

// ── the tray ─────────────────────────────────────────────────────────────────
// Floor, walls up to the board plane, posts the board sits on. The board's top
// face is the split, so the tray's walls stop at FLOOR + STANDOFF + BT.
TRAY_H = FLOOR + STANDOFF + BT;

module tray() {
    difference() {
        union() {
            shell(TRAY_H);
            // posts under the mounting holes
            for (h = HOLES)
                translate([h[0], by(h[1]), FLOOR])
                    cylinder(d = POST, h = STANDOFF);
            // ══ AND ONE MORE, WITH NO SCREW IN IT. ══
            // Both cables pull on the bottom edge and the bottom CORNERS are
            // taken -- by the barrel jack on the left and the Micro-Fit on the
            // right -- so no screw can sit there. This post carries that edge
            // on the tray instead. design.py says the same thing from the
            // board's side.
            translate([37, by(61), FLOOR]) cylinder(d = POST, h = STANDOFF);
        }
        // the cavity above the floor
        translate([0, 0, FLOOR]) cavity(TRAY_H);
        // screw pilots
        for (h = HOLES)
            translate([h[0], by(h[1]), FLOOR - E]) {
                cylinder(d = PILOT, h = STANDOFF + E * 2);
                cylinder(d = PILOT_D, h = 1.2);
            }
        // the lid's skirt laps into a rebate around the top of the tray wall
        translate([0, 0, TRAY_H - SKIRT])
            difference() {
                shell(SKIRT + E);
                translate([-GAP - WALL + SKIRT_T + FIT,
                           CY0 - WALL + SKIRT_T + FIT, -E])
                    rrect(OW - 2 * (SKIRT_T + FIT), OD - 2 * (SKIRT_T + FIT),
                          SKIRT + E * 3, max(0.1, CR - SKIRT_T));
            }
        // the barrel jack passes through the tray wall too: its body centre is
        // below the board plane, unlike every other connector here.
        opening_bottom(J1_X, J1_W, FLOOR + STANDOFF - J1_H / 2 + BT, J1_H);
        // and the AMS connector's shell sits on the board, so the tray only
        // needs to not foul the plug's underside
        opening_bottom(J2_X, J2_W, TRAY_H - 2.0, 2.0 + E);
        opening_left(by(J3_Y), J3_W, TRAY_H - 1.6, 1.6 + E);
    }
}

// ── the lid ──────────────────────────────────────────────────────────────────
LID_H = IH + CEIL;

module lid() {
    difference() {
        union() {
            shell(LID_H);
            // the skirt that laps down over the tray rebate
            translate([0, 0, -SKIRT])
                difference() {
                    shell(SKIRT);
                    translate([-GAP - WALL + SKIRT_T, CY0 - WALL + SKIRT_T, -E])
                        rrect(OW - 2 * SKIRT_T, OD - 2 * SKIRT_T,
                              SKIRT + E * 2, max(0.1, CR - SKIRT_T));
                }
        }
        // the inside
        translate([0, 0, -SKIRT - E]) cavity(IH + SKIRT);
        // ══ THE ANTENNA WINDOW: THINNED, NOT OPENED. ══
        // Plastic over the antenna is allowed; thick ribbed plastic is a worse
        // idea than thin plastic, and metal is forbidden outright. So the
        // ceiling over the keepout is reduced to ANT_WALL from the inside.
        translate([ANT_X[0], by(0) - E, IH - E])
            cube([ANT_X[1] - ANT_X[0], ANT_DEEP, CEIL - ANT_WALL + E * 2]);
        // AND THE END WALL, which is the one actually BESIDE the antenna. The
        // module's tip is at case y = 71.0 and the cavity ends at 73.4, so
        // full-thickness wall would put 2.4 mm of plastic 2.4 mm from the
        // radiator. The rule in the header says the wall over the keepout is
        // ANT_WALL; the ceiling obeyed it and this did not, which is the sort
        // of half-applied rule that reads as deliberate later.
        translate([ANT_X[0], CY0 + CD - E, -SKIRT - E])
            cube([ANT_X[1] - ANT_X[0], WALL - ANT_WALL + E * 2,
                  LID_H + SKIRT + E * 2]);
        // screw columns are bored right through
        for (h = HOLES)
            translate([h[0], by(h[1]), -SKIRT - E]) {
                cylinder(d = CLEAR, h = LID_H + SKIRT + E * 2);
                translate([0, 0, LID_H + SKIRT - CBORE_D])
                    cylinder(d = CBORE, h = CBORE_D + E);
            }
        // buttons
        for (b = BTN)
            translate([b[0], by(b[1]), IH - E])
                cylinder(d = BTN_D, h = CEIL + E * 2);
        // vents
        for (v = VENTS)
            translate([v[0] - VENT_L / 2, by(v[1]) - VENT_W / 2, IH - E])
                hull() for (x = [VENT_W / 2, VENT_L - VENT_W / 2])
                    translate([x, VENT_W / 2, 0])
                        cylinder(d = VENT_W, h = CEIL + E * 2);
        // connector notches, from the board plane up
        opening_bottom(J1_X, J1_W, -SKIRT - E, J1_H);
        opening_bottom(J2_X, J2_W, -SKIRT - E, 9.5);
        opening_left(by(J3_Y), J3_W, -SKIRT - E, 5.0);
    }
    // the columns the screws run down, landing on the board around its holes
    for (h = HOLES)
        difference() {
            translate([h[0], by(h[1]), 0]) cylinder(d = COL, h = IH);
            translate([h[0], by(h[1]), -E]) cylinder(d = CLEAR, h = IH + E * 2);
        }
}

if (part == "tray" || part == "both") tray();
if (part == "lid"  || part == "both") translate([0, 0, TRAY_H + 12]) lid();
