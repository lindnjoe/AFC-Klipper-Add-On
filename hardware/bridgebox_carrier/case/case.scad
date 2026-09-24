/*
 * BridgeBox carrier -- two-piece printed case.  All units mm.
 *
 *   openscad -o out/tray.stl -D 'part="tray"' case.scad
 *   openscad -o out/lid.stl  -D 'part="lid"'  case.scad
 *
 * WHY IT SPLITS AT THE BOARD PLANE
 *
 * The one dimension this project does not have is height. KiCad footprints are
 * plan views; no 3D models are installed, so every component's height here
 * would be a number from a datasheet I cannot check against the parts in the
 * user's hand -- and the Pico's height in particular depends on which headers
 * it is sat on, which is the builder's choice, not the board's.
 *
 * So the split line is the top face of the PCB. The tray holds the board and
 * ends there; the lid is a shell that covers everything above it. Every
 * connector opening is then a notch in the lid wall whose BOTTOM EDGE IS THE
 * BOARD SURFACE -- exactly where every connector's own bottom edge is, because
 * they all sit on the board. A guessed component height can only make a notch
 * too tall, never misaligned. That is the whole trick, and it is why the
 * numbers below are allowed to be generous instead of exact.
 *
 * ONE SET OF SCREWS
 *
 * Four M3x25 go in from the top: through the lid, down a column that lands on
 * the board around its mounting hole, through the board, and into a post in
 * the tray. One screw clamps lid, board and tray together. Undoing them frees
 * the lid and the board at once, which is what you want anyway -- the fuse is
 * inside.
 */

include <logo.scad>

part = "both";                  // "tray" | "lid" | "both"
insert = false;                 // true: bore the posts for M3 heat-set inserts

$fn = 64;
E = 0.01;

// ── the board, from gen_pcb.py ───────────────────────────────────────────────
// Keep these in step with gen_pcb.W / .H / .HOLES. check_case.py fails if they
// drift.
BW = 63.0;
BD = 54.0;
BT = 1.6;
HOLES = [[6, 32], [24, 48.5], [56, 50], [59, 8]];

// ── fit and wall ─────────────────────────────────────────────────────────────
GAP      = 0.4;                 // board edge to cavity wall, per side
WALL     = 2.4;
FLOOR    = 2.5;
CEIL     = 3.5;
STANDOFF = 6.0;                 // board underside above the tray floor
                                // (raised 1 mm so the J2 opening can drop
                                //  J2_DROP below the split and still leave wall
                                //  material -- "meat" -- under it; see J2_DROP)
IH       = 21.0;                // clear height above the board
CR       = 3.0;                 // outside corner radius
SKIRT    = 3.0;                 // how far the lid laps down over the tray
SKIRT_T  = 1.2;                 // ... and how thick that lap is
FIT      = 0.15;                // slip clearance between lap and rebate

// ── fasteners ────────────────────────────────────────────────────────────────
POST    = 7.0;                  // tray post outside diameter
COL     = 8.0;                  // lid column outside diameter
PILOT   = insert ? 4.0 : 2.6;   // post bore: insert seat, or self-tapping M3
PILOT_D = insert ? 5.5 : 6.5;
CLEAR   = 3.4;                  // M3 shank clearance through the lid column
CBORE   = 6.4;                  // socket-head counterbore in the lid top
CBORE_D = 1.5;

// ── openings ─────────────────────────────────────────────────────────────────
// Board coordinates converted to case coordinates by (BD - y): KiCad's y runs
// down the sheet, OpenSCAD's runs up. The LEFT wall (x = 0) carries the Pico's
// USB and the barrel jack; the FRONT wall (board y = 58, case y = 0) carries
// the AMS 4-pin, a quarter turn round as the board was laid out.
// The USB opening is a WINDOW, not a notch, and it is the one opening whose
// height is measured rather than guessed. The Pico does not sit on the board,
// it sits on headers, so its socket is well up the wall; USB_CZ is the socket
// centre above the board's UNDERSIDE, which is how it is measured on the
// assembly. The opening is then an ordinary cable-sized hole centred there
// instead of a slot running all the way down to the split line.
//
// The lid still drops straight down. That only works because the socket is
// set BACK from the carrier board's edge -- it overhangs the Pico's own PCB,
// not this one, and the Pico is placed far enough in that it stops 2.3 mm
// short of the outline. So the wall below the window sweeps past empty space.
// Move the Pico towards that edge and this stops being true.
USB_CZ = 15.0;                                     // socket centre, from the
                                                   // board's underside
USB_W = 13.0;                                      // plug moulding, not shell
USB_H = 7.5;                                       // ... likewise
USB_Y = BD - 13.0;                                 // Pico USB, board y 9.5-16.5
J1_W  = 11.5;  J1_AX =  6.3;  J1_Y  = BD - 45.0;   // barrel axis, board y 45.0
J2_W  = 13.5;  J2_H  = 11.0;  J2_X  = 37.5;        // Micro-Fit, board x 31.9-43.1
// The 4-pin (J2) opening drops J2_DROP BELOW the split so the connector's plug
// seats fully -- most connector openings stop at the board plane, but this one
// needs a little room under it. That means cutting into the tray's front wall
// (and the lid lap that faces it), not just the lid, so both parts carry the
// drop. STANDOFF is raised by the same 1 mm above, so the wall under the cut
// keeps its original thickness.
J2_DROP = 1.0;

// Vents, as [x0, x1, [board y, ...]] banks over the parts that want them:
// the Pico, the RS-485 module, and the fuse -- the one part that turns any
// real power into heat. A LIST rather than three hardcoded loops so the whole
// board description can be overridden from the command line, which is what
// renders the case for the older 84 x 58 board (see build.sh).
VENT_BANKS = [
    [34, 50, [5, 9, 13]],             // over the Pico (starts ~5 mm clear of the BOOTSEL slot)
    [30, 50, [46.5, 50]],             // low intake, below the engraved panel
];

// BOOTSEL. A Pico with no button access is a Pico you have to unscrew the case
// to flash the first time. The RP2350 Pico 2 moved the button: SW1 sits
// ~21.5 mm from the USB-end board edge (measured on the assembled board -- 9.5 mm
// further in than an earlier datasheet-figure estimate had it, and well past the
// original Pico's ~6.5 mm) and ~3.6 mm off the centre line. The opening is a slot
// laid ACROSS the board (rotated 90 from its length) at that distance from the
// USB end, running BOOT_OFF either side of the centre line so it reaches SW1
// whichever side it sits -- and clear of the (horizontal) Pico vent bank, which
// is shortened to start past it. BOOT_X = 3.5 (USB-end edge, board x) + 21.5.
BOOT_D   = 8.0;
BOOT_X   = 25.0;         // Pico 2 SW1: ~21.5 mm from the USB-end board edge
BOOT_OFF = 3.6;          // SW1's side offset from centre; the slot spans +/- it
BOOT_Y   = BD - 13.0;    // board centre line

VENTS = true;

// ── the mark ─────────────────────────────────────────────────────────────────
// Engraved into the lid's top face, in the clear band between the low screw
// bosses and the BOOTSEL slot. Mind that slot: it is a hull of two d8 circles,
// so it reaches 4 mm PAST the centres its numbers name -- y 33.4, not 37.4 --
// and sizing the panel off the centres puts the mark through it.
//
// 48 mm is the most this board holds, solved against every top-face opening.
// It is NOT vents that cap it here (removing them buys nothing) but the
// BOOTSEL slot and the screw bosses. The older 84 x 58 lid has room for 65 mm
// -- see build.sh.
//
// The first 84 x 58 lid off the printer came out with most of the letters and
// the nozzle solid, and two explanations were written here in turn -- that
// 65 mm was too small, then that the engraved grooves land at 0.71 mm, the one
// width a slicer handles worst. Neither was it. The same STL sliced in Bambu
// Studio prints the mark open at both sizes; only OrcaSlicer fills it in. A
// slicer bug, so the model carries no correction for it and LOGO_BOLD is 0.
// See logo.scad for what the lever does if it is ever genuinely needed.
//
// Size really is not the issue either way: an opening at the nozzle radius
// keeps 99.9% of the engraving at BOTH sizes, so every stroke is reachable.
//
// Depth goes to 0.8: four layers at 0.2 rather than three. A groove this
// shallow reads by shadow, and one more layer of shadow is nearly free on a
// part that is 6 mm thick. Still nowhere near structural.
LOGO_W     = 48.0;
LOGO_AT    = [34.7, 21.2];      // lid coordinates, centre of the mark
LOGO_DEPTH = 0.8;
LOGO_BOLD  = 0;

// ── derived ──────────────────────────────────────────────────────────────────
ZB = FLOOR + STANDOFF + BT;     // top face of the PCB -- the split line
OZ = ZB + IH + CEIL;            // outside of the lid

// Screw length. REACH is head seat down to the top of the tray post -- the
// part that is clamped and buys no thread. Anything shorter than REACH + 3 has
// nothing to bite; anything longer than REACH + PILOT_D bottoms out in a blind
// hole and jacks the lid back up, which feels like a stripped post and is not.
REACH = (OZ - CBORE_D) - (FLOOR + STANDOFF);

echo(str("outside  ", BW + 2 * (GAP + WALL), " x ", BD + 2 * (GAP + WALL),
         " x ", OZ, " mm"));
// Pick the standard length out of the window instead of naming one, so a
// change in height cannot leave the advice pointing at the old screw.
STD = [10, 12, 16, 20, 25, 30, 35, 40];
FITS = [for (l = STD) if (l >= REACH + 3 && l <= REACH + PILOT_D) l];
echo(str("split at ", ZB, " mm; M3 screw ", REACH + 3, " to ",
         REACH + PILOT_D, " mm long -- use M3 x ",
         len(FITS) > 0 ? FITS[0] : "?? (no standard length fits)"));

// ── outlines ─────────────────────────────────────────────────────────────────

module outer2d() {
    hull() for (p = [[-GAP - WALL + CR, -GAP - WALL + CR],
                     [BW + GAP + WALL - CR, -GAP - WALL + CR],
                     [BW + GAP + WALL - CR, BD + GAP + WALL - CR],
                     [-GAP - WALL + CR, BD + GAP + WALL - CR]])
        translate(p) circle(r = CR);
}

// The cavity, plus a relief circle at each corner. A printed inside corner is
// never square -- it carries the nozzle's radius -- and a square board corner
// pushed into one sits proud of the floor.
module inner2d() {
    translate([-GAP, -GAP]) square([BW + 2 * GAP, BD + 2 * GAP]);
    for (p = [[-GAP, -GAP], [BW + GAP, -GAP],
              [BW + GAP, BD + GAP], [-GAP, BD + GAP]])
        translate(p) circle(r = 1.4);
}

// ── tray ─────────────────────────────────────────────────────────────────────

module tray() {
    difference() {
        union() {
            difference() {
                linear_extrude(ZB) outer2d();
                translate([0, 0, FLOOR]) linear_extrude(ZB) inner2d();
                // Rebate the top of the wall on the OUTSIDE so the lid's lap
                // sits flush instead of standing proud of it.
                translate([0, 0, ZB - SKIRT]) linear_extrude(SKIRT + E)
                    difference() {
                        offset(1) outer2d();   // past the face, not on it
                        offset(-SKIRT_T) outer2d();
                    }
            }
            for (h = HOLES)
                translate([h[0], BD - h[1], FLOOR - E])
                    cylinder(d = POST, h = STANDOFF + E);
        }
        for (h = HOLES)
            translate([h[0], BD - h[1], FLOOR + STANDOFF - PILOT_D])
                cylinder(d = PILOT, h = PILOT_D + E);
        // The J2 opening drops J2_DROP below the split -- notch the top of the
        // tray's front wall to match the lid, so the connector seats fully.
        translate([J2_X - J2_W / 2, -GAP - WALL - 1, ZB - J2_DROP])
            cube([J2_W, WALL + 2, J2_DROP + E]);
    }
}

// ── lid ──────────────────────────────────────────────────────────────────────

// A notch in the left wall: through the wall, open at the board plane.
module notch_left(ycen, w, h) {
    translate([-GAP - WALL - 1, ycen - w / 2, ZB]) cube([WALL + 2, w, h]);
}

// A closed opening in the left wall, centred at zcen above the split line --
// for a connector that is NOT sitting on the board and so does not want the
// wall below it cut away. See the USB note above for what that costs.
module window_left(ycen, w, zcen, h) {
    translate([-GAP - WALL - 1, ycen - w / 2, ZB + zcen - h / 2])
        cube([WALL + 2, w, h]);
}

// The same, capped with a half round at the plug axis so a barrel plug's moulded
// body clears without cutting a square hole the size of the plug.
module notch_left_bore(ycen, w, ax) {
    hull() {
        translate([-GAP - WALL - 1, ycen - w / 2, ZB])
            cube([WALL + 2, w, E]);
        translate([-GAP - WALL - 1, ycen, ZB + ax])
            rotate([0, 90, 0]) cylinder(d = w, h = WALL + 2);
    }
}

// A notch in the front wall. `drop` extends it below the split (default 0) --
// through the lid's lap -- for a connector that must seat a little under the
// board plane; the tray wall is notched to match (see tray()).
module notch_front(xcen, w, h, drop = 0) {
    translate([xcen - w / 2, -GAP - WALL - 1, ZB - drop])
        cube([w, WALL + 2, h + drop]);
}

module slot(x0, x1, y, w) {
    hull() for (x = [x0, x1])
        translate([x, y, ZB + IH - E]) cylinder(d = w, h = CEIL + 2 * E);
}

module lid() {
    difference() {
        union() {
            translate([0, 0, ZB]) difference() {
                linear_extrude(IH + CEIL) outer2d();
                linear_extrude(IH) inner2d();
            }
            // The lap that drops into the tray's rebate.
            translate([0, 0, ZB - SKIRT]) linear_extrude(SKIRT)
                difference() {
                    outer2d();
                    offset(-SKIRT_T + FIT) outer2d();
                }
            for (h = HOLES)
                translate([h[0], BD - h[1], ZB]) cylinder(d = COL, h = IH);
        }
        for (h = HOLES) {
            translate([h[0], BD - h[1], ZB - E])
                cylinder(d = CLEAR, h = IH + CEIL + 2 * E);
            translate([h[0], BD - h[1], OZ - CBORE_D])
                cylinder(d = CBORE, h = CBORE_D + E);
        }

        window_left(USB_Y, USB_W, USB_CZ - BT, USB_H);
        notch_left_bore(J1_Y, J1_W, J1_AX);
        notch_front(J2_X, J2_W, J2_H, J2_DROP);

        // A stadium slot across the board, spanning SW1's side offset either
        // way, so BOOTSEL is reachable whichever side of centre the button sits.
        hull()
            for (dy = [-BOOT_OFF, BOOT_OFF])
                translate([BOOT_X, BOOT_Y + dy, ZB + IH - E])
                    cylinder(d = BOOT_D, h = CEIL + 2 * E);

        // Over the Pico, plus a low intake under the engraved panel.
        if (VENTS) {
            for (bank = VENT_BANKS)
                for (y = bank[2]) slot(bank[0], bank[1], BD - y, 2.6);
        }

        // The mark, sunk into the top face. Cut last and from the outside in,
        // so it is a recess in whatever surface survives the openings above
        // rather than a shape that has to be reconciled with them.
        translate([LOGO_AT[0], LOGO_AT[1], OZ - LOGO_DEPTH])
            linear_extrude(LOGO_DEPTH + E) bridgebox_logo(LOGO_W, LOGO_BOLD);
    }
}

// ── output ───────────────────────────────────────────────────────────────────

// "none" draws nothing, which is what preview.scad wants after it has
// included this file for the modules and the numbers.
if (part == "tray") tray();
else if (part == "lid") lid();
else if (part == "both") {
    tray();
    // Lid laid out beside the tray, flipped the way it prints: open side up,
    // top face on the bed. Nothing in it then needs support.
    translate([0, BD + 2 * (GAP + WALL) + 10, OZ])
        rotate([180, 0, 0]) lid();
}
