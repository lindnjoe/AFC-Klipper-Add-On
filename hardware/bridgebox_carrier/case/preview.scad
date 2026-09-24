/*
 * Pictures only. An exploded view with a stand-in for the board, so the fit
 * can be looked at rather than reasoned about. Never printed.
 *
 *   openscad -o out/preview.png --camera=... preview.scad
 */

include <case.scad>
part = "none";   // after the include on purpose: last assignment wins

LIFT = 26;      // how far the lid is floated above where it sits

tray();

// The board, as a plate with its four holes and the outline of the parts that
// have to clear something. Not a model of the board -- a placeholder at the
// right height.
color("darkgreen", 0.85)
translate([0, 0, FLOOR + STANDOFF]) difference() {
    cube([BW, BD, BT]);
    for (h = HOLES)
        translate([h[0], BD - h[1], -E]) cylinder(d = 3.2, h = BT + 2 * E);
}

// Parts, as blocks at their placed positions -- from parts.scad, which
// gen_preview_parts.py writes out of the REAL placement. These used to be
// literals here and they went stale the moment the board was re-laid-out,
// which made the pictures show the old board inside the new case.
//
// Heights are nominal and the case does not depend on them, which is the
// point of splitting at the board plane. The USB socket is the exception: it
// is drawn at the height case.scad's USB_CZ claims, so a window that misses
// it is visible here.
include <parts.scad>

for (q = PARTS3D)
    color(q[6], 0.9)
    translate([q[0], BD - q[3], FLOOR + STANDOFF + BT + q[5]])
        cube([q[2] - q[0], q[3] - q[1], q[4]]);

color("lightsteelblue", 0.55) translate([0, 0, LIFT]) lid();
