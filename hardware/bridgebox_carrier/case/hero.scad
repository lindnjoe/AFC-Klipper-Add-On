// Hero render: the assembled case, the board, and the 4-pin in its window.
//
// Corrected against a photograph of the printed article. The shell is BLACK,
// not natural nylon; it stands on the green board, which shows through the
// window under it; and it sits well back from the wall, so the window's own
// thickness reads around it.
include <case.scad>
include <parts.scad>
part = "none";

color("#6f7982") tray();
color("#8a949e") lid();

// The board, so the window shows what the photograph shows: green under the
// connector rather than a hole into nothing.
color("#1f6b3a") translate([0, 0, FLOOR + STANDOFF]) cube([BW, BD, BT]);

// The Micro-Fit, at gen_pcb.py's placement -- the transform preview.scad uses.
for (q = PARTS3D) if (q[7] == "J2 Micro-Fit") {
    w = q[2] - q[0];  d = q[3] - q[1];  hgt = q[4];
    translate([q[0], BD - q[3], FLOOR + STANDOFF + BT + q[5]])
        color("#1a1a1c") difference() {
            cube([w, d, hgt]);
            // Four circuits, two rows of two: square cavities with the corners
            // knocked off, which is what a Micro-Fit 3.0 actually presents to
            // the camera -- round bores read as a domino tile.
            for (cx = [-1, 1], cz = [-1, 1])
                translate([w/2 + cx*2.3, -E, hgt/2 + cz*1.9])
                    rotate([-90, 0, 0]) linear_extrude(3.4)
                        offset(r = 0.35, $fn = 16) offset(r = -0.35)
                            square([3.0, 2.8], center = true);
        }
}
