// The BridgeBox mark, as 2D geometry for engraving into the lid.
//
// The outlines live in bridgebox_mark.scad, which trace_logo.py generates from
// the artwork in docs/img/. Regenerate both together:
//
//     python3 trace_logo.py
//
// WHY A TRACE AND NOT DRAWN PRIMITIVES. An earlier version of this file drew
// the mark freehand -- rounded boxes, a nozzle, text() for the wordmark -- and
// it printed well but it was an interpretation, not the logo. The wordmark in
// particular is a specific typeface no installed font matches. Tracing the real
// artwork keeps the letterforms and the cable's S-curve.
//
// WHY POLYGONS AND NOT AN SVG. OpenSCAD 2021.01 imports SVG but ignores
// fill-rule, so an even-odd path came back with its holes filled and the whole
// mark inverted inside a frame. The tracer resolves holes itself, where the
// nesting is known, and emits plain polygons.
//
// ORIENTATION. The tracer flips y on the way out (image y runs down, OpenSCAD's
// runs up), so the geometry arrives the right way up. Do NOT add a mirror here
// -- a mirror on top of that flip is what put the wordmark upside down and
// backwards.
//
// WHAT THE TRACE COSTS. The artwork is line art at a weight meant for a screen.
// At the 45 mm the lid panel allows, its strokes land near what a 0.4 mm nozzle
// can hold, so trace_logo.py closes the bevel hairlines first -- see
// CLOSE_RADIUS there. Do not scale this much below 45 mm without re-checking
// that; the mark thins with it.

use <bridgebox_mark.scad>

// Height / width of the traced mark. REWRITTEN BY trace_logo.py -- edit the
// artwork or the tracer, not this line. check_case.py reads it to size the
// keep-out it verifies on the lid.
LOGO_HF = 3020/6696;

//: The traced artwork's pixel dimensions, which set the units the polygons are
//: in. Also rewritten by the tracer.
LOGO_SRC_W = 6696;
LOGO_SRC_H = 3020;

// The mark, `w` wide, centred on the origin.
//
// The traced geometry sits in pixel units with its origin at a corner, so it is
// moved to its own centre before being resized to millimetres.
//
// `bolden` EXISTS BUT IS NOT NEEDED, and the reason it is still here is worth
// knowing before anyone turns it back on.
//
// The first 84 x 58 lid off the printer came out with most of the letters and
// the nozzle solid, and this comment used to explain that as a groove-width
// problem: measured off the traced polygons at the 65 mm that lid uses, the
// median engraved groove is 0.71 mm -- wider than one 0.42 mm extrusion and
// narrower than two -- so the slicer could lay neither one bead nor two clean
// walls, gap-filled instead, and the gap fill closed a recess this shallow.
// Boldening every stroke to 1.00 mm was the fix that followed from it.
//
// That explanation was wrong. The same STL sliced in Bambu Studio prints the
// mark open, letters and nozzle both; only OrcaSlicer fills them. It is a
// slicer bug, not a geometry problem, and the model needs no correction for it
// -- so `bolden` defaults to 0 and case.scad passes 0.
//
// The measurement itself still stands and is why this is worth writing down:
// 0.71 mm IS an awkward groove, and a future slicer or a wider nozzle could
// make trouble with it again. If that happens, bolden is the lever -- it moves
// every stroke past two full extrusions, and offset() with a radius rounds
// interior corners on the way, which a nozzle has to do anyway. Check the
// slicer first.
//
// The offset is applied AFTER the resize so it means millimetres on the part
// rather than pixels in the artwork, and one value works at both lid sizes.
//
// :param w: overall width in mm
// :param bolden: extra half-width on every stroke, mm
module bridgebox_logo(w = 45, bolden = 0) {
    offset(r = bolden, $fn = 16)
        resize([w, 0], auto = true)
            translate([-LOGO_SRC_W / 2, -LOGO_SRC_H / 2])
                bridgebox_mark_raw();
}
