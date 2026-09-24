# The BridgeBox page

`bridgebox.html` is the buyer-facing explainer — what BridgeBox does, what you
need, and what it doesn't do. It is a single file: the logo and the case render
are embedded as data URIs, so it opens from disk with nothing alongside it.

**It states no price, no availability and no contents.** The footer says
"Available soon" and nothing more. Add those before sending the link anywhere.

## The two pictures

**The case** is a real render of `case.scad`, not a drawing. Regenerate it from
`hardware/bridgebox_carrier/case/hero.scad`:

```sh
cd hardware/bridgebox_carrier/case
xvfb-run -a openscad -o /tmp/case.png --imgsize=1600,1200 \
    --colorscheme=Cornfield --camera=34,28,12,57,0,0,215 hero.scad
```

Two things about that command are not preference:

- **`rz` must be 0.** OpenSCAD's own default three-quarter view (`55,0,25`)
  renders the engraved mark upside down. Rendering a contact sheet at 0 / 90 /
  180 / 270 is the quick way to re-find it if the model's orientation ever
  changes.
- **`Cornfield`, not a dark scheme.** The background gets keyed out to
  transparency afterwards, and the case's own shadowed faces sit within a few
  values of the dark schemes' background — a tolerance key eats the case. Flood
  the transparency inward from the edges so it cannot reach an interior shadow.

`docs/img/bridgebox_case_hero.webp` is that render, keyed and cropped, and is
what the page embeds.

**The panel** is inline SVG, drawn from `bridgebox_display/main/ui.c` rather
than from a screenshot. It is an illustration and its caption says so. If the
panel's layout changes, this drawing goes stale silently — it is not generated.

## Print and offline

The page loads its three faces from Google Fonts. For a PDF or a copy that has
to work with no network, inline the latin subsets as `@font-face` data URIs and
add `<!doctype html>` with `<meta charset="utf-8">` — **without the charset a
browser reads the file as latin-1 and every em dash becomes three characters of
noise.** The artifact runtime supplies both, which is why the hosted page never
shows it.

For print, force `print-color-adjust: exact` or the dark hero, the banded
sections and the whole panel figure come out as holes. Put `break-inside:
avoid` on the individual pieces — a card, a figure, a table row — and **not** on
their containers: `.panel-wrap` is three paragraphs plus the panel picture, too
tall to fit in the tail of a page, so avoiding a break inside it pushes the lot
onto the next sheet and strands its heading over half a page of nothing.
