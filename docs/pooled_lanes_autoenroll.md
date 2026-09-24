# Pooled lanes / live Bambu hot-swap

A Bambu AMS unit can be unplugged and re-plugged while Klipper runs: its lanes,
`T#` macros, and panel/dryer presence drop on removal and come back on
re-insert, with **no restart**. This works by *claiming* pre-provisioned
"unassigned" lane objects rather than creating new ones.

Why a pool at all: Klipper freezes its object graph at config parse — a lane,
its `T#` `gcode_macro`, and its status registrant are all config-time objects
with no supported API to add one to a running klippy. Bambu lanes are
**bus-fed and pinless** (the AMS motors are driven over serial by the
firmware, not by MCU stepper pins), which is the property that makes a
hardware-agnostic pool feasible here where it would not be for a pin-bound AMS.

## How it works

At boot `AFC_BridgeBox` fabricates the whole bus ceiling as **inert pool
units** — `pool_ams` four-slot AMS units and `pool_ht` single-slot HT units
(defaults 4 + 8 = 12 units), sized to the ceiling the firmware enforces. Known
units (from the per-uid ledger) fill their named slots; the rest are generic
spares. Every pool unit and its lanes start `unassigned`: registered in no
registry, so invisible to the panel, `save_vars`, the dryer, and `T#`
assignment.

The chain-watch tick then drives membership off the firmware's **per-unit
online flag**:

- **Claim** — a unit that holds continuously online binds to a free slot live:
  its lanes register (`AFC_lane.activate_from_pool`), it joins the bridge
  (`AFC_BambuAMS.claim`), and each lane takes its **home tool** `T<lane#>` so
  the tools never depend on claim order. Normally instant; for `flap_window`
  seconds after a release it must hold online for `flap_claim_grace` first, so
  a marginal/flapping link cannot re-claim on a blip.
- **Release** — a claimed unit whose online flag stays false for
  `release_grace` is dropped live (`deactivate_to_pool` + `AFC_BambuAMS.release`):
  its lanes and `T#` macros are unregistered and the unit returns to the pool.
  Survivors are never touched — no relink, no deregister sweep, no cascade.

A restart regularizes any claimed spare's UID into the roster under a proper
family name and frees the generic slot again.

## Config knobs (`[AFC_BridgeBox <name>]`)

| Option | Default | Meaning |
|---|---|---|
| `pool_ams` | 4 | four-slot AMS pool units fabricated at boot |
| `pool_ht` | 8 | single-slot HT pool units fabricated at boot |
| `auto_drop` | False | drop a unit's lanes live when it goes offline |
| `release_grace` | 10 | seconds offline before a claimed unit is dropped |
| `claim_grace` | 0 | seconds online before a claim (normally instant) |
| `flap_claim_grace` | 15 | required online hold to reclaim within `flap_window` of a release |
| `flap_window` | 120 | seconds after a release that the higher reclaim bar applies |

Auto-drop is print-safe: `_is_printing()` blocks it while a print is active or
paused, so a pull mid-print leaves the unit claimed until the print ends.

## The inert invariant

An unassigned lane presents as inert until claimed: `map == []`, no `spool_id`,
no material/color/weight, `load_state == False`, a member of no `unit.lanes` /
`afc.lanes` / hub / extruder / buffer list, and `lane.unassigned == True` as the
single source of truth every filter checks. Holding that state makes most of
AFC's operational sweeps natural no-ops.

## Footprint on shared AFC files

The feature lives in the two Bambu-native modules (`AFC_BridgeBox.py`,
`AFC_BambuAMS.py` + bridge) plus a few surgical edits to shared files:

- `AFC_lane.py` — the `unassigned` flag, the registration gate, and
  `activate_from_pool` / `deactivate_to_pool`.
- `AFC_functions.py` — idempotent `register_tool_macro` so a live re-claim
  does not raise "already setup".
- `AFC_prep.py` — skip the boot banner for inert (pool-flag) units.
- `afc_dryer.py` — hide inert pool units from the panel.

## Deploy

This branch (`claude/pp-next-auto`) deploys exactly like `pp-next`:

    RUN_SHELL_COMMAND CMD=bridge_update PARAMS=claude/pp-next-auto

`claude/pp-next` stays the known-good fallback. The standalone variant is
mirrored on `claude/u1-standalone-auto`.
