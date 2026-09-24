# Bambu AMS hot-swap

How `AFC_BridgeBox` (the master) and the pooled `AFC_BambuAMS` units drop and
restore lanes and `T#` macros live when a Bambu AMS is unplugged or plugged
back in — no restart.

## The pool model

At boot the master fabricates the entire bus ceiling up front: every configured
slot becomes an inert `AFC_BambuAMS` unit object, with all of its lanes flagged
`unassigned` and parked in a pool. Nothing is registered to AFC yet — no lanes,
no `T#` macros. The known units (`Bambu_AMS_1`, `Bambu_AMS_2`,
`Bambu_AMS_HT_1`, …) each own a reserved slot; any extra generic spares fill out
the remaining pool capacity (`pool_ams` / `pool_ht`).

This up-front fabrication is what makes hot-swap restart-free: the unit object
already exists, so a plug/unplug only flips it between *pooled* and *claimed*
rather than creating or destroying Klipper objects.

## The online flag

Each unit carries a per-unit **online flag** that the bridge flips within about
a second of a physical plug/unplug. The master's watch tick reads these flags
and drives two transitions:

- **Claim (plug in)** — a UID that is `online` but not yet `bound` is claimed
  onto its slot once it has held online past `claim_grace`. `_claim_pool_unit`
  registers the lanes, pins each lane's home tool to `T<lane#>`, and brings the
  unit live. No restart.
- **Release (unplug)** — a `bound` UID whose online flag stays false for
  `release_grace` (default 10s) drops its lanes and `T#`, but the slot is
  **held** for that unit's return (its UID stays on the slot). Survivors are
  never touched. A re-plug inside the window is simply seen online again (its
  clock resets) and nothing happens.

## Layout — fixed bands

The pool lays out in **fixed bands keyed off the declared pool sizes**, so a
unit's lanes (and therefore its tools) never move when other units come and go.
The 4-slot AMS pool always **reserves `pool_ams × 4` lanes** from `lane_base` —
whether or not that many AMS are present — and the HT band starts immediately
above it at `lane_base + pool_ams × 4`, one lane per HT. Each lane is pinned to
`T<lane#>`.

A unit's place in its band is its **rank** — the index behind its name, so
`Bambu_AMS_HT_1` is HT rank 0, always. Position is therefore a function of the
name and the **declared** `pool_ams`/`pool_ht`, never of how many units happen to
be online. With `lane_base` 12 and `pool_ams` 4:

| Unit | Lanes | Tools |
|------|-------|-------|
| AMS rank 0 | lane12–15 | `T12`–`T15` |
| AMS rank 1 | lane16–19 | `T16`–`T19` |
| AMS rank 2 / 3 (spare) | lane20–23 / 24–27 | … |
| HT rank 0  | **lane28** | **`T28`** |
| HT rank 1 … | lane29 … | `T29` … |

The HT band base is **fixed at `lane_base + pool_ams × 4` = 28** here whether one
AMS or all four are online — it never **collapses** toward the low lanes on a
restart with fewer AMS present (the bug this layout replaced). Set `pool_ams` to
2 and the whole HT band slides down to start at lane20; that declared count is
the one knob that moves it. `FORGET` frees a name/rank for the next unit to reuse.

> If you ever run **more** real AMS than the declared `pool_ams`, the AMS band
> widens to fit them and the HT band moves up to stay clear — it never overlaps.
> Keep `pool_ams` at your true maximum (≤ 4, HT ≤ 8) and the HT band stays put.

## Reserved slots — hot-swap keeps its name and T#

Each unit keeps its **own reserved lanes and name** (pinned per UID). That is
what makes hot-swap seamless, with no reboot:

- **Pull a unit** → its lanes/`T#` drop live, but they stay **reserved** under
  its name (UID held). Survivors are untouched.
- **Plug it back** → it reclaims the **same lanes, name, and `T#`** live.
- A **brand-new** unit takes the **lowest free rank** in its family's band (and
  the matching lane block) and the **lowest unheld name** of its type.

Lanes AND names are pinned per UID, so a unit coming or going **never renumbers
or renames a survivor**, and a reboot brings everyone back on the same
lanes/`T#`/name.

### Naming

- A unit is named the **lowest unheld name of its type** the first time it is
  seen, and **keeps that name** for good — it is the unit's identity (macros,
  Spoolman bindings and learned values hang off it). A tombstone's name stays
  held for its return; only `FORGET` releases a name, and then the next unit
  reuses it.
- The generation is deliberately NOT in the name — every 4-lane unit (`boxed`,
  `ams1`, `ams2`) is one `ams` type, so the ams1/ams2 verdict never forces a
  rename; it only steers `has_heater` and the dry ceiling.
- **Default names** are `Bambu_AMS_#` (AMS) and `Bambu_AMS_HT_#` (HT). Spares use
  the same scheme, so a first-plugged unit lands on a **real name live**.

### Naming units — `ams_names` / `ht_names`

Give units your own names in config, by enrollment order:

```ini
[AFC_BridgeBox chain1]
ams_names: PLA_Station, PETG_Station, Support, Spare_AMS
ht_names:  Dryer_A, Dryer_B
```

`ams_names[0]` names the first AMS to be enrolled, `ams_names[1]` the next, and
so on; `ht_names` does the same for HTs. Anything past the end of a list falls
back to the `Bambu_AMS_#` / `Bambu_AMS_HT_#` default. Because a unit **keeps**
its name, editing a list names the **next new** unit — an existing one is not
retroactively renamed (that would break its references). To re-badge one,
`FORGET` it and let it re-enroll, or `AFC_BRIDGEBOX_ASSIGN` it.

By default a fresh unit auto-claims the **lowest free block** and its next name
— it always gets a real home, so there is never a stuck/unassigned state. To
pin a *specific* unit to a *specific* named bay:

- `AFC_BRIDGEBOX_ASSIGN UID=<uid> NAME=<bay>` — bind that uid to that bay, live
  (it moves there at once if online) and persisted across reboots. Same-family
  only (an HT bay is one lane, an AMS bay four). Refuses an occupied bay.
- `AFC_BRIDGEBOX_ASSIGN UID=<uid>` (no `NAME`) — pop the bay-**picker** for that
  unit so you can click a destination.
- `AFC_BRIDGEBOX_UNASSIGN UID=<uid>` (or `NAME=<bay>`) — unlink the pin; the unit
  reverts to a floating spare. Unlike `FORGET`, it **keeps** the learned values.
  Refuses an online unit unless `FORCE=1`.
- `AFC_BRIDGEBOX_BAYS` — pop the **bay manager**: every bay, its occupant, and an
  Unassign button per occupied bay.

> **Two UIDs, kept separate.** The *unit UID* is the AMS unit's hardware ID —
> what claims a slot. The *spool UID* is the RFID tag in each lane — what
> Spoolman binds. A lane's spool identity rides on its **RFID tag, not its lane
> number**: `deactivate_to_pool` clears the lane's spool on release and a claim
> re-reads the tag.

Slot reuse is **family-scoped and never crosses**: an HT can only land on a free
HT slot and an AMS/boxed only on a free AMS slot (different lane counts).

## Retiring a unit — `AFC_BRIDGEBOX_FORGET`

Normal removal keeps a unit's slot **reserved** for its return. When a unit is
gone for good, `AFC_BRIDGEBOX_FORGET UID=<uid>` (or `NAME=<unit name>`) retires
it: it erases the unit's name and learned values, and frees its slot to the pool
**LIVE** — so the next same-family hot-swapped unit claims those lanes/`T#`
immediately, no reboot. The freed slot keeps its object name, so a same-type
replacement assumes it; a cross-generation replacement (an ams2 onto a freed
ams1 slot) wears that name until the next restart regularises it — still a real
name, never a pool one.

**Forgetting a unit that is still plugged in** works too — it drops the unit's
lanes/`T#` right then, and holds its UID on a **suppress list** so the scout does
not simply re-enroll the hardware you just cleared. The hold lasts only while the
unit stays on the wire: **physically pull it and the hold clears**, so a genuine
re-plug enrolls it fresh (lowest free block, its next name) — that is how you add
a suppressed unit back without a restart. You can also put it straight back with
`AFC_BRIDGEBOX_ASSIGN UID=<uid> NAME=<bay>`, which lifts the hold as it re-pins the
unit. The one case still refused (without `FORCE=1`) is forgetting an **online**
unit **mid-print** — yanking a live lane out from under the job would disrupt it,
the same reason auto-drop is gated during printing.

## Popups

When **not printing**, a plug/unplug raises a Mainsail/Fluidd dialog (Klipper's
`action:prompt` protocol — no panel change needed):

- **New unit added** → *"New AMS on `<bay>`"*. It has already claimed the lowest
  free bay of its family (a real, live home), and the dialog only **offers** to
  move it to one of your other free named bays. Dismiss keeps it where it is —
  cancelling never leaves it unassigned, because the claim already placed it.
- **Unit unplugged** → *"AMS removed: `<bay>`"*. Its bay is held for a re-plug
  (nothing to do — Dismiss). A **Forget** button frees the bay for good.

Several units plugged in at once are **queued** and shown one at a time
(deduped by uid), so none is clobbered by the next dialog. During a print no
popup fires and auto-drop is deferred, but a new unit is still claimed onto the
first free bay of its family. Every popup is also reachable on demand:
`AFC_BRIDGEBOX_ASSIGN UID=<uid>` (picker) and `AFC_BRIDGEBOX_BAYS` (manager).

## Anti-flap

`claim_grace` is short normally, but a unit released within `flap_window`
(default 120s) must hold online for the longer `flap_claim_grace` (default 15s)
before it can re-claim, so a flapping cable can't thrash the pool. The watch
tick also polls adaptively — roughly 3s while settling, a 5s heartbeat while any
unit is bound, 6s while a slot awaits a re-plug, relaxing to 30s when everything
is claimed and online — so brief offline stretches are always caught even if a
sparse sample reads online.

The **release** side is symmetric. A physically-absent unit can leave a
*phantom* online flag that flaps true every second or two — a pulled AMS whose
slot still blips online on an otherwise HT-only chain. A single blip must not
reset the drop clock, or the unit's lanes never release. So an online read only
cancels a pending drop once the unit has held **continuously** online for
`release_settle` (default 5s), and the drop only fires on a tick the unit is
actually offline — a genuine re-plug reading solidly online is spared, while a
flag that spends most ticks offline still drops after `release_grace`.

## Follower restore

On boot, if AFC's own records show a lane loaded to the toolhead, the owning
unit re-engages that lane's follower once (mode:4, one-shot) at about +8s. It is
a single deferred engage with no keep-alive timer, so it restores the loaded
lane's follower without flooding the shared bus.

## The print gate

**Auto-drop is gated during printing.** In `AFC_BridgeBox._is_printing()`, the
release path only fires when `auto_drop` is enabled *and* no print is active. A
print is considered active from either `print_stats` (`printing` or `paused`) or
`idle_timeout` (`Printing`), so a pause does not sneak a drop through.

Only the **drop** half is gated, not the **claim** half:

- A **pull during a print** is left alone — the unit stays claimed, lanes
  intact, follower undisturbed. The drop is deferred until the print ends, then
  the release clock runs and it drops normally. Yanking a lane out from under an
  active follower/feed mid-print would disrupt the print, so we never do it.
- A **re-plug during a print** is still allowed to claim — adding lanes and
  `T#` mid-print is safe and non-disruptive.

## Relevant config knobs

Set on the `[AFC_BridgeBox ...]` section:

| Knob | Default | Effect |
|------|---------|--------|
| `auto_drop` | `True` | Enable live release on unplug (gated during printing). |
| `release_grace` | `10` | Seconds a bound unit must stay offline before it drops. |
| `release_settle` | `5` | Seconds a bound unit must hold *continuously* online before an online read cancels a pending drop — so a physically-absent unit whose online flag phantom-flaps still drops. |
| `claim_grace` | `0` | Seconds an online unit must hold before it claims (normal case). |
| `flap_claim_grace` | `15` | Longer claim hold required if the unit was released within `flap_window`. |
| `flap_window` | `120` | Window after a release during which `flap_claim_grace` applies. |
| `pool_ams` | — | Generic AMS spare slots fabricated in the pool. |
| `pool_ht` | — | Generic HT spare slots fabricated in the pool. |
| `ams_names` | — | Comma list naming the AMS bays, lowest position first. |
| `ht_names` | — | Comma list naming the HT bays, lowest position first. |
