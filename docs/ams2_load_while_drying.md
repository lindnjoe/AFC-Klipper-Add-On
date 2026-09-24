# Loading a tray while the AMS 2 chamber is drying

A real Bambu printer does this. Our bridge does not, and after a long session
it still does not. This is the state of it: what is established, what has been
ruled out and how, and what is left. It exists so the next attempt starts from
the end of this one.

## The failure, stated as tightly as the evidence allows

With the dry **confirmed running by the unit** (`dry_state == 2`, not merely
commanded) and the bay staged, the AMS accepts our tray select —

```
en:1,mode:4,idx:0,ref:127        (its own status line; appears in successful
en:1,mode:1,idx:N,ref:0           loads too -- see "misreadings" below)
```

— and then emits **nothing at all** for the rest of the load. Not one
`[AMS_SWITCH]` line: no `feed to dw ok`, no stall, no retry. Sixty-plus feed
kicks, every one correctly addressed. The tray eventually blips `sw_sta
0 -> 1 -> 0` and the load gives up.

Heater off, same unit, same bay, same command: **loads in 10 seconds**, with
the unit answering continuously.

It is not failing the feed. It is declining to run the feeder.

## Reproducing it

`tools/dry_load_probe.py` gets the rig to the state and **refuses to run if it
cannot**. That guard is not decoration: five probes in the session that
produced this document were invalid because the dryer was not actually running,
and each was read as a result before the state was checked. Hours went into
interpreting numbers from runs that could not have produced one.

The AMS refuses a dry start with filament out of the bay, so the unload has to
go all the way back — `TOOL_UNLOAD` then `LANE_UNLOAD` — or the dry start is
silently refused and the "test" runs with a cold chamber.

## What is established

* The feed frame we send is **byte-identical** to the printer's. With the spool
  in bay 1 both are `C5/03 0003000000`. The frame is not the difference.
* The unit runs its **own** preload to `sw_sta 1 -> 3` in two seconds, mid-dry,
  immediately after our load gives up. The hardware is capable; the heater does
  not prevent tray motion.
* A printer **does** load mid-dry, twice over, with zero `2C/02` frames in the
  window — it never pauses its own dry to do it.
* The follower `mode:4` and the chamber run concurrently on a real printer
  (`en:1,mode:4,idx:0,ref:127` with `[AMS_CHMB]s:2` either side).
* Only the ORDER is exclusive: dry-then-load is fine; the AMS itself refuses a
  dry start while a lane is loaded out (`err, filament hub load!`), and Bambu's
  own software refuses the same thing.

  **Superseded for the AMS HT, 2026-09-16.** HT firmware `05.00.22.19` heats
  while printing, so on an updated HT that order is no longer exclusive: it
  starts a cycle with a lane at the toolhead, and it loads and unloads with the
  chamber running. The host-side pre-checks that mirrored the old refusal are
  gone (`AFC_BAMBU_HEATER_START` and `_unit_load_lane`). An un-updated HT still
  refuses, and is still caught -- by `last_dry_error` rather than up front.
  The version is readable passively from the unit's own boot narration:
  `[AMS_PMSM_C]get ams_id, N3S05-SN:<serial>, version:05.00.22.19`. Nothing
  here changes for the boxed AMS 2, which is what the rest of this file is
  about.

## Ruled out, with the evidence

| explanation | how it died |
|---|---|
| `C5/04` byte[8] | it is the capacity-measure checkbox, `stamp_capen` owns it, proven both ways on one unit |
| bounded feed bursts / cadence | the deadline is ~25 s and we re-kick at 4 s, so the gap does not exist |
| drying blocks staging | the printer's capture runs tray and feeder motors mid-dry |
| the shared `s_tslot` | a trace of every tray-index write over a load: 67 of 67 named the right slot |
| the AMS retry cycle | that was a finicky PTFE on the rig, not a mechanism |
| dry start dropping the bay | slot stays `present` + `idle` across the start |
| tray lock (`AMS_BDC`) | absent from run 2, which loaded anyway |
| `09/A5` vs `09/7F` | a real deviation, fixed in 1.58, load unchanged |
| the tray index itself | bay 1 (slot 0) fails identically to bay 2 |
| heartbeat len-41 and `C5/A0` | added in 1.60, load unchanged |
| "a drying unit wants a fuller bus" | see below — it makes things actively worse |
| our HT polls (`MC->0x0018`) | muted, load unchanged |

## The fuller-bus idea, and why it cannot work

The printer's bus carries traffic ours does not, dominated by `MC->EXT EE/02`
at **130/s** — over half of all its frames. Sending it looked like the
narrowest remaining form of "mimic the capture". It is not mimicry, it is
starvation, and the A/B is unambiguous:

```
synthesised stream ON    AFC_BAMBU_HEATER_START -> never confirms, 150 s
synthesised stream OFF   the same command, seconds later -> dry_state==2 in 13 s
```

A real printer's bus has several independent talkers — MC, AP2, AHB, the
extruder — each with its own budget. **Our bridge is one talker doing all of it
serially around blocking round-trips.** Reproducing the aggregate density from a
single serial master cannot work, and this is the second time the file has
learned it: a dense poll was once made automatic for a following HT and
"FLOODED the bus and starved the ht_poll_seq feed poll".

It also broke scan-and-measure on real hardware. The three frames stay in the
tree behind `MUTE_EXT_CONV`, **default off**, so a future theory can re-run the
experiment for the price of a command.

## Misreadings worth not repeating

* `idx:0` on `en:1,mode:4,idx:0,ref:127` is the **unit's own status**, not a
  tray we chose. It appears verbatim in loads that succeed. Two separate
  "leads" were this line being read as a command.
* A corpus-wide count of `09/A5` looked like proof that `A5` was normal. Most of
  that corpus is **our own traffic**. Split by who was driving the bus, every
  frame from a real printer loading an AMS 2 carries `7F`.
* Absence of a log line proves nothing until you know where that stream is
  written. `evt:tx` goes to `AFC_BambuAMS.log`, not `AFC.log`.
* A host-side deploy needs a klipper **service** restart. `FIRMWARE_RESTART`
  re-runs the config but the process persists and `sys.modules` keeps the old
  module, so edited extras do not reload. Several "fixes" were tested while not
  actually running.

## Fixed along the way, on their own merits

Five dry gates removed (1.56) · dry-start re-assert removed, the printer sends
**one** frame where we sent twelve (1.57) · `09/A5` → `09/7F` in `PH_ENTER`,
`L2C`, `L2P` (1.58) · the feeder poll `L2P` now names the same tray as its
command, it had shipped a baked tray 0 on every bay (1.59) · the pre-load prime
on the load path.

## THE UNIT SAYS WHY, AND IT IS NOT THE FEED

Found by looking at what the printer does BEFORE the load rather than during
it. Two findings, and the second explains every earlier dead end.

**The printer arms the AMS all through print-start.** Bucketed by 10 s, run 2:

```
bucket     ->AMS 11/04    C5/03 (feed)
22:01:0           35            0      homing / warm-up
22:02:0           33            0      still no feed
22:03:0           14         1165      the load
22:04:0            7         1951
```

`11/04` is the assist/arm, streamed at ~3.5/s long before anything feeds, then
tailing off once the load starts. We send it only after the filament arrives.
Arming by hand mid-dry (`AFC_BAMBU_FOLLOWER ... ENABLE=1`) DOES take -- the unit
answers `en:1,mode:4,idx:0,ref:127` with the chamber hot -- so the arm is not
refused. (`follow_arm_acked` reads False anyway: the host's tracking is wrong,
not the unit.)

**And then the unit states the actual reason:**

```
[AMS_SWITCH]SWITHC_feed ignore. idx_set:255                  (x19, always 255)
[AMS_SWITCH]feed finish 0, mode:1, dw_len:0.000 m, idx_set:255, idx_ref: 0
```

against a load that WORKS:

```
[AMS_SWITCH]feed finish 0, mode:4, dw_len:2.643 m, idx_set:0,   idx_ref: 1
```

TWO INDICES, AND WE HAVE ONLY EVER SET ONE. `idx_ref` is what was asked for and
it arrives correctly. `idx_set` is what the switch controller has ADOPTED, and
while drying it never leaves 255 -- so the feed is discarded, in the unit's own
words, for want of a selected tray.

That is why nothing done to the feed frame ever mattered, including making it
byte-identical to the printer's. The feed was never the problem. The SELECTION
was, and `AMS_LINK`/`AMS_COMMON` accepting `en:1,mode:1,idx:0` is not the same
thing as the switch controller adopting it.

The printer's own load carries `[AMS_SWITCH]AMS_CTRL_switch start` immediately
before `feed tray:0`. Ours carries `AMS_CTRL_state_switch finish` -- a different
message from a different path. What starts the switch controller, and why it
does not start while the chamber is in CTC mode, is the open question.

## What the trigger is NOT, checked since

* **`C5/03 0001FF0000` (mode 01 / tray FF) is not the trigger.** Run 1 sends ten
  of them immediately before the feed and it looked decisive. Run 2 sends
  **zero** and loads anyway. A one-capture pattern is not a mechanism, and this
  is the third time that mistake has been made in this file.
* **`AMS_CTRL_switch start` is ours to trigger.** Our own UNLOAD path produces
  it. So the switch controller is not something only a printer can start --
  though that unload ran with the dry unconfirmed, so it is "works when not
  drying" again rather than a new signal. It needs re-running against a
  confirmed dry.

## Two things worth chasing, neither confirmed

**Our phase machine may sit in `PH_IDLE` for the whole load.** One sample
during a dry-load read `phase=0 idle 01/00` throughout. `PH_IDLE` makes the
state channel send mode 01 with tray **0xFF** -- literally "no tray selected",
which is exactly what `idx_set:255` reports back. `PH_IDLE -> PH_DRIVE` needs
`s_motion != 0`, so if the feed is not setting `s_motion` under a dry, the
state channel never leaves idle and the unit is told there is no tray for the
entire load. THE SAMPLE WAS TAKEN ON AN INVALID RUN (the bay had not staged)
and the clean re-run was blocked by the rig, so this is unconfirmed -- but it
is the first hypothesis that predicts `idx_set:255` rather than merely
accompanying it.

**We emit modes the printer never does.** Our C5/03 repertoire during a failing
load carries `000FFF0000` (mode 0F, the error/park) x16 and `000EFF0000`
(mode 0E, the clear). Neither appears anywhere in either printer capture. A
parked unit would have no selected tray.

## Rig note, for whoever picks this up

Testing repeatedly gets blocked by one state: `LANE_UNLOAD` reports "eject
done" while `loaded_to_hub` stays true and the filament is still at the hub.
The AMS then refuses every dry start (`err, filament hub load!`), so the next
"test" runs with a cold chamber and means nothing. Five invalid probes in this
session were that, and `dry_load_probe.py` exists to refuse rather than report
them. If the bay will not clear, it needs hands.

## PH_IDLE was wrong too, and the chain is now specific

Measured on a clean rig, dry confirmed by the unit, bay staged:

```
ams_phase = 1  "drive 03/00"      dry_state = 2      [AMS_SWITCH] lines: 0
```

We ARE in `PH_DRIVE`, streaming mode 03 with the right tray, and the unit still
ignores it. So the state channel is not sending "no tray" and `PH_IDLE` does not
explain `idx_set:255`. Fourteen explanations down.

What it leaves is a chain where every link is observed:

```
dry running
  -> our mode-09 feeder select never engages the tray   (sw_sta stays 1, never 3)
  -> the switch controller never adopts an index        (idx_set stays 255)
  -> "[AMS_SWITCH]SWITHC_feed ignore. idx_set:255"      (the feed is discarded)
```

and the printer breaks that chain at the first link: run 2 carries
`[AMS_TRAY]tray[0] sw_sta update, 1 -> 3` with `[AMS_CHMB]s:2` telemetry either
side of it. It engages the tray mid-dry. We never do -- `bb_prime`, whose whole
job is that engagement, produced ZERO narration under a confirmed dry.

**The target is now one transition: `sw_sta 1 -> 3` while the chamber is hot.**
Not "make the load work". Everything downstream follows from it, and every
frame-level difference chased so far lives downstream of it, which is why none
of them mattered.

Known to produce `sw_sta 1 -> 3`: the unit's OWN autonomous preload, which does
it in two seconds flat -- but every instance on record is with the chamber cold,
so it has not been shown to work mid-dry either. That is the next thing to
test, and it is cheap: trigger an insert-edge preload with the dry confirmed.

## sw_sta 1 -> 3 DOES happen while hot. The chain above is wrong.

Stated confidently one revision earlier and false. Every `sw_sta` transition in
the log, with the tray-switch ADC and the chamber reading beside it:

```
07:06:57  1->3  u_in 3028  s:2 53.9C
07:15:09  1->3  u_in 3030  s:2 54.3C
08:22:17  1->3  u_in 3021  s:2 54.2C
10:57:13  1->3  u_in 3059  s:2 54.2C
```

The unit engages the tray routinely with the chamber at 54 C, for its own
preload. So "it will not engage while hot" is wrong, and so is the thermal-drift
idea that replaced it -- the chamber is flat at 54 C across all of them.

**What the ADC does show:**

```
the unit's own preload, mid-dry    u_in ~3000       filament at the switch
our failing load (07:11)           u_in  461, 322   nothing at the switch
```

Six times the difference. During our failing loads the filament is genuinely not
at the bay switch -- that is a sensor reading, not a reporting artifact, and it
is not something a frame can fix. The bay also reads empty right now with the
dry running.

So before any more protocol work: **is the spool actually seated at the switch
when these loads run?** Every "the unit ignores our feed" observation is
consistent with there being nothing in the bay to feed, and that possibility has
never been checked at the machine with the dry running.

## Where to start next

The difference is not in any frame we send, and not in the bulk of frames we do
not. That leaves **sequence and timing** — what the printer does *between* the
select and the feed that we do not — which needs the two streams aligned on a
common clock rather than compared as censuses. `tools/capture_diff.py` compares
families and columns; it does not compare ORDER. That is the tool that does not
exist yet.

Worth testing first, because it is cheap and would reframe everything: whether
a **second** AMS 2 on the same bus loads while the first dries. If it does, the
restriction is per-unit and internal, and no amount of bus work will move it.

## THE LOAD OPENS WITH A RELEASE BURST, AND WE HAVE NEVER SENT IT

Found by extracting the log-drain narration out of the two real-printer
load-while-drying captures and then going back to the RAW FRAMES either side of
it, rather than comparing censuses. Both runs open the load identically, on the
drive channel, in the second before the first feed frame:

    ams2_load_during_dry_real_printer       21:49:31  3DC50CC8 03 00 01 FF 00 00  x11
    ams2_load_during_dry_real_printer_run2  22:03:33  3DC50CC8 03 00 01 FF 00 00  x11

then 03/00 drive, x115 and x88. Eleven frames, both runs, and nothing else
changes: the state channel goes `04 00 01 00 03 FF 00` (idle, tray FF) ->
`04 00 03 00 03 00 00` (drive, tray 0) at the same moment. That is the WHOLE
difference between the printer's idle and the printer's load.

The unit answers the pair:

    [AMS_LINK]en:1,mode:0,idx:255,ref:0        the release
    [AMS_LINK]en:1,mode:1,idx:0,ref:0          tray 0 SELECTED
    [AMS_SWITCH]AMS_CTRL_switch start          the switch controller starts
    [AMS_BDC]tray lock:0->1
    [AMS_SWITCH]feed tray:0
    [AMS_TRAY]tray[0] sw_sta update, 1 -> 3    engaged, chamber at 44C

`AMS_CTRL_switch start` is the line our loads have never produced, and
`idx_set:255` — the unit's own stated reason for discarding every feed — is
exactly what a switch controller that was never started reports. Our loads open
straight into `03/tray`, so the FF -> tray edge the adoption keys on never
exists. `mode:1` in that narration is raw mode `0x01` with ref `0x00`; `mode:4,
ref:127` is `0x09/0x7F`, which is what `bb_prime` sends — so what we have been
calling the "prime" the unit reads as *arm the follower*, not *select the tray*.

Recorded as a correction: one revision ago this same burst was written off with
"run 2 sends ZERO and loads anyway". That was counted off the narration, not the
wire. The wire has eleven of them in both runs.

Shipped as `PH_SELECT` in fw 1.63. NOT yet confirmed
on hardware — the bridge dropped off WiFi before the flash.

## And the operator was right about what unstages the bay

"our attempted load is what causes it to be not staged" / "just stopping the
heater makes it show back up". Both true, and the log names the step:

    11:29:54  recover 1/2  ->  [AMS_COMMON]state:0,tray_now:255,tray_exit:1
    11:31:18  recover 2/2  ->  [AMS_COMMON]state:0,tray_now:255,tray_exit:0
    11:32:42  "The AMS moved filament only 50mm during the attempt"

`tray_exit` is the bitmask of bays holding filament (bit 0 = lane12/slot 0,
bit 1 = lane13/slot 1 — an earlier reading of it as a boolean was wrong), and
OUR OWN recovery cleared it. `rehome()` is mode 0F/0E, the park and the clear;
a parked unit has no selected tray and pulls the filament back off the bay
switch. Normally free: the unit notices the bay and runs its own
`[AMS_PRELOAD]`. While drying it does NOT — that preload is what
`AMS_DRY_STATE_UNLOCK` releases, so it runs only once the heater STOPS:

    11:24:20  [AMS_CHMB]state:heating -> finish ... CTC_STATE_OFF
    11:24:22  [AMS_CHMB]AMS_DRY_STATE_UNLOCK
    11:24:23  [AMS_PRELOAD]preload start, feed start, tray:0, sw_sta:1
    11:24:23  tray[0] sw_sta update, 1 -> 3, u_in_out:3028,3277

The printer sends no 0F/0E during a load at all, drying or not. Since 1.63 the
recovery retries the feed without the park while a cycle is running.

Also corrected: `u_in_out` is two ADC channels, not one, and `sw_sta` is their
bitmask — `u_in` high alone = 1 (at the bay), both high = 3 (engaged in the
feeder), neither = 0. Empty reads ~2430, present ~3030, engaged puts `u_out` at
3277. So the `u_in` ~300 readings during failing loads are BELOW the empty
value, not a sixth of the present one, and the previous entry's "the filament is
genuinely not at the bay switch" does not follow from them — the operator
confirmed the filament never moves at all during these attempts.

## The frames now MATCH, and the unit still does nothing

fw 1.64, chamber confirmed at 45C, bay staged, `CHANGE_TOOL LANE=lane12`. What
went out and what came back:

    printer   [AMS_LINK]en:1,mode:0,idx:255,ref:0     release  (03 00 01 FF 00 00)
              [AMS_LINK]en:1,mode:1,idx:0,ref:0       drive    (03 00 03 00 00 00)
              [AMS_COMMON]state:2,tray_now:255,tray_exit:1
              [AMS_SWITCH]AMS_CTRL_switch start       ... and it loads

    ours      [AMS_LINK]en:1,mode:0,idx:255,ref:0     release
              [AMS_LINK]en:1,mode:1,idx:0,ref:0       drive
              (nothing. no state line at all, for the whole 84 s window.)

Two frame bugs were real and are fixed getting here — the missing release burst
(1.63) and `bb_select` sending 09/7F, the follower arm, in place of a select
(1.64). Both are on the wire now, in the printer's order, and the unit answers
them with the printer's own two narration lines. It then does not transition.

So the remaining difference is NOT the content or the order of the frames in
the load itself. The printer's unit reaches `state:2` and ours never leaves its
current state; whatever admits a drying unit to state 2 is upstream of the
select.

## The rotate trigger is not it either

`3C/02` with byte[14] = 0x01, the 312 s pulse documented in
`captures/ams_rotate_trigger.txt`. Run 2's load falls inside a pulse:

    run 2   22:03:28..22:03:36  TRIGGER x10   load at 22:03:34   INSIDE
    run 1   21:50:39..21:50:47  TRIGGER x10   load at 21:49:32   67 s BEFORE it

Run 1 loads with no trigger anywhere near it, and with the wind doors CLOSED
(`wd:0000`, against run 2's `wd:1111`). So neither the trigger nor the doors
gate the load. Sixteen and seventeen.

## Where to start next, revised

`bambu_ams_bridge/docs/ARBITRATION.md` — single-owner select discipline, which
this bridge has never had and which the printer's captures show in every idle
and every transaction. Our bus carries an AMS 2 and an HT; the printer's
grammar parks the non-owner and gives ALL short-dialect traffic to the owner
for the length of a transaction. `[AMS_LINK]ams-80 select ack, req ams-00,
mode:1` is in our own logs, so the units do arbitrate; we simply never drive
it. That is the one structural thing left that is upstream of the select, and
it is already designed in that file.

## SOLVED: it is the AMS's own firmware, and the frames were already right

Settled on hardware, both units on the same bus, same bridge build, minutes
apart. The updated AMS 2 -- taken off a real Bambu printer that had updated it
-- loaded a tray with its chamber heating, no workaround, using exactly what
fw 1.64 sends:

    [AMS_CHMB]s:2|...|ap:45.5|...|t:380          heater on
    [AMS_LINK]en:1,mode:0,idx:255,ref:0           the release burst (1.63)
    [AMS_COMMON]en:1,mode:1,idx:0,ref:0           the drive
    [AMS_SWITCH]AMS_CTRL_switch start
    [AMS_BDC]tray lock:0->1
    [AMS_TRAY]tray[0] sw_sta update, 1 -> 3       ENGAGED, chamber hot
    [AMS_SWITCH]feed to dw ok, len_det:0.050 m
    lane16 reached the toolhead sensor after 4 feed kick(s)
    [AMS_COMMON]state:4,tray_now:0,tray_exit:3
    lane16 is now loaded in toolhead t:79.129
    [AMS_CHMB]s:2|...|ap:43.5|...|t:420          still heating

`sw_sta 1 -> 3` while hot: the transition the older unit does ZERO times in
22 MB of narration. The cycle then kept running with the lane loaded, which the
older unit also refuses.

The generation split is visible on our own wire, no inference:

    updated   [AMS_LINK]ams-0x01 select ack, req ams-0x01, mode:1
              [AMS_CHMB]s:2|rf:55,0|vt:25.3,25|ap:29.9|hts:41,24,00|...|t:30
    older     [AMS_LINK]ams0 select, req ams0
              [AMS_CHMB]s:2, rf:55, cd:55, vt:48.0,49, ap:52.0, ht:26,44, ...

Same bus, same second, different message formats. The pipe form is the one in
every real-printer load-while-drying capture we hold.

## What was kept, and what was deleted

KEPT, because the printer's own wire justifies them independently of any of
this -- and the successful load above ran on them:

  * fw 1.63 PH_SELECT, the `03 00 01 FF 00 00` x11 release burst that opens
    every load in both real-printer captures. We opened straight into
    `03/tray` and never presented the FF -> tray edge.
  * fw 1.64 `bb_select`, which was sending L2C -- op-03 mode 0x09 ref 0x7F,
    which the unit narrates as mode 4, the FOLLOWER ARM. Every load had opened
    by telling the unit to hold a tray it had not selected.
  * Skipping the 0F/0E re-home while a unit dries. A park drops the tray
    selection and pulls filament off the bay switch, and a drying unit will not
    run the preload that re-seats it, which is the operator's "the load
    unstaged my spool and stopping the heater brings it back".

DELETED: the `dry_pause_for_load` workaround (stop the dry, load, restart it)
and its `AFC_BAMBU_DRYPAUSE` toggle. It did produce the first mid-dry load on
the older unit, but it cannot be made to work: that unit will not START a dry
while a lane is loaded either -- it zeroes the command rather than refusing it
(`pw_lim:0, dur:0, tmpr:0` against a good start's `pw_lim:100, dur:480,
tmpr:45`) -- so the cycle cannot come back until the print ends. Carrying a
held cycle to the unload was working around a limitation that the firmware
update simply removes.

## The remaining path for un-updated units

Put them on a Bambu printer and let it update them. The open question is
whether that update can be captured off the wire and replayed from our own
bridge; see `bambu_ams_bridge/docs/AMS_FW_UPDATE.md`.
