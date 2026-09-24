# AMS 2 Pro — what the unit says while it dries

Everything here is measured on the rig, on the dates given. Where the answer
is "no", the negative result is recorded with the same care as the positive
one: each of them cost a test that nobody should have to repeat.

The AMS 2 Pro narrates in plain text, drained off the bus by the 1A/02 log
poll. `[AMS_CHMB]` is the chamber controller — the dryer.

## The telemetry line, every ~10 s while a cycle runs

Two units on one bus, at the same second, one of each punctuation style the
firmware emits (the separators differ between builds; both are parsed):

```
[AMS_CHMB]s:2, rf:55, cd:55, vt:39.2,41, ap:43.2, ht:36,33,
          pw:32,92,95,30,35,29,34, ad:1,0.5  wd:0,0,0,0, fa:92,86
[AMS_CHMB]s:2|rf:55,0|vt:44.5,46|ap:50.4|hts:56,33,00|
          pw:72,56;32;29,34;29,34|ad:4,23.8|wd:0000|fa:99,96|t:610
```

| field | what it is |
|---|---|
| `s:` | chamber state (2 = heating) |
| `rf:` | target °C |
| `cd:` | cool-down setpoint |
| `vt:` | chamber temperature |
| `ap:` | ambient |
| `ht:` / `hts:` | heatsink temps; the FIRST value is humidity |
| `pw:` | **duty percentages**, not amps |
| `ad:` | the power supply — see below |
| `wd:` | vent door positions |
| `fa:` | fan RPM |
| `t:` | seconds into the cycle |

**There is no current in this line.** `pw:95` says the element is at 95 %
duty, which tells you nothing about amps, because the amps depend on the rail
driving it — the thing one actually wants to know. The only amps the unit ever
utters are in the self-check burst below, and they are door-motor amps.

The line exists **only while a cycle is running**. There is no idle source for
any of it, including `ad:`. That single fact shapes every design below.

## `ad:` is the 24 V jack

Measured 2026-09-13 on one unit, adapter switched in and out under an
otherwise identical cycle:

```
adapter IN    ad:4,23.8      23.6 .. 23.9   over  122 samples
adapter OUT   ad:1,0.5        0.5 ..  0.6   over 1490 samples
```

Two populations three orders of magnitude apart, and in the whole log never a
low voltage under `ad:4` nor a high one under `ad:1`. The AMS HT reports a
single value, `ad:2`.

This needed measuring because the doubt was real: 23.9 is also a perfectly
plausible room temperature sitting among `vt`, `ap` and `ht`, which are all
temperatures. 0.5 is not a plausible anything except an unplugged jack.

The first number tracks the second perfectly across every sample taken, so it
is probably a supply-source code — but it is redundant, and the voltage is the
number with physical meaning, so the interlock gates on the voltage.

## Why any of this matters: two dryers trip the supply

An AMS 2 Pro heats off the bus wire's 24 V with no adapter plugged in, and one
of them is within budget. A second one is not. Measured 2026-09-13 with two
bus-powered units:

* the supply collapses about **3 s** after the second heater engages
* every unit on the wire re-runs its power-up self-check
* whatever the bridge was doing at the time is lost

Nothing on the bus refuses it, because it is a power fault and not a protocol
one. Two tests were first misread as "no trip in 25 s" — they *were* trips, and
what gave them away was `"busy (state 10)"` and `"AMS finished its power-up
self-check"` in the log, not any error on the command.

The mixed pair — one on the bus, one on its own adapter, both heating — was run
for 156 s with zero reset symptoms and both chambers climbing. That is the
configuration the interlock permits.

## The interlock

`AFC_BambuAMS._bus_supply_conflict`, in front of the send:

> a start is refused only while ANOTHER AMS 2 on this bus is drying AND
> reporting that it is running off the bus.

That is the whole rule, and it gives the four cases for free. Stop the
bus-powered unit and its telemetry stops, so the next start is allowed. Plug an
adapter into it **mid-cycle** and its next line reads 23.8, so the next start is
allowed without stopping anything. A unit on its own adapter never blocks
anything. A unit that is drying but has not yet said how it is powered blocks —
refusing costs ten seconds, guessing costs the bus.

The unit being *started* is never consulted, because nothing can be known about
an idle unit's jack. Only units already heating constrain it, which is exactly
what makes the question answerable before the frame goes out.

### It scales to a full bus without counting anything

Bambu allows four boxed units, and the rule holds at four for the same reason
it holds at two: whether a start is safe depends only on whether the **one** bus
allowance is already spent, and every unit that has spent it is by definition
heating and therefore narrating.

| already drying | starting a fourth |
|---|---|
| three on adapters | **allowed** — the bus allowance is unspent |
| two on adapters, one on the bus | **refused**, naming the bus one |
| two on adapters, one silent | **refused** — the quiet one could be either |
| all four on adapters | every one of them may start while the others run |

No census of the configured units is taken and none is needed — which matters,
because a census would have to decide what an unclaimed bay, or a unit that has
never spoken, counts as. The interlock does not cap how many units may dry. It
caps how many may dry **off the bus**, and that distinction is the whole reason
for reading the jack.

Two starts issued inside one ~10 s reporting interval cannot both get through:
the first sets its drying flag immediately and has no reading yet, so the second
is refused on the unknown rather than waved through on the silence. That is what
stands between a scripted "dry everything" macro and a dead bus.

### The flag is not the evidence

A unit counts as heating if it says so **or** if it is narrating chamber
telemetry past its own last start/stop. The host's `_drying` flag alone is not
enough: it is adopted from telemetry inside `get_status`, so a cycle the
*printer* started — or one already running when Klipper came up — reads False
until something polls that unit. A panel polls every unit constantly and hides
this; a headless host, or a fleet where the operator has the fourth unit's card
closed, does not. The heater draws either way.

The stop grace is honoured on the other side of that: a stopping AMS emits
another line or two while it winds down, and those must not read as a running
cycle or stopping a unit would block the next start for the length of the grace.

`dry_bus_interlock: False` in the unit's config turns it off; `FORCE=1` on a
single `AFC_BAMBU_HEATER_START` overrides it once. `AFC_BambuAMS.dry_blocked`
publishes the refusal so a panel can grey the button rather than let the
operator discover the rule by pressing it.

## Three things that do NOT work, and why

**The idle 0x3C frame does not carry the jack voltage.** Whole-frame captures
taken with the adapter in and out differ in exactly one place — bytes 23–26 of
the 69-byte reply, `16 16 16 16` plugged against `17 17 17 17` unplugged. That
is 22 °C against 23 °C: a temperature quad drifting by a degree over the
minutes between the two captures, not a rail.

**The self-check currents do not discriminate.** The burst at t ≈ 1.5–5.1 s
after a start looks promising and is not:

```
[AMS_DOOR]state[0]:0 -> 1, pwm:0.57, i:0.75 A
[AMS_CHMB]wind_door[0] res ok, i_0:3, i_sum:130, i_avr:653, cnt:200, res:7656
[AMS_CHMB]PTC[0] ok! i_0:3, res:8019
```

`i:0.75 A` is the **vent door motor**, which runs off a regulated rail and
reads the same either way. `res` on the PTC lines swings from 7188 to 15950 —
but it tracks chamber temperature, not supply: hold the power state constant
and repeat the cycle and the number keeps moving. It is a thermistor.

**Start-then-verify is too slow.** The obvious fallback is to start the second
unit, read its `ad:` and stop it if it is on the bus. The bus resets ~3 s after
the second heater engages; the first telemetry line arrives ~5 s after that,
and with a ~10 s cadence the true exposure is 9–19 s depending on where the
start falls in the cycle. The window is on the wrong side of the failure, which
is why the interlock asks about units that are *already* heating instead.
