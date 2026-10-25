# Bassline controls redesign: the loop, four pads, two dials

Status: **model implemented and witnessed on the host** (`main/control_model.{h,cpp}`, `tools/control_model_witness.cpp`,
54 checks). **Not yet wired into the firmware** (`state_machine.cpp`, `bass_engine.cpp`). Wiring needs a device run,
and that is an image change on the mesh board, so it follows the same protocol as the mesh work: CI build, console,
soak, then an explicit go-ahead.

## 1. What the user asked for, in one place

- The bassline is an **8-bar loop**. Every recording lands on that grid (8 bars of 16 sixteenths).
- The **two dials** control the loop on the fly.
- The **four pads** each have a single-tap mode and a double-tap mode. A double tap, while held, changes what the
  dials do.
- **Recording** behaves like the KO2 slider: a setting touched during the loop is recorded and replayed, and a
  setting not touched reverts at the turnaround. Persistence is defined in section 3.

## 2. The control map

Pads are indexed 0 to 3 as in `state_machine.cpp`. Dial A is the first pot, dial B the second.

| Pad | Gesture | Dial A | Dial B |
|---|---|---|---|
| 0 | single | **scale**, friendly list only | **width**: held pitch, through arp patterns, higher and lower |
| 0 | double (held) | **gate width**, 0 to 100% | **arp speed**: half, quarter, double, triplet, quad |
| 1 | single | **pitch offset**, -1 to +1 octave along the scale | **rhythm**: one of 32 step patterns |
| 1 | double (held) | **random octave up/down** likelihood, rises with intensity | **random note drop-out**, rises with intensity |
| 2 | single | **LFO type** (mod wheel, CC 1) | **LFO speed** |
| 2 | double (held) | **LFO width** | **filter shift**, -100% to +100% (clamp the filter) |
| 3 | single | **trance gate**: one of 32 series, repetitive to random | **octave**, -3 to +3 |
| 3 | double (held) | **loop window**: shorten or expand the loop | **chromatic aberration**, rising to all random |

The pad order is the one the state machine already uses.

## 3. Loop recording, the rule

Each dial-controlled setting (one dial in one mode) owns a `LoopTrack` with 128 steps:

- **While touched**, each step's live value is written to the track at the playhead.
- **At the turnaround**, a setting still touched becomes **persistent**. Its track is kept and replayed on every later
  pass, and touching it again overwrites the positions it passes.
- **At the turnaround**, a setting not touched and not persistent **reverts** to its baseline (the value at the start
  of the loop). Nothing is kept.
- A **persistent** track that is not touched keeps replaying until it is **cleared**.

These are the rules that `LoopTrack` implements and the witness checks. The request said "unless our adjustment period
is still underway", and I read that as "touched at the turnaround". Section 6 lists this as a decision.

## 4. Defaults where the request was open

These are the choices I made. Each one is a single value that is easy to change.

1. **Friendly scales:** aeolian, phrygian, minorPentatonic, phrygianDom, harmonicMinor, hungarianMinor. Dorian,
   melodicMinor and mixolydian are left off.
2. **32 step patterns:** Euclidean rhythms E(1..32, 32). Pattern i has i+1 onsets. The patterns are distinct.
3. **Trance gates:** series 0 to 7 are repetitive (an 8-step Euclidean figure tiled), 8 to 23 are polyrhythmic
   (E(3..18, 32)), and 24 to 31 are random, with density rising. The random patterns come from a seed set on entry
   (`tranceGate(series, entrySeed)`), so the same entry gives the same pattern, and each new change gives a new seed.
4. **Octave dial:** seven positions, -3 to +3.
5. **Arp speeds:** half, quarter, double, triplet (3/2), quad.
6. **Filter shift:** the dial maps linearly to -1..+1, with centre at 0.5.
7. **LFO types:** four, to be named once the mod-wheel LFO exists (the constant is `kLfoTypeCount`).

## 5. What already exists and what does not

- **Exists:** the `LoopTrack` rules, the scale whitelist, the 32-step bank, the trance-gate series, the octave, gate,
  arp-speed and filter-shift mappings, all witnessed on the host.
- **Does not exist yet:**
  - the 8-bar loop in the bass engine (`bass_engine.cpp` still builds a 64-bar phrase and regenerates it);
  - the arp modes for width (`bass_engine.cpp` has no arp);
  - the LFO on the mod wheel (no CC 1 LFO);
  - the chromatic aberration and random drop-out (the random octave and note removal are new);
  - the loop-window shortening (no loop window);
  - the wiring of pads and dials to these in `state_machine.cpp`.

## 6. Decisions for the user (these change behaviour)

These are the only points where the request could be read more than one way. Each one changes what the device does,
so each needs a yes or a different answer before the firmware is wired.

1. **Do the eight approaches (ROLL, ACID, FUNK, GLITCH, STAB, DRIFT, PEDAL, BREAK) survive?** The pads now mean
   something else, so the approaches have nowhere to live. My default is to keep them as **voice presets** reached by a
   long press, which is currently the nudge gesture, so nothing is lost.
2. **Does the loop replace the 64-bar phrase?** My default is yes: the 8-bar loop replaces the phrase, and the
   per-bar shaping runs inside it.
3. **"Adjustment period still underway":** I took it to mean "touched at the turnaround". A different reading (for
   example, touched within the last bar) changes section 3.
4. **Clearing a persistent track:** my default is a double-tap of a pad while holding that pad's dial to zero. There is
   no clear gesture in the request.

## 7. Next steps, in order

1. Wire `control_model` into `state_machine.cpp` (pad modes, dial routing, double-tap while held) with no change
   to the audio path. Host-testable with a state-machine witness.
2. Make the bass engine loop 8 bars, behind the existing `kBarsPerPhrase`, and record dial automation with `LoopTrack`.
3. The arp, LFO, drop-out and aberration features, one per row, each with its own witness.
4. Device run: CI build, one console open, soak, then the go-ahead, under the same protocol as the mesh work.
