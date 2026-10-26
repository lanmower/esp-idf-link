# Bassline controls: the 8-bar loop, four pads, two dials

Status: **implemented in firmware** (`main/bass_engine.cpp`, `main/state_machine.cpp`, `main/control_model.{h,cpp}`)
and host-witnessed in CI (`tools/control-model-witness.sh`, `tools/bass-loop-witness.sh`). The approach-based engine,
the 64-bar phrase and the grooves were removed.

## The control map

Pads are numbered 0 to 3. Dial 0 is the first pot, dial 1 the second.

| Pad | Gesture | Dial 0 | Dial 1 |
|---|---|---|---|
| 0 | single tap | scale (friendly list only) | width: held pitch to arp, higher and lower |
| 0 | double tap, held | gate width, 0 to 100 percent | arp speed: half, quarter, double, triplet, quad |
| 1 | single tap | pitch offset, -1 to +1 octave along the scale | rhythm: one of 32 step patterns |
| 1 | double tap, held | random octave up and down, by intensity | random note drop-out, by intensity |
| 2 | single tap | LFO type (mod wheel, CC 1) | LFO speed |
| 2 | double tap, held | LFO width, -100 to +100 percent | filter shift, -100 to +100 percent (CC 74) |
| 3 | single tap | trance-gate series (32, repetitive to random) | octave, -3 to +3 |
| 3 | double tap, held | loop window, 1 to 8 bars | chromatic aberration, by intensity |

A single tap selects the pad's single settings and starts the sequencer. A double tap, while held, sends the dials to
that pad's double settings. Releasing the pad returns them to the single settings.

## The loop and the recorder

- The loop is 8 bars of sixteenths (128 steps). The loop window shortens or expands it, and the change applies at the
  next turnaround.
- Every setting is recorded through `ctl::LoopTrack`:
  - while a dial moves, its setting is recorded at each step it passes (touch expires 250 ms after the last move);
  - a released change holds until the turnaround;
  - a change still touched at the turnaround becomes persistent: it replays on later passes, and touching it again
    overwrites the positions it passes;
  - a change not held through the turnaround reverts to the setting's baseline.

## Defaults I chose where the request was open

1. **Approach names** (ROLL, ACID and the rest) are removed. Their pads now carry the new settings.
2. **The loop replaces the 64-bar phrase.** The loop is the phrase.
3. **Adjustment period** means "touched at the turnaround". A different reading changes the recorder only.
4. **Clearing a persistent setting** is not yet a gesture in firmware: `ctl::LoopTrack::clear()` exists, but no pad
   or dial calls it.
5. **Fake sidechain** is series 0 of the volume-modulation dial: it skips the first sixteenth of every
   beat. It is part of the dial's range, not always on, so a fully open gate is reachable.
6. **Volume-modulation dial range:** series 0 fake sidechain, 1-8 repetitive tilings (8 is fully open,
   every sixteenth), 9-24 polyrhythmic, 25-31 random. Every sixteenth is one dial position away.
7. **Random patterns** are seeded when a setting is touched, so each new change gives a new pattern, and the same
   entry gives the same pattern.
8. **Defaults at boot:** scale 0, width 0, pitch offset 0, rhythm 31 (every sixteenth), LFO sine at 0.25 cycles
   per bar, volume modulation series 8 (fully open), octave 0, gate width 50 percent, arp 16ths, no random octave
   or drop-out, LFO width 0, filter centred, loop 8 bars, no aberration. Those defaults are a gated
   sixteenth-note roll, because the rhythm is every sixteenth and the gate is fully open.

## Still open

- Hardware check of the new control map on the board. The host witnesses cover the model and the engine's output,
  not the synths.
- The clear gesture (item 4 above).
