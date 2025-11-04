User Operation Guide
Here is a guide based on the implemented controls:

General Operation
- Pads: 4 touch-sensitive pads trigger/control effects.
- Potentiometers (Pots): 2 potentiometers control parameters.
- Pot Control Assignment: The pots always control parameters for the effect associated with the last pad (0, 1, 2, or 3) that was interacted with (pressed or double-tapped in the case of Pad 2). The pots keep controlling that context even after the pad is released, until a different pad establishes a new context.
- Adjust Mode (Pads 0, 1, 3): Hold the primary effect pad (0, 1, or 3). While holding, tapping a *different* pad triggers a secondary function (submenu action) for the *held* pad's effect. Moving the pots while holding also counts as interaction.
- Synth Switching: Hold all four pads for ~1 second to toggle between Mininova and MicroKorg synth modes. A log message confirms the switch. On MicroKorg, reverb parameters have no effect.

Pad Functions (Pads 0, 1, 3 - Latching Effects)
General Latching Behaviour (Pads 0, 1, 3):
- Press: Instantly latches the effect ON if it was OFF. Enters "Adjust Mode" - pots immediately control this effect, interaction is tracked.
- Hold: Effect remains ON. Pots control parameters. Tapping other pads activates submenus.
- Release (Quick Tap): If held briefly (< ~200ms) AND no interaction occurred (no pot move, no secondary tap), the effect latches OFF.
- Release (Long Hold / Interaction): If held longer OR interaction occurred, the effect remains latched ON.

Pad 0: Sidechain (SC)
- Latching: Follows General Latching Behaviour.
- Pots Control (when SC is last interacted):
  - Pot 1: Sidechain Depth.
  - Pot 2: Sidechain Sheer (Attack/Release Time).
- Adjust Mode (Hold Pad 0, Tap another pad):
  - Tap Pad 1: Select Sidechain Pattern 1.
  - Tap Pad 3: Select Sidechain Pattern 3.
  - (Tap Pad 2: No Action).

Pad 1: Arpeggiator (Arp)
- Latching: Follows General Latching Behaviour.
- Default Mode: When latched ON, defaults to NOTE mode.
- Double-Tap: Toggles Arp between NOTE mode and CHORD mode.
- Speed: NOTE mode plays faster (currently 2x) than CHORD mode.
- Pots Control (when Arp is last interacted):
  - Pot 1: Arp Gate Length.
  - Pot 2: Arp Swing Amount.
- Adjust Mode (Hold Pad 1, Tap another pad):
  - Tap Pad 0: Cycle Rhythm Pattern (specific to current NOTE/CHORD mode).
  - Tap Pad 2: Cycle Scale.
  - Tap Pad 3: Manually advance Progression Step.

Pad 3: Filter (Filt)
- Latching: Follows General Latching Behaviour.
- Pots Control (when Filt is last interacted):
  - Pot 1: Filter Cutoff.
  - Pot 2: Filter Resonance.
- Adjust Mode (Hold Pad 3, Tap another pad):
  - First tap on Pad 0, 1, or 2: Patches LFO to Filter Cutoff and enables LFO.
  - Subsequent Taps (while holding Pad 3):
    - Tap Pad 0: Cycle LFO Shape.
    - Tap Pad 1: Cycle LFO Sync Rate.
    - Tap Pad 2: Cycle LFO Depth.

Pad 2: Delay / Reverb (Del/Rev) - Momentary Effect
- Primary Action: Momentary.
- Tap & Hold: Activates Delay (FX Level 100%). Pots control Delay Time (1) / Feedback (2).
- Double-Tap & Hold: Activates Reverb (FX Level 100%). Pots control Reverb Decay (1) / Damping (2).
- Release: Deactivates Delay/Reverb (FX Level 0%).
- Adjust Mode (Hold Pad 2, Tap another pad):
  - Tapping Pad 0, 1, or 3: Toggles the *potentiometer context* between Delay Adjust and Reverb Adjust (changes which effect the pots control when Pad 2 is subsequently held). The effect itself remains momentary (only active while Pad 2 is held).

Potentiometer Behaviour Detail
- Smoothing: Uses Exponential Moving Average (EMA) for smoothness.
- Jitter Rejection: Uses a secondary, slower EMA to establish a "stable center". Movement is only registered if the primary smoothed value deviates significantly from this stable center, preventing noise from causing parameter changes while allowing slow, deliberate movements.