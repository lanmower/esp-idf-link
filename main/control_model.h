// Control model for the bassline: the 8-bar loop, the 4 pads with single and double-tap
// functions, the two dials, and the automation that is recorded while a dial is being moved.
//
// Host-compilable: only <cstdint> and <cstddef> are included, so the witness in
// tools/control_model_witness.cpp exercises this code off-device.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ctl {

// The loop is 8 bars of 16 sixteenths. Every recording lands on this grid.
constexpr int kStepsPerBar = 16;
constexpr int kLoopBars = 8;
constexpr int kLoopSteps = kStepsPerBar * kLoopBars;  // 128

// Pads are indexed 0..3 in the order state_machine.cpp numbers them (APPROACH_FOR_PAD).
constexpr int kPadCount = 4;

// Single-tap modes, one per pad. The two dials control the pair of parameters the mode names.
enum SingleMode : uint8_t {
    SINGLE_SCALE_WIDTH = 0,  // dial A: scale (friendly list only); dial B: width, held pitch -> arp
    SINGLE_PITCH_RHYTHM,     // dial A: pitch offset, -1 to +1 octave along the scale; dial B: 32-step rhythm
    SINGLE_LFO,              // dial A: LFO type; dial B: LFO speed, on the mod wheel
    SINGLE_GATE_OCTAVE,      // dial A: trance-gate series; dial B: octave, -3 to +3
    SINGLE_MODE_COUNT
};

// Double-tap modes, one per pad, active while the pad is held.
enum DoubleMode : uint8_t {
    DOUBLE_GATE_ARP = 0,     // dial A: gate width 0 to 100%; dial B: arp speed (half, quarter, double, triplet, quad)
    DOUBLE_OCTAVE_DROP,      // dial A: random octave up/down likelihood; dial B: random note drop-out
    DOUBLE_LFO_FILTER,       // dial A: LFO width; dial B: filter shift, -100% to +100%
    DOUBLE_LOOP_ABERRATION,  // dial A: loop window length; dial B: chromatic aberration quantity
    DOUBLE_MODE_COUNT
};

// Friendly scales: the subset of the nine bassline scales that suits EDM and psytrance. Indices
// are the bassline_interpreter scale indices (aeolian, phrygian, minorPentatonic, phrygianDom,
// harmonicMinor, hungarianMinor).
constexpr int kFriendlyScaleCount = 6;
extern const int kFriendlyScales[kFriendlyScaleCount];
bool isFriendlyScale(int scaleIdx);
// Maps a dial position in [0, 1] to a friendly scale index.
int friendlyScaleForDial(float dial01);

// The 32-step bank. Pattern i is the Euclidean rhythm E(i + 1, 32): onsets spread as evenly as
// possible over 32 sixteenths. Bit n is step n.
constexpr int kStepPatternCount = 32;
uint32_t stepPattern(int index);
int stepPatternForDial(float dial01);

// Trance-gate series (volume modulation, 32 steps). Indices 0..7 are repetitive and short,
// 8..23 are polyrhythmic Euclidean, 24..31 are random. The random patterns are seeded from the
// entry seed, so the same entry always gives the same pattern, and a new change gives a new seed.
constexpr int kGateSeriesCount = 32;
uint32_t tranceGate(int seriesIndex, uint32_t entrySeed);
int tranceGateIndexForDial(float dial01);

// Octave shift for the octave dial, -3 to +3.
int octaveShiftForDial(float dial01);

// Gate width for double-tap A on pad 0, 0 (nothing) to 1 (100%).
float gateWidthForDial(float dial01);

// Arp speed for double-tap B on pad 0. Returns the multiplier as numerator over denominator in
// sixteenths of a step: half = 1/2, quarter = 1/4, double = 2, triplet = 3/2 (three notes in the
// time of two), quad = 4.
struct ArpSpeed {
    int numerator;
    int denominator;
};
ArpSpeed arpSpeedForDial(float dial01);

// Filter shift, -1 (full down) to +1 (full up), from the dial position.
float filterShiftForDial(float dial01);

// LFO type index for the single-tap LFO mode, 0..(kLfoTypeCount - 1).
constexpr int kLfoTypeCount = 4;
int lfoTypeForDial(float dial01);

// Loop automation for one setting (one dial in one mode). This is the KO2-style recording rule:
//   - while the setting is touched, each step's value is written to the track at the playhead;
//   - at the loop turnaround, a setting that is still touched becomes persistent: its track is
//     kept and replayed on every later pass, and the touch keeps overwriting it;
//   - a setting that is not touched at the turnaround and is not persistent reverts to its
//     baseline, which is the value it had at the start of the loop;
//   - a persistent track that is not being touched keeps replaying, until it is cleared.
class LoopTrack {
public:
    void setBaseline(float value);
    float baseline() const { return baseline_; }

    // Touch state for the current pass.
    void setTouched(bool touched);
    bool touched() const { return touched_; }

    // The value this setting has at a loop step, given the live dial value. While touched, the
    // live value is written to the track at the step and is returned unchanged.
    float valueAt(int step, float liveValue);

    // Call once when the loop wraps back to step 0.
    void turnaround();

    // Removes the persistent recording and returns the setting to its baseline.
    void clear();

    bool persistent() const { return persistent_; }
    bool hasRecordedStep(int step) const;

private:
    float baseline_ = 0.0f;
    float values_[kLoopSteps] = {};
    bool valid_[kLoopSteps] = {};
    bool touched_ = false;
    bool persistent_ = false;
};

}  // namespace ctl
