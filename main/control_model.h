// Control model for the bassline sequencer: an 8-bar loop, four pads with a single-tap and a
// double-tap setting each, two dials, and the automation recorded while a dial is being moved.
//
// Host-compilable: only <cstdint> is included, so tools/control_model_witness.cpp exercises it
// off-device.
#pragma once

#include <cstdint>

namespace ctl {

constexpr int kStepsPerBar  = 16;                           // sixteenths per bar
constexpr int kTicksPerStep = 12;                           // scheduler ticks per sixteenth
constexpr int kTicksPerBar  = kStepsPerBar * kTicksPerStep;
// The loop is at most eight beats: two bars of sixteenths. No control lengthens it past that.
constexpr int kLoopBars     = 2;
constexpr int kLoopSteps    = kStepsPerBar * kLoopBars;     // 32 steps: the largest loop
constexpr int kLoopMaxBeats = kLoopBars * 4;                // 8 beats

constexpr int kPadCount     = 4;
constexpr int kControlCount = 16;                           // 8 single-tap, 8 double-tap settings

// Setting index: single-tap pad p, dial d is p*2+d; double-tap pad p, dial d is 8+p*2+d.
constexpr int controlIndex(bool doubleTap, int pad, int dial) {
    return (doubleTap ? 8 : 0) + pad * 2 + dial;
}

// Friendly scales, EDM and psytrance suited. Scale indices run 0..kScaleCount-1.
constexpr int kScaleCount = 6;
int scaleSize(int scale);
int scaleSemitone(int scale, int degree);

// Dial mappings: each takes a dial position in [0, 1].
int   scaleForDial(float v);            // friendly scale index
float widthForDial(float v);            // 0 = held pitch, 1 = widest arp
float pitchOctaveForDial(float v);      // -1 .. +1 octave along the scale
int   rhythmForDial(float v);           // 32-step rhythm pattern index
int   lfoTypeForDial(float v);          // 0 sine, 1 triangle, 2 saw, 3 square
float lfoCyclesPerBarForDial(float v);  // 0.25 .. 4 cycles per bar
float gateWidthForDial(float v);        // 0 .. 1 of the arp period
int   arpSpeedIndexForDial(float v);    // 0 half, 1 quarter, 2 16ths, 3 triplet, 4 quad
int   arpPeriodTicks(int arpIndex);     // ticks between arp notes
float probabilityForDial(float v);      // 0 .. 1 (random octave, drop-out, aberration)
float lfoWidthForDial(float v);         // -1 .. +1 depth on the mod wheel
float filterShiftForDial(float v);      // -1 .. +1 around the centre cutoff
int   gateSeriesForDial(float v);       // 0 .. 31 trance-gate series
int   octaveForDial(float v);           // -3 .. +3
int   loopBarsForDial(float v);         // 1 .. 2 bars (4 .. 8 beats)
float aberrationForDial(float v);       // 0 .. 1

// The 32-step bank. Pattern i is the Euclidean rhythm E(i + 1, 32); bit n is step n.
constexpr int kRhythmPatternCount = 32;
uint32_t stepPattern(int index);

// Trance-gate series (volume modulation, 32 steps). 0..7 repetitive, 8..23 polyrhythmic,
// 24..31 random, seeded by the entry seed so each new change gives a new pattern.
constexpr int kGateSeriesCount = 32;
uint32_t tranceGate(int seriesIndex, uint32_t entrySeed);

// Deterministic hashing, so a seeded pattern is repeated exactly.
uint32_t hash32(uint32_t a, uint32_t b, uint32_t c);
float    hashUnit(uint32_t a, uint32_t b, uint32_t c);   // [0, 1)

// Automation for one setting. KO2-style recording:
//   - while the setting is touched, each step's value is written to the track and played live;
//   - once a dial is released, its value is held until the loop turns around;
//   - at the turnaround, a setting that is still touched becomes persistent: its track is kept
//     and replayed on every later pass, and touching it again overwrites the positions it passes;
//   - a setting that is not touched and not persistent reverts to its baseline;
//   - a persistent track that is not touched keeps replaying, until it is cleared.
class LoopTrack {
public:
    void setBaseline(float value);
    float baseline() const { return baseline_; }

    void setTouched(bool touched);
    bool touched() const { return touched_; }

    // The value at a loop step. While touched, the live value is recorded at the step and returned.
    float valueAt(int step, float liveValue);

    // Call once when the loop wraps back to step 0.
    void turnaround();

    // Removes the persistent recording and returns the setting to its baseline.
    void clear();

    bool persistent() const { return persistent_; }
    bool hasRecordedStep(int step) const;

private:
    float baseline_   = 0.0f;
    float values_[kLoopSteps] = {};
    bool  valid_[kLoopSteps]  = {};
    float hold_       = 0.0f;
    bool  holdValid_  = false;
    bool  touched_    = false;
    bool  persistent_ = false;
};

}  // namespace ctl
