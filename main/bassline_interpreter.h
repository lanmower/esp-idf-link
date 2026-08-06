#pragma once
// Portable (no ESP-IDF dependency) bassline note-generation core.
//
// Replaces the old 8-genre lookup-table generators (genFunk/genItalo/...)
// in bass_engine.cpp with a single interpreter driven by 8 continuous
// dials, grouped into 4 banks of 2 -- one bank per touch pad, one dial per
// physical potentiometer, matching the hardware exactly (see AGENTS.md /
// readme.md "Bassline interpreter control surface").
//
// Design validated against a Python port + DawDreamer audio renders before
// this port -- see ../../DawDreamer/experiments/bassline/ in the sibling
// repo (groove-template onset generation + constrained DP pitch solver +
// contour shaping + articulation, following the same structure as the
// SUBSTRATE HTML bassline interpreter this was modeled on).
//
// Kept free of ESP-IDF/Arduino headers on purpose so it can be
// host-compiled and unit-tested with plain g++/clang++ -- there is no
// on-device build toolchain available in most dev sandboxes, so this is
// the only part of the note-generation path that can be verified by
// actually running it before flashing.

#include <cstdint>

namespace bli {

// ---------------------------------------------------------------
// Scales -- identical table to the one the old bass_engine.cpp used,
// kept unchanged so the "sound" of any given scale doesn't shift.
// ---------------------------------------------------------------
constexpr int kNumScales = 9;
extern const int kScaleLens[kNumScales];
extern const int kScales[kNumScales][7];
const char* scaleName(int scaleIdx);
int scaleDegree(int scaleIdx, int degree);      // sc()
void chordToneIntervals(int scaleIdx, int out[4]); // root/3rd/5th/7th, by stacking scale thirds

// ---------------------------------------------------------------
// Dial parameter block: 4 banks x 2 dials = 8 continuous [0,1] params.
// One bank is "active" at a time (selected by the last touch pad tapped);
// its 2 dials are driven live by the project's 2 physical potentiometers.
// ---------------------------------------------------------------
enum Bank {
    BANK_HARMONY = 0,   // dial0 = gravity (chord-tone pull), dial1 = color (scale + tension)
    BANK_GROOVE  = 1,   // dial0 = energy (template + density), dial1 = swing (syncopation + ratio)
    BANK_MOTION  = 2,   // dial0 = contour (pitch shape), dial1 = variation (leap/randomness)
    BANK_VOICE   = 3,   // dial0 = sweep (filter automation), dial1 = artic (accent/ghost/slide)
    BANK_COUNT   = 4
};

struct Dials {
    float harmonyGravity  = 0.66f;
    float harmonyColor    = 0.32f;
    float grooveEnergy    = 0.50f;
    float grooveSwing     = 0.36f;
    float motionContour   = 0.50f;
    float motionVariation = 0.36f;
    float voiceSweep      = 0.46f;
    float voiceArtic      = 0.40f;

    // dialIdx: 0 or 1, matches physical pot 0/1.
    float get(int bank, int dialIdx) const;
    void  set(int bank, int dialIdx, float v01);

    int pickScaleIdx() const { return bandIndex(harmonyColor, kNumScales); }

    static int bandIndex(float v01, int count) {
        int idx = static_cast<int>(v01 * count);
        if (idx < 0) idx = 0;
        if (idx >= count) idx = count - 1;
        return idx;
    }
};

// ---------------------------------------------------------------
// One 16-step motif slot -- field-compatible with bass_engine.cpp's
// existing MS struct so regeneratePhrase()'s arc/turnaround/transform
// machinery (which is unchanged) can keep consuming it directly.
// ---------------------------------------------------------------
struct Step {
    int   note = -1;   // -1 = empty; else absolute MIDI note in [root, root+15]
    float len  = 0.f;   // duration in 16th-note steps
    int   vel  = 0;      // 0-127
    int   fcc  = 0;      // filter CC value 0-127
    float pb   = 0.f;     // pitch bend semitones (nonzero only on slides)
};

// Random source is injected: the firmware plugs in esp_random() (true HW
// entropy, matching how the rest of bass_engine.cpp already works), while
// host-side tools/tests plug in a seeded PRNG for reproducibility.
struct RngSource {
    virtual ~RngSource() = default;
    virtual float next01() = 0;   // uniform [0, 1)
};

constexpr int kStepsPerBar = 16;

// Generates one bar of interpreted bassline, relative to `root` (the
// caller adds the chord-degree offset per bar, same convention the old
// genXXX functions used).
void generateMotif(Step m[kStepsPerBar], int root, int scaleIdx,
                    const Dials& dials, RngSource& rng);

// Turnaround tail -- called once at the phrase's final bar. Writes up to
// 3 notes into `out` (positions are step offsets within the trailing
// 8-step tail, i.e. 0..7) and returns how many were written.
struct TurnNote {
    float stepOffset;  // 0..8, position within the turnaround tail
    int   note;         // absolute MIDI note (root + interval already applied)
    float len;           // in 16th-note steps
    int   vel;
    int   fcc;
    float pb;
};
int generateTurnaround(TurnNote out[3], int root, int scaleIdx,
                        const Dials& dials, RngSource& rng);

}  // namespace bli
