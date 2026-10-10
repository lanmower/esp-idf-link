#pragma once

#include <cstdint>

namespace bli {

constexpr int kChordRoot = 0;
constexpr int kChordThird = 1;
constexpr int kChordFifth = 2;
constexpr int kChordSeventh = 3;

constexpr int kNumScales = 9;
extern const int kScaleLens[kNumScales];
extern const int kScales[kNumScales][7];
const char* scaleName(int scaleIdx);
int scaleDegree(int scaleIdx, int degree);
void chordToneIntervals(int scaleIdx, int out[4]);

enum Bank {
    BANK_HARMONY = 0,
    BANK_GROOVE  = 1,
    BANK_MOTION  = 2,
    BANK_VOICE   = 3,
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

struct Step {
    int   note = -1;
    float len  = 0.f;
    int   vel  = 0;
    int   fcc  = 0;
    float pb   = 0.f;
};

struct RngSource {
    virtual ~RngSource() = default;
    virtual float next01() = 0;
};

constexpr int kStepsPerBar = 16;

void generateMotif(Step m[kStepsPerBar], int root, int scaleIdx,
                   const Dials& dials, RngSource& rng);

struct TurnNote {
    float stepOffset;
    int   note;
    float len;
    int   vel;
    int   fcc;
    float pb;
};
int generateTurnaround(TurnNote out[3], int root, int scaleIdx,
                       const Dials& dials, RngSource& rng);

}
