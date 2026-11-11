#pragma once

#include <cstdint>

namespace ctl {

constexpr int kStepsPerBar  = 16;
constexpr int kTicksPerStep = 12;
constexpr int kTicksPerBar  = kStepsPerBar * kTicksPerStep;
constexpr int kLoopBars     = 2;
constexpr int kLoopSteps    = kStepsPerBar * kLoopBars;
constexpr int kLoopMaxBeats = kLoopBars * 4;

constexpr int kPadCount     = 4;
constexpr int kControlCount = 16;

constexpr int controlIndex(bool doubleTap, int pad, int dial) {
    return (doubleTap ? 8 : 0) + pad * 2 + dial;
}

constexpr int kScaleCount = 6;
int scaleSize(int scale);
int scaleSemitone(int scale, int degree);

int   scaleForDial(float v);
float widthForDial(float v);
float pitchOctaveForDial(float v);
int   rhythmForDial(float v);
int   lfoTypeForDial(float v);
float lfoCyclesPerBarForDial(float v);
float gateWidthForDial(float v);
int   arpSpeedIndexForDial(float v);
int   arpPeriodTicks(int arpIndex);
float probabilityForDial(float v);
float lfoWidthForDial(float v);
float filterShiftForDial(float v);
int   gateSeriesForDial(float v);
int   octaveForDial(float v);
int   loopBarsForDial(float v);
float aberrationForDial(float v);

constexpr int kRhythmPatternCount = 32;
uint32_t stepPattern(int index);

constexpr int kGateSeriesCount = 33;
uint32_t tranceGate(int seriesIndex);

uint32_t hash32(uint32_t a, uint32_t b, uint32_t c);
float    hashUnit(uint32_t a, uint32_t b, uint32_t c);

class LoopTrack {
public:
    void setBaseline(float value);
    float baseline() const { return baseline_; }

    void setTouched(bool touched);
    bool touched() const { return touched_; }

    void startRecording(float value);

    float valueAt(int step, float liveValue);

    void clear();

    bool hasRecordedStep(int step) const;

private:
    float baseline_   = 0.0f;
    float values_[kLoopSteps] = {};
    bool  valid_[kLoopSteps]  = {};
    float hold_       = 0.0f;
    bool  holdValid_  = false;
    bool  touched_    = false;
};

}
