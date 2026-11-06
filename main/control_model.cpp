#include "control_model.h"

namespace ctl {

namespace {

constexpr int kMaxScaleDegrees = 7;

constexpr int kScaleSizes[kScaleCount] = {7, 7, 5, 7, 7, 7};

constexpr int kAeolian[kMaxScaleDegrees]          = {0, 2, 3, 5, 7, 8, 10};
constexpr int kPhrygian[kMaxScaleDegrees]         = {0, 1, 3, 5, 7, 8, 10};
constexpr int kMinorPentatonic[kMaxScaleDegrees]  = {0, 3, 5, 7, 10, 0, 0};
constexpr int kPhrygianDominant[kMaxScaleDegrees] = {0, 1, 4, 5, 7, 8, 10};
constexpr int kHarmonicMinor[kMaxScaleDegrees]    = {0, 2, 3, 5, 7, 8, 11};
constexpr int kHungarianMinor[kMaxScaleDegrees]   = {0, 2, 3, 6, 7, 8, 11};
constexpr const int* kScaleSemitones[kScaleCount] = {
    kAeolian, kPhrygian, kMinorPentatonic, kPhrygianDominant, kHarmonicMinor, kHungarianMinor
};

constexpr int kArpPeriodHalf       = 24;
constexpr int kArpPeriodQuarter    = 48;
constexpr int kArpPeriodSixteenths = 12;
constexpr int kArpPeriodTriplet    = 8;
constexpr int kArpPeriodQuad       = 3;
constexpr int kArpPeriods[5] = {
    kArpPeriodHalf, kArpPeriodQuarter, kArpPeriodSixteenths, kArpPeriodTriplet, kArpPeriodQuad
};

float clamp01(float v) {
    if (v <= 0.0f) return 0.0f;
    if (v >= 1.0f) return 1.0f;
    return v;
}

int indexFor(float v, int count) {
    int idx = static_cast<int>(clamp01(v) * static_cast<float>(count));
    return idx >= count ? count - 1 : idx;
}

uint32_t euclidean(int k, int n) {
    uint32_t bits = 0;
    for (int i = 0; i < n; i++) {
        if ((i * k) % n < k) bits |= (1u << i);
    }
    return bits;
}

uint32_t fakeSidechainGate() {
    uint32_t bits = 0;
    for (int i = 0; i < 32; i++) {
        if (i % 4 != 0) bits |= (1u << i);
    }
    return bits;
}

}

int scaleSize(int scale) {
    if (scale < 0) scale = 0;
    if (scale >= kScaleCount) scale = kScaleCount - 1;
    return kScaleSizes[scale];
}

int scaleSemitone(int scale, int degree) {
    if (scale < 0) scale = 0;
    if (scale >= kScaleCount) scale = kScaleCount - 1;
    if (degree < 0) degree = 0;
    if (degree >= kScaleSizes[scale]) degree = kScaleSizes[scale] - 1;
    return kScaleSemitones[scale][degree];
}

int scaleForDial(float v)          { return indexFor(v, kScaleCount); }
float widthForDial(float v)        { return clamp01(v); }
float pitchOctaveForDial(float v)  { return clamp01(v) * 2.0f - 1.0f; }
int rhythmForDial(float v)         { return indexFor(v, kRhythmPatternCount); }
int lfoTypeForDial(float v)        { return indexFor(v, 4); }
float lfoCyclesPerBarForDial(float v) { return 0.25f + 3.75f * clamp01(v); }
float gateWidthForDial(float v)    { return clamp01(v); }
int arpSpeedIndexForDial(float v)  { return indexFor(v, 5); }
int arpPeriodTicks(int arpIndex) {
    if (arpIndex < 0) arpIndex = 0;
    if (arpIndex > 4) arpIndex = 4;
    return kArpPeriods[arpIndex];
}
float probabilityForDial(float v)  { return clamp01(v); }
float lfoWidthForDial(float v)     { return clamp01(v) * 2.0f - 1.0f; }
float filterShiftForDial(float v)  { return clamp01(v) * 2.0f - 1.0f; }
int gateSeriesForDial(float v)     { return indexFor(v, kGateSeriesCount); }
int octaveForDial(float v)         { return indexFor(v, 7) - 3; }
int loopBarsForDial(float v)       { return 1 + indexFor(v, kLoopBars); }
float aberrationForDial(float v)   { return clamp01(v); }

uint32_t stepPattern(int index) {
    if (index < 0) index = 0;
    if (index >= kRhythmPatternCount) index = kRhythmPatternCount - 1;
    return euclidean(index + 1, 32);
}

uint32_t tranceGate(int seriesIndex, uint32_t entrySeed) {
    if (seriesIndex < 0) seriesIndex = 0;
    if (seriesIndex >= kGateSeriesCount) seriesIndex = kGateSeriesCount - 1;

    if (seriesIndex == 0) {
        return fakeSidechainGate();
    }
    if (seriesIndex <= 8) {
        const uint32_t base = euclidean(seriesIndex, 8);
        uint32_t bits = 0;
        for (int i = 0; i < 32; i++) {
            if (base & (1u << (i % 8))) bits |= (1u << i);
        }
        return bits;
    }
    if (seriesIndex <= 24) {
        return euclidean(seriesIndex - 9 + 3, 32);
    }
    uint32_t state = entrySeed * 2654435761u + static_cast<uint32_t>(seriesIndex) * 40503u + 1u;
    const uint32_t threshold = static_cast<uint32_t>((seriesIndex - 25 + 1) * 64);
    uint32_t bits = 0;
    for (int i = 0; i < 32; i++) {
        state = state * 1664525u + 1013904223u;
        const uint32_t r = state >> 22;
        if (r < threshold) bits |= (1u << i);
    }
    return bits;
}

uint32_t hash32(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t x = a * 0x9E3779B1u ^ b * 0x85EBCA77u ^ c * 0xC2B2AE3Du;
    x ^= x >> 15;
    x *= 0x2C1B3C6Du;
    x ^= x >> 12;
    x *= 0x297A2D39u;
    x ^= x >> 15;
    return x;
}

float hashUnit(uint32_t a, uint32_t b, uint32_t c) {
    return static_cast<float>(hash32(a, b, c) >> 8) / 16777216.0f;
}

void LoopTrack::setBaseline(float value) {
    baseline_ = value;
}

void LoopTrack::setTouched(bool touched) {
    if (touched && !touched_) {
        for (int i = 0; i < kLoopSteps; i++) valid_[i] = false;
        persistent_ = false;
    }
    touched_ = touched;
}

void LoopTrack::startRecording(float value) {
    for (int i = 0; i < kLoopSteps; i++) valid_[i] = false;
    persistent_ = false;
    hold_ = value;
    holdValid_ = true;
    touched_ = true;
}

float LoopTrack::valueAt(int step, float liveValue) {
    if (step < 0) step = 0;
    if (step >= kLoopSteps) step = kLoopSteps - 1;
    if (touched_) {
        values_[step] = liveValue;
        valid_[step] = true;
        hold_ = liveValue;
        holdValid_ = true;
        return liveValue;
    }
    if (valid_[step]) return values_[step];
    if (holdValid_) return hold_;
    return baseline_;
}

void LoopTrack::turnaround() {
    if (touched_ || holdValid_) persistent_ = true;
}

void LoopTrack::clear() {
    persistent_ = false;
    holdValid_ = false;
    for (int i = 0; i < kLoopSteps; i++) valid_[i] = false;
}

bool LoopTrack::hasRecordedStep(int step) const {
    if (step < 0 || step >= kLoopSteps) return false;
    return valid_[step];
}

}
