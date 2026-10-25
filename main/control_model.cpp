#include "control_model.h"

namespace ctl {

const int kFriendlyScales[kFriendlyScaleCount] = {1, 2, 3, 6, 7, 8};

bool isFriendlyScale(int scaleIdx) {
    for (int i = 0; i < kFriendlyScaleCount; i++) {
        if (kFriendlyScales[i] == scaleIdx) return true;
    }
    return false;
}

static int clampIndex(float dial01, int count) {
    if (dial01 <= 0.0f) return 0;
    if (dial01 >= 1.0f) return count - 1;
    int idx = static_cast<int>(dial01 * static_cast<float>(count));
    return idx >= count ? count - 1 : idx;
}

int friendlyScaleForDial(float dial01) {
    return kFriendlyScales[clampIndex(dial01, kFriendlyScaleCount)];
}

// Euclidean rhythm E(k, n): step i is an onset when (i * k) mod n is less than k. That spreads k
// onsets as evenly as possible over n steps.
static uint32_t euclidean(int k, int n) {
    uint32_t bits = 0;
    for (int i = 0; i < n; i++) {
        if ((i * k) % n < k) bits |= (1u << i);
    }
    return bits;
}

uint32_t stepPattern(int index) {
    if (index < 0) index = 0;
    if (index >= kStepPatternCount) index = kStepPatternCount - 1;
    return euclidean(index + 1, 32);
}

int stepPatternForDial(float dial01) {
    return clampIndex(dial01, kStepPatternCount);
}

// A small deterministic generator, so a seeded pattern repeats exactly.
static uint32_t lcgNext(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

uint32_t tranceGate(int seriesIndex, uint32_t entrySeed) {
    if (seriesIndex < 0) seriesIndex = 0;
    if (seriesIndex >= kGateSeriesCount) seriesIndex = kGateSeriesCount - 1;

    if (seriesIndex < 8) {
        // Repetitive and short: an 8-step Euclidean figure, tiled over the 32 steps.
        const uint32_t base = euclidean(seriesIndex + 1, 8);
        uint32_t bits = 0;
        for (int i = 0; i < 32; i++) {
            if (base & (1u << (i % 8))) bits |= (1u << i);
        }
        return bits;
    }
    if (seriesIndex < 24) {
        // Polyrhythmic: Euclidean E(k, 32) with k running from 3 to 18.
        return euclidean(seriesIndex - 8 + 3, 32);
    }
    // Random: the density rises with the index, and the bits come from the entry seed.
    uint32_t state = entrySeed * 2654435761u + static_cast<uint32_t>(seriesIndex) * 40503u + 1u;
    const int threshold = (seriesIndex - 24 + 1) * 64;  // 64 to 512 out of 1024 per step
    uint32_t bits = 0;
    for (int i = 0; i < 32; i++) {
        const int r = static_cast<int>(lcgNext(state) >> 22);  // 0..1023
        if (r < threshold) bits |= (1u << i);
    }
    return bits;
}

int tranceGateIndexForDial(float dial01) {
    return clampIndex(dial01, kGateSeriesCount);
}

int octaveShiftForDial(float dial01) {
    if (dial01 <= 0.0f) return -3;
    if (dial01 >= 1.0f) return 3;
    return static_cast<int>(dial01 * 7.0f) - 3;  // seven positions, -3 to +3
}

float gateWidthForDial(float dial01) {
    if (dial01 < 0.0f) return 0.0f;
    if (dial01 > 1.0f) return 1.0f;
    return dial01;
}

ArpSpeed arpSpeedForDial(float dial01) {
    static const ArpSpeed kSpeeds[5] = {
        {1, 2},  // half
        {1, 4},  // quarter
        {2, 1},  // double
        {3, 2},  // triplet
        {4, 1},  // quad
    };
    return kSpeeds[clampIndex(dial01, 5)];
}

float filterShiftForDial(float dial01) {
    float v = dial01 * 2.0f - 1.0f;
    if (v < -1.0f) return -1.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

int lfoTypeForDial(float dial01) {
    return clampIndex(dial01, kLfoTypeCount);
}

void LoopTrack::setBaseline(float value) {
    baseline_ = value;
}

void LoopTrack::setTouched(bool touched) {
    touched_ = touched;
}

float LoopTrack::valueAt(int step, float liveValue) {
    if (step < 0) step = 0;
    if (step >= kLoopSteps) step = kLoopSteps - 1;
    if (touched_) {
        values_[step] = liveValue;
        valid_[step] = true;
        return liveValue;
    }
    if (persistent_ && valid_[step]) return values_[step];
    return baseline_;
}

void LoopTrack::turnaround() {
    if (touched_) {
        // Still being adjusted at the loop boundary: the recording becomes permanent.
        persistent_ = true;
        return;
    }
    if (!persistent_) {
        // A change that was not held through the turnaround reverts, and nothing is kept.
        for (int i = 0; i < kLoopSteps; i++) valid_[i] = false;
    }
}

void LoopTrack::clear() {
    persistent_ = false;
    for (int i = 0; i < kLoopSteps; i++) valid_[i] = false;
}

bool LoopTrack::hasRecordedStep(int step) const {
    if (step < 0 || step >= kLoopSteps) return false;
    return valid_[step];
}

}  // namespace ctl
