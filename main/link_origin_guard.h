#pragma once

#include <cmath>
#include <cstdint>

constexpr double LINK_ORIGIN_WATCH_QUANTUM_BEATS = 1.0e9;
constexpr double LINK_ORIGIN_SHIFT_BEATS = 1.0 / 24.0;

struct LinkOriginWatch {
    bool valid = false;
    double lastBeat = 0.0;
    int64_t lastTimeUs = 0;
    double lastTempoBpm = 0.0;

    bool observe(double beat, int64_t timeUs, double tempoBpm, double thresholdBeats)
    {
        bool shifted = false;
        if (valid && timeUs > lastTimeUs && lastTempoBpm > 0.0) {
            const double expectedBeat = lastBeat + (double)(timeUs - lastTimeUs) * lastTempoBpm / 60e6;
            shifted = std::fabs(beat - expectedBeat) > thresholdBeats;
        }
        valid = true;
        lastBeat = beat;
        lastTimeUs = timeUs;
        lastTempoBpm = tempoBpm;
        return shifted;
    }
};
