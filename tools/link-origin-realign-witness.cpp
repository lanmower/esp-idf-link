#include "link_platform_shim.hpp"

#include <ableton/link/Beats.hpp>
#include <ableton/link/Phase.hpp>
#include <ableton/link/Tempo.hpp>
#include <ableton/link/Timeline.hpp>

#include "link_origin_guard.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>

namespace
{
using ableton::link::Beats;
using ableton::link::Tempo;
using ableton::link::Timeline;

int failures = 0;

void check(const char* name, bool ok)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++failures;
}

Timeline makeTimeline(double bpm, double beatOrigin, std::int64_t timeOriginUs)
{
    return Timeline{Tempo(bpm), Beats{beatOrigin}, std::chrono::microseconds(timeOriginUs)};
}

Timeline setTempoLikeLink(const Timeline& base, double bpm, std::int64_t atUs)
{
    const auto at = std::chrono::microseconds(atUs);
    const Timeline desired{Tempo(bpm), base.toBeats(at), at};
    Timeline out = base;
    out.tempo = desired.tempo;
    out.timeOrigin = desired.fromBeats(base.beatOrigin);
    return out;
}

Timeline forceBeatLikeLink(const Timeline& base, std::int64_t atUs, double shiftBeats)
{
    const auto at = std::chrono::microseconds(atUs);
    return Timeline{base.tempo, Beats{base.toBeats(at).floating() + shiftBeats}, at};
}

struct Sweep {
    int shifts;
    std::int64_t firstUs;
};

Sweep sweep(const std::function<Timeline(std::int64_t)>& timelineAt,
            std::int64_t fromUs, std::int64_t toUs, std::int64_t stepUs)
{
    LinkOriginWatch watch;
    Sweep result{0, -1};
    for (std::int64_t t = fromUs; t <= toUs; t += stepUs) {
        const Timeline tl = timelineAt(t);
        const double beat = ableton::link::toPhaseEncodedBeats(
            tl, std::chrono::microseconds(t), Beats{LINK_ORIGIN_WATCH_QUANTUM_BEATS}).floating();
        if (watch.observe(beat, t, tl.tempo.bpm(), LINK_ORIGIN_SHIFT_BEATS)) {
            if (result.shifts == 0) result.firstUs = t;
            ++result.shifts;
        }
    }
    return result;
}

void runScenarios()
{
    const std::int64_t endUs = 2000000;
    const Timeline base = makeTimeline(120.0, 0.0, 0);

    check("steady 120 BPM timeline: no origin shift over 2 s",
          sweep([&](std::int64_t) { return base; }, 0, endUs, 1000).shifts == 0);

    const Timeline tempoChanged = setTempoLikeLink(base, 174.0, 300000);
    check("setTempo 120 -> 174 keeps the beat continuous: no origin shift",
          sweep([&](std::int64_t t) { return t < 300000 ? base : tempoChanged; },
                0, endUs, 1000).shifts == 0);

    Timeline offsetCorrected = tempoChanged;
    offsetCorrected.timeOrigin = offsetCorrected.timeOrigin + std::chrono::microseconds(3000);
    check("3 ms clock-offset correction at 174 BPM is below one pulse: no origin shift",
          sweep([&](std::int64_t t) { return t < 900000 ? tempoChanged : offsetCorrected; },
                0, endUs, 1000).shifts == 0);

    const std::int64_t shiftUs = 1200000;
    const Timeline forced5 = forceBeatLikeLink(tempoChanged, shiftUs, 5.5);
    const Sweep s5 = sweep([&](std::int64_t t) { return t < shiftUs ? tempoChanged : forced5; },
                           0, endUs, 1000);
    check("origin shift of 5.5 beats without LTMP is detected once, at the shift tick",
          s5.shifts == 1 && s5.firstUs == shiftUs);

    const Timeline forced16 = forceBeatLikeLink(tempoChanged, shiftUs, 16.0);
    const Sweep s16 = sweep([&](std::int64_t t) { return t < shiftUs ? tempoChanged : forced16; },
                            0, endUs, 1000);
    check("origin shift of exactly 16 beats (a quantum multiple) is detected",
          s16.shifts == 1 && s16.firstUs == shiftUs);

    const Timeline forced001 = forceBeatLikeLink(tempoChanged, shiftUs, 0.01);
    const Sweep s001 = sweep([&](std::int64_t t) { return t < shiftUs ? tempoChanged : forced001; },
                             0, endUs, 1000);
    check("forced origin re-anchor of 0.01 beat is flagged once (the origin moved; X restarts at the anchor)",
          s001.shifts == 1 && s001.firstUs == shiftUs);

    const Timeline negativeBase = makeTimeline(120.0, -2.885, 0);
    const Timeline forcedNegative = forceBeatLikeLink(negativeBase, shiftUs, 5.5);
    const Sweep sNeg = sweep([&](std::int64_t t) { return t < shiftUs ? negativeBase : forcedNegative; },
                             0, endUs, 1000);
    check("origin shift from a negative-beat timeline is detected once",
          sNeg.shifts == 1 && sNeg.firstUs == shiftUs);
}
}

int main()
{
    runScenarios();
    if (failures == 0) {
        std::printf("LINK ORIGIN REALIGN WITNESS: PASS (0 failures)\n");
        return 0;
    }
    std::printf("LINK ORIGIN REALIGN WITNESS: FAIL (%d failures)\n", failures);
    return 1;
}
