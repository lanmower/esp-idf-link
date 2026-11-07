#include "link_platform_shim.hpp"

#include <ableton/link/Beats.hpp>
#include <ableton/link/ClientSessionTimelines.hpp>
#include <ableton/link/Phase.hpp>
#include <ableton/link/Tempo.hpp>
#include <ableton/link/Timeline.hpp>

#include "phrase_boundary_guard.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr std::int64_t kMicrosPerBeat = 500000;
constexpr double kBpm = 120.0;
constexpr double kPhraseBeats = 64.0;
constexpr double kLinkQuantum = 16.0;
constexpr std::int64_t kScanStepUs = 1000;

struct FirstTakeCase
{
    double originBeats;
    std::int64_t takeTimeUs;
};

struct PhraseCrossing
{
    std::int64_t timeUs;
    double phraseBeat;
    double sppUnits;
};

int g_checks = 0;
int g_failures = 0;

void check(const char* label, bool condition)
{
    ++g_checks;
    if (!condition) {
        ++g_failures;
        std::printf("FAIL: %s\n", label);
    }
}

std::string readSource(const char* path)
{
    std::ifstream in(path);
    if (!in) {
        std::printf("FAIL: cannot read %s\n", path);
        ++g_failures;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool contains(const std::string& haystack, const char* needle)
{
    return haystack.find(needle) != std::string::npos;
}

ableton::link::Timeline reseatOriginAtTake(ableton::link::Timeline timeline,
                                           std::chrono::microseconds time,
                                           double quantumBeats)
{
    using namespace ableton::link;
    const Beats quantum{quantumBeats};
    const Beats current = toPhaseEncodedBeats(timeline, time, quantum);
    const Beats closest = closestPhaseMatch(current, Beats{0.0}, quantum);
    timeline = shiftClientTimeline(timeline, closest - current);
    timeline.beatOrigin = timeline.beatOrigin + (Beats{0.0} - closest);
    return timeline;
}

std::vector<PhraseCrossing> forwardPhraseCrossings(const ableton::link::Timeline& timeline,
                                                   std::int64_t fromUs, std::int64_t toUs)
{
    using namespace ableton::link;
    const Beats quantum{kPhraseBeats};
    std::vector<PhraseCrossing> crossings;
    bool baseline = false;
    int last = 0;
    for (std::int64_t t = fromUs; t <= toUs; t += kScanStepUs) {
        const double phraseBeat =
            toPhaseEncodedBeats(timeline, std::chrono::microseconds{t}, quantum).floating();
        const int current = static_cast<int>(std::floor(phraseBeat / kPhraseBeats));
        const PhraseBoundaryStep step = stepPhraseBoundary(baseline, last, current);
        baseline = true;
        last = step.lastPhraseNumber;
        if (step.crossed) {
            double units = std::fmod(phraseBeat * 4.0, 16384.0);
            if (units < 0.0) units += 16384.0;
            crossings.push_back({t, phraseBeat, units});
        }
    }
    return crossings;
}

const PhraseCrossing* nearestCrossing(const std::vector<PhraseCrossing>& crossings,
                                      std::int64_t takeUs)
{
    const PhraseCrossing* best = nullptr;
    double bestDistance = 1e30;
    for (const PhraseCrossing& x : crossings) {
        const double d = std::fabs(static_cast<double>(x.timeUs - takeUs));
        if (d < bestDistance) {
            bestDistance = d;
            best = &x;
        }
    }
    return best;
}

const PhraseCrossing* crossingNear(const std::vector<PhraseCrossing>& crossings,
                                   std::int64_t takeUs, double offsetBeats)
{
    const PhraseCrossing* best = nullptr;
    double bestDistance = 1e30;
    for (const PhraseCrossing& x : crossings) {
        const double d = std::fabs(
            static_cast<double>(x.timeUs - takeUs) / static_cast<double>(kMicrosPerBeat)
            - offsetBeats);
        if (d < bestDistance) {
            bestDistance = d;
            best = &x;
        }
    }
    return bestDistance <= 0.01 ? best : nullptr;
}

double crossingOffsetBeats(const PhraseCrossing* crossing, std::int64_t takeUs)
{
    if (crossing == nullptr) return 1e30;
    return static_cast<double>(crossing->timeUs - takeUs) / static_cast<double>(kMicrosPerBeat);
}

void parityChecks()
{
    const std::string sync = readSource("main/link_sync.cpp");
    const std::string mainHeader = readSource("main/main.h");
    const std::string guard = readSource("main/phrase_boundary_guard.h");
    check("shipped main.h keeps PHRASE_BEATS 64.0",
          contains(mainHeader, "#define PHRASE_BEATS 64.0"));
    check("shipped main.h keeps LINK_QUANTUM 16.0",
          contains(mainHeader, "#define LINK_QUANTUM 16.0"));
    check("shipped link_sync.cpp reads phraseBeat at PHRASE_BEATS",
          contains(sync, "info.phraseBeat = state.beatAtTime(time, PHRASE_BEATS);"));
    check("shipped link_sync.cpp derives the phrase number from phraseBeat",
          contains(sync, "info.currentPhraseNumber = static_cast<int>(std::floor(info.phraseBeat / PHRASE_BEATS));"));
    check("shipped link_sync.cpp uses the monotonic phrase guard",
          contains(sync, "stepPhraseBoundary("));
    check("shipped guard only crosses on a higher phrase number",
          contains(guard, "if (currentPhraseNumber > lastPhraseNumber)"));
    check("shipped link_sync.cpp sends SPP from phraseBeat",
          contains(sync, "send_song_position(quantumInfo.phraseBeat);"));
    check("shipped link_sync.cpp wraps SPP with fmod",
          contains(sync, "std::fmod(sessionBeat * MIDI_SPP_UNITS_PER_BEAT, kSppWrap)"));
    check("shipped LTMP reseat uses the looper quantum",
          contains(sync, "ss.forceBeatAtTime(0.0, std::chrono::microseconds(req.phaseBeat0Us), req.phaseQuantum);"));
}

void firstTakeOriginChecks()
{
    using namespace ableton::link;
    const FirstTakeCase cases[] = {
        {7.0, 1250000},
        {1.0, 625000},
        {-9.5, 2375000},
        {48.0, 1500000},
        {128.0, 750000},
    };
    const double sharedGridReseats[] = {kLinkQuantum, kPhraseBeats};
    const int caseCount = static_cast<int>(sizeof(cases) / sizeof(cases[0]));
    int fourBeatMisses = 0;

    for (const FirstTakeCase& c : cases) {
        const std::chrono::microseconds takeTime{c.takeTimeUs};
        const Timeline raw{Tempo{kBpm}, Beats{c.originBeats}, std::chrono::microseconds{0}};
        const double takeBeat = raw.toBeats(takeTime).floating();
        const double gridPhase = std::fabs(std::fmod(takeBeat, kLinkQuantum));
        check("take beat 0 is off the shared 16-beat grid",
              gridPhase > 0.25 && gridPhase < 15.75);

        const auto rawCrossings = forwardPhraseCrossings(
            raw, c.takeTimeUs - 70 * kMicrosPerBeat, c.takeTimeUs + 200 * kMicrosPerBeat);
        const double rawOffset = crossingOffsetBeats(
            nearestCrossing(rawCrossings, c.takeTimeUs), c.takeTimeUs);
        check("uncorrected off-grid origin misses the take's beat 0",
              std::fabs(rawOffset) > 0.5);
        std::printf("  [first take] origin %7.2f takeBeat %8.3f raw boundary %+8.3f beats\n",
                    c.originBeats, takeBeat, rawOffset);

        for (double reseatQuantum : sharedGridReseats) {
            const Timeline corrected = reseatOriginAtTake(raw, takeTime, reseatQuantum);
            const double phraseAtTake =
                toPhaseEncodedBeats(corrected, takeTime, Beats{kPhraseBeats}).floating();
            check("corrected origin has phraseBeat 0 at the take's beat 0",
                  std::fabs(phraseAtTake) < 1e-6);
            const auto crossings = forwardPhraseCrossings(
                corrected, c.takeTimeUs - 70 * kMicrosPerBeat,
                c.takeTimeUs + 200 * kMicrosPerBeat);
            const PhraseCrossing* atTake = nearestCrossing(crossings, c.takeTimeUs);
            const double offset = crossingOffsetBeats(atTake, c.takeTimeUs);
            check("boundary lands on the take's beat 0", std::fabs(offset) <= 0.002);
            check("SPP is 0 at the take's beat 0",
                  atTake != nullptr && std::fabs(atTake->sppUnits) < 1e-6);
            for (int n = 1; n <= 3; ++n) {
                const PhraseCrossing* next = crossingNear(
                    crossings, c.takeTimeUs, n * kPhraseBeats);
                check("later boundary lands on a 64-beat grid from the take",
                      next != nullptr
                      && std::fabs(next->sppUnits - n * kPhraseBeats * 4.0) < 1e-6);
            }
            std::printf("  [first take] origin %7.2f reseat %2.0f: boundary %+6.3f beats, SPP %5.0f\n",
                        c.originBeats, reseatQuantum, offset,
                        atTake == nullptr ? -1.0 : atTake->sppUnits);
        }

        const Timeline fourBeat = reseatOriginAtTake(raw, takeTime, 4.0);
        const auto fourCrossings = forwardPhraseCrossings(
            fourBeat, c.takeTimeUs - 70 * kMicrosPerBeat,
            c.takeTimeUs + 200 * kMicrosPerBeat);
        const double fourOffset = crossingOffsetBeats(
            nearestCrossing(fourCrossings, c.takeTimeUs), c.takeTimeUs);
        if (std::fabs(fourOffset) > 0.002) ++fourBeatMisses;
    }

    check("4-beat LTMP reseat is not a shared-grid origin", fourBeatMisses > 0);
    std::printf("4-beat reseat misses the take beat 0 in %d of %d cases\n",
                fourBeatMisses, caseCount);
}
}

int main()
{
    parityChecks();
    firstTakeOriginChecks();
    std::printf("first take origin: %d checks, %d failures\n", g_checks, g_failures);
    std::printf("FIRST-TAKE ORIGIN WITNESS: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
