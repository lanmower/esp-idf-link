#include "link_platform_shim.hpp"

#include <ableton/link/Beats.hpp>
#include <ableton/link/ClientSessionTimelines.hpp>
#include <ableton/link/Phase.hpp>
#include <ableton/link/Tempo.hpp>
#include <ableton/link/Timeline.hpp>

#include "ltmp_request.h"
#include "phrase_boundary_guard.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
constexpr double kMicrobeatsPerBeat = 1e6;
constexpr double kMicrosPerMinute = 60000000.0;
constexpr double kTempoMinBpm = 20.0;
constexpr double kTempoMaxBpm = 999.0;

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

constexpr FirstTakeCase kFirstTakeCases[] = {
    {7.0, 1250000},
    {1.0, 625000},
    {-9.5, 2375000},
    {48.0, 1500000},
    {128.0, 750000},
};
constexpr int kFirstTakeCaseCount =
    static_cast<int>(sizeof(kFirstTakeCases) / sizeof(kFirstTakeCases[0]));

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
    check("shipped listener divides quantumUb by MICROBEATS_PER_BEAT",
          contains(sync, "quantum = (double)quantumUb / MICROBEATS_PER_BEAT;"));
    check("shipped listener publishes the parsed phase pair",
          contains(sync, "ltmpPublish(s_ltmpRequest, s_ltmpGate, hasTempo, bpm, hasPhase, beat0us, quantum);"));
    check("shipped MICROBEATS_PER_BEAT is 1e6",
          contains(sync, "MICROBEATS_PER_BEAT          = 1e6;"));
}

void firstTakeOriginChecks()
{
    using namespace ableton::link;
    const double sharedGridReseats[] = {kLinkQuantum, kPhraseBeats};
    const int caseCount = kFirstTakeCaseCount;
    int fourBeatMisses = 0;

    for (const FirstTakeCase& c : kFirstTakeCases) {
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

struct LtmpWireResult
{
    bool hasTempo;
    double bpm;
    bool hasPhase;
    std::int64_t beat0Us;
    double quantum;
};

std::array<std::uint8_t, 28> makeLtmp28(std::int64_t microsPerBeat,
                                        std::int64_t beat0Us, double quantum)
{
    std::array<std::uint8_t, 28> buf{};
    std::memcpy(buf.data(), "LTMP", 4);
    std::memcpy(buf.data() + 4, &microsPerBeat, 8);
    std::memcpy(buf.data() + 12, &beat0Us, 8);
    const std::int64_t quantumUb = static_cast<std::int64_t>(quantum * kMicrobeatsPerBeat);
    std::memcpy(buf.data() + 20, &quantumUb, 8);
    return buf;
}

LtmpWireResult parseLtmpPacket(const std::uint8_t* buf, std::size_t n)
{
    LtmpWireResult r{false, 0.0, false, 0, 4.0};
    if (n < 4 || std::memcmp(buf, "LTMP", 4) != 0) return r;
    if (n >= 12) {
        std::int64_t mpb = 0;
        std::memcpy(&mpb, buf + 4, 8);
        if (mpb > 0) {
            r.bpm = kMicrosPerMinute / static_cast<double>(mpb);
            r.hasTempo = r.bpm >= kTempoMinBpm && r.bpm <= kTempoMaxBpm;
        }
        if (n >= 28) {
            std::memcpy(&r.beat0Us, buf + 12, 8);
            std::int64_t quantumUb = 0;
            std::memcpy(&quantumUb, buf + 20, 8);
            if (quantumUb > 0) {
                r.quantum = static_cast<double>(quantumUb) / kMicrobeatsPerBeat;
                r.hasPhase = true;
            }
        }
    }
    return r;
}

ableton::link::Timeline applySetTempo(ableton::link::Timeline timeline, double bpm,
                                      std::chrono::microseconds atTime)
{
    using namespace ableton::link;
    const Timeline desired{Tempo{bpm}, timeline.toBeats(atTime), atTime};
    timeline.tempo = desired.tempo;
    timeline.timeOrigin = desired.fromBeats(timeline.beatOrigin);
    return timeline;
}

void ltmpReseatChecks()
{
    using namespace ableton::link;
    const int failuresBefore = g_failures;
    const double quanta[] = {kLinkQuantum, kPhraseBeats};
    int wireCases = 0;
    int fourBeatMisses = 0;

    const auto shortShape = makeLtmp28(kMicrosPerBeat, 1250000, kLinkQuantum);
    check("12-byte LTMP shape has no phase reseat",
          !parseLtmpPacket(shortShape.data(), 12).hasPhase);
    const auto zeroQuantum = makeLtmp28(kMicrosPerBeat, 1250000, 0.0);
    check("zero quantumUb has no phase reseat",
          !parseLtmpPacket(zeroQuantum.data(), 28).hasPhase);

    for (const FirstTakeCase& c : kFirstTakeCases) {
        const std::chrono::microseconds takeTime{c.takeTimeUs};
        const Timeline raw{Tempo{kBpm}, Beats{c.originBeats}, std::chrono::microseconds{0}};
        for (double quantum : quanta) {
            const auto wire = makeLtmp28(kMicrosPerBeat, c.takeTimeUs, quantum);
            const LtmpWireResult parsed = parseLtmpPacket(wire.data(), wire.size());
            check("28-byte LTMP has a tempo request", parsed.hasTempo);
            check("28-byte LTMP has a phase request", parsed.hasPhase);
            check("28-byte LTMP preserves beat0us", parsed.beat0Us == c.takeTimeUs);
            check("28-byte LTMP honours the wire quantum", parsed.quantum == quantum);

            LtmpRequest request;
            LtmpGate gate;
            ltmpPublish(request, gate, parsed.hasTempo, parsed.bpm,
                        parsed.hasPhase, parsed.beat0Us, parsed.quantum);
            const LtmpRequest snap = ltmpTakeSnapshot(request, gate);
            check("snapshot keeps the phase request", snap.phasePending);
            check("snapshot keeps beat0us", snap.phaseBeat0Us == c.takeTimeUs);
            check("snapshot keeps the wire quantum", snap.phaseQuantum == quantum);
            check("snapshot clears pending flags",
                  !request.phasePending && !request.tempoPending);

            const Timeline afterTempo = applySetTempo(raw, snap.tempoBpm, takeTime);
            const std::int64_t tempoShiftUs = (afterTempo.timeOrigin - raw.timeOrigin).count();
            check("matching tempo leaves timeOrigin unchanged",
                  tempoShiftUs >= -1 && tempoShiftUs <= 1);
            const Timeline reseated = reseatOriginAtTake(
                afterTempo, std::chrono::microseconds{snap.phaseBeat0Us}, snap.phaseQuantum);
            const double phraseAtTake =
                toPhaseEncodedBeats(reseated, takeTime, Beats{kPhraseBeats}).floating();
            check("LTMP reseat puts phraseBeat 0 on the take's beat 0",
                  std::fabs(phraseAtTake) < 1e-6);
            const auto crossings = forwardPhraseCrossings(
                reseated, c.takeTimeUs - 70 * kMicrosPerBeat,
                c.takeTimeUs + 200 * kMicrosPerBeat);
            const PhraseCrossing* atTake = nearestCrossing(crossings, c.takeTimeUs);
            const double offset = crossingOffsetBeats(atTake, c.takeTimeUs);
            check("LTMP reseat boundary lands on the take's beat 0",
                  std::fabs(offset) <= 0.002);
            check("LTMP reseat SPP is 0 at the take's beat 0",
                  atTake != nullptr && std::fabs(atTake->sppUnits) < 1e-6);
            for (int n = 1; n <= 3; ++n) {
                const PhraseCrossing* next = crossingNear(
                    crossings, c.takeTimeUs, n * kPhraseBeats);
                check("LTMP reseat keeps the 64-beat SPP grid",
                      next != nullptr
                      && std::fabs(next->sppUnits - n * kPhraseBeats * 4.0) < 1e-6);
            }
            ++wireCases;
            std::printf("  [LTMP 28B] origin %7.2f q=%2.0f: boundary %+6.3f beats, SPP %5.0f\n",
                        c.originBeats, quantum, offset,
                        atTake == nullptr ? -1.0 : atTake->sppUnits);
        }
        const auto fourWire = makeLtmp28(kMicrosPerBeat, c.takeTimeUs, 4.0);
        const LtmpWireResult fourParsed = parseLtmpPacket(fourWire.data(), fourWire.size());
        const Timeline fourReseat = reseatOriginAtTake(raw, takeTime, fourParsed.quantum);
        const auto fourCrossings = forwardPhraseCrossings(
            fourReseat, c.takeTimeUs - 70 * kMicrosPerBeat,
            c.takeTimeUs + 200 * kMicrosPerBeat);
        if (std::fabs(crossingOffsetBeats(
                nearestCrossing(fourCrossings, c.takeTimeUs), c.takeTimeUs)) > 0.002) {
            ++fourBeatMisses;
        }
    }
    check("4-beat wire quantum is not a shared-grid origin", fourBeatMisses > 0);
    std::printf("28-byte LTMP reseat: %d wire cases, %d failures\n",
                wireCases, g_failures - failuresBefore);
    std::printf("LTMP QUANTUM HONOURED: %s\n",
                g_failures == failuresBefore ? "PASS" : "FAIL");
}
}

int main()
{
    parityChecks();
    firstTakeOriginChecks();
    ltmpReseatChecks();
    std::printf("first take origin: %d checks, %d failures\n", g_checks, g_failures);
    std::printf("FIRST-TAKE ORIGIN WITNESS: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
