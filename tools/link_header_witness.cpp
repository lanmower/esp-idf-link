#include "link_platform_shim.hpp"

#include <ableton/link/Beats.hpp>
#include <ableton/link/ClientSessionTimelines.hpp>
#include <ableton/link/Phase.hpp>
#include <ableton/link/Tempo.hpp>
#include <ableton/link/Timeline.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace mirror
{
std::int64_t iabs64(std::int64_t value)
{
    return value < 0 ? -value : value;
}

std::int64_t phaseUb(std::int64_t beatsUb, std::int64_t quantumUb)
{
    if (quantumUb == 0) return 0;
    const std::int64_t bins = (iabs64(beatsUb) + quantumUb) / quantumUb;
    return (beatsUb + bins * quantumUb) % quantumUb;
}

std::int64_t nextPhaseMatchUb(std::int64_t xUb, std::int64_t targetUb, std::int64_t quantumUb)
{
    const std::int64_t desired = phaseUb(targetUb, quantumUb);
    const std::int64_t xPhase = phaseUb(xUb, quantumUb);
    return xUb + (desired - xPhase + quantumUb) % quantumUb;
}

std::int64_t closestPhaseMatchUb(std::int64_t xUb, std::int64_t targetUb, std::int64_t quantumUb)
{
    return nextPhaseMatchUb(xUb - quantumUb / 2, targetUb, quantumUb);
}

std::int64_t beatAtTimeUb(double originBeats, std::int64_t timeUs,
                          std::int64_t microsPerBeat, std::int64_t quantumBeats)
{
    const std::int64_t originUb = static_cast<std::int64_t>(originBeats * 1e6);
    const std::int64_t quantumUb = quantumBeats * 1000000;
    const std::int64_t deltaUb = static_cast<std::int64_t>(
        (static_cast<double>(timeUs) / static_cast<double>(microsPerBeat)) * 1e6);
    const std::int64_t beatUb = originUb + deltaUb;
    return closestPhaseMatchUb(beatUb, deltaUb, quantumUb);
}

std::int64_t linearBeatAtTimeUb(double originBeats, std::int64_t timeUs,
                                std::int64_t microsPerBeat)
{
    return static_cast<std::int64_t>(originBeats * 1e6)
        + static_cast<std::int64_t>(
            (static_cast<double>(timeUs) / static_cast<double>(microsPerBeat)) * 1e6);
}
}

namespace
{
constexpr double kOriginBeats[] = {0.0, 16.0, 32.0, 48.0, 7.0, -9.5, 1.0, 64.0, 128.0};
constexpr std::int64_t kTimesUs[] = {0, 125000, 500000, 750000, 1000000, 3750000, 16000000, -500000};
constexpr std::int64_t kQuanta[] = {16, 64};
constexpr double kBpm = 120.0;
constexpr std::int64_t kMicrosPerBeat = 500000;
constexpr std::chrono::microseconds kForceTime{1000000};

std::int64_t modelBeatAtTimeUb(double originBeats, std::int64_t timeUs,
                               std::int64_t microsPerBeat, std::int64_t quantumBeats)
{
#ifdef LINK_HEADER_WITNESS_LINEAR_MODEL
    (void)quantumBeats;
    return mirror::linearBeatAtTimeUb(originBeats, timeUs, microsPerBeat);
#else
    return mirror::beatAtTimeUb(originBeats, timeUs, microsPerBeat, quantumBeats);
#endif
}

ableton::link::Timeline forceBeatAtTimeOnTimeline(ableton::link::Timeline timeline,
                                                  std::chrono::microseconds time,
                                                  std::int64_t quantumBeats)
{
    using namespace ableton::link;
    const Beats quantum{static_cast<double>(quantumBeats)};
    const Beats currentBeatAtTime = toPhaseEncodedBeats(timeline, time, quantum);
    const Beats closestInPhase = closestPhaseMatch(currentBeatAtTime, Beats{0.0}, quantum);
    timeline = shiftClientTimeline(timeline, closestInPhase - currentBeatAtTime);
    timeline.beatOrigin = timeline.beatOrigin + (Beats{0.0} - closestInPhase);
    return timeline;
}
}

int main()
{
    using namespace ableton::link;
    int comparisons = 0;
    int mismatches = 0;
    int linearMismatches = 0;
    int forceReseats = 0;
    int forceMismatches = 0;

    std::printf("SHIMMED SYMBOLS:");
    for (int i = 0; i < kLinkShimSymbolCount; ++i) std::printf(" %s", kLinkShimSymbols[i]);
    std::printf("\n");
    std::printf("SYSTEM BYTE ORDER: htons ntohs htonl ntohl from arpa/inet.h\n");
    std::printf("REAL LINK HEADERS: Beats.hpp Tempo.hpp Timeline.hpp Phase.hpp ClientSessionTimelines.hpp\n");

    for (double origin : kOriginBeats)
    {
        for (std::int64_t quantumBeats : kQuanta)
        {
            for (std::int64_t timeUs : kTimesUs)
            {
                Timeline timeline{Tempo{kBpm}, Beats{origin}, std::chrono::microseconds{0}};
                const std::int64_t realUb =
                    toPhaseEncodedBeats(timeline, std::chrono::microseconds{timeUs},
                                        Beats{static_cast<double>(quantumBeats)}).microBeats();
                const std::int64_t modelUb =
                    modelBeatAtTimeUb(origin, timeUs, kMicrosPerBeat, quantumBeats);
                const std::int64_t linearUb =
                    mirror::linearBeatAtTimeUb(origin, timeUs, kMicrosPerBeat);
                ++comparisons;
                if (realUb != modelUb) ++mismatches;
                if (realUb != linearUb) ++linearMismatches;
            }

            Timeline timeline{Tempo{kBpm}, Beats{origin}, std::chrono::microseconds{0}};
            const Timeline reseated = forceBeatAtTimeOnTimeline(timeline, kForceTime, quantumBeats);
            const std::int64_t realUb =
                toPhaseEncodedBeats(reseated, kForceTime,
                                    Beats{static_cast<double>(quantumBeats)}).microBeats();
            const std::int64_t effectiveTimeUs = kForceTime.count() - reseated.timeOrigin.count();
            const std::int64_t modelUb = modelBeatAtTimeUb(
                reseated.beatOrigin.floating(), effectiveTimeUs, kMicrosPerBeat, quantumBeats);
            ++forceReseats;
            if (realUb != modelUb) ++forceMismatches;
        }
    }

    std::printf("toPhaseEncodedBeats: %d comparisons, %d mismatches\n", comparisons, mismatches);
    std::printf("forceBeatAtTime: %d reseats, %d mismatches\n", forceReseats, forceMismatches);
    std::printf("negative control: linear model differs in %d of %d comparisons\n",
                linearMismatches, comparisons);

    const bool shimmed = kLinkShimSymbolCount == 2;
    const bool negativeControlRejected = linearMismatches > 0;
    const bool pass = shimmed && negativeControlRejected && mismatches == 0 && forceMismatches == 0;
    std::printf("link header witness: %s\n", pass ? "PASS" : "FAIL");
    return pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
