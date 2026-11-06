#include "ltmp_request.h"

#include <cstdio>
#include <cstdlib>
#include <thread>

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(name, cond)                                 \
    do {                                                  \
        ++g_checks;                                       \
        if (!(cond)) { std::printf("FAIL %s\n", (name)); ++g_fail; } \
    } while (0)

struct LegacyFields {
    bool    tempoPending  = false;
    double  tempoBpm      = 0.0;
    bool    phasePending  = false;
    int64_t phaseBeat0Us  = 0;
    double  phaseQuantum  = 4.0;
};

static void checkLegacyLostRequest() {
    LegacyFields f;
    f.tempoBpm = 120.0;
    f.tempoPending = true;
    const bool seen = f.tempoPending;
    f.tempoPending = false;
    f.tempoBpm = 140.0;
    f.tempoPending = true;
    const double applied = f.tempoBpm;
    CHECK("legacy handler saw the pending request", seen);
    CHECK("legacy loses the first request value", applied != 120.0);
    CHECK("legacy leaves a stale duplicate pending flag", f.tempoPending);
    CHECK("legacy applied the later value without clearing it", applied == 140.0);
}

static void checkLegacyTornPhase() {
    LegacyFields f;
    f.phaseBeat0Us = 1000;
    f.phaseQuantum = 16.0;
    f.phasePending = true;
    f.phasePending = false;
    const int64_t beat0 = f.phaseBeat0Us;
    f.phaseBeat0Us = 2000;
    f.phaseQuantum = 64.0;
    f.phasePending = true;
    const double quantum = f.phaseQuantum;
    CHECK("legacy tears the phase pair", beat0 == 1000 && quantum == 64.0);
    CHECK("legacy torn pair is neither published pair", !(beat0 == 2000 && quantum == 64.0));
}

static void checkSnapshotKeepsRequestsAndPairs() {
    LtmpRequest r;
    LtmpGate gate;
    ltmpPublishTempo(r, gate, 120.0);
    LtmpRequest a = ltmpTakeSnapshot(r, gate);
    CHECK("snapshot A keeps tempo request", a.tempoPending && a.tempoBpm == 120.0);
    CHECK("snapshot clears pending after copying", !r.tempoPending && !r.phasePending);
    ltmpPublishTempo(r, gate, 140.0);
    LtmpRequest b = ltmpTakeSnapshot(r, gate);
    CHECK("snapshot B keeps the later request, no loss", b.tempoPending && b.tempoBpm == 140.0);
    CHECK("request cleared again", !r.tempoPending);

    ltmpPublishPhase(r, gate, 1000, 16.0);
    LtmpRequest p = ltmpTakeSnapshot(r, gate);
    CHECK("phase pair A is consistent", p.phasePending && p.phaseBeat0Us == 1000 && p.phaseQuantum == 16.0);
    ltmpPublishPhase(r, gate, 2000, 64.0);
    LtmpRequest q = ltmpTakeSnapshot(r, gate);
    CHECK("phase pair B is consistent", q.phasePending && q.phaseBeat0Us == 2000 && q.phaseQuantum == 64.0);

    LtmpRequest empty = ltmpTakeSnapshot(r, gate);
    CHECK("empty snapshot has no pending work", !empty.tempoPending && !empty.phasePending);
}

static void checkConcurrentPublishNeverTears() {
    LtmpRequest r;
    LtmpGate gate;
    constexpr int kPublishes = 2000;
    std::atomic<bool> done{false};
    std::thread writer([&]() {
        for (int i = 1; i <= kPublishes; ++i) {
            ltmpPublishRequest(r, gate, static_cast<double>(i),
                               static_cast<int64_t>(i) * 1000, static_cast<double>(i));
        }
        done.store(true);
    });

    int seen = 0;
    int consistent = 0;
    auto take = [&]() {
        LtmpRequest snap = ltmpTakeSnapshot(r, gate);
        if (!snap.tempoPending && !snap.phasePending) return;
        ++seen;
        if (snap.tempoPending && snap.phasePending &&
            snap.phaseBeat0Us == static_cast<int64_t>(snap.tempoBpm) * 1000 &&
            snap.phaseQuantum == snap.tempoBpm) {
            ++consistent;
        }
    };

    while (!done.load()) {
        take();
        std::this_thread::yield();
    }
    writer.join();
    take();
    CHECK("concurrent witness saw at least one publish", seen > 0);
    CHECK("every snapshot is a whole published pair", consistent == seen);
}

int main() {
    checkLegacyLostRequest();
    checkLegacyTornPhase();
    checkSnapshotKeepsRequestsAndPairs();
    checkConcurrentPublishNeverTears();
    std::printf("ltmp request witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
