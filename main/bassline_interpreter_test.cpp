// Host-only sanity/stress test for bassline_interpreter.{h,cpp}.
// Not part of the ESP-IDF build (not listed in CMakeLists.txt SRCS) --
// build and run manually with plain g++, since no on-device toolchain is
// available in most dev sandboxes:
//
//   g++ -std=c++17 -Wall -Wextra -O2 bassline_interpreter.cpp bassline_interpreter_test.cpp -o /tmp/bli_test && /tmp/bli_test
//
// Exercises every dial at its extremes and across a dense random sweep,
// asserting every emitted MIDI-ish value stays in range and that no
// motif/turnaround combination crashes or hangs (the DP solver's inner
// loops are bounded by kMaxCandidates/kStepsPerBar so a hang would
// indicate a real logic bug, not slowness).
#include "bassline_interpreter.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <initializer_list>

// Simple deterministic PRNG (mulberry32) for host testing -- the firmware
// itself plugs esp_random() into this same RngSource interface.
struct HostRng : bli::RngSource {
    uint32_t state;
    explicit HostRng(uint32_t seed) : state(seed) {}
    float next01() override {
        state += 0x6D2B79F5u;
        uint32_t t = state;
        t = (t ^ (t >> 15)) * (t | 1u);
        t ^= t + (t ^ (t >> 7)) * (t | 61u);
        return static_cast<float>((t ^ (t >> 14)) & 0xFFFFFFFFu) / 4294967296.f;
    }
};

static int g_checks = 0;
static int g_fails = 0;

static void check(bool cond, const char* msg) {
    g_checks++;
    if (!cond) {
        g_fails++;
        std::printf("FAIL: %s\n", msg);
    }
}

static void validateMotif(const bli::Step m[bli::kStepsPerBar], int root, const char* ctx) {
    int noteCount = 0;
    for (int i = 0; i < bli::kStepsPerBar; i++) {
        const auto& s = m[i];
        if (s.note < 0) continue;
        noteCount++;
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s step %d note=%d in [-24,48] range of root", ctx, i, s.note - root);
        check(s.note - root >= -24 && s.note - root <= 48, buf);
        std::snprintf(buf, sizeof(buf), "%s step %d vel=%d in [1,127]", ctx, i, s.vel);
        check(s.vel >= 1 && s.vel <= 127, buf);
        std::snprintf(buf, sizeof(buf), "%s step %d fcc=%d in [0,127]", ctx, i, s.fcc);
        check(s.fcc >= 0 && s.fcc <= 127, buf);
        std::snprintf(buf, sizeof(buf), "%s step %d len>0", ctx, i);
        check(s.len > 0.f, buf);
        std::snprintf(buf, sizeof(buf), "%s step %d pb in [-4,4]", ctx, i);
        check(s.pb >= -4.f && s.pb <= 4.f, buf);
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s produced at least 1 note", ctx);
    check(noteCount > 0, buf);
}

int main() {
    const int root = 40;  // E2, matches the firmware's fixed ROOT

    // --- 1. Every dial pinned at 0.0 and 1.0, one at a time, others default ---
    for (int bank = 0; bank < bli::BANK_COUNT; bank++) {
        for (int dialIdx = 0; dialIdx < 2; dialIdx++) {
            for (float extreme : {0.0f, 1.0f}) {
                bli::Dials d;
                d.set(bank, dialIdx, extreme);
                for (int scaleIdx = 0; scaleIdx < bli::kNumScales; scaleIdx++) {
                    HostRng rng(1000u + bank * 100 + dialIdx * 10 + scaleIdx);
                    bli::Step m[bli::kStepsPerBar];
                    bli::generateMotif(m, root, scaleIdx, d, rng);
                    char ctx[64];
                    std::snprintf(ctx, sizeof(ctx), "bank%d/dial%d=%.0f scale%d", bank, dialIdx, extreme, scaleIdx);
                    validateMotif(m, root, ctx);

                    bli::TurnNote tn[3];
                    int tnCount = bli::generateTurnaround(tn, root, scaleIdx, d, rng);
                    check(tnCount > 0 && tnCount <= 3, "turnaround count in (0,3]");
                    for (int i = 0; i < tnCount; i++) {
                        check(tn[i].note - root >= -24 && tn[i].note - root <= 24, "turnaround note in range");
                        check(tn[i].vel >= 1 && tn[i].vel <= 127, "turnaround vel in range");
                        check(tn[i].fcc >= 0 && tn[i].fcc <= 127, "turnaround fcc in range");
                    }
                }
            }
        }
    }
    std::printf("Extremes sweep: %d checks, %d fails\n", g_checks, g_fails);

    // --- 2. Dense random sweep across the full 8-dial space ---
    int preSweepFails = g_fails;
    for (int trial = 0; trial < 2000; trial++) {
        HostRng seedRng(9000u + trial);
        bli::Dials d;
        d.harmonyGravity  = seedRng.next01();
        d.harmonyColor    = seedRng.next01();
        d.grooveEnergy    = seedRng.next01();
        d.grooveSwing     = seedRng.next01();
        d.motionContour   = seedRng.next01();
        d.motionVariation = seedRng.next01();
        d.voiceSweep      = seedRng.next01();
        d.voiceArtic      = seedRng.next01();

        int scaleIdx = d.pickScaleIdx();
        HostRng rng(50000u + trial);
        bli::Step m[bli::kStepsPerBar];
        bli::generateMotif(m, root, scaleIdx, d, rng);
        char ctx[64];
        std::snprintf(ctx, sizeof(ctx), "random trial %d", trial);
        validateMotif(m, root, ctx);
    }
    std::printf("Random sweep (2000 trials): %d new fails\n", g_fails - preSweepFails);

    // --- 3. get()/set() round-trip for all 8 (bank, dialIdx) slots ---
    {
        bli::Dials d;
        for (int bank = 0; bank < bli::BANK_COUNT; bank++) {
            for (int dialIdx = 0; dialIdx < 2; dialIdx++) {
                d.set(bank, dialIdx, 0.73f);
                check(std::abs(d.get(bank, dialIdx) - 0.73f) < 1e-5f, "dial get/set round-trip");
            }
        }
    }

    std::printf("\nTOTAL: %d checks, %d fails\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
