// Host witness for main/control_model.{h,cpp}. Build and run with tools/control-model-witness.sh.
// Each check states the property it witnesses; a failed check exits non-zero with its name.
#include "control_model.h"

#include <cstdio>
#include <cstdlib>

using namespace ctl;

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(name, cond)                                              \
    do {                                                               \
        g_checks++;                                                    \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s\n", name);                            \
            g_fail++;                                                  \
        }                                                              \
    } while (0)

static int popcount32(uint32_t v) {
    int n = 0;
    while (v) { n += v & 1u; v >>= 1; }
    return n;
}

static void checkMappings() {
    CHECK("six friendly scales", kScaleCount == 6);
    CHECK("aeolian has seven degrees", scaleSize(0) == 7);
    CHECK("minor pentatonic has five degrees", scaleSize(2) == 5);
    CHECK("scale 0 dial 0 is the first scale", scaleForDial(0.0f) == 0);
    CHECK("scale dial 1 is the last scale", scaleForDial(1.0f) == kScaleCount - 1);
    CHECK("pitch offset spans -1 to +1 octave", pitchOctaveForDial(0.0f) == -1.0f && pitchOctaveForDial(1.0f) == 1.0f);
    CHECK("pitch offset centre is the root", pitchOctaveForDial(0.5f) == 0.0f);
    CHECK("octave spans -3 to +3", octaveForDial(0.0f) == -3 && octaveForDial(1.0f) == 3);
    CHECK("loop window spans 1 to 8 bars", loopBarsForDial(0.0f) == 1 && loopBarsForDial(1.0f) == 8);
    CHECK("arp periods are half, quarter, double, triplet, quad", arpPeriodTicks(0) == 24 && arpPeriodTicks(1) == 48 && arpPeriodTicks(2) == 12 && arpPeriodTicks(3) == 8 && arpPeriodTicks(4) == 3);
    CHECK("filter shift spans -1 to +1", filterShiftForDial(0.0f) == -1.0f && filterShiftForDial(1.0f) == 1.0f);
    CHECK("LFO width spans -1 to +1", lfoWidthForDial(0.0f) == -1.0f && lfoWidthForDial(1.0f) == 1.0f);
    CHECK("gate width 0 is nothing", gateWidthForDial(0.0f) == 0.0f);
    CHECK("gate width 1 is 100 percent", gateWidthForDial(1.0f) == 1.0f);
    CHECK("LFO type covers four shapes", lfoTypeForDial(1.0f) == 3);
}

static void checkStepPatterns() {
    CHECK("32 rhythm patterns", kRhythmPatternCount == 32);
    bool countOk = true;
    for (int i = 0; i < kRhythmPatternCount; i++) {
        if (popcount32(stepPattern(i)) != i + 1) countOk = false;
    }
    CHECK("pattern i has i+1 onsets", countOk);
    CHECK("pattern 31 is all onsets", stepPattern(31) == 0xFFFFFFFFu);
    CHECK("rhythm dial 1 selects pattern 31", rhythmForDial(1.0f) == 31);
}

static void checkTranceGates() {
    CHECK("32 gate series", kGateSeriesCount == 32);
    bool repetitive = true;
    for (int s = 0; s < 8; s++) {
        const uint32_t g = tranceGate(s, 7u);
        for (int i = 0; i < 24; i++) {
            if (((g >> i) & 1u) != ((g >> (i + 8)) & 1u)) repetitive = false;
        }
    }
    CHECK("series 0 to 7 repeat every 8 steps", repetitive);
    CHECK("series 8 is polyrhythmic with three onsets", popcount32(tranceGate(8, 0u)) == 3);
    CHECK("a random series is deterministic for a seed", tranceGate(27, 1234u) == tranceGate(27, 1234u));
    CHECK("a new seed gives a new random pattern", tranceGate(27, 1234u) != tranceGate(27, 1235u));
    CHECK("random density rises with the index", popcount32(tranceGate(31, 9u)) > popcount32(tranceGate(24, 9u)));
}

static void checkHashes() {
    CHECK("hash is deterministic", hash32(1, 2, 3) == hash32(1, 2, 3));
    CHECK("hash unit stays in [0, 1)", hashUnit(7, 8, 9) >= 0.0f && hashUnit(7, 8, 9) < 1.0f);
}

static void checkUntouchedReadsBaseline() {
    LoopTrack t;
    t.setBaseline(0.5f);
    CHECK("an untouched setting reads its baseline", t.valueAt(3, 0.9f) == 0.5f);
    t.turnaround();
    CHECK("an untouched setting stays off the record", !t.persistent());
}

static void checkHoldUntilTurnaround() {
    LoopTrack t;
    t.setBaseline(0.1f);
    t.setTouched(true);
    t.valueAt(0, 0.9f);
    t.setTouched(false);
    CHECK("a released change holds its last value until the turnaround", t.valueAt(5, 0.1f) == 0.9f);
    t.turnaround();
    CHECK("after the turnaround the change returns to baseline", t.valueAt(5, 0.1f) == 0.1f);
}

static void checkTouchPersists() {
    LoopTrack t;
    t.setBaseline(0.5f);
    t.setTouched(true);
    for (int s = 0; s < 10; s++) t.valueAt(s, 0.8f);
    t.turnaround();  // still touched at the loop boundary, so the recording is kept
    CHECK("touched at the turnaround becomes persistent", t.persistent());
    t.setTouched(false);
    CHECK("a persistent recording replays on the next pass", t.valueAt(4, 0.5f) == 0.8f);
    CHECK("a position never recorded plays the baseline", t.valueAt(20, 0.5f) == 0.5f);
    t.turnaround();
    CHECK("a persistent recording survives an untouched turnaround", t.persistent());
    CHECK("and replays again on the following pass", t.valueAt(9, 0.5f) == 0.8f);
}

static void checkReleaseReverts() {
    LoopTrack t;
    t.setBaseline(0.25f);
    t.setTouched(true);
    for (int s = 0; s < 6; s++) t.valueAt(s, 0.9f);
    t.setTouched(false);  // released before the turnaround
    t.turnaround();
    CHECK("a change released before the turnaround does not persist", !t.persistent());
    CHECK("it reverts to the baseline on the next pass", t.valueAt(2, 0.25f) == 0.25f);
}

static void checkOverwrite() {
    LoopTrack t;
    t.setBaseline(0.0f);
    t.setTouched(true);
    for (int s = 0; s < 8; s++) t.valueAt(s, 0.3f);
    t.turnaround();
    t.setTouched(true);  // second pass: touched again over steps 4 to 7
    for (int s = 4; s < 8; s++) t.valueAt(s, 0.7f);
    t.setTouched(false);
    CHECK("touching a persistent setting overwrites the recorded position", t.valueAt(5, 0.0f) == 0.7f);
    CHECK("untouched positions keep their earlier recording", t.valueAt(1, 0.0f) == 0.3f);
    CHECK("the overwritten value replays on later passes", t.valueAt(6, 0.0f) == 0.7f);
}

static void checkClear() {
    LoopTrack t;
    t.setBaseline(0.1f);
    t.setTouched(true);
    t.valueAt(0, 0.9f);
    t.turnaround();
    t.setTouched(false);
    t.clear();
    CHECK("clear removes the persistent recording", !t.persistent());
    CHECK("clear returns the setting to its baseline", t.valueAt(0, 0.9f) == 0.1f);
}

int main() {
    checkMappings();
    checkStepPatterns();
    checkTranceGates();
    checkHashes();
    checkUntouchedReadsBaseline();
    checkHoldUntilTurnaround();
    checkTouchPersists();
    checkReleaseReverts();
    checkOverwrite();
    checkClear();
    std::printf("control model witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
