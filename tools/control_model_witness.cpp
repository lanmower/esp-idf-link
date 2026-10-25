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

static void checkScales() {
    CHECK("friendly list has six scales", kFriendlyScaleCount == 6);
    CHECK("aeolian is friendly", isFriendlyScale(1));
    CHECK("phrygian is friendly", isFriendlyScale(2));
    CHECK("minorPentatonic is friendly", isFriendlyScale(3));
    CHECK("dorian is not on the friendly list", !isFriendlyScale(0));
    CHECK("melodicMinor is not on the friendly list", !isFriendlyScale(4));
    CHECK("mixolydian is not on the friendly list", !isFriendlyScale(5));
    bool allFriendly = true;
    for (int d = 0; d <= 100; d++) {
        if (!isFriendlyScale(friendlyScaleForDial(d / 100.0f))) allFriendly = false;
    }
    CHECK("every dial position maps to a friendly scale", allFriendly);
    CHECK("dial 0 is the first friendly scale", friendlyScaleForDial(0.0f) == kFriendlyScales[0]);
    CHECK("dial 1 is the last friendly scale", friendlyScaleForDial(1.0f) == kFriendlyScales[kFriendlyScaleCount - 1]);
}

static void checkStepPatterns() {
    CHECK("32 step patterns", kStepPatternCount == 32);
    bool countOk = true;
    for (int i = 0; i < kStepPatternCount; i++) {
        if (popcount32(stepPattern(i)) != i + 1) countOk = false;
    }
    CHECK("pattern i has i+1 onsets (Euclidean E(i+1, 32))", countOk);
    CHECK("E(2,32) puts onsets on steps 0 and 16", stepPattern(1) == ((1u << 0) | (1u << 16)));
    CHECK("pattern 31 is all onsets", stepPattern(31) == 0xFFFFFFFFu);
    bool distinct = true;
    for (int i = 0; i < kStepPatternCount; i++)
        for (int j = i + 1; j < kStepPatternCount; j++)
            if (stepPattern(i) == stepPattern(j)) distinct = false;
    CHECK("all 32 patterns are distinct", distinct);
    CHECK("dial 0 selects pattern 0", stepPatternForDial(0.0f) == 0);
    CHECK("dial 1 selects pattern 31", stepPatternForDial(1.0f) == 31);
}

static void checkTranceGates() {
    CHECK("32 gate series", kGateSeriesCount == 32);
    bool repetitive = true;
    for (int s = 0; s < 8; s++) {
        const uint32_t g = tranceGate(s, 7u);
        for (int i = 0; i < 24; i++)
            if (((g >> i) & 1u) != ((g >> (i + 8)) & 1u)) repetitive = false;
    }
    CHECK("series 0 to 7 repeat every 8 steps", repetitive);
    CHECK("series 8 is polyrhythmic E(3,32), 3 onsets", popcount32(tranceGate(8, 0u)) == 3);
    CHECK("series 23 is polyrhythmic E(18,32), 18 onsets", popcount32(tranceGate(23, 0u)) == 18);
    CHECK("random series is deterministic for a seed", tranceGate(27, 1234u) == tranceGate(27, 1234u));
    CHECK("a new seed gives a new random pattern", tranceGate(27, 1234u) != tranceGate(27, 1235u));
    CHECK("random density rises with the index", popcount32(tranceGate(31, 9u)) > popcount32(tranceGate(24, 9u)));
    CHECK("dial 0 selects series 0", tranceGateIndexForDial(0.0f) == 0);
    CHECK("dial 1 selects series 31", tranceGateIndexForDial(1.0f) == 31);
}

static void checkDials() {
    CHECK("octave dial 0 is -3", octaveShiftForDial(0.0f) == -3);
    CHECK("octave dial 1 is +3", octaveShiftForDial(1.0f) == 3);
    bool inRange = true;
    int prev = -99;
    bool monotone = true;
    for (int d = 0; d <= 100; d++) {
        const int o = octaveShiftForDial(d / 100.0f);
        if (o < -3 || o > 3) inRange = false;
        if (o < prev) monotone = false;
        prev = o;
    }
    CHECK("octave shift stays in -3 to +3", inRange);
    CHECK("octave shift is monotone in the dial", monotone);
    CHECK("gate width 0 is nothing", gateWidthForDial(0.0f) == 0.0f);
    CHECK("gate width 1 is 100%", gateWidthForDial(1.0f) == 1.0f);
    ArpSpeed half = arpSpeedForDial(0.0f);
    ArpSpeed quad = arpSpeedForDial(1.0f);
    CHECK("arp speed dial 0 is half", half.numerator == 1 && half.denominator == 2);
    CHECK("arp speed dial 1 is quad", quad.numerator == 4 && quad.denominator == 1);
    bool triplet = false;
    for (int d = 0; d <= 100; d++) {
        const ArpSpeed a = arpSpeedForDial(d / 100.0f);
        if (a.numerator == 3 && a.denominator == 2) triplet = true;
    }
    CHECK("arp speed list includes triplets", triplet);
    CHECK("filter shift dial 0 is -100%", filterShiftForDial(0.0f) == -1.0f);
    CHECK("filter shift dial 0.5 is centred", filterShiftForDial(0.5f) == 0.0f);
    CHECK("filter shift dial 1 is +100%", filterShiftForDial(1.0f) == 1.0f);
    CHECK("LFO type dial 1 is the last type", lfoTypeForDial(1.0f) == kLfoTypeCount - 1);
}

static void checkLoopNotTouched() {
    LoopTrack t;
    t.setBaseline(0.5f);
    CHECK("untouched setting reads its baseline", t.valueAt(3, 0.9f) == 0.5f);
    t.turnaround();
    CHECK("untouched setting stays off the record", !t.persistent());
    CHECK("after a pass with no touch, reads the baseline", t.valueAt(3, 0.9f) == 0.5f);
}

static void checkLoopTouchPersists() {
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

static void checkLoopReleaseReverts() {
    LoopTrack t;
    t.setBaseline(0.25f);
    t.setTouched(true);
    for (int s = 0; s < 6; s++) t.valueAt(s, 0.9f);
    t.setTouched(false);  // released before the turnaround
    t.turnaround();
    CHECK("a change released before the turnaround does not persist", !t.persistent());
    CHECK("it reverts to the baseline on the next pass", t.valueAt(2, 0.25f) == 0.25f);
}

static void checkLoopOverwrite() {
    LoopTrack t;
    t.setBaseline(0.0f);
    t.setTouched(true);
    for (int s = 0; s < 8; s++) t.valueAt(s, 0.3f);
    t.turnaround();
    t.setTouched(true);  // second pass: the setting is touched again over steps 4 to 7
    for (int s = 4; s < 8; s++) t.valueAt(s, 0.7f);
    t.setTouched(false);  // released; the reads below must not write to the track
    CHECK("touching a persistent setting overwrites the recorded position", t.valueAt(5, 0.0f) == 0.7f);
    CHECK("untouched positions keep their earlier recording", t.valueAt(1, 0.0f) == 0.3f);
    CHECK("the overwritten value replays on later passes", t.valueAt(6, 0.0f) == 0.7f);
}

static void checkLoopClear() {
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
    checkScales();
    checkStepPatterns();
    checkTranceGates();
    checkDials();
    checkLoopNotTouched();
    checkLoopTouchPersists();
    checkLoopReleaseReverts();
    checkLoopOverwrite();
    checkLoopClear();
    std::printf("control model witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
