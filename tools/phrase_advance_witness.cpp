// Host witness for phraseAdvance (main/bass_engine.cpp). Build and run with tools/phrase-advance-witness.sh.
// Each check states the property it witnesses; a failed check exits non-zero with its name.
#include "bass_engine.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

// The engine's MIDI output, stubbed for the host: this witness checks phrase advance, not sends.
void send_midi_message(const uint8_t* message, size_t size) {
    (void)message;
    (void)size;
}
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value) {
    (void)channel;
    (void)cc_num;
    (void)value;
}

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

static constexpr double kPhrase = 1024.0;

int main() {
    const PhraseAdvance same = phraseAdvance(500.0, 500.0, kPhrase);
    CHECK("equal positions advance by zero without wrapping", !same.wrapped && same.steps == 0.0);

    const PhraseAdvance forward = phraseAdvance(500.0, 501.5, kPhrase);
    CHECK("a forward move advances by its distance", !forward.wrapped && forward.steps == 1.5);

    const PhraseAdvance noise = phraseAdvance(500.0, 500.0 - 1e-13, kPhrase);
    CHECK("float noise at zero advance does not wrap the phrase", !noise.wrapped && noise.steps == 0.0);

    const PhraseAdvance noiseAtEnd = phraseAdvance(1023.9999999999999, 1023.9999999999998, kPhrase);
    CHECK("noise just before the phrase end does not wrap", !noiseAtEnd.wrapped && noiseAtEnd.steps == 0.0);

    const PhraseAdvance belowThreshold = phraseAdvance(500.0, 500.0 - 5e-7, kPhrase);
    CHECK("a backward move under the noise threshold is zero advance", !belowThreshold.wrapped && belowThreshold.steps == 0.0);

    const PhraseAdvance wrap = phraseAdvance(1000.0, 10.0, kPhrase);
    CHECK("a real wrap advances by the distance across the phrase end", wrap.wrapped && wrap.steps == 34.0);

    const PhraseAdvance wrapNear = phraseAdvance(1020.0, 4.0, kPhrase);
    CHECK("a wrap close to the phrase end advances the short way round", wrapNear.wrapped && wrapNear.steps == 8.0);

    const PhraseAdvance backward = phraseAdvance(500.0, 499.9, kPhrase);
    CHECK("a backward move past the noise threshold is still a wrap", backward.wrapped);

    const double oldNoiseAdvance = (500.0 - 1e-13) - 500.0;
    const double oldInlineAdvance = oldNoiseAdvance < 0.0 ? oldNoiseAdvance + kPhrase : oldNoiseAdvance;
    CHECK("the pre-fix rule turned that noise into a whole-phrase advance", oldInlineAdvance > 1.0);

    std::printf("phrase advance witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
