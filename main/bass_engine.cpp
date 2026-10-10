#include "main.h"
#include "bass_engine.h"
#include "io_helpers.h"
#include "esp_log.h"
#include "esp_random.h"
#include <algorithm>
#include <cstring>
#include <cmath>

static const char* TAG = "BASS";

static constexpr int   kRootNoteE2             = 40;
static constexpr int   kStepsPerBeat           = 4;
static constexpr int   kBarsPerPhrase          = 64;
static constexpr int   kStepsPerPhrase         = kBarsPerPhrase * bli::kStepsPerBar;
static constexpr int   kSectionsPerPhrase      = 4;
static constexpr int   kBarsPerSection         = kBarsPerPhrase / kSectionsPerPhrase;
static constexpr int   kBarsPerChordCell       = 4;
static constexpr int   kFirstSection           = 0;
static constexpr int   kBridgeSection          = 1;
static constexpr int   kLastSection            = kSectionsPerPhrase - 1;
static constexpr int   kHalfPhraseSteps        = kStepsPerPhrase / 2;
static constexpr double kMaxAdvanceBeatsPerProcess = 1.0;
static constexpr double kMaxAdvanceStepsPerProcess = kMaxAdvanceBeatsPerProcess * kStepsPerBeat;
static constexpr int   kTurnaroundTailSteps    = 8;
static constexpr float kTurnaroundStartStep    = kStepsPerPhrase - kTurnaroundTailSteps;

static constexpr int   kOctaveSemitones        = 12;
static constexpr int   kMaxSemitonesAboveRoot  = bli::kRegisterSpan;
static constexpr int   kMaxSemitonesBelowRoot  = 12;
static constexpr int   kFifthScaleDegreeIndex  = 4;
static constexpr int   kFifthSemitones         = 7;
static constexpr int   kThirdSemitones         = 3;
static constexpr int   kStep0MaxSemitonesFromRoot = 2;
static constexpr int   kMidBarStep             = bli::kStepsPerBar / 2;
static constexpr float kMidBarFifthProbability = 0.4f;
static constexpr int   kSubDropFloorNote       = 36;
static constexpr int   kSubDropFilterCCReduction = 30;
static constexpr int   kFilterCCFloor          = 10;
static constexpr float kSimplifyThinProbability = 0.6f;
static constexpr float kModalShiftProbability  = 0.5f;
static constexpr float kModalShiftFifthProbability = 0.5f;
static constexpr int   kDriftDropBeatMaxIndex  = 3;
static constexpr float kDriftTailProbability   = 0.5f;

static constexpr float kScaleJitterProbability = 0.35f;
static constexpr int   kScaleJitterRange       = 3;
static constexpr int   kScaleHoldMinPhrases    = 2;
static constexpr int   kScaleHoldRangePhrases  = 3;
static constexpr int   kArcPeriodPhrases       = 8;
static constexpr float kMotifBlendProbability  = 0.4f;
static constexpr float kMaxSwingSteps          = 0.4f;
static constexpr float kArticulatedTurnaroundThreshold = 0.6f;
static constexpr int   kFrequentTurnaroundPeriodPhrases = 2;
static constexpr int   kSparseTurnaroundPeriodPhrases   = 4;
static constexpr float kSamePositionEpsilonSteps = 0.01f;
static constexpr double kWrapBackwardThreshold = -200.0;

static constexpr uint8_t kMidiNoteOn         = 0x90;
static constexpr uint8_t kMidiNoteOff        = 0x80;
static constexpr uint8_t kMidiControlChange  = 0xB0;
static constexpr uint8_t kMidiPitchBend      = 0xE0;
static constexpr uint8_t kMidiCCFilter       = 74;
static constexpr uint8_t kMidiCCAllNotesOff  = 123;
static constexpr int     kPitchBendCentreRaw = 8192;
static constexpr int     kPitchBendHalfRange = 8191;
static constexpr int     kPitchBendRawMax    = 16383;
static constexpr float   kPitchBendSemitoneSpan = 12.f;

static int scaleDegreeSemitones(int scIdx, int degree) { return bli::scaleDegree(scIdx, degree); }

enum ChordDegree {
    kDegreeTonic       = 0,
    kDegreeSupertonic  = 1,
    kDegreeMediant     = 2,
    kDegreeSubdominant = 3,
    kDegreeDominant    = 4,
    kDegreeSubmediant  = 5,
    kDegreeLeadingTone = 6,
};

enum ChordProgression {
    kProgTonicSubmediantSubdominantLeadingTone = 0,
    kProgTonicLeadingToneSubmediantLeadingTone = 1,
    kProgTonicSubdominantLeadingToneMediant    = 2,
    kProgDarkCadence                           = 3,
    kProgTonicMediantLeadingToneSubdominant    = 4,
    kProgTonicPedalSubmediantLeadingTone       = 5,
    kNumProgs                                  = 6,
};

static const int kChordDegreeProgs[kNumProgs][kBarsPerChordCell] = {
    {kDegreeTonic, kDegreeSubmediant, kDegreeSubdominant, kDegreeLeadingTone},
    {kDegreeTonic, kDegreeLeadingTone, kDegreeSubmediant, kDegreeLeadingTone},
    {kDegreeTonic, kDegreeSubdominant, kDegreeLeadingTone, kDegreeMediant},
    {kDegreeTonic, kDegreeSubmediant, kDegreeLeadingTone, kDegreeDominant},
    {kDegreeTonic, kDegreeMediant, kDegreeLeadingTone, kDegreeSubdominant},
    {kDegreeTonic, kDegreeTonic, kDegreeSubmediant, kDegreeLeadingTone},
};

static float rand01() {
    return (float)(esp_random() & 0xFFFFFF) / (float)0x1000000;
}
static int randBelow(int n) {
    if (n <= 1) return 0;
    return (int)(esp_random() % (uint32_t)n);
}
static bool randChance(float prob) { return rand01() < prob; }

BassEngine::MS BassEngine::mn(int note, float len, int vel, int fcc,
                               float pbSemitones, float tsBeats) {
    return {note, len, vel, fcc, pbSemitones, tsBeats};
}

void BassEngine::msInit(MS m[bli::kStepsPerBar]) {
    for (int i = 0; i < bli::kStepsPerBar; i++) m[i] = {kEmptyNote, 0.f, 0, 0, 0.f, 0.f};
}

struct EspRng : bli::RngSource {
    float next01() override { return rand01(); }
};

void BassEngine::genInterpreted(MS m[bli::kStepsPerBar], int root, int scIdx) {
    EspRng rng;
    bli::Step steps[bli::kStepsPerBar];
    bli::generateMotif(steps, root, scIdx, m_dials, rng);
    for (int i = 0; i < bli::kStepsPerBar; i++) {
        if (steps[i].note < 0) {
            m[i] = {kEmptyNote, 0.f, 0, 0, 0.f, 0.f};
        } else {
            m[i] = mn(steps[i].note, steps[i].len, steps[i].vel, steps[i].fcc, steps[i].pb);
        }
    }
}

void BassEngine::anchorMotif(MS m[bli::kStepsPerBar], int root, int scIdx) {
    int fifthInterval = scaleDegreeSemitones(scIdx, kFifthScaleDegreeIndex);

    if (m[0].note < 0 || std::abs(m[0].note - root) > kStep0MaxSemitonesFromRoot)
        m[0] = mn(root, 0.5f, 120, 90);

    if (m[kMidBarStep].note < 0)
        m[kMidBarStep] = mn(root + (randChance(kMidBarFifthProbability) ? fifthInterval : 0),
                            0.5f, 112, 85);

    clampRange(m, root);
}

void BassEngine::clampRange(MS m[bli::kStepsPerBar], int root) {
    for (int i = 0; i < bli::kStepsPerBar; i++) {
        if (m[i].note < 0) continue;
        while (m[i].note > root + kMaxSemitonesAboveRoot) m[i].note -= kOctaveSemitones;
        while (m[i].note < root - kMaxSemitonesBelowRoot) m[i].note += kOctaveSemitones;
    }
}

void BassEngine::transformMotif(const MS src[bli::kStepsPerBar], MS dst[bli::kStepsPerBar], int type,
                                 int root, int scIdx) {
    memcpy(dst, src, bli::kStepsPerBar * sizeof(MS));
    if (type == MOTIF_SIMPLIFY) {
        for (int i = 0; i < bli::kStepsPerBar; i++)
            if (dst[i].note >= 0 && i % kStepsPerBeat != 0 && randChance(kSimplifyThinProbability))
                dst[i].note = kEmptyNote;
    } else if (type == MOTIF_SUB_DROP) {
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (dst[i].note < 0) continue;
            if (dst[i].note > kSubDropFloorNote) dst[i].note -= kOctaveSemitones;
            dst[i].fcc = std::max(kFilterCCFloor, dst[i].fcc - kSubDropFilterCCReduction);
        }
    } else if (type == MOTIF_MODAL_SHIFT) {
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (dst[i].note < 0 || i % kStepsPerBeat == 0) continue;
            if (randChance(kModalShiftProbability))
                dst[i].note += randChance(kModalShiftFifthProbability) ? kFifthSemitones
                                                                      : kThirdSemitones;
        }
    } else {
        int dropBeat = (randBelow(kDriftDropBeatMaxIndex) + 1) * kStepsPerBeat;
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (i >= dropBeat && i < dropBeat + kStepsPerBeat) dst[i].note = kEmptyNote;
        }
        if (dst[bli::kStepsPerBar - 1].note < 0 && randChance(kDriftTailProbability))
            dst[bli::kStepsPerBar - 1] = mn(root, 0.15f, 65, 45);
    }
}

void BassEngine::addNote(float posInSteps, int note, float len, int vel, int fcc,
                         float pbSemitones) {
    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [posInSteps](const NoteSlot& n) {
            return std::fabs(n.posInSteps - posInSteps) < kSamePositionEpsilonSteps;
        }),
        m_phrase.end());
    m_phrase.push_back({posInSteps, note, len, vel, fcc, pbSemitones});
}

void BassEngine::turnInterpreted(int base, int scIdx) {
    EspRng rng;
    bli::TurnNote tn[3];
    int n = bli::generateTurnaround(tn, base, scIdx, m_dials, rng);
    for (int i = 0; i < n; i++) {
        addNote(kTurnaroundStartStep + tn[i].stepOffset, tn[i].note, tn[i].len, tn[i].vel,
                tn[i].fcc, tn[i].pb);
    }
}

const BassEngine::ApproachShape& BassEngine::shapeFor(int approach) {
    static const ApproachShape kShapes[APP_COUNT] = {
        {1.00f, 0.50f, 0.10f, 0.00f, 0.35f,  6.f, 18.f, 0.02f, true,  false, false},
        {0.85f, 0.45f, 0.08f, 0.15f, 0.50f, 22.f, 55.f, 0.00f, false, false, false},
        {0.50f, 0.55f, 0.12f, 0.38f, 0.60f, 30.f, 25.f, 0.05f, false, true,  true },
        {0.32f, 0.70f, 0.18f, 0.45f, 0.75f, 34.f, 30.f, 0.09f, false, true,  true },
        {0.25f, 0.22f, 0.35f, 0.50f, 0.25f, 12.f, 15.f, 0.00f, false, false, false},
        {0.40f, 1.60f, 0.06f, 0.10f, 0.20f, 10.f, 45.f, 0.03f, false, false, false},
        {0.20f, 2.50f, 0.15f, 0.20f, 0.15f,  8.f, 35.f, 0.00f, false, false, false},
        {0.55f, 0.35f, 0.22f, 0.50f, 0.85f, 36.f, 40.f, 0.06f, false, true,  true },
    };
    if (approach < APP_ROLL || approach >= APP_COUNT) approach = APP_ROLL;
    return kShapes[approach];
}

float BassEngine::syncopationFraction(const MS m[bli::kStepsPerBar]) {
    int occupied = 0;
    int offGrid  = 0;
    for (int i = 0; i < bli::kStepsPerBar; i++) {
        if (m[i].note < 0) continue;
        occupied++;
        if (i % 2 != 0) offGrid++;
    }
    return occupied > 0 ? (float)offGrid / (float)occupied : 0.f;
}

void BassEngine::shapeBar(MS out[bli::kStepsPerBar], const MS src[bli::kStepsPerBar],
                          int bar, int root, int scIdx) {
    static constexpr float kMetricalWeightUnit = 0.3333333f;
    static constexpr float kRollingCarryProb   = 0.55f;
    static constexpr int   kQuestionBar        = 6;
    static constexpr int   kAnswerBar          = 7;
    static constexpr int   kQuestionGapStart   = 8;
    static constexpr int   kQuestionGapEnd     = 12;
    static constexpr float kAnswerFillProb     = 0.6f;
    static constexpr int   kSyncAdjustPasses   = 4;
    static constexpr float kSyncTolerance      = 0.02f;
    static constexpr float kOctaveJumpStepProb = 0.35f;
    static constexpr float kPickupProb         = 0.35f;
    static constexpr float kPickupLengthSteps  = 0.25f;
    static constexpr int   kPickupVelDrop      = 20;
    static constexpr float kMinGateSteps       = 0.35f;
    static constexpr int   kVelocityWobble     = 3;
    static constexpr float kTwoPi              = 6.28318530717958647692f;
    static constexpr float kQuarterTurn        = 1.57079632679489661923f;

    const ApproachShape& sh = shapeFor(m_approach);

    for (int i = 0; i < bli::kStepsPerBar; i++) out[i] = src[i];

    if (sh.rolling) {
        MS carried = mn(kEmptyNote, 0.f, 0, 0);
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (out[i].note >= 0) carried = out[i];
            else if (carried.note >= 0 && randChance(kRollingCarryProb)) out[i] = carried;
        }
    }

    if (sh.density < 1.0f) {
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (out[i].note < 0) continue;
            const int metricalWeight = (i % 8 == 0) ? 3 : (i % 4 == 0) ? 2 : (i % 2 == 0) ? 1 : 0;
            const float keepProb =
                sh.density + (float)(metricalWeight * (1.0f - sh.density)) * kMetricalWeightUnit;
            if (!randChance(std::min(1.0f, keepProb))) out[i].note = kEmptyNote;
        }
    }

    if (sh.questionAnswer) {
        if (bar % 8 == kQuestionBar) {
            for (int i = kQuestionGapStart; i < kQuestionGapEnd; i++) out[i].note = kEmptyNote;
        } else if (bar % 8 == kAnswerBar) {
            MS fill = mn(kEmptyNote, 0.f, 0, 0);
            for (int i = 0; i < bli::kStepsPerBar && fill.note < 0; i++)
                if (out[i].note >= 0) fill = out[i];
            if (fill.note >= 0) {
                for (int i = kQuestionGapStart; i < kQuestionGapEnd; i++)
                    if (out[i].note < 0 && randChance(kAnswerFillProb)) out[i] = fill;
            }
        }
    }

    if (sh.syncTarget > 0.f) {
        for (int pass = 0; pass < kSyncAdjustPasses; pass++) {
            const float frac = syncopationFraction(out);
            if (frac >= sh.syncTarget - kSyncTolerance && frac <= sh.syncTarget + kSyncTolerance) break;
            const bool needMore = frac < sh.syncTarget;
            for (int i = 0; i < bli::kStepsPerBar; i++) {
                const bool offGrid = (i % 2 != 0);
                if (needMore == offGrid) continue;
                if (out[i].note < 0) continue;
                const int target = needMore ? i + 1 : i - 1;
                if (target < 0 || target >= bli::kStepsPerBar) continue;
                if (out[target].note >= 0) continue;
                out[target] = out[i];
                out[i].note = kEmptyNote;
                break;
            }
        }
    }

    const int editCount = (int)(sh.mutatePerBar * 2.0f + rand01());
    for (int e = 0; e < editCount; e++) {
        const int i = randBelow(bli::kStepsPerBar);
        if (out[i].note < 0) continue;
        switch (randBelow(3)) {
            case 0: {
                const int diatonicThird = std::max(1, bli::scaleDegree(scIdx, 2));
                const int shift = randChance(0.5f) ? kFifthSemitones : diatonicThird;
                out[i].note += randChance(0.5f) ? shift : -shift;
                break;
            }
            case 1:
                out[i].note = kEmptyNote;
                break;
            default: {
                const int j = (i + 1) % bli::kStepsPerBar;
                const MS tmp = out[i];
                out[i] = out[j];
                out[j] = tmp;
                break;
            }
        }
    }

    if (randChance(sh.octaveJumpProb)) {
        for (int i = 0; i < bli::kStepsPerBar; i++)
            if (out[i].note >= 0 && randChance(kOctaveJumpStepProb)) out[i].note += kOctaveSemitones;
    }

    if (sh.pickups && bar % kBarsPerChordCell == kBarsPerChordCell - 1 && randChance(kPickupProb)) {
        const int last = bli::kStepsPerBar - 1;
        if (out[last].note < 0) {
            MS lead = mn(kEmptyNote, 0.f, 0, 0);
            for (int i = last; i >= 0; i--)
                if (out[i].note >= 0) { lead = out[i]; break; }
            if (lead.note >= 0) {
                out[last] = lead;
                out[last].len = kPickupLengthSteps;
                out[last].vel = std::max(1, out[last].vel - kPickupVelDrop);
            }
        }
    }

    const float barPhase = (float)(bar % kBarsPerSection) / (float)kBarsPerSection;
    const float velLfo   = std::sin(kTwoPi * barPhase);
    const float filtLfo  = std::sin(kTwoPi * barPhase + kQuarterTurn);

    for (int i = 0; i < bli::kStepsPerBar; i++) {
        if (out[i].note < 0) continue;
        out[i].len = std::max(kMinGateSteps, out[i].len * sh.gate);
        out[i].vel = std::max(1, std::min(127,
            out[i].vel + (int)(velLfo * sh.velDrift) + (randBelow(kVelocityWobble * 2 + 1) - kVelocityWobble)));
        out[i].fcc = std::max(0, std::min(127, out[i].fcc + (int)(filtLfo * sh.filtSweepDepth)));
        out[i].tsBeats += (rand01() * 2.0f - 1.0f) * sh.timingSteps / (float)kStepsPerBeat;
    }

    clampRange(out, root);
}

void BassEngine::regeneratePhrase(bool advanceArc) {
    if (advanceArc) m_phraseCount++;
    m_phrase.clear();

    if (m_scaleHoldPhrasesRemaining <= 0) {
        int base = m_dials.pickScaleIdx();
        int jitter = randChance(kScaleJitterProbability) ? (randBelow(kScaleJitterRange) - 1) : 0;
        m_scaleIdx = std::max(0, std::min(bli::kNumScales - 1, base + jitter));
        m_scaleHoldPhrasesRemaining = kScaleHoldMinPhrases + randBelow(kScaleHoldRangePhrases);
    } else {
        m_scaleHoldPhrasesRemaining--;
    }
    m_progIdx = randBelow(kNumProgs);
    const int* prog = kChordDegreeProgs[m_progIdx];

    MS motA[bli::kStepsPerBar], motB[bli::kStepsPerBar], motC[bli::kStepsPerBar], motD[bli::kStepsPerBar];
    MS freshA[bli::kStepsPerBar];
    genInterpreted(freshA, kRootNoteE2, m_scaleIdx);

    if (m_hasPrevPhraseMotif) {
        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (freshA[i].note >= 0 && m_prevPhraseMotifA[i].note >= 0 &&
                randChance(kMotifBlendProbability))
                motA[i] = m_prevPhraseMotifA[i];
            else
                motA[i] = freshA[i];
        }
    } else {
        memcpy(motA, freshA, sizeof(motA));
    }
    memcpy(m_prevPhraseMotifA, motA, sizeof(motA));
    m_hasPrevPhraseMotif = true;

    anchorMotif(motA, kRootNoteE2, m_scaleIdx);

    int arc = m_phraseCount % kArcPeriodPhrases;
    int xformB = (arc < 2) ? MOTIF_SIMPLIFY : (arc < 5) ? MOTIF_MODAL_SHIFT
                                                        : randBelow(kMotifTransformCount);
    int xformC = (arc < 3) ? MOTIF_SIMPLIFY : (arc < 6) ? MOTIF_SUB_DROP
                                                        : randBelow(kMotifTransformCount);
    transformMotif(motA, motB, xformB, kRootNoteE2, m_scaleIdx); clampRange(motB, kRootNoteE2);
    transformMotif(motA, motC, xformC, kRootNoteE2, m_scaleIdx); clampRange(motC, kRootNoteE2);
    transformMotif(motA, motD, MOTIF_DRIFT, kRootNoteE2, m_scaleIdx); clampRange(motD, kRootNoteE2);

    float varVal = m_dials.motionVariation;
    const char* structure;
    float rnd = rand01();
    if      (varVal < 0.2f) structure = "AAAD";
    else if (varVal < 0.5f) structure = (rnd < 0.6f) ? "AAAB" : "ABAB";
    else if (varVal < 0.8f) structure = (rnd < 0.5f) ? "ABAC" : "AAAB";
    else                    structure = "ABAC";

    int secProgs[kSectionsPerPhrase] = {m_progIdx, (m_progIdx + 1) % kNumProgs,
                                        (m_progIdx + 2) % kNumProgs, m_progIdx};

    float swingSteps = m_dials.grooveSwing * kMaxSwingSteps;

    for (int bar = 0; bar < kBarsPerPhrase; bar++) {
        int sec       = bar / kBarsPerSection;
        int barInSec  = bar % kBarsPerSection;
        int barInCell = barInSec % kBarsPerChordCell;

        const int* secProg = kChordDegreeProgs[secProgs[sec]];
        int rootSemitoneOffset = scaleDegreeSemitones(m_scaleIdx, secProg[barInCell]);

        char part;
        if (sec == kFirstSection || sec == kLastSection) {
            part = structure[barInCell];
        } else if (sec == kBridgeSection) {
            static const char BRIDGE[kBarsPerChordCell] = {'B','A','B','A'};
            part = BRIDGE[barInCell];
        } else {
            static const char PEAK[kBarsPerChordCell]   = {'C','B','C','B'};
            part = PEAK[barInCell];
        }

        const MS* motif = (part=='A') ? motA : (part=='B') ? motB :
                          (part=='C') ? motC : motD;

        MS barMotif[bli::kStepsPerBar];
        shapeBar(barMotif, motif, bar, kRootNoteE2, m_scaleIdx);

        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (barMotif[i].note < 0) continue;
            int note = barMotif[i].note + rootSemitoneOffset;
            note = std::max(0, std::min(127, note));

            float tsSteps = barMotif[i].tsBeats * kStepsPerBeat;
            float basePos = (float)(bar * bli::kStepsPerBar + i);
            float posInSteps = basePos + tsSteps;

            if (i % 2 != 0) posInSteps += swingSteps;
            if (posInSteps < 0.f) posInSteps = 0.f;

            m_phrase.push_back({posInSteps, note, barMotif[i].len, barMotif[i].vel,
                                 barMotif[i].fcc, barMotif[i].pbSemitones});
        }
    }

    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [](const NoteSlot& n) { return n.posInSteps >= kTurnaroundStartStep; }), m_phrase.end());

    int turnaroundRate = (m_dials.voiceArtic > kArticulatedTurnaroundThreshold)
                         ? kFrequentTurnaroundPeriodPhrases : kSparseTurnaroundPeriodPhrases;
    if (m_phraseCount % turnaroundRate == 0) {
        int baseSemitoneOffset =
            scaleDegreeSemitones(m_scaleIdx, prog[(kBarsPerSection - 1) % kBarsPerChordCell]);
        int tBase = std::max(0, std::min(127, kRootNoteE2 + baseSemitoneOffset));
        turnInterpreted(tBase, m_scaleIdx);
    }

    std::sort(m_phrase.begin(), m_phrase.end(),
              [](const NoteSlot& a, const NoteSlot& b) {
                  return a.posInSteps < b.posInSteps;
              });

    ESP_LOGI(TAG, "Phrase[%d] approach=%d bank=%d scale=%s prog=%d notes=%d bars=64",
             m_phraseCount, m_approach, m_activeBank, bli::scaleName(m_scaleIdx), m_progIdx,
             (int)m_phrase.size());
}

void BassEngine::playNote(const NoteSlot& n, double bpm) {
    int note = std::max(0, std::min(127, n.note));
    int vel  = std::max(1, std::min(127, n.velocity));
    int fcc  = std::max(0, std::min(127, n.filterCC74));

    uint8_t noteOn[3] = {kMidiNoteOn, (uint8_t)note, (uint8_t)vel};
    send_midi_message(noteOn, 3);

    send_midi_cc(1, kMidiCCFilter, (uint8_t)fcc);

    if (n.pitchBendSemitones != 0.f) {
        int raw = kPitchBendCentreRaw + (int)(n.pitchBendSemitones / kPitchBendSemitoneSpan
                                              * kPitchBendHalfRange);
        raw = std::max(0, std::min(kPitchBendRawMax, raw));
        uint8_t pb[3] = {kMidiPitchBend, (uint8_t)(raw & 0x7F), (uint8_t)(raw >> 7)};
        send_midi_message(pb, 3);
    } else {
        uint8_t pb[3] = {kMidiPitchBend, (uint8_t)(kPitchBendCentreRaw & 0x7F),
                         (uint8_t)(kPitchBendCentreRaw >> 7)};
        send_midi_message(pb, 3);
    }

    double offPosInSteps = n.posInSteps + n.lengthInSteps;
    m_activeNotes.push_back({note, offPosInSteps});
}

void BassEngine::processNoteOffs(double phrasePosInSteps, double) {
    for (auto it = m_activeNotes.begin(); it != m_activeNotes.end(); ) {
        double diff = phrasePosInSteps - it->offPosInSteps;
        if (diff >= 0.0 || diff < kWrapBackwardThreshold) {
            uint8_t noteOff[3] = {kMidiNoteOff, (uint8_t)it->note, 0x00};
            send_midi_message(noteOff, 3);
            it = m_activeNotes.erase(it);
        } else {
            ++it;
        }
    }
}

BassEngine::BassEngine() = default;

void BassEngine::setActiveBank(int bank) {
    if (bank < 0 || bank >= bli::BANK_COUNT) return;
    m_activeBank = bank;

    if (!m_active) {
        m_active = true;
        m_lastPhrasePosSteps       = kNoPhrasePositionYet;
        m_lastBarInPhrase          = kNoBarYet;
        m_regenPending             = false;
        m_phraseCount              = 0;
        m_hasPrevPhraseMotif       = false;
        m_scaleHoldPhrasesRemaining = 0;
        regeneratePhrase();
    }
    ESP_LOGI(TAG, "Bank set to %d", m_activeBank);
}

void BassEngine::setApproach(int approach) {
    if (approach < APP_ROLL || approach >= APP_COUNT) return;
    m_approach = approach;
    m_activeBank = std::min(bli::BANK_COUNT - 1, approach / 2);

    if (!m_active) {
        m_active = true;
        m_lastPhrasePosSteps       = kNoPhrasePositionYet;
        m_lastBarInPhrase          = kNoBarYet;
        m_regenPending             = false;
        m_phraseCount              = 0;
        m_hasPrevPhraseMotif       = false;
        m_scaleHoldPhrasesRemaining = 0;
        regeneratePhrase();
    } else {
        m_regenPending = true;
    }
    ESP_LOGI(TAG, "Approach set to %d (bank %d)", m_approach, m_activeBank);
}

void BassEngine::setDial(int dialIdx, float v01) {
    m_dials.set(m_activeBank, dialIdx, v01);
    if (m_active) m_regenPending = true;
}

void BassEngine::nudge() {
    if (m_active) m_regenPending = true;
}

void BassEngine::stop() {
    m_active = false;
    uint8_t allOff[3] = {kMidiControlChange, kMidiCCAllNotesOff, 0};
    send_midi_message(allOff, 3);
    m_activeNotes.clear();
    ESP_LOGI(TAG, "Engine stopped");
}

void BassEngine::process(const ableton::Link::SessionState& state,
                         const std::chrono::microseconds& time) {
    if (!m_active) return;

    double bpm = state.tempo();
    double beat = state.beatAtTime(time, LINK_QUANTUM);
    if (beat < 0.0) return;

    double phrasePosInSteps = std::fmod(beat * kStepsPerBeat, kStepsPerPhrase);

    if (m_lastPhrasePosSteps >= 0.0 &&
        phrasePosInSteps < m_lastPhrasePosSteps - kHalfPhraseSteps) {
        regeneratePhrase(true);
        m_regenPending = false;
        m_lastBarInPhrase = kNoBarYet;
    }

    int currentBarInPhrase = static_cast<int>(phrasePosInSteps / bli::kStepsPerBar);
    if (m_regenPending && m_lastBarInPhrase >= 0 && currentBarInPhrase != m_lastBarInPhrase) {
        regeneratePhrase(false);
        m_regenPending = false;
    }
    m_lastBarInPhrase = currentBarInPhrase;

    if (m_lastPhrasePosSteps < 0.0) {
        m_lastPhrasePosSteps = phrasePosInSteps;
        return;
    }

    processNoteOffs(phrasePosInSteps, bpm);

    double advanceSteps = phrasePosInSteps - m_lastPhrasePosSteps;
    if (advanceSteps < 0.0) advanceSteps += kStepsPerPhrase;
    if (advanceSteps > kMaxAdvanceStepsPerProcess) {
        ESP_LOGW(TAG, "phrase position jumped %.1f steps -- reseating, not replaying", advanceSteps);
        m_lastPhrasePosSteps = phrasePosInSteps;
        return;
    }

    bool wrapped = (phrasePosInSteps < m_lastPhrasePosSteps);
    for (const auto& n : m_phrase) {
        bool due;
        if (!wrapped) {
            due = (n.posInSteps > m_lastPhrasePosSteps && n.posInSteps <= phrasePosInSteps);
        } else {
            due = (n.posInSteps > m_lastPhrasePosSteps || n.posInSteps <= phrasePosInSteps);
        }
        if (due) {
            playNote(n, bpm);
        }
    }

    m_lastPhrasePosSteps = phrasePosInSteps;
}

BassEngine g_bassEngine;
