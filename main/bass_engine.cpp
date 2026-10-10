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

static int scaleDegree(int scIdx, int degree) { return bli::scaleDegree(scIdx, degree); }

static const int kNumProgs = 6;
static const int kProgs[kNumProgs][kBarsPerChordCell] = {
    {0, 5, 3, 6},
    {0, 6, 5, 6},
    {0, 3, 6, 2},
    {0, 5, 6, 4},
    {0, 2, 6, 3},
    {0, 0, 5, 6},
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
    int fifthInterval = scaleDegree(scIdx, kFifthScaleDegreeIndex);

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
    const int* prog = kProgs[m_progIdx];

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

        const int* secProg = kProgs[secProgs[sec]];
        int rootSemitoneOffset = secProg[barInCell];

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

        for (int i = 0; i < bli::kStepsPerBar; i++) {
            if (motif[i].note < 0) continue;
            int note = motif[i].note + rootSemitoneOffset;
            note = std::max(0, std::min(127, note));

            float tsSteps = motif[i].tsBeats * kStepsPerBeat;
            float basePos = (float)(bar * bli::kStepsPerBar + i);
            float posInSteps = basePos + tsSteps;

            if (i % 2 != 0) posInSteps += swingSteps;

            m_phrase.push_back({posInSteps, note, motif[i].len, motif[i].vel,
                                 motif[i].fcc, motif[i].pbSemitones});
        }
    }

    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [](const NoteSlot& n) { return n.posInSteps >= kTurnaroundStartStep; }), m_phrase.end());

    int turnaroundRate = (m_dials.voiceArtic > kArticulatedTurnaroundThreshold)
                         ? kFrequentTurnaroundPeriodPhrases : kSparseTurnaroundPeriodPhrases;
    if (m_phraseCount % turnaroundRate == 0) {
        int baseSemitoneOffset = prog[(kBarsPerSection - 1) % kBarsPerChordCell];
        int tBase = std::max(0, std::min(127, kRootNoteE2 + baseSemitoneOffset));
        turnInterpreted(tBase, m_scaleIdx);
    }

    std::sort(m_phrase.begin(), m_phrase.end(),
              [](const NoteSlot& a, const NoteSlot& b) {
                  return a.posInSteps < b.posInSteps;
              });

    ESP_LOGI(TAG, "Phrase[%d] bank=%d scale=%s prog=%d notes=%d bars=64",
             m_phraseCount, m_activeBank, bli::scaleName(m_scaleIdx), m_progIdx,
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
