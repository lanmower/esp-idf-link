#pragma once

#include <vector>
#include <cstdint>
#include <ableton/Link.hpp>
#include <chrono>
#include "bassline_interpreter.h"

struct NoteSlot {
    float posInSteps;
    int   note;
    float lengthInSteps;
    int   velocity;
    int   filterCC74;
    float pitchBendSemitones;
};

class BassEngine {
public:
    enum Approach {
        APP_ROLL   = 0,
        APP_ACID   = 1,
        APP_FUNK   = 2,
        APP_GLITCH = 3,
        APP_STAB   = 4,
        APP_DRIFT  = 5,
        APP_PEDAL  = 6,
        APP_BREAK  = 7,
        APP_COUNT  = 8
    };

    BassEngine();

    void setActiveBank(int bank);
    int  activeBank() const { return m_activeBank; }

    void setApproach(int approach);
    int  approach() const { return m_approach; }

    void setDial(int dialIdx, float v01);

    void nudge();

    void process(const ableton::Link::SessionState& state,
                 const std::chrono::microseconds& time);

    void stop();

    bool isActive() const { return m_active; }
    const bli::Dials& dials() const { return m_dials; }

private:
    enum { MOTIF_SIMPLIFY = 0, MOTIF_SUB_DROP = 1, MOTIF_MODAL_SHIFT = 2, MOTIF_DRIFT = 3 };
    static constexpr int    kMotifTransformCount = 4;
    static constexpr int    kEmptyNote           = -1;
    static constexpr int    kNoBarYet            = -1;
    static constexpr double kNoPhrasePositionYet = -1.0;

    struct MS {
        int   note;
        float len;
        int   vel;
        int   fcc;
        float pbSemitones;
        float tsBeats;
    };
    static MS mn(int note, float len, int vel, int fcc,
                 float pbSemitones = 0.f, float tsBeats = 0.f);
    static void msInit(MS m[bli::kStepsPerBar]);

    void regeneratePhrase(bool advanceArc = true);

    void genInterpreted(MS m[bli::kStepsPerBar], int root, int scIdx, const bli::Dials& dials);

    void transformMotif(const MS src[bli::kStepsPerBar], MS dst[bli::kStepsPerBar],
                        int type, int root, int scIdx);
    void anchorMotif(MS m[bli::kStepsPerBar], int root, int scIdx);
    static void clampRange(MS m[bli::kStepsPerBar], int root);

    struct ApproachShape {
        float density;
        float gate;
        float octaveJumpProb;
        float syncTarget;
        float mutatePerBar;
        float velDrift;
        float filtSweepDepth;
        float timingSteps;
        bool  rolling;
        bool  questionAnswer;
        bool  pickups;
        float minGateSteps;
        float grooveEnergyBias;
        float contourBias;
        float swingBias;
        float articBias;
        float colorBias;
    };
    static const ApproachShape& shapeFor(int approach);
    bli::Dials dialsForApproach() const;
    void shapeBar(MS out[bli::kStepsPerBar], const MS src[bli::kStepsPerBar],
                  int bar, int root, int scIdx);
    static float syncopationFraction(const MS m[bli::kStepsPerBar]);

    void turnInterpreted(int base, int scIdx, const bli::Dials& dials);

    void addNote(float posInSteps, int note, float len, int vel, int fcc,
                 float pbSemitones = 0.f);

    void playNote        (const NoteSlot& n, double bpm);
    void processNoteOffs (double phrasePosInSteps, double bpm);

    std::vector<NoteSlot> m_phrase;

    struct ActiveNote { int note; double offPosInSteps; };
    std::vector<ActiveNote> m_activeNotes;

    int        m_activeBank    = bli::BANK_HARMONY;
    int        m_approach      = APP_ROLL;
    bli::Dials m_dials;
    bool   m_active         = false;
    int    m_phraseCount    = 0;
    double m_lastPhrasePosSteps = kNoPhrasePositionYet;
    int    m_scaleIdx       = 0;
    int    m_progIdx        = 0;
    bool   m_regenPending   = false;
    int    m_scaleHoldPhrasesRemaining = 0;
    int    m_lastBarInPhrase = kNoBarYet;
    MS     m_prevPhraseMotifA[bli::kStepsPerBar] = {};
    bool   m_hasPrevPhraseMotif = false;
};

extern BassEngine g_bassEngine;
