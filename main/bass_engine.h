#pragma once

#include <vector>
#include <cstdint>
#include <ableton/Link.hpp>
#include <chrono>

// A single note event in the 256-step phrase buffer.
// pos: position in steps (0.0 – 255.75), fractional for sub-step placement.
struct NoteSlot {
    float pos;
    int   note;       // MIDI note number
    float length;     // duration in 16th notes
    int   velocity;   // 0-127
    int   filterCC;   // value for CC74
    float pitchBend;  // semitones (positive = up)
};

// 8 genres:
//   Single tap  on pad 0-3 → GENRE_FUNK / ITALO / SYNTHPOP / PSYTRANCE
//   Double tap  on pad 0-3 → GENRE_PROG / AFROHOUSE / UKG / GFUNK
enum GenreId {
    GENRE_FUNK      = 0,
    GENRE_ITALO     = 1,
    GENRE_SYNTHPOP  = 2,
    GENRE_PSYTRANCE = 3,
    GENRE_PROG      = 4,
    GENRE_AFROHOUSE = 5,
    GENRE_UKG       = 6,
    GENRE_GFUNK     = 7,
    GENRE_COUNT     = 8
};

class BassEngine {
public:
    BassEngine();

    // Select a genre and start playing. Regenerates phrase immediately.
    void setGenre(int genre_idx);

    // Macro controls (0.0 – 1.0). Re-generates phrase on the fly.
    void setCtrl1(float v);
    void setCtrl2(float v);

    // Call from the main tick task every timer period.
    void process(const ableton::Link::SessionState& state,
                 const std::chrono::microseconds& time);

    void stop();

    bool isActive()     const { return m_active; }
    int  currentGenre() const { return m_genre; }

private:
    // Internal motif slot (16 per bar, note==-1 means empty)
    struct MS {
        int   note;
        float len;
        int   vel;
        int   fcc;
        float pb;   // pitch bend semitones
        float ts;   // time-shift in beats
    };
    static MS mn(int note, float len, int vel, int fcc,
                 float pb = 0.f, float ts = 0.f);
    static void msInit(MS m[16]);

    // Phrase generation
    void regeneratePhrase();

    // Per-genre motif generators (fill m[0..15])
    void genFunk     (MS m[16], int root, int scIdx, float c1, float c2);
    void genItalo    (MS m[16], int root, int scIdx, float c1, float c2);
    void genSynthpop (MS m[16], int root, int scIdx, float c1, float c2);
    void genPsytrance(MS m[16], int root, int scIdx, float c1, float c2);
    void genProg     (MS m[16], int root, int scIdx, float c1, float c2);
    void genAfrohouse(MS m[16], int root, int scIdx, float c1, float c2);
    void genUkg      (MS m[16], int root, int scIdx, float c1, float c2);
    void genGfunk    (MS m[16], int root, int scIdx, float c1, float c2);

    // Motif transforms
    void transformMotif(const MS src[16], MS dst[16], int type, int root, int scIdx);
    // Anchor strong beats and clamp range to prevent tonal drift
    void anchorMotif(MS m[16], int root, int scIdx);

    // Turnarounds – inject notes into m_phrase directly (called at tStart = 248)
    void turnFunk     (int base, int scIdx, float c1, float c2);
    void turnItalo    (int base);
    void turnSynthpop (int base);
    void turnPsytrance(int base, int scIdx, float c1);
    void turnProg     (int base);
    void turnAfrohouse(int base);
    void turnUkg      (int base);
    void turnGfunk    (int base, int scIdx, float c1);

    // Helper: append a note to m_phrase
    void addNote(float pos, int note, float len, int vel, int fcc, float pb = 0.f);

    // Scheduling / MIDI
    void playNote        (const NoteSlot& n, double bpm);
    void processNoteOffs (double phrasePos, double bpm);

    // State
    std::vector<NoteSlot> m_phrase;

    struct ActiveNote { int note; double offPos; };
    std::vector<ActiveNote> m_activeNotes;

    int    m_genre          = GENRE_FUNK;
    float  m_ctrl1          = 0.5f;
    float  m_ctrl2          = 0.5f;
    bool   m_active         = false;
    int    m_phraseCount    = 0;
    double m_lastPos        = -1.0;
    int    m_scaleIdx       = 0;
    int    m_progIdx        = 0;
    bool   m_regenPending   = false;   // deferred regen at phrase boundary
    int    m_scaleHoldCount = 0;       // phrases remaining on current scale
    MS     m_prevMotA[16]   = {};      // motif A from previous phrase for continuity
    bool   m_hasPrevMotA    = false;
};

extern BassEngine g_bassEngine;
