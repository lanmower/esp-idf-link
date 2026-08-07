#pragma once

#include <vector>
#include <cstdint>
#include <ableton/Link.hpp>
#include <chrono>
#include "bassline_interpreter.h"

// A single note event in the 1024-step phrase buffer.
// pos: position in steps (0.0 - 1023.75), fractional for sub-step placement.
struct NoteSlot {
    float pos;
    int   note;       // MIDI note number
    float length;     // duration in 16th notes
    int   velocity;   // 0-127
    int   filterCC;   // value for CC74
    float pitchBend;  // semitones (positive = up)
};

// Control surface: 4 touch pads select which of 4 dial "banks" the
// project's 2 physical potentiometers currently address (see
// bassline_interpreter.h for what each bank/dial pair controls). This
// replaces the old 8-genre tap/double-tap lookup-table scheme -- there is
// no more discrete genre selection, only continuous interpreted
// generation. See readme.md for the full interaction model.
class BassEngine {
public:
    BassEngine();

    // Tapping a pad selects which bank the 2 pots address. The very first
    // bank selection also starts playback (with the current -- default,
    // unless already dialed in -- parameters). Later taps just retarget
    // the pots; they do not interrupt or regenerate the playing phrase.
    void setActiveBank(int bank);
    int  activeBank() const { return m_activeBank; }

    // Pot movement while a bank is active -- dialIdx is 0 or 1 (which of
    // the 2 physical pots moved). Re-generates the phrase on the fly (at
    // the next bar boundary, like the old setCtrl1/2 hot-swap).
    void setDial(int dialIdx, float v01);

    // Long-press gesture: reroll the current bar with fresh randomness
    // while keeping the same dial-driven character (same mechanism as a
    // dial change -- takes effect at the next bar boundary, no click).
    void nudge();

    // Call from the main tick task every timer period.
    void process(const ableton::Link::SessionState& state,
                 const std::chrono::microseconds& time);

    void stop();

    bool isActive() const { return m_active; }
    const bli::Dials& dials() const { return m_dials; }

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

    // Phrase generation -- advanceArc=false for mid-phrase hot-swap (no arc increment)
    void regeneratePhrase(bool advanceArc = true);

    // Fills m[0..15] by running the interpreted generator (groove
    // template + DP pitch solver + contour + articulation, see
    // bassline_interpreter.h) with the current dials, and converts its
    // output into the MS representation the arc/transform/turnaround
    // machinery below already knows how to arrange into a 64-bar phrase.
    void genInterpreted(MS m[16], int root, int scIdx);

    // Motif transforms (unchanged -- these are generic arrangement
    // variety on top of whatever generator filled motif A, not part of
    // the note-generation itself)
    void transformMotif(const MS src[16], MS dst[16], int type, int root, int scIdx);
    // Anchor strong beats and clamp range to prevent tonal drift
    void anchorMotif(MS m[16], int root, int scIdx);
    static void clampRange(MS m[16], int root);

    // Turnaround -- inject notes into m_phrase directly (called at tStart = 1016)
    void turnInterpreted(int base, int scIdx);

    // Helper: append a note to m_phrase
    void addNote(float pos, int note, float len, int vel, int fcc, float pb = 0.f);

    // Scheduling / MIDI
    void playNote        (const NoteSlot& n, double bpm);
    void processNoteOffs (double phrasePos, double bpm);

    // State
    std::vector<NoteSlot> m_phrase;

    struct ActiveNote { int note; double offPos; };
    std::vector<ActiveNote> m_activeNotes;

    int        m_activeBank    = bli::BANK_HARMONY;
    bli::Dials m_dials;
    bool   m_active         = false;
    int    m_phraseCount    = 0;
    double m_lastPos        = -1.0;
    int    m_scaleIdx       = 0;
    int    m_progIdx        = 0;
    bool   m_regenPending   = false;   // deferred regen at bar boundary
    int    m_scaleHoldCount = 0;       // phrases remaining on current scale
    int    m_lastBar        = -1;      // bar index in phrase (0-15) for bar-crossing detection
    MS     m_prevMotA[16]   = {};      // motif A from previous phrase for continuity
    bool   m_hasPrevMotA    = false;
};

extern BassEngine g_bassEngine;
