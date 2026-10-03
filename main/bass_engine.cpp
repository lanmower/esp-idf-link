// bass_engine.cpp
// Phrase = 1024 steps (64 bars x 16 steps).  1 step = 1/16 note.
// Link beat -> phrase pos:  pos = fmod(beat * 4.0, 1024.0)
//
// Note generation is delegated to bassline_interpreter.{h,cpp} (groove
// template + DP pitch solver + contour + articulation, driven by 8
// continuous dials in 4 banks -- see bass_engine.h and readme.md). This
// file owns the surrounding song-form machinery that is independent of
// how a single bar's notes get chosen: the 64-bar arc (4 sections x 16
// bars, motif transforms simplify/subDrop/modalShift/drift, AAAD/ABAB/
// ABAC structure), scale-hold continuity, chord-progression rotation,
// turnarounds, and MIDI scheduling.

#include "bass_engine.h"
#include "io_helpers.h"
#include "esp_log.h"
#include "esp_random.h"
#include <algorithm>
#include <cstring>
#include <cmath>

static const char* TAG = "BASS";

// ---- Fixed root note (E2) - hardware has no root-note menu ----
static const int ROOT = 40;

// Safe scale degree access -- delegates to bassline_interpreter.h's scale
// table (single source of truth; used by both the phrase-arrangement code
// here and the note generator itself).
static int sc(int scIdx, int degree) { return bli::scaleDegree(scIdx, degree); }

// Generic chord progressions (scale-degree roots), genre-agnostic --
// replaces the old per-genre GCFG.progs table. Validated musically via
// the DawDreamer Python port (see ../../DawDreamer/experiments/bassline/).
static const int kProgs[6][4] = {
    {0, 5, 3, 6},   // i - VI - iv - VII
    {0, 6, 5, 6},   // i - VII - VI - VII
    {0, 3, 6, 2},   // i - iv - VII - III
    {0, 5, 6, 4},   // i - VI - VII - V (dark cadence)
    {0, 2, 6, 3},   // i - III - VII - iv
    {0, 0, 5, 6},   // pedal-leaning: i - i - VI - VII
};
static const int kNumProgs = 6;

// ---- Random helpers ----
static float rand01() {
    return (float)(esp_random() & 0xFFFFFF) / (float)0x1000000;
}
static int ri(int n) {
    if (n <= 1) return 0;
    return (int)(esp_random() % (uint32_t)n);
}
static bool rc(float prob) { return rand01() < prob; }

// ---- MS helpers ----
BassEngine::MS BassEngine::mn(int note, float len, int vel, int fcc,
                               float pb, float ts) {
    return {note, len, vel, fcc, pb, ts};
}

void BassEngine::msInit(MS m[16]) {
    for (int i = 0; i < 16; i++) m[i] = {-1, 0.f, 0, 0, 0.f, 0.f};
}

// ---- Interpreted note generation ----
// Bridges esp_random() (true HW entropy, matching how the rest of this
// file already sources randomness) into bassline_interpreter.h's
// injectable RngSource interface.
struct EspRng : bli::RngSource {
    float next01() override { return rand01(); }
};

void BassEngine::genInterpreted(MS m[16], int root, int scIdx) {
    EspRng rng;
    bli::Step steps[16];
    bli::generateMotif(steps, root, scIdx, m_dials, rng);
    for (int i = 0; i < 16; i++) {
        if (steps[i].note < 0) {
            m[i] = {-1, 0.f, 0, 0, 0.f, 0.f};
        } else {
            m[i] = mn(steps[i].note, steps[i].len, steps[i].vel, steps[i].fcc, steps[i].pb);
        }
    }
}

// ---- Anchor: prevent drift from tonal center ----
// Ensures step 0 and step 8 stay on strong tones; clamps all notes to a
// singable range (root-2 semitones to root+14) so continuity blending
// can't walk notes into incoherent register extremes.
void BassEngine::anchorMotif(MS m[16], int root, int scIdx) {
    int sc4v = sc(scIdx, 4);  // 5th degree -- strong mid-bar landing tone

    // Step 0: must be root (or very close)
    if (m[0].note < 0 || std::abs(m[0].note - root) > 2)
        m[0] = mn(root, 0.5f, 120, 90);

    // Step 8 (mid-phrase downbeat): root or 5th
    if (m[8].note < 0)
        m[8] = mn(root + (rc(0.4f) ? sc4v : 0), 0.5f, 112, 85);

    // Clamp all notes -- upper: root+15 (bass stays in bass register)
    // Lower: root-12 (allow sub-bass octave; tighter clamp was cancelling sub-bass drops)
    for (int i = 0; i < 16; i++) {
        if (m[i].note < 0) continue;
        while (m[i].note > root + 15) m[i].note -= 12;
        while (m[i].note < root - 12) m[i].note += 12;
    }
}

void BassEngine::clampRange(MS m[16], int root) {
    for (int i = 0; i < 16; i++) {
        if (m[i].note < 0) continue;
        while (m[i].note > root + 15) m[i].note -= 12;
        while (m[i].note < root - 12) m[i].note += 12;
    }
}

// ---- Motif transforms ----
// type: 0=simplify, 1=subDrop, 2=modalShift, 3=drift
void BassEngine::transformMotif(const MS src[16], MS dst[16], int type,
                                 int root, int scIdx) {
    memcpy(dst, src, 16 * sizeof(MS));
    if (type == 0) { // simplify: thin out non-downbeats
        for (int i = 0; i < 16; i++)
            if (dst[i].note >= 0 && i%4 != 0 && rc(0.6f)) dst[i].note = -1;
    } else if (type == 1) { // subDrop: drop octave + close filter
        for (int i = 0; i < 16; i++) {
            if (dst[i].note < 0) continue;
            if (dst[i].note > 36) dst[i].note -= 12;
            dst[i].fcc = std::max(10, dst[i].fcc - 30);
        }
    } else if (type == 2) { // modalShift: shift offbeats by 5th or 3rd
        for (int i = 0; i < 16; i++) {
            if (dst[i].note < 0 || i%4 == 0) continue;
            if (rc(0.5f)) dst[i].note += rc(0.5f) ? 7 : 3;
        }
    } else { // drift: thin out a beat's worth of hits for rhythmic variation
        int dropBeat = (ri(3)+1) * 4;
        for (int i = 0; i < 16; i++) {
            if (i >= dropBeat && i < dropBeat+4) dst[i].note = -1;
        }
        // Reinforce the tail with a clean root hit for phrase pull
        if (dst[15].note < 0 && rc(0.5f))
            dst[15] = mn(root, 0.15f, 65, 45);
    }
}

// ---- Turnarounds ----
// tStart = 1016 (= 64 bars x 16 steps - 8 trailing steps)
static const float T = 1016.f;

void BassEngine::addNote(float pos, int note, float len, int vel, int fcc, float pb) {
    // Remove any existing note at this exact position
    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [pos](const NoteSlot& n) { return std::fabs(n.pos - pos) < 0.01f; }),
        m_phrase.end());
    m_phrase.push_back({pos, note, len, vel, fcc, pb});
}

// Turnaround tail: hands off to bassline_interpreter.h's
// generateTurnaround (same harmony-gravity/artic-driven cadence logic
// validated in the DawDreamer experiments), then injects the resulting
// notes into m_phrase the same way the old genre-specific turnXXX
// functions did.
void BassEngine::turnInterpreted(int base, int scIdx) {
    EspRng rng;
    bli::TurnNote tn[3];
    int n = bli::generateTurnaround(tn, base, scIdx, m_dials, rng);
    for (int i = 0; i < n; i++) {
        addNote(T + tn[i].stepOffset, tn[i].note, tn[i].len, tn[i].vel, tn[i].fcc, tn[i].pb);
    }
}

// ---- Phrase regeneration ----
void BassEngine::regeneratePhrase(bool advanceArc) {
    if (advanceArc) m_phraseCount++;
    m_phrase.clear();

    // Hold scale for 2-4 phrases for harmonic continuity, then re-pick.
    // Base index comes from the harmonyColor dial (banded across all 9
    // scales); a little jitter keeps successive holds from being bit-for-
    // bit identical when the dial hasn't moved, echoing the old code's
    // "occasional harmonic shift" feel without needing a fixed per-genre
    // scale pool.
    if (m_scaleHoldCount <= 0) {
        int base = m_dials.pickScaleIdx();
        int jitter = rc(0.35f) ? (ri(3) - 1) : 0;  // -1, 0, or +1
        m_scaleIdx = std::max(0, std::min(bli::kNumScales - 1, base + jitter));
        m_scaleHoldCount = 2 + ri(3);  // hold 2-4 phrases
    } else {
        m_scaleHoldCount--;
    }
    m_progIdx = ri(kNumProgs);
    const int* prog = kProgs[m_progIdx];

    // Generate motif A -- blend with previous if available for continuity
    MS motA[16], motB[16], motC[16], motD[16];
    MS freshA[16];
    genInterpreted(freshA, ROOT, m_scaleIdx);

    // Blend: rhythm (which positions fire) always comes from freshA -- preserves the hook.
    // Only blend pitches where both phrases have a note, for harmonic smoothness.
    if (m_hasPrevMotA) {
        for (int i = 0; i < 16; i++) {
            if (freshA[i].note >= 0 && m_prevMotA[i].note >= 0 && rc(0.4f))
                motA[i] = m_prevMotA[i];  // carry pitch/vel from prev, rhythm from fresh
            else
                motA[i] = freshA[i];
        }
    } else {
        memcpy(motA, freshA, sizeof(motA));
    }
    memcpy(m_prevMotA, motA, sizeof(motA));
    m_hasPrevMotA = true;

    // Anchor to tonal center after blend (prevents drift over time)
    anchorMotif(motA, ROOT, m_scaleIdx);

    // Transforms -- complexity arc: simpler early, richer over time
    // phraseCount mod 8 cycles through an arc: simplify -> modal -> drift -> full
    int arc = m_phraseCount % 8;
    int xformB = (arc < 2) ? 0 : (arc < 5) ? 2 : ri(4); // simplify -> modal -> random
    int xformC = (arc < 3) ? 0 : (arc < 6) ? 1 : ri(4); // simplify -> subDrop -> random
    transformMotif(motA, motB, xformB, ROOT, m_scaleIdx); clampRange(motB, ROOT);
    transformMotif(motA, motC, xformC, ROOT, m_scaleIdx); clampRange(motC, ROOT);
    transformMotif(motA, motD, 3,      ROOT, m_scaleIdx); clampRange(motD, ROOT);

    // Phrase structure driven by the motion-variation dial: low = repetitive
    // (AAAD), high = complex (ABAC) -- same shape the old ctrl2 drove.
    float varVal = m_dials.motionVariation;
    const char* structure;
    float rnd = rand01();
    if      (varVal < 0.2f) structure = "AAAD";
    else if (varVal < 0.5f) structure = (rnd < 0.6f) ? "AAAB" : "ABAB";
    else if (varVal < 0.8f) structure = (rnd < 0.5f) ? "ABAC" : "AAAB";
    else                    structure = "ABAC";

    // 64-bar arrangement: 4 sections x 16 bars each
    // Each section gets a distinct chord progression and motif character:
    //   sec0 (bars  0-15): core groove   -- variation-driven structure, prog A
    //   sec1 (bars 16-31): bridge        -- B/A alternation,             prog B
    //   sec2 (bars 32-47): peak energy   -- C/B push,                    prog C
    //   sec3 (bars 48-63): reprise       -- variation-driven structure,  prog A
    // Within each section a 4-bar chord cell repeats 4x -- hypnotic, not identical
    int secProgs[4] = {m_progIdx, (m_progIdx + 1) % kNumProgs,
                        (m_progIdx + 2) % kNumProgs, m_progIdx};

    // Swing offset for odd steps -- folded from the groove-swing dial
    // (replaces the old fixed per-genre cfg.swingSteps).
    float swingSteps = m_dials.grooveSwing * 0.4f;

    // Build 64-bar phrase (256 beats = 1024 steps)
    for (int bar = 0; bar < 64; bar++) {
        int sec       = bar / 16;
        int barInSec  = bar % 16;
        int barInCell = barInSec % 4;

        const int* secProg = kProgs[secProgs[sec]];
        int rootOffset = secProg[barInCell];

        char part;
        if (sec == 0 || sec == 3) {
            part = structure[barInCell];              // dial-driven shape
        } else if (sec == 1) {
            static const char BRIDGE[4] = {'B','A','B','A'};
            part = BRIDGE[barInCell];                 // bridge: variation leads
        } else {
            static const char PEAK[4]   = {'C','B','C','B'};
            part = PEAK[barInCell];                   // peak: sub-drop + modal push
        }

        const MS* motif = (part=='A') ? motA : (part=='B') ? motB :
                          (part=='C') ? motC : motD;

        for (int i = 0; i < 16; i++) {
            if (motif[i].note < 0) continue;
            int note = motif[i].note + rootOffset;
            note = std::max(0, std::min(127, note));

            // Convert time-shift from beats to steps (* 4)
            float tsSteps = motif[i].ts * 4.f;
            float basePos = (float)(bar*16 + i);
            float pos = basePos + tsSteps;

            // Apply swing to odd base steps
            if (i % 2 != 0) pos += swingSteps;

            m_phrase.push_back({pos, note, motif[i].len, motif[i].vel,
                                 motif[i].fcc, motif[i].pb});
        }
    }

    // Turnaround - erase steps 248..255 then inject new ones
    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [](const NoteSlot& n) { return n.pos >= T; }), m_phrase.end());

    // More articulated dial settings call for more frequent flourishes.
    int turnaroundRate = (m_dials.voiceArtic > 0.6f) ? 2 : 4;
    if (m_phraseCount % turnaroundRate == 0) {
        int baseOffset = prog[15 % 4];
        int tBase = std::max(0, std::min(127, ROOT + baseOffset));
        turnInterpreted(tBase, m_scaleIdx);
    }

    // Sort by position
    std::sort(m_phrase.begin(), m_phrase.end(),
              [](const NoteSlot& a, const NoteSlot& b) { return a.pos < b.pos; });

    ESP_LOGI(TAG, "Phrase[%d] bank=%d scale=%s prog=%d notes=%d bars=64",
             m_phraseCount, m_activeBank, bli::scaleName(m_scaleIdx), m_progIdx,
             (int)m_phrase.size());
}

// ---- MIDI output ----
void BassEngine::playNote(const NoteSlot& n, double bpm) {
    int note = std::max(0, std::min(127, n.note));
    int vel  = std::max(1, std::min(127, n.velocity));
    int fcc  = std::max(0, std::min(127, n.filterCC));

    // Note On
    uint8_t noteOn[3] = {0x90, (uint8_t)note, (uint8_t)vel};
    send_midi_message(noteOn, 3);

    // CC74 (brightness / filter)
    send_midi_cc(1, 74, (uint8_t)fcc);

    // Pitch bend
    if (n.pitchBend != 0.f) {
        int raw = 8192 + (int)(n.pitchBend / 12.f * 8191.f);
        raw = std::max(0, std::min(16383, raw));
        uint8_t pb[3] = {0xE0, (uint8_t)(raw & 0x7F), (uint8_t)(raw >> 7)};
        send_midi_message(pb, 3);
    } else {
        // Centre pitch bend
        uint8_t pb[3] = {0xE0, 0x00, 0x40};
        send_midi_message(pb, 3);
    }

    // Schedule note-off: length in 16th notes -> length in phrase steps = length
    // (1 step = 1/16 note), note-off phrasePos = pos + length
    double offPos = n.pos + n.length;
    m_activeNotes.push_back({note, offPos});
}

void BassEngine::processNoteOffs(double phrasePos, double /*bpm*/) {
    for (auto it = m_activeNotes.begin(); it != m_activeNotes.end(); ) {
        // Handle wrap-around at phrase boundary
        double diff = phrasePos - it->offPos;
        if (diff >= 0.0 || diff < -200.0) { // second condition catches wrap
            uint8_t noteOff[3] = {0x80, (uint8_t)it->note, 0x00};
            send_midi_message(noteOff, 3);
            it = m_activeNotes.erase(it);
        } else {
            ++it;
        }
    }
}

// ---- Main process tick ----
BassEngine::BassEngine() = default;

void BassEngine::setActiveBank(int bank) {
    if (bank < 0 || bank >= bli::BANK_COUNT) return;
    m_activeBank = bank;

    if (!m_active) {
        // First-ever bank selection starts playback with whatever dial
        // values are already set (the constructor's defaults, unless a
        // pot was somehow moved before any pad tap -- harmless either
        // way). Later bank switches only retarget the 2 pots; they never
        // interrupt or regenerate the phrase that's already playing.
        m_active = true;
        m_lastPos      = -1.0;
        m_lastBar      = -1;
        m_regenPending = false;
        m_phraseCount    = 0;
        m_hasPrevMotA    = false;
        m_scaleHoldCount = 0;
        regeneratePhrase();
    }
    ESP_LOGI(TAG, "Bank set to %d", m_activeBank);
}

void BassEngine::setDial(int dialIdx, float v01) {
    m_dials.set(m_activeBank, dialIdx, v01);
    if (m_active) m_regenPending = true;
}

void BassEngine::nudge() {
    // Reroll the current bar with fresh randomness, same dial-driven
    // character -- goes through the same bar-boundary hot-swap as a dial
    // change, so it never clicks or interrupts mid-note.
    if (m_active) m_regenPending = true;
}

void BassEngine::stop() {
    m_active = false;
    // All notes off
    uint8_t allOff[3] = {0xB0, 123, 0};
    send_midi_message(allOff, 3);
    m_activeNotes.clear();
    ESP_LOGI(TAG, "Engine stopped");
}

void BassEngine::process(const ableton::Link::SessionState& state,
                         const std::chrono::microseconds& time) {
    if (!m_active) return;

    double bpm = state.tempo();
    double beat = state.beatAtTime(time, 16.0);
    if (beat < 0.0) return;

    // Map beat to position within 1024-step phrase.
    // 1024 steps = 256 beats (64 bars x 4 beats/bar).
    double phrasePos = std::fmod(beat * 4.0, 1024.0);

    // Phrase wrap: advance arc and regenerate if pending
    if (m_lastPos >= 0.0 && phrasePos < m_lastPos - 512.0) {
        regeneratePhrase(true);
        m_regenPending = false;
        m_lastBar = -1;
    }

    // Bar-boundary hot-swap: apply user-triggered ctrl changes within ~1 bar
    int currentBar = static_cast<int>(phrasePos / 16);
    if (m_regenPending && m_lastBar >= 0 && currentBar != m_lastBar) {
        regeneratePhrase(false);  // no arc advance -- just apply new knob values
        m_regenPending = false;
    }
    m_lastBar = currentBar;

    if (m_lastPos < 0.0) {
        m_lastPos = phrasePos;
        return; // skip first tick to avoid replaying old notes
    }

    // Process note-offs first
    processNoteOffs(phrasePos, bpm);

    // Play any notes that fall in (m_lastPos, phrasePos]
    // Handle wrap-around: if phrasePos < m_lastPos we wrapped
    bool wrapped = (phrasePos < m_lastPos);
    for (const auto& n : m_phrase) {
        bool due;
        if (!wrapped) {
            due = (n.pos > m_lastPos && n.pos <= phrasePos);
        } else {
            due = (n.pos > m_lastPos || n.pos <= phrasePos);
        }
        if (due) {
            playNote(n, bpm);
        }
    }

    m_lastPos = phrasePos;
}

// Global instance
BassEngine g_bassEngine;
