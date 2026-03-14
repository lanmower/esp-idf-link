// bass_engine.cpp
// C++ port of the 8-genre bass algorithm from the HTML prototype.
// Phrase = 256 steps (16 bars × 16 steps).  1 step = 1/16 note.
// Link beat → phrase pos:  pos = fmod(beat * 4.0, 256.0)

#include "bass_engine.h"
#include "io_helpers.h"
#include "esp_log.h"
#include "esp_random.h"
#include <algorithm>
#include <cstring>
#include <cmath>

static const char* TAG = "BASS";

// ── Fixed root note (E2) – hardware has no root-note menu ─────────────────
static const int ROOT = 40;

// ── Scales [index][degree] ─────────────────────────────────────────────────
//  0 dorian, 1 aeolian, 2 phrygian, 3 minorPentatonic,
//  4 melodicMinor, 5 mixolydian, 6 phrygianDom, 7 harmonicMinor, 8 hungarianMinor
static const int SCALE_LENS[9] = {7,7,7,5,7,7,7,7,7};
static const int SCALES[9][7] = {
    {0,2,3,5,7,9,10},  // 0 dorian
    {0,2,3,5,7,8,10},  // 1 aeolian
    {0,1,3,5,7,8,10},  // 2 phrygian
    {0,3,5,7,10,3,5},  // 3 minorPentatonic (5 real notes, last 2 repeated)
    {0,2,3,5,7,9,11},  // 4 melodicMinor
    {0,2,4,5,7,9,10},  // 5 mixolydian
    {0,1,4,5,7,8,10},  // 6 phrygianDom
    {0,2,3,5,7,8,11},  // 7 harmonicMinor
    {0,2,3,6,7,8,11},  // 8 hungarianMinor
};

// Safe scale degree access
static int sc(int scIdx, int degree) {
    int len = SCALE_LENS[scIdx];
    return SCALES[scIdx][degree % len];
}

// ── Genre configuration ────────────────────────────────────────────────────
struct GenreCfg {
    int   turnaroundRate;   // fire turnaround every N phrase matrices
    float swingSteps;       // step offset for odd steps (pre-computed at genre default BPM)
    int   scales[2];        // two scale options to pick from
    int   progs[3][4];      // three chord-progression options
};

static const GenreCfg GCFG[8] = {
    // FUNK  (105 BPM, swing 0.04s → 0.28 steps)
    {2, 0.28f, {5,0}, {{0,0,0,5},{0,3,4,5},{0,0,-2,0}}},
    // ITALO (120 BPM, swing 0)
    {4, 0.00f, {8,2}, {{0,-4,-5,-2},{0,0,-1,-4},{0,3,0,-5}}},
    // SYNTHPOP (115 BPM, swing 0.015s → 0.115 steps)
    {2, 0.12f, {1,7}, {{0,-4,-2,-5},{0,8,7,0},{0,3,7,8}}},
    // PSYTRANCE (142 BPM, swing 0)
    {4, 0.00f, {6,2}, {{0,0,0,1},{0,0,-5,-4},{0,0,-4,-5}}},
    // PROG (124 BPM, swing 0.01s → 0.083 steps)
    {4, 0.08f, {1,2}, {{0,8,7,0},{0,1,0,-4},{0,-5,-4,-2}}},
    // AFROHOUSE (122 BPM, swing 0.035s → 0.285 steps)
    {4, 0.29f, {1,2}, {{0,0,-2,0},{0,0,-4,-5},{0,3,0,-4}}},
    // UKG (132 BPM, swing 0.05s → 0.44 steps)
    {2, 0.44f, {1,3}, {{0,-5,-4,-2},{0,3,-2,-4},{0,0,5,3}}},
    // GFUNK (95 BPM, swing 0.045s → 0.285 steps)
    {1, 0.29f, {3,3}, {{0,0,-2,5},{0,-4,-2,0},{0,5,-2,-4}}},
};

// ── Random helpers ─────────────────────────────────────────────────────────
static float rand01() {
    return (float)(esp_random() & 0xFFFFFF) / (float)0x1000000;
}
static int ri(int n) {
    if (n <= 1) return 0;
    return (int)(esp_random() % (uint32_t)n);
}
static bool rc(float prob) { return rand01() < prob; }

template<int N>
static int r_arr(const int (&arr)[N]) { return arr[ri(N)]; }

// ── MS helpers ─────────────────────────────────────────────────────────────
BassEngine::MS BassEngine::mn(int note, float len, int vel, int fcc,
                               float pb, float ts) {
    return {note, len, vel, fcc, pb, ts};
}

void BassEngine::msInit(MS m[16]) {
    for (int i = 0; i < 16; i++) m[i] = {-1, 0.f, 0, 0, 0.f, 0.f};
}

// ── Genre generators ───────────────────────────────────────────────────────

void BassEngine::genFunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    m[0] = mn(root, 0.4f, 127, 110);

    static const int GROOVES[4][4] = {{3,7,10,14},{2,5,8,14},{3,6,11,14},{2,5,9,13}};
    const int* groove = GROOVES[ri(4)];

    for (int k = 0; k < 4; k++) {
        int s = groove[k];
        if (!rc(0.3f + c1*0.7f)) continue;
        bool isPop = rc(0.1f + c1*0.5f);
        int n = root;
        if (isPop) {
            n += 12;
        } else if (rc(c2*0.9f)) {
            static const int filth[] = {6,1,10,3};
            n += r_arr(filth);
        } else {
            static const int pocket[] = {0,0,5,7};
            n += r_arr(pocket);
        }
        m[s] = mn(n, 0.25f, isPop?127:105, isPop?120:80);
        if (s > 0 && m[s-1].note < 0 && rc(0.3f + c2*0.6f))
            m[s-1] = mn(root, 0.1f, 45, 20);
    }
    if (c2 > 0.3f && rc(c2) && m[15].note < 0)
        m[15] = mn(root+6, 0.2f, 100, 75);
}

void BassEngine::genItalo(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc2 = sc(scIdx, 2);
    int sc1v = sc(scIdx, 1);
    int sc4  = sc(scIdx, 4);
    int degs[3] = {12, sc2 ? sc2 : 3, sc1v ? sc1v : sc4 ? sc4 : 7};

    for (int s = 0; s < 16; s++) {
        bool isAccent = (s % 4 == 0);
        float len = 0.6f - c2*0.45f;
        int vel = isAccent ? 120 : (s%2==0 ? 100 : 80);
        int n = root;
        if (c1 > 0.1f && s%2 != 0) {
            int deg = degs[ri(3)];
            if (rc(c1)) n += deg;
        }
        m[s] = mn(n, len, vel, vel-20);
    }
}

void BassEngine::genSynthpop(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    // Three arp patterns and gap patterns
    static const int ARPS[3][4] = {{0,0,12,0},{0,3,7,0},{0,12,0,7}};
    static const int GAPS[2][4] = {{3,7,11,15},{1,5,9,13}};
    const int* arp = ARPS[ri(3)];
    const int* gaps = GAPS[ri(2)];

    int sc2v = sc(scIdx, 2);
    int sc5v = sc(scIdx, 5);

    for (int s = 0; s < 16; s++) {
        // Syncopated gaps
        bool isGap = false;
        for (int g = 0; g < 4; g++) if (gaps[g] == s) { isGap = true; break; }
        if (isGap && rc(0.3f + c2*0.7f)) continue;
        if (s >= 14 && rc(c2)) continue;

        int interval = arp[(s/2) % 4];
        // Twilight tension on back half
        if (s >= 8 && rc(c1*0.9f))
            interval = rc(0.5f) ? (sc2v ? sc2v : 3) : (sc5v ? sc5v : 8);

        m[s] = mn(root + interval, 0.7f, 105, 90);
    }
}

void BassEngine::genPsytrance(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    bool bind = (c1 >= 0.98f);
    int sc1v = sc(scIdx, 1);

    for (int s = 0; s < 16; s++) {
        if (s % 4 == 0) continue; // kick slot: silent
        int stepInBeat = s % 4;
        // Gallop skip
        if (c2 > 0.2f && rc(c2*0.5f) && (stepInBeat == 1 || stepInBeat == 3)) continue;

        bool isAccent = ((s+1) % 4 == 0);
        int n = bind ? root : (rc(c1) ? root : root + (isAccent && rc(0.4f) ? sc1v : 0));

        // Time-shift (stored in ts as beats; converted to steps (* 4) when building phrase)
        float ts = 0.f;
        float vm = 1.0f;
        if (c2 <= 0.5f) {
            float swingAmt = (c2 / 0.5f) * 0.08f;  // 0-0.08 seconds
            if (stepInBeat == 1 || stepInBeat == 3) ts = swingAmt;
        } else {
            float t = (c2 - 0.5f) * 2.0f;
            if (stepInBeat == 1) { ts = t * 0.04f; }
            if (stepInBeat == 2) { ts = t * 0.083f; }
            if (stepInBeat == 3) { ts = t * 0.16f; vm = 1.0f - t*0.8f; }
        }
        if (vm < 0.1f) continue;

        int vel = (int)((isAccent ? 120 : 90) * vm);
        int fcc = (int)((isAccent ? 100 : 50) * vm);
        m[s] = mn(n, 0.25f, vel, fcc, 0.f, ts);
    }
}

void BassEngine::genProg(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    static const int PATS[3][5] = {{2,5,8,11,14},{2,6,10,14,-1},{3,6,9,12,14}};
    const int* pat = PATS[ri(3)];
    int patLen = 5;

    int sc2v = sc(scIdx, 2);
    int sc6v = sc(scIdx, 6);

    for (int idx = 0; idx < patLen; idx++) {
        if (pat[idx] < 0) break;
        int s = pat[idx];
        bool tie = (idx+1 < patLen && pat[idx+1] >= 0 && pat[idx+1] - s <= 2);
        int jump = rc(c2) ? r_arr((const int[3]){sc2v?sc2v:3, sc6v?sc6v:10, -12}) : 0;
        float len = tie ? 0.6f + c1*1.5f : 0.4f + c1*0.4f;
        m[s] = mn(root + jump, len, 115, 60);
    }
}

void BassEngine::genAfrohouse(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    static const int PATS[3][5] = {{3,6,8,11,14},{0,4,7,12,14},{2,5,8,10,13}};
    const int* pat = PATS[ri(3)];
    int sc2v = sc(scIdx, 2);

    for (int idx = 0; idx < 5; idx++) {
        int s = pat[idx];
        if (rc(c1) && s != 0 && s != 8) continue;
        int n = root;
        if (idx%2 != 0 && rc(0.4f)) n += sc2v;
        if (rc(c2)) n -= 12;
        m[s] = mn(n, 0.5f, 105, (int)(70 - c2*30));
    }
}

void BassEngine::genUkg(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    static const int PATS[3][5] = {{0,3,8,10,13},{2,5,8,11,14},{0,7,10,13,15}};
    const int* pat = PATS[ri(3)];

    int sc2v = sc(scIdx, 2);
    int sc4v = sc(scIdx, 4);
    int sc6v = sc(scIdx, 6);
    static const int twilight[4] = {3, 7, 10, 12};

    for (int idx = 0; idx < 5; idx++) {
        int s = pat[idx];
        bool isDown = (s == 0 || s == 8);
        if (isDown && rc(c2)) continue; // mute downbeats

        int n = root;
        float pb = 0.f;
        if (!isDown && rc(c1)) {
            // Twilight jumps
            static const int tw[4] = {3,7,10,12};
            n += tw[ri(4)];
            if (rc(0.4f)) { static const int bends[] = {-2,2,-12}; pb = r_arr(bends); }
        }
        m[s] = mn(n, isDown ? 0.8f : 0.3f+c2*0.2f, 115, isDown?110:80, pb);
    }
}

void BassEngine::genGfunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc3v = sc(scIdx, 3);
    int sc4v = sc(scIdx, 4);
    int sc2v = sc(scIdx, 2);

    m[0] = mn(root, 1.5f, 120, 90);

    if (rc(0.2f + c2*0.8f)) m[3] = mn(root+12, 0.5f, 100, 70);
    if (rc(0.4f + c2*0.6f)) m[6] = mn(root+(sc2v?sc2v:3), 0.75f, 110, 80);
    m[8] = mn(root, 1.5f, 115, 85);

    if (rc(c1 + 0.3f)) {
        m[13] = mn(root+(sc3v?sc3v:5), 0.5f, 100, 80);
        float slideSize = rc(0.5f) ? -2.f : 3.f;
        m[14] = mn(root+(sc4v?sc4v:7), 1.0f, 120, 100, slideSize*c1*3.f);
    } else {
        if (rc(0.5f)) m[14] = mn(root-12, 1.0f, 110, 80, 2.f);
    }
}

// ── Motif transforms ───────────────────────────────────────────────────────
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
    } else { // drift: random drop + occasional octave pop
        int dropBeat = (ri(3)+1) * 4;
        for (int i = 0; i < 16; i++) {
            if (i >= dropBeat && i < dropBeat+2) { dst[i].note = -1; continue; }
            if (dst[i].note >= 0 && rc(0.2f)) {
                dst[i].note += 12;
                dst[i].vel = 127;
            }
        }
        if (dst[15].note < 0 && rc(0.5f))
            dst[15] = mn(root, 0.15f, 60, 40);
    }
}

// ── Turnarounds ────────────────────────────────────────────────────────────
// tStart = 248 (= 16 bars × 16 steps – 8 trailing steps)
static const float T = 248.f;

void BassEngine::addNote(float pos, int note, float len, int vel, int fcc, float pb) {
    // Remove any existing note at this exact position
    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [pos](const NoteSlot& n) { return std::fabs(n.pos - pos) < 0.01f; }),
        m_phrase.end());
    m_phrase.push_back({pos, note, len, vel, fcc, pb});
}

void BassEngine::turnFunk(int base, int scIdx, float c1, float c2) {
    int sc2v = sc(scIdx, 2);
    addNote(T+4, base+5,                          0.5f, 100, 80);
    addNote(T+5.5f, base+(c2>0.5f ? 6 : (sc2v?sc2v:3)), 0.5f, 110, 90);
    addNote(T+7, base+7,                          0.5f, 127, 127);
}
void BassEngine::turnItalo(int base) {
    addNote(T+4, base+12, 0.5f, 120, 120);
    addNote(T+6, base+7,  0.5f, 120,  90);
}
void BassEngine::turnSynthpop(int base) {
    addNote(T+4, base+12, 1.5f, 110, 110, -12.f);
}
void BassEngine::turnPsytrance(int base, int scIdx, float c1) {
    bool bind = (c1 >= 0.98f);
    addNote(T+4, bind ? base : base+1,   0.25f, 110, 100);
    addNote(T+5, bind ? base : base+4,   0.25f, 110, 110);
    addNote(T+7, bind ? base : base+12,  0.25f, 127, 127);
}
void BassEngine::turnProg(int base) {
    addNote(T+6, base-12, 1.0f, 100, 80);
}
void BassEngine::turnAfrohouse(int base) {
    addNote(T+6, base-12, 1.5f, 100, 40);
}
void BassEngine::turnUkg(int base) {
    addNote(T+5.5f, base+12, 0.4f, 110,  90);
    addNote(T+7,    base,    1.0f, 120, 100, 2.f);
}
void BassEngine::turnGfunk(int base, int scIdx, float c1) {
    int sc2v = sc(scIdx, 2);
    addNote(T+6, base+(sc2v?sc2v:3), 2.0f, 110, 90, -3.f*c1);
}

// ── Phrase regeneration ────────────────────────────────────────────────────
void BassEngine::regeneratePhrase() {
    m_phraseCount++;
    m_phrase.clear();

    const GenreCfg& cfg = GCFG[m_genre];

    // Pick scale and chord progression
    m_scaleIdx = cfg.scales[ri(2)];
    m_progIdx  = ri(3);
    const int* prog = cfg.progs[m_progIdx];

    // Generate motif A
    MS motA[16], motB[16], motC[16], motD[16];
    switch (m_genre) {
        case GENRE_FUNK:      genFunk     (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_ITALO:     genItalo    (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_SYNTHPOP:  genSynthpop (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_PSYTRANCE: genPsytrance(motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_PROG:      genProg     (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_AFROHOUSE: genAfrohouse(motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_UKG:       genUkg      (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_GFUNK:     genGfunk    (motA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
    }

    // Transforms
    static const int XFORMS[4] = {0, 1, 2, 3};
    transformMotif(motA, motB, XFORMS[ri(4)], ROOT, m_scaleIdx);
    transformMotif(motA, motC, XFORMS[ri(4)], ROOT, m_scaleIdx);
    transformMotif(motA, motD, 3,              ROOT, m_scaleIdx); // drift for D

    // Phrase structure
    float varVal = 0.35f; // fixed mid-variance
    const char* structure;
    float rnd = rand01();
    if      (varVal < 0.2f) structure = "AAAD";
    else if (varVal < 0.5f) structure = (rnd < 0.6f) ? "AAAB" : "ABAB";
    else if (varVal < 0.8f) structure = (rnd < 0.5f) ? "ABAC" : "AAAB";
    else                    structure = "ABAC";

    // Psytrance monotony override
    bool psy_bind = (m_genre == GENRE_PSYTRANCE && m_ctrl1 >= 0.98f);

    // Build 16-bar phrase
    for (int bar = 0; bar < 16; bar++) {
        int rootOffset = prog[bar % 4];
        if (psy_bind) rootOffset = 0;

        char part = structure[bar % 4];
        const MS* motif = (part=='A') ? motA : (part=='B') ? motB :
                          (part=='C') ? motC : motD;

        for (int i = 0; i < 16; i++) {
            if (motif[i].note < 0) continue;
            int note = motif[i].note + rootOffset;
            if (psy_bind) note = ROOT;
            note = std::max(0, std::min(127, note));

            // Convert time-shift from beats to steps (* 4)
            float tsSteps = motif[i].ts * 4.f;
            float basePos = (float)(bar*16 + i);
            float pos = basePos + tsSteps;

            // Apply genre swing to odd base steps
            if (i % 2 != 0) pos += cfg.swingSteps;

            m_phrase.push_back({pos, note, motif[i].len, motif[i].vel,
                                 motif[i].fcc, motif[i].pb});
        }
    }

    // Turnaround – erase steps 248..255 then inject new ones
    m_phrase.erase(std::remove_if(m_phrase.begin(), m_phrase.end(),
        [](const NoteSlot& n) { return n.pos >= T; }), m_phrase.end());

    if (m_phraseCount % cfg.turnaroundRate == 0) {
        int baseOffset = psy_bind ? 0 : prog[15 % 4];
        int tBase = std::max(0, std::min(127, ROOT + baseOffset));
        switch (m_genre) {
            case GENRE_FUNK:      turnFunk     (tBase, m_scaleIdx, m_ctrl1, m_ctrl2); break;
            case GENRE_ITALO:     turnItalo    (tBase);                               break;
            case GENRE_SYNTHPOP:  turnSynthpop (tBase);                               break;
            case GENRE_PSYTRANCE: turnPsytrance(tBase, m_scaleIdx, m_ctrl1);          break;
            case GENRE_PROG:      turnProg     (tBase);                               break;
            case GENRE_AFROHOUSE: turnAfrohouse(tBase);                               break;
            case GENRE_UKG:       turnUkg      (tBase);                               break;
            case GENRE_GFUNK:     turnGfunk    (tBase, m_scaleIdx, m_ctrl1);          break;
        }
    }

    // Sort by position
    std::sort(m_phrase.begin(), m_phrase.end(),
              [](const NoteSlot& a, const NoteSlot& b) { return a.pos < b.pos; });

    ESP_LOGI(TAG, "Phrase[%d] genre=%d scale=%d prog=%d notes=%d",
             m_phraseCount, m_genre, m_scaleIdx, m_progIdx, (int)m_phrase.size());
}

// ── MIDI output ────────────────────────────────────────────────────────────
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

    // Schedule note-off: length in 16th notes → length in phrase steps = length
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

// ── Main process tick ──────────────────────────────────────────────────────
BassEngine::BassEngine() = default;

void BassEngine::setGenre(int genre_idx) {
    if (genre_idx < 0 || genre_idx >= GENRE_COUNT) return;
    bool changed = (m_genre != genre_idx) || !m_active;
    m_genre  = genre_idx;
    m_active = true;
    if (changed) {
        // Stop any sounding notes
        for (auto& an : m_activeNotes) {
            uint8_t noteOff[3] = {0x80, (uint8_t)an.note, 0x00};
            send_midi_message(noteOff, 3);
        }
        m_activeNotes.clear();
        m_phraseCount = 0;
        m_lastPos     = -1.0;
        regeneratePhrase();
    }
    ESP_LOGI(TAG, "Genre set to %d", m_genre);
}

void BassEngine::setCtrl1(float v) {
    m_ctrl1 = v;
    if (m_active) regeneratePhrase();
}

void BassEngine::setCtrl2(float v) {
    m_ctrl2 = v;
    if (m_active) regeneratePhrase();
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

    // Map beat to position within 256-step phrase.
    // 256 steps = 64 beats (16 bars × 4 beats/bar).
    double phrasePos = std::fmod(beat * 4.0, 256.0);

    // Detect phrase wrap → regenerate
    if (m_lastPos >= 0.0 && phrasePos < m_lastPos - 128.0) {
        regeneratePhrase();
    }

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
