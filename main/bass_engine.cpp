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
    // i→IV→bIII→bVII | i→bVII→IV→V | i→bIII→bVII→IV  (dark modal funk, avoids naive I-IV-V)
    {2, 0.28f, {5,0}, {{0,5,3,10},{0,10,5,7},{0,3,10,5}}},
    // ITALO (120 BPM, swing 0)
    // i→bVI→IV→bVII | i→bVII→bVI→V | i→IV→bVI→bVII  (Moroder drama, descending minor)
    {4, 0.00f, {8,2}, {{0,8,5,10},{0,10,8,7},{0,5,8,10}}},
    // SYNTHPOP (115 BPM, swing 0.015s → 0.115 steps)
    // i→bVI→bIII→V | i→IV→bVII→bVI | i→bIII→bVII→IV  (atmospheric, JM Jarre / DM feel)
    {2, 0.12f, {1,7}, {{0,8,3,7},{0,5,10,8},{0,3,10,5}}},
    // PSYTRANCE (142 BPM, swing 0)
    // mostly static with phrygian tension flicks (hypnotic repetition is the point)
    {4, 0.00f, {6,2}, {{0,0,1,0},{0,1,0,7},{0,0,10,1}}},
    // PROG (124 BPM, swing 0.01s → 0.083 steps)
    // i→bVI→bII→V | i→bIII→V→bVII | i→IV→bII→bVI  (bold chromaticism, Squarepusher/Meshuggah)
    {4, 0.08f, {1,2}, {{0,8,1,7},{0,3,7,10},{0,5,1,8}}},
    // AFROHOUSE (122 BPM, swing 0.035s → 0.285 steps)
    // hypnotic minimal root motion, occasional bVI/bVII colour
    {4, 0.29f, {1,2}, {{0,0,10,7},{0,8,0,10},{0,5,0,3}}},
    // UKG (132 BPM, swing 0.05s → 0.44 steps)
    // i→bVI→IV→bVII | i→bIII→bVII→V | i→IV→bVI→bIII
    {2, 0.44f, {1,3}, {{0,8,5,10},{0,3,10,7},{0,5,8,3}}},
    // GFUNK (95 BPM, swing 0.045s → 0.285 steps)
    // i→IV→i→bVII | i→V→IV→bIII | i→bIII→V→IV  (Dre / Warren G feel-good dark)
    {1, 0.29f, {3,3}, {{0,5,0,10},{0,7,5,3},{0,3,7,5}}},
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

// FUNK
// ctrl1 = pocket density: how many syncopated groove hits fire
// ctrl2 = melodic spice: 0=root+5th pocket, 1=chromatic approach notes + tritone stabs
void BassEngine::genFunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    // Strong downbeat always — length shortens with density for more percussive feel
    m[0] = mn(root, 0.5f - c1*0.2f, 127, 110);

    // Four groove templates weighted toward syncopation
    static const int GROOVES[4][4] = {{3,7,10,14},{2,6,9,13},{3,5,10,13},{2,7,11,14}};
    const int* groove = GROOVES[ri(4)];

    // Spicy note palette: scale tones + approach notes
    // Low spice: 5th, minor 7th  |  High spice: b5, b2, maj7 (approach notes)
    static const int POCKET[]  = {0, 5, 7, 10};          // root, 4th, 5th, m7
    static const int SPICE[]   = {6, 1, 11, 10, 3};      // b5, b2, maj7, m7, m3
    static const int APPROACH[] = {-1, 1, -2, 2};        // chromatic approaches to root

    for (int k = 0; k < 4; k++) {
        int s = groove[k];
        if (!rc(0.25f + c1*0.75f)) continue;

        int n = root;
        bool isApproach = rc(c2 * 0.5f);
        if (isApproach) {
            // Chromatic approach into root on the very next step
            n += r_arr(APPROACH);
            m[s] = mn(n, 0.13f, 90, 40);
        } else if (rc(c2)) {
            n += SPICE[ri(5)];
            m[s] = mn(n, 0.2f, 110, int(60 + c2*50));
        } else {
            n += POCKET[ri(4)];
            m[s] = mn(n, 0.25f, 105, 80);
        }
        // Ghost note one step before with higher c1
        if (s > 1 && m[s-1].note < 0 && rc(c1 * 0.6f))
            m[s-1] = mn(root, 0.1f, 40, 15);
    }
    // Tritone pickup on step 15 at high spice
    if (c2 > 0.5f && rc(c2 - 0.3f) && m[15].note < 0)
        m[15] = mn(root + 6, 0.15f, 95, 70);
}

// ITALO
// ctrl1 = melodic drive: 0=pure root drone, 1=active pitch movement on every offbeat
// ctrl2 = articulation: 0=legato pumping (long notes), 1=staccato attack (short, filter open)
void BassEngine::genItalo(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc2v = sc(scIdx, 2);
    int sc4v = sc(scIdx, 4);
    int sc6v = sc(scIdx, 6);
    // Walking note sequence built from scale degrees
    int walk[4] = {0, sc2v ? sc2v : 3, sc4v ? sc4v : 7, sc6v ? sc6v : 10};

    for (int s = 0; s < 16; s++) {
        bool isAccent = (s % 4 == 0);
        float len = 0.65f - c2 * 0.5f;          // legato→staccato
        int fcc = isAccent ? int(80 + c2*47) : int(50 + c2*60);  // filter opens on staccato
        int vel = isAccent ? 125 : (s%2==0 ? 105 : 85);

        int n = root;
        if (s % 2 != 0) {
            // Offbeats: walk through scale at high c1, hold root at low c1
            float walkChance = c1 * 0.9f;
            if (rc(walkChance)) {
                int step = (s / 2) % 4;
                n += walk[step];
                // At very high c1, occasional chromatic approach from above
                if (c1 > 0.7f && rc((c1 - 0.7f) * 2.0f))
                    n = root + (walk[step] > 0 ? walk[step] - 1 : 11);
            }
        }
        m[s] = mn(n, len, vel, fcc);
    }
}

// SYNTHPOP
// ctrl1 = harmonic reach: 0=root+octave patterns, 1=wide modal arpeggios with tension tones
// ctrl2 = syncopation: 0=straight 16ths, 1=highly gapped/syncopated
void BassEngine::genSynthpop(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc2v = sc(scIdx, 2);
    int sc4v = sc(scIdx, 4);
    int sc5v = sc(scIdx, 5);
    int sc6v = sc(scIdx, 6);

    // Arp shapes: low c1 = octave-based, high c1 = rich modal shapes
    // Each shape is 4 intervals cycling across 16 steps
    int shape[4];
    if (c1 < 0.33f) {
        // Simple octave movement
        const int S[4] = {0, 0, 12, 0};
        memcpy(shape, S, sizeof(shape));
    } else if (c1 < 0.66f) {
        // Scale-based arpeggio using 3rd and 5th
        shape[0] = 0;
        shape[1] = sc2v ? sc2v : 3;
        shape[2] = sc4v ? sc4v : 7;
        shape[3] = 12;
    } else {
        // Wider reach: 5th, octave, and modal tension tones
        shape[0] = 0;
        shape[1] = sc4v ? sc4v : 7;
        shape[2] = sc6v ? sc6v : 10;
        shape[3] = sc5v ? sc5v+12 : 9;  // high tension interval
    }

    // Gap template: more gaps at high c2
    bool gap[16] = {};
    int numGaps = int(2 + c2 * 6);
    static const int GAP_SLOTS[8] = {1, 3, 5, 7, 9, 11, 13, 15};
    for (int g = 0; g < numGaps && g < 8; g++) {
        if (rc(0.5f + c2 * 0.4f)) gap[GAP_SLOTS[g]] = true;
    }

    for (int s = 0; s < 16; s++) {
        if (gap[s]) continue;
        int interval = shape[(s / 2) % 4];
        // At high c1 and back half: inject modal tension note
        if (s >= 10 && c1 > 0.5f && rc((c1 - 0.4f) * 1.5f))
            interval = sc5v ? sc5v : 8;
        int fcc = int(70 + c1 * 50);
        m[s] = mn(root + interval, 0.65f, s%4==0 ? 115 : 95, fcc);
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

// PROG
// ctrl1 = note density: 0=sparse long held tones, 1=dense rhythmic hits
// ctrl2 = harmonic adventure: 0=root/5th only, 1=chromatic passing tones + tritone subs
void BassEngine::genProg(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc2v = sc(scIdx, 2);
    int sc3v = sc(scIdx, 3);
    int sc4v = sc(scIdx, 4);
    int sc6v = sc(scIdx, 6);

    // Density controls which pattern template to use
    static const int PAT_SPARSE[4]  = {0, 8, -1, -1};
    static const int PAT_MID[5]     = {0, 5, 8, 12, -1};
    static const int PAT_DENSE[7]   = {0, 2, 5, 7, 9, 11, 14};
    const int* pat;
    int patLen;
    if (c1 < 0.35f)      { pat = PAT_SPARSE; patLen = 4; }
    else if (c1 < 0.7f)  { pat = PAT_MID;    patLen = 5; }
    else                 { pat = PAT_DENSE;   patLen = 7; }

    int safeNotes[4]  = {0, sc4v?sc4v:7, 12, sc2v?sc2v:3};
    int spiceNotes[4] = {sc3v?sc3v:5, sc6v?sc6v:10, 6, sc2v?sc2v+12:15};

    for (int idx = 0; idx < patLen; idx++) {
        int s = pat[idx];
        if (s < 0) break;
        bool nextClose = (idx+1 < patLen && pat[idx+1] >= 0 && pat[idx+1] - s <= 2);
        float len = nextClose ? 0.5f + c1*0.8f : 1.0f - c1*0.6f;
        int n = root;
        if (rc(c2)) {
            n += (c2 > 0.6f && rc(c2 - 0.4f)) ? spiceNotes[ri(4)] : safeNotes[ri(4)];
        }
        int vel = (s == 0 || s == 8) ? 120 : int(95 + c1*20);
        m[s] = mn(n, len, vel, int(50 + c2*60));
    }
}

// AFROHOUSE
// ctrl1 = groove density: 0=two-note minimal (beat 1+3), 1=all five syncopated hits
// ctrl2 = sub-bass texture: 0=upper register bright, 1=sub-octave drops + closed filter
void BassEngine::genAfrohouse(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    static const int PATS[3][5] = {{3,6,8,11,14},{0,4,7,12,14},{2,5,8,10,13}};
    const int* pat = PATS[ri(3)];
    int sc2v = sc(scIdx, 2);
    int sc4v = sc(scIdx, 4);
    // Spice: pentatonic shapes + chromatic neighbours
    int spice[4] = {sc2v?sc2v:3, sc4v?sc4v:7, sc2v?sc2v+7:10, 5};

    for (int idx = 0; idx < 5; idx++) {
        int s = pat[idx];
        bool isAnchor = (s == 0 || s == 8);
        // Low c1 = sparse (only anchor beats), high c1 = all hits
        if (!isAnchor && !rc(c1 * 1.1f)) continue;

        int n = root;
        // Melodic movement on offbeats
        if (!isAnchor && rc(0.5f))
            n += spice[ri(4)];
        // Sub-bass drop at high c2
        if (rc(c2 * 0.8f))
            n -= 12;

        int fcc = int(90 - c2 * 55);  // filter closes with sub-bass
        m[s] = mn(n, isAnchor ? 0.7f : 0.45f, isAnchor ? 115 : 100, fcc);
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

// GFUNK
// ctrl1 = slide intensity: 0=clean held tones, 1=chromatic slides + pitch bends on every hit
// ctrl2 = melodic richness: 0=root-centric (just root+octave), 1=full upper-register melody
void BassEngine::genGfunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc2v = sc(scIdx, 2);
    int sc3v = sc(scIdx, 3);
    int sc4v = sc(scIdx, 4);
    int sc5v = sc(scIdx, 5);

    // Downbeats always present, length modulated by slide intensity
    float downLen = 1.8f - c1 * 0.8f;
    m[0] = mn(root, downLen, 122, 88);
    m[8] = mn(root, downLen - 0.2f, 118, 82);

    // Octave pop: always at high c2, sometimes at low c2
    if (rc(0.15f + c2 * 0.85f))
        m[3] = mn(root + 12, 0.45f, 105, int(65 + c2*40), c1 > 0.4f ? -2.f : 0.f);

    // Mid-bar melodic fill driven by c2
    if (rc(0.3f + c2 * 0.7f)) {
        int melody[4] = {sc2v?sc2v:3, sc3v?sc3v:5, sc4v?sc4v:7, sc5v?sc5v:9};
        m[6] = mn(root + melody[ri(4)], 0.7f, 112, int(70 + c2*45));
    }

    // Slide run into next phrase at high c1
    if (rc(0.25f + c1 * 0.75f)) {
        float slideDown = -(1.f + c1 * 2.f);
        float slideUp   =  (1.f + c1 * 3.f);
        m[13] = mn(root + (sc3v?sc3v:5), 0.5f, 102, 82, 0.f);
        m[14] = mn(root + (sc4v?sc4v:7), 1.2f, 122, 105, rc(0.6f) ? slideUp : slideDown);
    } else if (rc(0.5f)) {
        m[14] = mn(root - 12, 0.9f, 108, 78, c1 * 2.5f);
    }

    // At high c2: add a high inner melody note
    if (c2 > 0.6f && rc(c2 - 0.4f)) {
        int hiNote = root + (sc5v?sc5v+12:21);
        m[11] = mn(hiNote, 0.5f, 95, int(60 + c2*60), c1 > 0.5f ? 2.f : 0.f);
    }
}

// ── Anchor: prevent drift from tonal center ────────────────────────────────
// Ensures step 0 and step 8 stay on strong tones; clamps all notes to a
// singable range (root-2 semitones to root+14) so continuity blending
// can't walk notes into incoherent register extremes.
void BassEngine::anchorMotif(MS m[16], int root, int scIdx) {
    int sc4v = sc(scIdx, 4);  // 5th degree — strong mid-bar landing tone

    // Step 0: must be root (or very close)
    if (m[0].note < 0 || std::abs(m[0].note - root) > 2)
        m[0] = mn(root, 0.5f, 120, 90);

    // Step 8 (mid-phrase downbeat): root or 5th
    if (m[8].note < 0)
        m[8] = mn(root + (rc(0.4f) ? sc4v : 0), 0.5f, 112, 85);

    // Clamp all notes within a musically sane range around the root
    for (int i = 0; i < 16; i++) {
        if (m[i].note < 0) continue;
        while (m[i].note > root + 15) m[i].note -= 12;
        while (m[i].note < root - 3)  m[i].note += 12;
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
void BassEngine::regeneratePhrase(bool advanceArc) {
    if (advanceArc) m_phraseCount++;
    m_phrase.clear();

    const GenreCfg& cfg = GCFG[m_genre];

    // Hold scale for 2-4 phrases for harmonic continuity, then change
    if (m_scaleHoldCount <= 0) {
        m_scaleIdx      = cfg.scales[ri(2)];
        m_scaleHoldCount = 2 + ri(3); // hold 2-4 phrases
    } else {
        m_scaleHoldCount--;
    }
    m_progIdx = ri(3);
    const int* prog = cfg.progs[m_progIdx];

    // Generate motif A — blend with previous if available for continuity
    MS motA[16], motB[16], motC[16], motD[16];
    MS freshA[16];
    switch (m_genre) {
        case GENRE_FUNK:      genFunk     (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_ITALO:     genItalo    (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_SYNTHPOP:  genSynthpop (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_PSYTRANCE: genPsytrance(freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_PROG:      genProg     (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_AFROHOUSE: genAfrohouse(freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_UKG:       genUkg      (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
        case GENRE_GFUNK:     genGfunk    (freshA, ROOT, m_scaleIdx, m_ctrl1, m_ctrl2); break;
    }

    // Blend with previous motif for continuity: keep ~40% of slots from prev phrase
    if (m_hasPrevMotA) {
        for (int i = 0; i < 16; i++) {
            bool keepPrev = (m_prevMotA[i].note >= 0) && rc(0.4f);
            motA[i] = keepPrev ? m_prevMotA[i] : freshA[i];
        }
    } else {
        memcpy(motA, freshA, sizeof(motA));
    }
    memcpy(m_prevMotA, motA, sizeof(motA));
    m_hasPrevMotA = true;

    // Anchor to tonal center after blend (prevents drift over time)
    anchorMotif(motA, ROOT, m_scaleIdx);

    // Transforms — complexity arc: simpler early, richer over time
    // phraseCount mod 8 cycles through an arc: simplify → modal → drift → full
    int arc = m_phraseCount % 8;
    int xformB = (arc < 2) ? 0 : (arc < 5) ? 2 : ri(4); // simplify → modal → random
    int xformC = (arc < 3) ? 0 : (arc < 6) ? 1 : ri(4); // simplify → subDrop → random
    transformMotif(motA, motB, xformB, ROOT, m_scaleIdx);
    transformMotif(motA, motC, xformC, ROOT, m_scaleIdx);
    transformMotif(motA, motD, 3,      ROOT, m_scaleIdx);

    // Phrase structure driven by ctrl2: low = repetitive (AAAD), high = complex (ABAC)
    float varVal = m_ctrl2;
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
        m_phraseCount    = 0;
        m_lastPos        = -1.0;
        m_lastBar        = -1;
        m_hasPrevMotA    = false;
        m_scaleHoldCount = 0;
        m_regenPending   = false;
        regeneratePhrase();
    }
    ESP_LOGI(TAG, "Genre set to %d", m_genre);
}

void BassEngine::setCtrl1(float v) {
    m_ctrl1 = v;
    if (m_active) m_regenPending = true;
}

void BassEngine::setCtrl2(float v) {
    m_ctrl2 = v;
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

    // Map beat to position within 256-step phrase.
    // 256 steps = 64 beats (16 bars × 4 beats/bar).
    double phrasePos = std::fmod(beat * 4.0, 256.0);

    // Detect phrase wrap → advance arc and regenerate
    bool phraseWrapped = (m_lastPos >= 0.0 && phrasePos < m_lastPos - 128.0);
    if (phraseWrapped) {
        regeneratePhrase(true);
        m_regenPending = false;
        m_lastBar = -1;
    }

    // At any bar boundary: apply pending ctrl changes immediately (hot-swap, no arc advance)
    int currentBar = static_cast<int>(phrasePos / 16);
    if (m_regenPending && m_lastBar >= 0 && currentBar != m_lastBar) {
        regeneratePhrase(false);
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
