// bass_engine.cpp
// C++ port of the 8-genre bass algorithm from the HTML prototype.
// Phrase = 1024 steps (64 bars x 16 steps).  1 step = 1/16 note.
// Link beat -> phrase pos:  pos = fmod(beat * 4.0, 1024.0)

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

// ---- Scales [index][degree] ----
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

// ---- Genre configuration ----
struct GenreCfg {
    int   turnaroundRate;   // fire turnaround every N phrase matrices
    float swingSteps;       // step offset for odd steps (pre-computed at genre default BPM)
    int   scales[2];        // two scale options to pick from
    int   progs[3][4];      // three chord-progression options
};

static const GenreCfg GCFG[8] = {
    // FUNK  (105 BPM, swing 0.04s -> 0.28 steps)
    // i->IV->bIII->bVII | i->bVII->IV->V | i->bIII->bVII->IV  (dark modal funk, avoids naive I-IV-V)
    {2, 0.28f, {5,0}, {{0,5,3,10},{0,10,5,7},{0,3,10,5}}},
    // ITALO (120 BPM, swing 0)
    // i->bVI->IV->bVII | i->bVII->bVI->V | i->IV->bVI->bVII  (Moroder drama, descending minor)
    {4, 0.00f, {8,2}, {{0,8,5,10},{0,10,8,7},{0,5,8,10}}},
    // SYNTHPOP (115 BPM, swing 0.015s -> 0.115 steps)
    // i->bVI->bIII->V | i->IV->bVII->bVI | i->bIII->bVII->IV  (atmospheric, JM Jarre / DM feel)
    {2, 0.12f, {1,7}, {{0,8,3,7},{0,5,10,8},{0,3,10,5}}},
    // PSYTRANCE (142 BPM, swing 0)
    // mostly static with phrygian tension flicks (hypnotic repetition is the point)
    {4, 0.00f, {6,2}, {{0,0,1,0},{0,1,0,7},{0,0,10,1}}},
    // PROG (124 BPM, swing 0.01s -> 0.083 steps)
    // i->bVI->V->bVII | i->bIII->V->bVII | i->IV->bVI->bIII (dark modal, no bII cross-intervals)
    {4, 0.08f, {1,2}, {{0,8,7,10},{0,3,7,10},{0,5,8,3}}},
    // AFROHOUSE (122 BPM, swing 0.035s -> 0.285 steps)
    // hypnotic minimal root motion, occasional bVI/bVII colour
    {4, 0.29f, {1,2}, {{0,0,10,7},{0,8,0,10},{0,5,0,3}}},
    // UKG (132 BPM, swing 0.05s -> 0.44 steps)
    // i->bVI->IV->bVII | i->bIII->bVII->V | i->IV->bVI->bIII
    {2, 0.44f, {1,3}, {{0,8,5,10},{0,3,10,7},{0,5,8,3}}},
    // GFUNK (95 BPM, swing 0.045s -> 0.285 steps)
    // i->IV->i->bVII | i->V->IV->bIII | i->bIII->V->IV  (Dre / Warren G feel-good dark)
    // Scales: dorian + harmonicMinor -- richer than straight pentatonic
    {1, 0.29f, {0,7}, {{0,5,0,10},{0,7,5,3},{0,3,7,5}}},
};

// ---- Random helpers ----
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

// ---- MS helpers ----
BassEngine::MS BassEngine::mn(int note, float len, int vel, int fcc,
                               float pb, float ts) {
    return {note, len, vel, fcc, pb, ts};
}

void BassEngine::msInit(MS m[16]) {
    for (int i = 0; i < 16; i++) m[i] = {-1, 0.f, 0, 0, 0.f, 0.f};
}

// ---- Genre generators ----

// FUNK -- locked industrial groove, never melodic noise
// ctrl1 = hit density: minimal 2-hit groove -> full syncopated pocket
// ctrl2 = harmonic depth: root+5th only -> octave drops + b7 colour
void BassEngine::genFunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc4v = sc(scIdx, 4);  // 5th

    // Unmovable downbeat -- the anchor
    m[0] = mn(root, 0.35f, 127, 118);

    // Locked groove templates -- rhythmic identity is the hook, not note variety
    // c1 selects groove character deterministically: same dial = same groove skeleton
    static const int GROOVES[4][5] = {
        {3, 6, 10, 13, -1},
        {2, 5,  9, 12, 15},
        {3, 7, 10, 14, -1},
        {2, 6, 11, 13, -1},
    };
    int gIdx = (c1 < 0.3f) ? 0 : (c1 < 0.6f) ? 1 : (c1 < 0.85f) ? 2 : 3;
    const int* groove = GROOVES[gIdx];

    // 3 pitch choices: root | 5th | minor-3rd colour (never major 3rd -- guards mixolydian)
    int pit5 = sc4v ? sc4v : 7;
    // Only use scale's 3rd if it's minor (<=3 semitones). Mixolydian gives major 3rd(4)=pop sound.
    int pit3 = (sc(scIdx, 2) > 0 && sc(scIdx, 2) <= 3) ? sc(scIdx, 2) : 0;

    for (int k = 0; k < 5; k++) {
        if (groove[k] < 0) break;
        int s = groove[k];
        // c1 full range: 0=almost silent (1 hit/bar), 1=every slot fires
        if (!rc(c1)) continue;
        int n = root;
        float roll = rand01();
        // c2 full range: 0=root only, 1=active harmonic movement (5th, octave, b3)
        if (c2 > 0.7f && roll < 0.3f)        n = root + 12;
        else if (c2 > 0.35f && roll < 0.55f) n = root + pit5;
        else if (pit3 > 0 && c2 > 0.5f && rc(c2 - 0.3f)) n = root + pit3;
        int vel = (s % 8 == 0) ? 118 : int(80 + c1 * 35);
        int fcc = int(60 + c2 * 60);  // filter wide open at high harmonic depth
        m[s] = mn(n, 0.22f, vel, fcc);
        if (s > 0 && m[s-1].note < 0 && rc(c1 * 0.35f))
            m[s-1] = mn(root, 0.08f, 28, 15);
    }
    // Sub-bass drop on beat 3: c1>0.5 gives occasional, c1>0.8 gives consistent
    if (m[8].note < 0 && rc(c1 * c1))
        m[8] = mn(root - 12, 0.4f, 115, int(30 + c2 * 40));
}

// ITALO -- dark Giorgio Moroder driving pulse, space is the groove
// ctrl1 = drive: minimal accents -> full pulsing offbeats
// ctrl2 = articulation: fat legato -> tight staccato punches
void BassEngine::genItalo(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc4v = sc(scIdx, 4);  // 5th -- the only melodic movement needed
    int sc6v = sc(scIdx, 6);  // b7 for colour at high drive

    // Downbeats -- beat 1 always anchors; beats 2/3/4 can breathe at high c2
    // Rigid 4-on-the-floor at low c2, syncopated space at high c2 (darker groove)
    for (int beat = 0; beat < 4; beat++) {
        int s = beat * 4;
        // Inner beats skip at high c2 for rhythmic breath -- keeps it dark and funky
        if (beat > 0 && c2 > 0.5f && rc((c2 - 0.35f) * 0.6f)) continue;
        float len = 0.75f - c2 * 0.45f;
        int fcc = int(70 + c2 * 55);
        m[s] = mn(root, len, beat == 0 ? 127 : 112, fcc);
    }

    // Offbeat fills: scale-locked only, no chromatic noise
    // Low c1: silent offbeats (space = groove)
    // High c1: 5th and b7 fills pulse the energy
    // Moroder bass: the root pulse IS the groove; fills are sparse accents not melody
    int fills[2] = {sc4v ? sc4v : 7, sc6v ? sc6v : 10};
    static const int OFFS[4] = {2, 6, 10, 14};
    for (int k = 0; k < 4; k++) {
        if (!rc(c1 * 0.4f)) continue;
        int s = OFFS[k];
        int n = root + fills[rc(0.4f) ? 1 : 0];
        m[s] = mn(n, 0.3f - c2 * 0.15f, int(88 + c1 * 20), int(55 + c2 * 55));
    }
}

// SYNTHPOP -- dark driving pulse, Carpenter Brut / hard synth energy
// ctrl1 = pulse density: 4-on-floor skeleton -> full syncopated hits
// ctrl2 = harmonic tension: root/octave -> 5th/b7 fills with sub drops
void BassEngine::genSynthpop(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc4v = sc(scIdx, 4);  // 5th
    // Clamp to b7 max -- harmonicMinor gives maj7(11) which sounds classical not dark
    int sc6v = std::min(sc(scIdx, 6), 10);

    // Driving skeleton -- downbeats locked, offbeat gaps create tension
    static const int TEMPLATES[3][8] = {
        {0, 4, 8, 12, -1, -1, -1, -1},           // 4-on-floor skeleton
        {0, 3, 8, 11, 14, -1, -1, -1},            // syncopated push
        {0, 2, 6,  8, 11, 13, -1, -1},            // dense industrial
    };
    int tIdx = (c1 < 0.35f) ? 0 : (c1 < 0.7f) ? 1 : 2;
    const int* tmpl = TEMPLATES[tIdx];

    for (int k = 0; k < 8; k++) {
        if (tmpl[k] < 0) break;
        int s = tmpl[k];
        bool isDown = (s % 4 == 0);
        int n = root;
        if (!isDown && rc(c2 * 0.7f))
            n += rc(0.5f) ? (sc4v ? sc4v : 7) : (sc6v ? sc6v : 10);
        // Sub drop: octave below on offbeats at high c2
        if (!isDown && c2 > 0.6f && rc(c2 - 0.4f)) n = root - 12;
        int vel = isDown ? 122 : int(88 + c1 * 25);
        int fcc = isDown ? int(65 + c2 * 50) : int(80 + c2 * 40);
        m[s] = mn(n, isDown ? 0.7f - c2 * 0.4f : 0.25f, vel, fcc);
    }
}

// PSYTRANCE -- relentless 16th gallop, root-locked except rare phrygian b2 flick
// ctrl1 = density: 4-note accent pulse -> full 12-note 16th drive
// ctrl2 = intensity: uniform velocity -> dynamic accents + b2 injection
void BassEngine::genPsytrance(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc1v = sc(scIdx, 1);  // phrygian b2 -- the tension flick

    for (int beat = 0; beat < 4; beat++) {
        int base = beat * 4;
        // base+0 = downbeat = kick territory -- bass silent here always

        // base+1 (beat-e): fires only at high density
        if (rc(c1 * 0.7f))
            m[base+1] = mn(root, 0.22f, int(72 + c1*22), int(35 + c2*35));

        // base+2 (beat-+): fires at medium density -- the 8th-note offbeat
        if (rc(c1 * 0.9f))
            m[base+2] = mn(root, 0.22f, int(80 + c1*20), int(42 + c2*38));

        // base+3 (beat-a): the gallop accent -- always fires, can be b2 flick
        bool useB2 = (c2 > 0.45f && rc(c2 * 0.18f));
        m[base+3] = mn(root + (useB2 ? sc1v : 0), 0.22f,
                       int(100 + c2*22), int(58 + c2*45));
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

    // Deterministic note cycle -- avoids random-pick monotony (same note twice kills groove)
    // Anchors: steps 0/8 = root; inner hits cycle through 5th -> b7 -> b3 in order
    int cycle[4] = {0, sc4v?sc4v:7, sc6v?sc6v:10, sc2v?sc2v:3};

    for (int idx = 0; idx < patLen; idx++) {
        int s = pat[idx];
        if (s < 0) break;
        bool nextClose = (idx+1 < patLen && pat[idx+1] >= 0 && pat[idx+1] - s <= 2);
        float len = nextClose ? 0.5f + c1*0.8f : 1.0f - c1*0.6f;
        bool isAnchor = (s == 0 || s == 8);
        // Anchors = root; inner hits cycle through harmonic sequence deterministically
        int n = root + (isAnchor ? 0 : cycle[idx % 4]);
        // Tritone accent: only on last hit of dense pattern, very sparse -- prog colour not mud
        if (!isAnchor && idx == patLen-1 && c2 > 0.75f && rc(0.2f)) n = root + 6;
        int vel = isAnchor ? 120 : int(95 + c1*20);
        m[s] = mn(n, len, vel, int(50 + c2*60));
    }
}

// AFROHOUSE
// ctrl1 = groove density: 0=two-note minimal (beat 1+3), 1=all five syncopated hits
// ctrl2 = sub-bass texture: 0=upper register bright, 1=sub-octave drops + closed filter
void BassEngine::genAfrohouse(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    // c1 selects groove skeleton deterministically -- the pattern IS the hook
    static const int PATS[3][5] = {{3,6,8,11,14},{0,4,7,12,14},{2,5,8,10,13}};
    int pIdx = (c1 < 0.4f) ? 0 : (c1 < 0.75f) ? 1 : 2;
    const int* pat = PATS[pIdx];
    int sc4v = sc(scIdx, 4);
    int sc6v = sc(scIdx, 6);

    for (int idx = 0; idx < 5; idx++) {
        int s = pat[idx];
        bool isAnchor = (s == 0 || s == 8);
        if (!isAnchor && !rc(c1 * 1.1f)) continue;

        int n = root;
        // Position-driven spice: first half of bar -> 5th, second half -> b7
        // Creates a consistent hook interval: predictable color at each position
        if (!isAnchor) {
            int spiceNote = (s % 16 < 8) ? (sc4v?sc4v:7) : (sc6v?sc6v:10);
            if (rc(c2 * 0.65f)) n += spiceNote;
        }
        // Sub-bass: only when note is still root -- melodic+sub collision = mud
        if (n == root && rc(c2 * 0.8f))
            n -= 12;

        int fcc = int(90 - c2 * 55);  // filter closes with sub-bass
        m[s] = mn(n, isAnchor ? 0.7f : 0.45f, isAnchor ? 115 : 100, fcc);
    }
}

// UKG -- dark broken rhythms, punchy and relentless, no pitch wandering
// ctrl1 = jump intensity: root-locked -> 5th/octave leaps on offbeats
// ctrl2 = gap density: full pattern -> stripped syncopated skeleton
void BassEngine::genUkg(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc4v = sc(scIdx, 4);  // 5th
    int pit5 = sc4v ? sc4v : 7;

    // c2 selects groove skeleton deterministically -- gap density drives pattern choice
    static const int PATS[3][6] = {
        {0, 3,  8, 10, 13, -1},
        {2, 5,  8, 11, 14, -1},
        {0, 7, 10, 13, 15, -1},
    };
    int pIdx = (c2 < 0.35f) ? 0 : (c2 < 0.7f) ? 1 : 2;
    const int* pat = PATS[pIdx];

    for (int idx = 0; idx < 6; idx++) {
        if (pat[idx] < 0) break;
        int s = pat[idx];
        bool isDown = (s % 8 == 0);

        // High c2 strips non-anchor hits for more space
        if (!isDown && rc(c2 * 0.5f)) continue;

        int n = root;
        // Scale-locked interval jumps only -- no pitch bends
        if (!isDown && rc(c1 * 0.8f))
            n += rc(0.55f) ? pit5 : 12;  // 5th or octave, nothing else

        int vel = isDown ? 118 : int(85 + c1 * 25);
        int fcc = isDown ? int(70 + c2 * 45) : int(85 + c1 * 30);
        m[s] = mn(n, isDown ? 0.75f : 0.28f + c2 * 0.15f, vel, fcc);
    }
}

// GFUNK -- smooth dark groove, deliberate slides only, never random pitch noise
// ctrl1 = slide power: clean held tones -> structural slides at phrase tail
// ctrl2 = fill richness: root/5th skeleton -> 4th/b7 inner fills
void BassEngine::genGfunk(MS m[16], int root, int scIdx, float c1, float c2) {
    msInit(m);
    int sc3v = sc(scIdx, 3);  // 4th
    int sc4v = sc(scIdx, 4);  // 5th
    int sc6v = sc(scIdx, 6);  // b7

    // Fat locked downbeats -- the foundation
    float downLen = 1.6f - c1 * 0.6f;
    m[0] = mn(root,     downLen,        122, 90);
    m[8] = mn(root,     downLen - 0.2f, 118, 85);

    // Octave pop on beat 1.75 -- signature G-Funk hook, always clean
    if (rc(0.3f + c2 * 0.7f))
        m[3] = mn(root + 12, 0.4f, 108, int(68 + c2 * 38));

    // Inner fill: 4th or 5th, no chromatic noise
    // Mid-bar fill: high baseline so it reliably defines the G-Funk pocket feel
    if (rc(0.55f + c2 * 0.45f)) {
        int fill = rc(0.5f) ? (sc3v ? sc3v : 5) : (sc4v ? sc4v : 7);
        m[6] = mn(root + fill, 0.6f, 110, int(72 + c2 * 42));
    }

    // b7 colour hit at high richness -- dark and funky
    if (c2 > 0.55f && rc(c2 - 0.3f) && m[11].note < 0) {
        int dark = sc6v ? sc6v : 10;  // b7 -- always dark, never bright pop
        m[11] = mn(root + dark, 0.45f, 98, int(62 + c2 * 50));
    }

    // Tail slide: structural, deliberate -- only at steps 13/14
    if (rc(0.2f + c1 * 0.8f)) {
        // One clean slide into the next phrase root -- no random bends elsewhere
        float slide = c1 > 0.5f ? (rc(0.5f) ? 3.f : -3.f) : 0.f;
        m[14] = mn(root + (sc4v ? sc4v : 7), 1.0f, 118, 100, slide);
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

void BassEngine::turnFunk(int base, int scIdx, float c1, float c2) {
    int sc2v = sc(scIdx, 2);
    // Guard major 3rd: only use b3 if scale gives minor 3rd (<=3)
    int darkColour = (sc2v > 0 && sc2v <= 3) ? sc2v : 5;
    addNote(T+4,    base+5,              0.5f, 100, 80);
    addNote(T+5.5f, base+(c2>0.5f ? darkColour : 5), 0.5f, 110, 90);
    addNote(T+7,    base+7,              0.5f, 127, 127);
}
void BassEngine::turnItalo(int base) {
    addNote(T+4, base+12, 0.5f, 120, 120);
    addNote(T+6, base+7,  0.5f, 120,  90);
}
void BassEngine::turnSynthpop(int base) {
    addNote(T+4, base+12, 1.5f, 110, 110);  // clean octave hit, no pitch bend
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
    addNote(T+5.5f, base+12, 0.4f, 110, 90);
    addNote(T+7,    base,    1.0f, 120, 100);  // clean landing, no detuned bend
}
void BassEngine::turnGfunk(int base, int scIdx, float c1) {
    int sc2v = sc(scIdx, 2);
    addNote(T+6, base+(sc2v?sc2v:3), 2.0f, 110, 90, -3.f*c1);
}

// ---- Phrase regeneration ----
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

    // Generate motif A -- blend with previous if available for continuity
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

    // 64-bar arrangement: 4 sections x 16 bars each
    // Each section gets a distinct chord progression and motif character:
    //   sec0 (bars  0-15): core groove   -- ctrl2-driven structure, prog A
    //   sec1 (bars 16-31): bridge        -- B/A alternation,         prog B
    //   sec2 (bars 32-47): peak energy   -- C/B push,                prog C
    //   sec3 (bars 48-63): reprise       -- ctrl2-driven structure,  prog A
    // Within each section a 4-bar chord cell repeats 4x -- hypnotic, not identical
    int secProgs[4] = {m_progIdx, (m_progIdx+1)%3, (m_progIdx+2)%3, m_progIdx};

    // Build 64-bar phrase (256 beats = 1024 steps)
    for (int bar = 0; bar < 64; bar++) {
        int sec       = bar / 16;
        int barInSec  = bar % 16;
        int barInCell = barInSec % 4;

        const int* secProg = cfg.progs[secProgs[sec]];
        int rootOffset = psy_bind ? 0 : secProg[barInCell];

        char part;
        if (sec == 0 || sec == 3) {
            part = structure[barInCell];              // user ctrl2 shape
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

    // Turnaround - erase steps 248..255 then inject new ones
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

    ESP_LOGI(TAG, "Phrase[%d] genre=%d scale=%d prog=%d notes=%d bars=64",
             m_phraseCount, m_genre, m_scaleIdx, m_progIdx, (int)m_phrase.size());
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

void BassEngine::setGenre(int genre_idx) {
    if (genre_idx < 0 || genre_idx >= GENRE_COUNT) return;
    bool genreChanged = (m_genre != genre_idx);
    m_genre  = genre_idx;
    m_active = true;
    {  // always regenerate -- reselecting same genre gives fresh pattern
        for (auto& an : m_activeNotes) {
            uint8_t noteOff[3] = {0x80, (uint8_t)an.note, 0x00};
            send_midi_message(noteOff, 3);
        }
        m_activeNotes.clear();
        m_lastPos        = -1.0;
        m_lastBar        = -1;
        m_regenPending   = false;
        // Full reset on genre change; fresh ideas (no motif blending) on same-genre reselect
        if (genreChanged) {
            m_phraseCount    = 0;
            m_hasPrevMotA    = false;
            m_scaleHoldCount = 0;
        } else {
            m_hasPrevMotA    = false;  // drop old motif so we get genuinely new material
            m_scaleHoldCount = 0;     // force new scale pick for fresh harmonic colour
        }
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
