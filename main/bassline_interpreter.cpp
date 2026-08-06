#include "bassline_interpreter.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace bli {

// =================================================================
// 1. Scales -- unchanged from the legacy bass_engine.cpp table.
// =================================================================
const int kScaleLens[kNumScales] = {7, 7, 7, 5, 7, 7, 7, 7, 7};
const int kScales[kNumScales][7] = {
    {0, 2, 3, 5, 7, 9, 10},   // 0 dorian
    {0, 2, 3, 5, 7, 8, 10},   // 1 aeolian
    {0, 1, 3, 5, 7, 8, 10},   // 2 phrygian
    {0, 3, 5, 7, 10, 3, 5},   // 3 minorPentatonic (5 real notes, last 2 repeated)
    {0, 2, 3, 5, 7, 9, 11},   // 4 melodicMinor
    {0, 2, 4, 5, 7, 9, 10},   // 5 mixolydian
    {0, 1, 4, 5, 7, 8, 10},   // 6 phrygianDom
    {0, 2, 3, 5, 7, 8, 11},   // 7 harmonicMinor
    {0, 2, 3, 6, 7, 8, 11},   // 8 hungarianMinor
};
static const char* kScaleNames[kNumScales] = {
    "dorian", "aeolian", "phrygian", "minorPentatonic", "melodicMinor",
    "mixolydian", "phrygianDom", "harmonicMinor", "hungarianMinor",
};

const char* scaleName(int scaleIdx) { return kScaleNames[scaleIdx]; }

int scaleDegree(int scaleIdx, int degree) {
    int len = kScaleLens[scaleIdx];
    int d = degree % len;
    if (d < 0) d += len;
    return kScales[scaleIdx][d];
}

void chordToneIntervals(int scaleIdx, int out[4]) {
    // Root/3rd/5th/7th built by stacking scale-degree thirds (same
    // construction SUBSTRATE's buildChords() and the DawDreamer Python
    // port's chord_tones() use).
    out[0] = scaleDegree(scaleIdx, 0);
    out[1] = scaleDegree(scaleIdx, 2);
    out[2] = scaleDegree(scaleIdx, 4);
    out[3] = scaleDegree(scaleIdx, 6);
}

// =================================================================
// 2. Dials
// =================================================================
float Dials::get(int bank, int dialIdx) const {
    switch (bank) {
        case BANK_HARMONY: return dialIdx == 0 ? harmonyGravity : harmonyColor;
        case BANK_GROOVE:  return dialIdx == 0 ? grooveEnergy   : grooveSwing;
        case BANK_MOTION:  return dialIdx == 0 ? motionContour  : motionVariation;
        case BANK_VOICE:   return dialIdx == 0 ? voiceSweep     : voiceArtic;
        default:           return 0.5f;
    }
}

void Dials::set(int bank, int dialIdx, float v01) {
    v01 = std::max(0.f, std::min(1.f, v01));
    switch (bank) {
        case BANK_HARMONY: (dialIdx == 0 ? harmonyGravity : harmonyColor) = v01; break;
        case BANK_GROOVE:  (dialIdx == 0 ? grooveEnergy   : grooveSwing)  = v01; break;
        case BANK_MOTION:  (dialIdx == 0 ? motionContour  : motionVariation) = v01; break;
        case BANK_VOICE:   (dialIdx == 0 ? voiceSweep     : voiceArtic)   = v01; break;
        default: break;
    }
}

// =================================================================
// 3. Groove templates -- 16th-note onset probability, condensed from
//    the DawDreamer-validated Python set (dropped the busiest DnB/
//    footwork templates that need a drum bed to make sense; this is a
//    solo bass voice). Ordered sparse -> busy so a single "energy"
//    dial sweeps through a coherent progression.
// =================================================================
struct Groove { const char* name; float w[kStepsPerBar]; };
static const Groove kGrooves[] = {
    {"minimal",      {1, 0, 0, 0,  0, 0, .3f, 0,   .5f, 0, 0, 0,   0, 0, .3f, 0}},
    {"deep",         {1, 0, 0, 0,  0, 0, .36f, 0,  .6f, 0, 0, 0,   0, 0, .3f, 0}},
    {"house_bump",   {.92f, 0, .78f, .06f, .22f, 0, .78f, .06f, .58f, 0, .78f, .06f, .22f, 0, .78f, .16f}},
    {"four_floor",   {1, 0, .1f, .16f, .9f, 0, .1f, .16f, .95f, 0, .1f, .16f, .9f, 0, .16f, .32f}},
    {"acid_303",     {1, .36f, .7f, .4f, .6f, .46f, .76f, .36f, .86f, .4f, .7f, .46f, .6f, .4f, .8f, .56f}},
    {"garage_2step", {1, 0, .16f, .56f, .1f, .2f, .8f, .16f, .36f, 0, .7f, .2f, .16f, .56f, .36f, .5f}},
    {"dembow",       {1, 0, 0, .86f, 0, 0, .9f, 0,  .36f, 0, 0, .86f, 0, 0, .76f, .2f}},
    {"funk_synco",   {1, 0, .36f, .7f, .2f, .46f, .16f, .6f, .56f, .2f, .5f, .36f, .3f, .6f, .4f, .56f}},
    {"techno_roll",  {1, .6f, .66f, .6f, .8f, .6f, .66f, .6f, .9f, .6f, .66f, .6f, .8f, .6f, .7f, .66f}},
};
static constexpr int kNumGrooves = sizeof(kGrooves) / sizeof(kGrooves[0]);

// 4/4 metric weight of each 16th -- strong on downbeats, weaker off-grid.
static const float kMetric[kStepsPerBar] = {
    1, .12f, .34f, .16f,  .78f, .12f, .36f, .16f,
    .9f, .12f, .34f, .16f, .72f, .14f, .4f, .28f,
};

// =================================================================
// 4. Contour shapes -- x in [0,1] -> [0,1]. Cheap to evaluate: only
//    called once per onset when a motif is (re)generated, never per
//    audio sample.
// =================================================================
enum ContourShape { CT_LEVEL, CT_CLIMB, CT_DESCEND, CT_ARCH, CT_VALLEY, CT_WAVE, CT_ZIGZAG, CT_TERRACED, CT_COUNT };

static float evalContour(int shape, float x) {
    switch (shape) {
        case CT_LEVEL:    return 0.5f;
        case CT_CLIMB:    return x;
        case CT_DESCEND:  return 1.f - x;
        case CT_ARCH:     return std::sin(float(M_PI) * x);
        case CT_VALLEY:   return 1.f - std::sin(float(M_PI) * x);
        case CT_WAVE:     return (1.f - std::cos(2.f * float(M_PI) * x)) / 2.f;
        case CT_ZIGZAG:   return 1.f - std::fabs(std::fmod(x * 2.f, 2.f) - 1.f);
        case CT_TERRACED: return std::floor(x * 4.f) / 3.f;
        default:          return 0.5f;
    }
}

// =================================================================
// 5. Rhythm: onset generation from a groove template blended toward
//    density/syncopation, mirroring SUBSTRATE's buildRhythm().
// =================================================================
static int buildOnsets(const Dials& dials, RngSource& rng, int outSteps[kStepsPerBar]) {
    float energy = dials.grooveEnergy;
    float sync = dials.grooveSwing;  // swing dial also pushes syncopation weight

    int gIdx = Dials::bandIndex(energy, kNumGrooves);
    const float* tmpl = kGrooves[gIdx].w;
    float density = 0.25f + energy * 0.65f;

    int n = 0;
    for (int i = 0; i < kStepsPerBar; i++) {
        float w = tmpl[i];
        w = w * (1.f - sync * kMetric[i] * 0.7f) + sync * (1.f - kMetric[i]) * 0.6f;
        float p = w * (0.4f + 1.55f * density) - (1.f - density) * 0.16f;
        p = std::max(0.f, std::min(1.f, p));
        if (rng.next01() < p) outSteps[n++] = i;
    }
    if (n == 0 || outSteps[0] != 0) {
        if (rng.next01() < 0.7f) {
            // insert a downbeat if missing (shift array right)
            for (int k = std::min(n, kStepsPerBar - 1); k > 0; k--) outSteps[k] = outSteps[k - 1];
            outSteps[0] = 0;
            n = std::min(n + 1, kStepsPerBar);
        }
    }
    if (n == 0) { outSteps[0] = 0; n = 1; }
    return n;
}

// =================================================================
// 6. Pitch: constrained DP (Viterbi) over the in-scale candidate pool
//    within [root, root+span] -- identical structure to SUBSTRATE's
//    buildPitches() / Liu's eq.28, restricted to scale tones only
//    (validated in the DawDreamer experiments: no audible loss for a
//    bass voice, and keeps the embedded candidate pool small).
// =================================================================
static constexpr int kRegisterSpan = 15;   // semitones, matches the old anchorMotif clamp
static constexpr int kMaxCandidates = 16;

static float harmonicCost(int m, int root, const int chordTones[4], float tension) {
    int pc = ((m - root) % 12 + 12) % 12;
    if (pc == chordTones[0] % 12) return 0.f;
    if (pc == chordTones[2] % 12) return 0.09f;   // 5th
    if (pc == chordTones[1] % 12) return 0.14f;   // 3rd
    if (pc == chordTones[3] % 12) return 0.21f;   // 7th
    return 0.52f * (1.f - 0.75f * tension);       // other scale tone
}

static float transCost(int a, int b, int leap, float octAppetite, float repeat) {
    int d = std::abs(a - b);
    if (d == 0) return (1.f - repeat) * 0.75f;
    if (d == 12) return 0.42f * (1.f - octAppetite) + 0.05f;
    float c = 0.12f + (d / 12.f) * 0.6f;
    if (d > leap) c += (d - leap) * 0.32f;
    if (d == 1 || d == 2) c *= 0.72f;
    return c;
}

// xorshift-free "gumbel-ish" noise from the injected RNG, matching the
// SUBSTRATE / Python port's exploration temperature term.
static float gumbelNoise(RngSource& rng) {
    float u = std::max(1e-6f, std::min(1.f - 1e-6f, rng.next01()));
    return -std::log(-std::log(u));
}

static void buildPitches(const int onsets[], int n, int root, int scaleIdx,
                          const Dials& dials, RngSource& rng, int outPitches[]) {
    int chordTones[4];
    chordToneIntervals(scaleIdx, chordTones);
    float gravity = dials.harmonyGravity;
    float tension = 1.f - dials.harmonyColor;
    float depth = 0.4f + dials.motionVariation * 0.3f;
    float variationTemp = dials.motionVariation * 0.85f;
    int leap = 3 + static_cast<int>(dials.motionVariation * 9.f);
    float octAppetite = dials.motionVariation * 0.5f;
    float repeat = 1.f - dials.motionVariation * 0.6f;

    int contourShape = Dials::bandIndex(dials.motionContour, CT_COUNT);

    // Scale-restricted candidate pool within [root, root+span].
    int pool[kMaxCandidates];
    int poolLen = 0;
    for (int m = root; m <= root + kRegisterSpan && poolLen < kMaxCandidates; m++) {
        int rel = ((m - root) % 12 + 12) % 12;
        bool inScale = false;
        for (int d = 0; d < 7; d++) {
            if (scaleDegree(scaleIdx, d) == rel) { inScale = true; break; }
        }
        if (inScale) pool[poolLen++] = m;
    }
    if (poolLen == 0) { pool[0] = root; poolLen = 1; }

    float targets[kStepsPerBar];
    for (int t = 0; t < n; t++) {
        float x = (n > 1) ? (float(t) / float(n - 1)) : 0.f;
        targets[t] = root + evalContour(contourShape, x) * kRegisterSpan;
    }

    float emit[kStepsPerBar][kMaxCandidates];
    for (int t = 0; t < n; t++) {
        float m = kMetric[onsets[t] % kStepsPerBar];
        for (int k = 0; k < poolLen; k++) {
            float e = harmonicCost(pool[k], root, chordTones, tension) * (0.3f + 0.7f * m * gravity * 1.35f);
            float dev = std::fabs(pool[k] - targets[t]) / float(kRegisterSpan);
            e += depth * dev * dev * 2.2f;
            e += variationTemp * gumbelNoise(rng) * 0.5f;
            emit[t][k] = e;
        }
    }

    float dp[kStepsPerBar][kMaxCandidates];
    int bp[kStepsPerBar][kMaxCandidates];
    for (int k = 0; k < poolLen; k++) dp[0][k] = emit[0][k];
    for (int t = 1; t < n; t++) {
        for (int k = 0; k < poolLen; k++) {
            float best = 1e9f;
            int bi = 0;
            for (int j = 0; j < poolLen; j++) {
                float v = dp[t - 1][j] + 1.5f * transCost(pool[j], pool[k], leap, octAppetite, repeat);
                if (v < best) { best = v; bi = j; }
            }
            dp[t][k] = best + emit[t][k];
            bp[t][k] = bi;
        }
    }

    int bi = 0;
    float bv = 1e9f;
    for (int k = 0; k < poolLen; k++) {
        if (dp[n - 1][k] < bv) { bv = dp[n - 1][k]; bi = k; }
    }
    for (int t = n - 1; t >= 0; t--) {
        outPitches[t] = pool[bi];
        bi = bp[t][bi];
    }
}

// =================================================================
// 7. Articulation: gate length, accent/ghost/slide, filter-CC sweep
//    (driven by the same contour that shapes pitch -- validated in
//    the DawDreamer experiments to read as a natural "brightness
//    follows pitch" correlation without a dedicated curve-shape dial).
// =================================================================
static void buildArticulation(const int onsets[], const int pitches[], int n,
                               const Dials& dials, RngSource& rng, Step m[kStepsPerBar]) {
    float artic = dials.voiceArtic;
    float sweepDepth = dials.voiceSweep;
    int contourShape = Dials::bandIndex(dials.motionContour, CT_COUNT);

    float accentP = 0.15f + artic * 0.45f;
    float ghostP = 0.05f + artic * 0.35f;
    float slideP = artic * 0.4f;
    float gate = 0.45f + (1.f - artic) * 0.35f;

    for (int i = 0; i < n; i++) {
        int s = onsets[i];
        float mw = kMetric[s % kStepsPerBar];
        int nextS = (i + 1 < n) ? onsets[i + 1] : kStepsPerBar;
        int gap = nextS - s;
        float dur = std::max(0.15f, std::min(gap * gate * (1.f + (rng.next01() - 0.5f) * 0.5f), gap * 1.2f));

        bool accent = rng.next01() < accentP * (0.3f + 1.1f * mw);
        bool ghost = (!accent) && (rng.next01() < ghostP * (1.15f - mw));
        bool slide = (gap <= 3) && (i + 1 < n) && (rng.next01() < slideP);

        int vel = static_cast<int>((0.55f + mw * 0.3f) * 100.f + (accent ? 25 : 0) - (ghost ? 45 : 0));
        vel = std::max(1, std::min(127, vel));

        float x = s / float(kStepsPerBar);
        float curveVal = evalContour(contourShape, x);
        int fcc = static_cast<int>(40.f + curveVal * 60.f * (0.4f + sweepDepth) + (accent ? 25 : 0));
        fcc = std::max(0, std::min(127, fcc));

        m[s].note = pitches[i];
        m[s].len = dur;
        m[s].vel = vel;
        m[s].fcc = fcc;
        m[s].pb = 0.f;
        if (slide) {
            // subtle slide flavor: a small pitch-bend ramp toward the next note
            int nextPitch = (i + 1 < n) ? pitches[i + 1] : pitches[i];
            m[s].pb = std::max(-3.f, std::min(3.f, (nextPitch - pitches[i]) * 0.4f));
        }
    }
}

// =================================================================
// 8. Top-level entry points
// =================================================================
void generateMotif(Step m[kStepsPerBar], int root, int scaleIdx,
                    const Dials& dials, RngSource& rng) {
    for (int i = 0; i < kStepsPerBar; i++) m[i] = Step{};

    int onsets[kStepsPerBar];
    int n = buildOnsets(dials, rng, onsets);

    int pitches[kStepsPerBar];
    buildPitches(onsets, n, root, scaleIdx, dials, rng, pitches);

    buildArticulation(onsets, pitches, n, dials, rng, m);
}

int generateTurnaround(TurnNote out[3], int root, int scaleIdx,
                        const Dials& dials, RngSource& rng) {
    // Output notes are absolute MIDI (root + interval), matching the old
    // turnXXX functions' convention: they wrote directly into m_phrase via
    // addNote(), bypassing the "motif + per-bar rootOffset" pipeline that
    // generateMotif's callers use, so they must already be absolute.
    int chordTones[4];
    chordToneIntervals(scaleIdx, chordTones);
    int fifth = chordTones[2];
    int third = chordTones[1];
    int seventh = chordTones[3];

    int n = 0;
    bool useFifth = rng.next01() < (0.4f + dials.harmonyGravity * 0.4f);
    out[n++] = {4.f, root + (useFifth ? fifth : third), 0.5f, 100, 90, 0.f};

    bool useColor = rng.next01() < dials.voiceArtic;
    out[n++] = {5.5f, root + (useColor ? seventh : fifth), 0.5f, 105, 95, 0.f};

    float slidePb = 0.f;
    if (rng.next01() < dials.voiceArtic * 0.5f) {
        slidePb = (rng.next01() < 0.5f) ? 3.f : -3.f;
    }
    out[n++] = {7.f, root, 1.0f, 120, 110, slidePb};

    return n;
}

}  // namespace bli
