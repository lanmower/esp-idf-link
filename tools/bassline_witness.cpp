// Off-device witness for the bassline generator.
//
// Runs the REAL BassEngine (main/bass_engine.cpp, copied verbatim) against a synthetic constant-
// tempo Link session and (a) prints the phrase as a bar x 16th grid with density/register/repetition
// metrics, and (b) can write the result as a standard MIDI file so a change can be AUDITIONED in a
// DAW rather than argued about. See tools/bassline-witness.sh for the build.
//
// Build requirements: nothing but a C++17 compiler. bassline_interpreter stays host-compilable
// (own header + <algorithm> <cmath> <cstring>) and bass_engine is compiled here against the stubs
// in tools/hoststub/, so no ESP-IDF tree is needed.
#include "bass_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr int    kBars     = 64;   // BassEngine::kBarsPerPhrase
constexpr int    kStepsBar = 16;   // bli::kStepsPerBar
constexpr double kBpm      = 145.0;
constexpr int    kRootNote = 40;   // BassEngine::kRootNoteE2
constexpr int    kTpq      = 480;  // MIDI ticks per quarter note

struct Ev {
    long    tick;
    uint8_t kind;   // 0 note-on, 1 note-off, 2 cc, 3 pitch bend
    uint8_t a, b;
};

std::vector<Ev> g_events;
double          g_nowBeat = 0.0;

// MIDI file (format 0) writing.
void putVarLen(std::vector<uint8_t>& out, unsigned long v) {
    uint8_t buf[8];
    int i = 0;
    buf[i++] = (uint8_t)(v & 0x7F);
    v >>= 7;
    while (v) { buf[i++] = (uint8_t)((v & 0x7F) | 0x80); v >>= 7; }
    for (int k = i - 1; k >= 0; k--) out.push_back(buf[k]);
}
void putBe32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back((uint8_t)(v >> 24)); out.push_back((uint8_t)(v >> 16));
    out.push_back((uint8_t)(v >> 8));  out.push_back((uint8_t)v);
}
void putBe16(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back((uint8_t)(v >> 8)); out.push_back((uint8_t)v);
}

bool writeMidi(const char* path, const std::vector<Ev>& evs) {
    std::vector<Ev> e = evs;
    // Note-offs sort before note-ons at the same tick so a repeated pitch retriggers.
    std::sort(e.begin(), e.end(), [](const Ev& x, const Ev& y) {
        if (x.tick != y.tick) return x.tick < y.tick;
        return x.kind > y.kind;
    });

    std::vector<uint8_t> trk;
    const uint32_t usPerQuarter = (uint32_t)(60000000.0 / kBpm);
    putVarLen(trk, 0);
    trk.push_back(0xFF); trk.push_back(0x51); trk.push_back(0x03);
    trk.push_back((uint8_t)(usPerQuarter >> 16));
    trk.push_back((uint8_t)(usPerQuarter >> 8));
    trk.push_back((uint8_t)usPerQuarter);

    long prev = 0;
    for (const Ev& ev : e) {
        putVarLen(trk, (unsigned long)(ev.tick - prev));
        prev = ev.tick;
        if (ev.kind == 0)       { trk.push_back(0x90); trk.push_back(ev.a); trk.push_back(ev.b); }
        else if (ev.kind == 1)  { trk.push_back(0x80); trk.push_back(ev.a); trk.push_back(0x00); }
        else if (ev.kind == 2)  { trk.push_back(0xB0); trk.push_back(ev.a); trk.push_back(ev.b); }
        else                    { trk.push_back(0xE0); trk.push_back(ev.a); trk.push_back(ev.b); }
    }
    putVarLen(trk, (unsigned long)0);
    trk.push_back(0xFF); trk.push_back(0x2F); trk.push_back(0x00);

    std::vector<uint8_t> f;
    f.insert(f.end(), {'M','T','h','d'});
    putBe32(f, 6); putBe16(f, 0); putBe16(f, 1); putBe16(f, kTpq);
    f.insert(f.end(), {'M','T','r','k'});
    putBe32(f, (uint32_t)trk.size());
    f.insert(f.end(), trk.begin(), trk.end());

    FILE* fp = fopen(path, "wb");
    if (!fp) { printf("cannot write %s\n", path); return false; }
    fwrite(f.data(), 1, f.size(), fp);
    fclose(fp);
    return true;
}

} // namespace

// The two emitters the bass engine calls.
void send_midi_message(const uint8_t* m, size_t n) {
    if (n < 2) return;
    const long tick = (long)(g_nowBeat * kTpq);
    const uint8_t hi = (uint8_t)(m[0] & 0xF0);
    if (n == 3 && hi == 0x90 && m[2] > 0) g_events.push_back({tick, 0, m[1], m[2]});
    else if (n == 3 && (hi == 0x80 || (hi == 0x90 && m[2] == 0))) g_events.push_back({tick, 1, m[1], 0});
    else if (n == 3 && hi == 0xE0) g_events.push_back({tick, 3, m[1], m[2]});
}
void send_midi_cc(uint8_t, uint8_t cc, uint8_t v) {
    g_events.push_back({(long)(g_nowBeat * kTpq), 2, cc, v});
}

namespace {

void simulateApproach(int approach) {
    g_events.clear();
    BassEngine engine;
    engine.setApproach(approach);

    ableton::Link::SessionState st;
    st.m_bpm = kBpm;
    // Simulate at 1/64 notes, not 1/16: a note scheduled at step 2.053 only fires once the phrase
    // position passes it, so at 1/16 resolution it is emitted during the step-3 call and lands in
    // the wrong grid cell (and collides with the note genuinely at step 3). That under-reported
    // density as ~13/16 when the motif really carries all 16.
    constexpr int kSub = 4;
    const long long stepUs = (long long)(0.25 / kSub * 60.0 / kBpm * 1e6);
    for (int s = 0; s < kBars * kStepsBar * kSub; s++) {
        const auto time = std::chrono::microseconds(s * stepUs);
        g_nowBeat = st.beatAtTime(time, 16.0);
        engine.process(st, time);
    }
}

void runApproach(int approach, const char* name, bool showGrid, const char* midPath) {
    simulateApproach(approach);

    int grid[kBars][kStepsBar];
    for (int b = 0; b < kBars; b++) for (int i = 0; i < kStepsBar; i++) grid[b][i] = -1;
    for (const Ev& e : g_events) {
        if (e.kind != 0) continue;
        const double pos = (double)e.tick / (double)kTpq * 4.0;
        const int step = (int)(pos + 0.5) % (kBars * kStepsBar);
        grid[step / kStepsBar][step % kStepsBar] = e.a;
    }

    std::string rows[kBars];
    int lo = 999, hi = -999, filled = 0;
    for (int b = 0; b < kBars; b++) {
        char r[kStepsBar + 1];
        for (int i = 0; i < kStepsBar; i++) {
            if (grid[b][i] < 0) { r[i] = '.'; continue; }
            filled++;
            const int iv = grid[b][i] - kRootNote;
            lo = std::min(lo, iv); hi = std::max(hi, iv);
            r[i] = (iv >= 0 && iv < 10) ? char('0' + iv) : char('a' + (iv - 10));
        }
        r[kStepsBar] = 0;
        rows[b] = std::string(r);
    }

    if (showGrid) {
        printf("\n======== %s (approach %d) @ %.0f bpm ========\n", name, approach, kBpm);
        printf("       ");
        for (int i = 0; i < kStepsBar; i++) printf("%2d", i);
        printf("\n");
        for (int b = 0; b < kBars; b++) printf("  bar%2d %s\n", b, rows[b].c_str());
    }

    std::set<std::string> uniq(rows, rows + kBars);
    int run = 1, best = 1;
    for (int b = 1; b < kBars; b++) {
        if (rows[b] == rows[b - 1]) { run++; best = std::max(best, run); } else run = 1;
    }
    // Dead air is the failure this catches: a held rest silenced a whole repeatBars group, so STAB
    // opened with 8 silent bars. Nothing else in the metrics sees it.
    int empty = 0, silRun = 0, worstSil = 0;
    for (int b = 0; b < kBars; b++) {
        if (rows[b].find_first_not_of('.') == std::string::npos) {
            empty++; silRun++;
            if (silRun > worstSil) worstSil = silRun;
        } else silRun = 0;
    }
    // The grid keeps one note per 16th (last write wins), so it hides every note that shares a
    // cell. The true count and the kick-share come from the raw events, never from the grid.
    size_t notes = 0, onKick = 0;
    for (const Ev& e : g_events) {
        if (e.kind != 0) continue;
        notes++;
        const double pos = (double)e.tick / (double)kTpq * 4.0;
        const long long s16 = (long long)std::floor(pos + 0.5) % kStepsBar;
        if (s16 % 4 == 0) onKick++;
    }
    const size_t collided = notes - (size_t)filled;
    printf("  %-7s notes=%4zu  perBar=%4.1f  kickShare=%4.1f%%  collided=%4zu  register=%+d..+%d (%2d st)"
           "  distinctBars=%2d/64  longestHold=%d  emptyBars=%2d longestSilence=%d\n",
           name, notes, (double)notes / kBars, notes ? 100.0 * (double)onKick / (double)notes : 0.0,
           collided, lo, hi, hi - lo, (int)uniq.size(), best, empty, worstSil);

    if (midPath && writeMidi(midPath, g_events))
        printf("  wrote %s (%zu events, %.0f bpm, %d bars)\n", midPath, g_events.size(), kBpm, kBars);
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Property and mutation modes.
//
// `--property` checks invariants of the GENERATED output over a matrix of roots, scales, dial
// positions and seeds. A candidate is only admitted when running the generator is the only way to
// settle it: an invariant a static_assert already proves, or one readable straight off a constant
// table, is body-derivable and is listed under `rejected` instead of being checked.
//
// `--mutate <id> <dir>` rewrites one source copy with a named mutation, so a CI job can score each
// mutant by rebuilding and re-running `--property`.
// ---------------------------------------------------------------------------------------------
namespace {

struct Prop {
    const char* name;
    long        checks = 0;
    long        fails  = 0;
    std::string ce;
};

enum PropId {
    P_MOTIF_REGISTER, P_MOTIF_SCALE, P_MOTIF_ROOT, P_MOTIF_INTERVAL, P_MOTIF_VEL,
    P_MOTIF_VEL_FLOOR, P_MOTIF_PB, P_MOTIF_LEN, P_MOTIF_STEP0, P_MOTIF_DENSITY_PEAK,
    P_MOTIF_CHORDTONE, P_MOTIF_REPEAT, P_MOTIF_OCTAVE, P_MOTIF_WIDE, P_MOTIF_TOP_ROLL,
    P_MOTIF_CONTOUR, P_MOTIF_METER_ACCENT, P_MOTIF_SPARSE_LOW,
    P_TURN_NONNEG, P_TURN_INCREASING, P_TURN_GAP, P_TURN_REGISTER, P_TURN_VEL, P_TURN_LEN,
    P_TURN_COUNT,
    P_ENG_WINDOW, P_ENG_VEL, P_ENG_SOUNDS, P_ENG_SILENCE, P_ENG_COLLISION,
    P_COUNT
};

Prop g_prop[P_COUNT] = {
    {"motif-register-bounded"},  {"motif-scale-membership"}, {"motif-root-favoured"},
    {"motif-interval-octave-bound"}, {"motif-velocity-bounded"}, {"motif-velocity-usable-floor"},
    {"motif-pitchbend-bounded"}, {"motif-length-bounded"}, {"motif-step0-majority"},
    {"motif-density-peaks-at-top-energy"},
    {"motif-chordtone-share-floor"}, {"motif-repeat-share-ceiling"},
    {"motif-octave-share-ceiling"}, {"motif-wide-leap-share-ceiling"},
    {"motif-top-energy-rolls"}, {"motif-contour-tracks-dial"}, {"motif-meter-accent"},
    {"motif-sparse-at-low-energy"},
    {"turnaround-offset-nonneg"}, {"turnaround-offset-increasing"}, {"turnaround-gap-floor"},
    {"turnaround-pitch-register"}, {"turnaround-velocity-bounded"},
    {"turnaround-length-bounded"}, {"turnaround-note-count-bounded"},
    {"engine-notes-in-bass-window"}, {"engine-velocity-bounded"}, {"engine-every-voice-sounds"},
    {"engine-no-long-silence"}, {"engine-no-grid-collision"},
};

void ck(PropId id, bool ok, const std::string& where) {
    Prop& p = g_prop[id];
    p.checks++;
    if (!ok) {
        p.fails++;
        if (p.ce.empty()) p.ce = where;
    }
}

struct SeededRng : bli::RngSource {
    std::mt19937 g;
    explicit SeededRng(uint32_t s) : g(s) {}
    float next01() override { return float(g() >> 8) / 16777216.0f; }
};

bool isScaleTone(int scaleIdx, int semitonesAboveRoot) {
    int rel = ((semitonesAboveRoot % 12) + 12) % 12;
    for (int d = 0; d < bli::kScaleLens[scaleIdx]; d++)
        if (bli::scaleDegree(scaleIdx, d) == rel) return true;
    return false;
}

std::string cfg(int root, int scaleIdx, float energy, float variation, int seed, int ci) {
    char b[192];
    snprintf(b, sizeof(b), "root=%d scale=%s energy=%.2f variation=%.2f contour=%d seed=%d",
             root, bli::scaleName(scaleIdx), energy, variation, ci, seed);
    return std::string(b);
}

constexpr int   kRoots[]      = {28, 40};
constexpr float kEnergies[]   = {0.00f, 0.25f, 0.50f, 0.75f, 1.00f};
constexpr float kVariations[] = {0.0f, 0.5f, 1.0f};
// One dial position per ContourShape band, so every shape in evalContour is exercised.
constexpr float kContours[]   = {0.06f, 0.19f, 0.31f, 0.44f, 0.56f, 0.69f, 0.81f, 0.94f};
constexpr int   kNumContours  = sizeof(kContours) / sizeof(kContours[0]);
constexpr int   kNumEnergies  = sizeof(kEnergies) / sizeof(kEnergies[0]);
constexpr int   kSeeds        = 12;

// Thresholds are measured off the unmutated generator, then held: a mutant that moves a share past
// one of them is killed by the property that owns it.
constexpr double kChordToneShareMin = 0.90;
constexpr double kRepeatShareMax    = 0.84;
constexpr double kOctaveShareMax    = 0.18;
constexpr double kWideLeapShareMax  = 0.18;
constexpr double kTopEnergyRollsMin = 15.0;
constexpr double kContourCorrMin    = 0.60;
constexpr double kStep0MajorityMin  = 0.85;
constexpr double kMeterAccentMinVel = 20.0;
constexpr double kSparseLowMax      = 2.0;
constexpr float  kTurnGapFloorSteps = 0.25f;

// The strong 16ths of the bar, pinned here rather than read from the generator's kMetric: the
// accent contrast between strong and weak steps is what the metrical accent property measures.
constexpr bool isStrongStep(int step) {
    return step % 16 == 0 || step % 16 == 4 || step % 16 == 8 || step % 16 == 12;
}

float contourShapeValue(int shape, float x) {    switch (shape) {
        case 0:  return 0.5f;
        case 1:  return x;
        case 2:  return 1.f - x;
        case 3:  return std::sin(float(M_PI) * x);
        case 4:  return 1.f - std::sin(float(M_PI) * x);
        case 5:  return (1.f - std::cos(2.f * float(M_PI) * x)) / 2.f;
        case 6:  return 1.f - std::fabs(std::fmod(x * 2.f, 2.f) - 1.f);
        default: return std::floor(x * 4.f) / 3.f;
    }
}

void runInterpreterProperties() {
    double onsetSum[kNumEnergies] = {0};
    long   onsetN[kNumEnergies]   = {0};
    long   step0Hit = 0, step0Tot = 0;
    long   pcHistogram[12] = {0};
    long   pcTotal = 0;
    long   chordToneHits = 0, pitchSamples = 0;
    long   repeatHits = 0, octaveHits = 0, wideHits = 0, intervalTotal = 0;
    double strongVelSum = 0, weakVelSum = 0;
    long   strongVelN = 0, weakVelN = 0;
    double corrSxy[kNumContours] = {0}, corrSxx[kNumContours] = {0}, corrSyy[kNumContours] = {0};
    double corrSx[kNumContours] = {0}, corrSy[kNumContours] = {0};
    long   corrN[kNumContours] = {0};

    for (int root : kRoots) {
        for (int scaleIdx = 0; scaleIdx < bli::kNumScales; scaleIdx++) {
            int chordTones[4];
            bli::chordToneIntervals(scaleIdx, chordTones);
            for (int ei = 0; ei < kNumEnergies; ei++) {
                for (float variation : kVariations) {
                    for (int ci = 0; ci < kNumContours; ci++) {
                    for (int seed = 0; seed < kSeeds; seed++) {
                        const std::string at = cfg(root, scaleIdx, kEnergies[ei], variation, seed, ci);
                        bli::Dials dials;
                        dials.grooveEnergy    = kEnergies[ei];
                        dials.motionVariation = variation;
                        dials.motionContour   = kContours[ci];
                        SeededRng rng(uint32_t(root * 1000003 + scaleIdx * 7919 + ei * 131 + seed));

                        bli::Step m[bli::kStepsPerBar];
                        bli::generateMotif(m, root, scaleIdx, dials, rng);

                        int onsets[bli::kStepsPerBar], n = 0;
                        for (int i = 0; i < bli::kStepsPerBar; i++)
                            if (m[i].note != -1) onsets[n++] = i;
                        onsetSum[ei] += n;
                        onsetN[ei]++;
                        step0Tot++;
                        if (n > 0 && onsets[0] == 0) step0Hit++;

                        bool rootSeen = false;
                        for (int i = 0; i < n; i++) {
                            const int s = onsets[i];
                            const int rel = m[s].note - root;
                            char w[256];
                            snprintf(w, sizeof(w), "%s step=%d note=%d", at.c_str(), s, m[s].note);
                            ck(P_MOTIF_REGISTER, rel >= 0 && rel <= bli::kRegisterSpan, w);
                            ck(P_MOTIF_SCALE, isScaleTone(scaleIdx, rel), w);
                            ck(P_MOTIF_VEL, m[s].vel >= 1 && m[s].vel <= 127, w);
                            ck(P_MOTIF_VEL_FLOOR, m[s].vel >= 25, w);
                            ck(P_MOTIF_PB, m[s].pb >= -3.f && m[s].pb <= 3.f, w);
                            const int gapToNext =
                                (i + 1 < n) ? onsets[i + 1] - s : bli::kStepsPerBar - s;
                            ck(P_MOTIF_LEN, m[s].len > 0.f && m[s].len <= gapToNext * 1.2f + 1e-3f, w);
                            if (i > 0) {
                                snprintf(w, sizeof(w), "%s step=%d..%d notes=%d,%d", at.c_str(),
                                         onsets[i - 1], s, m[onsets[i - 1]].note, m[s].note);
                                ck(P_MOTIF_INTERVAL,
                                   std::abs(m[s].note - m[onsets[i - 1]].note) <= 12, w);
                            }
                            pcHistogram[((rel % 12) + 12) % 12]++;
                            pcTotal++;
                            pitchSamples++;
                            if (isStrongStep(s)) { strongVelSum += m[s].vel; strongVelN++; }
                            else                 { weakVelSum   += m[s].vel; weakVelN++; }
                            for (int c = 0; c < 4; c++)
                                if (((rel % 12) + 12) % 12 == chordTones[c] % 12) {
                                    chordToneHits++;
                                    break;
                                }
                            if (i > 0) {
                                const int d = std::abs(m[s].note - m[onsets[i - 1]].note);
                                intervalTotal++;
                                if (d == 0) repeatHits++;
                                if (d == 12) octaveHits++;
                                if (d > 7) wideHits++;
                            }
                            if (n > 1) {
                                const float xv = contourShapeValue(ci, float(i) / float(n - 1));
                                const float yv = float(m[s].note - root);
                                corrSxy[ci] += double(xv) * yv;
                                corrSxx[ci] += double(xv) * xv;
                                corrSyy[ci] += double(yv) * yv;
                                corrSx[ci] += xv;
                                corrSy[ci] += yv;
                                corrN[ci]++;
                            }
                        }

                        bli::TurnNote turn[3];
                        const int tn = bli::generateTurnaround(turn, root, scaleIdx, dials, rng);
                        ck(P_TURN_COUNT, tn >= 1 && tn <= 3, at);
                        for (int i = 0; i < tn; i++) {
                            char w[256];
                            snprintf(w, sizeof(w), "%s turn[%d] off=%.3f note=%d", at.c_str(), i,
                                     turn[i].stepOffset, turn[i].note);
                            ck(P_TURN_NONNEG, turn[i].stepOffset >= 0.f, w);
                            if (i > 0)
                                ck(P_TURN_INCREASING,
                                   turn[i].stepOffset > turn[i - 1].stepOffset, w);
                            if (i > 0) {
                                snprintf(w, sizeof(w), "%s turn gap %.4f steps", at.c_str(),
                                         turn[i].stepOffset - turn[i - 1].stepOffset);
                                ck(P_TURN_GAP,
                                   turn[i].stepOffset - turn[i - 1].stepOffset
                                       >= kTurnGapFloorSteps, w);
                            }
                            const int rel = turn[i].note - root;
                            ck(P_TURN_REGISTER, rel >= 0 && rel <= bli::kRegisterSpan, w);
                            ck(P_TURN_VEL, turn[i].vel >= 1 && turn[i].vel <= 127, w);
                            ck(P_TURN_LEN, turn[i].len > 0.f && turn[i].len < 2.f, w);
                        }
                    }
                    }
                }
            }
        }
    }

    double top = onsetSum[kNumEnergies - 1] / double(onsetN[kNumEnergies - 1]);
    for (int ei = 0; ei < kNumEnergies; ei++) {
        double mean = onsetSum[ei] / double(onsetN[ei]);
        char w[192];
        snprintf(w, sizeof(w), "energy=%.2f meanOnsets=%.3f topEnergyMean=%.3f",
                 kEnergies[ei], mean, top);
        ck(P_MOTIF_DENSITY_PEAK, top >= mean - 1e-9, w);
    }

    const double chordShare = double(chordToneHits) / double(pitchSamples);
    const double repeatShare = double(repeatHits) / double(intervalTotal);
    const double octaveShare = double(octaveHits) / double(intervalTotal);
    const double wideShare = double(wideHits) / double(intervalTotal);
    printf("measurements: chordToneShare=%.4f repeatShare=%.4f octaveShare=%.4f wideLeapShare=%.4f"
           " topEnergyMeanOnsets=%.3f step0=%.4f\n",
           chordShare, repeatShare, octaveShare, wideShare, top,
           double(step0Hit) / double(step0Tot));
    {
        char w[192];
        snprintf(w, sizeof(w), "chordToneShare=%.4f (want >= %.2f)", chordShare, kChordToneShareMin);
        ck(P_MOTIF_CHORDTONE, chordShare >= kChordToneShareMin, w);
        snprintf(w, sizeof(w), "repeatShare=%.4f (want <= %.2f)", repeatShare, kRepeatShareMax);
        ck(P_MOTIF_REPEAT, repeatShare <= kRepeatShareMax, w);
        snprintf(w, sizeof(w), "octaveShare=%.4f (want <= %.2f)", octaveShare, kOctaveShareMax);
        ck(P_MOTIF_OCTAVE, octaveShare <= kOctaveShareMax, w);
        snprintf(w, sizeof(w), "wideLeapShare=%.4f (want <= %.2f)", wideShare, kWideLeapShareMax);
        ck(P_MOTIF_WIDE, wideShare <= kWideLeapShareMax, w);
        snprintf(w, sizeof(w), "topEnergyMeanOnsets=%.3f (want >= %.2f)", top, kTopEnergyRollsMin);
        ck(P_MOTIF_TOP_ROLL, top >= kTopEnergyRollsMin, w);
    const double strongVel = strongVelSum / double(strongVelN);
    const double weakVel   = weakVelSum / double(weakVelN);
    printf("measurements: strongStepVel=%.2f weakStepVel=%.2f meterAccentGap=%.2f\n",
           strongVel, weakVel, strongVel - weakVel);
    {
        char w[192];
        snprintf(w, sizeof(w), "strongStepVel=%.2f weakStepVel=%.2f gap=%.2f (want >= %.2f)",
                 strongVel, weakVel, strongVel - weakVel, kMeterAccentMinVel);
        ck(P_MOTIF_METER_ACCENT, strongVel - weakVel >= kMeterAccentMinVel, w);
    }
        const double lowMean = onsetSum[0] / double(onsetN[0]);
        snprintf(w, sizeof(w), "lowEnergyMeanOnsets=%.3f (want <= %.2f)", lowMean, kSparseLowMax);
        ck(P_MOTIF_SPARSE_LOW, lowMean <= kSparseLowMax, w);
    }
    for (int ci = 0; ci < kNumContours; ci++) {
        const double n = double(corrN[ci]);
        const double sxy = corrSxy[ci] - corrSx[ci] * corrSy[ci] / n;
        const double sxx = corrSxx[ci] - corrSx[ci] * corrSx[ci] / n;
        const double syy = corrSyy[ci] - corrSy[ci] * corrSy[ci] / n;
        const double r = (sxx <= 0.0 || syy <= 0.0) ? 1.0 : sxy / std::sqrt(sxx * syy);
        printf("measurements: contour band %d r=%.4f\n", ci, r);
        char w[192];
        snprintf(w, sizeof(w), "contour band %d r=%.4f (want >= %.2f)", ci, r, kContourCorrMin);
        ck(P_MOTIF_CONTOUR, r >= kContourCorrMin, w);
    }
    {
        int mode = 0;
        for (int pc = 1; pc < 12; pc++) if (pcHistogram[pc] > pcHistogram[mode]) mode = pc;
        char w[256];
        snprintf(w, sizeof(w), "modal pitch class is +%d (%ld/%ld), root share %ld/%ld"
                 " (want the root to lead)", mode, pcHistogram[mode], pcTotal,
                 pcHistogram[0], pcTotal);
        ck(P_MOTIF_ROOT, mode == 0, w);
    }
    {
        char w[192];
        snprintf(w, sizeof(w), "step0fraction=%.4f (want >= %.2f)",
                 double(step0Hit) / double(step0Tot), kStep0MajorityMin);
        ck(P_MOTIF_STEP0, double(step0Hit) / double(step0Tot) >= kStep0MajorityMin, w);
    }
}

// The witness pins the bass window itself (kRootNote +/-), independently of the engine's own
// constant, so a mutation of kRootNoteE2 is caught here rather than being self-consistent.
constexpr int kEngineNoteLo = kRootNote - 7;
constexpr int kEngineNoteHi = kRootNote + 12;
constexpr int kMaxSilentBars = 2;

// One phrase per approach samples each voice's turnaround once, and a turnaround is drawn only
// every few phrases, so a single pass leaves rare branches (the octave-down closing note) unseen.
// Four passes per approach make the engine-side checks cover the branch the matrix hits head-on.
constexpr int kEnginePasses = 4;

void runEngineProperties() {
    for (int pass = 0; pass < kEnginePasses; pass++) {
    for (int a = 0; a < BassEngine::APP_COUNT; a++) {
        simulateApproach(a);
        char at[64];
        snprintf(at, sizeof(at), "pass=%d approach=%d", pass, a);

        int grid[kBars][kStepsBar];
        for (int b = 0; b < kBars; b++) for (int i = 0; i < kStepsBar; i++) grid[b][i] = -1;
        size_t notes = 0;
        bool collided = false;
        for (const Ev& e : g_events) {
            if (e.kind != 0) continue;
            notes++;
            char w[128];
            snprintf(w, sizeof(w), "%s note=%d vel=%d", at, (int)e.a, (int)e.b);
            ck(P_ENG_WINDOW, e.a >= kEngineNoteLo && e.a <= kEngineNoteHi, w);
            ck(P_ENG_VEL, e.b >= 1 && e.b <= 127, w);
            const double pos = (double)e.tick / (double)kTpq * 4.0;
            const int step = (int)(pos + 0.5) % (kBars * kStepsBar);
            const int cell = grid[step / kStepsBar][step % kStepsBar];
            if (cell >= 0 && cell != (int)e.a) collided = true;
            grid[step / kStepsBar][step % kStepsBar] = e.a;
        }
        ck(P_ENG_SOUNDS, notes > 0, at);
        ck(P_ENG_COLLISION, !collided, at);

        int silRun = 0, worstSil = 0;
        for (int b = 0; b < kBars; b++) {
            bool empty = true;
            for (int i = 0; i < kStepsBar; i++) if (grid[b][i] >= 0) { empty = false; break; }
            if (empty) { silRun++; if (silRun > worstSil) worstSil = silRun; } else silRun = 0;
        }
        char w[128];
        snprintf(w, sizeof(w), "%s longestSilence=%d (want <= %d)", at, worstSil, kMaxSilentBars);
        ck(P_ENG_SILENCE, worstSil <= kMaxSilentBars, w);
    }
    }
}

int reportProperties() {
    int held = 0;
    std::string failing;
    for (int i = 0; i < P_COUNT; i++) {
        const Prop& p = g_prop[i];
        const bool ok = p.fails == 0;
        if (ok) held++;
        else {
            if (!failing.empty()) failing += ",";
            failing += p.name;
        }
    }
    printf("property suite: interpreter matrix %d roots x %d scales x %d energies x %d variations"
           " x %d seeds, engine %d approaches\n",
           (int)(sizeof(kRoots) / sizeof(kRoots[0])), bli::kNumScales, kNumEnergies,
           (int)(sizeof(kVariations) / sizeof(kVariations[0])), kSeeds, BassEngine::APP_COUNT);
    for (int i = 0; i < P_COUNT; i++) {
        const Prop& p = g_prop[i];
        const bool ok = p.fails == 0;
        printf("  %-5s %-34s checks=%-8ld %s%s\n", ok ? "HOLD" : "FAIL", p.name, p.checks,
               ok ? "" : "counterexample: ", ok ? "" : p.ce.c_str());
    }
    printf("properties held: %d/%d\n", held, P_COUNT);

    printf("rejected as body-derivable (not admitted, not checked):\n");
    printf("  - kickShare below 50%% for melodic voices: readable off the psy_roll weights on"
           " steps 0/4/8/12\n");
    printf("  - microtiming within its cap: already proven by static_assert in"
           " main/bassline_interpreter.cpp\n");
    printf("  - every scale row fits its width: already proven by static_assert\n");
    printf("  - a motif is never empty: guarded explicitly by `if (n == 0)` in buildOnsets\n");
    printf("  - turnaround closes on the root: readable off kClosingOctaves\n");
    printf("failing: %s\n", failing.empty() ? "none" : failing.c_str());
    return held == P_COUNT ? 0 : 1;
}

struct Mutant {
    const char* id;
    const char* file;
    const char* name;
    const char* hint;   // the one line to touch, found by substring
    const char* from;
    const char* to;
};

// Mutation set `psy-bass-v1`: 20 source-level mutations of the two files the bassline generator
// lives in. Anchors are line-hints plus an in-line replacement, so reformatted whitespace does not
// silently no-op a mutant (an anchor that does not match aborts the run instead).
static const Mutant kMutants[] = {
    {"M01", "bass_engine.cpp",          "root-e2-plus-1",            "kRootNoteE2  ",            "40",                                       "41"},
    {"M02", "bass_engine.cpp",          "root-e2-minus-12",          "kRootNoteE2  ",            "40",                                       "28"},
    {"M03", "bass_engine.cpp",          "bass-high-note-plus-12",    "kBassHighNote  ",          "+ 12",                                     "+ 24"},
    {"M04", "bassline_interpreter.cpp", "psy-roll-collapsed",        "{\"psy_roll\"",                   "1, .93f, .9f, .93f, 1, .93f, .9f, .93f, 1, .93f, .9f, .93f, 1, .93f, .9f, .96f", "1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0"},
    {"M05", "bassline_interpreter.cpp", "groove-band-order-reversed","int gIdx = Dials::bandIndex(",    "Dials::bandIndex(energy, kNumGrooves)",    "(kNumGrooves - 1 - Dials::bandIndex(energy, kNumGrooves))"},
    {"M06", "bassline_interpreter.cpp", "octave-transit-free",       "if (d == 12) return",             "0.30f",                                    "0.f"},
    {"M07", "bassline_interpreter.cpp", "repeat-transit-free",       "if (d == 0)  return",             "0.18f",                                    "0.f"},
    {"M08", "bassline_interpreter.cpp", "nonchord-cost-free",        "0.75f * tension",                 "0.95f",                                    "0.f"},
    {"M09", "bassline_interpreter.cpp", "root-cost-raised",          "kChordRoot] % 12) return 0.f;",   "return 0.f;",                              "return 0.5f;"},
    {"M10", "bassline_interpreter.cpp", "contour-arch-inverted",     "case CT_ARCH:",                   "std::sin(float(M_PI) * x)",                "-std::sin(float(M_PI) * x)"},
    {"M11", "bassline_interpreter.cpp", "metric-first-half-flat",    "1, .12f, .34f, .16f,  .78f, .12f, .36f, .16f,", "1, .12f, .34f, .16f,  .78f, .12f, .36f, .16f,", "1, 1, 1, 1,  1, 1, 1, 1,"},
    {"M12", "bassline_interpreter.cpp", "downbeat-anchor-off",       "if (rng.next01() < 0.7f) {",      "0.7f",                                     "-1.f"},
    {"M13", "bassline_interpreter.cpp", "ghost-drop-45",             "ghost ? 25 : 0",           "ghost ? 25 : 0",                           "ghost ? 45 : 0"},
    {"M14", "bassline_interpreter.cpp", "leap-window-24",            "int leap = 7 +",                  "7 +",                                      "24 +"},
    {"M15", "bassline_interpreter.cpp", "pool-scale-filter-off",     "if (inScale) pool[poolLen++]",    "if (inScale) ",                            ""},
    {"M16", "bassline_interpreter.cpp", "turnaround-gap-zero",       "kMinNoteGapSteps  ",       "0.5f",                                     "0.0f"},
    {"M17", "bassline_interpreter.cpp", "turnaround-squeeze-off",    "if (offsets[0] < 0.f) {",         "if (offsets[0] < 0.f)",                    "if (false)"},
    {"M18", "bassline_interpreter.cpp", "leap-penalty-off",          "if (d > leap) c +=",              "if (d > leap)",                            "if (false)"},
    {"M19", "bassline_interpreter.cpp", "density-slope-flat",        "float density = 0.32f",           "0.32f + energy * 0.62f",                   "0.6f"},
    {"M20", "bassline_interpreter.cpp", "note-transposed-plus-7",    "m[s].note = pitches[i];",         "pitches[i];",                              "pitches[i] + 7;"},
};

int listMutants() {
    for (const Mutant& m : kMutants) printf("%s %s %s\n", m.id, m.file, m.name);
    return 0;
}

std::string readFile(const char* path, bool& ok) {
    FILE* fp = fopen(path, "rb");
    if (!fp) { ok = false; return {}; }
    std::string s;
    char buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) s.append(buf, n);
    fclose(fp);
    ok = true;
    return s;
}

int applyMutation(const char* id, const char* dir) {
    const Mutant* m = nullptr;
    for (const Mutant& c : kMutants) if (!strcmp(c.id, id)) m = &c;
    if (!m) { printf("unknown mutant %s\n", id); return 2; }

    const std::string path = std::string(dir) + "/" + m->file;
    bool ok = false;
    std::string src = readFile(path.c_str(), ok);
    if (!ok) { printf("cannot read %s\n", path.c_str()); return 2; }

    size_t pos = 0, hit = std::string::npos;
    int hits = 0;
    while ((pos = src.find(m->hint, pos)) != std::string::npos) {
        hit = pos; hits++; pos += strlen(m->hint);
    }
    if (hits != 1) {
        printf("anchor %s matched %d lines in %s (want exactly 1)\n", m->hint, hits, path.c_str());
        return 2;
    }
    size_t lineBeg = src.rfind('\n', hit);
    lineBeg = (lineBeg == std::string::npos) ? 0 : lineBeg + 1;
    size_t lineEnd = src.find('\n', hit);
    lineEnd = (lineEnd == std::string::npos) ? src.size() : lineEnd;

    std::string line = src.substr(lineBeg, lineEnd - lineBeg);
    const size_t at = line.find(m->from);
    if (at == std::string::npos) {
        printf("pattern %s not on the anchored line of %s\n", m->from, path.c_str());
        return 2;
    }
    line.replace(at, strlen(m->from), m->to);
    src.replace(lineBeg, lineEnd - lineBeg, line);

    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) { printf("cannot write %s\n", path.c_str()); return 2; }
    fwrite(src.data(), 1, src.size(), fp);
    fclose(fp);
    printf("applied %s (%s) to %s\n", m->id, m->name, path.c_str());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && !strcmp(argv[1], "--mutants")) return listMutants();
    if (argc == 4 && !strcmp(argv[1], "--mutate")) return applyMutation(argv[2], argv[3]);
    if (argc == 2 && !strcmp(argv[1], "--property")) {
        runInterpreterProperties();
        runEngineProperties();
        return reportProperties();
    }    const char* midPath = nullptr;
    bool all = false;
    int  only = -1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--mid") && i + 1 < argc) midPath = argv[++i];
        else if (!strcmp(argv[i], "--all")) all = true;
        else if (!strcmp(argv[i], "--approach") && i + 1 < argc) only = atoi(argv[++i]);
        else { printf("usage: bassline_witness [--all] [--approach N] [--mid file.mid]\n"); return 2; }
    }
    if (only >= 0) {
        static const char* kNames[8] = {"ROLL","ACID","FUNK","GLITCH","STAB","DRIFT","PEDAL","BREAK"};
        runApproach(only, (only < 8 ? kNames[only] : "?"), true, midPath);
        return 0;
    }
    runApproach(BassEngine::APP_ROLL, "ROLL", !all, midPath);
    if (all) {
        runApproach(BassEngine::APP_ACID,   "ACID",   false, nullptr);
        runApproach(BassEngine::APP_FUNK,   "FUNK",   false, nullptr);
        runApproach(BassEngine::APP_GLITCH, "GLITCH", false, nullptr);
        runApproach(BassEngine::APP_STAB,   "STAB",   false, nullptr);
        runApproach(BassEngine::APP_DRIFT,  "DRIFT",  false, nullptr);
        runApproach(BassEngine::APP_PEDAL,  "PEDAL",  false, nullptr);
        runApproach(BassEngine::APP_BREAK,  "BREAK",  false, nullptr);
    }
    return 0;
}
