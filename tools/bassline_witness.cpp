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

void runApproach(int approach, const char* name, bool showGrid, const char* midPath) {
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

int main(int argc, char** argv) {
    const char* midPath = nullptr;
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
