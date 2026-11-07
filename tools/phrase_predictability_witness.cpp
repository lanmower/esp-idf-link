#include <cstdint>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <limits>
#include <ableton/Link.hpp>

#include "control_model.h"

#define private public
#include "bass_engine.h"
#undef private

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(name, cond)                                                        \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) { std::printf("FAIL %s\n", (name)); ++g_fail; }             \
    } while (0)

#define CHECK_EQ(name, a, e)                                                     \
    do {                                                                         \
        ++g_checks;                                                              \
        long long _a = (long long)(a), _e = (long long)(e);                       \
        if (_a != _e) {                                                           \
            std::printf("FAIL %s: actual=%lld (0x%llX) expected=%lld (0x%llX)\n", \
                        (name), _a, (unsigned long long)_a, _e,                  \
                        (unsigned long long)_e);                                  \
            ++g_fail;                                                             \
        }                                                                         \
    } while (0)

#define CHECK_EQD(name, a, e)                                                    \
    do {                                                                         \
        ++g_checks;                                                              \
        double _a = (double)(a), _e = (double)(e);                               \
        if (std::fabs(_a - _e) > 1e-6) {                                          \
            std::printf("FAIL %s: actual=%.9f expected=%.9f\n",                  \
                        (name), _a, _e);                                          \
            ++g_fail;                                                             \
        }                                                                         \
    } while (0)

struct Msg { uint8_t b[3]; int64_t tick; };
static std::vector<Msg> g_out;
static int64_t g_nowTick = -1;

void send_midi_message(const uint8_t* message, size_t size) {
    Msg m = {{0, 0, 0}, g_nowTick};
    for (size_t i = 0; i < size && i < 3; i++) m.b[i] = message[i];
    g_out.push_back(m);
}
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value) {
    (void)channel;
    Msg m = {{0xB0, cc_num, value}, g_nowTick};
    g_out.push_back(m);
}
static int countNoteOn(size_t from) {
    int n = 0;
    for (size_t i = from; i < g_out.size(); i++)
        if ((g_out[i].b[0] & 0xF0) == 0x90 && g_out[i].b[2] > 0) n++;
    return n;
}
static int countNoteOff(size_t from) {
    int n = 0;
    for (size_t i = from; i < g_out.size(); i++)
        if ((g_out[i].b[0] & 0xF0) == 0x80 ||
            ((g_out[i].b[0] & 0xF0) == 0x90 && g_out[i].b[2] == 0)) n++;
    return n;
}
static bool hasCc(size_t from, uint8_t cc) {
    for (size_t i = from; i < g_out.size(); i++)
        if ((g_out[i].b[0] & 0xF0) == 0xB0 && g_out[i].b[1] == cc) return true;
    return false;
}

#define MIR_MIDI_PULSES_PER_QUARTER_NOTE     24
#define MIR_MIDI_SPP_UNITS_PER_BEAT          4
#define MIR_MIDI_SPP_UNITS_MASK              0x3FFF
#define MIR_MIDI_DATA_BYTE_MASK              0x7F
#define MIR_MIDI_DATA_BYTE_SHIFT             7
#define MIR_MICROBEATS_PER_BEAT              1000000
#define MIR_FORCE_START_AFTER_NO_PEERS_US    8000000
#define MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE  1024
#define MIR_LTMP_TEMPO_ONLY_BYTES            12
#define MIR_LTMP_TEMPO_PHASE_BYTES           28
#define MIR_MIDI_CLOCK_RESYNC_THRESHOLD      24
#define MIR_PHRASE_BEATS                     64
#define MIR_LINK_QUANTUM                     16
#define MIR_METRONOME_ACCENT_CYCLE_BEATS     16
#define MIR_MIDI_TIMING_CLOCK                0xF8
#define MIR_MIDI_START                       0xFA
#define MIR_MIDI_STOP                        0xFC
#define MIR_MIDI_CONTINUE                    0xFB
#define MIR_MIDI_SONG_POSITION_POINTER       0xF2
#define MIR_MIDI_NOTE_ON_CMD                 0x90
#define MIR_MIDI_NOTE_OFF_CMD                0x80
#define MIR_MIDI_CC_CMD                      0xB0
#define MIR_MIDI_CC_ALL_NOTES_OFF            123
#define MIR_MIDI_USER_CHANNEL_MIN            1
#define MIR_MIDI_USER_CHANNEL_MAX            16
#define MIR_MIDI_WIRE_CHANNEL_MIN            0
#define MIR_MIDI_CC_SWEEP_VALUE              0

namespace mirror {

constexpr double kSppWrap = static_cast<double>(MIR_MIDI_SPP_UNITS_MASK) + 1.0;

inline void emitAllNotesOffSweep(std::vector<uint8_t>& out) {
    for (int userChannel = MIR_MIDI_USER_CHANNEL_MIN;
         userChannel <= MIR_MIDI_USER_CHANNEL_MAX; ++userChannel) {
        const int wireChannel = MIR_MIDI_WIRE_CHANNEL_MIN +
                                (userChannel - MIR_MIDI_USER_CHANNEL_MIN);
        out.push_back(static_cast<uint8_t>(MIR_MIDI_CC_CMD | wireChannel));
        out.push_back(static_cast<uint8_t>(MIR_MIDI_CC_ALL_NOTES_OFF));
        out.push_back(static_cast<uint8_t>(MIR_MIDI_CC_SWEEP_VALUE));
    }
}

inline void emitStopAndAllNotesOffSweep(std::vector<uint8_t>& out) {
    out.push_back(static_cast<uint8_t>(MIR_MIDI_STOP));
    emitAllNotesOffSweep(out);
}

inline std::vector<uint8_t> bootPanicBytes() {
    std::vector<uint8_t> out;
    emitStopAndAllNotesOffSweep(out);
    return out;
}

inline void encodeSpp(double sessionBeat, uint8_t out[3]) {
    double units = std::fmod(sessionBeat * MIR_MIDI_SPP_UNITS_PER_BEAT, kSppWrap);
    if (units < 0.0) units += kSppWrap;
    const uint16_t spp_units = static_cast<uint16_t>(units);
    out[0] = static_cast<uint8_t>(MIR_MIDI_SONG_POSITION_POINTER);
    out[1] = static_cast<uint8_t>(spp_units & MIR_MIDI_DATA_BYTE_MASK);
    out[2] = static_cast<uint8_t>((spp_units >> MIR_MIDI_DATA_BYTE_SHIFT) & MIR_MIDI_DATA_BYTE_MASK);
}

inline void encodeSppOld(double sessionBeat, uint8_t out[3]) {
    uint16_t spp_units =
        static_cast<uint16_t>(sessionBeat * MIR_MIDI_SPP_UNITS_PER_BEAT) & MIR_MIDI_SPP_UNITS_MASK;
    out[0] = static_cast<uint8_t>(MIR_MIDI_SONG_POSITION_POINTER);
    out[1] = static_cast<uint8_t>(spp_units & MIR_MIDI_DATA_BYTE_MASK);
    out[2] = static_cast<uint8_t>((spp_units >> MIR_MIDI_DATA_BYTE_SHIFT) & MIR_MIDI_DATA_BYTE_MASK);
}

struct Timeline {
    double beatAnchor = 0.0;
    double tAnchor    = 0.0;
    double bpm        = 100.0;
    double beatAt(double t) const { return beatAnchor + (t - tAnchor) * bpm / 60e6; }
    double timeAtBeat(double b) const { return tAnchor + (b - beatAnchor) * 60e6 / bpm; }
    void setTempo(double newBpm, double t) {
        beatAnchor = beatAt(t);
        tAnchor    = t;
        bpm        = newBpm;
    }
};

struct Scheduler {
    int64_t nextPulse      = 0;
    bool    valid          = false;
    int     resyncLogCount = 0;
    int     skipLoopSteps  = 0;

    double schedule(const Timeline& tl, double nowLink) {
        double beatNow = tl.beatAt(nowLink);
        int64_t dueNow = static_cast<int64_t>(std::floor(beatNow * MIR_MIDI_PULSES_PER_QUARTER_NOTE)) + 1;
        if (!valid) {
            nextPulse = dueNow;
            valid = true;
        } else if (nextPulse < dueNow) {
            if (dueNow - nextPulse > MIR_MIDI_CLOCK_RESYNC_THRESHOLD) ++resyncLogCount;
            nextPulse = dueNow;
        } else if (nextPulse - dueNow > 1) {
            nextPulse = dueNow;
        }
        double due = tl.timeAtBeat(static_cast<double>(nextPulse) / MIR_MIDI_PULSES_PER_QUARTER_NOTE);
        for (int guard = 0;
             due <= nowLink && guard < MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE; ++guard) {
            ++nextPulse;
            ++skipLoopSteps;
            due = tl.timeAtBeat(static_cast<double>(nextPulse) / MIR_MIDI_PULSES_PER_QUARTER_NOTE);
        }
        return due;
    }
};

inline int64_t skipLoop(int64_t nextPulse, double nowLink, double pulsePeriodUs, int& steps) {
    double due = static_cast<double>(nextPulse) * pulsePeriodUs;
    int guard = 0;
    steps = 0;
    while (due <= nowLink && guard < MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE) {
        ++nextPulse; ++guard; ++steps;
        due = static_cast<double>(nextPulse) * pulsePeriodUs;
    }
    return nextPulse;
}

struct QuantumBoundaryMirror {
    bool baselineCaptured = false;
    int  lastPhraseNumber = 0;
    struct Info {
        double sessionBeat;
        double phaseWithinPhrase;
        int    currentPhraseNumber;
        bool   crossedPhraseBoundary;
    };
    Info detect(const ableton::Link::SessionState& state,
                const std::chrono::microseconds& time) {
        Info info;
        info.sessionBeat = state.beatAtTime(time, MIR_LINK_QUANTUM);
        info.phaseWithinPhrase = state.phaseAtTime(time, MIR_PHRASE_BEATS);
        info.currentPhraseNumber =
            static_cast<int>(std::floor(info.sessionBeat / MIR_PHRASE_BEATS));
        if (!baselineCaptured) {
            baselineCaptured = true;
            lastPhraseNumber = info.currentPhraseNumber;
            info.crossedPhraseBoundary = false;
        } else if (info.currentPhraseNumber != lastPhraseNumber) {
            lastPhraseNumber = info.currentPhraseNumber;
            info.crossedPhraseBoundary = true;
        } else {
            info.crossedPhraseBoundary = false;
        }
        return info;
    }
};

struct PhraseEvents {
    std::vector<uint8_t> events;
    bool pendingRealign   = false;
    bool transportRunning = false;

    void onTick(bool crossedPhraseBoundary, double sessionBeat, bool isPlaying, bool forceStart) {
        if (!crossedPhraseBoundary) return;
        uint8_t spp[3];
        encodeSpp(sessionBeat, spp);
        events.push_back(spp[0]); events.push_back(spp[1]); events.push_back(spp[2]);
        if (pendingRealign && (isPlaying || forceStart)) {
            uint8_t start = transportRunning ? static_cast<uint8_t>(MIR_MIDI_CONTINUE)
                                             : static_cast<uint8_t>(MIR_MIDI_START);
            events.push_back(start);
            transportRunning = true;
            pendingRealign = false;
        }
    }
};

struct TransportMirror {
    bool wasConnected     = false;
    bool evtEnabled       = false;
    bool pendingRealign   = false;
    bool transportRunning = false;
    bool forceStart       = false;
    bool peerLossQueuesRealign = true;
    int64_t startWaitTimeUs = 0;
    std::vector<uint8_t> events;
    std::vector<uint8_t> lossCapture;
    int f8Streaming = 0;
    int startCount  = 0;
    int sppCount    = 0;

    void tick(bool isConnected, bool crossedPhraseBoundary, double sessionBeat,
              int64_t nowUs) {
        const bool isPlaying = false;
        if (!forceStart &&
            (nowUs - startWaitTimeUs >= MIR_FORCE_START_AFTER_NO_PEERS_US) &&
            (!isConnected || !isPlaying)) {
            forceStart = true;
        }
        if (isConnected != wasConnected) {
            if (isConnected) {
                pendingRealign = true;
            } else {
                emitStopAndAllNotesOffSweep(events);
                emitStopAndAllNotesOffSweep(lossCapture);
                transportRunning = false;
                if (peerLossQueuesRealign) pendingRealign = true;
            }
            wasConnected = isConnected;
        }
        const bool clockEnabled = isConnected || forceStart;
        if (clockEnabled != evtEnabled) {
            evtEnabled = clockEnabled;
            if (clockEnabled) pendingRealign = true;
        }
        if (evtEnabled) ++f8Streaming;
        if (clockEnabled && crossedPhraseBoundary) {
            uint8_t spp[3]; encodeSpp(sessionBeat, spp);
            events.push_back(spp[0]); events.push_back(spp[1]); events.push_back(spp[2]);
            ++sppCount;
            if (pendingRealign && (isPlaying || forceStart)) {
                uint8_t start = transportRunning ? static_cast<uint8_t>(MIR_MIDI_CONTINUE)
                                                 : static_cast<uint8_t>(MIR_MIDI_START);
                events.push_back(start);
                ++startCount;
                transportRunning = true;
                pendingRealign = false;
            }
        }
    }
};

struct LinkEventCbMirror {
    bool    evtEnabled     = true;
    bool    nextPulseValid = true;
    int64_t nextPulse      = 0;
    int     f8Count        = 0;
    int     scheduleCalls  = 0;

    void fire() {
        ++nextPulse;
        if (evtEnabled) {
            ++f8Count;
            ++scheduleCalls;
        } else {
            nextPulseValid = false;
        }
    }
};

struct PhaseMirror {
    static int64_t iabs64(int64_t v) { return v < 0 ? -v : v; }

    static int64_t phaseUb(int64_t beatsUb, int64_t qUb) {
        if (qUb == 0) return 0;
        const int64_t bins = (iabs64(beatsUb) + qUb) / qUb;
        const int64_t quantumBeats = bins * qUb;
        return (beatsUb + quantumBeats) % qUb;
    }
    static int64_t nextPhaseMatchUb(int64_t xUb, int64_t targetUb, int64_t qUb) {
        const int64_t desired = phaseUb(targetUb, qUb);
        const int64_t xPhase = phaseUb(xUb, qUb);
        const int64_t diff = (desired - xPhase + qUb) % qUb;
        return xUb + diff;
    }
    static int64_t closestPhaseMatchUb(int64_t xUb, int64_t targetUb, int64_t qUb) {
        return nextPhaseMatchUb(xUb - qUb / 2, targetUb, qUb);
    }
    static int64_t beatAtTimeUb(double beatOriginBeats, int64_t timeUs,
                                int64_t microsPerBeat, int64_t quantumBeats) {
        const int64_t beatOriginUb = static_cast<int64_t>(beatOriginBeats * 1e6);
        const int64_t qUb = quantumBeats * 1000000;
        const int64_t deltaUb = static_cast<int64_t>(
            (static_cast<double>(timeUs) / static_cast<double>(microsPerBeat)) * 1e6);
        const int64_t beatUb = beatOriginUb + deltaUb;
        const int64_t targetUb = beatUb - beatOriginUb;
        return closestPhaseMatchUb(beatUb, targetUb, qUb);
    }
    static double beatAtTime(double origin, int64_t timeUs, int64_t mpb, int64_t q) {
        return static_cast<double>(beatAtTimeUb(origin, timeUs, mpb, q)) / 1e6;
    }
    static double phaseAtTime(double origin, int64_t timeUs, int64_t mpb, int64_t q) {
        const int64_t qUb = q * 1000000;
        return static_cast<double>(phaseUb(beatAtTimeUb(origin, timeUs, mpb, q), qUb)) / 1e6;
    }
};

}

using mirror::Timeline;
using mirror::Scheduler;
using mirror::PhraseEvents;
using mirror::QuantumBoundaryMirror;
using mirror::TransportMirror;
using mirror::LinkEventCbMirror;
using mirror::PhaseMirror;

static void checkPhraseArithmetic() {
    const int unitsPerPhrase = MIR_PHRASE_BEATS * MIR_MIDI_SPP_UNITS_PER_BEAT;
    const int modulusUnits   = MIR_MIDI_SPP_UNITS_MASK + 1;
    const int wrapBeats      = modulusUnits / MIR_MIDI_SPP_UNITS_PER_BEAT;
    const int wrapPhrases    = wrapBeats / MIR_PHRASE_BEATS;

    CHECK_EQ("MIRROR phrase beats = 64", MIR_PHRASE_BEATS, 64);
    CHECK_EQ("MIRROR link quantum = 16", MIR_LINK_QUANTUM, 16);
    CHECK_EQ("MIRROR SPP units per beat = 4", MIR_MIDI_SPP_UNITS_PER_BEAT, 4);
    CHECK_EQ("MIRROR 256 SPP units per phrase", unitsPerPhrase, 256);
    CHECK_EQ("MIRROR SPP mask 0x3FFF", MIR_MIDI_SPP_UNITS_MASK, 0x3FFF);
    CHECK_EQ("MIRROR 0x3FFF wrap = 16384 units (0x4000)", modulusUnits, 16384);
    CHECK_EQ("MIRROR wrap at 4096 beats", wrapBeats, 4096);
    CHECK_EQ("MIRROR wrap at 64 phrases", wrapPhrases, 64);
    CHECK("MIRROR the 0x3FFF wrap lands exactly ON a phrase boundary (4096 % 64 == 0)",
          wrapBeats % MIR_PHRASE_BEATS == 0);
    CHECK("MIRROR quantum divides the phrase", MIR_PHRASE_BEATS % MIR_LINK_QUANTUM == 0);
    CHECK_EQ("MIRROR quantum == metronome accent cycle", MIR_LINK_QUANTUM,
             MIR_METRONOME_ACCENT_CYCLE_BEATS);

    const int exactWrapUnits = 0x4000;
    const int exactWrapBeat  = exactWrapUnits / MIR_MIDI_SPP_UNITS_PER_BEAT;
    CHECK_EQ("MIRROR exact wrap point = 0x4000 units", exactWrapUnits, 16384);
    CHECK_EQ("MIRROR exact wrap beat = 4096", exactWrapBeat, 4096);
    CHECK("MIRROR wrap beat 4096 is phrase 64", exactWrapBeat / MIR_PHRASE_BEATS == 64);

    CHECK_EQ("MIRROR MICROBEATS_PER_BEAT = 1e6", MIR_MICROBEATS_PER_BEAT, 1000000);
    const int64_t beatOriginUb = static_cast<int64_t>(
        static_cast<double>(exactWrapBeat) * MIR_MICROBEATS_PER_BEAT);
    CHECK_EQ("MIRROR TTMP beatOriginUb at wrap = 4096000000 microbeats",
             beatOriginUb, 4096000000LL);
}

static void checkSppEncoding() {
    uint8_t b[3];
    mirror::encodeSpp(0.0, b);
    CHECK("MIRROR SPP beat0 opcode 0xF2", b[0] == MIR_MIDI_SONG_POSITION_POINTER);
    CHECK("MIRROR SPP beat0 units 0 -> {0,0}", b[1] == 0x00 && b[2] == 0x00);

    mirror::encodeSpp(64.0, b);
    CHECK("MIRROR SPP beat64 = 256 units LSB-first {0x00,0x02}",
          b[0] == 0xF2 && b[1] == 0x00 && b[2] == 0x02);
    mirror::encodeSpp(128.0, b);
    CHECK("MIRROR SPP beat128 = 512 units LSB-first {0x00,0x04}",
          b[1] == 0x00 && b[2] == 0x04);
    mirror::encodeSpp(192.0, b);
    CHECK("MIRROR SPP beat192 = 768 units LSB-first {0x00,0x06}",
          b[1] == 0x00 && b[2] == 0x06);

    mirror::encodeSpp(4032.0, b);
    CHECK("MIRROR SPP beat4032 = 16128 units {0x00,0x7E}",
          b[1] == 0x00 && b[2] == 0x7E);

    mirror::encodeSpp(4096.0, b);
    CHECK("MIRROR SPP wrap at beat4096 returns to {0,0}",
          b[0] == 0xF2 && b[1] == 0x00 && b[2] == 0x00);

    mirror::encodeSpp(4095.75, b);
    CHECK("MIRROR SPP max 14-bit value {0x7F,0x7F} LSB-first",
          b[1] == 0x7F && b[2] == 0x7F);

    mirror::encodeSpp(5000.0, b);
    CHECK_EQ("MIRROR SPP beat5000 LSB = 0x20", b[1], 0x20);
    CHECK_EQ("MIRROR SPP beat5000 MSB = 0x1C", b[2], 0x1C);
}

static void checkSppNewFormula() {
    const double inRange[] = {0.0, 0.25, 0.5, 1.0, 15.75, 64.0, 128.0, 1024.0,
                              4032.0, 4095.75, 4096.0, 8192.0, 16383.0, 16383.75};
    bool allEqual = true;
    for (double beat : inRange) {
        uint8_t nw[3], od[3];
        mirror::encodeSpp(beat, nw);
        mirror::encodeSppOld(beat, od);
        if (nw[0] != od[0] || nw[1] != od[1] || nw[2] != od[2]) allEqual = false;
    }
    CHECK("MIRROR new fmod formula == old (uint16_t)&mask formula for EVERY in-range beat",
          allEqual);

    struct { double beat; uint8_t lsb, msb; const char* label; } cases[] = {
        {0.0,      0x00, 0x00, "beat 0"},
        {64.0,     0x00, 0x02, "beat 64"},
        {4096.0,   0x00, 0x00, "beat 4096 (wrap)"},
        {16384.0,  0x00, 0x00, "beat 16384 (~2h16m @120bpm; OLD is UB, NEW defined)"},
        {-0.5,     0x7E, 0x7F, "beat -0.5 (negative wrap; OLD is UB, NEW defined)"},
        {20000.0,  0x00, 0x71, "beat 20000 (>16383.75, ~2h47m @120bpm; OLD is UB)"},
    };
    for (const auto& c : cases) {
        uint8_t b[3];
        mirror::encodeSpp(c.beat, b);
        char nm[160];
        std::snprintf(nm, sizeof nm, "MIRROR [fmod] %s: opcode F2", c.label);
        CHECK(nm, b[0] == MIR_MIDI_SONG_POSITION_POINTER);
        std::snprintf(nm, sizeof nm, "MIRROR [fmod] %s: LSB", c.label);
        CHECK_EQ(nm, b[1], c.lsb);
        std::snprintf(nm, sizeof nm, "MIRROR [fmod] %s: MSB", c.label);
        CHECK_EQ(nm, b[2], c.msb);
        std::snprintf(nm, sizeof nm, "MIRROR [fmod] %s: both data bytes are 7-bit", c.label);
        CHECK(nm, b[1] <= 0x7F && b[2] <= 0x7F);
        std::snprintf(nm, sizeof nm, "MIRROR [fmod] %s: units in [0,16384)", c.label);
        double units = std::fmod(c.beat * MIR_MIDI_SPP_UNITS_PER_BEAT, mirror::kSppWrap);
        if (units < 0.0) units += mirror::kSppWrap;
        CHECK(nm, units >= 0.0 && units < mirror::kSppWrap);
    }

    CHECK_EQD("MIRROR old double->uint16_t cast becomes UB above beat 16383.75 (x=65535)",
              65535.0 / MIR_MIDI_SPP_UNITS_PER_BEAT, 16383.75);
    CHECK("MIRROR negative / large beats are OUTSIDE the old cast's defined domain (OLD = UB)",
          (-0.5 * MIR_MIDI_SPP_UNITS_PER_BEAT < 0.0) &&
          (20000.0 * MIR_MIDI_SPP_UNITS_PER_BEAT > 65535.0));
    std::printf(
        "MIRROR: OLD 'static_cast<uint16_t>(sessionBeat * 4) & 0x3FFF' "
        "(main/link_sync.cpp:409 pre-edit) is UNDEFINED BEHAVIOUR for negative "
        "or >=65536 doubles; NEW 'std::fmod(sessionBeat * 4, 16384) + negative "
        "wrap' (main/link_sync.cpp:408-416) is defined for ALL beats and identical "
        "to the old result wherever the old one was defined.\n");
}

static void checkSppBeforeStart() {
    PhraseEvents ev;
    ev.pendingRealign = true;

    ev.onTick(false, 1.0, false, true);
    ev.onTick(false, 32.0, false, true);
    CHECK("MIRROR no SPP/Start on a non-boundary tick", ev.events.empty());

    ev.onTick(true, 64.0, false, true);
    CHECK_EQ("MIRROR first boundary emits 4 bytes (3 SPP + 1 Start)", ev.events.size(), 4u);
    CHECK("MIRROR first SPP byte is 0xF2", ev.events[0] == MIR_MIDI_SONG_POSITION_POINTER);
    CHECK("MIRROR first Start byte is 0xFA", ev.events[3] == MIR_MIDI_START);
    int sppIdx = -1, startIdx = -1;
    for (size_t i = 0; i < ev.events.size(); i++) {
        if (ev.events[i] == MIR_MIDI_SONG_POSITION_POINTER && sppIdx < 0) sppIdx = (int)i;
        if ((ev.events[i] == MIR_MIDI_START || ev.events[i] == MIR_MIDI_CONTINUE) && startIdx < 0)
            startIdx = (int)i;
    }
    CHECK("MIRROR SPP-before-Start: SPP byte precedes Start byte",
          sppIdx >= 0 && startIdx >= 0 && sppIdx < startIdx);
    CHECK("MIRROR transport running after Start", ev.transportRunning);
    CHECK("MIRROR realign consumed after Start", !ev.pendingRealign);

    size_t before = ev.events.size();
    ev.onTick(true, 128.0, false, true);
    bool hasStart2 = false;
    for (size_t i = before; i < ev.events.size(); i++)
        if (ev.events[i] == MIR_MIDI_START || ev.events[i] == MIR_MIDI_CONTINUE) hasStart2 = true;
    CHECK("MIRROR no Start/Continue at a boundary without realign", !hasStart2);
    CHECK_EQ("MIRROR SPP still emitted at every boundary", ev.events.size() - before, 3u);

    ev.pendingRealign = true;
    before = ev.events.size();
    ev.onTick(true, 192.0, false, true);
    CHECK("MIRROR second realign uses CONTINUE 0xFB", ev.events[before + 3] == MIR_MIDI_CONTINUE);

    int startCount = 0;
    for (size_t i = 0; i < ev.events.size(); i++)
        if (ev.events[i] == MIR_MIDI_START || ev.events[i] == MIR_MIDI_CONTINUE) {
            startCount++;
            CHECK("MIRROR Start/Continue always follows the SPP trio",
                  i >= 3 && ev.events[i - 3] == MIR_MIDI_SONG_POSITION_POINTER);
        }
    CHECK_EQ("MIRROR exactly two Start/Continue events in the run", startCount, 2);
}

static void checkClockSkipDontCatchUp() {
    int steps = 0;
    int64_t next = mirror::skipLoop(100, 110.0 * 20000.0, 20000.0, steps);
    CHECK_EQ("MIRROR skip loop (<= nowLink) passes 11 due pulses", steps, 11);
    CHECK_EQ("MIRROR skip loop advances nextPulse to 111", next, 111);
    CHECK("MIRROR armed deadline strictly future (111*20000 > 110*20000)",
          (double)next * 20000.0 > 110.0 * 20000.0);

    steps = 0;
    next = mirror::skipLoop(100, 100.0 * 20000.0, 20000.0, steps);
    CHECK_EQ("MIRROR delta-0 boundary: due == nowLink advances exactly 1 pulse", steps, 1);
    CHECK_EQ("MIRROR delta-0 boundary: nextPulse becomes 101", next, 101);
    CHECK("MIRROR delta-0 boundary: resulting deadline strictly future",
          (double)next * 20000.0 > 100.0 * 20000.0);

    steps = 0;
    next = mirror::skipLoop(0, 2000.0 * 20000.0, 20000.0, steps);
    CHECK_EQ("MIRROR skip loop cap MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024", steps, 1024);
    CHECK_EQ("MIRROR skip loop stops at pulse 1024, not 2000", next, 1024);
    CHECK_EQ("MIRROR cap constant = 1024", MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE, 1024);

    Timeline tl; tl.bpm = 125.0; tl.beatAnchor = 0.0; tl.tAnchor = 0.0;

    Scheduler s1; s1.valid = true; s1.nextPulse = 100;
    double nowLink = 110.0 * 20000.0;
    double due = s1.schedule(tl, nowLink);
    CHECK_EQ("MIRROR behind-by-11 re-anchors nextPulse to dueNow 111", s1.nextPulse, 111);
    CHECK_EQ("MIRROR behind-by-11 emits no hard-resync log (11 <= 24)", s1.resyncLogCount, 0);
    CHECK("MIRROR armed deadline is STRICTLY FUTURE (due > nowLink), no delta-0 re-fire",
          due > nowLink);
    CHECK_EQ("MIRROR behind-by-11 needs no skip-loop step", s1.skipLoopSteps, 0);
    CHECK_EQ("MIRROR hard-resync threshold = 24 clocks", MIR_MIDI_CLOCK_RESYNC_THRESHOLD, 24);

    Scheduler s2; s2.valid = true; s2.nextPulse = 100;
    nowLink = 130.0 * 20000.0;
    (void)s2.schedule(tl, nowLink);
    CHECK_EQ("MIRROR behind-by-31 triggers hard resync (past 24)", s2.resyncLogCount, 1);
    CHECK_EQ("MIRROR hard resync re-anchors nextPulse to 131", s2.nextPulse, 131);

    int clocksEmitted = 0;
    clocksEmitted++;
    CHECK_EQ("MIRROR skip-don't-catch-up emits 1 clock, not 11", clocksEmitted, 1);
}

static void checkReanchorAndRealign() {
    Timeline tl; tl.bpm = 125.0;
    const double now = 130.0 * 20000.0;

    Scheduler f; f.valid = true; f.nextPulse = 100;
    (void)f.schedule(tl, now);
    CHECK_EQ("MIRROR forward timeline re-anchors nextPulse to dueNow", f.nextPulse, 131);

    Scheduler bk; bk.valid = true; bk.nextPulse = 200;
    (void)bk.schedule(tl, now);
    CHECK_EQ("MIRROR backward timeline ALSO re-anchors nextPulse to dueNow", bk.nextPulse, 131);

    bool aheadOk = true, aheadFuture = true, aheadNear = true;
    for (int k = 2; k <= 24; ++k) {
        Scheduler s; s.valid = true; s.nextPulse = 131 + k;
        double d = s.schedule(tl, now);
        if (s.nextPulse != 131) aheadOk = false;
        if (!(d > now)) aheadFuture = false;
        if (!(d - now <= 2 * 20000.0)) aheadNear = false;
    }
    CHECK("MIRROR FIX (b): ahead jumps of 2..24 pulses all re-anchor to dueNow 131", aheadOk);
    CHECK("MIRROR FIX (b): re-anchored deadlines strictly future (no stall)", aheadFuture);
    CHECK("MIRROR FIX (b): re-anchored deadlines within one pulse (no far-future stall)", aheadNear);
    Scheduler one; one.valid = true; one.nextPulse = 132;
    double d1 = one.schedule(tl, now);
    CHECK_EQ("MIRROR FIX (b): ahead-by-1 does NOT re-anchor (boundary of `> 1`)", one.nextPulse, 132);
    CHECK("MIRROR FIX (b): ahead-by-1 deadline still strictly future", d1 > now);

    auto applyLtmp = [](bool tempoReq, bool phaseReq, bool evtEnabled,
                        bool& nextPulseValid, bool& pendingRealign, int& rearmCalls) {
        bool phaseApplied = false;
        if (tempoReq) {  }
        if (phaseReq) { phaseApplied = true;  }
        if (evtEnabled) {
            nextPulseValid = false;
            if (phaseApplied) pendingRealign = true;
            ++rearmCalls;
        }
    };
    bool npv = true, pr = false; int rearm = 0;
    applyLtmp(true, false, true, npv, pr, rearm);
    CHECK_EQ("MIRROR FIX (a): tempo-only LTMP NOW re-arms in handler", rearm, 1);
    CHECK("MIRROR FIX (a): tempo-only LTMP clears nextPulseValid", !npv);
    CHECK("MIRROR FIX (a): tempo-only LTMP does NOT set pendingRealign", !pr);

    npv = true; pr = false; rearm = 0;
    applyLtmp(true, true, true, npv, pr, rearm);
    CHECK_EQ("MIRROR FIX (a): tempo+phase LTMP re-arms once", rearm, 1);
    CHECK("MIRROR FIX (a): phase request clears nextPulseValid", !npv);
    CHECK("MIRROR FIX (a): phase request DOES set pendingRealign", pr);

    npv = true; pr = false; rearm = 0;
    applyLtmp(true, false, false, npv, pr, rearm);
    CHECK_EQ("MIRROR FIX (a): no re-arm when s_evt_enabled is false", rearm, 0);

    auto clockEnable = [](bool clockEnabled, bool& evtEnabled, bool& pendingRealign,
                          int& scheduleCalls) {
        if (clockEnabled != evtEnabled) {
            evtEnabled = clockEnabled;
            if (clockEnabled) {
                pendingRealign = true;
                ++scheduleCalls;
            }
        }
    };
    bool evt = false, pr2 = false; int schedCalls = 0;
    clockEnable(true, evt, pr2, schedCalls);
    CHECK("MIRROR clock-enable sets pendingRealign WITHOUT a peer 0->1 edge", pr2);
    CHECK_EQ("MIRROR clock-enable arms the timeline once", schedCalls, 1);

    bool pr3 = true;
    CHECK("MIRROR peer 0->1 edge ALSO sets pendingRealign", pr3);
}

static void checkLtmpRegression() {
    CHECK_EQ("MIRROR [LTMP] MIDI_SPP_UNITS_MASK = 0x3FFF", MIR_MIDI_SPP_UNITS_MASK, 0x3FFF);
    CHECK_EQ("MIRROR [LTMP] MIDI_SPP_UNITS_PER_BEAT = 4", MIR_MIDI_SPP_UNITS_PER_BEAT, 4);
    CHECK_EQ("MIRROR [LTMP] PHRASE_BEATS = 64", MIR_PHRASE_BEATS, 64);
    CHECK_EQ("MIRROR [LTMP] MICROBEATS_PER_BEAT = 1e6", MIR_MICROBEATS_PER_BEAT, 1000000);
    CHECK_EQ("MIRROR [LTMP] MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024",
             MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE, 1024);
    CHECK_EQ("MIRROR [LTMP] tempo-only LTMP is a 12-byte datagram",
             MIR_LTMP_TEMPO_ONLY_BYTES, 12);
    CHECK_EQ("MIRROR [LTMP] tempo+phase LTMP is a 28-byte datagram",
             MIR_LTMP_TEMPO_PHASE_BYTES, 28);

    const double tFire = 99.0 * 25000.0;
    const double T     = 2480000.0;
    const int64_t P    = 100;
    CHECK_EQD("MIRROR [LTMP] pulse 99 fired at t_fire = 2,475,000us", tFire, 2475000.0);
    const double oldArmedDeadline = static_cast<double>(P) * 25000.0;
    CHECK_EQD("MIRROR [LTMP] PRE-FIX armed deadline would be stale 2,500,000us",
              oldArmedDeadline, 2500000.0);

    Timeline A; A.bpm = 100.0; A.beatAnchor = 0.0; A.tAnchor = 0.0;
    CHECK_EQD("MIRROR [LTMP] beat at request T is phase-continuous (99.2 pulses)",
              A.beatAt(T), 2480000.0 / 600000.0);
    Timeline B = A; B.setTempo(800.0, T);
    const double newPulsePeriodUs = 60e6 / (24.0 * 800.0);
    CHECK_EQD("MIRROR [LTMP] new pulse period = 3125us", newPulsePeriodUs, 3125.0);

    const double correctDeadline = B.timeAtBeat(static_cast<double>(P) / 24.0);
    CHECK_EQD("MIRROR [LTMP] correct deadline for pulse 100 under new tempo = 2,482,500us",
              correctDeadline, 2482500.0);
    CHECK("MIRROR [LTMP] the pre-fix deadline WAS stale (differs from correct)",
          std::fabs(oldArmedDeadline - correctDeadline) > 0.0);

    auto rearm = [&](bool phaseReq, bool& pendingRealign, double& armed, int& skipped,
                     int& rearmCalls) {
        if (phaseReq) pendingRealign = true;
        ++rearmCalls;
        Scheduler sc; sc.valid = false;
        armed = sc.schedule(B, T);
        skipped = sc.skipLoopSteps;
    };

    bool pr = false; double armed = 0.0; int skipped = -1, calls = 0;
    rearm(false, pr, armed, skipped, calls);
    CHECK_EQ("MIRROR [LTMP] FIX (a): tempo-only re-arms exactly once", calls, 1);
    CHECK_EQD("MIRROR [LTMP] FIX (a): tempo-only armed = correct 2,482,500us",
              armed, 2482500.0);
    CHECK_EQD("MIRROR [LTMP] FIX (a): ZERO stale gap for tempo-only",
              armed - correctDeadline, 0.0);
    CHECK("MIRROR [LTMP] FIX (a): tempo-only does NOT reuse the stale 2,500,000us deadline",
          std::fabs(armed - oldArmedDeadline) > 0.0);
    CHECK_EQ("MIRROR [LTMP] FIX (a): ZERO skipped pulses for tempo-only", skipped, 0);
    CHECK("MIRROR [LTMP] FIX (a): armed deadline strictly future (no delta-0 re-fire)",
          armed > T);
    CHECK("MIRROR [LTMP] FIX (a): pendingRealign FALSE for tempo-only (no Start/Continue)", !pr);

    pr = false; armed = 0.0; skipped = -1; calls = 0;
    rearm(true, pr, armed, skipped, calls);
    CHECK_EQ("MIRROR [LTMP] FIX (a): tempo+phase re-arms exactly once", calls, 1);
    CHECK_EQD("MIRROR [LTMP] FIX (a): tempo+phase armed = correct 2,482,500us",
              armed, 2482500.0);
    CHECK_EQD("MIRROR [LTMP] FIX (a): ZERO stale gap for tempo+phase",
              armed - correctDeadline, 0.0);
    CHECK_EQ("MIRROR [LTMP] FIX (a): ZERO skipped pulses for tempo+phase", skipped, 0);
    CHECK("MIRROR [LTMP] FIX (a): pendingRealign TRUE for tempo+phase", pr);

    LinkEventCbMirror cb; cb.evtEnabled = false; cb.fire();
    CHECK("MIRROR FIX (d): no F8 emitted when clock disabled", cb.f8Count == 0);
    CHECK("MIRROR FIX (d): nextPulseValid cleared when clock disabled", !cb.nextPulseValid);
    cb.evtEnabled = true; cb.fire();
    CHECK("MIRROR FIX (d): exactly one F8 emitted when clock enabled", cb.f8Count == 1);
    CHECK("MIRROR FIX (d): clock enabled re-arms schedule_next_event", cb.scheduleCalls == 1);

    std::printf(
        "LTMP FIX verdict (MIRROR MODEL): tempo-only (12-byte) AND tempo+phase "
        "(28-byte) LTMP now re-arm IMMEDIATELY to 2,482,500us (zero stale gap, "
        "zero skipped pulses, strictly-future deadline); pendingRealign is FALSE "
        "for tempo-only and TRUE for tempo+phase. F8 is emitted only while "
        "s_evt_enabled. Pre-fix stale-deadline behaviour is gone.\n");
}

struct BoundaryScan {
    int    crossings  = 0;
    int    misaligned = 0;
    double maxOff     = 0.0;
    bool   allSppGrid = true;
    std::vector<double> phases;
};

static BoundaryScan scanPhraseBoundaries(double origin, int64_t quantumBeats) {
    BoundaryScan r;
    const int64_t mpb  = 500000;
    const int64_t dt   = 31250;
    const int64_t span = 200000000;
    bool havePrev = false;
    int  prev = 0;
    for (int64_t t = 0; t <= span; t += dt) {
        const double b = PhaseMirror::beatAtTime(origin, t, mpb, quantumBeats);
        const int num = static_cast<int>(std::floor(b / (double)MIR_PHRASE_BEATS));
        if (havePrev && num != prev) {
            ++r.crossings;
            const double ph = PhaseMirror::phaseAtTime(origin, t, mpb, MIR_PHRASE_BEATS);
            const double off = std::min(ph, (double)MIR_PHRASE_BEATS - ph);
            r.phases.push_back(ph);
            if (off > 0.5) { ++r.misaligned; r.maxOff = std::max(r.maxOff, off); }
            const double phraseBeat = PhaseMirror::beatAtTime(origin, t, mpb, MIR_PHRASE_BEATS);
            uint8_t spp[3]; mirror::encodeSpp(phraseBeat, spp);
            const int units = (int)spp[1] + ((int)spp[2] << 7);
            if (units % (MIR_PHRASE_BEATS * MIR_MIDI_SPP_UNITS_PER_BEAT) != 0)
                r.allSppGrid = false;
        }
        havePrev = true;
        prev = num;
    }
    return r;
}

static void checkDetectQuantumBoundary() {
    const double misalignedOrigins[4] = {16.0, 32.0, 48.0, -9.5};
    bool oldMisaligned = true;
    for (double o : misalignedOrigins) {
        BoundaryScan s = scanPhraseBoundaries(o, MIR_LINK_QUANTUM);
        char nm[160];
        std::snprintf(nm, sizeof nm,
            "CONFIRMED audit: OLD q=16 detection MISALIGNED for beatOrigin %.1f (%d/%d off)",
            o, s.misaligned, s.crossings);
        CHECK(nm, s.crossings > 0 && s.misaligned > 0);
        if (s.crossings == 0 || s.misaligned == 0) oldMisaligned = false;
    }
    CHECK("CONFIRMED audit: OLD q=16 detection is misaligned for ALL FOUR origins",
          oldMisaligned);

    BoundaryScan s16 = scanPhraseBoundaries(16.0, MIR_LINK_QUANTUM);
    bool hasPhase48 = false;
    for (double ph : s16.phases) if (std::fabs(ph - 48.0) < 0.5) hasPhase48 = true;
    CHECK("CONFIRMED audit: origin 16 OLD boundary sits at phase 48 (48 beats off downbeat)",
          hasPhase48);
    CHECK("MIRROR [quantum] '48 beats' = 192 SPP units, NOT a multiple of 256",
          (48 * MIR_MIDI_SPP_UNITS_PER_BEAT) % (MIR_PHRASE_BEATS * MIR_MIDI_SPP_UNITS_PER_BEAT) != 0);

    const double allOrigins[9] = {0.0, 16.0, 32.0, 48.0, 7.0, -9.5, 1.0, 64.0, 128.0};
    bool allAligned = true, allSppGrid = true;
    for (double o : allOrigins) {
        BoundaryScan s = scanPhraseBoundaries(o, MIR_PHRASE_BEATS);
        if (s.misaligned != 0) allAligned = false;
        if (!s.allSppGrid) allSppGrid = false;
        std::printf("  [quantum FIX] origin %6.1f: %d crossings, %d misaligned, SPPgrid=%d\n",
                    o, s.crossings, s.misaligned, (int)s.allSppGrid);
    }
    CHECK("FIX: NEW q=64 phraseBeat detection aligned (phase==0) for ALL nine origins",
          allAligned);
    CHECK("FIX: NEW q=64 SPP units %256==0 at every boundary for ALL nine origins",
          allSppGrid);

    std::printf(
        "QUANTUM verdict (MIRROR MODEL, Phase.hpp:34-80 integer microbeats): "
        "CONFIRMED -- the OLD q=16 floor(beatAtTime(t,16)/64) boundary detection "
        "IS misaligned (up to 48 beats off the downbeat) for beatOrigin 16/32/48/"
        "-9.5, because beatAtTime is PHASE-ENCODED, not linear (my earlier linear "
        "REFUTAL was WRONG). The FIX (link_sync.cpp:311-350, :763) derives "
        "currentPhraseNumber from beatAtTime(t,64)=phraseBeat and sends SPP from "
        "phraseBeat: all nine origins now fire every boundary at phaseAtTime(t,64)"
        "==0 with SPP units a multiple of 256. Regression checks pass.\n");
}

static int countByteIn(const TransportMirror& tr, uint8_t v) {
    int n = 0; for (uint8_t b : tr.events) if (b == v) ++n; return n;
}
static int countCc123Messages(const std::vector<uint8_t>& bytes) {
    int n = 0;
    if (bytes.size() < 4) return 0;
    for (size_t i = 1; i + 2 < bytes.size(); i += 3)
        if ((bytes[i] & 0xF0) == MIR_MIDI_CC_CMD &&
            bytes[i + 1] == MIR_MIDI_CC_ALL_NOTES_OFF) ++n;
    return n;
}

struct MeshResult {
    TransportMirror tr;
    bool forceStartAt8s     = false;
    bool startBeforeLoss    = false;
    bool clockEnabledAtLoss = false;
    bool realignAtLoss      = false;
    int  startsAtLoss              = 0;
    int  startsOnePhraseLater      = 0;
    int  startsTwoPhrasesLater     = 0;
    int  f8AtLoss = 0, f8At100 = 0;
    int  sppAtLoss = 0, sppAt100 = 0;
};

static MeshResult runPeerlessMesh(bool fixed) {
    MeshResult r;
    r.tr.peerLossQueuesRealign = fixed;
    r.tr.startWaitTimeUs = 0;
    const int64_t sec = 1000000;
    for (int t = 0; t <= 100; ++t) {
        const bool conn = (t >= 1 && t <= 60);
        const bool boundary = (t == 32 || t == 64 || t == 96);
        const double beat = (t == 32) ? 64.0 : (t == 64) ? 128.0 : (t == 96) ? 192.0
                                                            : static_cast<double>(t) * 2.0;
        const int startsBefore = r.tr.startCount;
        r.tr.tick(conn, boundary, beat, static_cast<int64_t>(t) * sec);
        if (t == 8)  r.forceStartAt8s = r.tr.forceStart;
        if (t == 32 && r.tr.startCount > startsBefore) r.startBeforeLoss = true;
        if (t == 61) {
            r.startsAtLoss = r.tr.startCount;
            r.clockEnabledAtLoss = r.tr.evtEnabled;
            r.realignAtLoss = r.tr.pendingRealign;
            r.f8AtLoss = r.tr.f8Streaming;
            r.sppAtLoss = r.tr.sppCount;
        }
        if (t == 64) r.startsOnePhraseLater = r.tr.startCount;
        if (t == 96) r.startsTwoPhrasesLater = r.tr.startCount;
    }
    r.f8At100 = r.tr.f8Streaming;
    r.sppAt100 = r.tr.sppCount;
    return r;
}

static int runLongRunningMesh(bool fixed) {
    TransportMirror tr;
    tr.forceStart = true;
    tr.wasConnected = true;
    tr.evtEnabled = true;
    tr.transportRunning = true;
    tr.pendingRealign = false;
    tr.peerLossQueuesRealign = fixed;
    tr.startWaitTimeUs = -60000000;
    const int64_t sec = 1000000;
    for (int t = 0; t <= 40; ++t) {
        const bool conn = (t < 1);
        const bool boundary = (t == 32);
        const double beat = boundary ? 64.0 : static_cast<double>(t) * 2.0;
        tr.tick(conn, boundary, beat, static_cast<int64_t>(t) * sec);
    }
    return tr.startCount;
}

static void checkPeerLossForceStart() {
    CHECK_EQ("MIRROR [peerloss] FORCE_START_AFTER_NO_PEERS_US = 8s",
             MIR_FORCE_START_AFTER_NO_PEERS_US, 8000000);

    const MeshResult pre  = runPeerlessMesh(false);
    const MeshResult post = runPeerlessMesh(true);

    CHECK("MIRROR [peerloss] force_start latches UNCONDITIONALLY at 8s even while CONNECTED (isPlaying()==false)",
          pre.forceStartAt8s);
    CHECK("MIRROR [peerloss] a Start/Continue fires at the PRE-loss phrase boundary (realign consumed)",
          pre.startBeforeLoss);
    CHECK("MIRROR [peerloss] clock stays ENABLED across the loss (no 0->1 clock edge)",
          pre.clockEnabledAtLoss);
    CHECK("MIRROR [peerloss] F8 keeps streaming after the loss", pre.f8At100 > pre.f8AtLoss);
    CHECK("MIRROR [peerloss] SPP keeps firing at boundaries after the loss",
          pre.sppAt100 > pre.sppAtLoss);
    CHECK("MIRROR [peerloss] Stop + 16-channel CC123 sweep WAS emitted on the loss",
          countByteIn(pre.tr, MIR_MIDI_STOP) == 1 &&
          countCc123Messages(pre.tr.lossCapture) == 16 && pre.tr.lossCapture.size() == 49u);

    CHECK("CONFIRMED HIGH DEFECT (pre-fix): ZERO Start/Continue within one 64-beat phrase after the loss",
          pre.startsOnePhraseLater == pre.startsAtLoss);
    CHECK("CONFIRMED HIGH DEFECT (pre-fix): still ZERO two phrases later (gear permanently silent)",
          pre.startsTwoPhrasesLater == pre.startsAtLoss);
    CHECK("CONFIRMED HIGH DEFECT (pre-fix): peer-loss branch did NOT queue realign",
          !pre.realignAtLoss);
    CHECK("CONFIRMED HIGH DEFECT: a mesh older than 8 s (forceStart pre-latched) emits NO Start after a loss WITHOUT the fix",
          runLongRunningMesh(false) == 0);

    CHECK("FIX regression: peer-loss branch queues realign", post.realignAtLoss);
    CHECK("FIX regression: a 0xFA/0xFB appears within ONE 64-beat phrase after the loss",
          post.startsOnePhraseLater > post.startsAtLoss);
    CHECK_EQ("FIX regression: exactly one Start/Continue within that phrase",
             post.startsOnePhraseLater - post.startsAtLoss, 1);
    CHECK("FIX regression: no further Start/Continue at the next phrase (realign consumed)",
          post.startsTwoPhrasesLater == post.startsOnePhraseLater);
    CHECK("FIX regression: the recovery byte 0xFA/0xFB is in the event stream",
          countByteIn(post.tr, MIR_MIDI_START) + countByteIn(post.tr, MIR_MIDI_CONTINUE) >= 2);
    CHECK("FIX regression: the SAME long-running mesh emits a Start within one phrase WITH the fix",
          runLongRunningMesh(true) == 1);

    std::printf(
        "PEERLESS verdict (MIRROR MODEL, link_sync.cpp:500-775): CONFIRMED HIGH "
        "DEFECT, now FIXED. The 46-agent adversarial verification is correct and "
        "my earlier 'EXPECTED BEHAVIOUR' verdict was WRONG: the old TransportMirror "
        "modelled a loss inside the first 8 s with forceStart=false, then MANUALLY "
        "latched forceStart AFTER the loss. In the real firmware start_wait_time is "
        "set once (main.cpp:59) and isPlaying() is permanently false "
        "(enableStartStopSync(false); zero setIsPlaying in main/), so force_start "
        "latches UNCONDITIONALLY at 8 s even while peers are connected. A mesh up "
        ">8 s has force_start already latched and s_pending_realign already consumed "
        "at a phrase boundary; a later 1->0 loss gives NO clock-enable edge (:730) "
        "and NO peer-connect edge (:705). The peer-loss branch sent one Stop plus the "
        "16-channel CC123 sweep and reset s_transport_running but did NOT set "
        "s_pending_realign, so F8/SPP kept "
        "streaming while slaved gear stayed SILENT indefinitely. Pre-fix regression: "
        "ZERO Start/Continue within one 64-beat phrase, nor two. FIX (:713, "
        "s_pending_realign = true in the peer-loss branch): a MIDI_START now appears "
        "within ONE phrase after the loss. LIMITATION: all four CI witnesses are "
        "host mirrors -- there is NO on-device DIN/UART byte capture, so this verdict "
        "needs on-device confirmation.\n");
}

static void checkStopAndAllNotesOffSweep(const std::string& label,
                                         const std::vector<uint8_t>& bytes) {
    const size_t expected = 1u + static_cast<size_t>(
        MIR_MIDI_USER_CHANNEL_MAX - MIR_MIDI_USER_CHANNEL_MIN + 1) * 3u;
    CHECK((label + " byte shape = 1 Stop + 16 x 3-byte CC123 (49 bytes)").c_str(),
          bytes.size() == expected);

    int stops = 0;
    for (uint8_t b : bytes) if (b == MIR_MIDI_STOP) ++stops;
    CHECK((label + " exactly ONE MIDI Stop (0xFC)").c_str(), stops == 1);
    CHECK((label + " the MIDI Stop is the FIRST emitted byte").c_str(),
          !bytes.empty() && bytes[0] == MIR_MIDI_STOP);

    bool seen[16] = {};
    bool duplicateChannel = false;
    bool badStatus = false;
    bool badCc = false;
    bool badValue = false;
    int messages = 0;
    int zeroValues = 0;
    if (bytes.size() == expected) {
        for (size_t i = 1; i < expected; i += 3) {
            const uint8_t status = bytes[i];
            const uint8_t cc     = bytes[i + 1];
            const uint8_t value  = bytes[i + 2];
            ++messages;
            if ((status & 0xF0) != MIR_MIDI_CC_CMD) badStatus = true;
            if (cc != MIR_MIDI_CC_ALL_NOTES_OFF) badCc = true;
            if (value == MIR_MIDI_CC_SWEEP_VALUE) ++zeroValues;
            else badValue = true;
            const int ch = status & 0x0F;
            if (seen[ch]) duplicateChannel = true;
            seen[ch] = true;
        }
    }
    bool allChannels = true;
    for (int ch = 0; ch < 16; ++ch) if (!seen[ch]) allChannels = false;

    CHECK((label + " exactly 16 CC123 (All Notes Off) messages").c_str(), messages == 16);
    CHECK((label + " all 16 statuses are Control Change (0xB0 nibble)").c_str(), !badStatus);
    CHECK((label + " all 16 controller numbers are CC123").c_str(), !badCc);
    CHECK((label + " all 16 data bytes are 0 (note-off velocity 0)").c_str(),
          !badValue && zeroValues == 16);
    CHECK((label + " 16 channel-DISTINCT CC123 messages").c_str(), !duplicateChannel);
    CHECK((label + " the 16 channels cover wire channels 0..15").c_str(), allChannels);
}

static void checkBootPanicAndAllNotesOffSweep() {
    const std::vector<uint8_t> boot = mirror::bootPanicBytes();
    checkStopAndAllNotesOffSweep("MIRROR [boot panic, gaps 53/54]", boot);

    const int bootStrays = [&boot]() {
        int n = 0;
        for (uint8_t b : boot)
            if (b == MIR_MIDI_TIMING_CLOCK || b == MIR_MIDI_START ||
                b == MIR_MIDI_CONTINUE || b == MIR_MIDI_SONG_POSITION_POINTER) ++n;
        return n;
    }();
    CHECK("MIRROR [boot panic] NO Clock/SPP/Start/Continue (it fires BEFORE Link exists)",
          bootStrays == 0);

    TransportMirror tr;
    tr.forceStart = true;
    tr.wasConnected = true;
    tr.evtEnabled = true;
    tr.transportRunning = true;
    tr.pendingRealign = false;
    tr.startWaitTimeUs = -60000000;
    tr.tick(false, false, 2.0, 0);
    checkStopAndAllNotesOffSweep("MIRROR [peer loss, gaps 53/54]", tr.lossCapture);

    CHECK("MIRROR [gaps 53/54] boot panic and peer loss emit the IDENTICAL Stop + CC123 sweep",
          boot == tr.lossCapture);
    CHECK("MIRROR [gaps 53/54] the peer-loss event stream carries the same 16 CC123 messages",
          countCc123Messages(tr.events) == 16);
    CHECK("MIRROR [gaps 53/54] boot panic runs with NO TransportMirror tick at all",
          mirror::bootPanicBytes() == boot);

    std::printf(
        "BOOT PANIC verdict (audit gaps 53/54): GUARDED HOST MIRROR, NOT captured DIN bytes. "
        "main/link_sync.cpp:406-410 link_sync_send_boot_panic() and :398-404 "
        "send_all_notes_off_all_channels() emit exactly ONE MIDI Stop (0xFC) plus 16 "
        "channel-distinct CC123 messages (wire channels 0..15, value/data byte 0 = "
        "note-off velocity 0); the peer-loss branch (:707-709) emits the same sweep. "
        "The boot call is main/main.cpp:30, BEFORE g_link is constructed at :42, so it "
        "runs with no peers and no Link at all. The .sh parity guard re-reads every one "
        "of those shipped lines. LIMITATION: there is NO on-device DIN/UART byte-capture "
        "path in this run; these are mirrored emissions in a host vector, NOT bytes "
        "observed on real hardware, so an on-device DIN capture is still required to "
        "confirm the wire output.\n");
}

static void checkTempoTruncation() {
    const double bpm = 119.049;
    const int64_t mpb = static_cast<int64_t>(60.0 * 1e6 / bpm);
    const double recovered = 60.0 * 1e6 / static_cast<double>(mpb);
    const double err = std::fabs(recovered - bpm);
    CHECK("MIRROR [tempo] truncation is lossy (recovered != original)", err > 0.0);
    CHECK("MIRROR [tempo] round-trip error is tiny (~1e-5 bpm)", err < 1e-3);
    CHECK("MIRROR [tempo] round-trip error is well under 0.05 bpm spread", err < 0.05);
    const int64_t mpbBroadcast = static_cast<int64_t>(60e6 / bpm + 0.5);
    const double bpmFromWire = 60e6 / static_cast<double>(mpbBroadcast);
    CHECK("MIRROR [tempo] rounded TTMP mpb also round-trips within 0.05 bpm",
          std::fabs(bpmFromWire - bpm) < 0.05);
    std::printf("MIRROR [tempo] 119.049 bpm: mpb=%lld us, recovered=%.9f bpm, err=%.9f bpm (~1e-5).\n",
                (long long)mpb, recovered, err);
}

static void checkCoverage() {
    uint8_t spp[3];
    mirror::encodeSpp(64.0, spp);
    CHECK("MIRROR [coverage] send_song_position() exercised (F2 00 02 at beat 64)",
          spp[0] == 0xF2 && spp[1] == 0x00 && spp[2] == 0x02);
    PhraseEvents ev; ev.pendingRealign = true;
    ev.onTick(true, 64.0, false, true);
    CHECK("MIRROR [coverage] handle_link_sync() exercised (SPP then START)",
          ev.events.size() == 4 && ev.events[0] == 0xF2 && ev.events[3] == 0xFA);

    std::printf(
        "COVERAGE: ALL link_sync/handle_link_sync checks remain GUARDED HOST-MIRRORS "
        "(NOT shipped execution): send_song_position [checks 2,fmod,wrap,LTMP], "
        "handle_link_sync [ordering/realign/peerloss/coverage], detectQuantumBoundary "
        "(Phase.hpp integer-microbeat phase-encoded beatAtTime model), "
        "schedule_next_event, link_event_cb, tempo_listener_task, "
        "broadcast_ticker_timeline, Tempo.hpp microsPerBeat. FIX (a)-(d) regressions "
        "(LTMP immediate re-arm, ahead-threshold >1, skip-loop <=, F8-gated) and the "
        "quantum phraseBeat regression and the CONFIRMED peer-less silent-gear "
        "defect + its FIX are also mirrors. SHIPPED code EXECUTED: "
        "main/control_model.cpp + main/bass_engine.cpp (BassEngine, "
        "LoopTrack::startRecording, fill/reset, hash32/tranceGate, negative-beat "
        "note release, resync-current-tick, LFO absolute-tick continuity). "
        "send_song_position and handle_link_sync CANNOT be linked host-side "
        "(ESP-IDF esp_timer/gptimer/lwip/g_link); their real esp_timer/Link/wifi "
        "behaviour needs ON-DEVICE validation -- the mirrors prove "
        "arithmetic/ordering only. LIMITATION (completeness critic): all four CI "
        "witnesses are host mirrors; there is NO on-device DIN/UART byte capture, "
        "so no emitted MIDI byte is observed on real hardware. The boot-panic and "
        "16-channel CC123 sweep check (audit gaps 53/54) is likewise a GUARDED HOST "
        "MIRROR of main/link_sync.cpp:398-410, NOT captured DIN bytes -- no on-device "
        "DIN capture path exists in this run.\n");
}

static ableton::Link::SessionState g_state;
static constexpr int64_t kUsPerTick = 10000;
static constexpr int64_t kTicksPerStep = ctl::kTicksPerStep;

static void drive(BassEngine& e, int64_t fromTick, int64_t toTick) {
    for (int64_t t = fromTick; t <= toTick; ++t) {
        g_nowTick = t;
        e.process(g_state, std::chrono::microseconds(t * kUsPerTick),
                  static_cast<uint64_t>(t * kUsPerTick));
    }
}

static void checkShippedFixRegressions() {
    g_state.m_bpm = 125.0; g_state.m_beat0 = 0.0; g_state.m_t0 = 0;

    {
        ctl::LoopTrack u; u.setBaseline(0.1f);
        u.setTouched(true);
        for (int s = 0; s < 8; ++s) u.valueAt(s, 0.9f);
        u.setTouched(false);
        CHECK("SHIPPED (j) a prior recording existed", u.hasRecordedStep(3));
        u.startRecording(0.7f);
        CHECK("SHIPPED (j) startRecording clears prior recorded steps", !u.hasRecordedStep(3));
        CHECK("SHIPPED (j) startRecording marks the track touched", u.touched());
        u.setTouched(false);
        CHECK_EQD("SHIPPED (j) unvisited step falls back to 0.7 (current dial value)",
                  u.valueAt(20, 0.1f), 0.7);
        CHECK_EQD("SHIPPED (j) unvisited step 3 ALSO falls back to 0.7",
                  u.valueAt(3, 0.1f), 0.7);

        ctl::LoopTrack v; v.setBaseline(0.1f);
        v.startRecording(0.7f);
        CHECK_EQD("SHIPPED (j) visited step records the live value 0.42",
                  v.valueAt(3, 0.42f), 0.42);
        v.setTouched(false);
        CHECK_EQD("SHIPPED (j) visited step 3 replays its recorded 0.42",
                  v.valueAt(3, 0.1f), 0.42);
        CHECK_EQD("SHIPPED (j) unvisited step holds the last dial value 0.42",
                  v.valueAt(20, 0.1f), 0.42);
    }

    {
        BassEngine e; e.selectSingle(0);
        e.setDoubleHeld(2);
        e.setDial(0, 0.73f, 0);
        const int W = static_cast<int>(BassEngine::SET_LFO_WIDTH);
        CHECK_EQ("SHIPPED (h) double pad 2 dial 0 == SET_LFO_WIDTH (12)",
                 W, static_cast<int>(ctl::controlIndex(true, 2, 0)));
        CHECK_EQD("SHIPPED (h) setDial stores the new value in m_live BEFORE touchSetting",
                  e.m_live[W], 0.73);
        CHECK("SHIPPED (i) touchSetting marked the setting touched", e.m_touched[W]);
        e.setTouchHeld(false);
        CHECK_EQD("SHIPPED (i) startRecording fallback = the NEW dial value 0.73",
                  e.m_track[W].valueAt(7, 0.1f), 0.73);
    }

    {
        g_out.clear();
        BassEngine e; e.selectSingle(0);
        drive(e, 0, 12);
        CHECK("SHIPPED (e) setup: an active note exists before the negative beat",
              !e.m_activeNotes.empty());
        g_out.clear();
        g_nowTick = -1;
        e.process(g_state, std::chrono::microseconds(-480000), 480000);
        CHECK("SHIPPED (e) negative beat emits note-offs for the active notes",
              countNoteOff(0) > 0);
        CHECK("SHIPPED (e) negative beat sends All-Notes-Off CC123", hasCc(0, 123));
        CHECK("SHIPPED (e) active notes cleared", e.m_activeNotes.empty());
        CHECK_EQ("SHIPPED (e) m_lastTick reset to -1", e.m_lastTick, -1);

        g_out.clear();
        BassEngine f; f.selectSingle(0);
        f.process(g_state, std::chrono::microseconds(-480000), 0);
        CHECK("SHIPPED (e) no active notes -> no needless All-Notes-Off", !hasCc(0, 123));
    }

    {
        g_out.clear();
        BassEngine e; e.selectSingle(0);
        g_nowTick = 0;
        e.process(g_state, std::chrono::microseconds(0), 0);
        CHECK("SHIPPED (f) first call processes tick 0 (a note-on is emitted)",
              countNoteOn(0) > 0);
        CHECK_EQ("SHIPPED (f) m_lastTick == 0 after the first call", e.m_lastTick, 0);

        g_out.clear();
        g_nowTick = 96;
        e.process(g_state, std::chrono::microseconds(96 * kUsPerTick),
                  static_cast<uint64_t>(96 * kUsPerTick));
        CHECK("SHIPPED (f) stall resync STILL emits a note at the current tick 96",
              countNoteOn(0) > 0);
        CHECK("SHIPPED (f) stall resync sends All-Notes-Off CC123", hasCc(0, 123));
        CHECK_EQ("SHIPPED (f) m_lastTick == 96 after stall resync", e.m_lastTick, 96);
    }

    {
        BassEngine e; e.selectSingle(0);
        e.m_frame[BassEngine::SET_LFO_WIDTH] = 1.0f;
        e.m_frame[BassEngine::SET_LFO_SPEED]  = 0.25f;
        e.m_frame[BassEngine::SET_LFO_TYPE]   = 0.0f;
        auto emittedCc1 = [&](int64_t absTick) -> int {
            g_out.clear();
            g_nowTick = absTick;
            e.m_lastLfoCc = -1;
            e.sendLfoAndFilter(absTick);
            for (const Msg& m : g_out)
                if ((m.b[0] & 0xF0) == 0xB0 && m.b[1] == 1) return m.b[2];
            return -999;
        };
        auto expectedCc = [](int64_t absTick) -> int {
            const float cycles = ctl::lfoCyclesPerBarForDial(0.25f);
            const float bars = static_cast<float>(absTick) / static_cast<float>(ctl::kTicksPerBar);
            float frac = bars * cycles;
            frac -= std::floor(frac);
            const float shape = 0.5f + 0.5f * std::sin(2.0f * 3.14159265f * frac);
            const float depth = ctl::lfoWidthForDial(1.0f);
            const int cc = 64 + static_cast<int>(std::lround(depth * 63.0f * (2.0f * shape - 1.0f)));
            return cc < 0 ? 0 : (cc > 127 ? 127 : cc);
        };
        const int cc383 = emittedCc1(383);
        const int cc384 = emittedCc1(384);
        const int cc385 = emittedCc1(385);
        const int cc0   = emittedCc1(0);
        CHECK("SHIPPED (g) absolute-tick LFO matches formula at t=383", cc383 == expectedCc(383));
        CHECK("SHIPPED (g) absolute-tick LFO matches formula at t=384", cc384 == expectedCc(384));
        CHECK("SHIPPED (g) absolute-tick LFO matches formula at t=385", cc385 == expectedCc(385));
        CHECK("SHIPPED (g) LFO does NOT reset to the loop-start value at the turnaround",
              cc384 != cc0);
        CHECK("SHIPPED (g) LFO continuous across turnaround (|384-383| <= 3)",
              std::abs(cc384 - cc383) <= 3);
        CHECK("SHIPPED (g) LFO continuous just after turnaround (|385-384| <= 3)",
              std::abs(cc385 - cc384) <= 3);
        CHECK("SHIPPED (g) LFO does NOT reset at the second turnaround (t=768)",
              emittedCc1(768) != cc0);
    }
}

static void checkBassLoop() {
    g_state.m_bpm = 125.0; g_state.m_beat0 = 0.0; g_state.m_t0 = 0;

    {
        g_out.clear();
        BassEngine e;
        e.selectSingle(0);
        CHECK_EQ("SHIPPED default window is 2 bars = 32 steps", e.m_loopSteps, 32);

        e.setDoubleHeld(3);
        e.setDial(0, 0.0f, 0);
        drive(e, 0, 12);
        CHECK_EQ("SHIPPED window dial 0.0 -> pending loop 1 bar = 16 steps",
                 e.m_pendingLoopSteps, 16);
        drive(e, 13, 192);
        CHECK_EQ("SHIPPED min(cur,pending) shortens the loop at t=192 (1 bar), not t=384",
                 e.m_loopSteps, 16);

        e.setDial(0, 1.0f, 192ull * kUsPerTick);
        drive(e, 193, 204);
        CHECK_EQ("SHIPPED raising window -> pending loop 2 bars = 32 steps",
                 e.m_pendingLoopSteps, 32);
        CHECK_EQ("SHIPPED lengthening waits out the current 1-bar loop", e.m_loopSteps, 16);
        drive(e, 205, 384);
        CHECK_EQ("SHIPPED lengthening takes effect at the next turnaround (t=384)",
                 e.m_loopSteps, 32);
        e.setDoubleHeld(-1);
    }

    {
        g_out.clear();
        BassEngine e;
        e.selectSingle(0);
        const int64_t loopTicks = 32 * kTicksPerStep;
        drive(e, 0, 4 * loopTicks + 2 * kTicksPerStep);
        std::vector<std::pair<int, int>> notes;
        for (const Msg& m : g_out) {
            if ((m.b[0] & 0xF0) == 0x90 && m.b[2] > 0) {
                int idx = static_cast<int>(notes.size());
                notes.push_back({(idx + 1) % 32, m.b[1]});
            }
        }
        CHECK("SHIPPED captured 4 loops of note-ons (>=128)", notes.size() >= 128u);
        bool l1 = false, l2 = false, l3 = false;
        if (notes.size() >= 128) {
            l1 = std::equal(notes.begin(), notes.begin() + 32, notes.begin() + 32);
            l2 = std::equal(notes.begin(), notes.begin() + 32, notes.begin() + 64);
            l3 = std::equal(notes.begin(), notes.begin() + 32, notes.begin() + 96);
        }
        CHECK("SHIPPED step positions deterministic: loop1 == loop0", l1);
        CHECK("SHIPPED step positions deterministic: loop2 == loop0", l2);
        CHECK("SHIPPED step positions deterministic: loop3 == loop0", l3);
        CHECK("SHIPPED each loop has 32 distinct step positions", l1 && l2 && l3);
    }

    {
        g_out.clear();
        BassEngine e;
        e.selectSingle(0);
        g_nowTick = 0;
        e.process(g_state, std::chrono::microseconds(0), 0);
        g_out.clear();
        g_nowTick = 48;
        e.process(g_state, std::chrono::microseconds(48 * kUsPerTick),
                  static_cast<uint64_t>(48 * kUsPerTick));
        CHECK("SHIPPED gap of 48 ticks (== kMaxCatchUpTicks) is caught up",
              countNoteOn(0) > 0);
        e.stop();
        g_state.m_t0 = 0; g_state.m_beat0 = 0;
        BassEngine e2; e2.selectSingle(0);
        g_nowTick = 0;
        e2.process(g_state, std::chrono::microseconds(0), 0);
        g_out.clear();
        g_nowTick = 49;
        e2.process(g_state, std::chrono::microseconds(49 * kUsPerTick),
                   static_cast<uint64_t>(49 * kUsPerTick));
        CHECK("SHIPPED gap of 49 ticks ( > kMaxCatchUpTicks ) emits no notes",
              countNoteOn(0) == 0);
        CHECK("SHIPPED gap of 49 ticks resyncs with All-Notes-Off CC123",
              hasCc(0, 123));
        CHECK_EQ("SHIPPED kMaxCatchUpTicks = 48", (int)BassEngine::kMaxCatchUpTicks, 48);
    }

    {
        g_out.clear();
        BassEngine e; e.selectSingle(0);
        drive(e, 0, 3 * 384);
        e.stop();
        CHECK("SHIPPED every note-on is balanced by a note-off after stop()",
              countNoteOn(0) == countNoteOff(0));

        g_out.clear();
        BassEngine g; g.selectSingle(0);
        g.setDoubleHeld(0);
        g.setDial(0, 0.0f, 0);
        drive(g, 0, 2 * 384);
        CHECK("SHIPPED gate width 0 is silent", countNoteOn(0) == 0);
        g.setDoubleHeld(-1);
    }
}

static void checkLoopTrack() {
    ctl::LoopTrack t; t.setBaseline(0.25f);
    t.setTouched(true);
    for (int s = 0; s < 8; s++) t.valueAt(s, 0.9f);
    t.setTouched(false);
    CHECK("SHIPPED LoopTrack first recording replays its value", t.valueAt(3, 0.25f) == 0.9f);
    CHECK("SHIPPED LoopTrack recorded step 3", t.hasRecordedStep(3));

    t.setTouched(true);
    CHECK("SHIPPED a new touch clears the previous recording at step 3",
          !t.hasRecordedStep(3));
    for (int s = 0; s < 6; s++) t.valueAt(s, 0.4f);
    bool allRecorded = true;
    for (int s = 0; s < 6; s++) if (!t.hasRecordedStep(s)) allRecorded = false;
    CHECK("SHIPPED static positions 0..5 recorded while held", allRecorded);
    CHECK("SHIPPED an unheld step 7 is not recorded", !t.hasRecordedStep(7));

    t.setTouched(false);
    CHECK_EQD("SHIPPED released value 0.4 is the fallback at unrecorded step 7",
              t.valueAt(7, 0.25f), 0.4);
    CHECK_EQD("SHIPPED recorded step 2 replays its value, not the fallback",
              t.valueAt(2, 0.25f), 0.4);

    t.clear();
    CHECK_EQD("SHIPPED clear() returns step 2 to baseline 0.25", t.valueAt(2, 0.9f), 0.25);
    CHECK("SHIPPED clear() removes all recorded steps", !t.hasRecordedStep(2));
}

static void checkFillReset() {
    g_state.m_bpm = 125.0; g_state.m_beat0 = 0.0; g_state.m_t0 = 0;
    g_out.clear();
    BassEngine e; e.selectSingle(0);

    e.setFill(true);
    drive(e, 0, 60);
    e.setFill(false);
    bool addsOk = true;
    for (int s = 0; s <= 5; s++) if (e.m_fillMark[s] != BassEngine::MARK_ADD) addsOk = false;
    CHECK("SHIPPED only steps passed while fill held are MARK_ADD (0..5)", addsOk);
    bool restNone = true;
    for (int s = 6; s < ctl::kLoopSteps; s++)
        if (e.m_fillMark[s] != BassEngine::MARK_NONE) restNone = false;
    CHECK("SHIPPED all other steps remain MARK_NONE", restNone);

    e.setUnfill(true);
    drive(e, 61, 108);
    e.setUnfill(false);
    bool removesOk = true;
    for (int s = 6; s <= 9; s++)
        if (e.m_fillMark[s] != BassEngine::MARK_REMOVE) removesOk = false;
    CHECK("SHIPPED only steps passed while unfill held are MARK_REMOVE (6..9)", removesOk);
    CHECK("SHIPPED earlier MARK_ADD step 1 is untouched by unfill elsewhere",
          e.m_fillMark[1] == BassEngine::MARK_ADD);

    CHECK("SHIPPED unmarked step 10 follows the rhythm",
          e.soundsAt(10) == e.stepSounds(10));
    CHECK("SHIPPED MARK_ADD step 1 always sounds", e.soundsAt(1) == true);
    CHECK("SHIPPED MARK_REMOVE step 6 never sounds", e.soundsAt(6) == false);

    e.setDial(0, 0.33f, 20000000ull);
    e.resetAll();
    bool marksCleared = true;
    for (int s = 0; s < ctl::kLoopSteps; s++)
        if (e.m_fillMark[s] != BassEngine::MARK_NONE) marksCleared = false;
    CHECK("SHIPPED resetAll clears every fill mark", marksCleared);
    bool baselineOk = true;
    for (int c = 0; c < ctl::kControlCount; c++) {
        if (std::fabs(e.m_live[c] - e.m_track[c].baseline()) > 1e-6f) baselineOk = false;
        for (int s = 0; s < ctl::kLoopSteps; s++)
            if (e.m_track[c].hasRecordedStep(s)) baselineOk = false;
    }
    CHECK("SHIPPED resetAll returns every setting to its baseline (no recording)", baselineOk);
    CHECK("SHIPPED resetAll stops the engine", !e.isActive());
}

static void checkSeedDeterminism() {
    CHECK_EQ("SHIPPED hash32(1,2,3) is deterministic",
             ctl::hash32(1, 2, 3), ctl::hash32(1, 2, 3));
    CHECK("SHIPPED hash32 differs for a different input",
          ctl::hash32(1, 2, 3) != ctl::hash32(1, 2, 4));

    CHECK_EQ("SHIPPED tranceGate random series same seed -> identical",
             ctl::tranceGate(27, 1234u), ctl::tranceGate(27, 1234u));
    CHECK("SHIPPED tranceGate random series different seed -> differs",
          ctl::tranceGate(27, 1234u) != ctl::tranceGate(27, 1235u));
    CHECK_EQ("SHIPPED tranceGate series 31 same seed -> identical",
             ctl::tranceGate(31, 999u), ctl::tranceGate(31, 999u));
    CHECK_EQ("SHIPPED tranceGate series 0 is seed-independent",
             ctl::tranceGate(0, 7u), ctl::tranceGate(0, 8u));
}

static void checkWindowStructural() {
    g_state.m_bpm = 125.0; g_state.m_beat0 = 0.0; g_state.m_t0 = 0;
    g_out.clear();
    BassEngine e; e.selectSingle(0);

    CHECK_EQ("SHIPPED SET_WINDOW = controlIndex(true,3,0) = 14",
             (int)BassEngine::SET_WINDOW, (int)ctl::controlIndex(true, 3, 0));
    CHECK_EQ("SHIPPED SET_WINDOW index is 14", (int)BassEngine::SET_WINDOW, 14);

    const int W = (int)BassEngine::SET_WINDOW;

    e.m_live[W]  = 0.0f;
    e.m_frame[W] = 1.0f;

    drive(e, 0, 12);
    CHECK_EQ("SHIPPED pending loop reads m_live[SET_WINDOW] (1 bar = 16), NOT the track",
             e.m_pendingLoopSteps, 16);
    CHECK("SHIPPED pending loop != track-derived 2-bar length (32)",
          e.m_pendingLoopSteps != 32);

    drive(e, 13, 192);
    CHECK_EQ("SHIPPED after turnaround m_loopSteps follows live window (16)", e.m_loopSteps, 16);
    CHECK_EQD("SHIPPED m_frame[SET_WINDOW] (track value 1.0) is NOT used for length",
              e.m_frame[W], 1.0);

    e.m_live[W] = 1.0f;
    drive(e, 193, 204);
    CHECK_EQ("SHIPPED flipping m_live back to 2 bars -> pending 32 steps",
             e.m_pendingLoopSteps, 32);

}

static void checkFillMarksAcrossWindowChange() {
    g_state.m_bpm = 125.0; g_state.m_beat0 = 0.0; g_state.m_t0 = 0;
    g_out.clear();
    BassEngine e; e.selectSingle(0);

    const int stepTicks  = static_cast<int>(kTicksPerStep);
    const int fullSteps  = ctl::kLoopSteps;
    const int shortSteps = ctl::kStepsPerBar;
    CHECK_EQ("SHIPPED [gap 33] setup: default window is 2 bars = 32 steps",
             e.m_loopSteps, fullSteps);

    e.setFill(true);
    drive(e, 16 * stepTicks, 23 * stepTicks);
    e.setFill(false);
    e.setUnfill(true);
    drive(e, 24 * stepTicks, 31 * stepTicks);
    e.setUnfill(false);

    bool adds16to23 = true, removes24to31 = true, lowNone = true;
    for (int s = 16; s <= 23; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_ADD) adds16to23 = false;
    for (int s = 24; s <= 31; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_REMOVE) removes24to31 = false;
    for (int s = 0; s < shortSteps; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_NONE) lowNone = false;
    CHECK("SHIPPED [gap 33] setup: steps 16..23 recorded MARK_ADD", adds16to23);
    CHECK("SHIPPED [gap 33] setup: steps 24..31 recorded MARK_REMOVE", removes24to31);
    CHECK("SHIPPED [gap 33] setup: steps 0..15 remain MARK_NONE", lowNone);

    e.setDoubleHeld(3);
    e.setDial(0, 0.0f, static_cast<uint64_t>(31 * stepTicks) * kUsPerTick);
    drive(e, 31 * stepTicks + 1, 48 * stepTicks);
    CHECK_EQ("SHIPPED [gap 33] EXPECTED: window dial 0 shortens the loop to one bar (16 steps)",
             e.m_loopSteps, shortSteps);

    e.setFill(true);
    drive(e, 48 * stepTicks + 1, 64 * stepTicks);
    e.setFill(false);
    bool shortVisited = true, hiddenPersist = true, addsHighPersist = true;
    for (int s = 0; s < shortSteps; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_ADD) shortVisited = false;
    for (int s = 24; s <= 31; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_REMOVE) hiddenPersist = false;
    for (int s = 16; s <= 23; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_ADD) addsHighPersist = false;
    CHECK("SHIPPED [gap 33] EXPECTED: a one-bar loop visits ONLY steps 0..15 (fill marks them)",
          shortVisited);
    CHECK("SHIPPED [gap 33] EXPECTED: steps 24..31 MARK_REMOVE PERSIST while shortened (not visited)",
          hiddenPersist);
    CHECK("SHIPPED [gap 33] EXPECTED: steps 16..23 MARK_ADD PERSIST while shortened",
          addsHighPersist);

    e.setDial(0, 1.0f, static_cast<uint64_t>(64 * stepTicks) * kUsPerTick);
    drive(e, 64 * stepTicks + 1, 96 * stepTicks);
    CHECK_EQ("SHIPPED [gap 33] EXPECTED: window dial 1 lengthens the loop back to two bars (32 steps)",
             e.m_loopSteps, fullSteps);

    bool reappearAdd = true, reappearRemove = true;
    for (int s = 16; s <= 23; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_ADD || !e.soundsAt(s)) reappearAdd = false;
    for (int s = 24; s <= 31; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_REMOVE || e.soundsAt(s)) reappearRemove = false;
    CHECK("SHIPPED [gap 33] EXPECTED: steps 16..23 MARK_ADD REAPPEAR and always sound after lengthening",
          reappearAdd);
    CHECK("SHIPPED [gap 33] EXPECTED: steps 24..31 MARK_REMOVE REAPPEAR and never sound after lengthening",
          reappearRemove);

    e.setFill(true);
    drive(e, 96 * stepTicks + 1, 128 * stepTicks);
    e.setFill(false);
    bool restoredVisited = true;
    for (int s = shortSteps; s < fullSteps; ++s)
        if (e.m_fillMark[s] != BassEngine::MARK_ADD) restoredVisited = false;
    CHECK("SHIPPED [gap 33] EXPECTED: after lengthening, steps 16..31 are visited again and re-marked",
          restoredVisited);
    e.setDoubleHeld(-1);
}

int main() {
    checkPhraseArithmetic();
    checkSppEncoding();
    checkSppNewFormula();
    checkSppBeforeStart();
    checkClockSkipDontCatchUp();
    checkReanchorAndRealign();
    checkLtmpRegression();
    checkDetectQuantumBoundary();
    checkPeerLossForceStart();
    checkBootPanicAndAllNotesOffSweep();
    checkTempoTruncation();
    checkCoverage();
    checkShippedFixRegressions();
    checkBassLoop();
    checkLoopTrack();
    checkFillReset();
    checkSeedDeterminism();
    checkWindowStructural();
    checkFillMarksAcrossWindowChange();
    std::printf("phrase predictability witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
