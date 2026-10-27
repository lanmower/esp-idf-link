// Host witness for the bass loop sequencer (main/bass_engine.cpp). Build and run with
// tools/bass-loop-witness.sh. Each check states the property it witnesses.
#include "bass_engine.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(name, cond)                                              \
    do {                                                               \
        g_checks++;                                                    \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s\n", name);                            \
            g_fail++;                                                  \
        }                                                              \
    } while (0)

struct Msg {
    uint8_t b[3];
};

static std::vector<Msg> g_out;

void send_midi_message(const uint8_t* message, size_t size) {
    Msg m = {{0, 0, 0}};
    for (size_t i = 0; i < size && i < 3; i++) m.b[i] = message[i];
    g_out.push_back(m);
}

void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value) {
    (void)channel;
    Msg m = {{0xB0, cc_num, value}};
    g_out.push_back(m);
}

static int countNoteOn(size_t from) {
    int n = 0;
    for (size_t i = from; i < g_out.size(); i++) {
        if ((g_out[i].b[0] & 0xF0) == 0x90 && g_out[i].b[2] > 0) n++;
    }
    return n;
}

static int countNoteOff(size_t from) {
    int n = 0;
    for (size_t i = from; i < g_out.size(); i++) {
        if ((g_out[i].b[0] & 0xF0) == 0x80 || ((g_out[i].b[0] & 0xF0) == 0x90 && g_out[i].b[2] == 0)) n++;
    }
    return n;
}

static bool hasCc(size_t from, uint8_t cc) {
    for (size_t i = from; i < g_out.size(); i++) {
        if ((g_out[i].b[0] & 0xF0) == 0xB0 && g_out[i].b[1] == cc) return true;
    }
    return false;
}

static ableton::Link::SessionState g_state;
static constexpr int64_t kStepUs = 1000;                 // one simulated millisecond per call
static constexpr int64_t kBarUs  = 1655172;              // 4 beats at 145 bpm
static constexpr int kDialEvery  = 10;                   // keep a dial touched across the run

// Runs `bars` bars from `start`. If `dialIdx` >= 0, the dial is kept at `value` throughout.
static int64_t runBars(BassEngine& e, int64_t start, int bars, int dialIdx = -1, float value = 0.0f) {
    const int64_t end = start + static_cast<int64_t>(bars) * kBarUs;
    int n = 0;
    for (int64_t t = start; t <= end; t += kStepUs, n++) {
        if (dialIdx >= 0 && n % kDialEvery == 0) {
            e.setDial(dialIdx, value, static_cast<uint64_t>(t));
        }
        e.process(g_state, std::chrono::microseconds(t), static_cast<uint64_t>(t));
    }
    return end;
}

// Runs one bar from startUs and returns how many notes sounded in it. If dialIdx >= 0 the dial
// moves between dialFromUs and dialToUs, touching that setting for the recorder.
static int notesInBar(BassEngine& e, int64_t startUs, int dialIdx = -1, float value = 0.0f,
                      int64_t dialFromUs = 0, int64_t dialToUs = 0) {
    g_out.clear();
    const int64_t end = startUs + kBarUs;
    int n = 0;
    if (dialIdx >= 0) e.setTouchHeld(true);
    for (int64_t t = startUs; t < end; t += kStepUs, ++n) {
        const int64_t into = t - startUs;
        if (dialIdx >= 0 && into >= dialFromUs && into < dialToUs && n % kDialEvery == 0) {
            e.setDial(dialIdx, value, static_cast<uint64_t>(t));
        }
        e.process(g_state, std::chrono::microseconds(t), static_cast<uint64_t>(t));
    }
    if (dialIdx >= 0) e.setTouchHeld(false);
    return countNoteOn(0);
}

static void checkControlIndices() {
    CHECK("single pad 0 dial 0 is setting 0 (scale)", ctl::controlIndex(false, 0, 0) == 0);
    CHECK("double pad 3 dial 1 is setting 15 (aberration)", ctl::controlIndex(true, 3, 1) == 15);
    CHECK("double pad 1 dial 1 is setting 11 (random drop-out)", ctl::controlIndex(true, 1, 1) == 11);
}

static void checkEngineSounds() {
    g_state.m_bpm = 145.0;
    g_out.clear();
    BassEngine e;
    e.selectSingle(0);
    runBars(e, 0, 4);
    CHECK("a tap on a pad produces notes with the default settings", countNoteOn(0) > 0);
    CHECK("the first pass emits no note-off before any note ends", countNoteOff(0) <= countNoteOn(0));
    CHECK("filter CC74 is sent on the first step", hasCc(0, 74));
    CHECK("mod wheel CC1 is sent by the LFO", hasCc(0, 1));

    // Four bars at 145 bpm hold 64 sixteenths. The default rhythm is every sixteenth and the
    // default volume modulation is fully open, so the default is a note on every sixteenth.
    CHECK("the default plays a note on every sixteenth", countNoteOn(0) >= 60);

    e.stop();
    CHECK("stop balances every note-on with a note-off", countNoteOn(0) == countNoteOff(0));
    for (const Msg& m : g_out) {
        if ((m.b[0] & 0xF0) == 0x90) CHECK("note numbers stay in the MIDI range", m.b[1] <= 127);
    }
}

// The window dial shortens the loop. It must take effect within the new length, not after a full
// pass of the old one, or dropping to one bar still waits out the default eight bars (32 beats).
// Gate width is recorded, so a change released before the turnaround persists: the silence it
// leaves carries into the bars after it.
static void checkWindowShortensPromptly() {
    g_state.m_bpm = 145.0;
    BassEngine e;
    e.selectSingle(0);

    // Pad 3 double-tap dial 0 is the loop window; 0.0 is one bar.
    e.setDoubleHeld(3);
    e.setDial(0, 0.0f, 0);
    e.setDoubleHeld(-1);

    int64_t t = 0;
    const int baseline = notesInBar(e, t); t += kBarUs;
    CHECK("the sequencer sounds notes with the default settings", baseline > 0);

    // Pad 0 double-tap dial 0 is gate width, held at zero through the middle third of a bar.
    e.setDoubleHeld(0);
    const int muted = notesInBar(e, t, 0, 0.0f, kBarUs / 3, 2 * kBarUs / 3); t += kBarUs;
    e.setDoubleHeld(-1);
    CHECK("gate width held at zero silences the rest of the bar", muted < baseline);
    CHECK("a released gate change replays its recorded silence on the next pass", notesInBar(e, t) < baseline);
}

static void checkGateZeroIsSilent() {
    g_out.clear();
    BassEngine e;
    e.selectSingle(0);
    e.setDoubleHeld(0);
    // Double-tap pad 0, dial 0: gate width, held at zero.
    runBars(e, 0, 2, 0, 0.0f);
    CHECK("gate width held at zero plays nothing", countNoteOn(0) == 0);
    e.setDoubleHeld(-1);
}

static void checkDropOutIsSilent() {
    g_out.clear();
    BassEngine e;
    e.selectSingle(1);
    e.setDoubleHeld(1);
    // Double-tap pad 1, dial 1: random note drop-out, held at full.
    runBars(e, 0, 2, 1, 1.0f);
    CHECK("random drop-out at full removes every note", countNoteOn(0) == 0);
    e.setDoubleHeld(-1);
}

static void checkDoubleRoutesDials() {
    // With no pad double-held, dial 0 belongs to the active single pad. A tap on pad 2 selects
    // its settings, and the notes still sound.
    g_out.clear();
    BassEngine e;
    e.selectSingle(2);
    runBars(e, 0, 2, 0, 0.5f);
    CHECK("pad 2 single-tap settings still produce notes", countNoteOn(0) > 0);
}

int main() {
    checkControlIndices();
    checkEngineSounds();
    checkWindowShortensPromptly();
    checkGateZeroIsSilent();
    checkDropOutIsSilent();
    checkDoubleRoutesDials();
    std::printf("bass loop witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
