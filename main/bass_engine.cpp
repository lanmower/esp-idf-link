#include "bass_engine.h"
#include "io_helpers.h"
#include "main.h"
#include <algorithm>
#include <cmath>

BassEngine g_bassEngine;

namespace {

constexpr uint8_t kNoteOnStatus  = 0x90;
constexpr uint8_t kNoteOffStatus = 0x80;
constexpr uint8_t kChannelOne    = 1;      // user channel, as send_midi_cc takes it
constexpr uint8_t kCcModWheel    = 1;
constexpr uint8_t kCcFilter      = 74;
constexpr uint8_t kCcAllNotesOff = 123;
constexpr int     kMaxArpOffsets = 16;
constexpr int64_t kTicksPerBeat  = 4 * ctl::kTicksPerStep;
constexpr float   kPi            = 3.14159265f;

int floorDiv(int a, int b) {
    return a >= 0 ? a / b : -((-a + b - 1) / b);
}

// Arp offsets in scale degrees. Width 0 is the held pitch; wider arps go higher, and past the
// halfway point they come back down through the lower degrees.
int arpOffsets(float width, int out[kMaxArpOffsets]) {
    const float w = ctl::widthForDial(width);
    const int span = static_cast<int>(std::lround(w * 4.0f));
    if (span <= 0) {
        out[0] = 0;
        return 1;
    }
    int n = 0;
    for (int d = 0; d <= span; ++d) out[n++] = d;
    if (w > 0.5f) {
        for (int d = span - 1; d >= -span; --d) out[n++] = d;
    }
    return n;
}

// LFO shape in [0, 1] at a fractional cycle position.
float lfoShape(int type, float frac) {
    switch (type) {
        case 1:  return frac < 0.5f ? 2.0f * frac : 2.0f - 2.0f * frac;   // triangle
        case 2:  return frac;                                              // saw
        case 3:  return frac < 0.5f ? 1.0f : 0.0f;                         // square
        default: return 0.5f + 0.5f * std::sin(2.0f * kPi * frac);         // sine
    }
}

}  // namespace

BassEngine::BassEngine() {
    // Baseline for each setting: the value it has when a loop starts.
    const float defaults[ctl::kControlCount] = {
        0.0f,   // scale
        0.0f,   // width
        0.5f,   // pitch offset (centre: the root)
        0.5f,   // rhythm
        0.0f,   // LFO type
        0.25f,  // LFO speed
        0.0625f,  // trance-gate series 2: three onsets, clear of the skipped sixteenths
        0.5f,   // octave (centre)
        0.5f,   // gate width
        0.1f,   // arp speed (half)
        0.0f,   // random octave
        0.0f,   // random drop-out
        0.5f,   // LFO width (centre: none)
        0.5f,   // filter shift (centre)
        1.0f,   // loop window (8 bars)
        0.0f,   // chromatic aberration
    };
    for (int c = 0; c < ctl::kControlCount; ++c) {
        m_track[c].setBaseline(defaults[c]);
        m_live[c]  = defaults[c];
        m_frame[c] = defaults[c];
        m_entrySeed[c] = 1u + static_cast<uint32_t>(c);
    }
    m_pendingLoopSteps = ctl::loopBarsForDial(defaults[SET_WINDOW]) * ctl::kStepsPerBar;
    m_loopSteps = m_pendingLoopSteps;
    m_activeNotes.reserve(32);
}

int BassEngine::settingFor(int dial) const {
    if (m_doubleHeld >= 0) return ctl::controlIndex(true, m_doubleHeld, dial);
    return ctl::controlIndex(false, m_activeSingle, dial);
}

void BassEngine::selectSingle(int pad) {
    if (pad < 0 || pad >= ctl::kPadCount) return;
    m_activeSingle = pad;
    m_active = true;
}

void BassEngine::setDoubleHeld(int pad) {
    m_doubleHeld = (pad >= 0 && pad < ctl::kPadCount) ? pad : -1;
    if (m_doubleHeld >= 0) m_active = true;
}

void BassEngine::setDial(int dial, float v01, uint64_t nowUs) {
    if (dial < 0 || dial > 1) return;
    const int c = settingFor(dial);
    if (!m_touched[c]) {
        // A new change: its random patterns are seeded afresh.
        m_touched[c] = true;
        m_track[c].setTouched(true);
        m_entrySeed[c] = ctl::hash32(m_seedCounter++, static_cast<uint32_t>(c), 0x51u) | 1u;
    }
    m_live[c] = std::clamp(v01, 0.0f, 1.0f);
    m_lastMoveUs[c] = nowUs;
}

void BassEngine::expireTouches(uint64_t nowUs) {
    for (int c = 0; c < ctl::kControlCount; ++c) {
        if (m_touched[c] && nowUs - m_lastMoveUs[c] > kTouchHoldUs) {
            m_touched[c] = false;
            m_track[c].setTouched(false);
        }
    }
}

void BassEngine::process(const ableton::Link::SessionState& state,
                         const std::chrono::microseconds& time) {
    expireTouches(static_cast<uint64_t>(time.count()));
    if (!m_active) return;

    const double beat = state.beatAtTime(time, LINK_QUANTUM);
    if (beat < 0.0) return;
    const int64_t now = static_cast<int64_t>(std::floor(beat * static_cast<double>(kTicksPerBeat)));

    if (m_lastTick < 0 || now < m_lastTick || now - m_lastTick > kMaxCatchUpTicks) {
        // First call, a stall or a backward jump: resynchronise without catching up.
        allNotesOff();
        m_lastTick = now;
        return;
    }
    for (int64_t t = m_lastTick + 1; t <= now; ++t) tick(t);
    m_lastTick = now;
}

void BassEngine::tick(int64_t t) {
    if (t % (static_cast<int64_t>(m_loopSteps) * ctl::kTicksPerStep) == 0) {
        for (int c = 0; c < ctl::kControlCount; ++c) m_track[c].turnaround();
        m_loopSteps = m_pendingLoopSteps;
    }
    const int64_t loopTicks = static_cast<int64_t>(m_loopSteps) * ctl::kTicksPerStep;
    const int64_t p = t % loopTicks;

    if (p % ctl::kTicksPerStep == 0) {
        stepBoundary(static_cast<int>(p / ctl::kTicksPerStep));
    }
    endDueNotes(t);

    const int period = ctl::arpPeriodTicks(ctl::arpSpeedIndexForDial(m_frame[SET_ARP]));
    if (p % period == 0) {
        arpEvent(static_cast<int>(p / ctl::kTicksPerStep), p, period, t);
    }
    if (p % kLfoEveryTicks == 0) {
        sendLfoAndFilter(p);
    }
}

void BassEngine::stepBoundary(int step) {
    for (int c = 0; c < ctl::kControlCount; ++c) {
        m_frame[c] = m_track[c].valueAt(step, m_live[c]);
    }
    if (step == 0) {
        // The loop window is applied at the next turnaround, so a change never cuts a pass short.
        m_pendingLoopSteps = ctl::loopBarsForDial(m_frame[SET_WINDOW]) * ctl::kStepsPerBar;
    }
    const float shift = ctl::filterShiftForDial(m_frame[SET_FILTER]);
    const int cc = std::clamp(64 + static_cast<int>(std::lround(shift * 64.0f)), 0, 127);
    if (cc != m_lastFilterCc) {
        send_midi_cc(kChannelOne, kCcFilter, static_cast<uint8_t>(cc));
        m_lastFilterCc = cc;
    }
}

void BassEngine::arpEvent(int step, int64_t loopTick, int period, int64_t t) {
    // Onset from the rhythm pattern.
    const uint32_t rhythm = ctl::stepPattern(ctl::rhythmForDial(m_frame[SET_RHYTHM]));
    if (!((rhythm >> (step % 32)) & 1u)) return;

    const float gate = ctl::gateWidthForDial(m_frame[SET_GATE]);
    if (gate <= 0.0f) return;

    // Volume modulation: the trance-gate series, and the fake sidechain that skips the first
    // sixteenth of every beat.
    const uint32_t series = ctl::tranceGate(ctl::gateSeriesForDial(m_frame[SET_SERIES]),
                                            m_entrySeed[SET_SERIES]);
    if (!((series >> (step % 32)) & 1u)) return;
    if (step % 4 == 0) return;

    // Pitch from the arp, around the selected pitch.
    const int64_t k = loopTick / period;
    int offsets[kMaxArpOffsets];
    const int n = arpOffsets(m_frame[SET_WIDTH], offsets);
    const int offset = offsets[k % n];

    const int scale = ctl::scaleForDial(m_frame[SET_SCALE]);
    const int size = ctl::scaleSize(scale);
    const int degree = static_cast<int>(std::lround(ctl::pitchOctaveForDial(m_frame[SET_PITCH]) *
                                                    static_cast<float>(size))) + offset;
    const int octaveOfDegree = floorDiv(degree, size);
    const int index = degree - octaveOfDegree * size;
    int note = kRootNote + 12 * octaveOfDegree + ctl::scaleSemitone(scale, index);
    note += 12 * ctl::octaveForDial(m_frame[SET_OCTAVE]);

    // Random octave, drop-out and aberration: deterministic for each position and entry seed.
    const uint32_t s  = static_cast<uint32_t>(step);
    const uint32_t kk = static_cast<uint32_t>(k);
    if (ctl::hashUnit(m_entrySeed[SET_RAND_OCT], s, kk) < ctl::probabilityForDial(m_frame[SET_RAND_OCT])) {
        note += (ctl::hash32(m_entrySeed[SET_RAND_OCT], s + 101u, kk) & 1u) ? 12 : -12;
    }
    if (ctl::hashUnit(m_entrySeed[SET_RAND_DROP], s, kk) < ctl::probabilityForDial(m_frame[SET_RAND_DROP])) {
        return;
    }
    if (ctl::hashUnit(m_entrySeed[SET_ABERRATION], s, kk) < ctl::aberrationForDial(m_frame[SET_ABERRATION])) {
        const uint32_t h = ctl::hash32(m_entrySeed[SET_ABERRATION], s + 211u, kk);
        const int magnitude = 1 + static_cast<int>(h % 3u);
        note += (h & 0x100u) ? magnitude : -magnitude;
    }
    note = std::clamp(note, 0, 127);

    const int length = std::max(1, static_cast<int>(std::lround(static_cast<float>(period) * gate)));
    if (m_activeNotes.size() >= kMaxActiveNotes) {
        sendNoteOff(m_activeNotes.front().note);
        m_activeNotes.erase(m_activeNotes.begin());
    }
    sendNoteOn(note);
    m_activeNotes.push_back({note, m_fill ? t + 1000000000LL : t + length});
}

void BassEngine::sendLfoAndFilter(int64_t loopTick) {
    const float cycles = ctl::lfoCyclesPerBarForDial(m_frame[SET_LFO_SPEED]);
    const float bars = static_cast<float>(loopTick) / static_cast<float>(ctl::kTicksPerBar);
    float frac = bars * cycles;
    frac -= std::floor(frac);
    const float shape = lfoShape(ctl::lfoTypeForDial(m_frame[SET_LFO_TYPE]), frac);
    const float depth = ctl::lfoWidthForDial(m_frame[SET_LFO_WIDTH]);
    const int cc = std::clamp(64 + static_cast<int>(std::lround(depth * 63.0f * (2.0f * shape - 1.0f))),
                              0, 127);
    if (cc != m_lastLfoCc) {
        send_midi_cc(kChannelOne, kCcModWheel, static_cast<uint8_t>(cc));
        m_lastLfoCc = cc;
    }
}

void BassEngine::endDueNotes(int64_t t) {
    if (m_fill) return;   // filled notes stay on until a clear
    for (auto it = m_activeNotes.begin(); it != m_activeNotes.end();) {
        if (it->offTick <= t) {
            sendNoteOff(it->note);
            it = m_activeNotes.erase(it);
        } else {
            ++it;
        }
    }
}

void BassEngine::stop() {
    m_active = false;
    m_lastTick = -1;
    m_lastFilterCc = -1;
    m_lastLfoCc = -1;
    allNotesOff();
}

void BassEngine::clearAll() {
    for (int c = 0; c < ctl::kControlCount; ++c) m_track[c].clear();
    m_fill = false;
    allNotesOff();
}

void BassEngine::setFill(bool on) {
    m_fill = on;
}

void BassEngine::allNotesOff() {
    for (const ActiveNote& n : m_activeNotes) sendNoteOff(n.note);
    m_activeNotes.clear();
    send_midi_cc(kChannelOne, kCcAllNotesOff, 0);
}

void BassEngine::sendNoteOn(int note) {
    const uint8_t msg[3] = {kNoteOnStatus, static_cast<uint8_t>(note), 100};
    send_midi_message(msg, 3);
}

void BassEngine::sendNoteOff(int note) {
    const uint8_t msg[3] = {kNoteOffStatus, static_cast<uint8_t>(note), 0};
    send_midi_message(msg, 3);
}
