#pragma once

#include <cstdint>
#include <chrono>
#include <vector>
#include <ableton/Link.hpp>
#include "control_model.h"

// The bassline sequencer. An 8-bar loop of sixteenths, driven by the Link beat. Pads choose which
// settings the two dials control (single tap), and double-tap-and-hold switches the dials to that
// pad's double-tap settings. Every setting is recorded as the loop plays (see ctl::LoopTrack).
class BassEngine {
public:
    BassEngine();

    // A single tap on a pad selects its single-tap settings and starts the sequencer.
    void selectSingle(int pad);

    // Double-tap-and-hold: the dials control this pad's double-tap settings. -1 when released.
    void setDoubleHeld(int pad);
    int  doubleHeld() const { return m_doubleHeld; }
    int  activeSingle() const { return m_activeSingle; }

    // Dial position in [0, 1] at a timestamp. Moving a dial touches its setting for the recorder.
    void setDial(int dial, float v01, uint64_t nowUs);

    void process(const ableton::Link::SessionState& state,
                 const std::chrono::microseconds& time);

    void stop();

    // Three pads held together clear the recordings and the sounding notes. Two pads held together
    // turn fill on: sounding notes stay on and more keep sounding until a clear.
    void clearAll();
    void setFill(bool on);

    bool isActive() const { return m_active; }

private:
    enum Setting {
        SET_SCALE = 0,
        SET_WIDTH,
        SET_PITCH,
        SET_RHYTHM,
        SET_LFO_TYPE,
        SET_LFO_SPEED,
        SET_SERIES,
        SET_OCTAVE,
        SET_GATE,
        SET_ARP,
        SET_RAND_OCT,
        SET_RAND_DROP,
        SET_LFO_WIDTH,
        SET_FILTER,
        SET_WINDOW,
        SET_ABERRATION
    };

    struct ActiveNote {
        int     note;
        int64_t offTick;
    };

    static constexpr int64_t kMaxCatchUpTicks = 48;   // one beat; a longer gap is resynchronised
    static constexpr uint64_t kTouchHoldUs    = 250000;
    static constexpr int     kLfoEveryTicks   = 3;
    static constexpr int     kRootNote        = 33;   // A1
    static constexpr int     kNoteVelocity    = 100;
    static constexpr size_t  kMaxActiveNotes  = 24;

    int   settingFor(int dial) const;
    void  expireTouches(uint64_t nowUs);
    void  tick(int64_t t);
    void  stepBoundary(int step);
    void  arpEvent(int step, int64_t loopTick, int period, int64_t t);
    void  sendLfoAndFilter(int64_t loopTick);
    void  endDueNotes(int64_t t);
    void  allNotesOff();
    void  sendNoteOn(int note);
    void  sendNoteOff(int note);

    ctl::LoopTrack m_track[ctl::kControlCount];
    float    m_live[ctl::kControlCount]       = {};
    float    m_frame[ctl::kControlCount]      = {};
    uint64_t m_lastMoveUs[ctl::kControlCount] = {};
    bool     m_touched[ctl::kControlCount]    = {};
    uint32_t m_entrySeed[ctl::kControlCount]  = {};
    uint32_t m_seedCounter = 1;

    int      m_loopSteps        = ctl::kLoopSteps;
    int      m_pendingLoopSteps = ctl::kLoopSteps;
    int      m_activeSingle     = 0;
    int      m_doubleHeld       = -1;
    bool     m_active           = false;
    bool     m_fill             = false;
    int64_t  m_lastTick         = -1;
    int      m_lastFilterCc     = -1;
    int      m_lastLfoCc        = -1;
    std::vector<ActiveNote> m_activeNotes;
};

extern BassEngine g_bassEngine;
