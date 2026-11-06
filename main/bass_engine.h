#pragma once

#include <cstdint>
#include <chrono>
#include <vector>
#include <ableton/Link.hpp>
#include "control_model.h"

class BassEngine {
public:
    BassEngine();

    void selectSingle(int pad);

    void setDoubleHeld(int pad);
    int  doubleHeld() const { return m_doubleHeld; }
    int  activeSingle() const { return m_activeSingle; }

    void setTouchHeld(bool held);

    void setDial(int dial, float v01, uint64_t nowUs);

    void process(const ableton::Link::SessionState& state,
                 const std::chrono::microseconds& time,
                 uint64_t nowUs);

    void stop();

    void setFill(bool on);
    void setUnfill(bool on);
    void clearFill();
    void resetAll();

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

    static constexpr int64_t kMaxCatchUpTicks = 48;
    static constexpr uint64_t kTouchHoldUs    = 250000;
    static constexpr int     kLfoEveryTicks   = 3;
    static constexpr int     kRootNoteA1      = 33;
    static constexpr int     kNoteVelocity    = 100;
    static constexpr size_t  kMaxActiveNotes  = 24;

    int   settingFor(int dial) const;
    void  expireTouches(uint64_t nowUs);
    void  tick(int64_t t);
    void  stepBoundary(int step);
    void  arpEvent(int step, int64_t loopTick, int period, int64_t t);
    void  sendLfoAndFilter(int64_t absTick);
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
    bool     m_unfill           = false;
    enum FillMark : uint8_t { MARK_NONE, MARK_ADD, MARK_REMOVE };
    FillMark m_fillMark[ctl::kLoopSteps] = {};
    bool     stepSounds(int step) const;
    bool     soundsAt(int step) const;
    void     touchSetting(int c);
    bool     m_touchHeld = false;
    void     untouchSetting(int c);
    void     updateRecording();
    int64_t  m_lastTick         = -1;
    int      m_lastFilterCc     = -1;
    int      m_lastLfoCc        = -1;
    std::vector<ActiveNote> m_activeNotes;
};

extern BassEngine g_bassEngine;
