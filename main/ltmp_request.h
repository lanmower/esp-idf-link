#pragma once

#include <atomic>
#include <cstdint>

struct LtmpRequest {
    bool    tempoPending  = false;
    double  tempoBpm      = 0.0;
    bool    phasePending  = false;
    int64_t phaseBeat0Us  = 0;
    double  phaseQuantum  = 4.0;
};

class LtmpGate {
public:
    void lock() {
        while (flag_.test_and_set(std::memory_order_acquire)) {}
    }
    void unlock() { flag_.clear(std::memory_order_release); }

private:
    std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

inline void ltmpPublishTempo(LtmpRequest& r, LtmpGate& gate, double bpm) {
    gate.lock();
    r.tempoBpm = bpm;
    r.tempoPending = true;
    gate.unlock();
}

inline void ltmpPublishPhase(LtmpRequest& r, LtmpGate& gate, int64_t beat0Us, double quantum) {
    gate.lock();
    r.phaseBeat0Us = beat0Us;
    r.phaseQuantum = quantum;
    r.phasePending = true;
    gate.unlock();
}

inline void ltmpPublishRequest(LtmpRequest& r, LtmpGate& gate, double bpm, int64_t beat0Us, double quantum) {
    gate.lock();
    r.tempoBpm = bpm;
    r.tempoPending = true;
    r.phaseBeat0Us = beat0Us;
    r.phaseQuantum = quantum;
    r.phasePending = true;
    gate.unlock();
}

inline void ltmpPublish(LtmpRequest& r, LtmpGate& gate, bool hasTempo, double bpm,
                        bool hasPhase, int64_t beat0Us, double quantum) {
    gate.lock();
    if (hasTempo) {
        r.tempoBpm = bpm;
        r.tempoPending = true;
    }
    if (hasPhase) {
        r.phaseBeat0Us = beat0Us;
        r.phaseQuantum = quantum;
        r.phasePending = true;
    }
    gate.unlock();
}

inline LtmpRequest ltmpTakeSnapshot(LtmpRequest& r, LtmpGate& gate) {
    gate.lock();
    LtmpRequest snap = r;
    r.tempoPending = false;
    r.phasePending = false;
    gate.unlock();
    return snap;
}
