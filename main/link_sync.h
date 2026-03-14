#ifndef LINK_SYNC_H
#define LINK_SYNC_H

#include "main.h"
#include "ableton/Link.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/rtc.h"

// --- Quantum Boundary Detection Info Struct ---
// Used for consistent phase calculations across all effect implementations
struct QuantumInfo {
    double sessionBeat;          // Current beat from Link session
    double phaseWithinQuantum;   // Phase position within the quantum (0.0 to LINK_QUANTUM)
    int currentQuantumNumber;    // Which quantum number we're in
    bool crossedQuantumBoundary; // True if we just crossed a quantum boundary
    int beatInQuantum;           // Current beat number within the quantum
    double beatFraction;         // Fraction of the current beat (0.0 to 1.0)
};

// Detect quantum boundary
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time);

// Initialize Link timer
void init_link_timer(TaskHandle_t task_handle);

// Main Link Sync Handler function
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                       int& lastTicks, int& length, int& lastBeat, int& currentBuzzerFreq, bool& was_playing,
                       const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

#endif // LINK_SYNC_H 