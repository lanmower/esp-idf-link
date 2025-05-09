#ifndef LINK_SYNC_H
#define LINK_SYNC_H

#include "main.h"

// Struct to hold quantum boundary information
struct QuantumInfo {
    double sessionBeat;           // Current beat in session timeline
    double phaseWithinQuantum;    // Phase within quantum (0.0 to LINK_QUANTUM)
    int currentQuantumNumber;     // Current quantum number (increments each quantum)
    bool crossedQuantumBoundary;  // True if we just crossed a quantum boundary
    double beatFraction;          // Fraction of the current beat (0.0 to 1.0)
    int beatInQuantum;            // Current beat within the quantum
};

// Function Prototypes
void init_link_timer(TaskHandle_t task_handle);
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                        int& lastTicks, int& length, int& lastBeat, int& currentBuzzerFreq, bool& was_playing,
                        const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

// Common function to detect quantum boundaries and calculate phase information
// This ensures all effects stay in phrase with Link
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time);

#endif // LINK_SYNC_H