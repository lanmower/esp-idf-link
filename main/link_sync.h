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
    // Phrase boundary (16 bars / PHRASE_BEATS) -- the transport-correction boundary,
    // distinct from the musical quantum above.
    int currentPhraseNumber;     // Which PHRASE_BEATS-length phrase we're in
    bool crossedPhraseBoundary;  // True if we just crossed a phrase boundary
    double phaseWithinPhrase;    // Phase position within the phrase (0.0 to PHRASE_BEATS)
};

// Detect quantum boundary
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time);

// Initialize Link timer
void init_link_timer(TaskHandle_t task_handle);

// Start the looper tempo-set (LTMP) multicast listener task. Call after g_link is
// created and WiFi is up.
void link_start_tempo_listener();

// Main Link Sync Handler function
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                       int& lastTicks, int& length, int& lastBeat, int& currentBuzzerFreq, bool& was_playing,
                       const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

#endif // LINK_SYNC_H 