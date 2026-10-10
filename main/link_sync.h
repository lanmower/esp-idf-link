#ifndef LINK_SYNC_H
#define LINK_SYNC_H

#include "main.h"
#include "ableton/Link.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/rtc.h"

struct QuantumInfo {
    double sessionBeat;
    double phaseWithinQuantum;
    int currentQuantumNumber;
    bool crossedQuantumBoundary;
    int beatInQuantum;
    double beatFraction;
    int currentPhraseNumber;
    bool crossedPhraseBoundary;
    double phaseWithinPhrase;
};

QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time);

void init_link_timer(TaskHandle_t task_handle);

void link_start_tempo_listener();

void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                       bool& was_playing,
                       const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

#endif
