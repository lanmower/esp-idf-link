#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <chrono>
#include <ableton/Link.hpp>
#include "main.h"  // NUM_TOUCH_PADS, NUM_POTS

// All input state for a single tick
struct InputEvent {
    uint64_t timestamp_us;
    bool pad_pressed_this_tick[NUM_TOUCH_PADS];
    bool pad_held[NUM_TOUCH_PADS];
    bool pot_moved[NUM_POTS];
    int  pot_value[NUM_POTS];
    int  pot_delta[NUM_POTS];
};

// Main tick handler
void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time);

#endif // STATE_MACHINE_H
