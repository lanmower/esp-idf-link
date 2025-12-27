#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <chrono> // For std::chrono::microseconds
#include <ableton/Link.hpp> // Ableton Link header

// Include headers defining the types needed for extern declarations
#include "effect_handler.h" // Defines ControlContext
#include "effect_arp.h"     // Defines ArpMode

#include "main.h" // For NUM_TOUCH_PADS, NUM_POTS, EffectMode, ControlContext etc.
                 // This should bring in the definitions for ControlContext and ArpMode

// Structure to hold the state of all inputs at a given tick
struct InputEvent {
    uint64_t timestamp_us;
    bool pad_pressed_this_tick[NUM_TOUCH_PADS];
    bool pad_held[NUM_TOUCH_PADS];
    bool pot_moved[NUM_POTS];
    int pot_value[NUM_POTS];
    int pot_delta[NUM_POTS];
    // Add Link state info needed for transitions?
    // double current_beat; // Example
};

// --- Extern State Variables (Defined in effect_handler.cpp) ---
extern ControlContext g_current_control_context;
extern ControlContext g_last_pot_control_context;
extern bool g_arp_latched;
extern bool g_filter_latched;
extern bool g_sidechain_latched;
extern int g_pad_index_being_held_for_adjust;
extern uint64_t g_adjust_hold_press_time_us;
extern bool g_interaction_during_adjust_hold;

// Arp Specific State (Used for double-tap mode switch)
extern ArpMode g_current_arp_mode;
extern uint64_t g_arp_pad_last_tap_time_us; // Used for double-tap detection

// Delay/Reverb specific state (used for double-tap/hold)
extern EffectType g_last_pad2_effect;

// Synth Switching State
extern uint64_t g_all_pads_held_start;
extern bool g_all_pads_were_held;


// --- Main State Processing Function ---
// Processes the current input event and Link state to update the system state
// and trigger appropriate actions (MIDI messages, effect logic).
void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time);

#endif // STATE_MACHINE_H