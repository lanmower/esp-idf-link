#include "effect_handler.h"
#include <array> // Include for std::array
#include <cmath> // For std::abs, std::max, std::min
#include "touch_handler.h" // Include for touch pad interaction functions
#include "esp_log.h"
#include "link_sync.h"
#include "io_helpers.h"
#include "synth_interface.h" // Include the synth interface
#include "midi_helpers.h" // Added for scaling functions
#include "main.h"
#include "sidechain_constants.h" // Include for sidechain constants
#include "arp_constants.h" // Include for ARP constants (and MAX_ARP_INDEX_WRAP)
#include "lfo_constants.h" // Needed for LFO parameters
#include "state_machine.h" // Include state machine for extern variable declarations

// Include headers for individual effect handlers
#include "effect_arp.h"
#include "effect_filter.h" // Include for filter constants and g_filter_lfo_patched
#include "effect_sidechain.h"
#include "input_handler.h" // For read_inputs function declaration
#include "synth_mininova.h" // Include synth headers for switching
#include "synth_microkorg.h"

static const char *TAG_EFFECT = "EFFECT_HANDLER";

// REMOVED global state variables now defined in state_machine.cpp
// or moved to effect_filter.cpp (g_filter_lfo_patched)

// Keep synth-related globals defined in main.cpp
extern SynthInterface* g_current_synth; // Defined in main.cpp
extern SynthType g_synth_type; // Need access to global synth type declared in main.h

// --- State Variables Adjusted by Pots (Definitions Kept Here) ---
// Declared extern in effect_handler.h
int s_global_filter_cutoff = 127;
int s_global_filter_resonance = 0;
int s_current_sidechain_depth = 100;
int s_current_sidechain_sheer = 0;
int s_current_delay_time = 64;
int s_current_delay_feedback = 64;
int s_current_reverb_decay = 64;
int s_current_reverb_damping = 64;

// REMOVED pad_map_effect definition
// const int pad_map_effect[NUM_TOUCH_PADS] = { ... };

// --- Initialization --- (Keep, resets pot-controlled vars)
void initialize_effects() {
    // Reset latches and state machine vars (These are globals defined in state_machine.cpp)
    g_arp_latched = false;
    g_filter_latched = true; // Filter active by default
    g_sidechain_latched = false;
    g_current_control_context = ControlContext::NONE;
    g_last_pot_control_context = ControlContext::FILTER_ADJUST; // Reset pot context too
    g_pad_index_being_held_for_adjust = -1;
    g_adjust_hold_press_time_us = 0;
    g_interaction_during_adjust_hold = false;
    g_arp_pad_last_tap_time_us = 0; // Reset arp double tap timer
    g_current_arp_mode = ARP_MODE_NOTE; // Reset arp mode
    g_last_pad2_effect = EFFECT_DELAY; // Reset pad 2 effect tracking

    // Initialize Pot-controlled parameters (defined here)
    s_global_filter_cutoff = 127;
    s_global_filter_resonance = 0;
    s_current_sidechain_depth = 100;
    s_current_sidechain_sheer = 0;
    s_current_delay_time = 64;
    s_current_delay_feedback = 64;
    s_current_reverb_decay = 64;
    s_current_reverb_damping = 64;

    // Reset Filter LFO patch state (defined in effect_filter.cpp)
    initialize_filter(); // Call filter init which resets g_filter_lfo_patched

    ESP_LOGI(TAG_EFFECT, "Effects initialized. Default Mode: FILTER, Control Context: NONE");
}

// --- Internal Helper: Find Secondary Tapped Pad --- (Keep)
int find_secondary_tapped_pad(int primary_index, const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used) {
    for (int j = 0; j < 4; ++j) {
        if (j != primary_index && pad_pressed_this_tick[j]) {
            pads_used[j] = true; // Mark as used by the context
            return j; // Return the index of the tapped pad
        }
    }
    return -1; // No secondary pad tapped
}

// --- Pot Adjustment Handler Implementations (Keep) ---

// Sidechain Adjust Pot Handler
void handle_sidechain_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;
    // Pot 1: Controls the depth of ducking (0 = no ducking, 127 = full ducking)
    _update_pot_param(s_current_sidechain_depth, pot1_delta, 0, 127, "SC Duck Depth", pot1_used);
    // Pot 2: Controls the sheer/slope of the ducking transition
    _update_pot_param(s_current_sidechain_sheer, pot2_delta, 0, 127, "SC Sheer", pot2_used);
}

// Filter Adjust Pot Handler
void handle_filter_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;

    // Pot 1: Controls filter cutoff frequency
    if (_update_pot_param(s_global_filter_cutoff, pot1_delta, 0, 127, "Filter Cutoff", pot1_used)) {
        // Only send MIDI if LFO is NOT patched, otherwise handle_filter_active handles it
        if (!g_filter_lfo_patched) {
            g_current_synth->setFilterCutoff(s_global_filter_cutoff);
            ESP_LOGD(TAG_EFFECT, "Filter Cutoff adjusted: %d", s_global_filter_cutoff);
        }
    }

    // Pot 2: Controls filter resonance
    if (_update_pot_param(s_global_filter_resonance, pot2_delta, 0, 127, "Filter Resonance", pot2_used)) {
        g_current_synth->setFilterResonance(s_global_filter_resonance);
        ESP_LOGD(TAG_EFFECT, "Filter Resonance adjusted: %d", s_global_filter_resonance);
    }
}

// Delay Pots Handler
void handle_delay_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;
    if(_update_pot_param(s_current_delay_time, pot1_delta, 0, 127, "Delay Time", pot1_used)) {
        g_current_synth->setDelayTime(s_current_delay_time);
    }
    if(_update_pot_param(s_current_delay_feedback, pot2_delta, 0, 127, "Delay Feedback", pot2_used)) {
        g_current_synth->setDelayFeedback(s_current_delay_feedback);
    }
}

// Reverb Pots Handler
void handle_reverb_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;
     if(_update_pot_param(s_current_reverb_decay, pot1_delta, 0, 127, "Reverb Decay", pot1_used)) {
         g_current_synth->setReverbDecay(s_current_reverb_decay);
     }
    if(_update_pot_param(s_current_reverb_damping, pot2_delta, 0, 127, "Reverb Damping", pot2_used)) {
        g_current_synth->setReverbDamping(s_current_reverb_damping);
    }
}

// Arp Pots Handler defined in effect_arp.cpp


// --- Default Pot Handler (Keep) ---
void handle_default_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    ESP_LOGV(TAG_EFFECT, "Handle Default Pots (Context NONE) - Last Pot Context: %d", (int)g_last_pot_control_context);
    switch(g_last_pot_control_context) {
        case ControlContext::ARP_ADJUST:
            handle_arp_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::SIDECHAIN_ADJUST:
            handle_sidechain_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::DELAY_ADJUST:
            handle_delay_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::REVERB_ADJUST:
            handle_reverb_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::FILTER_ADJUST:
        case ControlContext::NONE: // Fallback to filter if NONE or unknown
        default:
            handle_filter_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
    }
}

// --- Centralized Potentiometer Control Dispatch (Keep) ---
void dispatch_pot_controls(int pot1_delta, int pot2_delta) {
    if (pot1_delta == 0 && pot2_delta == 0) return; // No movement

    bool pot1_used = false;
    bool pot2_used = false;

    // Dispatch based on the CURRENT control context (determined by state machine)
    switch (g_current_control_context) {
        case ControlContext::ARP_ADJUST:
            handle_arp_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::SIDECHAIN_ADJUST:
            handle_sidechain_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::FILTER_ADJUST:
            handle_filter_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::DELAY_ADJUST:
             handle_delay_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::REVERB_ADJUST:
             handle_reverb_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::NONE:
        default:
            handle_default_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
             break;
    }

}


// --- Secondary Pad Tap Handlers (Keep Definition for Delay/Reverb here) ---
// Called by the state machine (process_state_event)

// Delay/Reverb Adjust Pad Handler
bool handle_delay_reverb_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used) {
    if (!g_current_synth) return false;

    // Check if any pad was tapped while Pad 2 is held
    bool any_tap = false;
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        if (i != 2 && pad_pressed_this_tick[i]) {
            any_tap = true;
            pads_used[i] = true; // Mark as used
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust TAP: Pad %d", i);
        }
    }

    // If any pad was tapped, toggle between Delay and Reverb
    if (any_tap) {
        // Tapping any other pad toggles between Delay and Reverb pot control focus
        if (g_last_pad2_effect == EFFECT_DELAY) {
            g_last_pad2_effect = EFFECT_REVERB;
            g_current_synth->selectFxSlot1Effect(EFFECT_REVERB);
            g_last_pot_control_context = ControlContext::REVERB_ADJUST;
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust: Switched to Reverb");
        } else {
            g_last_pad2_effect = EFFECT_DELAY;
            g_current_synth->selectFxSlot1Effect(EFFECT_DELAY);
            g_last_pot_control_context = ControlContext::DELAY_ADJUST;
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust: Switched to Delay");
        }
        return true; // Adjustment made
    }

    return false; // No adjustment made
}


// --- Main Control Logic Handler (REMOVED) ---
/*
void handle_control_logic(const ableton::Link::SessionState& state, const std::chrono::microseconds& time) {
    // ... Entire implementation removed ...
        }
*/

// --- Active Logic Handler Implementations (Definitions moved to effect files) ---
// Definitions for handle_arp_active, handle_sidechain_active, handle_filter_active
// are now in their respective effect_X.cpp files.

