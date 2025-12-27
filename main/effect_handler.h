#ifndef EFFECT_HANDLER_H
#define EFFECT_HANDLER_H

#include <ableton/Link.hpp> // Include Link header for SessionState
#include <chrono>           // Include chrono for microseconds
#include <array>            // Include for std::array
#include <stdint.h>
#include "main.h" // Include main header for global types/constants (includes synth_interface.h)
#include "link_sync.h"
#include "esp_log.h"
#include "midi_helpers.h" // For clamp_value and other MIDI functions
#include <algorithm> // For std::max / std::min

// Include effect-specific headers needed for declarations or shared constants
#include "effect_arp.h"
#include "effect_filter.h" // Include for g_filter_lfo_patched extern
#include "effect_sidechain.h"
#include "lfo_constants.h"

// --- Constants ---
#define SIDECHAIN_PAD_INDEX 0
#define ARP_PAD_INDEX 1
#define DELAY_REVERB_PAD_INDEX 2
#define FILTER_PAD_INDEX 3

// --- Enums ---
// Defines the possible effect modes
enum EffectMode {
    EFFECT_MODE_FILTER,
    EFFECT_MODE_ARP,
    EFFECT_MODE_SIDECHAIN,
    // Add other modes like EFFECT_MODE_REVERB, EFFECT_MODE_DELAY
    EFFECT_MODE_COUNT // Optional: for counting modes
};

// Defines the context for potentiometer and secondary pad controls
enum ControlContext {
    NONE,             // Default state, pots might control global params (e.g., filter)
    FILTER_ADJUST,    // Filter pad held
    ARP_ADJUST,       // Arp pad held
    SIDECHAIN_ADJUST, // Sidechain pad held
    DELAY_ADJUST,     // Added for Pad 2 Delay pot control
    REVERB_ADJUST,    // Added for Pad 2 Reverb pot control
    // Add other contexts like REVERB_ADJUST, DELAY_ADJUST
};

// --- Global State Definitions ---
extern EffectMode g_current_effect_mode;
extern ControlContext g_current_control_context;
extern ControlContext g_last_pot_control_context; // Added to track last context for pots
extern SynthInterface* g_current_synth;
extern SynthType g_synth_type;

// Example: Filter parameters (if handled globally or in default mode)
extern int s_global_filter_cutoff; // Make extern if needed elsewhere
extern int s_global_filter_resonance; // Make extern if needed elsewhere

// Example: Sidechain parameters (if adjust pots are handled here)
// extern int s_current_sidechain_depth; // Now declared extern
// extern int s_current_sidechain_sheer; // Now declared extern
// Note: These are now DECLARED here, DEFINED in effect_handler.cpp
extern int s_current_sidechain_depth;
extern int s_current_sidechain_sheer;

// --- Additional Global State Definitions ---
// extern bool g_arp_latched;
// extern bool g_filter_latched;
// extern bool g_sidechain_latched;
// extern bool g_filter_lfo_patched; // Moved to effect_filter.h

// Extern declarations for parameters adjusted by pots (Defined in effect_handler.cpp)
extern int s_current_delay_time;
extern int s_current_delay_feedback;
extern int s_current_reverb_decay;
extern int s_current_reverb_damping;

// --- Initialization ---
void initialize_effects();

// --- Central Control Functions (Defined in effect_handler.cpp) ---
// void update_control_context(const bool pad_held[]); // REMOVED - Merged
void dispatch_pot_controls(int pot1_delta, int pot2_delta); // Kept, simplified
// void dispatch_pad_controls(...); // REMOVED - Merged

// --- Pot Adjustment Handlers (Defined in effect_handler.cpp, except Arp) ---
// These handle pot adjustments *within* a specific context
void handle_default_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_filter_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_sidechain_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_delay_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_reverb_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
// Arp pot handler is defined in effect_arp.cpp
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used); // Prototype needed

// --- Secondary Pad Tap Handlers (Defined in respective effect_X.cpp files) ---
// These handle secondary taps *within* a specific context
bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);
bool handle_sidechain_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);
bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);
bool handle_delay_reverb_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

// --- Active Logic Handlers (Defined in respective effect_X.cpp files) ---
// Called periodically based on g_current_effect_mode
bool handle_arp_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int note_index);
void handle_sidechain_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int depth, int sheer);
void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

// --- Helper Function Prototypes (Defined in effect_handler.cpp) ---
// Internal helper used by dispatch_pad_controls
int find_secondary_tapped_pad(int primary_index, const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

// --- Internal Helper Definitions ---

// Pot Parameter Update (Template Definition)
// Updates a parameter based on delta, clamps it, logs, and sets used flag.
// Improved to be more responsive to small changes
template<typename T>
bool _update_pot_param(T& param, int delta, T min_val, T max_val, const char* param_name, bool& used_flag) {
    // Always mark the pot as used if there was any delta
    if (delta != 0) {
        used_flag = true;
    } else {
        return false; // No change if delta is zero
    }

    T old_val = param;

    // Apply delta with care for potential overflow/underflow
    T new_val_unclamped = param + delta;

    // Clamp the value to the valid range
    T new_val = std::max(min_val, std::min(max_val, new_val_unclamped));

    // Update the parameter even if the change is small
    if (new_val != old_val) {
        param = new_val;

        // Log with appropriate format specifier for T
        if constexpr (std::is_same_v<T, int> || std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>) {
            ESP_LOGD("POT_UPDATE", "%s: %d (Delta: %d)", param_name, static_cast<int>(param), delta);
        } else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
            ESP_LOGD("POT_UPDATE", "%s: %.2f (Delta: %d)", param_name, static_cast<double>(param), delta);
        } else {
            // Fallback logging for other types
            ESP_LOGD("POT_UPDATE", "%s updated (Delta: %d)", param_name, delta);
        }
        return true;
    }

    // Return false if the value didn't change (due to clamping at min/max)
    return false;
}

#endif // EFFECT_HANDLER_H