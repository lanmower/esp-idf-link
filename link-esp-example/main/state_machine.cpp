#include "state_machine.h"
#include "effect_handler.h" // For calling pot handlers, secondary handlers, active logic, pot param externs
#include "input_handler.h" // For InputEvent struct (already included in state_machine.h)
#include "synth_interface.h" // For g_current_synth actions
#include "synth_mininova.h" // For synth switching
#include "synth_microkorg.h" // For synth switching
#include "effect_arp.h" // For arp specific actions/state
#include "effect_filter.h" // For filter specific actions/state and g_filter_lfo_patched
#include "effect_sidechain.h" // For sidechain specific actions/state
#include "esp_log.h"
#include "esp_timer.h" // For timing constants

// --- Timing Constants (Copied from effect_handler.cpp) ---
// Defined globally as extern in main.h, just need derived value here
const uint64_t double_tap_window_us = DOUBLE_TAP_TIME_MS * 1000;
// Defined here as it's specific to handler logic
const uint64_t QUICK_TAP_DURATION_MS = 200;
const uint64_t quick_tap_duration_us = QUICK_TAP_DURATION_MS * 1000;

// Static variables for Pad 2 (Delay/Reverb) logic
// These might eventually become part of a larger state struct
static bool s_pad2_is_held_for_effect = false; // True if currently held activating Delay/Reverb
static uint64_t s_pad2_last_tap_time_us = 0; // For double-tap detection
static bool s_pad2_is_reverb_mode = false; // Track if we're in reverb mode (via double-tap)
// EffectType s_last_pad2_effect = EFFECT_DELAY; // Defined globally now

// Track when effects were just latched to prevent immediate unlatching
static bool s_effect_just_latched[NUM_TOUCH_PADS] = {false, false, false, false};

// Static tracking for pad release detection (Needed for state machine)
static bool s_pad_was_held_last_tick[NUM_TOUCH_PADS] = {false, false, false, false};

// Arp State
static int s_arp_note_index = 0; // Kept here, might move later



// Define the extern state variables declared in state_machine.h
// These are effectively moved from effect_handler.cpp
ControlContext g_current_control_context = ControlContext::NONE;
ControlContext g_last_pot_control_context = ControlContext::FILTER_ADJUST;
bool g_arp_latched = false;
bool g_filter_latched = true; // Filter active by default
bool g_sidechain_latched = false;
int g_pad_index_being_held_for_adjust = -1;
uint64_t g_adjust_hold_press_time_us = 0;
bool g_interaction_during_adjust_hold = false;
ArpMode g_current_arp_mode = ARP_MODE_NOTE;
uint64_t g_arp_pad_last_tap_time_us = 0;
EffectType g_last_pad2_effect = EFFECT_DELAY;
uint64_t g_all_pads_held_start = 0;
bool g_all_pads_were_held = false;
uint64_t pad_press_time[NUM_TOUCH_PADS] = {0, 0, 0, 0}; // Definition for touch_handler.cpp

// --- Static File-Scope Constants ---
static const ControlContext PAD_INDEX_TO_CONTEXT[NUM_TOUCH_PADS] = {
    ControlContext::SIDECHAIN_ADJUST, ControlContext::ARP_ADJUST,
    ControlContext::NONE, /* Pad 2 handled separately */ ControlContext::FILTER_ADJUST
};
static const EffectMode PAD_INDEX_TO_EFFECT_MODE[NUM_TOUCH_PADS] = {
    EFFECT_MODE_SIDECHAIN, EFFECT_MODE_ARP,
    (EffectMode)-1, /* Pad 2 handled separately */ EFFECT_MODE_FILTER
};

// --- Main State Processing Function ---
void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time)
{
    // --- 1. Synth switching logic --- (Needs access to synth globals)
    bool all_held = event.pad_held[0] && event.pad_held[1] && event.pad_held[2] && event.pad_held[3];
    if (all_held) {
        if (!g_all_pads_were_held) {
            g_all_pads_held_start = event.timestamp_us;
            g_all_pads_were_held = true;
        } else if (event.timestamp_us - g_all_pads_held_start > 1000000) { // 1 second in us
             if (g_current_synth) { delete g_current_synth; g_current_synth = nullptr; }
            if (g_synth_type == SYNTH_MININOVA) {
                g_current_synth = new SynthMicroKorg(1);
                g_synth_type = SYNTH_MICROKORG;
                ESP_LOGI("STATE_MACHINE", "Switched to MicroKorg synth (Reverb will be ignored)");
            } else {
                g_current_synth = new SynthMininova(1);
                g_synth_type = SYNTH_MININOVA;
                ESP_LOGI("STATE_MACHINE", "Switched to Mininova synth");
            }
             if (!g_current_synth) { ESP_LOGE("STATE_MACHINE", "Failed to allocate new synth instance!"); }
             else { g_current_synth->sendAllNotesOff(); }
             g_all_pads_were_held = false; g_all_pads_held_start = 0;
             // Reset effect states on synth switch?
             initialize_effects(); // Call effect init to reset latches etc.
        }
    } else {
        g_all_pads_were_held = false;
        g_all_pads_held_start = 0;
    }

    // Initialize array to track secondary taps detected this tick
    std::array<bool, 4> secondary_pad_taps_detected = {false, false, false, false};

    // --- 2. Pad 2 (Delay/Reverb) Simplified Logic ---
    // Check if Pad 2 is a secondary tap while another pad is being held
    if (event.pad_pressed_this_tick[2] && g_pad_index_being_held_for_adjust != -1 && g_pad_index_being_held_for_adjust != 2) {
        // This is a secondary tap while another pad is held - mark it for handling later
        secondary_pad_taps_detected[2] = true;
        g_interaction_during_adjust_hold = true;
        ESP_LOGD("STATE_MACHINE", "Interaction flag SET true due to secondary tap on Pad 2 (Adjust Pad: %d).", g_pad_index_being_held_for_adjust);
    }
    // Normal Pad 2 press handling (only if not a secondary tap)
    else if (event.pad_pressed_this_tick[2]) {
        // Record press time for hold duration calculation
        pad_press_time[2] = event.timestamp_us;

        // Check for double-tap
        if (event.timestamp_us - s_pad2_last_tap_time_us < double_tap_window_us) {
            // Double tap detected!
            s_pad2_last_tap_time_us = 0; // Reset last tap time
            s_pad2_is_reverb_mode = true;

            // Double tap sets delay input to 0% and reverb input to 100%
            g_current_synth->selectFxSlot1Effect(EFFECT_REVERB);
            g_current_synth->setFxSlot1Level(127); // Reverb at 100%
            g_last_pad2_effect = EFFECT_REVERB;
            g_last_pot_control_context = ControlContext::REVERB_ADJUST;
            ESP_LOGD("STATE_MACHINE", "Pad 2 Double-Tap -> Reverb ON, Delay OFF (Momentary)");
        } else {
            // Single tap - record for potential double-tap detection
            s_pad2_last_tap_time_us = event.timestamp_us;
            s_pad2_is_reverb_mode = false;

            // Single tap sets delay input to 100%
            g_current_synth->selectFxSlot1Effect(EFFECT_DELAY);
            g_current_synth->setFxSlot1Level(127); // Delay at 100%
            g_last_pad2_effect = EFFECT_DELAY;
            g_last_pot_control_context = ControlContext::DELAY_ADJUST;
            ESP_LOGD("STATE_MACHINE", "Pad 2 Pressed -> Delay ON (Momentary)");
        }

        // Mark as held for effect (momentary)
        s_pad2_is_held_for_effect = true;
        g_interaction_during_adjust_hold = false;
    }

    // Release detection for Pad 2
    bool pad2_released_now = !event.pad_held[2] && s_pad_was_held_last_tick[2];
    if (pad2_released_now) {
        // Reset press time
        pad_press_time[2] = 0;

        // Always turn off effects when Pad 2 is released (momentary behavior)
        g_current_synth->setFxSlot1Level(0); // Deactivate effect

        if (s_pad2_is_reverb_mode) {
            ESP_LOGD("STATE_MACHINE", "Pad 2 Released -> Reverb OFF (momentary)");
        } else {
            ESP_LOGD("STATE_MACHINE", "Pad 2 Released -> Delay OFF (momentary)");
        }

        // Reset the held state
        s_pad2_is_held_for_effect = false;
        s_pad2_is_reverb_mode = false;
    }

    // --- 3. Handle Press/Release/Hold for Pads 0, 1, 3 ---
    // Note: secondary_pad_taps_detected array is initialized at the beginning of the function
    // and Pad 2 secondary tap is already handled above

    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        if (i == 2) continue; // Skip normal processing for Pad 2 (already handled above)

        // Calculate press/release for this pad based on current event vs last tick state
        bool pressed_now = event.pad_held[i] && !s_pad_was_held_last_tick[i];
        bool released_now = !event.pad_held[i] && s_pad_was_held_last_tick[i];

        // --- Check for Secondary Taps for Pads 0, 1, 3 ---
        // Use pad_pressed_this_tick from the event to detect the *instant* of a secondary tap
        if (g_pad_index_being_held_for_adjust != -1 && event.pad_pressed_this_tick[i] && i != g_pad_index_being_held_for_adjust) {
            secondary_pad_taps_detected[i] = true; // Mark secondary tap occurred
            g_interaction_during_adjust_hold = true; // Flag interaction immediately
            ESP_LOGD("STATE_MACHINE", "Interaction flag SET true due to secondary tap on Pad %d (Adjust Pad: %d).", i, g_pad_index_being_held_for_adjust);
            continue; // Skip primary handling for this pad if it's a secondary tap
        }

        // --- Check for ARP Double-Tap First (Pad 1 Only) ---
        if (i == ARP_PAD_INDEX && event.pad_pressed_this_tick[i]) {
            if (event.timestamp_us - g_arp_pad_last_tap_time_us < double_tap_window_us) {
                // Double Tap Detected!

                // Immediately latch the arp in chord mode
                g_arp_latched = true;
                s_effect_just_latched[i] = true; // Mark as just latched to prevent immediate unlatching

                // Reset to chord mode
                reset_arp_to_chord_mode();

                ESP_LOGD("STATE_MACHINE", "ARP Double-Tap: Latched ON in CHORD mode");
                g_arp_pad_last_tap_time_us = 0; // Reset last tap time

                // Set up for chord mode configuration if the pad is held
                g_pad_index_being_held_for_adjust = i;
                g_interaction_during_adjust_hold = false;
                g_last_pot_control_context = ControlContext::ARP_ADJUST; // Set pot focus immediately

                if (g_current_synth) g_current_synth->sendAllNotesOff();
                continue; // Skip normal press/release handling for this pad this tick
            } else {
                g_arp_pad_last_tap_time_us = event.timestamp_us; // Record first tap time
            }
        }
        // --- End ARP Double-Tap Check ---

        // --- Press Handling (using derived pressed_now) ---
        // Only process press events when the pad is first pressed
        if (pressed_now) {
            // Record press time for get_time_held_ms/us
            pad_press_time[i] = event.timestamp_us;

            EffectMode target_mode = PAD_INDEX_TO_EFFECT_MODE[i];
            ControlContext target_context = PAD_INDEX_TO_CONTEXT[i];
            bool* target_latch_flag = nullptr;
            if (target_mode == EFFECT_MODE_ARP) target_latch_flag = &g_arp_latched;
            else if (target_mode == EFFECT_MODE_FILTER) target_latch_flag = &g_filter_latched;
            else if (target_mode == EFFECT_MODE_SIDECHAIN) target_latch_flag = &g_sidechain_latched;

            // --- Immediate Latch ON if not already latched, or prepare for adjust if already latched ---
            if (target_latch_flag) {
                if (!(*target_latch_flag)) {
                    // Effect is not latched, so latch it immediately
                    *target_latch_flag = true;
                    // Mark this effect as just latched to prevent immediate unlatching
                    s_effect_just_latched[i] = true;
                    ESP_LOGI("STATE_MACHINE", "Effect Mode LATCH ON via Pad %d Press -> Mode %d", i, (int)target_mode);

                    if (target_mode == EFFECT_MODE_ARP) {
                        g_current_arp_mode = ARP_MODE_NOTE;
                        ESP_LOGI("STATE_MACHINE", "ARP latched ON, defaulting to NOTE mode.");
                    }
                    if (target_mode == EFFECT_MODE_FILTER) {
                        // Reset filter to default lowpass settings
                        reset_filter_to_lowpass();
                        ESP_LOGI("STATE_MACHINE", "Filter latched ON, reset to default lowpass settings");
                    }
                    if (target_mode == EFFECT_MODE_SIDECHAIN) {
                        // Reset sidechain to default 1/4 note pattern (index 0) and set to full
                        reset_sidechain_to_default();
                        ESP_LOGI("STATE_MACHINE", "Sidechain latched ON, reset to default 1/4 pattern.");
                    }
                }
            }

            // Set up adjust hold state immediately (regardless of latch state)
            g_pad_index_being_held_for_adjust = i;
            g_interaction_during_adjust_hold = false;
            g_last_pot_control_context = target_context; // Set pot focus immediately
            ESP_LOGD("STATE_MACHINE", "Pad %d Pressed: Adjust mode active. Pot Context -> %d", i, (int)g_last_pot_control_context);
        }

        // --- Release Handling (using derived released_now) ---
        if (released_now) {
            // Clear press time
            uint64_t hold_duration_us = event.timestamp_us - pad_press_time[i];
            pad_press_time[i] = 0;
            ESP_LOGD("STATE_MACHINE", "Pad %d Released. Hold duration: %llu us", i, hold_duration_us);

            // Check if this is the pad currently held for adjust mode
            if (i == g_pad_index_being_held_for_adjust) {
                EffectMode target_mode = PAD_INDEX_TO_EFFECT_MODE[i];
                bool* target_latch_flag = nullptr;
                if (target_mode == EFFECT_MODE_ARP) target_latch_flag = &g_arp_latched;
                else if (target_mode == EFFECT_MODE_FILTER) target_latch_flag = &g_filter_latched;
                else if (target_mode == EFFECT_MODE_SIDECHAIN) target_latch_flag = &g_sidechain_latched;

                // For quick taps with no interaction, unlatch if already latched
                const uint64_t QUICK_TAP_THRESHOLD_US = 300000; // 300ms
                bool is_quick_tap = (hold_duration_us < QUICK_TAP_THRESHOLD_US);

                // Check if this effect was just latched during this press
                bool was_just_latched = s_effect_just_latched[i];

                // If it's a quick tap with no interaction and the effect is latched AND it wasn't just latched, unlatch it
                if (is_quick_tap && !g_interaction_during_adjust_hold && target_latch_flag && *target_latch_flag && !was_just_latched) {
                    // This is a tap on an already latched effect (not a new latch), so unlatch it
                    *target_latch_flag = false;
                    ESP_LOGI("STATE_MACHINE", "Pad %d Quick Tap Released -> Latch Turned OFF (was already latched)", i);

                    if (g_current_synth) g_current_synth->sendAllNotesOff();

                    if (target_mode == EFFECT_MODE_FILTER && g_current_synth) {
                        g_current_synth->deactivateFilter();
                        if (g_filter_lfo_patched) {
                            g_current_synth->unpatchLfoFromFilter();
                            g_filter_lfo_patched = false;
                            ESP_LOGI("STATE_MACHINE", "LFO Unpatched from Filter due to Latch OFF");
                        }
                    }
                    if (target_mode == EFFECT_MODE_SIDECHAIN && g_current_synth) {
                        g_current_synth->setSidechainLevel(127); // Reset sidechain level
                    }
                } else {
                    // Keep the effect latched if:
                    // 1. It was just latched during this press
                    // 2. There was interaction during the hold
                    // 3. It wasn't a quick tap
                    ESP_LOGD("STATE_MACHINE", "Pad %d Released -> Keeping latch ON (quick_tap=%s, interaction=%s, just_latched=%s)",
                             i, is_quick_tap ? "YES" : "NO", g_interaction_during_adjust_hold ? "YES" : "NO", was_just_latched ? "YES" : "NO");
                }

                // Reset the just latched flag for this pad
                s_effect_just_latched[i] = false;
            }
        }

        // If this pad was the one being held, but is now released, clear the adjust state.
        if (released_now && i == g_pad_index_being_held_for_adjust) {
             ESP_LOGD("STATE_MACHINE", "Clearing adjust hold state for released pad %d.", i);
             g_pad_index_being_held_for_adjust = -1;
             g_interaction_during_adjust_hold = false;
             // Keep g_last_pot_control_context as is
        }
    } // End for loop i=0..3 (excluding pad 2)

    // Update last held state for ALL pads *after* all handling for the tick
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        s_pad_was_held_last_tick[i] = event.pad_held[i];
        if (!event.pad_held[i]) { // Ensure press time is cleared if not held
            pad_press_time[i] = 0;
        }
    }

    // --- 4. Update Control Context ---
    ControlContext determined_context = ControlContext::NONE;
    if (g_pad_index_being_held_for_adjust != -1) {
        determined_context = PAD_INDEX_TO_CONTEXT[g_pad_index_being_held_for_adjust];
    } else if (s_pad2_is_held_for_effect) { // Check if Pad 2 is held for Delay/Reverb adjust
        determined_context = (g_last_pad2_effect == EFFECT_DELAY) ? ControlContext::DELAY_ADJUST : ControlContext::REVERB_ADJUST;
    }
    // Only log if context actually changes
    if (determined_context != g_current_control_context) {
        ESP_LOGD("STATE_MACHINE", "ControlContext changed: %d -> %d", (int)g_current_control_context, (int)determined_context);
        g_current_control_context = determined_context;
        // If entering Delay/Reverb adjust context, update pot focus
        if (determined_context == ControlContext::DELAY_ADJUST || determined_context == ControlContext::REVERB_ADJUST) {
            g_last_pot_control_context = determined_context;
        }
    }

    // --- 5. Handle Pot Interaction & Dispatch ---
    bool pots_moved_this_tick = event.pot_moved[0] || event.pot_moved[1];
    if (g_pad_index_being_held_for_adjust != -1 && pots_moved_this_tick) {
        g_interaction_during_adjust_hold = true;
        ESP_LOGD("STATE_MACHINE", "Interaction flag SET true due to pot movement (Adjust Pad: %d).", g_pad_index_being_held_for_adjust);
    }

    // Use the specific pot deltas from the event struct
    int pot1_delta = event.pot_delta[0];
    int pot2_delta = event.pot_delta[1];

    // Dispatch pot controls based on CURRENT context (if adjusting) or LAST context (if not)
    // Note: dispatch_pot_controls is defined in effect_handler.cpp
    if (pots_moved_this_tick) { // Only dispatch if there was actual movement
        dispatch_pot_controls(pot1_delta, pot2_delta);
    }

    // --- 6. Dispatch Secondary Pad Taps ---
    if (g_pad_index_being_held_for_adjust != -1 || g_current_control_context == ControlContext::DELAY_ADJUST || g_current_control_context == ControlContext::REVERB_ADJUST) {
        std::array<bool, 4> pads_used_by_secondary = {false, false, false, false}; // Local usage tracking
        ControlContext current_adjust_context = g_current_control_context;

        // Log secondary taps for debugging
        bool any_secondary_taps = false;
        for (int i = 0; i < NUM_TOUCH_PADS; i++) {
            if (secondary_pad_taps_detected[i]) {
                any_secondary_taps = true;
                ESP_LOGD("STATE_MACHINE", "Secondary tap detected on Pad %d for context %d", i, (int)current_adjust_context);
            }
        }

        if (any_secondary_taps) {
            ESP_LOGD("STATE_MACHINE", "Dispatching secondary taps for context %d", (int)current_adjust_context);
        }

        // Pass the *detected* secondary taps (secondary_pad_taps_detected) to the appropriate handler
        // Note: These handlers are defined in their respective effect files (arp, sidechain, filter)
        if (current_adjust_context == ControlContext::ARP_ADJUST) {
            handle_arp_adjusting_pads(link_state, link_time, secondary_pad_taps_detected.data(), pads_used_by_secondary);
        } else if (current_adjust_context == ControlContext::SIDECHAIN_ADJUST) {
            handle_sidechain_adjusting_pads(secondary_pad_taps_detected.data(), pads_used_by_secondary);
        } else if (current_adjust_context == ControlContext::FILTER_ADJUST) {
            handle_filter_adjusting_pads(secondary_pad_taps_detected.data(), pads_used_by_secondary);
        } else if (current_adjust_context == ControlContext::DELAY_ADJUST || current_adjust_context == ControlContext::REVERB_ADJUST) {
            // Use the secondary_pad_taps_detected array for consistency
            handle_delay_reverb_adjusting_pads(secondary_pad_taps_detected.data(), pads_used_by_secondary);
        }

        // g_interaction_during_adjust_hold was set when tap detected, no need to check handler return
    }

    // --- 7. Handle Active Effect Logic ---
    // Note: These handlers are defined in their respective effect files

    // Track which effects were active in the previous tick
    static bool s_arp_was_active = false;
    static bool s_sidechain_was_active = false;
    static bool s_filter_was_active = false;

    // Track which pads are currently being held
    static bool s_pad_being_held[NUM_TOUCH_PADS] = {false, false, false, false};



    // Update pad hold state based on current event
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        // Only update if there's a state change
        if (event.pad_held[i] != s_pad_being_held[i]) {
            s_pad_being_held[i] = event.pad_held[i];
            ESP_LOGD("STATE_MACHINE", "Pad %d hold state changed to: %s",
                     i, s_pad_being_held[i] ? "HELD" : "RELEASED");
        }
    }

    // Only call handlers when the effect state changes or when continuous processing is needed

    // Arpeggiator needs to run continuously when active
    if (g_arp_latched) {
        if (handle_arp_active(link_state, link_time, s_arp_note_index)) { // Check return value
            s_arp_note_index = (s_arp_note_index + 1) % MAX_ARP_INDEX_WRAP; // Increment ONLY if note played
        }
        s_arp_was_active = true;
    } else if (s_arp_was_active) {
        // Arp was just deactivated
        s_arp_was_active = false;
        ESP_LOGD("STATE_MACHINE", "Arpeggiator deactivated");
    }

    // Sidechain only needs to be processed on state change or at beat boundaries
    if (g_sidechain_latched) {
        if (!s_sidechain_was_active) {
            // Sidechain was just activated
            ESP_LOGD("STATE_MACHINE", "Sidechain activated");
            s_sidechain_was_active = true;
        }
        // Always process sidechain when active (it handles its own timing internally)
        handle_sidechain_active(link_state, link_time, s_current_sidechain_depth, s_current_sidechain_sheer);
    } else if (s_sidechain_was_active) {
        // Sidechain was just deactivated
        s_sidechain_was_active = false;
        ESP_LOGD("STATE_MACHINE", "Sidechain deactivated");
        // Reset sidechain level to maximum (no ducking)
        if (g_current_synth) g_current_synth->setSidechainLevel(127);
    }

    // Filter only needs to be processed on state change or when LFO is active
    if (g_filter_latched) {
        if (!s_filter_was_active) {
            // Filter was just activated
            ESP_LOGD("STATE_MACHINE", "Filter activated");
            s_filter_was_active = true;
        }
        // Always process filter when active (it handles LFO internally)
        handle_filter_active(link_state, link_time);
    } else if (s_filter_was_active) {
        // Filter was just deactivated
        s_filter_was_active = false;
        ESP_LOGD("STATE_MACHINE", "Filter deactivated");
        // Ensure filter is deactivated on the synth
        if (g_current_synth) g_current_synth->deactivateFilter();
    }

}