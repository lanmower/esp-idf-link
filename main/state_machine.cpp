#include "state_machine.h"
#include "effect_handler.h" // For calling pot handlers, secondary handlers, active logic, pot param externs
#include "input_handler.h" // For InputEvent struct (already included in state_machine.h)
#include "synth_interface.h" // For g_current_synth actions
#include "synth_mininova.h" // For synth switching
#include "synth_microkorg.h" // For synth switching
#include "effect_arp.h" // For arp specific actions/state
#include "effect_filter.h" // For filter specific actions/state and g_filter_lfo_patched
#include "effect_sidechain.h" // For sidechain specific actions/state
#include "io_helpers.h" // For set_buzzer_state
#include "esp_log.h"
#include "esp_timer.h" // For timing constants

// --- Extern declarations for variables needed for synth switching ---
extern int s_global_filter_cutoff;
extern int s_global_filter_resonance;
extern int8_t s_lfo_depth_bipolar;
extern int s_current_sidechain_pattern_index;
extern int s_current_sidechain_depth;

// --- Timing Constants (Copied from effect_handler.cpp) ---
// Defined globally as extern in main.h, just need derived value here
const uint64_t double_tap_window_us = 400000; // 400ms (increased from 300ms)
// Defined here as it's specific to handler logic
const uint64_t QUICK_TAP_DURATION_MS = 200;
const uint64_t quick_tap_duration_us = QUICK_TAP_DURATION_MS * 1000;

// Static variables for Pad 2 (Delay/Reverb) logic
// These might eventually become part of a larger state struct
static bool s_pad2_is_held_for_effect = false; // True if currently held activating Delay/Reverb
static uint64_t s_pad2_last_tap_time_us = 0; // For double-tap detection
static bool s_pad2_is_reverb_mode = false; // Track if we're in reverb mode (via double-tap)
// EffectType s_last_pad2_effect = EFFECT_DELAY; // Defined globally now

// Add variables for Pad 0 (Sidechain) double-tap detection for synth switching
static uint64_t s_pad0_last_tap_time_us = 0; // Last time Pad 0 was tapped
static bool s_synth_switch_triggered = false; // Flag to prevent multiple switches on the same double-tap

// Track when effects were just latched to prevent immediate unlatching
static bool s_effect_just_latched[NUM_TOUCH_PADS] = {false, false, false, false};

// Static tracking for pad release detection (Needed for state machine)
static bool s_pad_was_held_last_tick[NUM_TOUCH_PADS] = {false, false, false, false};

// Arp State
static int s_arp_note_index = 0; // Kept here, might move later

// Add global variables for double tap detection on sidechain pad
static uint64_t g_sidechain_pad_last_tap_time_us = 0; // Last time sidechain pad was tapped
const uint64_t sidechain_double_tap_window_us = 400000; // 400ms window for double tap

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
    // Commented out unused variables
    // bool pad_pressed[4] = {false, false, false, false};
    // bool pad_released[4] = {false, false, false, false};

    // --- Record when we process this event for accurate timing of subsequent events ---
    static uint64_t last_event_timestamp_us = 0;
    last_event_timestamp_us = event.timestamp_us;

    // --- 1. Synth switching logic --- (Needs access to synth globals)
    // Remove the old all-pads-held logic for synth switching
    // bool all_held = event.pad_held[0] && event.pad_held[1] && event.pad_held[2] && event.pad_held[3];
    // 
    // // Log pad states for debugging
    // static bool last_all_held = false;
    // if (all_held != last_all_held) {
    //     ESP_LOGI("STATE_MACHINE", "All pads held: %s [%d,%d,%d,%d]", 
    //              all_held ? "YES" : "NO",
    //              event.pad_held[0], event.pad_held[1], event.pad_held[2], event.pad_held[3]);
    //     last_all_held = all_held;
    // }
    // 
    // // Track when all pads are held
    // if (all_held) {
    //     if (!g_all_pads_were_held) {
    //         g_all_pads_held_start = event.timestamp_us;
    //         g_all_pads_were_held = true;
    //         ESP_LOGI("STATE_MACHINE", "All pads held - starting timer");
    //     } else {
    //         // Check if held for long enough
    //         const uint64_t all_held_duration_us = event.timestamp_us - g_all_pads_held_start;
    //         if (all_held_duration_us > 1000000) { // 1 second
    //             if (!g_all_held_action_triggered) {
    //                 // Toggle synth mode
    //                 g_current_synth_mode = (g_current_synth_mode == SYNTH_MODE_MININOVA) ? 
    //                                         SYNTH_MODE_MICROKORG : SYNTH_MODE_MININOVA;
    //                 
    //                 // Update the UI and load the new patch bank
    //                 set_pad_leds();
    //                 load_patch_bank();
    //                 
    //                 g_all_held_action_triggered = true;
    //                 ESP_LOGI("STATE_MACHINE", "Synth mode toggled to: %s", 
    //                         g_current_synth_mode == SYNTH_MODE_MININOVA ? "MININOVA" : "MICROKORG");
    //                 
    //                 // Play a confirmation sound
    //                 if (g_current_synth_mode == SYNTH_MODE_MININOVA) {
    //                     set_buzzer_state(true, 1000); // Higher tone for MiniNova
    //                 } else {
    //                     set_buzzer_state(true, 500);  // Lower tone for MicroKorg
    //                 }
    //                 vTaskDelay(pdMS_TO_TICKS(100));
    //                 set_buzzer_state(false, 0);
    //             }
    //         }
    //     }
    // } else {
    //     g_all_pads_were_held = false;
    //     g_all_held_action_triggered = false;
    // }

    // Initialize array to track secondary taps detected this tick
    std::array<bool, 4> secondary_pad_taps_detected = {false, false, false, false};

    // Arrays to track if pads are used (consumed) by various handlers
    std::array<bool, 4> pads_used_for_submenu_navigation = {false, false, false, false};
    std::array<bool, 4> pads_used_for_secondary_action = {false, false, false, false};

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

            if (g_synth_type == SYNTH_MININOVA) {
                // Ensure proper routing: gate -> reverb/delay in parallel
                g_current_synth->selectFxSlot1Effect(EFFECT_REVERB);
                g_current_synth->setFxSlot1Level(127); // Reverb input at 100%
                ESP_LOGD("STATE_MACHINE", "MiniNova: Reverb ON (tap-and-hold)");
            } else {
                // MicroKorg doesn't have reverb
                ESP_LOGD("STATE_MACHINE", "MicroKorg: No reverb available");
            }
            
            g_last_pad2_effect = EFFECT_REVERB;
            g_last_pot_control_context = ControlContext::REVERB_ADJUST;
        } else {
            // Single tap - record for potential double-tap detection
            s_pad2_last_tap_time_us = event.timestamp_us;
            s_pad2_is_reverb_mode = false;

            // Set up delay for both synths
            g_current_synth->selectFxSlot1Effect(EFFECT_DELAY);
            
            if (g_synth_type == SYNTH_MININOVA) {
                g_current_synth->setFxSlot1Level(127); // Delay input at 100%
                ESP_LOGD("STATE_MACHINE", "MiniNova: Delay ON (tap-and-hold)");
            } else {
                // For MicroKorg, we'll control feedback with pots while held
                ESP_LOGD("STATE_MACHINE", "MicroKorg: Delay ON (tap-and-hold)");
            }
            
            g_last_pad2_effect = EFFECT_DELAY;
            g_last_pot_control_context = ControlContext::DELAY_ADJUST;
        }

        // Mark as held for effect (momentary)
        s_pad2_is_held_for_effect = true;
        g_interaction_during_adjust_hold = false;
    }

    // Process pot controls for delay/reverb when Pad 2 is held
    if (s_pad2_is_held_for_effect) {
        if (g_synth_type == SYNTH_MICROKORG) {
            // In MicroKorg mode, pots control delay parameters while held
            if (!s_pad2_is_reverb_mode) { // Delay mode
                // Pot 1: Delay time
                if (event.pot_moved[0]) {
                    int delay_time = event.pot_value[0];
                    g_current_synth->setDelayTime(delay_time);
                    ESP_LOGD("STATE_MACHINE", "MicroKorg Delay: Time = %d", delay_time);
                }
                
                // Pot 2: Delay feedback (amount) - this is what creates the tail
                if (event.pot_moved[1]) {
                    int delay_feedback = event.pot_value[1];
                    g_current_synth->setDelayFeedback(delay_feedback);
                    ESP_LOGD("STATE_MACHINE", "MicroKorg Delay: Feedback = %d", delay_feedback);
                }
            } else { // Reverb mode (MicroKorg doesn't have reverb, so we skip)
                ESP_LOGD("STATE_MACHINE", "MicroKorg: No reverb available");
            }
        } else if (g_synth_type == SYNTH_MININOVA) {
            // In MiniNova mode, pots control delay/reverb parameters while held
            if (!s_pad2_is_reverb_mode) { // Delay mode
                // Pot 1: Delay time
                if (event.pot_moved[0]) {
                    int delay_time = event.pot_value[0];
                    g_current_synth->setDelayTime(delay_time);
                    ESP_LOGD("STATE_MACHINE", "MiniNova Delay: Time = %d", delay_time);
                }
                
                // Pot 2: Delay feedback (amount)
                if (event.pot_moved[1]) {
                    int delay_feedback = event.pot_value[1];
                    g_current_synth->setDelayFeedback(delay_feedback);
                    ESP_LOGD("STATE_MACHINE", "MiniNova Delay: Feedback = %d", delay_feedback);
                }
            } else { // Reverb mode
                // Pot 1: Reverb time/decay
                if (event.pot_moved[0]) {
                    int reverb_time = event.pot_value[0];
                    g_current_synth->setReverbDecay(reverb_time);
                    ESP_LOGD("STATE_MACHINE", "MiniNova Reverb: Decay = %d", reverb_time);
                }
                
                // Pot 2: Reverb damping (amount)
                if (event.pot_moved[1]) {
                    int reverb_damping = event.pot_value[1];
                    g_current_synth->setReverbDamping(reverb_damping);
                    ESP_LOGD("STATE_MACHINE", "MiniNova Reverb: Damping = %d", reverb_damping);
                }
            }
        }
    }

    // Release detection for Pad 2
    bool pad2_released_now = !event.pad_held[2] && s_pad_was_held_last_tick[2];
    if (pad2_released_now) {
        // Reset press time
        pad_press_time[2] = 0;

        if (g_synth_type == SYNTH_MICROKORG) {
            // For MicroKorg: Reset delay feedback to 0 (stops new echoes but lets tail continue)
            if (!s_pad2_is_reverb_mode) {
                g_current_synth->setDelayFeedback(0); // Stop feedback loop
                ESP_LOGD("STATE_MACHINE", "MicroKorg: Delay feedback reset to 0 (tail continues)");
            }
        } else if (g_synth_type == SYNTH_MININOVA) {
            // For MiniNova: Turn off input to effects (lets tail continue)
            g_current_synth->setFxSlot1Level(0); // Stop new input to effect
            
            if (s_pad2_is_reverb_mode) {
                ESP_LOGD("STATE_MACHINE", "MiniNova: Reverb input OFF (tail continues)");
            } else {
                ESP_LOGD("STATE_MACHINE", "MiniNova: Delay input OFF (tail continues)");
            }
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

        // --- Handle double tap on Pad 0 (Sidechain) for synth switching ---
        if (i == 0 && event.pad_pressed_this_tick[i]) {
            uint64_t time_since_last_tap = event.timestamp_us - s_pad0_last_tap_time_us;
            
            // Check if this is a double tap (within the window) and not already triggered
            if (time_since_last_tap < double_tap_window_us && !s_synth_switch_triggered) {
                // Reset last tap time to prevent triple-tap detection
                s_pad0_last_tap_time_us = 0;
                
                // Toggle synth type
                if (g_synth_type == SYNTH_MININOVA) {
                    g_synth_type = SYNTH_MICROKORG;
                    ESP_LOGI("STATE_MACHINE", "Switching to MICROKORG mode via double-tap on Sidechain pad");
                } else {
                    g_synth_type = SYNTH_MININOVA;
                    ESP_LOGI("STATE_MACHINE", "Switching to MININOVA mode via double-tap on Sidechain pad");
                }
                
                // Set up for the new synth
                // Avoid memory leak by deleting the old synth first
                if (g_current_synth != nullptr) {
                    delete g_current_synth;
                }
                
                if (g_synth_type == SYNTH_MININOVA) {
                    g_current_synth = new SynthMininova();
                } else {
                    g_current_synth = new SynthMicroKorg();
                }
                
                // Play a confirmation tone
                set_buzzer_state(true, g_synth_type == SYNTH_MININOVA ? 1000 : 500);
                vTaskDelay(pdMS_TO_TICKS(100));
                set_buzzer_state(false, 0);
                
                // If arp is active, reload MIDI files from the appropriate folder
                if (g_arp_latched && g_midi_player_active) {
                    ESP_LOGI("STATE_MACHINE", "Reloading MIDI files for new synth type");
                    reset_arp_to_midi_player(false);
                }
                
                // Set flag to prevent multiple triggers
                s_synth_switch_triggered = true;
            } else {
                // Record this tap for potential double-tap detection
                s_pad0_last_tap_time_us = event.timestamp_us;
                s_synth_switch_triggered = false;
            }
        }
        
        // Reset synth switch triggered flag on pad release
        if (i == 0 && released_now) {
            s_synth_switch_triggered = false;
        }

        // --- Check for Secondary Taps for Pads 0, 1, 3 ---
        // Use pad_pressed_this_tick from the event to detect the *instant* of a secondary tap
        if (g_pad_index_being_held_for_adjust != -1 && event.pad_pressed_this_tick[i] && i != g_pad_index_being_held_for_adjust) {
            secondary_pad_taps_detected[i] = true; // Mark secondary tap occurred
            g_interaction_during_adjust_hold = true; // Flag interaction immediately
            ESP_LOGI("STATE_MACHINE", "Secondary tap detected! Pad %d tapped while Pad %d held for adjust. Current context: %d", 
                     i, g_pad_index_being_held_for_adjust, (int)g_current_control_context);
            
            // Special handling for arp pad held and submenu pad tapped
            if (g_pad_index_being_held_for_adjust == ARP_PAD_INDEX) {
                ESP_LOGI("STATE_MACHINE", "ARP submenu action: Pad %d tapped while ARP held", i);
                pads_used_for_submenu_navigation[i] = true;
            }
            
            // IMPORTANT: Don't let the tapped pad take over as the primary adjust pad
            // Mark this pad as used to prevent it from becoming the new adjust pad
            pads_used_for_secondary_action[i] = true;
            
            // Skip the normal press handling for this secondary pad
            continue;
        }

        // --- Check for ARP Double-Tap First (Pad 1 Only) ---
        if (i == ARP_PAD_INDEX && event.pad_pressed_this_tick[i] && !pads_used_for_secondary_action[i]) {
            // Log the time since last ARP tap
            uint64_t time_since_last_tap = event.timestamp_us - g_arp_pad_last_tap_time_us;
            
            // If this is within the double-tap window, it's a double tap
            if (time_since_last_tap < double_tap_window_us && time_since_last_tap > 50000) { // >50ms to avoid bounce
                ESP_LOGI("STATE_MACHINE", "ARP Double-Tap detected!");
                
                // Double Tap Detected! - Cycle to next file
                g_midi_player.nextFile();
                s_effect_just_latched[i] = true; // Mark as just latched to prevent immediate unlatching
                
                ESP_LOGI("STATE_MACHINE", "ARP Double-Tap: Cycled to next MIDI file");
                g_arp_pad_last_tap_time_us = 0; // Reset last tap time

                // Skip the rest of the pad handling for this pad
                pads_used_for_secondary_action[i] = true;
                continue;
            }
            
            // For single tap, update the last tap time but let it fall through to normal handling
            g_arp_pad_last_tap_time_us = event.timestamp_us;
        }

        // --- Press Handling (using derived pressed_now) ---
        // Only process press events when the pad is first pressed
        if (pressed_now) {
            // Skip if this pad was already handled as a secondary tap
            if (pads_used_for_secondary_action[i]) {
                ESP_LOGI("STATE_MACHINE", "Pad %d press skipped - already processed as secondary action", i);
                continue;
            }
        
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
                        // Start MIDI player when ARP is latched on
                        reset_arp_to_midi_player(false);
                        ESP_LOGI("STATE_MACHINE", "ARP latched ON, starting MIDI player");
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
                    if (target_mode == EFFECT_MODE_ARP) {
                        // Stop MIDI player when ARP is unlatched
                        g_midi_player.stop();
                        g_midi_player_active = false;
                        ESP_LOGI("STATE_MACHINE", "ARP unlatched, stopped MIDI player");
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

        // --- NEW: Sidechain Pad Double-Tap for Synth Switching ---
        if (i == SIDECHAIN_PAD_INDEX && event.pad_pressed_this_tick[i] && !pads_used_for_secondary_action[i]) {
            uint64_t time_since_last_tap = event.timestamp_us - g_sidechain_pad_last_tap_time_us;
            ESP_LOGI("STATE_MACHINE", "Sidechain pad pressed. Last tap: %llu ms ago (window: %llu ms)", 
                    time_since_last_tap / 1000, sidechain_double_tap_window_us / 1000);
            
            // If this is within the double-tap window, it's a double tap (avoiding bounce)
            if (time_since_last_tap < sidechain_double_tap_window_us && time_since_last_tap > 50000) {
                ESP_LOGI("STATE_MACHINE", "Sidechain Double-Tap detected! Switching synth mode.");
                
                // Stop any active MIDI playback
                g_midi_player.stop();
                g_midi_player_active = false;
                
                // Delete the current synth interface with proper nullptr safety check
                if (g_current_synth) {
                    ESP_LOGI("STATE_MACHINE", "Deleting current synth");
                    g_current_synth->sendAllNotesOff(); // Make sure all notes are off before switching
                    delete g_current_synth;
                    g_current_synth = nullptr;
                }
                
                // Add small delay to ensure clean switching
                vTaskDelay(pdMS_TO_TICKS(10));
                
                // Toggle synth mode and create a new synth instance
                if (g_synth_type == SYNTH_MININOVA) {
                    g_synth_type = SYNTH_MICROKORG;
                    try {
                        g_current_synth = new SynthMicroKorg(1); // Use MIDI channel 1
                        ESP_LOGI("STATE_MACHINE", "Switched to MicroKorg synthesizer");
                        
                        // Play a test note to confirm new synth is working
                        g_current_synth->sendNoteOn(60, 100); // Middle C
                        vTaskDelay(pdMS_TO_TICKS(100));
                        g_current_synth->sendNoteOff(60, 64);
                    } catch (...) {
                        ESP_LOGE("STATE_MACHINE", "Failed to create MicroKorg instance, reverting to MiniNova");
                        g_synth_type = SYNTH_MININOVA;
                        g_current_synth = new SynthMininova(1); // Fall back to MiniNova
                    }
                } else {
                    g_synth_type = SYNTH_MININOVA;
                    try {
                        g_current_synth = new SynthMininova(1); // Use MIDI channel 1
                        ESP_LOGI("STATE_MACHINE", "Switched to MiniNova synthesizer");
                        
                        // Play a test note to confirm new synth is working
                        g_current_synth->sendNoteOn(60, 100); // Middle C
                        vTaskDelay(pdMS_TO_TICKS(100));
                        g_current_synth->sendNoteOff(60, 64);
                    } catch (...) {
                        ESP_LOGE("STATE_MACHINE", "Failed to create MiniNova instance, reverting to MicroKorg");
                        g_synth_type = SYNTH_MICROKORG;
                        g_current_synth = new SynthMicroKorg(1); // Fall back to MicroKorg
                    }
                }
                
                // Safety check to ensure we have a valid synth
                if (!g_current_synth) {
                    ESP_LOGE("STATE_MACHINE", "CRITICAL: Failed to create any synth instance!");
                    // Emergency fallback - create a MiniNova as last resort
                    g_synth_type = SYNTH_MININOVA;
                    g_current_synth = new SynthMininova(1);
                }
                
                // Reapply active effect states
                if (g_current_synth) {
                    // Reapply filter state
                    if (g_filter_latched) {
                        g_current_synth->activateFilter(s_global_filter_cutoff, s_global_filter_resonance);
                        if (g_filter_lfo_patched) {
                            g_current_synth->patchLfoToFilter(s_lfo_depth_bipolar);
                        }
                    }
                    
                    // Reapply sidechain state
                    if (g_sidechain_latched) {
                        g_current_synth->setSidechainPattern(s_current_sidechain_pattern_index);
                        g_current_synth->setSidechainLevel(s_current_sidechain_depth);
                    }
                    
                    // Restart MIDI player if needed
                    if (g_arp_latched) {
                        reset_arp_to_midi_player(g_current_arp_mode == ARP_MODE_CHORD);
                    }
                }
                
                // Produce beep to indicate synth switch
                set_buzzer_state(true, g_synth_type == SYNTH_MININOVA ? 1000 : 500);
                vTaskDelay(pdMS_TO_TICKS(100)); 
                set_buzzer_state(false, 0);
                
                // Mark this pad as used to prevent double actions
                pads_used_for_secondary_action[i] = true;
            }
            
            // Update last tap time for next time
            g_sidechain_pad_last_tap_time_us = event.timestamp_us;
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

        //ESP_LOGI("STATE_MACHINE", "Secondary tap check - Adjust pad: %d, Context: %d", 
        //        g_pad_index_being_held_for_adjust, (int)current_adjust_context);

        // Log secondary taps for debugging
        bool any_secondary_taps = false;
        for (int i = 0; i < NUM_TOUCH_PADS; i++) {
            if (secondary_pad_taps_detected[i]) {
                any_secondary_taps = true;
                ESP_LOGI("STATE_MACHINE", "Secondary tap detected on Pad %d for context %d", i, (int)current_adjust_context);
            }
        }

        if (any_secondary_taps) {
            ESP_LOGI("STATE_MACHINE", "Dispatching secondary taps for context %d", (int)current_adjust_context);
        } else {
            ESP_LOGD("STATE_MACHINE", "No secondary taps detected despite having adjust pad: %d", g_pad_index_being_held_for_adjust);
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