#include "effect_sidechain.h"
// #include "midi_helpers.h" // No longer needed directly
#include "midi_helpers.h" // Added for scaling functions
#include "main.h" // For std::array, g_link, scale_pot_value etc.
#include "synth_interface.h" // Include the synth interface
#include "esp_log.h"
#include "effect_handler.h" // Include for extern declarations & EffectState
#include "sidechain_constants.h" // Include the defined patterns
#include "state_machine.h" // Include for g_sidechain_latched
#include "link_sync.h" // For quantum boundary detection

static const char *TAG_SC = "SIDECHAIN";

// Define the speed factor for sidechain (4x faster)
const double SIDECHAIN_SPEED_FACTOR = 4.0;

// --- Sidechain State (Static to this file) ---
static int s_current_sidechain_pattern_index = SIDECHAIN_DEFAULT_PATTERN_INDEX;
static int s_last_sc_step_index = -1;
static float s_current_smoothed_level = 0.0f;

// --- Reset Sidechain to Default Settings ---
// Called when sidechain is latched
void reset_sidechain_to_default() {
    // Reset to default 1/4 note pattern (index 0)
    s_current_sidechain_pattern_index = 0; // Quarter note pattern

    // Reset the step index to force immediate update
    s_last_sc_step_index = -1;

    // Set depth to maximum (127)
    s_current_sidechain_depth = 127;

    // Set sheer to default value
    s_current_sidechain_sheer = 0;

    // Notify the synth of the pattern change
    if (g_current_synth) {
        g_current_synth->setSidechainPattern(s_current_sidechain_pattern_index);
    }

    ESP_LOGD(TAG_SC, "Sidechain reset to default: Pattern=%d, Depth=%d, Sheer=%d",
             s_current_sidechain_pattern_index, s_current_sidechain_depth, s_current_sidechain_sheer);
}

// --- Sidechain Secondary Pad Tap Handler ---
// Called from state_machine.cpp when g_current_control_context == SIDECHAIN_ADJUST
bool handle_sidechain_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used)
{
    if (!g_current_synth) return false;

    int tapped_pad = find_secondary_tapped_pad(SIDECHAIN_PAD_INDEX, pad_pressed_this_tick, pads_used);

    if (tapped_pad != -1) {
        // Select pattern based on which pad was tapped
        int new_pattern_index = -1;
        // Mapping: Pad 0 -> Pattern 0, Pad 1 -> Pattern 1, Pad 3 -> Pattern 3
        if (tapped_pad == 0) new_pattern_index = 0;
        else if (tapped_pad == 1) new_pattern_index = 1;
        else if (tapped_pad == 3) new_pattern_index = 3;
        // else: other pads don't change pattern

        // Check if index is valid and different from current
        if (new_pattern_index >= 0 && new_pattern_index < NUM_SIDECHAIN_PATTERNS && new_pattern_index != s_current_sidechain_pattern_index) {
            s_current_sidechain_pattern_index = new_pattern_index;
            ESP_LOGD(TAG_SC, "SC Adjust TAP: Select Pattern -> %d (via Pad %d)", s_current_sidechain_pattern_index, tapped_pad);
            g_current_synth->setSidechainPattern(s_current_sidechain_pattern_index); // Assuming synth needs pattern index
            s_last_sc_step_index = -1; // Reset step tracking on pattern change
            return true; // Adjustment made
        } else if (new_pattern_index == s_current_sidechain_pattern_index) {
             ESP_LOGD(TAG_SC, "SC Adjust: Pad %d tapped, pattern %d already selected.", tapped_pad, new_pattern_index);
        } else if (new_pattern_index != -1) { // Log only if a valid pad was mapped but index was bad
             ESP_LOGW(TAG_SC, "SC Adjust: Invalid pattern index %d attempted via Pad %d tap.", new_pattern_index, tapped_pad);
        }
    }

    return false; // No valid secondary tap adjustment
}

// --- Active Sidechain Logic ---
// Called periodically from state_machine.cpp when sidechain is latched
void handle_sidechain_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time,
                           int depth, int sheer) // Uses passed depth and sheer
{
    // Check if sidechain mode is active using the latch flag
    // Note: g_sidechain_latched is checked by the caller (state_machine.cpp)
    if (!g_current_synth || !g_link) return;

    // Get quantum boundary and phase information using the common function
    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    // Extract values from the quantum info for use in this function
    const double phase_within_quantum = quantumInfo.phaseWithinQuantum;
    const double quantum_beat = quantumInfo.sessionBeat;
    const bool quantum_boundary_crossed = quantumInfo.crossedQuantumBoundary;

    // Calculate step duration for the original speed
    const double original_beats_per_step = LINK_QUANTUM / static_cast<double>(SIDECHAIN_RHYTHM_STEPS);

    // Apply the speed factor to make the sidechain 4x faster
    // This means the pattern will repeat 4 times within one quantum

    // Add a phase offset to start the sidechain pattern slightly earlier
    // This ensures the ducking happens before the arpeggiator's notes start
    // Using 1/16th note offset (0.25 steps in a 16-step pattern)
    const double PHASE_OFFSET = original_beats_per_step * 0.25;

    // Apply offset and speed factor
    double scaled_phase = (phase_within_quantum + PHASE_OFFSET) * SIDECHAIN_SPEED_FACTOR;

    // Ensure phase wraps around properly within the quantum
    while (scaled_phase >= LINK_QUANTUM * SIDECHAIN_SPEED_FACTOR) {
        scaled_phase -= LINK_QUANTUM * SIDECHAIN_SPEED_FACTOR;
    }

    // Log quantum boundary crossing for sidechain
    if (quantum_boundary_crossed) {
        ESP_LOGI(TAG_SC, "SC: Quantum boundary crossed at beat %.2f", quantum_beat);
    }

    // Calculate the step index based on the scaled phase
    // We use modulo to wrap around the pattern when it repeats
    int current_step_index = static_cast<int>(floor(scaled_phase / original_beats_per_step)) % SIDECHAIN_RHYTHM_STEPS;
    current_step_index = clamp_value(current_step_index, 0, SIDECHAIN_RHYTHM_STEPS - 1);

    // Log quantum alignment information for debugging
    ESP_LOGV(TAG_SC, "SC Quantum Alignment: Beat=%.2f, Phase=%.2f, Offset=%.2f, ScaledPhase=%.2f, Step=%d/%d, SpeedFactor=%.1f",
             quantum_beat, phase_within_quantum, PHASE_OFFSET, scaled_phase, current_step_index, SIDECHAIN_RHYTHM_STEPS, SIDECHAIN_SPEED_FACTOR);

    if (current_step_index != s_last_sc_step_index) {
        ESP_LOGV(TAG_SC, "SC Step Change: %d -> %d (Beat: %.2f)", s_last_sc_step_index, current_step_index, quantum_beat);

        // Get the current pattern
        if (s_current_sidechain_pattern_index >= 0 && s_current_sidechain_pattern_index < NUM_SIDECHAIN_PATTERNS) {
            const auto& pattern = SIDECHAIN_PATTERNS[s_current_sidechain_pattern_index];
            bool gate_on = pattern[current_step_index]; // true = sound on, false = ducked

            // Use the passed parameters
            // For pot 1 (depth): 0 means no ducking (stays at 100%), 127 means full ducking (goes to 0%)
            // This inverts the behavior so higher values = more ducking
            uint8_t min_level_target = 127 - clamp_value(depth, 0, 127); // Invert depth for low point
            uint8_t sheer_param = clamp_value(sheer, 0, 127); // Clamp passed sheer

            // Determine target level based on pattern gate
            // High point (gate_on) is always 127 (100%), low point is controlled by pot 1
            float target_level_float = gate_on ? 127.0f : (float)min_level_target;

            // Calculate slew rate based on sheer value
            float alpha = 0.01f + ( (float)sheer_param / 127.0f ) * 0.98f; // Range approx 0.01 to 0.99

            // Apply smoothing (linear interpolation)
            s_current_smoothed_level = alpha * target_level_float + (1.0f - alpha) * s_current_smoothed_level;

            // Round to nearest uint8_t for MIDI/synth output
            uint8_t final_level = (uint8_t)(s_current_smoothed_level + 0.5f);
            final_level = clamp_value(final_level, (uint8_t)0, (uint8_t)127); // Final clamp

            ESP_LOGV(TAG_SC, "SC Pattern %d, Step %d: Gate=%s, Target=%d, MinLevel=%d, Sheer=%d, Alpha=%.2f, Smoothed=%.1f, Final=%d",
                     s_current_sidechain_pattern_index, current_step_index, gate_on ? "ON" : "OFF",
                     (int)target_level_float, min_level_target,
                     sheer_param, alpha, s_current_smoothed_level, final_level);

            g_current_synth->setSidechainLevel(final_level);

        } else {
            ESP_LOGW(TAG_SC, "Invalid sidechain pattern index: %d", s_current_sidechain_pattern_index);
        }

        s_last_sc_step_index = current_step_index;
    }
}