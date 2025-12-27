#include "effect_filter.h"
#include "main.h" // Include main context
#include "synth_interface.h" // Include the synth interface
#include "effect_handler.h" // Include for accessing global state like pot context if needed? (Check dependencies)
#include "esp_log.h"
#include "lfo_constants.h" // Include LFO definitions
#include <cmath> // For fmod, sin
#include "link_sync.h" // For quantum boundary detection

static const char *TAG_FILTER = "EFFECT_FILTER";

// --- Filter State ---
// Define the LFO patched state (declared extern in .h)
bool g_filter_lfo_patched = false;

// Define global pot-adjusted parameters (declared extern in effect_handler.h)
// These are used locally by handle_filter_active and handle_filter_adjust_pots
// Consider if they should also move here if filter becomes more self-contained.
extern int s_global_filter_cutoff;
extern int s_global_filter_resonance;

// Access global synth pointer (defined in main.cpp)
extern SynthInterface* g_current_synth;
// Access global Link object (defined in main.cpp)
extern std::unique_ptr<ableton::Link> g_link;

// LFO parameters (used when LFO is patched)
static int s_lfo_shape_index = 0;
static int s_lfo_rate_index = 4; // Default to 1/4 note? index in LFO_SYNC_RATES
int8_t s_lfo_depth_bipolar = 63; // Max positive depth by default (-64 to +63)

// LFO State Variables (Static to this file, controlled via effect_handler)

// --- Active Filter Logic ---
// Can handle LFO modulation if patched
void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    if (!g_current_synth || !g_link) return;

    // If LFO is patched, calculate modulation
    if (g_filter_lfo_patched) {
        double tempo = state.tempo();
        if (tempo <= 0) return;

        // Get quantum boundary and phase information using the common function
        QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

        // Extract values from the quantum info for use in this function
        double beat = quantumInfo.sessionBeat;
        const bool quantum_boundary_crossed = quantumInfo.crossedQuantumBoundary;

        // Log quantum boundary crossing for filter
        if (quantum_boundary_crossed) {
            ESP_LOGI(TAG_FILTER, "Filter: Quantum boundary crossed at beat %.2f", beat);
        }

        // Get LFO Period in Beats
        double lfo_period_beats = 1.0; // Default
        if (s_lfo_rate_index >= 0 && s_lfo_rate_index < NUM_LFO_SYNC_RATES) {
            // Map the LFO_SYNC_RATES index to the actual beat period
            const double lfo_period_map[] = {
                16.0,    // 4 Bars
                8.0,     // 2 Bars
                4.0,     // 1 Bar
                3.0,     // 1/2 Dotted
                2.0,     // 1/2 Note
                4.0/3.0, // 1/2 Triplet
                1.5,     // 1/4 Dotted
                1.0,     // 1/4 Note
                2.0/3.0, // 1/4 Triplet
                0.75,    // 1/8 Dotted
                0.5,     // 1/8 Note
                1.0/3.0, // 1/8 Triplet
                0.375,   // 1/16 Dotted
                0.25,    // 1/16 Note
                1.0/6.0, // 1/16 Triplet
                0.125    // 1/32 Note
            };
            if (s_lfo_rate_index < sizeof(lfo_period_map)/sizeof(lfo_period_map[0])) {
                lfo_period_beats = lfo_period_map[s_lfo_rate_index];
            } else {
                ESP_LOGW(TAG_FILTER, "LFO rate index out of bounds for period map!");
            }
        }

        // Calculate Phase (0.0 to 1.0)
        double phase = fmod(beat / lfo_period_beats, 1.0);

        // Get LFO value based on shape
        double lfo_value_norm = 0.0; // Normalized 0.0 to 1.0
        int shape_index = s_lfo_shape_index; // Use index directly
        switch (shape_index) {
             case 0: // Sine (approx using cosine for 0 start)
                 lfo_value_norm = 0.5 - 0.5 * cos(phase * 2.0 * M_PI);
                 break;
             case 1: // Triangle
                 lfo_value_norm = 2.0 * ((phase < 0.5) ? phase : 1.0 - phase);
                 break;
             case 2: // Saw Down
                 lfo_value_norm = 1.0 - phase;
                 break;
             case 3: // Square
                 lfo_value_norm = (phase < 0.5) ? 0.0 : 1.0;
                 break;
            // case 4: // S&H - Needs state persistence, skipping for now
            //     break;
            default:
                 lfo_value_norm = 0.5; // Default to center
        }

        // Scale by bipolar depth (-64 to +63)
        double lfo_mod_value = lfo_value_norm * (double)s_lfo_depth_bipolar;

        // Apply to base cutoff (using the extern variable defined in effect_handler.cpp)
        double final_cutoff_double = (double)s_global_filter_cutoff + lfo_mod_value;

        // Clamp and send
        int final_cutoff_int = clamp_value((int)round(final_cutoff_double), 0, 127);
        g_current_synth->setFilterCutoff(final_cutoff_int);

        ESP_LOGV(TAG_FILTER, "Filter LFO: Beat=%.2f RateIdx=%d Period=%.2f Ph=%.2f Shp=%d Val=%.2f Mod=%.1f Cutoff=%d (%d)",
                 beat, s_lfo_rate_index, lfo_period_beats, phase, shape_index, lfo_value_norm, lfo_mod_value, final_cutoff_int, s_global_filter_cutoff);

    } else {
         // When LFO not patched, ensure filter is at base cutoff value
         // This prevents LFO value from sticking if unpatched mid-cycle
         static int last_sent_cutoff = -1;
         // Use the extern cutoff value defined in effect_handler.cpp
         if (s_global_filter_cutoff != last_sent_cutoff) {
            g_current_synth->setFilterCutoff(s_global_filter_cutoff);
            last_sent_cutoff = s_global_filter_cutoff;
            ESP_LOGV(TAG_FILTER, "Filter LFO Inactive: Setting base cutoff %d", s_global_filter_cutoff);
         }
    }
}

// --- Filter Secondary Pad Tap Handler ---
// Called from state_machine.cpp when g_current_control_context == FILTER_ADJUST
bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used)
{
    if (!g_current_synth) return false;

    // Use the helper function from effect_handler to find the tapped pad
    // Need to include effect_handler.h for this
    int tapped_pad = find_secondary_tapped_pad(FILTER_PAD_INDEX, pad_pressed_this_tick, pads_used);

    if (tapped_pad != -1) {
        ESP_LOGD(TAG_FILTER, "Filter Adjust TAP: Pad %d detected", tapped_pad);
        bool adjustment_made = false;

        // Always enable LFO when a pad is tapped in filter adjust mode
        if (!g_filter_lfo_patched) {
            ESP_LOGD(TAG_FILTER, "Filter Adjust TAP: Patching LFO -> Filter Freq");
            g_current_synth->patchLfoToFilter(s_lfo_depth_bipolar);
            g_filter_lfo_patched = true;
            adjustment_made = true; // Patching counts as an adjustment
        }

        // Apply different LFO presets based on which pad was tapped
        switch (tapped_pad) {
            case 0: // Pad 0 - Wobble Bass LFO Preset
                // Set to square wave at 1/8 note
                s_lfo_shape_index = 3; // Square wave
                s_lfo_rate_index = 10; // 1/8 note (index 10)
                s_lfo_depth_bipolar = 40; // Moderate depth
                s_global_filter_resonance = 80; // High resonance for squelchy wobble
                
                // Apply the settings
                g_current_synth->setLfoShape(s_lfo_shape_index);
                g_current_synth->setLfoRateSync(LFO_SYNC_RATES[s_lfo_rate_index]);
                g_current_synth->setLfoSyncEnabled(true);
                g_current_synth->setFilterResonance(s_global_filter_resonance);
                
                ESP_LOGI(TAG_FILTER, "LFO Preset 1: Wobble Bass - Square wave at 1/8 note, high resonance");
                adjustment_made = true;
                break;
                
            case ARP_PAD_INDEX: // Pad 1 - Smooth Sweep LFO Preset
                // Set to sine wave at 1 bar
                s_lfo_shape_index = 0; // Sine wave
                s_lfo_rate_index = 2;  // 1 bar (index 2)
                s_lfo_depth_bipolar = 50; // Medium-high depth
                s_global_filter_resonance = 30; // Moderate resonance
                
                // Apply the settings
                g_current_synth->setLfoShape(s_lfo_shape_index);
                g_current_synth->setLfoRateSync(LFO_SYNC_RATES[s_lfo_rate_index]);
                g_current_synth->setLfoSyncEnabled(true);
                g_current_synth->setFilterResonance(s_global_filter_resonance);
                
                ESP_LOGI(TAG_FILTER, "LFO Preset 2: Smooth Sweep - Sine wave at 1 bar, moderate resonance");
                adjustment_made = true;
                break;
                
            case 3: // Pad 3 - Fast Rhythmic LFO Preset
                // Set to triangle wave at 1/16 triplet
                s_lfo_shape_index = 1; // Triangle wave
                s_lfo_rate_index = 14; // 1/16 triplet (index 14)
                s_lfo_depth_bipolar = 30; // Lower depth for subtle effect
                s_global_filter_resonance = 50; // Medium resonance
                
                // Apply the settings
                g_current_synth->setLfoShape(s_lfo_shape_index);
                g_current_synth->setLfoRateSync(LFO_SYNC_RATES[s_lfo_rate_index]);
                g_current_synth->setLfoSyncEnabled(true);
                g_current_synth->setFilterResonance(s_global_filter_resonance);
                
                ESP_LOGI(TAG_FILTER, "LFO Preset 3: Fast Rhythmic - Triangle wave at 1/16 triplet, medium resonance");
                adjustment_made = true;
                break;
                
            default: // No other pads have assigned actions
                break;
        }
        return adjustment_made;
    }

    return false; // No secondary tap adjustment
}

// Function to initialize filter-specific state
void initialize_filter() {
    g_filter_lfo_patched = false;
    s_lfo_shape_index = 0;
    s_lfo_rate_index = 4; // Default to 1/4 note?
    s_lfo_depth_bipolar = 63;
    ESP_LOGI(TAG_FILTER, "Filter Initialized");
}

// Reset filter to default lowpass settings
void reset_filter_to_lowpass() {
    // Set default filter parameters
    // Use the extern variables defined in effect_handler.cpp
    s_global_filter_cutoff = 64;    // Mid-range cutoff
    s_global_filter_resonance = 20; // Moderate resonance

    // Ensure LFO is unpatched
    if (g_filter_lfo_patched && g_current_synth) {
        g_current_synth->unpatchLfoFromFilter();
        g_filter_lfo_patched = false;
    }

    // Apply the settings to the synth
    if (g_current_synth) {
        g_current_synth->activateFilter(s_global_filter_cutoff, s_global_filter_resonance);
        ESP_LOGD(TAG_FILTER, "Filter reset to lowpass: Cutoff=%d, Resonance=%d",
                 s_global_filter_cutoff, s_global_filter_resonance);
    }
}