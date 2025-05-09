#include "input_handler.h"
#include "io_helpers.h" // Assuming low-level ADC/Touch reads are here or included by it
#include "esp_log.h"
#include <cmath> // For std::abs
#include "link_sync.h"
#include "effect_handler.h"
#include "touch_handler.h" // Include for NUM_TOUCH_PADS
#include "main.h"          // Include for NUM_POTS, NUM_TOUCH_PADS etc.
#include "esp_timer.h"     // For esp_timer_get_time()

// Threshold for pot movement detection
// const int POT_MOVE_THRESHOLD = 2; // REMOVED - Not used with jitter threshold logic

// Jitter threshold for pots (applied against stable center)
const int POT_JITTER_THRESHOLD = 3; // Ignore changes if current value is within this distance of stable center

// Variables to store previous states
static int s_previous_pot_val[NUM_POTS] = {-1, -1}; // Tracks the *last reported* value for delta calculation
static bool s_previous_touch_state[NUM_TOUCH_PADS] = {false, false, false, false};

// --- Initialization ---
void initialize_inputs() {
    ESP_LOGI("Input", "Initializing inputs...");
    InputEvent initial_event; // Use InputEvent struct
    // Initialize dummy values if needed for the first call
    // ... (previous values are static, initialized above)

    // Call read_inputs once to populate initial states
    read_inputs(initial_event);

    // Store the initial values read by read_controls (which is called by read_inputs)
    for (int i = 0; i < NUM_POTS; ++i) {
        s_previous_pot_val[i] = initial_event.pot_value[i];
    }
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        s_previous_touch_state[i] = initial_event.pad_held[i];
    }

    ESP_LOGI("Input", "Inputs initialized. Initial Pot0: %d, Pot1: %d", s_previous_pot_val[0], s_previous_pot_val[1]);
}

// --- Read All Inputs --- (Modified Signature)
// Reads physical inputs, populates InputEvent struct.
// Returns true if any significant change occurred, false otherwise.
bool read_inputs(InputEvent& current_event) // Changed signature
{
    // static const char* TAG = "ReadInputs"; // Unused variable warning
    int current_pot_val[NUM_POTS];
    int current_stable_center[NUM_POTS];
    bool current_touch_state[NUM_TOUCH_PADS];
    bool current_pad_pressed_this_tick[NUM_TOUCH_PADS]; // Local variable for read_controls output

    // Get current timestamp
    current_event.timestamp_us = esp_timer_get_time();

    // Call the low-level read_controls function
    bool controls_changed_raw = read_controls(
        current_pot_val,           // Raw (well, smoothed) pot values 0-127
        current_stable_center,     // Stable center values 0-127
        current_touch_state,       // Current touch sensor ON/OFF state
        current_pad_pressed_this_tick, // Output: true if pad JUST pressed this tick
        s_previous_pot_val,        // Input: Last *reported* pot values
        s_previous_touch_state     // Input: Last touch sensor state
    );

    bool significant_change_detected = false;

    // --- Process Pots with improved sensitivity for slow movements ---
    for (int i = 0; i < NUM_POTS; ++i) {
        // Track cumulative movement since last reported value
        int cumulative_delta = current_pot_val[i] - s_previous_pot_val[i];

        // Check if value has changed at all from the last reported value
        // This makes the control more responsive to slow movements
        if (cumulative_delta != 0) {
            // Only apply jitter threshold to small movements near the stable center
            bool is_near_center = std::abs(current_pot_val[i] - current_stable_center[i]) <= POT_JITTER_THRESHOLD;

            // If we're not near center OR the cumulative change is significant, report the movement
            if (!is_near_center || std::abs(cumulative_delta) > 1) {
                current_event.pot_moved[i] = true;
                current_event.pot_delta[i] = cumulative_delta;
                current_event.pot_value[i] = current_pot_val[i];
                s_previous_pot_val[i] = current_pot_val[i]; // Update last reported value
                significant_change_detected = true;

                ESP_LOGV("INPUT", "Pot %d moved: val=%d, delta=%d, near_center=%s",
                         i, current_pot_val[i], cumulative_delta, is_near_center ? "YES" : "NO");
            } else {
                // Small movement near center - ignore to prevent jitter
                current_event.pot_moved[i] = false;
                current_event.pot_delta[i] = 0;
                current_event.pot_value[i] = s_previous_pot_val[i];
            }
        } else {
            // No change from last reported value
            current_event.pot_moved[i] = false;
            current_event.pot_delta[i] = 0;
            current_event.pot_value[i] = s_previous_pot_val[i];
        }
    }

    // --- Process Touch Pads ---
    // Direct pass-through of touch states from read_controls
    // The debouncing and state tracking is now handled entirely in read_controls

    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        // Update the held state directly from read_controls
        current_event.pad_held[i] = current_touch_state[i];

        // Update the pressed_this_tick state directly from read_controls
        current_event.pad_pressed_this_tick[i] = current_pad_pressed_this_tick[i];

        // If either state is true, consider it a significant change
        if (current_event.pad_held[i] != s_previous_touch_state[i] || current_event.pad_pressed_this_tick[i]) {
            significant_change_detected = true;
            ESP_LOGD("INPUT", "Pad %d state: %s%s",
                    i,
                    current_event.pad_held[i] ? "HELD" : "RELEASED",
                    current_event.pad_pressed_this_tick[i] ? " (JUST PRESSED)" : "");
        }

        // Update the previous state for the next call
        s_previous_touch_state[i] = current_touch_state[i];
    }

    // Return true if any pot moved significantly OR any touch pad state changed
    return significant_change_detected;
}

/* // OLD read_inputs function - REMOVE
void read_inputs(
    bool pad_pressed_this_tick[4], // Output: True if pad just pressed
    bool pad_held[4],             // Output: True if pad is currently held
    bool& pot1_moved,             // Output: True if pot 1 moved significantly
    int& pot1_value,              // Output: Current value of pot 1 (0-127)
    int& pot1_delta,              // Output: Change in pot 1 value since last read
    bool& pot2_moved,             // Output: True if pot 2 moved significantly
    int& pot2_value,              // Output: Current value of pot 2 (0-127)
    int& pot2_delta               // Output: Change in pot 2 value since last read
) {
    // ... (Implementation removed) ...
}
*/