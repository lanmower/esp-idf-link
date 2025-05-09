//idf 4.4.4
#include "main.h"
#include "io_helpers.h"
#include "link_sync.h"
#include "input_handler.h"
#include "effect_handler.h"
#include "state_machine.h" // Include the new state machine header
#include "synth_interface.h"
#include "synth_mininova.h" // Include the implementation header
#include "synth_microkorg.h" // Include the MicroKorg header
#include "esp_log.h" // Include logging header
#include <stdio.h>
// Effect headers are NOT needed here anymore, included via state_machine.cpp
// #include "effect_arp.h"
// #include "effect_sidechain.h"
// #include "effect_filter.h"

// Define the logging tag for this file
static const char *TAG = "MAIN";

// --- Global Variable Definitions ---
// Synth Interface Pointer
SynthInterface* g_current_synth = nullptr;

// Add a variable to track which synth is active
// enum SynthType { SYNTH_MININOVA, SYNTH_MICROKORG }; // Now declared in main.h
SynthType g_synth_type = SYNTH_MININOVA; // Removed static

// Link Object
std::unique_ptr<ableton::Link> g_link;

// Control States (Potentiometers & Touch Pads)
// int current_pot_val[2] = {0, 0};
// bool current_touch_state[4] = {false, false, false, false};
// int last_pot_val[2] = {-1, -1};
// bool last_touch_state[4] = {false, false, false, false};
// bool previous_touch_state[4] = {false, false, false, false};

// Interaction State Variables
// bool touch_held_state[4] = {false, false, false, false};
// uint64_t pad_press_time[4] = {0, 0, 0, 0};
// uint64_t pad_release_time[4] = {0, 0, 0, 0};
// bool pot1_moved_while_held = false;
// bool pot2_moved_while_held = false;

// Double Tap Timing Constants (Defined in main.h, used across handlers)
const uint64_t DOUBLE_TAP_TIME_MS = 300;
const uint64_t HOLD_TIME_MS = 200;

// MIDI Send Optimization State
// (Defined in respective .cpp files where they are exclusively used,
// e.g., sidechain state in effect_sidechain.cpp)

// --- tickTask --- (Main application loop)
void tickTask(void *userParam) {
    // --- Initialization ---
    init_uart_midi();
    init_adc();
    init_touch_pads();
    setup_buzzer();

    // Initialize the Synth Interface (Mininova implementation)
    // Use MIDI channel 1 by default
    g_current_synth = new SynthMininova(1);
    g_synth_type = SYNTH_MININOVA;
    if (!g_current_synth) {
        ESP_LOGE(TAG, "Failed to initialize synth interface!");
        // Handle error appropriately - maybe halt or enter error state
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    ESP_LOGI(TAG, "Synth Interface Initialized (Mininova)");

    // Initialize Inputs (Reads initial state)
    initialize_inputs();

    // Initialize Effects (Resets latches, pot params etc)
    initialize_effects();

    TaskHandle_t current_task_handle = xTaskGetCurrentTaskHandle();
    init_link_timer(current_task_handle);

    // Initialize Link object with default tempo
    g_link = std::make_unique<ableton::Link>(120.0);

    // Enable Link and set quantum for proper phrase alignment
    g_link->enable(true);

    // Capture the initial session state
    auto sessionState = g_link->captureAppSessionState();

    // Force the initial beat to align with quantum boundaries
    // This ensures all systems start in phrase with the quantum
    const auto time = g_link->clock().micros();
    const double initialBeat = sessionState.beatAtTime(time, LINK_QUANTUM);
    const double initialPhase = sessionState.phaseAtTime(time, LINK_QUANTUM);

    // Calculate the nearest quantum boundary
    const double quantumBoundary = std::floor(initialBeat / LINK_QUANTUM) * LINK_QUANTUM;

    // Force the beat to the quantum boundary
    sessionState.forceBeatAtTime(quantumBoundary, time, LINK_QUANTUM);

    // Commit the changes back to Link
    g_link->commitAppSessionState(sessionState);

    ESP_LOGI(TAG, "Link initialized and aligned to quantum boundary: %.1f (from beat %.2f, phase %.2f)",
             quantumBoundary, initialBeat, initialPhase);

    ESP_LOGI(TAG, "Initialization complete. Starting main loop.");

    // --- Link State Variables (Local to tickTask scope, passed by ref to handle_link_sync) ---
    bool was_connected = false;
    int64_t start_wait_time = esp_timer_get_time();
    bool force_start = false;
    static int lastTicks = 0;
    static int length = LENGTH_NORMAL;
    static int lastBeat = -1;
    static int currentBuzzerFreq = FREQ_NORMAL;
    static bool was_playing = false;

    // Input Event struct to hold current inputs
    InputEvent current_input_event;

    // --- Main Loop ---
    while (true) {
        // Wait for timer ISR notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // --- START TIMING CRITICAL SECTION ---
        const auto time = g_link->clock().micros();
        const auto state = g_link->captureAppSessionState();

        // Cache previous touch state (needed by control logic)
        // memcpy(previous_touch_state, last_touch_state, sizeof(last_touch_state));

        // Handle Link connection, metronome, MIDI clock/sync
        // Pass link state variables by reference
        int current_ticks_local = lastTicks; // Use local copy to check if tick advanced
        handle_link_sync(was_connected, start_wait_time, force_start,
                         current_ticks_local, length, lastBeat, currentBuzzerFreq, was_playing,
                         state, time);
        // --- END TIMING CRITICAL SECTION ---

        // --- Process Controls and State Machine ---
        // Read all inputs into the event struct
        read_inputs(current_input_event);

        // Process the input event through the state machine
        // The state machine now handles calling effect logic (active, adjust pots, secondary taps)
        // We can optionally only call this if input changed OR time advanced, but calling always is safer
        // if (input_changed || current_ticks_local > lastTicks) {
        process_state_event(current_input_event, state, time);
        // }

        // --- Synth switching logic ---
        // MOVED to handle_control_logic in effect_handler.cpp

        lastTicks = current_ticks_local; // Update the static lastTicks

    } // End while(true)
} // End tickTask


// --- app_main --- (Entry point)
extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Increased stack size for safety after adding complexity
    xTaskCreate(tickTask, "tickTask", 10240, nullptr, 10, nullptr);

    ESP_LOGI(TAG, "app_main finished, tickTask running.");
}

