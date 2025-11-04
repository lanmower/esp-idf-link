#include "link_sync.h"
#include "io_helpers.h" // For set_buzzer_state
#include <cmath> // For floor
#include <driver/gptimer.h>
#include "esp_rom_sys.h" // For esp_rom_delay_us

// Add logging tag
static const char *TAG_LINK = "LINK_SYNC";

// Static variables for quantum boundary detection
static int s_last_quantum_number = -1;
static gptimer_handle_t s_link_gptimer = nullptr;
static gptimer_handle_t s_midi_note_gptimer = nullptr;

static bool IRAM_ATTR link_gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Use 1 as notification value for the main timer
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(user_data), 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

static bool IRAM_ATTR midi_note_gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_data) {
    // Add specialized handling for MIDI note processing
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Use 2 as notification value to distinguish from the main timer which uses 1
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(user_data), 2, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    
    return xHigherPriorityTaskWoken == pdTRUE;
}

// Common function to detect quantum boundaries and calculate phase information
// This ensures all effects stay in phrase with Link
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time) {
    QuantumInfo info;

    // Get the current beat position in the session timeline
    info.sessionBeat = state.beatAtTime(time, LINK_QUANTUM);

    // Calculate phase within the quantum (0.0 to LINK_QUANTUM)
    // This ensures the phase is always aligned with quantum boundaries
    info.phaseWithinQuantum = state.phaseAtTime(time, LINK_QUANTUM);

    // Calculate the current quantum number (which quantum we're in)
    info.currentQuantumNumber = static_cast<int>(std::floor(info.sessionBeat / LINK_QUANTUM));

    // Calculate the exact quantum boundary beat
    double exactQuantumBoundary = info.currentQuantumNumber * LINK_QUANTUM;

    // Calculate how far we are from the quantum boundary in beats
    double beatsFromBoundary = info.sessionBeat - exactQuantumBoundary;

    // Detect quantum boundary crossing with improved precision
    // We consider a boundary crossed if:
    // 1. The quantum number changed, OR
    // 2. We're very close to the boundary (within 0.01 beats) coming from the previous quantum
    bool nearBoundary = (beatsFromBoundary < 0.01) && (beatsFromBoundary >= 0.0);
    info.crossedQuantumBoundary = ((info.currentQuantumNumber != s_last_quantum_number && s_last_quantum_number != -1) ||
                                  (nearBoundary && info.currentQuantumNumber > s_last_quantum_number));

    // Update the static last quantum number
    if (info.crossedQuantumBoundary || info.currentQuantumNumber > s_last_quantum_number) {
        s_last_quantum_number = info.currentQuantumNumber;
    }

    // Calculate the current beat within the quantum
    int currentBeat = static_cast<int>(std::floor(info.phaseWithinQuantum));
    info.beatInQuantum = currentBeat % static_cast<int>(LINK_QUANTUM);

    // Calculate the fraction of the current beat (0.0 to 1.0)
    info.beatFraction = info.phaseWithinQuantum - std::floor(info.phaseWithinQuantum);

    // Log quantum boundary crossing with more detailed information
    if (info.crossedQuantumBoundary) {
        ESP_LOGI(TAG_LINK, "Quantum boundary crossed: %d, Beat: %.2f, Phase: %.2f, BeatsFromBoundary: %.4f",
                 info.currentQuantumNumber, info.sessionBeat, info.phaseWithinQuantum, beatsFromBoundary);

        // If we're near but not exactly at the boundary, force alignment
        if (beatsFromBoundary > 0.001) {
            ESP_LOGW(TAG_LINK, "Not precisely at quantum boundary - consider realignment");
        }
    }

    return info;
}

// Initialize Timer for Link Task
void init_link_timer(TaskHandle_t task_handle) {
    // Zero-initialize the struct to catch all members, including those in unnamed structs
    gptimer_config_t timer_config = {}; 
    timer_config.clk_src = GPTIMER_CLK_SRC_APB;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1 MHz = 1us tick
    timer_config.intr_priority = 3;       // Higher priority for metronome timer
    timer_config.flags.intr_shared = 0; // Assuming timer interrupt is not shared
    // timer_config.flags.allow_pd and timer_config.flags.backup_before_sleep will be zero-initialized

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_link_gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = link_gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_link_gptimer, &cbs, task_handle));

    ESP_ERROR_CHECK(gptimer_set_raw_count(s_link_gptimer, 0));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = LINK_TICK_PERIOD,
        .reload_count = 0,  // For periodic, set reload_count to 0 and use flags
        .flags = {
            .auto_reload_on_alarm = 1 // Enable auto-reload
        }
    };
    ESP_ERROR_CHECK(::gptimer_set_alarm_action(s_link_gptimer, &alarm_config)); // Correct function name
    ESP_ERROR_CHECK(gptimer_enable(s_link_gptimer));
    ESP_ERROR_CHECK(gptimer_start(s_link_gptimer));
    ESP_LOGI(TAG_LINK, "Link GPTimer Initialized (Period: %d us)", LINK_TICK_PERIOD);
    
    // Initialize the MIDI note processing timer with higher frequency
    gptimer_config_t midi_timer_config = {};
    midi_timer_config.clk_src = GPTIMER_CLK_SRC_APB;
    midi_timer_config.direction = GPTIMER_COUNT_UP;
    midi_timer_config.resolution_hz = 1000000; // 1 MHz = 1us tick
    midi_timer_config.intr_priority = 1;       // Lower priority than metronome timer
    midi_timer_config.flags.intr_shared = 0;
    
    ESP_ERROR_CHECK(gptimer_new_timer(&midi_timer_config, &s_midi_note_gptimer));
    
    gptimer_event_callbacks_t midi_cbs = {
        .on_alarm = midi_note_gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_midi_note_gptimer, &midi_cbs, task_handle));
    
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_midi_note_gptimer, 0));
    gptimer_alarm_config_t midi_alarm_config = {
        .alarm_count = MIDI_PROCESS_PERIOD,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = 1
        }
    };
    ESP_ERROR_CHECK(::gptimer_set_alarm_action(s_midi_note_gptimer, &midi_alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_midi_note_gptimer));
    ESP_ERROR_CHECK(gptimer_start(s_midi_note_gptimer));
    ESP_LOGI(TAG_LINK, "MIDI Note GPTimer Initialized (Period: %d us)", MIDI_PROCESS_PERIOD);
}

// Main Link Synchronization Logic (called from tickTask)
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                        int& lastTicks, int& length, int& lastBeat, int& currentBuzzerFreq, bool& was_playing,
                        const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    // Check peer status & force start timeout
    bool is_connected = g_link->numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= 3000000)) {
        force_start = true;
        ESP_LOGW(TAG_LINK, "No Link peers found for 60s, forcing start.");
    }

    // Handle connection changes (send MIDI Stop/Start)
    if (is_connected != was_connected) {
        ESP_LOGI(TAG_LINK, "Link peers changed: %d", g_link->numPeers());
        if (is_connected) {
            const uint8_t stop_msg[] = {MIDI_STOP};
            uart_write_bytes(MIDI_UART, (const char *)stop_msg, 1);
            uart_wait_tx_done(MIDI_UART, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(1));
            const uint8_t start_msg[] = {MIDI_START};
            uart_write_bytes(MIDI_UART, (const char *)start_msg, 1);
            uart_wait_tx_done(MIDI_UART, portMAX_DELAY);
            ESP_LOGI(TAG_LINK, "Sent MIDI Stop/Start on Link connection.");
        }
        was_connected = is_connected;
    }

    // Get quantum boundary and phase information using the common function
    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    // Extract values from the quantum info for use in this function
    const double sessionBeat = quantumInfo.sessionBeat;
    const double phase = quantumInfo.phaseWithinQuantum;
    const int currentBeat = static_cast<int>(std::floor(phase));
    const int beatInQuantum = quantumInfo.beatInQuantum;
    const bool crossedQuantumBoundary = quantumInfo.crossedQuantumBoundary;

    // Calculate MIDI ticks (24 per quarter note)
    const int ticks = std::floor(sessionBeat * 24);

    // If we crossed a quantum boundary, force realignment of the session state
    // This ensures all systems stay perfectly in phrase with the quantum
    if (crossedQuantumBoundary) {
        // Calculate the exact quantum boundary beat
        double exactQuantumBoundary = quantumInfo.currentQuantumNumber * LINK_QUANTUM;

        // Create a copy of the session state for modification
        auto sessionStateCopy = state;

        // Force the beat to the exact quantum boundary
        // This ensures all systems are perfectly aligned with the quantum
        sessionStateCopy.forceBeatAtTime(exactQuantumBoundary, time, LINK_QUANTUM);

        // We can't commit the session state here because it's a const reference
        // But this calculation ensures our phase calculations are aligned with quantum boundaries

        ESP_LOGD(TAG_LINK, "Forced quantum alignment at boundary: %.2f", exactQuantumBoundary);
    }

    // Detect beat boundary crossing
    bool crossedBeat = (currentBeat != lastBeat);

    // Metronome and MIDI Sync Logic
    if (is_connected || force_start) {
        // Calculate the exact position within the quantum for precise timing
        double exactPositionInQuantum = phase;
        double exactBeatInQuantum = std::floor(exactPositionInQuantum);
        double exactFractionOfBeat = exactPositionInQuantum - exactBeatInQuantum;

        // If we crossed a quantum boundary, force a reset of the metronome state
        if (crossedQuantumBoundary) {
            // Special handling for quantum boundary - use the highest tone
            length = LENGTH_16BEAT;
            currentBuzzerFreq = FREQ_16BEAT;
            lastBeat = currentBeat;

            // Force the buzzer on at quantum boundaries for better alignment
            set_buzzer_state(true, currentBuzzerFreq);

            ESP_LOGI(TAG_LINK, "Metronome reset at quantum boundary (Beat: %.2f, Phase: %.2f)",
                     sessionBeat, phase);

            // Force realignment of all timed systems at quantum boundaries
            // This is a good place to add any additional quantum boundary synchronization
        }
        else if (crossedBeat) {
            // Update metronome beep length/frequency based on beat position within quantum
            // Use exact beat position for more precise timing
            int exactBeatPosition = static_cast<int>(exactBeatInQuantum);

            // Determine the importance of this beat within the quantum
            if (exactBeatPosition % static_cast<int>(LINK_QUANTUM) == 0) {
                length = LENGTH_16BEAT;
                currentBuzzerFreq = FREQ_16BEAT;
            }
            else if (exactBeatPosition % 8 == 0) {
                length = LENGTH_8BEAT;
                currentBuzzerFreq = FREQ_8BEAT;
            }
            else if (exactBeatPosition % 4 == 0) {
                length = LENGTH_4BEAT;
                currentBuzzerFreq = FREQ_4BEAT;
            }
            else {
                length = LENGTH_NORMAL;
                currentBuzzerFreq = FREQ_NORMAL;
            }

            lastBeat = currentBeat;

            ESP_LOGD(TAG_LINK, "Beat crossed: %d (in quantum: %d), Exact: %.4f",
                     currentBeat, beatInQuantum, exactPositionInQuantum);
        }

        // Control metronome buzzer with precise timing
        // Only if we didn't just cross a quantum boundary (which has its own buzzer logic)
        if (!crossedQuantumBoundary) {
            // Use exact fraction of beat for more precise timing
            bool shouldPlay = exactFractionOfBeat < (static_cast<double>(length) / 150.0);
            set_buzzer_state(shouldPlay, currentBuzzerFreq);
        }

        // Handle MIDI Start/Stop based on Link play state
        bool is_playing = state.isPlaying();

        // If we crossed a quantum boundary and we're playing,
        // send a MIDI realign message to ensure all connected MIDI devices are in phrase
        if (crossedQuantumBoundary && is_playing) {
            // Calculate the exact quantum boundary beat
            double exactQuantumBoundary = quantumInfo.currentQuantumNumber * LINK_QUANTUM;

            // Send MIDI Stop followed by Start to force realignment at quantum boundaries
            const uint8_t stop_msg = MIDI_STOP;
            const uint8_t start_msg = MIDI_START;

            // Send Stop message - PERFORMANCE: Use non-blocking UART write
            uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
            
            // Short delay between messages, but non-blocking
            esp_rom_delay_us(500); // microsecond delay instead of vTaskDelay

            // Send Start message - PERFORMANCE: Use non-blocking UART write
            uart_write_bytes(MIDI_UART, (const char *)&start_msg, 1);

            // Send Song Position Pointer message to ensure exact alignment
            // SPP is in MIDI beats (16th notes), so multiply by 4
            uint16_t spp_pos = static_cast<uint16_t>(exactQuantumBoundary * 4) % 16384;
            uint8_t spp_lsb = spp_pos & 0x7F;
            uint8_t spp_msb = (spp_pos >> 7) & 0x7F;
            const uint8_t spp_msg[] = {0xF2, spp_lsb, spp_msb};
            uart_write_bytes(MIDI_UART, (const char *)spp_msg, sizeof(spp_msg));

            ESP_LOGI(TAG_LINK, "Sent MIDI realignment at quantum boundary (Beat: %.2f, SPP: %d)",
                     exactQuantumBoundary, spp_pos);
        }

        // Regular play state change handling
        if (was_playing != is_playing) {
            const uint8_t msg = is_playing ? MIDI_START : MIDI_STOP;
            if (is_playing) { // Send Stop first on Start for alignment
                 const uint8_t stop_msg = MIDI_STOP;
                 uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
                 // PERFORMANCE: Use non-blocking delay
                 esp_rom_delay_us(500);
            }
            uart_write_bytes(MIDI_UART, (const char *)&msg, 1);
            ESP_LOGI(TAG_LINK, "Sent MIDI %s", is_playing ? "Start" : "Stop");
            was_playing = is_playing;
        }

        // Send MIDI Timing Clock and SPP if a new tick occurred
        // Send MIDI clock on every tick to maintain proper tempo
        if (ticks > lastTicks) {
            const uint8_t timing_msg = MIDI_TIMING_CLOCK;
            uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);

            // PERFORMANCE: Reduce SPP frequency - send it less often
            if (ticks % 600 == 0) { // Reduced from 150 to 600 ticks (4x less frequent)
                uint16_t pos = (ticks / 6) % 32767;
                uint8_t pos_lsb = pos & 0x7F;
                uint8_t pos_msb = (pos >> 7) & 0x7F;
                const uint8_t spp_msg[] = {0xF2, pos_lsb, pos_msb};
                uart_write_bytes(MIDI_UART, (const char *)spp_msg, sizeof(spp_msg));
            }
        }
    } else {
        set_buzzer_state(false); // Ensure buzzer is off if not connected/started
    }

    lastTicks = ticks; // Update lastTicks regardless of whether control logic ran
}