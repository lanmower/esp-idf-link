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

// Simple quantum boundary detection using phase reset
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time) {
    QuantumInfo info;

    info.sessionBeat = state.beatAtTime(time, LINK_QUANTUM);
    info.phaseWithinQuantum = state.phaseAtTime(time, LINK_QUANTUM);
    info.currentQuantumNumber = static_cast<int>(std::floor(info.sessionBeat / LINK_QUANTUM));
    info.beatInQuantum = static_cast<int>(std::floor(info.phaseWithinQuantum));
    info.beatFraction = info.phaseWithinQuantum - std::floor(info.phaseWithinQuantum);

    // Simple crossing detection: quantum number changed
    info.crossedQuantumBoundary = (info.currentQuantumNumber != s_last_quantum_number && s_last_quantum_number != -1);

    if (info.crossedQuantumBoundary) {
        s_last_quantum_number = info.currentQuantumNumber;
        ESP_LOGI(TAG_LINK, "Quantum boundary %d, beat %.2f", info.currentQuantumNumber, info.sessionBeat);
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

    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    const double sessionBeat = quantumInfo.sessionBeat;
    const double phase = quantumInfo.phaseWithinQuantum;
    const int beatInQuantum = quantumInfo.beatInQuantum;
    const bool crossedQuantumBoundary = quantumInfo.crossedQuantumBoundary;

    // Calculate MIDI timing clocks (24 per quarter note)
    const int midiClocks = static_cast<int>(sessionBeat * 24);

    // Detect beat boundary crossing
    bool crossedBeat = (beatInQuantum != lastBeat);

    // Metronome and MIDI Sync Logic
    if (is_connected || force_start) {
        // Metronome frequency based on beat position (emphasize 16, 8, 4, 1)
        if (crossedQuantumBoundary) {
            length = LENGTH_16BEAT;
            currentBuzzerFreq = FREQ_16BEAT;
            ESP_LOGI(TAG_LINK, "Quantum boundary at beat %.1f", sessionBeat);
        } else if (crossedBeat) {
            if (beatInQuantum % 8 == 0) {
                length = LENGTH_8BEAT;
                currentBuzzerFreq = FREQ_8BEAT;
            } else if (beatInQuantum % 4 == 0) {
                length = LENGTH_4BEAT;
                currentBuzzerFreq = FREQ_4BEAT;
            } else {
                length = LENGTH_NORMAL;
                currentBuzzerFreq = FREQ_NORMAL;
            }
        }

        if (crossedBeat || crossedQuantumBoundary) {
            lastBeat = beatInQuantum;
        }

        // Buzzer timing: play for 'length' milliseconds at beat boundary
        bool shouldPlay = (phase - beatInQuantum) < (static_cast<double>(length) / 1000.0);
        set_buzzer_state(shouldPlay, currentBuzzerFreq);

        bool is_playing = state.isPlaying();

        // Send MIDI Start/Stop when play state changes
        if (was_playing != is_playing) {
            const uint8_t msg = is_playing ? MIDI_START : MIDI_STOP;
            uart_write_bytes(MIDI_UART, (const char *)&msg, 1);
            ESP_LOGI(TAG_LINK, "MIDI %s at beat %.1f", is_playing ? "START" : "STOP", sessionBeat);
            was_playing = is_playing;
        }

        // Send MIDI Timing Clock (24 per quarter note)
        if (midiClocks > lastTicks) {
            const uint8_t timing_msg = MIDI_TIMING_CLOCK;
            uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);

            // Send Song Position Pointer periodically (every 4 beats to avoid spam)
            // SPP = beat position * 4 (since SPP is in sixteenth notes)
            if (beatInQuantum % 4 == 0 && crossedBeat) {
                uint16_t spp_beats = static_cast<uint16_t>(sessionBeat * 4) & 0x3FFF;
                uint8_t spp_lsb = spp_beats & 0x7F;
                uint8_t spp_msb = (spp_beats >> 7) & 0x7F;
                const uint8_t spp_msg[] = {0xF2, spp_lsb, spp_msb};
                uart_write_bytes(MIDI_UART, (const char *)spp_msg, sizeof(spp_msg));
            }
        }
    } else {
        set_buzzer_state(false);
    }

    lastTicks = midiClocks;
}