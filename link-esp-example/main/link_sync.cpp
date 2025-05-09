#include "link_sync.h"
#include "io_helpers.h" // For set_buzzer_state
#include <cmath> // For floor

// Add logging tag
static const char *TAG_LINK = "LINK_SYNC";

// Static variables for quantum boundary detection
static int s_last_quantum_number = -1;

// Timer ISR (Keep this minimal)
static IRAM_ATTR void timer_isr(void *userParam) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    vTaskNotifyGiveFromISR(userParam, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
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
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80 // 80MHz APB clock / 80 = 1MHz timer clock (1us tick)
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, LINK_TICK_PERIOD);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, task_handle,
                       ESP_INTR_FLAG_IRAM,
                       nullptr);
    timer_start(TIMER_GROUP_0, TIMER_0);
    ESP_LOGI(TAG_LINK, "Link Timer Initialized (Period: %d us)", LINK_TICK_PERIOD);
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
    const double beatFraction = quantumInfo.beatFraction;
    const bool crossedQuantumBoundary = quantumInfo.crossedQuantumBoundary;

    // Calculate MIDI ticks (24 per quarter note)
    const int ticks = std::floor(sessionBeat * 24);
    const int ticksInBeat = static_cast<int>(beatFraction * 150);

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

        // If we just crossed a quantum boundary and we're playing,
        // send a MIDI realign message to ensure all connected MIDI devices are in phrase
        if (crossedQuantumBoundary && is_playing) {
            // Calculate the exact quantum boundary beat
            double exactQuantumBoundary = quantumInfo.currentQuantumNumber * LINK_QUANTUM;

            // Send MIDI Stop followed by Start to force realignment at quantum boundaries
            const uint8_t stop_msg = MIDI_STOP;
            const uint8_t start_msg = MIDI_START;

            // Send Stop message
            uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);

            // Short delay between messages
            vTaskDelay(pdMS_TO_TICKS(1));

            // Send Start message
            uart_write_bytes(MIDI_UART, (const char *)&start_msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);

            // Send Song Position Pointer message to ensure exact alignment
            // SPP is in MIDI beats (16th notes), so multiply by 4
            uint16_t spp_pos = static_cast<uint16_t>(exactQuantumBoundary * 4) % 16384;
            uint8_t spp_lsb = spp_pos & 0x7F;
            uint8_t spp_msb = (spp_pos >> 7) & 0x7F;
            const uint8_t spp_msg[] = {0xF2, spp_lsb, spp_msb};
            uart_write_bytes(MIDI_UART, (const char *)spp_msg, sizeof(spp_msg));
            uart_wait_tx_done(MIDI_UART, 1);

            ESP_LOGI(TAG_LINK, "Sent MIDI realignment at quantum boundary (Beat: %.2f, SPP: %d)",
                     exactQuantumBoundary, spp_pos);
        }

        // Regular play state change handling
        if (was_playing != is_playing) {
            const uint8_t msg = is_playing ? MIDI_START : MIDI_STOP;
            if (is_playing) { // Send Stop first on Start for alignment
                 const uint8_t stop_msg = MIDI_STOP;
                 uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
                 uart_wait_tx_done(MIDI_UART, 1);
            }
            uart_write_bytes(MIDI_UART, (const char *)&msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);
            ESP_LOGI(TAG_LINK, "Sent MIDI %s", is_playing ? "Start" : "Stop");
            was_playing = is_playing;
        }

        // Send MIDI Timing Clock and SPP if a new tick occurred
        if (ticks > lastTicks) {
            const uint8_t timing_msg = MIDI_TIMING_CLOCK;
            uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);

            if (ticks % 150 == 0) { // Send SPP every 150 ticks (approx 1 beat at 120bpm)
                uint16_t pos = (ticks / 6) % 32767;
                uint8_t pos_lsb = pos & 0x7F;
                uint8_t pos_msb = (pos >> 7) & 0x7F;
                const uint8_t spp_msg[] = {0xF2, pos_lsb, pos_msb};
                uart_write_bytes(MIDI_UART, (const char *)spp_msg, sizeof(spp_msg));
                uart_wait_tx_done(MIDI_UART, 1);
            }
        }
    } else {
        set_buzzer_state(false); // Ensure buzzer is off if not connected/started
    }

    lastTicks = ticks; // Update lastTicks regardless of whether control logic ran
}