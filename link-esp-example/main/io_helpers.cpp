#include "io_helpers.h"
#include <cmath> // Include for pow, round
#include <algorithm> // Include for std::min, std::max
#include "esp_log.h" // Include logging header

// Define the logging tag for this file
static const char *TAG_IO = "IO_HELPERS";

// Initialize MIDI UART
void init_uart_midi() {
    uart_config_t uart_config = {
        .baud_rate = 31250,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB
    };
    uart_param_config(MIDI_UART, &uart_config);
    uart_set_pin(MIDI_UART, MIDI_TX_PIN, MIDI_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MIDI_UART, 512, 0, 0, NULL, 0);
    ESP_LOGI(TAG_IO, "MIDI UART Initialized (TX:%d, RX:%d)", MIDI_TX_PIN, MIDI_RX_PIN);
}

// Initialize ADC for Potentiometers
void init_adc() {
    ESP_LOGI(TAG_IO, "Initializing ADC1...");
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(POT_ADC_CHANNEL_1, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(POT_ADC_CHANNEL_2, ADC_ATTEN));
}

// Initialize Touch Pads
void init_touch_pads() {
    ESP_LOGI(TAG_IO, "Initializing Touch Pad...");
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));

    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_1, TOUCH_THRESHOLD));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_ARP, TOUCH_THRESHOLD));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_REV, TOUCH_THRESHOLD));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_FILT, TOUCH_THRESHOLD));

    ESP_LOGI(TAG_IO, "Touch pads initialized for direct reading");
}

// Setup Buzzer PWM
void setup_buzzer() {
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = FREQ_NORMAL,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG_IO, "Buzzer PWM Initialized (GPIO:%d)", BUZZER);
}

// Control Buzzer State
void set_buzzer_state(bool on, uint32_t frequency) {
    static uint32_t lastFreq = 0;
    if (on) {
        if (frequency != lastFreq) {
             // Only reconfigure timer if frequency actually changed
            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_MODE,
                .duty_resolution = LEDC_DUTY_RES,
                .timer_num = LEDC_TIMER,
                .freq_hz = frequency,
                .clk_cfg = LEDC_AUTO_CLK
            };
            esp_err_t err = ledc_timer_config(&ledc_timer);
            if (err == ESP_OK) { // Update lastFreq only on success
                 lastFreq = frequency;
            } else {
                 ESP_LOGE(TAG_IO, "Error setting buzzer frequency: %d", frequency);
            }
        }
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        if (lastFreq != 0) { // Only turn off if it was potentially on
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            lastFreq = 0; // Mark as off
        }
    }
}

// --- NEW: Dynamic Range Scaling Function ---
// Scales a value within the observed min/max range to 0-127 using an exponential curve.
int scale_pot_value(int value, int min_observed, int max_observed, double exponent) {
    // If the observed range is invalid or zero, return the middle value
    if (min_observed >= max_observed) {
        return 64;
    }

    double input_range = static_cast<double>(max_observed - min_observed);
    double value_double = static_cast<double>(value);

    // Normalize the input value to the range [0.0, 1.0]
    double normalized_input = (value_double - min_observed) / input_range;

    // Clamp normalized input to ensure it's within [0.0, 1.0]
    normalized_input = std::max(0.0, std::min(1.0, normalized_input));

    // Apply the exponential scaling
    double scaled_value_normalized = pow(normalized_input, exponent);

    // Scale to the output range [0, 127] and round
    int final_value = static_cast<int>(round(scaled_value_normalized * 127.0));

    // Clamp the final output to [0, 127]
    return std::max(0, std::min(127, final_value));
}

// Read Potentiometers and Touch Pads
bool read_controls(
    int pot_vals[NUM_POTS],
    int pot_stable_center[NUM_POTS],
    bool touch_state[NUM_TOUCH_PADS],
    bool pad_pressed_this_tick[NUM_TOUCH_PADS],
    int last_pot_vals[NUM_POTS],
    bool last_touch_state[NUM_TOUCH_PADS]
) {
    bool changed = false;
    const touch_pad_t touch_pads[] = {TOUCH_PAD_1, TOUCH_PAD_ARP, TOUCH_PAD_REV, TOUCH_PAD_FILT};

    // --- EMA Smoothing State ---
    static float smoothed_pot1_raw = -1.0f; // Use float for EMA calculation
    static float smoothed_pot2_raw = -1.0f;
    const float EMA_ALPHA = 0.08f; // Smoothing factor for main value
    // Second EMA for stable center tracking
    static float stable_center_pot1_raw = -1.0f;
    static float stable_center_pot2_raw = -1.0f;
    const float STABLE_CENTER_ALPHA = 0.01f; // Much slower smoothing

    // --- Dynamic Range Tracking --- // REMOVED
    // static int pot1_min_observed = 4095;
    // static int pot1_max_observed = 0;
    // static int pot2_rev_min_observed = 4095;
    // static int pot2_rev_max_observed = 0;

    // --- Scaling Exponents --- // REMOVED
    // const double POT1_EXPONENT = 2.0;
    // const double POT2_EXPONENT = 0.5;

    // Read Potentiometers
    int pot1_raw = adc1_get_raw(POT_ADC_CHANNEL_1);
    int pot2_raw = adc1_get_raw(POT_ADC_CHANNEL_2);

    // Initialize or apply EMA smoothing for main value
    if (smoothed_pot1_raw < 0.0f) { // Initialize on first run
        smoothed_pot1_raw = (float)pot1_raw;
        stable_center_pot1_raw = (float)pot1_raw; // Init stable center too
    } else {
        smoothed_pot1_raw = EMA_ALPHA * (float)pot1_raw + (1.0f - EMA_ALPHA) * smoothed_pot1_raw;
        stable_center_pot1_raw = STABLE_CENTER_ALPHA * (float)pot1_raw + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot1_raw;
    }
    if (smoothed_pot2_raw < 0.0f) { // Initialize on first run
        smoothed_pot2_raw = (float)pot2_raw;
        stable_center_pot2_raw = (float)pot2_raw; // Init stable center too
    } else {
        smoothed_pot2_raw = EMA_ALPHA * (float)pot2_raw + (1.0f - EMA_ALPHA) * smoothed_pot2_raw;
        stable_center_pot2_raw = STABLE_CENTER_ALPHA * (float)pot2_raw + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot2_raw;
    }

    // Convert smoothed values back to int for further processing
    int pot1_smoothed_int = (int)(smoothed_pot1_raw + 0.5f);
    int pot2_smoothed_int = (int)(smoothed_pot2_raw + 0.5f);
    // Convert stable center values
    int pot1_stable_center_int = (int)(stable_center_pot1_raw + 0.5f);
    int pot2_stable_center_int = (int)(stable_center_pot2_raw + 0.5f);

    // Reverse Pot 2 smoothed value // REMOVED
    // int pot2_smoothed_reversed = 4095 - pot2_smoothed_int;

    // --- Update Dynamic Range --- // REMOVED
    // pot1_min_observed = std::min(pot1_min_observed, pot1_smoothed_int);
    // pot1_max_observed = std::max(pot1_max_observed, pot1_smoothed_int);
    // pot2_rev_min_observed = std::min(pot2_rev_min_observed, pot2_smoothed_reversed);
    // pot2_rev_max_observed = std::max(pot2_rev_max_observed, pot2_smoothed_reversed);

    // ESP_LOGV(TAG_IO, "Pot1 Range: [%d, %d], Pot2Rev Range: [%d, %d]",
    //     pot1_min_observed, pot1_max_observed, pot2_rev_min_observed, pot2_rev_max_observed);

    // Apply NEW dynamic exponential scaling // REMOVED
    // int pot1_scaled_final = scale_pot_value(pot1_smoothed_int, pot1_min_observed, pot1_max_observed, POT1_EXPONENT);
    // int pot2_scaled_final = scale_pot_value(pot2_smoothed_reversed, pot2_rev_min_observed, pot2_rev_max_observed, POT2_EXPONENT);

    // --- Scale smoothed 0-4095 values linearly to 0-127 ---
    int pot1_output = static_cast<int>(round(((double)pot1_smoothed_int / 4095.0) * 127.0));
    int pot2_output = static_cast<int>(round(((double)pot2_smoothed_int / 4095.0) * 127.0));
    // Scale stable center values
    int pot1_stable_center_scaled = static_cast<int>(round(((double)pot1_stable_center_int / 4095.0) * 127.0));
    int pot2_stable_center_scaled = static_cast<int>(round(((double)pot2_stable_center_int / 4095.0) * 127.0));

    // Clamp final values just in case
    pot1_output = std::max(0, std::min(127, pot1_output));
    pot2_output = std::max(0, std::min(127, pot2_output));
    pot1_stable_center_scaled = std::max(0, std::min(127, pot1_stable_center_scaled));
    pot2_stable_center_scaled = std::max(0, std::min(127, pot2_stable_center_scaled));

    // Always output the latest scaled values (0-127 range)
    pot_vals[0] = pot1_output;
    pot_vals[1] = pot2_output;
    // Output stable center values (Need to modify function signature and call site)
    // For now, just store them to check logic
    // int stable_center_out[NUM_POTS] = { pot1_stable_center_scaled, pot2_stable_center_scaled };
    pot_stable_center[0] = pot1_stable_center_scaled; // Assign to output parameter
    pot_stable_center[1] = pot2_stable_center_scaled; // Assign to output parameter

    // Check if the value actually changed compared to the last *output* value for the 'changed' flag
    if (pot_vals[0] != last_pot_vals[0] || pot_vals[1] != last_pot_vals[1]) { // simplified check
        changed = true;
        //ESP_LOGV(TAG_IO, "Pot 1 Changed: Raw=%d Smoothed=%d", pot1_raw, pot_vals[0]); // Updated Log
    }

    // Ultra-simple touch pad handling - just raw threshold comparison
    // No filtering, no debouncing, no state tracking - just raw values

    // Static variable to track previous state for pressed_this_tick detection
    static bool previous_touch_state[NUM_TOUCH_PADS] = {false, false, false, false};

    // Process each touch pad
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        uint16_t touch_val = 0;
        esp_err_t read_err = ESP_FAIL;

        // Try to read the touch value with retries
        const int MAX_RETRIES = 3;
        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            // Read the touch value
            read_err = touch_pad_read(touch_pads[i], &touch_val);

            if (read_err == ESP_OK) {
                // Reading successful
                break;
            } else if (read_err == ESP_ERR_INVALID_STATE) {
                // If invalid state, wait a bit and retry
                ESP_LOGW(TAG_IO, "Touch pad %d not ready (retry %d/%d)", i, retry+1, MAX_RETRIES);
                vTaskDelay(1); // Short delay (1 tick) before retry
            } else {
                // Other error, no retry
                break;
            }
        }

        if (read_err != ESP_OK) {
            ESP_LOGE(TAG_IO, "Error reading touch pad %d! Error: %s", i, esp_err_to_name(read_err));
            // Keep previous state on error
            continue;
        }

        // Simple threshold comparison - no hysteresis, no filtering
        bool current_touch_state = (touch_val < TOUCH_THRESHOLD);

        // Detect if this is a new press
        bool just_pressed = current_touch_state && !previous_touch_state[i];

        // Update output values
        touch_state[i] = current_touch_state;
        pad_pressed_this_tick[i] = just_pressed;

        // Log state changes
        if (current_touch_state != previous_touch_state[i]) {
            const char* pad_name = (i == 0) ? "PAD0" : (i == 1) ? "ARP" : (i == 2) ? "DELREV" : "FILT";
            ESP_LOGI(TAG_IO, "Pad %d (%s): %s (Raw Val:%d, Threshold:%d)",
                     i, pad_name,
                     current_touch_state ? "PRESSED" : "RELEASED",
                     touch_val, TOUCH_THRESHOLD);

            // Mark that something changed
            changed = true;

            // Update previous state
            previous_touch_state[i] = current_touch_state;
        }
    }
    return changed;
}

// --- MIDI Sending Functions (Implementations for midi_helpers.h) ---

// Sends raw MIDI bytes over UART
void send_midi_message(const uint8_t* message, size_t size) {
    if (message == nullptr || size == 0) {
        ESP_LOGE("MIDI", "Invalid MIDI message buffer or size");
        return;
    }
    int bytes_written = uart_write_bytes(MIDI_UART, (const char*)message, size);
    if (bytes_written != (int)size) {
        ESP_LOGE("MIDI", "Failed to write all MIDI bytes. Expected %d, wrote %d", size, bytes_written);
        // Consider adding uart_wait_tx_done or error handling if needed
    }
    // ESP_LOG_BUFFER_HEXDUMP("MIDI_TX", message, size, ESP_LOG_DEBUG); // Optional: Log sent bytes
}

// Sends a MIDI Control Change (CC) message
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value) {
    if (channel < 1 || channel > 16) {
        ESP_LOGE("MIDI", "Invalid MIDI channel: %d", channel);
        return;
    }
    uint8_t midi_msg[3];
    midi_msg[0] = 0xB0 | (channel - 1); // CC command OR channel (0-15)
    midi_msg[1] = cc_num & 0x7F;       // CC number (0-119 valid, 120-127 reserved)
    midi_msg[2] = value & 0x7F;        // CC value (0-127)
    send_midi_message(midi_msg, sizeof(midi_msg));
}

// Sends a MIDI Non-Registered Parameter Number (NRPN) message (MSB only)
void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb) {
    // NRPN Sequence:
    // 1. Set NRPN MSB (CC 99)
    send_midi_cc(channel, 99, nrpn_msb);
    // 2. Set NRPN LSB (CC 98)
    send_midi_cc(channel, 98, nrpn_lsb);
    // 3. Set Data Entry MSB (CC 6)
    send_midi_cc(channel, 6, value_msb);
    // Optional: Add CC 38 for Data Entry LSB if needed
    // Optional: Reset NRPN/RPN selection (CC 101=127, CC 100=127)
    // send_midi_cc(channel, 101, 127);
    // send_midi_cc(channel, 100, 127);
}