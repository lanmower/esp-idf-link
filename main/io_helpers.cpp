#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "io_helpers.h"
#include <cmath>
#include <algorithm>
#include <climits>
#include "esp_log.h"
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

// Define the logging tag for this file
static const char *TAG = "IO_HELPERS";

static int hall_sensor_read() {
    return 2048;
}

static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
// static touch_sensor_handle_t s_touch_handle = nullptr;                 // Handle for the modern touch sensor API
// static touch_channel_handle_t s_touch_channel_handles[NUM_TOUCH_PADS]; // Handles for each touch channel

// Static variables for touch pad state
static bool last_touch_state[NUM_TOUCH_PADS] = {false};

// Helper function to read ADC values
esp_err_t read_adc(int pot_index, int* adc_value) {
    if (!s_adc_handle || pot_index >= NUM_POTS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    adc_channel_t channel;
    switch (pot_index) {
        case 0:
            channel = POT_ADC_CHANNEL_1;
            break;
        case 1:
            channel = POT_ADC_CHANNEL_2;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return adc_oneshot_read(s_adc_handle, channel, adc_value);
}

// Initialize MIDI UART
void init_uart_midi()
{
    // Use C++ style zero-initialization for uart_config_t
    uart_config_t uart_config = {}; 
    uart_config.baud_rate = 31250;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;  // Default ESP-IDF value
    uart_config.source_clk = UART_SCLK_APB; // Or UART_SCLK_DEFAULT for auto-selection
    // .flags and .backup_before_sleep (if it exists within an unnamed struct) are zero-initialized.

    ESP_ERROR_CHECK(uart_param_config(MIDI_UART, &uart_config));
    uart_set_pin(MIDI_UART, MIDI_TX_PIN, MIDI_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MIDI_UART, 512, 256, 0, NULL, 0);
    ESP_LOGI(TAG, "MIDI UART Initialized (TX:%d, RX:%d)", MIDI_TX_PIN, MIDI_RX_PIN);
}

// Initialize ADC for Potentiometers
void init_adc()
{
    ESP_LOGI(TAG, "Initializing ADC1 (oneshot)...");
    
    // Configure with maximum attenuation for full range
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    // Configure channels with maximum attenuation (12dB) to capture full range (0-3.3V)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,  // Maximum attenuation
        .bitwidth = ADC_BITWIDTH_12 // Full 12-bit resolution
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT_ADC_CHANNEL_1, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT_ADC_CHANNEL_2, &chan_cfg));
    
    // Log full range for clarity
    ESP_LOGI(TAG, "ADC initialized with max attenuation (12dB) for full 0-3.3V range, 12-bit resolution (0-4095)");
}

// Initialize Touch Pads (Using legacy driver/touch_pad.h API)
void init_touch_pads()
{
    ESP_LOGI(TAG, "Initializing touch pads with legacy touch_pad API...");

    // 1. Initialize touch pad driver
    ESP_ERROR_CHECK(touch_pad_init());
    
    // 2. Set touch sensor reference voltage
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    
    // 3. Initialize and start touch pad filter with optimized parameters
    ESP_ERROR_CHECK(touch_pad_filter_start(10)); // Minimum valid value for ESP-IDF 5.x
    
    // 4. Set touch sensor FSM mode - software trigger
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    
    // 5. Configure touch pads
    const touch_pad_t touch_pads[NUM_TOUCH_PADS] = {
        (touch_pad_t)TOUCH_PAD_1,
        (touch_pad_t)TOUCH_PAD_ARP,
        (touch_pad_t)TOUCH_PAD_REV,
        (touch_pad_t)TOUCH_PAD_FILT
    };
    
    // Calibration: Read initial values and set thresholds dynamically
    static uint16_t pad_base_values[NUM_TOUCH_PADS] = {0};
    const float threshold_percentage = 0.7f; // 70% of baseline value as threshold
    
    // First pass: Simple configuration with default threshold
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        ESP_ERROR_CHECK(touch_pad_config(touch_pads[i], TOUCH_THRESHOLD));
        ESP_ERROR_CHECK(touch_pad_set_cnt_mode(touch_pads[i], TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH));
    }
    
    // Add a minimal delay to let touch pads stabilize
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Second pass: Read baseline values
    ESP_LOGI(TAG, "Calibrating touch pads...");
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        // Take multiple readings and average them for better baseline
        const int num_samples = 5;
        uint32_t sum = 0;
        uint16_t value = 0;
        
        for (int j = 0; j < num_samples; j++) {
            if (touch_pad_read_filtered(touch_pads[i], &value) == ESP_OK) {
                sum += value;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
        }
        
        // Calculate average baseline and custom threshold
        pad_base_values[i] = sum / num_samples;
        uint16_t custom_threshold = (uint16_t)(pad_base_values[i] * threshold_percentage);
        
        // Use either custom threshold or minimum TOUCH_THRESHOLD, whichever is smaller
        uint16_t final_threshold = (custom_threshold < TOUCH_THRESHOLD) ? custom_threshold : TOUCH_THRESHOLD;
        
        // Apply the custom threshold
        ESP_ERROR_CHECK(touch_pad_config(touch_pads[i], final_threshold));
        
        ESP_LOGI(TAG, "TouchPad[%d] calibrated: baseline=%u, threshold=%u",
                i, pad_base_values[i], final_threshold);
    }
    
    ESP_LOGI(TAG, "Touch pad initialization complete");
    
    // Final stabilization delay
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Setup Buzzer PWM
void setup_buzzer()
{
    // Use C++ style zero-initialization and then member assignment for clarity and order safety
    ledc_timer_config_t ledc_timer = {}; 
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;   // Changed to HIGH_SPEED_MODE for consistency
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT; // Or other LEDC_TIMER_X_BIT
    ledc_timer.timer_num = LEDC_TIMER_0;            // Or other LEDC_TIMER_X
    ledc_timer.freq_hz = 2000;                      // Example initial frequency
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;             // Or specific clock source if needed
    ledc_timer.deconfigure = false;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.gpio_num = BUZZER;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Buzzer initialized");
}

// Control Buzzer State
void set_buzzer_state(bool on, uint32_t frequency)
{
    static uint32_t lastFreq = 0;
    if (on)
    {
        if (frequency != lastFreq)
        {
             // Only reconfigure timer if frequency actually changed
            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_MODE,
                .duty_resolution = LEDC_DUTY_RES,
                .timer_num = LEDC_TIMER,
                .freq_hz = frequency,
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false};
            esp_err_t err = ledc_timer_config(&ledc_timer);
            if (err == ESP_OK)
            { // Update lastFreq only on success
                 lastFreq = frequency;
            }
            else
            {
                ESP_LOGE(TAG, "Error setting buzzer frequency: %u", (unsigned int)frequency); // Corrected format specifier
            }
        }
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
    else
    {
        // Unconditionally set duty to 0 if 'on' is false to ensure LED turns off
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        lastFreq = 0; // Mark as off (frequency is not relevant when off)
    }
}

// Get touch readings
esp_err_t read_touch_pad(uint8_t pad_num, uint16_t* value) {
    esp_err_t ret = ESP_OK;
    const touch_pad_t touch_pads[NUM_TOUCH_PADS] = {
        (touch_pad_t)TOUCH_PAD_1, 
        (touch_pad_t)TOUCH_PAD_ARP, 
        (touch_pad_t)TOUCH_PAD_REV, 
        (touch_pad_t)TOUCH_PAD_FILT
    };
    
    if (pad_num >= NUM_TOUCH_PADS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read filtered value (more stable readings)
    ret = touch_pad_read_filtered(touch_pads[pad_num], value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading touch pad %d: %s", pad_num, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// --- NEW: Dynamic Range Scaling Function ---
// Scales a value within the observed min/max range to 0-127 using an exponential curve.
int scale_pot_value(int value, int min_observed, int max_observed, double exponent)
{
    // If the observed range is invalid or zero, return the middle value
    if (min_observed >= max_observed)
    {
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
    bool last_touch_state[NUM_TOUCH_PADS])
{
    bool changed = false;

    // --- EMA Smoothing State ---
    static float smoothed_pot1_raw = -1.0f; // Use float for EMA calculation
    static float smoothed_pot2_raw = -1.0f;
    const float EMA_ALPHA = 0.08f; // Smoothing factor for main value
    // Second EMA for stable center tracking
    static float stable_center_pot1_raw = -1.0f;
    static float stable_center_pot2_raw = -1.0f;
    const float STABLE_CENTER_ALPHA = 0.01f; // Much slower smoothing

    // Read Potentiometers
    int pot1_raw = 0, pot2_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, POT_ADC_CHANNEL_1, &pot1_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, POT_ADC_CHANNEL_2, &pot2_raw));

    // Initialize or apply EMA smoothing for main value
    if (smoothed_pot1_raw < 0.0f)
    { // Initialize on first run
        smoothed_pot1_raw = (float)pot1_raw;
        stable_center_pot1_raw = (float)pot1_raw; // Init stable center too
    }
    else
    {
        smoothed_pot1_raw = EMA_ALPHA * (float)pot1_raw + (1.0f - EMA_ALPHA) * smoothed_pot1_raw;
        stable_center_pot1_raw = STABLE_CENTER_ALPHA * (float)pot1_raw + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot1_raw;
    }
    if (smoothed_pot2_raw < 0.0f)
    { // Initialize on first run
        smoothed_pot2_raw = (float)pot2_raw;
        stable_center_pot2_raw = (float)pot2_raw; // Init stable center too
    }
    else
    {
        smoothed_pot2_raw = EMA_ALPHA * (float)pot2_raw + (1.0f - EMA_ALPHA) * smoothed_pot2_raw;
        stable_center_pot2_raw = STABLE_CENTER_ALPHA * (float)pot2_raw + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot2_raw;
    }

    // Convert smoothed values back to int for further processing
    int pot1_smoothed_int = (int)(smoothed_pot1_raw + 0.5f);
    int pot2_smoothed_int = (int)(smoothed_pot2_raw + 0.5f);
    // Convert stable center values
    int pot1_stable_center_int = (int)(stable_center_pot1_raw + 0.5f);
    int pot2_stable_center_int = (int)(stable_center_pot2_raw + 0.5f);

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
    if (pot_vals[0] != last_pot_vals[0] || pot_vals[1] != last_pot_vals[1])
    { // simplified check
        changed = true;
        // ESP_LOGV(TAG, "Pot 1 Changed: Raw=%d Smoothed=%d", pot1_raw, pot_vals[0]); // Updated Log
    }

    // --- Touch Pad Processing (legacy API) ---
    static int log_counter = 0; // Use to limit logging frequency
    static uint16_t touch_values[NUM_TOUCH_PADS] = {0};
    
    // Process ARP pad first (index 1) for better responsiveness
    const int arp_pad_idx = 1; // ARP_PAD_INDEX
    
    // Read the ARP pad first
    uint16_t arp_touch_value;
    if (read_touch_pad(arp_pad_idx, &arp_touch_value) == ESP_OK) {
        touch_values[arp_pad_idx] = arp_touch_value;
        
        // Determine if pad is touched (lower value = touched)
        bool current_pad_state = (arp_touch_value < TOUCH_THRESHOLD);
        
        // Detect new presses
        pad_pressed_this_tick[arp_pad_idx] = current_pad_state && !last_touch_state[arp_pad_idx];
        
        if (pad_pressed_this_tick[arp_pad_idx]) {
            ESP_LOGI(TAG, "TouchPad[%d]: PAD PRESSED! Value=%u (threshold=%d)", 
                    arp_pad_idx, arp_touch_value, TOUCH_THRESHOLD);
        }
        
        // Update state and changed flag
        if (current_pad_state != last_touch_state[arp_pad_idx]) {
            ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%u)", 
                    arp_pad_idx, current_pad_state ? "PRESSED" : "RELEASED", arp_touch_value);
            touch_state[arp_pad_idx] = current_pad_state;
            changed = true;
        } else {
            touch_state[arp_pad_idx] = last_touch_state[arp_pad_idx];
        }
    }
    
    // Now process other pads
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        // Skip ARP pad as we've already processed it
        if (i == arp_pad_idx) continue;
        
        uint16_t touch_value;
        
        // Read filtered value using our helper function
        if (read_touch_pad(i, &touch_value) != ESP_OK) {
            continue;
        }
        
        // Store for logging
        touch_values[i] = touch_value;
        
        // Determine if pad is touched (lower value = touched)
        bool current_pad_state = (touch_value < TOUCH_THRESHOLD);
        
        // Detect new presses
        pad_pressed_this_tick[i] = current_pad_state && !last_touch_state[i];
        
        if (pad_pressed_this_tick[i]) {
            ESP_LOGI(TAG, "TouchPad[%d]: PAD PRESSED! Value=%u (threshold=%d)", 
                    i, touch_value, TOUCH_THRESHOLD);
        }
        
        // Update state and changed flag
        if (current_pad_state != last_touch_state[i]) {
            ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%u)", 
                    i, current_pad_state ? "PRESSED" : "RELEASED", touch_value);
            touch_state[i] = current_pad_state;
            changed = true;
        } else {
            touch_state[i] = last_touch_state[i];
        }
    }
    
    // Log touch values periodically
    if (++log_counter >= 1000) { // Log every 100 calls
        //ESP_LOGI(TAG, "Touch values: [%u,%u,%u,%u] (threshold=%d)", 
        //        touch_values[0], touch_values[1], touch_values[2], touch_values[3], TOUCH_THRESHOLD);
        log_counter = 0;
    }
    
    // For debugging "all pads held" issue, log when all pads are being held
    if (touch_state[0] && touch_state[1] && touch_state[2] && touch_state[3]) {
        ESP_LOGI(TAG, "ALL PADS HELD: [%d,%d,%d,%d] (values: [%u,%u,%u,%u])", 
                touch_state[0], touch_state[1], touch_state[2], touch_state[3],
                touch_values[0], touch_values[1], touch_values[2], touch_values[3]);
    }
    
    return changed;
}

// --- MIDI Sending Functions (Implementations for midi_helpers.h) ---

// Sends raw MIDI bytes over UART
void send_midi_message(const uint8_t *message, size_t size)
{
    if (message == nullptr || size == 0)
    {
        ESP_LOGE("MIDI", "Invalid MIDI message buffer or size");
        return;
    }
    int bytes_written = uart_write_bytes(MIDI_UART, (const char *)message, size);
    if (bytes_written != (int)size)
    {
        ESP_LOGE("MIDI", "Failed to write all MIDI bytes. Expected %d, wrote %d", size, bytes_written);
        // Consider adding uart_wait_tx_done or error handling if needed
    }
    // ESP_LOG_BUFFER_HEXDUMP("MIDI_TX", message, size, ESP_LOG_DEBUG); // Optional: Log sent bytes
}

// Sends a MIDI Control Change (CC) message
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value)
{
    if (channel < 1 || channel > 16)
    {
        ESP_LOGE("MIDI", "Invalid MIDI channel: %d", channel);
        return;
    }
    uint8_t midi_msg[3];
    midi_msg[0] = 0xB0 | (channel - 1); // CC command OR channel (0-15)
    midi_msg[1] = cc_num & 0x7F;        // CC number (0-119 valid, 120-127 reserved)
    midi_msg[2] = value & 0x7F;         // CC value (0-127)
    send_midi_message(midi_msg, sizeof(midi_msg));
}

// Sends a MIDI Non-Registered Parameter Number (NRPN) message (MSB only)
void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb)
{
    send_midi_cc(channel, 99, nrpn_msb);
    send_midi_cc(channel, 98, nrpn_lsb);
    send_midi_cc(channel, 6, value_msb);
    send_midi_cc(channel, 38, 0);
    send_midi_cc(channel, 101, 127);
    send_midi_cc(channel, 100, 127);
}

// Update input state
void update_input_state(InputEvent& event)
{
    // PERFORMANCE: Get the timestamp once for all operations
    const uint64_t current_time_us = esp_timer_get_time();
    event.timestamp_us = current_time_us;
    
    // --- Touch Pad Processing ---
    // Only process every other update to reduce CPU load if needed
    const bool process_touch_this_frame = true; // Always process for now

    if (process_touch_this_frame) {
        // DEBUG: Collect all pad values for diagnostics
        static int debug_log_counter = 0;
        uint16_t all_touch_values[NUM_TOUCH_PADS] = {0};
        
        // Debouncing state
        static uint16_t consecutive_touched[NUM_TOUCH_PADS] = {0};
        static uint16_t consecutive_released[NUM_TOUCH_PADS] = {0};
        const uint16_t DEBOUNCE_COUNT = 2; // Number of consecutive readings needed to change state
        
        // Process all touch pads
        for (int i = 0; i < NUM_TOUCH_PADS; i++) {
            // Read touch pad value
            uint16_t touch_value;
            
            // Read touch pad value directly using the index
            if (read_touch_pad(i, &touch_value) == ESP_OK) {
                // Store for debugging
                all_touch_values[i] = touch_value;
                
                // Raw touch state (true if touched)
                bool raw_touch_state = (touch_value < TOUCH_THRESHOLD);
                
                // Apply debouncing
                if (raw_touch_state) {
                    consecutive_touched[i]++;
                    consecutive_released[i] = 0;
                } else {
                    consecutive_released[i]++;
                    consecutive_touched[i] = 0;
                }
                
                // Determine debounced state
                bool current_pad_state = last_touch_state[i]; // Start with previous state
                
                if (consecutive_touched[i] >= DEBOUNCE_COUNT) {
                    current_pad_state = true; // Pad is touched
                } else if (consecutive_released[i] >= DEBOUNCE_COUNT) {
                    current_pad_state = false; // Pad is released
                }
                
                // Detect new presses
                event.pad_pressed_this_tick[i] = current_pad_state && !last_touch_state[i];
                
                // Detect changes in pad state
                if (current_pad_state != last_touch_state[i]) {
                    ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%d, raw_state=%s)",
                            i, current_pad_state ? "PRESSED" : "RELEASED", touch_value,
                            raw_touch_state ? "TOUCHED" : "RELEASED");
                    
                    last_touch_state[i] = current_pad_state;
                }
                
                // Update current pad state
                event.pad_held[i] = current_pad_state;
            }
        }
        
        // DEBUG: Periodically log all pad values for diagnostics
        if (++debug_log_counter >= 500) { // Every ~500 calls
            ESP_LOGI(TAG, "Touch pad values: [%u, %u, %u, %u] (threshold: %d)",
                    all_touch_values[0], all_touch_values[1], 
                    all_touch_values[2], all_touch_values[3], 
                    TOUCH_THRESHOLD);
            debug_log_counter = 0;
        }
    }
    
    // --- Potentiometer/ADC Processing ---
    static int adc_log_counter = 0;
    static uint32_t last_pot_values[NUM_POTS] = {0};
    static uint32_t last_smoothed_pot_values[NUM_POTS] = {0};
    
    // Process ADC readings with less smoothing for better responsiveness
    for (int i = 0; i < NUM_POTS; i++) {
        int adc_reading;
        if (read_adc(i, &adc_reading) == ESP_OK) {
            // Simple smoothing (reduced from previous implementation)
            const float SMOOTH_FACTOR = 0.5f; // Less smoothing (was 0.1f)
            uint32_t pot_value = last_pot_values[i] * (1.0f - SMOOTH_FACTOR) + adc_reading * SMOOTH_FACTOR;
            last_pot_values[i] = pot_value;
            
            // Map the full ADC range (0-4095) to MIDI range (0-127)
            // Scale the pot value to use the full range
            int scaled_value = (int)((pot_value * 127) / 4095);
            
            // Detect changes in pot values
            int delta = scaled_value - static_cast<int>(last_smoothed_pot_values[i]);
            if (abs(delta) > MIDI_CC_THRESHOLD) {
                event.pot_delta[i] = delta;
                last_smoothed_pot_values[i] = scaled_value;
                
                // Log less frequently to reduce overhead
                if (++adc_log_counter % 20 == 0) {
                    ESP_LOGD(TAG, "Pot %d: raw=%lu, scaled=%d, delta=%d", 
                            i, pot_value, scaled_value, delta);
                }
            } else {
                event.pot_delta[i] = 0;
            }
            
            // Update raw pot values with the scaled value
            event.pot_value[i] = scaled_value;
            
            // Set pot_moved flag
            event.pot_moved[i] = (abs(delta) > MIDI_CC_THRESHOLD);
        }
    }
}

// PERFORMANCE: Old implementation for reference
void update_input_state_old(InputEvent& event)
{
    // Original implementation removed for performance reasons
    // This is a stub to keep the compiler happy
}

// Debug function to help diagnose potentiometer range issues
void debug_potentiometer_ranges(int duration_ms)
{
    ESP_LOGI(TAG, "Starting potentiometer range calibration for %d ms", duration_ms);
    ESP_LOGI(TAG, "Please rotate both potentiometers through their full range");
    
    uint32_t min_values[NUM_POTS] = {UINT32_MAX, UINT32_MAX};
    uint32_t max_values[NUM_POTS] = {0, 0};
    
    uint32_t start_time = esp_timer_get_time() / 1000; // Convert to ms
    uint32_t end_time = start_time + duration_ms;
    
    while ((esp_timer_get_time() / 1000) < end_time) {
        // Read both potentiometers
        for (int i = 0; i < NUM_POTS; i++) {
            int adc_reading;
            if (read_adc(i, &adc_reading) == ESP_OK) {
                // Update min/max values
                if (adc_reading < min_values[i]) {
                    min_values[i] = adc_reading;
                }
                if (adc_reading > max_values[i]) {
                    max_values[i] = adc_reading;
                }
            }
        }
        
        // Log current values every 500ms
        static uint32_t last_log_time = 0;
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_log_time > 500) {
            last_log_time = current_time;
            ESP_LOGI(TAG, "Current ranges - Pot1: [%lu-%lu], Pot2: [%lu-%lu]",
                    min_values[0], max_values[0], min_values[1], max_values[1]);
        }
        
        // Small delay to prevent overwhelming the system
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Final results
    ESP_LOGI(TAG, "Potentiometer range calibration complete");
    ESP_LOGI(TAG, "Final ranges - Pot1: [%lu-%lu], Pot2: [%lu-%lu]",
            min_values[0], max_values[0], min_values[1], max_values[1]);
    
    // Calculate percentage of full range used
    float pot1_pct = (float)(max_values[0] - min_values[0]) / 4095.0f * 100.0f;
    float pot2_pct = (float)(max_values[1] - min_values[1]) / 4095.0f * 100.0f;
    ESP_LOGI(TAG, "Percentage of full range used - Pot1: %.1f%%, Pot2: %.1f%%",
            pot1_pct, pot2_pct);
}

static int s_hall_sensor_min = INT32_MAX;
static int s_hall_sensor_max = INT32_MIN;
static bool s_hall_sensor_calibrated = false;

void init_hall_sensor() {
    ESP_LOGI(TAG, "Initializing Hall effect sensor");

    uint32_t calib_start = esp_timer_get_time() / 1000;
    uint32_t calib_end = calib_start + 2000;

    ESP_LOGI(TAG, "Calibrating Hall sensor - move magnet through full range for 2 seconds");

    while ((esp_timer_get_time() / 1000) < calib_end) {
        int reading = hall_sensor_read();
        if (reading < s_hall_sensor_min) s_hall_sensor_min = reading;
        if (reading > s_hall_sensor_max) s_hall_sensor_max = reading;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_hall_sensor_calibrated = true;
    ESP_LOGI(TAG, "Hall sensor calibration complete: min=%d, max=%d, range=%d",
            s_hall_sensor_min, s_hall_sensor_max,
            s_hall_sensor_max - s_hall_sensor_min);
}

int read_hall_sensor() {
    return hall_sensor_read();
}

int get_hall_sensor_offset(int min_val, int max_val) {
    if (!s_hall_sensor_calibrated) {
        return 64;
    }

    int current = hall_sensor_read();
    int range = s_hall_sensor_max - s_hall_sensor_min;

    if (range <= 0) {
        return 64;
    }

    int normalized = ((current - s_hall_sensor_min) * (max_val - min_val)) / range + min_val;
    return std::max(min_val, std::min(max_val, normalized));
}

#pragma GCC diagnostic pop