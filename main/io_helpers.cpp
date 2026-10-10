#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "io_helpers.h"
#include <cmath>
#include <algorithm>
#include <climits>
#include "esp_log.h"
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

static constexpr int MIDI_DIN_BAUD_RATE = 31250;
static constexpr int MIDI_UART_RX_BUFFER_BYTES = 512;
static constexpr int MIDI_UART_TX_BUFFER_BYTES = 256;
static constexpr int MIDI_CONTROL_CHANGE_CMD = 0xB0;
extern constexpr uint8_t MIDI_USER_CHANNEL_MIN = 1;
extern constexpr uint8_t MIDI_USER_CHANNEL_MAX = 16;
static constexpr uint8_t MIDI_WIRE_CHANNEL_MIN = 0;
static constexpr uint8_t MIDI_WIRE_CHANNEL_MAX =
    MIDI_WIRE_CHANNEL_MIN + (MIDI_USER_CHANNEL_MAX - MIDI_USER_CHANNEL_MIN);
static_assert(MIDI_WIRE_CHANNEL_MAX == 15,
              "MIDI user channels 1..16 must map onto wire channels 0..15");
static constexpr int MIDI_DATA_BYTE_MASK = 0x7F;
static constexpr int MIDI_VALUE_MID = 64;
static constexpr int MIDI_CC_NRPN_PARAM_MSB = 99;
static constexpr int MIDI_CC_NRPN_PARAM_LSB = 98;
static constexpr int MIDI_CC_DATA_ENTRY_MSB = 6;
static constexpr int MIDI_CC_DATA_ENTRY_LSB = 38;
static constexpr int MIDI_CC_RPN_PARAM_MSB = 101;
static constexpr int MIDI_CC_RPN_PARAM_LSB = 100;
static constexpr int MIDI_CC_PARAM_DESELECT = 127;
static constexpr int HALL_SENSOR_STUB_READING = 2048;
static constexpr float TOUCH_CALIBRATION_THRESHOLD_RATIO = 0.7f;

static const char *TAG = "IO_HELPERS";

static int hall_sensor_read() {
    return HALL_SENSOR_STUB_READING;
}

static adc_oneshot_unit_handle_t s_adc_handle = nullptr;

static bool last_touch_state[NUM_TOUCH_PADS] = {false};

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

void init_uart_midi()
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = MIDI_DIN_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk = UART_SCLK_APB;

    ESP_ERROR_CHECK(uart_param_config(MIDI_UART, &uart_config));
    uart_set_pin(MIDI_UART, MIDI_TX_PIN, MIDI_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MIDI_UART, MIDI_UART_RX_BUFFER_BYTES, MIDI_UART_TX_BUFFER_BYTES, 0, NULL, 0);
    ESP_LOGI(TAG, "MIDI UART Initialized (TX:%d, RX:%d)", MIDI_TX_PIN, MIDI_RX_PIN);
}

void init_adc()
{
    ESP_LOGI(TAG, "Initializing ADC1 (oneshot)...");
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT_ADC_CHANNEL_1, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, POT_ADC_CHANNEL_2, &chan_cfg));
    ESP_LOGI(TAG, "ADC initialized with max attenuation (12dB) for full 0-3.3V range, 12-bit resolution (0-4095)");
}

static const touch_pad_t kTouchPads[NUM_TOUCH_PADS] = {
    (touch_pad_t)TOUCH_PAD_1,
    (touch_pad_t)TOUCH_PAD_ARP,
    (touch_pad_t)TOUCH_PAD_REV,
    (touch_pad_t)TOUCH_PAD_FILT
};

void init_touch_pads()
{
    ESP_LOGI(TAG, "Initializing touch pads with legacy touch_pad API...");

    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    static uint16_t pad_base_values[NUM_TOUCH_PADS] = {0};
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        ESP_ERROR_CHECK(touch_pad_config(kTouchPads[i], TOUCH_THRESHOLD));
        ESP_ERROR_CHECK(touch_pad_set_cnt_mode(kTouchPads[i], TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Calibrating touch pads...");
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        const int num_samples = 5;
        uint32_t sum = 0;
        uint16_t value = 0;
        for (int j = 0; j < num_samples; j++) {
            if (touch_pad_read_filtered(kTouchPads[i], &value) == ESP_OK) {
                sum += value;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        pad_base_values[i] = sum / num_samples;
        uint16_t custom_threshold = (uint16_t)(pad_base_values[i] * TOUCH_CALIBRATION_THRESHOLD_RATIO);
        uint16_t final_threshold = (custom_threshold < TOUCH_THRESHOLD) ? custom_threshold : TOUCH_THRESHOLD;
        ESP_ERROR_CHECK(touch_pad_config(kTouchPads[i], final_threshold));
        ESP_LOGI(TAG, "TouchPad[%d] calibrated: baseline=%u, threshold=%u",
                i, pad_base_values[i], final_threshold);
    }
    ESP_LOGI(TAG, "Touch pad initialization complete");
    vTaskDelay(pdMS_TO_TICKS(20));
}

void setup_buzzer()
{
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.timer_num = LEDC_TIMER;
    ledc_timer.freq_hz = 2000;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer.deconfigure = false;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = LEDC_MODE;
    ledc_channel.channel = LEDC_CHANNEL;
    ledc_channel.timer_sel = LEDC_TIMER;
    ledc_channel.gpio_num = BUZZER;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Buzzer initialized");
}

static uint32_t s_buzzer_freq = 0;

void prime_buzzer_freq(uint32_t frequency)
{
    if (frequency == s_buzzer_freq) return;
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err == ESP_OK)
    {
        s_buzzer_freq = frequency;
    }
    else
    {
        ESP_LOGE(TAG, "Error setting buzzer frequency: %u", (unsigned int)frequency);
    }
}

void set_buzzer_state(bool on, uint32_t frequency)
{
    if (!on)
    {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        s_buzzer_freq = 0;
        return;
    }
    prime_buzzer_freq(frequency);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

esp_err_t read_touch_pad(uint8_t pad_num, uint16_t* value) {
    if (pad_num >= NUM_TOUCH_PADS) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = touch_pad_read_filtered(kTouchPads[pad_num], value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading touch pad %d: %s", pad_num, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

int scale_pot_value(int value, int min_observed, int max_observed, double exponent)
{
    if (min_observed >= max_observed)
    {
        return MIDI_VALUE_MID;
    }

    double input_range = static_cast<double>(max_observed - min_observed);
    double value_double = static_cast<double>(value);

    double normalized_input = (value_double - min_observed) / input_range;

    normalized_input = std::max(0.0, std::min(1.0, normalized_input));

    double scaled_value_normalized = pow(normalized_input, exponent);

    int final_value = static_cast<int>(round(scaled_value_normalized * MIDI_VALUE_MAX));

    return std::max(0, std::min(MIDI_VALUE_MAX, final_value));
}

bool read_controls(
    int pot_vals[NUM_POTS],
    int pot_stable_center[NUM_POTS],
    bool pad_touched[NUM_TOUCH_PADS],
    bool pad_pressed_this_tick[NUM_TOUCH_PADS],
    int last_pot_vals[NUM_POTS],
    bool last_pad_touched[NUM_TOUCH_PADS])
{
    bool changed = false;

    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        pad_touched[i] = last_pad_touched[i];
        pad_pressed_this_tick[i] = false;
    }

    static float smoothed_pot1_raw = -1.0f;
    static float smoothed_pot2_raw = -1.0f;
    const float EMA_ALPHA = 0.08f;
    static float stable_center_pot1_raw = -1.0f;
    static float stable_center_pot2_raw = -1.0f;
    const float STABLE_CENTER_ALPHA = 0.01f;

    int pot1_raw = 0, pot2_raw = 0;
    const bool pot1_read_ok = s_adc_handle != nullptr
        && adc_oneshot_read(s_adc_handle, POT_ADC_CHANNEL_1, &pot1_raw) == ESP_OK;
    const bool pot2_read_ok = s_adc_handle != nullptr
        && adc_oneshot_read(s_adc_handle, POT_ADC_CHANNEL_2, &pot2_raw) == ESP_OK;
    if (!pot1_read_ok || !pot2_read_ok) {
        ESP_LOGW(TAG, "ADC read failed (ch1=%s ch2=%s) -- reusing last smoothed pot values",
                 pot1_read_ok ? "ok" : "fail", pot2_read_ok ? "ok" : "fail");
    }

    if (pot1_read_ok) {
        if (smoothed_pot1_raw < 0.0f) {
            smoothed_pot1_raw = (float)pot1_raw;
            stable_center_pot1_raw = (float)pot1_raw;
        } else {
            smoothed_pot1_raw = EMA_ALPHA * (float)pot1_raw + (1.0f - EMA_ALPHA) * smoothed_pot1_raw;
            stable_center_pot1_raw = STABLE_CENTER_ALPHA * (float)pot1_raw
                                   + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot1_raw;
        }
    }
    if (pot2_read_ok) {
        if (smoothed_pot2_raw < 0.0f) {
            smoothed_pot2_raw = (float)pot2_raw;
            stable_center_pot2_raw = (float)pot2_raw;
        } else {
            smoothed_pot2_raw = EMA_ALPHA * (float)pot2_raw + (1.0f - EMA_ALPHA) * smoothed_pot2_raw;
            stable_center_pot2_raw = STABLE_CENTER_ALPHA * (float)pot2_raw
                                   + (1.0f - STABLE_CENTER_ALPHA) * stable_center_pot2_raw;
        }
    }

    int pot1_smoothed_int = (int)(smoothed_pot1_raw + 0.5f);
    int pot2_smoothed_int = (int)(smoothed_pot2_raw + 0.5f);
    int pot1_stable_center_int = (int)(stable_center_pot1_raw + 0.5f);
    int pot2_stable_center_int = (int)(stable_center_pot2_raw + 0.5f);

    int pot1_output = static_cast<int>(round(((double)pot1_smoothed_int / ADC_RAW_MAX) * MIDI_VALUE_MAX));
    int pot2_output = static_cast<int>(round(((double)pot2_smoothed_int / ADC_RAW_MAX) * MIDI_VALUE_MAX));
    int pot1_stable_center_scaled = static_cast<int>(round(((double)pot1_stable_center_int / ADC_RAW_MAX) * MIDI_VALUE_MAX));
    int pot2_stable_center_scaled = static_cast<int>(round(((double)pot2_stable_center_int / ADC_RAW_MAX) * MIDI_VALUE_MAX));

    pot1_output = std::max(0, std::min(MIDI_VALUE_MAX, pot1_output));
    pot2_output = std::max(0, std::min(MIDI_VALUE_MAX, pot2_output));
    pot1_stable_center_scaled = std::max(0, std::min(MIDI_VALUE_MAX, pot1_stable_center_scaled));
    pot2_stable_center_scaled = std::max(0, std::min(MIDI_VALUE_MAX, pot2_stable_center_scaled));

    pot_vals[0] = pot1_output;
    pot_vals[1] = pot2_output;
    pot_stable_center[0] = pot1_stable_center_scaled;
    pot_stable_center[1] = pot2_stable_center_scaled;

    if (pot_vals[0] != last_pot_vals[0] || pot_vals[1] != last_pot_vals[1])
    {
        changed = true;
    }

    static uint16_t touch_values[NUM_TOUCH_PADS] = {0};
    const int arp_pad_idx = 1;
    uint16_t arp_touch_value;
    if (read_touch_pad(arp_pad_idx, &arp_touch_value) == ESP_OK) {
        touch_values[arp_pad_idx] = arp_touch_value;
        bool current_pad_state = (arp_touch_value < TOUCH_THRESHOLD);
        pad_pressed_this_tick[arp_pad_idx] = current_pad_state && !last_pad_touched[arp_pad_idx];
        if (pad_pressed_this_tick[arp_pad_idx]) {
            ESP_LOGI(TAG, "TouchPad[%d]: PAD PRESSED! Value=%u (threshold=%d)",
                    arp_pad_idx, arp_touch_value, TOUCH_THRESHOLD);
        }
        if (current_pad_state != last_pad_touched[arp_pad_idx]) {
            ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%u)",
                    arp_pad_idx, current_pad_state ? "PRESSED" : "RELEASED", arp_touch_value);
            pad_touched[arp_pad_idx] = current_pad_state;
            changed = true;
        } else {
            pad_touched[arp_pad_idx] = last_pad_touched[arp_pad_idx];
        }
    }
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        if (i == arp_pad_idx) continue;
        uint16_t touch_value;
        if (read_touch_pad(i, &touch_value) != ESP_OK) {
            continue;
        }
        touch_values[i] = touch_value;
        bool current_pad_state = (touch_value < TOUCH_THRESHOLD);
        pad_pressed_this_tick[i] = current_pad_state && !last_pad_touched[i];
        if (pad_pressed_this_tick[i]) {
            ESP_LOGI(TAG, "TouchPad[%d]: PAD PRESSED! Value=%u (threshold=%d)",
                    i, touch_value, TOUCH_THRESHOLD);
        }
        if (current_pad_state != last_pad_touched[i]) {
            ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%u)",
                    i, current_pad_state ? "PRESSED" : "RELEASED", touch_value);
            pad_touched[i] = current_pad_state;
            changed = true;
        } else {
            pad_touched[i] = last_pad_touched[i];
        }
    }
    if (pad_touched[0] && pad_touched[1] && pad_touched[2] && pad_touched[3]) {
        ESP_LOGI(TAG, "ALL PADS HELD: [%d,%d,%d,%d] (values: [%u,%u,%u,%u])",
                pad_touched[0], pad_touched[1], pad_touched[2], pad_touched[3],
                touch_values[0], touch_values[1], touch_values[2], touch_values[3]);
    }
    return changed;
}

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
    }
}

uint8_t midi_wire_channel_from_user_channel(uint8_t user_channel)
{
    return static_cast<uint8_t>(MIDI_WIRE_CHANNEL_MIN + (user_channel - MIDI_USER_CHANNEL_MIN));
}

void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value)
{
    if (channel < MIDI_USER_CHANNEL_MIN || channel > MIDI_USER_CHANNEL_MAX)
    {
        ESP_LOGE("MIDI", "Invalid MIDI channel: %d", channel);
        return;
    }
    const uint8_t wire_channel = midi_wire_channel_from_user_channel(channel);
    uint8_t midi_msg[3];
    midi_msg[0] = MIDI_CONTROL_CHANGE_CMD | wire_channel;
    midi_msg[1] = cc_num & MIDI_DATA_BYTE_MASK;
    midi_msg[2] = value & MIDI_DATA_BYTE_MASK;
    send_midi_message(midi_msg, sizeof(midi_msg));
}

void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb)
{
    send_midi_cc(channel, MIDI_CC_NRPN_PARAM_MSB, nrpn_msb);
    send_midi_cc(channel, MIDI_CC_NRPN_PARAM_LSB, nrpn_lsb);
    send_midi_cc(channel, MIDI_CC_DATA_ENTRY_MSB, value_msb);
    send_midi_cc(channel, MIDI_CC_DATA_ENTRY_LSB, 0);
    send_midi_cc(channel, MIDI_CC_RPN_PARAM_MSB, MIDI_CC_PARAM_DESELECT);
    send_midi_cc(channel, MIDI_CC_RPN_PARAM_LSB, MIDI_CC_PARAM_DESELECT);
}

void update_input_state(InputEvent& event)
{
    const uint64_t current_time_us = esp_timer_get_time();
    event.timestamp_us = current_time_us;

    static int debug_log_counter = 0;
    uint16_t all_touch_values[NUM_TOUCH_PADS] = {0};
    static uint16_t consecutive_touched[NUM_TOUCH_PADS] = {0};
    static uint16_t consecutive_released[NUM_TOUCH_PADS] = {0};
    const uint16_t DEBOUNCE_COUNT = 2;
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        event.pad_held[i] = last_touch_state[i];
        event.pad_pressed_this_tick[i] = false;
    }
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        uint16_t touch_value;
        if (read_touch_pad(i, &touch_value) == ESP_OK) {
            all_touch_values[i] = touch_value;
            bool raw_touch_state = (touch_value < TOUCH_THRESHOLD);
            if (raw_touch_state) {
                consecutive_touched[i]++;
                consecutive_released[i] = 0;
            } else {
                consecutive_released[i]++;
                consecutive_touched[i] = 0;
            }
            bool current_pad_state = last_touch_state[i];
            if (consecutive_touched[i] >= DEBOUNCE_COUNT) {
                current_pad_state = true;
            } else if (consecutive_released[i] >= DEBOUNCE_COUNT) {
                current_pad_state = false;
            }
            event.pad_pressed_this_tick[i] = current_pad_state && !last_touch_state[i];
            if (current_pad_state != last_touch_state[i]) {
                ESP_LOGI(TAG, "TouchPad[%d]: State change to %s (value=%d, raw_state=%s)",
                        i, current_pad_state ? "PRESSED" : "RELEASED", touch_value,
                        raw_touch_state ? "TOUCHED" : "RELEASED");
                last_touch_state[i] = current_pad_state;
            }
            event.pad_held[i] = current_pad_state;
        }
    }
    if (++debug_log_counter >= 500) {
        ESP_LOGI(TAG, "Touch pad values: [%u, %u, %u, %u] (threshold: %d)",
                all_touch_values[0], all_touch_values[1],
                all_touch_values[2], all_touch_values[3],
                TOUCH_THRESHOLD);
        debug_log_counter = 0;
    }

    static int adc_log_counter = 0;
    static uint32_t last_pot_values[NUM_POTS] = {0};
    static uint32_t last_smoothed_pot_values[NUM_POTS] = {0};
    for (int i = 0; i < NUM_POTS; i++) {
        event.pot_moved[i] = false;
        event.pot_delta[i] = 0;
        event.pot_value[i] = static_cast<int>(last_smoothed_pot_values[i]);
    }
    for (int i = 0; i < NUM_POTS; i++) {
        int adc_reading;
        if (read_adc(i, &adc_reading) == ESP_OK) {
            const float SMOOTH_FACTOR = 0.5f;
            uint32_t pot_value = last_pot_values[i] * (1.0f - SMOOTH_FACTOR) + adc_reading * SMOOTH_FACTOR;
            last_pot_values[i] = pot_value;
            int scaled_value = (int)((pot_value * MIDI_VALUE_MAX) / ADC_RAW_MAX);
            int delta = scaled_value - static_cast<int>(last_smoothed_pot_values[i]);
            if (abs(delta) > MIDI_CC_THRESHOLD) {
                event.pot_delta[i] = delta;
                last_smoothed_pot_values[i] = scaled_value;
                if (++adc_log_counter % 20 == 0) {
                    ESP_LOGD(TAG, "Pot %d: raw=%lu, scaled=%d, delta=%d",
                            i, pot_value, scaled_value, delta);
                }
            } else {
                event.pot_delta[i] = 0;
            }
            event.pot_value[i] = scaled_value;
            event.pot_moved[i] = (abs(delta) > MIDI_CC_THRESHOLD);
        }
    }
}

void debug_potentiometer_ranges(int duration_ms)
{
    ESP_LOGI(TAG, "Starting potentiometer range calibration for %d ms", duration_ms);
    ESP_LOGI(TAG, "Please rotate both potentiometers through their full range");
    uint32_t min_values[NUM_POTS] = {UINT32_MAX, UINT32_MAX};
    uint32_t max_values[NUM_POTS] = {0, 0};
    uint32_t start_time = esp_timer_get_time() / 1000;
    uint32_t end_time = start_time + duration_ms;
    while ((esp_timer_get_time() / 1000) < end_time) {
        for (int i = 0; i < NUM_POTS; i++) {
            int adc_reading;
            if (read_adc(i, &adc_reading) == ESP_OK) {
                if (adc_reading < min_values[i]) {
                    min_values[i] = adc_reading;
                }
                if (adc_reading > max_values[i]) {
                    max_values[i] = adc_reading;
                }
            }
        }
        static uint32_t last_log_time = 0;
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_log_time > 500) {
            last_log_time = current_time;
            ESP_LOGI(TAG, "Current ranges - Pot1: [%lu-%lu], Pot2: [%lu-%lu]",
                    min_values[0], max_values[0], min_values[1], max_values[1]);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Potentiometer range calibration complete");
    ESP_LOGI(TAG, "Final ranges - Pot1: [%lu-%lu], Pot2: [%lu-%lu]",
            min_values[0], max_values[0], min_values[1], max_values[1]);
    float pot1_pct = (float)(max_values[0] - min_values[0]) / ADC_RAW_MAX * 100.0f;
    float pot2_pct = (float)(max_values[1] - min_values[1]) / ADC_RAW_MAX * 100.0f;
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
        return MIDI_VALUE_MID;
    }

    int current = hall_sensor_read();
    int range = s_hall_sensor_max - s_hall_sensor_min;

    if (range <= 0) {
        return MIDI_VALUE_MID;
    }

    int normalized = ((current - s_hall_sensor_min) * (max_val - min_val)) / range + min_val;
    return std::max(min_val, std::min(max_val, normalized));
}

#pragma GCC diagnostic pop
