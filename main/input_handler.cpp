#include "input_handler.h"
#include "io_helpers.h"
#include "esp_log.h"
#include <cmath>
#include "link_sync.h"
#include "main.h"
#include "esp_timer.h"

static constexpr int POT_JITTER_THRESHOLD = 3;
static constexpr int POT_REPORT_MIN_DELTA = 1;

static int s_last_reported_pot_val[NUM_POTS] = {-1, -1};
static bool s_previous_touch_state[NUM_TOUCH_PADS] = {false, false, false, false};

void initialize_inputs() {
    ESP_LOGI("Input", "Initializing inputs...");
    InputEvent initial_event;

    read_inputs(initial_event);

    for (int i = 0; i < NUM_POTS; ++i) {
        s_last_reported_pot_val[i] = initial_event.pot_value[i];
    }
    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        s_previous_touch_state[i] = initial_event.pad_held[i];
    }

    ESP_LOGI("Input", "Inputs initialized. Initial Pot0: %d, Pot1: %d", s_last_reported_pot_val[0], s_last_reported_pot_val[1]);
}

bool read_inputs(InputEvent& current_event)
{
    int current_pot_val[NUM_POTS] = {0, 0};
    int current_stable_center[NUM_POTS] = {0, 0};
    bool current_pad_touched[NUM_TOUCH_PADS] = {false, false, false, false};
    bool current_pad_pressed_this_tick[NUM_TOUCH_PADS] = {false, false, false, false};

    current_event.timestamp_us = esp_timer_get_time();

    read_controls(
        current_pot_val,
        current_stable_center,
        current_pad_touched,
        current_pad_pressed_this_tick,
        s_last_reported_pot_val,
        s_previous_touch_state
    );

    bool significant_change_detected = false;

    for (int i = 0; i < NUM_POTS; ++i) {
        int cumulative_delta = current_pot_val[i] - s_last_reported_pot_val[i];

        if (cumulative_delta != 0) {
            bool is_near_center = std::abs(current_pot_val[i] - current_stable_center[i]) <= POT_JITTER_THRESHOLD;

            if (!is_near_center || std::abs(cumulative_delta) > POT_REPORT_MIN_DELTA) {
                current_event.pot_moved[i] = true;
                current_event.pot_delta[i] = cumulative_delta;
                current_event.pot_value[i] = current_pot_val[i];
                s_last_reported_pot_val[i] = current_pot_val[i];
                significant_change_detected = true;

                ESP_LOGV("INPUT", "Pot %d moved: val=%d, delta=%d, near_center=%s",
                         i, current_pot_val[i], cumulative_delta, is_near_center ? "YES" : "NO");
            } else {
                current_event.pot_moved[i] = false;
                current_event.pot_delta[i] = 0;
                current_event.pot_value[i] = s_last_reported_pot_val[i];
            }
        } else {
            current_event.pot_moved[i] = false;
            current_event.pot_delta[i] = 0;
            current_event.pot_value[i] = s_last_reported_pot_val[i];
        }
    }

    for (int i = 0; i < NUM_TOUCH_PADS; ++i) {
        current_event.pad_held[i] = current_pad_touched[i];
        current_event.pad_pressed_this_tick[i] = current_pad_pressed_this_tick[i];

        if (current_event.pad_held[i] != s_previous_touch_state[i] || current_event.pad_pressed_this_tick[i]) {
            significant_change_detected = true;
            ESP_LOGD("INPUT", "Pad %d state: %s%s",
                    i,
                    current_event.pad_held[i] ? "HELD" : "RELEASED",
                    current_event.pad_pressed_this_tick[i] ? " (JUST PRESSED)" : "");
        }

        s_previous_touch_state[i] = current_pad_touched[i];
    }

    return significant_change_detected;
}
