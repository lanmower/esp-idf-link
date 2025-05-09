#include "touch_handler.h"
#include "driver/touch_pad.h"
#include "esp_log.h"

// REMOVED unused TAG
// static const char *TAG_TOUCH = "TOUCH_HANDLER";

// Global state for touch pads - These are DEFINED in main.cpp and declared EXTERN in touch_handler.h
// int current_pot_val[2] = {0, 0}; // REMOVED DEFINITION
// int last_pot_val[2] = {0, 0};   // REMOVED DEFINITION

// REMOVED Global definitions moved to main.cpp or elsewhere
// bool current_touch_state[4] = {false, false, false, false};
// bool last_touch_state[4] = {false, false, false, false};
uint64_t pad_press_time[4] = {0, 0, 0, 0}; // This needs to be defined somewhere, likely main or effect_handler? Check usage.
// bool touch_held_state[4] = {false, false, false, false}; // Not clearly used globally

// Placeholder - Touch initialization logic might be in io_helpers.cpp
/*
void touch_init() { ... }
*/

// Placeholder - Touch reading logic might be in io_helpers.cpp
/*
void read_touch_pads() { ... }
*/

// Function to get the time a pad has been held in microseconds
// Uses the global pad_press_time array (declared extern in .h, defined in state_machine.cpp)
uint64_t get_time_held_us(int pad_index, uint64_t current_time_us) {
    if (pad_index < 0 || pad_index >= NUM_TOUCH_PADS) { // Use defined constant
        return 0; // Invalid index
    }
    if (pad_press_time[pad_index] == 0) {
        return 0; // Pad not currently pressed or press time not recorded
    }
    // Check for timer wrap-around / inconsistency
    if (current_time_us < pad_press_time[pad_index]) {
        ESP_LOGW("TIME_HELD", "Timer wrap? current=%llu < press=%llu for pad %d", current_time_us, pad_press_time[pad_index], pad_index);
        return 0; // Return 0 if current time is before press time
    }
    return current_time_us - pad_press_time[pad_index];
}

// Function to get the time a pad has been held in milliseconds
// Calls the microsecond version and converts
uint64_t get_time_held_ms(int pad_index, uint64_t current_time_us) {
    return get_time_held_us(pad_index, current_time_us) / 1000;
} 