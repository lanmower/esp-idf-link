#ifndef IO_HELPERS_H
#define IO_HELPERS_H

#include "main.h"
#include "esp_adc_cal.h"

// Function Prototypes
void init_uart_midi();
void init_adc();
void init_touch_pads();
void setup_buzzer();
void set_buzzer_state(bool on, uint32_t frequency = FREQ_NORMAL);
bool read_controls(
    int pot_vals[NUM_POTS],                 // Output: Scaled value (0-127) based on faster EMA
    int pot_stable_center[NUM_POTS],      // Output: Scaled stable center (0-127) based on slower EMA
    bool touch_state[NUM_TOUCH_PADS],       // Output: Current touch state (true=pressed)
    bool pad_pressed_this_tick[NUM_TOUCH_PADS], // Output: True if pad just pressed this tick
    int last_pot_vals[NUM_POTS],            // Input: Previous tick's pot_vals output
    bool last_touch_state[NUM_TOUCH_PADS]   // Input: Previous tick's touch_state output
);

#endif // IO_HELPERS_H 