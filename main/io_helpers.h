#ifndef IO_HELPERS_H
#define IO_HELPERS_H

#include "main.h"
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "state_machine.h" // Include for InputEvent definition

// Function Prototypes
void init_uart_midi();
void init_adc();
void init_touch_pads();
esp_err_t read_touch_pad(uint8_t pad_num, uint16_t* value);
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

// MIDI functions
void send_midi_message(const uint8_t *message, size_t size);
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value);
void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb);

// Input state update
void update_input_state(InputEvent& event);

// Debug helper for potentiometer calibration
void debug_potentiometer_ranges(int duration_ms);

// Helper function to read ADC values
esp_err_t read_adc(int pot_index, int* adc_value);

#endif // IO_HELPERS_H 