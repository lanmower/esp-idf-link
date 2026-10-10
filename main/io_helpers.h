#ifndef IO_HELPERS_H
#define IO_HELPERS_H

#include "main.h"
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "state_machine.h"

void init_uart_midi();
void init_adc();
void init_touch_pads();
esp_err_t read_touch_pad(uint8_t pad_num, uint16_t* value);
void setup_buzzer();
void set_buzzer_state(bool on, uint32_t frequency = FREQ_NORMAL);
void prime_buzzer_freq(uint32_t frequency);
bool read_controls(
    int pot_vals[NUM_POTS],
    int pot_stable_center[NUM_POTS],
    bool pad_touched[NUM_TOUCH_PADS],
    bool pad_pressed_this_tick[NUM_TOUCH_PADS],
    int last_pot_vals[NUM_POTS],
    bool last_pad_touched[NUM_TOUCH_PADS]
);

void send_midi_message(const uint8_t *message, size_t size);
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value);
void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb);

void update_input_state(InputEvent& event);

void debug_potentiometer_ranges(int duration_ms);

esp_err_t read_adc(int pot_index, int* adc_value);

void init_hall_sensor();
int read_hall_sensor();
int get_hall_sensor_offset(int min_val, int max_val);

#endif
