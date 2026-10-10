#ifndef MIDI_HELPERS_H
#define MIDI_HELPERS_H

#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"
#include "esp_log.h"
#include <algorithm>

void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value);
void send_midi_nrpn(uint8_t channel, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t value_msb);
void send_midi_message(const uint8_t* message, size_t size);
int scale_pot_value(int pot_value, int min_out, int max_out);
int scale_pot_value_center_bias(int pot_value, int min_out, int center_out, int max_out);

uint8_t scale_pot_value_exp(int input_value, int output_min, int output_max, float exponent, bool invert_input = false);

template<typename T>
T clamp_value(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(max_val, value));
}

#endif
