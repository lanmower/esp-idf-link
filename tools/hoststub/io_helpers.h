// Host stub of io_helpers.h: the two emitters the bass engine calls.
#pragma once
#include <cstddef>
#include <cstdint>
void send_midi_message(const uint8_t* message, size_t size);
void send_midi_cc(uint8_t channel, uint8_t cc_num, uint8_t value);
