#ifndef EFFECT_FILTER_H
#define EFFECT_FILTER_H

#include <ableton/Link.hpp>
#include <chrono>
#include <array>
#include <stdint.h>
#include "main.h"

extern bool g_filter_lfo_patched;
extern int8_t s_lfo_depth_bipolar;

bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

void initialize_filter();

void reset_filter_to_lowpass();

#endif
