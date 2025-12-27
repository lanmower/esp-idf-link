#ifndef EFFECT_FILTER_H
#define EFFECT_FILTER_H

#include <ableton/Link.hpp>
#include <chrono>
#include <array>
#include <stdint.h>
#include "main.h" // Include for types like EffectMode, ControlContext if needed

// --- Extern State Variables ---
extern bool g_filter_lfo_patched; // Defined in effect_filter.cpp
extern int8_t s_lfo_depth_bipolar; // Needed for synth switching in state_machine.cpp

// --- Function Prototypes ---

// Secondary pad tap handler (Called by state machine)
bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

// Active logic handler (Called by state machine)
void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

// Initialization function
void initialize_filter();

// Reset filter to default lowpass settings (called when filter is latched)
void reset_filter_to_lowpass();

#endif // EFFECT_FILTER_H