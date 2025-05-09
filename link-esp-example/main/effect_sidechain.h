#ifndef EFFECT_SIDECHAIN_H
#define EFFECT_SIDECHAIN_H

#include <ableton/Link.hpp>
#include <chrono>
#include <array>
#include <stdint.h>

// Function Prototypes

// Secondary pad tap handler (Called by state machine)
bool handle_sidechain_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

// Active logic handler (Called by state machine)
void handle_sidechain_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int depth, int sheer);

// Reset sidechain to default settings (Called when sidechain is latched)
void reset_sidechain_to_default();

#endif // EFFECT_SIDECHAIN_H