#ifndef EFFECT_HANDLER_H
#define EFFECT_HANDLER_H

#include <ableton/Link.hpp>
#include <chrono>
#include <array>
#include <stdint.h>
#include "main.h"
#include "link_sync.h"
#include "esp_log.h"
#include "midi_helpers.h"
#include <algorithm>

#include "effect_arp.h"
#include "effect_filter.h"
#include "effect_sidechain.h"
#include "lfo_constants.h"
#include "touch_handler.h"

#define SIDECHAIN_PAD_INDEX 0
#define ARP_PAD_INDEX 1
#define DELAY_REVERB_PAD_INDEX 2
#define FILTER_PAD_INDEX 3

enum EffectMode {
    EFFECT_MODE_FILTER,
    EFFECT_MODE_ARP,
    EFFECT_MODE_SIDECHAIN,
    EFFECT_MODE_COUNT
};

enum ControlContext {
    NONE,
    FILTER_ADJUST,
    ARP_ADJUST,
    SIDECHAIN_ADJUST,
    DELAY_ADJUST,
    REVERB_ADJUST,
};

extern EffectMode g_current_effect_mode;
extern ControlContext g_current_control_context;
extern ControlContext g_last_pot_control_context;
extern SynthInterface* g_current_synth;
extern SynthType g_synth_type;

extern int s_global_filter_cutoff;
extern int s_global_filter_resonance;

extern int s_current_sidechain_depth;
extern int s_current_sidechain_sheer;

extern int s_current_delay_time;
extern int s_current_delay_feedback;
extern int s_current_reverb_decay;
extern int s_current_reverb_damping;

void initialize_effects();

void dispatch_pot_controls(int pot1_delta, int pot2_delta);

void handle_default_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_filter_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_sidechain_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_delay_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_reverb_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);

bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);
bool handle_sidechain_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);
bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);
bool handle_delay_reverb_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);

bool handle_arp_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int note_index);
void handle_sidechain_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int depth, int sheer);
void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time);

int find_secondary_tapped_pad(int primary_index, const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);

template<typename T>
bool _update_pot_param(T& param, int delta, T min_val, T max_val, const char* param_name, bool& used_flag) {
    if (delta != 0) {
        used_flag = true;
    } else {
        return false;
    }

    T old_val = param;

    T new_val_unclamped = param + delta;

    T new_val = std::max(min_val, std::min(max_val, new_val_unclamped));

    if (new_val != old_val) {
        param = new_val;

        if constexpr (std::is_same_v<T, int> || std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>) {
            ESP_LOGD("POT_UPDATE", "%s: %d (Delta: %d)", param_name, static_cast<int>(param), delta);
        } else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
            ESP_LOGD("POT_UPDATE", "%s: %.2f (Delta: %d)", param_name, static_cast<double>(param), delta);
        } else {
            ESP_LOGD("POT_UPDATE", "%s updated (Delta: %d)", param_name, delta);
        }
        return true;
    }

    return false;
}

#endif
