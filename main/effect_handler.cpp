#include "effect_handler.h"
#include <array>
#include <cmath>
#include "touch_handler.h"
#include "esp_log.h"
#include "link_sync.h"
#include "io_helpers.h"
#include "synth_interface.h"
#include "midi_helpers.h"
#include "main.h"
#include "sidechain_constants.h"
#include "arp_constants.h"
#include "lfo_constants.h"
#include "state_machine.h"

#include "effect_arp.h"
#include "effect_filter.h"
#include "effect_sidechain.h"
#include "input_handler.h"
#include "synth_mininova.h"
#include "synth_microkorg.h"

static const char *TAG_EFFECT = "EFFECT_HANDLER";

extern SynthInterface* g_current_synth;
extern SynthType g_synth_type;

int s_global_filter_cutoff = 127;
int s_global_filter_resonance = 0;
int s_current_sidechain_depth = SIDECHAIN_DEFAULT_DEPTH;
int s_current_sidechain_sheer = 0;
int s_current_delay_time = 64;
int s_current_delay_feedback = 64;
int s_current_reverb_decay = 64;
int s_current_reverb_damping = 64;

void initialize_effects() {
    g_arp_latched = false;
    g_filter_latched = true;
    g_sidechain_latched = false;
    g_current_control_context = ControlContext::NONE;
    g_last_pot_control_context = ControlContext::FILTER_ADJUST;
    g_pad_index_being_held_for_adjust = -1;
    g_adjust_hold_press_time_us = 0;
    g_interaction_during_adjust_hold = false;
    g_arp_pad_last_tap_time_us = 0;
    g_current_arp_mode = ARP_MODE_NOTE;
    g_last_pad2_effect = EFFECT_DELAY;

    s_global_filter_cutoff = 127;
    s_global_filter_resonance = 0;
    s_current_sidechain_depth = SIDECHAIN_DEFAULT_DEPTH;
    s_current_sidechain_sheer = 0;
    s_current_delay_time = 64;
    s_current_delay_feedback = 64;
    s_current_reverb_decay = 64;
    s_current_reverb_damping = 64;

    initialize_filter();

    ESP_LOGI(TAG_EFFECT, "Effects initialized. Default Mode: FILTER, Control Context: NONE");
}

int find_secondary_tapped_pad(int primary_index, const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used) {
    for (int j = 0; j < 4; ++j) {
        if (j != primary_index && pad_pressed_this_tick[j]) {
            pads_used[j] = true;
            return j;
        }
    }
    return -1;
}

void handle_sidechain_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;

    _update_pot_param(s_current_sidechain_depth, pot1_delta, 0, 127, "SC Duck Depth", pot1_used);

    _update_pot_param(s_current_sidechain_sheer, pot2_delta, 0, 127, "SC Curve", pot2_used);

    if (g_synth_type == SYNTH_MININOVA) {
        SynthMininova* mininova = static_cast<SynthMininova*>(g_current_synth);

        uint8_t wetdry = gateWetDryForDepth(s_current_sidechain_depth);
        mininova->setGateWetDry(wetdry);

        uint8_t eslew = gateESlewForSheer(s_current_sidechain_sheer);
        mininova->setGateESlew(eslew);
    }
}

void handle_filter_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;

    if (_update_pot_param(s_global_filter_cutoff, pot1_delta, 0, 127, "Filter Cutoff", pot1_used)) {
        if (!g_filter_lfo_patched) {
            g_current_synth->setFilterCutoff(s_global_filter_cutoff);
            ESP_LOGD(TAG_EFFECT, "Filter Cutoff adjusted: %d", s_global_filter_cutoff);
        }
    }

    if (_update_pot_param(s_global_filter_resonance, pot2_delta, 0, 127, "Filter Resonance", pot2_used)) {
        g_current_synth->setFilterResonance(s_global_filter_resonance);
        ESP_LOGD(TAG_EFFECT, "Filter Resonance adjusted: %d", s_global_filter_resonance);
    }
}

void handle_delay_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;
    if(_update_pot_param(s_current_delay_time, pot1_delta, 0, 127, "Delay Time", pot1_used)) {
        g_current_synth->setDelayTime(s_current_delay_time);
    }
    if(_update_pot_param(s_current_delay_feedback, pot2_delta, 0, 127, "Delay Feedback", pot2_used)) {
        g_current_synth->setDelayFeedback(s_current_delay_feedback);
    }
}

void handle_reverb_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (!g_current_synth) return;
     if(_update_pot_param(s_current_reverb_decay, pot1_delta, 0, 127, "Reverb Decay", pot1_used)) {
         g_current_synth->setReverbDecay(s_current_reverb_decay);
     }
    if(_update_pot_param(s_current_reverb_damping, pot2_delta, 0, 127, "Reverb Damping", pot2_used)) {
        g_current_synth->setReverbDamping(s_current_reverb_damping);
    }
}

void handle_default_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    ESP_LOGV(TAG_EFFECT, "Handle Default Pots (Context NONE) - Last Pot Context: %d", (int)g_last_pot_control_context);
    switch(g_last_pot_control_context) {
        case ControlContext::ARP_ADJUST:
            handle_arp_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::SIDECHAIN_ADJUST:
            handle_sidechain_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::DELAY_ADJUST:
            handle_delay_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::REVERB_ADJUST:
            handle_reverb_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::FILTER_ADJUST:
        case ControlContext::NONE:
        default:
            handle_filter_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
    }
}

void dispatch_pot_controls(int pot1_delta, int pot2_delta) {
    if (pot1_delta == 0 && pot2_delta == 0) return;

    bool pot1_used = false;
    bool pot2_used = false;

    switch (g_current_control_context) {
        case ControlContext::ARP_ADJUST:
            handle_arp_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::SIDECHAIN_ADJUST:
            handle_sidechain_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::FILTER_ADJUST:
            handle_filter_adjust_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::DELAY_ADJUST:
             handle_delay_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::REVERB_ADJUST:
             handle_reverb_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
            break;
        case ControlContext::NONE:
        default:
            handle_default_pots(pot1_delta, pot2_delta, pot1_used, pot2_used);
             break;
    }

}

bool handle_delay_reverb_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used) {
    if (!g_current_synth) return false;

    bool any_tap = false;
    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        if (i != DELAY_REVERB_PAD_INDEX && pad_pressed_this_tick[i]) {
            any_tap = true;
            pads_used[i] = true;
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust TAP: Pad %d", i);
        }
    }

    if (any_tap) {
        if (g_last_pad2_effect == EFFECT_DELAY) {
            g_last_pad2_effect = EFFECT_REVERB;
            g_current_synth->selectFxSlot1Effect(EFFECT_REVERB);
            g_last_pot_control_context = ControlContext::REVERB_ADJUST;
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust: Switched to Reverb");
        } else {
            g_last_pad2_effect = EFFECT_DELAY;
            g_current_synth->selectFxSlot1Effect(EFFECT_DELAY);
            g_last_pot_control_context = ControlContext::DELAY_ADJUST;
            ESP_LOGD(TAG_EFFECT, "Delay/Reverb Adjust: Switched to Delay");
        }
        return true;
    }

    return false;
}
