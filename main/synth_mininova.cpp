#include "synth_mininova.h"
#include "midi_helpers.h" // For send_midi_cc, send_midi_nrpn
#include "main.h" // For MIDI command constants
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // For vTaskDelay
#include "esp_log.h" // For logging if needed

// Define the logging tag for this file
static const char *TAG_MININOVA = "SYNTH_MININOVA";

// --- Mininova Specific MIDI Constants ---
// Moved from main.h and effect_filter.cpp
namespace {
    // General MIDI
    // const uint8_t MIDI_NOTE_OFF_CMD = 0x80; // Removed - Defined as macro in main.h
    const uint8_t MIDI_ALL_NOTES_OFF_CC = 123;

    // FX Routing Type
    const uint8_t FX_ROUTING_NRPN_MSB = 0;
    const uint8_t FX_ROUTING_NRPN_LSB = 98;
    const uint8_t FX_ROUTING_TYPE_1 = 1;  // 0-indexed, second routing type

    // Sidechain
    const uint8_t SIDECHAIN_CC = 59; // PostFXLevel used as ducking control

    // Gate Effect (for sidechain)
    const uint8_t GATE_SELECT_NRPN_MSB = 0;
    const uint8_t GATE_SELECT_NRPN_LSB = 97;
    const uint8_t GATE_EFFECT_CODE = 1;  // Gate effect code

    // Gate Parameters
    const uint8_t GATE_HOLD_NRPN_MSB = 1;
    const uint8_t GATE_HOLD_NRPN_LSB = 1;
    const uint8_t GATE_ESLEW_NRPN_MSB = 1;
    const uint8_t GATE_ESLEW_NRPN_LSB = 2;
    const uint8_t GATE_KEYSYNC_NRPN_MSB = 1;
    const uint8_t GATE_KEYSYNC_NRPN_LSB = 3;
    const uint8_t GATE_DELAY_NRPN_MSB = 1;
    const uint8_t GATE_DELAY_NRPN_LSB = 4;
    const uint8_t GATE_RSYNC_NRPN_MSB = 1;
    const uint8_t GATE_RSYNC_NRPN_LSB = 5;
    const uint8_t GATE_LVL1_NRPN_MSB = 1;
    const uint8_t GATE_LVL1_NRPN_LSB = 10;
    const uint8_t GATE_LVL2_NRPN_MSB = 1;
    const uint8_t GATE_LVL2_NRPN_LSB = 11;
    const uint8_t GATE_LVL3_NRPN_MSB = 1;
    const uint8_t GATE_LVL3_NRPN_LSB = 12;
    const uint8_t GATE_LVL4_NRPN_MSB = 1;
    const uint8_t GATE_LVL4_NRPN_LSB = 13;
    const uint8_t GATE_LVL5_NRPN_MSB = 1;
    const uint8_t GATE_LVL5_NRPN_LSB = 14;
    const uint8_t GATE_LVL6_NRPN_MSB = 1;
    const uint8_t GATE_LVL6_NRPN_LSB = 15;
    const uint8_t GATE_LVL7_NRPN_MSB = 1;
    const uint8_t GATE_LVL7_NRPN_LSB = 16;
    const uint8_t GATE_LVL8_NRPN_MSB = 1;
    const uint8_t GATE_LVL8_NRPN_LSB = 17;

    // Delay/Reverb FX Slot 1 Control
    const uint8_t FX1_SELECT_NRPN_MSB = 0;
    const uint8_t FX1_SELECT_NRPN_LSB = 99;
    const uint8_t FX1_EFFECT_DELAY1 = 6;
    const uint8_t FX1_EFFECT_REVERB1 = 8;
    const uint8_t FX1_LEVEL_CC = 91;

    // Delay 1 Parameters
    const uint8_t DELAY1_TIME_NRPN_MSB = 1;
    const uint8_t DELAY1_TIME_NRPN_LSB = 6;
    const uint8_t DELAY1_FEEDBACK_NRPN_MSB = 1;
    const uint8_t DELAY1_FEEDBACK_NRPN_LSB = 8;
    const uint8_t DELAY1_SYNC_NRPN_MSB = 1;
    const uint8_t DELAY1_SYNC_NRPN_LSB = 7;

    // Reverb 1 Parameters
    const uint8_t REVERB1_DECAY_NRPN_MSB = 1;
    const uint8_t REVERB1_DECAY_NRPN_LSB = 19;
    const uint8_t REVERB1_DAMPING_NRPN_MSB = 1;
    const uint8_t REVERB1_DAMPING_NRPN_LSB = 20;

    // Filter 1 Parameters
    const uint8_t FILT1_FREQ_CC = 74;
    const uint8_t FILT1_RES_CC = 71;
    const uint8_t FILT1_TYPE_NRPN_MSB = 0;
    const uint8_t FILT1_TYPE_NRPN_LSB = 68;
    const uint8_t FILT1_TYPE_LP24 = 3;

    // LFO 2 Parameters
    const uint8_t LFO2_SHAPE_NRPN_MSB = 0;
    const uint8_t LFO2_SHAPE_NRPN_LSB = 40;
    const uint8_t LFO2_SYNC_ENABLE_NRPN_MSB = 1; // Assumed
    const uint8_t LFO2_SYNC_ENABLE_NRPN_LSB = 40; // Assumed
    const uint8_t LFO2_RATE_SYNC_NRPN_MSB = 0;
    const uint8_t LFO2_RATE_SYNC_NRPN_LSB = 86;

    // Mod Matrix Slot 1 Parameters
    const uint8_t MOD1_SOURCE_NRPN_MSB = 0;
    const uint8_t MOD1_SOURCE_NRPN_LSB = 56;
    const uint8_t MOD1_DEST_NRPN_MSB = 0;
    const uint8_t MOD1_DEST_NRPN_LSB = 57;
    const uint8_t MOD1_DEPTH_NRPN_MSB = 0;
    const uint8_t MOD1_DEPTH_NRPN_LSB = 58;

    // Mod Matrix Source/Destination Values
    const uint8_t MOD_SRC_LFO2 = 8;         // LFO2+
    const uint8_t MOD_DEST_FILT1_FREQ = 21; // Filter 1 Freq
} // end anonymous namespace

// --- Constructor ---
SynthMininova::SynthMininova(uint8_t channel) : midi_channel(channel) {
    if (midi_channel == 0 || midi_channel > 16) {
        ESP_LOGW(TAG_MININOVA, "Invalid MIDI channel %d, defaulting to 1", channel);
        midi_channel = 1;
    }
}

// --- Interface Implementation ---

void SynthMininova::sendNoteOff(uint8_t note, uint8_t velocity) {
    uint8_t note_off_msg[] = {static_cast<uint8_t>(MIDI_NOTE_OFF_CMD | (midi_channel - 1)), note, velocity};
    send_midi_message(note_off_msg, sizeof(note_off_msg));
}

void SynthMininova::sendAllNotesOff() {
    send_midi_cc(midi_channel, MIDI_ALL_NOTES_OFF_CC, 0);
}

void SynthMininova::sendControlChange(uint8_t controller, uint8_t value) {
    send_midi_cc(midi_channel, controller, value);
}

void SynthMininova::setSidechainLevel(uint8_t level) {
    ESP_LOGD(TAG_MININOVA, "Setting Sidechain Level (CC 59): %d", level);
    send_midi_cc(midi_channel, SIDECHAIN_CC, level);
}

void SynthMininova::setSidechainPattern(uint8_t pattern_index) {
    ESP_LOGI(TAG_MININOVA, "Setting Sidechain Pattern (Placeholder NRPN 1:80): %d", pattern_index);
    
    // First, ensure the effect routing is set to type 1 (second slot)
    send_midi_nrpn(midi_channel, FX_ROUTING_NRPN_MSB, FX_ROUTING_NRPN_LSB, FX_ROUTING_TYPE_1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Set Gate as the first effect in the chain
    send_midi_nrpn(midi_channel, GATE_SELECT_NRPN_MSB, GATE_SELECT_NRPN_LSB, GATE_EFFECT_CODE);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Configure gate effect parameters
    // Hold: 73
    send_midi_nrpn(midi_channel, GATE_HOLD_NRPN_MSB, GATE_HOLD_NRPN_LSB, 73);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // E-Slew: 104
    send_midi_nrpn(midi_channel, GATE_ESLEW_NRPN_MSB, GATE_ESLEW_NRPN_LSB, 104);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Keysync: On (1)
    send_midi_nrpn(midi_channel, GATE_KEYSYNC_NRPN_MSB, GATE_KEYSYNC_NRPN_LSB, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Delay: -15 (convert to 0-127 range, 0 = max negative)
    send_midi_nrpn(midi_channel, GATE_DELAY_NRPN_MSB, GATE_DELAY_NRPN_LSB, 49);  // -15 mapped to 0-127 range
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // R-Sync: 2nd
    send_midi_nrpn(midi_channel, GATE_RSYNC_NRPN_MSB, GATE_RSYNC_NRPN_LSB, 2);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Set gate levels based on pattern index
    // GtLvl1: 0 (default always 0)
    send_midi_nrpn(midi_channel, GATE_LVL1_NRPN_MSB, GATE_LVL1_NRPN_LSB, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Configure different gate patterns based on the pattern index
    switch (pattern_index) {
        case 0:  // Basic 4/4 house pattern
            send_midi_nrpn(midi_channel, GATE_LVL2_NRPN_MSB, GATE_LVL2_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL3_NRPN_MSB, GATE_LVL3_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL4_NRPN_MSB, GATE_LVL4_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL5_NRPN_MSB, GATE_LVL5_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL6_NRPN_MSB, GATE_LVL6_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL7_NRPN_MSB, GATE_LVL7_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL8_NRPN_MSB, GATE_LVL8_NRPN_LSB, 127);
            break;
        case 1:  // Techno pattern with gaps
            send_midi_nrpn(midi_channel, GATE_LVL2_NRPN_MSB, GATE_LVL2_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL3_NRPN_MSB, GATE_LVL3_NRPN_LSB, 0);
            send_midi_nrpn(midi_channel, GATE_LVL4_NRPN_MSB, GATE_LVL4_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL5_NRPN_MSB, GATE_LVL5_NRPN_LSB, 0);
            send_midi_nrpn(midi_channel, GATE_LVL6_NRPN_MSB, GATE_LVL6_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL7_NRPN_MSB, GATE_LVL7_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL8_NRPN_MSB, GATE_LVL8_NRPN_LSB, 0);
            break;
        case 2:  // Complex pattern
            send_midi_nrpn(midi_channel, GATE_LVL2_NRPN_MSB, GATE_LVL2_NRPN_LSB, 100);
            send_midi_nrpn(midi_channel, GATE_LVL3_NRPN_MSB, GATE_LVL3_NRPN_LSB, 50);
            send_midi_nrpn(midi_channel, GATE_LVL4_NRPN_MSB, GATE_LVL4_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL5_NRPN_MSB, GATE_LVL5_NRPN_LSB, 80);
            send_midi_nrpn(midi_channel, GATE_LVL6_NRPN_MSB, GATE_LVL6_NRPN_LSB, 30);
            send_midi_nrpn(midi_channel, GATE_LVL7_NRPN_MSB, GATE_LVL7_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL8_NRPN_MSB, GATE_LVL8_NRPN_LSB, 60);
            break;
        case 3:  // Fast pumping pattern
            send_midi_nrpn(midi_channel, GATE_LVL2_NRPN_MSB, GATE_LVL2_NRPN_LSB, 0);
            send_midi_nrpn(midi_channel, GATE_LVL3_NRPN_MSB, GATE_LVL3_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL4_NRPN_MSB, GATE_LVL4_NRPN_LSB, 0);
            send_midi_nrpn(midi_channel, GATE_LVL5_NRPN_MSB, GATE_LVL5_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL6_NRPN_MSB, GATE_LVL6_NRPN_LSB, 0);
            send_midi_nrpn(midi_channel, GATE_LVL7_NRPN_MSB, GATE_LVL7_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL8_NRPN_MSB, GATE_LVL8_NRPN_LSB, 0);
            break;
        default:  // Default pattern (all full)
            send_midi_nrpn(midi_channel, GATE_LVL2_NRPN_MSB, GATE_LVL2_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL3_NRPN_MSB, GATE_LVL3_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL4_NRPN_MSB, GATE_LVL4_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL5_NRPN_MSB, GATE_LVL5_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL6_NRPN_MSB, GATE_LVL6_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL7_NRPN_MSB, GATE_LVL7_NRPN_LSB, 127);
            send_midi_nrpn(midi_channel, GATE_LVL8_NRPN_MSB, GATE_LVL8_NRPN_LSB, 127);
            break;
    }
}

void SynthMininova::activateDelay() {
    // First, ensure the effect routing is set to type 1 (second slot)
    send_midi_nrpn(midi_channel, FX_ROUTING_NRPN_MSB, FX_ROUTING_NRPN_LSB, FX_ROUTING_TYPE_1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Set delay as the second effect in the chain
    send_midi_nrpn(midi_channel, FX1_SELECT_NRPN_MSB, FX1_SELECT_NRPN_LSB, FX1_EFFECT_DELAY1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void SynthMininova::activateReverb() {
    // First, ensure the effect routing is set to type 1 (second slot)
    send_midi_nrpn(midi_channel, FX_ROUTING_NRPN_MSB, FX_ROUTING_NRPN_LSB, FX_ROUTING_TYPE_1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Set reverb as the third effect in the chain
    send_midi_nrpn(midi_channel, FX1_SELECT_NRPN_MSB, FX1_SELECT_NRPN_LSB, FX1_EFFECT_REVERB1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void SynthMininova::setDelayTime(uint8_t value) {
    send_midi_nrpn(midi_channel, DELAY1_TIME_NRPN_MSB, DELAY1_TIME_NRPN_LSB, value);
}

void SynthMininova::setDelayFeedback(uint8_t value) {
    send_midi_nrpn(midi_channel, DELAY1_FEEDBACK_NRPN_MSB, DELAY1_FEEDBACK_NRPN_LSB, value);
}

void SynthMininova::setDelaySyncRate(uint8_t rate_val) {
    send_midi_nrpn(midi_channel, DELAY1_SYNC_NRPN_MSB, DELAY1_SYNC_NRPN_LSB, rate_val);
}

void SynthMininova::disableDelaySync() {
    send_midi_nrpn(midi_channel, DELAY1_SYNC_NRPN_MSB, DELAY1_SYNC_NRPN_LSB, 0); // Value 0 disables sync
}

void SynthMininova::setReverbDecay(uint8_t value) {
    send_midi_nrpn(midi_channel, REVERB1_DECAY_NRPN_MSB, REVERB1_DECAY_NRPN_LSB, value);
}

void SynthMininova::setReverbDamping(uint8_t value) {
    send_midi_nrpn(midi_channel, REVERB1_DAMPING_NRPN_MSB, REVERB1_DAMPING_NRPN_LSB, value);
}

void SynthMininova::selectFxSlot1Effect(EffectType type) {
    if (type == EFFECT_DELAY) {
        activateDelay();
    } else if (type == EFFECT_REVERB) {
        activateReverb();
    }
}

void SynthMininova::setFxSlot1Level(uint8_t level) {
    send_midi_cc(midi_channel, FX1_LEVEL_CC, level);
}

void SynthMininova::activateFilter(uint8_t default_cutoff, uint8_t default_res) {
    // Set Filter Type to LP24
    send_midi_nrpn(midi_channel, FILT1_TYPE_NRPN_MSB, FILT1_TYPE_NRPN_LSB, FILT1_TYPE_LP24);
    vTaskDelay(pdMS_TO_TICKS(1));
    // Set default Cutoff and Resonance
    setFilterCutoff(default_cutoff);
    setFilterResonance(default_res);
}

void SynthMininova::deactivateFilter() {
    setFilterCutoff(127); // Set cutoff fully open
    setFilterResonance(0);
    // Maybe set filter routing to bypass? Requires NRPN 0/60 value 0
    // send_midi_nrpn(midi_channel, 0, 60, 0); // Optional: Bypass filter
    unpatchLfoFromFilter(); // Ensure LFO is unpatched
}

void SynthMininova::setFilterCutoff(uint8_t value) {
    send_midi_cc(midi_channel, FILT1_FREQ_CC, value);
}

void SynthMininova::setFilterResonance(uint8_t value) {
    send_midi_cc(midi_channel, FILT1_RES_CC, value);
}

void SynthMininova::patchLfoToFilter(uint8_t initial_depth_midi) {
    ESP_LOGD(TAG_MININOVA, "Patching LFO2 -> Filter Freq (Mod Slot 1)");
    send_midi_nrpn(midi_channel, MOD1_SOURCE_NRPN_MSB, MOD1_SOURCE_NRPN_LSB, MOD_SRC_LFO2);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_midi_nrpn(midi_channel, MOD1_DEST_NRPN_MSB, MOD1_DEST_NRPN_LSB, MOD_DEST_FILT1_FREQ);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_midi_nrpn(midi_channel, MOD1_DEPTH_NRPN_MSB, MOD1_DEPTH_NRPN_LSB, initial_depth_midi);
    vTaskDelay(pdMS_TO_TICKS(1));
    setLfoSyncEnabled(true); // Ensure LFO sync is ON when patching
}

void SynthMininova::unpatchLfoFromFilter() {
    ESP_LOGD(TAG_MININOVA, "Unpatching LFO from Filter (Mod Slot 1 Depth = 64)");
    // Set depth to 0 (MIDI value 64) to disable modulation
    send_midi_nrpn(midi_channel, MOD1_DEPTH_NRPN_MSB, MOD1_DEPTH_NRPN_LSB, 64);
}

void SynthMininova::setLfoShape(uint8_t shape_val) {
    send_midi_nrpn(midi_channel, LFO2_SHAPE_NRPN_MSB, LFO2_SHAPE_NRPN_LSB, shape_val);
}

void SynthMininova::setLfoRateSync(uint8_t rate_val) {
    send_midi_nrpn(midi_channel, LFO2_RATE_SYNC_NRPN_MSB, LFO2_RATE_SYNC_NRPN_LSB, rate_val);
    // Ensure sync is enabled if we are setting a sync rate
    setLfoSyncEnabled(true);
}

void SynthMininova::setLfoDepth(int8_t signed_depth) {
    // Convert signed -64 to +63 to MIDI 0-127
    int midi_depth = signed_depth + 64;
    if (midi_depth < 0) midi_depth = 0;
    if (midi_depth > 127) midi_depth = 127;
    send_midi_nrpn(midi_channel, MOD1_DEPTH_NRPN_MSB, MOD1_DEPTH_NRPN_LSB, (uint8_t)midi_depth);
}

void SynthMininova::setLfoSyncEnabled(bool enabled) {
    send_midi_nrpn(midi_channel, LFO2_SYNC_ENABLE_NRPN_MSB, LFO2_SYNC_ENABLE_NRPN_LSB, enabled ? 1 : 0);
}

void SynthMininova::sendNoteOn(uint8_t note, uint8_t velocity) {
    ESP_LOGV(TAG_MININOVA, "Sending Note ON: Note=%d, Vel=%d, Chan=%d", note, velocity, midi_channel);
    if (note > 127) note = 127;
    if (velocity > 127) velocity = 127;
    uint8_t note_on_msg[] = { static_cast<uint8_t>(MIDI_NOTE_ON_CMD | (midi_channel - 1)), note, velocity };
    send_midi_message(note_on_msg, sizeof(note_on_msg));
}

void SynthMininova::setGateESlew(uint8_t value) {
    // Set E-Slew parameter for the gate effect
    ESP_LOGD(TAG_MININOVA, "Setting Gate E-Slew: %d", value);
    send_midi_nrpn(midi_channel, GATE_ESLEW_NRPN_MSB, GATE_ESLEW_NRPN_LSB, value);
}

void SynthMininova::setGateWetDry(uint8_t value) {
    // Set Wet/Dry mix for the gate effect (FX1 level)
    ESP_LOGD(TAG_MININOVA, "Setting Gate Wet/Dry: %d", value);
    send_midi_cc(midi_channel, FX1_LEVEL_CC, value);
} 