#include "synth_microkorg.h"
#include "midi_helpers.h"
#include "main.h"
#include "esp_log.h"

// Define the logging tag for this file
static const char *TAG_MICROKORG = "SYNTH_MICROKORG";

// --- MicroKorg Implementation ---
void SynthMicroKorg::sendNoteOff(uint8_t note, uint8_t velocity) {
    uint8_t note_off_msg[] = {static_cast<uint8_t>(MIDI_NOTE_OFF_CMD | (midi_channel - 1)), note, velocity};
    send_midi_message(note_off_msg, sizeof(note_off_msg));
}

void SynthMicroKorg::sendAllNotesOff() {
    send_midi_cc(midi_channel, 123, 0); // MIDI CC 123: All Notes Off
}

void SynthMicroKorg::sendControlChange(uint8_t controller, uint8_t value) {
    send_midi_cc(midi_channel, controller, value);
}

void SynthMicroKorg::sendModWheel(uint8_t value) {
    ESP_LOGD(TAG_MICROKORG, "Setting Modwheel (CC 1): %d", value);
    send_midi_cc(midi_channel, 1, value);
}

void SynthMicroKorg::sendPitchBend(int16_t value) {
    uint8_t pitch_bend_msg[] = {
        static_cast<uint8_t>(0xE0 | (midi_channel - 1)),
        static_cast<uint8_t>(value & 0x7F),
        static_cast<uint8_t>((value >> 7) & 0x7F)
    };
    send_midi_message(pitch_bend_msg, sizeof(pitch_bend_msg));
    ESP_LOGD(TAG_MICROKORG, "Pitch Bend: %d (LSB: %d, MSB: %d)", value, pitch_bend_msg[1], pitch_bend_msg[2]);
}

void SynthMicroKorg::sendNoteOn(uint8_t note, uint8_t velocity) {
    // Clamp values before sending
    note = clamp_value(note, (uint8_t)0, (uint8_t)127);
    velocity = clamp_value(velocity, (uint8_t)0, (uint8_t)127);
    uint8_t note_on_msg[] = {static_cast<uint8_t>(MIDI_NOTE_ON_CMD | (midi_channel - 1)), note, velocity};
    send_midi_message(note_on_msg, sizeof(note_on_msg));
}

void SynthMicroKorg::setSidechainPattern(uint8_t pattern_index) {
    // No direct support; could be mapped to a CC or ignored
    ESP_LOGI(TAG_MICROKORG, "setSidechainPattern: pattern %d (no direct support)", pattern_index);
}

void SynthMicroKorg::setSidechainLevel(uint8_t level) {
    // Use CC 7 (Volume) as a stand-in for sidechain ducking
    send_midi_cc(midi_channel, 7, level);
}

void SynthMicroKorg::activateDelay() {
    // MicroKorg: Delay On/Off is CC 12 (Effect 1 On/Off)
    send_midi_cc(midi_channel, 12, 127); // 127 = On
}

void SynthMicroKorg::setDelayTime(uint8_t value) {
    // MicroKorg: Delay Time is CC 14 (0-127)
    send_midi_cc(midi_channel, 14, value);
}

void SynthMicroKorg::setDelayFeedback(uint8_t value) {
    // MicroKorg: Delay Depth is CC 15 (0-127)
    send_midi_cc(midi_channel, 15, value);
}

void SynthMicroKorg::setDelaySyncRate(uint8_t rate_val) {
    // MicroKorg: Delay Sync is CC 13 (0=off, 1-32=sync values)
    send_midi_cc(midi_channel, 13, rate_val);
}

void SynthMicroKorg::disableDelaySync() {
    // Set Delay Sync to 0 (off)
    send_midi_cc(midi_channel, 13, 0);
}

void SynthMicroKorg::setDelayDepth(uint8_t value) {
    // MicroKorg: Delay Depth/Feedback is CC 15 (0-127)
    send_midi_cc(midi_channel, 15, value);
}

void SynthMicroKorg::activateReverb() {
    // MicroKorg doesn't have a dedicated reverb, we'll use modulation depth as a stand-in
    // for demonstration purposes (CC 1 - Modulation Wheel)
    send_midi_cc(midi_channel, 1, 127); // 127 = Maximum reverb
    ESP_LOGI(TAG_MICROKORG, "activateReverb: Using modulation as stand-in (MicroKorg has no reverb)");
}

void SynthMicroKorg::setReverbDecay(uint8_t value) {
    // No direct support for reverb on MicroKorg, map to a general CC for demonstration
    // CC 16 is general purpose controller on many synths
    send_midi_cc(midi_channel, 16, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbDecay: Using CC 16 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbDamping(uint8_t value) {
    // No direct support for reverb on MicroKorg, map to a general CC for demonstration
    // CC 17 is general purpose controller on many synths
    send_midi_cc(midi_channel, 17, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbDamping: Using CC 17 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbLevel(uint8_t value) {
    // No direct support for reverb on MicroKorg, use another general CC
    // CC 18 is general purpose controller on many synths
    send_midi_cc(midi_channel, 18, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbLevel: Using CC 18 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbTime(uint8_t value) {
    // No direct support for reverb on MicroKorg, use another general CC
    // CC 19 is general purpose controller on many synths
    send_midi_cc(midi_channel, 19, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbTime: Using CC 19 as stand-in (value: %d)", value);
}

void SynthMicroKorg::selectFxSlot1Effect(EffectType type) {
    // Only Delay is supported, ignore Reverb
    if (type == EFFECT_DELAY) {
        activateDelay();
    } else {
        ESP_LOGI(TAG_MICROKORG, "selectFxSlot1Effect: Reverb ignored");
    }
}

void SynthMicroKorg::setFxSlot1Level(uint8_t level) {
    // Use CC 91 (Effect 1 Depth) for delay level
    send_midi_cc(midi_channel, 91, level);
}

void SynthMicroKorg::activateFilter(uint8_t default_cutoff, uint8_t default_res) {
    setFilterCutoff(default_cutoff);
    setFilterResonance(default_res);
}

void SynthMicroKorg::deactivateFilter() {
    setFilterCutoff(127);
    setFilterResonance(0);
}

void SynthMicroKorg::setFilterCutoff(uint8_t value) {
    // MicroKorg: Filter Cutoff is CC 74
    send_midi_cc(midi_channel, 74, value);
}

void SynthMicroKorg::setFilterResonance(uint8_t value) {
    // MicroKorg: Filter Resonance is CC 71
    send_midi_cc(midi_channel, 71, value);
}

void SynthMicroKorg::patchLfoToFilter(uint8_t initial_depth_midi) {
    // LFO2 to Filter Depth: CC 78 (LFO2 Intensity)
    send_midi_cc(midi_channel, 78, initial_depth_midi);
}

void SynthMicroKorg::unpatchLfoFromFilter() {
    send_midi_cc(midi_channel, 78, 0);
}

void SynthMicroKorg::setLfoShape(uint8_t shape_val) {
    // MicroKorg: LFO1 Shape is CC 73, LFO2 Shape is CC 75
    // We'll use LFO2 (CC 75)
    send_midi_cc(midi_channel, 75, shape_val);
}

void SynthMicroKorg::setLfoRateSync(uint8_t rate_val) {
    // LFO2 Rate: CC 76
    send_midi_cc(midi_channel, 76, rate_val);
}

void SynthMicroKorg::setLfoDepth(int8_t signed_depth) {
    // LFO2 Depth: CC 77
    int midi_depth = signed_depth + 64;
    if (midi_depth < 0) midi_depth = 0;
    if (midi_depth > 127) midi_depth = 127;
    send_midi_cc(midi_channel, 77, (uint8_t)midi_depth);
}

void SynthMicroKorg::setLfoSyncEnabled(bool enabled) {
    // LFO2 Sync: CC 79 (0=off, 1=on)
    send_midi_cc(midi_channel, 79, enabled ? 1 : 0);
}