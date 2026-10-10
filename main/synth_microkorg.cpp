#include "synth_microkorg.h"
#include "midi_helpers.h"
#include "main.h"
#include "esp_log.h"

namespace {

constexpr uint8_t kCcModWheel = 1;
constexpr uint8_t kCcVolumeUsedAsSidechainLevel = 7;
constexpr uint8_t kCcDelayOnOff = 12;
constexpr uint8_t kCcDelaySync = 13;
constexpr uint8_t kCcDelayTime = 14;
constexpr uint8_t kCcDelayDepth = 15;
constexpr uint8_t kCcReverbDecayStandIn = 16;
constexpr uint8_t kCcReverbDampingStandIn = 17;
constexpr uint8_t kCcReverbLevelStandIn = 18;
constexpr uint8_t kCcReverbTimeStandIn = 19;
constexpr uint8_t kCcFilterResonance = 71;
constexpr uint8_t kCcFilterCutoff = 74;
constexpr uint8_t kCcLfo2Shape = 75;
constexpr uint8_t kCcLfo2Rate = 76;
constexpr uint8_t kCcLfo2Depth = 77;
constexpr uint8_t kCcLfo2FilterIntensity = 78;
constexpr uint8_t kCcLfo2Sync = 79;
constexpr uint8_t kCcEffect1Depth = 91;
constexpr uint8_t kCcAllNotesOff = 123;

constexpr uint8_t kMidiBipolarZero = 64;
constexpr uint8_t kMidiMax = 127;

}

static const char *TAG_MICROKORG = "SYNTH_MICROKORG";

void SynthMicroKorg::sendNoteOff(uint8_t note, uint8_t velocity) {
    uint8_t note_off_msg[] = {static_cast<uint8_t>(MIDI_NOTE_OFF_CMD | (midi_channel - 1)), note, velocity};
    send_midi_message(note_off_msg, sizeof(note_off_msg));
}

void SynthMicroKorg::sendAllNotesOff() {
    send_midi_cc(midi_channel, kCcAllNotesOff, 0);
}

void SynthMicroKorg::sendControlChange(uint8_t controller, uint8_t value) {
    send_midi_cc(midi_channel, controller, value);
}

void SynthMicroKorg::sendModWheel(uint8_t value) {
    ESP_LOGD(TAG_MICROKORG, "Setting Modwheel (CC 1): %d", value);
    send_midi_cc(midi_channel, kCcModWheel, value);
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
    note = clamp_value(note, (uint8_t)0, (uint8_t)127);
    velocity = clamp_value(velocity, (uint8_t)0, (uint8_t)127);
    uint8_t note_on_msg[] = {static_cast<uint8_t>(MIDI_NOTE_ON_CMD | (midi_channel - 1)), note, velocity};
    send_midi_message(note_on_msg, sizeof(note_on_msg));
}

void SynthMicroKorg::setSidechainPattern(uint8_t pattern_index) {
    ESP_LOGI(TAG_MICROKORG, "setSidechainPattern: pattern %d (no direct support)", pattern_index);
}

void SynthMicroKorg::setSidechainLevel(uint8_t level) {
    send_midi_cc(midi_channel, kCcVolumeUsedAsSidechainLevel, level);
}

void SynthMicroKorg::activateDelay() {
    send_midi_cc(midi_channel, kCcDelayOnOff, kMidiMax);
}

void SynthMicroKorg::setDelayTime(uint8_t value) {
    send_midi_cc(midi_channel, kCcDelayTime, value);
}

void SynthMicroKorg::setDelayFeedback(uint8_t value) {
    send_midi_cc(midi_channel, kCcDelayDepth, value);
}

void SynthMicroKorg::setDelaySyncRate(uint8_t rate_val) {
    send_midi_cc(midi_channel, kCcDelaySync, rate_val);
}

void SynthMicroKorg::disableDelaySync() {
    send_midi_cc(midi_channel, kCcDelaySync, 0);
}

void SynthMicroKorg::setDelayDepth(uint8_t value) {
    send_midi_cc(midi_channel, kCcDelayDepth, value);
}

void SynthMicroKorg::activateReverb() {
    send_midi_cc(midi_channel, kCcModWheel, kMidiMax);
    ESP_LOGI(TAG_MICROKORG, "activateReverb: Using modulation as stand-in (MicroKorg has no reverb)");
}

void SynthMicroKorg::setReverbDecay(uint8_t value) {
    send_midi_cc(midi_channel, kCcReverbDecayStandIn, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbDecay: Using CC 16 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbDamping(uint8_t value) {
    send_midi_cc(midi_channel, kCcReverbDampingStandIn, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbDamping: Using CC 17 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbLevel(uint8_t value) {
    send_midi_cc(midi_channel, kCcReverbLevelStandIn, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbLevel: Using CC 18 as stand-in (value: %d)", value);
}

void SynthMicroKorg::setReverbTime(uint8_t value) {
    send_midi_cc(midi_channel, kCcReverbTimeStandIn, value);
    ESP_LOGI(TAG_MICROKORG, "setReverbTime: Using CC 19 as stand-in (value: %d)", value);
}

void SynthMicroKorg::selectFxSlot1Effect(EffectType type) {
    if (type == EFFECT_DELAY) {
        activateDelay();
    } else {
        ESP_LOGI(TAG_MICROKORG, "selectFxSlot1Effect: Reverb ignored");
    }
}

void SynthMicroKorg::setFxSlot1Level(uint8_t level) {
    send_midi_cc(midi_channel, kCcEffect1Depth, level);
}

void SynthMicroKorg::activateFilter(uint8_t default_cutoff, uint8_t default_res) {
    setFilterCutoff(default_cutoff);
    setFilterResonance(default_res);
}

void SynthMicroKorg::deactivateFilter() {
    setFilterCutoff(kMidiMax);
    setFilterResonance(0);
}

void SynthMicroKorg::setFilterCutoff(uint8_t value) {
    send_midi_cc(midi_channel, kCcFilterCutoff, value);
}

void SynthMicroKorg::setFilterResonance(uint8_t value) {
    send_midi_cc(midi_channel, kCcFilterResonance, value);
}

void SynthMicroKorg::patchLfoToFilter(uint8_t initial_depth_midi) {
    send_midi_cc(midi_channel, kCcLfo2FilterIntensity, initial_depth_midi);
}

void SynthMicroKorg::unpatchLfoFromFilter() {
    send_midi_cc(midi_channel, kCcLfo2FilterIntensity, kMidiBipolarZero);
}

void SynthMicroKorg::setLfoShape(uint8_t shape_val) {
    send_midi_cc(midi_channel, kCcLfo2Shape, shape_val);
}

void SynthMicroKorg::setLfoRateSync(uint8_t rate_val) {
    send_midi_cc(midi_channel, kCcLfo2Rate, rate_val);
}

void SynthMicroKorg::setLfoDepth(int8_t signed_depth) {
    int midi_depth = signed_depth + kMidiBipolarZero;
    if (midi_depth < 0) midi_depth = 0;
    if (midi_depth > 127) midi_depth = 127;
    send_midi_cc(midi_channel, kCcLfo2Depth, (uint8_t)midi_depth);
}

void SynthMicroKorg::setLfoSyncEnabled(bool enabled) {
    send_midi_cc(midi_channel, kCcLfo2Sync, enabled ? 1 : 0);
}
