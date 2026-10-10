#ifndef SYNTH_INTERFACE_H
#define SYNTH_INTERFACE_H

#include <cstdint>
#include "types.h"

class SynthInterface {
public:
    virtual ~SynthInterface() = default;

    // --- General ---
    virtual void sendNoteOff(uint8_t note, uint8_t velocity = 64) = 0;
    virtual void sendAllNotesOff() = 0;
    virtual void sendControlChange(uint8_t controller, uint8_t value) = 0;
    virtual void sendModWheel(uint8_t value) = 0; // CC 1
    virtual void sendPitchBend(int16_t value) = 0; // 14-bit pitch bend

    // --- Sidechain ---
    virtual void setSidechainPattern(uint8_t pattern_index) = 0;
    virtual void setSidechainLevel(uint8_t level) = 0;

    // --- Arpeggiator ---
    virtual void sendNoteOn(uint8_t note, uint8_t velocity) = 0;

    // --- Delay ---
    virtual void activateDelay() = 0; // Selects Delay 1 in FX Slot 1
    virtual void setDelayTime(uint8_t value) = 0;
    virtual void setDelayFeedback(uint8_t value) = 0;
    virtual void setDelaySyncRate(uint8_t rate_val) = 0; // Use values from Table 3
    virtual void disableDelaySync() = 0;
    virtual void setDelayDepth(uint8_t value) { }

    // --- Reverb ---
    virtual void activateReverb() = 0; // Selects Reverb 1 in FX Slot 1
    virtual void setReverbDecay(uint8_t value) = 0;
    virtual void setReverbDamping(uint8_t value) = 0;
    virtual void setReverbLevel(uint8_t value) { }
    virtual void setReverbTime(uint8_t value) { }

    // --- Delay/Reverb FX Slot Control ---
    virtual void selectFxSlot1Effect(EffectType type) = 0;
    virtual void setFxSlot1Level(uint8_t level) = 0; // Controls CC 91

    // --- Filter ---
    // 'activate' implicitly selects LP24 type and sets defaults
    virtual void activateFilter(uint8_t default_cutoff = 64, uint8_t default_res = 10) = 0;
    virtual void deactivateFilter() = 0;
    virtual void setFilterCutoff(uint8_t value) = 0;
    virtual void setFilterResonance(uint8_t value) = 0;

    // --- LFO (for Filter Modulation) ---
    // Assumes LFO2 modulating Filter1 Freq via Mod Matrix Slot 1
    virtual void patchLfoToFilter(uint8_t initial_depth_midi = 64) = 0; // Depth is 0-127 MIDI
    virtual void unpatchLfoFromFilter() = 0; // Sets Mod Depth to 0 (MIDI 64)
    virtual void setLfoShape(uint8_t shape_val) = 0;
    virtual void setLfoRateSync(uint8_t rate_val) = 0; // Use values from Table 3
    virtual void setLfoDepth(int8_t signed_depth) = 0; // -64 to +63 -> MIDI 0-127
    virtual void setLfoSyncEnabled(bool enabled) = 0;
};

#endif // SYNTH_INTERFACE_H
