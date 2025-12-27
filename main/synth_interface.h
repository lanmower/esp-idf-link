#ifndef SYNTH_INTERFACE_H
#define SYNTH_INTERFACE_H

#include <cstdint>
#include "types.h" // Include the types header for EffectType

// Pure virtual base class defining the synthesizer control interface
class SynthInterface {
public:
    virtual ~SynthInterface() = default; // Virtual destructor

    // --- General ---
    virtual void sendNoteOff(uint8_t note, uint8_t velocity = 64) = 0;
    virtual void sendAllNotesOff() = 0; // Good practice
    virtual void sendControlChange(uint8_t controller, uint8_t value) = 0; // Send arbitrary CC
    virtual void sendModWheel(uint8_t value) = 0; // CC 1
    virtual void sendPitchBend(int16_t value) = 0; // 14-bit pitch bend

    // --- Sidechain ---
    virtual void setSidechainPattern(uint8_t pattern_index) = 0; // TODO: Implement in derived classes
    // Note: Sidechain volume is often controlled by modulating amp env or a VCA.
    // Using PostFXLevel CC 59 as per previous implementation for now.
    virtual void setSidechainLevel(uint8_t level) = 0; // 0-127 directly for CC 59

    // --- Arpeggiator ---
    // Add Note On specific method for Arp
    virtual void sendNoteOn(uint8_t note, uint8_t velocity) = 0;

    // --- Delay ---
    virtual void activateDelay() = 0; // Selects Delay 1 in FX Slot 1
    virtual void setDelayTime(uint8_t value) = 0; // 0-127
    virtual void setDelayFeedback(uint8_t value) = 0; // 0-127
    virtual void setDelaySyncRate(uint8_t rate_val) = 0; // Use values from Table 3
    virtual void disableDelaySync() = 0; // Set sync NRPN to 0
    // MicroKorg-specific but added to interface with default empty implementation
    virtual void setDelayDepth(uint8_t value) { /* Default empty implementation */ }

    // --- Reverb ---
    virtual void activateReverb() = 0; // Selects Reverb 1 in FX Slot 1
    virtual void setReverbDecay(uint8_t value) = 0; // 0-127
    virtual void setReverbDamping(uint8_t value) = 0; // 0-127
    // MicroKorg-specific but added to interface with default empty implementations
    virtual void setReverbLevel(uint8_t value) { /* Default empty implementation */ }
    virtual void setReverbTime(uint8_t value) { /* Default empty implementation */ }

    // --- Delay/Reverb FX Slot Control ---
    virtual void selectFxSlot1Effect(EffectType type) = 0; // Use full type definition
    virtual void setFxSlot1Level(uint8_t level) = 0; // Controls CC 91 - Renamed back

    // --- Filter ---
    // 'activate' implicitly selects LP24 type and sets defaults
    virtual void activateFilter(uint8_t default_cutoff = 64, uint8_t default_res = 10) = 0;
    // 'deactivate' sets cutoff/res to 0 and potentially bypasses routing
    virtual void deactivateFilter() = 0;
    virtual void setFilterCutoff(uint8_t value) = 0; // 0-127
    virtual void setFilterResonance(uint8_t value) = 0; // 0-127

    // --- LFO (for Filter Modulation) ---
    // Assumes LFO2 modulating Filter1 Freq via Mod Matrix Slot 1
    virtual void patchLfoToFilter(uint8_t initial_depth_midi = 64) = 0; // Depth is 0-127 MIDI
    virtual void unpatchLfoFromFilter() = 0; // Sets Mod Depth to 0 (MIDI 64)
    virtual void setLfoShape(uint8_t shape_val) = 0; // 0=Sin, 1=Tri, 2=Saw, 3=Sqr, 4=S&H
    virtual void setLfoRateSync(uint8_t rate_val) = 0; // Use values from Table 3
    virtual void setLfoDepth(int8_t signed_depth) = 0; // -64 to +63 -> MIDI 0-127
    virtual void setLfoSyncEnabled(bool enabled) = 0; // Controls LFO sync on/off

    // virtual void selectFxSync(uint8_t syncValue) = 0; // Removed for now
    // virtual void enableFxSync(bool enable) = 0; // Removed for now
};

#endif // SYNTH_INTERFACE_H