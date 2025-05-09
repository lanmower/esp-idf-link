#ifndef SYNTH_MICROKORG_H
#define SYNTH_MICROKORG_H

#include "synth_interface.h"

// Implementation of SynthInterface for Korg MicroKorg
class SynthMicroKorg : public SynthInterface {
public:
    SynthMicroKorg(uint8_t channel = 1) : midi_channel(channel) {}
    virtual ~SynthMicroKorg() override = default;

    // General
    void sendNoteOff(uint8_t note) override;
    void sendAllNotesOff() override;
    void sendNoteOn(uint8_t note, uint8_t velocity) override;

    // Sidechain
    void setSidechainPattern(uint8_t pattern_index) override;
    void setSidechainLevel(uint8_t level) override;

    // Delay
    void activateDelay() override;
    void setDelayTime(uint8_t value) override;
    void setDelayFeedback(uint8_t value) override;
    void setDelaySyncRate(uint8_t rate_val) override;
    void disableDelaySync() override;

    // Reverb (no-op)
    void activateReverb() override {}
    void setReverbDecay(uint8_t) override {}
    void setReverbDamping(uint8_t) override {}

    // Delay/Reverb FX Slot Control
    void selectFxSlot1Effect(EffectType type) override;
    void setFxSlot1Level(uint8_t level) override;

    // Filter
    void activateFilter(uint8_t default_cutoff = 64, uint8_t default_res = 10) override;
    void deactivateFilter() override;
    void setFilterCutoff(uint8_t value) override;
    void setFilterResonance(uint8_t value) override;

    // LFO
    void patchLfoToFilter(uint8_t initial_depth_midi = 64) override;
    void unpatchLfoFromFilter() override;
    void setLfoShape(uint8_t shape_val) override;
    void setLfoRateSync(uint8_t rate_val) override;
    void setLfoDepth(int8_t signed_depth) override;
    void setLfoSyncEnabled(bool enabled) override;

private:
    uint8_t midi_channel;
};

#endif // SYNTH_MICROKORG_H
