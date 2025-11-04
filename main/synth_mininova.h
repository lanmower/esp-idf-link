#ifndef SYNTH_MININOVA_H
#define SYNTH_MININOVA_H

#include "synth_interface.h"
// #include "main.h" // Removed - EffectType comes via synth_interface.h -> types.h

// Implementation of SynthInterface for Novation Mininova
class SynthMininova : public SynthInterface {
public:
    SynthMininova(uint8_t channel = 1); // Constructor with MIDI channel
    virtual ~SynthMininova() override = default;

    // Override all pure virtual functions from SynthInterface
    void sendNoteOff(uint8_t note, uint8_t velocity = 64) override;
    void sendAllNotesOff() override;
    void sendControlChange(uint8_t controller, uint8_t value) override;
    void setSidechainLevel(uint8_t level) override;
    void activateDelay() override;
    void setDelayTime(uint8_t value) override;
    void setDelayFeedback(uint8_t value) override;
    void setDelaySyncRate(uint8_t rate_val) override;
    void disableDelaySync() override;
    void activateReverb() override;
    void setReverbDecay(uint8_t value) override;
    void setReverbDamping(uint8_t value) override;
    void selectFxSlot1Effect(EffectType type) override;
    void setFxSlot1Level(uint8_t level) override;
    void activateFilter(uint8_t default_cutoff = 64, uint8_t default_res = 10) override;
    void deactivateFilter() override;
    void setFilterCutoff(uint8_t value) override;
    void setFilterResonance(uint8_t value) override;
    void patchLfoToFilter(uint8_t initial_depth_midi = 64) override;
    void unpatchLfoFromFilter() override;
    void setLfoShape(uint8_t shape_val) override;
    void setLfoRateSync(uint8_t rate_val) override;
    void setLfoDepth(int8_t signed_depth) override;
    void setLfoSyncEnabled(bool enabled) override;
    void sendNoteOn(uint8_t note, uint8_t velocity) override;
    void setSidechainPattern(uint8_t pattern_index) override;
    
    // Additional methods specific to Mininova
    void setGateESlew(uint8_t value);
    void setGateWetDry(uint8_t value);

private:
    uint8_t midi_channel;
};

#endif // SYNTH_MININOVA_H 