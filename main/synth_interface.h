#ifndef SYNTH_INTERFACE_H
#define SYNTH_INTERFACE_H

#include <cstdint>
#include "types.h"

constexpr uint8_t kNoteOffVelocity = 0;
constexpr uint8_t kDefaultFilterCutoff = 64;
constexpr uint8_t kDefaultFilterResonance = 10;
constexpr uint8_t kLfoDepthBipolarZero = 64;

class SynthInterface {
public:
    virtual ~SynthInterface() = default;

    virtual void sendNoteOff(uint8_t note, uint8_t velocity = kNoteOffVelocity) = 0;
    virtual void sendAllNotesOff() = 0;
    virtual void sendControlChange(uint8_t controller, uint8_t value) = 0;
    virtual void sendModWheel(uint8_t value) = 0;
    virtual void sendPitchBend(int16_t value) = 0;

    virtual void setSidechainPattern(uint8_t pattern_index) = 0;
    virtual void setSidechainLevel(uint8_t level) = 0;

    virtual void sendNoteOn(uint8_t note, uint8_t velocity) = 0;

    virtual void activateDelay() = 0;
    virtual void setDelayTime(uint8_t value) = 0;
    virtual void setDelayFeedback(uint8_t value) = 0;
    virtual void setDelaySyncRate(uint8_t rate_val) = 0;
    virtual void disableDelaySync() = 0;
    virtual void setDelayDepth(uint8_t value) { }

    virtual void activateReverb() = 0;
    virtual void setReverbDecay(uint8_t value) = 0;
    virtual void setReverbDamping(uint8_t value) = 0;
    virtual void setReverbLevel(uint8_t value) { }
    virtual void setReverbTime(uint8_t value) { }

    virtual void selectFxSlot1Effect(EffectType type) = 0;
    virtual void setFxSlot1Level(uint8_t level) = 0;

    virtual void activateFilter(uint8_t default_cutoff = kDefaultFilterCutoff,
                                uint8_t default_res = kDefaultFilterResonance) = 0;
    virtual void deactivateFilter() = 0;
    virtual void setFilterCutoff(uint8_t value) = 0;
    virtual void setFilterResonance(uint8_t value) = 0;

    virtual void patchLfoToFilter(uint8_t initial_depth_midi = kLfoDepthBipolarZero) = 0;
    virtual void unpatchLfoFromFilter() = 0;
    virtual void setLfoShape(uint8_t shape_val) = 0;
    virtual void setLfoRateSync(uint8_t rate_val) = 0;
    virtual void setLfoDepth(int8_t signed_depth) = 0;
    virtual void setLfoSyncEnabled(bool enabled) = 0;
};

#endif
