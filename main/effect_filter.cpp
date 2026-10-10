#include "effect_filter.h"
#include "main.h"
#include "synth_interface.h"
#include "effect_handler.h"
#include "esp_log.h"
#include "lfo_constants.h"
#include <cmath>
#include "link_sync.h"

static const char *TAG_FILTER = "EFFECT_FILTER";

bool g_filter_lfo_patched = false;

extern int s_global_filter_cutoff;
extern int s_global_filter_resonance;

extern SynthInterface* g_current_synth;
extern std::unique_ptr<ableton::Link> g_link;

static constexpr int kLfoRateIndexOneBar = 2;
static constexpr int kLfoRateIndexHalfNote = 4;
static constexpr int kLfoRateIndexEighthNote = 10;
static constexpr int kLfoRateIndexSixteenthTriplet = 14;

struct FilterLfoPreset {
    int padIndex;
    int shapeIndex;
    int rateIndex;
    int8_t depthBipolar;
    int resonance;
    const char* description;
};

constexpr std::array<FilterLfoPreset, NUM_TOUCH_PADS - 1> kFilterLfoPresets = {{
    {SIDECHAIN_PAD_INDEX, LFO_SHAPE_SQR, kLfoRateIndexEighthNote, 40, 80,
     "LFO Preset 1: Wobble Bass - Square wave at 1/8 note, high resonance"},
    {ARP_PAD_INDEX, LFO_SHAPE_SIN, kLfoRateIndexOneBar, 50, 30,
     "LFO Preset 2: Smooth Sweep - Sine wave at 1 bar, moderate resonance"},
    {DELAY_REVERB_PAD_INDEX, LFO_SHAPE_TRI, kLfoRateIndexSixteenthTriplet, 30, 50,
     "LFO Preset 3: Fast Rhythmic - Triangle wave at 1/16 triplet, medium resonance"}
}};

constexpr bool filterLfoPresetsBindReachablePads() {
    for (int i = 0; i < static_cast<int>(kFilterLfoPresets.size()); i++) {
        const FilterLfoPreset& preset = kFilterLfoPresets[i];
        if (preset.padIndex < 0 || preset.padIndex >= NUM_TOUCH_PADS) return false;
        if (preset.padIndex == FILTER_PAD_INDEX) return false;
        if (preset.rateIndex < 0 || preset.rateIndex >= NUM_LFO_SYNC_RATES) return false;
    }
    return true;
}

static_assert(filterLfoPresetsBindReachablePads(),
              "Every filter LFO preset must bind to a pad the filter handler can receive and to a declared LFO sync rate");

static int s_lfo_shape_index = LFO_SHAPE_SIN;
static int s_lfo_rate_index = kLfoRateIndexHalfNote;
int8_t s_lfo_depth_bipolar = 63;

void handle_filter_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    if (!g_current_synth || !g_link) return;

    if (g_filter_lfo_patched) {
        double tempo = state.tempo();
        if (tempo <= 0) return;

        QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

        double beat = quantumInfo.sessionBeat;
        const bool quantum_boundary_crossed = quantumInfo.crossedQuantumBoundary;

        if (quantum_boundary_crossed) {
            ESP_LOGI(TAG_FILTER, "Filter: Quantum boundary crossed at beat %.2f", beat);
        }

        double lfo_period_beats = 1.0;
        if (s_lfo_rate_index >= 0 && s_lfo_rate_index < NUM_LFO_SYNC_RATES) {
            lfo_period_beats = LFO_SYNC_RATE_PERIOD_BEATS[s_lfo_rate_index];
        } else {
            ESP_LOGW(TAG_FILTER, "LFO rate index out of bounds for period table!");
        }

        double phase = fmod(beat / lfo_period_beats, 1.0);

        double lfo_value_norm = 0.0;
        int shape_index = s_lfo_shape_index;
        switch (shape_index) {
             case LFO_SHAPE_SIN:
                 lfo_value_norm = 0.5 - 0.5 * cos(phase * 2.0 * M_PI);
                 break;
             case LFO_SHAPE_TRI:
                 lfo_value_norm = 2.0 * ((phase < 0.5) ? phase : 1.0 - phase);
                 break;
             case LFO_SHAPE_SAW:
                 lfo_value_norm = 1.0 - phase;
                 break;
             case LFO_SHAPE_SQR:
                 lfo_value_norm = (phase < 0.5) ? 0.0 : 1.0;
                 break;
            default:
                 lfo_value_norm = 0.5;
        }

        double lfo_mod_value = lfo_value_norm * (double)s_lfo_depth_bipolar;

        double final_cutoff_double = (double)s_global_filter_cutoff + lfo_mod_value;

        int final_cutoff_int = clamp_value((int)round(final_cutoff_double), 0, 127);
        g_current_synth->setFilterCutoff(final_cutoff_int);

        ESP_LOGV(TAG_FILTER, "Filter LFO: Beat=%.2f RateIdx=%d Period=%.2f Ph=%.2f Shp=%d Val=%.2f Mod=%.1f Cutoff=%d (%d)",
                 beat, s_lfo_rate_index, lfo_period_beats, phase, shape_index, lfo_value_norm, lfo_mod_value, final_cutoff_int, s_global_filter_cutoff);

    } else {
         static int last_sent_cutoff = -1;
         if (s_global_filter_cutoff != last_sent_cutoff) {
            g_current_synth->setFilterCutoff(s_global_filter_cutoff);
            last_sent_cutoff = s_global_filter_cutoff;
            ESP_LOGV(TAG_FILTER, "Filter LFO Inactive: Setting base cutoff %d", s_global_filter_cutoff);
         }
    }
}

bool handle_filter_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used)
{
    if (!g_current_synth) return false;

    int tapped_pad = find_secondary_tapped_pad(FILTER_PAD_INDEX, pad_pressed_this_tick, pads_used);

    if (tapped_pad != -1) {
        ESP_LOGD(TAG_FILTER, "Filter Adjust TAP: Pad %d detected", tapped_pad);
        bool adjustment_made = false;

        if (!g_filter_lfo_patched) {
            ESP_LOGD(TAG_FILTER, "Filter Adjust TAP: Patching LFO -> Filter Freq");
            g_current_synth->patchLfoToFilter(s_lfo_depth_bipolar);
            g_filter_lfo_patched = true;
            adjustment_made = true;
        }

        for (const FilterLfoPreset& preset : kFilterLfoPresets) {
            if (preset.padIndex != tapped_pad) continue;

            s_lfo_shape_index = preset.shapeIndex;
            s_lfo_rate_index = preset.rateIndex;
            s_lfo_depth_bipolar = preset.depthBipolar;
            s_global_filter_resonance = preset.resonance;

            g_current_synth->setLfoShape(s_lfo_shape_index);
            g_current_synth->setLfoRateSync(LFO_SYNC_RATES[s_lfo_rate_index]);
            g_current_synth->setLfoSyncEnabled(true);
            g_current_synth->setFilterResonance(s_global_filter_resonance);

            ESP_LOGI(TAG_FILTER, "%s", preset.description);
            adjustment_made = true;
            break;
        }
        return adjustment_made;
    }

    return false;
}

void initialize_filter() {
    g_filter_lfo_patched = false;
    s_lfo_shape_index = LFO_SHAPE_SIN;
    s_lfo_rate_index = kLfoRateIndexHalfNote;
    s_lfo_depth_bipolar = 63;
    ESP_LOGI(TAG_FILTER, "Filter Initialized");
}

void reset_filter_to_lowpass() {
    s_global_filter_cutoff = 64;
    s_global_filter_resonance = 20;

    if (g_filter_lfo_patched && g_current_synth) {
        g_current_synth->unpatchLfoFromFilter();
        g_filter_lfo_patched = false;
    }

    if (g_current_synth) {
        g_current_synth->activateFilter(s_global_filter_cutoff, s_global_filter_resonance);
        ESP_LOGD(TAG_FILTER, "Filter reset to lowpass: Cutoff=%d, Resonance=%d",
                 s_global_filter_cutoff, s_global_filter_resonance);
    }
}
