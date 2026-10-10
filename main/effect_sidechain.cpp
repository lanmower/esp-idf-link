#include "effect_sidechain.h"
#include "midi_helpers.h"
#include "main.h"
#include "synth_interface.h"
#include "esp_log.h"
#include "effect_handler.h"
#include "sidechain_constants.h"
#include "state_machine.h"
#include "link_sync.h"
#include "synth_mininova.h"

static const char *TAG_SC = "SIDECHAIN";

constexpr double SIDECHAIN_SPEED_FACTOR = 4.0;

constexpr double SIDECHAIN_BEATS_PER_STEP = LINK_QUANTUM / (SIDECHAIN_SPEED_FACTOR * SIDECHAIN_RHYTHM_STEPS);
constexpr double SIDECHAIN_BEATS_PER_PATTERN_PASS = SIDECHAIN_BEATS_PER_STEP * SIDECHAIN_RHYTHM_STEPS;

static_assert(SIDECHAIN_BEATS_PER_PATTERN_PASS == static_cast<double>(SIDECHAIN_BEATS_PER_PATTERN),
              "one SIDECHAIN_RHYTHM_STEPS pass no longer spans SIDECHAIN_BEATS_PER_PATTERN beats");

struct SidechainPadPatternBinding {
    int padIndex;
    int patternIndex;
};

constexpr std::array<SidechainPadPatternBinding, NUM_SIDECHAIN_PATTERNS> SIDECHAIN_PAD_PATTERN_BINDINGS = {{
    {SIDECHAIN_PAD_INDEX, SC_PATTERN_INDEX_QUARTER},
    {ARP_PAD_INDEX, SC_PATTERN_INDEX_OFFBEAT_EIGHTH},
    {DELAY_REVERB_PAD_INDEX, SC_PATTERN_INDEX_SYNCOPATED},
    {FILTER_PAD_INDEX, SC_PATTERN_INDEX_FOUR_FLOOR}
}};

constexpr bool sidechainPadPatternBindingsSelectEveryPattern() {
    for (int i = 0; i < static_cast<int>(SIDECHAIN_PAD_PATTERN_BINDINGS.size()); i++) {
        const SidechainPadPatternBinding binding = SIDECHAIN_PAD_PATTERN_BINDINGS[i];

        if (binding.padIndex < 0 || binding.padIndex >= NUM_TOUCH_PADS) return false;

        if (binding.patternIndex < 0 || binding.patternIndex >= NUM_SIDECHAIN_PATTERNS) return false;

        for (int j = 0; j < i; j++) {
            if (SIDECHAIN_PAD_PATTERN_BINDINGS[j].padIndex == binding.padIndex) return false;
            if (SIDECHAIN_PAD_PATTERN_BINDINGS[j].patternIndex == binding.patternIndex) return false;
        }
    }

    return true;
}

static_assert(sidechainPadPatternBindingsSelectEveryPattern(),
              "every sidechain pattern must bind a distinct pad in range, and no pad may bind two patterns");

constexpr int sidechainPatternIndexForPad(int padIndex) {
    for (int i = 0; i < static_cast<int>(SIDECHAIN_PAD_PATTERN_BINDINGS.size()); i++) {
        if (SIDECHAIN_PAD_PATTERN_BINDINGS[i].padIndex == padIndex) {
            return SIDECHAIN_PAD_PATTERN_BINDINGS[i].patternIndex;
        }
    }

    return -1;
}

static int find_tapped_pattern_pad(const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used) {
    for (int pad = 0; pad < NUM_TOUCH_PADS; pad++) {
        if (pad_pressed_this_tick[pad]) {
            pads_used[pad] = true;
            return pad;
        }
    }

    return -1;
}

extern SynthType g_synth_type;

int s_current_sidechain_pattern_index = SIDECHAIN_DEFAULT_PATTERN_INDEX;
static int s_last_sc_step_index = -1;
static float s_current_smoothed_level = 0.0f;

void reset_sidechain_to_default() {
    s_current_sidechain_pattern_index = SIDECHAIN_DEFAULT_PATTERN_INDEX;

    s_last_sc_step_index = -1;

    s_current_sidechain_depth = SIDECHAIN_DEFAULT_DEPTH;

    s_current_sidechain_sheer = 0;

    if (g_current_synth) {
        g_current_synth->setSidechainPattern(s_current_sidechain_pattern_index);

        if (g_synth_type == SYNTH_MININOVA) {
            SynthMininova* mininova = static_cast<SynthMininova*>(g_current_synth);
            mininova->setGateESlew(gateESlewForSheer(s_current_sidechain_sheer));
            mininova->setGateWetDry(gateWetDryForDepth(s_current_sidechain_depth));
        }
    }

    ESP_LOGI(TAG_SC, "Sidechain reset to default: Pattern=%d, Depth=%d, Sheer=%d",
             s_current_sidechain_pattern_index, s_current_sidechain_depth, s_current_sidechain_sheer);
}

bool handle_sidechain_adjusting_pads(const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used)
{
    if (!g_current_synth) return false;

    int tapped_pad = find_tapped_pattern_pad(pad_pressed_this_tick, pads_used);

    if (tapped_pad < 0) return false;

    int new_pattern_index = sidechainPatternIndexForPad(tapped_pad);

    if (new_pattern_index < 0) {
        ESP_LOGW(TAG_SC, "SC Adjust: Pad %d binds no sidechain pattern.", tapped_pad);
        return false;
    }

    if (new_pattern_index == s_current_sidechain_pattern_index) {
        ESP_LOGD(TAG_SC, "SC Adjust: Pad %d tapped, pattern %d already selected.", tapped_pad, new_pattern_index);
        return false;
    }

    s_current_sidechain_pattern_index = new_pattern_index;

    ESP_LOGD(TAG_SC, "SC Adjust TAP: Select Pattern -> %d (via Pad %d)", s_current_sidechain_pattern_index, tapped_pad);

    g_current_synth->setSidechainPattern(s_current_sidechain_pattern_index);

    s_last_sc_step_index = -1;

    return true;
}

void handle_sidechain_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time,
                           int depth, int sheer)
{
    if (!g_current_synth || !g_link) return;

    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    const double phase_within_quantum = quantumInfo.phaseWithinQuantum;
    const double quantum_beat = quantumInfo.sessionBeat;
    const bool quantum_boundary_crossed = quantumInfo.crossedQuantumBoundary;

    const double original_beats_per_step = LINK_QUANTUM / static_cast<double>(SIDECHAIN_RHYTHM_STEPS);

    const double PHASE_OFFSET = SIDECHAIN_BEATS_PER_STEP * SIDECHAIN_PATTERN_LEAD_STEPS;

    double scaled_phase = (phase_within_quantum + PHASE_OFFSET) * SIDECHAIN_SPEED_FACTOR;

    while (scaled_phase >= LINK_QUANTUM * SIDECHAIN_SPEED_FACTOR) {
        scaled_phase -= LINK_QUANTUM * SIDECHAIN_SPEED_FACTOR;
    }

    if (quantum_boundary_crossed) {
        ESP_LOGI(TAG_SC, "SC: Quantum boundary crossed at beat %.2f", quantum_beat);
    }

    int current_step_index = static_cast<int>(floor(scaled_phase / original_beats_per_step)) % SIDECHAIN_RHYTHM_STEPS;
    current_step_index = clamp_value(current_step_index, 0, SIDECHAIN_RHYTHM_STEPS - 1);

    ESP_LOGV(TAG_SC, "SC Quantum Alignment: Beat=%.2f, Phase=%.2f, Offset=%.2f, ScaledPhase=%.2f, Step=%d/%d, SpeedFactor=%.1f",
             quantum_beat, phase_within_quantum, PHASE_OFFSET, scaled_phase, current_step_index, SIDECHAIN_RHYTHM_STEPS, SIDECHAIN_SPEED_FACTOR);

    if (current_step_index != s_last_sc_step_index) {
        ESP_LOGV(TAG_SC, "SC Step Change: %d -> %d (Beat: %.2f)", s_last_sc_step_index, current_step_index, quantum_beat);

        if (s_current_sidechain_pattern_index >= 0 && s_current_sidechain_pattern_index < NUM_SIDECHAIN_PATTERNS) {
            const auto& pattern = SIDECHAIN_PATTERNS[s_current_sidechain_pattern_index];
            bool gate_on = pattern[current_step_index];

            uint8_t min_level_target = 127 - clamp_value(depth, SIDECHAIN_DEPTH_MIN, SIDECHAIN_DEPTH_MAX);
            uint8_t sheer_param = clamp_value(sheer, SIDECHAIN_SHEER_MIN, SIDECHAIN_SHEER_MAX);

            float target_level_float = gate_on ? 127.0f : (float)min_level_target;

            float smoothing_alpha = sheerSmoothingAlpha(sheer);

            s_current_smoothed_level = smoothing_alpha * target_level_float + (1.0f - smoothing_alpha) * s_current_smoothed_level;

            uint8_t final_level = (uint8_t)(s_current_smoothed_level + 0.5f);
            final_level = clamp_value(final_level, (uint8_t)0, (uint8_t)127);

            ESP_LOGV(TAG_SC, "SC Pattern %d, Step %d: Gate=%s, Target=%d, MinLevel=%d, Sheer=%d, Alpha=%.2f, Smoothed=%.1f, Final=%d",
                     s_current_sidechain_pattern_index, current_step_index, gate_on ? "ON" : "OFF",
                     (int)target_level_float, min_level_target,
                     sheer_param, smoothing_alpha, s_current_smoothed_level, final_level);

            g_current_synth->setSidechainLevel(final_level);

        } else {
            ESP_LOGW(TAG_SC, "Invalid sidechain pattern index: %d", s_current_sidechain_pattern_index);
        }

        s_last_sc_step_index = current_step_index;
    }
}
