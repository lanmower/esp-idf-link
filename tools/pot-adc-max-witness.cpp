#include "pot_adc_scale.h"
#include "control_model.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

constexpr int kAdcRawMax = 4095;
constexpr int kMidiMax = 127;

static int g_checks = 0;
static int g_fail = 0;

#define CHECK(name, cond) do { ++g_checks; if (!(cond)) { std::printf("FAIL %s\n", name); ++g_fail; } } while (0)

static int popcount32(uint32_t v) { int n = 0; while (v) { n += static_cast<int>(v & 1u); v >>= 1; } return n; }

static uint32_t steadyEma(uint32_t raw)
{
    uint32_t value = 0;
    for (int i = 0; i < 512; ++i)
        value = static_cast<uint32_t>(value * (1.0f - 0.5f) + raw * 0.5f);
    return value;
}

static int gateNoteOns(float dial)
{
    const uint32_t rhythm = ctl::stepPattern(ctl::rhythmForDial(0.97f));
    const uint32_t series = ctl::tranceGate(ctl::gateSeriesForDial(dial));
    int n = 0;
    for (int step = 0; step < 32; ++step)
        if (((rhythm >> step) & 1u) && ((series >> step) & 1u)) ++n;
    return n;
}

int main()
{
    CHECK("the old truncating scale reported 126 at a physical maximum",
          (steadyEma(kAdcRawMax) * kMidiMax) / kAdcRawMax == 126);
    CHECK("the steady-state EMA settles one raw count short of full scale", steadyEma(kAdcRawMax) == 4094);
    CHECK("physical maximum raw 4095 reports 127", pot_adc_to_midi(4095, kAdcRawMax, kMidiMax) == 127);
    CHECK("steady-state raw 4094 reports 127", pot_adc_to_midi(4094, kAdcRawMax, kMidiMax) == 127);
    CHECK("an ADC saturating at 4080 reports 127", pot_adc_to_midi(4080, kAdcRawMax, kMidiMax) == 127);
    CHECK("raw 0 reports 0", pot_adc_to_midi(0, kAdcRawMax, kMidiMax) == 0);
    CHECK("mid scale reports 64", pot_adc_to_midi(2048, kAdcRawMax, kMidiMax) == 64);

    bool monotone = true;
    int prev = -1;
    for (uint32_t raw = 0; raw <= 4095; ++raw) {
        const int v = pot_adc_to_midi(raw, kAdcRawMax, kMidiMax);
        if (v < prev || v < 0 || v > kMidiMax) monotone = false;
        prev = v;
    }
    CHECK("the ADC scale is monotone and bounded 0..127", monotone);

    const uint32_t pot0 = steadyEma(kAdcRawMax);
    const uint32_t pot1 = steadyEma(kAdcRawMax);
    CHECK("pot0 physical maximum reports 127", pot_adc_to_midi(pot0, kAdcRawMax, kMidiMax) == 127);
    CHECK("pot1 physical maximum reports 127", pot_adc_to_midi(pot1, kAdcRawMax, kMidiMax) == 127);

    const int midi = pot_adc_to_midi(pot0, kAdcRawMax, kMidiMax);
    const float dial = static_cast<float>(midi) / static_cast<float>(kMidiMax);
    CHECK("the fixed mapping reaches the top gate series",
          ctl::gateSeriesForDial(dial) == ctl::kGateSeriesCount - 1);
    CHECK("the top gate series has no onsets",
          ctl::tranceGate(ctl::gateSeriesForDial(dial)) == 0u);
    CHECK("one level below the top leaves one sixteenth",
          popcount32(ctl::tranceGate(ctl::kGateSeriesCount - 2)) == 1);
    CHECK("127 produces zero gate note-ons", gateNoteOns(dial) == 0);
    CHECK("the fully open gate still produces note-ons", gateNoteOns(0.0f) > 0);

    std::printf("POT ADC MAX WITNESS: PASS (%d checks, %d failed)\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
