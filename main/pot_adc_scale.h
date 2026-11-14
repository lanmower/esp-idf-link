#pragma once

#include <cstdint>

inline int pot_adc_to_midi(uint32_t raw, int adc_raw_max, int midi_value_max)
{
    if (adc_raw_max <= 0 || midi_value_max <= 0) return 0;
    const uint32_t top = static_cast<uint32_t>(adc_raw_max);
    if (raw >= top) return midi_value_max;
    const uint32_t top_band = top / 128u;
    if (raw + top_band >= top) return midi_value_max;
    const uint32_t limit = static_cast<uint32_t>(midi_value_max);
    const uint32_t rounded = static_cast<uint32_t>(
        (static_cast<uint64_t>(raw) * limit + top / 2) / top);
    return rounded >= limit ? midi_value_max : static_cast<int>(rounded);
}
