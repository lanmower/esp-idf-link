#ifndef LFO_CONSTANTS_H
#define LFO_CONSTANTS_H

#include <vector>
#include <cstdint>

// LFO Shapes (Indices corresponding to setLfoShape values)
// 0=Sin, 1=Tri, 2=Saw, 3=Sqr, 4=S&H
const int NUM_LFO_SHAPES = 5;

// LFO Sync Rates (Indices corresponding to setLfoRateSync values from Table 3)
// Example subset of Mininova Table 3 NRPN values for LFO Rate Sync (0/86)
constexpr uint8_t LFO_SYNC_RATE_4_BARS = 3;
constexpr uint8_t LFO_SYNC_RATE_2_BARS = 7;
constexpr uint8_t LFO_SYNC_RATE_1_BAR = 11;
constexpr uint8_t LFO_SYNC_RATE_DOTTED_HALF = 15;
constexpr uint8_t LFO_SYNC_RATE_HALF = 19;
constexpr uint8_t LFO_SYNC_RATE_HALF_TRIPLET = 23;
constexpr uint8_t LFO_SYNC_RATE_DOTTED_QUARTER = 27;
constexpr uint8_t LFO_SYNC_RATE_QUARTER = 31;
constexpr uint8_t LFO_SYNC_RATE_QUARTER_TRIPLET = 35;
constexpr uint8_t LFO_SYNC_RATE_DOTTED_EIGHTH = 39;
constexpr uint8_t LFO_SYNC_RATE_EIGHTH = 43;
constexpr uint8_t LFO_SYNC_RATE_EIGHTH_TRIPLET = 47;
constexpr uint8_t LFO_SYNC_RATE_DOTTED_SIXTEENTH = 51;
constexpr uint8_t LFO_SYNC_RATE_SIXTEENTH = 55;
constexpr uint8_t LFO_SYNC_RATE_SIXTEENTH_TRIPLET = 59;
constexpr uint8_t LFO_SYNC_RATE_THIRTYSECOND = 63;

const std::vector<uint8_t> LFO_SYNC_RATES = {
    LFO_SYNC_RATE_4_BARS,
    LFO_SYNC_RATE_2_BARS,
    LFO_SYNC_RATE_1_BAR,
    LFO_SYNC_RATE_DOTTED_HALF,
    LFO_SYNC_RATE_HALF,
    LFO_SYNC_RATE_HALF_TRIPLET,
    LFO_SYNC_RATE_DOTTED_QUARTER,
    LFO_SYNC_RATE_QUARTER,
    LFO_SYNC_RATE_QUARTER_TRIPLET,
    LFO_SYNC_RATE_DOTTED_EIGHTH,
    LFO_SYNC_RATE_EIGHTH,
    LFO_SYNC_RATE_EIGHTH_TRIPLET,
    LFO_SYNC_RATE_DOTTED_SIXTEENTH,
    LFO_SYNC_RATE_SIXTEENTH,
    LFO_SYNC_RATE_SIXTEENTH_TRIPLET,
    LFO_SYNC_RATE_THIRTYSECOND
};
const int NUM_LFO_SYNC_RATES = LFO_SYNC_RATES.size();

#endif // LFO_CONSTANTS_H
