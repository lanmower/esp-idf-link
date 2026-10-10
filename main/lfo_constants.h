#ifndef LFO_CONSTANTS_H
#define LFO_CONSTANTS_H

#include <vector>
#include <cstdint>

constexpr int LFO_SHAPE_SIN = 0;
constexpr int LFO_SHAPE_TRI = 1;
constexpr int LFO_SHAPE_SAW = 2;
constexpr int LFO_SHAPE_SQR = 3;
constexpr int LFO_SHAPE_SAMPLE_AND_HOLD = 4;
constexpr int LFO_SHAPE_FIRST = LFO_SHAPE_SIN;
constexpr int LFO_SHAPE_LAST = LFO_SHAPE_SAMPLE_AND_HOLD;
constexpr int LFO_SHAPE_COUNT = LFO_SHAPE_LAST - LFO_SHAPE_FIRST + 1;
static_assert(LFO_SHAPE_FIRST == 0,
              "LFO_SHAPE_* index a zero-based shape table, so the run must start at 0");
const int NUM_LFO_SHAPES = LFO_SHAPE_COUNT;
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

constexpr uint8_t LFO_SYNC_RATE_FIRST = LFO_SYNC_RATE_4_BARS;
constexpr uint8_t LFO_SYNC_RATE_LAST = LFO_SYNC_RATE_THIRTYSECOND;
constexpr uint8_t LFO_SYNC_RATE_STRIDE = 4;
constexpr int LFO_SYNC_RATE_COUNT = 16;
static_assert(LFO_SYNC_RATE_LAST - LFO_SYNC_RATE_FIRST ==
                  (LFO_SYNC_RATE_COUNT - 1) * LFO_SYNC_RATE_STRIDE,
              "The LFO sync rates must stay a contiguous stride-4 run from FIRST to LAST");

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

#endif
