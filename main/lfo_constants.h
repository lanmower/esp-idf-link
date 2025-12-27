#ifndef LFO_CONSTANTS_H
#define LFO_CONSTANTS_H

#include <vector>
#include <cstdint>

// LFO Shapes (Indices corresponding to setLfoShape values)
// 0=Sin, 1=Tri, 2=Saw, 3=Sqr, 4=S&H
const int NUM_LFO_SHAPES = 5;

// LFO Sync Rates (Indices corresponding to setLfoRateSync values from Table 3)
// Example subset of Mininova Table 3 NRPN values for LFO Rate Sync (0/86)
const std::vector<uint8_t> LFO_SYNC_RATES = {
    3,  // 4 Bars
    7,  // 2 Bars
    11, // 1 Bar
    15, // 1/2 Dotted
    19, // 1/2 Note
    23, // 1/2 Triplet
    27, // 1/4 Dotted
    31, // 1/4 Note
    35, // 1/4 Triplet
    39, // 1/8 Dotted
    43, // 1/8 Note
    47, // 1/8 Triplet
    51, // 1/16 Dotted
    55, // 1/16 Note
    59, // 1/16 Triplet
    63  // 1/32 Note
};
const int NUM_LFO_SYNC_RATES = LFO_SYNC_RATES.size();

#endif // LFO_CONSTANTS_H 