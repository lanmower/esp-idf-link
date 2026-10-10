#ifndef SIDECHAIN_CONSTANTS_H
#define SIDECHAIN_CONSTANTS_H

#include <vector>
#include <array>

// Define number of steps per pattern (e.g., 16 for 16th notes over 1 bar)
const int SIDECHAIN_RHYTHM_STEPS = 16;

// true = Gate ON (sound allowed), false = Gate OFF (sound ducked)
const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_QUARTER = {
    false, false, true,  true,
    false, false, true,  true,
    false, false, true,  true,
    false, false, true,  true
};

const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_OFFBEAT_EIGHTH = {
    false, false, true,  false, true,  false, true,  false,
    true,  false, true,  false, true,  false, true,  false
};

const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_SYNCOPATED = {
    false, false, true,  false, true,  false, false, true,
    false, false, true,  false, true,  false, false, true
};

const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_FOUR_FLOOR = {
    false, false, true,  true,  false, false, true,  true,
    false, false, true,  true,  false, false, true,  true
};

const std::vector<std::array<bool, SIDECHAIN_RHYTHM_STEPS>> SIDECHAIN_PATTERNS = {
    SC_PATTERN_QUARTER,
    SC_PATTERN_OFFBEAT_EIGHTH,
    SC_PATTERN_SYNCOPATED,
    SC_PATTERN_FOUR_FLOOR
};

const int NUM_SIDECHAIN_PATTERNS = SIDECHAIN_PATTERNS.size();

const int SIDECHAIN_DEFAULT_PATTERN_INDEX = 0;

#endif // SIDECHAIN_CONSTANTS_H
