#ifndef SIDECHAIN_CONSTANTS_H
#define SIDECHAIN_CONSTANTS_H

#include <vector>
#include <array>

// Define number of steps per pattern (e.g., 16 for 16th notes over 1 bar)
const int SIDECHAIN_RHYTHM_STEPS = 16;

// Define Sidechain Patterns (Arrays of bools)
// true = Gate ON (sound allowed), false = Gate OFF (sound ducked)
// Example Patterns:

// Pattern 0: True 1/4 note pumping (duck ON the beat, sound allowed OFF beat)
// Ensure the quietest part is at the start of the beat
const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_QUARTER = {
    false, false, true,  true,  // Beat 1: duck at start, then allow
    false, false, true,  true,  // Beat 2
    false, false, true,  true,  // Beat 3
    false, false, true,  true   // Beat 4
};

// Pattern 1: Off-beat 1/8 notes (OFF, ON, OFF, ON, ...)
// Ensure the quietest part is at the start of the beat
const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_OFFBEAT_EIGHTH = {
    false, false, true,  false, true,  false, true,  false,
    true,  false, true,  false, true,  false, true,  false
};

// Pattern 2: More complex syncopated rhythm
// Ensure the quietest part is at the start of the beat
const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_SYNCOPATED = {
    false, false, true,  false, true,  false, false, true,
    false, false, true,  false, true,  false, false, true
};

// Pattern 3: Four-on-the-floor Kick pattern style (OFF, ON, ON, ON...)
// Ensure the quietest part is at the start of the beat
const std::array<bool, SIDECHAIN_RHYTHM_STEPS> SC_PATTERN_FOUR_FLOOR = {
    false, false, true,  true,  false, false, true,  true,
    false, false, true,  true,  false, false, true,  true
};


// Array of all defined patterns
const std::vector<std::array<bool, SIDECHAIN_RHYTHM_STEPS>> SIDECHAIN_PATTERNS = {
    SC_PATTERN_QUARTER,
    SC_PATTERN_OFFBEAT_EIGHTH,
    SC_PATTERN_SYNCOPATED,
    SC_PATTERN_FOUR_FLOOR
};

// Number of available patterns
const int NUM_SIDECHAIN_PATTERNS = SIDECHAIN_PATTERNS.size();

// Default pattern index for sidechain (0 = true quarter-note pumping)
const int SIDECHAIN_DEFAULT_PATTERN_INDEX = 0;

#endif // SIDECHAIN_CONSTANTS_H