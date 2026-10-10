#ifndef ARP_CONSTANTS_H
#define ARP_CONSTANTS_H

#include <vector>
#include <stdint.h>

const std::vector<std::vector<int>> ARP_SCALES = {
    {0, 2, 3, 5, 7, 10},
    {0, 3, 5, 7, 10},
    {0, 2, 3, 7, 9, 10},
    {0, 2, 4, 5, 7, 9, 10},
    {0, 2, 4, 7, 9},
    {0, 2, 4, 5, 7, 9, 11},
};
const int NUM_ARP_SCALES = ARP_SCALES.size();

const std::vector<std::vector<int>> ARP_CHORDS = {
    {0, 3, 7},
    {0, 4, 7},
    {0, 3, 7, 10},
    {0, 4, 7, 11},
    {0, 7, 12},
    {0, 7},
    {0, 12},
    {0, 5, 7},
    {0, 2, 7},
    {0, 7, 14},
    {0, 5, 10},
    {0, 4, 11},
    {0, 3, 10},
    {4, 7, 12},
    {3, 7, 12},
};
const int NUM_ARP_CHORDS = ARP_CHORDS.size();

const std::vector<std::vector<int>> ARP_VOICING_GROUPS = {
    {0, 7, 12, 16},
    {0, 7, 16, 19},
    {0, 12, 19, 24},
    {0, 4, 7, 11, 14},
    {0, 3, 7, 10, 14},
    {0, 4, 7, 14},
    {0, 7, 10, 17},
    {0, 7, 14, 17},
    {0, 5, 7, 14},
};
const int NUM_ARP_VOICING_GROUPS = ARP_VOICING_GROUPS.size();

const std::vector<std::vector<int>> ARP_CHORD_PROGRESSIONS = {
    {0, 5, 3, 4},
    {0, 3, 4, 5},
    {0, 3, 4, 0},
    {5, 0, 3, 4},
    {0, 5, 0, 5},
    {0, 3, 0, 5},
    {0, 4, 0, 5},
    {0, 0, 5, 3},
    {0, 3, 0, 4},
    {0, 5, 4, 3},
    {0, 0, 3, 5},
};
const int NUM_ARP_CHORD_PROGRESSIONS = ARP_CHORD_PROGRESSIONS.size();

const std::vector<std::vector<int>> ARP_NOTE_PROGRESSIONS = {
    {0, 2, 0, 4},
    {0, 3, 7, 3},
    {0, 4, 7, 4},
    {0, 7, 12, 7},
    {0, 12, 0, 7},
    {0, 7, 12, 19},
    {0, 4, 0, 7, 0, 12, 0, 7},
    {0, 2, 4, 7, 12, 7, 4, 2},
    {0, 0, 7, 7, 12, 12, 7, 0},
    {0, 4, 7, 12, 16, 12, 7, 4},
    {0, 3, 7, 10, 14, 10, 7, 3},
    {0, 7, 12, 16, 19, 16, 12, 7}
};
const int NUM_ARP_NOTE_PROGRESSIONS = ARP_NOTE_PROGRESSIONS.size();

const std::vector<std::vector<int>> ARP_DEADMAU5_PATTERNS = {
    {0, 12, 7, 4, 0, 12, 7, 4},
    {0, 7, 12, 7, 4, 7, 12, 7},
    {0, 4, 7, 12, 7, 4, 7, 12},
    {0, 7, 3, 7, 12, 7, 3, 7},
    {0, 4, 7, 11, 16, 11, 7, 4},
    {0, 3, 7, 10, 14, 10, 7, 3},
    {0, 7, 10, 12, 19, 12, 10, 7},
    {0, 12, 7, 14, 19, 14, 7, 12}
};
const int NUM_ARP_DEADMAU5_PATTERNS = ARP_DEADMAU5_PATTERNS.size();

const int NUM_ARP_PROGRESSIONS = NUM_ARP_NOTE_PROGRESSIONS;

const std::vector<std::vector<bool>> ARP_RHYTHMS = {
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1}
};
const int NUM_ARP_RHYTHMS = ARP_RHYTHMS.size();

const std::vector<std::vector<bool>> ARP_CHORD_RHYTHMS = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0}
};
const int NUM_ARP_CHORD_RHYTHMS = ARP_CHORD_RHYTHMS.size();

// Values are MIDI velocity bytes (0-127)
const std::vector<std::vector<int>> ARP_VELOCITY_PATTERNS = {
    {100, 100, 100, 100, 100, 100, 100, 100},
    {120, 90, 90, 90, 100, 90, 90, 90},
    {90, 110, 90, 110, 90, 110, 90, 110},
    {70, 80, 90, 100, 110, 120, 110, 100},
    {120, 110, 100, 90, 80, 70, 80, 90},
    {120, 80, 100, 80, 110, 80, 100, 80}
};
const int NUM_ARP_VELOCITY_PATTERNS = ARP_VELOCITY_PATTERNS.size();

const std::vector<std::vector<int>> ARP_BASSLINE_PATTERNS = {
    {0, 0, 0, 0},
    {0, 7, 0, 7},
    {0, 2, 3, 5},
    {0, 12, 0, 12},
    {0, 0, 7, 0},
    {0, 7, 12, 7}
};
const int NUM_ARP_BASSLINE_PATTERNS = ARP_BASSLINE_PATTERNS.size();

const int MAX_ARP_INDEX_WRAP = 128;

enum ArpPattern { UP, DOWN, UP_DOWN, DOWN_UP, RANDOM };
const int NUM_ARP_PATTERNS = 5;

#endif // ARP_CONSTANTS_H
