#ifndef ARP_CONSTANTS_H
#define ARP_CONSTANTS_H

// Arpeggiator constants, scales, rhythms, progressions

#include <vector> // Added for std::vector
#include <stdint.h>

// --- Scales ---
// Strictly curated for deadmau5-style techno - only proven musical options
const std::vector<std::vector<int>> ARP_SCALES = {
    {0, 2, 3, 5, 7, 10},       // Minor (classic techno scale)
    {0, 3, 5, 7, 10},          // Minor Pentatonic (clean)
    {0, 2, 3, 7, 9, 10},       // Dorian (excellent for techno)
    {0, 2, 4, 5, 7, 9, 10},    // Mixolydian (techno feel)
    {0, 2, 4, 7, 9},           // Major Pentatonic (clean)
    {0, 2, 4, 5, 7, 9, 11},    // Major (clean)
};
const int NUM_ARP_SCALES = ARP_SCALES.size();

// --- Chords ---
// Expanded with deadmau5-style voicings - focus on clean, open sounds
const std::vector<std::vector<int>> ARP_CHORDS = {
    // Classic triads
    {0, 3, 7},          // Minor triad (foundation)
    {0, 4, 7},          // Major triad (clean)
    
    // Extended chords for techno
    {0, 3, 7, 10},      // Minor 7th (classic techno)
    {0, 4, 7, 11},      // Major 7th (clean, spacey)
    
    // Open/power voicings (deadmau5 favorites)
    {0, 7, 12},         // Stacked fifths (deadmau5 signature)
    {0, 7},             // Power fifth (minimal)
    {0, 12},            // Octave (clean)
    
    // Suspended chords (great for techno)
    {0, 5, 7},          // Sus4 (open)
    {0, 2, 7},          // Sus2 (open)
    
    // Spacious modern voicings
    {0, 7, 14},         // Open fifth + 9th (cinematic)
    {0, 5, 10},         // Quartal (modern)
    {0, 4, 11},         // Major 7 no 5th (sparse)
    {0, 3, 10},         // Minor 7 no 5th (sparse)
    
    // Inversions with good bass positioning
    {4, 7, 12},         // 1st inversion major (smooth)
    {3, 7, 12},         // 1st inversion minor (smooth)
};
const int NUM_ARP_CHORDS = ARP_CHORDS.size();

// --- Voicing Groups ---
// Deadmau5 tracks often use specific chord voicing patterns - these group them by type
const std::vector<std::vector<int>> ARP_VOICING_GROUPS = {
    // Group 1: Strobe-style open voicings
    {0, 7, 12, 16},     // Open voicing with 7th on top
    {0, 7, 16, 19},     // Open voicing with added 3rd
    {0, 12, 19, 24},    // Wide spread cinematic
    
    // Group 2: I Remember-style extended voicings
    {0, 4, 7, 11, 14},  // Major9 compact 
    {0, 3, 7, 10, 14},  // Minor9 compact
    {0, 4, 7, 14},      // Major add9 (no 7th)
    
    // Group 3: Ghosts n Stuff-style sparse voicings
    {0, 7, 10, 17},     // Power chord with 7th
    {0, 7, 14, 17},     // Power chord with 9th
    {0, 5, 7, 14},      // Sus4 with 9th
};
const int NUM_ARP_VOICING_GROUPS = ARP_VOICING_GROUPS.size();

// --- Chord Progressions ---
// Carefully selected for deadmau5-style 4-bar phrases - all musically sound
const std::vector<std::vector<int>> ARP_CHORD_PROGRESSIONS = {
    // Classic dance music progressions
    {0, 5, 3, 4},       // i-vi-iv-v (most common in electronic)
    {0, 3, 4, 5},       // i-iv-v-vi (standard EDM)
    {0, 3, 4, 0},       // i-iv-v-i (classic resolution)
    {5, 0, 3, 4},       // vi-i-iv-v (tension builder)
    
    // Minimal techno loops
    {0, 5, 0, 5},       // i-vi-i-vi (simple but effective)
    {0, 3, 0, 5},       // i-iv-i-vi (house pattern)
    {0, 4, 0, 5},       // i-v-i-vi (strong movement)
    
    // Deadmau5 specific patterns
    {0, 0, 5, 3},       // i-i-vi-iv (Strobe-like) 
    {0, 3, 0, 4},       // i-iv-i-v (I Remember-like)
    {0, 5, 4, 3},       // i-vi-v-iv (Ghosts n Stuff-like)
    {0, 0, 3, 5},       // i-i-iv-vi (Raise Your Weapon-like)
};
const int NUM_ARP_CHORD_PROGRESSIONS = ARP_CHORD_PROGRESSIONS.size();

// --- Note Progressions ---
// Optimal patterns for deadmau5-style plucks and leads
const std::vector<std::vector<int>> ARP_NOTE_PROGRESSIONS = {
    // Simple patterns (most reliable)
    {0, 2, 0, 4},       // Simple up-down (clean)
    {0, 3, 7, 3},       // Minor triad pattern
    {0, 4, 7, 4},       // Major triad pattern
    {0, 7, 12, 7},      // Octave jump pattern (signature deadmau5)
    
    // Signature deadmau5-style patterns
    {0, 12, 0, 7},      // Octave jump with fifth (iconic)
    {0, 7, 12, 19},     // Octave+fifth climb (epic)
    {0, 4, 0, 7, 0, 12, 0, 7},   // Pedal point pattern (Strobe-like)
    
    // Scalar patterns
    {0, 2, 4, 7, 12, 7, 4, 2},   // Full scale run (melodic)
    {0, 0, 7, 7, 12, 12, 7, 0},  // Staggered climb (rhythmic)
    
    // Extended patterns for interest
    {0, 4, 7, 12, 16, 12, 7, 4}, // Extended chord (Ghosts n Stuff-like)
    {0, 3, 7, 10, 14, 10, 7, 3}, // Extended minor (I Remember-like)
    {0, 7, 12, 16, 19, 16, 12, 7}  // Wide range (cinematic)
};
const int NUM_ARP_NOTE_PROGRESSIONS = ARP_NOTE_PROGRESSIONS.size();

// --- Signature deadmau5 Patterns ---
// Recreating iconic patterns from tracks like "Strobe", "I Remember", "Ghosts 'n' Stuff"
const std::vector<std::vector<int>> ARP_DEADMAU5_PATTERNS = {
    // Strobe Pattern (iconic)
    {0, 12, 7, 4, 0, 12, 7, 4},
    // Ghosts 'n' Stuff Pattern
    {0, 7, 12, 7, 4, 7, 12, 7},
    // I Remember Pattern 
    {0, 4, 7, 12, 7, 4, 7, 12},
    // Faxing Berlin Pattern
    {0, 7, 3, 7, 12, 7, 3, 7},
    // Some Chords Pattern
    {0, 4, 7, 11, 16, 11, 7, 4},
    // The Veldt Pattern
    {0, 3, 7, 10, 14, 10, 7, 3},
    // Raise Your Weapon Pattern
    {0, 7, 10, 12, 19, 12, 10, 7},
    // Cthulhu Sleeps Pattern
    {0, 12, 7, 14, 19, 14, 7, 12}
};
const int NUM_ARP_DEADMAU5_PATTERNS = ARP_DEADMAU5_PATTERNS.size();

// For generic use (e.g., UI cycling), treat as note progressions by default
const int NUM_ARP_PROGRESSIONS = NUM_ARP_NOTE_PROGRESSIONS;

// --- Rhythms ---
// Proven deadmau5-style rhythms that align perfectly with 4/4 bars
const std::vector<std::vector<bool>> ARP_RHYTHMS = {
    // 16-step: Classic pluck pattern (signature deadmau5)
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    // 16-step: Offset plucks (modern)
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    // 16-step: Sparse pattern (minimal)
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    // 16-step: "Strobe" rhythm
    {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    // 16-step: Syncopated pattern (interesting)
    {1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0},
    // 16-step: Ghost notes pattern (adds movement)
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1},
    // 8-step: Clean, fast arp (16th notes)
    {1, 1, 1, 1, 1, 1, 1, 1}
};
const int NUM_ARP_RHYTHMS = ARP_RHYTHMS.size();

// --- Chord Rhythms ---
// Optimal for deadmau5-style progression changes - all perfectly aligned with 4/4
const std::vector<std::vector<bool>> ARP_CHORD_RHYTHMS = {
    // 16-step: Classic 4-chord progression (1 per 4 steps)
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    // 16-step: Sparse chords (spacious)
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    // 8-step: Two chords per bar (half-notes)
    {1, 0, 0, 0, 1, 0, 0, 0},
    // 16-step: "Strobe" chord rhythm
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    // 16-step: First-beat only (minimal)
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    // 16-step: Progressive house pattern
    {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0}
};
const int NUM_ARP_CHORD_RHYTHMS = ARP_CHORD_RHYTHMS.size();

// --- Velocity Patterns ---
// Adds dynamic expression to notes (values are 0-127)
const std::vector<std::vector<int>> ARP_VELOCITY_PATTERNS = {
    // Flat (no variation) - for clean plucks
    {100, 100, 100, 100, 100, 100, 100, 100},
    // Accent on first beat - for techno feel
    {120, 90, 90, 90, 100, 90, 90, 90},
    // Accent on offbeats - for groove
    {90, 110, 90, 110, 90, 110, 90, 110},
    // Gradual build up - for tension
    {70, 80, 90, 100, 110, 120, 110, 100},
    // Gradual fade - for release
    {120, 110, 100, 90, 80, 70, 80, 90},
    // Bounce pattern - for movement
    {120, 80, 100, 80, 110, 80, 100, 80}
};
const int NUM_ARP_VELOCITY_PATTERNS = ARP_VELOCITY_PATTERNS.size();

// --- NEW: Bassline Patterns ---
// Specific patterns for bassline use with chord progressions
const std::vector<std::vector<int>> ARP_BASSLINE_PATTERNS = {
    // Root notes only
    {0, 0, 0, 0},
    // Root-fifth pattern
    {0, 7, 0, 7},
    // Walking bass
    {0, 2, 3, 5},
    // Octave jump bass
    {0, 12, 0, 12},
    // Classic techno bass
    {0, 0, 7, 0},
    // Deadmau5 style bass movement
    {0, 7, 12, 7}
};
const int NUM_ARP_BASSLINE_PATTERNS = ARP_BASSLINE_PATTERNS.size();

// --- Maximum Index for Wrapping --- 
// Used in effect_handler to wrap the raw note index. Choose a value larger
// than the longest reasonably expected progression sequence.
const int MAX_ARP_INDEX_WRAP = 128; 

// --- Patterns ---
enum ArpPattern { UP, DOWN, UP_DOWN, DOWN_UP, RANDOM };
const int NUM_ARP_PATTERNS = 5;

#endif // ARP_CONSTANTS_H