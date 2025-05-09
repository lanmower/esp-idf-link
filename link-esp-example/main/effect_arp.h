#ifndef EFFECT_ARP_H
#define EFFECT_ARP_H

#include "main.h"
#include <array> // For std::array
#include <vector> // For std::vector
#include "arp_constants.h" // Include constants
#include "synth_interface.h" // For SynthInterface type
#include "ableton/Link.hpp" // For Link types
#include <chrono> // For std::chrono::microseconds

// Note: Internal generation state variables are now static within effect_arp.cpp
// extern int last_played_arp_note;
// extern uint64_t arp_note_off_time_us;
// extern int current_arp_progression_step;
// extern int last_triggered_step_index;
// extern std::vector<int> last_played_arp_chord_notes;

// Arp Mode Enum
enum ArpMode { ARP_MODE_NOTE = 0, ARP_MODE_CHORD = 1 };
extern ArpMode g_current_arp_mode;

// Function Prototypes
// EffectState handle_arp_taps(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, bool pressed, bool released, uint64_t current_time_us); // REMOVED
bool handle_arp_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int note_index);
bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, const bool pad_pressed_this_tick[], std::array<bool, 4>& pads_used);

// Reset arpeggiator to chord mode (called when double-tapping the arp pad)
void reset_arp_to_chord_mode();

// Function to handle potentiometer adjustments when Arp Adjust is active
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);

// Helper function (can remain if used only internally)
int get_arp_note_from_degree(int scale_idx, int scale_degree, int root, int octave);

// Extern parameters (defined in effect_arp.cpp) - these control the Arp's behavior
extern int g_current_arp_scale_index;
extern int g_current_arp_rhythm_index;
extern int g_current_arp_gate; // 0-127
extern int g_current_arp_swing; // 0-127
extern int g_current_arp_chord_rhythm_index;
extern int g_current_arp_progression_index;
extern int g_current_arp_root_note;
extern int g_current_arp_octave;
extern int g_current_arp_velocity;
extern int g_current_arp_velocity_pattern; // Added for velocity patterns
extern int g_current_arp_deadmau5_pattern; // Added for signature patterns
extern int g_current_arp_voicing_group;    // Added for voicing groups
extern int g_current_arp_bassline_pattern; // Added for bassline patterns
extern bool g_use_deadmau5_patterns;       // Flag to use signature patterns

#endif // EFFECT_ARP_H