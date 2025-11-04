#ifndef EFFECT_ARP_H
#define EFFECT_ARP_H

#include "main.h"
#include <array> // For std::array
#include <vector> // For std::vector
#include "synth_interface.h" // For SynthInterface type
#include "ableton/Link.hpp" // For Link types
#include <chrono> // For std::chrono::microseconds
#include "midi_file.h" // Add MIDI file player

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

// Constants for submenu indices
#define SUBMENU_FILTER     0
#define SUBMENU_REVERSE    1 
#define SUBMENU_SIDECHAIN  2

// Reset arpeggiator to use MIDI file player
void reset_arp_to_midi_player(bool chord_mode = false);

// Function to handle potentiometer adjustments when Arp Adjust is active
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);

// Global MIDI player instance
extern MidiFilePlayer g_midi_player;

// Extern parameters that control the Arp's behavior
extern int g_current_arp_transpose;       // Transpose value for MIDI files
extern double g_current_arp_playback_rate; // Playback rate: 0.25, 0.5, 1.0, 2.0
extern bool g_midi_player_active;         // Flag to indicate if MIDI player is active

// Folder paths for different submenu types
extern const char* NOTES_BASE_FOLDER;
extern const char* CHORDS_BASE_FOLDER;

extern const char* NOTES_FILTER_MIDI_FOLDER;
extern const char* NOTES_REVERSE_MIDI_FOLDER;
extern const char* NOTES_SIDECHAIN_MIDI_FOLDER;
extern const char* NOTES_ARP_MIDI_FOLDER;

extern const char* CHORDS_FILTER_MIDI_FOLDER;
extern const char* CHORDS_REVERSE_MIDI_FOLDER;
extern const char* CHORDS_SIDECHAIN_MIDI_FOLDER;
extern const char* CHORDS_ARP_MIDI_FOLDER;

// For backward compatibility
extern const char* FILTER_MIDI_FOLDER;
extern const char* REVERSE_MIDI_FOLDER;
extern const char* SIDECHAIN_MIDI_FOLDER;

#endif // EFFECT_ARP_H