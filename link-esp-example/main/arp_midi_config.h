#ifndef ARP_MIDI_CONFIG_H
#define ARP_MIDI_CONFIG_H

#include <vector>
#include <string>

// Forward declare ArpMode if its definition is in a file that might include this one,
// or ensure a common header defines it. For now, we assume it's accessible.
// enum ArpMode { ARP_MODE_NOTE, ARP_MODE_CHORD }; // Example if not defined elsewhere

enum ArpMidiCategory {
    MIDI_CAT_ARP,
    MIDI_CAT_FIL,
    MIDI_CAT_REV,
    MIDI_CAT_SID
};

extern ArpMidiCategory g_current_arp_midi_category;
// g_current_arp_mode (of type ArpMode) is assumed to be globally accessible,
// typically defined in state_machine.h/cpp.

// Base path for MIDI files (e.g., "main/loops/")
extern const std::string G_ARP_MIDI_BASE_PATH;

// File lists (filenames only, e.g., "UNISON_Vader.mid")
// Notes Mode
extern const std::vector<std::string> G_NOTES_ARP_FILES;
extern const std::vector<std::string> G_NOTES_FIL_FILES;
extern const std::vector<std::string> G_NOTES_REV_FILES;
extern const std::vector<std::string> G_NOTES_SID_FILES;

// Chords Mode
extern const std::vector<std::string> G_CHORDS_ARP_FILES;
extern const std::vector<std::string> G_CHORDS_FIL_FILES;
extern const std::vector<std::string> G_CHORDS_REV_FILES;
extern const std::vector<std::string> G_CHORDS_SID_FILES;

// Current indices for each list
// Notes Mode
extern int g_notes_arp_file_idx;
extern int g_notes_fil_file_idx;
extern int g_notes_rev_file_idx;
extern int g_notes_sid_file_idx;

// Chords Mode
extern int g_chords_arp_file_idx;
extern int g_chords_fil_file_idx;
extern int g_chords_rev_file_idx;
extern int g_chords_sid_file_idx;

// Helper function to get the full current MIDI file path
// The actual implementation will be in arp_midi_config.cpp
// It will require access to g_current_arp_mode.
std::string get_current_midi_file_path();

#endif // ARP_MIDI_CONFIG_H 