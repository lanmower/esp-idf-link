#include "arp_midi_config.h"
#include "state_machine.h" // For g_current_arp_mode (ArpMode type)

// Initialize the current MIDI category
ArpMidiCategory g_current_arp_midi_category = MIDI_CAT_ARP;

// Define the base path for all MIDI loop files
const std::string G_ARP_MIDI_BASE_PATH = "main/loops/";

// --- MIDI File Lists Definitions ---
// These vectors store the filenames for each category and mode.
// Paths are relative to subfolders like "notes/arp/", "chords/fil/", etc.

// Notes Mode Files
const std::vector<std::string> G_NOTES_ARP_FILES = {
    "UNISON_Vader (181 BPM - Gbm).mid", "UNISON_Shook (159 BPM - Am).mid",
    "UNISON_New York (100 BPM - Gm).mid", "UNISON_Mints (100 BPM - Gm).mid",
    "UNISON_Marmite (130 BPM - Em).mid", "UNISON_Hacksaw (112 BPM - Cm).mid",
    "UNISON_Furious (101 BPM - Bm).mid", "UNISON_Cathedral (165 BPM - Dbm).mid",
    "UNISON_Blues (147 BPM - Bbm).mid", "UNISON_Amber (123 BPM - Fm).mid"
};
// PLACEHOLDER: Populate these lists with your actual MIDI filenames
const std::vector<std::string> G_NOTES_FIL_FILES = {};
const std::vector<std::string> G_NOTES_REV_FILES = {};
const std::vector<std::string> G_NOTES_SID_FILES = {};

// Chords Mode Files
// PLACEHOLDER: Populate this list with your actual MIDI filenames for chords/arp
const std::vector<std::string> G_CHORDS_ARP_FILES = {};
const std::vector<std::string> G_CHORDS_FIL_FILES = {
    "G Major Prog 02 (I-IVadd9-vim7-V) - Unison.mid", "F Minor Prog 09 (i-iim7-V7-i-V7alt-i-V7alt) - Unison.mid",
    "Eb Minor Prog 05 (im7-biim7-im7-V7-im7-#viim7-im7-V7) - Unison.mid", "Eb Major Prog 08 (I-V-iiim7-iim7) - Unison.mid",
    "E Minor Prog 10 (i-iidim7-i-#viidim7) - Unison.mid", "D Minor Prog 12 (VImaj7-ivm7-vm7-im7) - Unison.mid",
    "C# Minor Prog 08 (i-III-II-#viidim) - Unison.mid", "C Minor Prog 07 (im7-VI6-IIIadd9-III-vm7) - Unison.mid",
    "B Minor Prog 11 (i-VImaj7-Vsus-V) - Unison.mid", "B Minor Prog 06 (i-VI-IIIadd9-V) - Unison.mid"
};
const std::vector<std::string> G_CHORDS_REV_FILES = {
    "E - bVIM7 ivmadd9 I I - Spiritual Nostalgic.mid", "E - im vm ivm bIIM7 - Mysterious Dark.mid",
    "G - bIIM bVIM biii bviim - Mysterious Surprised.mid", "Eb - im ii vm IV - Nostalgic Hopeful.mid",
    "A - i VI VII v - Mysterious Rebellious.mid", "B - i VII VI iv - Sad Romantic.mid",
    "B - iv VI v VII - Mysterious Hopeful.mid", "A - VII iv v i - Mysterious Dark.mid"
};
const std::vector<std::string> G_CHORDS_SID_FILES = {
    "Twin_C#m_100.mid", "Seismic_G#m_125.mid", "Holiday_F#m_120.mid",
    "Groove_Bm_90.mid", "Extra_F#m_128.mid", "Dry_Dm_100.mid"
};

// --- Current File Indices --- 
// Initialize all indices to 0, pointing to the first file in each list.
int g_notes_arp_file_idx = 0;
int g_notes_fil_file_idx = 0;
int g_notes_rev_file_idx = 0;
int g_notes_sid_file_idx = 0;

int g_chords_arp_file_idx = 0;
int g_chords_fil_file_idx = 0;
int g_chords_rev_file_idx = 0;
int g_chords_sid_file_idx = 0;

// Helper function to construct the full path to the currently selected MIDI file.
std::string get_current_midi_file_path() {
    std::string mode_folder;
    std::string category_folder;
    const std::vector<std::string>* current_file_list = nullptr;
    int current_idx = 0;

    // Determine folder and file list based on current ARP mode (notes or chords)
    if (g_current_arp_mode == ARP_MODE_NOTE) {
        mode_folder = "notes/";
        switch (g_current_arp_midi_category) {
            case MIDI_CAT_ARP: category_folder = "arp/"; current_file_list = &G_NOTES_ARP_FILES; current_idx = g_notes_arp_file_idx; break;
            case MIDI_CAT_FIL: category_folder = "fil/"; current_file_list = &G_NOTES_FIL_FILES; current_idx = g_notes_fil_file_idx; break;
            case MIDI_CAT_REV: category_folder = "rev/"; current_file_list = &G_NOTES_REV_FILES; current_idx = g_notes_rev_file_idx; break;
            case MIDI_CAT_SID: category_folder = "sid/"; current_file_list = &G_NOTES_SID_FILES; current_idx = g_notes_sid_file_idx; break;
        }
    } else { // ARP_MODE_CHORD
        mode_folder = "chords/";
        switch (g_current_arp_midi_category) {
            case MIDI_CAT_ARP: category_folder = "arp/"; current_file_list = &G_CHORDS_ARP_FILES; current_idx = g_chords_arp_file_idx; break;
            case MIDI_CAT_FIL: category_folder = "fil/"; current_file_list = &G_CHORDS_FIL_FILES; current_idx = g_chords_fil_file_idx; break;
            case MIDI_CAT_REV: category_folder = "rev/"; current_file_list = &G_CHORDS_REV_FILES; current_idx = g_chords_rev_file_idx; break;
            case MIDI_CAT_SID: category_folder = "sid/"; current_file_list = &G_CHORDS_SID_FILES; current_idx = g_chords_sid_file_idx; break;
        }
    }

    if (!current_file_list || current_file_list->empty()) {
        // ESP_LOGW("MIDI_CONFIG", "No MIDI files in selected list: %s%s", mode_folder.c_str(), category_folder.c_str());
        return ""; // Return empty string if list is empty or not found
    }

    // Basic bounds check for the index
    if (current_idx < 0 || current_idx >= current_file_list->size()) {
        // ESP_LOGW("MIDI_CONFIG", "MIDI file index out of bounds for %s%s. Resetting to 0.", mode_folder.c_str(), category_folder.c_str());
        // This part of logic should ideally be coupled with cycling logic to prevent out of bounds.
        // For now, just return empty or first element to avoid crash.
        // current_idx = 0; // Or handle error appropriately
        return ""; // Safer to return empty if index is bad before cycling logic is in place
    }

    return G_ARP_MIDI_BASE_PATH + mode_folder + category_folder + (*current_file_list)[current_idx];
} 