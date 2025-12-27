#include "effect_arp.h"
#include "esp_log.h"
#include "synth_interface.h"
#include "effect_handler.h"
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <ctime>
#include <random>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include "esp_timer.h"

static const char* TAG = "ARP";

// Initialize global objects and variables
MidiFilePlayer g_midi_player;
bool g_midi_player_active = false;
int g_current_arp_transpose = 0;
double g_current_arp_playback_rate = 1.0;
// ArpMode g_current_arp_mode is already defined in state_machine.cpp

// Define folder paths for MIDI files
const char* NOTES_BASE_FOLDER = "/spiffs/loops/notes";
const char* CHORDS_BASE_FOLDER = "/spiffs/loops/chords";

const char* NOTES_FILTER_MIDI_FOLDER = "/spiffs/loops/notes/fil";
const char* NOTES_REVERSE_MIDI_FOLDER = "/spiffs/loops/notes/rev";
const char* NOTES_SIDECHAIN_MIDI_FOLDER = "/spiffs/loops/notes/sid";
const char* NOTES_ARP_MIDI_FOLDER = "/spiffs/loops/notes/arp";

const char* CHORDS_FILTER_MIDI_FOLDER = "/spiffs/loops/chords/fil";
const char* CHORDS_REVERSE_MIDI_FOLDER = "/spiffs/loops/chords/rev";
const char* CHORDS_SIDECHAIN_MIDI_FOLDER = "/spiffs/loops/chords/sid";
const char* CHORDS_ARP_MIDI_FOLDER = "/spiffs/loops/chords/arp";

// For backward compatibility
const char* FILTER_MIDI_FOLDER = "/spiffs/loops/notes/fil";
const char* REVERSE_MIDI_FOLDER = "/spiffs/loops/notes/rev";
const char* SIDECHAIN_MIDI_FOLDER = "/spiffs/loops/notes/sid";

// Helper function to check if a directory exists
bool directoryExists(const char* path) {
    // In SPIFFS, directories don't really exist, but we can check if files with this path prefix exist
    DIR* dir = opendir(path);
    if (dir) {
        closedir(dir);
        return true;
    }
    
    // For SPIFFS, directories are implied if files exist with that prefix
    // Try to find any file that starts with this path
    char search_path[512];
    snprintf(search_path, sizeof(search_path), "%s/", path);
    
    // Get one level up to search
    DIR* parent_dir = opendir("/spiffs");
    if (!parent_dir) {
        return false;
    }
    
    bool exists = false;
    struct dirent* entry;
    while ((entry = readdir(parent_dir)) != NULL) {
        char full_path[512];
        snprintf(full_path, sizeof(full_path), "/spiffs/%s", entry->d_name);
        
        // Check if this path starts with our search path
        if (strstr(full_path, search_path) == full_path) {
            exists = true;
            break;
        }
    }
    
    closedir(parent_dir);
    return exists;
}

// Reset arpeggiator to use MIDI file player
void reset_arp_to_midi_player(bool unused) {
    // Stop any currently playing MIDI
    g_midi_player.stop();
    
    // Check if synth interface is valid
    if (!g_current_synth) {
        ESP_LOGE(TAG, "ERROR: g_current_synth is null! MIDI playback will fail.");
        return;
    }
    
    // Use different folders based on synth type
    const char* folder_type = (g_synth_type == SYNTH_MININOVA) ? "chords" : "notes";
    char target_folder[64] = {0};
    snprintf(target_folder, sizeof(target_folder), "/spiffs/loops/%s/arp", folder_type);
    
    ESP_LOGI(TAG, "Setting MIDI folder: %s (synth: %s)", target_folder, 
             (g_synth_type == SYNTH_MININOVA) ? "Mininova" : "MicroKorg");
    
    // Set the folder for the MIDI player
    g_midi_player.setFolder(target_folder);
    
    // Reset transpose and playback rate to defaults
    g_current_arp_transpose = 0;
    g_current_arp_playback_rate = 1.0;
    g_midi_player.setTranspose(g_current_arp_transpose);
    g_midi_player.setPlaybackRate(g_current_arp_playback_rate);
    
    // Start the MIDI player with full quantum sync
    if (g_midi_player.start()) {
        g_midi_player_active = true;
        
        // Log the file information for the arpeggiator
        std::string currentFile = g_midi_player.getCurrentFileName();
        if (!currentFile.empty()) {
            size_t lastSlash = currentFile.find_last_of('/');
            std::string filename = (lastSlash != std::string::npos) ? 
                                  currentFile.substr(lastSlash + 1) : currentFile;
            
            ESP_LOGI(TAG, "MIDI player started with file: %s", filename.c_str());
        } else {
            ESP_LOGI(TAG, "MIDI player started with unknown file");
        }
    } else {
        g_midi_player_active = false;
        ESP_LOGE(TAG, "Failed to start MIDI player");
    }
}

// Handle the arp when it's active
bool handle_arp_active(const ableton::Link::SessionState& state, 
                      const std::chrono::microseconds& time,
                      int note_index) {
    if (!g_midi_player_active) {
        return false;
    }
    
    // Get the current beat for Link synchronization
    const double sessionBeat = state.beatAtTime(time, LINK_QUANTUM);
    
    // Process the MIDI file player with the current Link state
    // This ensures the arpeggiator stays in sync with the Link session
    g_midi_player.process(state, time);
    
    // Always return true to increment the note index in the state machine
    return true;
}

// Handle submenu navigation when the arp pad is held
bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state,
                               const std::chrono::microseconds& time,
                               const bool pad_pressed_this_tick[],
                               std::array<bool, 4>& pads_used) {
    // PERFORMANCE: Removed verbose debug logging
    
    // Check if any pads are actually pressed (quick validation)
    bool any_pads_pressed = false;
    for (int i = 0; i < 4; i++) {
        if (i != ARP_PAD_INDEX && pad_pressed_this_tick[i]) {
            any_pads_pressed = true;
            // Essential log kept
            ESP_LOGI(TAG, "ARP handler detected pad %d pressed", i);
            break;
        }
    }
    
    if (!any_pads_pressed) {
        // No secondary taps to process
        return false;
    }
    
    bool handled = false;
    
    // Check each pad (except the arp pad itself)
    for (int i = 0; i < 4; i++) {
        if (i == ARP_PAD_INDEX) continue; // Skip the arp pad itself
        
        if (pad_pressed_this_tick[i]) {
            pads_used[i] = true; // Mark pad as used
            handled = true;
            
            // When holding arp and pressing another pad, play the current file number from that pad's folder
            
            // Stop current playback
            ESP_LOGI(TAG, "Switching to effect folder for pad %d while maintaining file index", i);
            g_midi_player.stop();
            g_midi_player_active = false;
            
            const char* target_folder = nullptr;
            char folder_path[64] = {0};
            
            // Use different folders based on synth type
            const char* folder_type = (g_synth_type == SYNTH_MININOVA) ? "chords" : "notes";
            
            // Determine which folder to use based on the pad and synth type
            if (i == FILTER_PAD_INDEX) { // Filter pad (3)
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/fil", folder_type);
                target_folder = folder_path;
            } else if (i == SIDECHAIN_PAD_INDEX) { // Sidechain pad (0)
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/sid", folder_type);
                target_folder = folder_path;
            } else if (i == DELAY_REVERB_PAD_INDEX) { // Delay/Reverb pad (2)
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/rev", folder_type);
                target_folder = folder_path;
            }
            
            // Verify target folder is set
            if (target_folder == nullptr) {
                ESP_LOGE(TAG, "Failed to determine target folder for pad %d", i);
                return handled;
            }
            
            // Get the current file index before switching folders
            size_t currentIndex = g_midi_player.getCurrentFileIndex();
            
            // Set the folder for the MIDI player
            g_midi_player.setFolder(target_folder);
            
            // Try to maintain the same file index in the new folder
            g_midi_player.setCurrentFileIndex(currentIndex);
            
            // Start the MIDI player
            if (g_midi_player.start()) {
                g_midi_player_active = true;
                ESP_LOGI(TAG, "Started MIDI playback from folder: %s with file index: %zu", target_folder, currentIndex);
            } else {
                ESP_LOGE(TAG, "Failed to start MIDI playback from folder: %s", target_folder);
            }
            break; // Only handle one pad press at a time
        }
    }
    
    return handled;
}

// Handle pot adjustments when the arp pad is held
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    // Pot 1: Note length scaling from 0% to 200%
    if (pot1_delta != 0) {
        int pot1_value = g_midi_player.getPot1Value(); // Get current raw value (0-127)
        pot1_value += pot1_delta;  // Apply the delta
        pot1_value = std::min(127, std::max(0, pot1_value)); // Clamp to valid range
        
        // Map 0-127 to 0.0-2.0 (0% to 200%)
        double note_length_scale = (pot1_value / 127.0) * 2.0;
        
        // Set the note length scale in the MIDI player
        g_midi_player.setNoteLengthScale(note_length_scale);
        g_midi_player.setPot1Value(pot1_value); // Store the raw value
        
        ESP_LOGI(TAG, "Note Length Scale: %.0f%% (Pot1: %d)", note_length_scale * 100, pot1_value);
        pot1_used = true;
    }
    
    // Pot 2: Velocity scaling from 0-127
    if (pot2_delta != 0) {
        int pot2_value = g_midi_player.getPot2Value(); // Get current raw value (0-127)
        pot2_value += pot2_delta;  // Apply the delta
        pot2_value = std::min(127, std::max(0, pot2_value)); // Clamp to valid range
        
        // Map 0-127 to velocity scale (0.0-1.0)
        double velocity_scale = pot2_value / 127.0;
        
        // Set the velocity scale in the MIDI player
        g_midi_player.setVelocityScale(velocity_scale);
        g_midi_player.setPot2Value(pot2_value); // Store the raw value
        
        ESP_LOGI(TAG, "Velocity Scale: %d (Pot2: %d)", pot2_value, pot2_value);
        pot2_used = true;
    }
}