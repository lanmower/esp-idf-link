#include "effect_arp.h"
#include "midi_helpers.h" // Added for scaling functions
#include "arp_constants.h" // Include the constants
#include "synth_interface.h" // Include the synth interface
#include <cstdlib> // For rand()
#include "effect_handler.h" // Include for extern Arp variables & ControlContext access
#include <random> // For randomization
#include "esp_timer.h" // For timing
#include <rom/ets_sys.h> // For esp_rom_delay_us
#include <algorithm> // For std::max / std::min
#include "esp_log.h" // For logging
#include "state_machine.h" // For access to g_current_arp_mode, g_link etc.
#include "link_sync.h" // For quantum boundary detection

// Define the logging tag for this file
static const char *TAG_ARP = "EFFECT_ARP";

// Define the speed factor for melodies relative to chords
const double MELODY_SPEED_FACTOR = 2.0; // e.g., 2.0 means twice as fast

// --- Arp Parameters (Externally controllable, defined here, declared extern in effect_arp.h) ---
int g_current_arp_gate = 80; // Default Gate (0-127 range, matches MIDI)
int g_current_arp_swing = 64; // Default Swing (0-127 range, matches MIDI 50% = 64)
int g_current_arp_scale_index = 0;
int g_current_arp_rhythm_index = 0;
int g_current_arp_chord_rhythm_index = 0;
int g_current_arp_progression_index = 0; // Index for selecting which progression pattern to use
int g_current_arp_root_note = 60;
int g_current_arp_octave = 0;
int g_current_arp_velocity = 100;
int g_current_arp_velocity_pattern = 0; // NEW: Index for velocity pattern
int g_current_arp_deadmau5_pattern = 0; // NEW: Index for signature deadmau5 patterns
int g_current_arp_voicing_group = 0;    // NEW: Index for voicing group patterns
int g_current_arp_bassline_pattern = 0; // NEW: Index for bassline patterns
bool g_use_deadmau5_patterns = false;   // NEW: Flag to use deadmau5 patterns
// ArpMode g_current_arp_mode = ARP_MODE_NOTE; // Now defined in state_machine.cpp

// --- Internal Arp State (Static to this file) ---
static int s_current_arp_progression_step = 0; // Step within the selected progression
static int s_last_played_arp_note = -1;
static std::vector<int> s_last_played_arp_chord_notes; // Track all notes in last played chord
static int s_last_triggered_step_index = -1;
// Note off timing
static uint64_t s_arp_note_off_time_us = 0;


// --- Helper Functions ---
// Get the MIDI note number based on scale, scale_degree, root, and octave
int get_arp_note_from_degree(int scale_idx, int scale_degree, int root, int octave) {
    if (scale_idx < 0 || scale_idx >= NUM_ARP_SCALES) scale_idx = 0; // Safety check

    const auto& scale = ARP_SCALES[scale_idx];
    int scale_size = scale.size();
    if (scale_size == 0) return -1; // Invalid scale

    // Ensure scale_degree wraps correctly for negative inputs if needed
    int degree_index = scale_degree % scale_size;
    if (degree_index < 0) degree_index += scale_size;
    int octave_offset = scale_degree / scale_size;

    int note = root + scale[degree_index] + (octave + octave_offset) * 12;

    // Clamp note to valid MIDI range
    return clamp_value(note, 0, 127);
}

// --- Helper: Get random int in range [min, max] ---
int arp_random_int(int min, int max) {
    if (min > max) std::swap(min, max);
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}

// --- Helper: Get random float in range [min, max] ---
float arp_random_float(float min, float max) {
     if (min > max) std::swap(min, max);
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// --- Internal Helper: Get current progression length ---
int _get_current_progression_length() {
    int current_prog_len = 1; // Default to 1 to avoid modulo by zero
    if (g_current_arp_mode == ARP_MODE_NOTE) {
        if (g_use_deadmau5_patterns) {
            // Get length from deadmau5 patterns if enabled
            int deadmau5_idx = g_current_arp_deadmau5_pattern % NUM_ARP_DEADMAU5_PATTERNS;
            current_prog_len = ARP_DEADMAU5_PATTERNS[deadmau5_idx].size();
        } else if (g_current_arp_progression_index >= 0 && g_current_arp_progression_index < NUM_ARP_NOTE_PROGRESSIONS) {
            current_prog_len = ARP_NOTE_PROGRESSIONS[g_current_arp_progression_index].size();
        }
    } else { // ARP_MODE_CHORD
        if (g_current_arp_progression_index >= 0 && g_current_arp_progression_index < NUM_ARP_CHORD_PROGRESSIONS) {
            current_prog_len = ARP_CHORD_PROGRESSIONS[g_current_arp_progression_index].size();
        }
    }
    
    ESP_LOGV(TAG_ARP, "Current progression length: %d steps", current_prog_len);
    return (current_prog_len > 0) ? current_prog_len : 1; // Ensure it's at least 1
}


// --- Reset arpeggiator to chord mode with deadmau5-friendly settings ---
void reset_arp_to_chord_mode() {
    // Set mode to chord
    g_current_arp_mode = ARP_MODE_CHORD;

    // Reset with deadmau5-friendly defaults for chord mode
    g_current_arp_chord_rhythm_index = 0;  // Classic 4-chord progression
    g_current_arp_progression_index = 0;   // i-vi-iv-v progression (minor scale friendly)
    g_current_arp_scale_index = 0;         // Minor scale (classic for techno)
    g_current_arp_gate = 100;              // Longer notes for better flow (0-127)
    g_current_arp_swing = 72;              // Slight swing for groove (0-127, 64 is neutral)
    g_current_arp_velocity_pattern = 1;    // Accent on first beat pattern
    g_use_deadmau5_patterns = false;       // Start with regular progressions
    
    // For voicing groups, start with 0 (off) to use the smart chord selection by default
    // This ensures chord types match the scale appropriately
    g_current_arp_voicing_group = 0;       // Start with smart chord selection (no special voicing)
    
    // Start with bass pattern for fuller sound
    g_current_arp_bassline_pattern = 1;    // Use simple root note bass pattern

    // Reset internal state
    s_last_triggered_step_index = -1;
    s_current_arp_progression_step = 0;

    // Turn off any currently playing notes
    if (g_current_synth) {
        g_current_synth->sendAllNotesOff();
    }

    ESP_LOGI(TAG_ARP, "Arp reset to CHORD mode with deadmau5-optimized settings");
}

// --- Function dedicated to pot handling for Arp Adjust context ---
void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    // Note: Uses extern parameters defined in this file
    _update_pot_param(g_current_arp_gate, pot1_delta, 0, 127, "Arp Gate", pot1_used);
    _update_pot_param(g_current_arp_swing, pot2_delta, 0, 127, "Arp Swing", pot2_used);
}

// --- Handles secondary taps ONLY when Arp Adjust is active (Pad 1 held) ---
// Pot handling is done centrally via handle_arp_adjust_pots
bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state,
                            const std::chrono::microseconds& time,
                            const bool pad_pressed_this_tick[],
                            std::array<bool, 4>& pads_used) // Output: pads consumed
{
    // Improved submenu handling for arpeggiator
    bool adjustment_made = false;

    // Use helper to find the tapped secondary pad
    int tapped_pad = find_secondary_tapped_pad(ARP_PAD_INDEX, pad_pressed_this_tick, pads_used);

    // Handle action based on which secondary pad was tapped
    if (tapped_pad == 0) { // Pad 0 -> RHYTHM submenu
        if (g_current_arp_mode == ARP_MODE_NOTE) {
            // In note mode, cycle note rhythms
            g_current_arp_rhythm_index = (g_current_arp_rhythm_index + 1) % NUM_ARP_RHYTHMS;
            ESP_LOGI(TAG_ARP, "Arp RHYTHM submenu: Note Rhythm Index -> %d/%d",
                    g_current_arp_rhythm_index + 1, NUM_ARP_RHYTHMS);
        } else {
            // In chord mode, cycle chord rhythms
            g_current_arp_chord_rhythm_index = (g_current_arp_chord_rhythm_index + 1) % NUM_ARP_CHORD_RHYTHMS;
            ESP_LOGI(TAG_ARP, "Arp RHYTHM submenu: Chord Rhythm Index -> %d/%d",
                    g_current_arp_chord_rhythm_index + 1, NUM_ARP_CHORD_RHYTHMS);
        }
        s_last_triggered_step_index = -1; // Reset step tracking
        adjustment_made = true;

    } else if (tapped_pad == SIDECHAIN_PAD_INDEX) { // Pad 2 -> PROGRESSION submenu
        // Toggle between regular progressions and deadmau5 patterns for note mode
        if (g_current_arp_mode == ARP_MODE_NOTE) {
            // If not using deadmau5 patterns, first switch to those
            if (!g_use_deadmau5_patterns) {
                g_use_deadmau5_patterns = true;
                g_current_arp_deadmau5_pattern = 0; // Start with first pattern
                ESP_LOGI(TAG_ARP, "Arp PROGRESSION submenu: Switched to deadmau5 patterns (%d/%d)",
                         g_current_arp_deadmau5_pattern + 1, NUM_ARP_DEADMAU5_PATTERNS);
            } else {
                // Already using deadmau5 patterns, cycle through them
                g_current_arp_deadmau5_pattern = (g_current_arp_deadmau5_pattern + 1) % NUM_ARP_DEADMAU5_PATTERNS;
                if (g_current_arp_deadmau5_pattern == 0) {
                    // When we wrap around, switch back to regular progressions
                    g_use_deadmau5_patterns = false;
                    g_current_arp_progression_index = (g_current_arp_progression_index + 1) % NUM_ARP_NOTE_PROGRESSIONS;
                    ESP_LOGI(TAG_ARP, "Arp PROGRESSION submenu: Note Progression Index -> %d/%d",
                            g_current_arp_progression_index + 1, NUM_ARP_NOTE_PROGRESSIONS);
                } else {
                    ESP_LOGI(TAG_ARP, "Arp PROGRESSION submenu: deadmau5 pattern -> %d/%d",
                            g_current_arp_deadmau5_pattern + 1, NUM_ARP_DEADMAU5_PATTERNS);
                }
            }
        } else {
            // In chord mode, use more complex UI flow to allow both progression and voicing/bassline selection
            // Cycle: progressions -> voicings -> basslines -> progressions
            static int chord_submenu_state = 0; // 0=progressions, 1=voicings, 2=basslines
            
            switch (chord_submenu_state) {
                case 0: // Progressions state
                    g_current_arp_progression_index = (g_current_arp_progression_index + 1) % NUM_ARP_CHORD_PROGRESSIONS;
                    ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Progression Index -> %d/%d",
                            g_current_arp_progression_index + 1, NUM_ARP_CHORD_PROGRESSIONS);
                    
                    // Switch to voicings state when we wrap around
                    if (g_current_arp_progression_index == 0) {
                        chord_submenu_state = 1;
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Switching to VOICING selection");
                    }
                    break;
                    
                case 1: // Voicings state
                    // Cycle through: OFF, 1, 2, ... NUM_VOICING_GROUPS
                    g_current_arp_voicing_group = (g_current_arp_voicing_group + 1) % (NUM_ARP_VOICING_GROUPS + 1);
                    if (g_current_arp_voicing_group == 0) {
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Voicing Group -> OFF");
                        chord_submenu_state = 2; // Move to basslines when we wrap
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Switching to BASSLINE selection");
                    } else {
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Voicing Group -> %d/%d",
                                g_current_arp_voicing_group, NUM_ARP_VOICING_GROUPS);
                    }
                    break;
                    
                case 2: // Basslines state
                    // Cycle through: OFF, 1, 2, ... NUM_BASSLINE_PATTERNS
                    g_current_arp_bassline_pattern = (g_current_arp_bassline_pattern + 1) % (NUM_ARP_BASSLINE_PATTERNS + 1);
                    if (g_current_arp_bassline_pattern == 0) {
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Bassline Pattern -> OFF");
                        chord_submenu_state = 0; // Back to progressions
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Switching to PROGRESSION selection");
                    } else {
                        ESP_LOGI(TAG_ARP, "Arp CHORD submenu: Bassline Pattern -> %d/%d",
                                g_current_arp_bassline_pattern, NUM_ARP_BASSLINE_PATTERNS);
                    }
                    break;
            }
        }
        // Reset progression step when changing progression
        s_current_arp_progression_step = 0;
        s_last_triggered_step_index = -1; // Reset rhythm step tracking
        adjustment_made = true;

    } else if (tapped_pad == 3) { // Pad 3 -> SCALE/VELOCITY submenu
        // Double-tap behavior: First tap cycles scales, second tap cycles velocity patterns
        uint64_t current_time_us = esp_timer_get_time();
        static uint64_t last_scale_tap_time = 0;
        static bool in_velocity_mode = false;
        
        // If it's been more than 1 second since last tap, reset to scale mode
        if (current_time_us - last_scale_tap_time > 1000000) {
            in_velocity_mode = false;
        }
        
        if (!in_velocity_mode) {
            // Regular tap behavior - cycle scales
            g_current_arp_scale_index = (g_current_arp_scale_index + 1) % NUM_ARP_SCALES;
            ESP_LOGI(TAG_ARP, "Arp SCALE submenu: Scale Index -> %d/%d",
                    g_current_arp_scale_index + 1, NUM_ARP_SCALES);
            
            // Switch to velocity mode for the next tap
            in_velocity_mode = true;
        } else {
            // Second tap in sequence - cycle velocity patterns
            g_current_arp_velocity_pattern = (g_current_arp_velocity_pattern + 1) % NUM_ARP_VELOCITY_PATTERNS;
            ESP_LOGI(TAG_ARP, "Arp VELOCITY submenu: Velocity Pattern -> %d/%d",
                    g_current_arp_velocity_pattern + 1, NUM_ARP_VELOCITY_PATTERNS);
            
            // If we wrap around to 0, go back to scale mode
            if (g_current_arp_velocity_pattern == 0) {
                in_velocity_mode = false;
            }
        }
        
        // Update the time of the last tap
        last_scale_tap_time = current_time_us;
        adjustment_made = true;

    } else if (tapped_pad == ARP_PAD_INDEX) { // Pad 1 double-tap -> Toggle between note and chord modes
        if (g_current_arp_mode == ARP_MODE_NOTE) {
            g_current_arp_mode = ARP_MODE_CHORD;
            ESP_LOGI(TAG_ARP, "Switched to CHORD mode");
        } else {
            g_current_arp_mode = ARP_MODE_NOTE;
            ESP_LOGI(TAG_ARP, "Switched to NOTE mode");
        }
        s_last_triggered_step_index = -1; // Reset step tracking
        s_current_arp_progression_step = 0; // Reset progression step
        adjustment_made = true;
    }

    // Mark all pads as consumed if adjustment was made
    if (adjustment_made) {
        for (int i = 0; i < 4; i++) {
            if (pad_pressed_this_tick[i]) {
                pads_used[i] = true;
            }
        }
    }

    return adjustment_made;
}


// --- Internal Helper: Play Arp Chord ---
void _play_arp_chord(int velocity, int note_index) {
    int chord_prog_idx = g_current_arp_progression_index % NUM_ARP_CHORD_PROGRESSIONS;
    const auto& chord_prog = ARP_CHORD_PROGRESSIONS[chord_prog_idx];
    if (chord_prog.empty()) {
        ESP_LOGE(TAG_ARP, "Chord progression %d is empty!", chord_prog_idx);
        return;
    }

    // FIXED: Don't apply modulo again, use the note_index directly
    // The modulo was already applied in handle_arp_active when incrementing s_current_arp_progression_step
    // int chord_step_idx = note_index % chord_prog.size();
    int chord_step_idx = note_index;
    int chord_root_degree = chord_prog[chord_step_idx];
    
    ESP_LOGV(TAG_ARP, "Chord progression step: %d/%d (progression idx: %d)", 
             chord_step_idx + 1, chord_prog.size(), chord_prog_idx);

    // Determine if we should use a voicing group instead of a basic chord
    bool use_voicing_group = (g_current_arp_voicing_group > 0);
    
    // Get the intervals to use - either from a basic chord or a voicing group
    std::vector<int> intervals_to_use;
    
    // Create arrays to classify scales by type - used for smart chord selection
    // Indices of scales that are minor-type scales (containing a minor 3rd)
    const std::vector<int> MINOR_SCALES = {0, 1, 2}; // Minor, Minor Pentatonic, Dorian
    // Indices of scales that are major-type scales (containing a major 3rd)
    const std::vector<int> MAJOR_SCALES = {3, 4, 5}; // Mixolydian, Major Pentatonic, Major
    
    // Define chord indices by type (used for smart mapping)
    const std::vector<int> MINOR_CHORDS = {0, 2, 12, 14}; // Minor triad, Minor 7th, Minor 7 no 5th, 1st inv minor
    const std::vector<int> MAJOR_CHORDS = {1, 3, 11, 13}; // Major triad, Major 7th, Major 7 no 5th, 1st inv major
    const std::vector<int> NEUTRAL_CHORDS = {4, 5, 6, 7, 8, 9, 10}; // Power/open/sus chords (work with any scale)
    
    if (use_voicing_group) {
        // Use a voicing group for more complex chord structures
        int voicing_idx = (g_current_arp_voicing_group - 1) % NUM_ARP_VOICING_GROUPS;
        intervals_to_use = ARP_VOICING_GROUPS[voicing_idx];
        ESP_LOGV(TAG_ARP, "Using voicing group %d for chord", voicing_idx);
    } else {
        // Smart chord selection based on scale type
        // Get current scale index and determine if it's major or minor
        int scale_idx = g_current_arp_scale_index % NUM_ARP_SCALES;
        bool is_minor_scale = std::find(MINOR_SCALES.begin(), MINOR_SCALES.end(), scale_idx) != MINOR_SCALES.end();
        bool is_major_scale = std::find(MAJOR_SCALES.begin(), MAJOR_SCALES.end(), scale_idx) != MAJOR_SCALES.end();
        
        // Default chord selection algorithm based on progression - will be overridden with smart selection
        int chord_type_idx = chord_prog_idx % NUM_ARP_CHORDS;
        
        // Smart chord selection based on scale type
        if (is_minor_scale) {
            // For minor-type scales, use either minor or neutral chords
            // Randomly choose minor or neutral based on position in progression
            if (chord_step_idx == 0 || chord_step_idx == 3) {
                // For 1st and 4th positions, prefer minor chords
                chord_type_idx = MINOR_CHORDS[chord_step_idx % MINOR_CHORDS.size()];
            } else if (chord_step_idx == 2) {
                // For 3rd position, use neutral chords (open sound)
                chord_type_idx = NEUTRAL_CHORDS[chord_step_idx % NEUTRAL_CHORDS.size()];
            } else {
                // For other positions, sometimes use major chords for interest
                chord_type_idx = (arp_random_int(0, 10) < 3) ? 
                    MAJOR_CHORDS[chord_step_idx % MAJOR_CHORDS.size()] : 
                    MINOR_CHORDS[chord_step_idx % MINOR_CHORDS.size()];
            }
        } else if (is_major_scale) {
            // For major-type scales, use either major or neutral chords
            if (chord_step_idx == 0 || chord_step_idx == 3) {
                // For 1st and 4th positions, prefer major chords
                chord_type_idx = MAJOR_CHORDS[chord_step_idx % MAJOR_CHORDS.size()];
            } else if (chord_step_idx == 2) {
                // For 3rd position, use neutral chords
                chord_type_idx = NEUTRAL_CHORDS[chord_step_idx % NEUTRAL_CHORDS.size()];
            } else {
                // For other positions, sometimes use minor chords for interest
                chord_type_idx = (arp_random_int(0, 10) < 3) ? 
                    MINOR_CHORDS[chord_step_idx % MINOR_CHORDS.size()] : 
                    MAJOR_CHORDS[chord_step_idx % MAJOR_CHORDS.size()];
            }
        } else {
            // For other scales, use neutral chords mostly
            if (arp_random_int(0, 10) < 7) {
                chord_type_idx = NEUTRAL_CHORDS[chord_step_idx % NEUTRAL_CHORDS.size()];
            } else {
                // Occasionally use other chord types for variety
                chord_type_idx = chord_step_idx % NUM_ARP_CHORDS;
            }
        }
        
        // Safety check
        if (chord_type_idx < 0 || chord_type_idx >= NUM_ARP_CHORDS) 
            chord_type_idx = 0;
            
        intervals_to_use = ARP_CHORDS[chord_type_idx];
        ESP_LOGV(TAG_ARP, "Using chord type %d for chord with scale %d", chord_type_idx, scale_idx);
    }
    
    // Check if we should add a bassline note
    bool add_bassline = (g_current_arp_bassline_pattern > 0);
    int bassline_note = -1;
    
    if (add_bassline) {
        int bassline_idx = (g_current_arp_bassline_pattern - 1) % NUM_ARP_BASSLINE_PATTERNS;
        const auto& bassline = ARP_BASSLINE_PATTERNS[bassline_idx];
        int pattern_step = chord_step_idx % bassline.size();
        int bass_degree = bassline[pattern_step];
        
        // Calculate the bassline note - one octave lower than root
        bassline_note = get_arp_note_from_degree(g_current_arp_scale_index, 
                                             chord_root_degree + bass_degree, 
                                             g_current_arp_root_note, 
                                             g_current_arp_octave - 1);
        
        ESP_LOGV(TAG_ARP, "Adding bassline note: %d (pattern %d, step %d)", 
                 bassline_note, bassline_idx, pattern_step);
    }

    int base_note = get_arp_note_from_degree(g_current_arp_scale_index, chord_root_degree, 
                                          g_current_arp_root_note, g_current_arp_octave);

    if (base_note != -1) {
        ESP_LOGV(TAG_ARP, "Chord ON: Base=%d, Vel=%d, ProgStep=%d/%d", 
                 base_note, velocity, chord_step_idx + 1, chord_prog.size());
        
        s_last_played_arp_chord_notes.clear(); // Clear previous chord notes
        
        // Add bassline note if it exists
        if (bassline_note != -1) {
            int bass_velocity = static_cast<int>(velocity * 1.1f); // Slightly louder
            bass_velocity = clamp_value(bass_velocity, 0, 127);
            g_current_synth->sendNoteOn(static_cast<uint8_t>(bassline_note), 
                                      static_cast<uint8_t>(bass_velocity));
            s_last_played_arp_chord_notes.push_back(bassline_note);
            ESP_LOGV(TAG_ARP, "  Bass Note ON: %d (vel: %d)", bassline_note, bass_velocity);
        }
        
        // Add chord notes
        for (int interval : intervals_to_use) {
            int note = clamp_value(base_note + interval, 0, 127);
            g_current_synth->sendNoteOn(static_cast<uint8_t>(note), 
                                      static_cast<uint8_t>(velocity));
            s_last_played_arp_chord_notes.push_back(note); // Track notes for Note Off
            ESP_LOGV(TAG_ARP, "  Chord Note ON: %d", note);
        }
    } else {
        ESP_LOGW(TAG_ARP, "ARP Chord: Invalid base note for degree %d, scale %d", 
                chord_root_degree, g_current_arp_scale_index);
    }
}

// --- Internal Helper: Play Arp Note ---
void _play_arp_note(int velocity, int note_index) {
    int note_to_play = -1;
    
    // Check if we should use deadmau5 patterns
    if (g_use_deadmau5_patterns && g_current_arp_mode == ARP_MODE_NOTE) {
        int deadmau5_idx = g_current_arp_deadmau5_pattern % NUM_ARP_DEADMAU5_PATTERNS;
        const auto& pattern = ARP_DEADMAU5_PATTERNS[deadmau5_idx];
        if (!pattern.empty()) {
            // FIXED: Don't apply modulo again, use note_index directly
            // The modulo was already applied in handle_arp_active
            // int pattern_step = note_index % pattern_len;
            int pattern_step = note_index;
            int degree = pattern[pattern_step];
            
            // Ensure degree is within the current scale for better harmony
            int scale_idx = g_current_arp_scale_index % NUM_ARP_SCALES;
            int scale_size = ARP_SCALES[scale_idx].size();
            
            // Adjust the degree to ensure it stays within the scale
            // This preserves the general contour of the pattern while avoiding dissonance
            if (scale_size > 0) {
                // For octaves (degree % 12 == 0), preserve them as they always sound good
                if (degree % 12 != 0) {
                    // Find the closest scale degree for non-octave intervals
                    int octave_offset = degree / 12;
                    int degree_within_octave = degree % 12;
                    
                    // Find the closest note in scale to the original degree
                    int closest_scale_note = -1;
                    int min_distance = 12;
                    
                    for (int scale_note : ARP_SCALES[scale_idx]) {
                        int distance = std::abs(scale_note - degree_within_octave);
                        if (distance < min_distance) {
                            min_distance = distance;
                            closest_scale_note = scale_note;
                        }
                    }
                    
                    // Replace with closest scale note
                    degree = closest_scale_note + (octave_offset * 12);
                }
            }
            
            note_to_play = get_arp_note_from_degree(scale_idx, degree, g_current_arp_root_note, g_current_arp_octave);
            
            ESP_LOGV(TAG_ARP, "deadmau5 Pattern %d Step %d -> Adjusted Degree %d, Note %d", 
                    deadmau5_idx, pattern_step, degree, note_to_play);
        }
    } else {
        int note_prog_idx = g_current_arp_progression_index % NUM_ARP_NOTE_PROGRESSIONS;
        const auto& progression = ARP_NOTE_PROGRESSIONS[note_prog_idx];
        if (progression.empty()) {
            ESP_LOGE(TAG_ARP, "Note progression %d is empty!", note_prog_idx);
            return;
        }

        // FIXED: Don't apply modulo again, use note_index directly 
        // int progression_length = progression.size();
        // int note_step_idx = (progression_length > 0) ? (note_index % progression_length) : 0;
        int note_step_idx = note_index;

        int scale_degree = progression[note_step_idx];
        
        // Safety check - ensure scalar value is valid for current scale
        int scale_idx = g_current_arp_scale_index % NUM_ARP_SCALES;
        int scale_size = ARP_SCALES[scale_idx].size();
        
        // Preserve octaves and fifths, but check other intervals for harmony
        if (scale_size > 0 && scale_degree % 12 != 0 && scale_degree % 12 != 7) {
            // For non-octave/fifth intervals, check if they're in the scale
            int octave_offset = scale_degree / 12;
            int degree_within_octave = scale_degree % 12;
            
            // Find if this exact degree is in the scale
            bool in_scale = false;
            for (int scale_note : ARP_SCALES[scale_idx]) {
                if (scale_note == degree_within_octave) {
                    in_scale = true;
                    break;
                }
            }
            
            // If not in scale, find closest scale tone
            if (!in_scale) {
                int closest_scale_note = -1;
                int min_distance = 12;
                
                for (int scale_note : ARP_SCALES[scale_idx]) {
                    int distance = std::abs(scale_note - degree_within_octave);
                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_scale_note = scale_note;
                    }
                }
                
                // Apply correction to keep within scale
                scale_degree = closest_scale_note + (octave_offset * 12);
                ESP_LOGV(TAG_ARP, "Corrected out-of-scale degree %d to %d for better harmony", 
                        progression[note_step_idx], scale_degree);
            }
        }
        
        note_to_play = get_arp_note_from_degree(scale_idx, scale_degree, g_current_arp_root_note, g_current_arp_octave);

        ESP_LOGV(TAG_ARP, "Note ON: Index=%d -> ProgStep=%d, Degree=%d, Note=%d, Vel=%d", 
                note_index, note_step_idx, scale_degree, note_to_play, velocity);
    }

    if (note_to_play != -1) {
        g_current_synth->sendNoteOn(clamp_value<int>(note_to_play, 0, 127), static_cast<uint8_t>(velocity));
        s_last_played_arp_note = note_to_play; // Track for Note Off
    } else {
        ESP_LOGW(TAG_ARP, "ARP Note: Failed to get valid note for progression step %d, scale %d", note_index, g_current_arp_scale_index);
    }
}


// --- Update handleArpActive with stronger quantum alignment for progressions ---
bool handle_arp_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int note_index)
{
    bool note_played = false;
    if (!g_current_synth || !g_link) return note_played;

    // Get current time and tempo
    uint64_t current_time_us = g_link->clock().micros().count();
    double tempo = state.tempo();
    if (tempo <= 0) return note_played; // Avoid division by zero if tempo is invalid
    double seconds_per_beat = 60.0 / tempo;
    
    // Define a small timing offset to send notes early
    // Calculate based on tempo - faster tempos need smaller offsets to avoid getting too far ahead
    // At 120 BPM, a sixteenth note is ~125ms, so an offset of ~10-20ms is reasonable
    double early_send_offset_beats = 0.01; // 1/100th of a beat early (adjust as needed)
    double early_send_offset_us = early_send_offset_beats * seconds_per_beat * 1000000.0;
    uint64_t early_trigger_offset_us = static_cast<uint64_t>(early_send_offset_us);

    // --- Note Off Logic --- (Based on time)
    // Always check if we need to turn off notes
    if (s_arp_note_off_time_us > 0) {
        if (current_time_us >= s_arp_note_off_time_us) {
            // Turn off any playing notes
            if (s_last_played_arp_note != -1) {
                g_current_synth->sendNoteOff(s_last_played_arp_note);
                ESP_LOGD(TAG_ARP, "ARP Note OFF: %d", s_last_played_arp_note);
                s_last_played_arp_note = -1;
            }
            for (int note : s_last_played_arp_chord_notes) {
                g_current_synth->sendNoteOff(note);
                ESP_LOGD(TAG_ARP, "ARP Chord Note OFF: %d", note);
            }
            s_last_played_arp_chord_notes.clear();
            s_arp_note_off_time_us = 0; // Reset schedule
        } else {
            // Check if the note off time is too far in the future (safety check)
            uint64_t max_note_duration_us = static_cast<uint64_t>(seconds_per_beat * LINK_QUANTUM * 1000000.0);
            if (s_arp_note_off_time_us > current_time_us + max_note_duration_us) {
                ESP_LOGW(TAG_ARP, "Note off time too far in future, clamping");
                s_arp_note_off_time_us = current_time_us + max_note_duration_us;
            }
        }
    }

    // --- Quantum-Aligned Rhythm and Progression ---
    // Get quantum boundary and phase information using the common function
    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    // Extract values from the quantum info for use in this function
    double phase_within_quantum = quantumInfo.phaseWithinQuantum;
    double quantum_beat = quantumInfo.sessionBeat;
    bool quantum_boundary_crossed = quantumInfo.crossedQuantumBoundary;
    bool use_chord = (g_current_arp_mode == ARP_MODE_CHORD);

    // Handle quantum boundary crossing - ENHANCED for stronger alignment
    if (quantum_boundary_crossed) {
        ESP_LOGI(TAG_ARP, "ARP: Quantum boundary crossed at beat %.2f", quantum_beat);

        // Turn off any currently playing notes at quantum boundary
        if (s_last_played_arp_note != -1) {
            g_current_synth->sendNoteOff(s_last_played_arp_note);
            ESP_LOGD(TAG_ARP, "ARP Note OFF at quantum boundary: %d", s_last_played_arp_note);
            s_last_played_arp_note = -1;
        }
        for (int note : s_last_played_arp_chord_notes) {
            g_current_synth->sendNoteOff(note);
            ESP_LOGD(TAG_ARP, "ARP Chord Note OFF at quantum boundary: %d", note);
        }
        s_last_played_arp_chord_notes.clear();
        s_arp_note_off_time_us = 0; // Reset schedule

        // Reset step tracking to force recalculation
        s_last_triggered_step_index = -1;
        
        // ENHANCED QUANTUM ALIGNMENT: Always reset to first note in progression at quantum boundary
        s_current_arp_progression_step = 0;
        ESP_LOGI(TAG_ARP, "Arp progression reset to FIRST STEP at quantum boundary to ensure sync");
    }

    // Get the current rhythm pattern
    int current_rhythm_idx = use_chord ?
        (g_current_arp_chord_rhythm_index % NUM_ARP_CHORD_RHYTHMS) :
        (g_current_arp_rhythm_index % NUM_ARP_RHYTHMS);
    const std::vector<bool>& rhythm = use_chord ?
        ARP_CHORD_RHYTHMS[current_rhythm_idx] :
        ARP_RHYTHMS[current_rhythm_idx];
    int rhythm_steps = rhythm.size();

    // Safety check for empty rhythm
    if (rhythm_steps <= 0) {
        ESP_LOGE(TAG_ARP, "Error: Selected rhythm (chord=%d, index=%d) is empty!", use_chord, current_rhythm_idx);
        s_last_triggered_step_index = -1;
        return note_played;
    }

    // Double-check that our current progression length is valid (safety check)
    int current_prog_len = _get_current_progression_length();
    if (current_prog_len <= 0) {
        ESP_LOGE(TAG_ARP, "Error: Invalid progression length: %d, defaulting to 1", current_prog_len);
        current_prog_len = 1;
    }
    
    // Verify our progression step is within bounds
    if (s_current_arp_progression_step < 0 || s_current_arp_progression_step >= current_prog_len) {
        ESP_LOGW(TAG_ARP, "Progression step out of bounds: %d/%d, resetting to 0", 
                s_current_arp_progression_step, current_prog_len);
        s_current_arp_progression_step = 0;
    }

    // ENHANCED QUANTUM ALIGNMENT: Calculate step duration to ensure perfect quantum alignment
    // Ensure rhythm aligns perfectly with the quantum by using beat phase
    double beats_per_quantum = LINK_QUANTUM;
    double beats_per_step = beats_per_quantum / static_cast<double>(rhythm_steps);

    // Apply speed factor for melody mode
    if (!use_chord) {
        beats_per_step /= MELODY_SPEED_FACTOR;
    }
    
    // Apply early triggering offset to phase - shift our perception of the current time 
    // slightly ahead so we trigger notes early
    double adjusted_phase_within_quantum = phase_within_quantum + early_send_offset_beats;
    
    // Handle edge case: if we're right at the end of the quantum with our adjustment
    if (adjusted_phase_within_quantum >= LINK_QUANTUM) {
        adjusted_phase_within_quantum -= LINK_QUANTUM;
    }

    // ENHANCED QUANTUM ALIGNMENT: Calculate current step position relative to start of current quantum
    // This ensures phases always stay aligned even after tempo changes
    // NOW using the adjusted phase to trigger slightly early
    int current_step_index = static_cast<int>(floor(adjusted_phase_within_quantum / beats_per_step));

    // Ensure step index is within bounds (should always be the case, but just to be safe)
    current_step_index = current_step_index % rhythm_steps;

    // Log quantum alignment information for debugging
    ESP_LOGV(TAG_ARP, "ARP Quantum Alignment: Beat=%.2f, Phase=%.2f, Adjusted Phase=%.2f, Step=%d/%d, Prog=%d",
             quantum_beat, phase_within_quantum, adjusted_phase_within_quantum, current_step_index, rhythm_steps, s_current_arp_progression_step);

    // Only trigger a new note if the step has changed
    if (current_step_index != s_last_triggered_step_index) {
        ESP_LOGD(TAG_ARP, "ARP Step Change: %d -> %d (Beat: %.2f, Early Trigger: %.2f ms ahead)",
                s_last_triggered_step_index, current_step_index, quantum_beat, early_send_offset_us / 1000.0);

        // Check if this step should play a note based on the rhythm pattern
        bool should_play = rhythm[current_step_index];

        if (should_play) {
            // Turn off any currently playing notes
            if (s_last_played_arp_note != -1) {
                g_current_synth->sendNoteOff(s_last_played_arp_note);
                s_last_played_arp_note = -1;
            }
            for (int note : s_last_played_arp_chord_notes) {
                g_current_synth->sendNoteOff(note);
            }
            s_last_played_arp_chord_notes.clear();

            // Apply velocity pattern
            int base_velocity = g_current_arp_velocity;
            const auto& vel_pattern = ARP_VELOCITY_PATTERNS[g_current_arp_velocity_pattern % NUM_ARP_VELOCITY_PATTERNS];
            int pattern_position = current_step_index % vel_pattern.size();
            int pattern_velocity = vel_pattern[pattern_position];
            
            // Scale the pattern velocity by the base velocity
            float velocity_scale = static_cast<float>(base_velocity) / 100.0f; // Base velocity as scaling factor
            int velocity = static_cast<int>(pattern_velocity * velocity_scale);
            
            // Add small random variation
            velocity += arp_random_int(-5, 5);
            velocity = clamp_value(velocity, 0, 127);

            // Apply simple swing for even-numbered steps
            // Swing is applied by shortening the note duration, not delaying the note
            float swing_factor = 1.0f;
            if ((rhythm_steps % 2 == 0) && (current_step_index % 2 != 0) && g_current_arp_swing != 64) {
                // Calculate swing factor: 1.0 at 64, down to 0.5 at 127 (shorter notes)
                swing_factor = 1.0f - (std::max(0.0f, (static_cast<float>(g_current_arp_swing) - 64.0f) / 127.0f) * 0.5f);
            }

            // Advance to the next progression step whenever a note is played
            // But only if we haven't just crossed a quantum boundary
            if (!quantum_boundary_crossed) {
                // Advance progression step - explicitly tied to rhythmic position
                int prev_step = s_current_arp_progression_step;
                s_current_arp_progression_step = (s_current_arp_progression_step + 1) % current_prog_len;
                ESP_LOGI(TAG_ARP, "Arp progression: %d -> %d (length: %d)",
                        prev_step, s_current_arp_progression_step, current_prog_len);
            }

            // Play the note or chord immediately - use progression step instead of note index
            if (use_chord) {
                // For chord mode, use the current progression step
                _play_arp_chord(velocity, s_current_arp_progression_step);
                note_played = !s_last_played_arp_chord_notes.empty();
            } else {
                // For note mode, use the current progression step
                _play_arp_note(velocity, s_current_arp_progression_step);
                note_played = (s_last_played_arp_note != -1);
            }

            // Calculate note duration based on gate parameter
            float gate_percent = static_cast<float>(g_current_arp_gate) / 127.0f;
            float note_duration_beats = beats_per_step * gate_percent * swing_factor;

            // ENHANCED QUANTUM ALIGNMENT: Calculate time to next step relative to quantum
            // This ensures we handle phase changes correctly 
            int next_step_index = (current_step_index + 1) % rhythm_steps;
            double next_step_phase = next_step_index * beats_per_step;
            
            // Calculate time to next step based on actual phase, not adjusted phase,
            // since note offs should occur at the mathematically correct time
            double time_to_next_step_beats = next_step_phase - phase_within_quantum;
            if (time_to_next_step_beats <= 0) {
                // We're at the last step in the rhythm, so wrap to the start of the next quantum
                time_to_next_step_beats = LINK_QUANTUM - phase_within_quantum;
            }
            
            // Special handling for the 4th chord in progressions to prevent it from being cut short
            if (use_chord && s_current_arp_progression_step == 3) {
                // For the 4th chord position, ensure it gets at least as much time as other chords
                // by using the standard chord duration
                double min_chord_duration = beats_per_step * 0.95;
                time_to_next_step_beats = std::max(time_to_next_step_beats, min_chord_duration);
            }
            
            // Apply early triggering adjustment only to note-off timing
            // This ensures notes have their proper duration and release properly
            time_to_next_step_beats -= early_send_offset_beats;
            if (time_to_next_step_beats <= 0) {
                // Avoid negative durations
                time_to_next_step_beats = beats_per_step * 0.1; // Very short duration
            }

            // Ensure note duration doesn't exceed time to next step (95% of time to avoid overlap)
            float time_to_next_step_beats_f = static_cast<float>(time_to_next_step_beats * 0.95);
            if (note_duration_beats > time_to_next_step_beats_f) {
                note_duration_beats = time_to_next_step_beats_f;
            }

            // Convert to microseconds
            uint64_t note_duration_us = static_cast<uint64_t>(note_duration_beats * seconds_per_beat * 1000000.0);

            // Ensure minimum duration
            const uint64_t MIN_DURATION_US = 10000; // 10ms minimum
            note_duration_us = std::max(note_duration_us, MIN_DURATION_US);

            // Ensure maximum duration (safety check)
            const uint64_t MAX_DURATION_US = static_cast<uint64_t>(seconds_per_beat * beats_per_step * 1000000.0);
            if (note_duration_us > MAX_DURATION_US) {
                note_duration_us = MAX_DURATION_US;
            }

            // Schedule note off
            s_arp_note_off_time_us = current_time_us + note_duration_us;

            ESP_LOGD(TAG_ARP, "ARP Note ON: Step=%d/%d, ProgStep=%d/%d, Duration=%llu us, Gate=%d, Early=%llu us",
                    current_step_index, rhythm_steps, s_current_arp_progression_step, current_prog_len, note_duration_us, 
                    g_current_arp_gate, early_trigger_offset_us);
        } else {
            ESP_LOGV(TAG_ARP, "ARP Step %d is silent.", current_step_index);
        }

        // Update last triggered step
        s_last_triggered_step_index = current_step_index;
    }

    return note_played;
}