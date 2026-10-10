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

MidiFilePlayer g_midi_player;
bool g_midi_player_active = false;
int g_current_arp_transpose = 0;
double g_current_arp_playback_rate = 1.0;

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

const char* FILTER_MIDI_FOLDER = "/spiffs/loops/notes/fil";
const char* REVERSE_MIDI_FOLDER = "/spiffs/loops/notes/rev";
const char* SIDECHAIN_MIDI_FOLDER = "/spiffs/loops/notes/sid";

bool directoryExists(const char* path) {
    DIR* dir = opendir(path);
    if (dir) {
        closedir(dir);
        return true;
    }

    char search_path[512];
    snprintf(search_path, sizeof(search_path), "%s/", path);

    DIR* parent_dir = opendir("/spiffs");
    if (!parent_dir) {
        return false;
    }

    bool exists = false;
    struct dirent* entry;
    while ((entry = readdir(parent_dir)) != NULL) {
        char full_path[512];
        snprintf(full_path, sizeof(full_path), "/spiffs/%s", entry->d_name);

        if (strstr(full_path, search_path) == full_path) {
            exists = true;
            break;
        }
    }

    closedir(parent_dir);
    return exists;
}

void reset_arp_to_midi_player(bool unused) {
    g_midi_player.stop();

    if (!g_current_synth) {
        ESP_LOGE(TAG, "ERROR: g_current_synth is null! MIDI playback will fail.");
        return;
    }

    const char* folder_type = (g_synth_type == SYNTH_MININOVA) ? "chords" : "notes";
    char target_folder[64] = {0};
    snprintf(target_folder, sizeof(target_folder), "/spiffs/loops/%s/arp", folder_type);

    ESP_LOGI(TAG, "Setting MIDI folder: %s (synth: %s)", target_folder,
             (g_synth_type == SYNTH_MININOVA) ? "Mininova" : "MicroKorg");

    g_midi_player.setFolder(target_folder);

    g_current_arp_transpose = 0;
    g_current_arp_playback_rate = 1.0;
    g_midi_player.setTranspose(g_current_arp_transpose);
    g_midi_player.setPlaybackRate(g_current_arp_playback_rate);

    if (g_midi_player.start()) {
        g_midi_player_active = true;

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

bool handle_arp_active(const ableton::Link::SessionState& state,
                      const std::chrono::microseconds& time,
                      int note_index) {
    if (!g_midi_player_active) {
        return false;
    }

    g_midi_player.process(state, time);

    return true;
}

bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state,
                               const std::chrono::microseconds& time,
                               const bool pad_pressed_this_tick[],
                               std::array<bool, 4>& pads_used) {
    bool any_pads_pressed = false;
    for (int i = 0; i < 4; i++) {
        if (i != ARP_PAD_INDEX && pad_pressed_this_tick[i]) {
            any_pads_pressed = true;
            ESP_LOGI(TAG, "ARP handler detected pad %d pressed", i);
            break;
        }
    }

    if (!any_pads_pressed) {
        return false;
    }

    bool handled = false;

    for (int i = 0; i < 4; i++) {
        if (i == ARP_PAD_INDEX) continue;

        if (pad_pressed_this_tick[i]) {
            pads_used[i] = true;
            handled = true;

            ESP_LOGI(TAG, "Switching to effect folder for pad %d while maintaining file index", i);
            g_midi_player.stop();
            g_midi_player_active = false;

            const char* target_folder = nullptr;
            char folder_path[64] = {0};

            const char* folder_type = (g_synth_type == SYNTH_MININOVA) ? "chords" : "notes";

            if (i == FILTER_PAD_INDEX) {
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/fil", folder_type);
                target_folder = folder_path;
            } else if (i == SIDECHAIN_PAD_INDEX) {
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/sid", folder_type);
                target_folder = folder_path;
            } else if (i == DELAY_REVERB_PAD_INDEX) {
                snprintf(folder_path, sizeof(folder_path), "/spiffs/loops/%s/rev", folder_type);
                target_folder = folder_path;
            }

            if (target_folder == nullptr) {
                ESP_LOGE(TAG, "Failed to determine target folder for pad %d", i);
                return handled;
            }

            size_t currentIndex = g_midi_player.getCurrentFileIndex();

            g_midi_player.setFolder(target_folder);

            g_midi_player.setCurrentFileIndex(currentIndex);

            if (g_midi_player.start()) {
                g_midi_player_active = true;
                ESP_LOGI(TAG, "Started MIDI playback from folder: %s with file index: %zu", target_folder, currentIndex);
            } else {
                ESP_LOGE(TAG, "Failed to start MIDI playback from folder: %s", target_folder);
            }
            break;
        }
    }

    return handled;
}

void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used) {
    if (pot1_delta != 0) {
        int pot1_value = g_midi_player.getPot1Value();
        pot1_value += pot1_delta;
        pot1_value = std::min(127, std::max(0, pot1_value));

        double note_length_scale = (pot1_value / 127.0) * 2.0;

        g_midi_player.setNoteLengthScale(note_length_scale);
        g_midi_player.setPot1Value(pot1_value);

        ESP_LOGI(TAG, "Note Length Scale: %.0f%% (Pot1: %d)", note_length_scale * 100, pot1_value);
        pot1_used = true;
    }

    if (pot2_delta != 0) {
        int pot2_value = g_midi_player.getPot2Value();
        pot2_value += pot2_delta;
        pot2_value = std::min(127, std::max(0, pot2_value));

        double velocity_scale = pot2_value / 127.0;

        g_midi_player.setVelocityScale(velocity_scale);
        g_midi_player.setPot2Value(pot2_value);

        ESP_LOGI(TAG, "Velocity Scale: %.0f%% (Pot2: %d)", velocity_scale * 100, pot2_value);
        pot2_used = true;
    }
}
