//idf 4.4.4
#include "main.h"
#include "io_helpers.h"
#include "link_sync.h"
#include "input_handler.h"
#include "state_machine.h"
#include "bass_engine.h"
#include "synth_interface.h"
#include "synth_mininova.h"
#include "network_midi.h"
#include "wifi_config.h"
#include "provisioning.h"
#include "esp_log.h"
#include <stdio.h>
#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"

// Define the logging tag for this file
static const char *TAG = "MAIN";

bool g_provisioning_complete = false;

// Required globals (still declared extern in main.h)
SynthInterface* g_current_synth = nullptr;
SynthType       g_synth_type    = SYNTH_MININOVA;

// Double Tap Timing Constants
const uint64_t DOUBLE_TAP_TIME_MS = 300;
const uint64_t HOLD_TIME_MS       = 200;

// Link Object
std::unique_ptr<ableton::Link> g_link;

// --- tickTask --- (Main application loop)
void tickTask(void *userParam) {
    // --- Initialization ---
    init_uart_midi();
    init_adc();
    init_hall_sensor();
    ESP_LOGE("MAIN", "Calling init_touch_pads()...");
    init_touch_pads();
    ESP_LOGE("MAIN", "init_touch_pads() finished.");
    setup_buzzer();

    // Initialize Inputs (Reads initial state)
    initialize_inputs();

    TaskHandle_t current_task_handle = xTaskGetCurrentTaskHandle();
    init_link_timer(current_task_handle); // <<--- RE-ENABLING Link GPTimer
    // ESP_LOGW(TAG, "Link GPTimer initialization has been SKIPPED for touch sensor debugging."); // REMOVED WARNING LOG

    // Initialize Link object with default tempo
    g_link = std::make_unique<ableton::Link>(120.0);

    // Enable Link and set quantum for proper phrase alignment
    g_link->enable(true);

    // Capture the initial session state
    auto sessionState = g_link->captureAppSessionState();

    // Force the initial beat to align with quantum boundaries
    // This ensures all systems start in phrase with the quantum
    const auto time = g_link->clock().micros();
    const double initialBeat = sessionState.beatAtTime(time, LINK_QUANTUM);
    const double initialPhase = sessionState.phaseAtTime(time, LINK_QUANTUM);

    // Calculate the nearest quantum boundary
    const double quantumBoundary = std::floor(initialBeat / LINK_QUANTUM) * LINK_QUANTUM;

    // Force the beat to the quantum boundary
    sessionState.forceBeatAtTime(quantumBoundary, time, LINK_QUANTUM);

    // Commit the changes back to Link
    g_link->commitAppSessionState(sessionState);

    ESP_LOGI(TAG, "Link initialized and aligned to quantum boundary: %.1f (from beat %.2f, phase %.2f)",
             quantumBoundary, initialBeat, initialPhase);

    ESP_LOGI(TAG, "Initialization complete. Starting main loop.");

    // --- Link State Variables (Local to tickTask scope, passed by ref to handle_link_sync) ---
    bool was_connected = false;
    int64_t start_wait_time = esp_timer_get_time();
    bool force_start = false;
    static int lastTicks = 0;
    static int length = LENGTH_NORMAL;
    static int lastBeat = -1;
    static int currentBuzzerFreq = FREQ_NORMAL;
    static bool was_playing = false;

    // Input Event struct to hold current inputs
    InputEvent current_input_event;
    
    // Variables to track notification source
    uint32_t ulNotifiedValue;
    const uint32_t MAIN_TIMER_NOTIFICATION = 1; // Link timer notification value

    // --- Main Loop ---
    while (true) {
        // Wait for timer notification from either timer
        // Use a timeout to ensure we don't get stuck if there's an issue with the timers
        if (xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(20)) != pdTRUE) {
            // Timeout occurred, continue the loop
            continue;
        }

        // --- START TIMING CRITICAL SECTION ---
        const auto time = g_link->clock().micros();
        const auto state = g_link->captureAppSessionState();

        bool is_main_timer_notification = (ulNotifiedValue == MAIN_TIMER_NOTIFICATION);

        // Handle regular tick processing only on main timer notifications
        if (is_main_timer_notification) {
            // Handle Link connection, metronome, MIDI clock/sync - only on normal tick timer
            // Pass link state variables by reference
            int current_ticks_local = lastTicks; // Use local copy to check if tick advanced
            handle_link_sync(was_connected, start_wait_time, force_start,
                            current_ticks_local, length, lastBeat, currentBuzzerFreq, was_playing,
                            state, time);
            
            // --- Process Controls and State Machine - only on normal tick timer ---
            // Read all inputs into the event struct
            update_input_state(current_input_event);
            
            // Process the input event through the state machine
            process_state_event(current_input_event, state, time);
            
            lastTicks = current_ticks_local; // Update the static lastTicks
        }

        // Bass engine note scheduling is handled inside process_state_event.

    } // End while(true)
} // End tickTask

// ── Dead code removed: copy_file / copy_midi_files_to_spiffs / init_spiffs ──
// (SPIFFS and MIDI file playback replaced by bass_engine.cpp)

#if 0
bool copy_file(const char* src_path, const char* dest_path) {
    FILE* src_file = fopen(src_path, "rb");
    if (!src_file) {
        ESP_LOGE(TAG, "Failed to open source file: %s (errno: %d)", src_path, errno);
        return false;
    }

    FILE* dest_file = fopen(dest_path, "wb");
    if (!dest_file) {
        ESP_LOGE(TAG, "Failed to create destination file: %s (errno: %d)", dest_path, errno);
        fclose(src_file);
        return false;
    }

    // Copy data in chunks
    const size_t BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE];
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, BUFFER_SIZE, src_file)) > 0) {
        if (fwrite(buffer, 1, bytes_read, dest_file) != bytes_read) {
            ESP_LOGE(TAG, "Error writing to file: %s", dest_path);
            fclose(src_file);
            fclose(dest_file);
            return false;
        }
    }

    fclose(src_file);
    fclose(dest_file);
    ESP_LOGI(TAG, "Copied file: %s -> %s", src_path, dest_path);
    return true;
}



// Function to copy MIDI files from data directory to SPIFFS
void copy_midi_files_to_spiffs() {
    // Source and destination directory pairs
    struct DirPair {
        const char* src_dir;
        const char* dest_dir;
    };

    const DirPair dir_pairs[] = {
        {"data/loops/notes/arp", "/spiffs/loops/notes/arp"},
        {"data/loops/notes/fil", "/spiffs/loops/notes/fil"},
        {"data/loops/notes/rev", "/spiffs/loops/notes/rev"},
        {"data/loops/notes/sid", "/spiffs/loops/notes/sid"},
        {"data/loops/chords/arp", "/spiffs/loops/chords/arp"},
        {"data/loops/chords/fil", "/spiffs/loops/chords/fil"},
        {"data/loops/chords/rev", "/spiffs/loops/chords/rev"},
        {"data/loops/chords/sid", "/spiffs/loops/chords/sid"}
    };

    // Verify the data directory exists
    struct stat st;
    if (stat("data", &st) != 0) {
        ESP_LOGE(TAG, "Data directory not found. MIDI files won't be copied. (errno: %d, %s)", 
                errno, strerror(errno));
        ESP_LOGE(TAG, "Make sure you have a data directory in the project root.");
        
        // List the root directory to debug
        DIR* dir = opendir("/");
        if (dir) {
            struct dirent* entry;
            ESP_LOGI(TAG, "Contents of root directory:");
            while ((entry = readdir(dir)) != NULL) {
                ESP_LOGI(TAG, "  %s", entry->d_name);
            }
            closedir(dir);
        }
        
        return;
    }

    // Copy files from each directory
    bool any_copied = false;
    for (const DirPair& pair : dir_pairs) {
        // Check if source directory exists
        if (stat(pair.src_dir, &st) != 0) {
            ESP_LOGW(TAG, "Source directory does not exist: %s (errno: %d, %s)", 
                    pair.src_dir, errno, strerror(errno));
            continue;
        }
        
        // Check if destination directory exists
        if (stat(pair.dest_dir, &st) != 0) {
            ESP_LOGW(TAG, "Destination directory does not exist: %s (errno: %d, %s)", 
                    pair.dest_dir, errno, strerror(errno));
            continue;
        }
        
        ESP_LOGI(TAG, "Copying files from %s to %s", pair.src_dir, pair.dest_dir);
        
        // Try to copy the files
        DIR* dir = opendir(pair.src_dir);
        if (!dir) {
            ESP_LOGE(TAG, "Failed to open source directory: %s (errno: %d, %s)", 
                    pair.src_dir, errno, strerror(errno));
            continue;
        }
        
        // Count successful copies
        int file_count = 0;
        
        // Read directory entries
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            // Skip "." and ".." entries
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }
            
            // Build full source path
            char src_path[512];
            int src_len = snprintf(src_path, sizeof(src_path), "%s/%s", pair.src_dir, entry->d_name);
            if (src_len >= sizeof(src_path)) {
                ESP_LOGE(TAG, "Source path too long: %s/%s", pair.src_dir, entry->d_name);
                continue;
            }
            
            // Check if it's a file
            if (stat(src_path, &st) == 0 && S_ISREG(st.st_mode)) {
                // Build destination path
                char dest_path[512];
                int dest_len = snprintf(dest_path, sizeof(dest_path), "%s/%s", pair.dest_dir, entry->d_name);
                if (dest_len >= sizeof(dest_path)) {
                    ESP_LOGE(TAG, "Destination path too long: %s/%s", pair.dest_dir, entry->d_name);
                    continue;
                }
                
                // Copy the file
                if (copy_file(src_path, dest_path)) {
                    file_count++;
                    any_copied = true;
                }
            }
        }
        
        closedir(dir);
        ESP_LOGI(TAG, "Copied %d files from %s to %s", file_count, pair.src_dir, pair.dest_dir);
    }
    
    if (!any_copied) {
        ESP_LOGW(TAG, "No MIDI files were copied. Arpeggiator may not work properly.");
    } else {
        ESP_LOGI(TAG, "MIDI files successfully copied to SPIFFS");
    }
}

void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",  // Use the storage partition from the partition table
        .max_files = 10,
        .format_if_mount_failed = false  // Don't format if mount fails - we want the pre-flashed data
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition info (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    
    // List directories to verify they're properly mounted
    auto list_dir = [](const char* path) {
        DIR* dir = opendir(path);
        if (!dir) {
            ESP_LOGE(TAG, "Failed to open directory: %s (errno: %d, %s)", 
                    path, errno, strerror(errno));
            return;
        }
        
        ESP_LOGI(TAG, "Directory listing for: %s", path);
        struct dirent* entry;
        int count = 0;
        while ((entry = readdir(dir)) != NULL) {
            if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
                ESP_LOGI(TAG, "  %s", entry->d_name);
                count++;
            }
        }
        
        if (count == 0) {
            ESP_LOGW(TAG, "  (empty directory)");
        }
        
        closedir(dir);
    };
    
    // List SPIFFS root
    list_dir("/spiffs");
    
    // List important directories
    const char* important_dirs[] = {
        "/spiffs/loops",
        "/spiffs/loops/notes",
        "/spiffs/loops/notes/arp",
        "/spiffs/loops/notes/fil",
        "/spiffs/loops/notes/rev",
        "/spiffs/loops/notes/sid",
        "/spiffs/loops/chords",
        "/spiffs/loops/chords/arp",
        "/spiffs/loops/chords/fil",
        "/spiffs/loops/chords/rev",
        "/spiffs/loops/chords/sid"
    };
    
    for (const char* dir : important_dirs) {
        list_dir(dir);
    }
}
#endif // 0

// --- app_main --- (Entry point)
extern "C" void app_main() {
    printf("\n\n===== APP MAIN STARTED =====\n\n");
    fflush(stdout);

    printf("[1] Setting log level...\n");
    fflush(stdout);
    esp_log_level_set("touch", ESP_LOG_VERBOSE);

    printf("[2] Initializing NVS...\n");
    fflush(stdout);
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("[3] Initializing netif...\n");
    fflush(stdout);
    ESP_ERROR_CHECK(esp_netif_init());
    printf("[4] Creating event loop...\n");
    fflush(stdout);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    printf("[5] Initializing WiFi config...\n");
    fflush(stdout);
    wifi_config_init();
    printf("[6] Attempting WiFi connect...\n");
    fflush(stdout);

    printf("[6a] Calling wifi_config_connect()...\n");
    fflush(stdout);
    esp_err_t connect_result = wifi_config_connect();
    printf("[6b] wifi_config_connect() returned: %d (ESP_OK=%d)\n", connect_result, ESP_OK);
    fflush(stdout);

    if (connect_result == ESP_OK) {
        int wait_count = 0;
        while (!wifi_is_connected() && wait_count < 40) {
            vTaskDelay(pdMS_TO_TICKS(500));
            wait_count++;
        }

        if (wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected successfully");
        } else {
            ESP_LOGW(TAG, "WiFi connection timeout, starting provisioning mode");
            goto provisioning_mode;
        }
    } else {
        goto provisioning_mode;
    }

    goto wifi_ok;

provisioning_mode:
    {
        ESP_LOGI(TAG, "Entering WiFi provisioning mode");
        wifi_start_hotspot("Link-Device", "");
        provisioning_start_server();

        while (!g_provisioning_complete) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        provisioning_stop_server();
        wifi_stop_hotspot();
        ESP_LOGI(TAG, "Waiting for WiFi driver to stabilize...");
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Attempting WiFi connection with saved credentials");
        wifi_config_connect();
        int wait_count = 0;
        while (!wifi_is_connected() && wait_count < 60) {
            vTaskDelay(pdMS_TO_TICKS(500));
            wait_count++;
        }

        if (wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected after provisioning");
        } else {
            ESP_LOGE(TAG, "WiFi connection failed after provisioning (waited 30s), continuing without WiFi");
        }
    }

wifi_ok:
    network_midi_init();

    xTaskCreate(tickTask, "tickTask", 10240, nullptr, 15, nullptr);

    ESP_LOGI(TAG, "app_main finished, tickTask running.");
}
