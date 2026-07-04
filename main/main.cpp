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
#include "esp_log.h"
#include <stdio.h>
#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"
#include "esp_mac.h"

static const char *TAG = "MAIN";

SynthInterface* g_current_synth = nullptr;
SynthType       g_synth_type    = SYNTH_MININOVA;

const uint64_t DOUBLE_TAP_TIME_MS = 300;
const uint64_t HOLD_TIME_MS       = 200;

std::unique_ptr<ableton::Link> g_link;

void tickTask(void *userParam) {
    init_uart_midi();
    init_adc();
    init_hall_sensor();
    init_touch_pads();
    setup_buzzer();
    initialize_inputs();

    TaskHandle_t current_task_handle = xTaskGetCurrentTaskHandle();
    init_link_timer(current_task_handle);

    // Brief settle so the network interface is stable before Link opens its multicast socket
    vTaskDelay(pdMS_TO_TICKS(500));
    g_link = std::make_unique<ableton::Link>(120.0);
    g_link->enable(true);
    // Share transport start/stop across peers so all Ableton Link features work
    // (tempo + beat/phase quantum already via enable(); this adds start-stop-sync).
    g_link->enableStartStopSync(true);
    link_start_tempo_listener();   // accept looper LTMP tempo-set commands

    ESP_LOGI(TAG, "Link init complete");

    bool was_connected = false;
    int64_t start_wait_time = esp_timer_get_time();
    bool force_start = false;
    static int lastTicks = 0;
    static int length = LENGTH_NORMAL;
    static int lastBeat = -1;
    static int currentBuzzerFreq = FREQ_NORMAL;
    static bool was_playing = false;
    InputEvent current_input_event;
    uint32_t ulNotifiedValue;

    while (true) {
        if (xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(20)) != pdTRUE)
            continue;

        const auto time = g_link->clock().micros();
        const auto state = g_link->captureAppSessionState();

        if (ulNotifiedValue & 1) {
            int ticks = lastTicks;
            handle_link_sync(was_connected, start_wait_time, force_start,
                             ticks, length, lastBeat, currentBuzzerFreq, was_playing,
                             state, time);
            update_input_state(current_input_event);
            process_state_event(current_input_event, state, time);
            lastTicks = ticks;
        }
    }
}

extern "C" void app_main() {
    printf("\n===== TICKER BOOT =====\n");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(wifi_config_init());

    // Stagger scan start using last MAC byte so devices don't race to both become AP.
    // Range: 0-3825ms. Devices with different MACs will scan at different times.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t scan_delay_ms = mac[5] * 15;
    if (scan_delay_ms > 0) {
        ESP_LOGI(TAG, "MAC-based scan delay: %" PRIu32 "ms", scan_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(scan_delay_ms));
    }

    ESP_LOGI(TAG, "Scanning for 'ticker' network...");
    uint8_t best_bssid[6] = {0};
    int matches = wifi_scan_best_bssid("ticker", best_bssid);

    if (matches > 0) {
        ESP_LOGI(TAG, "Found 'ticker' -- joining as STA");
        wifi_connect_sta("ticker", "");
        int wait = 0;
        while (!wifi_is_connected() && wait < 60) {
            vTaskDelay(pdMS_TO_TICKS(500));
            wait++;
        }
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "Could not join 'ticker', hosting instead");
            wifi_start_link_ap("ticker");
            wifi_start_link_relay();
        } else {
            ESP_LOGI(TAG, "Joined 'ticker' network");
            wifi_join_link_multicast();
        }
    } else {
        // No 'ticker' found. Two co-booting boards each see nothing (a single-radio
        // board mid-boot cannot reliably scan-detect a peer AP), so a naive
        // "host if none found" makes BOTH host isolated APs -> two L2 domains ->
        // Link never crosses. Elect deterministically by STA-MAC instead of by scan:
        // the numerically-lowest MAC hosts immediately; higher-MAC boards keep
        // rescanning and join the winner as soon as its AP appears. A genuinely lone
        // board (no lower peer ever shows up) still self-hosts after the window.
        //
        // The hold before we host is STRICTLY MONOTONIC in our MAC: lower MAC -> shorter
        // hold -> hosts first; higher MACs, still scanning each second, see that AP appear
        // and join it. This gives a total order with no cross-device visibility required
        // during the hold, and is confirmed by re-scanning for the winner's real AP.
        // rank from the low 3 MAC bytes (the OUI-independent, per-unit portion).
        uint32_t mac_rank = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
        // Scale rank (0 .. 2^24-1) into a bounded hold of 0 .. HOLD_MAX_MS. The lowest
        // MAC holds ~0ms (hosts almost immediately); the highest holds HOLD_MAX_MS. A
        // lone board simply hosts when its own hold expires with no peer seen.
        const uint32_t HOLD_MAX_MS = 6000;
        uint32_t hold_ms = (uint32_t)(((uint64_t)mac_rank * HOLD_MAX_MS) >> 24);
        ESP_LOGI(TAG, "No 'ticker' -- MAC-ordered host hold %" PRIu32 "ms (rank=%" PRIu32 ")",
                 hold_ms, mac_rank);
        bool joined = false;
        uint32_t waited_ms = 0;
        // Rescan every 1s during the hold; join the instant a lower peer's AP appears.
        while (waited_ms < hold_ms) {
            uint32_t step = (hold_ms - waited_ms > 1000) ? 1000 : (hold_ms - waited_ms);
            vTaskDelay(pdMS_TO_TICKS(step));
            waited_ms += step;
            uint8_t bssid2[6] = {0};
            if (wifi_scan_best_bssid("ticker", bssid2) > 0) {
                ESP_LOGI(TAG, "Peer 'ticker' appeared during hold -- joining as STA");
                wifi_connect_sta("ticker", "");
                int wait2 = 0;
                while (!wifi_is_connected() && wait2 < 60) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    wait2++;
                }
                if (wifi_is_connected()) {
                    ESP_LOGI(TAG, "Joined 'ticker' network");
                    wifi_join_link_multicast();
                    joined = true;
                    break;
                }
                ESP_LOGW(TAG, "Join attempt failed -- continuing hold");
            }
        }
        if (!joined) {
            ESP_LOGI(TAG, "Hold expired, no peer 'ticker' -- hosting AP");
            wifi_start_link_ap("ticker");
            wifi_start_link_relay();
        }
    }

    // Supervisor self-heals the mesh: reconnects a dropped STA, re-hosts if the host
    // disappears, and resolves a dual-host race (lower BSSID wins). Runs in all roles.
    wifi_start_supervisor("ticker");

    // Persistent station-side Link bridge: forwards our Link multicast to the AP
    // gateway whenever we are a STA (self-activates on IP, robust to every join path
    // including supervisor yield/reconnect). Idle while we are the host.
    wifi_start_station_bridge();

    network_midi_init();
    xTaskCreate(tickTask, "tickTask", 10240, nullptr, 15, nullptr);
    ESP_LOGI(TAG, "app_main done, tickTask running");
}
