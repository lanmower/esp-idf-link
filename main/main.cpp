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
        ESP_LOGI(TAG, "No 'ticker' found -- hosting AP");
        wifi_start_link_ap("ticker");
        wifi_start_link_relay();
    }

    // Supervisor self-heals the mesh: reconnects a dropped STA, re-hosts if the host
    // disappears, and resolves a dual-host race (lower BSSID wins). Runs in all roles.
    wifi_start_supervisor("ticker");

    network_midi_init();
    xTaskCreate(tickTask, "tickTask", 10240, nullptr, 15, nullptr);
    ESP_LOGI(TAG, "app_main done, tickTask running");
}
