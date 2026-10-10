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

    const int NETIF_SETTLE_BEFORE_LINK_SOCKET_MS = 500;
    vTaskDelay(pdMS_TO_TICKS(NETIF_SETTLE_BEFORE_LINK_SOCKET_MS));
    g_link = std::make_unique<ableton::Link>(120.0);
    g_link->enable(true);
    g_link->enableStartStopSync(false);
    link_start_tempo_listener();

    ESP_LOGI(TAG, "Link init complete");

    bool was_connected = false;
    int64_t start_wait_time = esp_timer_get_time();
    bool force_start = false;
    static bool was_playing = false;
    InputEvent current_input_event;
    uint32_t ulNotifiedValue;

    while (true) {
        if (xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(20)) != pdTRUE)
            continue;

        const auto time = g_link->clock().micros();
        const auto state = g_link->captureAppSessionState();

        if (ulNotifiedValue & LINK_TICK_NOTIFY_BIT) {
            handle_link_sync(was_connected, start_wait_time, force_start,
                             was_playing, state, time);
            update_input_state(current_input_event);
            process_state_event(current_input_event, state, time);
        }
    }
}

static uint32_t sta_mac_rank(const uint8_t mac[kMacLen]) {
    return ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
}

static uint32_t mac_ordered_host_hold_ms(uint32_t mac_rank) {
    const uint32_t HOLD_MAX_MS = 6000;
    return (uint32_t)(((uint64_t)mac_rank * HOLD_MAX_MS) >> 24);
}

static bool join_ticker_during_hold(uint32_t hold_ms) {
    const uint32_t RESCAN_STEP_MS = 1000;
    const int JOIN_WAIT_TRIES = 60;
    uint32_t waited_ms = 0;
    while (waited_ms < hold_ms) {
        uint32_t step = (hold_ms - waited_ms > RESCAN_STEP_MS) ? RESCAN_STEP_MS : (hold_ms - waited_ms);
        vTaskDelay(pdMS_TO_TICKS(step));
        waited_ms += step;
        uint8_t bssid[kMacLen] = {0};
        if (wifi_scan_best_bssid("ticker", bssid) > 0) {
            ESP_LOGI(TAG, "Peer 'ticker' appeared during hold -- joining as STA");
            wifi_connect_sta("ticker", "");
            int wait = 0;
            while (!wifi_is_connected() && wait < JOIN_WAIT_TRIES) {
                vTaskDelay(pdMS_TO_TICKS(500));
                wait++;
            }
            if (wifi_is_connected()) {
                ESP_LOGI(TAG, "Joined 'ticker' network");
                wifi_join_link_multicast();
                return true;
            }
            ESP_LOGW(TAG, "Join attempt failed -- continuing hold");
        }
    }
    return false;
}

extern "C" void app_main() {
    printf("\n===== TICKER BOOT =====\n");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(wifi_config_init());

    const uint32_t SCAN_STAGGER_MS_PER_MAC_BYTE = 15;
    uint8_t mac[kMacLen];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t scan_delay_ms = mac[5] * SCAN_STAGGER_MS_PER_MAC_BYTE;
    if (scan_delay_ms > 0) {
        ESP_LOGI(TAG, "MAC-based scan delay: %" PRIu32 "ms", scan_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(scan_delay_ms));
    }

    ESP_LOGI(TAG, "Scanning for 'ticker' network...");
    uint8_t best_bssid[kMacLen] = {0};
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
        const uint32_t mac_rank = sta_mac_rank(mac);
        const uint32_t hold_ms = mac_ordered_host_hold_ms(mac_rank);
        ESP_LOGI(TAG, "No 'ticker' -- MAC-ordered host hold %" PRIu32 "ms (rank=%" PRIu32 ")",
                 hold_ms, mac_rank);
        if (!join_ticker_during_hold(hold_ms)) {
            ESP_LOGI(TAG, "Hold expired, no peer 'ticker' -- hosting AP");
            wifi_start_link_ap("ticker");
            wifi_start_link_relay();
        }
    }

    wifi_start_supervisor("ticker");

    network_midi_init();
    xTaskCreate(tickTask, "tickTask", 10240, nullptr, 15, nullptr);
    ESP_LOGI(TAG, "app_main done, tickTask running");
}
