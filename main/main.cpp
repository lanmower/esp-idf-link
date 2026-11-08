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
#include <stdarg.h>
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

static constexpr size_t kLogRingBytes = 8192;
static char s_log_ring[kLogRingBytes];
static size_t s_log_head = 0;
static size_t s_log_tail = 0;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_log_drain_task = nullptr;

static int async_log_vprintf(const char* format, va_list args) {
    char line[320];
    const int written = vsnprintf(line, sizeof line, format, args);
    if (written <= 0) {
        return written;
    }
    const size_t length = static_cast<size_t>(written) < sizeof line
                              ? static_cast<size_t>(written) : sizeof line - 1;
    portENTER_CRITICAL(&s_log_mux);
    for (size_t i = 0; i < length; ++i) {
        const size_t next = (s_log_head + 1) % kLogRingBytes;
        if (next == s_log_tail) {
            break;
        }
        s_log_ring[s_log_head] = line[i];
        s_log_head = next;
    }
    portEXIT_CRITICAL(&s_log_mux);
    if (s_log_drain_task != nullptr) {
        xTaskNotifyGive(s_log_drain_task);
    }
    return written;
}

static void log_drain_task_run(void*) {
    char chunk[256];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        for (;;) {
            size_t count = 0;
            portENTER_CRITICAL(&s_log_mux);
            while (s_log_tail != s_log_head && count < sizeof chunk) {
                chunk[count++] = s_log_ring[s_log_tail];
                s_log_tail = (s_log_tail + 1) % kLogRingBytes;
            }
            portEXIT_CRITICAL(&s_log_mux);
            if (count == 0) {
                break;
            }
            fwrite(chunk, 1, count, stdout);
            fflush(stdout);
        }
    }
}

static void log_async_init() {
    xTaskCreate(log_drain_task_run, "log_drain", 3072, nullptr, 2, &s_log_drain_task);
    esp_log_set_vprintf(async_log_vprintf);
}

void tickTask(void *userParam) {
    init_uart_midi();
    link_sync_send_boot_panic();
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

    const int64_t link_init_us = esp_timer_get_time();
    auto bounce_link_session = [&](const char* why) {
        g_link->enable(false);
        g_link->enable(true);
        g_link->enableStartStopSync(false);
        ESP_LOGW(TAG, "Link session bounce (%s) at %lld ms", why,
                 (long long)((esp_timer_get_time() - link_init_us) / 1000));
    };

    bool was_connected = false;
    int64_t start_wait_time = esp_timer_get_time();
    bool force_start = false;
    static bool was_playing = false;
    InputEvent current_input_event;
    uint32_t ulNotifiedValue;
    static uint32_t s_input_tick = 0;
    static constexpr uint32_t INPUT_POLL_TICKS = 8;
    static int64_t s_last_wake_us = 0;

    while (true) {
        if (xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(20)) != pdTRUE)
            continue;

        const int64_t wakeUs = esp_timer_get_time();
        const int64_t wakeGapUs = wakeUs - s_last_wake_us;
        s_last_wake_us = wakeUs;
        link_service_clock(wakeUs);
        const int64_t serviceUs = esp_timer_get_time() - wakeUs;
        midi_bulk_pump();
        if (link_consume_session_reset())
            bounce_link_session("LRST");

        const int64_t captureT0 = esp_timer_get_time();
        const auto time = g_link->clock().micros();
        const auto state = g_link->captureAppSessionState();
        const int64_t captureUs = esp_timer_get_time() - captureT0;

        int64_t syncUs = 0;
        int64_t inputUs = 0;
        int64_t processUs = 0;
        if (ulNotifiedValue & LINK_TICK_NOTIFY_BIT) {
            const int64_t syncT0 = esp_timer_get_time();
            handle_link_sync(was_connected, start_wait_time, force_start,
                             was_playing, state, time);
            syncUs = esp_timer_get_time() - syncT0;
            if (s_input_tick++ % INPUT_POLL_TICKS == 0) {
                const int64_t inputT0 = esp_timer_get_time();
                update_input_state(current_input_event);
                inputUs = esp_timer_get_time() - inputT0;
            } else {
                current_input_event.timestamp_us = esp_timer_get_time();
                for (int i = 0; i < NUM_POTS; ++i) current_input_event.pot_moved[i] = false;
            }
            const int64_t processT0 = esp_timer_get_time();
            process_state_event(current_input_event, state, time);
            processUs = esp_timer_get_time() - processT0;
        }
        const int64_t totalUs = esp_timer_get_time() - wakeUs;
        if (totalUs > 20000 || wakeGapUs > 20000) {
            ESP_LOGW("TICK", "wake_gap %lld service %lld capture %lld sync %lld input %lld process %lld total %lld us",
                     (long long)wakeGapUs, (long long)serviceUs, (long long)captureUs, (long long)syncUs,
                     (long long)inputUs, (long long)processUs, (long long)totalUs);
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
    log_async_init();
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
    xTaskCreatePinnedToCore(tickTask, "tickTask", 10240, nullptr, 24, nullptr, 1);
    ESP_LOGI(TAG, "app_main done, tickTask running");
}
