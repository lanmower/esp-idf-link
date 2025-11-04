#include <ableton/Link.hpp>
#include <ableton/platforms/esp32/Clock.hpp>
#include <ableton/platforms/esp32/Esp32Platform.hpp>
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_spi_flash.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include <iostream>
#include <sstream>

#define LED GPIO_NUM_2
#define PRINT_LINK_STATE false

static const char* TAG = "link_example";

static gptimer_handle_t g_example_gptimer = nullptr;

static bool IRAM_ATTR timer_group0_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    reinterpret_cast<ableton::Link*>(user_data)->clock().tick();
    return pdFALSE;
}

static void example_timer_init(ableton::Link* link, double timerPeriodS)
{
    const uint64_t timerPeriodUS = static_cast<uint64_t>(timerPeriodS * 1000000.0);
    ESP_LOGI(TAG, "Timer period: %llu us", timerPeriodUS);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_example_gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_group0_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_example_gptimer, &cbs, link));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = timerPeriodUS,
        .reload_count = 0,
        .auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm(g_example_gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(g_example_gptimer));
    ESP_ERROR_CHECK(gptimer_start(g_example_gptimer));
}

void printTask(void* userParam)
{
  auto link = static_cast<ableton::Link*>(userParam);
  const auto quantum = 4.0;

  while (true)
  {
    const auto sessionState = link->captureAppSessionState();
    const auto numPeers = link->numPeers();
    const auto time = link->clock().micros();
    const auto beats = sessionState.beatAtTime(time, quantum);
    std::cout << std::defaultfloat << "| peers: " << numPeers << " | "
              << "tempo: " << sessionState.tempo() << " | " << std::fixed
              << "beats: " << beats << " |" << std::endl;
    vTaskDelay(800 / portTICK_PERIOD_MS);
  }
}

void tickTask(void* userParam)
{
  SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(userParam);
  ableton::Link link(120.0f);
  link.enable(true);

  if (PRINT_LINK_STATE)
  {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);

  while (true)
  {
    xSemaphoreTake(handle, portMAX_DELAY);

    const auto state = link.captureAudioSessionState();
    const auto phase = state.phaseAtTime(link.clock().micros(), 1.);
    gpio_set_level(LED, fmodf(phase, 1.) < 0.1);
  }
}

extern "C" void app_main()
{
  ableton::platforms::esp32::Platform::initialize();

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  wifi_config_t sta_config = {};
  strcpy((char*)sta_config.sta.ssid, CONFIG_LINK_WIFI_SSID);
  strcpy((char*)sta_config.sta.password, CONFIG_LINK_WIFI_PASSWORD);
  sta_config.sta.bssid_set = false;

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());

  ESP_LOGI(TAG, "WiFi initialized");

  ableton::Link link(120.0);
  link.enable(true);

  example_timer_init(&link, 1.0 / CONFIG_LINK_TIMER_FREQUENCY);

  ESP_LOGI(TAG, "Link enabled");

  SemaphoreHandle_t tickSemphr = xSemaphoreCreateBinary();
  example_timer_init(&link, 1.0 / CONFIG_LINK_TIMER_FREQUENCY);

  xTaskCreate(tickTask, "tick", 8192, tickSemphr, configMAX_PRIORITIES - 1, nullptr);
}
