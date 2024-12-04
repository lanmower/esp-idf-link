#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include <esp_log.h>

#include <ableton/Link.hpp>

#include "esp_wifi.h"

#define LED GPIO_NUM_2  // Ensure this is the correct pin for the R32 D1
#define BUZZER GPIO_NUM_4 // Define the pin for the buzzer
#define PRINT_LINK_STATE false

#define USB_UART UART_NUM_0   // USB UART
#define MIDI_UART UART_NUM_2  // Hardware MIDI UART
#define USB_TX_PIN GPIO_NUM_1  // TXD0
#define USB_RX_PIN GPIO_NUM_3  // RXD0
#define MIDI_TX_PIN GPIO_NUM_17
#define MIDI_RX_PIN GPIO_NUM_16
#define USB_MIDI true
#define LINK_TICK_PERIOD 100

static const char *TAG = "LINK_APP";

void IRAM_ATTR timer_group0_isr(void *userParam) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

  vTaskNotifyGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void printTask(void *userParam) {
  while (true) {
    auto link = static_cast<ableton::Link *>(userParam);
    const auto quantum = 4.0;
    const auto sessionState = link->captureAppSessionState();
    const auto numPeers = link->numPeers();
    const auto time = link->clock().micros();
    const auto beats = sessionState.beatAtTime(time, quantum);
    std::cout << std::defaultfloat << "| peers: " << numPeers << " | "
              << "tempo: " << sessionState.tempo() << " | " << std::fixed
              << "beats: " << beats << " |" << std::endl;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void timerGroup0Init(int timerPeriodUS, void *userParam) {
  timer_config_t config = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80  // A more stable value that still provides good precision
  };

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timerPeriodUS);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_group0_isr, userParam, 
                    ESP_INTR_FLAG_IRAM,  // Keep ISR in IRAM for consistent timing
                    nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void initUartPort(uart_port_t port, int txPin, int rxPin) {
  uart_config_t uart_config = {
    .baud_rate = (port == USB_UART) ? 256000 : 31250,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
  };

  uart_param_config(port, &uart_config);
  uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(port, 512, 0, 0, NULL, 0);
}

void tickTask(void *userParam) {
  ESP_LOGI(TAG, "Initializing Ableton Link...");
  ableton::Link link(120.0f);
  link.enable(true);

  ESP_LOGI(TAG, "Setting up UART...");
  initUartPort(USB_UART, USB_TX_PIN, USB_RX_PIN);
  initUartPort(MIDI_UART, MIDI_TX_PIN, MIDI_RX_PIN);

  ESP_LOGI(TAG, "Initializing timer...");
  timerGroup0Init(LINK_TICK_PERIOD, xTaskGetCurrentTaskHandle());

  if (PRINT_LINK_STATE) {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

  ESP_LOGI(TAG, "Waiting for Link peers...");
  bool was_connected = false;
  int64_t start_wait_time = esp_timer_get_time();
  bool force_start = false;

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Check peer status
    bool is_connected = link.numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= 60000000)) {  // 60 seconds in microseconds
      ESP_LOGI(TAG, "No peers found after 1 minute, starting anyway");
      force_start = true;
    }
    
    if (is_connected != was_connected) {
      if (is_connected) {
        ESP_LOGI(TAG, "Link peer connected!");
      } else {
        ESP_LOGI(TAG, "Link peer disconnected");
      }
      was_connected = is_connected;
    }

    const auto state = link.captureAudioSessionState();
    const int beats = std::floor(state.beatAtTime(link.clock().micros(), 1.));
    const int mticks = std::floor(state.beatAtTime(link.clock().micros(), 1.) * 2400);
    const int ticks = std::floor(state.beatAtTime(link.clock().micros(), 1.) * 24);
    static int lastTicks;
    const auto phase = state.phaseAtTime(link.clock().micros(), 64.);
    static int length = 1;
    
    if (is_connected || force_start) {
      length = (fmodf(phase, 4.) <= 0.1) ? 20 : length;
      length = (fmodf(phase, 8.) <= 0.1) ? 50 : length;
      length = (fmodf(phase, 16.) <= 0.1) ? 80 : length;
      length = (fmodf(phase, 32.) <= 0.1) ? 120 : length;
      length = (fmodf(phase, 4.) > 0.1) ? 2 : length;
      gpio_set_level(LED, (mticks % 2400) < length);
      gpio_set_level(BUZZER, (mticks % 2400) < length);
      
      static bool was_playing = false;
      bool is_playing = state.isPlaying();
      
      // Send MIDI Stop/Continue when play state changes
      if (was_playing != is_playing) {
        uint8_t status = is_playing ? 0xFB : 0xFC; // 0xFB = Continue, 0xFC = Stop
        uint8_t data[1] = {status};
        uart_write_bytes(USB_UART, (char *)data, 1);
        uart_write_bytes(MIDI_UART, (char *)data, 1);
        was_playing = is_playing;
      }
      
      if (ticks > lastTicks) {
        // Only send start and position messages exactly at the phase boundary
        if(mticks % 24 == 0 && phase < 0.1 && fmodf(phase, 16.) <= 0.1) {
          uint8_t data[1] = {0xfa};
          uart_write_bytes(USB_UART, (char *)data, 1);
          uart_write_bytes(MIDI_UART, (char *)data, 1);

          uint8_t sppData[3];
          sppData[0] = 0xF2;
          int positionInBeats = 0;
          sppData[1] = (positionInBeats & 0x7F);
          sppData[2] = (positionInBeats >> 7) & 0x7F;
          uart_write_bytes(USB_UART, (char *)sppData, 3);
          uart_write_bytes(MIDI_UART, (char *)sppData, 3);
        }
        
        // Ensure we only send timing clock if we haven't just sent a start message
        else if (phase >= 0.1 || fmodf(phase, 16.) > 0.1) {
          #ifdef USB_MIDI
            uint8_t data[4] = {0x0f, 0xf8, 0x00, 0x00};
            uart_write_bytes(USB_UART, (char *)data, 4);
            uart_write_bytes(MIDI_UART, (char *)data, 4);
          #else
            uint8_t data[1] = {0xf8};
            uart_write_bytes(USB_UART, (char *)data, 1);
            uart_write_bytes(MIDI_UART, (char *)data, 1);
          #endif
        }
      }
    } else {
      gpio_set_level(LED, 0);
      gpio_set_level(BUZZER, 0);
    }
    lastTicks = ticks;
  }
}

void midiForwardTask(void *userParam) {
  const size_t BUF_SIZE = 128;
  uint8_t data[BUF_SIZE];
  
  ESP_LOGI(TAG, "Starting MIDI forwarding task");
  
  while (true) {
    // Read data from MIDI UART
    int len = uart_read_bytes(MIDI_UART, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
      ESP_LOGD(TAG, "MIDI->USB: Forwarding %d bytes", len);
      uart_write_bytes(USB_UART, (const char*)data, len);
    }
    
    // Read data from USB UART
    len = uart_read_bytes(USB_UART, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
      ESP_LOGD(TAG, "USB->MIDI: Forwarding %d bytes", len);
      uart_write_bytes(MIDI_UART, (const char*)data, len);
    }
    
    // Small delay to prevent tight loop
    vTaskDelay(1);
  }
}

extern "C" void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  xTaskCreate(tickTask, "ticks", 8192, nullptr, 10, nullptr);
  xTaskCreate(midiForwardTask, "midi_fwd", 2048, nullptr, 5, nullptr);
}
