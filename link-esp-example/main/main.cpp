//idf 4.4.4
#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include "esp_wifi.h"
#include "protocol_examples_common.h"

#include <ableton/Link.hpp>

#define LED GPIO_NUM_4  // Ensure this is the correct pin for the R32 D1
#define BUZZER GPIO_NUM_2 // Define the pin for the buzzer
#define PRINT_LINK_STATE false

#define USB_UART UART_NUM_0   // USB UART
#define MIDI_UART UART_NUM_2  // Hardware MIDI UART
#define USB_TX_PIN GPIO_NUM_1  // TXD0
#define USB_RX_PIN GPIO_NUM_3  // RXD0
#define MIDI_TX_PIN GPIO_NUM_17
#define MIDI_RX_PIN GPIO_NUM_16
#define USB_MIDI true
#define LINK_TICK_PERIOD 100

// PWM configuration for passive buzzer
#define LEDC_MODE              LEDC_HIGH_SPEED_MODE  // Use high speed mode for better frequency accuracy
#define LEDC_DUTY_RES         LEDC_TIMER_10_BIT  // Set duty resolution to 10 bits
#define LEDC_DUTY             (512)              // 50% duty cycle (512 out of 1024)
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_CHANNEL          LEDC_CHANNEL_0
#define LEDC_OUTPUT_IO        BUZZER             // Define buzzer GPIO

// Different frequencies for different beat positions (in Hz)
#define FREQ_16BEAT            2093u  // C7 note
#define FREQ_8BEAT             1568u  // G6 note
#define FREQ_4BEAT             1319u  // E6 note
#define FREQ_NORMAL            1047u  // C6 note

// Different lengths for different beat positions (in ticks)
#define LENGTH_NORMAL          1    // Short beep for regular beats
#define LENGTH_16BEAT          20    // Longer beep for measure start
#define LENGTH_8BEAT           10    // Medium-long beep for half measure
#define LENGTH_4BEAT           5    // Medium beep for quarter measure

#define MIDI_TIMING_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_CONTINUE 0xFB

static const char *TAG = "LINK_APP";

void IRAM_ATTR timer_isr(void *userParam) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

  vTaskNotifyGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
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
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, userParam, 
                    ESP_INTR_FLAG_IRAM,  // Keep ISR in IRAM for consistent timing
                    nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void initUartPort(uart_port_t port, int txPin, int rxPin) {
  uart_config_t uart_config = {
    .baud_rate = 31250,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB
  };

  uart_param_config(port, &uart_config);
  uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(port, 512, 0, 0, NULL, 0);
}

void setupBuzzer() {
    // Configure LEDC timer for buzzer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = FREQ_NORMAL,  // Start with normal frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}  // Initialize flags to 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void setBuzzerState(bool on, uint32_t frequency = FREQ_NORMAL) {
    static uint32_t lastFreq = FREQ_NORMAL;
    
    if (on) {
        // Only update frequency if it changed
        if (frequency != lastFreq) {
            // Configure timer for passive buzzer
            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_MODE,
                .duty_resolution = LEDC_DUTY_RES,
                .timer_num = LEDC_TIMER,
                .freq_hz = frequency,
                .clk_cfg = LEDC_AUTO_CLK
            };
            ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
            lastFreq = frequency;
        }
        
        // Set 50% duty cycle for passive buzzer to produce sound
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        // Turn off by setting duty to 0
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}

void tickTask(void *userParam) {
  ableton::Link link(120.0f);
  link.enable(true);

  initUartPort(USB_UART, USB_TX_PIN, USB_RX_PIN);
  initUartPort(MIDI_UART, MIDI_TX_PIN, MIDI_RX_PIN);

  timerGroup0Init(LINK_TICK_PERIOD, xTaskGetCurrentTaskHandle());

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
  
  // Setup buzzer PWM
  setupBuzzer();

  bool was_connected = false;
  int64_t start_wait_time = esp_timer_get_time();
  bool force_start = false;

  int currentFreq = FREQ_NORMAL;

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Check peer status
    bool is_connected = link.numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= 3000000)) {  // 60 seconds in microseconds
      force_start = true;
    }
    
    if (is_connected != was_connected) {
      if (is_connected) {
        // Send MIDI reset sequence when connection is established
        const uint8_t stop_msg[] = {MIDI_STOP};
        uart_write_bytes(MIDI_UART, (const char *)stop_msg, 1);
        uart_wait_tx_done(MIDI_UART, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay
        
        const uint8_t start_msg[] = {MIDI_START};
        uart_write_bytes(MIDI_UART, (const char *)start_msg, 1);
        uart_wait_tx_done(MIDI_UART, portMAX_DELAY);
      }
      was_connected = is_connected;
    }

    const auto state = link.captureAppSessionState();
    const auto quantum = 16.0;
    const auto time = link.clock().micros();
    
    // Calculate both regular and adjusted phases
    const auto phase = state.phaseAtTime(time, quantum);
    
    // Calculate ticks with consistent timing
    const int mticks = std::floor(phase * 150);
    const int ticks = std::floor(state.beatAtTime(time, quantum) * 24);
    
    static int lastMTicks = 0;
    static int lastTicks = 0;
    static int length = LENGTH_NORMAL;
    static int lastBeat = -1;
    static int currentFreq = FREQ_NORMAL;
    
    // Use regular phase for LED and buzzer timing
    const int currentBeat = static_cast<int>(std::floor(phase));
    const int beatInQuantum = currentBeat % static_cast<int>(quantum);
    const double beatFraction = phase - std::floor(phase);
    const int ticksInBeat = static_cast<int>(beatFraction * 150);
    
    bool crossedBeat = (currentBeat != lastBeat);
    
    if (is_connected || force_start) {
      if (crossedBeat) {
        // Update beat characteristics based on position in quantum
        if (beatInQuantum == 0) {  // First beat of quantum (measure start)
          length = LENGTH_16BEAT;
          currentFreq = FREQ_16BEAT;
        } else if (beatInQuantum == 8) {  // Middle of measure (8th beat)
          length = LENGTH_8BEAT;
          currentFreq = FREQ_8BEAT;
        } else if (beatInQuantum == 4 || beatInQuantum == 12) {  // Quarter points
          length = LENGTH_4BEAT;
          currentFreq = FREQ_4BEAT;
        } else {  // Regular beats
          length = LENGTH_NORMAL;
          currentFreq = FREQ_NORMAL;
        }
        lastBeat = currentBeat;
      }
      
      // Determine if we should be playing based on position within beat
      bool shouldPlay = ticksInBeat < length;
      
      // Set LED and buzzer states
      gpio_set_level(LED, shouldPlay);
      setBuzzerState(shouldPlay, currentFreq);
      
      static bool was_playing = false;
      bool is_playing = state.isPlaying();
      
      // Send MIDI Stop/Start when play state changes
      if (was_playing != is_playing) {
        if (!is_playing) {
          // When stopping, send Stop
          const uint8_t stop_msg = MIDI_STOP;
          uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
          uart_wait_tx_done(MIDI_UART, 1);
          uart_write_bytes(USB_UART, (const char *)&stop_msg, 1);
          uart_wait_tx_done(USB_UART, 1);
        } else {
          // When starting, send Stop then Start to ensure phase alignment
          const uint8_t stop_msg = MIDI_STOP;
          uart_write_bytes(MIDI_UART, (const char *)&stop_msg, 1);
          uart_wait_tx_done(MIDI_UART, 1);
          
          const uint8_t start_msg = MIDI_START;
          uart_write_bytes(MIDI_UART, (const char *)&start_msg, 1);
          uart_wait_tx_done(MIDI_UART, 1);

          // Handle USB separately
          uart_write_bytes(USB_UART, (const char *)&stop_msg, 1);
          uart_wait_tx_done(USB_UART, 1);
          uart_write_bytes(USB_UART, (const char *)&start_msg, 1);
          uart_wait_tx_done(USB_UART, 1);
        }
        was_playing = is_playing;
      }
      
      // Check for tick transitions
      if (ticks > lastTicks) {
        // For MIDI UART, send timing clock messages
        const uint8_t timing_msg = MIDI_TIMING_CLOCK;
        uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);
        uart_wait_tx_done(MIDI_UART, 1);

        // Reset phase at measure boundaries - send Start earlier with timing reinforcement
        if (beatInQuantum == 15) {
          if (ticksInBeat >= 120) {  // Send Start even earlier (was 130)
            const uint8_t start_msg = MIDI_START;
            uart_write_bytes(MIDI_UART, (const char *)&start_msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);
            
            // Send timing clock immediately after Start
            uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);
          } else if (ticksInBeat >= 140) {  // Send extra timing clock closer to boundary
            uart_write_bytes(MIDI_UART, (const char *)&timing_msg, 1);
            uart_wait_tx_done(MIDI_UART, 1);
          }
        }

        // Handle USB UART timing separately (keep existing USB code with preemptive timing)
        if ((mticks % 2400) == 2350) {
            const uint8_t stop_msg[] = {MIDI_STOP};
            uart_write_bytes(USB_UART, (const char *)stop_msg, sizeof(stop_msg));
            uart_wait_tx_done(USB_UART, 1);

            const uint8_t spp_zero[] = {0xF2, 0x00, 0x00};
            uart_write_bytes(USB_UART, (const char *)spp_zero, sizeof(spp_zero));
            uart_wait_tx_done(USB_UART, 1);

            for(int i = 0; i < 4; i++) {
                uart_write_bytes(USB_UART, (const char *)&timing_msg, 1);
                uart_wait_tx_done(USB_UART, 1);
            }

            const uint8_t start_msg[] = {MIDI_START};
            uart_write_bytes(USB_UART, (const char *)start_msg, sizeof(start_msg));
            uart_wait_tx_done(USB_UART, 1);
        }

        // Keep SPP messages for USB UART only
        if (mticks % 150 == 0) {
            uint16_t pos = (mticks / 6) % 32767;
            uint8_t pos_lsb = pos & 0x7F;
            uint8_t pos_msb = (pos >> 7) & 0x7F;
            
            const uint8_t spp_msg[] = {0xF2, pos_lsb, pos_msb};
            uart_write_bytes(USB_UART, (const char *)spp_msg, sizeof(spp_msg));
            uart_wait_tx_done(USB_UART, 1);
        }

        // Regular timing clock messages for USB UART (reuse timing_msg)
        uart_write_bytes(USB_UART, (const char *)&timing_msg, 1);
        uart_wait_tx_done(USB_UART, 1);
      }
    } else {
      gpio_set_level(LED, 0);
      setBuzzerState(false);
    }
    lastMTicks = mticks;
    lastTicks = ticks;
  }
}

extern "C" void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  xTaskCreate(tickTask, "ticks", 8192, nullptr, 10, nullptr);
}
