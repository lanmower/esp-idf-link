#ifndef MAIN_H
#define MAIN_H

#include <driver/gptimer.h>

#include <string.h>
#include <math.h>
#include <memory>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/touch_pad.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include "esp_wifi.h"
#include "protocol_examples_common.h"
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_timer.h>

#include <ableton/Link.hpp>

#include "arp_constants.h"
#include "synth_interface.h"
#include "types.h"

#define BUZZER GPIO_NUM_13
#define LINK_TICK_PERIOD 250
#define NUM_POTS 2
#define NUM_TOUCH_PADS 4

#define MIDI_UART UART_NUM_2
#define MIDI_TX_PIN GPIO_NUM_17
#define MIDI_RX_PIN GPIO_NUM_16

#define TOUCH_PAD_1    TOUCH_PAD_NUM0
#define TOUCH_PAD_ARP  TOUCH_PAD_NUM5
#define TOUCH_PAD_REV  TOUCH_PAD_NUM6
#define TOUCH_PAD_FILT TOUCH_PAD_NUM7
#define TOUCH_THRESHOLD 700

#define POT_ADC_CHANNEL_1 ADC_CHANNEL_6
#define POT_ADC_CHANNEL_2 ADC_CHANNEL_0
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_BITWIDTH_DEFAULT
#define MIDI_CC_THRESHOLD 1

#define LINK_QUANTUM 16.0
#define PHRASE_BEATS 64.0

#define LEDC_MODE              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES         LEDC_TIMER_10_BIT
#define LEDC_DUTY             (512)
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_CHANNEL          LEDC_CHANNEL_0
#define LEDC_OUTPUT_IO        BUZZER
#define FREQ_16BEAT            2093u
#define FREQ_8BEAT             1568u
#define FREQ_4BEAT             1319u
#define FREQ_NORMAL            1047u
#define LENGTH_NORMAL          1
#define LENGTH_16BEAT          20
#define LENGTH_8BEAT           10
#define LENGTH_4BEAT           5

#define MIDI_TIMING_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_CONTINUE 0xFB
#define MIDI_SONG_POSITION_POINTER 0xF2
#define MIDI_NOTE_ON_CMD 0x90
#define MIDI_NOTE_OFF_CMD 0x80
#define MIDI_CC_CMD 0xB0
#define MIDI_CC_ALL_NOTES_OFF 123

#define MIDI_CLOCK_RESYNC_THRESHOLD 24

enum SynthType { SYNTH_MININOVA, SYNTH_MICROKORG };
extern SynthType g_synth_type;

extern std::unique_ptr<ableton::Link> g_link;

extern const uint64_t DOUBLE_TAP_TIME_MS;
extern const uint64_t HOLD_TIME_MS;

extern SynthInterface* g_current_synth;

#endif // MAIN_H
