#ifndef MAIN_H
#define MAIN_H

#include <driver/gptimer.h> // Explicitly include for gptimer functions

// --- Standard Includes ---
#include <string.h> // Added for memcpy
#include <math.h>   // Added for fmod
#include <memory>   // For std::unique_ptr

// --- ESP-IDF Includes ---
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/touch_pad.h> // Use legacy touch API instead
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
#include <esp_timer.h> // For timing functions

// --- Third-Party Includes ---
#include <ableton/Link.hpp>

// --- Project Includes ---
// (Will add headers for other modules later)
#include "arp_constants.h" // For Arp data structures
#include "synth_interface.h" // Include the new synth interface
#include "types.h" // Include the new types header

// --- Global Constants ---
#define BUZZER GPIO_NUM_13 // Define the pin for the buzzer
// #define LINK_TICK_PERIOD 100 // Original: 100us period (10 kHz) for Link Timer
// #define LINK_TICK_PERIOD 10000 // New Value: 10ms period (100 Hz) for Link Timer - to reduce system load
// #define LINK_TICK_PERIOD 130 // 3840hz  (80x 96hz)
#define LINK_TICK_PERIOD 250 // 4000hz - tighter beat detection, halves worst-case jitter
#define NUM_POTS 2         // Number of potentiometers
#define NUM_TOUCH_PADS 4 // Number of touch pads

// MIDI UART
#define MIDI_UART UART_NUM_2  // Hardware MIDI UART
#define MIDI_TX_PIN GPIO_NUM_17
#define MIDI_RX_PIN GPIO_NUM_16

// Touch Pads (Using touch_pad_t from legacy touch API)
#define TOUCH_PAD_1    TOUCH_PAD_NUM0 // GPIO04 (Channel 0)
#define TOUCH_PAD_ARP  TOUCH_PAD_NUM5 // GPIO12 (Channel 5)
#define TOUCH_PAD_REV  TOUCH_PAD_NUM6 // GPIO14 (Channel 6)
#define TOUCH_PAD_FILT TOUCH_PAD_NUM7 // GPIO27 (Channel 7)
#define TOUCH_THRESHOLD 700 // Increased threshold for better sensitivity

// Potentiometers
#define POT_ADC_CHANNEL_1 ADC_CHANNEL_6 // GPIO34
#define POT_ADC_CHANNEL_2 ADC_CHANNEL_0 // GPIO39
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_BITWIDTH_DEFAULT
#define MIDI_CC_THRESHOLD 1

// Link Constants
// LINK_QUANTUM is the musical phase quantum (16 beats = 4 bars of 4/4) used for
// beat/phase tracking and the metronome buzzer.
#define LINK_QUANTUM 16.0
// PHRASE_BEATS is the transport-correction boundary (16 bars = 64 beats). External
// gear is realigned (SPP / Start) only at this boundary, NOT every quantum, so that
// brand-varied targets (KO2, Volca Drum, MicroKorg, Micron, MiniNova, RC-505 MK2)
// stay in phrase without transport spam disrupting their normal operation.
#define PHRASE_BEATS 64.0

// Metronome Constants
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

// MIDI Constants
#define MIDI_TIMING_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_CONTINUE 0xFB
#define MIDI_SPP 0xF2          // Song Position Pointer (status + LSB + MSB)
#define MIDI_NOTE_ON_CMD 0x90
#define MIDI_NOTE_OFF_CMD 0x80
#define MIDI_CC_CMD 0xB0
#define MIDI_CC_ALL_NOTES_OFF 123  // CC123: All Notes Off (sent on all 16 channels at stop)

// Maximum MIDI timing clocks emitted in a single tick. At LINK_TICK_PERIOD=250us
// (4 kHz) and 24ppqn, even 300 BPM needs <<1 clock/tick, so a healthy tick emits 0-1.
// Capping catch-up prevents a post-stall burst of 0xF8 bytes that external gear reads
// as a tempo spike; beyond the cap we resync the counter to the live beat instead.
#define MIDI_MAX_CLOCKS_PER_TICK 2
// If the clock counter falls further than this many clocks behind the live Link beat
// (e.g. after a WiFi/scan stall), abandon catch-up and hard-resync to the current
// position rather than flooding the bus. 24 clocks = one quarter note.
#define MIDI_CLOCK_RESYNC_THRESHOLD 24

// --- Enums ---
// Moved to synth_mininova.cpp

// --- Extern Global Variables ---
// Declared here, defined in main.cpp or specific effect files

// Link Object
extern std::unique_ptr<ableton::Link> g_link;

// Add a variable to track which synth is active
enum SynthType { SYNTH_MININOVA, SYNTH_MICROKORG };
extern SynthType g_synth_type; // Declared extern

// Double Tap Timing
extern const uint64_t DOUBLE_TAP_TIME_MS; // 300
extern const uint64_t HOLD_TIME_MS; // 200

// Synth Interface Pointer
extern SynthInterface* g_current_synth;

#endif // MAIN_H