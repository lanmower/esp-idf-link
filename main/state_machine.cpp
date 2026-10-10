#include "state_machine.h"
#include "bass_engine.h"
#include "esp_log.h"

static const char* TAG = "SM";

static const uint64_t NUDGE_HOLD_US = 500000;
static const uint64_t DOUBLE_TAP_WINDOW_US = 300000;

static_assert(NUM_TOUCH_PADS <= bli::BANK_COUNT);

static const int APPROACH_FOR_PAD[NUM_TOUCH_PADS][2] = {
    {BassEngine::APP_ROLL,  BassEngine::APP_ACID},
    {BassEngine::APP_FUNK,  BassEngine::APP_GLITCH},
    {BassEngine::APP_STAB,  BassEngine::APP_DRIFT},
    {BassEngine::APP_PEDAL, BassEngine::APP_BREAK},
};

static uint64_t s_press_start_us[NUM_TOUCH_PADS] = {0, 0, 0, 0};
static uint64_t s_tap_stamp_us[NUM_TOUCH_PADS] = {0, 0, 0, 0};
static bool s_tap_pending[NUM_TOUCH_PADS] = {false, false, false, false};
static bool s_nudge_fired[NUM_TOUCH_PADS] = {false, false, false, false};
static bool s_was_held[NUM_TOUCH_PADS] = {false, false, false, false};

void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time)
{
    if (event.pot_moved[0])
        g_bassEngine.setDial(0, static_cast<float>(event.pot_value[0]) / MIDI_VALUE_MAX);
    if (event.pot_moved[1])
        g_bassEngine.setDial(1, static_cast<float>(event.pot_value[1]) / MIDI_VALUE_MAX);

    static bool s_all4_latched = false;
    bool all4 = event.pad_held[0] && event.pad_held[1]
             && event.pad_held[2] && event.pad_held[3];
    if (all4) {
        if (!s_all4_latched) {
            s_all4_latched = true;
            ESP_LOGI(TAG, "All 4 pads held -> stop playback");
            g_bassEngine.stop();
        }
        for (int i = 0; i < NUM_TOUCH_PADS; i++) {
            s_was_held[i]       = event.pad_held[i];
            s_press_start_us[i] = 0;
            s_nudge_fired[i]    = false;
            s_tap_pending[i]    = false;
        }
        g_bassEngine.process(link_state, link_time);
        return;
    }
    s_all4_latched = false;

    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        bool held     = event.pad_held[i];
        bool pressed  = held && !s_was_held[i];
        bool released = !held && s_was_held[i];
        s_was_held[i] = held;

        if (pressed) {
            s_press_start_us[i] = event.timestamp_us;
            s_nudge_fired[i] = false;

            const bool doubleTap = s_tap_pending[i] &&
                                   (event.timestamp_us - s_tap_stamp_us[i]) <= DOUBLE_TAP_WINDOW_US;
            const int approach = APPROACH_FOR_PAD[i][doubleTap ? 1 : 0];
            s_tap_pending[i] = !doubleTap;
            s_tap_stamp_us[i] = event.timestamp_us;

            ESP_LOGI(TAG, "Pad %d %s tap -> approach %d", i,
                     doubleTap ? "double" : "single", approach);
            g_bassEngine.setApproach(approach);
        } else if (held && !s_nudge_fired[i]) {
            uint64_t heldFor = event.timestamp_us - s_press_start_us[i];
            if (heldFor >= NUDGE_HOLD_US) {
                s_nudge_fired[i] = true;
                s_tap_pending[i] = false;
                ESP_LOGI(TAG, "Pad %d long-press -> nudge", i);
                g_bassEngine.nudge();
            }
        } else if (released) {
            s_press_start_us[i] = 0;
            s_nudge_fired[i] = false;
        }
    }

    g_bassEngine.process(link_state, link_time);
}
