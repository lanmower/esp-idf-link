#include "state_machine.h"
#include "bass_engine.h"
#include "esp_log.h"

static const char* TAG = "SM";

// A second tap on the same pad within this window is a double tap.
static const uint64_t DOUBLE_TAP_WINDOW_US = 300000;

static uint64_t s_tap_stamp_us[NUM_TOUCH_PADS]   = {0, 0, 0, 0};
static bool     s_tap_pending[NUM_TOUCH_PADS]    = {false, false, false, false};
static bool     s_was_held[NUM_TOUCH_PADS]       = {false, false, false, false};
static bool     s_double_held_by[NUM_TOUCH_PADS] = {false, false, false, false};

void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time)
{
    static bool s_all4_latched = false;
    static int  s_held_count = 0;

    int held_count = 0;
    for (int i = 0; i < NUM_TOUCH_PADS; i++) held_count += event.pad_held[i] ? 1 : 0;
    if (held_count == 3 && s_held_count < 3) {
        g_bassEngine.clearAll();
    } else if (held_count == 2 && s_held_count != 2) {
        ESP_LOGI(TAG, "two pads held -> fill on");
        g_bassEngine.setFill(true);
    } else if (held_count != 2 && s_held_count == 2) {
        ESP_LOGI(TAG, "fill off");
        g_bassEngine.setFill(false);
    }
    s_held_count = held_count;
    const bool all4 = event.pad_held[0] && event.pad_held[1]
                   && event.pad_held[2] && event.pad_held[3];
    if (all4) {
        if (!s_all4_latched) {
            s_all4_latched = true;
            g_bassEngine.stop();
        }
        for (int i = 0; i < NUM_TOUCH_PADS; i++) {
            s_was_held[i] = event.pad_held[i];
            s_tap_pending[i] = false;
            s_double_held_by[i] = false;
        }
        g_bassEngine.setDoubleHeld(-1);
        g_bassEngine.process(link_state, link_time, event.timestamp_us);
        return;
    }
    s_all4_latched = false;

    for (int i = 0; i < NUM_TOUCH_PADS; i++) {
        const bool held     = event.pad_held[i];
        const bool pressed  = held && !s_was_held[i];
        const bool released = !held && s_was_held[i];
        s_was_held[i] = held;

        if (pressed) {
            const bool doubleTap = s_tap_pending[i] &&
                                   (event.timestamp_us - s_tap_stamp_us[i]) <= DOUBLE_TAP_WINDOW_US;
            s_tap_pending[i] = !doubleTap;
            s_tap_stamp_us[i] = event.timestamp_us;
            if (doubleTap) {
                // Double-tap and hold: the dials control this pad's double-tap settings.
                s_double_held_by[i] = true;
                g_bassEngine.setDoubleHeld(i);
            } else {
                g_bassEngine.selectSingle(i);
            }
        } else if (released && s_double_held_by[i]) {
            s_double_held_by[i] = false;
            if (g_bassEngine.doubleHeld() == i) g_bassEngine.setDoubleHeld(-1);
        }
    }

    for (int dial = 0; dial < NUM_POTS; dial++) {
        if (event.pot_moved[dial]) {
            g_bassEngine.setDial(dial,
                                 static_cast<float>(event.pot_value[dial]) / MIDI_VALUE_MAX,
                                 event.timestamp_us);
        }
    }

    g_bassEngine.process(link_state, link_time, event.timestamp_us);
}
