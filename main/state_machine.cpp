// state_machine.cpp
//
// Bassline interpreter control surface (see bass_engine.h / readme.md):
//   Tap a pad (0-3)   -> select which dial bank (Harmony/Groove/Motion/
//                        Voice) the 2 pots address. Switches instantly on
//                        press -- no tap/double-tap disambiguation delay,
//                        unlike the old genre scheme this replaces.
//                        The very first-ever tap also starts playback.
//   Long-press a pad  -> nudge: reroll the current bar with fresh
//                        randomness, same dial-driven character.
//   Hold all 4 pads   -> panic: stop playback (unchanged).
//   Pot 0 -> dial 0 of the active bank
//   Pot 1 -> dial 1 of the active bank
//
// No buzzer/LED feedback on bank-select or nudge: the buzzer is already
// fully committed to the Link-quantum metronome click (link_sync.cpp
// fires it on every beat), so layering pad-gesture chirps onto the same
// GPIO would just get masked or would itself mask the metronome. Bank
// switches are silent context changes (matches the old pot-context-
// switching behaviour, which was also silent); a nudge is audible
// through the bassline itself at the next bar.

#include "state_machine.h"
#include "bass_engine.h"
#include "esp_log.h"

static const char* TAG = "SM";

// Holding a pad this long (without releasing) fires a nudge in addition
// to the bank-select that already happened on press.
static const uint64_t NUDGE_HOLD_US = 500000;  // 500ms

// Per-pad: timestamp the current hold started (0 = not held)
static uint64_t s_press_start_us[4] = {0, 0, 0, 0};
// Whether the long-press nudge has already fired for the current hold
static bool s_nudge_fired[4] = {false, false, false, false};
// Track previous held state to detect press/release edges
static bool s_was_held[4] = {false, false, false, false};

void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time)
{
    // ---- Pots -> the active bank's 2 dials ----
    if (event.pot_moved[0])
        g_bassEngine.setDial(0, event.pot_value[0] / 127.0f);
    if (event.pot_moved[1])
        g_bassEngine.setDial(1, event.pot_value[1] / 127.0f);

    // ---- All four pads held -> stop MIDI note playback (panic) ----
    // Rising-edge latched: fire stop() once when all 4 first held together, then
    // suppress the per-pad handling this tick so a subsequent release isn't
    // read as a bank-select tap. Latch clears when fewer than 4 are held.
    static bool s_all4_latched = false;
    bool all4 = event.pad_held[0] && event.pad_held[1]
             && event.pad_held[2] && event.pad_held[3];
    if (all4) {
        if (!s_all4_latched) {
            s_all4_latched = true;
            ESP_LOGI(TAG, "All 4 pads held -> stop playback");
            g_bassEngine.stop();
        }
        // Consume the gesture: mark pads held and clear per-pad gesture
        // state so the eventual release isn't read as a tap or long-press.
        for (int i = 0; i < 4; i++) {
            s_was_held[i]       = event.pad_held[i];
            s_press_start_us[i] = 0;
            s_nudge_fired[i]    = false;
        }
        g_bassEngine.process(link_state, link_time);
        return;
    }
    s_all4_latched = false;

    // ---- Touch pads -> bank select (tap) / nudge (long-press) ----
    for (int i = 0; i < 4; i++) {
        bool held     = event.pad_held[i];
        bool pressed  = held && !s_was_held[i];   // rising edge
        bool released = !held && s_was_held[i];   // falling edge
        s_was_held[i] = held;

        if (pressed) {
            s_press_start_us[i] = event.timestamp_us;
            s_nudge_fired[i] = false;
            ESP_LOGI(TAG, "Pad %d -> bank %d", i, i);
            g_bassEngine.setActiveBank(i);
        } else if (held && !s_nudge_fired[i]) {
            uint64_t heldFor = event.timestamp_us - s_press_start_us[i];
            if (heldFor >= NUDGE_HOLD_US) {
                s_nudge_fired[i] = true;
                ESP_LOGI(TAG, "Pad %d long-press -> nudge", i);
                g_bassEngine.nudge();
            }
        } else if (released) {
            s_press_start_us[i] = 0;
            s_nudge_fired[i] = false;
        }
    }

    // ---- Bass engine tick ----
    g_bassEngine.process(link_state, link_time);
}
