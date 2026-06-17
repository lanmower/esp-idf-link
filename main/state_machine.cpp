// state_machine.cpp
//
// Simple genre controller:
//   Single tap  on pad 0-3 -> GENRE_FUNK / ITALO / SYNTHPOP / PSYTRANCE
//   Double tap  on pad 0-3 -> GENRE_PROG / AFROHOUSE / UKG / GFUNK
//   Pot 0 -> ctrl1  (macro 1 for active genre)
//   Pot 1 -> ctrl2  (macro 2 for active genre)

#include "state_machine.h"
#include "bass_engine.h"
#include "esp_log.h"

static const char* TAG = "SM";

// Double-tap detection window (microseconds)
static const uint64_t DOUBLE_TAP_US = 400000;

// Genre mapping
// Single tap  -> pad 0..3 -> genre 0..3 (funk, italo, synthpop, psytrance)
// Double tap  -> pad 0..3 -> genre 4..7 (prog, afrohouse, ukg, gfunk)
static const int GENRE_SINGLE[4] = {0, 1, 2, 3};
static const int GENRE_DOUBLE[4] = {4, 5, 6, 7};

// Per-pad: timestamp of the last press (0 = never pressed)
static uint64_t s_last_press_us[4] = {0, 0, 0, 0};
// Track previous held state to detect press edges
static bool s_was_held[4] = {false, false, false, false};

void process_state_event(const InputEvent& event,
                         const ableton::Link::SessionState& link_state,
                         const std::chrono::microseconds& link_time)
{
    // ---- Pots -> macro knobs ----
    if (event.pot_moved[0])
        g_bassEngine.setCtrl1(event.pot_value[0] / 127.0f);
    if (event.pot_moved[1])
        g_bassEngine.setCtrl2(event.pot_value[1] / 127.0f);

    // ---- Touch pads -> genre selection ----
    for (int i = 0; i < 4; i++) {
        // Detect rising edge (press)
        bool pressed = event.pad_held[i] && !s_was_held[i];
        s_was_held[i] = event.pad_held[i];

        if (!pressed) continue;

        uint64_t now     = event.timestamp_us;
        uint64_t elapsed = (s_last_press_us[i] > 0) ? (now - s_last_press_us[i]) : UINT64_MAX;

        if (elapsed < DOUBLE_TAP_US) {
            // Double tap detected -> secondary genre
            int genre = GENRE_DOUBLE[i];
            ESP_LOGI(TAG, "Pad %d double-tap -> genre %d", i, genre);
            g_bassEngine.setGenre(genre);
            s_last_press_us[i] = 0; // reset so triple-tap won't re-trigger
        } else {
            // First (or isolated) tap -> primary genre
            int genre = GENRE_SINGLE[i];
            ESP_LOGI(TAG, "Pad %d tap -> genre %d", i, genre);
            g_bassEngine.setGenre(genre);
            s_last_press_us[i] = now;
        }
    }

    // ---- Bass engine tick ----
    g_bassEngine.process(link_state, link_time);
}
