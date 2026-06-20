#include "link_sync.h"
#include "io_helpers.h"
#include <cmath>
#include <cstring>
#include <driver/gptimer.h>
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <lwip/sockets.h>
#include <lwip/inet.h>

// --- Phase broadcast for bare-metal peers behind a unicast-RX wall (the Pi looper) ---
// The Pi's bcm4343 WiFi delivers multicast/broadcast but NOT unicast-to-self, so the
// standard Ableton Link unicast ping/pong measurement never completes there and the Pi
// can sync tempo but not PHASE (witnessed: Pi :4445 WLAN uniRx stays 0). As the session
// master we therefore ALSO multicast our current Link-clock micros to the Link group on
// a dedicated port; the Pi uses it as the measured ghost offset (one-way WiFi latency
// ~1-3ms is <1% of a beat). Payload: "LCLK"(4) + int64 LE link-clock micros. Standard
// Link apps (Live) ignore this port and keep using real measurement.
#define LINK_PHASE_PORT 20810
static const char* LINK_MCAST_ADDR = "224.76.78.75";
static int s_clk_sock = -1;

static void broadcast_link_clock(int64_t linkMicros) {
    if (s_clk_sock < 0) {
        s_clk_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_clk_sock < 0) return;
        uint8_t ttl = 2;
        setsockopt(s_clk_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    }
    uint8_t pkt[12];
    memcpy(pkt, "LCLK", 4);
    memcpy(pkt + 4, &linkMicros, 8);          // little-endian; both ends are LE
    struct sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(LINK_PHASE_PORT);
    dst.sin_addr.s_addr = inet_addr(LINK_MCAST_ADDR);
    sendto(s_clk_sock, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst);
}

// --- Master-clock compatibility for the non-negotiable targets ---
// Our emission set is brand-agnostic raw MIDI: continuous 24ppqn clock (0xF8), and at
// the 16-bar phrase boundary only: SPP (0xF2) + Start/Continue (0xFA/0xFB). Stop (0xFC)
// + All-Notes-Off (CC123) on transport stop / peer loss. How each target consumes it:
//   KO2 (Korg KO II) : locks to ext clock; Start needed to run; SPP repositions cleanly
//                      at phrase boundary (infrequent SPP avoids its known SPP sensitivity).
//   Volca Drum       : syncs to clock pulse only; ignores SPP/SPP-spam harmless now that
//                      SPP fires once per phrase, not every 4 beats.
//   MicroKorg        : arp/delay sync follows clock; needs a stable (non-bursting) clock
//                      -- the per-tick cap + resync guarantees that.
//   Micron           : clock + Start; phrase-aligned Start keeps its sequencer in phrase.
//   MiniNova         : arp/LFO sync to clock; stable clock keeps modulation locked.
//   RC-505 MK2       : loop station; locks to clock+Start, SPP repositions; note-offs use
//                      velocity 0 (see io_helpers / CLAUDE.md) and CC123 on stop avoids
//                      stuck loops.
// None require per-device clock code; the single emission path serves all. Per-device
// parameter control (NRPN) lives in the synth_* classes and is unaffected.

// Add logging tag
static const char *TAG_LINK = "LINK_SYNC";

static int s_last_quantum_number = -1;
static int s_last_phrase_number = -1;
static gptimer_handle_t s_link_gptimer = nullptr;
static esp_timer_handle_t s_buzzer_off_timer = nullptr;

static void buzzer_off_cb(void*) {
    set_buzzer_state(false);
}

static bool IRAM_ATTR link_gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(user_data), 1, eSetBits, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}


// Simple quantum boundary detection using phase reset
QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time) {
    QuantumInfo info;

    info.sessionBeat = state.beatAtTime(time, LINK_QUANTUM);
    info.phaseWithinQuantum = state.phaseAtTime(time, LINK_QUANTUM);
    info.currentQuantumNumber = static_cast<int>(std::floor(info.sessionBeat / LINK_QUANTUM));
    info.beatInQuantum = static_cast<int>(std::floor(info.phaseWithinQuantum));
    info.beatFraction = info.phaseWithinQuantum - std::floor(info.phaseWithinQuantum);

    // Phrase boundary tracking (16 bars / PHRASE_BEATS). Computed from the same
    // monotonic sessionBeat so quantum and phrase share one timeline.
    info.phaseWithinPhrase = state.phaseAtTime(time, PHRASE_BEATS);
    info.currentPhraseNumber = static_cast<int>(std::floor(info.sessionBeat / PHRASE_BEATS));

    if (s_last_quantum_number == -1) {
        s_last_quantum_number = info.currentQuantumNumber;
        info.crossedQuantumBoundary = false;
    } else if (info.currentQuantumNumber != s_last_quantum_number) {
        s_last_quantum_number = info.currentQuantumNumber;
        info.crossedQuantumBoundary = true;
        ESP_LOGI(TAG_LINK, "Quantum boundary %d, beat %.2f", info.currentQuantumNumber, info.sessionBeat);
    } else {
        info.crossedQuantumBoundary = false;
    }

    if (s_last_phrase_number == -1) {
        s_last_phrase_number = info.currentPhraseNumber;
        info.crossedPhraseBoundary = false;
    } else if (info.currentPhraseNumber != s_last_phrase_number) {
        s_last_phrase_number = info.currentPhraseNumber;
        info.crossedPhraseBoundary = true;
        ESP_LOGI(TAG_LINK, "Phrase boundary %d, beat %.2f", info.currentPhraseNumber, info.sessionBeat);
    } else {
        info.crossedPhraseBoundary = false;
    }

    return info;
}

// Initialize Timer for Link Task
void init_link_timer(TaskHandle_t task_handle) {
    // Zero-initialize the struct to catch all members, including those in unnamed structs
    gptimer_config_t timer_config = {}; 
    timer_config.clk_src = GPTIMER_CLK_SRC_APB;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1 MHz = 1us tick
    timer_config.intr_priority = 3;       // Higher priority for metronome timer
    timer_config.flags.intr_shared = 0; // Assuming timer interrupt is not shared
    // timer_config.flags.allow_pd and timer_config.flags.backup_before_sleep will be zero-initialized

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_link_gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = link_gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_link_gptimer, &cbs, task_handle));

    ESP_ERROR_CHECK(gptimer_set_raw_count(s_link_gptimer, 0));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = LINK_TICK_PERIOD,
        .reload_count = 0,  // For periodic, set reload_count to 0 and use flags
        .flags = {
            .auto_reload_on_alarm = 1 // Enable auto-reload
        }
    };
    ESP_ERROR_CHECK(::gptimer_set_alarm_action(s_link_gptimer, &alarm_config)); // Correct function name
    ESP_ERROR_CHECK(gptimer_enable(s_link_gptimer));
    ESP_ERROR_CHECK(gptimer_start(s_link_gptimer));
    ESP_LOGI(TAG_LINK, "Link GPTimer Initialized (Period: %d us)", LINK_TICK_PERIOD);

    esp_timer_create_args_t buzzer_timer_args = {};
    buzzer_timer_args.callback = buzzer_off_cb;
    buzzer_timer_args.name = "buzzer_off";
    ESP_ERROR_CHECK(esp_timer_create(&buzzer_timer_args, &s_buzzer_off_timer));
}

// Send MIDI realtime/transport byte(s). Realtime status bytes (0xF8/0xFA/0xFB/0xFC)
// are single-byte and may legally interleave a running-status message, so the buzzer
// path and clock path never corrupt each other.
static void send_midi_bytes(const uint8_t* buf, size_t len) {
    uart_write_bytes(MIDI_UART, (const char*)buf, len);
}

// Clear hanging notes on every channel. Sent on transport Stop and on Link peer loss
// so brand-varied gear (RC-505 MK2/KO2 especially) never holds a note across a resync.
static void send_all_notes_off_all_channels() {
    for (uint8_t ch = 0; ch < 16; ++ch) {
        const uint8_t cc[] = { (uint8_t)(MIDI_CC_CMD | ch), MIDI_CC_ALL_NOTES_OFF, 0 };
        send_midi_bytes(cc, sizeof(cc));
    }
}

// Song Position Pointer carries position in MIDI beats (sixteenth notes). One Link beat
// (quarter note) = 4 SPP units. 14-bit value, LSB first.
static void send_song_position(double sessionBeat) {
    uint16_t spp_units = static_cast<uint16_t>(sessionBeat * 4.0) & 0x3FFF;
    const uint8_t spp[] = { MIDI_SPP,
                            (uint8_t)(spp_units & 0x7F),
                            (uint8_t)((spp_units >> 7) & 0x7F) };
    send_midi_bytes(spp, sizeof(spp));
}

// Main Link Synchronization Logic (called from tickTask)
// pending_realign: set when a peer joins or play-state begins; the actual MIDI
// Start+SPP is held until the next 16-bar phrase boundary so all gear begins the
// phrase together rather than jerking in mid-phrase.
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                        int& lastTicks, int& length, int& lastBeat, int& currentBuzzerFreq, bool& was_playing,
                        const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    static bool s_pending_realign = false;   // a Start+SPP is owed at the next phrase boundary
    static bool s_transport_running = false; // whether external gear currently believes it is playing

    // Broadcast our Link-clock micros every tick (~20ms) so multicast-only peers
    // (the Pi looper, which can't receive unicast measurement) can lock phase.
    broadcast_link_clock(time.count());

    // Check peer status & force start timeout. Hold force-start longer (8s) than the
    // original 5s so two co-booting devices have time to discover each other over WiFi
    // before either free-runs; this avoids both emitting transport at independent phases
    // and then snapping when they peer.
    bool is_connected = g_link->numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= 8000000)) {
        force_start = true;
        ESP_LOGW(TAG_LINK, "No Link peers found for 8s, forcing start.");
    }

    // Handle connection changes. On peer join we do NOT immediately Stop/Start; instead
    // we arm a phrase-aligned realign so gear snaps to the shared phrase at the next
    // 16-bar boundary. On peer loss we clear hanging notes.
    if (is_connected != was_connected) {
        ESP_LOGI(TAG_LINK, "Link peers changed: %d", g_link->numPeers());
        if (is_connected) {
            auto qi = detectQuantumBoundary(state, time);
            ESP_LOGI(TAG_LINK, "Link connected -- beat=%.3f phase=%.3f quantum=%d phrase=%d",
                     qi.sessionBeat, qi.phaseWithinQuantum, qi.currentQuantumNumber, qi.currentPhraseNumber);
            // Reset lastBeat to avoid spurious trigger from Link phase adjustment
            lastBeat = qi.beatInQuantum;
            // Resync clock counter to the live beat so we do not burst clocks for the
            // beats that elapsed before peering.
            lastTicks = static_cast<int>(qi.sessionBeat * 24);
            s_pending_realign = true;
        } else {
            // Peer lost -- stop external gear cleanly and clear any held notes.
            const uint8_t stop_msg[] = { MIDI_STOP };
            send_midi_bytes(stop_msg, 1);
            send_all_notes_off_all_channels();
            s_transport_running = false;
            ESP_LOGI(TAG_LINK, "Link peer lost -- sent Stop + All Notes Off.");
        }
        was_connected = is_connected;
    }

    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    const double sessionBeat = quantumInfo.sessionBeat;
    const int beatInQuantum = quantumInfo.beatInQuantum;
    const bool crossedQuantumBoundary = quantumInfo.crossedQuantumBoundary;
    const bool crossedPhraseBoundary = quantumInfo.crossedPhraseBoundary;

    // Target clock count: 24 per quarter note. Monotonic in sessionBeat.
    const int midiClocks = static_cast<int>(sessionBeat * 24);

    // Detect beat boundary crossing
    bool crossedBeat = (beatInQuantum != lastBeat);

    // Metronome and MIDI Sync Logic
    if (is_connected || force_start) {
        // Metronome frequency based on beat position (emphasize 16, 8, 4, 1)
        if (crossedQuantumBoundary) {
            length = LENGTH_16BEAT;
            currentBuzzerFreq = FREQ_16BEAT;
            ESP_LOGI(TAG_LINK, "Quantum boundary at beat %.1f", sessionBeat);
        } else if (crossedBeat) {
            if (beatInQuantum % 8 == 0) {
                length = LENGTH_8BEAT;
                currentBuzzerFreq = FREQ_8BEAT;
            } else if (beatInQuantum % 4 == 0) {
                length = LENGTH_4BEAT;
                currentBuzzerFreq = FREQ_4BEAT;
            } else {
                length = LENGTH_NORMAL;
                currentBuzzerFreq = FREQ_NORMAL;
            }
        }

        if (crossedBeat || crossedQuantumBoundary) {
            lastBeat = beatInQuantum;
            // Edge-triggered buzzer: fire on beat crossing, schedule off
            set_buzzer_state(true, currentBuzzerFreq);
            esp_timer_stop(s_buzzer_off_timer);
            esp_timer_start_once(s_buzzer_off_timer, (int64_t)length * 1000);
        }

        bool is_playing = state.isPlaying();

        // Play-state changes: stopping is immediate (and clears held notes); starting
        // is deferred to the next phrase boundary so gear begins in phrase.
        if (was_playing != is_playing) {
            if (is_playing) {
                s_pending_realign = true;  // honor at next phrase boundary below
            } else {
                const uint8_t msg = MIDI_STOP;
                send_midi_bytes(&msg, 1);
                send_all_notes_off_all_channels();
                s_transport_running = false;
                ESP_LOGI(TAG_LINK, "MIDI STOP at beat %.1f (+ All Notes Off)", sessionBeat);
            }
            was_playing = is_playing;
        }

        // Phrase boundary (16 bars): the only place we realign external transport.
        // Emit SPP so gear repositions to the exact phrase start, then Start/Continue
        // if a realign is pending. Doing this at the phrase boundary (never every few
        // beats) keeps KO2/Volca/etc. in phrase without transport spam.
        if (crossedPhraseBoundary) {
            send_song_position(sessionBeat);
            if (s_pending_realign && (is_playing || force_start)) {
                const uint8_t start_msg = s_transport_running ? MIDI_CONTINUE : MIDI_START;
                send_midi_bytes(&start_msg, 1);
                s_transport_running = true;
                s_pending_realign = false;
                ESP_LOGI(TAG_LINK, "Phrase-aligned %s + SPP at beat %.1f",
                         start_msg == MIDI_START ? "START" : "CONTINUE", sessionBeat);
            }
        }

        // Emit due MIDI timing clocks, but cap per-tick output. If we have fallen far
        // behind the live beat (post-stall), hard-resync the counter instead of bursting
        // -- a flood of 0xF8 reads as a tempo spike on every target.
        int behind = midiClocks - lastTicks;
        if (behind > MIDI_CLOCK_RESYNC_THRESHOLD) {
            ESP_LOGW(TAG_LINK, "Clock %d behind -- resyncing to beat %.2f (no burst)", behind, sessionBeat);
            lastTicks = midiClocks;
        } else {
            int emitted = 0;
            while (lastTicks < midiClocks && emitted < MIDI_MAX_CLOCKS_PER_TICK) {
                lastTicks++;
                emitted++;
                const uint8_t timing_msg = MIDI_TIMING_CLOCK;
                send_midi_bytes(&timing_msg, 1);
            }
        }
    } else {
        set_buzzer_state(false);
    }
}