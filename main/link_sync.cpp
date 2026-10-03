#include "link_sync.h"
#include "io_helpers.h"
#include <cmath>
#include <cstring>
#include <driver/gptimer.h>
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include "esp_netif.h"
#include "wifi_config.h"   // wifi_is_ap_active() for the queryable status reply

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
    // handle_link_sync runs at LINK_TICK_PERIOD (4 kHz); phase only needs ~50 Hz.
    // Rate-limit to one packet per 20ms so we don't flood multicast-only peers
    // (a 4 kHz flood saturates the Pi's single radio-RX drain and starves its
    // control plane). 20ms << one beat, so phase accuracy is unaffected.
    static int64_t s_lastSend = 0;
    int64_t nowUs = esp_timer_get_time();
    if (nowUs - s_lastSend < 20000) return;
    s_lastSend = nowUs;
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

    // Send out EVERY active netif, setting IP_MULTICAST_IF per interface. We may be
    // AP or STA (boot race), and a raw socket with no IP_MULTICAST_IF exits the wrong
    // interface on a dual-netif device -- which is why the first cut reached the peer
    // for Link's own socket (it sets the egress) but not for ours (Pi clkRx stayed 0).
    // Iterating every netif with an IP guarantees the packet leaves the one actually
    // connected to the peer regardless of role.
    esp_netif_t* nif = esp_netif_next_unsafe(NULL);
    for (; nif; nif = esp_netif_next_unsafe(nif)) {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(nif, &ipinfo) != ESP_OK || ipinfo.ip.addr == 0) continue;
        struct in_addr ifaddr; ifaddr.s_addr = ipinfo.ip.addr;
        setsockopt(s_clk_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof ifaddr);
        sendto(s_clk_sock, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst);
    }
}

// --- Ticker -> looper timeline broadcast (bidirectional Link tempo) ---
// Ableton Link lets ANY device set the group tempo. The looper->esp direction works
// via LTMP (tempo_listener_task). The esp->looper direction was MISSING: we run the
// real Link lib, but its native discovery never reaches the Pi (the Pi's bcm4343 has
// a unicast-RX wall: Pi :4445 WLAN RALV stays empty, peers=0), so when WE change the
// tempo the looper never learns it. So we ALSO multicast our CURRENT Link timeline
// on TTMP_PORT, per-netif (same IP_MULTICAST_IF fix as the clock broadcast), in the
// format the looper parses into a synthetic owner peer. Combined with LCLK (clock
// offset), the looper adopts our tempo AND phase. Standard Link apps ignore TTMP.
// Payload: "TTMP"(4) + i64 LE microsPerBeat + i64 LE beatOriginMicroBeats(link clock)
//          + i64 LE timeOriginMicros(link clock).  (28 bytes)
#define LINK_TTMP_PORT 20812
static int s_ttmp_sock = -1;
static void broadcast_ticker_timeline(const ableton::Link::SessionState& state,
                                      int64_t linkMicros) {
    static int64_t s_lastSend = 0;
    if (linkMicros - s_lastSend < 100000) return;   // ~10 Hz; tempo changes are rare
    s_lastSend = linkMicros;
    if (s_ttmp_sock < 0) {
        s_ttmp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_ttmp_sock < 0) return;
        uint8_t ttl = 2;
        setsockopt(s_ttmp_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    }
    // Current timeline at 'linkMicros': tempo + the beat at that instant define a
    // (beatOrigin @ timeOrigin) the looper extrapolates from. quantum here only
    // affects phaseAtTime, not beatAtTime, so use LINK_QUANTUM for the beat value.
    double bpm = state.tempo();
    // Ableton's own Test Plan TEMPO-4 exercises the FULL Link range and names
    // 20bpm and 999bpm explicitly: an app must stay in sync across it. A 400
    // ceiling silently dropped any legitimate session tempo above it instead of
    // following, so the bound is Link's real range.
    if (!(bpm >= 20.0 && bpm <= 999.0)) return;
    int64_t mpb = (int64_t)(60000000.0 / bpm + 0.5);
    double beats = state.beatAtTime(std::chrono::microseconds(linkMicros), LINK_QUANTUM);
    int64_t beatOriginUb = (int64_t)(beats * 1e6);
    int64_t timeOrigin   = linkMicros;
    uint8_t pkt[28];
    memcpy(pkt,      "TTMP", 4);
    memcpy(pkt + 4,  &mpb, 8);
    memcpy(pkt + 12, &beatOriginUb, 8);
    memcpy(pkt + 20, &timeOrigin, 8);
    struct sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(LINK_TTMP_PORT);
    dst.sin_addr.s_addr = inet_addr(LINK_MCAST_ADDR);
    esp_netif_t* nif = esp_netif_next_unsafe(NULL);
    for (; nif; nif = esp_netif_next_unsafe(nif)) {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(nif, &ipinfo) != ESP_OK || ipinfo.ip.addr == 0) continue;
        struct in_addr ifaddr; ifaddr.s_addr = ipinfo.ip.addr;
        setsockopt(s_ttmp_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof ifaddr);
        sendto(s_ttmp_sock, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst);
    }
}

// --- Looper tempo-set listener (Ableton Link: ANY device may set the group tempo) ---
// The looper (a bare-metal Link peer behind a unicast-RX wall) cannot be MEASURED by
// us (no ping/pong), so our official Link lib never adopts its broadcast timeline
// tempo. The looper therefore multicasts an explicit "LTMP"(4)+i64 LE microsPerBeat
// command to the Link group on LINK_TEMPO_PORT; we apply it via setTempo() so the
// ticker's clock (and any measured Live peer) follow the loop's tempo. recvfrom runs
// on its own task; the actual setTempo is deferred to the Link task (tickTask) which
// owns the session-state capture/commit -- committing from another task races it.
#define LINK_TEMPO_PORT 20811
static volatile bool   s_tempoReqPending = false;
static volatile double s_tempoReqBpm     = 0.0;
static volatile bool   s_phaseReqPending = false;
static volatile int64_t s_phaseReqBeat0us = 0;   // esp-clock micros of the loop downbeat (beat 0)
static volatile double  s_phaseReqQuantum = 4.0; // loop quantum in beats

// Queryable Link status, so the aloop<->esp mesh test can be SCRIPTED instead of
// eyeballed on a serial console. ../aloop already exposes peers/bpm/playing in
// /run/aloop/status.json; this is the ESP half. Send any UDP datagram to
// LINK_STATUS_PORT and get a one-line JSON reply from the same source port.
// Deliberately request/response (not a broadcast) so it costs nothing when idle
// and cannot pollute the Link multicast group.
#define LINK_STATUS_PORT 20812

// How late the hardware-scheduled metronome/clock alarms actually fire. Exposed over the
// mesh so the click's stability is measurable from a peer instead of needing a scope on
// the buzzer; defined with the scheduler below.
struct MetroStats { uint32_t fired; int64_t last; int64_t worst; double mean; double rms; };
MetroStats metro_stats();

static void status_responder_task(void*) {
    int rs = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rs < 0) { vTaskDelete(NULL); return; }
    int one = 1;
    setsockopt(rs, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in ba = {};
    ba.sin_family      = AF_INET;
    ba.sin_port        = htons(LINK_STATUS_PORT);
    ba.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(rs, (struct sockaddr*)&ba, sizeof ba) < 0) { close(rs); vTaskDelete(NULL); return; }

    uint8_t req[64];
    char reply[384];
    for (;;) {
        struct sockaddr_in src = {};
        socklen_t sl = sizeof src;
        int n = recvfrom(rs, req, sizeof req, 0, (struct sockaddr*)&src, &sl);
        if (n < 0) continue;
        if (!g_link) {
            int len = snprintf(reply, sizeof reply, "{\"link\":false}\n");
            sendto(rs, reply, len, 0, (struct sockaddr*)&src, sl);
            continue;
        }
        auto st = g_link->captureAppSessionState();
        const auto now = g_link->clock().micros();
        const MetroStats ms = metro_stats();
        int len = snprintf(reply, sizeof reply,
            "{\"link\":true,\"peers\":%u,\"bpm\":%.3f,\"playing\":%s,"
            "\"beat\":%.3f,\"phase\":%.3f,\"quantum\":%.1f,\"ap\":%s,"
            "\"metro\":{\"fired\":%u,\"late_last_us\":%lld,\"late_worst_us\":%lld,"
            "\"late_mean_us\":%.1f,\"late_rms_us\":%.1f}}\n",
            (unsigned)g_link->numPeers(),
            st.tempo(),
            st.isPlaying() ? "true" : "false",
            st.beatAtTime(now, LINK_QUANTUM),
            st.phaseAtTime(now, LINK_QUANTUM),
            (double)LINK_QUANTUM,
            wifi_is_ap_active() ? "true" : "false",
            (unsigned)ms.fired, (long long)ms.last, (long long)ms.worst, ms.mean, ms.rms);
        sendto(rs, reply, len, 0, (struct sockaddr*)&src, sl);
    }
}

static void tempo_listener_task(void*) {
    int rs = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rs < 0) { vTaskDelete(NULL); return; }
    int one = 1;
    setsockopt(rs, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in ba = {};
    ba.sin_family = AF_INET;
    ba.sin_port   = htons(LINK_TEMPO_PORT);
    ba.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(rs, (struct sockaddr*)&ba, sizeof ba) < 0) { close(rs); vTaskDelete(NULL); return; }
    struct ip_mreq mreq = {};
    inet_aton(LINK_MCAST_ADDR, &mreq.imr_multiaddr);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(rs, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof mreq);
    uint8_t buf[64];
    for (;;) {
        int n = recvfrom(rs, buf, sizeof buf, 0, NULL, NULL);
        if (n >= 12 && memcmp(buf, "LTMP", 4) == 0) {
            int64_t mpb;
            memcpy(&mpb, buf + 4, 8);
            if (mpb > 0) {
                double bpm = 60000000.0 / (double)mpb;
                // Same Test Plan TEMPO-4 range as the emit path above (20..999).
                if (bpm >= 20.0 && bpm <= 999.0) { s_tempoReqBpm = bpm; s_tempoReqPending = true; }
            }
            // Optional phase payload: esp-clock beat-0 micros + quantum (microbeats).
            if (n >= 28) {
                int64_t beat0us, quantumUb;
                memcpy(&beat0us,  buf + 12, 8);
                memcpy(&quantumUb, buf + 20, 8);
                if (quantumUb > 0) {
                    s_phaseReqBeat0us = beat0us;
                    s_phaseReqQuantum = (double)quantumUb / 1e6;
                    s_phaseReqPending = true;
                }
            }
        }
    }
}

void link_start_tempo_listener() {
    xTaskCreate(tempo_listener_task, "ltmp_rx", 4096, NULL, 5, NULL);
    // Queryable status on LINK_STATUS_PORT, started alongside the LTMP listener
    // so both come up at the same point (after g_link exists and WiFi is up).
    xTaskCreate(status_responder_task, "link_status", 4096, NULL, 5, NULL);
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
//                      -- the alarm scheduler emits one pulse per due time and drops
//                      any it cannot reach, so it never bursts.
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

// --- Hardware-scheduled Link-timeline events (metronome click + 24 ppqn MIDI clock) ---
// Neither can be emitted from the 4 kHz tick task: a tick only discovers a beat AFTER it
// has already passed, so every click and every 0xF8 inherits that wake-up's own latency
// plus whatever WiFi, logging and input polling happened during it -- milliseconds of
// jitter, which is exactly what an audible metronome exposes. Instead the due time of the
// NEXT pulse is asked for up front (SessionState::timeAtBeat) and one esp_timer alarm is
// armed to it; the alarm callback does the emitting, so the only error left is the alarm's
// dispatch latency (tens of microseconds) and it is the same error for the click and the
// clock. Link's ESP clock IS esp_timer (platforms/esp32/Clock.hpp), so a Link-clock
// microsecond timestamp is already an esp_timer deadline: no offset, no drift.
static esp_timer_handle_t s_evt_timer = nullptr;
static volatile bool      s_evt_armed   = false;
static volatile bool      s_evt_enabled = false;
static volatile int64_t   s_evt_due_us   = 0;
static volatile int64_t   s_last_fire_us = 0;
static int64_t            s_next_pulse       = 0;
static bool               s_next_pulse_valid = false;
static uint32_t           s_next_click_freq  = FREQ_NORMAL;
static int                s_next_click_ms    = LENGTH_NORMAL;

// Fired-alarm lateness, so the metronome's real stability is measurable over the mesh
// (UDP status) instead of needing a scope on the buzzer.
static volatile int64_t  s_fire_late_last_us  = 0;
static volatile int64_t  s_fire_late_worst_us = 0;
static uint32_t          s_fire_count         = 0;
static double            s_fire_late_sum_us   = 0.0;
static double            s_fire_late_sq_us    = 0.0;

MetroStats metro_stats() {
    MetroStats ms{};
    ms.fired = s_fire_count;
    ms.last  = s_fire_late_last_us;
    ms.worst = s_fire_late_worst_us;
    ms.mean  = s_fire_count ? s_fire_late_sum_us / (double)s_fire_count : 0.0;
    ms.rms   = s_fire_count ? std::sqrt(s_fire_late_sq_us / (double)s_fire_count) : 0.0;
    return ms;
}

static void metronome_accent_for_beat(double beat, uint32_t& freq, int& ms);
static void link_event_cb(void*);

static void buzzer_off_cb(void*) {
    set_buzzer_state(false);
    // Prime the next click's pitch now, while the buzzer is silent: the ON edge then
    // costs two duty writes instead of a PWM timer reconfigure landing on the beat.
    if (g_link) {
        const auto st = g_link->captureAppSessionState();
        const double beatNow = st.beatAtTime(g_link->clock().micros(), LINK_QUANTUM);
        metronome_accent_for_beat(std::floor(beatNow) + 1.0, s_next_click_freq, s_next_click_ms);
    }
    prime_buzzer_freq(s_next_click_freq);
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

    esp_timer_create_args_t evt_timer_args = {};
    evt_timer_args.callback = link_event_cb;
    evt_timer_args.name = "link_evt";
    ESP_ERROR_CHECK(esp_timer_create(&evt_timer_args, &s_evt_timer));
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

// Accent pattern within the quantum, as it was chosen when the beat was scheduled: bar
// top (16), half (8), quarter (4), else the plain beat.
static void metronome_accent_for_beat(double beat, uint32_t& freq, int& ms) {
    double pos = std::fmod(std::floor(beat), LINK_QUANTUM);
    if (pos < 0.0) pos += LINK_QUANTUM;
    const int b = static_cast<int>(pos);
    if (b == 0)          { freq = FREQ_16BEAT; ms = LENGTH_16BEAT; }
    else if (b % 8 == 0) { freq = FREQ_8BEAT;  ms = LENGTH_8BEAT;  }
    else if (b % 4 == 0) { freq = FREQ_4BEAT;  ms = LENGTH_4BEAT;  }
    else                 { freq = FREQ_NORMAL; ms = LENGTH_NORMAL; }
}

static void arm_event_timer(int64_t dueEspUs) {
    int64_t delta = dueEspUs - esp_timer_get_time();
    if (delta < 0) delta = 0;
    esp_timer_stop(s_evt_timer);
    if (esp_timer_start_once(s_evt_timer, static_cast<uint64_t>(delta)) != ESP_OK) return;
    s_evt_due_us = dueEspUs;
    s_evt_armed  = true;
}

static void schedule_next_event(const ableton::Link::SessionState& state) {
    const int64_t nowLink = g_link->clock().micros().count();
    const double beatNow = state.beatAtTime(std::chrono::microseconds(nowLink), LINK_QUANTUM);
    const int64_t dueNow = static_cast<int64_t>(std::floor(beatNow * 24.0)) + 1;
    if (!s_next_pulse_valid || s_next_pulse < dueNow) {
        if (s_next_pulse_valid && dueNow - s_next_pulse > MIDI_CLOCK_RESYNC_THRESHOLD)
            ESP_LOGW(TAG_LINK, "clock %lld pulse(s) behind -- resyncing to beat %.2f (no burst)",
                     (long long)(dueNow - s_next_pulse), beatNow);
        s_next_pulse = dueNow;
        s_next_pulse_valid = true;
    }
    // A tempo change, or a peer imposing phase, moves the timeline under a pending alarm.
    // Step forward to a pulse that is still ahead rather than firing a burst for the past.
    int64_t due = state.timeAtBeat((double)s_next_pulse / 24.0, LINK_QUANTUM).count();
    for (int guard = 0; due < nowLink && guard < 1024; guard++) {
        s_next_pulse++;
        due = state.timeAtBeat((double)s_next_pulse / 24.0, LINK_QUANTUM).count();
    }
    arm_event_timer(due);
}

static void link_event_cb(void*) {
    if (!g_link) { s_evt_armed = false; return; }
    const int64_t firedAt = esp_timer_get_time();
    const int64_t late    = firedAt - s_evt_due_us;
    s_evt_armed = false;
    s_last_fire_us = firedAt;
    s_fire_late_last_us = late;
    if (late > s_fire_late_worst_us) s_fire_late_worst_us = late;
    s_fire_count++;
    s_fire_late_sum_us += (double)late;
    s_fire_late_sq_us  += (double)late * (double)late;

    if ((s_next_pulse % 24) == 0) {
        const double beat = (double)s_next_pulse / 24.0;
        if (beat >= 0.0) {
            set_buzzer_state(true, s_next_click_freq);
            esp_timer_stop(s_buzzer_off_timer);
            esp_timer_start_once(s_buzzer_off_timer, (uint64_t)s_next_click_ms * 1000);
        }
    }
    const uint8_t timing_msg = MIDI_TIMING_CLOCK;
    send_midi_bytes(&timing_msg, 1);

    s_next_pulse++;
    if (s_evt_enabled) {
        schedule_next_event(g_link->captureAppSessionState());
    } else {
        s_next_pulse_valid = false;
        set_buzzer_state(false);
    }
}

// Main Link Synchronization Logic (called from tickTask)
// pending_realign: set when a peer joins or play-state begins; the actual MIDI
// Start+SPP is held until the next 16-bar phrase boundary so all gear begins the
// phrase together rather than jerking in mid-phrase.
void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                        bool& was_playing,
                        const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    static bool s_pending_realign = false;   // a Start+SPP is owed at the next phrase boundary
    static bool s_transport_running = false; // whether external gear currently believes it is playing

    // Broadcast our Link-clock micros every tick (~20ms) so multicast-only peers
    // (the Pi looper, which can't receive unicast measurement) can lock phase.
    broadcast_link_clock(time.count());
    // Mirror direction: broadcast our current Link timeline so the looper adopts our
    // tempo when WE change it (bidirectional Link tempo control).
    broadcast_ticker_timeline(state, time.count());

    // Apply a pending looper tempo-set (LTMP). Done here on the Link task so the
    // session-state capture/commit is single-owner (committing from the listener
    // task would race this one). setTempo at the current clock keeps beat continuity.
    if ((s_tempoReqPending || s_phaseReqPending) && g_link) {
        auto ss = g_link->captureAppSessionState();
        if (s_tempoReqPending) {
            s_tempoReqPending = false;
            ss.setTempo(s_tempoReqBpm, g_link->clock().micros());
            ESP_LOGI(TAG_LINK, "Tempo set to %.2f BPM by looper (LTMP)", s_tempoReqBpm);
        }
        if (s_phaseReqPending) {
            s_phaseReqPending = false;
            // Force beat 0 at the loop downbeat (already in our clock) so the whole
            // group's phrase aligns to the loop -- the looper sets tempo AND phrase.
            ss.forceBeatAtTime(0.0, std::chrono::microseconds(s_phaseReqBeat0us), s_phaseReqQuantum);
            ESP_LOGI(TAG_LINK, "Phase forced to loop downbeat (q=%.2f)", s_phaseReqQuantum);
        }
        g_link->commitAppSessionState(ss);
    }

    // Check peer status & force start timeout. Hold force-start longer (8s) than the
    // original 5s so two co-booting devices have time to discover each other over WiFi
    // before either free-runs; this avoids both emitting transport at independent phases
    // and then snapping when they peer.
    bool is_connected = g_link->numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= 8000000)) {
        force_start = true;
        ESP_LOGW(TAG_LINK, "No Link peers found for 8s, forcing start.");
    }

    // Witness whether Link's own send() hook is being called (i.e. Link is broadcasting
    // discovery). Logged from this normal task context, ~every 5s, so it is visible even
    // though the hook itself runs on Link's pinned asio thread.
    extern volatile uint32_t g_link_send_hook_calls;
    extern volatile uint32_t g_link_send_last_dstip;
    extern volatile uint32_t g_link_send_last_dport;
    extern volatile uint32_t g_link_pump_calls;
    extern volatile uint32_t g_link_scan_calls;
    extern volatile uint32_t g_link_scan_last_ip;
    extern volatile uint32_t g_link_scan_last_count;
    extern volatile uint32_t g_link_gw_init_attempts;
    extern volatile uint32_t g_link_gw_init_ok;
    extern volatile uint32_t g_link_gw_init_fail;
    static int64_t s_hookLogAt = 0;
    int64_t nowH = esp_timer_get_time();
    if (nowH - s_hookLogAt > 5000000) {
        s_hookLogAt = nowH;
        uint32_t sip = g_link_scan_last_ip;
        ESP_LOGI(TAG_LINK, "Link diag: scanIP=%u.%u.%u.%u gw(try=%u ok=%u fail=%u) send-hook=%u peers=%d",
                 sip & 0xff, (sip >> 8) & 0xff, (sip >> 16) & 0xff, (sip >> 24) & 0xff,
                 g_link_gw_init_attempts, g_link_gw_init_ok, g_link_gw_init_fail,
                 g_link_send_hook_calls, g_link->numPeers());
        const MetroStats ms = metro_stats();
        ESP_LOGI(TAG_LINK, "Metro: fired=%u late last=%lldus worst=%lldus mean=%.0fus rms=%.0fus",
                 (unsigned)ms.fired, (long long)ms.last, (long long)ms.worst, ms.mean, ms.rms);
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
            // Re-anchor the emitted pulse train on the live beat, so peering neither
            // bursts the pulses that elapsed before it nor leaves the clock behind it.
            s_next_pulse_valid = false;
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
    const bool crossedPhraseBoundary = quantumInfo.crossedPhraseBoundary;

    // Metronome and MIDI Sync Logic. The click and the 24 ppqn clock are no longer
    // emitted here: this tick would only notice a beat after it had passed. Enabling
    // arms the hardware-scheduled pulse train (link_event_cb) instead; this tick keeps
    // the phrase-aligned transport work, which is not microsecond-critical.
    const bool clockEnabled = is_connected || force_start;
    if (clockEnabled != s_evt_enabled) {
        s_evt_enabled = clockEnabled;
        s_next_pulse_valid = false;
        s_evt_armed = false;
        if (clockEnabled) {
            metronome_accent_for_beat(std::floor(sessionBeat) + 1.0, s_next_click_freq, s_next_click_ms);
            prime_buzzer_freq(s_next_click_freq);
            schedule_next_event(state);
            ESP_LOGI(TAG_LINK, "Timeline events armed at beat %.1f", sessionBeat);
        } else {
            esp_timer_stop(s_evt_timer);
            set_buzzer_state(false);
            ESP_LOGI(TAG_LINK, "Timeline events stopped");
        }
    }

    // Watchdog: an alarm that never got re-armed (a failed start, a callback that could
    // not run) would stop the clock silently, so re-arm from here once a pulse is really
    // overdue -- the threshold is far longer than the fire/re-arm window it must not race.
    if (s_evt_enabled && !s_evt_armed) {
        const double bpm = state.tempo();
        const int64_t pulseUs = (int64_t)(2.5e6 / (bpm > 1.0 ? bpm : 120.0));
        if (esp_timer_get_time() - s_last_fire_us > pulseUs * 3 + 5000)
            schedule_next_event(state);
    }

    if (clockEnabled) {
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

    }
}