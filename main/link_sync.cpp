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
#include "wifi_config.h"

static constexpr int     MIDI_PULSES_PER_QUARTER_NOTE = 24;
static constexpr int     MIDI_SPP_UNITS_PER_BEAT      = 4;
static constexpr int     MIDI_SPP_UNITS_MASK          = 0x3FFF;
static constexpr int     MIDI_DATA_BYTE_MASK          = 0x7F;
static constexpr int     MIDI_DATA_BYTE_SHIFT         = 7;
static constexpr int     MIDI_WIRE_CHANNEL_COUNT      = 16;
static constexpr double  LINK_TEMPO_MIN_BPM           = 20.0;
static constexpr double  LINK_TEMPO_MAX_BPM           = 999.0;
static constexpr double  FALLBACK_TEMPO_BPM           = 120.0;
static constexpr double  MICROS_PER_MINUTE            = 60000000.0;
static constexpr double  MICROBEATS_PER_BEAT          = 1e6;
static constexpr int     MULTICAST_TTL                = 2;
static constexpr int64_t CLOCK_BROADCAST_MIN_INTERVAL_US    = 20000;
static constexpr int64_t TIMELINE_BROADCAST_MIN_INTERVAL_US = 100000;
static constexpr int64_t FORCE_START_AFTER_NO_PEERS_US      = 8000000;
static constexpr int     MAX_CATCHUP_PULSES_PER_SCHEDULE    = 1024;
static constexpr int     WATCHDOG_MISSED_PULSES       = 3;
static constexpr int64_t WATCHDOG_GRACE_US            = 5000;

#define LINK_PHASE_PORT 20810
static int s_clk_sock = -1;

static void broadcast_link_clock(int64_t linkMicros) {
    static int64_t s_lastSend = 0;
    int64_t nowUs = esp_timer_get_time();
    if (nowUs - s_lastSend < CLOCK_BROADCAST_MIN_INTERVAL_US) return;
    s_lastSend = nowUs;
    if (s_clk_sock < 0) {
        s_clk_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_clk_sock < 0) return;
        uint8_t ttl = MULTICAST_TTL;
        setsockopt(s_clk_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    }
    uint8_t pkt[12];
    memcpy(pkt, "LCLK", 4);
    memcpy(pkt + 4, &linkMicros, 8);
    struct sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(LINK_PHASE_PORT);
    dst.sin_addr.s_addr = inet_addr(LINK_DISCOVERY_MULTICAST_ADDR);

    esp_netif_t* nif = esp_netif_next_unsafe(NULL);
    for (; nif; nif = esp_netif_next_unsafe(nif)) {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(nif, &ipinfo) != ESP_OK || ipinfo.ip.addr == 0) continue;
        struct in_addr ifaddr; ifaddr.s_addr = ipinfo.ip.addr;
        setsockopt(s_clk_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof ifaddr);
        sendto(s_clk_sock, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst);
    }
}

#define LINK_TTMP_PORT 20812
static int s_ttmp_sock = -1;
static void broadcast_ticker_timeline(const ableton::Link::SessionState& state,
                                      int64_t linkMicros) {
    static int64_t s_lastSend = 0;
    if (linkMicros - s_lastSend < TIMELINE_BROADCAST_MIN_INTERVAL_US) return;
    s_lastSend = linkMicros;
    if (s_ttmp_sock < 0) {
        s_ttmp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_ttmp_sock < 0) return;
        uint8_t ttl = MULTICAST_TTL;
        setsockopt(s_ttmp_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    }
    double bpm = state.tempo();
    if (!(bpm >= LINK_TEMPO_MIN_BPM && bpm <= LINK_TEMPO_MAX_BPM)) return;
    int64_t mpb = (int64_t)(MICROS_PER_MINUTE / bpm + 0.5);
    double beats = state.beatAtTime(std::chrono::microseconds(linkMicros), LINK_QUANTUM);
    int64_t beatOriginUb = (int64_t)(beats * MICROBEATS_PER_BEAT);
    int64_t timeOrigin   = linkMicros;
    uint8_t pkt[28];
    memcpy(pkt,      "TTMP", 4);
    memcpy(pkt + 4,  &mpb, 8);
    memcpy(pkt + 12, &beatOriginUb, 8);
    memcpy(pkt + 20, &timeOrigin, 8);
    struct sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(LINK_TTMP_PORT);
    dst.sin_addr.s_addr = inet_addr(LINK_DISCOVERY_MULTICAST_ADDR);
    esp_netif_t* nif = esp_netif_next_unsafe(NULL);
    for (; nif; nif = esp_netif_next_unsafe(nif)) {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(nif, &ipinfo) != ESP_OK || ipinfo.ip.addr == 0) continue;
        struct in_addr ifaddr; ifaddr.s_addr = ipinfo.ip.addr;
        setsockopt(s_ttmp_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof ifaddr);
        sendto(s_ttmp_sock, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst);
    }
}

#define LINK_TEMPO_PORT 20811
static volatile bool   s_tempoReqPending = false;
static volatile double s_tempoReqBpm     = 0.0;
static volatile bool   s_phaseReqPending = false;
static volatile int64_t s_phaseReqBeat0us = 0;
static volatile double  s_phaseReqQuantum = 4.0;

#define LINK_STATUS_PORT 20812

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
        if (len > (int)sizeof reply) len = (int)sizeof reply;
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
    inet_aton(LINK_DISCOVERY_MULTICAST_ADDR, &mreq.imr_multiaddr);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(rs, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof mreq);
    uint8_t buf[64];
    for (;;) {
        int n = recvfrom(rs, buf, sizeof buf, 0, NULL, NULL);
        if (n >= 12 && memcmp(buf, "LTMP", 4) == 0) {
            int64_t mpb;
            memcpy(&mpb, buf + 4, 8);
            if (mpb > 0) {
                double bpm = MICROS_PER_MINUTE / (double)mpb;
                if (bpm >= LINK_TEMPO_MIN_BPM && bpm <= LINK_TEMPO_MAX_BPM) { s_tempoReqBpm = bpm; s_tempoReqPending = true; }
            }
            if (n >= 28) {
                int64_t beat0us, quantumUb;
                memcpy(&beat0us,  buf + 12, 8);
                memcpy(&quantumUb, buf + 20, 8);
                if (quantumUb > 0) {
                    s_phaseReqBeat0us = beat0us;
                    s_phaseReqQuantum = (double)quantumUb / MICROBEATS_PER_BEAT;
                    s_phaseReqPending = true;
                }
            }
        }
    }
}

void link_start_tempo_listener() {
    xTaskCreate(tempo_listener_task, "ltmp_rx", 4096, NULL, 5, NULL);
    xTaskCreate(status_responder_task, "link_status", 4096, NULL, 5, NULL);
}

static const char *TAG_LINK = "LINK_SYNC";

static int s_last_quantum_number = 0;
static int s_last_phrase_number = 0;
static bool s_quantum_baseline_captured = false;
static bool s_phrase_baseline_captured = false;
static gptimer_handle_t s_link_gptimer = nullptr;
static esp_timer_handle_t s_buzzer_off_timer = nullptr;

static esp_timer_handle_t s_evt_timer = nullptr;
static volatile bool      s_evt_armed   = false;
static volatile bool      s_evt_enabled = false;
static volatile int64_t   s_evt_due_us   = 0;
static volatile int64_t   s_last_fire_us = 0;
static int64_t            s_next_pulse       = 0;
static bool               s_next_pulse_valid = false;
static uint32_t           s_next_click_freq  = FREQ_NORMAL;
static int                s_next_click_ms    = LENGTH_NORMAL;

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
    if (g_link) {
        const auto st = g_link->captureAppSessionState();
        const double beatNow = st.beatAtTime(g_link->clock().micros(), LINK_QUANTUM);
        metronome_accent_for_beat(std::floor(beatNow) + 1.0, s_next_click_freq, s_next_click_ms);
    }
    prime_buzzer_freq(s_next_click_freq);
}

static bool IRAM_ATTR link_gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(user_data), LINK_TICK_NOTIFY_BIT, eSetBits, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

QuantumInfo detectQuantumBoundary(const ableton::Link::SessionState& state,
                                 const std::chrono::microseconds& time) {
    QuantumInfo info;

    info.sessionBeat = state.beatAtTime(time, LINK_QUANTUM);
    info.phaseWithinQuantum = state.phaseAtTime(time, LINK_QUANTUM);
    info.currentQuantumNumber = static_cast<int>(std::floor(info.sessionBeat / LINK_QUANTUM));
    info.beatInQuantum = static_cast<int>(std::floor(info.phaseWithinQuantum));
    info.beatFraction = info.phaseWithinQuantum - std::floor(info.phaseWithinQuantum);

    info.phaseWithinPhrase = state.phaseAtTime(time, PHRASE_BEATS);
    info.currentPhraseNumber = static_cast<int>(std::floor(info.sessionBeat / PHRASE_BEATS));

    if (!s_quantum_baseline_captured) {
        s_quantum_baseline_captured = true;
        s_last_quantum_number = info.currentQuantumNumber;
        info.crossedQuantumBoundary = false;
    } else if (info.currentQuantumNumber != s_last_quantum_number) {
        s_last_quantum_number = info.currentQuantumNumber;
        info.crossedQuantumBoundary = true;
        ESP_LOGI(TAG_LINK, "Quantum boundary %d, beat %.2f", info.currentQuantumNumber, info.sessionBeat);
    } else {
        info.crossedQuantumBoundary = false;
    }

    if (!s_phrase_baseline_captured) {
        s_phrase_baseline_captured = true;
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

void init_link_timer(TaskHandle_t task_handle) {
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_APB;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;
    timer_config.intr_priority = 3;
    timer_config.flags.intr_shared = 0;

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_link_gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = link_gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_link_gptimer, &cbs, task_handle));

    ESP_ERROR_CHECK(gptimer_set_raw_count(s_link_gptimer, 0));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = LINK_TICK_PERIOD,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = 1
        }
    };
    ESP_ERROR_CHECK(::gptimer_set_alarm_action(s_link_gptimer, &alarm_config));
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

static void send_midi_bytes(const uint8_t* buf, size_t len) {
    uart_write_bytes(MIDI_UART, (const char*)buf, len);
}

static void send_all_notes_off_all_channels() {
    for (uint8_t ch = 0; ch < MIDI_WIRE_CHANNEL_COUNT; ++ch) {
        const uint8_t cc[] = { (uint8_t)(MIDI_CC_CMD | ch), MIDI_CC_ALL_NOTES_OFF, 0 };
        send_midi_bytes(cc, sizeof(cc));
    }
}

static void send_song_position(double sessionBeat) {
    uint16_t spp_units = static_cast<uint16_t>(sessionBeat * MIDI_SPP_UNITS_PER_BEAT) & MIDI_SPP_UNITS_MASK;
    const uint8_t spp[] = { MIDI_SONG_POSITION_POINTER,
                            (uint8_t)(spp_units & MIDI_DATA_BYTE_MASK),
                            (uint8_t)((spp_units >> MIDI_DATA_BYTE_SHIFT) & MIDI_DATA_BYTE_MASK) };
    send_midi_bytes(spp, sizeof(spp));
}

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
    s_evt_due_us = dueEspUs;
    s_evt_armed  = true;
    esp_timer_stop(s_evt_timer);
    if (esp_timer_start_once(s_evt_timer, static_cast<uint64_t>(delta)) != ESP_OK) {
        s_evt_armed = false;
    }
}

static void schedule_next_event(const ableton::Link::SessionState& state) {
    const int64_t nowLink = g_link->clock().micros().count();
    const double beatNow = state.beatAtTime(std::chrono::microseconds(nowLink), LINK_QUANTUM);
    const int64_t dueNow = static_cast<int64_t>(std::floor(beatNow * MIDI_PULSES_PER_QUARTER_NOTE)) + 1;
    if (!s_next_pulse_valid || s_next_pulse < dueNow) {
        if (s_next_pulse_valid && dueNow - s_next_pulse > MIDI_CLOCK_RESYNC_THRESHOLD)
            ESP_LOGW(TAG_LINK, "clock %lld pulse(s) behind -- resyncing to beat %.2f (no burst)",
                     (long long)(dueNow - s_next_pulse), beatNow);
        s_next_pulse = dueNow;
        s_next_pulse_valid = true;
    }
    int64_t due = state.timeAtBeat((double)s_next_pulse / MIDI_PULSES_PER_QUARTER_NOTE, LINK_QUANTUM).count();
    for (int guard = 0; due < nowLink && guard < MAX_CATCHUP_PULSES_PER_SCHEDULE; guard++) {
        s_next_pulse++;
        due = state.timeAtBeat((double)s_next_pulse / MIDI_PULSES_PER_QUARTER_NOTE, LINK_QUANTUM).count();
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

    if ((s_next_pulse % MIDI_PULSES_PER_QUARTER_NOTE) == 0) {
        const double beat = (double)s_next_pulse / MIDI_PULSES_PER_QUARTER_NOTE;
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

void handle_link_sync(bool& was_connected, int64_t& start_wait_time, bool& force_start,
                        bool& was_playing,
                        const ableton::Link::SessionState& state, const std::chrono::microseconds& time)
{
    static bool s_pending_realign = false;
    static bool s_transport_running = false;

    broadcast_link_clock(time.count());
    broadcast_ticker_timeline(state, time.count());

    if ((s_tempoReqPending || s_phaseReqPending) && g_link) {
        auto ss = g_link->captureAppSessionState();
        if (s_tempoReqPending) {
            s_tempoReqPending = false;
            ss.setTempo(s_tempoReqBpm, g_link->clock().micros());
            ESP_LOGI(TAG_LINK, "Tempo set to %.2f BPM by looper (LTMP)", s_tempoReqBpm);
        }
        if (s_phaseReqPending) {
            s_phaseReqPending = false;
            ss.forceBeatAtTime(0.0, std::chrono::microseconds(s_phaseReqBeat0us), s_phaseReqQuantum);
            ESP_LOGI(TAG_LINK, "Phase forced to loop downbeat (q=%.2f)", s_phaseReqQuantum);
        }
        g_link->commitAppSessionState(ss);
    }

    bool is_connected = g_link->numPeers() > 0;
    if (!is_connected && !force_start && (esp_timer_get_time() - start_wait_time >= FORCE_START_AFTER_NO_PEERS_US)) {
        force_start = true;
        ESP_LOGW(TAG_LINK, "No Link peers found for 8s, forcing start.");
    }

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

    QuantumInfo quantumInfo = detectQuantumBoundary(state, time);

    if (is_connected != was_connected) {
        ESP_LOGI(TAG_LINK, "Link peers changed: %d", g_link->numPeers());
        if (is_connected) {
            ESP_LOGI(TAG_LINK, "Link connected -- beat=%.3f phase=%.3f quantum=%d phrase=%d",
                     quantumInfo.sessionBeat, quantumInfo.phaseWithinQuantum,
                     quantumInfo.currentQuantumNumber, quantumInfo.currentPhraseNumber);
            s_next_pulse_valid = false;
            s_pending_realign = true;
        } else {
            const uint8_t stop_msg[] = { MIDI_STOP };
            send_midi_bytes(stop_msg, 1);
            send_all_notes_off_all_channels();
            s_transport_running = false;
            ESP_LOGI(TAG_LINK, "Link peer lost -- sent Stop + All Notes Off.");
        }
        was_connected = is_connected;
    }

    const double sessionBeat = quantumInfo.sessionBeat;
    const bool crossedPhraseBoundary = quantumInfo.crossedPhraseBoundary;

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

    if (s_evt_enabled && !s_evt_armed) {
        const double bpm = state.tempo();
        const int64_t pulseUs = (int64_t)(MICROS_PER_MINUTE / MIDI_PULSES_PER_QUARTER_NOTE / (bpm > 1.0 ? bpm : FALLBACK_TEMPO_BPM));
        if (esp_timer_get_time() - s_last_fire_us > pulseUs * WATCHDOG_MISSED_PULSES + WATCHDOG_GRACE_US)
            schedule_next_event(state);
    }

    if (clockEnabled) {
        bool is_playing = state.isPlaying();

        if (was_playing != is_playing) {
            if (is_playing) {
                s_pending_realign = true;
            } else {
                const uint8_t msg = MIDI_STOP;
                send_midi_bytes(&msg, 1);
                send_all_notes_off_all_channels();
                s_transport_running = false;
                ESP_LOGI(TAG_LINK, "MIDI STOP at beat %.1f (+ All Notes Off)", sessionBeat);
            }
            was_playing = is_playing;
        }

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
