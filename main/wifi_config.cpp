#include "wifi_config.h"
#include <string.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h>
#include <lwip/ip_addr.h>
#include <lwip/igmp.h>
#include <lwip/netif.h>
#include <esp_mac.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/raw.h>
#include <lwip/pbuf.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Ableton Link peer discovery multicast group (224.76.78.75)
static const ip4_addr_t LINK_MCAST = { .addr = PP_HTONL(LWIP_MAKEU32(224, 76, 78, 75)) };

static void igmp_join_link(esp_netif_t* netif) {
    if (!netif) return;
    struct netif* lwip_netif = (struct netif*)esp_netif_get_netif_impl(netif);
    if (!lwip_netif) return;
    err_t err = igmp_joingroup_netif(lwip_netif, &LINK_MCAST);
    ESP_LOGI("WIFI", "IGMP join 224.76.78.75: %s", err == ERR_OK ? "ok" : "failed");
}

static const char* TAG = "WIFI";
static bool g_wifi_connected = false;
static bool g_ap_active = false;
// Number of stations currently associated to our SoftAP. While >0 we are an
// established host with real clients, so the supervisor must NOT run the periodic
// off-channel rescan -- on a single-radio ESP32 that scan tunes away from the AP
// channel and drops our own clients (the STA-flap that kept Link at 0 peers).
static volatile int g_ap_client_count = 0;
static esp_netif_t* g_sta_netif = NULL;
static esp_netif_t* g_ap_netif = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        wifi_event_sta_disconnected_t* ev = (wifi_event_sta_disconnected_t*)data;
        ESP_LOGW(TAG, "STA disconnected (reason=%d)", ev ? ev->reason : -1);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_connected = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP started");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* ev = (wifi_event_ap_staconnected_t*)data;
        g_ap_client_count = g_ap_client_count + 1; // explicit (volatile ++ is deprecated in C++20+)
        ESP_LOGI(TAG, "Client joined: " MACSTR " (clients=%d)", MAC2STR(ev->mac), g_ap_client_count);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* ev = (wifi_event_ap_stadisconnected_t*)data;
        if (g_ap_client_count > 0) g_ap_client_count = g_ap_client_count - 1; // explicit (volatile -- deprecated)
        ESP_LOGI(TAG, "Client left: " MACSTR " (clients=%d)", MAC2STR(ev->mac), g_ap_client_count);
    }
}

esp_err_t wifi_config_init() {
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&cfg);
}

static bool g_wifi_started = false;

// Ensure the driver is started in a SCANNABLE mode without tearing down an active AP.
// esp_wifi_scan_start fails unless the driver is started AND the mode includes STA, so:
//  - create the STA netif once (no leak across rescans);
//  - if the AP is up, use APSTA (keep hosting while we scan); else STA;
//  - call esp_wifi_start() exactly once (tracked), since the mode defaults to STA after
//    esp_wifi_init but the driver is NOT started yet -- the original bug skipped start()
//    because get_mode already returned STA, so the very first scan failed.
static void ensure_sta_started() {
    if (!g_sta_netif) {
        g_sta_netif = esp_netif_create_default_wifi_sta();
    }
    wifi_mode_t want = g_ap_active ? WIFI_MODE_APSTA : WIFI_MODE_STA;
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != want) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(want));
    }
    if (!g_wifi_started) {
        esp_err_t err = esp_wifi_start();
        if (err == ESP_OK || err == ESP_ERR_WIFI_CONN /* already started */) {
            g_wifi_started = true;
        } else {
            ESP_LOGW(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
            // Treat ESP_ERR_WIFI_NOT_STOPPED etc. as already-running.
            g_wifi_started = true;
        }
        // Disable modem power-save: with WIFI_PS_MIN_MODEM (the default) the radio
        // sleeps between DTIM beacons and multicast frames are buffered/dropped, which
        // delayed Ableton Link discovery (all multicast on 224.76.78.75:20808) by
        // minutes. These are USB/mains-powered devices, so PS_NONE has no downside.
        esp_err_t pserr = esp_wifi_set_ps(WIFI_PS_NONE);
        if (pserr != ESP_OK) ESP_LOGW(TAG, "esp_wifi_set_ps: %s", esp_err_to_name(pserr));
        else ESP_LOGI(TAG, "WiFi power-save disabled (reliable multicast)");
    }
}

void wifi_get_sta_mac(uint8_t out_mac[6]) {
    esp_read_mac(out_mac, ESP_MAC_WIFI_STA);
}

// Scan for `ssid`. esp_wifi_scan_start(.,true) blocks until the scan completes, so the
// AP count/records read immediately after are valid. With cfg.ssid set, the driver
// filters to matching SSIDs. Returns match count; fills out_best_bssid with the lowest
// BSSID among matches (or leaves it untouched on zero matches).
int wifi_scan_best_bssid(const char* ssid, uint8_t out_best_bssid[6]) {
    ensure_sta_started();

    wifi_scan_config_t scan_cfg = {};
    scan_cfg.ssid = (uint8_t*)ssid;
    scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_cfg.scan_time.active.min = 100;
    scan_cfg.scan_time.active.max = 300;
    esp_err_t serr = esp_wifi_scan_start(&scan_cfg, true);
    if (serr != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(serr));
        return 0;
    }

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        ESP_LOGI(TAG, "Scan: no '%s' AP found", ssid);
        return 0;
    }

    // Pull records to inspect BSSIDs for the tie-break.
    static const uint16_t MAX_RECORDS = 12;
    wifi_ap_record_t records[MAX_RECORDS];
    uint16_t n = (count < MAX_RECORDS) ? count : MAX_RECORDS;
    if (esp_wifi_scan_get_ap_records(&n, records) != ESP_OK) {
        ESP_LOGW(TAG, "scan_get_ap_records failed; treating as %u matches, no BSSID", count);
        return count;
    }

    int best = -1;
    for (uint16_t i = 0; i < n; ++i) {
        if (best < 0 || memcmp(records[i].bssid, records[best].bssid, 6) < 0) {
            best = i;
        }
    }
    if (best >= 0) {
        memcpy(out_best_bssid, records[best].bssid, 6);
        ESP_LOGI(TAG, "Scan: %u '%s' AP(s); lowest BSSID %02x:%02x:%02x:%02x:%02x:%02x",
                 n, ssid,
                 out_best_bssid[0], out_best_bssid[1], out_best_bssid[2],
                 out_best_bssid[3], out_best_bssid[4], out_best_bssid[5]);
    }
    return n;
}

bool wifi_scan_for_ssid(const char* ssid) {
    uint8_t bssid[6];
    return wifi_scan_best_bssid(ssid, bssid) > 0;
}

esp_err_t wifi_connect_sta(const char* ssid, const char* password) {
    wifi_config_t cfg = {};
    strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char*)cfg.sta.password, password, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = (strlen(password) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_LOGI(TAG, "Connecting STA to '%s'", ssid);
    return esp_wifi_connect();
}

esp_err_t wifi_start_link_ap(const char* ssid) {
    if (!g_ap_netif) {
        g_ap_netif = esp_netif_create_default_wifi_ap();
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t cfg = {};
    strncpy((char*)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid) - 1);
    cfg.ap.ssid_len      = strlen(ssid);
    cfg.ap.channel       = 6;
    cfg.ap.authmode      = WIFI_AUTH_OPEN;
    cfg.ap.max_connection = 8;
    cfg.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    // The driver may already be started (a prior scan called esp_wifi_start). Starting
    // again returns an error we tolerate; only the mode/config change matters here.
    if (!g_wifi_started) {
        esp_err_t serr = esp_wifi_start();
        if (serr != ESP_OK) ESP_LOGW(TAG, "esp_wifi_start (AP): %s", esp_err_to_name(serr));
        g_wifi_started = true;
    }

    esp_netif_ip_info_t ip = {};
    ip.ip.addr      = ipaddr_addr("192.168.4.1");
    ip.gw.addr      = ipaddr_addr("192.168.4.1");
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(g_ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(g_ap_netif, &ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(g_ap_netif));

    g_ap_active = true;
    g_ap_client_count = 0; // fresh AP has no clients yet; STACONNECTED events count up
    ESP_LOGI(TAG, "Link AP '%s' on ch6, 192.168.4.1, max 8 clients", ssid);
    igmp_join_link(g_ap_netif);
    return ESP_OK;
}

void wifi_join_link_multicast() {
    igmp_join_link(g_sta_netif);
}

// Relay Ableton Link multicast (224.76.78.75:20808) between AP clients.
// Uses lwip raw PCB (raw_sendto_if_src) to preserve the original sender's
// source IP, which Ableton Link requires for direct peer connections.
static void link_multicast_relay_task(void*) {
    static const char* RELAY_TAG = "LINK_RELAY";
    static const char* MCAST_ADDR = "224.76.78.75";
    static const char* AP_ADDR    = "192.168.4.1";
    static const uint16_t LINK_PORT = 20808;

    // Receive socket: join multicast on AP interface
    int rs = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rs < 0) { ESP_LOGE(RELAY_TAG, "recv socket failed"); vTaskDelete(NULL); return; }
    int one = 1;
    setsockopt(rs, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(LINK_PORT);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(rs, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(RELAY_TAG, "bind failed"); close(rs); vTaskDelete(NULL); return;
    }
    struct ip_mreq mreq = {};
    inet_aton(MCAST_ADDR, &mreq.imr_multiaddr);
    inet_aton(AP_ADDR,    &mreq.imr_interface);
    setsockopt(rs, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

    // Raw PCB for sending with original source IP preserved
    LOCK_TCPIP_CORE();
    struct raw_pcb* rpcb = raw_new(IPPROTO_UDP);
    UNLOCK_TCPIP_CORE();
    if (!rpcb) {
        ESP_LOGE(RELAY_TAG, "raw_new failed -- relay disabled");
        close(rs); vTaskDelete(NULL); return;
    }

    uint32_t ap_ip    = inet_addr(AP_ADDR);
    uint32_t mcast_ip = inet_addr(MCAST_ADDR);

    static uint8_t payload[1472]; // MTU(1500) - IP(20) - UDP(8)

    int rx_log = 0;

    ESP_LOGI(RELAY_TAG, "Ableton Link relay running on %s:%u", MCAST_ADDR, LINK_PORT);

    for (;;) {
        struct sockaddr_in src = {};
        socklen_t sl = sizeof(src);
        int n = recvfrom(rs, payload, sizeof(payload), 0, (struct sockaddr*)&src, &sl);
        if (n <= 0) continue;

        // Skip packets originating from ourselves
        if (src.sin_addr.s_addr == ap_ip) continue;

        // Diagnostic: log the first several received packets (src + size).
        if (rx_log < 10) {
            ESP_LOGI(RELAY_TAG, "rx from %s:%u (%d bytes)",
                     inet_ntoa(src.sin_addr), ntohs(src.sin_port), n);
            rx_log++;
        }

        // Build UDP header + payload in a pbuf
        struct pbuf* p = pbuf_alloc(PBUF_RAW, (uint16_t)(8 + n), PBUF_RAM);
        if (!p) continue;

        // UDP header (network byte order)
        uint8_t* buf = (uint8_t*)p->payload;
        uint16_t sport = src.sin_port;
        uint16_t dport = PP_HTONS(LINK_PORT);
        uint16_t ulen  = lwip_htons((uint16_t)(8 + n));
        memcpy(buf + 0, &sport, 2);
        memcpy(buf + 2, &dport, 2);
        memcpy(buf + 4, &ulen,  2);
        buf[6] = 0; buf[7] = 0; // checksum = 0 (optional for IPv4)
        memcpy(buf + 8, payload, n);

        ip_addr_t src_addr, dst_addr;
        ip_addr_set_ip4_u32(&src_addr, src.sin_addr.s_addr);
        ip_addr_set_ip4_u32(&dst_addr, mcast_ip);

        LOCK_TCPIP_CORE();
        struct netif* ap_lwip = (struct netif*)esp_netif_get_netif_impl(g_ap_netif);
        if (ap_lwip) {
            // (1) Re-emit to the multicast group so OTHER AP clients receive it.
            raw_sendto_if_src(rpcb, p, &dst_addr, ap_lwip, &src_addr);
            // (2) Also deliver a unicast copy to the AP's own IP so the HOST's own
            // Ableton Link socket receives it -- the raw multicast re-send above is not
            // looped back to local sockets by lwIP, which left the host at 0 peers when
            // only a client (e.g. a tablet) was present. Unicast to 192.168.4.1 preserves
            // the original source IP (required for Link's direct peer connect).
            ip_addr_t ap_addr;
            ip_addr_set_ip4_u32(&ap_addr, ap_ip);
            raw_sendto_if_src(rpcb, p, &ap_addr, ap_lwip, &src_addr);
        }
        UNLOCK_TCPIP_CORE();

        pbuf_free(p);
    }
}

void wifi_start_link_relay() {
    xTaskCreate(link_multicast_relay_task, "link_relay", 4096, NULL, 5, NULL);
}

// Self-healing supervisor. Runs forever. Two roles:
//  - STA role (g_ap_active == false): if the STA connection drops, retry esp_wifi_connect
//    a few times; if it stays down, fall back to hosting the AP so the mesh self-heals
//    when the previous host disappears.
//  - AP role (g_ap_active == true): periodically re-scan; if another 'ticker' AP exists
//    whose BSSID is LOWER than our own STA MAC, we lost the tie-break (both ended up
//    hosting) -- drop the AP and join the lower one so exactly one host remains.
static void wifi_supervisor_task(void* arg) {
    const char* ssid = (const char*)arg;
    const int RECONNECT_TRIES = 6;     // ~6 * 2s = 12s before re-hosting
    uint8_t my_mac[6];
    wifi_get_sta_mac(my_mac);

    int sta_down_count = 0;
    int join_refresh_left = 5; // re-assert the multicast join for a few ticks after connect

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (!g_ap_active) {
            // STA role
            if (g_wifi_connected) {
                sta_down_count = 0;
                // The single IGMP join at GOT_IP can race netif readiness so the
                // membership doesn't stick. Re-assert it for the first ~10s after a
                // connection so the 224.76.78.75 group membership reliably takes and
                // Ableton Link discovery converges in seconds, not minutes.
                if (join_refresh_left > 0) {
                    igmp_join_link(g_sta_netif);
                    join_refresh_left--;
                }
                continue;
            }
            join_refresh_left = 5; // schedule re-joins for when we next connect
            sta_down_count++;
            if (sta_down_count <= RECONNECT_TRIES) {
                ESP_LOGW(TAG, "STA down (%d/%d) -- reconnecting", sta_down_count, RECONNECT_TRIES);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "STA down past %d tries -- host '%s' disappeared, re-hosting", RECONNECT_TRIES, ssid);
                esp_wifi_disconnect();
                esp_wifi_stop();
                g_wifi_started = false; // driver stopped; next start() must actually run
                if (g_sta_netif) { esp_netif_destroy(g_sta_netif); g_sta_netif = NULL; }
                wifi_start_link_ap(ssid);
                wifi_start_link_relay();
                sta_down_count = 0;
            }
        } else {
            // AP role: detect a co-host with a lower BSSID and yield to it so
            // exactly one host remains. Combined with the MAC-ordered boot election
            // (higher MACs defer hosting) a persistent dual-host is rare and always
            // resolves to the lowest BSSID.
            //
            // CRITICAL: only rescan while we have NO associated stations. The scan
            // runs in APSTA and tunes the single radio off our AP channel; doing that
            // while a client is connected drops the client (STA-flap -> Link never
            // peers). An established host with >=1 client has already won the
            // election -- there is nothing to resolve, so stay put and keep the AP
            // rock-stable. If a co-host with no clients exists, one of the two has
            // zero clients and will still rescan and yield, so convergence holds.
            if (g_ap_client_count > 0) {
                continue;
            }
            uint8_t best[6];
            int matches = wifi_scan_best_bssid(ssid, best);
            // Our own AP shows up in the scan as our AP MAC (STA MAC + 1 on ESP32).
            // Only yield if a DIFFERENT, strictly-lower BSSID is present.
            if (matches > 0 && memcmp(best, my_mac, 6) < 0) {
                ESP_LOGW(TAG, "Lost dual-host tie-break (lower BSSID seen) -- dropping AP, joining");
                esp_wifi_stop();
                g_wifi_started = false; // driver stopped; ensure_sta_started must re-start it
                if (g_ap_netif) { esp_netif_destroy(g_ap_netif); g_ap_netif = NULL; }
                g_ap_active = false;
                ensure_sta_started();
                wifi_connect_sta(ssid, "");
            }
        }
    }
}

void wifi_start_supervisor(const char* ssid) {
    // ssid is a string literal in app_main; safe to pass by pointer.
    xTaskCreate(wifi_supervisor_task, "wifi_super", 4096, (void*)ssid, 4, NULL);
}

bool wifi_is_connected() { return g_wifi_connected; }
bool wifi_is_ap_active()  { return g_ap_active; }
