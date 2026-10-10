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

static const ip4_addr_t LINK_DISCOVERY_MULTICAST_GROUP = { .addr = PP_HTONL(LWIP_MAKEU32(224, 76, 78, 75)) };

static void igmp_join_link(esp_netif_t* netif) {
    if (!netif) return;
    struct netif* lwip_netif = (struct netif*)esp_netif_get_netif_impl(netif);
    if (!lwip_netif) return;
    err_t err = igmp_joingroup_netif(lwip_netif, &LINK_DISCOVERY_MULTICAST_GROUP);
    ESP_LOGI("WIFI", "IGMP join 224.76.78.75: %s", err == ERR_OK ? "ok" : "failed");
}

static const char* TAG = "WIFI";
static bool g_wifi_connected = false;
static bool g_ap_active = false;
static volatile bool g_sta_wanted = false;
static volatile int g_ap_client_count = 0;

static bool ap_has_associated_stations() {
    return g_ap_client_count > 0;
}

#define MAX_AP_STA_IPS 8
static const uint32_t AP_STA_IP_SLOT_EMPTY = 0;
static volatile uint32_t g_ap_sta_ips[MAX_AP_STA_IPS] = {0};

static void remember_ap_sta_ip(uint32_t ip) {
    for (int i = 0; i < MAX_AP_STA_IPS; i++) if (g_ap_sta_ips[i] == ip) return;
    for (int i = 0; i < MAX_AP_STA_IPS; i++) if (g_ap_sta_ips[i] == AP_STA_IP_SLOT_EMPTY) { g_ap_sta_ips[i] = ip; return; }
}

static esp_netif_t* g_sta_netif = NULL;
static esp_netif_t* g_ap_netif = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        wifi_event_sta_disconnected_t* ev = (wifi_event_sta_disconnected_t*)data;
        ESP_LOGW(TAG, "STA disconnected (reason=%d)", ev ? ev->reason : -1);
        if (g_sta_wanted && !g_ap_active) esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_connected = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP started");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* ev = (wifi_event_ap_staconnected_t*)data;
        g_ap_client_count = g_ap_client_count + 1;
        ESP_LOGI(TAG, "Client joined: " MACSTR " (clients=%d)", MAC2STR(ev->mac), g_ap_client_count);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* ev = (wifi_event_ap_stadisconnected_t*)data;
        if (g_ap_client_count > 0) g_ap_client_count = g_ap_client_count - 1;
        if (g_ap_client_count == 0) {
            for (int i = 0; i < MAX_AP_STA_IPS; i++) g_ap_sta_ips[i] = AP_STA_IP_SLOT_EMPTY;
        }
        ESP_LOGI(TAG, "Client left: " MACSTR " (clients=%d)", MAC2STR(ev->mac), g_ap_client_count);
    } else if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t* ev = (ip_event_ap_staipassigned_t*)data;
        uint32_t ip = ev->ip.addr;
        ESP_LOGI(TAG, "Station got IP: " IPSTR, IP2STR(&ev->ip));
        remember_ap_sta_ip(ip);
    }
}

esp_err_t wifi_config_init() {
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&cfg);
}

static bool g_wifi_started = false;

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
        const bool start_tolerated = (err == ESP_OK || err == ESP_ERR_WIFI_CONN);
        if (!start_tolerated) {
            ESP_LOGW(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
        }
        g_wifi_started = true;
        esp_err_t pserr = esp_wifi_set_ps(WIFI_PS_NONE);
        if (pserr != ESP_OK) ESP_LOGW(TAG, "esp_wifi_set_ps: %s", esp_err_to_name(pserr));
        else ESP_LOGI(TAG, "WiFi power-save disabled (reliable multicast)");
    }
}

void wifi_get_sta_mac(uint8_t out_mac[6]) {
    esp_read_mac(out_mac, ESP_MAC_WIFI_STA);
}

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
    g_sta_wanted = true;
    return esp_wifi_connect();
}

esp_err_t wifi_start_link_ap(const char* ssid) {
    g_sta_wanted = false;
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
    g_ap_client_count = 0;
    ESP_LOGI(TAG, "Link AP '%s' on ch6, 192.168.4.1, max 8 clients", ssid);
    igmp_join_link(g_ap_netif);
    return ESP_OK;
}

void wifi_join_link_multicast() {
    igmp_join_link(g_sta_netif);
}

volatile uint32_t g_link_send_hook_calls = 0;
volatile uint32_t g_link_send_last_dstip = 0;
volatile uint32_t g_link_send_last_dport = 0;
volatile uint32_t g_link_pump_calls = 0;
volatile uint32_t g_link_scan_calls = 0;
volatile uint32_t g_link_scan_last_ip = 0;
volatile uint32_t g_link_scan_last_count = 0;
volatile uint32_t g_link_gw_init_attempts = 0;
volatile uint32_t g_link_gw_init_ok = 0;
volatile uint32_t g_link_gw_init_fail = 0;

static bool is_ipv4_multicast_dst(unsigned dstip) {
    const uint8_t first_octet = dstip & 0xff;
    return first_octet >= 224 && first_octet <= 239;
}

extern "C" void wifi_link_multicast_forward(const uint8_t* data, unsigned len, unsigned dport, unsigned dstip) {
    g_link_send_hook_calls = g_link_send_hook_calls + 1;
    g_link_send_last_dstip = dstip;
    g_link_send_last_dport = dport;
    if (!is_ipv4_multicast_dst(dstip)) return;

    static int s_fwd_sock = -1;
    if (s_fwd_sock < 0) {
        s_fwd_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_fwd_sock < 0) return;
    }
    struct sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons((uint16_t)dport);

    static int s_fwd_log = 0;
    if (g_ap_active) {
        int sent = 0;
        for (int i = 0; i < MAX_AP_STA_IPS; i++) {
            uint32_t ip = g_ap_sta_ips[i];
            if (ip == AP_STA_IP_SLOT_EMPTY) continue;
            dst.sin_addr.s_addr = ip;
            sendto(s_fwd_sock, data, len, 0, (struct sockaddr*)&dst, sizeof(dst));
            sent++;
        }
        if (s_fwd_log < 8) { ESP_LOGI(TAG, "LINK fwd(AP) %u bytes -> %d station(s)", len, sent); s_fwd_log++; }
    } else if (g_sta_netif) {
        esp_netif_ip_info_t staip = {};
        if (esp_netif_get_ip_info(g_sta_netif, &staip) == ESP_OK && staip.gw.addr != 0) {
            dst.sin_addr.s_addr = staip.gw.addr;
            sendto(s_fwd_sock, data, len, 0, (struct sockaddr*)&dst, sizeof(dst));
            if (s_fwd_log < 8) { ESP_LOGI(TAG, "LINK fwd(STA) %u bytes -> gateway", len); s_fwd_log++; }
        }
    }
}

static void link_multicast_relay_task(void*) {
    static const char* RELAY_TAG = "LINK_RELAY";
    static const char* MCAST_ADDR = "224.76.78.75";
    static const char* AP_ADDR    = "192.168.4.1";
    static const uint16_t LINK_PORT = 20808;

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

    LOCK_TCPIP_CORE();
    struct raw_pcb* rpcb = raw_new(IPPROTO_UDP);
    UNLOCK_TCPIP_CORE();
    if (!rpcb) {
        ESP_LOGE(RELAY_TAG, "raw_new failed -- relay disabled");
        close(rs); vTaskDelete(NULL); return;
    }

    uint32_t ap_ip    = inet_addr(AP_ADDR);
    uint32_t mcast_ip = inet_addr(MCAST_ADDR);

    static const uint16_t MAX_UDP_PAYLOAD_BYTES = 1500 - 20 - 8;
    static uint8_t payload[MAX_UDP_PAYLOAD_BYTES];

    int rx_log = 0;

    ESP_LOGI(RELAY_TAG, "Ableton Link relay running on %s:%u", MCAST_ADDR, LINK_PORT);

    for (;;) {
        struct sockaddr_in src = {};
        socklen_t sl = sizeof(src);
        int n = recvfrom(rs, payload, sizeof(payload), 0, (struct sockaddr*)&src, &sl);
        if (n <= 0) continue;

        bool from_self = (src.sin_addr.s_addr == ap_ip);

        if (!from_self && src.sin_addr.s_addr != 0) {
            remember_ap_sta_ip(src.sin_addr.s_addr);
        }

        if (rx_log < 10) {
            ESP_LOGI(RELAY_TAG, "rx from %s:%u (%d bytes)%s",
                     inet_ntoa(src.sin_addr), ntohs(src.sin_port), n,
                     from_self ? " [self]" : "");
            rx_log++;
        }

        struct pbuf* p = pbuf_alloc(PBUF_RAW, (uint16_t)(8 + n), PBUF_RAM);
        if (!p) continue;

        uint8_t* buf = (uint8_t*)p->payload;
        uint16_t sport = src.sin_port;
        uint16_t dport = PP_HTONS(LINK_PORT);
        uint16_t ulen  = lwip_htons((uint16_t)(8 + n));
        const uint16_t udp_checksum_omitted = 0;
        memcpy(buf + 0, &sport, 2);
        memcpy(buf + 2, &dport, 2);
        memcpy(buf + 4, &ulen,  2);
        memcpy(buf + 6, &udp_checksum_omitted, 2);
        memcpy(buf + 8, payload, n);

        ip_addr_t src_addr, dst_addr;
        ip_addr_set_ip4_u32(&src_addr, src.sin_addr.s_addr);
        ip_addr_set_ip4_u32(&dst_addr, mcast_ip);

        uint32_t sta_ips[MAX_AP_STA_IPS];
        int sta_ip_count = 0;
        for (int i = 0; i < MAX_AP_STA_IPS; i++) {
            uint32_t sta_ip = g_ap_sta_ips[i];
            if (sta_ip == AP_STA_IP_SLOT_EMPTY || sta_ip == src.sin_addr.s_addr) continue;
            sta_ips[sta_ip_count++] = sta_ip;
        }

        LOCK_TCPIP_CORE();
        struct netif* ap_lwip = (struct netif*)esp_netif_get_netif_impl(g_ap_netif);
        if (ap_lwip) {
            if (!from_self) {
                raw_sendto_if_src(rpcb, p, &dst_addr, ap_lwip, &src_addr);
                ip_addr_t ap_addr;
                ip_addr_set_ip4_u32(&ap_addr, ap_ip);
                raw_sendto_if_src(rpcb, p, &ap_addr, ap_lwip, &src_addr);
            }
            for (int i = 0; i < sta_ip_count; i++) {
                ip_addr_t sta_addr;
                ip_addr_set_ip4_u32(&sta_addr, sta_ips[i]);
                raw_sendto_if_src(rpcb, p, &sta_addr, ap_lwip, &src_addr);
            }
        }
        UNLOCK_TCPIP_CORE();

        pbuf_free(p);
    }
}

void wifi_start_link_relay() {
    xTaskCreate(link_multicast_relay_task, "link_relay", 4096, NULL, 5, NULL);
}

static void wifi_supervisor_task(void* arg) {
    const char* ssid = (const char*)arg;
    const int RECONNECT_TRIES = 6;
    const int IGMP_REASSERT_TICKS = 5;
    uint8_t my_mac[6];
    wifi_get_sta_mac(my_mac);

    int sta_down_count = 0;
    int igmp_reassert_ticks_left = IGMP_REASSERT_TICKS;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (!g_ap_active) {
            if (g_wifi_connected) {
                sta_down_count = 0;
                if (igmp_reassert_ticks_left > 0) {
                    igmp_join_link(g_sta_netif);
                    igmp_reassert_ticks_left--;
                }
                continue;
            }
            igmp_reassert_ticks_left = IGMP_REASSERT_TICKS;
            sta_down_count++;
            if (sta_down_count <= RECONNECT_TRIES) {
                ESP_LOGW(TAG, "STA down (%d/%d) -- reconnecting", sta_down_count, RECONNECT_TRIES);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "STA down past %d tries -- host '%s' disappeared, re-hosting", RECONNECT_TRIES, ssid);
                g_sta_wanted = false;
                esp_wifi_disconnect();
                esp_wifi_stop();
                g_wifi_started = false;
                if (g_sta_netif) { esp_netif_destroy(g_sta_netif); g_sta_netif = NULL; }
                wifi_start_link_ap(ssid);
                wifi_start_link_relay();
                sta_down_count = 0;
            }
        } else {
            if (ap_has_associated_stations()) {
                continue;
            }
            uint8_t best[6];
            int matches = wifi_scan_best_bssid(ssid, best);
            if (matches > 0 && memcmp(best, my_mac, 6) < 0) {
                ESP_LOGW(TAG, "Lost dual-host tie-break (lower BSSID seen) -- dropping AP, joining");
                esp_wifi_stop();
                g_wifi_started = false;
                if (g_ap_netif) { esp_netif_destroy(g_ap_netif); g_ap_netif = NULL; }
                g_ap_active = false;
                ensure_sta_started();
                wifi_connect_sta(ssid, "");
            }
        }
    }
}

void wifi_start_supervisor(const char* ssid) {
    xTaskCreate(wifi_supervisor_task, "wifi_super", 4096, (void*)ssid, 4, NULL);
}

bool wifi_is_connected() { return g_wifi_connected; }
bool wifi_is_ap_active()  { return g_ap_active; }
