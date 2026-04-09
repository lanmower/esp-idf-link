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
static esp_netif_t* g_sta_netif = NULL;
static esp_netif_t* g_ap_netif = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        ESP_LOGW(TAG, "STA disconnected");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_connected = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP started");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* ev = (wifi_event_ap_staconnected_t*)data;
        ESP_LOGI(TAG, "Client joined: " MACSTR, MAC2STR(ev->mac));
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

bool wifi_scan_for_ssid(const char* ssid) {
    g_sta_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_scan_config_t scan_cfg = {};
    scan_cfg.ssid = (uint8_t*)ssid;
    scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_cfg.scan_time.active.min = 100;
    scan_cfg.scan_time.active.max = 300;
    esp_wifi_scan_start(&scan_cfg, true);

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    ESP_LOGI(TAG, "Scan found %u APs matching '%s'", count, ssid);

    if (count == 0) {
        esp_wifi_stop();
        esp_netif_destroy(g_sta_netif);
        g_sta_netif = NULL;
        return false;
    }
    return true;
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
    g_ap_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t cfg = {};
    strncpy((char*)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid) - 1);
    cfg.ap.ssid_len      = strlen(ssid);
    cfg.ap.channel       = 6;
    cfg.ap.authmode      = WIFI_AUTH_OPEN;
    cfg.ap.max_connection = 8;
    cfg.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip = {};
    ip.ip.addr      = ipaddr_addr("192.168.4.1");
    ip.gw.addr      = ipaddr_addr("192.168.4.1");
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(g_ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(g_ap_netif, &ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(g_ap_netif));

    g_ap_active = true;
    ESP_LOGI(TAG, "Link AP '%s' on ch6, 192.168.4.1, max 8 clients", ssid);
    igmp_join_link(g_ap_netif);
    return ESP_OK;
}

void wifi_join_link_multicast() {
    igmp_join_link(g_sta_netif);
}

// Relay Ableton Link multicast (224.76.78.75:20808) between AP clients.
// Receives on the multicast group, then re-sends each packet back to the
// multicast group with the original source IP/port preserved via a raw socket
// (IP_HDRINCL), so Ableton Link peer-to-peer connections work correctly.
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

    // Send socket: raw, so we can set source IP = original sender
    int ss = socket(AF_INET, SOCK_RAW, IPPROTO_RAW);
    if (ss < 0) {
        ESP_LOGE(RELAY_TAG, "raw socket failed — relay disabled");
        close(rs); vTaskDelete(NULL); return;
    }
    setsockopt(ss, IPPROTO_IP, IP_HDRINCL, &one, sizeof(one));
    // Send on AP interface
    struct in_addr ap_if = {};
    inet_aton(AP_ADDR, &ap_if);
    setsockopt(ss, IPPROTO_IP, IP_MULTICAST_IF, &ap_if, sizeof(ap_if));

    uint32_t ap_ip = inet_addr(AP_ADDR);
    uint32_t mcast_ip = inet_addr(MCAST_ADDR);

    struct sockaddr_in dst = {};
    dst.sin_family      = AF_INET;
    dst.sin_port        = htons(LINK_PORT);
    dst.sin_addr.s_addr = mcast_ip;

    static uint8_t payload[1500];
    // IP header (20) + UDP header (8)
    static uint8_t pkt[1528];

    ESP_LOGI(RELAY_TAG, "Ableton Link relay running on %s:%u", MCAST_ADDR, LINK_PORT);

    for (;;) {
        struct sockaddr_in src = {};
        socklen_t sl = sizeof(src);
        int n = recvfrom(rs, payload, sizeof(payload), 0, (struct sockaddr*)&src, &sl);
        if (n <= 0) continue;

        // Skip packets originating from ourselves
        if (src.sin_addr.s_addr == ap_ip) continue;

        // Build raw IPv4 + UDP packet with original source IP
        struct {
            uint8_t  ihl_ver, tos;
            uint16_t len, id, off;
            uint8_t  ttl, proto;
            uint16_t cksum;
            uint32_t src_ip, dst_ip;
        } __attribute__((packed)) *ih = (void*)pkt;

        ih->ihl_ver = 0x45;
        ih->tos     = 0;
        ih->len     = htons((uint16_t)(20 + 8 + n));
        ih->id      = 0;
        ih->off     = 0;
        ih->ttl     = 4;
        ih->proto   = IPPROTO_UDP;
        ih->cksum   = 0;
        ih->src_ip  = src.sin_addr.s_addr;
        ih->dst_ip  = mcast_ip;

        // IP header checksum
        uint32_t ck = 0;
        uint16_t* w = (uint16_t*)pkt;
        for (int i = 0; i < 10; i++) ck += ntohs(w[i]);
        while (ck >> 16) ck = (ck & 0xFFFF) + (ck >> 16);
        ih->cksum = htons((uint16_t)~ck);

        // UDP header
        struct {
            uint16_t sport, dport, len, cksum;
        } __attribute__((packed)) *uh = (void*)(pkt + 20);
        uh->sport = src.sin_port;
        uh->dport = htons(LINK_PORT);
        uh->len   = htons((uint16_t)(8 + n));
        uh->cksum = 0; // UDP checksum optional for IPv4

        memcpy(pkt + 28, payload, n);
        sendto(ss, pkt, 28 + n, 0, (struct sockaddr*)&dst, sizeof(dst));
    }
}

void wifi_start_link_relay() {
    xTaskCreate(link_multicast_relay_task, "link_relay", 4096, NULL, 5, NULL);
}

bool wifi_is_connected() { return g_wifi_connected; }
bool wifi_is_ap_active()  { return g_ap_active; }
