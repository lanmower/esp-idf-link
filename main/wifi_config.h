#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <cstddef>
#include <esp_err.h>
#include <esp_wifi.h>
#include <stdbool.h>
#include <stdint.h>

constexpr std::size_t kMacLen = 6;

#define LINK_MCAST_OCTET_A 224
#define LINK_MCAST_OCTET_B 76
#define LINK_MCAST_OCTET_C 78
#define LINK_MCAST_OCTET_D 75

#define LINK_MCAST_OCTET_LITERAL_(octet) #octet
#define LINK_MCAST_OCTET_LITERAL(octet)  LINK_MCAST_OCTET_LITERAL_(octet)

#define LINK_DISCOVERY_MULTICAST_ADDR \
    LINK_MCAST_OCTET_LITERAL(LINK_MCAST_OCTET_A) "." \
    LINK_MCAST_OCTET_LITERAL(LINK_MCAST_OCTET_B) "." \
    LINK_MCAST_OCTET_LITERAL(LINK_MCAST_OCTET_C) "." \
    LINK_MCAST_OCTET_LITERAL(LINK_MCAST_OCTET_D)

#define LINK_DISCOVERY_MULTICAST_PORT 20808

constexpr uint8_t kScanAllChannels = 0;
constexpr uint8_t kTickerChannel   = 6;

esp_err_t wifi_config_init();
bool      wifi_scan_for_ssid(const char* ssid);
int       wifi_scan_best_bssid(const char* ssid, uint8_t out_best_bssid[kMacLen],
                               uint8_t channel = kScanAllChannels);
esp_err_t wifi_connect_sta(const char* ssid, const char* password);
esp_err_t wifi_start_link_ap(const char* ssid);
void      wifi_join_link_multicast();
void      wifi_start_link_relay();
bool      wifi_is_connected();
bool      wifi_is_ap_active();
void      wifi_get_sta_mac(uint8_t out_mac[kMacLen]);
void      wifi_start_supervisor(const char* ssid);

#endif
