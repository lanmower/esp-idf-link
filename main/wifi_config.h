#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <esp_err.h>
#include <esp_wifi.h>
#include <stdbool.h>
#include <stdint.h>

esp_err_t wifi_config_init();
bool      wifi_scan_for_ssid(const char* ssid);
// Scan for `ssid` and, if found, report the lowest (numerically smallest) BSSID seen
// in `out_best_bssid` (6 bytes). Returns the match count. Used for the dual-host
// tie-break: the device with the lowest STA MAC hosts; others join the lowest BSSID.
int       wifi_scan_best_bssid(const char* ssid, uint8_t out_best_bssid[6]);
esp_err_t wifi_connect_sta(const char* ssid, const char* password);
esp_err_t wifi_start_link_ap(const char* ssid);
void      wifi_join_link_multicast();
void      wifi_start_link_relay();
// Station-side Link bridge: unicast-forward the station's Link multicast to the AP
// gateway, because the SoftAP does not carry multicast host<->station. Call after a
// successful STA join.
void      wifi_start_station_bridge();
bool      wifi_is_connected();
bool      wifi_is_ap_active();
// Read this device's STA MAC (used as the tie-break key).
void      wifi_get_sta_mac(uint8_t out_mac[6]);
// Launch the self-healing supervisor task: monitors the link, reconnects a dropped
// STA, and re-hosts the 'ticker' AP if the host disappears. Owns the lifecycle after
// the initial scan/host/join decision in app_main.
void      wifi_start_supervisor(const char* ssid);

#endif
