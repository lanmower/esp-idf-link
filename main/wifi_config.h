#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <esp_err.h>
#include <esp_wifi.h>
#include <stdbool.h>
#include <stdint.h>

esp_err_t wifi_config_init();
bool      wifi_scan_for_ssid(const char* ssid);
int       wifi_scan_best_bssid(const char* ssid, uint8_t out_best_bssid[6]);
esp_err_t wifi_connect_sta(const char* ssid, const char* password);
esp_err_t wifi_start_link_ap(const char* ssid);
void      wifi_join_link_multicast();
void      wifi_start_link_relay();
bool      wifi_is_connected();
bool      wifi_is_ap_active();
void      wifi_get_sta_mac(uint8_t out_mac[6]);
void      wifi_start_supervisor(const char* ssid);

#endif
