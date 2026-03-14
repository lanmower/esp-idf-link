#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <esp_err.h>
#include <esp_wifi.h>
#include <stdbool.h>

esp_err_t wifi_config_init();
bool      wifi_scan_for_ssid(const char* ssid);
esp_err_t wifi_connect_sta(const char* ssid, const char* password);
esp_err_t wifi_start_link_ap(const char* ssid);
void      wifi_join_link_multicast();
bool      wifi_is_connected();
bool      wifi_is_ap_active();

#endif
