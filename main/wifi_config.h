#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdint.h>
#include <esp_err.h>
#include <esp_wifi.h>

typedef struct {
    char ssid[32];
    char password[64];
} WiFiCredentials;

esp_err_t wifi_config_init();
esp_err_t wifi_config_get_credentials(WiFiCredentials* creds);
esp_err_t wifi_config_set_credentials(const char* ssid, const char* password);
esp_err_t wifi_config_connect();
bool wifi_is_connected();
esp_err_t wifi_start_hotspot(const char* ssid, const char* password);
esp_err_t wifi_stop_hotspot();

#endif
