#include "wifi_config.h"
#include <string.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>
#include <lwip/ip_addr.h>

static const char* TAG = "WIFI_CONFIG";
static const char* NVS_NAMESPACE = "wifi_creds";
static bool g_wifi_connected = false;
static bool g_hotspot_active = false;
static esp_netif_t* g_ap_netif = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "WiFi station starting");
        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Connected to WiFi AP");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "Disconnected from WiFi AP");
            g_wifi_connected = false;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_connected = true;
    }
}

esp_err_t wifi_config_init() {
    ESP_LOGI(TAG, "Initializing WiFi config");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash needs cleanup, erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_netif_init();
    esp_event_loop_create_default();

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                       &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                       &wifi_event_handler, NULL, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    return ESP_OK;
}

esp_err_t wifi_config_get_credentials(WiFiCredentials* creds) {
    if (!creds) return ESP_ERR_INVALID_ARG;

    printf("[WIFI] Getting credentials from NVS...\n");
    fflush(stdout);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        printf("[WIFI] NVS namespace not found, using defaults\n");
        fflush(stdout);
        ESP_LOGW(TAG, "No WiFi credentials in NVS, using defaults");
        strncpy(creds->ssid, "Link-Device", sizeof(creds->ssid) - 1);
        strncpy(creds->password, "12345678", sizeof(creds->password) - 1);
        return ESP_OK;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t ssid_len = sizeof(creds->ssid);
    size_t pwd_len = sizeof(creds->password);

    ret = nvs_get_str(handle, "ssid", creds->ssid, &ssid_len);
    if (ret != ESP_OK) {
        printf("[WIFI] SSID not found in NVS (ret=%d), using default\n", ret);
        fflush(stdout);
        ESP_LOGW(TAG, "SSID not found, using default");
        strncpy(creds->ssid, "Link-Device", sizeof(creds->ssid) - 1);
    } else {
        printf("[WIFI] Got SSID from NVS: %s\n", creds->ssid);
        fflush(stdout);
    }

    ret = nvs_get_str(handle, "password", creds->password, &pwd_len);
    if (ret != ESP_OK) {
        printf("[WIFI] Password not found in NVS (ret=%d), using default\n", ret);
        fflush(stdout);
        ESP_LOGW(TAG, "Password not found, using default");
        strncpy(creds->password, "12345678", sizeof(creds->password) - 1);
    } else {
        printf("[WIFI] Got password from NVS: %s\n", creds->password);
        fflush(stdout);
    }

    printf("[WIFI] Final credentials - SSID: %s, Password: %s\n", creds->ssid, creds->password);
    fflush(stdout);

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t wifi_config_set_credentials(const char* ssid, const char* password) {
    if (!ssid || !password) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(handle, "ssid", ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SSID: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_set_str(handle, "password", password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set password: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    printf("[WIFI] Credentials saved to NVS: SSID=%s\n", ssid);
    fflush(stdout);
    ESP_LOGI(TAG, "WiFi credentials saved: SSID=%s", ssid);
    return ESP_OK;
}

esp_err_t wifi_config_connect() {
    WiFiCredentials creds;
    esp_err_t ret = wifi_config_get_credentials(&creds);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get credentials: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_netif_create_default_wifi_sta();

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, creds.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, creds.password, sizeof(wifi_config.sta.password) - 1);

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi connecting to: %s (pwd=%s)", creds.ssid, strlen(creds.password) > 0 ? "yes" : "none");

    return ESP_OK;
}

bool wifi_is_connected() {
    return g_wifi_connected;
}

esp_err_t wifi_start_hotspot(const char* ssid, const char* password) {
    if (!ssid || !password) return ESP_ERR_INVALID_ARG;

    if (g_hotspot_active) {
        ESP_LOGW(TAG, "Hotspot already active");
        return ESP_OK;
    }

    g_ap_netif = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char*)ap_config.ap.password, password, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.ssid_len = strlen(ssid);
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.max_connection = 4;

    if (strlen(password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Configure AP IP address
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = ipaddr_addr("192.168.4.1");
    ip_info.gw.addr = ipaddr_addr("192.168.4.1");
    ip_info.netmask.addr = ipaddr_addr("255.255.255.0");

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(g_ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(g_ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(g_ap_netif));

    g_hotspot_active = true;
    ESP_LOGI(TAG, "Hotspot started: SSID=%s, IP=192.168.4.1", ssid);

    return ESP_OK;
}

esp_err_t wifi_stop_hotspot() {
    if (!g_hotspot_active) {
        ESP_LOGW(TAG, "Hotspot not active");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_wifi_stop());
    g_hotspot_active = false;
    ESP_LOGI(TAG, "Hotspot stopped");

    return ESP_OK;
}
