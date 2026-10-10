#include "network_midi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <cstring>
#include <algorithm>
#include <sys/stat.h>
#include <dirent.h>

static const char* TAG = "NETWORK_MIDI";
static httpd_handle_t g_server = nullptr;
static char g_device_ip[16] = {0};

static esp_err_t upload_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Received file upload request");

    const char* filename = nullptr;
    if (strlen(req->uri) > 8) {
        filename = req->uri + 8;
    }

    if (!filename || strlen(filename) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Filename required");
        return ESP_FAIL;
    }

    char filepath[512];
    snprintf(filepath, sizeof(filepath), "/spiffs/loops/%s", filename);

    FILE* f = fopen(filepath, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file");
        return ESP_FAIL;
    }

    const int buf_size = 4096;
    char* buf = new char[buf_size];
    int received = 0;

    while (received < req->content_len) {
        int read = httpd_req_recv(req, buf, std::min(buf_size, (int)(req->content_len - received)));
        if (read <= 0) {
            break;
        }
        fwrite(buf, 1, read, f);
        received += read;
    }

    fclose(f);
    delete[] buf;

    ESP_LOGI(TAG, "File saved: %s (%d bytes)", filepath, received);

    httpd_resp_send(req, "File uploaded successfully", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t info_handler(httpd_req_t* req) {
    char response[256];
    snprintf(response, sizeof(response),
        "{\"device_ip\":\"%s\",\"device_name\":\"ESP32-Link\",\"version\":\"1.0\"}",
        g_device_ip);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t clear_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Clearing all MIDI clips from /spiffs/loops/");

    DIR* dir = opendir("/spiffs/loops");
    if (!dir) {
        httpd_resp_send(req, "Directory not found", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    struct dirent* entry;
    int deleted = 0;

    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG && strstr(entry->d_name, ".mid")) {
            char filepath[512];
            snprintf(filepath, sizeof(filepath), "/spiffs/loops/%s", entry->d_name);
            if (remove(filepath) == 0) {
                ESP_LOGI(TAG, "Deleted: %s", entry->d_name);
                deleted++;
            }
        }
    }

    closedir(dir);

    char response[256];
    snprintf(response, sizeof(response),
        "{\"status\":\"cleared\",\"files_deleted\":%d}", deleted);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Cleared %d files", deleted);
    return ESP_OK;
}

static void get_device_ip() {
    esp_netif_t* netif = esp_netif_get_default_netif();
    if (!netif) {
        strcpy(g_device_ip, "0.0.0.0");
        return;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, g_device_ip, sizeof(g_device_ip));
    } else {
        strcpy(g_device_ip, "0.0.0.0");
    }
}

void network_midi_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.max_uri_handlers = 8;
    config.max_open_sockets = 4;

    if (httpd_start(&g_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t upload_uri = {
        .uri = "/upload/*",
        .method = HTTP_POST,
        .handler = upload_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(g_server, &upload_uri);

    httpd_uri_t info_uri = {
        .uri = "/info",
        .method = HTTP_GET,
        .handler = info_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(g_server, &info_uri);

    httpd_uri_t clear_uri = {
        .uri = "/clear",
        .method = HTTP_POST,
        .handler = clear_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(g_server, &clear_uri);

    get_device_ip();
    ESP_LOGI(TAG, "Network MIDI server initialized on %s:8080", g_device_ip);
}

void network_midi_start() {
    if (g_server) {
        ESP_LOGI(TAG, "Network MIDI server already running on %s:8080", g_device_ip);
    }
}

void network_midi_stop() {
    if (g_server) {
        httpd_stop(g_server);
        g_server = nullptr;
        ESP_LOGI(TAG, "Network MIDI server stopped");
    }
}

const char* network_midi_get_device_ip() {
    return g_device_ip;
}
