#include "provisioning.h"
#include "wifi_config.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "PROVISIONING";
static httpd_handle_t server = NULL;

static esp_err_t provisioning_form_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Form request received from client");

    const char* resp_str = "<html><head><meta charset='UTF-8'><title>WiFi</title><style>body{font-family:Arial;padding:20px}input{width:100%;padding:8px;margin:10px 0}button{background:#4CAF50;color:white;padding:10px;border:0;width:100%;cursor:pointer}</style></head><body><h1>WiFi Setup</h1><form method='GET' action='/wifi_config'><label>SSID:</label><input type='text' name='s' required><label>Password:</label><input type='password' name='p'><button type='submit'>Save</button></form></body></html>";

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, resp_str, strlen(resp_str));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send form: %s", esp_err_to_name(ret));
    }

    return ret;
}

static void url_decode(char* src, char* dst, size_t dst_size) {
    int i = 0, j = 0;
    while (src[i] && j < (int)dst_size - 1) {
        if (src[i] == '%' && src[i+1] && src[i+2]) {
            int val = 0;
            if (sscanf(src + i + 1, "%2x", &val) == 1) {
                dst[j++] = (char)val;
                i += 3;
            } else {
                dst[j++] = src[i++];
            }
        } else if (src[i] == '+') {
            dst[j++] = ' ';
            i++;
        } else {
            dst[j++] = src[i++];
        }
    }
    dst[j] = '\0';
}

static esp_err_t provisioning_config_handler(httpd_req_t *req) {
    char ssid[32] = {0};
    char password[64] = {0};
    char buf[256] = {0};
    char ssid_raw[32] = {0};
    char password_raw[64] = {0};

    if (httpd_req_get_url_query_len(req) > sizeof(buf) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query too large");
        return ESP_OK;
    }

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Query: %s", buf);

    // Parse s= (SSID)
    char* s_ptr = strstr(buf, "s=");
    if (s_ptr) {
        s_ptr += 2;
        int i = 0;
        while (*s_ptr && *s_ptr != '&' && i < 31) {
            ssid_raw[i++] = *s_ptr++;
        }
        ssid_raw[i] = '\0';
        url_decode(ssid_raw, ssid, sizeof(ssid));
    }

    // Parse p= (Password)
    char* p_ptr = strstr(buf, "p=");
    if (p_ptr) {
        p_ptr += 2;
        int i = 0;
        while (*p_ptr && *p_ptr != '&' && i < 63) {
            password_raw[i++] = *p_ptr++;
        }
        password_raw[i] = '\0';
        url_decode(password_raw, password, sizeof(password));
    }

    if (strlen(ssid) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No SSID");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Saving: SSID=%s", ssid);
    wifi_config_set_credentials(ssid, password);

    const char* resp_str = "Saved";
    httpd_resp_send(req, resp_str, strlen(resp_str));

    g_provisioning_complete = true;
    return ESP_OK;
}

esp_err_t provisioning_start_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 4;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    config.task_priority = 5;
    config.stack_size = 32768;
    config.max_req_hdr_len = 32768;
    config.max_uri_len = 8192;

    ESP_LOGI(TAG, "Starting provisioning server on port 80");
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start provisioning server");
        return ESP_FAIL;
    }

    httpd_uri_t form_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = provisioning_form_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &form_uri) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register root handler");
    }

    httpd_uri_t config_uri = {
        .uri       = "/wifi_config",
        .method    = HTTP_GET,
        .handler   = provisioning_config_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &config_uri) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register config handler");
    }

    ESP_LOGI(TAG, "Provisioning server started successfully on http://192.168.4.1");
    return ESP_OK;
}

esp_err_t provisioning_stop_server() {
    if (server) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Provisioning server stopped");
    }
    return ESP_OK;
}
