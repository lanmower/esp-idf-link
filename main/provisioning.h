#ifndef PROVISIONING_H
#define PROVISIONING_H

#include <esp_err.h>

extern bool g_provisioning_complete;

esp_err_t provisioning_start_server();
esp_err_t provisioning_stop_server();

#endif
