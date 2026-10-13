// Host stub of esp_log.h: silent unless PSYLOG is set in the environment.
#pragma once
#include <cstdio>
#include <cstdlib>
#define PSYLOG_ON() (getenv("PSYLOG") != nullptr)
#define ESP_LOGI(tag, fmt, ...) do { if (PSYLOG_ON()) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__); } while (0)
#define ESP_LOGW(tag, fmt, ...) do { if (PSYLOG_ON()) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, fmt, ...) do { if (PSYLOG_ON()) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__); } while (0)
#define ESP_LOGD(tag, fmt, ...) do {} while (0)
