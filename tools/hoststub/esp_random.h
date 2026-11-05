#pragma once
#include <cstdint>
#include <random>
inline uint32_t esp_random() { static std::mt19937 g(20261013u); return g(); }
