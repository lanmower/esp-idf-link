#pragma once

#include <cstddef>

constexpr unsigned LINK_MEASUREMENT_TIMER_MS = 125;
constexpr unsigned LINK_MEASUREMENT_PING_CADENCE_MS = 250;
constexpr std::size_t LINK_MEASUREMENT_NUMBER_DATA_POINTS = 100;
constexpr std::size_t LINK_MEASUREMENT_NUMBER_MEASUREMENTS =
  5 * (LINK_MEASUREMENT_PING_CADENCE_MS / LINK_MEASUREMENT_TIMER_MS) + 2;

static_assert(LINK_MEASUREMENT_TIMER_MS < LINK_MEASUREMENT_PING_CADENCE_MS,
  "the measurement retry timer must be shorter than the pong round trip");
