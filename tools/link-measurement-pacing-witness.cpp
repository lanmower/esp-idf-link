#include <ableton/link/MeasurementPacing.hpp>

#include <cstddef>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <queue>
#include <vector>

namespace
{

struct PacingEvent
{
  unsigned timeMs;
  unsigned generation;
  bool isTimer;

  bool operator>(const PacingEvent& other) const
  {
    return timeMs > other.timeMs || (timeMs == other.timeMs && isTimer > other.isTimer);
  }
};

struct PacingOutcome
{
  double pongCadenceMs = 0.0;
  double pingCadenceMs = 0.0;
  std::size_t dataPoints = 0;
  std::size_t timerRetries = 0;
  bool completed = false;
};

PacingOutcome simulate(unsigned timerMs, unsigned pongDelayMs, std::size_t numberMeasurements,
  std::size_t numberDataPoints, bool resetBudgetOnPong, unsigned untilMs = 30000)
{
  using EventQueue = std::priority_queue<PacingEvent, std::vector<PacingEvent>,
    std::greater<PacingEvent>>;
  EventQueue events;
  unsigned generation = 0;
  unsigned deadline = 0;
  std::vector<unsigned> pongs;
  std::vector<unsigned> measurementPings;
  std::size_t measurementsStarted = 0;
  std::size_t timerRetries = 0;
  PacingOutcome outcome;
  bool firstPong = true;

  const auto arm = [&](unsigned atMs) {
    ++generation;
    deadline = atMs;
    events.push(PacingEvent{atMs, generation, true});
  };
  const auto pushPong = [&](unsigned atMs) {
    events.push(PacingEvent{atMs, 0u, false});
  };

  pushPong(pongDelayMs);
  arm(timerMs);

  while (!events.empty())
  {
    const PacingEvent event = events.top();
    events.pop();
    if (event.timeMs > untilMs)
    {
      break;
    }
    if (event.isTimer)
    {
      if (event.generation != generation || deadline != event.timeMs)
      {
        continue;
      }
      if (measurementsStarted >= numberMeasurements)
      {
        break;
      }
      ++measurementsStarted;
      ++timerRetries;
      pushPong(event.timeMs + pongDelayMs);
      arm(event.timeMs + timerMs);
      continue;
    }
    if (resetBudgetOnPong)
    {
      measurementsStarted = 0;
    }
    pongs.push_back(event.timeMs);
    measurementPings.push_back(event.timeMs);
    if (firstPong)
    {
      firstPong = false;
    }
    else
    {
      outcome.dataPoints += 2;
    }
    if (outcome.dataPoints > numberDataPoints)
    {
      outcome.completed = true;
      break;
    }
    arm(event.timeMs + timerMs);
  }

  const auto cadence = [](const std::vector<unsigned>& stamps) {
    if (stamps.size() < 2)
    {
      return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 1; i < stamps.size(); ++i)
    {
      sum += static_cast<double>(stamps[i] - stamps[i - 1]);
    }
    return sum / static_cast<double>(stamps.size() - 1);
  };
  outcome.pongCadenceMs = cadence(pongs);
  outcome.pingCadenceMs = cadence(measurementPings);
  outcome.timerRetries = timerRetries;
  return outcome;
}

}

int main()
{
  static_assert(LINK_MEASUREMENT_TIMER_MS == 125, "the retry timer must be 125 ms");
  static_assert(LINK_MEASUREMENT_PING_CADENCE_MS == 250, "the ping cadence target is 250 ms");
  static_assert(LINK_MEASUREMENT_TIMER_MS * 2 == LINK_MEASUREMENT_PING_CADENCE_MS,
    "one ping cadence must contain two retry windows");
  static_assert(LINK_MEASUREMENT_NUMBER_DATA_POINTS == 100, "the measurement needs 100 data points");
  static_assert(LINK_MEASUREMENT_NUMBER_MEASUREMENTS >= 5, "the retry budget must outlast one cadence");

  int failures = 0;
  for (unsigned pongDelayMs : {200u, 250u})
  {
    const PacingOutcome outcome = simulate(LINK_MEASUREMENT_TIMER_MS, pongDelayMs,
      LINK_MEASUREMENT_NUMBER_MEASUREMENTS, LINK_MEASUREMENT_NUMBER_DATA_POINTS, true);
    std::printf("PACING WITNESS: timer_ms=%u pong_delay_ms=%u pong_cadence_ms=%.1f "
                "ping_cadence_ms=%.1f data_points=%zu timer_retries=%zu completed=%s\n",
      LINK_MEASUREMENT_TIMER_MS, pongDelayMs, outcome.pongCadenceMs, outcome.pingCadenceMs,
      outcome.dataPoints, outcome.timerRetries, outcome.completed ? "yes" : "no");
    if (outcome.pongCadenceMs < 200.0 || outcome.pongCadenceMs > 280.0)
    {
      std::printf("WITNESS FAIL: pong cadence %.1f ms is not near the 250 ms target\n",
        outcome.pongCadenceMs);
      ++failures;
    }
    if (outcome.pingCadenceMs < 200.0 || outcome.pingCadenceMs > 280.0)
    {
      std::printf("WITNESS FAIL: ping cadence %.1f ms is not near the 250 ms target\n",
        outcome.pingCadenceMs);
      ++failures;
    }
    if (!outcome.completed || outcome.dataPoints <= LINK_MEASUREMENT_NUMBER_DATA_POINTS)
    {
      std::printf("WITNESS FAIL: measurement fails early at %zu data points\n", outcome.dataPoints);
      ++failures;
    }
  }
  if (failures != 0)
  {
    return 1;
  }
  std::printf("LINK MEASUREMENT PACING WITNESS: PASS\n");
  return 0;
}
