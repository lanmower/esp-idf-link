#!/usr/bin/env python3
import heapq
import os
import re
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
measurement_path = os.path.join(root, "components/link-esp/link/include/ableton/link/Measurement.hpp")
pacing_path = os.path.join(root, "components/link-esp/link/include/ableton/link/MeasurementPacing.hpp")
measurement = open(measurement_path, encoding="utf-8").read() if os.path.exists(measurement_path) else ""
pacing = open(pacing_path, encoding="utf-8").read() if os.path.exists(pacing_path) else ""


def pacing_value(name, default=None):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(\d+)", pacing)
    return int(match.group(1)) if match else default


def measurement_constant(name, default):
    match = re.search(r"static const std::size_t " + re.escape(name) + r"\s*=\s*([^;]+);", measurement)
    if not match:
        return default
    expression = match.group(1).strip()
    if expression.isdigit():
        return int(expression)
    return pacing_value(expression, default)


timer_match = re.search(r"mTimer\.expires_from_now\(std::chrono::milliseconds\(([^)]*)\)\);", measurement)
timer_expression = timer_match.group(1).strip() if timer_match else ""
if timer_expression.isdigit():
    timer_ms = int(timer_expression)
elif timer_expression:
    timer_ms = pacing_value(timer_expression, None)
else:
    timer_ms = None
if timer_ms is None:
    timer_ms = 250

ping_cadence_ms = pacing_value("LINK_MEASUREMENT_PING_CADENCE_MS", 250)
number_data_points = measurement_constant("kNumberDataPoints", 100)
number_measurements = measurement_constant("kNumberMeasurements", 12)
reset_budget_on_pong = "mMeasurementsStarted = 0;" in measurement


def simulate(timer, pong_delay, max_measurements, data_point_target, reset_on_pong, until_ms=30000):
    events = []
    generation = 0
    deadline = None

    def arm(at_ms):
        nonlocal generation, deadline
        generation += 1
        deadline = at_ms
        heapq.heappush(events, (at_ms, "timer", generation))

    heapq.heappush(events, (pong_delay, "pong", 0))
    arm(timer)
    measurement_pings = []
    pongs = []
    measurements_started = 0
    data_points = 0
    timer_retries = 0
    completed = False
    first_pong = True
    while events:
        at_ms, kind, event_generation = heapq.heappop(events)
        if at_ms > until_ms:
            break
        if kind == "timer":
            if event_generation != generation or deadline != at_ms:
                continue
            if measurements_started >= max_measurements:
                break
            measurements_started += 1
            timer_retries += 1
            heapq.heappush(events, (at_ms + pong_delay, "pong", 0))
            arm(at_ms + timer)
            continue
        if reset_on_pong:
            measurements_started = 0
        pongs.append(at_ms)
        measurement_pings.append(at_ms)
        if first_pong:
            first_pong = False
        else:
            data_points += 2
        if data_points > data_point_target:
            completed = True
            break
        arm(at_ms + timer)

    def cadence(stamps):
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        return sum(gaps) / len(gaps) if gaps else 0.0

    return {
        "pong_cadence_ms": cadence(pongs),
        "ping_cadence_ms": cadence(measurement_pings),
        "data_points": data_points,
        "timer_retries": timer_retries,
        "completed": completed,
    }


failures = []
if not pacing:
    failures.append("MeasurementPacing.hpp is missing")
if "#include <ableton/link/MeasurementPacing.hpp>" not in measurement:
    failures.append("Measurement.hpp does not include the pacing constants")
if "LINK_MEASUREMENT_TIMER_MS" not in timer_expression:
    failures.append("Measurement.hpp does not arm its ping timer from LINK_MEASUREMENT_TIMER_MS")
if "LINK_MEASUREMENT_PING_CADENCE_MS" not in measurement:
    failures.append("Measurement.hpp does not hold the ping-cadence invariant")
if not reset_budget_on_pong:
    failures.append("an accepted pong does not reset the timer-failure budget")
if timer_ms >= ping_cadence_ms:
    failures.append("the retry timer %d ms is not shorter than the ping cadence %d ms" % (timer_ms, ping_cadence_ms))
for name, value in (("LINK_MEASUREMENT_TIMER_MS", 125), ("LINK_MEASUREMENT_PING_CADENCE_MS", 250)):
    if pacing_value(name) != value:
        failures.append("MeasurementPacing.hpp lacks " + name + " = " + str(value))

for pong_delay in (200, 250):
    outcome = simulate(timer_ms, pong_delay, number_measurements, number_data_points, reset_budget_on_pong)
    print("PACING MODEL: timer_ms=%d pong_delay_ms=%d pong_cadence_ms=%.1f ping_cadence_ms=%.1f "
          "data_points=%d timer_retries=%d completed=%s" % (
              timer_ms, pong_delay, outcome["pong_cadence_ms"], outcome["ping_cadence_ms"],
              outcome["data_points"], outcome["timer_retries"],
              "yes" if outcome["completed"] else "no"))
    if not 200 <= outcome["pong_cadence_ms"] <= 280:
        failures.append("pong cadence %.1f ms at pong delay %d ms is not near the 250 ms target" % (
            outcome["pong_cadence_ms"], pong_delay))
    if not outcome["completed"] or outcome["data_points"] <= number_data_points:
        failures.append("measurement fails early at %d data points (budget %d, reset_on_pong=%s) at pong delay %d ms" % (
            outcome["data_points"], number_measurements,
            "yes" if reset_budget_on_pong else "no", pong_delay))

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("LINK MEASUREMENT PACING PARITY: PASS")
