#!/usr/bin/env python3
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
timer_path = os.path.join(root, "components/link-esp/link/include/ableton/platforms/asio/AsioTimer.hpp")
wrapper_path = os.path.join(root, "components/link-esp/link/include/ableton/platforms/asio/AsioWrapper.hpp")
peer_path = os.path.join(root, "components/link-esp/link/include/ableton/discovery/PeerGateway.hpp")
measurement_path = os.path.join(root, "components/link-esp/link/include/ableton/link/Measurement.hpp")
timer = open(timer_path, encoding="utf-8").read() if os.path.exists(timer_path) else ""
wrapper = open(wrapper_path, encoding="utf-8").read() if os.path.exists(wrapper_path) else ""
peer = open(peer_path, encoding="utf-8").read() if os.path.exists(peer_path) else ""
measurement = open(measurement_path, encoding="utf-8").read() if os.path.exists(measurement_path) else ""

failures = []
if "#if defined(ESP_PLATFORM)" not in timer:
    failures.append("AsioTimer has no ESP_PLATFORM branch")
if "esp_timer_start_once(" not in timer:
    failures.append("ESP AsioTimer does not start an esp_timer one-shot")
if "ESP_TIMER_TASK" not in timer:
    failures.append("ESP AsioTimer does not dispatch its timer from the esp_timer task")
if "std::chrono::steady_clock::time_point" not in timer:
    failures.append("AsioTimer TimePoint is not steady_clock")
if "std::chrono::steady_clock::now()" not in timer:
    failures.append("AsioTimer now() is not steady_clock::now()")
if "#include <asio/steady_timer.hpp>" not in timer and "#include <asio/steady_timer.hpp>" not in wrapper:
    failures.append("steady_timer header is not included for the host branch")
if "typename Timer::TimePoint" not in peer:
    failures.append("PeerGateway peer timeout does not use Timer::TimePoint")
if "std::chrono::system_clock::time_point" in peer:
    failures.append("PeerGateway still uses system_clock::time_point")
if "mTimer.expires_from_now(std::chrono::milliseconds(250));" not in measurement:
    failures.append("Measurement no longer uses the 250 ms ping timer")

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("LINK MEASUREMENT TIMER PARITY: PASS")
