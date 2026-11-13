#!/usr/bin/env python3
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
def read(path):
    full = os.path.join(root, path)
    return open(full, encoding="utf-8", errors="replace").read() if os.path.exists(full) else ""

link = read("main/link_sync.cpp")
io_cpp = read("main/io_helpers.cpp")
io_h = read("main/io_helpers.h")
input_cpp = read("main/input_handler.cpp")
header = read("main/live_pot_status.h")
readme = read("readme.md")
agents = read("AGENTS.md")

failures = []
if '#include "live_pot_status.h"' not in link:
    failures.append("link_sync.cpp does not include live_pot_status.h")
if "live_pot_status_suffix(" not in link:
    failures.append("status responder does not append the live pot suffix")
if "g_live_pot_value[0].load" not in link or "g_live_pot_value[1].load" not in link:
    failures.append("status responder does not read both live pot values")
if "std::atomic<int> g_live_pot_value[NUM_POTS]" not in io_cpp:
    failures.append("io_helpers.cpp does not define the live pot array")
if "g_live_pot_value[i].store" not in io_cpp:
    failures.append("runtime input path does not publish live pot values")
if "g_live_pot_value[i].store" not in input_cpp:
    failures.append("boot input path does not publish live pot values")
if "extern std::atomic<int> g_live_pot_value[NUM_POTS];" not in io_h:
    failures.append("io_helpers.h does not declare the live pot array")
if ',\\"pot0\\":%d' not in header or ',\\"pot1\\":%d' not in header:
    failures.append("live_pot_status.h does not format both pot fields")
for token in ("link", "peers", "bpm", "playing", "beat", "phase", "quantum", "ap", "metro"):
    if '\\"' + token + '\\"' not in link:
        failures.append("status responder lost existing field " + token)
if "pot0" not in readme or "pot1" not in readme:
    failures.append("readme network ports table does not list pot0/pot1")
if "pot0/pot1" not in agents:
    failures.append("AGENTS.md status table does not list pot0/pot1")

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("LIVE POT STATUS PARITY: PASS")
