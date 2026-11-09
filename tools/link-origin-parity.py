#!/usr/bin/env python3
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
with open(os.path.join(root, "main", "link_sync.cpp"), encoding="utf-8") as fh:
    src = fh.read()
header_path = os.path.join(root, "main", "link_origin_guard.h")
header = open(header_path, encoding="utf-8").read() if os.path.exists(header_path) else ""

failures = []
if '#include "link_origin_guard.h"' not in src:
    failures.append("link_sync.cpp does not include link_origin_guard.h")
start = src.find("void handle_link_sync(")
end = src.find("QuantumInfo quantumInfo = detectQuantumBoundary(state, time);", start)
if start < 0 or end < 0:
    failures.append("handle_link_sync or its detectQuantumBoundary call was not found")
    body = ""
else:
    body = src[start:end]
block_at = body.find("if (s_origin_watch.observe(")
if block_at < 0:
    failures.append("handle_link_sync has no origin-shift check before detectQuantumBoundary")
    block = ""
else:
    block = body[block_at:block_at + 420]
if "static LinkOriginWatch s_origin_watch;" not in body:
    failures.append("origin watch is not a static in handle_link_sync")
if "s_origin_watch.observe(state.beatAtTime(time, LINK_ORIGIN_WATCH_QUANTUM_BEATS)" not in block:
    failures.append("origin watch is not fed the origin-relative beat of this tick")
if "s_pending_realign = true;" not in block:
    failures.append("a Link origin shift does not set s_pending_realign")
if "s_phrase_baseline_captured = false;" not in block:
    failures.append("a Link origin shift does not reset the phrase baseline (a jump would read as a boundary)")
for token in ("struct LinkOriginWatch", "bool observe(", "LINK_ORIGIN_WATCH_QUANTUM_BEATS", "LINK_ORIGIN_SHIFT_BEATS"):
    if token not in header:
        failures.append("link_origin_guard.h lacks " + token)

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("LINK ORIGIN PARITY: PASS")
