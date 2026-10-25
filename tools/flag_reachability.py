#!/usr/bin/env python3
"""Preprocessor reachability check for WIFI_NO_TEARDOWN_ROLE_SWITCH.

For each named call in main/wifi_config.cpp, report whether the call is live (inside active code)
when the flag is 0 and when it is 1. Conditionals other than the flag are treated as taken, which
over-approximates reachability, so a call reported not live under flag=1 is not live under any
other value of the unrelated conditionals.

    python3 tools/flag_reachability.py [path/to/wifi_config.cpp]

The plan (docs/netif-redesign-plan.md, P9) requires this to report live=False for every
destroy and wifi-stop call when the flag is 1.
"""
import re
import sys

NAMES = ("esp_netif_destroy_default_wifi", "esp_netif_destroy", "esp_wifi_stop")
FLAG = "WIFI_NO_TEARDOWN_ROLE_SWITCH"


def live_lines(src, flag_value):
    stack = []
    active = [True] * (len(src) + 1)
    for i, raw in enumerate(src, 1):
        line = raw.strip()

        def current():
            return all(stack)

        m = re.match(r"#\s*if\s+(!?)\s*" + FLAG + r"\s*$", line)
        if m:
            val = flag_value
            if m.group(1):
                val = not val
            stack.append(val)
            active[i] = current()
            continue
        if re.match(r"#\s*else\b", line):
            stack[-1] = not stack[-1]
            active[i] = current()
            continue
        if re.match(r"#\s*endif\b", line):
            stack.pop()
            active[i] = current()
            continue
        if re.match(r"#\s*if\b", line):
            stack.append(True)
            active[i] = current()
            continue
        active[i] = current()
    return active


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "main/wifi_config.cpp"
    with open(path, encoding="utf8", errors="replace") as fh:
        src = fh.read().split("\n")
    off = live_lines(src, False)
    on = live_lines(src, True)
    hits = 0
    for i, line in enumerate(src, 1):
        for name in NAMES:
            if re.search(r"\b" + name + r"\s*\(", line):
                hits += 1
                print(f"{name:32s} line {i:5d}  flag=0 live={off[i]!s:5s}  flag=1 live={on[i]!s:5s}  | {line.strip()[:64]}")
    if hits == 0:
        print("no matches")
    bad = [i for i, line in enumerate(src, 1)
           if any(re.search(r"\b" + n + r"\s*\(", line) for n in NAMES) and on[i]]
    print("RESULT: no destroy or wifi-stop call is live with the flag on" if not bad
          else f"RESULT: LIVE with flag on at lines {bad}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
