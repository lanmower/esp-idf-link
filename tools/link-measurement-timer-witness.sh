#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

python3 "$here/link-measurement-timer-parity.py" "$root"

g++ -std=c++17 -Wall -Wextra -Werror -Wno-multichar -O1 \
  -I"$here" \
  -I"$root/components/link-esp/link/include" \
  -I"$root/components/espressif__asio/asio/asio/include" \
  -o "$tmp/link_measurement_timer_witness" \
  "$here/link-measurement-timer-witness.cpp"
"$tmp/link_measurement_timer_witness" > "$tmp/out.txt"
cat "$tmp/out.txt"
grep -q "LINK MEASUREMENT TIMER WITNESS: PASS" "$tmp/out.txt"
