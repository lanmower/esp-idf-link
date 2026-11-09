#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

python3 "$here/link-origin-parity.py" "$root"

g++ -std=c++17 -Wall -Wextra -Werror -Wno-multichar -O1 \
  -I"$here" \
  -I"$root/main" \
  -I"$root/components/link-esp/link/include" \
  -I"$root/components/espressif__asio/asio/asio/include" \
  -o "$tmp/link_origin_realign_witness" \
  "$here/link-origin-realign-witness.cpp"
cd "$root"
status=0
"$tmp/link_origin_realign_witness" > "$tmp/out.txt" || status=$?
cat "$tmp/out.txt"
test "$status" -eq 0
grep -q "LINK ORIGIN REALIGN WITNESS: PASS" "$tmp/out.txt"
