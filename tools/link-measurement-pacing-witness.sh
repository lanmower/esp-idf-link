#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

python3 "$here/link-measurement-pacing-parity.py" "$root"

g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$root/components/link-esp/link/include" -o "$tmp/link_measurement_pacing_witness" "$here/link-measurement-pacing-witness.cpp"
"$tmp/link_measurement_pacing_witness"
