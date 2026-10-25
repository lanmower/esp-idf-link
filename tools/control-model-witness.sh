#!/usr/bin/env bash
# Build and run the control-model witness off-device (no ESP-IDF needed).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
cp "$root/main/control_model.h" "$root/main/control_model.cpp" "$here/control_model_witness.cpp" "$tmp/"
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/control_model_witness.cpp" "$tmp/control_model.cpp"
"$tmp/witness"
