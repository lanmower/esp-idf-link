#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cp "$root/main/bass_engine.cpp" "$root/main/bass_engine.h" \
   "$root/main/control_model.cpp" "$root/main/control_model.h" \
   "$here/bass_loop_witness.cpp" "$tmp/"
cp -r "$here/hoststub/." "$tmp/"

g++ -std=c++17 -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/bass_loop_witness.cpp" "$tmp/bass_engine.cpp" "$tmp/control_model.cpp"
"$tmp/witness"
