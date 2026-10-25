#!/usr/bin/env bash
# Build and run the phrase-advance witness off-device (no ESP-IDF, no board).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cp "$root/main/bass_engine.cpp" "$root/main/bass_engine.h" \
   "$root/main/bassline_interpreter.cpp" "$root/main/bassline_interpreter.h" \
   "$here/phrase_advance_witness.cpp" "$tmp/"
cp -r "$here/hoststub/." "$tmp/"

g++ -std=c++17 -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/phrase_advance_witness.cpp" "$tmp/bass_engine.cpp" "$tmp/bassline_interpreter.cpp"
"$tmp/witness"
