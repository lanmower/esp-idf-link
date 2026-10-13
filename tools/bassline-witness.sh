#!/usr/bin/env bash
# Build and run the bassline witness off-device. No ESP-IDF, no Docker, no board.
#   ./tools/bassline-witness.sh                      # ROLL grid + metrics
#   ./tools/bassline-witness.sh --all                # every voice's metrics
#   ./tools/bassline-witness.sh --approach 0 --mid /tmp/roll.mid
#
# The engine sources are copied to a scratch dir because a quoted #include resolves against the
# including file's own directory first, so main/main.h would win over the host stub unless the
# sources are compiled from somewhere else.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cp "$root/main/bass_engine.cpp" "$root/main/bass_engine.h" \
   "$root/main/bassline_interpreter.cpp" "$root/main/bassline_interpreter.h" \
   "$here/bassline_witness.cpp" "$tmp/"
cp -r "$here/hoststub/." "$tmp/"

g++ -std=c++17 -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/bassline_witness.cpp" "$tmp/bass_engine.cpp" "$tmp/bassline_interpreter.cpp"
"$tmp/witness" "$@"
