#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$root/main" -o "$tmp/midi_bulk_queue_witness" "$here/midi_bulk_queue_witness.cpp"
cd "$root"
"$tmp/midi_bulk_queue_witness"
