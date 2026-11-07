#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
  -I"$root/main" -o "$tmp/phrase_boundary_guard_witness" \
  "$here/phrase_boundary_guard_witness.cpp"
cd "$root"
"$tmp/phrase_boundary_guard_witness"
