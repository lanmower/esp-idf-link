#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cp "$root/main/ltmp_request.h" "$root/tools/ltmp_request_witness.cpp" "$tmp/"
g++ -std=c++17 -Wall -Wextra -Werror -O1 -pthread -I"$tmp" -o "$tmp/witness" "$tmp/ltmp_request_witness.cpp"
"$tmp/witness"
