#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

python3 "$here/link-peer-endpoint-parity.py" "$root"

g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$root/main"   -o "$tmp/link_peer_endpoint_relearn_witness"   "$here/link-peer-endpoint-relearn-witness.cpp"

status=0
"$tmp/link_peer_endpoint_relearn_witness" > "$tmp/out.txt" || status=$?
cat "$tmp/out.txt"
test "$status" -eq 0
grep -q "LINK PEER ENDPOINT RELEARN WITNESS: PASS" "$tmp/out.txt"
