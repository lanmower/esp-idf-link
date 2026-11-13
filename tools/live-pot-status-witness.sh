#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

python3 "$here/live-pot-status-parity.py" "$root"

g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$root/main" \
  -o "$tmp/live_pot_status_witness" \
  "$here/live-pot-status-witness.cpp"
"$tmp/live_pot_status_witness" > "$tmp/out.txt"
cat "$tmp/out.txt"
python3 - "$tmp/out.txt" <<'PY'
import sys
text = open(sys.argv[1], encoding="utf-8").read()
if "LIVE POT STATUS WITNESS: PASS" not in text:
    raise SystemExit("live pot status witness: missing PASS line")
PY
