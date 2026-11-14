#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

python3 "$here/pot-adc-max-parity.py" "$root"

cp "$root/main/pot_adc_scale.h" "$root/main/control_model.h" "$root/main/control_model.cpp" "$tmp/"
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$tmp" -o "$tmp/witness" \
    "$here/pot-adc-max-witness.cpp" "$tmp/control_model.cpp"
"$tmp/witness" > "$tmp/out.txt"
cat "$tmp/out.txt"
python3 - "$tmp/out.txt" <<'PY'
import sys
text = open(sys.argv[1], encoding="utf-8").read()
if "POT ADC MAX WITNESS: PASS" not in text or "FAIL " in text:
    raise SystemExit("pot ADC max witness: missing PASS line or a check failed")
PY
