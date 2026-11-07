#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

flags=(
  -std=c++17 -Wall -Wextra -Werror -Wno-multichar -O1
  -I"$here"
  -I"$root/main"
  -I"$root/components/link-esp/link/include"
  -I"$root/components/espressif__asio/asio/asio/include"
)

g++ "${flags[@]}" -o "$tmp/first_take_origin_witness" \
  "$here/first-take-origin-witness.cpp"
cd "$root"
"$tmp/first_take_origin_witness" > "$tmp/out.txt"

python3 - "$tmp/out.txt" <<'PY'
import sys

out = open(sys.argv[1], encoding="utf-8").read()
required = (
    "FIRST-TAKE ORIGIN WITNESS: PASS",
    "first take origin: ",
    "0 failures",
    "4-beat reseat misses the take beat 0 in",
    "[first take] origin",
)
for line in required:
    if line not in out:
        raise SystemExit("first take origin witness: missing line: " + line)
if "FAIL:" in out:
    raise SystemExit("first take origin witness: check failed")
if "0 failures" not in out.split("first take origin: ", 1)[1].splitlines()[0]:
    raise SystemExit("first take origin witness: nonzero failures")
print(out, end="")
PY
