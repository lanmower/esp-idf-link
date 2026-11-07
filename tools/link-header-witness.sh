#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

flags=(
  -std=c++17 -Wall -Wextra -Werror -Wno-multichar -O1
  -I"$here"
  -I"$root/components/link-esp/link/include"
  -I"$root/components/espressif__asio/asio/asio/include"
)

g++ "${flags[@]}" -o "$tmp/link_header_witness" "$here/link_header_witness.cpp"
"$tmp/link_header_witness" > "$tmp/pass.out"

g++ "${flags[@]}" -DLINK_HEADER_WITNESS_LINEAR_MODEL \
  -o "$tmp/link_header_witness_linear" "$here/link_header_witness.cpp"
if "$tmp/link_header_witness_linear" > "$tmp/linear.out"; then
  printf '%s\n' 'NEGATIVE CONTROL FAILED: the linear beat model was accepted' >&2
  exit 1
fi

python3 - "$here/link_header_witness.cpp" "$tmp/pass.out" "$tmp/linear.out" <<'PY'
import sys

source_path, pass_path, linear_path = sys.argv[1:]
source = open(source_path, encoding="utf-8").read()
pass_out = open(pass_path, encoding="utf-8").read()
linear_out = open(linear_path, encoding="utf-8").read()

for header in ("Beats.hpp", "Tempo.hpp", "Timeline.hpp", "Phase.hpp", "ClientSessionTimelines.hpp"):
    needle = "#include <ableton/link/" + header + ">"
    if needle not in source:
        raise SystemExit("REAL-HEADER GUARD: " + needle + " missing from witness source")
if "hoststub" in source:
    raise SystemExit("REAL-HEADER GUARD: the witness must not use the hoststub Link.hpp")
if "#include \"link_platform_shim.hpp\"" not in source:
    raise SystemExit("SHIM GUARD: the witness must include the platform shim first")

required = (
    "SHIMMED SYMBOLS: htonll ntohll",
    "REAL LINK HEADERS: Beats.hpp Tempo.hpp Timeline.hpp Phase.hpp ClientSessionTimelines.hpp",
    "toPhaseEncodedBeats: 144 comparisons, 0 mismatches",
    "forceBeatAtTime: 18 reseats, 0 mismatches",
    "link header witness: PASS",
)
for line in required:
    if line not in pass_out:
        raise SystemExit("link header witness: missing line: " + line)
if "negative control: linear model differs in 0 of 144 comparisons" in pass_out:
    raise SystemExit("link header witness: negative control had no discriminating power")
if "link header witness: FAIL" not in linear_out:
    raise SystemExit("link header witness: the linear model was not rejected")

print(pass_out, end="")
print("REAL-HEADER GUARD OK: witness compiled the shipped vendored Link headers, not a stub.")
PY
