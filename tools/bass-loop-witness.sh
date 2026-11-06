#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cp "$root/main/bass_engine.cpp" "$root/main/bass_engine.h" \
   "$root/main/control_model.cpp" "$root/main/control_model.h" \
   "$here/bass_loop_witness.cpp" "$tmp/"
cp -r "$here/hoststub/." "$tmp/"

python3 - "$root/main/bass_engine.cpp" <<'PY'
import sys
src = open(sys.argv[1]).read()
body = src[src.index('void BassEngine::resetAll()'):]
body = body[:body.index('\n}\n')]
assert 'm_seedCounter = 1;' in body, 'resetAll must restore m_seedCounter'
assert 'm_entrySeed[c] = 1u + static_cast<uint32_t>(c);' in body, 'resetAll must restore m_entrySeed'
print('resetAll seed-state parity: OK')
PY

g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/bass_loop_witness.cpp" "$tmp/bass_engine.cpp" "$tmp/control_model.cpp"
"$tmp/witness"
