#!/usr/bin/env python3
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
PATHS = (
    "main/main.h",
    "main/io_helpers.cpp",
    "main/pot_adc_scale.h",
    "tools/bass_loop_witness.cpp",
    "tools/pot-adc-max-witness.sh",
    ".github/workflows/build.yml",
    "docs/PENDING_VALIDATION.md",
)

LEGACY_IO = '''#include "io_helpers.h"
void update_input_state(InputEvent& event) {
    uint32_t pot_value = last_pot_values[i] * (1.0f - SMOOTH_FACTOR) + adc_reading * SMOOTH_FACTOR;
    int scaled_value = (int)((pot_value * MIDI_VALUE_MAX) / ADC_RAW_MAX);
    event.pot_value[i] = scaled_value;
    g_live_pot_value[i].store(scaled_value, std::memory_order_relaxed);
}
'''

def read(path):
    full = os.path.join(root, path)
    return open(full, encoding="utf-8", errors="replace").read() if os.path.exists(full) else ""

def check(files):
    io = files.get("main/io_helpers.cpp", "")
    hdr = files.get("main/pot_adc_scale.h", "")
    mainh = files.get("main/main.h", "")
    bass = files.get("tools/bass_loop_witness.cpp", "")
    runner = files.get("tools/pot-adc-max-witness.sh", "")
    yml = files.get(".github/workflows/build.yml", "")
    pending = files.get("docs/PENDING_VALIDATION.md", "")
    failures = []
    if "constexpr int ADC_RAW_MAX = 4095;" not in mainh or "constexpr int MIDI_VALUE_MAX = 127;" not in mainh:
        failures.append("shipped ADC/MIDI constants moved")
    if '#include "pot_adc_scale.h"' not in io:
        failures.append("io_helpers.cpp does not include pot_adc_scale.h")
    if "pot_adc_to_midi(pot_value, ADC_RAW_MAX, MIDI_VALUE_MAX)" not in io:
        failures.append("steady-state pot mapping does not use the shared ADC scale")
    if "(pot_value * MIDI_VALUE_MAX) / ADC_RAW_MAX" in io:
        failures.append("old truncating ADC scale is still in update_input_state")
    if "g_live_pot_value[i].store(scaled_value" not in io:
        failures.append("UDP 20813 live pot publisher lost the scaled value")
    if "raw + top_band >= top" not in hdr or "top_band = top / 128u" not in hdr:
        failures.append("shared ADC scale lost its top-of-range clamp")
    if "+ top / 2" not in hdr:
        failures.append("shared ADC scale lost its rounded division")
    if "pot-adc-max-witness.cpp" not in runner or "POT ADC MAX WITNESS: PASS" not in runner:
        failures.append("host witness runner lost its witness or pass marker")
    if "bash tools/pot-adc-max-witness.sh" not in yml:
        failures.append("CI does not run the pot ADC maximum witness")
    if "pot-adc-max-witness.sh" not in pending:
        failures.append("docs/PENDING_VALIDATION.md has no pot ADC maximum line")
    if "sidechain MIDI 127 silences the beat" not in bass:
        failures.append("the real-engine 127 silence witness is missing")
    return failures

files = {p: read(p) for p in PATHS}
failures = check(files)
legacy = dict(files)
legacy["main/io_helpers.cpp"] = LEGACY_IO
legacy_failures = check(legacy)
if not legacy_failures:
    failures.append("parity guard does not reject the legacy truncating source")

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("POT ADC MAX PARITY: PASS (legacy source rejected: %d failures)" % len(legacy_failures))
