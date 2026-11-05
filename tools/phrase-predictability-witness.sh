#!/usr/bin/env bash
# Build and run the phrase-predictability witness off-device (no ESP-IDF, no board).
#   * The witness links the ACTUAL shipped main/control_model.cpp + main/bass_engine.cpp.
#   * Everything that cannot compile host-side (main/link_sync.cpp esp_timer/gptimer/
#     lwip/g_link) is modelled by a clearly-labelled MIRROR in the witness .cpp.
#     This script runs a PARITY GUARD first: it re-reads the shipped source and
#     refuses to build if any mirrored constant or formula has drifted.
# Nothing is copied into the repo; all work happens in a temp dir that is removed.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# ---- PARITY GUARD -----------------------------------------------------------
# Reads shipped source with bash/python3 (equivalent to gm codesearch/fs_read),
# parses it with python3, and compares to the MIRROR constants/line shapes in
# tools/phrase_predictability_witness.cpp. Loud failure on any drift.
python3 - "$root" "$here/phrase_predictability_witness.cpp" <<'PY'
import os, re, sys

ROOT = sys.argv[1]
WITNESS = sys.argv[2]

REL = [
    "main/main.h",
    "main/link_sync.cpp",
    "main/link_sync.h",
    "main/bass_engine.cpp",
    "main/control_model.h",
    "main/control_model.cpp",
    "components/link-esp/link/include/ableton/link/Tempo.hpp",
    "components/link-esp/link/include/ableton/link/Phase.hpp",
    "components/link-esp/link/include/ableton/Link.hpp",
]
src = {}
for rel in REL:
    with open(os.path.join(ROOT, rel), "r", encoding="utf-8", errors="replace") as f:
        src[rel] = f.read()
with open(WITNESS, "r", encoding="utf-8", errors="replace") as f:
    wit = f.read()

LS = "main/link_sync.cpp"
LSH = "main/link_sync.h"
MH = "main/main.h"
BE = "main/bass_engine.cpp"
CMH = "main/control_model.h"
CMC = "main/control_model.cpp"
TEMPO = "components/link-esp/link/include/ableton/link/Tempo.hpp"
PHASE = "components/link-esp/link/include/ableton/link/Phase.hpp"
LINKH = "components/link-esp/link/include/ableton/Link.hpp"

failures = 0
def complain(msg):
    global failures
    failures += 1
    print("PARITY FAIL: " + msg)

def parse_num(s):
    s = s.strip()
    if re.match(r'^0[xX]', s):
        return float(int(s, 16))
    return float(s)

# (mirror macro name, shipped file, regex with ONE capture group = the value)
CONST = [
    ("MIR_MIDI_PULSES_PER_QUARTER_NOTE", LS, r'MIDI_PULSES_PER_QUARTER_NOTE\s*=\s*([0-9]+)'),
    ("MIR_MIDI_SPP_UNITS_PER_BEAT",      LS, r'MIDI_SPP_UNITS_PER_BEAT\s*=\s*([0-9]+)'),
    ("MIR_MIDI_SPP_UNITS_MASK",          LS, r'MIDI_SPP_UNITS_MASK\s*=\s*(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_DATA_BYTE_MASK",          LS, r'MIDI_DATA_BYTE_MASK\s*=\s*(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_DATA_BYTE_SHIFT",         LS, r'MIDI_DATA_BYTE_SHIFT\s*=\s*([0-9]+)'),
    ("MIR_MICROBEATS_PER_BEAT",          LS, r'MICROBEATS_PER_BEAT\s*=\s*([0-9.eE+\-]+)'),
    ("MIR_FORCE_START_AFTER_NO_PEERS_US", LS, r'FORCE_START_AFTER_NO_PEERS_US\s*=\s*([0-9]+)'),
    ("MIR_MAX_CATCHUP_PULSES_PER_SCHEDULE", LS, r'MAX_CATCHUP_PULSES_PER_SCHEDULE\s*=\s*([0-9]+)'),
    ("MIR_MIDI_CLOCK_RESYNC_THRESHOLD",  MH, r'#define\s+MIDI_CLOCK_RESYNC_THRESHOLD\s+([0-9]+)'),
    ("MIR_PHRASE_BEATS",                 MH, r'#define\s+PHRASE_BEATS\s+([0-9.]+)'),
    ("MIR_LINK_QUANTUM",                 MH, r'#define\s+LINK_QUANTUM\s+([0-9.]+)'),
    ("MIR_METRONOME_ACCENT_CYCLE_BEATS", MH, r'#define\s+METRONOME_ACCENT_CYCLE_BEATS\s+([0-9]+)'),
    ("MIR_MIDI_TIMING_CLOCK",            MH, r'#define\s+MIDI_TIMING_CLOCK\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_START",                   MH, r'#define\s+MIDI_START\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_STOP",                    MH, r'#define\s+MIDI_STOP\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_CONTINUE",                MH, r'#define\s+MIDI_CONTINUE\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_SONG_POSITION_POINTER",   MH, r'#define\s+MIDI_SONG_POSITION_POINTER\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_NOTE_ON_CMD",             MH, r'#define\s+MIDI_NOTE_ON_CMD\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_NOTE_OFF_CMD",            MH, r'#define\s+MIDI_NOTE_OFF_CMD\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_CC_CMD",                  MH, r'#define\s+MIDI_CC_CMD\s+(0[xX][0-9A-Fa-f]+|[0-9]+)'),
    ("MIR_MIDI_CC_ALL_NOTES_OFF",        MH, r'#define\s+MIDI_CC_ALL_NOTES_OFF\s+([0-9]+)'),
    ("MIR_LTMP_TEMPO_ONLY_BYTES",        LS, r'n\s*>=\s*([0-9]+)\s*&&\s*memcmp\(buf,\s*"LTMP"'),
    ("MIR_LTMP_TEMPO_PHASE_BYTES",       LS, r'if\s*\(\s*n\s*>=\s*([0-9]+)\s*\)'),
]

for name, rel, rx in CONST:
    mm = re.search(r'#define\s+' + re.escape(name) + r'\s+([^\s]+)', wit)
    if not mm:
        complain("mirror constant %s not found in witness .cpp" % name)
        continue
    try:
        mirror_val = parse_num(mm.group(1))
    except ValueError:
        complain("mirror constant %s has unparseable value %r" % (name, mm.group(1)))
        continue
    found = re.findall(rx, src[rel])
    if not found:
        complain("shipped value for %s not found (regex %s) in %s" % (name, rx, rel))
        continue
    try:
        shipped_val = parse_num(found[0])
    except ValueError:
        complain("shipped value for %s unparseable: %r" % (name, found[0]))
        continue
    if mirror_val != shipped_val:
        complain("MIRROR/SHIPPED VALUE DRIFT %s: mirror=%s shipped=%s (%s)" %
                 (name, mirror_val, shipped_val, rel))

# (label, shipped file, regex, expect_present). present=False => must be ABSENT.
FORMULA = [
    ("send_song_position kSppWrap=mask+1", LS, r'kSppWrap\s*=\s*static_cast<double>\(MIDI_SPP_UNITS_MASK\)\s*\+\s*1\.0', True),
    ("send_song_position std::fmod", LS, r'std::fmod\(sessionBeat\s*\*\s*MIDI_SPP_UNITS_PER_BEAT,\s*kSppWrap\)', True),
    ("send_song_position negative wrap", LS, r'if\s*\(units\s*<\s*0\.0\)\s*units\s*\+=\s*kSppWrap', True),
    ("send_song_position uint16(units)", LS, r'static_cast<uint16_t>\(units\)', True),
    ("OLD UB double->uint16 cast ABSENT", LS, r'static_cast<uint16_t>\(sessionBeat\s*\*\s*MIDI_SPP_UNITS_PER_BEAT\)\s*&\s*MIDI_SPP_UNITS_MASK', False),
    ("SPP LSB data byte", LS, r'spp_units\s*&\s*MIDI_DATA_BYTE_MASK', True),
    ("SPP MSB data byte", LS, r'spp_units\s*>>\s*MIDI_DATA_BYTE_SHIFT\)\s*&\s*MIDI_DATA_BYTE_MASK', True),
    ("SPP opcode F2 in array", LS, r'MIDI_SONG_POSITION_POINTER,', True),
    ("QuantumInfo.phraseBeat field (quantum FIX)", LSH, r'double\s+phraseBeat;', True),
    ("phraseBeat = beatAtTime(time,PHRASE_BEATS) (quantum FIX)", LS, r'info\.phraseBeat\s*=\s*state\.beatAtTime\(time,\s*PHRASE_BEATS\);', True),
    ("currentPhraseNumber from phraseBeat (quantum FIX)", LS, r'static_cast<int>\(std::floor\(info\.phraseBeat\s*/\s*PHRASE_BEATS\)\)', True),
    ("OLD currentPhraseNumber from sessionBeat ABSENT (quantum FIX)", LS, r'static_cast<int>\(std::floor\(info\.sessionBeat\s*/\s*PHRASE_BEATS\)\)', False),
    ("detectQuantumBoundary phrase phase", LS, r'info\.phaseWithinPhrase\s*=\s*state\.phaseAtTime\(time,\s*PHRASE_BEATS\);', True),
    ("detectQuantumBoundary sessionBeat q=16", LS, r'info\.sessionBeat\s*=\s*state\.beatAtTime\(time,\s*LINK_QUANTUM\);', True),
    ("detectQuantumBoundary crossed=true", LS, r'info\.crossedPhraseBoundary\s*=\s*true;', True),
    ("Phase.hpp phase() :34", PHASE, r'inline Beats phase\(const Beats beats, const Beats quantum\)', True),
    ("Phase.hpp quantumBins :45", PHASE, r'\(llabs\(beats\.microBeats\(\)\) \+ quantumMicros\) / quantumMicros', True),
    ("Phase.hpp nextPhaseMatch() :54", PHASE, r'inline Beats nextPhaseMatch\(const Beats x, const Beats target, const Beats quantum\)', True),
    ("Phase.hpp phaseDiff :58", PHASE, r'\(desiredPhase - xPhase \+ quantum\) % quantum', True),
    ("Phase.hpp closestPhaseMatch() :65-67", PHASE, r'return nextPhaseMatch\(x - Beats\{0\.5 \* quantum\.floating\(\)\}, target, quantum\);', True),
    ("Phase.hpp toPhaseEncodedBeats() :76-80", PHASE, r'return closestPhaseMatch\(beat, beat - tl\.beatOrigin, quantum\);', True),
    ("schedule skip-loop cap", LS, r'guard\s*<\s*MAX_CATCHUP_PULSES_PER_SCHEDULE', True),
    ("schedule skip-loop condition FIX (c) due <= nowLink", LS, r'due\s*<=\s*nowLink', True),
    ("OLD strict due < nowLink ABSENT (FIX c)", LS, r'due\s*<\s*nowLink', False),
    ("schedule resync-behind threshold", LS, r'dueNow\s*-\s*s_next_pulse\s*>\s*MIDI_CLOCK_RESYNC_THRESHOLD', True),
    ("schedule resync-behind no-burst log", LS, r'behind -- resyncing to beat', True),
    ("schedule re-anchor-ahead FIX (b) > 1", LS, r's_next_pulse\s*-\s*dueNow\s*>\s*1', True),
    ("OLD ahead > MIDI_CLOCK_RESYNC_THRESHOLD ABSENT (FIX b)", LS, r's_next_pulse\s*-\s*dueNow\s*>\s*MIDI_CLOCK_RESYNC_THRESHOLD', False),
    ("LTMP re-arm gate FIX (a) is if(s_evt_enabled)", LS, r'if\s*\(s_evt_enabled\)\s*\{', True),
    ("OLD phaseApplied&&enabled LTMP gate ABSENT (FIX a)", LS, r'if\s*\(phaseApplied\s*&&\s*s_evt_enabled\)', False),
    ("LTMP pendingRealign ONLY for phase shape (FIX a)", LS, r'if\s*\(phaseApplied\)\s*s_pending_realign\s*=\s*true;', True),
    ("LTMP handler clears nextPulseValid (FIX a)", LS, r's_next_pulse_valid\s*=\s*false;', True),
    ("LTMP phaseApplied=true", LS, r'phaseApplied\s*=\s*true;', True),
    ("LTMP setTempo", LS, r'ss\.setTempo\(s_tempoReqBpm', True),
    ("LTMP forceBeatAtTime(0.0)", LS, r'ss\.forceBeatAtTime\(0\.0', True),
    ("link_event_cb re-arms schedule_next_event", LS, r'schedule_next_event\(g_link->captureAppSessionState\(\)\);', True),
    ("link_event_cb if(s_evt_enabled)", LS, r'if\s*\(s_evt_enabled\)\s*\{', True),
    ("LTMP 12-byte shape", LS, r'n\s*>=\s*12\s*&&\s*memcmp\(buf,\s*"LTMP",\s*4\)\s*==\s*0', True),
    ("LTMP 28-byte shape", LS, r'if\s*\(\s*n\s*>=\s*28\s*\)', True),
    ("force_start 8s latch reference", LS, r'FORCE_START_AFTER_NO_PEERS_US\)', True),
    ("clock-enable transition", LS, r'if\s*\(clockEnabled\s*!=\s*s_evt_enabled\)', True),
    ("peer-loss Stop + All Notes Off log", LS, r'Link peer lost -- sent Stop \+ All Notes Off\.', True),
    ("peer-loss transport reset", LS, r's_transport_running\s*=\s*false;', True),
    ("peer-loss STOP msg", LS, r'const uint8_t stop_msg\[\]\s*=\s*\{\s*MIDI_STOP\s*\};', True),
    ("send_all_notes_off_all_channels call", LS, r'send_all_notes_off_all_channels\(\);', True),
    ("crossedPhraseBoundary block", LS, r'if\s*\(crossedPhraseBoundary\)\s*\{', True),
    ("SPP sent from quantumInfo.phraseBeat (quantum FIX)", LS, r'send_song_position\(quantumInfo\.phraseBeat\);', True),
    ("OLD send_song_position(sessionBeat) ABSENT (quantum FIX)", LS, r'send_song_position\(sessionBeat\);', False),
    ("Start/Continue ternary", LS, r's_transport_running\s*\?\s*MIDI_CONTINUE\s*:\s*MIDI_START', True),
    ("realign && (is_playing||force_start)", LS, r'if\s*\(s_pending_realign\s*&&\s*\(is_playing\s*\|\|\s*force_start\)\)', True),
    ("TTMP beatOriginUb formula", LS, r'beatOriginUb\s*=\s*\(int64_t\)\(beats\s*\*\s*MICROBEATS_PER_BEAT\)', True),
    ("TTMP mpb rounded", LS, r'mpb\s*=\s*\(int64_t\)\(MICROS_PER_MINUTE\s*/\s*bpm\s*\+\s*0\.5\)', True),
    ("LTMP bpm recovery", LS, r'bpm\s*=\s*MICROS_PER_MINUTE\s*/\s*\(double\)mpb', True),
    ("phase quantum from microbeats", LS, r's_phaseReqQuantum\s*=\s*\(double\)quantumUb\s*/\s*MICROBEATS_PER_BEAT;', True),
    ("window structural reads m_live", BE, r'm_pendingLoopSteps\s*=\s*ctl::loopBarsForDial\(m_live\[SET_WINDOW\]\)', True),
    ("window NEVER reads m_frame", BE, r'loopBarsForDial\(m_frame\[SET_WINDOW\]\)', False),
    ("Tempo.hpp int64 truncation", TEMPO, r'static_cast<std::int64_t>\(60\.\s*\*\s*1e6\s*/\s*bpm\(\)\)', True),
    ("Tempo.hpp bpm reconstruction", TEMPO, r'60\.\s*\*\s*1e6\s*/\s*static_cast<double>\(microsPerBeat\.count\(\)\)', True),
    ("Link.hpp beat/phase invariant", LINKH, r'fmod\(beatAtTime\(t,\s*q\),\s*q\)\s*==\s*phaseAtTime\(t,\s*q\)', True),
    # ---- FIX (d): F8 emitted INSIDE if(s_evt_enabled) in link_event_cb ----
    ("link_event_cb F8 gated on s_evt_enabled (FIX d)", LS, r'if\s*\(s_evt_enabled\)\s*\{\s*const uint8_t timing_msg\s*=\s*MIDI_TIMING_CLOCK;', True),
    ("single F8 send site", LS, r'send_midi_bytes\(&timing_msg,\s*1\);', True),
    # ---- FIX (e): process() beat<0 guarded note release ----
    ("process beat<0 guarded allNotesOff (FIX e)", BE, r'if\s*\(beat\s*<\s*0\.0\)\s*\{\s*if\s*\(!m_activeNotes\.empty\(\)\)\s*allNotesOff\(\);', True),
    ("process beat<0 resets m_lastTick (FIX e)", BE, r'm_lastTick\s*=\s*-1;\s*\n\s*return;', True),
    # ---- FIX (f): resync m_lastTick = now - 1, no early return ----
    ("resync m_lastTick = now - 1 (FIX f)", BE, r'm_lastTick\s*=\s*now\s*-\s*1;', True),
    ("OLD resync early-return ABSENT (FIX f)", BE, r'allNotesOff\(\);\s*\n\s*m_lastTick\s*=\s*now;\s*\n\s*return;', False),
    # ---- FIX (g): sendLfoAndFilter absolute tick ----
    ("sendLfoAndFilter called with absolute t (FIX g)", BE, r'sendLfoAndFilter\(t\);', True),
    ("sendLfoAndFilter signature absTick (FIX g)", BE, r'void BassEngine::sendLfoAndFilter\(int64_t absTick\)', True),
    ("LFO bars uses absTick (FIX g)", BE, r'static_cast<float>\(absTick\)\s*/\s*static_cast<float>\(ctl::kTicksPerBar\)', True),
    ("OLD sendLfoAndFilter(p) ABSENT (FIX g)", BE, r'sendLfoAndFilter\(p\);', False),
    ("OLD LFO bars loopTick ABSENT (FIX g)", BE, r'static_cast<float>\(loopTick\)\s*/\s*static_cast<float>\(ctl::kTicksPerBar\)', False),
    # ---- FIX (h)+(i): setDial / touchSetting ----
    ("setDial clamps into m_live", BE, r'm_live\[c\]\s*=\s*std::clamp\(v01,\s*0\.0f,\s*1\.0f\);', True),
    ("touchSetting calls startRecording (FIX i)", BE, r'm_track\[c\]\.startRecording\(m_live\[c\]\);', True),
    ("OLD touchSetting setTouched(true) ABSENT (FIX i)", BE, r'm_track\[c\]\.setTouched\(true\);', False),
    # ---- FIX (j): LoopTrack::startRecording declaration + definition ----
    ("LoopTrack::startRecording declared (FIX j)", CMH, r'void\s+startRecording\(float value\);', True),
    ("LoopTrack::startRecording defined (FIX j)", CMC, r'void LoopTrack::startRecording\(float value\)', True),
]

for label, rel, rx, present in FORMULA:
    found = re.search(rx, src[rel]) is not None
    if present and not found:
        complain("FORMULA DRIFT (missing): %s [regex %s] in %s" % (label, rx, rel))
    if (not present) and found:
        complain("FORMULA DRIFT (unexpected presence): %s [regex %s] in %s" % (label, rx, rel))

# SPP-before-Start ordering inside the crossedPhraseBoundary block.
i = src[LS].find("if (crossedPhraseBoundary) {")
if i < 0:
    complain("crossedPhraseBoundary block not found (cannot verify SPP-before-Start)")
else:
    block = src[LS][i:i + 900]
    jspp = block.find("send_song_position(quantumInfo.phraseBeat)")
    jstart = block.find("s_transport_running ? MIDI_CONTINUE : MIDI_START")
    if jspp < 0 or jstart < 0 or jspp >= jstart:
        complain("ORDERING DRIFT: SPP must precede Start in crossedPhraseBoundary (spp_idx=%d start_idx=%d)" %
                 (jspp, jstart))

# Start/Continue must be sent from exactly ONE site (only at a boundary).
nsend = len(re.findall(r'send_midi_bytes\(&start_msg,\s*1\)', src[LS]))
if nsend != 1:
    complain("Start/Continue send-site count drifted: expected 1, found %d" % nsend)

# s_pending_realign must be set on the phase-req, peer-edge, clock-enable, playing paths.
nrealign = len(re.findall(r's_pending_realign\s*=\s*true;', src[LS]))
if nrealign < 3:
    complain("s_pending_realign=true site count drifted: expected >=3, found %d" % nrealign)

# FIX (a): the LTMP handler block must gate on s_evt_enabled, clear
# s_next_pulse_valid, set pendingRealign ONLY when phaseApplied, and re-arm.
i = src[LS].find("g_link->commitAppSessionState(ss);")
if i < 0:
    complain("FIX (a): LTMP handler commitAppSessionState(ss) not found")
else:
    blk = src[LS][i:i + 600]
    for label, rx in [
        ("if (s_evt_enabled) gate", r'if\s*\(s_evt_enabled\)\s*\{'),
        ("clears s_next_pulse_valid", r's_next_pulse_valid\s*=\s*false;'),
        ("pendingRealign ONLY for phase", r'if\s*\(phaseApplied\)\s*s_pending_realign\s*=\s*true;'),
        ("re-arms schedule_next_event", r'schedule_next_event\(g_link->captureAppSessionState\(\)\);'),
    ]:
        if not re.search(rx, blk):
            complain("FIX (a) LTMP handler drift: %s missing" % label)

# FIX (d): the single F8 send must sit INSIDE (after) the if(s_evt_enabled) gate.
jgate = src[LS].find("const uint8_t timing_msg = MIDI_TIMING_CLOCK;")
jf8 = src[LS].find("send_midi_bytes(&timing_msg, 1);")
jif = src[LS].rfind("if (s_evt_enabled) {", 0, jgate) if jgate >= 0 else -1
if not (jif >= 0 and jgate >= 0 and jf8 >= 0 and jif < jgate < jf8):
    complain("FIX (d) ordering drift: F8 must be emitted INSIDE if(s_evt_enabled)")

# FIX (h): setDial must assign m_live[c] BEFORE touchSetting(c).
msd = re.search(r'void BassEngine::setDial\([^)]*\)\s*\{([\s\S]*?)\n\}', src[BE])
if not msd:
    complain("FIX (h): BassEngine::setDial definition not found")
else:
    body = msd.group(1)
    ilive = body.find("m_live[c] = std::clamp")
    itouch = body.find("touchSetting(c);")
    if not (ilive >= 0 and itouch >= 0 and ilive < itouch):
        complain("FIX (h) ordering drift: m_live[c] must be assigned BEFORE touchSetting(c)")

# FIX (j): LoopTrack::startRecording body must seed hold_ at the current value.
msr = re.search(r'void LoopTrack::startRecording\(float value\)\s*\{([\s\S]*?)\n\}', src[CMC])
if not msr:
    complain("FIX (j): LoopTrack::startRecording definition not found")
else:
    body = msr.group(1)
    for label, rx in [
        ("clears valid_[]", r'valid_\[i\]\s*=\s*false;'),
        ("clears persistent_", r'persistent_\s*=\s*false;'),
        ("seeds hold_=value", r'hold_\s*=\s*value;'),
        ("sets holdValid_", r'holdValid_\s*=\s*true;'),
        ("sets touched_", r'touched_\s*=\s*true;'),
    ]:
        if not re.search(rx, body):
            complain("FIX (j) startRecording body drift: %s missing" % label)

if failures:
    print("PARITY GUARD: %d drift(s) detected -- REFUSING to build the witness." % failures)
    sys.exit(1)
print("PARITY GUARD OK: %d constants + %d formulas + ordering/body checks match shipped source." %
      (len(CONST), len(FORMULA)))
PY

# ---- BUILD (link the ACTUAL shipped host-compilable sources) ----------------
cp "$root/main/bass_engine.cpp" "$root/main/bass_engine.h" \
   "$root/main/control_model.cpp" "$root/main/control_model.h" \
   "$here/phrase_predictability_witness.cpp" "$tmp/"
cp -r "$here/hoststub/." "$tmp/"

g++ -std=c++17 -Wall -Wextra -Werror -O1 -I"$tmp" -o "$tmp/witness" \
    "$tmp/phrase_predictability_witness.cpp" "$tmp/bass_engine.cpp" "$tmp/control_model.cpp"
"$tmp/witness"
