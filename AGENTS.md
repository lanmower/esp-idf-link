# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file. Durable cross-session facts live here, not in
code. Do not extend this by editing `CLAUDE.md`. Re-verified against the tree
2026-10-10.

## esp-idf-link <-> aloopprime mesh: paired invariants (change BOTH or the mesh splits)

This project (ESP32 "ticker" box) and `../aloopprime` (Pi 4 hardware looper) form
ONE ad-hoc single-AP mesh so Link's multicast discovery reaches every device. No
credential provisioning: exactly one device hosts the open SSID `ticker`, the rest
join as stations. Every value below exists in BOTH trees; changing one alone
splits the mesh with no error on either side.

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| Mesh SSID | `main.cpp` `wifi_scan_best_bssid/sta/link_ap("ticker")` | `hostapd.conf`+`wpa_supplicant.conf` `ssid=ticker` |
| Auth | `wifi_connect_sta("ticker","")` open, empty password | open (`key_mgmt=NONE`; `wpa=` commented out) |
| AP address / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`:193`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link — cannot drift) |
| Link quantum | `main.h:55` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp:43` `enableStartStopSync(false)` | `link_bridge.cpp` `enableStartStopSync(false)` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` (`main.h:56`) is NOT the quantum — it is this project's own
SPP/transport-correction boundary (16 bars). It deliberately differs; do not
"align" it.

### Host election must stay MAC-ordered on both sides

Two devices cold-booting together can each scan before the other's AP exists, so
a naive "nothing found -> host" makes BOTH host — two isolated L2 domains Link
can never cross. This project holds for a duration strictly monotonic in its own
STA MAC (`HOLD_MAX_MS = 6000`), rescanning every second; the scan is staggered by
`mac[5] * 15` ms. `../aloopprime/src/net/autoap.sh` mirrors it (0-6 s,
lowest-wins). Invert the convention here and it MUST be inverted there in the same
change.

Both supervisors yield to a strictly-lower `ticker` BSSID, never while clients
are attached.

**The self-BSSID guard is explicit, not incidental.** The yield path compares
candidates against the box's own AP MAC read from the interface
(`esp_wifi_get_mac(WIFI_IF_AP, ...)`), not the STA MAC. The earlier form relied on
the SoftAP MAC happening to sort above the STA MAC; any MAC-derivation change, IDF
bump or override would make the box yield to itself and flap AP->STA->AP every 2 s
supervisor tick. Keep the interface read — `esp_read_mac()` reads eFuse and is
documented as possibly differing from it.

### By-the-book Ableton Link checklist (both projects)

From a full audit of both trees against Link's own headers.

- **Thread-correct session-state API.** `captureAppSessionState()` /
  `commitAppSessionState()` off the audio thread; the `AudioSessionState` variants
  only on it. This project uses the App variants throughout.
- **Start/stop sync is OFF on both sides on purpose: the mesh is clock-only.** A
  stop on one device must not stop the whole mesh. This project still CONSUMES
  transport (`state.isPlaying()` -> MIDI Start/Stop/Continue + all-notes-off) and
  never calls `setIsPlaying`. With sync disabled `isPlaying()` reads true for the
  whole session, so downstream gear gets one Start and never a Stop.
  `../aloopprime` was flipped to `false` 2026-10-10 for this; going back needs
  BOTH flipped in one change plus a firmware flash here.
- **The three notification callbacks** (`setNumPeersCallback`,
  `setTempoCallback`, `setStartStopCallback`) run on a Link-managed thread,
  documented **Realtime-safe: no**. Bounded logging / atomics only.
- **Tempo authority.** `setTempo` rewrites tempo for EVERY peer. This project sets
  tempo only on an explicit LTMP command; `../aloopprime` also refuses to propose
  when peers own the tempo. Do not make either an unconditional writer.
- **Quantum is a shared constant** (`LINK_QUANTUM 16.0` here, `kLinkQuantum`
  there). They move together.
- **Interface readiness is a real race; this project models the fix.** The 500 ms
  settle before constructing Link and the ~10 s IGMP re-assert are what
  `../aloopprime` was corrected against: a single IGMP join at GOT_IP can race
  netif readiness, so membership silently does not stick.

### The vendored Link is a FORK, checked in — not a submodule

`components/link-esp/link` is a **patched copy of Ableton Link committed directly
into this repo** (1506 tracked files, no nested `.git`), NOT a submodule. The
stale `.gitmodules` entry was removed because it invited a `git submodule update`
that would clobber the patch below with no compile error and silently kill ESP
peer discovery.

The divergence is the multicast relay hook:
`link/include/ableton/platforms/asio/Socket.hpp` declares `extern "C"
wifi_link_multicast_forward(...)` and calls it on every send, so
`wifi_config.cpp`'s relay can unicast-copy each Link discovery datagram across the
SoftAP host/station boundary that does not carry multicast. **Any Link version
bump must re-apply that hook** — losing it compiles perfectly and fails only at
runtime, as silent non-discovery.

There is no root `.gitmodules`, so `Dockerfile`, `setup.sh` and
`.github/workflows/build.yml` deliberately do NOT initialise submodules — no-ops
today, but the mechanism that would overwrite the fork the moment one came back.
Do not re-add it. Two nested ones survive, inert:
`components/link-esp/link/.gitmodules` is upstream's own (kept, so the fork's
divergence stays limited to the hook); `components/link-esp/.gitmodules` was this
repo's stale `[submodule "link"]` entry and is deleted.

## Metronome and MIDI clock are hardware-scheduled, not tick-emitted

The click and the 24 ppqn clock come from a **hardware-scheduled `esp_timer`
one-shot armed to `SessionState::timeAtBeat()` of the NEXT pulse**, never from the
4 kHz gptimer tick (`LINK_TICK_PERIOD 250` us). A tick only discovers a beat AFTER
it passed, so tick emission costs every click the tick's wake-up latency plus
WiFi/lwIP/logging contention — audible jitter. Raising the tick rate cannot
tighten the click; only the esp_timer one-shot sets the edge.

- **Link's ESP clock IS `esp_timer_get_time()`**, so a Link-clock microsecond
  timestamp is already an esp_timer deadline — no offset, no drift correction.
- **Lateness is instrumented.** `MetroStats` (`fired`/`last`/`worst`/`mean`/`rms`)
  rides in the UDP status reply as a `metro` object, so click stability is
  measurable from a peer with no scope.
- **The next click's PWM pitch is primed while the buzzer is silent**, so the ON
  edge is two duty writes instead of a `ledc_timer_config()` reconfigure landing
  on the beat.
- **No-burst invariant: the scheduler SKIPS pulses, it does not catch up.**
  `MIDI_MAX_CLOCKS_PER_TICK` is gone; the pulse counter steps past every pulse
  whose due time is already behind (capped `MAX_CATCHUP_PULSES_PER_SCHEDULE =
  1024`) — a burst of clocks reads as a tempo spike to any downstream PLL.
- `MIDI_CLOCK_RESYNC_THRESHOLD` is **log-only**: one warning, no burst.
- A watchdog re-arms the alarm from the tick when a pulse is really overdue — an
  alarm that never got re-armed would stop the clock silently.

Build trap: **`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults` is
load-bearing, not a tuning knob.** The default 10 is exhausted by the app's own
UDP sockets (Link relay, LCLK/TTMP broadcast, tempo listener, discovery
forward), leaving too few for Link's per-interface discovery gateway, which then
fails with EMFILE and never broadcasts -> peers never discover each other. 16 is
the ESP32 maximum.

## Firmware reaches the ticker over USB only

No OTA and no serial console on this machine: a firmware change reaches the device
only over USB.

- CI builds on push to any branch in `espressif/idf:latest` (`Dockerfile:1`,
  `.github/workflows/build.yml:13`) — **not a pinned 6.2**. On a green `main` it
  commits the three images under `build/` ("ci: update firmware binaries [skip
  ci]"). Flashable images come from git, not a local build.
- `node flash-ticker.js [COMx]` drives a real COM port with esptool 5.x. Offsets:
  0x1000 bootloader, 0x8000 partition table, app at the first app partition of
  the BUILT table (0x20000 with `partitions_large.csv`). **The app offset is
  DERIVED, never hardcoded** — `flash-ticker.js`, `flash.sh`,
  `tools/flash-usbip.py` all read `build/partition_table/partition-table.bin`:
  32-byte entries, little-endian magic `0x50AA`, then type(1) subtype(1)
  offset(4) size(4) label(16); app entries have type == 0. **Never parse
  `partitions_large.csv`**: it leaves every offset but nvs's blank because the
  build computes them, so parsing yields `int('')`. `--list` lists ports and
  refuses to guess when more than one serial port exists — this machine has two
  Bluetooth COM ports that are not the ESP32.
- esptool 5.x: `write-flash` is hyphenated (`write_flash` accepted but
  deprecated) and **`--verify` was dropped** — it always verifies ("Hash of data
  verified" per image). Passing `--verify` aborts with "No such option" and
  flashes nothing.
- The SPIFFS `storage` partition is flashed separately: `flash_midi_data.sh`
  builds a `0x19000` image from `./data` and writes it at **`0x317000`**. That
  offset IS derivable — running sum of sizes, app partitions rounded up to
  0x10000: nvs 0x011000, phy_init 0x017000, app0 0x020000, app1 0x1A0000,
  storage 0x317000 +0x19000 — but re-derive it from the built binary if the
  layout changes.

### No BOOT hold: `python tools/flash-usbip.py` flashes it hands-free

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 Windows driver limitation,
not the board. `tools/flash-usbip.py` talks CH341 over USB/IP (usbipd-win) behind
a pyserial-shaped shim, so the WCH driver never loads. `--dry` = enter download
mode, prove sync, reboot; bare = flash and boot. Measured 2026-10-10: exit 0,
~49 s wall, app written in ~19.8 s at 460800 (~510 kbit/s), "Hash of data
verified", boots `boot:0x13`. No BOOT hold, no WSL.

Four counter-intuitive facts make it work:

1. **The D1 R32's auto-reset is DIFFERENTIAL.** Each transistor is driven by the
   DTR#-RTS# difference: EN low only at (DTR# high, RTS# low) -> `0x40`; IO0 only
   at (DTR# low, RTS# high) -> `0x20`; both asserted (`0x60`) or both clear
   (`0x00`) conducts NEITHER — the opposite of esptool's hard-coded convention,
   hence every esptool reset gave `boot:0x13`. Entry is the flip **0x40 -> 0x20**;
   verified `boot:0x3`. Leave with `0x40` then `0x00`.
2. **The CH341 withholds the first replies.** One sync frame gets no answer for
   seconds, then past replies surface behind a later FULL-SIZE OUT. After the
   first, latency is ~20 ms. esptool sends one frame on a 0.1 s deadline, so prime
   with repeated framed sync frames (`SYNC_TIMEOUT` 2 s); typically 2-8.
3. **The first attach back often fails.** usbipd keeps the device claimed after a
   client disconnects, so the first `OP_REP_IMPORT` returns `status=4`;
   `open_port()` retries (10 x 6 s) and the second lands.
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`), not a
   board property.** Re-read it per host and after any replug; `usbipd bind
   --busid <busid>` once per boot or the tool cannot attach.

`tools/flash-usbip.py` passes **`--after no-reset-stub`, never `--after
no-reset`**, and guards `esptool.main()` with `except BaseException` (commit
`09da1052`). `no-reset` calls `soft_reset(True)` while the port is at the flash
baud with the ROM only speaking 115200 -> intermittent `FatalError: No more data
to read from the serial port` AFTER all three images verified; `FatalError` is
not `SystemExit`, so `except SystemExit` misses it and EN is never pulsed.

**Hands-free serial read** (how the SPIFFS mount line below was captured: 28 s at
115200, no BOOT hold, no WSL): `UsbipPort()` from `tools/usbip-port.py` MUST have
`.init()` called after construction — skipping it returns 4096 zero bytes per
bulk IN and floods the output. Then `hs(HOLD_EN)` -> short drain ->
`hs(RELEASE_BOTH)` resets the board; `drain(seconds)` returns the boot log. The
filename has a hyphen, so import it with
`importlib.util.spec_from_file_location`, not `import`.

### Flash baud: 460800 default; the baud numbers are USB/IP-only

The ROM only speaks 115200, so `enter_and_sync` pins it (`ROM_BAUD`); `--baud`
applies only after esptool uploads its RAM stub and re-times the port. On the
~1.26 MB app image, **USB/IP path only**: 115200 -> 72.2 s (139.9 kbit/s) vs
460800 -> 19.4 s (519.9 kbit/s), repeatable. 921600 dies just after `Changing baud
rate to 921600... Changed.` with `FatalError: No more data to read` from
`reset_chip -> soft_reset -> flash_begin`: nothing written, chip left in the stub;
`--dry` recovers it. **921600 is UNMEASURED on `flash-ticker.js`'s real COM port**
(PRD `flash-ticker-js-921600-unmeasured`) — not proven broken there. esptool
leaves the port at the flash baud, so `boot_app()` resets to `ROM_BAUD` before
pulsing EN — else no `boot:` line appears at all.

Layers: `tools/ch341.py` (raw USB/IP; `SET_CONFIGURATION` first or bulk IN is
dead), `tools/usbip-port.py` (bulk OUT + pyserial surface),
`tools/flash-usbip.py` (entry, priming, esptool); `tools/usbip-boot-entry.py` and
`tools/usbip-sync-latency.py` are the instruments behind facts 1-2, uncalled.
Benign: `flash_id()` reads `0xFFFFFF` in download mode, so esptool prints "Failed
to communicate with the flash chip" and auto-detects no size — erase/program still
go through the ROM.

## Mesh UDP protocol and MIDI emission

Ports, all little-endian `int64` payloads on both ends:

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group, ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets group tempo and phase |
| 20812 | esp -> group, ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |

- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per
  interface** (`esp_netif_next_unsafe`). We may be AP or STA (boot race) and a raw
  socket with no `IP_MULTICAST_IF` exits the wrong interface on a dual-netif
  device — Link's own socket reached the peer (it sets egress) while ours did not.
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers multicast but
  NOT unicast-to-self, so Link's ping/pong never completes there (peers=0). LCLK
  gives it phase; TTMP carries tempo back. Standard Link apps ignore all three
  ports.
- **The clock broadcast is rate-limited to one packet per 20 ms** (timeline
  100 ms): a 4 kHz flood saturates the Pi's single radio-RX drain and starves its
  control plane.
- **Tempo is bounded to Link's real range, 20..999 bpm** (Ableton's Test Plan
  TEMPO-4 names both ends). A 400 ceiling silently dropped legitimate tempos.
- `beatAtTime` is quantum-insensitive while `phaseAtTime` is not, so the timeline
  broadcast uses `LINK_QUANTUM`.
- LTMP/phase requests are applied on the Link task, not the listener task:
  `captureAppSessionState`/`commitAppSessionState` need a single owner.
- Status is request/response, not a broadcast: it costs nothing when idle and
  cannot pollute the Link multicast group.

MIDI emission — one path, no per-device clock code; byte values are named
constants in `main.h` (`0xF8` clock, `0xFA` start, `0xFC` stop, `0xFB` continue,
`0xF2` SPP, CC123 all-notes-off), rates `MIDI_PULSES_PER_QUARTER_NOTE = 24` and
`MIDI_SPP_UNITS_PER_BEAT = 4`.

- Continuous 24 ppqn clock; at the 16-bar phrase boundary only, SPP + Start/
  Continue; Stop + CC123 on all 16 channels on transport stop and on peer loss
  (`send_all_notes_off_all_channels`). SPP counts MIDI beats
  (sixteenths): one Link beat = 4 units, 14-bit, LSB first.
- Realtime bytes are single-byte and may legally interleave a running-status
  message, so the buzzer path and the clock path never corrupt each other.
- Note-offs go out as velocity 0; CC123 on stop is what keeps RC-505 MK2 loops
  from sticking.
- Target behaviour that constrains the design: KO2 locks to ext clock, needs
  Start, is SPP-sensitive (hence one SPP per phrase, not per 4 beats); Volca Drum
  follows the pulse and ignores SPP; MicroKorg/MiniNova arp, delay and LFO need it
  non-bursting; Micron needs Start and stays in phrase when Start is
  phrase-aligned; RC-505 MK2 locks to clock+Start and repositions on SPP.
- Start is deferred to the next phrase boundary. `s_transport_running` records what
  downstream gear was last TOLD (Start vs Continue) — it is not `isPlaying()`.
- Force-start waits 8 s, not 5 s, so two co-booting devices can discover each other
  before either free-runs at an independent phase.

## HTTP clip server is live; the mesh tops out at 8 stations

`main/network_midi.cpp` IS in the build (see SRCS note) and `network_midi_init()`
starts an `esp_http_server` on **port 8080** from `app_main`: `POST /upload/*`
writes the body to `/spiffs/loops/<name>`, `GET /info` returns
`{"device_ip":...}`, `POST /clear` deletes every `*.mid` there. Two bounds are
invisible at the call sites: the server reserves **4 sockets** of the 16-socket
budget, and the SoftAP allows **8 stations** (`cfg.ap.max_connection = 8` =
`MAX_AP_STA_IPS`) — 8 is also all the multicast forwarder can unicast to, so a
9th associates but never gets relayed Link traffic. WiFi power-save is off
(`WIFI_PS_NONE`): a dozing station misses multicast. Do not re-enable it.

### 8080 audit (2026-10-10): conclusions

Provable in `network_midi.cpp`: the URI wildcard is one segment and re-checked at
runtime (`is_single_clip_name`), so `/` and `..` can no longer escape
`/spiffs/loops` — and SPIFFS has no real directories, so an escaped file could
never be listed or cleared. Empty bodies are rejected before `fopen`; an
incomplete upload unlinks the partial file and answers 500; `new` is
`std::nothrow` under `CONFIG_COMPILER_CXX_EXCEPTIONS=y`; `/info` re-reads the IP
per request because the netif is recreated on every AP<->STA role change;
`network_midi_init()` is idempotent. **Still unchecked: `filepath` truncation in
`clear_handler`** (checked only in upload) — 512 bytes against
`CONFIG_HTTPD_MAX_URI_LEN=8192`.

**SPIFFS IS mounted and both storage endpoints are LIVE** (commit `9568578`;
`mount_clip_storage()`, called first from `network_midi_init`; `spiffs` added to
`main/CMakeLists.txt` REQUIRES). Proven on-device 2026-10-10 — USB/IP reset then
28 s of serial at 115200:

    I (15802) NETWORK_MIDI: Clip storage mounted at /spiffs: 46937 of 86846 bytes used
    I (15802) NETWORK_MIDI: Network MIDI server initialized on 192.168.4.1:8080

The older "nothing mounts SPIFFS, both storage endpoints are inert" row is void.
`format_if_mount_failed = false` on purpose — mounting must never format. Success
is quiet; failure logs `ESP_LOGE`. Uploads answer **503** when unmounted and
**413** when the body exceeds `total - used` (400 for a bad or empty name/body) —
the missing size cap now exists.

Checked, NOT defects: an embedded NUL (`%s` stops at NUL — a shortened path, never
an overflow); the existing error returns. esp_http_server runs all sessions in one
task, so handlers cannot interleave. Still open, not code: no auth on an open
SSID; `network_midi_start()` a no-op vs `_stop()` (both dead). `/clear` filters
`d_type == DT_REG`, which SPIFFS VFS may leave `DT_UNKNOWN`; `strstr(d_name,
".mid")` matches anywhere, so `notes.midi` dies too.

## MIDI file player (`main/midi_file.cpp`, files on SPIFFS)

- Parser relies on **MIDI running status**: a data byte with no status byte in
  front repeats the previous status. Note-on with **velocity 0 IS a note-off**; a
  note held at end-of-track is released implicitly.
- `process()` walks notes in `startBeat` order and early-breaks, so **the sort
  order is load-bearing**, not cosmetic.
- Trigger window is 0.03 beat; `playedNotes`/`sentCCs` dedup fires each once.
- Loop length rounds to the nearest quantum within 0.1, else up (ceil).
- **SPIFFS has no real directories**, so `setFolder()` cannot `opendir` a
  subfolder: it scans `/spiffs` and matches a name prefix.
- `updateTempo`'s 120 bpm reference and 0.5 bpm threshold are dormant
  (`syncToBpm` is false). See `CLAUDE.md` for why.
- Warts, left alone: `parseFile()` closes the file twice; the built-in default
  MIDI file's `MTrk` declares 19 bytes but writes 12, and its header encodes
  format 1. (Unbuilt; SPIFFS itself IS mounted.)

## Input and buzzer wiring facts

GPIO numbers, from `main/main.h` — a pin number cannot be expressed in code that
reads the macro.

| Function | Pad / channel | GPIO |
|---|---|---|
| Touch 1 | `TOUCH_PAD_NUM0` | 4 |
| Touch ARP | `TOUCH_PAD_NUM5` | 12 |
| Touch REV | `TOUCH_PAD_NUM6` | 14 |
| Touch FILT | `TOUCH_PAD_NUM7` | 27 |
| Pot 1 | `ADC_CHANNEL_6` | 34 |
| Pot 2 | `ADC_CHANNEL_0` | 39 |
| Buzzer (LEDC) | `BUZZER` | 13 |
| MIDI UART2 TX / RX | `MIDI_TX_PIN` / `MIDI_RX_PIN` | 17 / 16 |

- Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships
  `touch_sensor`; this tree is on the legacy one). A pad reads LOW when touched;
  ARP is read first so its press latency stays lowest.
- Pots are raw ADC 0-4095 -> MIDI 0-127; a slow EMA tracks the stable center
  beside the fast one used for control. In the MIDI file player **pot1 = note
  length (0..2), pot2 = velocity (0..1)**; the accessors are still
  `getPot1Value`/`getPot2Value` despite the names.
- `DOUBLE_TAP_TIME_MS = 300` / `HOLD_TIME_MS = 200` are **dead**: nothing reads
  either, so they are not the pad gesture timings.

## Constants whose constraint is not visible in the code

Constants with the right value but no visible rule.

- `MAX_ARP_INDEX_WRAP = 128` (`arp_constants.h`) is referenced nowhere. If wired
  up it must stay **larger than the longest reasonably expected progression
  sequence** — that bound, not 128, is the requirement.
- **E-Slew must stay derivable from sheer, anchored at 104.** `reset` sets sheer to
  0, so a derivation returning anything but 104 at sheer 0 makes the reset
  overwrite what `setSidechainPattern()` just sent. `gateESlewForSheer(sheer)` is
  `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax` — 104 at sheer 0
  rising to 127. The old `64 + sheer/2` gave 64. 104 is not magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`.** `gateWetDryForDepth(depth)`
  returns clamped depth and `SIDECHAIN_DEFAULT_DEPTH == SIDECHAIN_DEPTH_MAX ==
  127`, because MiniNova **CC 91 is wetness with 127 = full Gator** (User Manual
  v1.01: the Slot's FX Amount "needs to be at maximum - 127"). The old
  `127 - depth` paired the deepest ducking with a bypassed gate.
  `setFxSlot1Level()` sends the same CC 91. All of it lives outside SRCS, so none
  of it runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: the trailing
  `{3,5}` is deliberate padding so the row matches its neighbours; only the first
  five are read. Do not "fix" it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the
  anchorMotif clamp in `bass_engine.cpp` — recorded nowhere else, and neither file
  includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones and not MIDI notes (corrected in
  `c0fc974`). Intents exist nowhere else: `{0,5,3,6}` i-VI-iv-VII, `{0,6,5,6}`
  i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}` i-VI-VII-V ("dark
  cadence"), `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` the pedal row (a tonic drone).

## Synth CC/NRPN facts that are not derivable from the code

- **MicroKorg**: LFO2 shape is CC 75 (`kCcLfo2Shape`); delay sync is CC 13
  (`kCcDelaySync`), 0 = off and 1..32 are sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never
  confirmed on hardware** — the one NRPN in the tree with no verified source.
  Filter bypass (NRPN 0/60 value 0) is deliberately NOT sent by
  `deactivateFilter()`, which unpatches only the LFO.
- **MiniNova "Table 3"** is the source of `setDelaySyncRate` and `setLfoRateSync`;
  `main/lfo_constants.h` carries only a **subset** of the LFO rate-sync row (NRPN
  0/86) — do not treat that list as exhaustive.
- `SynthInterface` contracts that exist only in the implementations:
  `activateDelay()`/`activateReverb()` select Delay 1 / Reverb 1 in **FX Slot
  1**; `setFxSlot1Level()` is **CC 91**; `activateFilter()` implicitly selects
  **LP24**; the LFO methods assume **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` maps to MIDI `0..127` with **64 = zero**,
  so `unpatchLfoFromFilter()` writes 64, not 0.
- Note-off velocity 0 is the **default**, not a call-site convention:
  `sendNoteOff(uint8_t note, uint8_t velocity = 0)` is declared identically in
  `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The two surviving
  `= 64` defaults in `synth_interface.h` (`activateFilter`'s cutoff,
  `patchLfoToFilter`'s depth) are legitimately 64 and must stay.

## `bassline_interpreter` must stay host-compilable

It includes only its own header plus `<algorithm>` `<cmath>` `<cstring>` — no
ESP-IDF headers — so it compiles on the host with `g++ -std=c++17`, the only way
to exercise its DP and scale logic off-device. Do not add ESP-IDF includes. On
MinGW a host build also needs `-D_USE_MATH_DEFINES`, or `M_PI` is undefined.

## Five `main/*.cpp` files are not in the build

`main/CMakeLists.txt` SRCS names 11 translation units, and `network_midi.cpp` IS
one of them — compiled and linked, despite sitting below a blank line away from
the other ten, which is why earlier audits missed it. `effect_arp.cpp`,
`effect_filter.cpp`, `effect_handler.cpp`, `effect_sidechain.cpp` and
`midi_file.cpp` are NOT among them, and nothing `#include`s a `.cpp`, so that
cluster is reachable only from itself (`effect_handler.h` only by the four effect
`.cpp`s; `midi_file.h` only by `effect_arp.h` and `midi_file.cpp`). So the
MIDI-file-player behaviour in `CLAUDE.md` is not running on the device and edits
there cannot break the build. Adding them to SRCS is a behaviour change.

## CI autobump: how it commits, and the hazard it used to have

`.github/workflows/build.yml` commits the three `.bin` files and pushes them. The
retry loop **records the sha it built**, then `git reset --hard FETCH_HEAD`,
restores the three `.bin`s from that sha and commits with a 3-path pathspec, up to
5 attempts. **`reset --soft` is NOT usable here**: it moves HEAD to the fetched
tip but leaves the index on the tree this checkout built, so the commit diffs
`built_tree` minus `remote_tip` and silently reverts every source commit that
landed in between. A rebase would MERGE the previous build's `.bin`s and die on
"Cannot merge binary files".

That was the live failure mode, fixed at `bc633fc` (merge `3b9bcd3`): of 76
autobump commits stat-checked, **16 touched a non-`.bin` path** — all before
`bc633fc`; every autobump since (`ffa49c7`, `4caca07`, `b104d79`, `e667d4e`) is
bins-only. `49cd311` removed 264 lines of `AGENTS.md` and 60 of
`network_midi.cpp` while shipping two `.bin`s; `9eb06bd` reverted 13 paths;
`1c3fcb1`/`4f06700` and `8aec1f0`/`28268b5` are exact inverse mirror pairs — that
is how a whole commit silently vanished.

**Residual rule: any commit touching a non-`.bin` path must be re-verified
against HEAD after a pull** — a green build never proves a change survived.

Because CI rewrites the `.bin`s on `main` on nearly every push, the remote moves
between commit and push routinely; `remote_moved` is the normal outcome, not an
error. Push with `git_push {recover_remote_moved: true}`.

`../aloopprime` is the opposite: a fetch-only checkout (`origin.pushurl =
no-push`, no local git identity, branch months behind), so `git_finalize` there
commits and the push is refused. Local-only commits are the intended end state —
do not "fix" it by editing its `.git/config` or adding a remote.

## ESP-IDF 6.x API breaks this tree has already absorbed

Historical — none of it is visible in the current source, and each item silently
re-breaks on an IDF bump.

- `esp_netif_next()` -> `esp_netif_next_unsafe()` in 6.1
  (`components/link-esp/link/.../esp32/ScanIpIfAddrs.hpp`).
- Legacy `driver/adc.h` was removed in 6.1; `main/io_helpers.cpp` carries a
  `hall_sensor_read()` stub where that include used to be.
- (The 512-byte `filepath` in `network_midi.cpp` is not an IDF break — see the
  HTTP clip server.)

## The SoftAP multicast gap is this project's, and may not be aloopprime's

The ESP32 SoftAP does not carry Link's multicast between host and stations, which
is why `link_multicast_relay_task` exists: it re-emits each Link datagram to the
group, to the AP's own IP, and unicast to every associated station, preserving the
original source IP (Link needs the true source for direct peer connect). Whether
aloopprime's `brcmfmac` AP mode has the same gap is UNVERIFIED — `ap_isolate=0` may
suffice. Do not assume it needs this relay ported; see
`../aloopprime/docs/LINK-MESH-TESTING.md`.

## Comment sweep status (INVARIANT 3)

Sweep in progress (`e0d59ed`, `c9393bb`, plus per-file sweeps over `main/`).
Counted at `main/` HEAD `028267f`: **42 files, 7424 lines, 4 comment lines — one
block at `main/wifi_config.cpp:121-124`; `main/` is NOT comment-free.** Every
comment must become self-explanatory code; the rest is filed as PRD rows by
`prd-add`, not here. Re-count before quoting these numbers.
