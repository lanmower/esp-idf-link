# esp-idf-link — agent notes

`CLAUDE.md` includes this file. Durable cross-session facts belong here, not
inline in the code.

## esp-idf-link <-> aloop mesh: paired invariants (change BOTH or the mesh splits)

This project (the ESP32 "ticker" box) and `../aloop` (the Pi 4 hardware
looper) form ONE ad-hoc single-AP mesh so Ableton Link's multicast peer
discovery reaches every device. There is no credential provisioning:
exactly one device hosts the open SSID `ticker` and everyone else joins it
as a station — whichever device boots first wins, Pi or ESP32. Every value
below exists in BOTH trees; changing it in one project alone silently stops
the two from meshing, with no error on either side.

| Invariant | esp-idf-link | aloop |
|---|---|---|
| Mesh SSID | `main.cpp` `wifi_scan_best_bssid("ticker")` / `wifi_start_link_ap("ticker")` / `wifi_start_supervisor("ticker")` | `src/net/config/hostapd.conf` `ssid=ticker`, `wpa_supplicant.conf` `ssid="ticker"` |
| Auth | `wifi_connect_sta("ticker", "")` (open, empty password) | open (`key_mgmt=NONE`; `wpa=` lines commented out) |
| AP address / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link itself — cannot drift) |
| Link quantum | `main.h` `#define LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp` `g_link->enableStartStopSync(false)` | `link_bridge.cpp` `enableStartStopSync(false)` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` is NOT the Link quantum — it is this project's own
transport-correction/SPP boundary (16 bars). It intentionally differs from
`LINK_QUANTUM 16.0` and does not affect phase agreement with aloop.

### Host election must stay MAC-ordered on both sides

Two devices cold-booting together can each scan before the other's AP
exists, so a naive "nothing found -> host" makes BOTH host, producing two
isolated L2 domains Link can never cross. This project holds for a
duration strictly monotonic in its own STA MAC (`HOLD_MAX_MS = 6000`,
lowest MAC ≈ 0ms), rescanning every second and joining the instant a peer's
AP appears. `../aloop/src/net/autoap.sh` now implements the identical
scheme (same 0–6s bound, same lowest-wins convention) so a Pi and an ESP32
elect one host between them. Both supervisors also yield if another
`ticker` AP with a strictly-lower BSSID appears — but never while clients
are attached, since that would drop peers mid-session. If this convention
is ever inverted here (highest-wins), it MUST be inverted in aloop in the
same change.

### By-the-book Ableton Link integration checklist (both projects)

Derived from a full audit of both trees against Link's own header docs.

- **Thread-correct session-state API.** `captureAppSessionState()` /
  `commitAppSessionState()` off the audio thread; the `AudioSessionState`
  variants only on it. This project uses the App variants throughout.
- **Start/stop sync is OFF on both sides: the mesh is clock-only.** Tempo
  and phase are shared, transport is not. This project still CONSUMES
  transport (`state.isPlaying()` in `link_sync.cpp` -> MIDI
  Start/Stop/Continue + all-notes-off) and correctly does NOT call
  `setIsPlaying` — it has no local play/stop control. With sync disabled
  `isPlaying()` reads true for the whole session, so downstream gear gets one
  Start and never a Stop; that matches `../aloop`, whose transport never
  stops either (a paused looper is muted, not stopped). Enabling it on one
  side only splits the mesh, so it MUST be flipped on both in the same
  change; the ESP32 side additionally needs a firmware flash to take effect.
- **The three notification callbacks** — `setNumPeersCallback(std::size_t)`,
  `setTempoCallback(double)`, `setStartStopCallback(bool)`. Link's header
  documents each as invoked on a Link-managed thread and **Realtime-safe:
  no** — bounded logging / atomics only, never allocation or locks.
- **Tempo authority.** `setTempo` rewrites tempo for EVERY peer. This project
  only sets tempo on an explicit LTMP command (`s_tempoReq`), which is the
  correct restraint; `../aloop` additionally refuses to propose when peers
  already own the tempo. Do not make either side an unconditional writer.
- **Quantum is a shared constant.** `LINK_QUANTUM 16.0` here,
  `kLinkQuantum` in `../aloop/src/link/link_bridge.h`. They move together.
  `PHRASE_BEATS 64.0` is this project's own SPP/transport boundary and is
  deliberately different — do not "align" it to the quantum.
- **Interface readiness is a real race, and this project already models the
  fix.** The 500ms settle before constructing Link (`main.cpp`) and the ~10s
  IGMP re-assert (`wifi_config.cpp`, whose comment notes a single join at
  GOT_IP can race netif readiness so membership does not stick) are the
  reference `../aloop` was corrected against — it had no service ordering
  between its `aloop` and `autoap` units at all.

### The vendored Link is a FORK, checked in — not a submodule

`components/link-esp/link` is a **patched copy of Ableton Link committed
directly into this repo** (1506 tracked files, git mode `100755`, no nested
`.git`). It is NOT an active submodule, despite `.gitmodules` having once
declared `components/link-esp` as one pointing at
`mathiasbredholt/link-esp` — that stale entry has been removed, because it
invited a `git submodule update` that would have clobbered the patch below
with no compile error and silently killed ESP peer discovery.

The divergence from upstream is the multicast relay hook:
`link/include/ableton/platforms/asio/Socket.hpp` declares
`extern "C" wifi_link_multicast_forward(...)` (:30) and calls it on every
send (:67), so `wifi_config.cpp`'s relay can unicast-copy each Link
discovery datagram across the SoftAP host/station boundary that does not
carry multicast. Committed here in `88d6866` / `622f7ca`.

**Any future Link version bump must re-apply that hook.** Losing it
compiles perfectly and fails only at runtime, as silent non-discovery.

There is no `.gitmodules` at the repo root, and the root one is the only one git
reads. `Dockerfile`, `setup.sh` and `.github/workflows/build.yml` therefore
deliberately do NOT initialise submodules: no-ops today, they are exactly the
mechanism that would overwrite the fork above the moment a root `.gitmodules`
comes back. Do not re-add them. Two nested `.gitmodules` files survive, inert
only because git reads none but the root: `components/link-esp/link/.gitmodules`
is upstream's own, kept so the fork's divergence stays limited to the hook
above; `components/link-esp/.gitmodules` was this repo's stale
`[submodule "link"]` entry and is deleted -- leaving it invites the
`git submodule update` that clobbers the fork.

## Metronome and MIDI clock are hardware-scheduled, not tick-emitted

The click and the 24 ppqn `0xF8` come from a **hardware-scheduled
`esp_timer` one-shot armed to `SessionState::timeAtBeat()` of the NEXT
pulse** (`link_event_cb`), never from the 4 kHz gptimer tick task. A tick
only discovers a beat AFTER it passed, so tick emission gives every click
the tick's own wake-up latency plus WiFi/lwIP/logging/input-polling
contention -- milliseconds of audible jitter. Only the alarm's dispatch
latency (tens of microseconds) is left, and it is the same for the click
and the clock.

- The tick the scheduler replaced is `LINK_TICK_PERIOD 250` (250 us = 4 kHz).
  Raising its rate does not tighten the click, because a tick can only ever
  notice a beat AFTER it passed; only the esp_timer one-shot sets the edge.
- **Link's ESP clock IS `esp_timer_get_time()`**
  (`components/link-esp/link/include/ableton/platforms/esp32/Clock.hpp`), so
  a Link-clock microsecond timestamp is already an esp_timer deadline: no
  offset, no drift correction between the two.
- **Lateness is instrumented.** `MetroStats` (`fired`/`last`/`worst`/`mean`/
  `rms`) rides in the UDP status reply on port 20812 as a `metro` object, so
  click stability is measurable from a peer --
  `../mc-420/test/hardware/link-mesh-status.js --esp <ip>` -- with no scope
  on the buzzer.
- **The next click's PWM pitch is primed while the buzzer is silent**
  (`prime_buzzer_freq` from the buzzer-off callback), so the ON edge is two
  duty writes instead of a `ledc_timer_config()` reconfigure landing on the
  beat.
- **No-burst invariant, new mechanism.** There is no per-tick catch-up cap
  any more (`MIDI_MAX_CLOCKS_PER_TICK` is gone); the scheduler steps the
  pulse counter forward past every pulse whose due time is already behind,
  so a stall or a peer-imposed phase jump DROPS pulses instead of bursting
  them. A burst of `0xF8` reads as a tempo spike to any downstream device
  PLL.
- `MIDI_CLOCK_RESYNC_THRESHOLD` is now **log-only**: "we fell behind,
  resyncing", one warning, no burst. Its name predates that.
- A watchdog re-arms the alarm from the tick when a pulse is really overdue
  (threshold far longer than the fire/re-arm window), because an alarm that
  never got re-armed would stop the clock silently.

Build trap: **`-Werror=volatile` is on**, so `++` on a `volatile` is a hard
error -- the fired-alarm counter is deliberately non-volatile.

Build trap: **`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults` is
load-bearing, not a tuning knob.** The default 10 is exhausted by the app's own
UDP sockets (Link relay, LCLK/TTMP broadcast, tempo listener, discovery
forward), leaving too few for Ableton Link's per-interface discovery gateway,
which then fails to open with EMFILE ("Too many open files") and never
broadcasts -> peers never discover each other. 16 is the ESP32 maximum.

## Firmware reaches the ticker over USB only

No OTA path and no serial console on this machine: a firmware change gets to
the device only over USB.

- CI (espressif/idf docker, ESP-IDF 6.2) builds on push to any branch, and on
  a green `main` it commits the three images into the repo --
  `build/bootloader/bootloader.bin`,
  `build/partition_table/partition-table.bin`,
  `build/link-idf-example.bin` (commit "ci: update firmware binaries
  [skip ci]"). The flashable images therefore come from git, not from a local
  build.
- `node flash-ticker.js [COMx]` (repo root) does the whole thing over a COM
  port with esptool 5.x (`python -m esptool`; note v5's hyphenated
  `write-flash` subcommand -- though the underscored `write_flash` alias still
  works on 5.5.0, which `flash_midi_data.sh` relies on). Offsets: 0x1000 bootloader, 0x8000 partition
  table, and the app at the first app partition of the BUILT table (0x20000
  with `partitions_large.csv`; the script reads
  `build/partition_table/partition-table.bin`, never hardcode 0x10000).
  **Read the offset from the built binary, not the CSV** -- `partitions_large.csv`
  leaves every offset but nvs's empty because the build computes them, so
  parsing it yields `int('')`. Entries are 32 bytes, little-endian magic
  `0x50AA`, then type(1) subtype(1) offset(4) size(4) label(16); app entries
  have type == 0. `--list` lists ports, and it refuses to guess when more than
  one serial port exists -- this machine has two Bluetooth COM ports that are
  not the ESP32. esptool 5.x dropped `--verify` because it always verifies
  ("Hash of data verified" per image), so a `--verify` flag is not ignored --
  it aborts `write-flash` with "No such option" and flashes nothing.

- The SPIFFS `storage` partition (the MIDI files) is flashed separately from
  the firmware: `flash_midi_data.sh` builds a `0x19000` (100K) image from
  `./data` and writes it at **`0x317000`**. That offset is NOT re-derivable
  from `partitions_large.csv` -- the `storage` row leaves its offset blank
  (only `nvs` has one) because the build computes it. `0x317000` appears
  nowhere else in the repo but that script's own argument, so if the layout
  changes, re-read it from the built partition-table binary.

### No BOOT hold: `python tools/flash-usbip.py` flashes it hands-free

`flash-ticker.js` needs the BOOT/IO0 button held, but that is a limitation of
the WCH/CH340 Windows driver, not of the board. `tools/flash-usbip.py` talks to
the CH341 over USB/IP (usbipd-win, `usbipd bind --busid 2-2` once) with its own
pyserial-shaped shim, so the WCH driver is never loaded and Windows cannot
re-assert a line. Nothing has to be touched on the device:

    python tools/flash-usbip.py --dry      # enter download mode, prove sync, reboot
    python tools/flash-usbip.py            # flash the committed images and boot them

Two measured facts make it work, and both are counter-intuitive:

1. **The D1 R32's auto-reset circuit is DIFFERENTIAL.** Each transistor is
   driven by the DTR#-RTS# difference, so EN is pulled low only when
   (DTR# high, RTS# low) -> `0xA4` byte `0x40`, and IO0 only when
   (DTR# low, RTS# high) -> `0x20`. Asserting both (`0x60`) or clearing both
   (`0x00`) makes NEITHER transistor conduct. That is the opposite of the
   convention esptool hard-codes, which is why every esptool reset sequence
   produced `boot:0x13`. The entry is the flip **0x40 -> 0x20**: EN is released
   (its RC rises) while IO0 is already low. Verified `boot:0x3`. Going back is
   the reverse: `0x40` (EN low, IO0 high) then `0x00`.
2. **The CH341 withholds the first replies.** A single sync frame gets no
   answer for seconds, then several past replies surface at once behind a
   later FULL-SIZE OUT -- a 1-byte `0xC0` kick does not release them. After
   the first reply lands, latency is ~20 ms and stays there. esptool sends
   exactly one sync frame with a 0.1 s deadline, so the pipe must be primed
   with repeated correctly-framed sync frames first (and `SYNC_TIMEOUT` raised
   to 2 s); typically 2-8 frames.
3. **The first attach back often fails.** usbipd keeps the device claimed for
   a moment after a client disconnects, so the first `OP_REP_IMPORT` comes back
   `status=4`. `open_port()` retries (10 x 6 s) and the second lands; the
   `attach 1/10: OP_REP_IMPORT status=4` line at the start of a run is normal,
   not a failure.

Three consecutive `--dry` cycles each produced `boot:0x3`, sync, then
`boot:0x13` with the app's log, so entry is not a one-off.

### Flash baud: 460800 is the default, 921600 does not work

The ROM only ever speaks 115200, so `enter_and_sync` pins that (`ROM_BAUD`)
and `--baud` takes effect only after esptool uploads its RAM stub and re-times
the port -- passing `--baud` into the sync is what made the first 921600
attempt fail. Measured on the 1262560-byte app image: **115200 -> 72.2 s**
(139.9 kbit/s) versus **460800 -> 19.4 s** (519.9 kbit/s, `Hash of data
verified`, boots `boot:0x13`). Two consecutive runs both measured 19.4 s, so
460800 is repeatable, not a lucky run. Both numbers are the USB/IP path only;
`flash-ticker.js` drives a real COM port through the WCH driver at `--baud
921600`, a different transport that was not the one measured here. 921600 dies right after `Changing baud rate to 921600...
Changed.` with `FatalError: No more data to read from the serial port` from
`reset_chip -> soft_reset -> flash_begin`: no `Writing` line at all, nothing
written, chip left in the stub. `--dry` recovers it and the old firmware is
intact.

esptool leaves the port at the flash baud, so `boot_app()` resets it to
`ROM_BAUD` before pulsing EN -- without that the app's 115200 log is read at
460800 and the run silently shows no `boot:` line at all.

`tools/ch341.py` (raw USB/IP + the `ch341.c` init order: `SET_CONFIGURATION`
before anything, or bulk IN is dead), `tools/usbip-port.py` (bulk OUT + the
pyserial surface), and `tools/flash-usbip.py` (entry, priming, esptool) are the
three layers.

Known oddity, benign: `flash_id()` reads `0xFFFFFF` (SPI RDID returns all
ones) in download mode on this board, so esptool prints "Failed to communicate
with the flash chip" and auto-detects no size. It is a warning, not a failure
-- `detect_flash_size` returns None and `_set_flash_parameters` just skips
`flash_set_parameters`. Erase/program go through the ROM's own flash_begin/
flash_data and are unaffected.

## Mesh UDP protocol and MIDI emission

Ports, all little-endian `int64` payloads on both ends:

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group, ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets group tempo and phase |
| 20812 | esp -> group, ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |

- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF`
  per interface.** We may be AP or STA (boot race) and a raw socket with no
  `IP_MULTICAST_IF` exits the wrong interface on a dual-netif device -- Link's
  own socket reached the peer (it sets egress) while ours did not (Pi `clkRx`
  stayed 0).
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers
  multicast/broadcast but NOT unicast-to-self, so Link's ping/pong
  measurement never completes there (Pi `:4445` WLAN `uniRx`/`RALV` stay
  empty, peers=0). LCLK gives it phase; TTMP carries tempo back the other
  way, since Link's native discovery never reaches it. Standard Link apps
  ignore all three ports.
- **The clock broadcast is rate-limited to one packet per 20 ms**: a 4 kHz
  flood saturates the Pi's single radio-RX drain and starves its control
  plane. 20 ms is far under one beat, so phase accuracy is unaffected.
- **Tempo is bounded to Link's real range, 20..999 bpm** (Ableton's Test Plan
  TEMPO-4 exercises the full range and names both ends). A 400 ceiling
  silently dropped legitimate session tempos instead of following them.
- `beatAtTime` is quantum-insensitive while `phaseAtTime` is not, so the
  timeline broadcast uses `LINK_QUANTUM` for the beat value.
- LTMP/phase requests are applied on the Link task, not on the listener
  task: `captureAppSessionState`/`commitAppSessionState` need a single owner
  or the two tasks race.
- Status is deliberately request/response, not a broadcast: it costs nothing
  when idle and cannot pollute the Link multicast group.

MIDI emission -- one path, no per-device clock code:

- Continuous 24 ppqn `0xF8`; at the 16-bar phrase boundary only, SPP (`0xF2`)
  + Start/Continue (`0xFA`/`0xFB`); Stop (`0xFC`) + CC123 on all 16 channels
  on transport stop and on peer loss. SPP counts MIDI beats (sixteenths): one
  Link beat = 4 units, 14-bit, LSB first.
- Realtime bytes are single-byte and may legally interleave a running-status
  message, so the buzzer path and the clock path never corrupt each other.
- Note-offs go out as velocity 0; CC123 on stop is what keeps RC-505 MK2
  loops from sticking.
- Target behaviour that constrains the design: KO2 locks to ext clock, needs
  Start, and is SPP-sensitive (hence one SPP per phrase, not per 4 beats);
  Volca Drum follows the pulse and ignores SPP; MicroKorg/MiniNova arp, delay
  and LFO follow the pulse and need it non-bursting; Micron needs Start and
  stays in phrase when Start is phrase-aligned; RC-505 MK2 locks to
  clock+Start and repositions on SPP.
- Start is deferred to the next phrase boundary so gear begins in phrase
  rather than jerking mid-phrase. `s_transport_running` records what
  downstream gear was last TOLD (Start vs Continue) -- it is not
  `isPlaying()`.
- Force-start waits 8 s, not 5 s, so two co-booting devices can discover each
  other before either free-runs at an independent phase.

## MIDI file player (`main/midi_file.cpp`, files on SPIFFS)

- Parser relies on **MIDI running status**: a data byte with no status byte in
  front repeats the previous status. Note-on with **velocity 0 IS a note-off**;
  a note still held at end-of-track is released implicitly.
- `process()` walks notes in `startBeat` order and early-breaks, so **the sort
  order is load-bearing**, not cosmetic.
- Trigger window is 0.03 beat, with `playedNotes`/`sentCCs` dedup so a note or
  CC fires once per pass.
- Loop length rounds to the nearest quantum when within 0.1 of one, else
  rounds up (ceil).
- **SPIFFS has no real directories**, so `setFolder()` cannot `opendir` a
  subfolder: it falls back to scanning `/spiffs` and matching a name prefix.
- `updateTempo`'s 120 bpm reference and 0.5 bpm change threshold are currently
  dormant (`syncToBpm` is false). See CLAUDE.md for why that stays off.
- Known warts, deliberately left alone: `parseFile()` closes the file twice
  (explicit `fclose` plus a `FileGuard`); the built-in default MIDI file's
  `MTrk` declares 19 bytes but only 12 are written, and its header encodes
  format 1.

## Input and buzzer wiring facts

Touch pad / pot / MIDI wiring, from `main/main.h` (the GPIO numbers used to be
comments there and are now here, since a pin number cannot be expressed in
code that reads the macro):

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

- Touch pads use the **legacy `driver/touch_pad.h` API** (ESP-IDF 6.2 also
  ships the modern `touch_sensor` driver; this tree is on the legacy one). A
  pad reads LOW when touched. The ARP pad is read before the others so its
  press latency stays lowest.
- Pots are raw ADC 0-4095 mapped to MIDI 0-127; a slow EMA tracks the
  "stable center" alongside the fast one used for control.
- In the MIDI file player the two pots are **pot1 = note-length scale (0..2),
  pot2 = velocity scale (0..1)** (`effect_arp.cpp` `handle_arp_adjust_pots`,
  `midi_file.h` `noteLengthScalePotValue`/`velocityScalePotValue`). The old
  field comments said "speed" and "transpose" -- those were stale and are gone.
  The accessors are still `getPot1Value`/`getPot2Value`, called from
  `effect_arp.cpp`.
- Pad gesture timings are in `main/main.cpp`: `DOUBLE_TAP_TIME_MS = 300`,
  `HOLD_TIME_MS = 200` (both milliseconds).
- Synth target is one of `SynthType { SYNTH_MININOVA, SYNTH_MICROKORG }` in
  `g_synth_type` (`main/main.h`); the per-target MIDI behaviour is listed
  under MIDI emission above.

## Constants whose constraint is not visible in the code

Several constants survive with the right values but no longer carry the rule
that makes the value correct. All were comments; none can be expressed in code.

- `MAX_ARP_INDEX_WRAP = 128` (`main/arp_constants.h`) wraps the raw note index
  in `effect_handler`. It must stay **larger than the longest reasonably
  expected progression sequence** -- that bound, not 128, is the requirement.
  It is currently unreferenced, so nothing enforces or exercises it.
- **E-Slew default is 104**, and 104 is not an independent number: `effect_handler`
  computes `64 + (s_current_sidechain_sheer / 2)`, so it is the sheer default
  (80) halved and offset. `reset_sidechain_to_default()`'s `setGateESlew(104)`
  is the same value written as a bare literal. Changing one without the other
  silently changes the sidechain's default sound.
- `kScales[3]` in `main/bassline_interpreter.cpp` is `{0, 3, 5, 7, 10, 3, 5}`
  while `kScaleLens[3] == 5`. The trailing `{3, 5}` is deliberate padding, not
  a typo: the row is sized to match its neighbours and only its first five
  entries are ever read. Do not "fix" it to five entries.
- `kRegisterSpan = 15` (`main/bassline_interpreter.cpp`, semitones) must track
  the anchorMotif clamp in `bass_engine.cpp`. That coupling is recorded nowhere
  else and neither file references the other.

## Synth CC/NRPN facts that are not derivable from the code

- **MicroKorg**: LFO2 shape is CC 75 (`kCcLfo2Shape`); delay sync is CC 13
  (`kCcDelaySync`), where 0 = off and 1..32 are sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN is (MSB 1, LSB 40), and that pair is an
  **assumption never confirmed on hardware** -- the one NRPN in the tree with no
  verified source. Filter bypass exists as NRPN 0/60 value 0 and is deliberately
  NOT sent by `deactivateFilter()`, which unpatches only the LFO.
- **MiniNova "Table 3"** is the source of both `setDelaySyncRate` and
  `setLfoRateSync` values; `main/lfo_constants.h` carries only a **subset** of
  the LFO rate-sync row (NRPN 0/86). Do not treat that list as exhaustive.
- `SynthInterface` contracts that exist only in the implementations:
  `activateDelay()` selects Delay 1 and `activateReverb()` selects Reverb 1,
  both in **FX Slot 1**; `setFxSlot1Level()` is **CC 91**; `activateFilter()`
  implicitly selects the **LP24** type and applies its defaults; the LFO
  methods assume **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` maps to MIDI `0..127` with **64 = zero**,
  so `unpatchLfoFromFilter()` writes 64, not 0.
- Note-off velocity 0 is now the **default**, not merely a call-site convention:
  `sendNoteOff(uint8_t note, uint8_t velocity = 0)` is declared identically in
  `main/synth_interface.h`, `main/synth_microkorg.h` and `main/synth_mininova.h`.
  The two surviving `= 64` defaults in `synth_interface.h` (`activateFilter`'s
  cutoff, `patchLfoToFilter`'s depth) are legitimately 64 and must stay.

## `bassline_interpreter` must stay host-compilable

`main/bassline_interpreter.cpp` is deliberately kept free of ESP-IDF headers so
it compiles on the host with plain `g++ -std=c++17`. Do not add ESP-IDF includes
to it: that property is the only way to exercise the interpreter's DP and scale
logic off-device. On MinGW a host build of it additionally needs
`-D_USE_MATH_DEFINES`, or `M_PI` is not declared.

## Five `main/*.cpp` files are not in the build

`main/CMakeLists.txt` SRCS names 11 translation units. `effect_arp.cpp`,
`effect_filter.cpp`, `effect_handler.cpp`, `effect_sidechain.cpp` and
`midi_file.cpp` are NOT among them, and nothing in the tree `#include`s a
`.cpp`, so that cluster is never compiled or linked -- it is reachable only
from itself (`effect_handler.h` is included solely by those four effect
`.cpp`s; `midi_file.h` solely by `effect_arp.h` and `midi_file.cpp`). This is
long-standing, not a regression: HEAD's SRCS list carries the same 11 entries.
Consequence: the MIDI-file-player behaviour described in `CLAUDE.md` is not
running on the device, and edits to those five files cannot break the build.
Adding them to SRCS is a real behaviour change, not a cleanup.

## CI commits binaries with `reset --soft`, never a rebase

`.github/workflows/build.yml` commits the three `.bin` files and pushes them.
`idf.py build` leaves many other unstaged files under `build/`, so a plain
rebase refuses ("unstaged changes"), and a concurrent workflow advancing the
branch is the expected cause of a rejected push. The retry loop therefore
fetches, `git reset --soft FETCH_HEAD`, re-stages only the three `.bin`s and
re-commits a single commit, up to 5 attempts. A rebase would try to MERGE the
previous build's `.bin` files and die on "Cannot merge binary files".

## ESP-IDF 6.x API breaks this tree has already absorbed

Historical -- none of it is visible in the current source, and each item
silently re-breaks on an IDF bump:

- `esp_netif_next()` was renamed `esp_netif_next_unsafe()` in 6.1
  (`components/link-esp/link/include/ableton/platforms/esp32/ScanIpIfAddrs.hpp`).
- Legacy `driver/adc.h` was removed in 6.1; `main/io_helpers.cpp` carries a
  `hall_sensor_read()` stub where that include used to be.
- `main/main.cpp` wraps the provisioning_mode block in braces because a `goto`
  crossed a variable initialization boundary.
- `main/network_midi.cpp` uses a 512-byte filepath buffer, not 256: long
  filenames overflowed the `snprintf`.

## The SoftAP multicast gap is this project's, and may not be aloop's

The ESP32 SoftAP does not carry Link's multicast between host and stations,
which is why `link_multicast_relay_task` exists: it re-emits each Link
datagram to the group, to the AP's own IP, and unicast to every associated
station, preserving the original source IP (Link needs the true source for
direct peer connect). The equivalent gap on aloop's Broadcom `brcmfmac` AP
mode is UNVERIFIED — `ap_isolate=0` may be sufficient there. Do not assume
aloop needs this relay ported; see `../aloop/docs/LINK-MESH-TESTING.md`.
