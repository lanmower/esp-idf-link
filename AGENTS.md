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

## Metronome and MIDI clock are hardware-scheduled, not tick-emitted

The click and the 24 ppqn `0xF8` come from a **hardware-scheduled
`esp_timer` one-shot armed to `SessionState::timeAtBeat()` of the NEXT
pulse** (`link_event_cb`), never from the 4 kHz gptimer tick task. A tick
only discovers a beat AFTER it passed, so tick emission gives every click
the tick's own wake-up latency plus WiFi/lwIP/logging/input-polling
contention -- milliseconds of audible jitter. Only the alarm's dispatch
latency (tens of microseconds) is left, and it is the same for the click
and the clock.

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
- `node flash-ticker.js [COMx]` (repo root) does the whole thing with esptool
  5.x (`python -m esptool`; note v5's hyphenated `write-flash` subcommand).
  Offsets: 0x1000 bootloader, 0x8000 partition table, and the app at the first
  app partition of the built table (0x20000 with `partitions_large.csv`; the
  script reads it, never hardcode 0x10000). `--list` lists ports, and it
  refuses to guess when more than one serial port exists -- this machine has
  two Bluetooth COM ports that are not the ESP32.
- A COM port only exists while the ESP32 is plugged in; if esptool cannot
  connect, hold IO0/BOOT while it starts. On this Windows host with the CH340
  boards, esptool's reset sequences and every manual DTR/RTS wiring and timing
  variant tried left the chip in normal boot (`boot:0x13`), so the BOOT hold is
  needed.

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

## Input and buzzer wiring facts

- Touch pads use the **legacy `driver/touch_pad.h` API** (ESP-IDF 6.2 also
  ships the modern `touch_sensor` driver; this tree is on the legacy one). A
  pad reads LOW when touched. The ARP pad is read before the others so its
  press latency stays lowest.
- Pots are raw ADC 0-4095 mapped to MIDI 0-127; a slow EMA tracks the
  "stable center" alongside the fast one used for control.

### The SoftAP multicast gap is this project's, and may not be aloop's

The ESP32 SoftAP does not carry Link's multicast between host and stations,
which is why `link_multicast_relay_task` exists: it re-emits each Link
datagram to the group, to the AP's own IP, and unicast to every associated
station, preserving the original source IP (Link needs the true source for
direct peer connect). The equivalent gap on aloop's Broadcom `brcmfmac` AP
mode is UNVERIFIED — `ap_isolate=0` may be sufficient there. Do not assume
aloop needs this relay ported; see `../aloop/docs/LINK-MESH-TESTING.md`.
