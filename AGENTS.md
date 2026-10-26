# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this; durable facts live here, not in code — don't extend `CLAUDE.md`.
Compacted 2026-10-23: git log through `756b57e`; 2026-10-11 and 2026-10-13 live-state sections merged;
Windows-only flash notes marked as such (this host is Linux).

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `main.cpp` `wifi_*("ticker")` | `hostapd.conf`, `wpa_supplicant.conf` |
| Auth | `wifi_connect_sta("ticker","")` | open (`key_mgmt=NONE`; `wpa=` commented) |
| AP addr / DHCP | `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`kTickerChannel = 6`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `enableStartStopSync(false)` | same, `false` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |
| Tempo authority | sets only on explicit LTMP | refuses to propose when peers own tempo |

`PHRASE_BEATS 64.0` is NOT the quantum — SPP/transport boundary (16 bars). Pi spells `ticker` in
`config/aloop.conf:62`, `autoap.sh:7`. Audited 2026-10-11: all MATCH; deliberate asymmetries: IGMP one-sided
(pi none, `ap_isolate=0`), scan stagger one-sided, takeover esp ~60 s vs pi ~90-160 s. mc-420 runs a second
transport quantum of 128 for phase capture; phase is absolute-beat, so folding a 128-beat capture at `rb` is
wrong unless `rb | 128`.

**Boot decision (`main.cpp` app_main)** — no credential provisioning; ad-hoc single-AP mesh on open SSID
`ticker`. 1) scan staggered by last STA-MAC byte (0-3825 ms) so co-boots don't race; 2)
`wifi_scan_best_bssid("ticker",...)`: SSID-filtered active scan, `esp_wifi_scan_get_ap_records` -> lowest
BSSID; 3) match -> `wifi_connect_sta`, wait 30 s, else host; no match -> `wifi_start_link_ap` + relay.
`ensure_sta_started()` stops a default-STA netif leaking across rescans.

**Host election** — each device scans before the other's AP exists, so both host, and Link can't cross two
isolated L2 domains. Hence `HOLD_MAX_MS = 6000`, monotonic in own STA MAC (`mac_rank * HOLD_MAX_MS >> 24`),
rescan 1/s, stagger `mac[5] * 15` ms; `../aloopprime/src/net/autoap.sh` mirrors it (0-6 s, lowest-wins).
Invert both in one change.

Supervisor (`wifi_config.cpp`, 2 s cadence; `RECONNECT_TRIES=30`, IGMP re-assert 30 s, tx probe 10 s):
- STA: dropped link reconnects 30x (~60 s) before re-hosting — rides out a flapping host (a Pi reboot drops
  `ticker` past the old 12 s), else replaces it.
- **STA "up" = has IP** (`579d4ef323`): `g_wifi_connected` set only by `IP_EVENT_STA_GOT_IP`, cleared only by
  `WIFI_EVENT_STA_DISCONNECTED`, no `IP_EVENT_STA_LOST_IP` handler -> an association outliving its IP left the
  supervisor inert. Link drops a netif with address 0 and erases the gateway; 0 gateways = no send, no receive
  -> **peers 1->0 while associated, no Wi-Fi event**. Gate: `g_wifi_connected && sta_has_ip()`.
  **Proven 2026-10-23**: at a real 1->0 the station was associated with its IP and IGMP clean, so this class
  of drop is the Link SESSION, not the IP path. See Live state.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID **only with ZERO stations**
  (`ap_has_associated_stations()`) — one host wins. **Comparison MAC is the STA MAC**
  (`e4:65:b8:76:cf:c4`, 2nd unit; 1st `e4:65:b8:77:0d:14`; STA IP is DHCP from the host, observed `.10` on 2026-10-23 and `.19` on 2026-10-25, so never key on the IP), not the SoftAP MAC (`...:c5`);
  Pi `e4:5f:01:53:f0:f1` is lower, so this box yields. **Self-BSSID guard reads the interface**
  (`esp_wifi_get_mac(WIFI_IF_AP)`, never `esp_read_mac()` = eFuse) or it flaps AP->STA->AP every tick
  (`5faf01a`).
- IGMP re-joined on yield, not only at boot (`d1b7f9a`), in **AP role** too — a long-lived SoftAP can't age
  its join out. Lease proof: `WIFI: IP: 192.168.4.x`.
- **Hosting scan pinned to mesh channel**: needs `ensure_sta_started()` -> `WIFI_MODE_APSTA`; an all-channel
  scan dwells off-channel and interrupts SoftAP beaconing, so a station associating mid-scan never finishes
  DHCP. Supervisor passes `kTickerChannel`; boot/hold scans stay `kScanAllChannels`. Cost: a host off ch6 is
  never seen. **AP->STA yields run only after hosting once**, so a STA-only soak exercises none.

**Reading `peers=0`:** `gw(ok=1)` = gateway up; `dst=` on a high port (e.g. `192.168.4.1:54146`) means the
announcement arrived and Link is unicast-pinging — failure is ping/pong, NOT multicast. **`dst` is HOST
order** (asio `to_v4().to_uint()` vs network-order `ip4_addr_t::addr`) -> `htonl` before `IP2STR`;
pre-`579d4ef` images printed it reversed — same trap as `99.168.4.1`.

**How Link drops a peer** (read-only tree, do NOT patch): `UdpMessenger.hpp:74-77` swallows send errors and
`platforms/esp32/Context.hpp` discards the exception handler, so `asio/Context.hpp`'s catch loop isn't
compiled and **`repairGateway` is dead**; the equivalent (gateway erase + 5 s rescan) runs via `PeerGateways`
`Callback::operator()`. Routes: TTL ~6 s; ByeBye; **`gatewayClosed`** (`~GatewayObserver`,
`Peers.hpp:136-144`) erases ALL peers on that address — the 1->0-with-no-Wi-Fi-event route, also fired by
`enable(false)`'s `clear()`; `resetPeers()`.

**By-the-book Link checklist (both trees)**
- `captureAppSessionState()`/`commitAppSessionState()` off the audio thread; `AudioSessionState` variants only
  on it. This tree uses the App variants.
- **Start/stop sync OFF on purpose: mesh is clock-only** (`b94ac2d`); `isPlaying()` then reads **false** all
  session (transport is ESP-local). Re-enabling needs BOTH trees + a flash here.
- The three notification callbacks run on a Link thread, **Realtime-safe: no** — bounded logging / atomics
  only. `setTempo` rewrites tempo for EVERY peer; this tree sets it only on explicit LTMP.
- **Interface readiness is a race:** 500 ms settle before Link construction; IGMP re-asserted every 30 s,
  never one-shot.
- **The Pi advertises one NodeState across every IFF_RUNNING interface**, so its `mep4` flips `192.168.4.1`
  <-> ICS address; `Peers::sawPeerOnGateway` cached whichever arrived last. Off-link `mep4` is **counted and
  logged, never dropped** (`cfddf6d` reverted `4481a5a`).

**The vendored Link is a checked-in FORK, not a submodule.** `components/link-esp/link` is a patched copy
committed directly (1506 files, no nested `.git`); `git submodule update` would clobber the patch with no
compile error and silently kill ESP peer discovery — hence the stale `.gitmodules` entry was removed;
`Dockerfile`, `setup.sh`, `build.yml` don't init submodules. Patches live in `Socket.hpp`, `PeerGateways.hpp`,
`UdpMessenger.hpp`, `Controller.hpp`, `Peers.hpp` — re-apply on any bump. Two API facts, each costing a CI
failure (both compile on host, break on-device): vendored standalone asio takes **flags AFTER the endpoint**
(`send_to(buffer, to, 0, ec)`), opposite of Boost's `(buffer, flags, to, ec)`; `PeerState::endpoint` is a
**data member**, while `MeasurementService`/`PingResponder`/`Socket::endpoint()` are methods. **Compiled asio
is `components/espressif__asio` 1.32.0, not vendored `link/modules/asio-standalone` 1.17.0** — so
`asio/error.hpp` maps only `no_buffer_space = ENOBUFS`, yet the failure prints `send_to: Not enough space`
(`strerror_r` for **errno 12 / ENOMEM**). Tempo rides the wire as int64 microseconds-per-beat, **truncated**
(`Tempo.hpp:52`): 119.049 round-trips with ~1e-5 bpm error. **The relay hook is the divergence**:
`platforms/asio/Socket.hpp` declares `extern "C" wifi_link_multicast_forward(...)` and calls it on every send,
so `wifi_config.cpp` can unicast-copy Link discovery datagrams across the SoftAP boundary; **any Link bump
must re-apply it** — losing it fails only as silent non-discovery. Gated `if (!g_ap_active) return;`.
`Socket::send` retries a failed `send_to` 3x, exponential 1..8-tick backoff reset on success (`cfddf6d`,
`0a8760c`), recording ok/retry/attempt/giveup + errno instead of throwing.

## SoftAP multicast gap, and lwIP core context

ESP32 SoftAP won't carry Link multicast host<->stations, hence `link_multicast_relay_task`: re-emits each
datagram to the group, to the AP's own IP, and unicast to each station, preserving source IP (Link needs it to
connect directly). aloopprime `brcmfmac` sharing the gap UNVERIFIED (`ap_isolate=0` may suffice) — don't port
the relay by assumption. **The relay is a singleton, `g_ap_netif` mutex-guarded** (`0bffe4c`):
`link_relay_release()` frees socket + raw pcb on every path.

**`LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()` are EMPTY MACROS in this build** (`2c8807a`):
`CONFIG_LWIP_TCPIP_CORE_LOCKING` unset, Kconfig default `n`, so `LWIP_TCPIP_CORE_LOCKING == 0` — every site
using them ran lwIP core work from a foreign task with **no exclusion at all**. Real serialization is
`esp_netif_tcpip_exec(esp_netif_callback_fn fn, void* ctx)` (callback runs on the tcpip thread; exec blocks
till it returns). All IGMP joins and relay raw-pcb work (`raw_new`, `raw_remove`, `raw_sendto_if_src`) go
through it. **Never hold `ap_netif_lock` across an `esp_netif_tcpip_exec`** — the relay takes the ap lock then
blocks on the tcpip thread, so the supervisor snapshots `g_ap_netif` under the lock and joins outside it.

**The AP->STA yield is where role-switch leaks lived** (`dc9add9`): the relay was never stopped there
(`g_relay_stop` was only ever set inside `wifi_start_link_relay`), `g_relay_task` was non-volatile under a
spin, and the forward socket was a function-local `static` — one of only 16 — now closed on yield under a
mutex the send hook also takes, so `close()` can't land mid-`sendto`. `wifi_stop_link_relay()` bounds its wait
(100 x 20 ms). `wifi_join_link_multicast()` returns unless the STA already has an IP — the supervisor
re-asserts within 2 s of `sta_up`, so nothing reports from 0.0.0.0.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to `SessionState::timeAtBeat()` of the NEXT
pulse, never the 4 kHz gptimer tick (`LINK_TICK_PERIOD 250` us), which discovers a beat only after it passed;
raising the tick rate can't tighten the click. Link's ESP clock IS `esp_timer_get_time()`, so a Link timestamp
is already an esp_timer deadline.

- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`); a burst reads as a tempo spike downstream.
  `MIDI_CLOCK_RESYNC_THRESHOLD` is log-only — changes nothing.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`; past
  `kMaxAdvanceStepsPerProcess` it releases note-offs, re-anchors `m_lastPhrasePosSteps`, emits nothing
  (`5598ed6`). **A tempo change doesn't trip it**; only a phase jump does.
- **`volatile` bites twice**: `++` on a volatile counter is a hard compile error (`23e5170`, so `s_fire_count`
  is plain `uint32_t`); `xTaskCreate`'s last parameter is non-volatile `TaskHandle_t*` (`f0fc252`) — create
  into a local, then assign.

**`sdkconfig.defaults` is load-bearing, not tuning.** `CONFIG_LWIP_MAX_SOCKETS=16` (16 is the max): default
10 is exhausted by the app's own sockets (relay, LCLK/TTMP, tempo listener, discovery forward, 4 for HTTP),
leaving too few for Link's per-interface discovery gateway -> EMFILE -> peers never discover.
`LWIP_TCPIP_RECVMBOX_SIZE=64` (32->64) — ENOMEM from `sendto` means EITHER pool drained OR mailbox full;
`LWIP_UDP_RECVMBOX_SIZE=6`, `LWIP_STATS=y` (read `mem.err`/`link.drop`/`link.memerr`) — all `cfddf6d`.
`ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=128` (doubled 64->128 by `f09e91c` to probe the ENOMEM send failures).
**`CONFIG_LWIP_PBUF_POOL_SIZE` is NOT a v6.1 symbol**, removed; don't re-add.

## Firmware reaches the ticker over USB only

- CI builds on push to any branch. **The IDF image is digest-pinned** in `Dockerfile:1`
  (`ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b...fea95`); `resolve-idf-image` exits 1 unless the digest is
  exactly 64 lowercase hex digits — a toolchain bump is that one line. On green `main` it commits the three
  images under `build/`; flashable images come from git. Runners pinned `ubuntu-24.04` (`3a4413b`).
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition of the **BUILT** table
  (0x20000 with `partitions_large.csv`; read from the binary: 32-byte entries, LE magic `0x50AA`, then
  type(1) subtype(1) offset(4) size(4) label(16), app type == 0). **The app offset is DERIVED, never
  hardcoded** (`31a6b99`). **Never parse `partitions_large.csv`** — every offset but nvs's is blank.
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always verifies). SPIFFS `storage`
  flashes separately: `flash_midi_data.sh` builds a `0x19000` image from `./data` at **`0x317000`**
  (nvs 0x011000, phy_init 0x017000, app0 0x020000, app1 0x1A0000, storage 0x317000).
- **Flash from Linux (this host, measured 2026-10-23)**: `sudo pip install --break-system-packages esptool`
  (no pip module preinstalled; python 3.14 is PEP 668 managed and `ensurepip` yields no pip), then
  `sudo python3 -m esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default-reset --after
  hard-reset write-flash --flash-mode dio 0x1000 build/bootloader/bootloader.bin 0x8000
  build/partition_table/partition-table.bin 0x20000 build/link-idf-example.bin` — 1312688 B in 20.0 s, hash
  verified. **`default-reset` works here**: it connected, ran the stub, read MAC `e4:65:b8:76:cf:c4`.
  **Check the MAC on every flash.** On 2026-10-25 the board on `/dev/ttyUSB0` read `e4:65:b8:77:0d:14`
  (the 1st unit), not `e4:65:b8:76:cf:c4` (the 2nd, which every certification was run on). Which physical
  board is attached can change between sessions; the esptool `MAC:` line is the only proof.

### Flashing from the Windows box

WSL2 + CH340 over USB/IP: full procedure moved to `docs/flash-windows.md` (the D1 R32 differential
auto-reset, the CH341 handshake register, usbipd attach retries, `--after no-reset-stub`). Only that
path needs it; on Linux `default-reset` just works.

## Mesh UDP protocol and MIDI emission

Payloads little-endian `int64`; ports are `#define`s in `link_sync.cpp`: `LINK_PHASE_PORT 20810`,
`LINK_TEMPO_PORT 20811`, `LINK_TTMP_PORT` 20812, `LINK_STATUS_PORT` **20813** (moved from 20812 in `a100df8`,
untested on hardware until a device run polls it), `TX_PROBE_PORT 34999`. The authoritative table is `readme.md` `## Network ports`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets tempo and phase |
| 20811 | request | `LRST` | `enable(false); enable(true); enableStartStopSync(false)` — no Wi-Fi teardown |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20813 | req/resp | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro (was 20812 until `a100df8`) |
| 34999 | esp -> gw / group | diagnostic | `log_tx_probe` every 10 s + `log_send_failure_probe`; nothing replies |

- **aloop binds only `224.76.78.75:20808`** (Pi AF_PACKET capture): 20810/20812 dropped at its IP layer —
  `Udp: IgnoredMulti` ~57/s while `NoPorts` is frozen. **20808 stays pure Link**, so a peer wanting our phase
  must itself bind 20810/20812.
- **Every multicast send must iterate netifs and set `IP_MULTICAST_IF` per interface**
  (`esp_netif_next_unsafe`): else a raw socket exits the wrong interface on a dual-netif device.
- **Measured multicast rates, subscribed, 2026-10-25** (Pi `wlan0` capture, 30 s, all from `192.168.4.19`):
  20810 LCLK 1414 frames (~47/s), 20812 TTMP 286 (~9.5/s), 34999 TX probe 3 (~0.1/s). `IgnoredMulti` rose 1707,
  against 1703 captured on unbound ports. An earlier 81/s reading was a window artifact, not a second sender.
  Idle: LCLK ~1/s, TTMP ~0.2/s. The ESP sends on one interface only.
- **LCLK/TTMP are subscriber-driven** (`0a8760c`): a datagram to **20813 or 20811** counts as a subscriber hit
  (since `a100df8`; it was 20812 before, where the ESP's own TTMP looped back and kept itself subscribed). A
  hit holds for `SUBSCRIBER_HOLD_US` (15 s). Without one, LCLK drops 50 Hz -> 1 Hz and TTMP 10 Hz -> 0.2 Hz; clock rate-limited to one
  packet per 20 ms (timeline 100 ms). **`peers` readouts are not passive** — polling the ESP raises its
  broadcast rate for 15 s. The status responder (now 20813) replies to ANY 1..64-byte payload and cannot perturb the
  session (it only reads `captureAppSessionState`); `LRST`/`LTMP` on 20811 are prefix+length checked, so a
  short `PING` can never be mistaken for either.
- Tempo bounded to **20..999 bpm** (`LINK_TEMPO_MAX_BPM 999.0`, `396092d`). `beatAtTime` is quantum-insensitive,
  `phaseAtTime` isn't — the timeline broadcast uses `LINK_QUANTUM`. LTMP/phase requests apply on the Link
  task, not the listener task. Status is request/response, not broadcast; `MetroStats` rides in it as `metro`.

MIDI emission — one path, no per-device clock code; bytes are constants in `main.h` (`0xF8` clock, `0xFA`
start, `0xFC` stop, `0xFB` continue, `0xF2` SPP, CC123 all-notes-off); `MIDI_PULSES_PER_QUARTER_NOTE = 24`,
`MIDI_SPP_UNITS_PER_BEAT = 4` in `link_sync.cpp`. SPP + Start/Continue ONLY at the 16-bar boundary; Stop +
CC123 on all 16 channels on transport stop and peer loss. SPP counts MIDI beats (sixteenths): 1 Link beat = 4
units, 14-bit LSB first. Start deferred to the next phrase boundary; `s_transport_running` records what gear
was last TOLD, not `isPlaying()`.

**Anti-drift invariants (audited 2026-10-12).** A box slaved to 24 ppqn can't drift in *tempo* (one 24th per
F8), only lose *position*; on DIN the only recovery is SPP/Start. Four holes fixed in `link_sync.cpp`:
- `schedule_next_event()` re-anchors `s_next_pulse` both ways: only "behind" was caught, so a backward
  timeline (`forceBeatAtTime`, bounce) left it ahead and `timeAtBeat()`'s future deadline **stalled the clock
  for that many beats**.
- An LTMP phase request must RE-ARM too (clear `s_next_pulse_valid`, set `s_pending_realign`, call
  `schedule_next_event()`) — else the armed esp_timer keeps the stale deadline.
- `s_pending_realign` is set whenever the clock *becomes enabled*, not only on a 0->1 peer edge: the
  `force_start` (no-peer) path streamed F8, **never SPP or Start**, so slaved gear never ran.
- `link_sync_send_boot_panic()` (Stop + CC123 x16) once at boot, before Link exists.
Keep SPP at EVERY phrase boundary, then Start/Continue (SPP-before-Start mandatory); SPP wraps at 0x3FFF
units = 4096 beats = 64 phrases, so the wrap lands ON a boundary — move SPP off it and the wrap becomes a
jump. **Residual:** all MIDI shares one 256-byte UART TX ring buffer, so a realtime byte queues behind bulk
data, up to 256 B x 320 us = 82 ms late. Doesn't accumulate (pulses scheduled absolutely from `timeAtBeat`),
so it's jitter not drift — but it delays Start. Bigger buffer = worse; do NOT raise
`MIDI_UART_TX_BUFFER_BYTES`. Real fix = separate realtime path. Bass phrase 64 bars (256 beats), MIDI phrase
64 beats: 64 | 256, coincide every 4th MIDI phrase.

### MIDI DIN compatibility

- `send_midi_nrpn` = CC99, CC98, CC6, CC38=0, CC101=127, CC100=127. The null reset (CC101/100=127) is
  mandatory — without it, later CC6 is misread as NRPN data on MicroKorg, Micron, RC-505.
- MIDI file player: `syncToBpm` is **false** — `beatAtTime()` already gives tempo-independent beats; enabling
  it double-scaled `playbackRate`, corrupting `endBeatAbsolute` on any tempo change and leaving notes
  permanently stuck. At the loop boundary `sendAllNotesOff()` + `activeNotes.clear()` fires instead of
  remapping note end times.
- UART: `uart_driver_install(MIDI_UART, MIDI_UART_RX_BUFFER_BYTES 512, MIDI_UART_TX_BUFFER_BYTES 256, 0,
  NULL, 0)`; TX 256 not 0, else a burst NRPN run (`setSidechainPattern`, 12 NRPNs = 108 B) blocks the main
  loop ~35 ms. `MIDI_NRPN_BYTE_COUNT` = 18; assert bound `MIDI_LONGEST_BACK_TO_BACK_NRPN_RUN = 7`;
  `MIDI_DIN_BAUD_RATE = 31250` (320 us/B).
- **No per-target compatibility table exists in `link_sync.cpp`** — CLAUDE.md's pointer to one is dangling
  (`41e313a`); KO2 / RC-505 / MicroKorg facts survive only as the NRPN-null-reset rule above and the note-off
  caveat in the CC/NRPN section.

## Observing the running mesh

This host (the ICS gateway) has `eth0` 192.168.137.1/24 and an unrelated `wlan0`; **there is no route to
192.168.4.0/24**, so the ESP's status port is unreachable from here and the serial console is the only ESP
telemetry (5 s `Link diag`, 10 s txprobe = 0.2 Hz). The Pi can poll the status port (20813 since `a100df8`) directly and should.

- **aloop telemetry: UDP `status` -> `192.168.137.132:4445`**, JSON in ~150 ms, one hop, costs the ESP
  nothing. **The Pi is `.132`, not `.100`** — `.100` is `FAILED` in `ip neigh` and 4445/4446 both time out.
  Useful fields: `xruns`, `link{synced,bpm,peers,phase_err_beats}`, `wifi`, `eff_speed`,
  `beat_mark{beat,late_ms}`, `groove`, `loopers`. ESP status `bpm` is **3 dp**, aloop `link.bpm` is **1 dp**.
- **`beat_mark.late_ms` is a sawtooth** (0.3..500, mean 250), not lateness — it tracks position in the beat.
- **Host-AP failover is certified** (compile `Oct 23 2026 13:28:17`, `4806b18729`, with aloop `0967bb7c`).
- **Host-AP failover is also certified on `2a5162f`** (compile `Oct 25 2026 08:42:55`; the image with the
  status port at 20813, the epoch guard and the destroy-based switch). 100 s outage at 10:45:00Z, run
  detached on the Pi with a TERM-timeout trap. Re-host at 10:46:12Z, tie-break yield at 10:46:45Z, Link peer
  back after 103838 ms. Pi side: AP stations=1 after. Witness: `.gm/witness-out/mesh-failover-2a5162f-20261025.txt`.
  The phrase-reseat warning (`phrase position jumped 1024.0 steps`) was float noise at zero advance: `phraseAdvance` (`main/bass_engine.cpp`) treats a backward move under `kPhraseNoiseSteps` as zero, and `tools/phrase-advance-witness.sh` runs in the CI `host-witness` job (green on `79bebc3`, run 40217155630).
  A verified 100 s outage runs: peer loss with the IP held and IGMP clean -> `STA down (n/30)` over ~60 s ->
  `host 'ticker' disappeared, re-hosting` + AP/DHCP on 192.168.4.1 -> `Lost dual-host tie-break (lower
  BSSID seen)` -> STA IP -> `Link peer back after 113678 ms`. ~114 s end to end. Two bugs had to be fixed
  to get there, both in `wifi_config.cpp`:
  - **`wifi_stop_link_relay()` left `g_relay_stop` latched** on its failure path, so every later
    `wifi_start_link_relay()` created a relay that exited on its first pass — one bad stop made the box a
    permanently relay-less host. Clear the flag on the failure path.
  - **A blocking `recvfrom` is not bounded** by `SO_RCVTIMEO` in any way you can rely on, so the relay sat
    through 2 s, then 5 s, then a wake-up datagram without ever reaching its stop check, and the yield
    deferred forever with the ESP stuck as a second host. The relay now waits with `select()` and a 20 ms
    timeout: the wait is ours. **`LWIP_NETIF_LOOPBACK` is off by default**, so a wake datagram to
    127.0.0.1 goes nowhere — do not re-add one.
  - Before those fixes the yield panicked: `Guru Meditation`, Core 0, `InstrFetchProhibited`, PC
    `0xffffffff`, `rst:0xc (SW_CPU_RESET)`, ~70 ms after `relay did not stop` — the teardown freed
    `g_ap_netif` under a relay task still holding it. Use `esp_netif_destroy_default_wifi()`, not
    `esp_netif_destroy()` (the plain call leaves IDF's own wifi event handlers registered; and it returns
    **void** in v6.1, so do not compare it against `ESP_OK`), and settle 200 ms **before** the destroy so
    `WIFI_EVENT_AP_STOP` is delivered first.
- **`WIFI_NO_TEARDOWN_ROLE_SWITCH` (`main/wifi_config.h`, default `0`) is the role-switch redesign, OFF until a device run verifies it.** The certified destroy-based switch is what runs today. The redesign creates both netifs once in `wifi_config_init` (before `esp_wifi_init`), sets `WIFI_MODE_APSTA` once, and never calls `esp_netif_destroy*` or `esp_wifi_stop`: `role_enforce()` parks the out-of-role netif (`esp_netif_dhcps_stop`, `esp_netif_dhcpc_stop`, zero IP via `esp_netif_set_ip_info`, then `netif_set_down` through `esp_netif_tcpip_exec`) and activates the in-role one (`netif_set_up` the same way, plus `esp_netif_dhcpc_start` for STA). **`esp_netif_up`/`esp_netif_down` are NOT declared in IDF 6.2** (CI: "did you mean `esp_netif_obj`") — bring a netif up or down with `netif_set_up`/`netif_set_down` inside `esp_netif_tcpip_exec`. `esp_netif_set_default_netif()` is re-applied on every switch and in the `IP_EVENT_STA_GOT_IP` arm, because an override is not sticky across `ESP_NETIF_STOPPED`/`LOST_IP`; `udp_sendto_errno` picks the AP netif for `IP_MULTICAST_IF` while hosting. Turning it on requires a device, a console opened ONCE per run, and the 100 s outage run: accept only on zero destroy calls, exactly one non-zero IP after each switch, and the Link peer back in the 4018-5387 ms band. Unverified on-device: whether `esp_wifi_set_config(WIFI_IF_AP, ...)` applies to an already-started APSTA interface (the new path never stops Wi-Fi), and whether the parked AP still beacons `ticker` while we are a STA. The flag-ON path is compile-verified only (CI run 39987365532 green with the flag at 1); it has never run on a device.
- **`LRST` is certified and safe to repeat**: `printf 'LRST' | nc -u -w1 192.168.4.x 20811` -> peers 0,
  `peerloss` line, `Link peer lost -- sent Stop + All Notes Off.`, `Link peer back after N ms`, `Link
  connected`. **N = 4018 ms** on `54e3d17` (was ~2540 ms on the older image); bpm and beat stay monotonic;
  the telemetry counters never step backwards, so it is a session bounce and not a restart.
- **Detecting a Pi-side aloop restart from telemetry**: use the monotonic counters `resample_chain.samples` /
  `resample_fold.samples` and `beat_mark.beat` — a step backwards means aloop restarted. **Do NOT use
  `xruns`**: a restart that has not yet dropped a buffer leaves it at 0, so it cannot distinguish a restart
  from a quiet run (measured 2026-10-23: five restarts in 86 s with `xruns` pinned at 0). A restart drops the
  Link peer for 5-13 s and moves the peer's Link source port, and is by far the most common cause of a
  "peer loss" that is not a device fault.
- **Attaching the USB console RESETS the board**: every open of `/dev/ttyUSB0` prints `rst:0x1
  (POWERON_RESET), boot:0x13`. A soak must open the port **once** and never re-open — a second open turns the
  run into a reboot test. One 93.4%-retention reading was traced to exactly this.
- **Pi clock resets to 2024-04-09 every boot** (no RTC). The final device is offline, makes its own AP and cannot
  sync its clock, so wall time is a log-only detail for bench work. Do NOT step it during a certification run —
  a 926-day jump on a running PREEMPT_RT process is itself a perturbation. Align by offset, fix in a gap between
  windows, restart aloop after.
- **Pi root is tmpfs, so nothing survives a reboot**: no logs before boot, no `last reboot`, no pstore. The
  SSH host keys regenerate every boot, so the Pi's host key changes on every reboot (the 2026-10-25 change).
  The 2026-10-25 ~18:30Z reboot (the 63.8 s Link gap in the 3 h soak) has no recorded cause.
- **`ip_forward` on the Pi is 0**, so nothing else can route to 192.168.4.0/24 and `route add` is pointless.
  Turning it on would expose the open SSID to the ICS subnet; don't, unless asked.
- **Tempo cross-check, unresolved**: ESP 118.631 vs aloop `link.bpm` 118.6 vs `beat_len_samples` 24277
  (=> 118.594); `phase_err_beats` 0.000 and `eff_speed` 1.00000 say the boxes share one timeline, so this is
  readouts, not two tempos.
- `esp_wifi_statis_dump(uint32_t modules)` is the only Wi-Fi buffer-statistics API (`esp_wifi.h` v6.1); no
  public enum for its bitmask, so dump all with `0xffffffff`. AMPDU has no runtime control — only
  `ampdu_tx_enable` at init (`3d703ce`).
- **`bad` (badmagic) is benign**: Link's own `_link_v1` measurement traffic, 1:1 with `Link msr: rx`.

## Live state 2026-10-23: two-device mesh, measured with mc-420

Device runs `54e3d17`, compile `Oct 13 2026 13:49:32`, ELF `b65e475f9` (flashed 2026-10-23; was Oct 11 /
`dd5d08e11`). Acceptance agreed with mc-420: peer retention >=95% measured after t+60 s from any cold boot,
bpm spread <=0.05, `|phase_err_beats|` <=0.03, `eff_speed` spread <=0.005, sample gaps <=10%.

- **Cold-boot acquisition**: reboot -> `peers=1` in 14 s (stale peer ages out ~7 s, reacquire 14.1 s), 20 s on
  a second boot, 23 s after a flash. Measure retention from t+60 s or the boot reads as a failure.
- **10 min run 07:20:52Z-07:30:52Z (pre-flash image)**: 600 samples, 0 gaps, 540 post-acq at **100.0%**
  retention; bpm/phase/eff_speed spreads all 0. ESP: 124 `Link diag` lines, 123 `peers=1`, txprobe
  `e_gw=0 e_unknown=0 e_group=0 e_sock=0` throughout, heap 130-132k, rssi -33..-42.
- **30 min run 07:35:28Z-08:06:08Z (on `54e3d17`)**: 1798 samples, 1 gap, **99.7%** retention, spreads all 0.
  **One drop at t=1560.4 s (08:01:29Z)** and `44ce414`'s instrumentation settled it:
  `peerloss: flag=1 ap=0 ap_sta=0 ip=192.168.4.10 rssi=-33 chan=6 igmp(ok=53 fail=0) heap=132800`, netif
  `up=1 link=1 ip=192.168.4.10 gw=192.168.4.1` — **associated, IP held, IGMP clean, so a `peers` 1->0 is the
  Link SESSION, not the IP path.** Recovery `Link peer back after 5387 ms`. Cause: mc-420's lane ran
  `rc-service aloop restart` at that second (its `xruns` went 65 -> 0 and the peer's Link port moved
  52984 -> 40006). Not a device fault on either side.
- **ICMP is not the metric**: ~1% ICMP loss Pi->ESP (17/1765, then 33/3342) while Link retention stayed 100%
  and `peers0` count 0 — the Link session rides through brief loss, so "IP path down" is far worse than a few
  missed pings. ICMP at 1 Hz plus the status reply (20813 since `a100df8`; it was 20812 before) is the discriminator: both dead = IP path, ICMP alive with
  a session drop = Link state.
- **3 h run 08:12:36Z-11:12:36Z: PASS on every term, two independent captures.** Mine: 10791 telemetry
  samples, 0 gaps, 10731 post-acquisition, retention **100.0%**, bpm/phase/eff_speed spreads all 0, zero
  post-acquisition drops; ESP serial 2162 `Link diag` lines, **2162 `peers=1`**, the single `peers=0` is the
  boot, **zero `peerloss` and zero `sendfail` lines**. mc-420's capture agrees (10587 samples, 100%, 0 drops).
  This is the certification run: three hours, no unexplained event on either device.
- Metro jitter over the 30 min run: `late last` 34-69 us, `worst=13798us`, `mean=264us`, `rms=822us`.
- **Earlier, still open**: the ENOMEM send failure did NOT reproduce in a 1500 s soak on `df18c36`
  (`ok=16307 fail=0 giveup=0 errno=0`); only evidence is one ~6 s burst at t=218.9 s in a 600 s run
  (`err=12`, inside an IGMP re-join). **Session bounce** `enable(false)`/`enable(true)` re-joins in 2540 ms;
  the per-boot auto bounce was removed, only on-demand `LRST` to `192.168.4.x:20811` remains.

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps pad -> approach, single/double tap (`DOUBLE_TAP_WINDOW_US = 300000`): 0
`APP_ROLL`/`APP_ACID`, 1 `APP_FUNK`/`APP_GLITCH`, 2 `APP_STAB`/`APP_DRIFT`, 3 `APP_PEDAL`/`APP_BREAK`.
`setApproach()` is the ONLY thing that activates the engine. `shapeBar()` per bar from the 25-field
`ApproachShape` (`bass_engine.h:80`). **An approach is a VOICE, not just a preset** (`0f1271f`):
`dialsForApproach()` biases base dials -> `Dials::bandIndex` -> `kGrooves`/`ContourShape`/`kScaleNames`; a
switch clears `m_hasPrevPhraseMotif`, sets `m_regenPending`. PEDAL/DRIFT `minGate` 1.10/0.70 > 1 step =
sustained voices. Asserts cap microtiming (max-abs <= 0.17 step, SD <= 0.15) — don't raise
`kMicroTimingFullDepth`. Pitch bend: `raw` starts at `kPitchBendCentreRaw 8192`, emitted only when it changes
(deduped by `s_last_pitch_bend_raw`), high byte masked `(raw >> 7) & 0x7F`; `kPitchBendHalfRange 8191`,
`kPitchBendRawMax 16383`, `kPitchBendSemitoneSpan 12.f`.

### Psytrance musicality (reworked 2026-10-13, witnessed on host)

`tools/bassline-witness.sh` runs the REAL engine off-device (stubs in `tools/hoststub/`, 145 bpm): bar x 16th
grid + metrics; `--mid <file>` writes a playable MIDI file so a change is AUDITIONED; `--property` and
`--mutate` (added 2026-10-23) run the invariant and mutation suites. Witnessed BEFORE: ROLL emitted a
13-semitone melodic *sweep* and all 8 of 8 consecutive bars differed — a wandering jam. AFTER: `0 0 0 0 12 12
12 12 ...`, register +0..+12, 13 notes/bar, 4-bar holds. The invariants that hold the genre:
- **Rhythm, not melody.** `kGrooves` is psytrance-first (`psy_offbeat`, `psy_roll`, `psy_gallop`,
  `psy_ratchet`, + minimal, deep, `acid_303`, `techno_roll`, `funk_synco`); `house_bump`/`garage_2step`/
  `dembow`/`four_floor` removed — four_floor is the *kick* pattern. **Index 8 (top energy band) must stay a
  full 16th roll** — ROLL's effective `grooveEnergy` is 0.94, so a sparse template there silently stops the
  densest voice rolling.
- **Bass grammar** (`transCost`): repeat ~free, octave cheap, step costly, wide leap very costly; non-chord
  tones 0.95 not 0.52. Cheap repetition IS the hypnosis.
- **`ApproachShape::repeatBars`** = bars one shaped figure is HELD: ROLL 4, ACID 2, FUNK 2, GLITCH 1, STAB 4,
  DRIFT 8, PEDAL 8, BREAK 2. GLITCH=1 is deliberate (the one non-repeating voice). Copy-then-`varyRepeat`
  (vel +-4, CC74 +-6, occasional ghost) stops a hold sounding sampled. **A rest is never held**: when the
  shaped figure is empty, `figureKeyHeld` is left at -1. **And a bar that just rested may not rest again**
  (`756b57e`) — otherwise BREAK's `restBarProb` 0.22 produced three consecutive silent bars (~5 s at 145 bpm)
  by coin flip rather than by arrangement.
- **Register**: the chord offset is added AFTER `clampRange` bounded the motif, so the sum escaped the bass
  (+15 motif, +11 chord). Now octave-folded into `kBassLowNote 33`..`kBassHighNote 52`.
- **Syncopation capped** at `kSyncTargetCap 0.45`; **ghost drop is 25, not 45** (at vel 10-40 a ghost vanishes
  and holes the roll); **filter floor rises per section** `kSectionFilterRise {0,8,18,5}` — this is what opens
  the line across the phrase.
- **Witness resolution matters**: simulate at 1/64 notes, not 1/16 — at 1/16 a note at step 2.053 is emitted
  during the step-3 call and lands in the wrong cell, under-reporting density.

**Property and mutation suites (2026-10-23, verified here)**: 30/30 properties HOLD over 4 roots x 9 scales x
5 energies x 3 variations x 24 seeds (~144k checks on the motif properties). Five candidates were REJECTED as
body-derivable (readable off a constant table or already proven by `static_assert`) — that rejection is the
admission rule, not a gap. Mutation set psy-bass-v1: 20 mutants, 18 killed = 90%. The two survivors
(`leap-window-24`, `leap-penalty-off`) are equivalent: every sampled interval wider than 7 semitones is
exactly an octave (wideLeapShare 0.1212 vs octaveShare 0.1211), so the `d > leap` branch is unreachable.

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build: `network_midi_init()` starts `esp_http_server` on 8080 —
`POST /upload/*` -> `/spiffs/loops/<name>`, `GET /info` -> `{"device_ip":...}`, `POST /clear` deletes `*.mid`.
Invisible at the call sites: **4 sockets** of 16 reserved; SoftAP allows **8 stations**
(`cfg.ap.max_connection = 8` = `MAX_AP_STA_IPS`) — also all the forwarder can unicast to, so a 9th associates
but gets no Link traffic; `WIFI_PS_NONE` (a dozing station misses multicast) — don't re-enable. **SPIFFS IS
mounted** (`9568578`): `mount_clip_storage()` runs first; `format_if_mount_failed = false` — never formats.
Contract: 503 unmounted, 413 body over `total - used`, 400 bad/empty name/body, 500 incomplete (partial
unlinked). URI wildcard is one segment re-checked (`is_single_clip_name`), so `/` and `..` can't escape;
`/info` re-reads the IP per request. Open: no auth on an open SSID; `network_midi_start()`/`_stop()` dead;
`clear_handler` `filepath` truncation unchecked; `d_type == DT_REG` unproven.

## Wiring

GPIO from `main/main.h` — a pin can't be expressed by reading the macro.

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

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships `touch_sensor`); a pad reads LOW when
touched; ARP read first (lowest press latency). Pots: raw ADC 0-4095 -> MIDI 0-127; in the player pot1 = note
length (0..2), pot2 = velocity (0..1).

## Constants whose constraint is invisible in the code

- **E-Slew must stay derivable from sheer, anchored at 104**: `reset` sets sheer 0, so a derivation not
  returning 104 at sheer 0 makes reset overwrite what `setSidechainPattern()` just sent. 104 isn't magic;
  sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`** (`gateWetDryForDepth()`), because MiniNova **CC 91 is
  wetness, 127 = full Gator** (Manual v1.01). All outside SRCS — nothing here runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: trailing `{3,5}` is padding so the row
  matches its neighbours. Don't fix it.
- `kRegisterSpan = 15` (`bassline_interpreter.h`) must track the `anchorMotif` clamp in `bass_engine.cpp` —
  nowhere else; neither file includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones, not MIDI notes (fixed `c0fc974`): `{0,5,3,6}`
  i-VI-iv-VII, `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}` i-VI-VII-V ("dark cadence"),
  `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` pedal.
- `main.h` asserts `LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS`: the click's accent cycle IS the quantum —
  changing one alone breaks the build.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape CC 75 (`kCcLfo2Shape`); delay sync CC 13 (`kCcDelaySync`), 0 = off, 1..32 values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never confirmed on hardware**.
  Filter bypass (NRPN 0/60 value 0) is deliberately NOT sent by `deactivateFilter()`. `main/nrpn_chart.txt`
  is the chart (was misspelled `nprn_chart.txt`, `3d42c32`); `main/lfo_constants.h` carries only NRPN 0/86.
- `SynthInterface` contracts live only in the implementations: `activateDelay()`/`activateReverb()` select
  Delay 1 / Reverb 1 in **FX Slot 1**; `setFxSlot1Level()` is **CC 91**; `activateFilter()` implicitly selects
  **LP24**; LFO methods assume **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` -> `0..127`, **64 = zero**, so `unpatchLfoFromFilter()` writes 64. The
  two surviving `= 64` defaults (cutoff, patch depth) must stay. Note-off velocity 0 is the default across all
  three headers — passing note-on velocity sticks notes on RC-505 MK2 and KO2.

## Build shape

**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`, `effect_sidechain.cpp`,
`midi_file.cpp` — nothing `#include`s a `.cpp`, so that cluster is reachable only from itself.
`bassline_interpreter` must stay **host-compilable**: only its own header plus `<algorithm>` `<cmath>`
`<cstring>`, no ESP-IDF headers (MinGW: also `-D_USE_MATH_DEFINES`, or `M_PI` is undefined).
`tools/bassline_witness.cpp` extends that to the whole engine via `tools/hoststub/`. **`.gitattributes` pins
EOL** (`7e8dacd`): `eol=lf` for shell/py/cmake/csv/sdkconfig/Dockerfile, `eol=crlf` for `*.bat *.cmd`,
`binary` for `*.bin *.elf *.mid`; a CRLF-checked-out shell script dies in the Linux CI container.

**There is no local ESP-IDF on this host and none is needed — GitHub Actions is the toolchain.** The loop
for a firmware change is push -> Actions builds (~3-5 min) and commits the three `.bin` on green `main` ->
`git pull` -> flash (~25 s) -> observe on the console or via telemetry: about six minutes per edit. Never
defer a firmware change for want of a local toolchain. CI publishes only the three `.bin` and **not the
ELF**, so a panic backtrace cannot be symbolised from here.

## CI autobump: how it commits, and its hazard

`build.yml` commits the three `.bin`s: records the sha it built, `git reset --hard FETCH_HEAD`, restores the
`.bin`s, commits with a 3-path pathspec, up to 5 attempts (`111d902`: recommit on the latest ref, never
rebase — a rebase would MERGE the previous `.bin`s and die). **`reset --soft` is NOT usable**: it leaves the
index on the tree this checkout built, so the commit silently reverts every source commit in between.
`refuse_unless_bins_only()` (`8454302`); the binary commit is gated to `main` (`d0afa95`).

**Residual rule: re-verify any commit touching a non-`.bin` path against HEAD after a pull** — a green build
never proves a change survived; whole commits have vanished (one autobump dropped 264 lines of AGENTS.md
while shipping two `.bin`s). CI rewrites the `.bin`s on `main` on nearly every push, so `remote_moved` is
normal — push with `git_push {recover_remote_moved: true}`; `git_finalize` auto-recovers.

`../aloopprime` is the opposite — **fetch-only checkout** (`origin.pushurl = no-push`): `git_finalize` there
commits and gm refuses the push with `push_disabled_by_config`. Local-only commits are the intended end state;
don't "fix" it by editing its `.git/config`.

**mc-420** (Pi-side peer agent) is reachable by `SendMessage` from this host — the old "permanently refused,
drop files in `C:\dev\mc-420\.gm\`" note applied to a different setup; on this machine the agent is a session
on the same socket and its project is the sibling `/config/workspace/mc-420`. Its `reassoc-watch.js` logs only
`Date.now()` deltas, so its absolute times must be reconstructed. Any Pi ssh credential is for read-only
probing on our own device and must never be committed. One exception, by the owner's decision on 2026-10-26: the
  Pi's fixed SSH host key may be committed to the public project git, because the device is a music instrument. The
  repos are public, so a rotation is needed if that key is ever exposed beyond the device's use.

## ESP-IDF 6.x breaks absorbed, and comment sweep

Historical — none visible in current source, each silently re-breaks on a bump: `esp_netif_next()` ->
`esp_netif_next_unsafe()` (6.1, link's `ScanIpIfAddrs.hpp`); legacy `driver/adc.h` removed (6.1) ->
`hall_sensor_read()` stub in `main/io_helpers.cpp`; `CONFIG_LWIP_PBUF_POOL_SIZE` dropped. **Comment sweep
(INVARIANT 3)** measured 2026-10-11: **0 comments** — `main/` 42 files / 8811 lines (`82038c4`), `tools/` 5 py
/ 673 lines (`5342086`). Only false positive is the `"/upload/*"` string in `network_midi.cpp`. Recount
before quoting a number.

## gm usage gotchas

- `codesearch` is the canonical search verb. Exhaustive modes (`literal`/`regex`/`filename`) REFUSE unknown
  fields; scope with `path`/`files`/`glob` (`scope` is refused). `literal` scans the tree only, never commit
  messages — use `dual` for those.
- `git_finalize` takes `message`/`paths`/`files`/`allow_whole_index`/`recover_remote_moved`/`source_ref`/`rev`
  — `body` or top level, `body` wins. **`paths` must be inside `body`** (top level ->
  `blanket_stage_refused`). **Never name a gitignored path** (e.g. `build/*.bin`, anything under `.gm/`) —
  it is refused, so witnesses under `.gm/witness/` stay on disk and off the commit.
- `prd_add`/`prd_resolve`: send fields FLAT (nesting a payload `body` inside the envelope `body` used to
  overflow; fixed in gm-mcp 0.2.267 — if your client is older, go flat). `prd_resolve` REQUIRES a bound
  witness: either `witness_dispatch_id`, or all four of `witness_exit_code` (0), `witness_output_sha256`,
  `witness_output_path` (project-relative, re-hashed and re-read) and `witness_ts` (RFC 3339). It also wants
  `resolution`, not `reason`.
- `git_show` takes `rev`/`stat`/`path`; `git_log` takes `limit`/`range`/`path`. `bash`/`exec_js` take their
  script in `body` or `raw_body`. `session_id` is mandatory on every call.
- On this host `python3` EXISTS (3.14, pyserial present) — the "no python3, use node" note was for the Windows
  box. `/dev/ttyUSB0` is root:dialout, so serial work needs `sudo` (passwordless here).
