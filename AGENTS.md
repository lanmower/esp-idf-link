# esp-idf-link — agent notes

CLAUDE.md starts with @AGENTS.md; durable facts live here, not in code — don't extend CLAUDE.md.
Compacted 2026-11-05: git log through 112e107; 2026-10-11/23/25 live state merged; memory dir empty;
bass/SPP/comment-count claims corrected against current main/.

## Mesh with ../aloopprime: paired invariants (change BOTH or the mesh splits)

|Invariant|esp-idf-link|aloopprime|
|---|---|---|
|SSID|main.cpp wifi_*("ticker")|hostapd.conf, wpa_supplicant.conf|
|Auth|wifi_connect_sta("ticker","")|open (key_mgmt=NONE; wpa= commented)|
|AP addr / DHCP|esp_netif_set_ip_info 192.168.4.1/255.255.255.0|192.168.4.1/24, dnsmasq .2-.20|
|Channel|SoftAP ch6 (kTickerChannel = 6)|hostapd.conf channel=6|
|Link multicast|224.76.78.75:20808|same (hardcoded in Link)|
|Quantum|main.h LINK_QUANTUM 16.0|link_bridge.cpp quantum = 16.0|
|Start/stop sync|enableStartStopSync(false)|same, false|
|Host election|lowest MAC/BSSID wins|lowest MAC/BSSID wins|
|Tempo authority|sets only on explicit LTMP|refuses to propose when peers own tempo|

Boot (main.cpp app_main): open SSID ticker; scan stagger by last STA-MAC byte (0-3825 ms) -> wifi_scan_best_bssid("ticker",...) (SSID-filtered active scan, esp_wifi_scan_get_ap_records, lowest BSSID) -> match: wifi_connect_sta, 30 s else host; no match: wifi_start_link_ap + relay. ensure_sta_started() stops a default-STA netif leaking across rescans.

Host election: both scan before the other AP exists -> 2 isolated L2 domains. HOLD_MAX_MS=6000 monotonic in STA MAC (mac_rank * HOLD_MAX_MS >> 24), rescan 1/s, stagger mac[5]*15 ms; ../aloopprime/src/net/autoap.sh mirrors (0-6 s, lowest-wins); invert both together.

Supervisor (wifi_config.cpp, 2 s; RECONNECT_TRIES=30, IGMP re-assert 30 s): STA reconnects 30x (~60 s) before re-hosting. STA up = has IP (579d4ef): g_wifi_connected set by IP_EVENT_STA_GOT_IP, cleared by WIFI_EVENT_STA_DISCONNECTED; Link drops a 0-address netif and erases the gateway, so peers 1->0 with no Wi-Fi event (gate on sta_has_ip()). AP yields to a strictly-lower ticker BSSID only with zero stations; compare STA MAC (2nd e4:65:b8:76:cf:c4, 1st e4:65:b8:77:0d:14), never the IP or SoftAP MAC; self-BSSID guard esp_wifi_get_mac(WIFI_IF_AP), never esp_read_mac(). Hosting scan pinned to ch6 (needs WIFI_MODE_APSTA); all-channel scans break mid-scan DHCP. AP->STA yields only after hosting once.

Reading peers=0: gw(ok=1) gateway up; dst= on a high port (192.168.4.1:54146) = announcement arrived, Link unicast-pinging -> failure is ping/pong NOT multicast. dst is HOST order (asio to_v4().to_uint() vs network-order ip4_addr_t::addr) -> htonl before IP2STR; pre-579d4ef images reversed it (99.168.4.1 trap).

Link peer loss (read-only tree, do NOT patch): UdpMessenger.hpp:74-77 swallows send errors, platforms/esp32/Context.hpp discards the exception handler -> asio/Context.hpp catch loop not compiled, repairGateway dead; equivalent (gateway erase + 5 s rescan) via PeerGateways Callback::operator(). Routes: TTL ~6 s; ByeBye; gatewayClosed (Peers.hpp:136-144 triggers, Peers.hpp:302-312 erases ALL peers on that address — the 1->0-no-Wi-Fi-event route, also enable(false) clear()); resetPeers().

Link checklist (both trees): App captureAppSessionState()/commitAppSessionState() off audio thread, AudioSessionState variants only on it; start/stop sync OFF: clock-only mesh (b94ac2d), isPlaying() false all session (transport ESP-local), re-enable needs BOTH trees + flash; three callbacks on a Link thread, Realtime-safe: no (bounded logging/atomics); setTempo rewrites EVERY peer (here only explicit LTMP); interface readiness race: 500 ms settle before Link construction, IGMP re-assert every 30 s never one-shot; Pi advertises one NodeState per IFF_RUNNING interface -> mep4 flips 192.168.4.1 <-> ICS, Peers::sawPeerOnGateway caches last; off-link mep4 counted/logged never dropped (cfddf6d reverted 4481a5a).

Vendored Link is a checked-in FORK: components/link-esp/link patched copy (1506 files, no nested .git); git submodule update clobbers silently (stale .gitmodules removed; Dockerfile/setup.sh/build.yml don't init submodules); patches Socket.hpp, PeerGateways.hpp, UdpMessenger.hpp, Controller.hpp, Peers.hpp — re-apply on bump. Two API traps (host-compile, device-break): asio send_to(buffer, to, 0, ec) (flags AFTER endpoint, opposite Boost); PeerState::endpoint data member vs MeasurementService/PingResponder/Socket::endpoint() methods. Compiled asio is components/espressif__asio 1.32.0, not vendored 1.17.0 (errno 12/ENOMEM prints "Not enough space"); wire tempo is truncated int64 micros/beat (Tempo.hpp:52). Relay hook is the divergence: platforms/asio/Socket.hpp calls extern "C" wifi_link_multicast_forward(...) on every send -> wifi_config.cpp unicast-copies discovery datagrams across the SoftAP boundary; any bump must re-apply it (losing it = silent non-discovery), gated if (!g_ap_active) return; Socket::send retries failed send_to 3x, 1..8-tick backoff, logs ok/retry/attempt/giveup + errno (cfddf6d, 0a8760c).

## SoftAP multicast gap, and lwIP core context

ESP32 SoftAP won't carry Link multicast host<->stations, hence link_multicast_relay_task: re-emits each datagram to the group, AP own IP and unicast to each station, preserving source IP. Relay singleton, g_ap_netif mutex-guarded (0bffe4c): link_relay_release() frees socket + raw pcb. LOCK_TCPIP_CORE()/UNLOCK_TCPIP_CORE() are EMPTY MACROS (2c8807a; CONFIG_LWIP_TCPIP_CORE_LOCKING unset, LWIP_TCPIP_CORE_LOCKING==0) — lwIP core work ran from foreign tasks with no exclusion; real serialization is esp_netif_tcpip_exec(esp_netif_callback_fn fn, void* ctx) (callback on tcpip thread, exec blocks); all IGMP joins + relay raw-pcb work (raw_new, raw_remove, raw_sendto_if_src) go through it. Never hold ap_netif_lock across it (supervisor snapshots g_ap_netif under lock, joins outside). AP->STA yield leaks fixed (dc9add9): relay not stopped (g_relay_stop set only in wifi_start_link_relay), g_relay_task non-volatile under spin, forward socket function-local static — now closed on yield under send-hook mutex; wifi_stop_link_relay() waits 250 x 20 ms = 5 s (RELAY_STOP_MAX_POLLS, wifi_config.cpp:963; the "100 x 20 ms" note was stale); wifi_join_link_multicast() returns unless STA has an IP; supervisor re-asserts within 2 s of sta_up.

## Clock is hardware-scheduled, never tick-emitted

Click + 24 ppqn come from an esp_timer one-shot armed to SessionState::timeAtBeat() of the NEXT pulse, never the 4 kHz gptimer tick (LINK_TICK_PERIOD 250 us). Inputs sampled every 8 ticks (2 ms); note step every tick. The scheduler skips past due pulses (cap MAX_CATCHUP_PULSES_PER_SCHEDULE=1024) and arms a strictly future deadline; schedule_next_event() re-anchors both ways. BassEngine (main/bass_engine.cpp): 48 ticks/beat; a stall past kMaxCatchUpTicks=48 calls allNotesOff() and resynchronises without catching up.
sdkconfig.defaults load-bearing: CONFIG_LWIP_MAX_SOCKETS=16 (max; default 10 exhausted by relay/LCLK/TTMP/tempo listener/discovery forward/4 HTTP -> EMFILE -> no discovery); CONFIG_LWIP_TCPIP_RECVMBOX_SIZE=64 (ENOMEM = pool drained OR mailbox full); CONFIG_LWIP_UDP_RECVMBOX_SIZE=6, CONFIG_LWIP_STATS=y (mem.err/link.drop/link.memerr), CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=128 (f09e91c). CONFIG_LWIP_PBUF_POOL_SIZE is NOT a v6.1 symbol — don't re-add.

## Phrase boundary and SPP arithmetic (2026-11-05)

PHRASE_BEATS 64.0 (16 bars), LINK_QUANTUM 16.0, 4 SPP units/beat, SPP mask 0x3FFF (16384 units = 4096 beats = 64 phrases, so the wrap lands on a boundary). beatAtTime is PHASE-ENCODED (toPhaseEncodedBeats, Phase.hpp:76-81): derive phrase numbers from beatAtTime(time, PHRASE_BEATS), never beatAtTime(time, 16) (detectQuantumBoundary, link_sync.cpp ~334-370). SPP at EVERY phrase boundary; SPP-before-Start mandatory; fmod wrap for negative beats (389f9d7).

## Firmware, flash and CI (USB only)

- CI builds on any push; IDF image digest-pinned Dockerfile:1 (64 lowercase hex; resolve-idf-image exits 1 otherwise); green main commits the three build/ images; flashable images come from git; runners ubuntu-24.04.
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition of the
BUILT table (0x20000 with partitions_large.csv; parse the binary: 32-byte entries, LE magic
0x50AA, type(1) subtype(1) offset(4) size(4) label(16), app type == 0). App offset DERIVED,
never hardcoded (31a6b99); never parse partitions_large.csv (every offset but nvs's blank).
- esptool 5.x: write-flash hyphenated; --verify dropped (it always verifies). SPIFFS
storage flashes separately: flash_midi_data.sh builds a 0x19000 image from ./data at
0x317000 (nvs 0x011000, phy_init 0x017000, app0 0x020000, app1 0x1A0000, storage 0x317000).
- Flash (Linux): sudo python3 -m esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default-reset --after hard-reset write-flash --flash-mode dio 0x1000 bootloader.bin 0x8000 partition-table.bin 0x20000 app.bin, bins from a CI bin commit (git show <sha>:build/...). Check 'Hash of data verified' and the 'MAC:' line; the attached unit changes. Install: sudo pip install --break-system-packages esptool (python 3.14, PEP 668).

Build shape: not in build — effect_arp.cpp, effect_filter.cpp, effect_handler.cpp, effect_sidechain.cpp, midi_file.cpp (no .cpp is #included). bass_engine/control_model must stay host-compilable (own headers + <algorithm> <cmath> <cstring>, no ESP-IDF; MinGW needs -D_USE_MATH_DEFINES); tools/bass_loop_witness.cpp extends via tools/hoststub/. .gitattributes pins EOL (7e8dacd: lf shell/py/cmake/csv/sdkconfig/Dockerfile, crlf *.bat *.cmd, binary *.bin *.elf *.mid); CRLF shell script dies in Linux CI. CI host-witness runs tools/control-model-witness.sh, tools/bass-loop-witness.sh. No local ESP-IDF needed — GitHub Actions is the toolchain: push -> build ~3-5 min -> green main commits three .bin -> git pull -> flash ~25 s -> console/telemetry, ~6 min/edit; never defer a change for want of a local toolchain; CI publishes no ELF (panic backtraces can't be symbolised).

CI autobump hazard: build.yml commits the three .bin files — records the sha built,
git reset --hard FETCH_HEAD, restores the .bin files, commits with a 3-path pathspec, up to 5 attempts
(111d902: recommit on latest ref, never rebase — a rebase would MERGE the previous .bin files and die).
reset --soft NOT usable: leaves the index on the tree this checkout built, so the commit silently
reverts every source commit in between. refuse_unless_bins_only() (8454302); binary commit gated to
main (d0afa95). Residual rule: re-verify any commit touching a non-.bin path against HEAD after
a pull — a green build never proves a change survived; whole commits have vanished (one autobump
dropped 264 lines of AGENTS.md while shipping two .bin files). CI rewrites the .bin files on main on nearly
every push, so remote_moved is normal — push with git_push {recover_remote_moved: true};
git_finalize auto-recovers.

## Mesh UDP protocol and MIDI emission

Payloads little-endian int64; ports are #defines in link_sync.cpp (LINK_PHASE_PORT 20810,
LINK_TEMPO_PORT 20811, LINK_TTMP_PORT 20812, LINK_STATUS_PORT 20813 since a100df8,
TX_PROBE_PORT 34999). Authoritative table: readme.md ## Network ports.

|Port|Direction|Payload|Purpose|
|---|---|---|---|
|20810|esp -> group ~50 Hz|LCLK + Link-clock micros|phase for peers behind a unicast-RX wall|
|20811|looper -> esp|LTMP 12 B tempo-only / 28 B tempo+phase; LRST 4 B|set tempo/phase; session bounce|
|20812|esp -> group ~10 Hz|TTMP + microsPerBeat + beatOrigin microbeats + timeOrigin micros|esp -> looper timeline|
|20813|req/resp|any 1..64 B -> one-line JSON|status: peers/bpm/playing/beat/phase/quantum/ap/metro (was 20812)|
|34999|esp -> gw / group|diagnostic|log_tx_probe every 10 s + log_send_failure_probe; nothing replies|

- Every multicast send must iterate netifs and set IP_MULTICAST_IF per interface (esp_netif_next_unsafe), else a raw socket exits the wrong interface on a dual-netif device.
- LTMP wire shapes: 12 B = LTMP + int64 microsPerBeat; 28 B adds beat0us + quantumUb = quantum*1e6. Both re-arm the esp_timer deadline (112e107); only the phase shape sets s_pending_realign. LRST 4 B on 20811: enable(false); enable(true); enableStartStopSync(false) — no Wi-Fi teardown. LTMP/phase requests apply on the Link task, not listener; prefix+length checked, short PING can't be mistaken.

- Tempo bounded 20..999 bpm (LINK_TEMPO_MAX_BPM 999.0, 396092d); status is request/response, not broadcast; MetroStats rides in it as metro.
- LCLK/TTMP are subscriber-driven (0a8760c): a datagram to 20813 or 20811 is a subscriber hit (since a100df8; was 20812, where the ESP's own TTMP looped back and kept itself subscribed), held SUBSCRIBER_HOLD_US 15 s. Without one, LCLK 50->1 Hz, TTMP 10->0.2 Hz; clock limited to one packet/20 ms (timeline 100 ms). peers readouts are not passive — polling raises broadcasts 15 s. Status responder replies to ANY 1..64-byte payload, only reads captureAppSessionState, cannot perturb the session.

MIDI emission: one path, no per-device clock code; bytes are constants in main.h (0xF8 clock, 0xFA start, 0xFC stop, 0xFB continue, 0xF2 SPP, CC123 all-notes-off); MIDI_PULSES_PER_QUARTER_NOTE=24, MIDI_SPP_UNITS_PER_BEAT=4 in link_sync.cpp. SPP + Start/Continue ONLY at the 16-bar boundary; Stop + CC123 on all 16 channels on transport stop and peer loss. SPP counts MIDI beats (sixteenths), 1 Link beat = 4 units, 14-bit LSB first; Start deferred to next phrase boundary; s_transport_running records what gear was last TOLD, not isPlaying().

Anti-drift invariants (ca8f04b): a 24 ppqn slave can't drift in *tempo* (one 24th per F8), only lose *position*; DIN recovery = SPP/Start. Four link_sync.cpp holes fixed: (1) schedule_next_event() re-anchors s_next_pulse both ways (only "behind" was caught, so a backward forceBeatAtTime left it ahead and timeAtBeat()'s future deadline stalled the clock); (2) LTMP phase must RE-ARM (clear s_next_pulse_valid, set s_pending_realign, schedule_next_event()) — since 112e107 tempo-only shape re-arms too; (3) s_pending_realign set whenever the clock becomes enabled, not only 0->1 peer edge (the force_start path streamed F8, never SPP/Start); (4) link_sync_send_boot_panic() (Stop + CC123 x16) once at boot before Link exists.
Keep SPP at EVERY phrase boundary then Start/Continue (SPP-before-Start mandatory). Bass loop at most 8 beats (2 bars, 32 steps); MIDI phrase 64 beats -> coincide every 8th loop (old "64-bar/256-beat bass phrase, every 4th MIDI phrase" is obsolete). MIDI bulk pacing (main/midi_bulk_queue.h, MIDI_BULK_PACE_TICKS=2): NRPN/CC drains one byte per 500 us into the 256 B ring, so F8 (direct, link_sync.cpp send_midi_bytes) waits <=1 byte (320 us). FA/FB/FC/F2 stay ordered behind bulk (SPP-before-Start). Witness: tools/midi-bulk-queue-witness.sh.

### MIDI DIN compatibility

- send_midi_nrpn = CC99, CC98, CC6, CC38=0, CC101=127, CC100=127; null reset (CC101/100=127) mandatory — later CC6 is misread as NRPN data on MicroKorg/Micron/RC-505 without it.
- UART uart_driver_install(MIDI_UART, MIDI_UART_RX_BUFFER_BYTES 512, MIDI_UART_TX_BUFFER_BYTES 256, 0, NULL, 0): TX 256 not 0, else a 12-NRPN burst (108 B, setSidechainPattern) blocks the main loop ~35 ms; MIDI_NRPN_BYTE_COUNT=18, assert MIDI_LONGEST_BACK_TO_BACK_NRPN_RUN=7, MIDI_DIN_BAUD_RATE=31250 (320 us/B).
- No per-target compatibility table exists in link_sync.cpp — CLAUDE.md's pointer is dangling (41e313a, re-verified 2026-11-05); KO2/RC-505/MicroKorg facts survive only as the NRPN reset rule and the note-off velocity caveat.
- MIDI file player syncToBpm false (beatAtTime() is tempo-independent); enabling it double-scaled playbackRate, corrupted endBeatAbsolute on tempo change, stuck notes. At loop boundary sendAllNotesOff() + activeNotes.clear() replaces remapping note end times.

## Observing the running mesh

- aloop telemetry: UDP status -> 192.168.137.132:4445, JSON ~150 ms, one hop, costs ESP nothing. Pi is .132, not .100 (FAILED in ip neigh; 4445/4446 time out). Fields xruns, link{synced,bpm,peers,phase_err_beats}, wifi, eff_speed, beat_mark{beat,late_ms}, groove, loopers; ESP bpm 3 dp, aloop 1 dp; beat_mark.late_ms sawtooth, not lateness.

- LRST certified, safe to repeat: printf 'LRST' | nc -u -w1 192.168.4.x 20811 -> peers 0, peerloss, Link peer lost -- sent Stop + All Notes Off., Link peer back after N ms, Link connected; N = 4018 ms (54e3d17; older image ~2540 ms); bpm/beat monotonic, counters never step back = session bounce, not restart.
- Opening the USB console RESETS the board (rst:0x1 (POWERON_RESET), boot:0x13 on each /dev/ttyUSB0 open); a soak opens the port ONCE and never re-opens.
- Pi clock resets to 2024-04-09 every boot (no RTC); final device offline, wall time log-only; don't step it during a certification run (926-day PREEMPT_RT jump perturbs). Pi root is tmpfs: no logs/last reboot/pstore survive; SSH keys regenerate each boot.
- ip_forward on Pi = 0 -> nothing routes to 192.168.4.0/24, route add pointless; enabling exposes the open SSID to ICS — don't unless asked.

## Live state 2026-10-23: two-device mesh (mc-420)

Device 54e3d17 (ELF b65e475f9). Acceptance: retention >=95% after t+60 s from cold boot, bpm spread <=0.05, |phase_err_beats| <=0.03, eff_speed spread <=0.005, gaps <=10%. 3 h run 08:12:36Z-11:12:36Z: 10791 samples, 0 gaps, 100.0% post-acq, zero unexplained events. ICMP ~1% loss is not the metric; the 20813 status reply discriminates IP path from Link state. Host-AP failover certified 4806b18/2a5162f (witness .gm/witness-out/mesh-failover-2a5162f-20261025.txt). WIFI_NO_TEARDOWN_ROLE_SWITCH default 0, never run on a device.

## Phrase-predictability campaign (2026-11-05)

Witnesses run in CI host-witness (build.yml): phrase-predictability-witness.sh/.cpp (guarded MIRROR, parity guard over main/ and vendored Link), phrase_scheduler_fuzz.py (1000 seeds), phrase_telemetry_analyzer.py (self-test), control-model and bass-loop witnesses. 30-min 1 Hz baseline: zero gaps, 56 phrase boundaries, mean spacing 31.999937 s vs 32.0 s. Peer loss after the 8 s force_start latch sets s_pending_realign on 1->0 (fccce9c).


## Bass sequencer: eight-beat loop, pads and dials

main/bass_engine.cpp tick scheduler (12 ticks/sixteenth, 48/beat) driven by Link beat;
main/state_machine.cpp maps pads/dials; main/control_model.{h,cpp} holds 16 settings +
ctl::LoopTrack.

|Pad|Single tap (dial 0 / 1)|Double tap + hold (dial 0 / 1)|
|---|---|---|
|0|scale / width|gate width / arp speed|
|1|pitch offset / 32-step rhythm|random octave / random drop-out|
|2|LFO type / LFO speed (mod wheel CC 1)|LFO width / filter shift (CC 74)|
|3|trance-gate series / octave|loop window / chromatic aberration|

- Loop at most eight beats (2 bars, 32 steps): ctl::kLoopBars=2; window dial reads LIVE value never the track (a recorded window reverted at the very turnaround it should apply, so the loop never shortened); shortened window takes effect at next turnaround of NEW length.
- Single tap selects single settings, starts sequencer; double tap + hold routes dials to double settings. Fill: two pads mark passed steps filled, three mark removed; no rhythm copy, unheld steps follow it; fill/unfill taps don't start sequencer; four pads reset settings/recordings/fill to defaults and stop sequencer.
- Recording (ctl::LoopTrack): dials act only while a pad is held; touch lasts hold, ends on release (kTouchHoldUs=250000 expiry on input clock). startRecording(float) starts a clean line: clears per-step values, current dial value = fallback for unvisited steps; while touched valueAt records live value at every step passed (still positions included). On release the last value is the fallback FOREVER, across all passes (the old "holds until turnaround, then kept/replays" wording is misleading). With two/three pads held both dial settings record for the whole tap; release stops it.

## HTTP clip server (8080) and SPIFFS

main/network_midi.cpp IS in build: network_midi_init() starts esp_http_server on 8080; POST /upload/* -> /spiffs/loops/<name>, GET /info -> {"device_ip":...}, POST /clear deletes *.mid. Invisible at call sites: 4 sockets of 16; SoftAP 8 stations (cfg.ap.max_connection=8 = MAX_AP_STA_IPS), also the forwarder's unicast limit — a 9th associates but gets no Link traffic; WIFI_PS_NONE, don't re-enable. SPIFFS IS mounted (9568578): mount_clip_storage() first, format_if_mount_failed=false. Contract 503 unmounted / 413 over total-used / 400 bad name / 500 incomplete; wildcard one segment re-checked (is_single_clip_name) so / and .. can't escape; /info re-reads IP.

## Wiring

GPIO from main/main.h — a pin can't be expressed by reading the macro.

|Function|Pad / channel|GPIO|
|---|---|---|
|Touch 1|TOUCH_PAD_NUM0|4|
|Touch ARP|TOUCH_PAD_NUM5|12|
|Touch REV|TOUCH_PAD_NUM6|14|
|Touch FILT|TOUCH_PAD_NUM7|27|
|Pot 1|ADC_CHANNEL_6|34|
|Pot 2|ADC_CHANNEL_0|39|
|Buzzer (LEDC)|BUZZER|13|
|Metronome LED, plain (bar downbeat = longer flash)|METRONOME_LED|25|
|MIDI UART2 TX / RX|MIDI_TX_PIN / MIDI_RX_PIN|17 / 16|

Touch pads use the legacy driver/touch_pad.h API (IDF also ships touch_sensor); a pad reads LOW
when touched; ARP read first (lowest press latency). Pots: raw ADC 0-4095 -> MIDI 0-127; player
pot1 = note length (0..2), pot2 = velocity (0..1).

## Constants whose constraint is invisible in the code

- E-Slew must stay derivable from sheer, anchored at 104: reset sets sheer 0, so a derivation not returning 104 at sheer 0 overwrites what setSidechainPattern() just sent..
- Gate wet/dry = depth, not 127 - depth (gateWetDryForDepth()): MiniNova CC 91 is wetness, 127 = full Gator (Manual v1.01).
- kScales[3] = {0,3,5,7,10,3,5}, kScaleLens[3]==5: trailing {3,5} is padding to match neighbours; don't fix.
- kProgs rows are scale-degree indices, not semitones/notes (c0fc974): {0,5,3,6} i-VI-iv-VII, {0,6,5,6} i-VII-VI-VII, {0,3,6,2} i-iv-VII-III, {0,5,6,4} i-VI-VII-V, {0,2,6,3} i-III-VII-iv, {0,0,5,6} pedal.
- main.h asserts LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS: click accent cycle IS quantum — changing one alone breaks build.

## Synth CC/NRPN facts not derivable from the code

- MicroKorg: LFO2 shape CC 75 (kCcLfo2Shape); delay sync CC 13 (kCcDelaySync), 0=off, 1..32.
- MiniNova: LFO2_SYNC_ENABLE NRPN (MSB 1, LSB 40) is an unconfirmed assumption; deactivateFilter() deliberately omits filter bypass (NRPN 0/60 value 0). Chart main/nrpn_chart.txt (3d42c32); main/lfo_constants.h only NRPN 0/86.
- SynthInterface contracts live only in implementations: activateDelay()/activateReverb() = Delay 1/Reverb 1 in FX Slot 1; setFxSlot1Level() CC 91; activateFilter() implicitly LP24; LFO methods assume LFO2 -> Filter1 Freq via Mod Matrix Slot 1.
- LFO depth bipolar -64..+63 -> 0..127, 64 = zero (unpatchLfoFromFilter() writes 64; two = 64 defaults cutoff/patch depth must stay). Note-off velocity 0 default in all three headers — note-on velocity sticks notes on RC-505 MK2/KO2.

## ESP-IDF 6.x breaks absorbed, and comment sweep

IDF 6.x breaks (each re-breaks on a bump): esp_netif_next() -> esp_netif_next_unsafe() (6.1, Link ScanIpIfAddrs.hpp); driver/adc.h removed -> hall_sensor_read() stub main/io_helpers.cpp; CONFIG_LWIP_PBUF_POOL_SIZE dropped. Comment sweep (INVARIANT 3): the 2026-10-11 "0 comments" count (82038c4, 5342086) is stale — new witnesses/headers carry comments; re-measure before quoting a number.

## mc-420 and the sibling trees

mc-420 is the Pi-side peer (aloop); sibling /config/workspace/mc-420 is reachable by SendMessage. ../aloopprime is fetch-only (origin.pushurl = no-push): git_finalize commits, gm refuses push; never edit .git/config. Pi ssh credentials are read-only probing, never commit; the fixed SSH host key stays out of git (owner 2026-10-26). Campaign PRD rows live in gm's PRD store, not here.


## Comment-sweep facts (INVARIANT 4, 2026-11-05)
Verified against current main/; facts formerly carried as code comments.
- g_role_epoch (wifi_config.cpp:73) bumps before role teardown; the relay re-checks it after its esp_netif_tcpip_exec emit and drops work if the role moved. relay_wake_recv() was vestigial and is removed.
- After AP->STA yield the supervisor zeroes igmp_reassert_countdown so the next 2 s tick re-asserts IGMP (else IGMP_REASSERT_PERIOD_TICKS=15 x 2 s = 30 s). wifi_supervisor_task stack 8192 bytes (tie-break runs stop/destroy/start/connect/join inline). Flag-on WIFI_NO_TEARDOWN_ROLE_SWITCH gates console witnesses role: and ap cfg: (docs/netif-redesign-plan.md H3/H2).
- main.cpp INPUT_POLL_TICKS=8 x LINK_TICK_PERIOD 250 us = 2 ms: touch/pot reads every 8 ticks, note step every tick. state_machine.cpp DOUBLE_TAP_WINDOW_US=300 ms; TAP_SETTLE_US=40 ms debounces pad-count changes (flicker would end and re-seed the recorder).
- tools/failover-outage.sh runs ON THE PI as root (mc-420), busybox-safe, default 100 s: autoap stop -> kill hostapd -> hold -> autoap start; EXIT trap with started=0 restores the AP once; ESP judged from its own serial log; outage-only iw scan only between STOP and START.

## gm usage gotchas

- codesearch canonical: exhaustive literal/regex/filename REFUSE unknown fields; scope path/files/glob (not scope); literal tree only -> dual for commit messages; stale_tree = local-only results, no proven zero.
- git_finalize: message/paths/files/allow_whole_index/recover_remote_moved/source_ref/rev in body (or top level, body wins). paths must be inside body (top level -> blanket_stage_refused); never name a gitignored path (build/*.bin, .gm/) — witnesses under .gm/witness/ stay on disk/off commit.
- prd_add/prd_resolve: fields FLAT (nested payload body overflowed pre-0.2.267). prd_resolve REQUIRES bound witness: witness_dispatch_id, OR all four witness_exit_code (0), witness_output_sha256, witness_output_path (project-relative, re-hashed/re-read), witness_ts (RFC 3339); wants resolution, not reason.
- git_show rev/stat/path (paged via offset/next_offset); git_log limit/range/path; bash/exec_js script in body/raw_body; session_id mandatory on every call.


## Transport realign on a Link origin shift, and phase lock (2026-11-08/09)

- Link origin shift (fc5acfa; witness fix 166a2e8; CI run 42597578792 green on build and host-witness; CI bin 69ba16c flashed, Hash verified, MAC e4:65:b8:77:0d:14): handle_link_sync (link_sync.cpp, before detectQuantumBoundary) feeds main/link_origin_guard.h the origin-relative beat beatAtTime(time, 1e9) = beat minus beat origin. It stays continuous under setTempo and clock-offset correction and restarts at any forced beat. A jump above 1/24 beat sets s_pending_realign and resets s_phrase_baseline_captured, so Start or Continue (with SPP) fires at the next phrase boundary, never at the jump. Witness: tools/link-origin-realign-witness.cpp (real Link headers, CI); tools/link-origin-parity.py fails on the pre-fix source (six PARITY FAIL lines).
- DIN evidence, host shift +5.5 beats (tools/link-origin-shift-sim.py): pre-fix only SPP, no FA/FB/FC (.gm/bisect/owner-first-loop-passive/prefix); post-fix SPP and FB together at the first phrase boundary after the shift, about 46 beats later (.../postfix).
- The ESP reads no MIDI: MIDI_RX is configured (io_helpers.cpp:86-87) but never read. Inbound control is Link state plus UDP 20811 LTMP/LRST (link_sync.cpp:225-275). F8/SPP/Start/Continue/Stop are outputs only.
- Beep: one per beat in link_event_cb (s_next_pulse % 24 == 0), in the same callback as the F8 clock byte. Accent by 16-beat cycle (2093/1568/1319/1047 Hz; 20 ms or 1 ms). Audio on Scarlett input index 1 shows one onset per beat (43 in 25 s, median gap 0.4999 s at 120 BPM). Accent pitch and absolute F8 alignment are NOT verified by audio.
- Phase lock: midi-f8-capture.py and link-phase-probe.py --no-midi over the same 60 s; analyse with tools/link-phase-analyse.py (circular centre; LOCKED = stdev < 1 ms and no 5 s centre step > 0.05 pulse). Host beat residual <= 0.3 ms; the F8-to-grid steps are ESP-side, 1-3.5 ms at 174 BPM, mostly ~35 s after a tempo change. Locked in 2 of 11 runs before 4057302 and 0 of 8 after (not significant). Unconfirmed candidate: log_drain (4057302) and link_diag (dc1921e) at priority 2, time-sliced with the Link task every 10 ms; the fix would be priority 1. Not built.
- Stall fix 4057302 holds: HEAD runs show 0 F8 gaps over 20 ms; pre-fix builds show 2-33 gaps, up to 116 ms.
