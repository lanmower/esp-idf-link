# Pre-registered ESP on-device run plans — 2026-11-07

**NOT RUN. These are pre-registered plans only. No plan below is executed until
the owner explicitly approves a device window.** Devices remain fully frozen:
no flash, no serial open, no Pi/ssh, no LTMP/LRST, no power cycle, no device
poll of any kind. Opening `/dev/ttyUSB0` resets the board (rst:0x1
POWERON_RESET), so a window opens the serial console ONCE and never re-opens.
Each plan fixes its pass gate BEFORE the window so the result cannot be
reinterpreted afterwards. Each plan names the row whose acceptance it serves.
These plans are documents; no code, config, firmware image or PRD row was
changed to produce them.

Common prerequisites for any approved window:

- One ESP board attached; record the `esptool MAC:` line on every flash and
  check it against the two known units (`e4:65:b8:77:0d:14` 1st,
  `e4:65:b8:76:cf:c4` 2nd). Which board is attached changes.
- One serial console opened ONCE and kept open for the whole measurement;
  capture to a single append-only log with UTC timestamps.
- No re-flash mid-window. No Pi clock step (the Pi resets to 2024-04-09 on
  boot; wall time is log-only). Pi root is tmpfs, so copy any evidence off
  before reboot.
- Abort on any panic/`Guru Meditation` backtrace; CI publishes no ELF, so a
  panic cannot be symbolised.

---

## Plan 1 — relay-wake: does the wake datagram reach the relay socket in the ESP build?

Row: `relay-wake-recv-vestigial` (open, device-blocked bucket). This plan
measures ONLY the clause the host JIT witness could not settle: whether a
127.0.0.1:20808 datagram actually reaches the relay socket under ESP lwIP.
The removal + stop-within-5 s branch is already witnessed offline
(`.gm/witness-out/relay-wake-jit-witness-20261106.{txt,json}`:
`stop_without_wake.latency_ms = 20.18`, ceiling 5000 ms).

### Procedure (pre-registered)

1. Confirm the flashed image is the current `main` build and that
   `main/wifi_config.cpp` contains no `relay_wake_recv` and no `g_relay_sock`
   (post-`113a4af6` / `92b8c462`). The measurement is of the CURRENT build's
   lwIP loopback behaviour; do not re-add the removed function.
2. From a host on the same `ticker` AP, send ONE 1-byte UDP datagram to
   `127.0.0.1:20808` is NOT a valid probe of the ESP stack (127.0.0.1 is
   loopback on the SENDER, not the ESP). Instead, instrument the ESP build at
   the relay receive socket: add NO new runtime behaviour; use the existing
   `select()`/loop counters already logged, plus a single one-shot UDP socket
   bound to `LINK_DISCOVERY_MULTICAST_PORT` and a self-addressed datagram sent
   to the ESP's OWN AP address (`192.168.4.1`) from the ESP itself, logged as
   `WAKE_PROBE_SENT` / `WAKE_PROBE_RECEIVED`.
3. Trigger exactly one AP->STA role switch (the relay stop path) while the
   serial console stays open. Count `select()` returns caused by the probe
   datagram, and time `wifi_stop_link_relay()` from entry to return.
4. Repeat for 3 independent role switches in the same window; log all three.
   Do not change any sdkconfig between them. Do not enable
   `LWIP_NETIF_LOOPBACK` — the question is whether the shipped configuration
   delivers the loopback wake. If the owner later wants the counterfactual,
   that is a SEPARATE window with a separate pre-registered gate.

### Pass gate (fixed in advance; no post-hoc reinterpretation)

- `WAKE_PROBE_RECEIVED` count = 0 across all 3 switches AND
  `wifi_stop_link_relay()` completes in ≤ 5000 ms (expected ~20 ms) in all 3
  switches, with `g_relay_stop` observed at loop top and after `recvfrom`.
  => PASS: the loopback wake cannot reach the relay socket in this build; the
  removal is correct and the row's "in this build" clause is satisfied by
  measurement rather than inference.
- `WAKE_PROBE_RECEIVED` ≥ 1 OR stop latency > 5000 ms in any switch
  => FAIL: stop and report; do not close the row, do not patch live.

Fixed numeric gate: 0 received, ≤5000 ms, 3/3 switches. Any panic = abort,
not a PASS. Host-only Linux loopback delivery (`host_loopback_delivered: true`
in the JIT witness) is explicitly NOT evidence for this gate.

---

## Plan 2 — mesh-netif: 100 s outage

Row: `mesh-netif-no-teardown` (open, device-blocked). `WIFI_NO_TEARDOWN_ROLE_SWITCH`
defaults to 0 and has never run on a device; compile-verified only.

### Acceptance clauses (verbatim from `.gm/prd/mesh-netif-no-teardown.yml`)

> 100 s outage passes with zero esp_netif_destroy calls, zero panic, exactly one non-zero IP after each switch, Link peer back in the 4018-5387 ms band rather than 113678 ms.

### Procedure (pre-registered)

1. On the Pi (mc-420) as root, run `tools/failover-outage.sh` with the default
   `HOLD=100`: `autoap stop` -> kill `hostapd` -> hold 100 s -> `autoap start`;
   the script's EXIT trap restores the AP once. Outage-only `iw` scan runs only
   between STOP and START. Judge the ESP from its OWN serial log.

2. Serial console opened ONCE before STOP, kept open through recovery.

3. The ESP must be the STA of the `ticker` AP before STOP. Record the ESP's IP
   (`192.168.4.x`, DHCP .2-.20), the AP BSSID, and `peers=1` from status 20813
   BEFORE STOP. Do not poll status 20813 during the outage window unless the
   owner's approved window explicitly includes it; a status poll raises LCLK/
   TTMP broadcasts for 15 s and perturbs the measurement.

4. During the 100 s hold, capture the supervisor's 2 s cadence log: STA
   reconnect attempts (RECONNECT_TRIES=30 ≈ 60 s), IGMP re-assert (30 s),
   tx probe (10 s). Record every `role:`/`ap cfg:` line if the flag is on.

### Observation checklist (all items required, fixed in advance)

- [ ] `esp_netif_destroy` / `esp_netif_destroy_default_wifi` call count = 0
      across the whole window (source check: `tools/flag_reachability.py` must
      print `RESULT: no destroy or wifi-stop call is live with the flag on`,
      and the serial log must show no teardown lines.
- [ ] Panic/`Guru Meditation`/`assert failed` count = 0.
- [ ] Exactly ONE non-zero IP after each role switch; never two, never a
      0-address netif carrying Link (`g_wifi_connected` set only by
      IP_EVENT_STA_GOT_IP; a 0-address netif drops Link and erases the
      gateway, which is a Link SESSION event, NOT an IP-path test).
- [ ] Link peer re-acquired in 4018-5387 ms after AP restore (measured on the
      ESP's own log). 103838/113678 ms (the old 4806b18/2a5162f behaviour) is
      a FAIL.
- [ ] `peers=1` restore; no unexplained `peers=0` after recovery; ESP diag
      peers=1 count monotonic with the log.
- [ ] Host election: lowest MAC/BSSID wins; Pi `e4:5f:01:53:f0:f1` lower than
      the ESP AP MAC, so the ESP yields to the Pi; no AP->STA->AP flap/tick.
      Self-BSSID guard uses `esp_wifi_get_mac(WIFI_IF_AP)`, never `esp_read_mac()`
      eFuse. AP yields only with ZERO associated stations
      (`ap_has_associated_stations()`).
- [ ] IGMP re-joined on yield + AP role; supervisor zeroes
      `igmp_reassert_countdown` so the next 2 s tick re-asserts (else 30 s).
      `RELAY_SELECT_TIMEOUT_US = 20 ms`; `wifi_stop_link_relay()` ceiling
      250 x 20 ms = 5 s.
- [ ] No `esp_wifi_stop` calls; no `WIFI_NO_TEARDOWN_ROLE_SWITCH` flag change
      mid-window.
- [ ] MAC line recorded for the attached board on every flash.

### Pass gate (fixed in advance)

ALL checklist items true in a single 100 s window. Any panic = abort. A single
`esp_netif_destroy`, a second non-zero IP, or a peer-back time outside
4018-5387 ms = FAIL; stop and report, do not close the row. Host certification
witnesses for the old design (`mesh-failover-2a5162f-20261025.txt`) are
reference only, not a pass for this flag configuration.

---

## Plan 3 — phrase boundary guard: backwards rebase emits no boundary

Row: `phrase-cross-tree-quantum` (approved for close after reconnect; this
plan is the on-device check the owner's window would authorise). Firmware fix
landed in `9a8326671014fa1faf026ed1de0b414edafa8d09`: `stepPhraseBoundary()`
in `main/phrase_boundary_guard.h` crosses and advances the last phrase number
only on a strictly greater phrase number. Host witness:
`tools/phrase-boundary-guard-witness.cpp`, 17 checks / 0 failed, drift test
`PHRASE BOUNDARY GUARD DRIFT: PASS -- reverted guard refused, removed call
refused, restored accepted`. `PHRASE_BEATS 64.0`, `LINK_QUANTUM 16.0`,
SPP from `quantumInfo.phraseBeat` (`main/link_sync.cpp:765`), SPP bytes
`0xF2 lsb msb` (14-bit, LSB first). Stop + CC123 x16 on transport stop/peer
loss. SPP-before-Start mandatory at every 64-beat boundary.

### Procedure (pre-registered)

1. Flash the image built from `9a832667` (or later), MAC recorded. One serial
   console open ONCE.
2. Establish a running mesh with `peers=1`, clock enabled, and log at least 3
   consecutive forward 64-beat phrase boundaries as a CONTROL: each must emit
   exactly one `Phrase boundary N, beat ...` log line and one SPP (`0xF2`)
   followed by Start/Continue only when `s_pending_realign` is set.
3. Trigger ONE backwards Link rebase with a 28-byte LTMP tempo+phase request
   (`LTMP` + microsPerBeat + beat0us + quantumUb) to port 20811, choosing a
   `beat0us` that moves `beatAtTime(time, PHRASE_BEATS)` backwards by MORE
   THAN one 64-beat phrase: choose ≥ 96 beats (1.5 phrases), which clears
    the ±quantum/2 phase-encoded tolerance.
   Record the exact chosen `beat0us` and the before/after
   `currentPhraseNumber` values from the log.

4. Continue observing for at least 3 more phrase boundaries after the rebase.

### Thresholds (fixed in advance; no reinterpretation)

- Backwards step: exactly 0 `Phrase boundary ...` log lines and 0 SPP bytes
  attributable to the rebase itself. Any `Phrase boundary` line within the
  rebase window = FAIL. `s_last_phrase_number` must NOT be overwritten to the
  lower value (verify from the following boundary's number: it must be the
  retained pre-rebase number + 1).
- Forward control: 3/3 forward boundaries BEFORE the rebase each emit exactly
  1 boundary line and 1 SPP; forward behaviour unchanged.
- AFTER the rebase, 3/3 later forward boundaries each emit exactly 1 boundary
  line and 1 SPP with monotonically increasing phrase numbers, and SPP units
  = `fmod(phraseBeat * 4, 16384)` with LSB-first bytes; `SPPgrid=1`.
- All MIDI clock F8 pulses remain absolutely scheduled (skip-don't-catch-up);
  no burst of >1 F8 per callback; no cumulative drift.
- Zero panic, zero unexpected `Stop`/CC123 during the backwards step.
- No-start gate: if `s_pending_realign` is false, no Start/Continue at the
  rebase boundary.

### Pass gate (fixed in advance)

All thresholds above true in ONE window. 0 boundary lines / 0 SPP at the
backwards step, 3/3 before and 3/3 after. Any boundary line or SPP at the
backwards step = FAIL; stop and report. A host sim is NOT a substitute for
this on-device check; the row closes only after this window is approved and
the guard's host witness has already landed (it has, in `9a832667`).

---

## Standing statement

**NONE of the three plans above is run until the owner approves a device
window.** No flash, no serial open, no Pi action, no LTMP/LRST, no device poll
of any kind has been performed in producing this document. All gates are
pre-registered; a window that cannot meet a fixed gate is a FAIL, not a reason
to move the gate afterwards.
