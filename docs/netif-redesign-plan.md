# Flag-on netif role switch: plan, pre-registration and witness design

Row: `mesh-netif-no-teardown`. Status: **planned, not scheduled.** The board stays on `2a5162f` (the
certified image, failover certified 2026-10-25) until this plan is agreed with mc-420 and this file is
committed. This file is the pre-registration: the criteria below are fixed before any run, and the commit
that adds it is the record of that.

## 1. What the change is, and what it is not

`WIFI_NO_TEARDOWN_ROLE_SWITCH` (`main/wifi_config.h:12`) is `0` on `main`, so the running switch is the
certified destroy-based one. The flag is the only difference between the two images. Setting it to `1`
is a one-line commit; that commit is the only change in the candidate image. Any other difference
invalidates the comparison.

With the flag on, the switch never calls `esp_netif_destroy*` or `esp_wifi_stop`:

- Both netifs are created once in `wifi_config_init` (`main/wifi_config.cpp:218-232`), and the mode is set to
  `WIFI_MODE_APSTA` once.
- `role_enforce` (`:200-207`) parks the out-of-role netif (`role_park_netif`, `:183`: `dhcps_stop`,
  `dhcpc_stop`, zero IP, `netif_set_down` on the tcpip thread) and activates the in-role one
  (`role_activate_netif`, `:193`: `netif_set_up`, `dhcpc_start` for STA, `esp_netif_set_default_netif`).
- Every `esp_netif_destroy*` and `esp_wifi_stop` call in the switch sits inside an `#else` of the flag. The
  destroy calls are at `:1237` and `:1306`. A static check of the flag-on branch must find none reachable.

## 2. Hypotheses, stated so a run can refute them

- **H1 (convergence).** A 100 s host-AP outage with the flag on re-hosts, yields to the lower BSSID and
  rejoins, with no panic and no deferral, as the flag-off image does.
- **H2 (the AP beacons on a running APSTA).** `wifi_start_link_ap` (`:352`) reconfigures the AP with
  `esp_wifi_set_config(WIFI_IF_AP, ...)` (`:381`) while Wi-Fi is running and never restarts it. AGENTS.md
  lists this as unverified. H2 holds if the Pi sees the ESP's `ticker` during hosting.
- **H3 (the parked AP is silent).** While the ESP is a station, its AP netif is parked: no DHCP, no IP, and
  not up. H3 holds if the Pi sees no `ticker` at the ESP's AP BSSID while the ESP is a station.
- **H4 (default netif).** After each switch the default netif is the in-role one, and the Link sends and
  the HTTP server use it. Observed indirectly: Link sends have `fail=0 errno=0` after rejoin, and the Link
  peer returns.

## 3. Pass criteria (pre-registered numbers)

A run passes only if every line holds. Any failure is recorded as a failure, and no retuning is done inside
the same window.

| # | Criterion | Witness |
|---|---|---|
| P1 | No `Guru Meditation`, no `SW_CPU_RESET`, no `relay did not stop`, no `deferring` | ESP console |
| P2 | Re-host logged: `STA down past 30 tries`, then `AP started`, then DHCP on 192.168.4.1 | ESP console |
| P3 | Yield logged: `Lost dual-host tie-break`, then `connected with ticker` and `IP:` within 3 s | ESP console |
| P4 | `Link peer back` within **10 s of the `IP:` line** after the yield | ESP console |
| P5 | After the outage: Pi `hostapd=1 ssid=ticker type=AP stations=1` at AFTER+60 | Pi `outage.out` |
| P6 | Telemetry back to `peers=1` within 15 s of the rejoin, `phase_err ≤ 0.03` | 1 Hz telemetry |
| P7 | H2: during hosting, the Pi scan (every 10 s) lists `ticker` at the ESP's AP BSSID (STA MAC + 1, verified by the scan) | Pi scan log |
| P8 | H3: during the STA role, the Pi scan lists no `ticker` at the ESP's AP BSSID | Pi scan log |
| P9 | Static: every `esp_netif_destroy*` and `esp_wifi_stop` call in the switch is inside `#else` of the flag | source (`codesearch`), at the candidate commit |
| P10 | `sendfail` lines occur only in the outage transitions (down and the 2 s around rejoin) | ESP console |

**Steady-state gate, before any outage (P0).** The flag-on supervisor parks the out-of-role netif on every
2 s tick (`:1205`, `:1247`), which is new load. A 30 min soak with the same criteria as the 2a5162f soak must
pass first: peer retention ≥ 95%, zero drops, bpm spread ≤ 0.05, max phase error ≤ 0.03, eff spread ≤
0.005, sample gaps ≤ 10%, zero `peerloss`, zero panic.

## 4. Unknowns the run must answer, not assume

1. **H2**, whether reconfiguring a running APSTA AP applies. Answered only by P7.
2. **H3**, whether the parked AP stays silent. Answered only by P8.
3. **Default netif after a switch.** No direct witness exists. P4 and the send counters give indirect
   evidence. If P4 fails, the default netif is the first suspect.
4. **Per-tick parking cost.** P0 answers it. If retention drops in the soak, the change is rejected
   regardless of the outage result.

## 5. Procedure

1. Commit the flag change alone (`#define WIFI_NO_TEARDOWN_ROLE_SWITCH 1`). CI builds it. Record the CI run
   id and the sha256 of the binary.
2. Flash with the MAC check: confirm the MAC printed by `esptool` is `e4:65:b8:77:0d:14` (the board on the
   USB port) before writing.
3. One console open, held for the whole run. Record the compile stamp.
4. P0 soak, 30 min.
5. Only if P0 passes: one 100 s outage, fired by the mc-420 launcher at a second that mc-420 names and that
   I acknowledge, with the TERM-timeout trap. The Pi scan runs every 10 s from BEFORE through AFTER+60.
6. Evaluate P1 to P10 against the table. Record every number.
7. A single clean outage makes the flag-on image a **candidate only**. Accepting it as the default needs a
   second outage on a fresh boot, at least an hour later, and a 3 h soak. Each of those is a separate
   decision and needs mc-420's agreement.

## 6. Failure policy and rollback

- Any P1 failure, or P0 failure, ends the run. The board is flashed back to the certified image from the CI
  binary commit that followed `2a5162f` (`227125e` for the binary). Verify with `git show`, then record its
  sha256 before flashing.
- A failure is never retuned in the same window. A changed image is a new row, with a new pre-registration.
- The row stays open until a clean run, and the failure record stays in the repo.

## 7. Known risks to watch for

- `esp_wifi_set_config(WIFI_IF_AP, ...)` on a running APSTA (H2). If it does not apply, the AP does not
  beacon after the re-host, and P7 fails.
- The yield on the flag-on path calls `ensure_sta_started()` (`:1291`, `:1310`) after `g_ap_active = false`.
  If `role_enforce(g_sta_netif)` runs before the relay has fully stopped, a relay send can still reach the
  AP netif. The defer at `:1285` guards the stop, not the order.
- `esp_netif_set_default_netif` is set on `IP_EVENT_STA_GOT_IP` (`:129-134`) and in `role_activate_netif`.
  A `STA_LOST_IP` clears it (AGENTS.md). A down event between the two could leave the default on the parked
  netif. P4 and the send counters would show it.
- The `mc-420` side keeps polling 20813 every 5 s. The flag-on code changes the status responder's
  subscriber accounting only through `note_telemetry_subscriber`, which is unchanged. Confirm that in the P0
  soak.

## 8. What mc-420 needs to agree to before any flash

- This plan (this file, at the commit that adds it).
- The P0 soak and one outage with the criteria in section 3, at a second mc-420 names.
- The rollback in section 6, including who flashes it back and the commit it uses.
