Phase A / link-peer-mep-relearn:

- main/link_peer_endpoint_guard.h | pure endpoint-change guard | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- main/wifi_config.cpp | incoming mep4 relearn and Link session bounce | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-relearn-witness.cpp | host scenario witness | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-parity.py | parity guard that fails on the old source | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-relearn-witness.sh | host witness runner | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- .github/workflows/build.yml | host-witness step | witness: gh run list --commit <pushed-sha>
- build/link-idf-example.bin | CI build from the pushed SHA | witness: gh run view <build-run-id>
- /dev/ttyUSB0 | one flash and join verification | witness: esptool Hash of data verified, console peers=1, Pi telemetry synced=true

Phase A / link-measurement-timer-pacing:

- components/link-esp/link/include/ableton/platforms/asio/AsioTimer.hpp | steady-clock timer for 250 ms pings | witness: bash tools/link-measurement-timer-witness.sh
- components/link-esp/link/include/ableton/platforms/asio/AsioWrapper.hpp | include steady_timer | witness: bash tools/link-measurement-timer-witness.sh
- components/link-esp/link/include/ableton/discovery/PeerGateway.hpp | steady TimePoint for peer timeouts | witness: bash tools/link-measurement-timer-witness.sh
- tools/link-measurement-timer-witness.cpp | host compile witness | witness: bash tools/link-measurement-timer-witness.sh
- tools/link-measurement-timer-parity.py | parity guard that fails on the old clock | witness: bash tools/link-measurement-timer-witness.sh
- tools/link-measurement-timer-witness.sh | host witness runner | witness: bash tools/link-measurement-timer-witness.sh
- .github/workflows/build.yml | host-witness step | witness: gh run list --commit <pushed-sha>
- /dev/ttyUSB0 | one flash and joined-state verification | witness: esptool Hash of data verified, console peers=1, Pi telemetry synced=true

Phase A / link-measurement-esp-timer:

- components/link-esp/link/include/ableton/platforms/asio/AsioTimer.hpp | ESP esp_timer one-shot for 250 ms pings | witness: bash tools/link-measurement-timer-witness.sh
- tools/link-measurement-timer-parity.py | parity guard that fails on Asio-only timer code | witness: bash tools/link-measurement-timer-witness.sh
- .github/workflows/build.yml | CI build of the ESP timer branch | witness: gh run view <build-run-id>
- /dev/ttyUSB0 | one flash and joined-state verification | witness: esptool Hash of data verified, console peers=1, Pi telemetry synced=true

Phase A / live-pot-status:

- main/live_pot_status.h | pure status suffix formatter | witness: bash tools/live-pot-status-witness.sh
- main/io_helpers.h | live pot declaration | witness: bash tools/live-pot-status-witness.sh
- main/io_helpers.cpp | runtime live pot publish | witness: bash tools/live-pot-status-witness.sh
- main/input_handler.cpp | boot live pot publish | witness: bash tools/live-pot-status-witness.sh
- main/link_sync.cpp | UDP 20813 JSON with pot0/pot1 | witness: bash tools/live-pot-status-witness.sh
- tools/live-pot-status-witness.cpp | host formatting witness | witness: bash tools/live-pot-status-witness.sh
- tools/live-pot-status-parity.py | parity guard that fails on the old source | witness: bash tools/live-pot-status-witness.sh
- tools/live-pot-status-witness.sh | host witness runner | witness: bash tools/live-pot-status-witness.sh
- readme.md | Network ports table updated | witness: bash tools/live-pot-status-witness.sh
- AGENTS.md | status table updated | witness: bash tools/live-pot-status-witness.sh
- .github/workflows/build.yml | host-witness step | witness: gh run list --commit <pushed-sha>
- /dev/ttyUSB0 | one flash and UDP 20813 status verification | witness: esptool Hash of data verified, status JSON pot0/pot1

Phase A / link-measurement-pacing-250ms:

- components/link-esp/link/include/ableton/link/Measurement.hpp | 125 ms retry timer, 250 ms ping cadence, pong-reset retry budget | witness: bash tools/link-measurement-pacing-witness.sh

Phase A / link-measurement-pacing-regression-revert:

- 5fa78fd flashed firmware regressed: 0.286 s/ping vs 0.509 pre-fix, 0.583 s/pong vs 0.524 pre-fix, nodata +93 in 60 s, join ~349.7 s with LRST bounce vs 22.881 s on f748a68; reason = faster ESP pings flood the Pi measurement responder.

Phase A / sidechain-pot-adc-top-clamp:

- main/pot_adc_scale.h | pure ADC-to-MIDI top clamp and rounded scaling | witness: bash tools/pot-adc-max-witness.sh
- main/io_helpers.cpp | steady-state pot mapping uses the shared scale, both pots reach 127 | witness: bash tools/pot-adc-max-witness.sh
- tools/pot-adc-max-witness.cpp | host witness: physical maximum reports 127 and 127 yields zero gate note-ons | witness: bash tools/pot-adc-max-witness.sh
- tools/pot-adc-max-parity.py | parity guard that fails on the old truncating source | witness: bash tools/pot-adc-max-witness.sh
- tools/pot-adc-max-witness.sh | host witness runner | witness: bash tools/pot-adc-max-witness.sh
- .github/workflows/build.yml | host-witness step | witness: gh run list --commit <pushed-sha>
- build/link-idf-example.bin | CI build from the pushed SHA | witness: gh run view <build-run-id>
- /dev/ttyUSB0 | one flash and Pi-side UDP 20813 pot0=127 verification | witness: esptool Hash of data verified, status JSON pot0=127, peers=1
