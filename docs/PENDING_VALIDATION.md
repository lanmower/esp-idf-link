Phase A / link-peer-mep-relearn:

- main/link_peer_endpoint_guard.h | pure endpoint-change guard | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- main/wifi_config.cpp | incoming mep4 relearn and Link session bounce | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-relearn-witness.cpp | host scenario witness | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-parity.py | parity guard that fails on the old source | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- tools/link-peer-endpoint-relearn-witness.sh | host witness runner | witness: bash tools/link-peer-endpoint-relearn-witness.sh
- .github/workflows/build.yml | host-witness step | witness: gh run list --commit <pushed-sha>
- build/link-idf-example.bin | CI build from the pushed SHA | witness: gh run view <build-run-id>
- /dev/ttyUSB0 | one flash and join verification | witness: esptool Hash of data verified, console peers=1, Pi telemetry synced=true
