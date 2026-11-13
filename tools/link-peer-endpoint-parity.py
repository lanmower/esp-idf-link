#!/usr/bin/env python3
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
src_path = os.path.join(root, "main", "wifi_config.cpp")
header_path = os.path.join(root, "main", "link_peer_endpoint_guard.h")
src = open(src_path, encoding="utf-8").read() if os.path.exists(src_path) else ""
header = open(header_path, encoding="utf-8").read() if os.path.exists(header_path) else ""

failures = []
if '#include "link_peer_endpoint_guard.h"' not in src:
    failures.append("wifi_config.cpp does not include link_peer_endpoint_guard.h")
start = src.find('extern "C" void wifi_link_rx_datagram(')
end = src.find('extern "C" void wifi_link_rx_note(', start)
if start < 0 or end < 0:
    failures.append("wifi_link_rx_datagram or its next function was not found")
    body = ""
else:
    body = src[start:end]
if "static LinkPeerEndpointWatch s_peer_endpoint_watch;" not in body:
    failures.append("incoming Link datagrams do not hold a peer endpoint watch")
if "link_peer_endpoint_on_sta_subnet(mepIp, &subnetHost, &maskHost)" not in body:
    failures.append("incoming Link datagrams do not restrict the watch to the station subnet")
observe_at = body.find("s_peer_endpoint_watch.observe(")
if observe_at < 0:
    failures.append("incoming Link datagrams do not feed the peer endpoint watch")
    block = ""
else:
    block = body[observe_at:observe_at + 700]
if "link_request_session_reset();" not in block:
    failures.append("a changed peer endpoint does not request a Link session bounce")
for token in ("struct LinkPeerEndpointWatch", "bool observe(", "LINK_PEER_ENDPOINT_WATCH_COOLDOWN_US"):
    if token not in header:
        failures.append("link_peer_endpoint_guard.h lacks " + token)

for line in failures:
    print("PARITY FAIL: " + line)
if failures:
    sys.exit(1)
print("LINK PEER ENDPOINT PARITY: PASS")
