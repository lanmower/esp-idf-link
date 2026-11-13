#pragma once

#include <cstdint>

constexpr int64_t LINK_PEER_ENDPOINT_WATCH_COOLDOWN_US = 5000000;

struct LinkPeerEndpointWatch {
    bool valid = false;
    uint32_t lastIp = 0;
    uint16_t lastPort = 0;
    int64_t lastChangeUs = 0;

    bool observe(uint32_t ip, uint16_t port, int64_t nowUs, uint32_t subnet, uint32_t mask)
    {
        if (ip == 0 || port == 0 || mask == 0 || (ip & mask) != subnet) return false;
        if (!valid) {
            valid = true;
            lastIp = ip;
            lastPort = port;
            lastChangeUs = nowUs;
            return false;
        }
        if (ip != lastIp) {
            lastIp = ip;
            lastPort = port;
            lastChangeUs = nowUs;
            return false;
        }
        if (port == lastPort) return false;
        if (nowUs - lastChangeUs < LINK_PEER_ENDPOINT_WATCH_COOLDOWN_US) return false;
        lastPort = port;
        lastChangeUs = nowUs;
        return true;
    }
};
