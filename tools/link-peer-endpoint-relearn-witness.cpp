#include "link_peer_endpoint_guard.h"

#include <cstdio>

namespace
{
int failures = 0;

void check(const char* name, bool ok)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++failures;
}
}

int main()
{
    const uint32_t subnet = 0xC0A80400u;
    const uint32_t mask = 0xFFFFFF00u;
    const uint32_t piIp = 0xC0A80401u;
    const uint32_t otherOnLinkIp = 0xC0A80402u;
    const uint32_t offLinkIp = 0xC0A88984u;

    LinkPeerEndpointWatch watch;
    check("first on-link endpoint is a baseline, not a bounce",
          !watch.observe(piIp, 46321, 1000000, subnet, mask));
    check("same endpoint does not bounce",
          !watch.observe(piIp, 46321, 2000000, subnet, mask));
    check("off-link endpoint is ignored",
          !watch.observe(offLinkIp, 46321, 3000000, subnet, mask));
    check("zero port is ignored",
          !watch.observe(piIp, 0, 4000000, subnet, mask));
    check("restarted peer with a new on-link port bounces after cooldown",
          watch.observe(piIp, 42931, 7000000, subnet, mask));
    check("repeated new endpoint does not bounce again",
          !watch.observe(piIp, 42931, 8000000, subnet, mask));
    check("port flap inside cooldown does not bounce",
          !watch.observe(piIp, 46321, 9000000, subnet, mask));
    check("port change after cooldown bounces again",
          watch.observe(piIp, 46321, 13000000, subnet, mask));
    check("different on-link peer address rebaselines without a bounce",
          !watch.observe(otherOnLinkIp, 46321, 14000000, subnet, mask));
    check("different on-link peer address with new port bounces after cooldown",
          watch.observe(otherOnLinkIp, 41000, 20000000, subnet, mask));

    if (failures == 0) {
        std::printf("LINK PEER ENDPOINT RELEARN WITNESS: PASS (0 failures)\n");
        return 0;
    }
    std::printf("LINK PEER ENDPOINT RELEARN WITNESS: FAIL (%d failures)\n", failures);
    return 1;
}
