#include "live_pot_status.h"

#include <cstdio>
#include <string>

int main()
{
    char buf[64];
    int n = live_pot_status_suffix(buf, sizeof buf, 0, 127);
    const std::string nominal(buf, n);
    if (nominal != ",\"pot0\":0,\"pot1\":127}\n") return 1;
    n = live_pot_status_suffix(buf, sizeof buf, -7, 999);
    const std::string clamped(buf, n);
    if (clamped != ",\"pot0\":0,\"pot1\":127}\n") return 1;
    std::printf("LIVE POT STATUS WITNESS: PASS\n");
    return 0;
}
