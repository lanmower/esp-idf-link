#pragma once

#include <cstddef>
#include <cstdio>

inline int live_pot_status_suffix(char* dst, std::size_t capacity, int pot0, int pot1)
{
    if (pot0 < 0) pot0 = 0;
    else if (pot0 > 127) pot0 = 127;
    if (pot1 < 0) pot1 = 0;
    else if (pot1 > 127) pot1 = 127;
    return std::snprintf(dst, capacity, ",\"pot0\":%d,\"pot1\":%d}\n", pot0, pot1);
}
