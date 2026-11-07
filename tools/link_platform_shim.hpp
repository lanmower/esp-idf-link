#pragma once

#include <arpa/inet.h>
#include <cstdint>
#include <endian.h>

inline const char* const kLinkShimSymbols[] = {"htonll", "ntohll"};
inline constexpr int kLinkShimSymbolCount = 2;

#ifndef htonll
inline std::uint64_t htonll(std::uint64_t value)
{
    return htobe64(value);
}
#endif

#ifndef ntohll
inline std::uint64_t ntohll(std::uint64_t value)
{
    return be64toh(value);
}
#endif
