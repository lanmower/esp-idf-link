#include "link_platform_shim.hpp"

#include <ableton/platforms/asio/AsioTimer.hpp>
#include <ableton/discovery/PeerGateway.hpp>
#include <ableton/discovery/UdpMessenger.hpp>

#include <chrono>
#include <cstdio>
#include <type_traits>

int main()
{
    using Timer = ableton::platforms::asio::AsioTimer;
    static_assert(std::is_same<Timer::TimePoint, std::chrono::steady_clock::time_point>::value,
                  "AsioTimer must use a steady clock for 250 ms measurement pings");
    std::printf("LINK MEASUREMENT TIMER WITNESS: PASS\n");
    return 0;
}
