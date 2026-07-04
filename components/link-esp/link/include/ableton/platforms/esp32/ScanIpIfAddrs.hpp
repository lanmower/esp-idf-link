/* Copyright 2020, Ableton AG, Berlin and 2019, Mathias Bredholt, Torso Electronics,
 * Copenhagen. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <ableton/platforms/asio/AsioWrapper.hpp>
#include <arpa/inet.h>
#include <esp_netif.h>
#include <net/if.h>
#include <vector>
#include <cstdint>

namespace ableton
{
namespace platforms
{
namespace esp32
{

// ESP32 implementation of ip interface address scanner
struct ScanIpIfAddrs
{
  std::vector<::asio::ip::address> operator()()
  {
    extern volatile std::uint32_t g_link_scan_calls;
    extern volatile std::uint32_t g_link_scan_last_ip;
    extern volatile std::uint32_t g_link_scan_last_count;
    std::vector<::asio::ip::address> addrs;
    // Get first network interface
    esp_netif_t* esp_netif = esp_netif_next_unsafe(NULL);
    while (esp_netif)
    {
      // Check if interface is active
      if (esp_netif_is_netif_up(esp_netif))
      {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(esp_netif, &ip_info);
        // Skip interfaces without a valid IP (0.0.0.0) -- Link cannot bind/broadcast on them
        if (ip_info.ip.addr != 0)
        {
          addrs.emplace_back(::asio::ip::address_v4(ntohl(ip_info.ip.addr)));
          g_link_scan_last_ip = ip_info.ip.addr;
        }
      }
      // Get next network interface
      esp_netif = esp_netif_next_unsafe(esp_netif);
    }
    g_link_scan_calls = g_link_scan_calls + 1;
    g_link_scan_last_count = (std::uint32_t)addrs.size();
    return addrs;
  }
};

} // namespace esp32
} // namespace platforms
} // namespace ableton
