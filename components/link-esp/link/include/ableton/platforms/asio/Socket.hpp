/* Copyright 2016, Ableton AG, Berlin. All rights reserved.
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
 *
 *  If you would like to incorporate Link into a proprietary software application,
 *  please contact <link-devs@ableton.com>.
 */

#pragma once

#include <ableton/platforms/asio/AsioWrapper.hpp>
#include <ableton/util/SafeAsyncHandler.hpp>
#include <array>
#include <cassert>
#include <stdexcept>
#include <string>

// Provided by the application (main/wifi_config.cpp). Called for every multicast Link
// discovery datagram so the app can unicast-bridge it across the ESP32 SoftAP boundary,
// which does not carry multicast between the host and its stations. No-op elsewhere.
extern "C" void wifi_link_multicast_forward(const uint8_t* data, unsigned len, unsigned dport, unsigned dstip);
extern "C" void wifi_link_rx_datagram(unsigned srcip, unsigned srcport, unsigned len, const uint8_t* data);
extern "C" void wifi_link_send_dst(unsigned dstip, unsigned port);
extern "C" void wifi_link_send_note(unsigned kind, unsigned err);
#if defined(ESP_PLATFORM)
extern "C" void wifi_link_send_backoff();
#endif

namespace ableton
{
namespace platforms
{
namespace asio
{

template <std::size_t MaxPacketSize>
struct Socket
{
  Socket(::asio::io_service& io)
    : mpImpl(std::make_shared<Impl>(io))
  {
  }

  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;

  Socket(Socket&& rhs)
    : mpImpl(std::move(rhs.mpImpl))
  {
  }

  std::size_t send(const uint8_t* const pData,
    const size_t numBytes,
    const ::asio::ip::udp::endpoint& to)
  {
    assert(numBytes < MaxPacketSize);
    const unsigned dstip = to.address().is_v4() ? to.address().to_v4().to_uint() : 0u;
    wifi_link_send_dst(dstip, to.port());

    const unsigned kSendAttempts = 3;
    std::size_t sent = 0;
    for (unsigned attempt = 0; attempt < kSendAttempts; ++attempt)
    {
      ::asio::error_code ec;
      sent = mpImpl->mSocket.send_to(::asio::buffer(pData, numBytes), 0, to, ec);
      if (!ec)
      {
        wifi_link_send_note(attempt == 0 ? 0u : 1u, 0u);
        break;
      }
      wifi_link_send_note(2u, static_cast<unsigned>(ec.value()));
      if (attempt + 1 == kSendAttempts)
      {
        wifi_link_send_note(3u, static_cast<unsigned>(ec.value()));
        throw std::runtime_error{"send_to: " + ec.message()};
      }
#if defined(ESP_PLATFORM)
      wifi_link_send_backoff();
#endif
    }

    wifi_link_multicast_forward(pData, static_cast<unsigned>(numBytes), to.port(), dstip);
    return sent;
  }

  template <typename Handler>
  void receive(Handler handler)
  {
    mpImpl->mHandler = std::move(handler);
    mpImpl->mSocket.async_receive_from(
      ::asio::buffer(mpImpl->mReceiveBuffer, MaxPacketSize), mpImpl->mSenderEndpoint,
      util::makeAsyncSafe(mpImpl));
  }

  ::asio::ip::udp::endpoint endpoint() const
  {
    return mpImpl->mSocket.local_endpoint();
  }

  struct Impl
  {
    Impl(::asio::io_service& io)
      : mSocket(io, ::asio::ip::udp::v4())
    {
    }

    ~Impl()
    {
      // Ignore error codes in shutdown and close as the socket may
      // have already been forcibly closed
      ::asio::error_code ec;
      mSocket.shutdown(::asio::ip::udp::socket::shutdown_both, ec);
      mSocket.close(ec);
    }

    void operator()(const ::asio::error_code& error, const std::size_t numBytes)
    {
      if (!error && numBytes > 0 && numBytes <= MaxPacketSize)
      {
        const auto bufBegin = begin(mReceiveBuffer);
        wifi_link_rx_datagram(
          mSenderEndpoint.address().is_v4() ? mSenderEndpoint.address().to_v4().to_uint() : 0u,
          mSenderEndpoint.port(), static_cast<unsigned>(numBytes), mReceiveBuffer.data());
        mHandler(mSenderEndpoint, bufBegin, bufBegin + static_cast<ptrdiff_t>(numBytes));
      }
    }

    ::asio::ip::udp::socket mSocket;
    ::asio::ip::udp::endpoint mSenderEndpoint;
    using Buffer = std::array<uint8_t, MaxPacketSize>;
    Buffer mReceiveBuffer;
    using ByteIt = typename Buffer::const_iterator;
    std::function<void(const ::asio::ip::udp::endpoint&, ByteIt, ByteIt)> mHandler;
  };

  std::shared_ptr<Impl> mpImpl;
};

} // namespace asio
} // namespace platforms
} // namespace ableton
