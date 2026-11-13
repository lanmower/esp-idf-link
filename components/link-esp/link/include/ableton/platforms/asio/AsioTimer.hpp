#pragma once

#include <ableton/platforms/asio/AsioWrapper.hpp>
#include <ableton/util/SafeAsyncHandler.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#if defined(ESP_PLATFORM)
#include <esp_timer.h>
#include <mutex>
#include <stdexcept>
#endif

namespace ableton
{
namespace platforms
{
namespace asio
{

#if defined(ESP_PLATFORM)
class AsioTimer
{
public:
  using ErrorCode = ::asio::error_code;
  using TimePoint = std::chrono::steady_clock::time_point;

  AsioTimer(::asio::io_service& io)
    : mpImpl(std::make_shared<Impl>(io))
  {
    mpImpl->holder = new std::weak_ptr<Impl>(mpImpl);
    esp_timer_create_args_t args = {};
    args.callback = &AsioTimer::timerCallback;
    args.arg = mpImpl->holder;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "link_measure_timer";
    if (esp_timer_create(&args, &mpImpl->timer) != ESP_OK)
    {
      throw std::runtime_error{"esp_timer_create failed"};
    }
  }

  ~AsioTimer()
  {
    if (mpImpl)
    {
      cancel();
    }
  }

  AsioTimer(const AsioTimer&) = delete;
  AsioTimer& operator=(const AsioTimer&) = delete;

  AsioTimer(AsioTimer&& rhs)
    : mpImpl(std::move(rhs.mpImpl))
  {
  }

  void expires_at(TimePoint tp)
  {
    const auto delta = tp - std::chrono::steady_clock::now();
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();
    std::lock_guard<std::mutex> lock(mpImpl->mutex);
    mpImpl->pendingUs = us > 0 ? static_cast<std::uint64_t>(us) : 1;
  }

  template <typename T>
  void expires_from_now(T duration)
  {
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    std::lock_guard<std::mutex> lock(mpImpl->mutex);
    mpImpl->pendingUs = us > 0 ? static_cast<std::uint64_t>(us) : 1;
  }

  ErrorCode cancel()
  {
    if (!mpImpl)
    {
      return ErrorCode{};
    }
    std::lock_guard<std::mutex> lock(mpImpl->mutex);
    mpImpl->handler = nullptr;
    mpImpl->generation = mpImpl->generation + 1;
    esp_timer_stop(mpImpl->timer);
    return ErrorCode{};
  }

  template <typename Handler>
  void async_wait(Handler handler)
  {
    std::lock_guard<std::mutex> lock(mpImpl->mutex);
    mpImpl->generation = mpImpl->generation + 1;
    mpImpl->handler = [handler](ErrorCode ec) { handler(std::move(ec)); };
    esp_timer_stop(mpImpl->timer);
    esp_timer_start_once(mpImpl->timer, mpImpl->pendingUs);
  }

  TimePoint now() const
  {
    return std::chrono::steady_clock::now();
  }

private:
  struct Impl
  {
    explicit Impl(::asio::io_service& io)
      : service(io)
    {
    }

    ~Impl()
    {
      if (timer)
      {
        esp_timer_stop(timer);
        esp_timer_delete(timer);
      }
      delete holder;
    }

    ::asio::io_service& service;
    esp_timer_handle_t timer = nullptr;
    std::mutex mutex;
    std::function<void(ErrorCode)> handler;
    std::uint64_t pendingUs = 1;
    std::uint64_t generation = 0;
    std::weak_ptr<Impl>* holder = nullptr;
  };

  static void timerCallback(void* arg)
  {
    std::weak_ptr<Impl> weak = *static_cast<std::weak_ptr<Impl>*>(arg);
    std::shared_ptr<Impl> self = weak.lock();
    if (!self)
    {
      return;
    }
    std::function<void(ErrorCode)> handler;
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(self->mutex);
      if (!self->handler)
      {
        return;
      }
      generation = self->generation;
      handler = std::move(self->handler);
      self->handler = nullptr;
    }
    std::weak_ptr<Impl> weakSelf = self;
    self->service.post([weakSelf, generation, handler]() {
      std::shared_ptr<Impl> alive = weakSelf.lock();
      if (!alive)
      {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(alive->mutex);
        if (alive->generation != generation)
        {
          return;
        }
      }
      handler(ErrorCode{});
    });
  }

  std::shared_ptr<Impl> mpImpl;
};
#else
class AsioTimer
{
public:
  using ErrorCode = ::asio::error_code;
  using TimePoint = std::chrono::steady_clock::time_point;

  AsioTimer(::asio::io_service& io)
    : mpTimer(new ::asio::steady_timer(io))
    , mpAsyncHandler(std::make_shared<AsyncHandler>())
  {
  }

  ~AsioTimer()
  {
    if (mpTimer != nullptr)
    {
      cancel();
    }
  }

  AsioTimer(const AsioTimer&) = delete;
  AsioTimer& operator=(const AsioTimer&) = delete;

  AsioTimer(AsioTimer&& rhs)
    : mpTimer(std::move(rhs.mpTimer))
    , mpAsyncHandler(std::move(rhs.mpAsyncHandler))
  {
  }

  void expires_at(TimePoint tp)
  {
    mpTimer->expires_at(std::move(tp));
  }

  template <typename T>
  void expires_from_now(T duration)
  {
    mpTimer->expires_from_now(std::move(duration));
  }

  ErrorCode cancel()
  {
    ErrorCode ec;
    mpTimer->cancel(ec);
    mpAsyncHandler->mpHandler = nullptr;
    return ec;
  }

  template <typename Handler>
  void async_wait(Handler handler)
  {
    *mpAsyncHandler = std::move(handler);
    mpTimer->async_wait(util::makeAsyncSafe(mpAsyncHandler));
  }

  TimePoint now() const
  {
    return std::chrono::steady_clock::now();
  }

private:
  struct AsyncHandler
  {
    template <typename Handler>
    AsyncHandler& operator=(Handler handler)
    {
      mpHandler = [handler](ErrorCode ec) { handler(std::move(ec)); };
      return *this;
    }

    void operator()(ErrorCode ec)
    {
      if (mpHandler)
      {
        mpHandler(std::move(ec));
      }
    }

    std::function<void(const ErrorCode)> mpHandler;
  };

  std::unique_ptr<::asio::steady_timer> mpTimer;
  std::shared_ptr<AsyncHandler> mpAsyncHandler;
};
#endif

} // namespace asio
} // namespace platforms
} // namespace ableton
