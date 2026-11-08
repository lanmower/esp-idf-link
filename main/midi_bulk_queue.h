#pragma once

#include <cstddef>
#include <cstdint>

template <std::size_t Capacity>
class MidiBulkQueue
{
public:
    bool push(const uint8_t* message, std::size_t size)
    {
        if (size > Capacity - count_) {
            return false;
        }
        for (std::size_t i = 0; i < size; ++i) {
            buffer_[(head_ + count_) % Capacity] = message[i];
            ++count_;
        }
        return true;
    }

    bool pop(uint8_t& out)
    {
        if (count_ == 0) {
            return false;
        }
        out = buffer_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    std::size_t size() const { return count_; }

private:
    uint8_t buffer_[Capacity] = {};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

class MidiBulkPacer
{
public:
    explicit MidiBulkPacer(int ticksPerByte) : ticksPerByte_(ticksPerByte) {}

    bool onTick()
    {
        if (++phase_ < ticksPerByte_) {
            return false;
        }
        phase_ = 0;
        return true;
    }

private:
    int ticksPerByte_;
    int phase_ = 0;
};
