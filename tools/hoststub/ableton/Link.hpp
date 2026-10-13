// Host stub of ableton/Link.hpp: a linear-tempo SessionState, which is all the bass engine reads.
#pragma once
#include <chrono>
#include <cmath>
namespace ableton {
class Link {
public:
    class SessionState {
    public:
        double tempo() const { return m_bpm; }
        double beatAtTime(const std::chrono::microseconds& t, double) const {
            return m_beat0 + (t.count() - m_t0) / 1e6 * m_bpm / 60.0;
        }
        double phaseAtTime(const std::chrono::microseconds& t, double q) const {
            double b = beatAtTime(t, q);
            return b - std::floor(b / q) * q;
        }
        std::chrono::microseconds timeAtBeat(double beat, double) const {
            return std::chrono::microseconds((long long)(m_t0 + (beat - m_beat0) * 60.0 / m_bpm * 1e6));
        }
        bool isPlaying() const { return false; }
        double    m_bpm   = 145.0;
        double    m_beat0 = 0.0;
        long long m_t0    = 0;
    };
};
}
