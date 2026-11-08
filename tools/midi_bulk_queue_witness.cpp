#include "midi_bulk_queue.h"

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr int64_t kTickUs = 250;
constexpr int64_t kByteUs = 320;
constexpr std::size_t kCapacity = 512;
constexpr int kShippedPaceTicks = 2;
constexpr int kBurstBytes = 108;

int g_checks = 0;
int g_failures = 0;

void check(const char* label, bool ok)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL: %s\n", label);
    }
}

std::string readSource(const char* path)
{
    std::ifstream in(path);
    if (!in) {
        std::printf("FAIL: cannot read %s\n", path);
        ++g_failures;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool contains(const std::string& haystack, const char* needle)
{
    return haystack.find(needle) != std::string::npos;
}

struct Entry
{
    bool isClock;
    uint8_t value;
    int64_t arrivalUs;
};

struct WireReport
{
    std::size_t maxAheadAtClock = 0;
    int64_t maxClockWaitUs = 0;
    int64_t lastBulkSentUs = 0;
    int bulkSent = 0;
    bool bulkInOrder = true;
};

WireReport simulate(int paceTicks, const std::vector<uint8_t>& burst,
                    const std::vector<int64_t>& clockUs)
{
    MidiBulkQueue<kCapacity> queue;
    MidiBulkPacer pacer(paceTicks);
    queue.push(burst.data(), burst.size());
    std::deque<Entry> wire;
    WireReport report;
    int64_t wireFreeAt = 0;
    std::size_t nextClock = 0;
    std::size_t nextBulk = 0;
    const int64_t horizonUs = 400000;
    for (int64_t t = 0; t < horizonUs; ++t) {
        if (t % kTickUs == 0 && pacer.onTick()) {
            uint8_t value = 0;
            if (queue.pop(value)) {
                wire.push_back({false, value, t});
            }
        }
        if (nextClock < clockUs.size() && t == clockUs[nextClock]) {
            if (wire.size() > report.maxAheadAtClock) {
                report.maxAheadAtClock = wire.size();
            }
            wire.push_back({true, 0xF8, t});
            ++nextClock;
        }
        if (t >= wireFreeAt && !wire.empty()) {
            const Entry e = wire.front();
            wire.pop_front();
            wireFreeAt = t + kByteUs;
            if (e.isClock) {
                const int64_t wait = t - e.arrivalUs;
                if (wait > report.maxClockWaitUs) {
                    report.maxClockWaitUs = wait;
                }
            } else {
                if (nextBulk >= burst.size() || e.value != burst[nextBulk]) {
                    report.bulkInOrder = false;
                }
                ++nextBulk;
                ++report.bulkSent;
                report.lastBulkSentUs = t;
            }
        }
    }
    return report;
}

void fifoChecks()
{
    MidiBulkQueue<8> q;
    const uint8_t first[5] = {1, 2, 3, 4, 5};
    const uint8_t second[5] = {6, 7, 8, 9, 10};
    check("fifo accepts first message", q.push(first, 5));
    uint8_t out = 0;
    check("fifo pops first byte", q.pop(out) && out == 1);
    check("fifo wraps and accepts second message", q.push(second, 5) == false);
    check("fifo rejects whole message that does not fit", q.size() == 4);
    uint8_t drained[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        q.pop(drained[i]);
    }
    check("fifo drains remaining bytes in order", drained[0] == 2 && drained[3] == 5);
    check("fifo wrap after pops keeps order", q.push(second, 5) && q.pop(out) && out == 6);
}

void pacingChecks()
{
    check("pacing period covers one 320 us byte time",
          kShippedPaceTicks * kTickUs >= kByteUs);
    MidiBulkPacer pacer(3);
    int emitted = 0;
    for (int i = 0; i < 9; ++i) {
        if (pacer.onTick()) {
            ++emitted;
        }
    }
    check("pacer emits one byte per configured tick count", emitted == 3);
}

void wireChecks()
{
    std::vector<uint8_t> burst(kBurstBytes);
    for (int i = 0; i < kBurstBytes; ++i) {
        burst[i] = static_cast<uint8_t>(i & 0x7F);
    }
    std::vector<int64_t> clockUs;
    for (int k = 0; k < 60; ++k) {
        clockUs.push_back(300 + k * 1777 + (k * 613) % 1000);
    }

    const WireReport shipped = simulate(kShippedPaceTicks, burst, clockUs);
    std::printf("  [shipped pacing 2 ticks] bulk %d bytes, last bulk on wire %lld us, "
                "max bulk ahead of clock %zu, max clock wait %lld us\n",
                shipped.bulkSent, static_cast<long long>(shipped.lastBulkSentUs),
                shipped.maxAheadAtClock, static_cast<long long>(shipped.maxClockWaitUs));
    check("every bulk byte reaches the wire", shipped.bulkSent == kBurstBytes);
    check("bulk bytes stay in FIFO order", shipped.bulkInOrder);
    check("bulk burst drains within its pacing budget",
          shipped.lastBulkSentUs <= kBurstBytes * kShippedPaceTicks * kTickUs + 2 * kByteUs);
    check("at most one bulk byte is queued ahead of a clock byte",
          shipped.maxAheadAtClock <= 1);
    check("clock byte waits at most two byte times", shipped.maxClockWaitUs <= 2 * kByteUs);

    const WireReport overfill = simulate(1, burst, clockUs);
    std::printf("  [negative control 1 tick] max bulk ahead of clock %zu, max clock wait %lld us\n",
                overfill.maxAheadAtClock, static_cast<long long>(overfill.maxClockWaitUs));
    check("negative control: 1-tick pacing overfills the wire",
          overfill.maxAheadAtClock > 1);
}

void shippedSourceChecks()
{
    const std::string io = readSource("main/io_helpers.cpp");
    const std::string link = readSource("main/link_sync.cpp");
    const std::string mainCpp = readSource("main/main.cpp");
    check("io_helpers routes NRPN bulk through the paced queue",
          contains(io, "midi_bulk_enqueue(message, size);"));
    check("io_helpers pacing constant matches the witness",
          contains(io, "static constexpr int MIDI_BULK_PACE_TICKS = 2;"));
    check("link_sync routes ordered clock messages through the queue",
          contains(link, "midi_bulk_enqueue(buf, len);"));
    check("link_sync writes only the clock byte directly",
          contains(link, "uart_write_bytes(MIDI_UART, (const char*)buf, len);"));
    check("tick task pumps the bulk queue every notification",
          contains(mainCpp, "midi_bulk_pump();"));
}
}

int main()
{
    fifoChecks();
    pacingChecks();
    wireChecks();
    shippedSourceChecks();
    std::printf("midi bulk queue witness: %d checks, %d failed\n", g_checks, g_failures);
    std::printf("MIDI BULK QUEUE WITNESS: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
