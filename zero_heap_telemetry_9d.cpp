#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

struct alignas(64) TelemetryPayload {
    uint64_t timestamp;
    double sensor_data[4];
    uint32_t status_flags;
    uint32_t reserved;
};

static_assert(sizeof(TelemetryPayload) == 64, "TelemetryPayload mora biti tacno 64 bajta");
static_assert(alignof(TelemetryPayload) == 64, "TelemetryPayload mora biti poravnat na 64 bajta");

class DeterministicTelemetrySystem {
private:
    static constexpr std::size_t BUFFER_SIZE = 1024;
    static_assert((BUFFER_SIZE & (BUFFER_SIZE - 1)) == 0, "BUFFER_SIZE mora biti stepen dvojke");

    alignas(64) TelemetryPayload ring_buffer[BUFFER_SIZE];
    alignas(64) std::atomic<std::size_t> head{0};

public:
    DeterministicTelemetrySystem() noexcept {
        for (std::size_t i = 0; i < BUFFER_SIZE; ++i) {
            ring_buffer[i] = {};
        }
    }

    void update_cycle(const double (&raw_sensors)[4], uint32_t status) noexcept {
        const std::size_t idx = head.load(std::memory_order_relaxed);
        TelemetryPayload& current = ring_buffer[idx];

        current.timestamp = read_cpu_cycle_counter();
        current.sensor_data[0] = raw_sensors[0];
        current.sensor_data[1] = raw_sensors[1];
        current.sensor_data[2] = raw_sensors[2];
        current.sensor_data[3] = raw_sensors[3];
        current.status_flags = status;
        current.reserved = 0u;

        head.store((idx + 1) & (BUFFER_SIZE - 1), std::memory_order_release);
    }

    uint64_t now_cycles() const noexcept {
        return read_cpu_cycle_counter();
    }

    static constexpr std::size_t capacity() noexcept { return BUFFER_SIZE; }

private:
    inline uint64_t read_cpu_cycle_counter() const noexcept {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
        uint32_t lo = 0, hi = 0;
        asm volatile("rdtsc" : "=a"(lo), "=d"(hi) : : "memory");
        return (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(lo);
#else
        return 0;
#endif
    }
};

int main() {
    std::cout << "GCC: " << __GNUC__ << "." << __GNUC_MINOR__
#if defined(__x86_64__)
              << " | arch: x86_64"
#elif defined(__i386__)
              << " | arch: i386"
#else
              << " | arch: other"
#endif
              << "\n";

    constexpr std::size_t ITERATIONS = 1000000;
    DeterministicTelemetrySystem sys;
    double sensors[4] = {1.25, 2.50, 3.75, 5.00};

    uint64_t min_delta = (std::numeric_limits<uint64_t>::max)();
    uint64_t max_delta = 0;
    uint64_t sum_delta = 0;
    uint64_t first_t0 = 0, first_t1 = 0, last_t0 = 0, last_t1 = 0;

    const auto c_start = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < ITERATIONS; ++i) {
        sensors[0] += 0.0001;

        const uint64_t t0 = sys.now_cycles();
        sys.update_cycle(sensors, static_cast<uint32_t>(i));
        const uint64_t t1 = sys.now_cycles();

        if (i == 0) {
            first_t0 = t0;
            first_t1 = t1;
        }
        if (i == ITERATIONS - 1) {
            last_t0 = t0;
            last_t1 = t1;
        }

        if (t1 >= t0) {
            const uint64_t delta = t1 - t0;
            if (delta < min_delta) min_delta = delta;
            if (delta > max_delta) max_delta = delta;
            sum_delta += delta;
        }
    }

    const auto c_end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(c_end - c_start).count();
    const double avg_delta = static_cast<double>(sum_delta) / static_cast<double>(ITERATIONS);

    std::cout << "Elapsed ms: " << elapsed_ms << "\n"
              << "First t0/t1: " << first_t0 << " / " << first_t1 << "\n"
              << "Last t0/t1: " << last_t0 << " / " << last_t1 << "\n"
              << "Min: " << min_delta << " | Avg: " << avg_delta << " | Max: " << max_delta << "\n";

    return 0;
}
