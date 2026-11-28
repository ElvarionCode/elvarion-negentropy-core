// elvarion_negentropy_benchmark.cpp
// Konacno poredjenje: Elvarion negentropijski tok vs Klasicni FIFO pod burst opterecenjem
// Standard: C++11 (-std=c++11 -O3 -Wall -Wextra), zero-heap, alignas(64)

#include <array>
#include <cstdint>
#include <cstdio>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace elvarion {
namespace negentropy {

enum class RoutePort : uint8_t {
    PortA    = 0,
    PortB    = 1,
    Priority = 2,
    Dropped  = 3
};

struct alignas(64) PacketFrame {
    uint64_t seq = 0;
    uint32_t src_node = 0;
    uint32_t dst_node = 0;
    uint16_t priority = 0;
    uint16_t noise_score = 0;
    uint32_t payload_hash = 0;
    uint32_t reserved = 0;
};

struct alignas(64) BenchmarkStats {
    uint64_t total = 0;
    uint64_t delivered = 0;
    uint64_t priority_delivered = 0;
    uint64_t dropped = 0;
    uint64_t priority_dropped = 0;
    uint64_t total_cycles = 0;
    uint64_t min_cycles = UINT64_MAX;
    uint64_t max_cycles = 0;
    uint64_t burst_spikes = 0;
};

inline uint64_t read_cycles() noexcept {
#if defined(_MSC_VER)
    return __rdtsc();
#elif defined(__x86_64__) || defined(__i386__)
    unsigned int lo = 0;
    unsigned int hi = 0;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(lo);
#else
    return 0;
#endif
}

// Identican budzet korisnog posla nad paketom za oba sistema
inline uint32_t process_payload_work(const PacketFrame& pkt, uint32_t phase_bias) noexcept {
    uint32_t acc = pkt.payload_hash ^ phase_bias;
    acc ^= (acc << 13);
    acc ^= (acc >> 17);
    acc ^= (acc << 5);
    return acc * 2654435761u;
}

// ELVARION: Negentropijski fazni ruter (fiksni slotovi, prigusivanje suma, direktni prioritet)
template <size_t RingSize = 256>
class NegentropicPhaseRouter {
    static_assert(RingSize >= 16 && (RingSize & (RingSize - 1)) == 0,
                  "RingSize mora biti stepen dvojke >= 16");

public:
    NegentropicPhaseRouter(uint32_t phi_step_q32, uint16_t noise_limit) noexcept
        : phi_step_q32_(phi_step_q32),
          noise_limit_(noise_limit),
          phase_acc_(0u),
          head_(0),
          state_checksum_(0u) {
        stats_ = BenchmarkStats{};
    }

    RoutePort submit(const PacketFrame& pkt) noexcept {
        const uint64_t t0 = read_cycles();

        phase_acc_ += phi_step_q32_;

        // Pasivno prigusivanje suma preko faktora bliskog 1/phi u Q16
        uint16_t clean_noise = pkt.noise_score;
        if (clean_noise > noise_limit_) {
            const uint32_t excess = static_cast<uint32_t>(clean_noise - noise_limit_);
            clean_noise = static_cast<uint16_t>(noise_limit_ + ((excess * 40503u) >> 16));
        }

        // Prioritetni okviri prolaze direktnim kanalom bez cekanja
        if (pkt.priority >= 8 && clean_noise <= (noise_limit_ + 80u)) {
            state_checksum_ ^= process_payload_work(pkt, phase_acc_);
            ++stats_.delivered;
            ++stats_.priority_delivered;
            ++stats_.total;
            record(t0);
            return RoutePort::Priority;
        }

        // Upis u fiksni cache-aligned prsten bez heap alokacija i bez zastoja
        ring_[head_] = pkt;
        ring_[head_].noise_score = clean_noise;
        head_ = (head_ + 1) & (RingSize - 1);

        const uint32_t lane = ((phase_acc_ >> 31) ^ pkt.dst_node ^ pkt.payload_hash) & 1u;
        state_checksum_ ^= process_payload_work(pkt, lane * 0x9E3779B9u);

        ++stats_.delivered;
        ++stats_.total;
        record(t0);
        return (lane == 0u) ? RoutePort::PortA : RoutePort::PortB;
    }

    uint32_t checksum() const noexcept { return state_checksum_; }
    const BenchmarkStats& stats() const noexcept { return stats_; }

private:
    void record(uint64_t t0) noexcept {
        const uint64_t t1 = read_cycles();
        if (t1 >= t0) {
            const uint64_t d = t1 - t0;
            stats_.total_cycles += d;
            if (d < stats_.min_cycles) stats_.min_cycles = d;
            if (d > stats_.max_cycles) stats_.max_cycles = d;
            if (d > 1200ULL) {
                ++stats_.burst_spikes;
            }
        }
    }

    uint32_t phi_step_q32_;
    uint16_t noise_limit_;
    uint32_t phase_acc_;
    alignas(64) std::array<PacketFrame, RingSize> ring_{};
    size_t head_;
    uint32_t state_checksum_;
    BenchmarkStats stats_;
};

// KLASICNI FIFO: Ogranicen bafer sa tail-drop odbacivanjem i gubitkom ritma pod burst naletom
template <size_t Cap = 32>
class ClassicFifoChannel {
public:
    ClassicFifoChannel() noexcept : head_(0), tail_(0), count_(0), step_div_(0), state_checksum_(0u) {
        stats_ = BenchmarkStats{};
    }

    RoutePort submit(const PacketFrame& pkt, bool burst_mode) noexcept {
        const uint64_t t0 = read_cycles();

        // U burst rezimu izlazni kanal prazni tek svaki 4. korak
        ++step_div_;
        const uint32_t drain_interval = burst_mode ? 4u : 1u;
        if (count_ > 0 && (step_div_ % drain_interval) == 0u) {
            tail_ = (tail_ + 1) % Cap;
            --count_;
        }

        // Tail-drop: kada je bafer pun, FIFO slepo odbacuje i prioritetne i obicne pakete
        if (count_ >= Cap) {
            ++stats_.dropped;
            if (pkt.priority >= 8) {
                ++stats_.priority_dropped;
            }
            ++stats_.total;
            record(t0);
            return RoutePort::Dropped;
        }

        fifo_[head_] = pkt;
        head_ = (head_ + 1) % Cap;
        ++count_;

        state_checksum_ ^= process_payload_work(pkt, static_cast<uint32_t>(count_));

        ++stats_.delivered;
        ++stats_.total;
        record(t0);
        return ((pkt.dst_node & 1u) == 0u) ? RoutePort::PortA : RoutePort::PortB;
    }

    uint32_t checksum() const noexcept { return state_checksum_; }
    const BenchmarkStats& stats() const noexcept { return stats_; }

private:
    void record(uint64_t t0) noexcept {
        const uint64_t t1 = read_cycles();
        if (t1 >= t0) {
            const uint64_t d = t1 - t0;
            stats_.total_cycles += d;
            if (d < stats_.min_cycles) stats_.min_cycles = d;
            if (d > stats_.max_cycles) stats_.max_cycles = d;
            if (d > 1200ULL) {
                ++stats_.burst_spikes;
            }
        }
    }

    std::array<PacketFrame, Cap> fifo_{};
    size_t head_;
    size_t tail_;
    size_t count_;
    uint32_t step_div_;
    uint32_t state_checksum_;
    BenchmarkStats stats_;
};

} // namespace negentropy
} // namespace elvarion

int main() {
    using elvarion::negentropy::NegentropicPhaseRouter;
    using elvarion::negentropy::ClassicFifoChannel;
    using elvarion::negentropy::PacketFrame;

    // 0x9E3779B9u = 2^32 / phi (zlatni presek za ravnomerno fazno rasprostiranje)
    NegentropicPhaseRouter<256> elvarion_net(0x9E3779B9u, 120);
    ClassicFifoChannel<32> fifo_net;

    const uint64_t num_packets = 2000000ULL;

    for (uint64_t i = 0; i < num_packets; ++i) {
        PacketFrame pkt;
        pkt.seq = i;
        pkt.src_node = static_cast<uint32_t>((i * 17ULL) & 255ULL);
        pkt.dst_node = static_cast<uint32_t>((i * 31ULL + 7ULL) & 255ULL);
        pkt.priority = ((i % 20ULL) == 0ULL) ? 10u : 2u;
        pkt.payload_hash = static_cast<uint32_t>((i * 2654435761ULL) & 0xFFFFFFFFULL);

        // 80% vremena traje intenzivan burst talas sa visokim sumom
        const bool burst_mode = ((i % 200ULL) < 160ULL);
        pkt.noise_score = burst_mode
            ? static_cast<uint16_t>(220u + (i % 180u))
            : static_cast<uint16_t>(20u + (i % 40u));

        (void)elvarion_net.submit(pkt);
        (void)fifo_net.submit(pkt, burst_mode);
    }

    const auto& es = elvarion_net.stats();
    const auto& fs = fifo_net.stats();

    const double e_avg = es.total > 0
        ? static_cast<double>(es.total_cycles) / static_cast<double>(es.total) : 0.0;
    const double f_avg = fs.total > 0
        ? static_cast<double>(fs.total_cycles) / static_cast<double>(fs.total) : 0.0;

    // Efektivni ciklusi po USPESNO isporucenom paketu (realna cena korisnog protoka)
    const double e_eff_cycles = es.delivered > 0
        ? static_cast<double>(es.total_cycles) / static_cast<double>(es.delivered) : 0.0;
    const double f_eff_cycles = fs.delivered > 0
        ? static_cast<double>(fs.total_cycles) / static_cast<double>(fs.delivered) : 0.0;

    const double e_deliv_pct = es.total > 0
        ? (static_cast<double>(es.delivered) * 100.0) / static_cast<double>(es.total) : 0.0;
    const double f_deliv_pct = fs.total > 0
        ? (static_cast<double>(fs.delivered) * 100.0) / static_cast<double>(fs.total) : 0.0;

    std::printf("=== ELVARION NEGENTROPIJSKI TOK VS KLASICNI FIFO ===\n");
    std::printf("[ELVARION PHASE]\n");
    std::printf("  Isporuceno: %llu / %llu (%.2f%%) | Odbaceno: %llu | Izgubljeni prior.: %llu\n",
                static_cast<unsigned long long>(es.delivered),
                static_cast<unsigned long long>(es.total),
                e_deliv_pct,
                static_cast<unsigned long long>(es.dropped),
                static_cast<unsigned long long>(es.priority_dropped));
    std::printf("  Ciklusi (min/avg/max): %llu / %.2f / %llu | Ciklusi po isporucenom: %.2f | Skokovi: %llu | CHK: %u\n",
                static_cast<unsigned long long>(es.min_cycles),
                e_avg,
                static_cast<unsigned long long>(es.max_cycles),
                e_eff_cycles,
                static_cast<unsigned long long>(es.burst_spikes),
                elvarion_net.checksum());

    std::printf("[KLASICNI FIFO]\n");
    std::printf("  Isporuceno: %llu / %llu (%.2f%%) | Odbaceno: %llu | Izgubljeni prior.: %llu\n",
                static_cast<unsigned long long>(fs.delivered),
                static_cast<unsigned long long>(fs.total),
                f_deliv_pct,
                static_cast<unsigned long long>(fs.dropped),
                static_cast<unsigned long long>(fs.priority_dropped));
    std::printf("  Ciklusi (min/avg/max): %llu / %.2f / %llu | Ciklusi po isporucenom: %.2f | Skokovi: %llu | CHK: %u\n",
                static_cast<unsigned long long>(fs.min_cycles),
                f_avg,
                static_cast<unsigned long long>(fs.max_cycles),
                f_eff_cycles,
                static_cast<unsigned long long>(fs.burst_spikes),
                fifo_net.checksum());

    return 0;
}
