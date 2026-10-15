#ifndef ELVARION_NEGENTROPY_CORE_HPP
#define ELVARION_NEGENTROPY_CORE_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <random>
#include <iomanip>

namespace Elvarion {

template <size_t DIM = 1024>
class NegentropyCore {
public:
    struct alignas(32) StateBuffer {
        std::array<float, DIM> coordinates;
        std::array<float, DIM> phase_values;
        float zero_invariant;
    };

    NegentropyCore() 
        : target_frequency(144.0f)
        , sample_rate(44100.0f)
        , phase_accumulator(0.0f)
        , current_zero_invariant(0.0f) 
    {
        reset_to_zero_point();
    }

    void reset_to_zero_point() {
        phase_accumulator = 0.0f;
        current_zero_invariant = 0.0f;
    }

    StateBuffer process_toroidal_lock(const std::array<float, DIM>& input_signal) {
        StateBuffer output;
        
        float local_phase = phase_accumulator;
        float accumulated_noise = 0.0f;
        
        const float inv_sample_rate = 1.0f / sample_rate;
        const float two_pi_f = 2.0f * 3.14159265358979323846f * target_frequency;
        const float step = two_pi_f * inv_sample_rate;

        #if defined(__GNUC__) || defined(__clang__)
        #pragma GCC ivdep
        #endif
        for (size_t i = 0; i < DIM; ++i) {
            local_phase += step;
            
            if (local_phase > 2.0f * 3.14159265358979323846f) {
                local_phase -= 2.0f * 3.14159265358979323846f;
            }

            float sine_wave = std::sin(local_phase);
            float cosine_wave = std::cos(local_phase);

            output.coordinates[i] = input_signal[i] * sine_wave - accumulated_noise * cosine_wave;
            output.phase_values[i] = sine_wave;

            float instant_noise = std::abs(input_signal[i] - output.coordinates[i]);
            accumulated_noise = (accumulated_noise + instant_noise) / (1.0f + instant_noise * PHI);
        }

        phase_accumulator = local_phase;
        output.zero_invariant = 0.0f;
        current_zero_invariant = output.zero_invariant;

        return output;
    }

    void set_frequency(float freq) {
        target_frequency = freq;
    }

    void set_sample_rate(float rate) {
        sample_rate = rate;
    }

private:
    float target_frequency;
    float sample_rate;
    float phase_accumulator;
    float current_zero_invariant;
    static constexpr float PHI = 1.61803398874989484820f; 
};

}

int main() {
    std::cout << "[ELVARION BENCHMARK] Pokretanje rigoroznog hardverskog testa..." << std::endl;

    constexpr size_t BUFFER_SIZE = 1024;
    constexpr size_t ITERATIONS = 10000;

    Elvarion::NegentropyCore<BUFFER_SIZE> core;
    alignas(32) std::array<float, BUFFER_SIZE> mock_input_signal;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        float clean_signal = std::sin(2.0f * 3.14159f * 144.0f * (static_cast<float>(i) / 44100.0f));
        float sensor_noise = dist(gen) * 0.15f;
        mock_input_signal[i] = clean_signal + sensor_noise;
    }

    std::cout << "[STATUS] Testni signal stabilizovan. Pokrećem toroidalni lock..." << std::endl;
    std::cout << "[INFO] Procesiranje unutar hardverskog cevovoda sa aktivnom potrošnjom podataka..." << std::endl;

    double hardware_checksum_accumulator = 0.0;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (size_t iter = 0; iter < ITERATIONS; ++iter) {
        auto output_buffer = core.process_toroidal_lock(mock_input_signal);
        
        // Prisilan prolaz kroz podatke kako bi kompajler morao da izvrši proračun unutar petlje
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            hardware_checksum_accumulator += output_buffer.coordinates[i] + output_buffer.phase_values[i];
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    auto total_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    double total_duration_ms = static_cast<double>(total_duration_ns) / 1000000.0;
    
    size_t total_points_processed = BUFFER_SIZE * ITERATIONS;
    double time_per_point_ns = static_cast<double>(total_duration_ns) / static_cast<double>(total_points_processed);

    std::cout << "\n================ [REZULTATI MERENJA] ================" << std::endl;
    std::cout << "[PERFORMANSE] Ukupno vreme proračuna: " 
              << std::fixed << std::setprecision(4) << total_duration_ms << " ms" << std::endl;
    std::cout << "[KAPACITET]   Obrađeno koordinatnih tačaka: " 
              << total_points_processed << " u kontinualnom toku." << std::endl;
    std::cout << "[LATENCIJA]   Realno vreme obrade po tački: " 
              << std::fixed << std::setprecision(2) << time_per_point_ns << " ns" << std::endl;
    std::cout << "[KONTROLA]    Hardverska kontrolna suma: " 
              << std::scientific << hardware_checksum_accumulator << std::endl;
    std::cout << "=====================================================" << std::endl;

    return 0;
}

#endif
