#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <array>
#include <iomanip>
#include <numeric>

constexpr double PHI_LARGE = 1.618033988749895;
constexpr double PHI_SMALL = 0.618033988749895;
constexpr size_t NUM_SEEDS = 30;
constexpr size_t NUM_SAMPLES = 100000;

using State9D = std::array<double, 9>;

// Ultra-brza Padeova racionalna aproksimacija hiperboličkog tangenta za RTOS okruženja
inline double fast_tanh(double x) {
    if (x > 4.9) return 1.0;
    if (x < -4.9) return -1.0;
    double x2 = x * x;
    return x * (27.0 + x2) / (27.0 + 9.0 * x2);
}

// --- 1. ELVARION NEGENTROPY CORE ---
class Elvarion9DCore {
private: State9D prev_state;
public:
    Elvarion9DCore() { prev_state.fill(0.0); }
    inline State9D update(const State9D& measurement) {
        State9D current_state;
        for (size_t i = 0; i < 9; ++i) {
            double delta = measurement[i] - prev_state[i];
            current_state[i] = prev_state[i] + PHI_SMALL * fast_tanh(delta / PHI_LARGE);
        }
        return prev_state = current_state;
    }
};

// --- 2. ROBUSNI KALMANOV FILTER (Sa ugrađenim Gating-om) ---
class RobustKalman9D {
private:
    State9D x;
    std::array<std::array<double, 9>, 9> P;
    double Q_noise; double R_noise;
    double gate_threshold = 15.0;
public:
    RobustKalman9D(double q, double r) : Q_noise(q), R_noise(r) {
        x.fill(0.0);
        for (size_t i = 0; i < 9; ++i) P[i][i] = 1.0;
    }
    inline State9D update(const State9D& z) {
        for (size_t i = 0; i < 9; ++i) P[i][i] += Q_noise;
        for (size_t i = 0; i < 9; ++i) {
            double innovation = z[i] - x[i];
            if (std::abs(innovation) > gate_threshold) {
                innovation = (innovation > 0 ? gate_threshold : -gate_threshold);
            }
            double innovation_cov = P[i][i] + R_noise;
            double K = P[i][i] / innovation_cov;
            x[i] = x[i] + K * innovation;
            P[i][i] = (1.0 - K) * P[i][i];
        }
        return x;
    }
};

// Pomoćna funkcija za statističke proračune (Srednja vrednost i Standardna Devijacija)
void calculate_stats(const std::vector<double>& v, double& mean, double& stdev) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size() - mean * mean);
}

int main() {
    std::vector<double> e_rmses_shock(NUM_SEEDS), k_rmses_shock(NUM_SEEDS);
    std::vector<double> e_rmses_rec(NUM_SEEDS), k_rmses_rec(NUM_SEEDS);
    std::vector<double> e_rmses_clean(NUM_SEEDS), k_rmses_clean(NUM_SEEDS);
    std::vector<double> e_latencies(NUM_SEEDS), k_latencies(NUM_SEEDS);

    double noise_std = 4.5;
    size_t shock_start = 50000;
    size_t shock_end = 50050;
    size_t rec_end = 50550;

    for (size_t s = 0; s < NUM_SEEDS; ++s) {
        std::mt19937_64 rng(1000 + s); // Dinamički seed za svaki scenario
        std::normal_distribution<double> normal_dist(0.0, 1.0);

        std::vector<State9D> true_trajectory(NUM_SAMPLES);
        std::vector<State9D> noisy_measurements(NUM_SAMPLES);
        std::vector<State9D> clean_measurements(NUM_SAMPLES);

        for (size_t t = 0; t < NUM_SAMPLES; ++t) {
            double time = t * 0.001;
            for (size_t i = 0; i < 9; ++i) {
                true_trajectory[t][i] = 15.0 * std::sin(time * (i + 1));
                double base_noise = normal_dist(rng) * noise_std;
                
                clean_measurements[t][i] = true_trajectory[t][i] + base_noise;
                
                double shock_noise = base_noise;
                if (t >= shock_start && t < shock_end) {
                    shock_noise += 150.0; // Simulacija brutalnog udara vetra
                }
                noisy_measurements[t][i] = true_trajectory[t][i] + shock_noise;
            }
        }

        Elvarion9DCore elvarion;
        RobustKalman9D kalman(0.15, noise_std * noise_std);

        double e_err_shock = 0.0, k_err_shock = 0.0;
        double e_err_rec = 0.0, k_err_rec = 0.0;

        // --- MERENJE ELVARION CORE PERFORMANSI ---
        auto t0 = std::chrono::high_resolution_clock::now();
        for (size_t t = 0; t < NUM_SAMPLES; ++t) {
            State9D est = elvarion.update(noisy_measurements[t]);
            if (t >= shock_start && t < shock_end) {
                for(size_t i=0; i<9; ++i) e_err_shock += std::pow(est[i] - true_trajectory[t][i], 2);
            } else if (t >= shock_end && t < rec_end) {
                for(size_t i=0; i<9; ++i) e_err_rec += std::pow(est[i] - true_trajectory[t][i], 2);
            }
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        e_latencies[s] = std::chrono::duration<double, std::nano>(t1 - t0).count() / NUM_SAMPLES;

        // --- MERENJE KALMAN EKF PERFORMANSI ---
        auto t2 = std::chrono::high_resolution_clock::now();
        for (size_t t = 0; t < NUM_SAMPLES; ++t) {
            State9D est = kalman.update(noisy_measurements[t]);
            if (t >= shock_start && t < shock_end) {
                for(size_t i=0; i<9; ++i) k_err_shock += std::pow(est[i] - true_trajectory[t][i], 2);
            } else if (t >= shock_end && t < rec_end) {
                for(size_t i=0; i<9; ++i) k_err_rec += std::pow(est[i] - true_trajectory[t][i], 2);
            }
        }
        auto t3 = std::chrono::high_resolution_clock::now();
        k_latencies[s] = std::chrono::duration<double, std::nano>(t3 - t2).count() / NUM_SAMPLES;

        e_rmses_shock[s] = std::sqrt(e_err_shock / (50 * 9));
        k_rmses_shock[s] = std::sqrt(k_err_shock / (50 * 9));
        e_rmses_rec[s] = std::sqrt(e_err_rec / (500 * 9));
        k_rmses_rec[s] = std::sqrt(k_err_rec / (500 * 9));

        // --- MERENJE ČISTOG LETA BEZ UDARA ---
        Elvarion9DCore elvarion_c;
        RobustKalman9D kalman_c(0.15, noise_std * noise_std);
        double e_err_clean = 0.0, k_err_clean = 0.0;

        for (size_t t = 0; t < NUM_SAMPLES; ++t) {
            State9D est_e = elvarion_c.update(clean_measurements[t]);
            State9D est_k = kalman_c.update(clean_measurements[t]);
            for(size_t i=0; i<9; ++i) {
                e_err_clean += std::pow(est_e[i] - true_trajectory[t][i], 2);
                k_err_clean += std::pow(est_k[i] - true_trajectory[t][i], 2);
            }
        }
        e_rmses_clean[s] = std::sqrt(e_err_clean / (NUM_SAMPLES * 9));
        k_rmses_clean[s] = std::sqrt(k_err_clean / (NUM_SAMPLES * 9));
    }

    // Agregacija statističkih podataka
    double e_mean_shock, e_std_shock, k_mean_shock, k_std_shock;
    double e_mean_rec, e_std_rec, k_mean_rec, k_std_rec;
    double e_mean_clean, e_std_clean, k_mean_clean, k_std_clean;
    double e_mean_lat, e_std_lat, k_mean_lat, k_std_lat;

    calculate_stats(e_rmses_shock, e_mean_shock, e_std_shock);
    calculate_stats(k_rmses_shock, k_mean_shock, k_std_shock);
    calculate_stats(e_rmses_rec, e_mean_rec, e_std_rec);
    calculate_stats(k_rmses_rec, k_mean_rec, k_std_rec);
    calculate_stats(e_rmses_clean, e_mean_clean, e_std_clean);
    calculate_stats(k_rmses_clean, k_mean_clean, k_std_clean);
    calculate_stats(e_latencies, e_mean_lat, e_std_lat);
    calculate_stats(k_latencies, k_mean_lat, k_std_lat);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "=========================================================\n";
    std::cout << "  ZVANIČNI INDUSTRIJSKI VERIFIKACIONI IZVEŠTAJ (30 SEEDS) \n";
    std::cout << "=========================================================\n\n";
    
    std::cout << "1. FAZA IMPULSNOG UDARA (50 UZORAKA):\n";
    std::cout << "   - ELVARION: Srednja vrednost = " << e_mean_shock << " | StDev = " << e_std_shock << "\n";
    std::cout << "   - KALMAN:   Srednja vrednost = " << k_mean_shock << " | StDev = " << k_std_shock << "\n\n";

    std::cout << "2. FAZA OPORAVKA (500 UZORAKA POSLE UDARA):\n";
    std::cout << "   - ELVARION: Srednja vrednost = " << e_mean_rec << " | StDev = " << e_std_rec << "\n";
    std::cout << "   - KALMAN:   Srednja vrednost = " << k_mean_rec << " | StDev = " << k_std_rec << "\n\n";

    std::cout << "3. TEST ČISTOG LETA BEZ UDARA (NORMAL FLIGHT):\n";
    std::cout << "   - ELVARION: Prosečan RMSE = " << e_mean_clean << " | StDev = " << e_std_clean << "\n";
    std::cout << "   - KALMAN:   Prosečan RMSE = " << k_mean_clean << " | StDev = " << k_std_clean << "\n\n";

    std::cout << "4. RTOS ANALIZA VREMENA IZVRŠAVANJA (CELI 9D CIKLUS):\n";
    std::cout << "   - ELVARION Latencija: Srednja = " << e_mean_lat << " ns | Jitter (StDev) = " << e_std_lat << " ns\n";
    std::cout << "   - KALMAN Latencija:   Srednja = " << k_mean_lat << " ns | Jitter (StDev) = " << k_std_lat << " ns\n";
    std::cout << "=========================================================\n";

    return 0;
}
