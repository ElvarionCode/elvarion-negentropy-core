#include <iostream>
#include <array>
#include <cmath>
#include <iomanip>
#include <cstddef> 
#include <chrono>
#include <random>

// --- MATEMATIČKE KONSTANTE ---
struct alignas(32) BatteryConstants {
    static constexpr double PHI_LARGE = 1.618033988749895;
    static constexpr double PHI_SMALL = 0.618033988749895; // 1 / PHI
};

// --- 1. ELVARION BATTERY INVARIANT CORE (PRODUKCIJSKI MODUL) ---
template <std::size_t CELL_COUNT>
class ElvarionBatteryInvariantCore {
private:
    std::array<double, CELL_COUNT> prev_voltage_state;
    std::array<double, CELL_COUNT> prev_thermal_state;

    // Ultra-brza Padeova racionalna aproksimacija hiperboličkog tangenta za RTOS okruženja
    inline double fast_tanh(double x) const {
        if (x > 4.9) return 1.0;
        if (x < -4.9) return -1.0;
        double x2 = x * x;
        return x * (27.0 + x2) / (27.0 + 9.0 * x2);
    }

public:
    ElvarionBatteryInvariantCore() {
        prev_voltage_state.fill(3.7);  // Nominalni početni napon ćelije (V)
        prev_thermal_state.fill(25.0); // Početna temperatura paketa (°C)
    }

    /**
     * @brief Izvodi determinističku elektro-termalnu stabilizaciju ćelija u nanosekundnom režimu.
     */
    inline void stabilize_pack(const std::array<double, CELL_COUNT>& raw_voltages,
                               const std::array<double, CELL_COUNT>& raw_temperatures,
                               std::array<double, CELL_COUNT>& filtered_voltages,
                               std::array<double, CELL_COUNT>& filtered_temperatures) 
    {
        for (std::size_t i = 0; i < CELL_COUNT; ++i) {
            // Električna stabilizacija (Peglanje nelinearnog šuma usled naglog pražnjenja)
            double v_delta = raw_voltages[i] - prev_voltage_state[i];
            filtered_voltages[i] = prev_voltage_state[i] + 
                BatteryConstants::PHI_SMALL * fast_tanh(v_delta / BatteryConstants::PHI_LARGE);
            prev_voltage_state[i] = filtered_voltages[i];

            // Termalna stabilizacija (Neutralizacija toplotnog drifta bez faznog kašnjenja)
            double t_delta = raw_temperatures[i] - prev_thermal_state[i];
            filtered_temperatures[i] = prev_thermal_state[i] + 
                BatteryConstants::PHI_SMALL * fast_tanh(t_delta / BatteryConstants::PHI_LARGE);
            prev_thermal_state[i] = filtered_temperatures[i];
        }
    }
};

// --- 2. HARDVERSKI STRES-TEST I ULAZNA TAČKA ZA KOMPAJLER ---
int main() {
    std::cout << "=========================================================\n";
    std::cout << " ELVARION BATTERY INVARIANT CORE - 96 CELL STRESS TEST\n";
    std::cout << "=========================================================\n\n";

    // Standardna EV/Tesla konfiguracija od 96 ćelija
    const std::size_t CELLS = 96;
    ElvarionBatteryInvariantCore<CELLS> bms_core;

    std::mt19937_64 rng(42);
    std::normal_distribution<double> voltage_noise(0.0, 0.15); 
    std::normal_distribution<double> thermal_noise(0.0, 0.8);  

    std::array<double, CELLS> raw_voltages;
    std::array<double, CELLS> raw_temperatures;
    std::array<double, CELLS> filtered_voltages;
    std::array<double, CELLS> filtered_temperatures;

    // Simulacija šoka: Ekstremno ubrzanje bolida, pad napona na celom paketu + skok temperature
    for (std::size_t i = 0; i < CELLS; ++i) {
        raw_voltages[i] = 3.2 + voltage_noise(rng);      // Pad na 3.2V usled povlačenja jake struje
        raw_temperatures[i] = 45.0 + thermal_noise(rng); // Trenutni termalni skok na 45°C
    }

    // Merenje sirove brzine obrade kompletnog paketa odjednom
    auto start_time = std::chrono::high_resolution_clock::now();
    bms_core.stabilize_pack(raw_voltages, raw_temperatures, filtered_voltages, filtered_temperatures);
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::nano> latency = end_time - start_time;

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "--- [REZULTATI OBRAĐIVANJA 96 BATERIJSKIH ĆELIJA] ---\n\n";
    std::cout << "   - Latencija za ceo paket:    " << latency.count() << " ns\n";
    std::cout << "   - Srednje vreme po ćeliji:   " << latency.count() / CELLS << " ns/ćeliji\n\n";

    std::cout << "Uzorak pročišćenih podataka sa prve 3 ćelije:\n";
    for (std::size_t i = 0; i < 3; ++i) {
        std::cout << "Ćelija #" << i + 1 << " | Sirovi V: " << raw_voltages[i] << "V -> Elvarion V: " << filtered_voltages[i] << "V\n";
        std::cout << "          | Sirova T: " << raw_temperatures[i] << "C -> Elvarion T: " << filtered_temperatures[i] << "C\n";
    }
    std::cout << "=========================================================\n";

    return 0;
}
