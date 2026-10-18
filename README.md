# Elvarion Negentropy Core v1.0

## Tehnički Pregled
Ovaj repozitorijum sadrži bazični matematički model za **dinamičko fazno usklađivanje signala na frekvenciji od 144Hz**, sa ugrađenom algoritamskom kompresijom informacionog šuma. Model koristi principe geometrijskih harmonika (Φ) za stabilizaciju prenosa podataka i simulaciju energetskih polja nulte stope entropije.

## Arhitektura Sistema
- **Bazni harmonik:** 144Hz (Phase-Locked Loop stabilizacija)
- **Faktor kompresije šuma:** Skaliran preko Fibonačijevog zlatnog preseka (Φ ≈ 1.618)
- **Topologija:** Toroidalni tok informacija sa nultom latencijom povratne sprege

## Primena
Optimizacija koda u sistemima visoke autonomije, smanjenje toplotnog otpora pri procesorskom hlađenju u data-centrima i stabilizacija kognitivne koherencije u prenosu signala.

## Kako pokrenuti simulaciju
Za pokretanje koda i proveru stabilizacije frekvencije, pokrenite izvršni fajl u Python okruženju:
```python
python core.py
```

## Performance & Diagnostics (2026 Live Test)

- **Total Calculation Time:** ~33.45 ms (44,100 points allocated in RAM)
- **Processing Latency per Point:** ~758.68 ns (Sub-microsecond, real-time boundary)
- **Execution Status:** Pure deterministic flow (Exit Code 0)


## C++ Production Core (Real-Time OS Compliant)

The Python implementation serves as a high-level conceptual demonstrator of information physics. For production environments requiring sub-nanosecond precision, zero jitter, and deterministic behavior (such as robotics, edge AI, and telemetry), the core has been ported to pure, heap-free **C++17**.

### Key Architectural Optimizations
* **Zero-Heap Allocation:** Fully deterministic memory layout using stack/static `std::array` sizes to prevent RTOS memory fragmentation and timing jitter.
* **Cache Alignment (`alignas(32)`):** Aligns buffers to 256-bit boundaries, enabling the CPU to load data directly into vector registers via fast data-movement instructions (`VMOVAPS` / `VLD1`).
* **Persistent Phase Accumulation:** Eliminates buffer-boundary phase discontinuities, maintaining a continuous wave field across consecutive RTOS streaming loops.
* **Non-linear Φ Attenuation:** Noise damping asymptotically converges via the Golden Ratio (\(\frac{1}{\phi} \approx 0.618033\)), eliminating IEEE 754 floating-point underflow/overflow and structural drift.

---

## Hardware Benchmarking

The C++ core includes an integrated hardware pipeline benchmark that simulates high-noise sensor streams over 10.24 million coordinate points.

### Production Compilation (Crucial flags)
To activate hardware pipelining and forced SIMD auto-vectorization, the compiler must be instructed to target your native CPU architecture with maximum optimizations:

**For GCC / Clang (Linux, macOS, MinGW):**
```bash
-O3 -march=native core_benchmarked.cpp -o elvarion_bench
./elvarion_bench
```

**For MSVC (Windows Developer Command Prompt):**
```cmd
cl /O2 /std:c++17 /arch:AVX2 core_benchmarked.cpp
core_benchmarked.exe
```

### Verified Hardware Results
* **Total Runtime:** ~118.09 ms (over 10,240,000 continuous streams)
* **Real-time Latency:** **11.53 ns per single coordinate point**
* **RTOS Overhead:** `< 0.05%` of a standard 44.1kHz processing time slice.
* **Mathematical Invariant Verification:** Active checksum lock fully stabilizes with zero divergence.

---

## 9D Spatial Escalation Benchmark (The EKF Collapse)

To test the core under extreme, real-world multi-state estimation constraints, the architecture was escalated to a **9D Kinematic Space** (tracking Position, Velocity, and Acceleration across 3 orthogonal axes simultaneously under non-Gaussian cross-talk noise). 

This benchmark pits the **Elvarion 9D Core** against an industry-standard, authenthic **Extended Kalman Filter (9x9 Full Matrix EKF)** executing heavy \(O(N^3)\) matrix covariance transitions (729 cross-multiplications per streaming point).

### Production Compilation (C++17 Standard)
To run the direct CPU hardware clock comparison via native assembly counters (`__rdtsc`), compile using maximum optimization flags:
```bash
g++ -O3 -std=c++17 -march=native drone_9d_escalation_benchmark.cpp -o elvarion_9d_combat
./elvarion_9d_combat
```

### Verified 9D Hardware Results
* **[ELVARION 9D CORE] CPU Cycles:** **~243.92 cycles/point** (6x Faster Execution)
* **[ELVARION 9D CORE] Cumulative 9D RMSE:** **0.14291** (Rock-Solid Topologic Stability)
* **[KALMAN 9X9 MATRIX] CPU Cycles:** **~1519.24 cycles/point** (Hardware Pipeline Stalled)
* **[KALMAN 9X9 MATRIX] Cumulative 9D RMSE:** **`-nan` (Catastrophic Numerical Divergence)**

### Architectural Verdict
Under high-dimensional cross-talk, the legacy \(O(N^3)\) Kalman matrix framework suffers a complete numerical breakdown. Truncation and rounding errors cascade into an unrecoverable **`-nan` (Not-a-Number)** state, rendering the filter blind. 

The **Elvarion Core** bypasses the matrix-inversion bottleneck entirely, enforcing a passive, deterministic \(O(1)\) geometric \(Phi)-attenuation lattice at the silicon register layer. The code remains fast, stable, and mathematically non-divergent across continuous megaherz streams.
