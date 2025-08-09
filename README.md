## Honeycomb
Game engine for battery chemistry discovery and simulation.

<p align="left">
  <img src="logo.png" alt="Honeycomb Logo" width="140"/>
  <br/>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/releases"><img src="https://img.shields.io/github/v/release/tshimegamolefe-is/Honeycomb?color=brightgreen" alt="Latest Release"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/issues"><img src="https://img.shields.io/github/issues/tshimegamolefe-is/Honeycomb" alt="Issues"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/blob/main/LICENSE"><img src="https://img.shields.io/github/license/tshimegamolefe-is/Honeycomb" alt="License"/></a>
  <br/>
</p>

---

### Overview
Honeycomb is a C++23 engine focused on exploring and simulating battery chemistries. It blends an ECS architecture with a data‑driven periodic table and spin‑orbital electron configuration utilities to evaluate anode/cathode candidates and reason about properties relevant to energy storage.

### Core features
- ECS-first design for simulation systems
- Periodic table registry with immutable element properties and fast lookups
- Electron configuration utilities: spin‑orbitals, spectroscopic notation, valence, Aufbau filling
- Data-driven element loading (CSV) and formatted reporting
- CMake build, cross‑platform C++23

---

### Architecture (ECS and data)
- Element data
  - `engine/chem/element.hpp`: `Element` (atomic mass, standard state, block, electron configuration, oxidation states, electronegativity, van der Waals radius, ionization energy, electron affinity, melting/boiling points, density)
  - `engine/chem/periodic_table.hpp`: `PeriodicTable` registry (O(1) lookup by Z/symbol)
  - `engine/chem/element_loader.hpp`: CSV loader
  - `engine/chem/element_data.hpp`: bundled CSV dataset (can be extended)
- ECS component
  - `ElementRefComponent { int atomicNumber; }` for referencing elements in entities
- Electron configuration
  - `engine/chem/electron_config.hpp`: `ElectronOccupancy<MaxN>`, `Spin/SpinOrbital`, utilities

### Build & run
- Prereqs: CMake 3.20+, C++23 compiler
- Debug
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/Honeycomb
```
- Release
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/Honeycomb
```

### Example demo output
The demo prints a per‑element summary (H, Li, C, O, Na, Mg, Al, K, Cu), including:
- Atomic Mass, Standard State, Electron Configuration, Block
- Oxidation States, Electronegativity (Pauling), van der Waals radius
- Ionization Energy, Electron Affinity, Melting/Boiling Points, Density
It also lists candidate anodes via a simple electronegativity threshold.

### Extend the dataset
- Add/modify rows in `engine/chem/element_data.hpp` CSV text. Loader will ingest on startup.
- For large tables, externalize CSV and add a file‑based loader.

---

### Roadmap
- Full periodic table coverage (118 elements)
- Battery KPI computations (voltage, capacity, Wh/kg, Wh/L, round‑trip efficiency)
- Visualization layer and interactive exploration
- Save/load scenarios and results

### License
AGPL‑3.0‑or‑later. See `LICENSE`.
