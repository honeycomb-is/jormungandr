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
- ECS-first architecture – entities flow through systems such as `BatterySimulationSystem`
- Immutable periodic-table registry with O(1) lookup by Z/symbol
- Electron-configuration helpers: spin orbitals, spectroscopic notation, valence counting
- Flexible CSV loader with schema detection (internal vs. external) and graceful NaN handling
- Simplified Single Particle Model (SPM) that outputs live KPIs each timestep
- Periodic-table tensor (7 × 18 × F) for ML/GPU experimentation
- Human-readable console tables for elements and KPIs
- CMake build, cross-platform C++23

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
The demo currently performs the following steps:

1. Starts a Single Particle Model discharge and streams a live KPI table:

`Time  Voltage  Current  SoC  Power  Energy`

(Values update each timestep until the cell is empty.)

### Extend the dataset
- Add/modify rows in `engine/chem/element_data.hpp` CSV text. Loader will ingest on startup.
- For large tables, externalize CSV and add a file‑based loader.

---

### Roadmap
- Chemistry-specific OCV curves and parameter datasets
- GPU-accelerated tensor processing and ML-based chemistry recommendations
- 2D/3D renderer & GUI for interactive battery visualization
- ECS battery simulation system and scenario save/load
- Higher fidelity models (SPMe / DFN) and thermal & ageing effects

### License
AGPL‑3.0‑or‑later. See `LICENSE`.
