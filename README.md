<h1 align="center">Honeycomb</h1>
<p align="center">
  <em>Game engine for battery chemistry discovery & simulation</em>
</p>

<p align="center">
  <img src="logo.png" alt="Honeycomb Logo" width="200"/>
</p>

<p align="center">
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/releases">
    <img src="https://img.shields.io/github/v/release/tshimegamolefe-is/Honeycomb?color=brightgreen" alt="Latest Release"/>
  </a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/issues">
    <img src="https://img.shields.io/github/issues/tshimegamolefe-is/Honeycomb" alt="Issues"/>
  </a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/blob/main/LICENSE">
    <img src="https://img.shields.io/github/license/tshimegamolefe-is/Honeycomb" alt="License"/>
  </a>
</p>

---

## 🚀 Overview
Honeycomb is a C++23 game engine designed for simulating and discovering new battery chemistries.  
It combines **real electrochemical modeling** with a **gamified exploration** of the periodic table.  
My goal: find the optimal **anode/cathode combinations** and innovate on battery technology.

---

## 🛠 Features
- **ECS Architecture** — GameEngine → Scene → Systems → EntityManager → Entity → Component
- **Periodic Table Integration** — Explore all elements and their properties
- **Battery KPI Simulation** — Voltage, Wh/kg, Wh/L, capacity, efficiency
- **Gamified Design** — Build, test, and iterate on virtual battery chemistries
- **Performance First** — Cache-friendly SoA, multithreading, GPU-ready design
- **Cross-Platform** — SFML + OpenGL (Linux, macOS, Windows)

---

## 📸 Screenshots
*(Coming soon)*

---

## 🧪 Build & Run (macOS)

Prereqs:
- CMake 3.20+ and a C++23 compiler (Apple Clang on macOS)
- Optional: Homebrew to install CMake (`brew install cmake`)

Debug build:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/Honeycomb
```

Release build:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/Honeycomb
```

Clean rebuild:
```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/Honeycomb
```

Expected output includes electron configuration summaries for Carbon and Oxygen and a single-excitation check.

---

## 🧩 Chemistry module (electron configuration)

This engine includes a high-performance, bitset-based electron configuration system for modeling spin‑orbitals, designed for in‑engine simulation and gameplay logic.

- Performance: fixed-size bitset per `MaxN`, O(1) access, fast XOR/POPCNT-style ops
- Tools: spectroscopic notation, valence electron count, simple Madelung-based Aufbau filling
- ECS-ready: store `ElectronOccupancy<MaxN>` in components for entities


### Project structure (relevant)

```
engine/
  chem/
    electron_config.hpp
src/
  main.cpp
CMakeLists.txt
```
