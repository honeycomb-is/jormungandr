## Honeycomb
Interactive chemistry and cortex-inspired simulation playground (C++23, Metal + ImGui).

<p align="left">
  <img src="/assets/images/logo.png" alt="Honeycomb Logo" width="140"/>
  <br/>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/releases"><img src="https://img.shields.io/github/v/release/tshimegamolefe-is/Honeycomb?color=brightgreen" alt="Latest Release"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/issues"><img src="https://img.shields.io/github/issues/tshimegamolefe-is/Honeycomb" alt="Issues"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/blob/main/LICENSE"><img src="https://img.shields.io/github/license/tshimegamolefe-is/Honeycomb" alt="License"/></a>
  <br/>
  <sub>macOS Metal backend with Dear ImGui UI; fast CSV-driven element data; ECS components for ions, neurons, and cortical columns.</sub>
</p>

---

### What this project does (in plain words)
- **Explore the periodic table interactively**: Open a fast Metal window and browse all 118 elements. Search by symbol or name, scrub by Z, and see properties update instantly.
- **Visualize atoms**: Each element renders a spherical nucleus and animated, orbiting electrons arranged by principal shells.
- **Simulate simple reactions**: Pick two elements and a temperature, and the app predicts basic reactions (salts, oxides/sulfides, hydrogen halides), with rough Gibbs free energy and Arrhenius rates. A tiny reversible reaction integrator shows concentrations over time.
- **Lay the groundwork for predictive coding**: Early ECS components for ions, neurons, dendrites/spines, and cortical columns, plus a minimal membrane‑potential sandbox to prepare for cortical predictive coding experiments.

---

### Current features
- **GUI (Metal + ImGui)**
  - Periodic table browser with live property panel
  - Animated electron shells and nucleus visualization
  - Reactant A/B selection with side‑by‑side atom visuals (kinda a pain in the ass to use - but will improve)
  - Reaction panel: feasibility, rationale, ΔG estimate, k_forward/k_reverse, and product formulae
  - Simple reversible kinetics integrator with concentration plot
  - Fullscreen toggle; keyboard shortcuts: ←/→ = ±1 Z, Cmd+←/→ = ±10 Z

- **Chemistry data**
  - CSV loader: `assets/data/periodic_table.csv`
  - Immutable `periodic_table` registry with O(1) access by atomic number
  - Cache‑friendly `PeriodicTableTensor` for fast numeric lookups

- **ECS components (early)**
  - `ecs::component::element_ref` – references an element by Z
  - `ecs::component::ion` – ionic species with charge and concentration
  - `ecs::component::neuron` – basal/apical dendrites, Vm integrator, leak/input params
  - `ecs::component::cortical_column` – layer counts scaffold

---

### Roadmap (incremental)
- Extend reaction rules: acid–base neutralization, hydration/dehydration, simple organic patterns; add valence/electroneutrality balancing
- Plot time series (lines) and multi‑species kinetics; expose reversible rate editing
- Neuron networks: synapses, layered column dynamics; toward predictive coding with hidden layers
- TODO: GPU acceleration for tensor ops; dataset ingestion for training

---

### Build & run (macOS)
- Prereqs: CMake 3.20+, Clang (C++23), macOS with Metal; project vendors Dear ImGui (submodule)
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

Tips
- Toggle fullscreen from the top of the main window.
- Search by symbol or name (e.g., "Na" or "sodium").
- Keyboard: ←/→ to step Z; hold Command for ±10.

---

### Repository structure (high‑level)
- `src/app_metal.mm` – macOS entrypoint (Metal + ImGui UI)
- `include/honeycomb/chem/` – element model, periodic table, tensor, reaction API
- `src/honeycomb/chem/` – CSV loader and reaction implementation
- `include/honeycomb/ecs/component/` – ECS components (element_ref, ion, neuron, cortical_column)
- `external/imgui/` – Dear ImGui (submodule) with platform/render backends
- `assets/` – data and images (periodic table CSV)

---

### Notes on accuracy
This app is a learning and exploration tool. Reaction predictions, ΔG values, and rates are heuristic and illustrative, not authoritative. As the project evolves, rules and datasets will expand and become more rigorous.

### License
AGPL‑3.0‑or‑later. See `LICENSE`.
