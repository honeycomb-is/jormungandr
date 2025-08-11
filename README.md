## Honeycomb

<p align="center">
  <img src="/assets/images/logo.png" alt="Honeycomb Logo" width="300"/>
  <br/>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/releases"><img src="https://img.shields.io/github/v/release/tshimegamolefe-is/Honeycomb?color=brightgreen" alt="Latest Release"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/issues"><img src="https://img.shields.io/github/issues/tshimegamolefe-is/Honeycomb" alt="Issues"/></a>
  <a href="https://github.com/tshimegamolefe-is/Honeycomb/blob/main/LICENSE"><img src="https://img.shields.io/github/license/tshimegamolefe-is/Honeycomb" alt="License"/></a>
  <br/>
</p>


### Build Instructions

#### Prerequisites
- CMake 3.20 or higher
- GLFW3
- A modern C++ compiler with C++23 support
- macOS 10.15 or higher

#### Debug Build (Development)
```bash
# Configure and build in debug mode (default)
cmake -S . -B build
cmake --build build -j

# Run the debug build
./build/Honeycomb
```

#### Release Build (Production)
```bash
# Configure and build in release mode
cmake -S . -B build_prod -DCMAKE_BUILD_TYPE=Release
cmake --build build_prod -j
```

### License
AGPL‑3.0‑or‑later. See `LICENSE`.
