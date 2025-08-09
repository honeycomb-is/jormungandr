// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "engine/chem/periodic_table.hpp"

#include <array>
#include <cmath>

namespace battery::chem
{
    // Fixed-size feature vector for each element, focused on battery relevance.
    // Layout (example, length 8):
    // [0] normalized atomic mass
    // [1] electronegativity (Pauling) scaled
    // [2] vdw radius (nm) scaled
    // [3] ionization energy (eV) scaled
    // [4] electron affinity (eV) scaled
    // [5] density (g/cm^3) scaled
    // [6] block one-hot reduced: s=0.25, p=0.5, d=0.75, f=1.0, 0 if unknown
    // [7] valence electrons proxy (from oxidation range width)
    template <std::size_t FeatureDim = 8>
    struct PeriodicTableTensor
    {
        static constexpr int Rows = PeriodicTable::kRows;
        static constexpr int Cols = PeriodicTable::kCols;
        using Cell = std::array<double, FeatureDim>;
        using Grid = std::array<std::array<Cell, Cols>, Rows>;

        static double safe(double v, double def = 0.0)
        {
            if (std::isnan(v))
                return def;
            return v;
        }

        static double blockValue(Block b)
        {
            switch (b)
            {
            case Block::S:
                return 0.25;
            case Block::P:
                return 0.5;
            case Block::D:
                return 0.75;
            case Block::F:
                return 1.0;
            default:
                return 0.0;
            }
        }

        static Grid build(const PeriodicTable &pt)
        {
            Grid grid{};
            // Initialize to zeros
            for (auto &row : grid)
                for (auto &cell : row)
                    cell.fill(0.0);

            for (int z = 1; z <= PeriodicTable::kMaxZ; ++z)
            {
                const Element *e = pt.getByZ(z);
                if (!e)
                    continue;
                int r = e->period();
                int c = e->groupNumber();
                if (r < 1 || r > Rows || c < 1 || c > Cols)
                    continue;

                auto &fv = grid[static_cast<std::size_t>(r - 1)][static_cast<std::size_t>(c - 1)];

                // Simple min-max-like scaling constants (can be learned later)
                fv[0] = safe(e->atomicMassU() / 300.0);            // ~0..1
                fv[1] = safe(e->electronegativityPauling() / 4.0); // ~0..1
                fv[2] = safe(e->vdwRadiusPm() / 300.0);            // ~0..1
                fv[3] = safe(e->ionizationEnergyEV() / 20.0);      // ~0..1
                fv[4] = safe(e->electronAffinityEV() / 5.0);       // ~0..1
                fv[5] = safe(e->densityGCm3() / 25.0);             // ~0..1
                fv[6] = blockValue(e->block());

                // Oxidation state span as a crude valence proxy
                int minOx = 0, maxOx = 0;
                const auto &ox = e->oxidationStates();
                for (int v : ox)
                {
                    minOx = std::min(minOx, v);
                    maxOx = std::max(maxOx, v);
                }
                fv[7] = static_cast<double>(maxOx - minOx) / 10.0; // scaled width
            }
            return grid;
        }
    };

} // namespace battery::chem
