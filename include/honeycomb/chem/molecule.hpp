// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace honeycomb::chem
{
    // Minimal molecule descriptor for future expansion.
    struct Molecule
    {
        std::string formula; // e.g., H2O, NaCl
        // Composition as pairs of (atomic number, count)
        std::vector<std::pair<int, int>> composition;
    };
}
