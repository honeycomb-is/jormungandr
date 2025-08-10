// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "honeycomb/chem/periodic_table.hpp"

namespace honeycomb::chem
{
    struct ReactionInput
    {
        int Z_a{0}; // atomic number of reactant A
        int Z_b{0}; // atomic number of reactant B
        double temperature_K{298.15};
    };

    struct ReactionProduct
    {
        std::string formula; // e.g., "NaCl"
        int stoichiometry{1};
    };

    struct ReactionResult
    {
        bool feasible{false};
        std::string rationale;         // brief textual explanation
        double deltaG_kJ_per_mol{0.0}; // estimated Gibbs free energy change
        double k_forward{0.0};         // Arrhenius forward rate (arbitrary units)
        double k_reverse{0.0};         // Derived from deltaG and k_forward
        int nuA{1};                    // reactant A stoichiometry
        int nuB{1};                    // reactant B stoichiometry
        std::vector<ReactionProduct> products;
    };

    // Simple heuristic reaction predictor: ionic salt formation, oxides/sulfides, hydrogen halides.
    ReactionResult predict_reaction(const periodic_table &table, const ReactionInput &in);

} // namespace honeycomb::chem
