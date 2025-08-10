// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <cstdint>

namespace honeycomb::ecs::component
{

    // Represents an ionic species and its state within a compartment.
    struct ion
    {
        // Atomic number for element (e.g., Na=11, K=19, Cl=17, Ca=20)
        std::int16_t atomic_number{0};
        // Integer charge (e.g., +1 for Na+, -1 for Cl-)
        std::int8_t charge{0};
        // Concentration in mM
        float concentration_mM{0.0f};
    };

} // namespace honeycomb::ecs::component
