// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <cstdint>

namespace honeycomb::ecs::component
{

    // Lightweight ECS component that references an element by atomic number (Z).
    // Used to attach elemental identity to an entity without duplicating heavy data.
    struct element_ref
    {
        std::int16_t atomic_number{0}; // Z (1..118); 0 if unset
    };

} // namespace honeycomb::ecs::component
