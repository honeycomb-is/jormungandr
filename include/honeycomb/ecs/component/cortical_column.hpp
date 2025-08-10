// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <cstdint>

namespace honeycomb::ecs::component
{
    struct cortical_column
    {
        // Identity
        std::int32_t column_id{0};

        // Simplified layer population sizes
        std::int32_t layer2_3_neurons{1000};
        std::int32_t layer4_neurons{800};
        std::int32_t layer5_neurons{600};
        std::int32_t layer6_neurons{700};
    };
}
