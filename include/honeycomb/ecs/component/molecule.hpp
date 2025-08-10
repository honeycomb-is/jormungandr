// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <string>

namespace honeycomb::ecs::component
{
    struct molecule
    {
        std::string formula;
        int count{1};
    };
}
