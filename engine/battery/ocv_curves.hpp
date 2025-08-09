// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

namespace battery::battery_models
{
    // Very lightweight placeholder OCV curves for demo purposes.
    // TODO: replace with data-driven curves
    // (e.g., spline fits or lookup tables) parameterized by chemistry.

    inline double clamp01(double x)
    {
        return std::max(0.0, std::min(1.0, x));
    }

    // Graphite-like anode OCV vs. Li (V) as a function of stoichiometry in [0,1]
    inline double ocv_anode_graphite(double x)
    {
        x = clamp01(x);
        // Simple shape: low voltage plateau rising slightly with depletion
        // Roughly 0.08–0.2 V range
        const double v = 0.08 + 0.12 * (1.0 - x);
        return v;
    }

    // Generic layered oxide cathode OCV vs. Li (V)
    inline double ocv_cathode_layered(double x)
    {
        x = clamp01(x);
        // Simple convex shape around ~3.7-4.2 V
        const double v = 3.2 + 1.0 * (1.0 - std::pow(1.0 - x, 0.6));
        return v;
    }

} // namespace battery::battery_models
