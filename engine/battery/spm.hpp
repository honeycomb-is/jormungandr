// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "engine/battery/ocv_curves.hpp"

namespace battery::battery_models
{
    inline double clampValue(double v, double lo, double hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    struct SPMParameters
    {
        // Geom/electrochem (collapsed; normalized)
        double capacityAh{3.0};             // cell capacity (Ah)
        double nominalVoltage{3.7};         // nominal voltage (V)
        double internalResistanceOhm{0.05}; // simple lumped resistance (Ohm)

        // OCV functions; pick chem-specific externally
        double (*ocvAnode)(double){ocv_anode_graphite};
        double (*ocvCathode)(double){ocv_cathode_layered};
    };

    struct SPMState
    {
        // Stoichiometries [0,1] (surface ~bulk in basic SPM approximation)
        double xAnode{0.5};
        double xCathode{0.5};
        double soc{0.5}; // pack-level SoC proxy

        // Running totals
        double timeSeconds{0.0};
        double dischargedAh{0.0};
    };

    struct SPMOutputs
    {
        double voltageV{0.0};
        double currentA{0.0};
        double soc{0.0};
        double powerW{0.0};
        double energyWh{0.0};
    };

    class SingleParticleModel
    {
    public:
        explicit SingleParticleModel(const SPMParameters &p) : params(p) {}

        void reset(double initialSoc)
        {
            state = {};
            state.soc = clampValue(initialSoc, 0.0, 1.0);
            state.xAnode = 1.0 - state.soc; // simple mapping for demo
            state.xCathode = state.soc;     // conserve Li between electrodes
            state.timeSeconds = 0.0;
            state.dischargedAh = 0.0;
        }

        // Step the model by dt with applied current (A). Positive current = discharge.
        SPMOutputs step(double currentA, double dtSeconds)
        {
            // Update SoC by Coulomb counting
            const double dAh = (currentA * dtSeconds) / 3600.0;
            state.dischargedAh = clampValue(state.dischargedAh + dAh, 0.0, params.capacityAh);
            state.soc = clampValue(1.0 - state.dischargedAh / params.capacityAh, 0.0, 1.0);

            // Update electrode stoichiometries (very simplified coupling)
            state.xAnode = clampValue(1.0 - state.soc, 0.0, 1.0);
            state.xCathode = clampValue(state.soc, 0.0, 1.0);

            // Open-circuit voltage (cathode - anode)
            const double ocv = params.ocvCathode(state.xCathode) - params.ocvAnode(state.xAnode);

            // Terminal voltage with simple ohmic drop
            const double v = ocv - currentA * params.internalResistanceOhm;

            state.timeSeconds += dtSeconds;

            SPMOutputs out;
            out.voltageV = v;
            out.currentA = currentA;
            out.soc = state.soc;
            out.powerW = v * currentA;
            out.energyWh = (state.dischargedAh) * v; // crude proxy
            return out;
        }

        const SPMState &getState() const { return state; }
        const SPMParameters &getParams() const { return params; }

    private:
        SPMParameters params{};
        SPMState state{};
    };

} // namespace battery::battery_models
