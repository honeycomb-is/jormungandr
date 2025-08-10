// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <cstdint>

namespace honeycomb::ecs::component
{
    struct dendritic_spine
    {
        // Simplified biophysical params for future use
        float neck_resistance_MOhm{100.0f};
        float head_capacitance_pF{0.5f};
    };

    struct dendrite
    {
        // Counts for simplified modeling
        std::int32_t spine_count{0};
        float length_um{200.0f};
    };

    struct neuron
    {
        // Layer index (for cortical column hierarchy)
        std::int8_t layer{2};

        // Dendritic compartments
        dendrite basal;
        dendrite apical;

        // Membrane potential (mV)
        float Vm_mV{-70.0f};
        // Simple leak + input current parameters
        float leak_mV_per_s{10.0f}; // tends towards -70 mV
        float input_mV_per_s{0.0f}; // from synapses (set externally)
    };

} // namespace honeycomb::ecs::component
