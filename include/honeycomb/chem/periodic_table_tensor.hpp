// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "honeycomb/chem/periodic_table.hpp"

namespace honeycomb::chem
{

    // Lightweight, cache-friendly tensor view of periodic table numeric data.
    // Stores features in a contiguous [num_elements x num_features] row-major buffer.
    class PeriodicTableTensor
    {
    public:
        enum class Feature : std::uint8_t
        {
            Period = 0,
            GroupNumber,
            AtomicMassU,
            ElectronegativityPauling,
            VdwRadiusPm,
            IonizationEnergyEV,
            ElectronAffinityEV,
            MeltingPointK,
            BoilingPointK,
            DensityGCm3,
            Block,         // Block enum as float (S=1,P=2,D=3,F=4; Unknown=0)
            StandardState, // StandardState enum as float (Solid=1,Liquid=2,Gas=3; Unknown=0)
            Count
        };

        PeriodicTableTensor() = default;
        explicit PeriodicTableTensor(const periodic_table &table)
        {
            build(table);
        }

        void build(const periodic_table &table)
        {
            m_numElements = static_cast<int>(table.size());
            m_numFeatures = static_cast<int>(Feature::Count);
            m_data.assign(static_cast<std::size_t>(m_numElements) * static_cast<std::size_t>(m_numFeatures), 0.0f);

            for (int z = 1; z <= m_numElements; ++z)
            {
                const Element &e = table.by_atomic_number(z);
                set(z, Feature::Period, static_cast<float>(e.period()));
                set(z, Feature::GroupNumber, static_cast<float>(e.groupNumber()));
                set(z, Feature::AtomicMassU, static_cast<float>(e.atomicMassU()));
                set(z, Feature::ElectronegativityPauling, static_cast<float>(e.electronegativityPauling()));
                set(z, Feature::VdwRadiusPm, static_cast<float>(e.vdwRadiusPm()));
                set(z, Feature::IonizationEnergyEV, static_cast<float>(e.ionizationEnergyEV()));
                set(z, Feature::ElectronAffinityEV, static_cast<float>(e.electronAffinityEV()));
                set(z, Feature::MeltingPointK, static_cast<float>(e.meltingPointK()));
                set(z, Feature::BoilingPointK, static_cast<float>(e.boilingPointK()));
                set(z, Feature::DensityGCm3, static_cast<float>(e.densityGCm3()));
                set(z, Feature::Block, static_cast<float>(static_cast<int>(e.block())));
                set(z, Feature::StandardState, static_cast<float>(static_cast<int>(e.standardState())));
            }
        }

        int numElements() const { return m_numElements; }
        int numFeatures() const { return m_numFeatures; }

        // z is atomic number (1-based)
        inline float get(int z, Feature f) const
        {
            const int row = z - 1;
            const int col = static_cast<int>(f);
            return m_data[static_cast<std::size_t>(row) * static_cast<std::size_t>(m_numFeatures) + static_cast<std::size_t>(col)];
        }

        inline const float *rowPtr(int z) const
        {
            const int row = z - 1;
            return &m_data[static_cast<std::size_t>(row) * static_cast<std::size_t>(m_numFeatures)];
        }

    private:
        inline void set(int z, Feature f, float value)
        {
            const int row = z - 1;
            const int col = static_cast<int>(f);
            m_data[static_cast<std::size_t>(row) * static_cast<std::size_t>(m_numFeatures) + static_cast<std::size_t>(col)] = value;
        }

        std::vector<float> m_data;
        int m_numElements{0};
        int m_numFeatures{0};
    };

} // namespace honeycomb::chem
