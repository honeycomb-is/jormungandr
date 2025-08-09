// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace battery::chem
{

    enum class Block : uint8_t
    {
        Unknown = 0,
        S,
        P,
        D,
        F
    };

    enum class StandardState : uint8_t
    {
        Unknown = 0,
        Solid,
        Liquid,
        Gas
    };

    class Element
    {
    public:
        Element() = default;

        Element(int atomicNumber,
                std::string symbol,
                std::string name,
                std::string groupLabel,
                int period,
                int groupNumber,
                double atomicMassU,
                StandardState standardState,
                Block block,
                std::string electronConfiguration,
                std::vector<int> oxidationStates,
                double electronegativityPauling,
                int vdwRadiusPm,
                double ionizationEnergyEV,
                double electronAffinityEV,
                double meltingPointK,
                double boilingPointK,
                double densityGCm3)
            : m_atomicNumber(atomicNumber),
              m_symbol(std::move(symbol)),
              m_name(std::move(name)),
              m_groupLabel(std::move(groupLabel)),
              m_period(period),
              m_groupNumber(groupNumber),
              m_atomicMassU(atomicMassU),
              m_standardState(standardState),
              m_block(block),
              m_electronConfiguration(std::move(electronConfiguration)),
              m_oxidationStates(std::move(oxidationStates)),
              m_electronegativityPauling(electronegativityPauling),
              m_vdwRadiusPm(vdwRadiusPm),
              m_ionizationEnergyEV(ionizationEnergyEV),
              m_electronAffinityEV(electronAffinityEV),
              m_meltingPointK(meltingPointK),
              m_boilingPointK(boilingPointK),
              m_densityGCm3(densityGCm3)
        {
        }

        // Getters (immutable public API)
        int atomicNumber() const { return m_atomicNumber; }
        const std::string &symbol() const { return m_symbol; }
        const std::string &name() const { return m_name; }
        const std::string &groupLabel() const { return m_groupLabel; }
        int period() const { return m_period; }
        int groupNumber() const { return m_groupNumber; }
        double atomicMassU() const { return m_atomicMassU; }
        StandardState standardState() const { return m_standardState; }
        Block block() const { return m_block; }
        const std::string &electronConfiguration() const { return m_electronConfiguration; }
        const std::vector<int> &oxidationStates() const { return m_oxidationStates; }
        double electronegativityPauling() const { return m_electronegativityPauling; }
        int vdwRadiusPm() const { return m_vdwRadiusPm; }
        double ionizationEnergyEV() const { return m_ionizationEnergyEV; }
        double electronAffinityEV() const { return m_electronAffinityEV; }
        double meltingPointK() const { return m_meltingPointK; }
        double boilingPointK() const { return m_boilingPointK; }
        double densityGCm3() const { return m_densityGCm3; }

    private:
        int m_atomicNumber{0};
        std::string m_symbol{};
        std::string m_name{};
        std::string m_groupLabel{};
        int m_period{0};
        int m_groupNumber{0};
        double m_atomicMassU{0.0};
        StandardState m_standardState{StandardState::Unknown};
        Block m_block{Block::Unknown};
        std::string m_electronConfiguration{};
        std::vector<int> m_oxidationStates{};
        double m_electronegativityPauling{0.0};
        int m_vdwRadiusPm{0};
        double m_ionizationEnergyEV{0.0};
        double m_electronAffinityEV{0.0};
        double m_meltingPointK{0.0};
        double m_boilingPointK{0.0};
        double m_densityGCm3{0.0};
    };

} // namespace battery::chem
