// SPDX-License-Identifier: AGPL-3.0-or-later
#include "engine/chem/electron_config.hpp"
#include "engine/chem/periodic_table.hpp"
#include "engine/chem/element_loader.hpp"
#include "engine/chem/element_data.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

int main()
{
    using namespace battery::chem;
    using Config = ElectronOccupancy<4>;

    // Load periodic table dataset (subset for demo) and create ECS-style element refs
    PeriodicTable table;
    loadElementsCSV(table, data::kElementsCSV);

    std::vector<ElementRefComponent> entities;
    for (int z : {1, 3, 6, 8, 11, 12, 13, 19, 29})
        entities.push_back(ElementRefComponent{z});

    auto stateToString = [](StandardState s)
    {
        switch (s)
        {
        case StandardState::Solid:
            return "solid";
        case StandardState::Liquid:
            return "liquid";
        case StandardState::Gas:
            return "gas";
        default:
            return "unknown";
        }
    };

    auto blockToString = [](Block b)
    {
        switch (b)
        {
        case Block::S:
            return "s";
        case Block::P:
            return "p";
        case Block::D:
            return "d";
        case Block::F:
            return "f";
        default:
            return "unknown";
        }
    };

    auto formatOxidationStates = [](const std::vector<int> &ox)
    {
        if (ox.empty())
            return std::string("n/a");
        std::string out;
        for (std::size_t i = 0; i < ox.size(); ++i)
        {
            if (i > 0)
            {
                out += ", ";
            }
            int v = ox[i];
            if (v > 0)
                out.push_back('+');
            out += std::to_string(v);
        }
        return out;
    };

    // Print a human-readable summary per entity using the registry
    for (const auto &eRef : entities)
    {
        const Element *el = table.getByZ(eRef.atomicNumber);
        if (!el)
            continue;
        std::cout << "\n=== " << el->name() << " (" << el->symbol() << ") — Z=" << el->atomicNumber() << " ===\n"
                  << "Atomic Mass\t" << std::fixed << std::setprecision(4) << el->atomicMassU() << std::defaultfloat << " u\n"
                  << "Standard state:        " << stateToString(el->standardState()) << "\n"
                  << "Electron Configuration\t" << el->electronConfiguration() << "\n"
                  << "Block\t" << blockToString(el->block()) << "\n"
                  << "Oxidation States\t" << formatOxidationStates(el->oxidationStates()) << "\n"
                  << "Electronegativity (Pauling Scale)\t" << el->electronegativityPauling() << "\n"
                  << "Atomic Radius (van der Waals)\t" << el->vdwRadiusPm() << " pm\n"
                  << "Ionization Energy\t" << el->ionizationEnergyEV() << " eV\n"
                  << "Electron Affinity\t" << el->electronAffinityEV() << " eV\n"
                  << "Melting Point\t" << el->meltingPointK() << " K\n"
                  << "Boiling Point\t" << el->boilingPointK() << " K\n"
                  << "Density\t" << el->densityGCm3() << " g/cm³\n";
    }

    // Quick heuristic: candidate anodes tend to have low electronegativity (< ~1.2)
    std::cout << "\nCandidate anodes (low electronegativity):\n";
    for (const auto &eRef : entities)
    {
        const Element *el = table.getByZ(eRef.atomicNumber);
        if (!el)
            continue;
        if (el->electronegativityPauling() > 0.0 && el->electronegativityPauling() < 1.2)
        {
            std::cout << " - " << el->name() << " (" << el->symbol() << ") EN="
                      << el->electronegativityPauling() << "\n";
        }
    }

    // Example: Carbon ground-state (1s^2 2s^2 2p^2)
    Config carbon;
    carbon.set(so(1, 0, 0, Spin::Alpha));
    carbon.set(so(1, 0, 0, Spin::Beta));
    carbon.set(so(2, 0, 0, Spin::Alpha));
    carbon.set(so(2, 0, 0, Spin::Beta));
    carbon.set(so(2, 1, -1, Spin::Alpha));
    carbon.set(so(2, 1, 0, Spin::Alpha));

    std::cout << "Carbon | Z=6 | electrons = " << carbon.countElectrons()
              << ", notation = " << spectroscopicNotation(carbon)
              << ", valence = " << valenceElectronCount(carbon) << '\n';

    // Oxygen via Aufbau (approximate)
    auto oxygen = buildAufbauConfiguration<4>(8);
    std::cout << "Oxygen | Z=8 | electrons = " << oxygen.countElectrons()
              << ", notation = " << spectroscopicNotation(oxygen)
              << ", valence = " << valenceElectronCount(oxygen) << '\n';

    // Single excitation example
    Config carbon_excited = carbon;
    carbon_excited.reset(so(2, 1, 0, Spin::Alpha));
    carbon_excited.set(so(2, 1, +1, Spin::Beta));
    std::cout << "Differing spin-orbitals = "
              << Config::differingSpinOrbitals(carbon, carbon_excited)
              << ", single excitation? "
              << (Config::isSingleExcitation(carbon, carbon_excited) ? "yes" : "no")
              << '\n';

    return 0;
}