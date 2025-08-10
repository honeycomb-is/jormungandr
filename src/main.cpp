// SPDX-License-Identifier: AGPL-3.0-or-later
#include <iostream>

#include "honeycomb/chem/csv_loader.hpp"
#include "honeycomb/chem/periodic_table.hpp"
#include "honeycomb/ecs/component/element_ref.hpp"

int main()
{
    using namespace honeycomb;

    auto elements = chem::load_elements_from_csv("assets/data/periodic_table.csv");
    chem::periodic_table table{std::move(elements)};

    std::cout << "Enter atomic number (1-118): ";
    int atomic_number = 0;
    if (!(std::cin >> atomic_number) || atomic_number < 1 || atomic_number > static_cast<int>(table.size()))
    {
        std::cerr << "Invalid atomic number.\n";
        return 1;
    }

    ecs::component::element_ref ref{.atomic_number = static_cast<std::int16_t>(atomic_number)};
    const auto &e = table.by_atomic_number(ref.atomic_number);

    auto state_to_string = [](chem::StandardState s)
    {
        switch (s)
        {
        case chem::StandardState::Solid:
            return "Solid";
        case chem::StandardState::Liquid:
            return "Liquid";
        case chem::StandardState::Gas:
            return "Gas";
        default:
            return "Unknown";
        }
    };
    auto block_to_string = [](chem::Block b)
    {
        switch (b)
        {
        case chem::Block::S:
            return "s";
        case chem::Block::P:
            return "p";
        case chem::Block::D:
            return "d";
        case chem::Block::F:
            return "f";
        default:
            return "Unknown";
        }
    };

    std::cout << "\n=== Element Information ===\n";
    std::cout << "Z: " << e.atomicNumber() << "\n";
    std::cout << "Symbol: " << e.symbol() << "\n";
    std::cout << "Name: " << e.name() << "\n";
    std::cout << "Group label: " << e.groupLabel() << "\n";
    std::cout << "Period: " << e.period() << "\n";
    std::cout << "Group number: " << e.groupNumber() << "\n";
    std::cout << "CPK color (hex): " << e.cpkHexColor() << "\n";
    std::cout << "Atomic mass (u): " << e.atomicMassU() << "\n";
    std::cout << "Standard state: " << state_to_string(e.standardState()) << "\n";
    std::cout << "Block: " << block_to_string(e.block()) << "\n";
    std::cout << "Electron configuration: " << e.electronConfiguration() << "\n";
    std::cout << "Oxidation states: ";
    {
        const auto &ox = e.oxidationStates();
        if (ox.empty())
        {
            std::cout << "(none)";
        }
        else
        {
            for (std::size_t i = 0; i < ox.size(); ++i)
            {
                if (i)
                    std::cout << ", ";
                std::cout << ox[i];
            }
        }
        std::cout << "\n";
    }
    std::cout << "Electronegativity (Pauling): " << e.electronegativityPauling() << "\n";
    std::cout << "Atomic Radius(van der Waals) (pm): " << e.vdwRadiusPm() << "\n";
    std::cout << "Ionization energy (eV): " << e.ionizationEnergyEV() << "\n";
    std::cout << "Electron affinity (eV): " << e.electronAffinityEV() << "\n";
    std::cout << "Melting point (K): " << e.meltingPointK() << "\n";
    std::cout << "Boiling point (K): " << e.boilingPointK() << "\n";
    std::cout << "Density (g/cm^3): " << e.densityGCm3() << "\n";
    std::cout << "Year discovered: " << e.yearDiscovered() << "\n";

    return 0;
}
