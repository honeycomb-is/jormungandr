// SPDX-License-Identifier: AGPL-3.0-or-later
#include "engine/chem/periodic_table.hpp"
#include "engine/chem/element_loader.hpp"
#include "engine/chem/element_data.hpp"
#include "engine/battery/spm.hpp"
#include "engine/chem/periodic_table_tensor.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <array>

int main()
{
    using namespace battery::chem;
    // Load periodic table dataset (subset for demo) and create ECS-style element refs
    PeriodicTable table;
    // Prefer repository CSV under assets/, then fallback to local working dir, then embedded
    bool loadedFromAssets = false, loadedFromLocal = false;
    [[maybe_unused]] bool loadedFromEmbedded = false;
    try
    {
        loadFromFile(table, std::string("assets/periodic_table.csv"));
        loadedFromAssets = true;
    }
    catch (...)
    {
        try
        {
            loadFromFile(table, std::string("elements.csv"));
            loadedFromLocal = true;
        }
        catch (...)
        {
            loadElementsCSV(table, data::kElementsCSV);
            loadedFromEmbedded = true;
        }
    }

    std::cout << "\n=== Elements Loaded ===\n";
    std::cout << "Source: "
              << (loadedFromAssets ? "assets/periodic_table.csv" : loadedFromLocal ? "elements.csv"
                                                                                   : "embedded demo data")
              << "\n";
    std::cout << std::left
              << std::setw(4) << "Z" << "| "
              << std::setw(6) << "Sym" << "| "
              << std::setw(20) << "Name" << "| "
              << std::setw(6) << "Period" << "| "
              << std::setw(7) << "Group#" << '\n'
              << std::string(4, '-') << "+-" << std::string(6, '-') << "+-"
              << std::string(20, '-') << "+-" << std::string(6, '-') << "+-"
              << std::string(7, '-') << '\n';
    int loadedCount = 0;
    for (int z = 1; z <= PeriodicTable::kMaxZ; ++z)
    {
        const Element *e = table.getByZ(z);
        if (!e)
            continue;
        ++loadedCount;
    }
    std::cout << "Total elements loaded: " << loadedCount << "\n";

    std::vector<ElementRefComponent> entities;
    for (int z : {1, 3, 6, 8, 11, 12, 13, 19, 29})
        entities.push_back(ElementRefComponent{z});

    // Select game-relevant elements: Carbon anode and Lithium carrier
    const Element *anode = table.getBySymbol("C");
    const Element *carrier = table.getBySymbol("Li");
    auto blockToStr = [](Block b)
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
            return "n/a";
        }
    };

    std::cout << "\n=== Battery Chemistry (Selected) ===\n";
    std::cout << std::left
              << std::setw(12) << "Role" << "| "
              << std::setw(20) << "Element" << "| "
              << std::setw(6) << "Symbol" << "| "
              << std::setw(5) << "Block" << "| "
              << std::setw(23) << "Group" << "| "
              << std::setw(6) << "Period" << "| "
              << std::setw(6) << "Group#" << '\n'
              << std::string(12, '-') << "+-" << std::string(20, '-') << "+-"
              << std::string(6, '-') << "+-" << std::string(5, '-') << "+-"
              << std::string(23, '-') << "+-" << std::string(6, '-') << "+-"
              << std::string(6, '-') << '\n';
    if (anode)
    {
        std::cout << std::setw(12) << "Anode" << "| "
                  << std::setw(20) << anode->name() << "| "
                  << std::setw(6) << anode->symbol() << "| "
                  << std::setw(5) << blockToStr(anode->block()) << "| "
                  << std::setw(23) << anode->groupLabel() << "| "
                  << std::setw(6) << anode->period() << "| "
                  << std::setw(6) << anode->groupNumber() << '\n';
    }
    if (carrier)
    {
        std::cout << std::setw(12) << "Carrier" << "| "
                  << std::setw(20) << carrier->name() << "| "
                  << std::setw(6) << carrier->symbol() << "| "
                  << std::setw(5) << blockToStr(carrier->block()) << "| "
                  << std::setw(23) << carrier->groupLabel() << "| "
                  << std::setw(6) << carrier->period() << "| "
                  << std::setw(6) << carrier->groupNumber() << '\n';
    }
    std::cout << std::setw(12) << "Cathode" << "| "
              << std::setw(20) << "Layered oxide" << "| "
              << std::setw(6) << "n/a" << "| "
              << std::setw(5) << "n/a" << "| "
              << std::setw(23) << "n/a" << "| "
              << std::setw(6) << "n/a" << "| "
              << std::setw(6) << "n/a" << '\n';

    // Build tensor grid (7x18xFeatureDim). Useful for GPU/GEMM pipelines.
    const auto grid = PeriodicTableTensor<8>::build(table);

    // Use the grid to derive physics-informed scalars that tune SPM params.
    // Simple example: a linear projection (w · features) over the anode cell.
    double anodeScore = 0.0;
    if (anode)
    {
        const int r = anode->period() - 1;
        const int c = anode->groupNumber() - 1;
        if (r >= 0 && r < PeriodicTable::kRows && c >= 0 && c < PeriodicTable::kCols)
        {
            const auto &fv = grid[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            constexpr std::array<double, 8> w{0.10, 0.30, -0.05, 0.10, -0.05, 0.20, 0.10, 0.20};
            for (std::size_t i = 0; i < w.size(); ++i)
                anodeScore += w[i] * fv[i];
            if (anodeScore < 0.0)
                anodeScore = 0.0;
            if (anodeScore > 1.0)
                anodeScore = 1.0;
        }
    }

    // --- Real-time KPI demo using SPM (simplified) ---
    using namespace battery::battery_models;
    SPMParameters spmParams;
    spmParams.capacityAh = 3.0 * (1.0 + 0.10 * anodeScore);
    spmParams.nominalVoltage = 3.7 + 0.05 * anodeScore;
    spmParams.internalResistanceOhm = 0.05 * (1.0 - 0.20 * anodeScore);
    SingleParticleModel spm(spmParams);
    const double initSoc = 0.9;
    spm.reset(initSoc); // start at 90% SoC

    // Print model parameter summary
    std::cout << "\n=== Model Parameters (SPM) ===\n";
    std::cout << std::left
              << std::setw(26) << "Parameter" << "| Value" << '\n'
              << std::string(26, '-') << "+" << std::string(1, '-') << std::string(6, '-') << '\n';
    std::cout << std::setw(26) << "Capacity (Ah)" << "| " << std::fixed << std::setprecision(3) << spmParams.capacityAh << '\n';
    std::cout << std::setw(26) << "Nominal voltage (V)" << "| " << std::setprecision(3) << spmParams.nominalVoltage << '\n';
    std::cout << std::setw(26) << "Internal resistance (Ω)" << "| " << std::setprecision(4) << spmParams.internalResistanceOhm << '\n';
    const double currentA = spmParams.capacityAh; // 1C
    std::cout << std::setw(26) << "Discharge current (A)" << "| " << std::setprecision(3) << currentA << '\n';
    std::cout << std::setw(26) << "Initial SoC" << "| " << std::setprecision(3) << initSoc << '\n';

    // KPI intro and table header
    std::cout << "\n=== Realtime KPIs (1C Discharge) ===\n";
    std::cout << "SoC = State of Charge (fraction of nominal capacity remaining).\n"
                 "At 1C, current equals capacity (A), so SoC decreases ~1% per 36 s.\n"
                 "These KPIs illustrate: terminal Voltage sag (OCV - I·R), instantaneous Power (V·I),\n"
                 "and accumulated Energy (Wh) produced over time as the cell discharges.\n\n";
    std::cout << std::left
              << std::setw(6) << "t (s)" << "| "
              << std::setw(12) << "Voltage (V)" << "| "
              << std::setw(13) << "Current (A)" << "| "
              << std::setw(9) << "SoC (%)" << "| "
              << std::setw(11) << "Power (W)" << "| "
              << std::setw(12) << "Energy (Wh)" << '\n'
              << std::string(6, '-') << "+-" << std::string(12, '-') << "+-"
              << std::string(13, '-') << "+-" << std::string(9, '-') << "+-"
              << std::string(11, '-') << "+-" << std::string(12, '-') << '\n';
    for (int i = 0; i < 10; ++i)
    {
        auto out = spm.step(currentA, 1.0);
        std::cout << std::right << std::setw(6) << (i + 1) << "| "
                  << std::fixed << std::setprecision(5) << std::setw(12) << out.voltageV << "| "
                  << std::setprecision(5) << std::setw(13) << out.currentA << "| "
                  << std::setprecision(2) << std::setw(9) << (out.soc * 100.0) << "| "
                  << std::setprecision(4) << std::setw(11) << out.powerW << "| "
                  << std::setprecision(6) << std::setw(12) << out.energyWh << '\n';
    }

    return 0;
}