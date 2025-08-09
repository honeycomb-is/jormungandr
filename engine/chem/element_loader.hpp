// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "engine/chem/periodic_table.hpp"

#include <charconv>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace battery::chem
{

    inline StandardState parseStandardState(std::string_view s)
    {
        if (s == "solid")
            return StandardState::Solid;
        if (s == "liquid")
            return StandardState::Liquid;
        if (s == "gas")
            return StandardState::Gas;
        return StandardState::Unknown;
    }

    inline Block parseBlock(std::string_view s)
    {
        if (s == "s")
            return Block::S;
        if (s == "p")
            return Block::P;
        if (s == "d")
            return Block::D;
        if (s == "f")
            return Block::F;
        return Block::Unknown;
    }

    inline std::vector<std::string> splitCSVLine(std::string_view line)
    {
        std::vector<std::string> cols;
        cols.reserve(16);
        std::string current;
        bool inQuotes = false;
        for (size_t i = 0; i < line.size(); ++i)
        {
            char c = line[i];
            if (c == '"')
            {
                inQuotes = !inQuotes;
                continue;
            }
            if (c == ',' && !inQuotes)
            {
                cols.emplace_back(std::move(current));
                current.clear();
                continue;
            }
            current.push_back(c);
        }
        cols.emplace_back(std::move(current));
        return cols;
    }

    inline std::vector<int> parseOxidationStates(std::string_view s)
    {
        std::vector<int> out;
        std::string token;
        std::istringstream iss{std::string(s)};
        while (std::getline(iss, token, ' '))
        {
            if (token.empty())
                continue;
            try
            {
                out.push_back(std::stoi(token));
            }
            catch (...)
            {
            }
        }
        return out;
    }

    // Minimal CSV loader: expects header and columns:
    // Z,symbol,name,atomic_mass_u,standard_state,block,electron_config,oxidation_states,electronegativity_Pauling,vdw_radius_pm,ionization_eV,electron_affinity_eV,melting_K,boiling_K,density_g_cm3
    inline void loadElementsCSV(PeriodicTable &table, std::string_view csv)
    {
        std::istringstream ss{std::string(csv)};
        std::string line;
        bool firstLine = true;
        while (std::getline(ss, line))
        {
            if (line.empty())
                continue;
            if (firstLine)
            {
                firstLine = false;
                continue;
            }
            auto cols = splitCSVLine(line);
            if (cols.size() < 15)
                continue;

            int z = std::stoi(cols[0]);
            std::string symbol = cols[1];
            std::string name = cols[2];
            double mass = std::stod(cols[3]);
            StandardState state = parseStandardState(cols[4]);
            Block block = parseBlock(cols[5]);
            std::string econf = cols[6];
            auto ox = parseOxidationStates(cols[7]);
            double en = cols[8].empty() ? 0.0 : std::stod(cols[8]);
            int vdw = cols[9].empty() ? 0 : std::stoi(cols[9]);
            double ion = cols[10].empty() ? 0.0 : std::stod(cols[10]);
            double ea = cols[11].empty() ? 0.0 : std::stod(cols[11]);
            double mp = cols[12].empty() ? 0.0 : std::stod(cols[12]);
            double bp = cols[13].empty() ? 0.0 : std::stod(cols[13]);
            double density = cols[14].empty() ? 0.0 : std::stod(cols[14]);

            table.setElement(Element{z, std::move(symbol), std::move(name), mass, state, block, std::move(econf), std::move(ox), en, vdw, ion, ea, mp, bp, density});
        }
    }

} // namespace battery::chem
