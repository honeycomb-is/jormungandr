// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "engine/chem/periodic_table.hpp"

#include <charconv>
#include <sstream>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>
#include <limits>

namespace battery::chem
{

    inline StandardState parseStandardState(std::string_view s)
    {
        if (s == "solid")
            return StandardState::Solid;
        if (s == "liquid" || s == "liq")
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

    inline Block parseBlockFromElectronConfig(std::string_view ec)
    {
        // Infer block from last subshell letter
        for (size_t i = ec.size(); i > 0; --i)
        {
            char ch = static_cast<char>(std::tolower(ec[i - 1]));
            if (ch == 's')
                return Block::S;
            if (ch == 'p')
                return Block::P;
            if (ch == 'd')
                return Block::D;
            if (ch == 'f')
                return Block::F;
        }
        return Block::Unknown;
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
    // Z,symbol,name,group,period,group_number,atomic_mass_u,standard_state,block,electron_config,oxidation_states,electronegativity_Pauling,vdw_radius_pm,ionization_eV,electron_affinity_eV,melting_K,boiling_K,density_g_cm3
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
            if (cols.size() < 18)
                continue;

            int z = std::stoi(cols[0]);
            std::string symbol = cols[1];
            std::string name = cols[2];
            std::string group = cols[3];
            int period = cols[4].empty() ? 0 : std::stoi(cols[4]);
            int groupNo = cols[5].empty() ? 0 : std::stoi(cols[5]);
            double mass = std::stod(cols[6]);
            StandardState state = parseStandardState(cols[7]);
            Block block = parseBlock(cols[8]);
            std::string econf = cols[9];
            auto ox = parseOxidationStates(cols[10]);
            double en = cols[11].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[11]);
            int vdw = cols[12].empty() ? 0 : std::stoi(cols[12]);
            double ion = cols[13].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[13]);
            double ea = cols[14].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[14]);
            double mp = cols[15].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[15]);
            double bp = cols[16].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[16]);
            double density = cols[17].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[17]);

            table.setElement(Element{z, std::move(symbol), std::move(name), std::move(group), period, groupNo, mass, state, block, std::move(econf), std::move(ox), en, vdw, ion, ea, mp, bp, density});
        }
    }

    inline void loadFromFile(PeriodicTable &table, const std::string &filename)
    {
        std::ifstream ifs(filename);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Failed to open elements CSV: " + filename);
        }
        // Peek header
        std::string header;
        std::getline(ifs, header);
        // Reset stream to beginning
        ifs.clear();
        ifs.seekg(0, std::ios::beg);

        if (header.rfind("Atomic Number,Element,Symbol", 0) == 0)
        {
            // Parse the external assets CSV schema
            std::string line;
            bool first = true;
            while (std::getline(ifs, line))
            {
                if (line.empty())
                    continue;
                if (first)
                {
                    first = false;
                    continue;
                }
                auto cols = splitCSVLine(line);
                if (cols.size() < 22)
                    continue;

                // Column indices in assets CSV
                // 0:Atomic Number,1:Element,2:Symbol,3:Atomic Weight,4:Period,5:Group
                // 6:Phase,7:Most Stable Crystal,8:Type,9:Ionic Radius,10:Atomic Radius,11:Electronegativity,
                // 12:First Ionization Potential,13:Density,14:Melting (K),15:Boiling (K), ... ,20:Electron Configuration
                try
                {
                    int z = std::stoi(cols[0]);
                    std::string name = cols[1];
                    std::string symbol = cols[2];
                    double mass = cols[3].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[3]);
                    int period = cols[4].empty() ? 0 : std::stoi(cols[4]);
                    int groupNo = cols[5].empty() ? 0 : std::stoi(cols[5]);
                    std::string phase = cols[6];
                    std::string groupLabel = cols[8];
                    double atomicRadiusA = cols[10].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[10]);
                    double en = cols[11].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[11]);
                    double ion = cols[12].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[12]);
                    double density = cols[13].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[13]);
                    double mp = cols[14].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[14]);
                    double bp = cols[15].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(cols[15]);
                    std::string econf = cols.size() > 20 ? cols[20] : std::string();

                    // Map/derive fields
                    StandardState state = parseStandardState(phase);
                    Block block = parseBlockFromElectronConfig(econf);
                    int vdwPm = 0;
                    if (!std::isnan(atomicRadiusA))
                        vdwPm = static_cast<int>(std::round(atomicRadiusA * 100.0)); // Å -> pm approx

                    std::vector<int> ox; // unknown in this dataset
                    table.setElement(Element{z, std::move(symbol), std::move(name), std::move(groupLabel), period, groupNo, mass, state, block, std::move(econf), std::move(ox), en, vdwPm, ion, std::numeric_limits<double>::quiet_NaN(), mp, bp, density});
                }
                catch (...)
                {
                    // Skip malformed row
                }
            }
        }
        else
        {
            // Fallback to internal schema
            std::string csv;
            csv.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
            loadElementsCSV(table, csv);
        }
    }

} // namespace battery::chem
