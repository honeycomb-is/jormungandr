// SPDX-License-Identifier: AGPL-3.0-or-later
#include "honeycomb/chem/csv_loader.hpp"

#include <charconv>
#include <cstddef>
#include <fstream>
#include <sstream>

namespace honeycomb::chem
{

    namespace detail
    {
        static inline int derive_period(int Z)
        {
            if (Z < 1)
                return 0;
            if (Z <= 2)
                return 1;
            if (Z <= 10)
                return 2;
            if (Z <= 18)
                return 3;
            if (Z <= 36)
                return 4;
            if (Z <= 54)
                return 5;
            if (Z <= 86)
                return 6;
            if (Z <= 118)
                return 7;
            return 0;
        }

        static inline int derive_group(int Z)
        {
            static const int groups[119] = {
                /*0*/ 0,
                /*1 H*/ 1, /*2 He*/ 18,
                /*3 Li*/ 1, /*4 Be*/ 2, /*5 B*/ 13, /*6 C*/ 14, /*7 N*/ 15, /*8 O*/ 16, /*9 F*/ 17, /*10 Ne*/ 18,
                /*11 Na*/ 1, /*12 Mg*/ 2, /*13 Al*/ 13, /*14 Si*/ 14, /*15 P*/ 15, /*16 S*/ 16, /*17 Cl*/ 17, /*18 Ar*/ 18,
                /*19 K*/ 1, /*20 Ca*/ 2, /*21 Sc*/ 3, /*22 Ti*/ 4, /*23 V*/ 5, /*24 Cr*/ 6, /*25 Mn*/ 7, /*26 Fe*/ 8,
                /*27 Co*/ 9, /*28 Ni*/ 10, /*29 Cu*/ 11, /*30 Zn*/ 12, /*31 Ga*/ 13, /*32 Ge*/ 14, /*33 As*/ 15,
                /*34 Se*/ 16, /*35 Br*/ 17, /*36 Kr*/ 18,
                /*37 Rb*/ 1, /*38 Sr*/ 2, /*39 Y*/ 3, /*40 Zr*/ 4, /*41 Nb*/ 5, /*42 Mo*/ 6, /*43 Tc*/ 7, /*44 Ru*/ 8,
                /*45 Rh*/ 9, /*46 Pd*/ 10, /*47 Ag*/ 11, /*48 Cd*/ 12, /*49 In*/ 13, /*50 Sn*/ 14, /*51 Sb*/ 15,
                /*52 Te*/ 16, /*53 I*/ 17, /*54 Xe*/ 18,
                /*55 Cs*/ 1, /*56 Ba*/ 2, /*57 La*/ 3, /*58 Ce*/ 3, /*59 Pr*/ 3, /*60 Nd*/ 3, /*61 Pm*/ 3, /*62 Sm*/ 3,
                /*63 Eu*/ 3, /*64 Gd*/ 3, /*65 Tb*/ 3, /*66 Dy*/ 3, /*67 Ho*/ 3, /*68 Er*/ 3, /*69 Tm*/ 3, /*70 Yb*/ 3,
                /*71 Lu*/ 3, /*72 Hf*/ 4, /*73 Ta*/ 5, /*74 W*/ 6, /*75 Re*/ 7, /*76 Os*/ 8, /*77 Ir*/ 9, /*78 Pt*/ 10,
                /*79 Au*/ 11, /*80 Hg*/ 12, /*81 Tl*/ 13, /*82 Pb*/ 14, /*83 Bi*/ 15, /*84 Po*/ 16, /*85 At*/ 17, /*86 Rn*/ 18,
                /*87 Fr*/ 1, /*88 Ra*/ 2, /*89 Ac*/ 3, /*90 Th*/ 3, /*91 Pa*/ 3, /*92 U*/ 3, /*93 Np*/ 3, /*94 Pu*/ 3,
                /*95 Am*/ 3, /*96 Cm*/ 3, /*97 Bk*/ 3, /*98 Cf*/ 3, /*99 Es*/ 3, /*100 Fm*/ 3, /*101 Md*/ 3, /*102 No*/ 3,
                /*103 Lr*/ 3, /*104 Rf*/ 4, /*105 Db*/ 5, /*106 Sg*/ 6, /*107 Bh*/ 7, /*108 Hs*/ 8, /*109 Mt*/ 9,
                /*110 Ds*/ 10, /*111 Rg*/ 11, /*112 Cn*/ 12, /*113 Nh*/ 13, /*114 Fl*/ 14, /*115 Mc*/ 15, /*116 Lv*/ 16,
                /*117 Ts*/ 17, /*118 Og*/ 18};
            if (Z < 1 || Z > 118)
                return 0;
            return groups[Z];
        }
        static inline std::vector<std::string> parse_csv_line(const std::string &line)
        {
            std::vector<std::string> fields;
            std::string field;
            bool in_quotes = false;
            auto trim_ws = [](std::string &s)
            {
                const char *ws = " \t\n\r\f\v";
                if (!s.empty())
                {
                    s.erase(0, s.find_first_not_of(ws));
                    s.erase(s.find_last_not_of(ws) + 1);
                }
            };
            for (std::size_t i = 0; i < line.size(); ++i)
            {
                char c = line[i];
                if (c == '"')
                {
                    if (in_quotes && i + 1 < line.size() && line[i + 1] == '"')
                    {
                        // Escaped quote inside a quoted field
                        field.push_back('"');
                        ++i; // skip the second quote
                    }
                    else
                    {
                        in_quotes = !in_quotes;
                    }
                }
                else if (c == ',' && !in_quotes)
                {
                    fields.emplace_back(std::move(field));
                    field.clear();
                }
                else
                {
                    field.push_back(c);
                }
            }
            fields.emplace_back(std::move(field));
            // Trim outer quotes if any
            for (auto &f : fields)
            {
                trim_ws(f);
                if (f.size() >= 2 && f.front() == '"' && f.back() == '"')
                {
                    f = f.substr(1, f.size() - 2);
                }
            }
            return fields;
        }
        static inline std::string trim(std::string s)
        {
            const char *ws = " \t\n\r\f\v";
            s.erase(0, s.find_first_not_of(ws));
            s.erase(s.find_last_not_of(ws) + 1);
            return s;
        }

        static inline StandardState parse_state(const std::string &s)
        {
            if (s == "Solid")
                return StandardState::Solid;
            if (s == "Liquid")
                return StandardState::Liquid;
            if (s == "Gas")
                return StandardState::Gas;
            return StandardState::Unknown;
        }

        static inline Block parse_block_from_config(const std::string &config)
        {
            // Heuristic: determine block by highest subshell letter present
            // s, p, d, f in electron configuration
            if (config.find('f') != std::string::npos || config.find('F') != std::string::npos)
                return Block::F;
            if (config.find('d') != std::string::npos || config.find('D') != std::string::npos)
                return Block::D;
            if (config.find('p') != std::string::npos || config.find('P') != std::string::npos)
                return Block::P;
            if (config.find('s') != std::string::npos || config.find('S') != std::string::npos)
                return Block::S;
            return Block::Unknown;
        }

        static inline std::vector<int> parse_oxidation_states(const std::string &s)
        {
            std::vector<int> out;
            std::stringstream ss(s);
            std::string tok;
            while (std::getline(ss, tok, ','))
            {
                tok = trim(tok);
                if (tok.empty())
                    continue;
                try
                {
                    out.push_back(std::stoi(tok));
                }
                catch (...)
                {
                    // ignore malformed entries
                }
            }
            return out;
        }
    }

    std::vector<Element> load_elements_from_csv(const std::string &csv_path)
    {
        std::ifstream file(csv_path);
        std::vector<Element> elements;
        if (!file.is_open())
            return elements;

        std::string line;
        // Skip header
        if (!std::getline(file, line))
            return elements;

        while (std::getline(file, line))
        {
            if (detail::trim(line).empty())
                continue;
            auto cols = detail::parse_csv_line(line);

            // Columns per assets/data/periodic_table.csv
            // 0: AtomicNumber
            // 1: Symbol
            // 2: Name
            // 3: AtomicMass(u)
            // 4: CPKHexColor
            // 5: ElectronConfiguration
            // 6: Electronegativity(Pauling)
            // 7: AtomicRadius(vdWaals)
            // 8: IonizationEnergy(eV)
            // 9: ElectronAffinity(eV)
            // 10: OxidationStates
            // 11: StandardState
            // 12: MeltingPoint(K)
            // 13: BoilingPoint(K)
            // 14: Density(g/cm3)
            // 15: GroupBlock
            // 16: YearDiscovered

            auto get = [&](std::size_t idx) -> std::string
            {
                return idx < cols.size() ? cols[idx] : std::string{};
            };

            std::string sZ = get(0);
            std::string sym = get(1);
            std::string name = get(2);
            std::string sMass = get(3);
            std::string cpk = get(4);
            std::string config = get(5);
            std::string sEN = get(6);
            std::string sVdw = get(7);
            std::string sIE = get(8);
            std::string sEA = get(9);
            std::string ox = get(10);
            std::string state = get(11);
            std::string sMp = get(12);
            std::string sBp = get(13);
            std::string sDensity = get(14);
            std::string groupBlock = get(15);
            std::string year = get(16);

            auto to_double = [](const std::string &s) -> double
            {
                if (s.empty())
                    return 0.0;
                try
                {
                    return std::stod(s);
                }
                catch (...)
                {
                    return 0.0;
                }
            };
            auto to_int = [](const std::string &s) -> int
            {
                if (s.empty())
                    return 0;
                try
                {
                    return std::stoi(s);
                }
                catch (...)
                {
                    return 0;
                }
            };

            int Z = to_int(sZ);
            double mass = to_double(sMass);
            double en = sEN.empty() ? 0.0 : to_double(sEN);
            int vdw = sVdw.empty() ? 0 : to_int(sVdw);
            double ie = sIE.empty() ? 0.0 : to_double(sIE);
            double ea = sEA.empty() ? 0.0 : to_double(sEA);
            double mp = sMp.empty() ? 0.0 : to_double(sMp);
            double bp = sBp.empty() ? 0.0 : to_double(sBp);
            double density = sDensity.empty() ? 0.0 : to_double(sDensity);

            int period = detail::derive_period(Z);
            int group = detail::derive_group(Z);

            auto elem = Element(
                Z,
                sym,
                name,
                cpk,
                groupBlock, /* groupLabel */
                period,
                group,
                mass,
                detail::parse_state(state),
                detail::parse_block_from_config(config),
                config,
                detail::parse_oxidation_states(ox),
                en,
                vdw,
                ie,
                ea,
                mp,
                bp,
                density,
                year);

            elements.emplace_back(std::move(elem));
        }

        return elements;
    }

} // namespace honeycomb::chem
