#pragma once

#include <string>
#include <vector>

#include "Components.hpp"
#include "engine/data/PeriodicTable.hpp"

namespace Engine::ECS
{
    // Minimal optional-like helper without relying on std::optional
    template <typename T>
    struct Opt
    {
        bool present{false};
        T value{};
    };
    // A convenient bundle for conditionally-present element components
    struct ElementComponentSet
    {
        Opt<ElementAtomicNumber> atomicNumber;
        Opt<ElementSymbol> symbol;
        Opt<ElementName> name;
        Opt<ElementAtomicMassU> atomicMassU;
        Opt<ElementCPKHexColor> cpkHexColor;
        Opt<ElementElectronConfiguration> electronConfiguration;
        Opt<ElementElectronegativityPauling> electronegativityPauling;
        Opt<ElementAtomicRadiusVdW> atomicRadiusVdW;
        Opt<ElementIonizationEnergyEV> ionizationEnergyEV;
        Opt<ElementElectronAffinityEV> electronAffinityEV;
        Opt<ElementOxidationStates> oxidationStates;
        Opt<ElementStandardState> standardState;
        Opt<ElementMeltingPointK> meltingPointK;
        Opt<ElementBoilingPointK> boilingPointK;
        Opt<ElementDensityGPerCm3> densityGPerCm3;
        Opt<ElementGroupBlock> groupBlock;
        Opt<ElementYearDiscovered> yearDiscovered;
    };

    inline bool getFloat(const Engine::Data::json &e, const char *key, float &out)
    {
        if (!e.contains(key) || e[key].is_null())
            return false;
        if (e[key].is_number_float())
        {
            out = e[key].get<float>();
            return true;
        }
        if (e[key].is_number_integer())
        {
            out = static_cast<float>(e[key].get<long long>());
            return true;
        }
        return false;
    }

    inline bool getString(const Engine::Data::json &e, const char *key, std::string &out)
    {
        if (!e.contains(key) || e[key].is_null())
            return false;
        if (e[key].is_string())
        {
            out = e[key].get<std::string>();
            return true;
        }
        return false;
    }

    inline bool getOxidationStates(const Engine::Data::json &e, std::vector<int> &out)
    {
        constexpr const char *key = "OxidationStates";
        if (!e.contains(key) || e[key].is_null() || !e[key].is_array())
            return false;
        out.clear();
        out.reserve(e[key].size());
        for (const auto &v : e[key])
        {
            if (v.is_string())
            {
                const std::string s = v.get<std::string>();
                int sign = 1;
                size_t idx = 0;
                if (!s.empty() && (s[0] == '+' || s[0] == '-'))
                {
                    sign = (s[0] == '-') ? -1 : 1;
                    idx = 1;
                }
                try
                {
                    int mag = std::stoi(s.substr(idx));
                    out.push_back(sign * mag);
                }
                catch (...)
                { /* ignore */
                }
            }
            else if (v.is_number_integer())
            {
                out.push_back(static_cast<int>(v.get<long long>()));
            }
        }
        return !out.empty();
    }

    // Build a component set from a periodic-table JSON element object
    inline ElementComponentSet BuildElementComponents(const Engine::Data::json &e)
    {
        ElementComponentSet s;

        if (e.contains("AtomicNumber") && e["AtomicNumber"].is_number_integer())
        {
            s.atomicNumber.present = true;
            s.atomicNumber.value = ElementAtomicNumber{e["AtomicNumber"].get<int>()};
        }

        {
            std::string sym;
            if (getString(e, "Symbol", sym))
            {
                s.symbol.present = true;
                s.symbol.value = ElementSymbol{sym};
            }
        }

        {
            std::string name;
            if (getString(e, "Name", name))
            {
                s.name.present = true;
                s.name.value = ElementName{name};
            }
        }

        {
            float v;
            if (getFloat(e, "AtomicMass(u)", v))
            {
                s.atomicMassU.present = true;
                s.atomicMassU.value = ElementAtomicMassU{v};
            }
        }

        {
            std::string c;
            if (getString(e, "CPKHexColor", c))
            {
                s.cpkHexColor.present = true;
                s.cpkHexColor.value = ElementCPKHexColor{c};
            }
        }

        {
            std::string cfg;
            if (getString(e, "ElectronConfiguration", cfg))
            {
                s.electronConfiguration.present = true;
                s.electronConfiguration.value = ElementElectronConfiguration{cfg};
            }
        }

        {
            float v;
            if (getFloat(e, "Electronegativity(Pauling)", v))
            {
                s.electronegativityPauling.present = true;
                s.electronegativityPauling.value = ElementElectronegativityPauling{v};
            }
        }

        {
            float v;
            if (getFloat(e, "AtomicRadius(vdWaals)", v))
            {
                s.atomicRadiusVdW.present = true;
                s.atomicRadiusVdW.value = ElementAtomicRadiusVdW{v};
            }
        }

        {
            float v;
            if (getFloat(e, "IonizationEnergy(eV)", v))
            {
                s.ionizationEnergyEV.present = true;
                s.ionizationEnergyEV.value = ElementIonizationEnergyEV{v};
            }
        }

        {
            float v;
            if (getFloat(e, "ElectronAffinity(eV)", v))
            {
                s.electronAffinityEV.present = true;
                s.electronAffinityEV.value = ElementElectronAffinityEV{v};
            }
        }

        {
            std::vector<int> ox;
            if (getOxidationStates(e, ox))
            {
                s.oxidationStates.present = true;
                s.oxidationStates.value = ElementOxidationStates{ox};
            }
        }

        {
            std::string v;
            if (getString(e, "StandardState", v))
            {
                s.standardState.present = true;
                s.standardState.value = ElementStandardState{v};
            }
        }

        {
            float v;
            if (getFloat(e, "MeltingPoint(K)", v))
            {
                s.meltingPointK.present = true;
                s.meltingPointK.value = ElementMeltingPointK{v};
            }
        }

        {
            float v;
            if (getFloat(e, "BoilingPoint(K)", v))
            {
                s.boilingPointK.present = true;
                s.boilingPointK.value = ElementBoilingPointK{v};
            }
        }

        {
            float v;
            if (getFloat(e, "Density(g/cm3)", v))
            {
                s.densityGPerCm3.present = true;
                s.densityGPerCm3.value = ElementDensityGPerCm3{v};
            }
        }

        {
            std::string v;
            if (getString(e, "GroupBlock", v))
            {
                s.groupBlock.present = true;
                s.groupBlock.value = ElementGroupBlock{v};
            }
        }

        {
            std::string v;
            if (getString(e, "YearDiscovered", v))
            {
                s.yearDiscovered.present = true;
                s.yearDiscovered.value = ElementYearDiscovered{v};
            }
        }

        return s;
    }

} // namespace Engine::ECS
