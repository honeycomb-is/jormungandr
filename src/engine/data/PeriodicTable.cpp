// Periodic table implementation
#include "engine/data/PeriodicTable.hpp"

#include <fstream>

namespace Engine::Data
{
// Define a default assets root if the build didn't provide one
#ifndef HONEYCOMB_ASSETS_DIR
#define HONEYCOMB_ASSETS_DIR "assets"
#endif

    static std::string ResolvePath()
    {
        return std::string(HONEYCOMB_ASSETS_DIR) + "/data/pt.json";
    }

    static PeriodicTable BuildTable()
    {
        PeriodicTable pt;

        // Load JSON array from file
        std::ifstream f(ResolvePath());
        json arr = json::array();
        if (f)
        {
            try
            {
                f >> arr;
            }
            catch (...)
            {
                arr = json::array();
            }
        }

        pt.elements.reserve(arr.size());
        pt.byAtomicNumber.reserve(128);
        pt.bySymbol.reserve(140);

        for (const auto &e : arr)
            pt.elements.push_back(e);

        for (auto &e : pt.elements)
        {
            int Z = e.value("AtomicNumber", 0);
            if (Z > 0)
                pt.byAtomicNumber.emplace(Z, &e);
            std::string sym = e.value("Symbol", std::string{});
            if (!sym.empty())
                pt.bySymbol.emplace(sym, &e);
        }

        return pt;
    }

    const PeriodicTable &GetPeriodicTable()
    {
        static PeriodicTable pt = BuildTable();
        return pt;
    }

} // namespace Engine::Data
