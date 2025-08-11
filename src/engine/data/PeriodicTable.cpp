// Periodic table implementation
#include "engine/data/PeriodicTable.hpp"
#include "engine/data/pt_data.hpp"

namespace Engine::Data
{
    static PeriodicTable BuildTable()
    {
        PeriodicTable pt;

        // Load JSON array from embedded header
        json arr = json::array();
        try
        {
            arr = json::parse(std::string_view(Engine::Data::PT_JSON, Engine::Data::PT_JSON_LEN));
        }
        catch (...)
        {
            arr = json::array();
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
