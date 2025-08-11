#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <unordered_map>

namespace Engine::Data
{

    using json = nlohmann::json;

    struct PeriodicTable
    {
        // Flat array of elements in Z order (index = Z-1)
        std::vector<json> elements;

        // Fast lookups
        std::unordered_map<int, const json *> byAtomicNumber;   // Z -> element json*
        std::unordered_map<std::string, const json *> bySymbol; // Symbol -> element json*
    };

    // Singleton accessor
    const PeriodicTable &GetPeriodicTable();

    // Convenience helpers (O(1) via unordered_map)
    inline const json *GetElementByZ(int atomicNumber)
    {
        const auto &pt = GetPeriodicTable();
        auto it = pt.byAtomicNumber.find(atomicNumber);
        return (it == pt.byAtomicNumber.end()) ? nullptr : it->second;
    }

    inline const json *GetElementBySymbol(const std::string &symbol)
    {
        const auto &pt = GetPeriodicTable();
        auto it = pt.bySymbol.find(symbol);
        return (it == pt.bySymbol.end()) ? nullptr : it->second;
    }

    inline const std::vector<json> &ElementsVec()
    {
        return GetPeriodicTable().elements;
    }

} // namespace Engine::Data