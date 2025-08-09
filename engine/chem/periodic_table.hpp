// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "engine/chem/element.hpp"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace battery::chem
{

    // Lightweight registry backed by contiguous array indexed by atomic number
    class PeriodicTable
    {
    public:
        // The periodic table currently defines up to 118 elements.
        static constexpr int kMaxZ = 118;

        const Element *getByZ(int z) const
        {
            if (z < 1 || z > kMaxZ)
                return nullptr;
            const Element &e = m_byZ[static_cast<std::size_t>(z)];
            return e.atomicNumber() == 0 ? nullptr : &e;
        }

        const Element *getBySymbol(std::string_view symbol) const
        {
            auto it = m_symbolToZ.find(std::string(symbol));
            if (it == m_symbolToZ.end())
                return nullptr;
            return getByZ(it->second);
        }

        void setElement(const Element &e)
        {
            const int z = e.atomicNumber();
            if (z < 1 || z > kMaxZ)
                return;
            m_byZ[static_cast<std::size_t>(z)] = e;
            m_symbolToZ[e.symbol()] = z;
        }

    private:
        // Index 0 unused so index == atomic number
        std::array<Element, kMaxZ + 1> m_byZ{};
        std::unordered_map<std::string, int> m_symbolToZ;
    };

    // ECS component: reference an element by atomic number for compact storage
    struct ElementRefComponent
    {
        int atomicNumber{0};
    };

} // namespace battery::chem
