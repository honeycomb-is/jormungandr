// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "honeycomb/chem/element.hpp"
#include <span>
#include <vector>

namespace honeycomb::chem
{

    // Immutable view over the periodic table (indexed by atomic number - 1)
    class periodic_table
    {
    public:
        periodic_table() = default;
        explicit periodic_table(std::vector<Element> elements)
            : m_elements(std::move(elements)) {}

        std::size_t size() const { return m_elements.size(); }
        const Element &by_index(std::size_t idx) const { return m_elements.at(idx); }
        const Element &by_atomic_number(int z) const { return m_elements.at(static_cast<std::size_t>(z - 1)); }

        const std::vector<Element> &all() const { return m_elements; }

    private:
        std::vector<Element> m_elements;
    };

} // namespace honeycomb::chem
