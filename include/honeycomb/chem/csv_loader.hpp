// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "honeycomb/chem/element.hpp"
#include <string>
#include <vector>

namespace honeycomb::chem
{

    // Parse periodic_table.csv into Element models. Robust to missing/empty fields.
    std::vector<Element> load_elements_from_csv(const std::string &csv_path);

} // namespace honeycomb::chem
