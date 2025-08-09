// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <string_view>

namespace battery::chem::data
{

    // Minimal CSV dataset for demo (can be extended to full 118 later)
    // Header: Z,symbol,name,atomic_mass_u,standard_state,electron_config,oxidation_states,electronegativity_Pauling,vdw_radius_pm,ionization_eV,electron_affinity_eV,melting_K,boiling_K
    inline constexpr const char kElementsCSV[] = R"(Z,symbol,name,atomic_mass_u,standard_state,block,electron_config,oxidation_states,electronegativity_Pauling,vdw_radius_pm,ionization_eV,electron_affinity_eV,melting_K,boiling_K,density_g_cm3
1,H,Hydrogen,1.008,gas,s,1s1,1 -1,2.20,120,13.598,0.754,13.81,20.28,0.00008988
3,Li,Lithium,7.0,solid,s,[He]2s1,1,0.98,182,5.392,0.618,453.65,1615,0.534
6,C,Carbon,12.011,solid,p,[He]2s2 2p2,4 2 -4,2.55,170,11.260,1.263,3823,4098,2.267
8,O,Oxygen,15.999,gas,p,[He]2s2 2p4,-2 -1 1 2,3.44,152,13.618,1.461,54.36,90.20,0.001429
11,Na,Sodium,22.989,solid,s,[Ne]3s1,1,0.93,227,5.139,0.548,370.944,1156.09,0.968
12,Mg,Magnesium,24.305,solid,s,[Ne]3s2,2,1.31,173,7.646,0.000,923,1363,1.738
13,Al,Aluminum,26.982,solid,p,[Ne]3s2 3p1,3,1.61,184,5.986,0.440,933.47,2792,2.70
19,K,Potassium,39.098,solid,s,[Ar]4s1,1,0.82,275,4.341,0.501,336.53,1032,0.862
29,Cu,Copper,63.546,solid,d,[Ar]3d10 4s1,2 1,1.90,140,7.726,1.235,1357.77,2835,8.96
)";

} // namespace battery::chem::data
