// SPDX-License-Identifier: AGPL-3.0-or-later
#include "honeycomb/chem/reaction.hpp"

#include <cmath>
#include <sstream>

namespace honeycomb::chem
{
    static inline bool is_alkali(int group) { return group == 1; }
    static inline bool is_alkaline_earth(int group) { return group == 2; }
    static inline bool is_halogen(int group) { return group == 17; }
    static inline bool is_chalcogen(int group) { return group == 16; }

    static inline double arrhenius(double A, double Ea_kJ_per_mol, double T_K)
    {
        // k = A * exp(-Ea / (R*T)); R = 0.008314 kJ/(mol*K)
        const double R = 0.008314;
        return A * std::exp(-Ea_kJ_per_mol / (R * T_K));
    }

    ReactionResult predict_reaction(const periodic_table &table, const ReactionInput &in)
    {
        ReactionResult out{};
        if (in.Z_a < 1 || in.Z_b < 1 || in.Z_a > (int)table.size() || in.Z_b > (int)table.size())
        {
            out.rationale = "Invalid atomic numbers";
            return out;
        }
        const Element &A = table.by_atomic_number(in.Z_a);
        const Element &B = table.by_atomic_number(in.Z_b);

        // Heuristic 1: Alkali/alkaline earth + halogen => salt (e.g., Na + Cl -> NaCl)
        int gA = A.groupNumber();
        int gB = B.groupNumber();
        const Element *metal = nullptr;
        const Element *nonmetal = nullptr;

        if ((is_alkali(gA) || is_alkaline_earth(gA)) && is_halogen(gB))
        {
            metal = &A;
            nonmetal = &B;
        }
        if ((is_alkali(gB) || is_alkaline_earth(gB)) && is_halogen(gA))
        {
            metal = &B;
            nonmetal = &A;
        }

        if (metal && nonmetal)
        {
            // Stoichiometry: group 1 -> +1, group 2 -> +2; halogen -> -1
            int metal_charge = (metal->groupNumber() == 1) ? +1 : (metal->groupNumber() == 2 ? +2 : +1);
            int hal_charge = -1;
            int m = std::abs(hal_charge);
            int n = std::abs(metal_charge);

            std::ostringstream f;
            f << metal->symbol();
            if (n > 1)
                f << n;
            f << nonmetal->symbol();
            if (m > 1)
                f << m;

            // Rough energetics: favorable negative delta G if electronegativity difference large
            double deltaEN = std::abs(metal->electronegativityPauling() - nonmetal->electronegativityPauling());
            out.deltaG_kJ_per_mol = -50.0 * deltaEN; // crude
            out.k_forward = arrhenius(1e12, 20.0, in.temperature_K);
            // k_reverse via Keq = exp(-ΔG/RT)
            const double R = 0.008314;
            double Keq = std::exp(-out.deltaG_kJ_per_mol / (R * in.temperature_K));
            out.k_reverse = (Keq > 0.0) ? (out.k_forward / Keq) : 0.0;
            out.nuA = n;
            out.nuB = m;
            out.products = {ReactionProduct{f.str(), 1}};
            out.feasible = true;
            out.rationale = "Ionic salt formation (alkali/alkaline earth + halogen)";
            return out;
        }

        // Heuristic 2: Metal + chalcogen (O/S) -> oxide/sulfide (e.g., 2Na + O -> Na2O)
        if ((is_alkali(gA) || is_alkaline_earth(gA)) && is_chalcogen(gB))
        {
            metal = &A;
            nonmetal = &B;
        }
        if ((is_alkali(gB) || is_alkaline_earth(gB)) && is_chalcogen(gA))
        {
            metal = &B;
            nonmetal = &A;
        }
        if (metal && nonmetal)
        {
            int metal_charge = (metal->groupNumber() == 1) ? +1 : (metal->groupNumber() == 2 ? +2 : +1);
            int nonmetal_charge = (nonmetal->groupNumber() == 16) ? -2 : -2;
            int m = std::abs(nonmetal_charge);
            int n = std::abs(metal_charge);

            std::ostringstream f;
            f << metal->symbol();
            if (n > 1)
                f << n;
            f << nonmetal->symbol();
            if (m > 1)
                f << m;
            double deltaEN = std::abs(metal->electronegativityPauling() - nonmetal->electronegativityPauling());
            out.deltaG_kJ_per_mol = -40.0 * deltaEN;
            out.k_forward = arrhenius(5e11, 35.0, in.temperature_K);
            const double R2 = 0.008314;
            double Keq2 = std::exp(-out.deltaG_kJ_per_mol / (R2 * in.temperature_K));
            out.k_reverse = (Keq2 > 0.0) ? (out.k_forward / Keq2) : 0.0;
            out.nuA = n;
            out.nuB = m;
            out.products = {ReactionProduct{f.str(), 1}};
            out.feasible = true;
            out.rationale = "Oxide/sulfide formation (alkali/alkaline earth + chalcogen)";
            return out;
        }

        // Heuristic 3: Hydrogen + halogen -> hydrogen halide (HX)
        if ((A.symbol() == std::string("H") && is_halogen(gB)) || (B.symbol() == std::string("H") && is_halogen(gA)))
        {
            const Element *hal = is_halogen(gA) ? &A : &B;
            std::ostringstream f;
            f << "H" << hal->symbol();
            out.deltaG_kJ_per_mol = -30.0; // coarse favorable
            out.k_forward = arrhenius(8e11, 25.0, in.temperature_K);
            const double R3 = 0.008314;
            double Keq3 = std::exp(-out.deltaG_kJ_per_mol / (R3 * in.temperature_K));
            out.k_reverse = (Keq3 > 0.0) ? (out.k_forward / Keq3) : 0.0;
            out.nuA = 1;
            out.nuB = 1;
            out.products = {ReactionProduct{f.str(), 1}};
            out.feasible = true;
            out.rationale = "Hydrogen halide formation";
            return out;
        }

        // Default: no reaction predicted in this simple model
        out.feasible = false;
        out.rationale = "No reaction predicted by simple heuristics";
        out.deltaG_kJ_per_mol = 0.0;
        out.k_forward = 0.0;
        return out;
    }
}
