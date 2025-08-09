// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <bitset>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

namespace battery::chem
{

    enum class Spin : uint8_t
    {
        Alpha = 0,
        Beta = 1
    };

    struct SpinOrbital
    {
        int n;     // principal quantum number (n >= 1)
        int l;     // azimuthal quantum number (0 = s, 1 = p, 2 = d, ...), 0 <= l < n
        int m;     // magnetic quantum number (-l <= m <= l)
        Spin spin; // spin state (alpha/beta)
    };

    // Compile-time indexer: maps (n, l, m, spin) -> unique bit index
    template <int MaxN>
    struct SpinOrbitalIndexer
    {
        static_assert(MaxN >= 1, "MaxN must be >= 1");
        // Total number of spin-orbitals up to principal quantum number MaxN
        // 2 * sum_{n=1..MaxN} n^2 = MaxN*(MaxN+1)*(2*MaxN+1)/3
        static constexpr int kTotalBits = (MaxN * (MaxN + 1) * (2 * MaxN + 1)) / 3;

        static constexpr bool isValidQuantumNumbers(int n, int l, int m)
        {
            if (n < 1 || n > MaxN)
                return false;
            if (l < 0 || l >= n)
                return false;
            if (m < -l || m > l)
                return false;
            return true;
        }

        // Base offset for given n: 2 * sum_{k=1}^{n-1} k^2 = (n-1)*n*(2n-1)/3
        static constexpr int baseForN(int n)
        {
            const int nm1 = n - 1;
            return (nm1 * n * (2 * n - 1)) / 3;
        }

        // Base offset for given (n, l): add 2 * sum_{l'=0}^{l-1} (2*l' + 1) = 2 * l^2
        static constexpr int baseForNAndL(int n, int l)
        {
            return baseForN(n) + 2 * (l * l);
        }

        // Index within given (n, l, m, spin): each m has 2 spins
        static constexpr int index(int n, int l, int m, Spin spin)
        {
            const int base_nl = baseForNAndL(n, l);
            const int mOffset = 2 * (m + l); // maps m in [-l..l] -> [0..2l], then *2 for spins
            const int spinBit = static_cast<int>(spin);
            return base_nl + mOffset + spinBit;
        }
    };

    template <int MaxN>
    class ElectronOccupancy
    {
    public:
        using Indexer = SpinOrbitalIndexer<MaxN>;
        static constexpr int kNumBits = Indexer::kTotalBits;
        using Bitset = std::bitset<kNumBits>;

        ElectronOccupancy() = default;

        ElectronOccupancy(std::initializer_list<SpinOrbital> list)
        {
            for (const auto &so : list)
                set(so);
        }

        void set(const SpinOrbital &so)
        {
            validate(so);
            bits.set(static_cast<std::size_t>(Indexer::index(so.n, so.l, so.m, so.spin)));
        }

        void reset(const SpinOrbital &so)
        {
            validate(so);
            bits.reset(static_cast<std::size_t>(Indexer::index(so.n, so.l, so.m, so.spin)));
        }

        void toggle(const SpinOrbital &so)
        {
            validate(so);
            bits.flip(Indexer::index(so.n, so.l, so.m, so.spin));
        }

        bool test(const SpinOrbital &so) const
        {
            validate(so);
            return bits.test(static_cast<std::size_t>(Indexer::index(so.n, so.l, so.m, so.spin)));
        }

        int countElectrons() const { return static_cast<int>(bits.count()); }

        const Bitset &raw() const { return bits; }

        // Utilities
        static int differingSpinOrbitals(const ElectronOccupancy &a, const ElectronOccupancy &b)
        {
            return static_cast<int>((a.bits ^ b.bits).count());
        }

        static bool isSingleExcitation(const ElectronOccupancy &a, const ElectronOccupancy &b)
        {
            return differingSpinOrbitals(a, b) == 2;
        }

    private:
        Bitset bits{};

        static void validate(const SpinOrbital &so)
        {
            if (!Indexer::isValidQuantumNumbers(so.n, so.l, so.m))
            {
                throw std::invalid_argument("Invalid quantum numbers for spin-orbital");
            }
        }
    };

    // Small helpers to create spin-orbitals succinctly
    constexpr inline SpinOrbital so(int n, int l, int m, Spin s) { return SpinOrbital{n, l, m, s}; }

    // Compute electrons per subshell (n,l), and build spectroscopic notation
    template <int MaxN>
    int subshellElectronCount(const ElectronOccupancy<MaxN> &occ, int n, int l)
    {
        int count = 0;
        for (int m = -l; m <= l; ++m)
        {
            if (occ.test(so(n, l, m, Spin::Alpha)))
                ++count;
            if (occ.test(so(n, l, m, Spin::Beta)))
                ++count;
        }
        return count;
    }

    inline char spectroLetter(int l)
    {
        switch (l)
        {
        case 0:
            return 's';
        case 1:
            return 'p';
        case 2:
            return 'd';
        case 3:
            return 'f';
        case 4:
            return 'g';
        default:
            return '?';
        }
    }

    template <int MaxN>
    std::string spectroscopicNotation(const ElectronOccupancy<MaxN> &occ)
    {
        std::string out;
        bool first = true;
        for (int n = 1; n <= MaxN; ++n)
        {
            for (int l = 0; l <= n - 1; ++l)
            {
                const int electrons = subshellElectronCount(occ, n, l);
                if (electrons == 0)
                    continue;
                if (!first)
                    out.push_back(' ');
                first = false;
                out += std::to_string(n);
                out.push_back(spectroLetter(l));
                out += std::to_string(electrons);
            }
        }
        return out.empty() ? std::string("0") : out;
    }

    // Valence electrons: electrons in the highest-occupied principal shell
    template <int MaxN>
    int valenceElectronCount(const ElectronOccupancy<MaxN> &occ)
    {
        int highestNWithElectrons = 0;
        for (int n = 1; n <= MaxN; ++n)
        {
            int shellCount = 0;
            for (int l = 0; l <= n - 1; ++l)
                shellCount += subshellElectronCount(occ, n, l);
            if (shellCount > 0)
                highestNWithElectrons = n;
        }
        if (highestNWithElectrons == 0)
            return 0;
        int valence = 0;
        for (int l = 0; l <= highestNWithElectrons - 1; ++l)
            valence += subshellElectronCount(occ, highestNWithElectrons, l);
        return valence;
    }

    // Simple Madelung-based Aufbau filling (approximate; exceptions not handled)
    // Fills electrons by increasing n+l, then n. Within a subshell, fills one spin per m before pairing (Hund-like).
    template <int MaxN>
    ElectronOccupancy<MaxN> buildAufbauConfiguration(int atomicNumber)
    {
        if (atomicNumber < 0)
            throw std::invalid_argument("atomicNumber must be >= 0");
        using Config = ElectronOccupancy<MaxN>;
        Config occ;
        int remaining = atomicNumber;

        // Generate subshell order by Madelung rule
        std::vector<std::pair<int, int>> subshells;
        subshells.reserve(MaxN * (MaxN + 1) / 2);
        for (int n = 1; n <= MaxN; ++n)
        {
            for (int l = 0; l <= n - 1; ++l)
                subshells.emplace_back(n, l);
        }
        std::sort(subshells.begin(), subshells.end(), [](auto a, auto b)
                  {
        const int sA = a.first + a.second;
        const int sB = b.first + b.second;
        if (sA != sB) return sA < sB;
        return a.first < b.first; });

        for (auto [n, l] : subshells)
        {
            if (remaining == 0)
                break;
            const int capacity = 2 * (2 * l + 1);
            // First pass: half-fill with alpha spins across m
            for (int m = -l; m <= l && remaining > 0; ++m)
            {
                occ.set(so(n, l, m, Spin::Alpha));
                --remaining;
            }
            // Second pass: pair with beta spins across m
            for (int m = -l; m <= l && remaining > 0; ++m)
            {
                // If already fully filled (remaining <= 0) or capacity reached, break
                if (remaining <= 0)
                    break;
                occ.set(so(n, l, m, Spin::Beta));
                --remaining;
            }
            // If we somehow overshot, stop (should not happen)
            (void)capacity;
        }
        return occ;
    }

    // ECS-friendly component wrapper (fixed MaxN for uniform storage)
    template <int MaxN>
    struct ElectronConfigComponent
    {
        ElectronOccupancy<MaxN> occupancy;
    };

} // namespace battery::chem
