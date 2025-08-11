#pragma once

#include <vector>
#include <cmath>
#include "../ecs/Components.hpp"

namespace Engine::Gfx
{
    using Engine::ECS::MeshData;
    using Engine::ECS::VertexPC;

    inline MeshData CreateSphereUV(int stacks, int slices, float radius)
    {
        MeshData m;
        if (stacks < 2)
            stacks = 2;
        if (slices < 3)
            slices = 3;
        const int vertRows = stacks + 1;
        const int vertCols = slices + 1; // duplicate column for wrap-around
        m.vertices.reserve(static_cast<size_t>(vertRows) * static_cast<size_t>(vertCols));
        m.indices.reserve(static_cast<size_t>(stacks) * static_cast<size_t>(slices) * 6);

        for (int i = 0; i < vertRows; ++i)
        {
            float v = static_cast<float>(i) / static_cast<float>(stacks); // 0..1 -> theta 0..pi
            float theta = v * 3.1415926535f;
            float ct = std::cos(theta);
            float st = std::sin(theta);
            for (int j = 0; j < vertCols; ++j)
            {
                float u = static_cast<float>(j) / static_cast<float>(slices); // 0..1 -> phi 0..2pi
                float phi = u * 2.0f * 3.1415926535f;
                float cp = std::cos(phi);
                float sp = std::sin(phi);
                float x = radius * st * cp;
                float y = radius * ct;
                float z = radius * st * sp;
                // Simple color from normal mapped to 0..1
                float cr = (st * cp) * 0.5f + 0.5f;
                float cg = (ct) * 0.5f + 0.5f;
                float cb = (st * sp) * 0.5f + 0.5f;
                m.vertices.push_back(VertexPC{x, y, z, cr, cg, cb});
            }
        }

        auto idx = [&](int r, int c)
        { return static_cast<std::uint32_t>(r * vertCols + c); };
        for (int i = 0; i < stacks; ++i)
        {
            for (int j = 0; j < slices; ++j)
            {
                std::uint32_t i0 = idx(i, j);
                std::uint32_t i1 = idx(i, j + 1);
                std::uint32_t i2 = idx(i + 1, j);
                std::uint32_t i3 = idx(i + 1, j + 1);
                // Two triangles per quad
                m.indices.push_back(i0);
                m.indices.push_back(i2);
                m.indices.push_back(i1);
                m.indices.push_back(i1);
                m.indices.push_back(i2);
                m.indices.push_back(i3);
            }
        }
        return m;
    }

    // Low-poly sphere for species (atoms, electrons, nuclei, etc.)
    inline MeshData CreateSpeciesSphere(float radius, int lod = 0)
    {
        int stacks = 6;
        int slices = 8;
        if (lod == 1)
        {
            stacks = 8;
            slices = 12;
        }
        else if (lod == 2)
        {
            stacks = 12;
            slices = 16;
        }
        else if (lod > 2)
        {
            stacks = 4 + 2 * lod;
            slices = 6 + 3 * lod;
        }
        return CreateSphereUV(stacks, slices, radius);
    }

} // namespace Engine::Gfx
