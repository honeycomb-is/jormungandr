#pragma once

#include <vector>
#include <cmath>
#include "../ecs/Components.hpp"

namespace Engine::Gfx
{
    using Engine::ECS::MeshData;
    using Engine::ECS::VertexPC;

    // Cylinder aligned along Z axis, centered at origin
    inline MeshData CreateCylinder(float radius, float length, int slices = 16, bool capped = true)
    {
        MeshData m;
        if (slices < 3)
            slices = 3;
        const float half = length * 0.5f;

        // Build side vertices: two rows (bottom z=-half, top z=+half), duplicate last column for seam
        const int rows = 2;
        const int cols = slices + 1;
        m.vertices.reserve(static_cast<size_t>(rows) * static_cast<size_t>(cols) + (capped ? 2 : 0));
        m.indices.reserve(static_cast<size_t>(slices) * 6 + (capped ? static_cast<size_t>(slices) * 6 : 0));

        auto ringZ = [&](int r)
        { return (r == 0) ? -half : +half; };
        for (int r = 0; r < rows; ++r)
        {
            float z = ringZ(r);
            for (int j = 0; j < cols; ++j)
            {
                float u = static_cast<float>(j) / static_cast<float>(slices);
                float phi = u * 2.0f * 3.1415926535f;
                float cp = std::cos(phi);
                float sp = std::sin(phi);
                float x = radius * cp;
                float y = radius * sp;
                // Neutralish vertex color
                float cr = cp * 0.5f + 0.5f;
                float cg = 0.7f;
                float cb = sp * 0.5f + 0.5f;
                m.vertices.push_back(VertexPC{x, y, z, cr, cg, cb});
            }
        }

        auto idx = [&](int r, int c) -> std::uint32_t
        { return static_cast<std::uint32_t>(r * cols + c); };
        for (int j = 0; j < slices; ++j)
        {
            std::uint32_t i0 = idx(0, j);
            std::uint32_t i1 = idx(0, j + 1);
            std::uint32_t i2 = idx(1, j);
            std::uint32_t i3 = idx(1, j + 1);
            // Two triangles per quad (match winding with sphere function)
            m.indices.push_back(i0);
            m.indices.push_back(i2);
            m.indices.push_back(i1);
            m.indices.push_back(i1);
            m.indices.push_back(i2);
            m.indices.push_back(i3);
        }

        if (capped)
        {
            // Bottom cap at z=-half
            std::uint32_t bottomCenter = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back(VertexPC{0.0f, 0.0f, -half, 0.65f, 0.65f, 0.68f});
            // Top cap at z=+half
            std::uint32_t topCenter = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back(VertexPC{0.0f, 0.0f, +half, 0.65f, 0.65f, 0.68f});

            for (int j = 0; j < slices; ++j)
            {
                // Bottom fan: center, next, current (choose consistent winding)
                std::uint32_t b0 = idx(0, j);
                std::uint32_t b1 = idx(0, j + 1);
                m.indices.push_back(bottomCenter);
                m.indices.push_back(b1);
                m.indices.push_back(b0);

                // Top fan: center, current, next
                std::uint32_t t0 = idx(1, j);
                std::uint32_t t1 = idx(1, j + 1);
                m.indices.push_back(topCenter);
                m.indices.push_back(t0);
                m.indices.push_back(t1);
            }
        }

        return m;
    }

    // Axis-aligned box centered at origin with full extents sx, sy, sz
    inline MeshData CreateBox(float sx, float sy, float sz)
    {
        MeshData m;
        float hx = sx * 0.5f, hy = sy * 0.5f, hz = sz * 0.5f;
        // 8 vertices with simple vertex colors (not used if material.useVertexColor=false)
        struct P
        {
            float x, y, z;
            float r, g, b;
        };
        P verts[] = {
            {-hx, -hy, -hz, 1, 0, 0}, {hx, -hy, -hz, 0, 1, 0}, {hx, hy, -hz, 0, 0, 1}, {-hx, hy, -hz, 1, 1, 0}, {-hx, -hy, hz, 1, 0, 1}, {hx, -hy, hz, 0, 1, 1}, {hx, hy, hz, 1, 1, 1}, {-hx, hy, hz, 0.2f, 0.2f, 0.2f}};
        for (const auto &v : verts)
            m.vertices.push_back(VertexPC{v.x, v.y, v.z, v.r, v.g, v.b});
        auto quad = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d)
        {
            m.indices.push_back(a);
            m.indices.push_back(b);
            m.indices.push_back(c);
            m.indices.push_back(a);
            m.indices.push_back(c);
            m.indices.push_back(d);
        };
        // Faces: -Z, +Z, -X, +X, -Y, +Y
        quad(0, 1, 2, 3); // back
        quad(4, 7, 6, 5); // front
        quad(0, 3, 7, 4); // left
        quad(1, 5, 6, 2); // right
        quad(0, 4, 5, 1); // bottom
        quad(3, 2, 6, 7); // top
        return m;
    }

    // Simple XZ plane centered at origin with size sx by sz (two triangles)
    inline MeshData CreatePlane(float sx, float sz)
    {
        MeshData m;
        float hx = sx * 0.5f;
        float hz = sz * 0.5f;
        // Four corners
        m.vertices.push_back(VertexPC{-hx, 0.0f, -hz, 0.65f, 0.65f, 0.68f});
        m.vertices.push_back(VertexPC{+hx, 0.0f, -hz, 0.65f, 0.65f, 0.68f});
        m.vertices.push_back(VertexPC{+hx, 0.0f, +hz, 0.65f, 0.65f, 0.68f});
        m.vertices.push_back(VertexPC{-hx, 0.0f, +hz, 0.65f, 0.65f, 0.68f});
        // Two triangles
        m.indices.insert(m.indices.end(), {0u, 1u, 2u, 0u, 2u, 3u});
        return m;
    }

    inline MeshData CreateConeFrustum(float r0, float r1, float length, int slices = 32, bool capped = true)
    {
        MeshData m;
        if (slices < 3)
            slices = 3;
        const float half = length * 0.5f;
        const int rows = 2;
        const int cols = slices + 1;
        m.vertices.reserve(static_cast<size_t>(rows) * static_cast<size_t>(cols) + (capped ? 2 : 0));
        m.indices.reserve(static_cast<size_t>(slices) * 6 + (capped ? static_cast<size_t>(slices) * 6 : 0));

        auto ringR = [&](int r)
        { return (r == 0) ? r0 : r1; };
        auto ringZ = [&](int r)
        { return (r == 0) ? -half : +half; };
        for (int r = 0; r < rows; ++r)
        {
            float z = ringZ(r);
            float rr = ringR(r);
            for (int j = 0; j < cols; ++j)
            {
                float u = static_cast<float>(j) / static_cast<float>(slices);
                float phi = u * 2.0f * 3.1415926535f;
                float cp = std::cos(phi);
                float sp = std::sin(phi);
                float x = rr * cp;
                float y = rr * sp;
                m.vertices.push_back(VertexPC{x, y, z, 0.7f, 0.7f, 0.72f});
            }
        }

        auto idx = [&](int r, int c) -> std::uint32_t
        { return static_cast<std::uint32_t>(r * cols + c); };
        for (int j = 0; j < slices; ++j)
        {
            std::uint32_t i0 = idx(0, j);
            std::uint32_t i1 = idx(0, j + 1);
            std::uint32_t i2 = idx(1, j);
            std::uint32_t i3 = idx(1, j + 1);
            m.indices.push_back(i0);
            m.indices.push_back(i2);
            m.indices.push_back(i1);
            m.indices.push_back(i1);
            m.indices.push_back(i2);
            m.indices.push_back(i3);
        }
        if (capped)
        {
            std::uint32_t bottomCenter = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back(VertexPC{0.0f, 0.0f, -half, 0.65f, 0.65f, 0.68f});
            std::uint32_t topCenter = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back(VertexPC{0.0f, 0.0f, +half, 0.65f, 0.65f, 0.68f});
            for (int j = 0; j < slices; ++j)
            {
                std::uint32_t b0 = idx(0, j);
                std::uint32_t b1 = idx(0, j + 1);
                m.indices.push_back(bottomCenter);
                m.indices.push_back(b1);
                m.indices.push_back(b0);
                std::uint32_t t0 = idx(1, j);
                std::uint32_t t1 = idx(1, j + 1);
                m.indices.push_back(topCenter);
                m.indices.push_back(t0);
                m.indices.push_back(t1);
            }
        }
        return m;
    }

    // Capsule-like trailing segment: cylinder with short frustum tips
    inline MeshData CreateTrailingCapsule(float radius, float length, int slices = 32)
    {
        // Use a slightly shorter cylinder and two small frustums to mimic rounded caps
        float cylLen = length * 0.7f;
        float capLen = (length - cylLen) * 0.5f;
        if (capLen < 0.0f)
            capLen = 0.0f;
        MeshData cyl = CreateCylinder(radius, cylLen, slices, true);
        MeshData cap = CreateConeFrustum(radius * 0.98f, 0.6f * radius, capLen, slices, true);
        MeshData out;
        out.vertices.reserve(cyl.vertices.size() + cap.vertices.size() * 2);
        out.indices.reserve(cyl.indices.size() + cap.indices.size() * 2);
        // Copy cylinder as-is (centered), then translate caps along Z and merge
        // Cylinder
        out.vertices.insert(out.vertices.end(), cyl.vertices.begin(), cyl.vertices.end());
        out.indices.insert(out.indices.end(), cyl.indices.begin(), cyl.indices.end());
        auto appendMesh = [&](const MeshData &m, float zoff, bool flipZ)
        {
            std::uint32_t base = (std::uint32_t)out.vertices.size();
            for (const auto &v : m.vertices)
            {
                float z = flipZ ? -v.pz : v.pz;
                out.vertices.push_back(VertexPC{v.px, v.py, z + zoff, v.cr, v.cg, v.cb});
            }
            for (auto idx : m.indices)
                out.indices.push_back(base + idx);
        };
        appendMesh(cap, +(cylLen * 0.5f), false);
        appendMesh(cap, -(cylLen * 0.5f), true);
        return out;
    }
}
