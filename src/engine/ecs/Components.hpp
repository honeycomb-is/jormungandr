#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "../math/Mat4.hpp"

namespace Engine::ECS
{
    using Engine::Math::Mat4;
    using Engine::Math::Vec3;

    struct Transform
    {
        Vec3 position{0, 0, 0};
        Vec3 rotationEulerRad{0, 0, 0};
        Vec3 scale{1, 1, 1};

        Mat4 localMatrix() const { return Engine::Math::composeTRS(position, rotationEulerRad, scale); }
    };

    struct Camera
    {
        float fovYRadians{60.0f * 3.1415926535f / 180.0f};
        float aspect{16.0f / 9.0f};
        float nearPlane{0.1f};
        float farPlane{1000.0f};
    };

    struct VertexPC
    {
        float px, py, pz; // position
        float cr, cg, cb; // color
    };

    struct MeshData
    {
        // Immutable geometry buffers (CPU-side for now)
        std::vector<VertexPC> vertices;
        std::vector<std::uint32_t> indices;
    };

    struct Material
    {
        // Simple solid color material for CPU renderer
        float r{1.0f};
        float g{1.0f};
        float b{1.0f};
        bool useVertexColor{true}; // if false, use r/g/b for fill
    };

    // Species visual components
    struct Radius
    {
        float r{1.0f};
    };

    enum class RenderStyle : std::uint8_t
    {
        Solid,
        Wireframe,
        Hidden
    };

    struct Opacity
    {
        float alpha{1.0f};
    };

    // Dashboard: simple movable rigid for kinematics-only viz
    struct KinematicBody
    {
        Vec3 linearVelocity{0.0f, 0.0f, 0.0f};
        Vec3 angularVelocity{0.0f, 0.0f, 0.0f}; // rad/s about x,y,z
    };

} // namespace Engine::ECS
