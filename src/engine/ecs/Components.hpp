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
        // Placeholder for future shader/material params
        float dummy{0.0f};
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

    // Element property components (explicit, one component per PT key)
    struct ElementAtomicNumber
    {
        int value{0};
    };
    struct ElementSymbol
    {
        std::string value;
    };
    struct ElementName
    {
        std::string value;
    };
    struct ElementAtomicMassU
    {
        float value{0.0f};
    };
    struct ElementCPKHexColor
    {
        std::string value;
    };
    struct ElementElectronConfiguration
    {
        std::string value;
    };
    struct ElementElectronegativityPauling
    {
        float value{0.0f};
    };
    struct ElementAtomicRadiusVdW
    {
        float value{0.0f};
    };
    struct ElementIonizationEnergyEV
    {
        float value{0.0f};
    };
    struct ElementElectronAffinityEV
    {
        float value{0.0f};
    };
    struct ElementOxidationStates
    {
        std::vector<int> values;
    };
    struct ElementStandardState
    {
        std::string value;
    };
    struct ElementMeltingPointK
    {
        float value{0.0f};
    };
    struct ElementBoilingPointK
    {
        float value{0.0f};
    };
    struct ElementDensityGPerCm3
    {
        float value{0.0f};
    };
    struct ElementGroupBlock
    {
        std::string value;
    };
    struct ElementYearDiscovered
    {
        std::string value;
    };

} // namespace Engine::ECS
