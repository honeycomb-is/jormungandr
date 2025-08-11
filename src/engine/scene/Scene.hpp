#pragma once

#include <vector>
#include "../ecs/Components.hpp"

namespace Engine::Scene
{
    using Engine::ECS::Camera;
    using Engine::ECS::Material;
    using Engine::ECS::MeshData;
    using Engine::ECS::Opacity;
    using Engine::ECS::RenderStyle;
    using Engine::ECS::Transform;

    struct RenderableEntity
    {
        Transform transform;
        const MeshData *mesh{nullptr};
        Material material;
        RenderStyle style{RenderStyle::Solid};
        Opacity opacity{1.0f};
    };

    struct SceneResources
    {
        MeshData atomMesh;
        MeshData electronMesh;
    };

    struct Orbital
    {
        int centerRenderableIndex{0};
        float radius{1.0f};
        float angularSpeed{1.0f};
        float phase{0.0f};
        Engine::Math::Vec3 planeNormal{0.0f, 1.0f, 0.0f};
        int targetRenderableIndex{-1};
    };

    struct SceneData
    {
        Transform cameraTransform;
        Camera camera;

        SceneResources resources;
        std::vector<RenderableEntity> renderables;
        std::vector<Orbital> orbitals;
    };

} // namespace Engine::Scene
