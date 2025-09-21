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
        // Trolley dashboard meshes
        MeshData chassisMesh;
        MeshData wheelMesh;
        MeshData groundMesh;
        // Legacy fields kept to avoid breaking existing code paths
        MeshData headMesh;
        MeshData trailingCapsuleMesh;
        MeshData toothBoxMesh;
    };

    struct SceneData
    {
        Transform cameraTransform;
        Camera camera;

        SceneResources resources;
        std::vector<RenderableEntity> renderables;
    };

} // namespace Engine::Scene
