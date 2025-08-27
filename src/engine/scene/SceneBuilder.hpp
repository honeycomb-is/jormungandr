#pragma once

#include "Scene.hpp"
#include "../gfx/MeshFactory.hpp"

namespace Engine::SceneBuilder
{

    inline Engine::Scene::SceneData CreateDefaultScene()
    {
        Engine::Scene::SceneData scene;

        // Camera setup
        scene.cameraTransform.position = {0.0f, 0.0f, 5.0f};
        scene.cameraTransform.rotationEulerRad = {0.0f, 0.0f, 0.0f};
        scene.camera.fovYRadians = 60.0f * 3.1415926535f / 180.0f;
        scene.camera.aspect = 16.0f / 9.0f;
        scene.camera.nearPlane = 0.05f;
        scene.camera.farPlane = 5000.0f; // extend far plane for large scenes

        // Create meshes: head cylinder and trailing capsule segments
        // Diameter ~150mm -> radius 0.075m. Head default length ~0.6m, trailing ~0.5m
        scene.resources.headMesh = Engine::Gfx::CreateCylinder(0.075f, 0.6f, 64, true);
        scene.resources.trailingCapsuleMesh = Engine::Gfx::CreateTrailingCapsule(0.075f, 0.5f, 64);

        // Add a single renderable Jörmungandr segment at the origin
        Engine::Scene::RenderableEntity segment;
        segment.transform.position = {0.0f, 0.0f, 0.0f};
        segment.transform.rotationEulerRad = {0.0f, 0.0f, 0.0f};
        segment.transform.scale = {1.0f, 1.0f, 1.0f};
        segment.mesh = &scene.resources.headMesh;
        segment.material.useVertexColor = false;
        // Cutter head color: red
        segment.material.r = 1.0f;
        segment.material.g = 0.12f;
        segment.material.b = 0.12f;
        segment.style = Engine::ECS::RenderStyle::Solid;
        segment.opacity.alpha = 1.0f;
        scene.renderables.push_back(segment);

        // Tooth primitive (thin box); size will be scaled/placed per-tooth in UI
        scene.resources.toothBoxMesh = Engine::Gfx::CreateBox(0.02f, 0.01f, 0.02f);

        return scene;
    }

} // namespace Engine::SceneBuilder
