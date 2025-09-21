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

        // Create meshes for trolley dashboard: chassis, wheel, ground
        scene.resources.chassisMesh = Engine::Gfx::CreateBox(0.8f, 0.2f, 1.2f);
        scene.resources.wheelMesh = Engine::Gfx::CreateCylinder(0.15f, 0.06f, 48, true);
        scene.resources.groundMesh = Engine::Gfx::CreatePlane(40.0f, 40.0f);

        // Ground plane
        {
            Engine::Scene::RenderableEntity ground;
            ground.transform.position = {0.0f, -0.15f, 0.0f};
            ground.transform.rotationEulerRad = {0.0f, 0.0f, 0.0f};
            ground.transform.scale = {1.0f, 1.0f, 1.0f};
            ground.mesh = &scene.resources.groundMesh;
            ground.material.useVertexColor = false;
            ground.material.r = 0.18f;
            ground.material.g = 0.18f;
            ground.material.b = 0.20f;
            ground.opacity.alpha = 1.0f;
            ground.style = Engine::ECS::RenderStyle::Solid;
            scene.renderables.push_back(ground);
        }

        // Chassis
        int chassisIndex = -1;
        {
            Engine::Scene::RenderableEntity chassis;
            chassis.transform.position = {0.0f, 0.10f, 0.0f};
            chassis.transform.rotationEulerRad = {0.0f, 0.0f, 0.0f};
            chassis.transform.scale = {1.0f, 1.0f, 1.0f};
            chassis.mesh = &scene.resources.chassisMesh;
            chassis.material.useVertexColor = false;
            chassis.material.r = 0.85f;
            chassis.material.g = 0.10f;
            chassis.material.b = 0.10f;
            chassis.opacity.alpha = 1.0f;
            chassis.style = Engine::ECS::RenderStyle::Solid;
            scene.renderables.push_back(chassis);
            chassisIndex = (int)scene.renderables.size() - 1;
        }

        auto addWheel = [&](float x, float z)
        {
            Engine::Scene::RenderableEntity wheel;
            wheel.transform.position = {x, 0.0f, z};
            // Wheel cylinder axis along Z; rotate so wheel spins around X -> rotate Y by 90deg to align length with X
            wheel.transform.rotationEulerRad = {0.0f, 1.57079632679f, 0.0f};
            wheel.transform.scale = {1.0f, 1.0f, 1.0f};
            wheel.mesh = &scene.resources.wheelMesh;
            wheel.material.useVertexColor = false;
            wheel.material.r = 0.15f;
            wheel.material.g = 0.15f;
            wheel.material.b = 0.15f;
            wheel.opacity.alpha = 1.0f;
            wheel.style = Engine::ECS::RenderStyle::Solid;
            scene.renderables.push_back(wheel);
        };
        // Four wheels: front-left/right, rear-left/right
        addWheel(-0.35f, 0.55f);
        addWheel(0.35f, 0.55f);
        addWheel(-0.35f, -0.55f);
        addWheel(0.35f, -0.55f);

        return scene;
    }

} // namespace Engine::SceneBuilder
