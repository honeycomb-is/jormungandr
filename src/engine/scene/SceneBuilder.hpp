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
        scene.camera.farPlane = 200.0f;

        // Shared meshes: atom (bigger, medium LOD), electron (smaller, LOD=1 per request)
        scene.resources.atomMesh = Engine::Gfx::CreateSphereUV(12, 16, 1.0f);
        scene.resources.electronMesh = Engine::Gfx::CreateSpeciesSphere(0.2f, 2); // lod 1

        // Force electron mesh vertex color to neutral (white)
        for (auto &v : scene.resources.electronMesh.vertices)
        {
            v.cr = v.cg = v.cb = 1.0f;
        }

        // Atom at origin
        Engine::Scene::RenderableEntity atom;
        atom.transform.position = {0.0f, 0.0f, 0.0f};
        atom.transform.scale = {1.0f, 1.0f, 1.0f};
        atom.mesh = &scene.resources.atomMesh;
        atom.style = Engine::ECS::RenderStyle::Solid;
        atom.opacity.alpha = 1.0f;
        scene.renderables.push_back(atom);
        int atomIndex = static_cast<int>(scene.renderables.size() - 1);

        // Helper to add electrons
        auto addElectron = [&](float orbitRadius, float speed, float phase, Engine::Math::Vec3 normal)
        {
            Engine::Scene::RenderableEntity e;
            e.transform.position = {orbitRadius, 0.0f, 0.0f};
            e.transform.scale = {1.0f, 1.0f, 1.0f};
            e.mesh = &scene.resources.electronMesh;
            e.style = Engine::ECS::RenderStyle::Solid; // solid for occlusion correctness
            e.opacity.alpha = 0.85f;
            scene.renderables.push_back(e);
            int eIndex = static_cast<int>(scene.renderables.size() - 1);
            scene.orbitals.push_back(Engine::Scene::Orbital{atomIndex, orbitRadius, speed, phase, normal, eIndex});
        };

        addElectron(2.5f, 5.0f, 0.0f, {0, 1, 0});
        addElectron(20.2f, 0.6f, 1.4f, {0, 1, 0});

        return scene;
    }

} // namespace Engine::SceneBuilder
