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

        // Shared meshes: atom (bigger, medium LOD), electron (smaller, LOD=1 per request)
        scene.resources.atomMesh = Engine::Gfx::CreateSphereUV(12, 16, 1.0f);
        scene.resources.electronMesh = Engine::Gfx::CreateSpeciesSphere(0.2f, 1); // lod 1

        // Force electron mesh vertex color to neutral (white)
        for (auto &v : scene.resources.electronMesh.vertices)
        {
            v.cr = v.cg = v.cb = 1.0f;
        }

        // Start with an empty scene (no atoms) – user will spawn atoms from the Elements grid
        return scene;
    }

} // namespace Engine::SceneBuilder
