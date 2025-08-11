#pragma once

#include "../math/Mat4.hpp"
#include "../ecs/Components.hpp"

namespace Engine::Render
{
    using Engine::Math::composeTRS;
    using Engine::Math::lookAt;
    using Engine::Math::Mat4;
    using Engine::Math::perspective;
    using Engine::Math::Vec3;

    inline Mat4 ComputeViewMatrix(const Engine::ECS::Transform &camXform, const Vec3 &forward = {0, 0, -1}, const Vec3 &upWorld = {0, 1, 0})
    {
        // Build camera target from transform
        Mat4 camM = composeTRS(camXform.position, camXform.rotationEulerRad, {1, 1, 1});
        // Derive camera forward in world: for simplicity, rotate default forward by yaw/pitch/roll
        // Here we call lookAt using explicit position and target along -Z axis in camera space
        Vec3 camTarget = {camXform.position.x, camXform.position.y, camXform.position.z - 1.0f};
        return lookAt(camXform.position, camTarget, upWorld);
    }

    inline Mat4 ComputeProjectionMatrix(const Engine::ECS::Camera &cam)
    {
        return perspective(cam.fovYRadians, cam.aspect, cam.nearPlane, cam.farPlane);
    }

    inline Mat4 ComputeModelMatrix(const Engine::ECS::Transform &t)
    {
        return composeTRS(t.position, t.rotationEulerRad, t.scale);
    }
} // namespace Engine::Render
