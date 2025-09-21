#include "Honeycomb.hpp"

#include "imgui.h"

#include "engine/scene/SceneBuilder.hpp"
#include "engine/render/RenderPrimitives.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <random>
#include <deque>
#include <fstream>

namespace Honeycomb
{

    // Dashboard data models and simple simulation helpers
    struct Inputs
    {
        float payloadKg = 40.0f;
        float speed_mps = 0.6f;
        float slope_deg = 2.0f;
        float mu = 0.425f; // friction coefficient (worst-case default)
        float battCapacity_Ah = 40.0f;
        float battVoltage_V = 24.0f;
        float dischargeC = 1.0f;
        float armPayloadKg = 2.0f;
    };

    struct Outputs
    {
        float wheelTorque_Nm = 0.0f;
        float stoppingDistance_m = 0.0f;
        float runtime_hours = 0.0f;
    };

    namespace Sim
    {
        constexpr float g = 9.80665f;
        inline float ComputeStoppingDistance(float speed_mps, float mu, float slope_deg)
        {
            float slope = slope_deg * 3.1415926535f / 180.0f;
            float a_fric = mu * g * std::cos(slope);
            float a_grav = g * std::sin(slope);
            float a = a_fric - a_grav;
            if (a <= 0.01f)
                a = 0.01f;
            return (speed_mps * speed_mps) / (2.0f * a);
        }
        inline float ComputeWheelTorquePerWheel(float payloadKg, float speed_mps, float slope_deg, float mu, int numWheels)
        {
            float slope = slope_deg * 3.1415926535f / 180.0f;
            float mass = payloadKg + 20.0f; // trolley mass approx
            float F_slope = mass * g * std::sin(slope);
            float F_rr = mu * mass * g * 0.01f;
            float F_total = F_slope + F_rr + 0.2f * mass * speed_mps;
            float wheelRadius = 0.15f;
            float torqueTotal = F_total * wheelRadius;
            return torqueTotal / std::max(1, numWheels);
        }
        inline float EstimateRuntimeHours(float Ah, float V, float dischargeC, float currentA)
        {
            float capA = Ah * std::max(0.5f, 1.0f - 0.1f * (dischargeC - 1.0f));
            if (currentA <= 0.01f)
                return 12.0f;
            return capA / currentA;
        }
    }

    void RenderUI()
    {
        static bool opt_fullscreen = true;
        static bool opt_padding = false;
        static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        if (opt_fullscreen)
        {
            const ImGuiViewport *viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(viewport->WorkPos);
            ImGui::SetNextWindowSize(viewport->WorkSize);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
            window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
        }
        else
        {
            dockspace_flags &= ~ImGuiDockNodeFlags_PassthruCentralNode;
        }

        if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
            window_flags |= ImGuiWindowFlags_NoBackground;

        if (!opt_padding)
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("Honeycomb Engine", nullptr, window_flags);
        if (!opt_padding)
            ImGui::PopStyleVar();

        if (opt_fullscreen)
            ImGui::PopStyleVar(2);

        ImGuiIO &io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
        {
            ImGuiID dockspace_id = ImGui::GetID("Honeycomb Engine");
            ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
        }

        // 2D dashboard mode: free-body diagram and engineering plots
        {
            static Inputs inputs{};
            // Geometry in millimeters (mm)
            static float wheelbase_mm = 900.0f;
            static float cgFromRear_mm = 450.0f;
            static float cgHeight_mm = 350.0f;
            static float wheelRadius_mm = 150.0f;
            static float c_rr = 0.015f;
            static float rho_air = 1.2f;
            static float CdA = 0.6f;
            static int driveMode = 2; // 0=FWD,1=RWD,2=AWD
            static bool motorEnabled = true;
            static bool brakesApplied = false;
            static float actualSpeed_mps = 0.0f;
            static float velocityX_mps = 0.0f; // velocity components
            static float velocityY_mps = 0.0f;
            static float accelX_mps2 = 0.0f; // acceleration components
            static float accelY_mps2 = 0.0f;
            static float positionX_m = 0.0f; // trolley world position
            static float positionY_m = 0.0f;

            // Motor specifications
            static float motorPower_W = 250.0f;
            static float motorMaxTorque_Nm = 15.0f;
            static float motorMaxRPM = 3000.0f;
            static float motorEfficiency = 0.85f;

            // Axle and drivetrain
            static float axleDiameter_mm = 20.0f;
            static float axleMaterial_tensileStrength_MPa = 400.0f; // Steel
            static float gearReduction = 10.0f;
            static float chainEfficiency = 0.95f;

            // Trolley dimensions
            static float trolleyWidth_mm = 600.0f;
            static float trolleyHeight_mm = 800.0f;
            static float deckHeight_mm = 200.0f;
            static float groundClearance_mm = 50.0f;

            // Wheel specifications
            static float wheelWidth_mm = 50.0f;
            static float wheelLoad_rating_kg = 150.0f;
            static float bearingLoad_rating_N = 2000.0f;

            // Material properties
            static float chassisMaterial_density_kgm3 = 2700.0f; // Aluminum
            static float chassisThickness_mm = 3.0f;
            static float safetyFactor = 2.5f;

            static float simTime_s = 0.0f;
            static std::deque<ImVec2> sTorqueFront, sTorqueRear, sNf, sNr;

            auto clampf = [](float v, float lo, float hi)
            { return v < lo ? lo : (v > hi ? hi : v); };
            float dt = ImGui::GetIO().DeltaTime;
            simTime_s += dt;

            // Compute loads and torques (convert mm -> m for physics)
            const float g = 9.80665f;
            float mass = inputs.payloadKg + 20.0f;
            float theta = inputs.slope_deg * 3.1415926535f / 180.0f;
            float wheelbase_m = wheelbase_mm * 0.001f;
            float cgFromRear_m = cgFromRear_mm * 0.001f;
            float cgHeight_m = cgHeight_mm * 0.001f;
            float wheelRadius_m = wheelRadius_mm * 0.001f;
            float W = mass * g;
            float Nf = (W * (cgFromRear_m * std::cos(theta) - cgHeight_m * std::sin(theta))) / std::max(0.001f, wheelbase_m);
            float Nr = W * std::cos(theta) - Nf;
            if (Nf < 0.0f)
                Nf = 0.0f;
            if (Nr < 0.0f)
                Nr = 0.0f;

            // Gravity component along +X (right). Positive theta means slope rises to the right,
            // so gravity acts downslope to the left (negative X): -W*sin(theta)
            float F_grav = -W * std::sin(theta);
            float F_rr = c_rr * W * std::cos(theta);
            // Aerodynamic drag for control (commanded) vs dynamics (instantaneous)
            float speedUsed_mps = motorEnabled ? inputs.speed_mps : actualSpeed_mps;
            float F_aero_cmd = 0.5f * rho_air * CdA * inputs.speed_mps * std::fabs(inputs.speed_mps);
            float F_aero_inst = 0.5f * rho_air * CdA * actualSpeed_mps * std::fabs(actualSpeed_mps);
            // Required drive to maintain commanded speed in its direction
            float sign_cmd = (inputs.speed_mps >= 0.0f) ? 1.0f : -1.0f;
            float F_grav_req = W * std::sin(theta) * sign_cmd; // positive when uphill relative to commanded direction
            float F_rr_mag = c_rr * W * std::cos(theta);
            float F_drive_req = F_rr_mag + F_aero_cmd + std::max(0.0f, F_grav_req);

            float tractionFrontMax = inputs.mu * Nf;
            float tractionRearMax = inputs.mu * Nr;
            // Motor force target = feedforward (to hold cmd speed on slope) + feedback (track speed)
            float tau_track = 0.6f;
            float F_feedback = (inputs.speed_mps - actualSpeed_mps) / tau_track * mass;
            float F_motor_target = motorEnabled ? (F_drive_req + F_feedback) : 0.0f;
            // Distribute to axles per mode with axle traction limits
            float F_front = 0.0f, F_rear = 0.0f;
            if (driveMode == 0) // FWD
            {
                F_front = clampf(F_motor_target, -tractionFrontMax, tractionFrontMax);
                F_rear = 0.0f;
            }
            else if (driveMode == 1) // RWD
            {
                F_rear = clampf(F_motor_target, -tractionRearMax, tractionRearMax);
                F_front = 0.0f;
            }
            else // AWD (split, then reassign remainder)
            {
                float half = 0.5f * F_motor_target;
                F_front = clampf(half, -tractionFrontMax, tractionFrontMax);
                F_rear = clampf(half, -tractionRearMax, tractionRearMax);
                float remF = half - F_front;
                float remR = half - F_rear;
                if (remF != 0.0f)
                    F_rear = clampf(F_rear + remF, -tractionRearMax, tractionRearMax);
                if (remR != 0.0f)
                    F_front = clampf(F_front + remR, -tractionFrontMax, tractionFrontMax);
            }
            float T_front = F_front * wheelRadius_m;
            float T_rear = F_rear * wheelRadius_m;

            // Append time series
            sTorqueFront.emplace_back(simTime_s, T_front);
            sTorqueRear.emplace_back(simTime_s, T_rear);
            sNf.emplace_back(simTime_s, Nf);
            sNr.emplace_back(simTime_s, Nr);
            auto trim = [](std::deque<ImVec2> &q)
            { if ((int)q.size() > 1200) q.pop_front(); };
            trim(sTorqueFront);
            trim(sTorqueRear);
            trim(sNf);
            trim(sNr);

            // Vector physics: track velocity and acceleration components
            float oldVelX = velocityX_mps;
            float oldVelY = velocityY_mps;

            if (!motorEnabled)
            {
                // Net force acts along slope direction in world coordinates
                float cos_slope = std::cos(theta);
                float sin_slope = std::sin(theta);
                float F_net_world = F_motor_target + F_grav - F_rr - F_aero_inst;

                if (brakesApplied)
                {
                    // Maximum braking force available from all wheels
                    float F_brake_max = inputs.mu * W * std::cos(theta);
                    float v_mag = std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps);

                    if (v_mag > 0.001f)
                    {
                        // Apply braking force directly opposite to motion along slope direction
                        float brake_force_along_slope = F_brake_max;
                        if (v_mag < 0.1f)
                            brake_force_along_slope *= 3.0f; // Extra braking at low speeds

                        // Determine if we're moving uphill or downhill and oppose accordingly
                        float velocity_along_slope = velocityX_mps * cos_slope + velocityY_mps * sin_slope;
                        if (velocity_along_slope > 0.0f)
                            F_net_world -= brake_force_along_slope; // Moving upslope, brake downslope
                        else if (velocity_along_slope < 0.0f)
                            F_net_world += brake_force_along_slope; // Moving downslope, brake upslope

                        // Direct velocity damping for very effective braking
                        velocityX_mps *= (1.0f - 10.0f * dt); // Strong velocity damping
                        velocityY_mps *= (1.0f - 10.0f * dt);
                    }
                    else
                    {
                        // At very low speeds, lock the wheels completely
                        velocityX_mps = 0.0f;
                        velocityY_mps = 0.0f;
                        accelX_mps2 = 0.0f;
                        accelY_mps2 = 0.0f;
                        F_net_world = 0.0f; // No forces when locked
                    }
                }
                float F_x = F_net_world * cos_slope; // force component along +X (right)
                float F_y = F_net_world * sin_slope; // force component along +Y (up)

                accelX_mps2 = F_x / std::max(1.0f, mass);
                accelY_mps2 = F_y / std::max(1.0f, mass);

                velocityX_mps += accelX_mps2 * dt;
                velocityY_mps += accelY_mps2 * dt;

                // Damping for very small velocities
                if (std::fabs(velocityX_mps) < 0.0005f)
                    velocityX_mps = 0.0f;
                if (std::fabs(velocityY_mps) < 0.0005f)
                    velocityY_mps = 0.0f;

                actualSpeed_mps = std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps);
                speedUsed_mps = actualSpeed_mps;
            }
            else
            {
                // Motor enforces commanded speed along slope direction
                float cos_slope = std::cos(theta);
                float sin_slope = std::sin(theta);
                velocityX_mps = inputs.speed_mps * cos_slope;
                velocityY_mps = inputs.speed_mps * sin_slope;
                actualSpeed_mps = std::fabs(inputs.speed_mps);

                // Calculate acceleration for display
                accelX_mps2 = (velocityX_mps - oldVelX) / std::max(0.001f, dt);
                accelY_mps2 = (velocityY_mps - oldVelY) / std::max(0.001f, dt);

                // Brakes can override motor - apply strong braking even when motor is on
                if (brakesApplied)
                {
                    float v_mag = std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps);
                    if (v_mag > 0.001f)
                    {
                        // Strong braking dampens velocity directly
                        velocityX_mps *= (1.0f - 12.0f * dt); // Very strong braking with motor on
                        velocityY_mps *= (1.0f - 12.0f * dt);
                        actualSpeed_mps = std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps);
                    }
                    else
                    {
                        // Complete stop
                        velocityX_mps = 0.0f;
                        velocityY_mps = 0.0f;
                        actualSpeed_mps = 0.0f;
                    }
                }
            }

            // Update world position
            positionX_m += velocityX_mps * dt;
            positionY_m += velocityY_mps * dt;

            // 2D Free-body Diagram window
            ImGui::Begin("Diagram 2D");
            ImDrawList *dl2 = ImGui::GetWindowDrawList();
            ImVec2 wpos = ImGui::GetWindowPos();
            ImVec2 cmin = ImGui::GetWindowContentRegionMin();
            ImVec2 cmax = ImGui::GetWindowContentRegionMax();
            ImVec2 dmin = ImVec2(wpos.x + cmin.x, wpos.y + cmin.y);
            ImVec2 dmax = ImVec2(wpos.x + cmax.x, wpos.y + cmax.y);
            float dw = dmax.x - dmin.x, dh = dmax.y - dmin.y;
            dl2->AddRectFilled(dmin, dmax, IM_COL32(255, 255, 255, 255)); // White background
            dl2->AddRect(dmin, dmax, IM_COL32(200, 200, 200, 255), 0.0f, 0, 1.0f);

            // World basis for ground line at slope theta
            ImVec2 center = ImVec2(dmin.x + dw * 0.5f, dmin.y + dh * 0.65f);
            float pxPerMM = (dw * 0.7f) / (wheelbase_mm + 500.0f);
            ImVec2 u = ImVec2(std::cos(theta), -std::sin(theta));
            ImVec2 v = ImVec2(std::sin(theta), std::cos(theta));
            auto add = [](ImVec2 a, ImVec2 b)
            { return ImVec2(a.x + b.x, a.y + b.y); };
            auto mul = [](ImVec2 a, float s)
            { return ImVec2(a.x * s, a.y * s); };

            // Ground line and axes
            ImVec2 g0 = add(center, mul(u, -0.6f * dw));
            ImVec2 g1 = add(center, mul(u, 0.6f * dw));
            dl2->AddLine(g0, g1, IM_COL32(60, 60, 60, 255), 3.0f);
            auto arrow = [&](ImVec2 p, ImVec2 dir, float len_px, ImU32 col)
            {
                ImVec2 q = add(p, mul(dir, len_px));
                dl2->AddLine(p, q, col, 2.0f);
                ImVec2 ort = ImVec2(-dir.y, dir.x);
                ImVec2 h1 = add(q, add(mul(dir, -8.0f), mul(ort, 4.0f)));
                ImVec2 h2 = add(q, add(mul(dir, -8.0f), mul(ort, -4.0f)));
                dl2->AddTriangleFilled(q, h1, h2, col);
            };
            // removed axis arrows

            // Wheel centers (use engineering variables)
            ImVec2 rearC = add(center, mul(u, -0.5f * wheelbase_mm * pxPerMM));
            ImVec2 frontC = add(center, mul(u, 0.5f * wheelbase_mm * pxPerMM));
            float rpx = wheelRadius_mm * pxPerMM;
            // wheels sit below the ground line
            rearC = add(rearC, mul(v, -rpx));
            frontC = add(frontC, mul(v, -rpx));

            // Wheel color based on load status
            float wheelload_per_wheel_N = (inputs.payloadKg + 20.0f) * 9.81f / 4.0f;
            bool wheelOverloaded = wheelload_per_wheel_N > (wheelLoad_rating_kg * 9.81f);
            ImU32 wheelColor = wheelOverloaded ? IM_COL32(200, 40, 40, 255) : IM_COL32(40, 40, 40, 255);

            // Draw wheels with width indication
            float wheelWidthPx = wheelWidth_mm * pxPerMM * 0.3f; // scaled for visibility
            dl2->AddCircle(rearC, rpx, wheelColor, 32, 3.0f);
            dl2->AddCircle(frontC, rpx, wheelColor, 32, 3.0f);

            // Show wheel width as inner circles
            if (wheelWidthPx > 2.0f)
            {
                dl2->AddCircle(rearC, rpx - wheelWidthPx * 0.5f, wheelColor, 32, 1.0f);
                dl2->AddCircle(frontC, rpx - wheelWidthPx * 0.5f, wheelColor, 32, 1.0f);
            }

            // Wheel rotation marker (dot only) - based on actual physics
            static float wheelPhase = 0.0f;
            // Wheel rotation based on velocity along ground (u direction)
            float velocity_along_ground = velocityX_mps * std::cos(theta) + velocityY_mps * std::sin(theta);
            float omegaReal = (wheelRadius_m > 0.0f) ? (velocity_along_ground / wheelRadius_m) : 0.0f;
            // Positive velocity_along_ground = moving up the slope = clockwise rotation (when viewed from right side)
            wheelPhase += omegaReal * dt;
            if (wheelPhase > 6.283185f)
                wheelPhase -= 6.283185f;
            if (wheelPhase < -6.283185f)
                wheelPhase += 6.283185f;
            auto drawSpoke = [&](ImVec2 c)
            {
                float cs = std::cos(wheelPhase), sn = std::sin(wheelPhase);
                ImVec2 rad = ImVec2(u.x * cs + v.x * sn, u.y * cs + v.y * sn);
                ImVec2 tip = add(c, mul(rad, rpx * 0.8f));
                dl2->AddCircleFilled(tip, 3.0f, IM_COL32(20, 20, 20, 255)); // Dark marker, slightly larger
            };
            drawSpoke(rearC);
            drawSpoke(frontC);

            // Chassis box removed per request

            // CG position
            ImVec2 rearContact = add(rearC, mul(v, rpx));
            ImVec2 frontContact = add(frontC, mul(v, rpx));
            ImVec2 cg = add(rearContact, mul(u, cgFromRear_mm * pxPerMM));
            cg = add(cg, mul(v, -(cgHeight_mm + wheelRadius_mm) * pxPerMM));
            dl2->AddCircleFilled(cg, 4.0f, IM_COL32(180, 100, 20, 255)); // Dark orange

            // Arrow helper (reused above)
            // Force scaling: fixed base scale, multiply by magnitude
            float pixelsPerNewton = (dh > 0.0f ? (dh * 0.00025f) : 0.2f);
            auto scaleF = [&](float F)
            { float len = std::fabs(F) * pixelsPerNewton; return (len < 2.0f ? 2.0f : len); };
            // Normals (up along v)
            // normals point opposite of ground normal to visualize reaction force
            arrow(rearContact, ImVec2(-v.x, -v.y), scaleF(Nr), IM_COL32(20, 100, 180, 255));  // Dark blue
            arrow(frontContact, ImVec2(-v.x, -v.y), scaleF(Nf), IM_COL32(20, 100, 180, 255)); // Dark blue
            // Weight (down vertical)
            arrow(cg, ImVec2(0, 1), scaleF(W), IM_COL32(180, 20, 20, 255)); // Dark red
            // Drive/traction vectors: horizontal screen-space (right=+X), flip by travel direction
            if (F_rear > 0.0f)
            {
                ImVec2 dir = (actualSpeed_mps >= 0.0f) ? ImVec2(-1.0f, 0.0f) : ImVec2(1.0f, 0.0f);
                arrow(rearContact, dir, scaleF(F_rear), IM_COL32(20, 120, 20, 255)); // Dark green
            }
            if (F_front > 0.0f)
            {
                ImVec2 dir = (actualSpeed_mps >= 0.0f) ? ImVec2(-1.0f, 0.0f) : ImVec2(1.0f, 0.0f);
                arrow(frontContact, dir, scaleF(F_front), IM_COL32(20, 120, 20, 255)); // Dark green
            }

            // Torque arc magnitude and direction at each wheel
            auto drawTorqueArc = [&](ImVec2 c, float T, bool isBraking = false)
            {
                float Tmag = std::fabs(T);
                if (Tmag < 1e-3f && !isBraking)
                    return;
                float radius = rpx * 0.7f;
                int segs = 28;
                float span = std::min(2.2f, 0.0035f * Tmag + 0.4f);

                // Torque direction based on actual physics
                float signRot = (T >= 0.0f ? 1.0f : -1.0f);
                if (isBraking)
                    signRot = -signRot; // Brake torque opposes motion

                float start = 0.0f;
                float end = start + signRot * span;
                ImU32 col = isBraking ? IM_COL32(180, 20, 20, 255) : IM_COL32(160, 80, 20, 255); // Red for braking, orange for driving
                ImVec2 prev = ImVec2(c.x + radius * std::cos(start), c.y + radius * std::sin(start));
                for (int i = 1; i <= segs; ++i)
                {
                    float t = start + (end - start) * ((float)i / segs);
                    ImVec2 cur = ImVec2(c.x + radius * std::cos(t), c.y + radius * std::sin(t));
                    dl2->AddLine(prev, cur, col, isBraking ? 3.0f : 2.0f);
                    prev = cur;
                }
                ImVec2 dir = ImVec2(std::cos(end), std::sin(end));
                ImVec2 tip = ImVec2(c.x + radius * dir.x, c.y + radius * dir.y);
                ImVec2 ort = ImVec2(-dir.y, dir.x);
                ImVec2 h1 = ImVec2(tip.x + (-8.0f * dir.x + 4.0f * ort.x), tip.y + (-8.0f * dir.y + 4.0f * ort.y));
                ImVec2 h2 = ImVec2(tip.x + (-8.0f * dir.x - 4.0f * ort.x), tip.y + (-8.0f * dir.y - 4.0f * ort.y));
                dl2->AddTriangleFilled(tip, h1, h2, col);
            };

            // Calculate brake torques when brakes are applied
            float T_brake_rear = 0.0f, T_brake_front = 0.0f;
            if (brakesApplied)
            {
                float F_brake_max = inputs.mu * W * std::cos(theta);
                T_brake_rear = F_brake_max * 0.5f * wheelRadius_m; // Half brake force per wheel
                T_brake_front = F_brake_max * 0.5f * wheelRadius_m;
            }

            // Draw drive torques
            drawTorqueArc(rearC, T_rear, false);
            drawTorqueArc(frontC, T_front, false);

            // Draw brake torques (opposing rotation)
            if (brakesApplied)
            {
                drawTorqueArc(rearC, T_brake_rear, true);
                drawTorqueArc(frontC, T_brake_front, true);
            }

            // Labels (positioned lower to avoid torque label overlap)
            char buf[128];
            snprintf(buf, sizeof(buf), "Rear load: %.0f N", Nr);
            dl2->AddText(add(rearContact, mul(v, rpx + 35.0f)), IM_COL32(20, 20, 20, 255), buf); // Dark text, positioned lower
            snprintf(buf, sizeof(buf), "Front load: %.0f N", Nf);
            dl2->AddText(add(frontContact, mul(v, rpx + 35.0f)), IM_COL32(20, 20, 20, 255), buf); // Dark text, positioned lower
            snprintf(buf, sizeof(buf), "Weight: %.0f N", W);
            dl2->AddText(add(cg, ImVec2(6, 6)), IM_COL32(20, 20, 20, 255), buf); // Dark text
            snprintf(buf, sizeof(buf), "Drive force needed: %.0f N (Gravity %.0f N, Rolling resistance %.0f N, Air drag %.0f N)", F_drive_req, F_grav_req, F_rr_mag, F_aero_cmd);
            dl2->AddText(ImVec2(dmin.x + 8.0f, dmin.y + 8.0f), IM_COL32(20, 20, 20, 255), buf); // Dark text
            snprintf(buf, sizeof(buf), "Front torque: %.1f Nm  Rear torque: %.1f Nm", T_front, T_rear);
            dl2->AddText(ImVec2(dmin.x + 8.0f, dmin.y + 28.0f), IM_COL32(20, 20, 20, 255), buf); // Dark text

            // Per-wheel torque magnitude labels near wheels
            {
                char tbuf[64];
                if (brakesApplied)
                {
                    snprintf(tbuf, sizeof(tbuf), "Td=%.1f Tb=%.1f Nm", std::fabs(T_rear), T_brake_rear);
                    dl2->AddText(add(rearC, mul(v, rpx + 12.0f)), IM_COL32(20, 20, 20, 255), tbuf); // Dark text
                    snprintf(tbuf, sizeof(tbuf), "Td=%.1f Tb=%.1f Nm", std::fabs(T_front), T_brake_front);
                    dl2->AddText(add(frontC, mul(v, rpx + 12.0f)), IM_COL32(20, 20, 20, 255), tbuf); // Dark text
                }
                else
                {
                    snprintf(tbuf, sizeof(tbuf), "\xcf\x84=%.1f Nm", std::fabs(T_rear));
                    dl2->AddText(add(rearC, mul(v, rpx + 12.0f)), IM_COL32(160, 80, 20, 255), tbuf); // Dark orange
                    snprintf(tbuf, sizeof(tbuf), "\xcf\x84=%.1f Nm", std::fabs(T_front));
                    dl2->AddText(add(frontC, mul(v, rpx + 12.0f)), IM_COL32(160, 80, 20, 255), tbuf); // Dark orange
                }
            }
            ImGui::End();

            // Forces viewport removed

            // Graphs window (time series)
            ImGui::Begin("Graphs");
            ImDrawList *dlg = ImGui::GetWindowDrawList();
            ImVec2 gpos = ImGui::GetWindowPos();
            ImVec2 gmin = ImVec2(gpos.x + ImGui::GetWindowContentRegionMin().x, gpos.y + ImGui::GetWindowContentRegionMin().y);
            ImVec2 gmax = ImVec2(gpos.x + ImGui::GetWindowContentRegionMax().x, gpos.y + ImGui::GetWindowContentRegionMax().y);
            float gw = gmax.x - gmin.x, gh = gmax.y - gmin.y;
            dlg->AddRectFilled(gmin, gmax, IM_COL32(255, 255, 255, 255)); // White background
            dlg->AddRect(gmin, gmax, IM_COL32(150, 150, 150, 255));
            auto plotSeries = [&](const std::deque<ImVec2> &data, ImU32 col, float y0, const char *title)
            {
                if (data.size() < 2)
                    return;
                float h = (gh - 12.0f) / 3.0f;
                ImVec2 r0 = ImVec2(gmin.x + 6.0f, gmin.y + 6.0f + y0 * h);
                ImVec2 r1 = ImVec2(gmax.x - 6.0f, r0.y + h - 6.0f);
                dlg->AddRect(r0, r1, IM_COL32(120, 120, 120, 255), 0.0f, 0, 1.0f);
                float xmin = data.front().x, xmax = data.back().x;
                float ymin = data.front().y, ymax = data.front().y;
                for (const auto &p : data)
                {
                    if (p.y < ymin)
                        ymin = p.y;
                    if (p.y > ymax)
                        ymax = p.y;
                }
                if (xmax <= xmin)
                    xmax = xmin + 1.0f;
                if (ymax <= ymin)
                    ymax = ymin + 1.0f;
                auto toScr = [&](float x, float y)
                {
                    float u = (x - xmin) / (xmax - xmin);
                    float v = (y - ymin) / (ymax - ymin);
                    return ImVec2(r0.x + u * (r1.x - r0.x), r0.y + (1.0f - v) * (r1.y - r0.y));
                };
                for (size_t i = 1; i < data.size(); ++i)
                    dlg->AddLine(toScr(data[i - 1].x, data[i - 1].y), toScr(data[i].x, data[i].y), col, 2.0f);
                dlg->AddText(ImVec2(r0.x + 4, r0.y + 4), IM_COL32(20, 20, 20, 255), title); // Dark text
                // axes labels removed per request
            };
            plotSeries(sTorqueFront, IM_COL32(180, 80, 20, 255), 0.0f, "Front Torque (Nm)");      // Dark orange
            plotSeries(sTorqueRear, IM_COL32(20, 120, 20, 255), 1.0f, "Rear Torque (Nm)");        // Dark green
            plotSeries(sNf, IM_COL32(20, 100, 180, 255), 2.0f, "Normal Forces (N) [front=blue]"); // Dark blue
            // Overlay rear normals in green on same third
            if (sNr.size() > 1)
            {
                // reuse last plot rect by computing it again
                float h = (gh - 12.0f) / 3.0f;
                ImVec2 r0 = ImVec2(gmin.x + 6.0f, gmin.y + 6.0f + 2.0f * h);
                ImVec2 r1 = ImVec2(gmax.x - 6.0f, r0.y + h - 6.0f);
                float xmin = sNf.front().x, xmax = sNf.back().x;
                float ymin = sNf.front().y, ymax = sNf.front().y;
                for (const auto &p : sNf)
                {
                    if (p.y < ymin)
                        ymin = p.y;
                    if (p.y > ymax)
                        ymax = p.y;
                }
                for (const auto &p : sNr)
                {
                    if (p.y < ymin)
                        ymin = p.y;
                    if (p.y > ymax)
                        ymax = p.y;
                }
                if (xmax <= xmin)
                    xmax = xmin + 1.0f;
                if (ymax <= ymin)
                    ymax = ymin + 1.0f;
                auto toScr = [&](float x, float y)
                {
                    float u = (x - xmin) / (xmax - xmin);
                    float v = (y - ymin) / (ymax - ymin);
                    return ImVec2(r0.x + u * (r1.x - r0.x), r0.y + (1.0f - v) * (r1.y - r0.y));
                };
                for (size_t i = 1; i < sNr.size(); ++i)
                    dlg->AddLine(toScr(sNr[i - 1].x, sNr[i - 1].y), toScr(sNr[i].x, sNr[i].y), IM_COL32(20, 120, 20, 255), 2.0f); // Dark green
            }
            ImGui::End();

            // Controls window
            ImGui::Begin("Controls");
            ImGui::TextUnformatted("Inputs");
            ImGui::Checkbox("Motor enabled", &motorEnabled);
            ImGui::SameLine();
            ImGui::Checkbox("Apply brakes (B)", &brakesApplied);

            // Keyboard shortcut for brakes
            if (ImGui::IsKeyPressed(ImGuiKey_B))
                brakesApplied = !brakesApplied;
            ImGui::SliderFloat("Command speed (m/s)", &inputs.speed_mps, -1.5f, 1.5f, "%.2f");
            ImGui::Text("Actual speed: %.3f m/s", actualSpeed_mps);
            ImGui::Separator();
            ImGui::TextUnformatted("Vector State");
            ImGui::Text("Velocity: (%.3f, %.3f) m/s  |v|=%.3f", velocityX_mps, velocityY_mps,
                        std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps));
            ImGui::Text("Acceleration: (%.2f, %.2f) m/s²  |a|=%.2f", accelX_mps2, accelY_mps2,
                        std::sqrt(accelX_mps2 * accelX_mps2 + accelY_mps2 * accelY_mps2));
            ImGui::Text("Position: (%.2f, %.2f) m", positionX_m, positionY_m);
            ImGui::SliderFloat("Payload (kg)", &inputs.payloadKg, 0.0f, 210.0f, "%.0f");
            ImGui::SliderFloat("Slope (deg)", &inputs.slope_deg, -10.0f, 10.0f, "%.1f");
            ImGui::SliderFloat("Friction mu", &inputs.mu, 0.2f, 0.8f, "%.2f");
            ImGui::Separator();
            ImGui::TextUnformatted("Geometry");
            ImGui::SliderFloat("Wheelbase (mm)", &wheelbase_mm, 200.0f, 2000.0f, "%.0f");
            ImGui::SliderFloat("CG from rear (mm)", &cgFromRear_mm, 10.0f, wheelbase_mm - 10.0f, "%.0f");
            ImGui::SliderFloat("CG height (mm)", &cgHeight_mm, 50.0f, 900.0f, "%.0f");
            ImGui::SliderFloat("Wheel radius (mm)", &wheelRadius_mm, 40.0f, 300.0f, "%.0f");
            ImGui::Separator();
            ImGui::TextUnformatted("Losses");
            ImGui::SliderFloat("Rolling resistance coeff (c_rr)", &c_rr, 0.005f, 0.04f, "%.3f");
            ImGui::SliderFloat("Drag area (CdA)", &CdA, 0.2f, 1.2f, "%.2f");
            ImGui::SliderFloat("Air density (kg/m^3)", &rho_air, 0.9f, 1.4f, "%.2f");
            ImGui::Separator();
            ImGui::TextUnformatted("Drive Mode");
            const char *modes[] = {"FWD", "RWD", "AWD"};
            ImGui::Combo("##drive", &driveMode, modes, 3);
            ImGui::Separator();

            // Motor Design Section
            ImGui::TextUnformatted("Motor & Drivetrain");
            ImGui::SliderFloat("Motor Power (W)", &motorPower_W, 100.0f, 1000.0f, "%.0f");
            ImGui::SliderFloat("Max Torque (Nm)", &motorMaxTorque_Nm, 5.0f, 50.0f, "%.1f");
            ImGui::SliderFloat("Max RPM", &motorMaxRPM, 1000.0f, 6000.0f, "%.0f");
            ImGui::SliderFloat("Motor Efficiency", &motorEfficiency, 0.70f, 0.95f, "%.2f");
            ImGui::SliderFloat("Gear Reduction", &gearReduction, 1.0f, 20.0f, "%.1f");
            ImGui::SliderFloat("Chain Efficiency", &chainEfficiency, 0.85f, 0.98f, "%.2f");

            ImGui::Separator();

            // Axle Design Section
            ImGui::TextUnformatted("Axle & Bearings");
            ImGui::SliderFloat("Axle Diameter (mm)", &axleDiameter_mm, 15.0f, 40.0f, "%.1f");
            ImGui::SliderFloat("Material Strength (MPa)", &axleMaterial_tensileStrength_MPa, 200.0f, 800.0f, "%.0f");
            ImGui::SliderFloat("Bearing Rating (N)", &bearingLoad_rating_N, 1000.0f, 5000.0f, "%.0f");

            ImGui::Separator();

            // Wheel Design Section
            ImGui::TextUnformatted("Wheel Design");
            ImGui::SliderFloat("Wheel Width (mm)", &wheelWidth_mm, 30.0f, 100.0f, "%.0f");
            ImGui::SliderFloat("Wheel Load Rating (kg)", &wheelLoad_rating_kg, 100.0f, 300.0f, "%.0f");

            ImGui::Separator();

            // Trolley Dimensions Section
            ImGui::TextUnformatted("Trolley Dimensions");
            ImGui::SliderFloat("Width (mm)", &trolleyWidth_mm, 400.0f, 1000.0f, "%.0f");
            ImGui::SliderFloat("Height (mm)", &trolleyHeight_mm, 600.0f, 1200.0f, "%.0f");
            ImGui::SliderFloat("Deck Height (mm)", &deckHeight_mm, 100.0f, 400.0f, "%.0f");
            ImGui::SliderFloat("Ground Clearance (mm)", &groundClearance_mm, 20.0f, 100.0f, "%.0f");

            ImGui::Separator();

            // Material & Safety Section
            ImGui::TextUnformatted("Materials & Safety");
            ImGui::SliderFloat("Chassis Density (kg/m³)", &chassisMaterial_density_kgm3, 2000.0f, 8000.0f, "%.0f");
            ImGui::SliderFloat("Thickness (mm)", &chassisThickness_mm, 1.5f, 8.0f, "%.1f");
            ImGui::SliderFloat("Safety Factor", &safetyFactor, 1.5f, 4.0f, "%.1f");

            ImGui::Separator();

            // Battery inputs kept for future runtime estimation; not used in dynamics
            if (ImGui::Button("Export CSV"))
            { /* TODO: export series */
            }
            ImGui::End();

            // Minimap window
            ImGui::Begin("Minimap");
            {
                ImDrawList *dlMap = ImGui::GetWindowDrawList();
                ImVec2 mapPos = ImGui::GetWindowPos();
                ImVec2 mapMin = ImVec2(mapPos.x + ImGui::GetWindowContentRegionMin().x, mapPos.y + ImGui::GetWindowContentRegionMin().y);
                ImVec2 mapMax = ImVec2(mapPos.x + ImGui::GetWindowContentRegionMax().x, mapPos.y + ImGui::GetWindowContentRegionMax().y);
                float mapW = mapMax.x - mapMin.x, mapH = mapMax.y - mapMin.y;

                // White background
                dlMap->AddRectFilled(mapMin, mapMax, IM_COL32(255, 255, 255, 255));
                dlMap->AddRect(mapMin, mapMax, IM_COL32(100, 100, 100, 255), 0.0f, 0, 2.0f);

                // Calculate map scale and center
                float mapRange = 50.0f; // show ±50m range
                float scaleX = (mapW - 20.0f) / (2.0f * mapRange);
                float scaleY = (mapH - 20.0f) / (2.0f * mapRange);
                float scale = std::min(scaleX, scaleY);
                ImVec2 mapCenter = ImVec2(mapMin.x + mapW * 0.5f, mapMin.y + mapH * 0.5f);

                // Draw grid
                for (int i = -5; i <= 5; ++i)
                {
                    float gridStep = 10.0f; // 10m grid
                    float x = mapCenter.x + i * gridStep * scale;
                    float y = mapCenter.y + i * gridStep * scale;
                    ImU32 gridCol = (i == 0) ? IM_COL32(100, 100, 100, 180) : IM_COL32(180, 180, 180, 120);
                    if (x >= mapMin.x && x <= mapMax.x)
                        dlMap->AddLine(ImVec2(x, mapMin.y + 10), ImVec2(x, mapMax.y - 10), gridCol, (i == 0) ? 2.0f : 1.0f);
                    if (y >= mapMin.y && y <= mapMax.y)
                        dlMap->AddLine(ImVec2(mapMin.x + 10, y), ImVec2(mapMax.x - 10, y), gridCol, (i == 0) ? 2.0f : 1.0f);
                }

                // Draw trolley position
                ImVec2 trolleyMap = ImVec2(mapCenter.x + positionX_m * scale, mapCenter.y - positionY_m * scale);
                dlMap->AddCircleFilled(trolleyMap, 4.0f, IM_COL32(180, 20, 20, 255)); // Dark red

                // Draw velocity vector
                if (std::sqrt(velocityX_mps * velocityX_mps + velocityY_mps * velocityY_mps) > 0.1f)
                {
                    float velScale = 20.0f; // scale for velocity visualization
                    ImVec2 velEnd = ImVec2(trolleyMap.x + velocityX_mps * velScale, trolleyMap.y - velocityY_mps * velScale);
                    dlMap->AddLine(trolleyMap, velEnd, IM_COL32(20, 80, 180, 255), 3.0f); // Dark blue
                    // Arrow head
                    ImVec2 dir = ImVec2(velEnd.x - trolleyMap.x, velEnd.y - trolleyMap.y);
                    float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                    if (len > 5.0f)
                    {
                        dir.x /= len;
                        dir.y /= len;
                        ImVec2 ort = ImVec2(-dir.y, dir.x);
                        ImVec2 h1 = ImVec2(velEnd.x - 6 * dir.x + 3 * ort.x, velEnd.y - 6 * dir.y + 3 * ort.y);
                        ImVec2 h2 = ImVec2(velEnd.x - 6 * dir.x - 3 * ort.x, velEnd.y - 6 * dir.y - 3 * ort.y);
                        dlMap->AddTriangleFilled(velEnd, h1, h2, IM_COL32(20, 80, 180, 255)); // Dark blue
                    }
                }

                // Labels
                dlMap->AddText(ImVec2(mapMin.x + 5, mapMin.y + 5), IM_COL32(20, 20, 20, 255), "Minimap"); // Dark text
                char posBuf[64];
                snprintf(posBuf, sizeof(posBuf), "X:%.1fm Y:%.1fm", positionX_m, positionY_m);
                dlMap->AddText(ImVec2(mapMin.x + 5, mapMax.y - 20), IM_COL32(20, 20, 20, 255), posBuf); // Dark text
            }
            ImGui::End();

            // Design Calculations window
            ImGui::Begin("Design Calculations");
            {
                // Calculate useful engineering metrics
                float wheelload_per_wheel_N = (inputs.payloadKg + 20.0f) * 9.81f / 4.0f;
                float axle_stress_MPa = (wheelload_per_wheel_N * wheelbase_mm * 0.001f * 0.25f) / (3.14159f * std::pow(axleDiameter_mm * 0.001f, 3.0f) / 32.0f) / 1e6f;
                float motor_torque_available_Nm = motorMaxTorque_Nm * gearReduction * motorEfficiency * chainEfficiency;
                float max_tractive_force_N = motor_torque_available_Nm / (wheelRadius_mm * 0.001f);
                float trolley_volume_m3 = (trolleyWidth_mm * trolleyHeight_mm * wheelbase_mm) * 1e-9f;
                float estimated_mass_kg = trolley_volume_m3 * chassisMaterial_density_kgm3 * (chassisThickness_mm / 100.0f);

                ImGui::TextUnformatted("Loading Analysis");
                ImGui::Text("Load per wheel: %.0f N", wheelload_per_wheel_N);
                ImGui::Text("Wheel rating: %.0f N", wheelLoad_rating_kg * 9.81f);
                ImGui::Text("Load safety margin: %.1fx", (wheelLoad_rating_kg * 9.81f) / wheelload_per_wheel_N);

                ImGui::Separator();
                ImGui::TextUnformatted("Structural Analysis");
                ImGui::Text("Axle stress: %.1f MPa", axle_stress_MPa);
                ImGui::Text("Material limit: %.0f MPa", axleMaterial_tensileStrength_MPa / safetyFactor);
                ImGui::Text("Stress safety margin: %.1fx", (axleMaterial_tensileStrength_MPa / safetyFactor) / axle_stress_MPa);

                ImGui::Separator();
                ImGui::TextUnformatted("Power & Performance");
                ImGui::Text("Available motor torque: %.1f Nm", motor_torque_available_Nm);
                ImGui::Text("Max traction force: %.0f N", max_tractive_force_N);
                ImGui::Text("Power @ max speed: %.0f W", max_tractive_force_N * inputs.speed_mps);
                ImGui::Text("Motor power rating: %.0f W", motorPower_W);

                ImGui::Separator();
                ImGui::TextUnformatted("Mass Estimation");
                ImGui::Text("Estimated chassis mass: %.1f kg", estimated_mass_kg);
                ImGui::Text("Payload capacity: %.0f kg", inputs.payloadKg);
                ImGui::Text("Total system mass: %.1f kg", estimated_mass_kg + inputs.payloadKg);

                ImGui::Separator();
                ImGui::TextUnformatted("Design Status");

                // Safety indicators with detailed analysis
                bool wheelOverload = wheelload_per_wheel_N > (wheelLoad_rating_kg * 9.81f);
                bool axleOverstress = axle_stress_MPa > (axleMaterial_tensileStrength_MPa / safetyFactor);
                bool motorUnderpowered = (max_tractive_force_N * inputs.speed_mps) > motorPower_W;

                if (wheelOverload)
                {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "⚠ WHEEL OVERLOAD");
                    ImGui::Text("  Reduce payload or increase wheel rating");
                }
                if (axleOverstress)
                {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "⚠ AXLE OVERSTRESS");
                    ImGui::Text("  Increase axle diameter or use stronger material");
                }
                if (motorUnderpowered)
                {
                    ImGui::TextColored(ImVec4(1, 0.5, 0, 1), "⚠ MOTOR UNDERPOWERED");
                    ImGui::Text("  Increase motor power or reduce max speed");
                }
                if (!wheelOverload && !axleOverstress && !motorUnderpowered)
                {
                    ImGui::TextColored(ImVec4(0, 0.8, 0, 1), "✓ DESIGN OK");
                    ImGui::Text("  All components within safe operating limits");
                }

                // Additional engineering insights
                ImGui::Separator();
                ImGui::TextUnformatted("Design Recommendations");

                float power_to_weight = motorPower_W / (estimated_mass_kg + inputs.payloadKg);
                ImGui::Text("Power-to-weight ratio: %.1f W/kg", power_to_weight);

                if (power_to_weight < 3.0f)
                {
                    ImGui::TextColored(ImVec4(1, 0.5, 0, 1), "Consider lighter materials or more power");
                }
                else if (power_to_weight > 10.0f)
                {
                    ImGui::TextColored(ImVec4(0, 0.8, 0, 1), "Excellent power-to-weight ratio");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0, 0.8, 0, 1), "Good power-to-weight ratio");
                }
            }
            ImGui::End();

            // End main dockspace window and return (skip 3D)
            ImGui::End();
            return;
        }

        // Top menubar
        static bool reqStartDrill = false, reqStop = false, reqResetScene = false, reqClearPath = false;
        static bool reqPlanLeft = false, reqPlanRight = false, reqPlanDown = false, reqPlanUp = false;
        static bool reqPeriReset = false, reqPeriJam = false, reqPeriEStop = false, reqPeriAutoToggle = false;
        if (ImGui::BeginMenuBar())
        {
            if (ImGui::BeginMenu("Options"))
            {
                ImGui::MenuItem("Fullscreen", NULL, &opt_fullscreen);
                ImGui::MenuItem("Padding", NULL, &opt_padding);
                ImGui::Separator();

                if (ImGui::MenuItem("Flag: NoDockingOverCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_NoDockingOverCentralNode) != 0))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_NoDockingOverCentralNode;
                }
                if (ImGui::MenuItem("Flag: NoDockingSplit", "", (dockspace_flags & ImGuiDockNodeFlags_NoDockingSplit) != 0))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_NoDockingSplit;
                }
                if (ImGui::MenuItem("Flag: NoUndocking", "", (dockspace_flags & ImGuiDockNodeFlags_NoUndocking) != 0))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_NoUndocking;
                }
                if (ImGui::MenuItem("Flag: NoResize", "", (dockspace_flags & ImGuiDockNodeFlags_NoResize) != 0))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_NoResize;
                }
                if (ImGui::MenuItem("Flag: AutoHideTabBar", "", (dockspace_flags & ImGuiDockNodeFlags_AutoHideTabBar) != 0))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_AutoHideTabBar;
                }
                if (ImGui::MenuItem("Flag: PassthruCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode) != 0, opt_fullscreen))
                {
                    dockspace_flags ^= ImGuiDockNodeFlags_PassthruCentralNode;
                }
                ImGui::Separator();
                ImGui::EndMenu();
            }

            ImGui::EndMenuBar();
        }

        // Viewport
        ImGui::Begin("Viewport");
        ImDrawList *dl = ImGui::GetWindowDrawList();
        ImVec2 win_pos = ImGui::GetWindowPos();
        ImVec2 content_min = ImGui::GetWindowContentRegionMin();
        ImVec2 content_max = ImGui::GetWindowContentRegionMax();
        ImVec2 vp_min = ImVec2(win_pos.x + content_min.x, win_pos.y + content_min.y);
        ImVec2 vp_max = ImVec2(win_pos.x + content_max.x, win_pos.y + content_max.y);
        float vp_w = vp_max.x - vp_min.x;
        float vp_h = vp_max.y - vp_min.y;
        // Anti-aliasing / smoothing controls
        static bool aa_lines = true;
        static bool aa_fill = true;
        static bool aa_pixel_snap = true;
        if (aa_lines)
            dl->Flags |= ImDrawListFlags_AntiAliasedLines;
        else
            dl->Flags &= ~ImDrawListFlags_AntiAliasedLines;
        if (aa_fill)
            dl->Flags |= ImDrawListFlags_AntiAliasedFill;
        else
            dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
        ImGui::GetStyle().AntiAliasedLines = aa_lines;
        ImGui::GetStyle().AntiAliasedFill = aa_fill;
        dl->AddRect(vp_min, vp_max, IM_COL32(80, 80, 80, 255));

        // Static scene and camera state across frames
        static auto scene = Engine::SceneBuilder::CreateDefaultScene();
        static Inputs inputs{};
        static Outputs outputs{};
        static std::deque<ImVec2> sTorqueTime;
        static std::deque<ImVec2> sSpeedDist;
        static std::deque<ImVec2> sThermalMotor;
        static float simTime_s = 0.0f;
        static float simDist_m = 0.0f;
        static float thermalMotor_C = 0.0f;
        struct SpawnedAtom
        {
            int atomIndex;
            std::vector<int> electronIdx;
            std::vector<int> orbitalIdx;
            int Z;
            std::string symbol;
            bool alive;
        };
        static std::vector<SpawnedAtom> spawned;
        static float yaw = 0.0f;            // smoothed radians around Y
        static float pitch = 0.0f;          // smoothed radians around X
        static float cameraDistance = 5.0f; // smoothed
        static Engine::Math::Vec3 camTarget{0.0f, 0.0f, 0.0f};
        static float yaw_t = 0.0f; // targets
        static float pitch_t = 0.0f;
        static float cameraDistance_t = 5.0f;
        static ImVec2 last_mouse = ImVec2(0, 0);
        static bool dragging = false;

        // Cutter head persistent UI and animation state
        static float ui_radius_m = 0.075f;
        static float ui_length_m = 0.6f;
        static int ui_teeth = 16;
        static int ui_profile_idx = 0;
        static const char *kProfiles[] = {"carbide", "diamond", "pdc"};
        static float ui_rpm = 0.0f;                                  // default stationary
        static float cutter_phase = 0.0f;                            // radians
        static std::vector<int> toothIndices;                        // indices of tooth renderables in scene.renderables
        static float sj_pitch_deg = 0.0f, sj_yaw_deg = 0.0f;         // targets from input
        static float sj_pitch_cur_deg = 0.0f, sj_yaw_cur_deg = 0.0f; // rate-limited applied
        // Unlimited operator steering; limits removed per operator responsibility
        static float yawZeroBase_deg = 0.0f, pitchZeroBase_deg = 0.0f;
        static bool spinning = false;
        static float forward_gain_m_per_rev = 0.01f;               // advance per revolution
        static float minBendRadius_m = 1.5f;                       // curvature limit
        static float yawRateDeg_s = 25.0f, pitchRateDeg_s = 25.0f; // base steering rates
        // Peristalsis FSM
        enum class PeriState : int
        {
            IDLE = 0,
            ANCHOR_LOCK = 1,
            STROKE = 2,
            MID_LOCK = 3,
            RETRACT = 4,
            JAM_RECOVERY = 5,
            EMERGENCY_STOP = 6,
        };
        enum class PeriEvent : int
        {
            FRONT_LOCKED = 0,
            STROKE_DONE = 1,
            MID_LOCKED = 2,
            RETRACT_DONE = 3,
            JAM = 4,
            EMERGENCY_STOP = 5,
            RESET = 6,
        };
        static PeriState peristalsisState = PeriState::IDLE;
        static bool peristalsisAuto = false;
        static float periElapsedSec = 0.0f;
        static float periT_AnchorLock = 0.8f;
        static float periT_Stroke = 1.8f;
        static float periT_MidLock = 0.6f;
        static float periT_Retract = 1.2f;
        auto periStateName = [&](PeriState s) -> const char *
        {
            switch (s)
            {
            case PeriState::IDLE:
                return "IDLE";
            case PeriState::ANCHOR_LOCK:
                return "ANCHOR LOCK";
            case PeriState::STROKE:
                return "STROKE";
            case PeriState::MID_LOCK:
                return "MID LOCK";
            case PeriState::RETRACT:
                return "RETRACT";
            case PeriState::JAM_RECOVERY:
                return "JAM RECOVERY";
            case PeriState::EMERGENCY_STOP:
                return "EMERGENCY STOP";
            }
            return "?";
        };
        auto periColor = [&](PeriState s) -> ImU32
        {
            switch (s)
            {
            case PeriState::IDLE:
                return IM_COL32(200, 200, 210, 230);
            case PeriState::ANCHOR_LOCK:
                return IM_COL32(255, 180, 80, 230);
            case PeriState::STROKE:
                return IM_COL32(120, 255, 140, 230);
            case PeriState::MID_LOCK:
                return IM_COL32(120, 220, 255, 230);
            case PeriState::RETRACT:
                return IM_COL32(250, 235, 120, 230);
            case PeriState::JAM_RECOVERY:
                return IM_COL32(255, 150, 60, 230);
            case PeriState::EMERGENCY_STOP:
                return IM_COL32(255, 80, 80, 230);
            }
            return IM_COL32(220, 220, 220, 230);
        };
        auto periHandleEvent = [&](PeriEvent ev)
        {
            // RESET always returns to IDLE
            if (ev == PeriEvent::RESET)
            {
                peristalsisState = PeriState::IDLE;
                periElapsedSec = 0.0f;
                return;
            }
            // JAM/E-STOP preempt from any state
            if (ev == PeriEvent::JAM)
            {
                peristalsisState = PeriState::JAM_RECOVERY;
                periElapsedSec = 0.0f;
                return;
            }
            if (ev == PeriEvent::EMERGENCY_STOP)
            {
                peristalsisState = PeriState::EMERGENCY_STOP;
                periElapsedSec = 0.0f;
                return;
            }
            // Normal loop
            switch (peristalsisState)
            {
            case PeriState::IDLE:
                if (ev == PeriEvent::FRONT_LOCKED)
                {
                    peristalsisState = PeriState::ANCHOR_LOCK;
                    periElapsedSec = 0.0f;
                }
                break;
            case PeriState::ANCHOR_LOCK:
                if (ev == PeriEvent::FRONT_LOCKED)
                {
                    peristalsisState = PeriState::STROKE;
                    periElapsedSec = 0.0f;
                }
                break;
            case PeriState::STROKE:
                if (ev == PeriEvent::STROKE_DONE)
                {
                    peristalsisState = PeriState::MID_LOCK;
                    periElapsedSec = 0.0f;
                }
                break;
            case PeriState::MID_LOCK:
                if (ev == PeriEvent::MID_LOCKED)
                {
                    peristalsisState = PeriState::RETRACT;
                    periElapsedSec = 0.0f;
                }
                break;
            case PeriState::RETRACT:
                if (ev == PeriEvent::RETRACT_DONE)
                {
                    peristalsisState = PeriState::ANCHOR_LOCK;
                    periElapsedSec = 0.0f;
                }
                break;
            case PeriState::JAM_RECOVERY:
            case PeriState::EMERGENCY_STOP:
                // only RESET handled above
                break;
            }
        };
        // Autopilot: drill forward target meters
        static bool autopilotActive = false;
        static float autopilotTarget_m = 0.0f;
        static float autopilotProgress_m = 0.0f;
        static Engine::Math::Vec3 lastHeadPosForProgress{1e9f, 1e9f, 1e9f};
        // Obstacle and 90°-around autopilot plan
        static Engine::Math::Vec3 obstacleCenter{1.5f, -0.2f, 0.0f};
        static float obstacleRadius_m = 0.35f;
        static float obstacleClearance_m = 0.25f;
        static bool obstacleVisible = true;
        static float autopilotTurnRadius_m = 1.0f; // will be clamped to >= minBendRadius_m
        enum
        {
            SegStraight = 0,
            SegTurn = 1
        };
        struct APSegment
        {
            int kind;
            float length_m;
            bool pitchAxis;     // false=yaw turn, true=pitch turn
            float targetAbsDeg; // absolute target (yaw or pitch)
        };
        static std::vector<APSegment> apPlan;
        static int apIndex = -1;
        static float apSegProgress_m = 0.0f;
        static float apHoldYaw_deg = 0.0f;   // heading hold during straights
        static float apHoldPitch_deg = 0.0f; // pitch hold during straights

        // Mesh grid settings (separate viewport)
        static int gridNodesX = 10, gridNodesY = 6, gridNodesZ = 10;
        static float gridSpacingM = 0.6f;
        // Mesh grid origin zeroing and path trace
        static Engine::Math::Vec3 gridZero{0.0f, 0.0f, 0.0f};
        static std::vector<Engine::Math::Vec3> headPath;
        static Engine::Math::Vec3 lastPathSample{1e9f, 1e9f, 1e9f};
        static float pathSampleEveryM = 0.05f;
        // Body following: trail of past poses and spawned trailing segments
        struct Pose
        {
            Engine::Math::Vec3 p;
            float yawDeg;
            float pitchDeg;
        };
        static std::deque<Pose> poseDeque;
        static float segmentSpacing_m = 0.5f;     // base spacing (not used directly for collision-free placement)
        static float trail_length_m = 0.5f;       // realistic trailing segment center length (capsule)
        static float spacing_clearance_m = 0.03f; // small gap to prevent overlap
        static int numTrailingSegments = 4;
        static std::vector<int> trailingIdx; // indices of trailing segment renderables

        auto smoothTowards = [&](float current, float target, float dt, float tau)
        {
            if (tau <= 0.0f)
                return target;
            float alpha = 1.0f - std::exp(-dt / tau);
            return current + (target - current) * alpha;
        };

        // Handle input inside viewport
        bool hovered = ImGui::IsWindowHovered();
        bool active = ImGui::IsWindowFocused();
        if (hovered && active)
        {
            float scroll = ImGui::GetIO().MouseWheel;
            if (scroll != 0.0f)
            {
                cameraDistance_t -= scroll * 0.5f;
                if (cameraDistance_t < 0.25f)
                    cameraDistance_t = 0.25f;
                if (cameraDistance_t > 100.0f)
                    cameraDistance_t = 100.0f;
            }

            bool leftDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
            bool middleDown = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
            bool rightDown = ImGui::IsMouseDown(ImGuiMouseButton_Right);
            bool shift = ImGui::GetIO().KeyShift;
            bool ctrl = ImGui::GetIO().KeyCtrl;
            bool orbiting = (middleDown || rightDown || (leftDown && !shift));
            bool panning = ((shift || ctrl) && (leftDown || middleDown));

            if ((leftDown || middleDown) && !dragging)
            {
                dragging = true;
                last_mouse = ImGui::GetMousePos();
            }
            else if (!(leftDown || middleDown) && dragging)
            {
                dragging = false;
            }

            if (dragging)
            {
                ImVec2 cur = ImGui::GetMousePos();
                ImVec2 delta = ImVec2(cur.x - last_mouse.x, cur.y - last_mouse.y);
                last_mouse = cur;

                if (panning)
                {
                    // Pan camera target in view plane based on pixel delta
                    float viewHeightAtDist = 2.0f * std::tan(scene.camera.fovYRadians * 0.5f) * cameraDistance;
                    float pixelsPerWorldY = (vp_h > 0.0f) ? (vp_h / viewHeightAtDist) : 1.0f;
                    float dy_world = -delta.y / pixelsPerWorldY;
                    float dx_world = -delta.x / pixelsPerWorldY; // approximate using Y scaling
                    float cosY = std::cos(yaw), sinY = std::sin(yaw);
                    Engine::Math::Vec3 right{sinY, 0.0f, cosY};
                    Engine::Math::Vec3 up{0.0f, 1.0f, 0.0f};
                    camTarget = camTarget + right * dx_world + up * dy_world;
                }
                else if (orbiting)
                {
                    yaw_t += delta.x * 0.01f;
                    pitch_t += delta.y * 0.01f;
                    if (pitch_t > 1.5f)
                        pitch_t = 1.5f;
                    if (pitch_t < -1.5f)
                        pitch_t = -1.5f;
                }
            }
        }

        // Smooth towards targets
        float dt = ImGui::GetIO().DeltaTime;
        const float tau_orbit = 0.15f; // seconds to ~63% toward target
        const float tau_zoom = 0.12f;
        yaw = smoothTowards(yaw, yaw_t, dt, tau_orbit);
        pitch = smoothTowards(pitch, pitch_t, dt, tau_orbit);
        cameraDistance = smoothTowards(cameraDistance, cameraDistance_t, dt, tau_zoom);

        // Recompute dashboard outputs and series
        outputs.wheelTorque_Nm = Sim::ComputeWheelTorquePerWheel(inputs.payloadKg, inputs.speed_mps, inputs.slope_deg, inputs.mu, 4);
        outputs.stoppingDistance_m = Sim::ComputeStoppingDistance(inputs.speed_mps, inputs.mu, inputs.slope_deg);
        float powerW = outputs.wheelTorque_Nm * 4.0f * (inputs.speed_mps / 0.15f);
        float currentA = powerW / std::max(1.0f, inputs.battVoltage_V * 0.85f);
        outputs.runtime_hours = Sim::EstimateRuntimeHours(inputs.battCapacity_Ah, inputs.battVoltage_V, inputs.dischargeC, currentA);
        simTime_s += dt;
        simDist_m += inputs.speed_mps * dt;
        sTorqueTime.emplace_back(simTime_s, outputs.wheelTorque_Nm);
        sSpeedDist.emplace_back(simDist_m, inputs.speed_mps);
        thermalMotor_C += powerW * 0.0005f * dt; // placeholder thermal model
        sThermalMotor.emplace_back(simTime_s, 25.0f + thermalMotor_C);
        if ((int)sTorqueTime.size() > 1024)
            sTorqueTime.pop_front();
        if ((int)sSpeedDist.size() > 1024)
            sSpeedDist.pop_front();
        if ((int)sThermalMotor.size() > 1024)
            sThermalMotor.pop_front();

        // Keyboard input: locomotion and steering
        if (active)
        {
            // Space toggles spinning
            if (ImGui::IsKeyPressed(ImGuiKey_Space))
                spinning = !spinning;
            // RPM adjust with +/- (or = and -)
            if (ImGui::IsKeyDown(ImGuiKey_Equal) || ImGui::IsKeyDown(ImGuiKey_KeypadAdd))
                ui_rpm += 60.0f * dt;
            if (ImGui::IsKeyDown(ImGuiKey_Minus) || ImGui::IsKeyDown(ImGuiKey_KeypadSubtract))
            {
                ui_rpm -= 60.0f * dt;
                if (ui_rpm < 0.0f)
                    ui_rpm = 0.0f;
            }
            // Steering pivot with WASD/Arrows (affects joint angles)
            float steerRateDeg = 45.0f; // deg/s
            if (ImGui::IsKeyDown(ImGuiKey_A) || ImGui::IsKeyDown(ImGuiKey_LeftArrow))
                sj_yaw_deg -= steerRateDeg * dt;
            if (ImGui::IsKeyDown(ImGuiKey_D) || ImGui::IsKeyDown(ImGuiKey_RightArrow))
                sj_yaw_deg += steerRateDeg * dt;
            if (ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_UpArrow))
                sj_pitch_deg += steerRateDeg * dt;
            if (ImGui::IsKeyDown(ImGuiKey_S) || ImGui::IsKeyDown(ImGuiKey_DownArrow))
                sj_pitch_deg -= steerRateDeg * dt;
            // No clamps: operator is responsible for respecting physical limits
        }

        // Prevent camera from moving "into" the worm segment by clamping distance
        if (!scene.renderables.empty())
        {
            // Ray from camTarget to camera: d is unit direction used below for camera position
            Engine::Math::Vec3 d{
                std::cos(pitch) * std::sin(yaw),
                std::sin(pitch),
                std::cos(pitch) * std::cos(yaw)};

            const Engine::Math::Vec3 wormPos = scene.renderables[0].transform.position;
            // Estimate bounding sphere radius from current mesh (scaled)
            float headMaxR = 0.0f;
            float headMaxScale = 1.0f;
            {
                const auto &mesh = scene.resources.headMesh;
                for (const auto &v : mesh.vertices)
                {
                    float r2 = v.px * v.px + v.py * v.py + v.pz * v.pz;
                    if (r2 > headMaxR)
                        headMaxR = r2;
                }
                headMaxR = (headMaxR > 0.0f) ? std::sqrt(headMaxR) : 0.0f;
                const auto &s = scene.renderables[0].transform.scale;
                headMaxScale = std::max({std::abs(s.x), std::abs(s.y), std::abs(s.z)});
            }
            float headBoundRadius = headMaxR * headMaxScale;
            float margin = std::max(0.02f, scene.camera.nearPlane * 2.0f);

            // Sphere-ray intersection from camTarget toward camera
            Engine::Math::Vec3 oc = {camTarget.x - wormPos.x, camTarget.y - wormPos.y, camTarget.z - wormPos.z};
            float b = Engine::Math::dot(d, oc);
            float c = Engine::Math::dot(oc, oc) - headBoundRadius * headBoundRadius;
            float disc = b * b - c;
            if (disc >= 0.0f)
            {
                float sqrtDisc = std::sqrt(disc);
                float tEnter = -b - sqrtDisc;
                float tExit = -b + sqrtDisc;
                if (cameraDistance >= tEnter && cameraDistance <= tExit)
                {
                    float clamped = tExit + margin;
                    if (clamped < 0.0f)
                        clamped = margin;
                    cameraDistance = clamped;
                    if (cameraDistance_t < clamped)
                        cameraDistance_t = clamped;
                }
            }

            // Also clamp to trailing segments (approximate using same radius but along body path)
            if (!poseDeque.empty())
            {
                // Check nearest few poses
                int checkN = std::min((int)poseDeque.size(), 24);
                // Compute a bound radius for trailing segments (capsule mesh)
                float trailMaxR = 0.0f;
                {
                    const auto &tmesh = scene.resources.trailingCapsuleMesh;
                    for (const auto &v : tmesh.vertices)
                    {
                        float r2 = v.px * v.px + v.py * v.py + v.pz * v.pz;
                        if (r2 > trailMaxR)
                            trailMaxR = r2;
                    }
                    trailMaxR = (trailMaxR > 0.0f) ? std::sqrt(trailMaxR) : 0.0f;
                }
                float trailBoundRadius = trailMaxR; // trailing segments use unit scale
                for (int i = 0; i < checkN; ++i)
                {
                    Engine::Math::Vec3 p = poseDeque[i].p;
                    Engine::Math::Vec3 occ = {camTarget.x - p.x, camTarget.y - p.y, camTarget.z - p.z};
                    float bb = Engine::Math::dot(d, occ);
                    float cc = Engine::Math::dot(occ, occ) - trailBoundRadius * trailBoundRadius;
                    float dd = bb * bb - cc;
                    if (dd >= 0.0f)
                    {
                        float srt = std::sqrt(dd);
                        float tEn = -bb - srt;
                        float tEx = -bb + srt;
                        if (cameraDistance >= tEn && cameraDistance <= tEx)
                        {
                            float clamp2 = tEx + margin;
                            if (clamp2 > cameraDistance)
                                cameraDistance = clamp2;
                            if (clamp2 > cameraDistance_t)
                                cameraDistance_t = clamp2;
                        }
                    }
                }
            }
        }

        // Trolley kinematics: move chassis forward along +Z and spin wheels based on speed & mu
        if (scene.renderables.size() >= 6)
        {
            // 0 ground, 1 chassis, 2..5 wheels
            auto &chassis = scene.renderables[1];
            float slope = inputs.slope_deg * 3.1415926535f / 180.0f;
            auto clampf2 = [&](float x, float a, float b)
            { return x < a ? a : (x > b ? b : x); };
            float v = inputs.speed_mps * (0.2f + 0.8f * clampf2(inputs.mu, 0.0f, 1.0f));
            v += -std::sin(slope) * 0.4f;
            if (v < 0.0f)
                v = 0.0f;
            if (v > 2.0f)
                v = 2.0f;
            chassis.transform.position.z += v * dt;
            camTarget = {chassis.transform.position.x, chassis.transform.position.y + 0.2f, chassis.transform.position.z + 1.2f};

            float r = 0.15f;
            float omega = v / std::max(0.05f, r);
            static float wphase = 0.0f;
            wphase += omega * dt;
            if (wphase > 1000.0f)
                wphase -= 1000.0f;
            for (int wi = 2; wi <= 5; ++wi)
            {
                auto &w = scene.renderables[wi];
                // Spin around X; wheel mesh pre-rotated around Y by 90deg in builder
                w.transform.rotationEulerRad = {wphase, 1.57079632679f, 0.0f};
                float localX = (wi % 2 == 0) ? -0.35f : 0.35f;
                float localZ = (wi <= 3) ? 0.55f : -0.55f;
                w.transform.position = {localX, 0.0f, chassis.transform.position.z + localZ};
            }
        }

        // Update camera transform from yaw/pitch and distance
        scene.camera.aspect = (vp_h > 0.0f) ? (vp_w / vp_h) : scene.camera.aspect;
        float cx = camTarget.x + cameraDistance * std::cos(pitch) * std::sin(yaw);
        float cy = camTarget.y + cameraDistance * std::sin(pitch);
        float cz = camTarget.z + cameraDistance * std::cos(pitch) * std::cos(yaw);
        scene.cameraTransform.position = {cx, cy, cz};
        scene.cameraTransform.rotationEulerRad = {0.0f, 0.0f, 0.0f};

        if (!scene.renderables.empty())
        {
            auto &head = scene.renderables[0];
            // Soil resistance stub (depth-based)
            auto soilResist = [&](const Engine::Math::Vec3 &p) -> float
            {
                float depth = (p.y < 0.0f) ? (-p.y) : 0.0f; // deeper = harder
                float base = 0.3f;                          // baseline resistance
                return base + 0.4f * depth;                 // simple linear model
            };

            // Optional: autopilot override (yaw/pitch) set relative targets based on absolute desired
            if (autopilotActive && apIndex >= 0 && apIndex < (int)apPlan.size())
            {
                const auto &seg = apPlan[apIndex];
                float desiredYawAbs = apHoldYaw_deg;
                float desiredPitchAbs = apHoldPitch_deg;
                if (seg.kind == SegTurn)
                {
                    if (seg.pitchAxis)
                        desiredPitchAbs = seg.targetAbsDeg;
                    else
                        desiredYawAbs = seg.targetAbsDeg;
                }
                sj_yaw_deg = desiredYawAbs - yawZeroBase_deg;
                sj_pitch_deg = desiredPitchAbs - pitchZeroBase_deg;
            }

            // Rate-limit the applied joint angles based on soil and curvature limit
            float resist = soilResist(head.transform.position);
            float rateScale = 1.0f / (1.0f + 0.5f * resist); // 0..1
            float maxYawStep = yawRateDeg_s * rateScale * dt;
            float maxPitStep = pitchRateDeg_s * rateScale * dt;
            // Curvature limit: allowed combined angular rate ~ v / Rmin
            float mps_inst = (spinning && ui_rpm > 0.0f && peristalsisState == PeriState::STROKE) ? ((ui_rpm / 60.0f) * forward_gain_m_per_rev) : 0.0f;
            float allowedRateDeg = (minBendRadius_m > 0.01f) ? (mps_inst / minBendRadius_m * 57.2957795f) : 180.0f;
            if (maxYawStep > allowedRateDeg * dt)
                maxYawStep = allowedRateDeg * dt;
            if (maxPitStep > allowedRateDeg * dt)
                maxPitStep = allowedRateDeg * dt;
            auto stepTowards = [](float cur, float tgt, float maxStep)
            { float d = tgt - cur; if (d >  maxStep) d =  maxStep; if (d < -maxStep) d = -maxStep; return cur + d; };
            sj_yaw_cur_deg = stepTowards(sj_yaw_cur_deg, sj_yaw_deg, maxYawStep);
            sj_pitch_cur_deg = stepTowards(sj_pitch_cur_deg, sj_pitch_deg, maxPitStep);

            // Align head orientation (pitch/yaw) and add roll for spin
            {
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                head.transform.rotationEulerRad = {pr, yr, cutter_phase};
            }

            // Advance head forward along its current local +Z if spinning with RPM and peristalsis allows
            bool allowAdvance = true;
            if (peristalsisState == PeriState::IDLE || peristalsisState == PeriState::EMERGENCY_STOP)
                allowAdvance = false;
            if (peristalsisState == PeriState::JAM_RECOVERY)
                allowAdvance = false;
            if (peristalsisState == PeriState::RETRACT)
                allowAdvance = false;
            if (spinning && ui_rpm > 0.0f && allowAdvance)
            {
                float mps = (ui_rpm / 60.0f) * forward_gain_m_per_rev; // m/s
                float speedScale = 1.0f / (1.0f + 0.7f * resist);
                mps *= speedScale;
                // Use joint pitch/yaw to derive forward direction (head axis)
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                // Forward in world (approx): rotate (0,0,1) by yaw then pitch
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 old = head.transform.position;
                head.transform.position = head.transform.position + fwd * (mps * dt);
                // Autopilot progress
                if (lastHeadPosForProgress.x > 1e8f)
                    lastHeadPosForProgress = old;
                float dx = head.transform.position.x - lastHeadPosForProgress.x;
                float dy = head.transform.position.y - lastHeadPosForProgress.y;
                float dz = head.transform.position.z - lastHeadPosForProgress.z;
                float dmove = std::sqrt(dx * dx + dy * dy + dz * dz);
                autopilotProgress_m += dmove;
                lastHeadPosForProgress = head.transform.position;
                if (autopilotActive && apPlan.empty() && autopilotTarget_m > 0.0f && autopilotProgress_m >= autopilotTarget_m)
                {
                    spinning = false;                   // stop cutting
                    peristalsisState = PeriState::IDLE; // lock
                    autopilotActive = false;
                }

                // Update obstacle-avoidance plan segment progression
                if (autopilotActive && apIndex >= 0 && apIndex < (int)apPlan.size())
                {
                    const auto &seg = apPlan[apIndex];
                    if (seg.kind == SegStraight)
                    {
                        apSegProgress_m += dmove;
                        if (apSegProgress_m >= seg.length_m)
                        {
                            apIndex++;
                            apSegProgress_m = 0.0f;
                        }
                    }
                    else // SegTurn
                    {
                        float err = seg.pitchAxis ? std::fabs(sj_pitch_cur_deg - seg.targetAbsDeg)
                                                  : std::fabs(sj_yaw_cur_deg - seg.targetAbsDeg);
                        if (err <= 1.0f)
                        {
                            if (seg.pitchAxis)
                                apHoldPitch_deg = seg.targetAbsDeg;
                            else
                                apHoldYaw_deg = seg.targetAbsDeg;
                            apIndex++;
                        }
                    }
                    if (apIndex >= (int)apPlan.size())
                    {
                        autopilotActive = false;
                    }
                }
            }

            // Initialize pre-positioned body behind head on first run
            if (poseDeque.empty())
            {
                // Use current head axis to lay out a straight body behind
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 p = head.transform.position;
                poseDeque.clear();
                poseDeque.push_back(Pose{p, sj_yaw_cur_deg, sj_pitch_cur_deg});
                int total = numTrailingSegments + 6;
                for (int i = 1; i <= total; ++i)
                {
                    Engine::Math::Vec3 pi{p.x - fwd.x * (i * segmentSpacing_m),
                                          p.y - fwd.y * (i * segmentSpacing_m),
                                          p.z - fwd.z * (i * segmentSpacing_m)};
                    poseDeque.push_back(Pose{pi, sj_yaw_cur_deg, sj_pitch_cur_deg});
                }
            }

            // Path trace: append when moved enough and only when actually moving
            const Engine::Math::Vec3 hp = head.transform.position;
            auto dist2 = [&](const Engine::Math::Vec3 &a, const Engine::Math::Vec3 &b)
            { float dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z; return dx*dx+dy*dy+dz*dz; };
            if (spinning && ui_rpm > 0.0f)
            {
                if (lastPathSample.x > 1e8f)
                {
                    headPath.push_back(hp);
                    lastPathSample = hp;
                }
                else if (dist2(hp, lastPathSample) >= pathSampleEveryM * pathSampleEveryM)
                {
                    headPath.push_back(hp);
                    lastPathSample = hp;
                }
            }

            // Camera follow: center camera target on head in both viewports
            camTarget = head.transform.position;
        }

        // Ensure yellow teeth exist and are placed around the cutter head rim
        if (!scene.renderables.empty())
        {
            const int headIndex = 0;
            Engine::Math::Vec3 headPos = scene.renderables[headIndex].transform.position;
            float radius = ui_radius_m;
            float zFront = ui_length_m * 0.5f;
            Engine::Math::Vec3 toothScale{radius * 0.15f, radius * 0.06f, radius * 0.25f};

            // Grow tooth list as needed
            if ((int)toothIndices.size() < ui_teeth)
            {
                for (int i = (int)toothIndices.size(); i < ui_teeth; ++i)
                {
                    Engine::Scene::RenderableEntity tooth;
                    tooth.mesh = &scene.resources.toothBoxMesh;
                    tooth.material.useVertexColor = false;
                    tooth.material.r = 1.0f;
                    tooth.material.g = 0.85f;
                    tooth.material.b = 0.10f; // yellow
                    tooth.opacity.alpha = 1.0f;
                    tooth.style = Engine::ECS::RenderStyle::Solid;
                    scene.renderables.push_back(tooth);
                    toothIndices.push_back((int)scene.renderables.size() - 1);
                }
            }
            // Hide extras if teeth reduced
            if ((int)toothIndices.size() > ui_teeth)
            {
                for (int i = ui_teeth; i < (int)toothIndices.size(); ++i)
                {
                    int idx = toothIndices[i];
                    if (idx >= 0 && idx < (int)scene.renderables.size())
                        scene.renderables[idx].style = Engine::ECS::RenderStyle::Hidden;
                }
                toothIndices.resize(ui_teeth);
            }

            float eps = toothScale.z * 0.6f; // slight protrusion beyond front cap
            // Head basis for teeth
            float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
            float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
            float cp = std::cos(pr), sp = std::sin(pr);
            float cyh = std::cos(yr), syh = std::sin(yr);
            Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
            Engine::Math::Vec3 ref{0.0f, 1.0f, 0.0f};
            float dot_up = fwd.x * ref.x + fwd.y * ref.y + fwd.z * ref.z;
            if (std::fabs(dot_up) > 0.95f)
                ref = Engine::Math::Vec3{1.0f, 0.0f, 0.0f};
            auto cross = [](const Engine::Math::Vec3 &a, const Engine::Math::Vec3 &b)
            { return Engine::Math::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; };
            auto norm = [&](const Engine::Math::Vec3 &vv)
            { float l=std::sqrt(vv.x*vv.x+vv.y*vv.y+vv.z*vv.z); return (l>1e-6f)? Engine::Math::Vec3{vv.x/l,vv.y/l,vv.z/l} : Engine::Math::Vec3{1,0,0}; };
            Engine::Math::Vec3 u = norm(cross(ref, fwd));
            Engine::Math::Vec3 v = cross(fwd, u);
            Engine::Math::Vec3 cFront{headPos.x + fwd.x * (zFront + eps), headPos.y + fwd.y * (zFront + eps), headPos.z + fwd.z * (zFront + eps)};

            int activeCount = (int)toothIndices.size();
            for (int i = 0; i < activeCount; ++i)
            {
                int idx = toothIndices[i];
                float ang = (2.0f * 3.1415926535f) * (float)i / (float)std::max(1, ui_teeth) + cutter_phase;
                float ca = std::cos(ang), sa = std::sin(ang);
                Engine::Math::Vec3 ring{u.x * (radius * ca) + v.x * (radius * sa), u.y * (radius * ca) + v.y * (radius * sa), u.z * (radius * ca) + v.z * (radius * sa)};
                auto &tooth = scene.renderables[idx];
                tooth.style = Engine::ECS::RenderStyle::Solid;
                tooth.transform.scale = toothScale;
                // Align with head yaw/pitch and roll by ang
                tooth.transform.rotationEulerRad = {pr, yr, ang};
                tooth.transform.position = {cFront.x + ring.x, cFront.y + ring.y, cFront.z + ring.z};
            }
        }

        // Body following: maintain deque of poses and place trailing segments
        if (!scene.renderables.empty())
        {
            auto &head = scene.renderables[0];
            Pose headPose{head.transform.position, sj_yaw_cur_deg, sj_pitch_cur_deg};
            if (poseDeque.empty() || (poseDeque.front().p.x - headPose.p.x) * (poseDeque.front().p.x - headPose.p.x) +
                                             (poseDeque.front().p.y - headPose.p.y) * (poseDeque.front().p.y - headPose.p.y) +
                                             (poseDeque.front().p.z - headPose.p.z) * (poseDeque.front().p.z - headPose.p.z) >
                                         0.01f * 0.01f)
            {
                poseDeque.push_front(headPose);
                // keep reasonable length
                while ((int)poseDeque.size() > 400)
                    poseDeque.pop_back();
            }

            // allocate trailing segments (solid but nearly transparent)
            if ((int)trailingIdx.size() < numTrailingSegments)
            {
                for (int i = (int)trailingIdx.size(); i < numTrailingSegments; ++i)
                {
                    Engine::Scene::RenderableEntity seg;
                    seg.mesh = &scene.resources.trailingCapsuleMesh;
                    seg.material.useVertexColor = false;
                    seg.material.r = 0.65f;
                    seg.material.g = 0.65f;
                    seg.material.b = 0.68f;
                    seg.opacity.alpha = 0.08f;
                    seg.style = Engine::ECS::RenderStyle::Solid;
                    scene.renderables.push_back(seg);
                    trailingIdx.push_back((int)scene.renderables.size() - 1);
                }
            }

            auto lerp = [](const Engine::Math::Vec3 &a, const Engine::Math::Vec3 &b, float t) -> Engine::Math::Vec3
            { return Engine::Math::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t}; };

            auto samplePoseAt = [&](float sBack, Pose &out) -> bool
            {
                if (poseDeque.size() < 2)
                    return false;
                float accum = 0.0f;
                for (size_t i = 1; i < poseDeque.size(); ++i)
                {
                    Engine::Math::Vec3 a = poseDeque[i - 1].p, b = poseDeque[i].p;
                    float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
                    float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (accum + d >= sBack)
                    {
                        float t = (sBack - accum) / (d > 1e-6f ? d : 1.0f);
                        out.p = lerp(a, b, t);
                        // simple orientation carry-over
                        out.yawDeg = poseDeque[i].yawDeg;
                        out.pitchDeg = poseDeque[i].pitchDeg;
                        return true;
                    }
                    accum += d;
                }
                out = poseDeque.back();
                return true;
            };

            for (int i = 0; i < (int)trailingIdx.size(); ++i)
            {
                // Place centers separated by head length and trail length, plus a small clearance
                float sBack = 0.0f;
                if (i == 0)
                {
                    sBack = (ui_length_m * 0.5f) + (trail_length_m * 0.5f) + spacing_clearance_m;
                }
                else
                {
                    sBack = (ui_length_m * 0.5f) + (trail_length_m * 0.5f) + spacing_clearance_m + i * (trail_length_m + spacing_clearance_m);
                }
                Pose p{};
                if (samplePoseAt(sBack, p))
                {
                    auto &seg = scene.renderables[trailingIdx[i]];
                    seg.transform.position = p.p;
                    float pr = p.pitchDeg * 3.1415926535f / 180.0f;
                    float yr = p.yawDeg * 3.1415926535f / 180.0f;
                    seg.transform.rotationEulerRad = {pr, yr, 0.0f};
                    // Color second and second-last orange
                    if (i == 1 || i == (int)trailingIdx.size() - 2)
                    {
                        seg.material.r = 1.0f;
                        seg.material.g = 0.55f;
                        seg.material.b = 0.10f;
                        seg.opacity.alpha = 0.18f;
                    }
                    else
                    {
                        seg.material.r = 0.65f;
                        seg.material.g = 0.65f;
                        seg.material.b = 0.68f;
                        seg.opacity.alpha = 0.08f;
                    }
                }
            }

            // Removed Anchor/Thrust extra visuals
        }

        // Peristalsis auto progression (timed)
        periElapsedSec += dt;
        if (peristalsisAuto)
        {
            switch (peristalsisState)
            {
            case PeriState::IDLE:
                if (spinning && ui_rpm > 0.0f)
                    periHandleEvent(PeriEvent::FRONT_LOCKED);
                break;
            case PeriState::ANCHOR_LOCK:
                if (periElapsedSec >= periT_AnchorLock)
                    periHandleEvent(PeriEvent::FRONT_LOCKED);
                break;
            case PeriState::STROKE:
                if (periElapsedSec >= periT_Stroke)
                    periHandleEvent(PeriEvent::STROKE_DONE);
                break;
            case PeriState::MID_LOCK:
                if (periElapsedSec >= periT_MidLock)
                    periHandleEvent(PeriEvent::MID_LOCKED);
                break;
            case PeriState::RETRACT:
                if (periElapsedSec >= periT_Retract)
                    periHandleEvent(PeriEvent::RETRACT_DONE);
                break;
            case PeriState::JAM_RECOVERY:
            case PeriState::EMERGENCY_STOP:
                break;
            }
        }

        // Apply deferred menu actions now that statics and handlers exist
        if (reqClearPath)
        {
            headPath.clear();
            lastPathSample = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
            reqClearPath = false;
        }
        if (reqPeriReset)
        {
            periHandleEvent(PeriEvent::RESET);
            reqPeriReset = false;
        }
        if (reqPeriJam)
        {
            periHandleEvent(PeriEvent::JAM);
            reqPeriJam = false;
        }
        if (reqPeriEStop)
        {
            periHandleEvent(PeriEvent::EMERGENCY_STOP);
            reqPeriEStop = false;
        }
        if (reqPeriAutoToggle)
        {
            peristalsisAuto = !peristalsisAuto;
            reqPeriAutoToggle = false;
        }

        // Reset Scene via menu
        if (reqResetScene)
        {
            reqResetScene = false;
            yaw = 0.0f;
            pitch = 0.0f;
            cameraDistance = 5.0f;
            yaw_t = 0.0f;
            pitch_t = 0.0f;
            cameraDistance_t = 5.0f;
            camTarget = Engine::Math::Vec3{0.0f, 0.0f, 0.0f};
            ui_radius_m = 0.075f;
            ui_length_m = 0.6f;
            ui_teeth = 16;
            ui_profile_idx = 0;
            ui_rpm = 0.0f;
            cutter_phase = 0.0f;
            toothIndices.clear();
            sj_pitch_deg = 0.0f;
            sj_yaw_deg = 0.0f;
            sj_pitch_cur_deg = 0.0f;
            sj_yaw_cur_deg = 0.0f;
            yawZeroBase_deg = 0.0f;
            pitchZeroBase_deg = 0.0f;
            spinning = false;
            peristalsisState = PeriState::IDLE;
            autopilotActive = false;
            autopilotTarget_m = 0.0f;
            autopilotProgress_m = 0.0f;
            lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
            apPlan.clear();
            apIndex = -1;
            apSegProgress_m = 0.0f;
            apHoldYaw_deg = 0.0f;
            gridZero = Engine::Math::Vec3{0.0f, 0.0f, 0.0f};
            headPath.clear();
            lastPathSample = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
            poseDeque.clear();
            trailingIdx.clear();
            scene = Engine::SceneBuilder::CreateDefaultScene();
            scene.resources.headMesh = Engine::Gfx::CreateCylinder(ui_radius_m, ui_length_m, 64, true);
            if (!scene.renderables.empty())
            {
                auto &headR = scene.renderables[0];
                headR.mesh = &scene.resources.headMesh;
                headR.material.useVertexColor = false;
                headR.material.r = 1.0f;
                headR.material.g = 0.12f;
                headR.material.b = 0.12f;
                headR.style = Engine::ECS::RenderStyle::Solid;
                headR.opacity.alpha = 1.0f;
                headR.transform.position = {0.0f, 0.0f, 0.0f};
                headR.transform.rotationEulerRad = {0.0f, 0.0f, 0.0f};
                headR.transform.scale = {1.0f, 1.0f, 1.0f};
                camTarget = headR.transform.position;
            }
        }

        // No orbital/electron updates in worm visualization

        // Draw all renderables: backface cull and occlude via painter's sort on max depth
        if (vp_w > 4.0f && vp_h > 4.0f && !scene.renderables.empty())
        {
            auto V = Engine::Math::lookAt(scene.cameraTransform.position, camTarget, Engine::Math::Vec3{0, 1, 0});
            auto P = Engine::Render::ComputeProjectionMatrix(scene.camera);

            auto mul4 = [](const Engine::Math::Mat4 &mat, float x, float y, float z)
            {
                const auto &m = mat.m;
                float cx = m[0] * x + m[4] * y + m[8] * z + m[12];
                float cy = m[1] * x + m[5] * y + m[9] * z + m[13];
                float cz = m[2] * x + m[6] * y + m[10] * z + m[14];
                float cw = m[3] * x + m[7] * y + m[11] * z + m[15];
                return ImVec4(cx, cy, cz, cw);
            };

            struct TriScr
            {
                ImVec2 p0, p1, p2;
                float zsort; // use max depth for conservative ordering
                ImU32 color;
                bool wire;
            };

            std::vector<TriScr> tris;
            tris.reserve(8192);

            for (const auto &rend : scene.renderables)
            {
                if (rend.mesh == nullptr)
                    continue;
                if (rend.style == Engine::ECS::RenderStyle::Hidden)
                    continue;
                // Frustum/clip rejection: quick near/far and simple clip-space check
                auto M = Engine::Render::ComputeModelMatrix(rend.transform);
                auto MV = Engine::Math::multiply(V, M);
                auto MVP = Engine::Math::multiply(P, MV);

                // Compute object center in view space using origin (approx)
                ImVec4 centerV = mul4(MV, 0.0f, 0.0f, 0.0f);
                if (centerV.z > -scene.camera.nearPlane || centerV.z < -scene.camera.farPlane)
                    continue; // behind near plane or beyond far plane
                const auto &verts = rend.mesh->vertices;
                const auto &inds = rend.mesh->indices;
                for (size_t i = 0; i + 2 < inds.size(); i += 3)
                {
                    const auto &v0 = verts[inds[i + 0]];
                    const auto &v1 = verts[inds[i + 1]];
                    const auto &v2 = verts[inds[i + 2]];

                    // View-space positions for culling and depth
                    ImVec4 vv0 = mul4(MV, v0.px, v0.py, v0.pz);
                    ImVec4 vv1 = mul4(MV, v1.px, v1.py, v1.pz);
                    ImVec4 vv2 = mul4(MV, v2.px, v2.py, v2.pz);

                    // Backface culling disabled to ensure solid appearance with painter algorithm

                    // Project to clip then NDC then viewport
                    ImVec4 c0 = mul4(MVP, v0.px, v0.py, v0.pz);
                    ImVec4 c1 = mul4(MVP, v1.px, v1.py, v1.pz);
                    ImVec4 c2 = mul4(MVP, v2.px, v2.py, v2.pz);
                    float w0 = (c0.w == 0.0f ? 1.0f : c0.w);
                    float w1 = (c1.w == 0.0f ? 1.0f : c1.w);
                    float w2 = (c2.w == 0.0f ? 1.0f : c2.w);
                    ImVec2 p0 = ImVec2(vp_min.x + ((c0.x / w0) * 0.5f + 0.5f) * vp_w,
                                       vp_min.y + (1.0f - ((c0.y / w0) * 0.5f + 0.5f)) * vp_h);
                    ImVec2 p1 = ImVec2(vp_min.x + ((c1.x / w1) * 0.5f + 0.5f) * vp_w,
                                       vp_min.y + (1.0f - ((c1.y / w1) * 0.5f + 0.5f)) * vp_h);
                    ImVec2 p2 = ImVec2(vp_min.x + ((c2.x / w2) * 0.5f + 0.5f) * vp_w,
                                       vp_min.y + (1.0f - ((c2.y / w2) * 0.5f + 0.5f)) * vp_h);

                    float cr, cg, cb;
                    if (rend.material.useVertexColor)
                    {
                        cr = (v0.cr + v1.cr + v2.cr) / 3.0f;
                        cg = (v0.cg + v1.cg + v2.cg) / 3.0f;
                        cb = (v0.cb + v1.cb + v2.cb) / 3.0f;
                    }
                    else
                    {
                        cr = rend.material.r;
                        cg = rend.material.g;
                        cb = rend.material.b;
                    }
                    auto u8 = [](float c)
                    { int v = (int)(c * 255.0f); if (v<0) v=0; if (v>255) v=255; return (unsigned)v; };
                    int alpha = (int)(rend.opacity.alpha * 255.0f);
                    ImU32 col = IM_COL32(u8(cr), u8(cg), u8(cb), alpha);

                    // Sort by farthest point (most negative z) to draw far triangles first
                    float zmax = std::min(vv0.z, std::min(vv1.z, vv2.z));
                    bool wire = (rend.style == Engine::ECS::RenderStyle::Wireframe);
                    tris.push_back(TriScr{p0, p1, p2, zmax, col, wire});
                }
            }

            // Sort back-to-front using max depth (more negative first)
            std::sort(tris.begin(), tris.end(), [](const TriScr &a, const TriScr &b)
                      { return a.zsort < b.zsort; });

            // Draw with optional MSAA-like supersample resolve (simple 2x jitter blend)
            bool enableSupersample = true;
            if (!enableSupersample)
            {
                for (const auto &t : tris)
                {
                    if (t.wire)
                        dl->AddTriangle(t.p0, t.p1, t.p2, t.color, 0.8f);
                    else
                        dl->AddTriangleFilled(t.p0, t.p1, t.p2, t.color);
                }
            }
            else
            {
                ImVec2 jitter[2] = {ImVec2(0.25f, 0.25f), ImVec2(-0.25f, -0.25f)};
                for (int pass = 0; pass < 2; ++pass)
                {
                    ImVec2 j = jitter[pass];
                    for (const auto &t : tris)
                    {
                        ImVec2 p0 = ImVec2(t.p0.x + j.x, t.p0.y + j.y);
                        ImVec2 p1 = ImVec2(t.p1.x + j.x, t.p1.y + j.y);
                        ImVec2 p2 = ImVec2(t.p2.x + j.x, t.p2.y + j.y);
                        ImU32 col = (pass == 0) ? (t.color & 0x00FFFFFF) | (((((t.color >> 24) & 0xFF) / 2) & 0xFF) << 24)
                                                : t.color;
                        if (t.wire)
                            dl->AddTriangle(p0, p1, p2, col, 0.8f);
                        else
                            dl->AddTriangleFilled(p0, p1, p2, col);
                    }
                }
            }
        }

        // Minimal overlay (telemetry)
        {
            ImVec2 panel_min = ImVec2(vp_min.x + 8.0f, vp_min.y + 8.0f);
            float line_h = ImGui::GetFontSize() + 2.0f;
            int num_lines = 4;
            ImVec2 panel_max = ImVec2(panel_min.x + 360.0f, panel_min.y + line_h * num_lines + 8.0f);
            dl->AddRectFilled(panel_min, panel_max, IM_COL32(0, 0, 0, 110), 4.0f);
            dl->AddRect(panel_min, panel_max, IM_COL32(255, 255, 255, 24), 4.0f, 0, 1.0f);
            ImVec2 tp = ImVec2(panel_min.x + 8.0f, panel_min.y + 6.0f);
            char buf[128];
            dl->AddText(tp, IM_COL32(255, 255, 255, 230), "Telemetry");
            tp.y += line_h;
            snprintf(buf, sizeof(buf), "Speed: %.2f m/s  Mu: %.2f", inputs.speed_mps, inputs.mu);
            dl->AddText(tp, IM_COL32(225, 225, 235, 220), buf);
            tp.y += line_h;
            snprintf(buf, sizeof(buf), "Torque/whl: %.1f Nm  StopDist: %.2f m", outputs.wheelTorque_Nm, outputs.stoppingDistance_m);
            dl->AddText(tp, IM_COL32(225, 225, 235, 220), buf);
            tp.y += line_h;
            snprintf(buf, sizeof(buf), "Runtime est: %.2f h", outputs.runtime_hours);
            dl->AddText(tp, IM_COL32(200, 255, 200, 230), buf);
        }

        ImGui::End();

        // Graphs Viewport
        ImGui::Begin("Graphs");
        ImDrawList *dlGrid = ImGui::GetWindowDrawList();
        ImVec2 win_pos2 = ImGui::GetWindowPos();
        ImVec2 content_min2 = ImGui::GetWindowContentRegionMin();
        ImVec2 content_max2 = ImGui::GetWindowContentRegionMax();
        ImVec2 gv_min = ImVec2(win_pos2.x + content_min2.x, win_pos2.y + content_min2.y);
        ImVec2 gv_max = ImVec2(win_pos2.x + content_max2.x, win_pos2.y + content_max2.y);
        float gv_w = gv_max.x - gv_min.x;
        float gv_h = gv_max.y - gv_min.y;
        // Enable anti-aliasing for grid
        dlGrid->Flags |= ImDrawListFlags_AntiAliasedLines;
        dlGrid->Flags |= ImDrawListFlags_AntiAliasedFill;
        dlGrid->AddRect(gv_min, gv_max, IM_COL32(70, 70, 70, 255));

        if (gv_w > 4.0f && gv_h > 4.0f)
        {
            auto V2 = Engine::Math::lookAt(scene.cameraTransform.position, camTarget, Engine::Math::Vec3{0, 1, 0});
            Engine::ECS::Camera cam2 = scene.camera;
            cam2.aspect = gv_h > 0.0f ? (gv_w / gv_h) : cam2.aspect;
            cam2.nearPlane = std::max(scene.camera.nearPlane, gridSpacingM * 0.25f);
            auto P2 = Engine::Render::ComputeProjectionMatrix(cam2);
            auto mul4_local = [](const Engine::Math::Mat4 &mat, float x, float y, float z)
            {
                const auto &m = mat.m;
                float cx = m[0] * x + m[4] * y + m[8] * z + m[12];
                float cy = m[1] * x + m[5] * y + m[9] * z + m[13];
                float cz = m[2] * x + m[6] * y + m[10] * z + m[14];
                float cw = m[3] * x + m[7] * y + m[11] * z + m[15];
                return ImVec4(cx, cy, cz, cw);
            };
            auto projectSafe = [&](const Engine::Math::Vec3 &p, ImVec2 &out) -> bool
            {
                ImVec4 v = mul4_local(V2, p.x, p.y, p.z);
                if (v.z > -cam2.nearPlane)
                    return false;
                ImVec4 c = mul4_local(P2, v.x, v.y, v.z);
                float w = c.w;
                if (w <= 0.0f)
                    return false;
                float ndcX = c.x / w;
                float ndcY = c.y / w;
                if (ndcX < -1.2f || ndcX > 1.2f || ndcY < -1.2f || ndcY > 1.2f)
                    return false;
                out.x = gv_min.x + ((ndcX) * 0.5f + 0.5f) * gv_w;
                out.y = gv_min.y + (1.0f - ((ndcY) * 0.5f + 0.5f)) * gv_h;
                return true;
            };

            // Center grid around origin
            Engine::Math::Vec3 origin{-(gridNodesX - 1) * 0.5f * gridSpacingM,
                                      -(gridNodesY - 1) * 0.5f * gridSpacingM,
                                      -(gridNodesZ - 1) * 0.5f * gridSpacingM};

            // Apply caps and fog/LOD
            int maxNodesPerAxis = 32;
            if (gridNodesX < 2)
                gridNodesX = 2;
            else if (gridNodesX > maxNodesPerAxis)
                gridNodesX = maxNodesPerAxis;
            if (gridNodesY < 2)
                gridNodesY = 2;
            else if (gridNodesY > maxNodesPerAxis)
                gridNodesY = maxNodesPerAxis;
            if (gridNodesZ < 2)
                gridNodesZ = 2;
            else if (gridNodesZ > maxNodesPerAxis)
                gridNodesZ = maxNodesPerAxis;
            if (gridSpacingM < 0.1f)
                gridSpacingM = 0.1f;
            else if (gridSpacingM > 5.0f)
                gridSpacingM = 5.0f;

            auto fogFactor = [&](const Engine::Math::Vec3 &p) -> float
            {
                Engine::Math::Vec3 cam = scene.cameraTransform.position;
                float dx = p.x - cam.x, dy = p.y - cam.y, dz = p.z - cam.z;
                float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                float start = 6.0f * gridSpacingM;
                float end = 20.0f * gridSpacingM;
                if (d <= start)
                    return 1.0f;
                if (d >= end)
                    return 0.0f;
                return (end - d) / (end - start);
            };

            if (false)
                for (int ix = 0; ix < gridNodesX; ++ix)
                {
                    for (int iy = 0; iy < gridNodesY; ++iy)
                    {
                        for (int iz = 0; iz < gridNodesZ; ++iz)
                        {
                            Engine::Math::Vec3 p = {origin.x + ix * gridSpacingM,
                                                    origin.y + iy * gridSpacingM,
                                                    origin.z + iz * gridSpacingM};
                            float f = fogFactor(p);
                            if (f <= 0.02f)
                                continue;
                            int skip = (f > 0.66f) ? 1 : (f > 0.33f ? 2 : 3);

                            // Lines to +X, +Y, +Z neighbors
                            if (ix + 1 < gridNodesX)
                            {
                                Engine::Math::Vec3 q = {origin.x + (ix + 1) * gridSpacingM, p.y, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iy + 1 < gridNodesY)
                            {
                                Engine::Math::Vec3 q = {p.x, origin.y + (iy + 1) * gridSpacingM, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iz + 1 < gridNodesZ)
                            {
                                Engine::Math::Vec3 q = {p.x, p.y, origin.z + (iz + 1) * gridSpacingM};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            // Node
                            {
                                ImVec2 sp;
                                if (projectSafe(p, sp))
                                    dlGrid->AddCircleFilled(sp, (f > 0.6f) ? 2.2f : 1.6f, ImGui::GetColorU32(ImVec4(1.0f, 0.9f, 0.4f, 0.2f + 0.8f * f)));
                            }
                        }
                    }
                }

            // Head-centered local grid: always show a neighborhood around the cutter head
            if (!scene.renderables.empty())
            {
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                const int lnx = 9, lny = 7, lnz = 9; // modest neighborhood
                Engine::Math::Vec3 originH{
                    hp.x - (lnx - 1) * 0.5f * gridSpacingM,
                    hp.y - (lny - 1) * 0.5f * gridSpacingM,
                    hp.z - (lnz - 1) * 0.5f * gridSpacingM};

                for (int ix = 0; ix < lnx; ++ix)
                {
                    for (int iy = 0; iy < lny; ++iy)
                    {
                        for (int iz = 0; iz < lnz; ++iz)
                        {
                            Engine::Math::Vec3 p = {originH.x + ix * gridSpacingM,
                                                    originH.y + iy * gridSpacingM,
                                                    originH.z + iz * gridSpacingM};
                            float f = fogFactor(p);
                            if (f <= 0.02f)
                                continue;
                            int skip = (f > 0.66f) ? 1 : (f > 0.33f ? 2 : 3);

                            if (ix + 1 < lnx)
                            {
                                Engine::Math::Vec3 q = {originH.x + (ix + 1) * gridSpacingM, p.y, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iy + 1 < lny)
                            {
                                Engine::Math::Vec3 q = {p.x, originH.y + (iy + 1) * gridSpacingM, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iz + 1 < lnz)
                            {
                                Engine::Math::Vec3 q = {p.x, p.y, originH.z + (iz + 1) * gridSpacingM};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            // Node points
                            {
                                ImVec2 sp;
                                if (projectSafe(p, sp))
                                    dlGrid->AddCircleFilled(sp, (f > 0.6f) ? 2.2f : 1.6f, ImGui::GetColorU32(ImVec4(1.0f, 0.9f, 0.4f, 0.2f + 0.8f * f)));
                            }
                        }
                    }
                }
            }

            // Path-bounded grid: draw a grid covering the bounding box of the path to illustrate history
            if (headPath.size() >= 2)
            {
                Engine::Math::Vec3 pmin = headPath[0], pmax = headPath[0];
                for (const auto &p : headPath)
                {
                    if (p.x < pmin.x)
                        pmin.x = p.x;
                    if (p.x > pmax.x)
                        pmax.x = p.x;
                    if (p.y < pmin.y)
                        pmin.y = p.y;
                    if (p.y > pmax.y)
                        pmax.y = p.y;
                    if (p.z < pmin.z)
                        pmin.z = p.z;
                    if (p.z > pmax.z)
                        pmax.z = p.z;
                }
                Engine::Math::Vec3 center{(pmin.x + pmax.x) * 0.5f, (pmin.y + pmax.y) * 0.5f, (pmin.z + pmax.z) * 0.5f};
                Engine::Math::Vec3 size{(pmax.x - pmin.x), (pmax.y - pmin.y), (pmax.z - pmin.z)};
                // Determine node counts with caps
                auto nodesFor = [&](float extent) -> int
                { int n = (int)(extent / gridSpacingM) + 5; if (n < 3) n = 3; if (n > 48) n = 48; return n; };
                int pnx = nodesFor(size.x);
                int pny = nodesFor(size.y);
                int pnz = nodesFor(size.z);
                Engine::Math::Vec3 originP{center.x - (pnx - 1) * 0.5f * gridSpacingM,
                                           center.y - (pny - 1) * 0.5f * gridSpacingM,
                                           center.z - (pnz - 1) * 0.5f * gridSpacingM};

                for (int ix = 0; ix < pnx; ++ix)
                {
                    for (int iy = 0; iy < pny; ++iy)
                    {
                        for (int iz = 0; iz < pnz; ++iz)
                        {
                            Engine::Math::Vec3 p = {originP.x + ix * gridSpacingM,
                                                    originP.y + iy * gridSpacingM,
                                                    originP.z + iz * gridSpacingM};
                            float f = fogFactor(p);
                            if (f <= 0.02f)
                                continue;
                            int skip = (f > 0.66f) ? 1 : (f > 0.33f ? 2 : 3);

                            if (ix + 1 < pnx)
                            {
                                Engine::Math::Vec3 q = {originP.x + (ix + 1) * gridSpacingM, p.y, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iy + 1 < pny)
                            {
                                Engine::Math::Vec3 q = {p.x, originP.y + (iy + 1) * gridSpacingM, p.z};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            if (iz + 1 < pnz)
                            {
                                Engine::Math::Vec3 q = {p.x, p.y, originP.z + (iz + 1) * gridSpacingM};
                                if ((ix % skip) == 0 && (iy % skip) == 0 && (iz % skip) == 0)
                                {
                                    ImVec2 sp, sq;
                                    if (projectSafe(p, sp) && projectSafe(q, sq))
                                        dlGrid->AddLine(sp, sq, ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.8f, 0.02f + 0.10f * f)), f > 0.5f ? 1.5f : 1.0f);
                                }
                            }
                            ImVec2 sp;
                            if (projectSafe(p, sp))
                                dlGrid->AddCircleFilled(sp, (f > 0.6f) ? 2.2f : 1.6f, ImGui::GetColorU32(ImVec4(1.0f, 0.9f, 0.4f, 0.2f + 0.8f * f)));
                        }
                    }
                }
            }

            // Remove depth/lateral overlay; will plot charts below

            // Remove cutter head outline; graphs shown below

            // Remove head path trace; plots used instead

            // Remove obstacle rendering for dashboard graphs
        }
        ImGui::End();

        // Controls window (dashboard)
        ImGui::Begin("Controls");
        ImGui::TextUnformatted("Inputs");
        ImGui::SliderFloat("Payload (kg)", &inputs.payloadKg, 0.0f, 210.0f, "%.0f");
        ImGui::SliderFloat("Speed (m/s)", &inputs.speed_mps, 0.0f, 1.5f, "%.2f");
        ImGui::SliderFloat("Slope (deg)", &inputs.slope_deg, -10.0f, 10.0f, "%.1f");
        ImGui::SliderFloat("Friction mu", &inputs.mu, 0.2f, 0.8f, "%.2f");
        ImGui::Separator();
        ImGui::TextUnformatted("Battery");
        ImGui::SliderFloat("Capacity (Ah)", &inputs.battCapacity_Ah, 1.0f, 200.0f, "%.0f");
        ImGui::SliderFloat("Voltage (V)", &inputs.battVoltage_V, 12.0f, 52.0f, "%.0f");
        ImGui::SliderFloat("Discharge C", &inputs.dischargeC, 0.2f, 3.0f, "%.1f");
        ImGui::Separator();
        ImGui::TextUnformatted("Arm (placeholder)");
        ImGui::SliderFloat("Arm Payload (kg)", &inputs.armPayloadKg, 0.0f, 8.0f, "%.1f");
        ImGui::Separator();
        if (ImGui::Button("Export CSV"))
        {
            // TODO: export recent series and current inputs/outputs to CSV
        }
        ImGui::End();

        // Micro-Borer parameters removed

        // End main dockspace window
        ImGui::End();
    }
} // namespace Honeycomb