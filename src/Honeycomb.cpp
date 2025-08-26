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

    // Removed periodic table and electron configuration utilities

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
        dl->AddRect(vp_min, vp_max, IM_COL32(80, 80, 80, 255));

        // Static scene and camera state across frames
        static auto scene = Engine::SceneBuilder::CreateDefaultScene();
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
        static float sj_pitch_lim_deg = 18.0f, sj_yaw_lim_deg = 18.0f;
        static float yawZeroBase_deg = 0.0f, pitchZeroBase_deg = 0.0f;
        static bool spinning = false;
        static float forward_gain_m_per_rev = 0.01f;               // advance per revolution
        static float minBendRadius_m = 1.5f;                       // curvature limit
        static float yawRateDeg_s = 25.0f, pitchRateDeg_s = 25.0f; // base steering rates
        // Peristalsis FSM (stub): 0=RearLock, 1=Extend, 2=FrontLock, 3=Retract
        static int peristalsisState = 1;
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
        static float segmentSpacing_m = 0.5f;
        static int numTrailingSegments = 4;
        static std::vector<int> trailingIdx; // indices of trailing segment renderables
        static int anchorIdx = -1, thrustIdx = -1;

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
            if (sj_pitch_deg > sj_pitch_lim_deg)
                sj_pitch_deg = sj_pitch_lim_deg;
            if (sj_pitch_deg < -sj_pitch_lim_deg)
                sj_pitch_deg = -sj_pitch_lim_deg;
            if (sj_yaw_deg > sj_yaw_lim_deg)
                sj_yaw_deg = sj_yaw_lim_deg;
            if (sj_yaw_deg < -sj_yaw_lim_deg)
                sj_yaw_deg = -sj_yaw_lim_deg;
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
            float maxR = 0.0f;
            float maxScale = 1.0f;
            {
                const auto &mesh = scene.resources.wormSegmentMesh;
                for (const auto &v : mesh.vertices)
                {
                    float r2 = v.px * v.px + v.py * v.py + v.pz * v.pz;
                    if (r2 > maxR)
                        maxR = r2;
                }
                maxR = (maxR > 0.0f) ? std::sqrt(maxR) : 0.0f;
                const auto &s = scene.renderables[0].transform.scale;
                maxScale = std::max({std::abs(s.x), std::abs(s.y), std::abs(s.z)});
            }
            float boundRadius = maxR * maxScale;
            float margin = std::max(0.02f, scene.camera.nearPlane * 2.0f);

            // Sphere-ray intersection from camTarget toward camera
            Engine::Math::Vec3 oc = {camTarget.x - wormPos.x, camTarget.y - wormPos.y, camTarget.z - wormPos.z};
            float b = Engine::Math::dot(d, oc);
            float c = Engine::Math::dot(oc, oc) - boundRadius * boundRadius;
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
                for (int i = 0; i < checkN; ++i)
                {
                    Engine::Math::Vec3 p = poseDeque[i].p;
                    Engine::Math::Vec3 occ = {camTarget.x - p.x, camTarget.y - p.y, camTarget.z - p.z};
                    float bb = Engine::Math::dot(d, occ);
                    float cc = Engine::Math::dot(occ, occ) - boundRadius * boundRadius;
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

        // Cutter head animation (spin only when spinning flag true and RPM>0)
        if (!scene.renderables.empty())
        {
            float omega_rad_per_s = (spinning && ui_rpm > 0.0f) ? (ui_rpm * 0.10471975512f) : 0.0f;
            cutter_phase += omega_rad_per_s * dt;
            if (cutter_phase > 1000.0f)
                cutter_phase -= 1000.0f;
            // roll only here; full orientation set after steering update below
            scene.renderables[0].transform.rotationEulerRad.z = cutter_phase;
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
            float mps_inst = (spinning && ui_rpm > 0.0f && peristalsisState == 1) ? ((ui_rpm / 60.0f) * forward_gain_m_per_rev) : 0.0f;
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

            // Advance head forward along its current local +Z if spinning with RPM
            if (spinning && ui_rpm > 0.0f && peristalsisState == 1)
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
                    spinning = false;     // stop cutting
                    peristalsisState = 0; // lock
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
                    seg.mesh = &scene.resources.wormSegmentMesh;
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
                float sBack = (i + 1) * segmentSpacing_m;
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

            // Simple Anchor and Thrust visuals (solid, same mesh/material style as trailing)
            if (anchorIdx < 0 || thrustIdx < 0)
            {
                // Anchor
                Engine::Scene::RenderableEntity anc;
                anc.mesh = &scene.resources.wormSegmentMesh;
                anc.material.useVertexColor = false;
                anc.material.r = 0.65f;
                anc.material.g = 0.65f;
                anc.material.b = 0.68f;
                anc.opacity.alpha = 0.08f;
                anc.style = Engine::ECS::RenderStyle::Solid;
                scene.renderables.push_back(anc);
                anchorIdx = (int)scene.renderables.size() - 1;
                // Thrust
                Engine::Scene::RenderableEntity thr;
                thr.mesh = &scene.resources.wormSegmentMesh;
                thr.material.useVertexColor = false;
                thr.material.r = 0.65f;
                thr.material.g = 0.65f;
                thr.material.b = 0.68f;
                thr.opacity.alpha = 0.08f;
                thr.style = Engine::ECS::RenderStyle::Solid;
                scene.renderables.push_back(thr);
                thrustIdx = (int)scene.renderables.size() - 1;
            }
            // Place anchor/thrust roughly following first trailing poses
            Pose pA{}, pT{};
            samplePoseAt(segmentSpacing_m * (numTrailingSegments + 1), pA);
            samplePoseAt(segmentSpacing_m * (numTrailingSegments + 2), pT);
            if (anchorIdx >= 0)
            {
                auto &anc = scene.renderables[anchorIdx];
                anc.transform.position = pA.p;
                anc.transform.rotationEulerRad = {pA.pitchDeg * 3.1415926535f / 180.0f, pA.yawDeg * 3.1415926535f / 180.0f, 0.0f};
            }
            if (thrustIdx >= 0)
            {
                auto &thr = scene.renderables[thrustIdx];
                thr.transform.position = pT.p;
                thr.transform.rotationEulerRad = {pT.pitchDeg * 3.1415926535f / 180.0f, pT.yawDeg * 3.1415926535f / 180.0f, 0.0f};
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

            // Draw
            for (const auto &t : tris)
            {
                if (t.wire)
                    dl->AddTriangle(t.p0, t.p1, t.p2, t.color, 0.8f);
                else
                    dl->AddTriangleFilled(t.p0, t.p1, t.p2, t.color);
            }
        }

        // Controls overlay in main viewport
        {
            ImVec2 panel_min = ImVec2(vp_min.x + 8.0f, vp_min.y + 8.0f);
            float line_h = ImGui::GetFontSize() + 2.0f;
            int upcoming = 0;
            if (apIndex >= 0 && apIndex < (int)apPlan.size())
            {
                upcoming = std::min(6, (int)apPlan.size() - apIndex);
            }
            int lines = 8 + (upcoming > 0 ? (1 + upcoming) : 0);
            ImVec2 panel_max = ImVec2(panel_min.x + 340.0f, panel_min.y + line_h * lines + 8.0f);
            dl->AddRectFilled(panel_min, panel_max, IM_COL32(0, 0, 0, 140), 4.0f);
            dl->AddRect(panel_min, panel_max, IM_COL32(255, 255, 255, 30), 4.0f, 0, 1.0f);
            ImVec2 tp = ImVec2(panel_min.x + 8.0f, panel_min.y + 6.0f);
            auto addLine = [&](const char *txt, ImU32 col)
            {
                dl->AddText(tp, col, txt);
                tp.y += line_h;
            };
            char buf[128];
            addLine("Controls", IM_COL32(255, 255, 255, 230));
            addLine("Space: Toggle Spin", IM_COL32(235, 235, 240, 220));
            addLine("+ / -: RPM +/-", IM_COL32(235, 235, 240, 220));
            addLine("A/D or Left/Right: Yaw", IM_COL32(235, 235, 240, 220));
            addLine("W/S or Up/Down: Pitch", IM_COL32(235, 235, 240, 220));
            addLine("Forward only when spinning", IM_COL32(255, 120, 120, 220));
            float mps = (spinning && ui_rpm > 0.0f) ? ((ui_rpm / 60.0f) * forward_gain_m_per_rev) : 0.0f;
            snprintf(buf, sizeof(buf), "RPM: %.1f  Spinning: %s  Speed: %.3f m/s", ui_rpm, (spinning ? "On" : "Off"), mps);
            addLine(buf, IM_COL32(225, 225, 235, 220));
            snprintf(buf, sizeof(buf), "Yaw: %.1f deg  Pitch: %.1f deg", sj_yaw_deg, sj_pitch_deg);
            addLine(buf, IM_COL32(225, 225, 235, 220));

            // Upcoming autopilot events
            if (upcoming > 0)
            {
                addLine("Plan (next):", IM_COL32(200, 230, 255, 230));
                for (int k = 0; k < upcoming; ++k)
                {
                    const auto &seg = apPlan[apIndex + k];
                    if (seg.kind == SegStraight)
                    {
                        float len = seg.length_m;
                        if (k == 0)
                            len = std::max(0.0f, seg.length_m - apSegProgress_m);
                        snprintf(buf, sizeof(buf), "  %sStraight %.2fm", (k == 0 ? "> " : ""), len);
                        addLine(buf, IM_COL32(180, 255, 180, 230));
                    }
                    else
                    {
                        const char *axis = seg.pitchAxis ? "Pitch" : "Yaw";
                        snprintf(buf, sizeof(buf), "  %sTurn %s %+0.0fdeg", (k == 0 ? "> " : ""), axis, seg.targetAbsDeg);
                        addLine(buf, seg.pitchAxis ? IM_COL32(240, 180, 255, 230) : IM_COL32(180, 220, 255, 230));
                    }
                }
            }
        }

        ImGui::End();

        // Mesh Grid Viewport
        ImGui::Begin("Mesh Grid Viewport");
        ImDrawList *dlGrid = ImGui::GetWindowDrawList();
        ImVec2 win_pos2 = ImGui::GetWindowPos();
        ImVec2 content_min2 = ImGui::GetWindowContentRegionMin();
        ImVec2 content_max2 = ImGui::GetWindowContentRegionMax();
        ImVec2 gv_min = ImVec2(win_pos2.x + content_min2.x, win_pos2.y + content_min2.y);
        ImVec2 gv_max = ImVec2(win_pos2.x + content_max2.x, win_pos2.y + content_max2.y);
        float gv_w = gv_max.x - gv_min.x;
        float gv_h = gv_max.y - gv_min.y;
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

            // Overlay (top-left): live signed depth and lateral distances relative to zero origin
            if (!scene.renderables.empty())
            {
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                Engine::Math::Vec3 rel{hp.x - gridZero.x, hp.y - gridZero.y, hp.z - gridZero.z};
                float depth_m = -rel.y; // positive down, negative above origin
                float lateral_x_m = rel.x;
                float lateral_z_m = rel.z;
                float lateral_r_m = std::sqrt(lateral_x_m * lateral_x_m + lateral_z_m * lateral_z_m);

                ImVec2 tpos = ImVec2(gv_min.x + 8.0f, gv_min.y + 8.0f);
                float lh = ImGui::GetFontSize() + 4.0f;
                char buf[256];
                snprintf(buf, sizeof(buf), "Depth: %.3f m", depth_m);
                dlGrid->AddText(ImGui::GetFont(), ImGui::GetFontSize(), tpos, IM_COL32(255, 255, 255, 230), buf);
                tpos.y += lh;
                snprintf(buf, sizeof(buf), "Lateral X: %.3f m  Z: %.3f m  |R|: %.3f m", lateral_x_m, lateral_z_m, lateral_r_m);
                dlGrid->AddText(ImGui::GetFont(), ImGui::GetFontSize(), tpos, IM_COL32(230, 230, 240, 220), buf);
            }

            // Draw cutter head as a true-size capsule outline (radius & length) aligned to head axis
            if (!scene.renderables.empty())
            {
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                const float r = ui_radius_m;
                const float L = ui_length_m;
                // Head forward axis from current pitch/yaw
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                // Build orthonormal basis (u,v) perpendicular to fwd
                Engine::Math::Vec3 ref{0.0f, 1.0f, 0.0f};
                // If nearly parallel to up, use X axis
                float dot_up = fwd.x * ref.x + fwd.y * ref.y + fwd.z * ref.z;
                if (std::fabs(dot_up) > 0.95f)
                    ref = Engine::Math::Vec3{1.0f, 0.0f, 0.0f};
                // u = normalize(cross(ref, fwd)); v = cross(fwd, u)
                auto cross = [](const Engine::Math::Vec3 &a, const Engine::Math::Vec3 &b)
                { return Engine::Math::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; };
                auto norm = [&](const Engine::Math::Vec3 &v)
                { float l=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); return (l>1e-6f)? Engine::Math::Vec3{v.x/l,v.y/l,v.z/l} : Engine::Math::Vec3{1,0,0}; };
                Engine::Math::Vec3 u = norm(cross(ref, fwd));
                Engine::Math::Vec3 v = cross(fwd, u);

                Engine::Math::Vec3 cFront{hp.x + fwd.x * (0.5f * L), hp.y + fwd.y * (0.5f * L), hp.z + fwd.z * (0.5f * L)};
                Engine::Math::Vec3 cBack{hp.x - fwd.x * (0.5f * L), hp.y - fwd.y * (0.5f * L), hp.z - fwd.z * (0.5f * L)};
                const int segs = 32;

                ImVec2 front[33];
                ImVec2 back[33];
                int nf = 0, nb = 0;
                for (int i = 0; i <= segs; ++i)
                {
                    float a = (2.0f * 3.1415926535f) * (float)i / (float)segs;
                    float ca = std::cos(a), sa = std::sin(a);
                    Engine::Math::Vec3 ringOffF{u.x * (r * ca) + v.x * (r * sa), u.y * (r * ca) + v.y * (r * sa), u.z * (r * ca) + v.z * (r * sa)};
                    Engine::Math::Vec3 pf{cFront.x + ringOffF.x, cFront.y + ringOffF.y, cFront.z + ringOffF.z};
                    Engine::Math::Vec3 pb{cBack.x + ringOffF.x, cBack.y + ringOffF.y, cBack.z + ringOffF.z};
                    ImVec2 sp;
                    if (projectSafe(pf, sp) && nf < 33)
                        front[nf++] = sp;
                    if (projectSafe(pb, sp) && nb < 33)
                        back[nb++] = sp;
                }
                if (nf >= 3)
                    dlGrid->AddPolyline(front, nf, IM_COL32(255, 80, 80, 220), ImDrawFlags_Closed, 1.6f);
                if (nb >= 3)
                    dlGrid->AddPolyline(back, nb, IM_COL32(255, 80, 80, 160), ImDrawFlags_Closed, 1.2f);

                const int picks[4] = {0, segs / 4, segs / 2, 3 * segs / 4};
                for (int k = 0; k < 4; ++k)
                {
                    int i = picks[k];
                    float a = (2.0f * 3.1415926535f) * (float)i / (float)segs;
                    float ca = std::cos(a), sa = std::sin(a);
                    Engine::Math::Vec3 ringOffF{u.x * (r * ca) + v.x * (r * sa), u.y * (r * ca) + v.y * (r * sa), u.z * (r * ca) + v.z * (r * sa)};
                    Engine::Math::Vec3 pf{cFront.x + ringOffF.x, cFront.y + ringOffF.y, cFront.z + ringOffF.z};
                    Engine::Math::Vec3 pb{cBack.x + ringOffF.x, cBack.y + ringOffF.y, cBack.z + ringOffF.z};
                    ImVec2 spf, spb;
                    if (projectSafe(pf, spf) && projectSafe(pb, spb))
                        dlGrid->AddLine(spf, spb, IM_COL32(255, 80, 80, 180), 1.2f);
                }
            }

            // Draw path trace (polyline) of cutter head in grid view
            if (!headPath.empty())
            {
                std::vector<ImVec2> seg;
                seg.reserve(headPath.size());
                auto flushSeg = [&]()
                {
                    if (seg.size() >= 2)
                        dlGrid->AddPolyline(seg.data(), (int)seg.size(), IM_COL32(255, 120, 60, 160), 0, 2.0f);
                    seg.clear();
                };
                for (const auto &wp : headPath)
                {
                    ImVec2 sp;
                    if (projectSafe(wp, sp))
                    {
                        seg.push_back(sp);
                    }
                    else
                    {
                        flushSeg();
                    }
                }
                flushSeg();
            }

            // Render stationary obstacle only if within sensor coverage and fog
            if (obstacleVisible)
            {
                bool inSensor = false;
                if (!scene.renderables.empty())
                {
                    const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                    const int lnx = 9, lny = 7, lnz = 9; // match head-centered grid neighborhood
                    float hx = (lnx - 1) * 0.5f * gridSpacingM;
                    float hy = (lny - 1) * 0.5f * gridSpacingM;
                    float hz = (lnz - 1) * 0.5f * gridSpacingM;
                    if (obstacleCenter.x >= hp.x - hx && obstacleCenter.x <= hp.x + hx &&
                        obstacleCenter.y >= hp.y - hy && obstacleCenter.y <= hp.y + hy &&
                        obstacleCenter.z >= hp.z - hz && obstacleCenter.z <= hp.z + hz)
                        inSensor = true;
                }
                float f = fogFactor(obstacleCenter);
                if (inSensor && f > 0.02f)
                {
                    // Dotted blue obstacle: sample points on the sphere and draw per-point with fog alpha
                    const int rings = 16;
                    const int slices = 32;
                    for (int i = 0; i <= rings; ++i)
                    {
                        float vlat = (float)i / (float)rings;        // 0..1
                        float theta = (vlat - 0.5f) * 3.1415926535f; // -pi/2..pi/2
                        float r = obstacleRadius_m * std::cos(theta);
                        float y = obstacleCenter.y + obstacleRadius_m * std::sin(theta);
                        for (int j = 0; j < slices; ++j)
                        {
                            float u = (2.0f * 3.1415926535f) * (float)j / (float)slices;
                            Engine::Math::Vec3 p{obstacleCenter.x + r * std::cos(u), y, obstacleCenter.z + r * std::sin(u)};
                            float fp = fogFactor(p);
                            if (fp <= 0.02f)
                                continue;
                            ImVec2 sp;
                            if (projectSafe(p, sp))
                            {
                                int a = (int)(60 + 195 * fp);
                                if (a < 0)
                                    a = 0;
                                if (a > 255)
                                    a = 255;
                                dlGrid->AddCircleFilled(sp, 1.6f, IM_COL32(60, 140, 255, a));
                            }
                        }
                    }
                }
            }
        }
        ImGui::End();

        // Head-Centered Grid Settings (GPR/SONAR-like neighborhood)
        ImGui::Begin("Head-Centered Grid Settings");
        if (ImGui::Button("Zero Origin To Head"))
        {
            if (!scene.renderables.empty())
                gridZero = scene.renderables[0].transform.position;
            // Reset yaw/pitch baseline so subsequent commands are relative to new zero
            yawZeroBase_deg = sj_yaw_cur_deg;
            pitchZeroBase_deg = sj_pitch_cur_deg;
            sj_yaw_deg = 0.0f;
            sj_pitch_deg = 0.0f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Path"))
        {
            headPath.clear();
            lastPathSample = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
        }
        ImGui::End();

        // (Removed duplicate "Operator" window to avoid ID conflicts)
        // Micro-Borer parameters window: adjust radius and length of the segment
        ImGui::Begin("Micro-Borer Parameters");
        bool changed = false;
        ImGui::TextUnformatted("Cutter Head");
        // Tint sliders red for the cutter head controls
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0.95f, 0.15f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(1.00f, 0.10f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0.40f, 0.10f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.32f, 0.08f, 0.08f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.25f, 0.06f, 0.06f, 1.0f));
        changed |= ImGui::SliderFloat("Radius (m)", &ui_radius_m, 0.03f, 0.20f, "%.3f");
        changed |= ImGui::SliderFloat("Length (m)", &ui_length_m, 0.10f, 2.00f, "%.3f");
        // Cutter head teeth and RPM
        ImGui::SliderInt("Teeth", &ui_teeth, 4, 64);
        ImGui::SliderFloat("RPM", &ui_rpm, 0.0f, 600.0f, "%.1f");
        ImGui::PopStyleColor(5);
        // Tooth profile selection
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.40f, 0.10f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.50f, 0.12f, 0.12f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.60f, 0.16f, 0.16f, 1.0f));
        ImGui::Combo("Tooth Profile", &ui_profile_idx, kProfiles, (int)(sizeof(kProfiles) / sizeof(kProfiles[0])));
        ImGui::PopStyleColor(3);
        // Telemetry
        ImGui::Separator();
        ImGui::TextUnformatted("Telemetry");
        ImGui::Text("RPM: %.1f", ui_rpm);
        if (changed)
        {
            // Rebuild the cylinder mesh with new parameters
            scene.resources.wormSegmentMesh = Engine::Gfx::CreateCylinder(ui_radius_m, ui_length_m, 64, true);
        }
        ImGui::End();

        ImGui::End();

        // Operator controls (extend with obstacle avoidance)
        ImGui::Begin("Operator");
        ImGui::Checkbox("Autopilot", &autopilotActive);
        ImGui::SameLine();
        ImGui::TextDisabled("Bend R>=%.2fm", minBendRadius_m);
        ImGui::TextUnformatted("Simple Forward");
        ImGui::SliderFloat("Target Forward (m)", &autopilotTarget_m, 0.0f, 50.0f, "%.2f");
        if (ImGui::Button("Start Drill"))
        {
            apPlan.clear();
            apIndex = -1;
            apSegProgress_m = 0.0f;
            autopilotActive = true;
            autopilotProgress_m = 0.0f;
            lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
            spinning = true;
            peristalsisState = 1; // Extend
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop"))
        {
            apPlan.clear();
            apIndex = -1;
            apSegProgress_m = 0.0f;
            autopilotActive = false;
            spinning = false;
            peristalsisState = 0;
        }
        ImGui::Text("Progress: %.2f / %.2f m", autopilotProgress_m, autopilotTarget_m);
        ImGui::Separator();
        ImGui::TextUnformatted("Obstacle Avoid (90°)");
        ImGui::Checkbox("Show Obstacle", &obstacleVisible);
        ImGui::SliderFloat("Obstacle X", &obstacleCenter.x, -20.0f, 20.0f, "%.2f");
        ImGui::SliderFloat("Obstacle Y", &obstacleCenter.y, -10.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Obstacle Z", &obstacleCenter.z, -20.0f, 20.0f, "%.2f");
        ImGui::SliderFloat("Obstacle Radius (m)", &obstacleRadius_m, 0.05f, 5.0f, "%.2f");
        ImGui::SliderFloat("Clearance (m)", &obstacleClearance_m, 0.05f, 2.0f, "%.2f");
        ImGui::SliderFloat("Turn Radius (m)", &autopilotTurnRadius_m, 0.5f, 8.0f, "%.2f");
        ImGui::Separator();
        if (ImGui::Button("Reset Scene"))
        {
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
            peristalsisState = 1;
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
            anchorIdx = -1;
            thrustIdx = -1;
            scene = Engine::SceneBuilder::CreateDefaultScene();
        }
        if (ImGui::Button("Plan 90° Left"))
        {
            if (!scene.renderables.empty())
            {
                autopilotTurnRadius_m = std::max(autopilotTurnRadius_m, minBendRadius_m);
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 toObs = obstacleCenter - hp;
                float forwardDist = Engine::Math::dot(toObs, fwd);
                float margin = obstacleRadius_m + obstacleClearance_m + autopilotTurnRadius_m;
                float L1 = std::max(forwardDist - margin, 0.0f);
                float L2 = 2.0f * margin;
                bool append = autopilotActive && !apPlan.empty();
                float baseYaw = sj_yaw_cur_deg;
                float basePitch = sj_pitch_cur_deg;
                if (append)
                {
                    // accumulate final planned orientation from remaining segments
                    baseYaw = apHoldYaw_deg;
                    basePitch = apHoldPitch_deg;
                    for (int ii = apIndex; ii < (int)apPlan.size(); ++ii)
                    {
                        const auto &segp = apPlan[ii];
                        if (segp.kind == SegTurn)
                        {
                            if (segp.pitchAxis)
                                basePitch = segp.targetAbsDeg;
                            else
                                baseYaw = segp.targetAbsDeg;
                        }
                    }
                }
                else
                {
                    apPlan.clear();
                    apIndex = 0;
                    apSegProgress_m = 0.0f;
                    apHoldYaw_deg = baseYaw;
                    apHoldPitch_deg = basePitch;
                    autopilotActive = true;
                    spinning = true;
                    peristalsisState = 1;
                    autopilotProgress_m = 0.0f;
                    lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
                }
                float startYaw = baseYaw;
                if (L1 > 0.02f)
                    apPlan.push_back({SegStraight, L1, false, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, false, startYaw + 90.0f});
                apPlan.push_back({SegStraight, L2, false, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, false, startYaw});
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Plan 90° Right"))
        {
            if (!scene.renderables.empty())
            {
                autopilotTurnRadius_m = std::max(autopilotTurnRadius_m, minBendRadius_m);
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 toObs = obstacleCenter - hp;
                float forwardDist = Engine::Math::dot(toObs, fwd);
                float margin = obstacleRadius_m + obstacleClearance_m + autopilotTurnRadius_m;
                float L1 = std::max(forwardDist - margin, 0.0f);
                float L2 = 2.0f * margin;
                bool append = autopilotActive && !apPlan.empty();
                float baseYaw = sj_yaw_cur_deg;
                float basePitch = sj_pitch_cur_deg;
                if (append)
                {
                    baseYaw = apHoldYaw_deg;
                    basePitch = apHoldPitch_deg;
                    for (int ii = apIndex; ii < (int)apPlan.size(); ++ii)
                    {
                        const auto &segp = apPlan[ii];
                        if (segp.kind == SegTurn)
                        {
                            if (segp.pitchAxis)
                                basePitch = segp.targetAbsDeg;
                            else
                                baseYaw = segp.targetAbsDeg;
                        }
                    }
                }
                else
                {
                    apPlan.clear();
                    apIndex = 0;
                    apSegProgress_m = 0.0f;
                    apHoldYaw_deg = baseYaw;
                    apHoldPitch_deg = basePitch;
                    autopilotActive = true;
                    spinning = true;
                    peristalsisState = 1;
                    autopilotProgress_m = 0.0f;
                    lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
                }
                float startYaw = baseYaw;
                if (L1 > 0.02f)
                    apPlan.push_back({SegStraight, L1, false, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, false, startYaw - 90.0f});
                apPlan.push_back({SegStraight, L2, false, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, false, startYaw});
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Plan 90° Down"))
        {
            if (!scene.renderables.empty())
            {
                autopilotTurnRadius_m = std::max(autopilotTurnRadius_m, minBendRadius_m);
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 toObs = obstacleCenter - hp;
                float forwardDist = Engine::Math::dot(toObs, fwd);
                float margin = obstacleRadius_m + obstacleClearance_m + autopilotTurnRadius_m;
                float L1 = std::max(forwardDist - margin, 0.0f);
                float L2 = 2.0f * margin;
                bool append = autopilotActive && !apPlan.empty();
                float baseYaw = sj_yaw_cur_deg;
                float basePitch = sj_pitch_cur_deg;
                if (append)
                {
                    baseYaw = apHoldYaw_deg;
                    basePitch = apHoldPitch_deg;
                    for (int ii = apIndex; ii < (int)apPlan.size(); ++ii)
                    {
                        const auto &segp = apPlan[ii];
                        if (segp.kind == SegTurn)
                        {
                            if (segp.pitchAxis)
                                basePitch = segp.targetAbsDeg;
                            else
                                baseYaw = segp.targetAbsDeg;
                        }
                    }
                }
                else
                {
                    apPlan.clear();
                    apIndex = 0;
                    apSegProgress_m = 0.0f;
                    apHoldYaw_deg = baseYaw;
                    apHoldPitch_deg = basePitch;
                    autopilotActive = true;
                    spinning = true;
                    peristalsisState = 1;
                    autopilotProgress_m = 0.0f;
                    lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
                }
                float startPitch = basePitch;
                if (L1 > 0.02f)
                    apPlan.push_back({SegStraight, L1, true, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, true, startPitch + 90.0f});
                apPlan.push_back({SegStraight, L2, true, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, true, startPitch});
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Plan 90° Up"))
        {
            if (!scene.renderables.empty())
            {
                autopilotTurnRadius_m = std::max(autopilotTurnRadius_m, minBendRadius_m);
                const Engine::Math::Vec3 hp = scene.renderables[0].transform.position;
                float pr = sj_pitch_cur_deg * 3.1415926535f / 180.0f;
                float yr = sj_yaw_cur_deg * 3.1415926535f / 180.0f;
                float cp = std::cos(pr), sp = std::sin(pr);
                float cyh = std::cos(yr), syh = std::sin(yr);
                Engine::Math::Vec3 fwd{syh * cp, -sp, cyh * cp};
                Engine::Math::Vec3 toObs = obstacleCenter - hp;
                float forwardDist = Engine::Math::dot(toObs, fwd);
                float margin = obstacleRadius_m + obstacleClearance_m + autopilotTurnRadius_m;
                float L1 = std::max(forwardDist - margin, 0.0f);
                float L2 = 2.0f * margin;
                bool append = autopilotActive && !apPlan.empty();
                float baseYaw = sj_yaw_cur_deg;
                float basePitch = sj_pitch_cur_deg;
                if (append)
                {
                    baseYaw = apHoldYaw_deg;
                    basePitch = apHoldPitch_deg;
                    for (int ii = apIndex; ii < (int)apPlan.size(); ++ii)
                    {
                        const auto &segp = apPlan[ii];
                        if (segp.kind == SegTurn)
                        {
                            if (segp.pitchAxis)
                                basePitch = segp.targetAbsDeg;
                            else
                                baseYaw = segp.targetAbsDeg;
                        }
                    }
                }
                else
                {
                    apPlan.clear();
                    apIndex = 0;
                    apSegProgress_m = 0.0f;
                    apHoldYaw_deg = baseYaw;
                    apHoldPitch_deg = basePitch;
                    autopilotActive = true;
                    spinning = true;
                    peristalsisState = 1;
                    autopilotProgress_m = 0.0f;
                    lastHeadPosForProgress = Engine::Math::Vec3{1e9f, 1e9f, 1e9f};
                }
                float startPitch = basePitch;
                if (L1 > 0.02f)
                    apPlan.push_back({SegStraight, L1, true, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, true, startPitch - 90.0f});
                apPlan.push_back({SegStraight, L2, true, 0.0f});
                apPlan.push_back({SegTurn, 0.0f, true, startPitch});
            }
        }
        ImGui::End();
    }
} // namespace Honeycomb