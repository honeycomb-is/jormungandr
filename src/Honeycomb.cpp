#include "Honeycomb.hpp"

#include "imgui.h"

#include "engine/scene/SceneBuilder.hpp"
#include "engine/render/RenderPrimitives.hpp"
#include "engine/data/PeriodicTable.hpp"
#include <algorithm>
#include <vector>

namespace Honeycomb
{

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
        static float yaw = 0.0f;            // smoothed radians around Y
        static float pitch = 0.0f;          // smoothed radians around X
        static float cameraDistance = 5.0f; // smoothed
        static float yaw_t = 0.0f;          // targets
        static float pitch_t = 0.0f;
        static float cameraDistance_t = 5.0f;
        static ImVec2 last_mouse = ImVec2(0, 0);
        static bool dragging = false;

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
            bool shift = ImGui::GetIO().KeyShift;
            bool orbiting = (middleDown || (leftDown && !shift));

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

                if (shift && leftDown)
                {
                    // Translate the atom (and center) in view plane
                    scene.renderables[0].transform.position.x += delta.x * (1.0f / 200.0f);
                    scene.renderables[0].transform.position.y -= delta.y * (1.0f / 200.0f);
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

        // Update camera transform from yaw/pitch and distance
        scene.camera.aspect = (vp_h > 0.0f) ? (vp_w / vp_h) : scene.camera.aspect;
        float cx = cameraDistance * std::cos(pitch) * std::sin(yaw);
        float cy = cameraDistance * std::sin(pitch);
        float cz = cameraDistance * std::cos(pitch) * std::cos(yaw);
        scene.cameraTransform.position = {cx, cy, cz};
        scene.cameraTransform.rotationEulerRad = {0.0f, 0.0f, 0.0f};

        // Update orbitals (electrons)
        if (!scene.orbitals.empty())
        {
            for (auto &orb : scene.orbitals)
            {
                orb.phase += orb.angularSpeed * dt;
                // Build an orthonormal basis for the plane
                Engine::Math::Vec3 n = Engine::Math::normalize(orb.planeNormal);
                Engine::Math::Vec3 ref = (std::abs(n.y) < 0.99f) ? Engine::Math::Vec3{0, 1, 0} : Engine::Math::Vec3{1, 0, 0};
                Engine::Math::Vec3 u = Engine::Math::normalize(Engine::Math::cross(ref, n));
                Engine::Math::Vec3 v = Engine::Math::cross(n, u);
                float c = std::cos(orb.phase);
                float s = std::sin(orb.phase);
                Engine::Math::Vec3 offset = u * (orb.radius * c) + v * (orb.radius * s);
                Engine::Math::Vec3 centerPos = scene.renderables[orb.centerRenderableIndex].transform.position;
                scene.renderables[orb.targetRenderableIndex].transform.position = centerPos + offset;
            }
        }

        // Draw all renderables: backface cull and occlude via painter's sort on max depth
        if (vp_w > 4.0f && vp_h > 4.0f && !scene.renderables.empty())
        {
            auto V = Engine::Math::lookAt(scene.cameraTransform.position, scene.renderables[0].transform.position, Engine::Math::Vec3{0, 1, 0});
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
                auto M = Engine::Render::ComputeModelMatrix(rend.transform);
                auto MV = Engine::Math::multiply(V, M);
                auto MVP = Engine::Math::multiply(P, MV);

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

                    // Backface cull (camera looks -Z): keep normals with negative z
                    float e1x = vv1.x - vv0.x, e1y = vv1.y - vv0.y, e1z = vv1.z - vv0.z;
                    float e2x = vv2.x - vv0.x, e2y = vv2.y - vv0.y, e2z = vv2.z - vv0.z;
                    float nx = e1y * e2z - e1z * e2y;
                    float ny = e1z * e2x - e1x * e2z;
                    float nz = e1x * e2y - e1y * e2x;
                    if (nz >= 0.0f)
                        continue; // back-facing

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

                    float cr = (v0.cr + v1.cr + v2.cr) / 3.0f;
                    float cg = (v0.cg + v1.cg + v2.cg) / 3.0f;
                    float cb = (v0.cb + v1.cb + v2.cb) / 3.0f;
                    auto u8 = [](float c)
                    { int v = (int)(c * 255.0f); if (v<0) v=0; if (v>255) v=255; return (unsigned)v; };
                    int alpha = (int)(rend.opacity.alpha * 255.0f);
                    ImU32 col = IM_COL32(u8(cr), u8(cg), u8(cb), alpha);

                    float zmax = std::max(vv0.z, std::max(vv1.z, vv2.z));
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

        ImGui::End();

        ImGui::Begin("Elements");
        const auto &elems = Engine::Data::ElementsVec();
        if (ImGui::BeginTable("ptable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableSetupColumn("Z");
            ImGui::TableSetupColumn("Symbol");
            ImGui::TableSetupColumn("Name");
            ImGui::TableHeadersRow();
            for (const auto &e : elems)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%d", e.value("AtomicNumber", 0));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(e.value("Symbol", "").c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(e.value("Name", "").c_str());
            }
            ImGui::EndTable();
        }
        ImGui::End();

        ImGui::End();
    }
} // namespace Honeycomb