#include "Honeycomb.hpp"

#include "imgui.h"

#include "engine/scene/SceneBuilder.hpp"
#include "engine/render/RenderPrimitives.hpp"
#include "engine/data/PeriodicTable.hpp"
#include "engine/ecs/ElementComponents.hpp"
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <random>

namespace Honeycomb
{

    // Map noble-gas cores to per-shell electron counts
    static const std::unordered_map<std::string, std::vector<int>> kNobleCoreShells = {
        {"He", {2}},               // 1s2
        {"Ne", {2, 8}},            // 1s2 2s2 2p6
        {"Ar", {2, 8, 8}},         // [Ne]3s2 3p6
        {"Kr", {2, 8, 18, 8}},     // [Ar]3d10 4s2 4p6
        {"Xe", {2, 8, 18, 18, 8}}, // [Kr]4d10 5s2 5p6
        {"Rn", {2, 8, 18, 32, 18, 8}},
        {"Og", {2, 8, 18, 32, 32, 18, 8}}};

    static void AccumulateShell(std::vector<int> &shells, int n, int electrons)
    {
        if (n <= 0 || electrons <= 0)
            return;
        if ((int)shells.size() < n)
            shells.resize(n, 0);
        shells[n - 1] += electrons;
    }

    // Parse electron configuration like "[He]2s2 2p6 3s2" into per-shell occupancy [2, 8, ...]
    static std::vector<int> ParseElectronConfiguration(const std::string &cfg, int atomicNumber)
    {
        std::vector<int> shells;
        size_t i = 0;
        while (i < cfg.size())
        {
            char c = cfg[i];
            if (c == '[')
            {
                size_t j = cfg.find(']', i + 1);
                if (j != std::string::npos)
                {
                    std::string sym = cfg.substr(i + 1, j - i - 1);
                    auto it = kNobleCoreShells.find(sym);
                    if (it != kNobleCoreShells.end())
                    {
                        const auto &core = it->second;
                        if (shells.size() < core.size())
                            shells.resize(core.size(), 0);
                        for (size_t k = 0; k < core.size(); ++k)
                            shells[k] += core[k];
                    }
                    i = j + 1;
                    continue;
                }
            }
            if (std::isdigit((unsigned char)c))
            {
                int n = 0;
                while (i < cfg.size() && std::isdigit((unsigned char)cfg[i]))
                {
                    n = n * 10 + (cfg[i] - '0');
                    ++i;
                }
                // skip orbital letter(s)
                while (i < cfg.size() && std::isalpha((unsigned char)cfg[i]))
                {
                    ++i;
                }
                int e = 0;
                while (i < cfg.size() && std::isdigit((unsigned char)cfg[i]))
                {
                    e = e * 10 + (cfg[i] - '0');
                    ++i;
                }
                if (n > 0 && e > 0)
                    AccumulateShell(shells, n, e);
                continue;
            }
            ++i;
        }
        int total = 0;
        for (int v : shells)
            total += v;
        if (total == 0 && atomicNumber > 0)
        {
            // Fallback 2n^2 capacity until Z
            int remaining = atomicNumber;
            for (int n = 1; n <= 7 && remaining > 0; ++n)
            {
                int cap = 2 * n * n;
                int put = std::min(cap, remaining);
                AccumulateShell(shells, n, put);
                remaining -= put;
            }
        }
        return shells;
    }

    static Engine::Math::Vec3 ShellPlaneNormal(int n)
    {
        switch (n % 6)
        {
        case 1:
            return {0, 1, 0};
        case 2:
            return {1, 0, 0};
        case 3:
            return {0, 0, 1};
        case 4:
            return {1, 1, 0};
        case 5:
            return {1, 0, 1};
        default:
            return {0, 1, 1};
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

        // Update camera transform from yaw/pitch and distance
        scene.camera.aspect = (vp_h > 0.0f) ? (vp_w / vp_h) : scene.camera.aspect;
        float cx = camTarget.x + cameraDistance * std::cos(pitch) * std::sin(yaw);
        float cy = camTarget.y + cameraDistance * std::sin(pitch);
        float cz = camTarget.z + cameraDistance * std::cos(pitch) * std::cos(yaw);
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
        // Minecraft-like inventory grid: fixed columns, sequential by atomic number
        const int cols = 10;
        int count = static_cast<int>(elems.size());
        if (ImGui::BeginTable("ptable_grid", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            for (int i = 0; i < count; ++i)
            {
                if ((i % cols) == 0)
                    ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(i % cols);
                const auto &e = elems[i];
                int Z = e.value("AtomicNumber", 0);
                const std::string sym = e.value("Symbol", "");
                const std::string name = e.value("Name", "");
                ImGui::PushID(Z);
                ImGui::BeginGroup();
                if (ImGui::Button(sym.c_str(), ImVec2(-FLT_MIN, 0)))
                {
                    // Spawn new atom entity in scene based on element data
                    Engine::Scene::RenderableEntity atom;
                    atom.mesh = &scene.resources.atomMesh;
                    atom.style = Engine::ECS::RenderStyle::Solid;
                    atom.opacity.alpha = 1.0f;
                    // Color from CPKHexColor
                    auto hex = e.value("CPKHexColor", std::string("FFFFFF"));
                    if (hex.size() >= 6)
                    {
                        auto hexVal = [&](char c) -> int
                        { if (c>='0'&&c<='9') return c-'0'; if(c>='a'&&c<='f') return 10+(c-'a'); if(c>='A'&&c<='F') return 10+(c-'A'); return 0; };
                        int r = (hexVal(hex[0]) << 4) + hexVal(hex[1]);
                        int g = (hexVal(hex[2]) << 4) + hexVal(hex[3]);
                        int b = (hexVal(hex[4]) << 4) + hexVal(hex[5]);
                        atom.material.useVertexColor = false;
                        atom.material.r = r / 255.0f;
                        atom.material.g = g / 255.0f;
                        atom.material.b = b / 255.0f;
                    }
                    // Nucleus radius from vdW radius when available (pm -> scale), else fallback
                    float nuc = 0.6f;
                    if (e.contains("AtomicRadius(vdWaals)") && e["AtomicRadius(vdWaals)"].is_number())
                    {
                        float pm = e["AtomicRadius(vdWaals)"].get<float>();
                        float v = pm / 200.0f;
                        nuc = (v < 0.4f) ? 0.4f : (v > 2.0f ? 2.0f : v);
                    }
                    atom.transform.scale = {nuc, nuc, nuc};
                    // Position: randomized in a plane, spaced and jittered to avoid grouping identicals
                    static std::mt19937 rng{123456u};
                    std::uniform_real_distribution<float> jitter(-0.8f, 0.8f);
                    std::uniform_real_distribution<float> ringAngle(0.0f, 6.283185307f);
                    std::uniform_int_distribution<int> ringPick(0, 2);
                    int aliveCount = 0;
                    for (auto &sa : spawned)
                        if (sa.alive)
                            ++aliveCount;
                    int ring = ringPick(rng) + (aliveCount / 8); // slowly grow radius over time
                    float radiusPlane = 6.0f + ring * 3.5f;
                    float a = ringAngle(rng);
                    float jx = jitter(rng), jy = jitter(rng);
                    atom.transform.position = {radiusPlane * std::cos(a) + jx, 0.0f, radiusPlane * std::sin(a) + jy};
                    scene.renderables.push_back(atom);
                    int atomIndex = static_cast<int>(scene.renderables.size() - 1);

                    // Build electron shells from configuration
                    std::vector<int> shells = ParseElectronConfiguration(e.value("ElectronConfiguration", std::string()), Z);
                    SpawnedAtom rec{};
                    rec.atomIndex = atomIndex;
                    rec.Z = Z;
                    rec.symbol = sym;
                    rec.alive = true;
                    float baseOrbit = nuc * 1.6f;
                    for (size_t si = 0; si < shells.size(); ++si)
                    {
                        int n = (int)si + 1;
                        int k = shells[si];
                        if (k <= 0)
                            continue;
                        float r_orbit = baseOrbit * (float)(n * n);
                        Engine::Math::Vec3 normal = ShellPlaneNormal(n);
                        for (int ei = 0; ei < k; ++ei)
                        {
                            float phase = (2.0f * 3.1415926535f) * (float)ei / (float)k;
                            Engine::Scene::RenderableEntity eR;
                            eR.transform.scale = {1.0f, 1.0f, 1.0f};
                            eR.mesh = &scene.resources.electronMesh;
                            eR.style = Engine::ECS::RenderStyle::Solid;
                            eR.opacity.alpha = 0.85f;
                            scene.renderables.push_back(eR);
                            int eIndex = static_cast<int>(scene.renderables.size() - 1);
                            scene.orbitals.push_back(Engine::Scene::Orbital{atomIndex, r_orbit, 1.0f + 0.2f * n, phase, normal, eIndex});
                            rec.electronIdx.push_back(eIndex);
                            rec.orbitalIdx.push_back((int)scene.orbitals.size() - 1);
                        }
                    }
                    spawned.push_back(std::move(rec));
                }
                ImGui::TextDisabled("%s", name.c_str());
                // Removal UI: remove most recently spawned of this element
                {
                    bool hasOne = false;
                    for (auto &sa : spawned)
                        if (sa.alive && sa.symbol == sym)
                        {
                            hasOne = true;
                            break;
                        }
                    if (hasOne)
                    {
                        if (ImGui::SmallButton("Remove"))
                        {
                            for (int idx = (int)spawned.size() - 1; idx >= 0; --idx)
                            {
                                auto &sa = spawned[idx];
                                if (!sa.alive || sa.symbol != sym)
                                    continue;
                                if (sa.atomIndex >= 0 && sa.atomIndex < (int)scene.renderables.size())
                                    scene.renderables[sa.atomIndex].style = Engine::ECS::RenderStyle::Hidden;
                                for (int ei : sa.electronIdx)
                                {
                                    if (ei >= 0 && ei < (int)scene.renderables.size())
                                        scene.renderables[ei].style = Engine::ECS::RenderStyle::Hidden;
                                }
                                for (int oi : sa.orbitalIdx)
                                {
                                    if (oi >= 0 && oi < (int)scene.orbitals.size())
                                    {
                                        scene.orbitals[oi].centerRenderableIndex = -1;
                                        scene.orbitals[oi].targetRenderableIndex = -1;
                                    }
                                }
                                sa.alive = false;
                                break;
                            }
                        }
                    }
                }
                ImGui::EndGroup();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::End();

        ImGui::End();
    }
} // namespace Honeycomb