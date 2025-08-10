// SPDX-License-Identifier: AGPL-3.0-or-later
#import <Foundation/Foundation.h>
#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

#include "imgui.h"
#include "imgui_impl_metal.h"
#include "imgui_impl_osx.h"

#include "honeycomb/chem/csv_loader.hpp"
#include "honeycomb/chem/periodic_table.hpp"
#include "honeycomb/chem/periodic_table_tensor.hpp"
#include "honeycomb/chem/reaction.hpp"
#include "honeycomb/ecs/component/neuron.hpp"

@interface AppViewController : NSViewController <NSWindowDelegate, MTKViewDelegate>
@property (nonatomic, readonly) MTKView *mtkView;
@property (nonatomic, strong) id<MTLDevice> device;
@property (nonatomic, strong) id<MTLCommandQueue> commandQueue;
@end

static const char* to_string(honeycomb::chem::StandardState s)
{
    using honeycomb::chem::StandardState;
    switch (s)
    {
        case StandardState::Solid: return "Solid";
        case StandardState::Liquid: return "Liquid";
        case StandardState::Gas: return "Gas";
        default: return "Unknown";
    }
}

static const char* to_string(honeycomb::chem::Block b)
{
    using honeycomb::chem::Block;
    switch (b)
    {
        case Block::S: return "s";
        case Block::P: return "p";
        case Block::D: return "d";
        case Block::F: return "f";
        default: return "Unknown";
    }
}

@implementation AppViewController
{
    honeycomb::chem::periodic_table _table;
    int _selectedZ; // 1..118
    honeycomb::chem::PeriodicTableTensor _tensor; // fast numeric lookups
    std::vector<std::string> _oxidationTextCache;
    std::vector<std::array<int, 8>> _shellOccCache; // n=1..7, index 1..7 used
}

- (instancetype)initWithNibName:(nullable NSString *)nibNameOrNil bundle:(nullable NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (!self) return self;

    _device = MTLCreateSystemDefaultDevice();
    _commandQueue = [_device newCommandQueue];
    if (!_device) { NSLog(@"Metal not supported"); abort(); }

    // ImGui init
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplMetal_Init(_device);

    // Load data once
    @autoreleasepool {
        std::string csvPath = "assets/data/periodic_table.csv";
        auto elements = honeycomb::chem::load_elements_from_csv(csvPath);
        _table = honeycomb::chem::periodic_table{std::move(elements)};
        _tensor = honeycomb::chem::PeriodicTableTensor{_table};
    }
    _selectedZ = (_table.size() > 0) ? 1 : 0;

    // Precompute caches for fast UI rendering
    _oxidationTextCache.assign(_table.size(), std::string{});
    _shellOccCache.assign(_table.size(), std::array<int, 8>{});

    auto build_oxidation_text = [](const honeycomb::chem::Element &e) -> std::string {
        const auto &os = e.oxidationStates();
        if (os.empty()) return std::string{"(none)"};
        std::string s;
        s.reserve(os.size() * 3);
        for (size_t i = 0; i < os.size(); ++i) { if (i) s += ", "; s += std::to_string(os[i]); }
        return s;
    };

    auto noble_core_occ = [](const std::string &core) -> std::array<int,8> {
        // Precomputed principal shell occupancy for noble gases
        if (core == "He")  return {0,2,0,0,0,0,0,0};
        if (core == "Ne")  return {0,2,8,0,0,0,0,0};
        if (core == "Ar")  return {0,2,8,8,0,0,0,0};
        if (core == "Kr")  return {0,2,8,18,8,0,0,0};
        if (core == "Xe")  return {0,2,8,18,18,8,0,0};
        if (core == "Rn")  return {0,2,8,18,32,18,8,0};
        return {0,0,0,0,0,0,0,0};
    };

    auto parse_shell_occupancy = [&](const std::string &config) -> std::array<int,8> {
        std::array<int,8> occ{}; occ.fill(0);
        size_t i = 0; const size_t n = config.size();
        if (i < n && config[i] == '[') {
            size_t j = config.find(']', i+1);
            if (j != std::string::npos) {
                std::string core = config.substr(i+1, j-i-1);
                occ = noble_core_occ(core);
                i = j + 1;
            }
        }
        while (i < n) {
            while (i < n && std::isspace(static_cast<unsigned char>(config[i]))) ++i;
            if (i >= n) break;
            int principal = 0;
            while (i < n && std::isdigit(static_cast<unsigned char>(config[i]))) {
                principal = principal * 10 + (config[i]-'0');
                ++i;
            }
            if (principal < 1 || principal > 7) { ++i; continue; }
            if (i >= n || !std::isalpha(static_cast<unsigned char>(config[i]))) { ++i; continue; }
            char subshell = config[i++]; (void)subshell; // subshell not used for ring viz
            int count = 0;
            while (i < n && std::isdigit(static_cast<unsigned char>(config[i]))) {
                count = count * 10 + (config[i]-'0');
                ++i;
            }
            occ[static_cast<std::size_t>(principal)] += count;
        }
        return occ;
    };

    for (std::size_t idx = 0; idx < _table.size(); ++idx)
    {
        const auto &e = _table.by_index(idx);
        _oxidationTextCache[idx] = build_oxidation_text(e);
        _shellOccCache[idx] = parse_shell_occupancy(e.electronConfiguration());
    }

    return self;
}

- (MTKView *)mtkView { return (MTKView *)self.view; }

- (void)loadView
{
    self.view = [[MTKView alloc] initWithFrame:CGRectMake(0, 0, 1000, 720) device:nil];
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.mtkView.device = self.device;
    self.mtkView.delegate = self;
    self.mtkView.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
    self.mtkView.clearColor = MTLClearColorMake(0.10, 0.10, 0.12, 1.0);
    ImGui_ImplOSX_Init(self.view);
    [NSApp activateIgnoringOtherApps:YES];
}

- (void)drawInMTKView:(MTKView *)view
{
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize.x = (float)view.bounds.size.width;
    io.DisplaySize.y = (float)view.bounds.size.height;
    CGFloat framebufferScale = [view.window.screen backingScaleFactor];
    if (framebufferScale == 0) framebufferScale = [NSScreen mainScreen].backingScaleFactor;
    io.DisplayFramebufferScale = ImVec2((float)framebufferScale, (float)framebufferScale);

    id<MTLCommandBuffer> commandBuffer = [self.commandQueue commandBuffer];
    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if (renderPassDescriptor == nil) { [commandBuffer commit]; return; }

    ImGui_ImplMetal_NewFrame(renderPassDescriptor);
    ImGui_ImplOSX_NewFrame(view);
    ImGui::NewFrame();

    // Animation time used for orbiting electrons
    static float g_anim_t = 0.0f;
    g_anim_t += io.DeltaTime;

    // UI: Element Browser
    ImGui::SetNextWindowSize(ImVec2(520, 640), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Periodic Table Browser"))
    {
        if (ImGui::Button("Fullscreen"))
        {
            if (self.view.window != nil)
                [self.view.window toggleFullScreen:nil];
        }
        if (_table.size() == 0)
        {
            ImGui::TextColored(ImVec4(1,0.3f,0.3f,1), "Failed to load CSV at assets/data/periodic_table.csv");
        }
        else
        {
            // Controls
            ImGui::Text("Browse elements");
            int minZ = 1;
            int maxZ = (int)_table.size();

            // Search by symbol/name
            static char searchBuf[32] = "";
            ImGui::InputText("Search (symbol or name)", searchBuf, sizeof(searchBuf));
            if (ImGui::IsItemEdited())
            {
                std::string q = searchBuf; for (auto &c : q) c = (char)std::tolower((unsigned char)c);
                if (!q.empty())
                {
                    for (int z = 1; z <= maxZ; ++z)
                    {
                        const auto &el = _table.by_atomic_number(z);
                        std::string sym = el.symbol(); for (auto &c : sym) c = (char)std::tolower((unsigned char)c);
                        std::string nm = el.name(); for (auto &c : nm) c = (char)std::tolower((unsigned char)c);
                        if (sym == q || nm.find(q) != std::string::npos) { _selectedZ = z; break; }
                    }
                }
            }

            // Slider for fast scrubbing
            ImGui::SliderInt("Z", &_selectedZ, minZ, maxZ);
            // Stepper buttons for precise single-step
            ImGui::SameLine();
            if (ImGui::ArrowButton("##prev", ImGuiDir_Left)) { if (_selectedZ > minZ) _selectedZ--; }
            ImGui::SameLine();
            if (ImGui::ArrowButton("##next", ImGuiDir_Right)) { if (_selectedZ < maxZ) _selectedZ++; }
            ImGui::SameLine();
            if (ImGui::Button("-10")) { _selectedZ = std::max(minZ, _selectedZ - 10); }
            ImGui::SameLine();
            if (ImGui::Button("+10")) { _selectedZ = std::min(maxZ, _selectedZ + 10); }

            // Keyboard shortcuts: ←/→ for ±1, Cmd+←/→ for ±10
            if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows))
            {
                ImGuiIO &io2 = ImGui::GetIO();
                bool left = ImGui::IsKeyPressed(ImGuiKey_LeftArrow);
                bool right = ImGui::IsKeyPressed(ImGuiKey_RightArrow);
                bool cmd = io2.KeySuper; // Command on macOS
                if (left)  _selectedZ = std::max(minZ, _selectedZ - (cmd ? 10 : 1));
                if (right) _selectedZ = std::min(maxZ, _selectedZ + (cmd ? 10 : 1));
            }

            // Allow editing numeric input directly
            ImGui::InputInt("Atomic number", &_selectedZ);
            if (_selectedZ < minZ) _selectedZ = minZ;
            if (_selectedZ > maxZ) _selectedZ = maxZ;

            const auto &e = _table.by_atomic_number(_selectedZ);

            // Header
            ImGui::Separator();
            ImGui::Text("%s (%s)", e.name().c_str(), e.symbol().c_str());
            ImGui::Text("Group: %s", e.groupLabel().c_str());

            // Properties grid
            auto label_value = [](const char* label, const std::string &value)
            {
                ImGui::TextUnformatted(label);
                ImGui::NextColumn();
                ImGui::TextUnformatted(value.c_str());
                ImGui::NextColumn();
            };
            auto label_value_f = [](const char* label, double value)
            {
                ImGui::TextUnformatted(label);
                ImGui::NextColumn();
                ImGui::Text("%.6g", value);
                ImGui::NextColumn();
            };
            auto label_value_i = [](const char* label, int value)
            {
                ImGui::TextUnformatted(label);
                ImGui::NextColumn();
                ImGui::Text("%d", value);
                ImGui::NextColumn();
            };

            ImGui::Columns(2, nullptr, false);
            label_value_i("Atomic number", e.atomicNumber());
            label_value("Symbol", e.symbol());
            label_value("Block", to_string(e.block()));
            label_value_i("Period", e.period());
            label_value_i("Group number", e.groupNumber());
            label_value("Standard state", to_string(e.standardState()));
            label_value("CPK color", e.cpkHexColor());
            label_value_f("Atomic mass (u)", e.atomicMassU());
            label_value("Electron configuration", e.electronConfiguration());

            // Oxidation states
            {
                const std::string &ox = _oxidationTextCache[static_cast<std::size_t>(_selectedZ - 1)];
                label_value("Oxidation states", ox);
            }

            label_value_f("Electronegativity (Pauling)", e.electronegativityPauling());
            label_value_i("Atomic radius (vdW pm)", e.vdwRadiusPm());
            label_value_f("Ionization energy (eV)", e.ionizationEnergyEV());
            label_value_f("Electron affinity (eV)", e.electronAffinityEV());
            label_value_f("Melting point (K)", e.meltingPointK());
            label_value_f("Boiling point (K)", e.boilingPointK());
            label_value_f("Density (g/cm^3)", e.densityGCm3());
            label_value("Year discovered", e.yearDiscovered());
            ImGui::Columns(1);

            // Visualizations
            ImGui::Separator();
            ImGui::Text("Oxidation States");
            {
                ImGui::BeginChild("ox_states_row", ImVec2(0, 40), false, ImGuiWindowFlags_NoScrollbar);
                const auto &os = e.oxidationStates();
                for (size_t i = 0; i < os.size(); ++i)
                {
                    int val = os[i];
                    ImVec4 col = val == 0 ? ImVec4(0.5f,0.5f,0.5f,1.0f) : (val > 0 ? ImVec4(0.25f,0.65f,1.0f,1.0f) : ImVec4(1.0f,0.3f,0.3f,1.0f));
                    ImGui::PushStyleColor(ImGuiCol_Button, col);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(col.x*0.9f, col.y*0.9f, col.z*0.9f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(col.x*0.8f, col.y*0.8f, col.z*0.8f, 1.0f));
                    std::string label = (val > 0 ? "+" : "") + std::to_string(val);
                    ImGui::Button(label.c_str());
                    ImGui::PopStyleColor(3);
                    ImGui::SameLine();
                }
                ImGui::EndChild();
            }

            ImGui::Text("Electron Shells");
            {
                ImVec2 canvasSize = ImVec2(420, 420);
                ImVec2 p = ImGui::GetCursorScreenPos();
                ImDrawList *dl = ImGui::GetWindowDrawList();
                ImGui::InvisibleButton("orbital_canvas", canvasSize);
                ImVec2 center = ImVec2(p.x + canvasSize.x * 0.5f, p.y + canvasSize.y * 0.5f);

                const auto &occ = _shellOccCache[static_cast<std::size_t>(_selectedZ - 1)];
                int maxShell = 0; for (int n = 1; n <= 7; ++n) if (occ[static_cast<std::size_t>(n)] > 0) maxShell = n;
                float radiusMax = (std::min(canvasSize.x, canvasSize.y) * 0.5f) - 10.0f;
                float step = (maxShell > 0) ? (radiusMax / (maxShell + 1)) : radiusMax;

                // Nucleus (spherical in 2D)
                float nucleusRadius = std::max(8.0f, 12.0f + 0.05f * (float)e.atomicNumber());
                dl->AddCircleFilled(center, nucleusRadius, IM_COL32(240, 220, 70, 255));

                // Shell rings and orbiting electrons (animated)
                for (int n = 1; n <= maxShell; ++n)
                {
                    float r = step * (n + 0.5f);
                    dl->AddCircle(center, r, IM_COL32(180, 180, 200, 255), 0, 1.0f);
                    int electrons = occ[static_cast<std::size_t>(n)];
                    if (electrons <= 0) continue;
                    const float twoPi = 6.28318530718f;
                    float angleStep = twoPi / (float)electrons;
                    float omega = 0.6f / (float)n; // angular speed ~ 1/n
                    float base = g_anim_t * omega;
                    for (int k = 0; k < electrons; ++k)
                    {
                        float a = base + angleStep * k;
                        ImVec2 pos = ImVec2(center.x + r * std::cos(a), center.y + r * std::sin(a));
                        dl->AddCircleFilled(pos, 3.0f, IM_COL32(255, 255, 255, 255));
                    }
                }
            }

            // Reaction panel
            ImGui::Separator();
            ImGui::Text("Reaction Simulator (heuristic)");
            static int zA = _selectedZ;
            static int zB = std::min(_selectedZ + 1, maxZ);
            static double tempK = 298.15;
            static double concA = 1.0; // mol/L
            static double concB = 1.0; // mol/L
            static double concP = 0.0; // product
            static double t_s = 0.0;   // time in seconds
            static bool simulate = false;
            ImGui::InputInt("Reactant A (Z)", &zA);
            ImGui::InputInt("Reactant B (Z)", &zB);
            if (zA < minZ) zA = minZ; if (zA > maxZ) zA = maxZ;
            if (zB < minZ) zB = minZ; if (zB > maxZ) zB = maxZ;
            ImGui::InputDouble("Temperature (K)", &tempK, 1.0, 10.0, "%.2f");
            ImGui::InputDouble("[A] (mol/L)", &concA, 0.01, 0.1, "%.3f");
            ImGui::InputDouble("[B] (mol/L)", &concB, 0.01, 0.1, "%.3f");
            ImGui::InputDouble("time (s)", &t_s, 0.1, 1.0, "%.2f");

            if (ImGui::Button("Predict Reaction"))
            {
                (void)0; // trigger nothing here; compute immediately below every frame
            }
            {
                honeycomb::chem::ReactionInput rin{zA, zB, tempK};
                honeycomb::chem::ReactionResult rr = honeycomb::chem::predict_reaction(_table, rin);
                ImGui::Text("Feasible: %s", rr.feasible ? "Yes" : "No");
                ImGui::Text("Rationale: %s", rr.rationale.c_str());
                ImGui::Text("Delta G (kJ/mol): %.2f", rr.deltaG_kJ_per_mol);
                ImGui::Text("k_forward (a.u.): %.3e", rr.k_forward);
                ImGui::Text("k_reverse (a.u.): %.3e", rr.k_reverse);
                if (!rr.products.empty())
                {
                    ImGui::Text("Products:");
                    for (const auto &p : rr.products)
                    {
                        ImGui::BulletText("%s  x%d", p.formula.c_str(), p.stoichiometry);
                    }
                }

                // Simple reversible reaction integration: nuA*A + nuB*B <-> P
                if (rr.feasible)
                {
                    if (ImGui::Button(simulate ? "Pause" : "Simulate")) simulate = !simulate;
                    ImGui::SameLine();
                    if (ImGui::Button("Reset")) { concA = 1.0; concB = 1.0; concP = 0.0; t_s = 0.0; }

                    if (simulate)
                    {
                        // Fixed time step Euler integration (small dt)
                        double dt = 0.02; // s
                        for (int i = 0; i < 25; ++i) // integrate ~0.5 s per frame
                        {
                            double rate_f = rr.k_forward * std::pow(concA, rr.nuA) * std::pow(concB, rr.nuB);
                            double rate_r = rr.k_reverse * concP; // simple unimolecular reverse
                            double dP = (rate_f - rate_r) * dt;
                            // Apply stoichiometry
                            concP += dP; if (concP < 0) concP = 0;
                            concA -= rr.nuA * dP; if (concA < 0) concA = 0;
                            concB -= rr.nuB * dP; if (concB < 0) concB = 0;
                            t_s += dt;
                        }
                    }
                    ImGui::Text("[A]=%.3f  [B]=%.3f  [P]=%.3f  t=%.2fs", concA, concB, concP, t_s);

                    // Tiny bar plot
                    float bars[3] = {(float)concA, (float)concB, (float)concP};
                    ImGui::PlotHistogram("Concentrations", bars, 3, 0, nullptr, 0.0f, 2.0f, ImVec2(240, 80));
                }
            }

            // Reactant A and B visuals side-by-side (realtime)
            ImGui::Separator();
            ImGui::Text("Reactants");
            ImGui::Columns(2, nullptr, false);
            auto draw_reactant = [&](const char* label, int z)
            {
                const auto &re = _table.by_atomic_number(z);
                ImGui::Text("%s: %s (%s)", label, re.name().c_str(), re.symbol().c_str());
                ImVec2 size = ImVec2(280, 280);
                ImVec2 p0 = ImGui::GetCursorScreenPos();
                ImDrawList *dl2 = ImGui::GetWindowDrawList();
                ImGui::InvisibleButton(label, size);
                ImVec2 c = ImVec2(p0.x + size.x*0.5f, p0.y + size.y*0.5f);
                const auto &occ2 = _shellOccCache[static_cast<std::size_t>(z - 1)];
                int maxShell2 = 0; for (int n = 1; n <= 7; ++n) if (occ2[static_cast<std::size_t>(n)] > 0) maxShell2 = n;
                float rMax = (std::min(size.x, size.y)*0.5f) - 8.0f;
                float step2 = (maxShell2 > 0) ? (rMax / (maxShell2 + 1)) : rMax;
                float nucleusRadius2 = std::max(6.0f, 10.0f + 0.04f * (float)re.atomicNumber());
                dl2->AddCircleFilled(c, nucleusRadius2, IM_COL32(240, 220, 70, 255));
                for (int n = 1; n <= maxShell2; ++n)
                {
                    float r = step2 * (n + 0.5f);
                    dl2->AddCircle(c, r, IM_COL32(180,180,200,255), 0, 1.0f);
                    int electrons = occ2[static_cast<std::size_t>(n)]; if (electrons <= 0) continue;
                    const float twoPi = 6.28318530718f;
                    float angleStep = twoPi / (float)electrons;
                    float omega = 0.7f / (float)n; float base = g_anim_t * omega;
                    for (int k = 0; k < electrons; ++k)
                    {
                        float a = base + k*angleStep;
                        ImVec2 pos = ImVec2(c.x + r*std::cos(a), c.y + r*std::sin(a));
                        dl2->AddCircleFilled(pos, 2.8f, IM_COL32(255,255,255,255));
                    }
                }
            };
            draw_reactant("A", zA);
            ImGui::NextColumn();
            draw_reactant("B", zB);
            ImGui::Columns(1);

            // Neuron sandbox (very simple Vm integrator)
            ImGui::Separator();
            ImGui::Text("Neuron Sandbox");
            static honeycomb::ecs::component::neuron demoNeuron{};
            static bool neuronSim = false;
            static float targetRest_mV = -70.0f;
            ImGui::SliderFloat("Vm (mV)", &demoNeuron.Vm_mV, -90.0f, 30.0f);
            ImGui::SliderFloat("Input mV/s", &demoNeuron.input_mV_per_s, -50.0f, 50.0f);
            ImGui::SliderFloat("Leak mV/s", &demoNeuron.leak_mV_per_s, 0.0f, 50.0f);
            if (ImGui::Button(neuronSim ? "Pause Vm" : "Sim Vm")) neuronSim = !neuronSim;
            ImGui::SameLine(); if (ImGui::Button("Reset Vm")) demoNeuron.Vm_mV = targetRest_mV;
            if (neuronSim)
            {
                float dt = 0.02f; // s
                for (int i = 0; i < 25; ++i)
                {
                    float leak = (targetRest_mV - demoNeuron.Vm_mV) * (demoNeuron.leak_mV_per_s / 100.0f);
                    demoNeuron.Vm_mV += (leak + demoNeuron.input_mV_per_s) * dt;
                }
            }
            ImGui::Text("Vm = %.2f mV", demoNeuron.Vm_mV);
        }
    }
    ImGui::End();

    // Render
    ImGui::Render();
    ImDrawData *draw_data = ImGui::GetDrawData();
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    [encoder pushDebugGroup:@"ImGui"]; 
    ImGui_ImplMetal_RenderDrawData(draw_data, commandBuffer, encoder);
    [encoder popDebugGroup];
    [encoder endEncoding];

    [commandBuffer presentDrawable:view.currentDrawable];
    [commandBuffer commit];
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size { (void)view; (void)size; }

- (void)viewWillAppear
{
    [super viewWillAppear];
    self.view.window.delegate = self;
}

- (void)windowWillClose:(NSNotification *)notification
{
    (void)notification;
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplOSX_Shutdown();
    ImGui::DestroyContext();
}

@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property (nonatomic, strong) NSWindow *window;
@end

@implementation AppDelegate
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender { (void)sender; return YES; }
- (instancetype)init
{
    if (self = [super init])
    {
        NSViewController *root = [[AppViewController alloc] initWithNibName:nil bundle:nil];
        self.window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0,0,1000,720)
                                                 styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
        self.window.contentViewController = root;
        [self.window center];
        [self.window makeKeyAndOrderFront:self];
    }
    return self;
}
@end

int main(int, const char**)
{
    @autoreleasepool
    {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

        AppDelegate *appDelegate = [[AppDelegate alloc] init];
        [NSApp setDelegate:appDelegate];
        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}


