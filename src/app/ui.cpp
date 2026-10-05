// Toolbar and side panel (raygui immediate-mode controls).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "app.h"
#include "scenes.h"
#include "raygui.h"

namespace {

const Color kPanelBg{30, 33, 41, 255};
const Color kText{220, 224, 232, 255};
const Color kDim{150, 156, 170, 255};
const Color kAccent{120, 180, 255, 255};
const Color kWarn{255, 190, 90, 255};

std::string fmtInt(long long v) {
    std::string s = std::to_string(v < 0 ? -v : v);
    for (int i = (int)s.size() - 3; i > 0; i -= 3) s.insert(i, ",");
    return v < 0 ? "-" + s : s;
}

std::string fmtFloat(const char* fmt, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, fmt, v);
    return buf;
}

const char* toolLabel(Tool t, PhysicsModel m) {
    switch (t) {
        case Tool::Grab: return "1 Grab";
        case Tool::Attract: return "2 Attract";
        case Tool::Repel: return "3 Repel";
        case Tool::Ball:
            return m == PhysicsModel::SPH ? "4 Fluid" : m == PhysicsModel::LennardJones ? "4 Molecule" : m == PhysicsModel::NBody ? "4 Star" : "4 Ball";
        case Tool::Rope: return "5 Rope";
        case Tool::Blob: return "6 Blob";
        case Tool::Box: return "7 Box";
        case Tool::Polygon: return "8 Polygon";
        case Tool::Wall: return "9 Wall";
        case Tool::Erase: return "0 Erase";
        default: return "?";
    }
}

const char* toolTitle(Tool t) {
    static const char* names[] = {"Grab", "Attract", "Repel", "Spawn", "Rope", "Soft body", "Box", "Polygon", "Wall", "Erase"};
    return names[(int)t];
}

}  // namespace

void App::applyStyle() {
    GuiSetFont(font_);
    GuiSetStyle(DEFAULT, TEXT_SIZE, 15);
    GuiSetStyle(DEFAULT, TEXT_SPACING, 0);
    auto set = [](int prop, Color c) { GuiSetStyle(DEFAULT, prop, ColorToInt(c)); };
    set(BORDER_COLOR_NORMAL, {74, 80, 96, 255});
    set(BASE_COLOR_NORMAL, {43, 47, 58, 255});
    set(TEXT_COLOR_NORMAL, {216, 220, 230, 255});
    set(BORDER_COLOR_FOCUSED, {111, 168, 220, 255});
    set(BASE_COLOR_FOCUSED, {53, 64, 85, 255});
    set(TEXT_COLOR_FOCUSED, {255, 255, 255, 255});
    set(BORDER_COLOR_PRESSED, {143, 193, 238, 255});
    set(BASE_COLOR_PRESSED, {58, 110, 170, 255});
    set(TEXT_COLOR_PRESSED, {255, 255, 255, 255});
    set(BORDER_COLOR_DISABLED, {50, 54, 64, 255});
    set(BASE_COLOR_DISABLED, {34, 37, 45, 255});
    set(TEXT_COLOR_DISABLED, {95, 100, 112, 255});
    set(LINE_COLOR, {74, 80, 96, 255});
    set(BACKGROUND_COLOR, kPanelBg);
    GuiSetStyle(SLIDER, BASE_COLOR_PRESSED, ColorToInt({90, 150, 220, 255}));
    GuiSetStyle(DROPDOWNBOX, DROPDOWN_ITEMS_SPACING, 0);
    GuiSetStyle(LISTVIEW, LIST_ITEMS_HEIGHT, 26);
}

// ---------------------------------------------------------------------------
// toolbar
// ---------------------------------------------------------------------------

void App::drawToolbar() {
    hoverHint_.clear();
    DrawRectangle(0, 0, (int)worldRect_.width, (int)kToolbarHeight, Color{30, 33, 41, 255});
    DrawLineEx({0, kToolbarHeight - 1}, {worldRect_.width, kToolbarHeight - 1}, 1, Color{55, 60, 72, 255});
    const PhysicsModel model = world_.params.model;
    const float gap = 6, sep = 22;
    float available = worldRect_.width - 16 - sep - 9 * gap;
    float bw = std::clamp(available / 10.0f, 58.0f, 104.0f);
    float x = 8, y = 8, h = kToolbarHeight - 16;
    Vector2 mouse = GetMousePosition();
    if (dropdownOpen_) GuiLock();
    for (int k = 0; k < (int)Tool::Count; k++) {
        Tool t = (Tool)k;
        if (t == Tool::Ball) {
            DrawLineEx({x + sep / 2 - gap / 2, y + 2}, {x + sep / 2 - gap / 2, y + h - 2}, 1, Color{80, 86, 100, 255});
            x += sep;
        }
        Rectangle r{x, y, bw, h};
        std::string why;
        bool ok = toolAvailable(t, &why);
        bool active = tool_ == t;
        if (!ok) GuiDisable();
        bool before = active;
        GuiToggle(r, toolLabel(t, model), &active);
        GuiEnable();
        if (active && !before) selectTool(t);
        if (CheckCollisionPointRec(mouse, r) && !dropdownOpen_) hoverHint_ = ok ? "" : std::string(toolTitle(t)) + ": " + why;
        x += bw + gap;
    }
    GuiUnlock();
}

// ---------------------------------------------------------------------------
// panel rows
// ---------------------------------------------------------------------------

float App::sliderRow(float y, const char* label, float* value, float lo, float hi, const char* fmt, bool enabled, bool logScale) {
    const float labelW = 128, valueW = 70;
    text(label, panelX_, y + 3, 15, enabled ? kText : kDim);
    Rectangle r{panelX_ + labelW, y + 2, panelW_ - labelW - valueW, 18};
    if (!enabled) GuiDisable();
    if (logScale) {
        float t = std::log(*value / lo) / std::log(hi / lo);
        GuiSliderBar(r, nullptr, nullptr, &t, 0, 1);
        *value = lo * std::pow(hi / lo, std::clamp(t, 0.0f, 1.0f));
    } else {
        GuiSliderBar(r, nullptr, nullptr, value, lo, hi);
    }
    GuiEnable();
    std::string v = fmtFloat(fmt, *value);
    text(v, panelX_ + panelW_ - textWidth(v, 15), y + 3, 15, enabled ? kText : kDim);
    return y + 26;
}

float App::intSliderRow(float y, const char* label, int* value, int lo, int hi, bool enabled) {
    float f = (float)*value;
    float out = sliderRow(y, label, &f, (float)lo, (float)hi, "%.0f", enabled);
    *value = std::clamp((int)std::lround(f), lo, hi);
    return out;
}

float App::checkRow(float y, const char* label, bool* value, bool enabled) {
    if (!enabled) GuiDisable();
    GuiCheckBox({panelX_, y + 3, 16, 16}, label, value);
    GuiEnable();
    return y + 26;
}

float App::noteRow(float y, const std::string& note, Color c) {
    // greedy word wrap measured in pixels
    std::vector<std::string> lines;
    std::string line;
    size_t i = 0;
    while (i < note.size()) {
        size_t sp = note.find(' ', i);
        std::string word = note.substr(i, sp == std::string::npos ? std::string::npos : sp - i);
        std::string candidate = line.empty() ? word : line + " " + word;
        if (!line.empty() && textWidth(candidate, 14) > panelW_) {
            lines.push_back(line);
            line = word;
        } else {
            line = candidate;
        }
        if (sp == std::string::npos) break;
        i = sp + 1;
    }
    if (!line.empty()) lines.push_back(line);
    for (size_t k = 0; k < lines.size(); k++) text(lines[k], panelX_, y + k * 19, 14, c);
    return y + lines.size() * 19 + 4;
}

// ---------------------------------------------------------------------------
// panel
// ---------------------------------------------------------------------------

void App::drawPanel() {
    DrawRectangleRec(panelRect_, kPanelBg);
    DrawLineEx({panelRect_.x, 0}, {panelRect_.x, panelRect_.height}, 1, Color{55, 60, 72, 255});
    panelX_ = panelRect_.x + 16;
    panelW_ = kPanelWidth - 32;
    dropdownOpen_ = sceneDropdownOpen_ || broadphaseDropdownOpen_;
    broadphaseDropdownVisible_ = false;
    if (dropdownOpen_) GuiLock();

    float y = 12;
    y = drawHeader(y);
    y = drawSceneSection(y);
    y = drawToolSection(y);

    int tab = (int)tab_;
    float tw = (panelW_ - 8) / 3;
    GuiSetStyle(TOGGLE, GROUP_PADDING, 4);
    GuiToggleGroup({panelX_, y, tw, 28}, "Physics;Performance;View", &tab);
    tab_ = (Tab)tab;
    y += 38;
    switch (tab_) {
        case Tab::Physics: y = drawPhysicsTab(y); break;
        case Tab::Performance: y = drawPerformanceTab(y); break;
        case Tab::View: y = drawViewTab(y); break;
    }

    // footer: what is in the world
    float fy = panelRect_.height - 28;
    DrawRectangle((int)panelRect_.x + 1, (int)fy - 6, (int)kPanelWidth, 34, Color{25, 28, 35, 255});
    std::string footer = fmtInt(world_.p.size()) + " particles  |  " + fmtInt((long long)world_.bodies.size()) + " shapes  |  " +
                         fmtInt((long long)world_.links.size()) + " links  |  t = " + fmtFloat("%.1f s", world_.time());
    text(footer, panelX_, fy, 14, kDim);

    // dropdowns last so their open lists draw on top of everything
    GuiUnlock();
    if (broadphaseDropdownVisible_) {
        if (sceneDropdownOpen_) GuiLock();
        int active = (int)world_.params.broadphase;
        if (GuiDropdownBox(broadphaseDropdownRect_, "Uniform grid;Spatial hash;Quadtree;Sweep and prune;BVH;Brute force O(n^2)",
                           &active, broadphaseDropdownOpen_))
            broadphaseDropdownOpen_ = !broadphaseDropdownOpen_;
        GuiUnlock();
        if (active != (int)world_.params.broadphase) {
            world_.params.broadphase = (BroadphaseKind)active;
            world_.syncBroadphase();
            overlayStale_ = true;
            stepMsHistory_.clear();
            toast(std::string("Broad phase: ") + broadphaseName(world_.params.broadphase));
        }
    } else {
        broadphaseDropdownOpen_ = false;
    }
    {
        if (broadphaseDropdownOpen_) GuiLock();
        std::string items;
        for (const SceneInfo& s : sceneList()) items += (items.empty() ? "" : ";") + std::string(s.name);
        int active = scene_;
        if (GuiDropdownBox(sceneDropdownRect_, items.c_str(), &active, sceneDropdownOpen_)) sceneDropdownOpen_ = !sceneDropdownOpen_;
        GuiUnlock();
        if (active != scene_) {
            scene_ = active;
            sceneCount_ = 0;
            loadCurrentScene();
            toast(std::string("Scene: ") + sceneList()[scene_].name);
        }
    }
}

float App::drawHeader(float y) {
    text("Particle Simulator", panelX_, y, 22, WHITE, true);
    std::string fps = fmtInt(GetFPS()) + " FPS";
    text(fps, panelX_ + panelW_ - textWidth(fps, 15), y + 5, 15, kDim);
    y += 36;

    // state badge
    const char* label = "RUNNING";
    const char* desc = "time is advancing";
    Color c{46, 158, 91, 255};
    if (state_ == RunState::Paused) {
        label = "PAUSED";
        desc = "time is frozen";
        c = {211, 154, 44, 255};
    } else if (state_ == RunState::Editing) {
        label = "EDITING";
        desc = "time is frozen: editing the scene";
        c = {58, 123, 213, 255};
    }
    DrawRectangleRounded({panelX_, y, 104, 28}, 0.35f, 6, c);
    text(label, panelX_ + 52 - textWidth(label, 16, true) / 2, y + 5, 16, WHITE, true);
    text(desc, panelX_ + 116, y + 6, 15, kText);
    y += 38;

    float bw = (panelW_ - 18) / 4;
    bool running = state_ == RunState::Running;
    if (GuiButton({panelX_, y, bw, 30}, running ? "Pause" : "Run")) setState(running ? RunState::Paused : RunState::Running);
    if (state_ != RunState::Paused) GuiDisable();
    if (GuiButton({panelX_ + (bw + 6), y, bw, 30}, "Step")) stepOnce();
    GuiEnable();
    if (dropdownOpen_) GuiLock();
    if (GuiButton({panelX_ + 2 * (bw + 6), y, bw, 30}, state_ == RunState::Editing ? "Done" : "Edit"))
        setState(state_ == RunState::Editing ? RunState::Paused : RunState::Editing);
    if (GuiButton({panelX_ + 3 * (bw + 6), y, bw, 30}, "Reset")) {
        loadCurrentScene();
        toast("Scene reset");
    }
    y += 34;
    text("Space run/pause   N step   E edit   R reset   H help", panelX_, y, 13, kDim);
    return y + 26;
}

float App::drawSceneSection(float y) {
    DrawLineEx({panelX_, y}, {panelX_ + panelW_, y}, 1, Color{55, 60, 72, 255});
    y += 10;
    text("Scene", panelX_, y, 16, WHITE, true);
    const SceneInfo& info = sceneList()[scene_];
    std::string model = std::string("Model: ") + modelName(info.model);
    text(model, panelX_ + panelW_ - textWidth(model, 14), y + 2, 14, kAccent);
    y += 24;
    sceneDropdownRect_ = {panelX_, y, panelW_, 28};  // drawn at the end of drawPanel
    y += 34;
    std::string desc = info.description;
    size_t nl = desc.find('\n');
    text(desc.substr(0, nl), panelX_, y, 14, kDim);
    if (nl != std::string::npos) text(desc.substr(nl + 1), panelX_, y + 18, 14, kDim);
    y += 40;
    if (info.defaultCount > 0) {
        float count = (float)sceneCount_;
        float before = count;
        y = sliderRow(y, "Particles", &count, (float)info.minCount, (float)info.maxCount, "%.0f", true, true);
        int rounded = (int)std::lround(count);
        if (rounded > 2000) rounded = rounded / 100 * 100;
        if (count != before && rounded != sceneCount_) {
            sceneCount_ = rounded;
            sceneCountDirty_ = true;
        }
        // reload once the slider is released, not on every drag step
        if (sceneCountDirty_ && !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            sceneCountDirty_ = false;
            loadCurrentScene();
            toast("Scene reloaded with " + fmtInt(world_.p.size()) + " particles");
        }
        if (sceneCountDirty_) text("release to reload the scene", panelX_ + 128, y - 6, 12, kWarn);
    } else {
        text("Fixed layout (no particle count)", panelX_, y + 3, 14, kDim);
        y += 26;
    }
    return y + 4;
}

float App::drawToolSection(float y) {
    DrawLineEx({panelX_, y}, {panelX_ + panelW_, y}, 1, Color{55, 60, 72, 255});
    y += 10;
    const PhysicsModel model = world_.params.model;
    text(std::string("Tool: ") + toolTitle(tool_), panelX_, y, 16, WHITE, true);
    y += 26;
    const float sectionEnd = y + 80;  // fixed height so the tabs below don't jump around
    switch (tool_) {
        case Tool::Grab:
            y = noteRow(y, state_ == RunState::Running ? "Drag balls and shapes; release to throw. Pinned points can be dragged too."
                                                      : "While frozen, dragging moves an object to a new place.",
                        kDim);
            break;
        case Tool::Attract:
        case Tool::Repel:
            y = sliderRow(y, "Strength", &forceStrength_, 500, 40000, "%.0f", true, true);
            y = sliderRow(y, "Radius", &forceRadius_, 40, 400, "%.0f px");
            break;
        case Tool::Ball:
            if (model == PhysicsModel::NBody) {
                y = sliderRow(y, "Star mass", &starMass_, 1, 5000, "%.0f", true, true);
                y = noteRow(y, "Drag to give the star an initial velocity.", kDim);
            } else if (model == PhysicsModel::SPH || model == PhysicsModel::LennardJones) {
                y = noteRow(y, model == PhysicsModel::SPH ? "Hold the mouse to pour fluid particles."
                                                   : "Hold the mouse to add molecules at the current temperature.",
                            kDim);
            } else {
                y = sliderRow(y, "Radius", &ballRadius_, 2, 40, "%.0f px");
                checkRow(y, "Random sizes", &ballRandomSize_);
                GuiCheckBox({panelX_ + 170, y + 3, 16, 16}, "Spray", &ballSpray_);
                y += 26;
            }
            break;
        case Tool::Rope:
            y = checkRow(y, "Pin the start point", &ropePinned_);
            break;
        case Tool::Blob:
            y = sliderRow(y, "Radius", &blobRadius_, 25, 90, "%.0f px");
            break;
        case Tool::Box:
        case Tool::Polygon:
            y = sliderRow(y, "Size", &shapeSize_, 12, 140, "%.0f px");
            if (tool_ == Tool::Polygon) y = intSliderRow(y, "Sides", &polygonSides_, 3, 8);
            y = checkRow(y, "Static obstacle (never moves)", &shapeStatic_);
            break;
        case Tool::Wall:
            y = noteRow(y, "Drag to draw a static wall. Erase removes it.", kDim);
            break;
        case Tool::Erase:
            y = sliderRow(y, "Brush radius", &eraseRadius_, 5, 120, "%.0f px");
            break;
        default:
            break;
    }
    y = std::max(y, sectionEnd);
    DrawLineEx({panelX_, y}, {panelX_ + panelW_, y}, 1, Color{55, 60, 72, 255});
    return y + 10;
}

// ---------------------------------------------------------------------------
// tabs
// ---------------------------------------------------------------------------

float App::drawPhysicsTab(float y) {
    Params& P = world_.params;
    const PhysicsModel model = P.model;
    bool changed = false;

    // integrator
    int integ = (int)P.integrator;
    if (model == PhysicsModel::SPH || gpuActive_) GuiDisable();
    GuiToggleGroup({panelX_, y, (panelW_ - 4) / 2, 26}, "Symplectic Euler;Verlet", &integ);
    GuiEnable();
    if (dropdownOpen_) GuiLock();
    if (integ != (int)P.integrator && model != PhysicsModel::SPH && !gpuActive_) {
        P.integrator = (Integrator)integ;
        world_.invalidate();
        changed = true;
    }
    y += 32;
    const char* desc = "";
    if (gpuActive_) desc = "GPU: symplectic Euler with Jacobi contacts.";
    else if (model == PhysicsModel::SPH) desc = "SPH uses its own predict-relax scheme (Clavet 2005).";
    else if (model == PhysicsModel::Rigid)
        desc = P.integrator == Integrator::Verlet ? "Position-based Verlet: very stable stacks and cloth."
                                                  : "Impulses on velocities: exact energy when e = 1.";
    else desc = P.integrator == Integrator::Verlet ? "Velocity Verlet: 2nd order, small energy error."
                                                   : "Symplectic Euler: 1st order, larger energy error (see graph).";
    y = noteRow(y, desc, kDim);

    y = intSliderRow(y, "Substeps", &P.substeps, 1, 32);
    y = sliderRow(y, "Time scale", &P.timeScale, 0.1f, 2.0f, "%.2fx");

    switch (model) {
        case PhysicsModel::Rigid:
            y = sliderRow(y, "Gravity", &P.gravity.y, 0, 2000, "%.0f");
            y = sliderRow(y, "Air drag", &P.drag, 0, 3, "%.2f");
            y = sliderRow(y, "Restitution", &P.restitution, 0, 1, "%.2f");
            y = sliderRow(y, "Friction", &P.friction, 0, 1, "%.2f", !gpuActive_);
            y = intSliderRow(y, "Link iterations", &P.constraintIterations, 1, 20, !gpuActive_ && !world_.links.empty());
            {
                bool tear = P.tearRatio > 1;
                bool before = tear;
                checkRow(y, "Tear links", &tear, !world_.links.empty());
                if (tear != before) P.tearRatio = tear ? 1.6f : 0;
                if (tear) {
                    float ratio = P.tearRatio;
                    Rectangle r{panelX_ + 128, y + 2, panelW_ - 128 - 70, 18};
                    GuiSliderBar(r, nullptr, nullptr, &ratio, 1.2f, 3.0f);
                    P.tearRatio = ratio;
                    std::string v = fmtFloat("%.2fx", ratio);
                    text(v, panelX_ + panelW_ - textWidth(v, 15), y + 3, 15, kText);
                }
                y += 26;
            }
            break;
        case PhysicsModel::SPH:
            y = sliderRow(y, "Gravity", &P.gravity.y, 0, 2000, "%.0f");
            y = sliderRow(y, "Radius h", &P.sphRadius, 14, 40, "%.0f px");
            y = sliderRow(y, "Rest density", &P.sphRestDensity, 1, 12, "%.1f");
            y = sliderRow(y, "Stiffness", &P.sphStiffness, 5000, 200000, "%.0f", true, true);
            y = sliderRow(y, "Near stiffness", &P.sphNearStiffness, 5000, 400000, "%.0f", true, true);
            y = sliderRow(y, "Viscosity", &P.sphViscosityQuadratic, 0, 0.1f, "%.3f");
            y = sliderRow(y, "Wall bounce", &P.restitution, 0, 1, "%.2f");
            break;
        case PhysicsModel::LennardJones:
            y = checkRow(y, "Thermostat (Berendsen)", &P.thermostat);
            y = sliderRow(y, "Temperature T*", &P.targetTemperature, 0.05f, 3.0f, "%.2f", P.thermostat);
            y = sliderRow(y, "Gravity", &P.gravity.y, 0, 600, "%.0f");
            y = sliderRow(y, "Air drag", &P.drag, 0, 3, "%.2f");
            break;
        case PhysicsModel::NBody: {
            y = sliderRow(y, "G", &P.G, 100, 5000, "%.0f", true, true);
            y = sliderRow(y, "Softening", &P.softening, 1, 20, "%.1f px");
            int solver = P.barnesHut ? 0 : 1;
            GuiToggleGroup({panelX_, y, (panelW_ - 4) / 2, 26}, "Barnes-Hut;Direct O(n^2)", &solver);
            if ((solver == 0) != P.barnesHut) {
                P.barnesHut = solver == 0;
                world_.invalidate();
            }
            y += 32;
            y = sliderRow(y, "Opening angle", &P.theta, 0.1f, 1.5f, "%.2f", P.barnesHut);
            y = checkRow(y, "Walls", &P.walls);
            break;
        }
        default:
            break;
    }
    if (changed) gpuDirty_ = true;
    GuiUnlock();
    if (dropdownOpen_) GuiLock();

    float bottom = panelRect_.height - 44;
    if (model == PhysicsModel::LennardJones) {
        float h = std::max(60.0f, (bottom - y - 8) / 2);
        y = drawSpeedHistogram(y + 4, h);
        y = drawEnergyGraph(y + 4, std::max(50.0f, bottom - y - 8));
    } else {
        y = drawEnergyGraph(y + 4, std::max(60.0f, std::min(180.0f, bottom - y - 8)));
    }
    return y;
}

float App::drawEnergyGraph(float y, float height) {
    const float labelH = 20;
    const Diagnostics& d = world_.diagnostics();
    double total = d.kinetic + d.potential;
    std::string head = "Energy: total " + fmtFloat("%.4g", total);
    if (!energyHistory_.empty()) {
        double e0 = energyHistory_.front().kinetic + energyHistory_.front().potential;
        if (std::fabs(e0) > 1e-9) head += "  (" + fmtFloat("%+.2f%%", (total - e0) / std::fabs(e0) * 100) + " in view)";
    }
    text(head, panelX_, y, 14, kText);
    Rectangle r{panelX_, y + labelH, panelW_, height - labelH};
    if (r.height < 20) return y + height;
    DrawRectangleRec(r, Color{22, 25, 31, 255});
    DrawRectangleLinesEx(r, 1, Color{55, 60, 72, 255});
    if (energyHistory_.size() < 2) {
        text("runs while the simulation runs", r.x + 8, r.y + r.height / 2 - 8, 14, kDim);
        return y + height;
    }
    double lo = 1e300, hi = -1e300;
    for (const EnergySample& s : energyHistory_) {
        double vals[3] = {s.kinetic, s.potential, (double)s.kinetic + s.potential};
        for (double v : vals) {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    }
    double span = std::max(hi - lo, std::max(std::fabs(hi), std::fabs(lo)) * 0.02 + 1e-9);
    lo -= span * 0.08;
    hi = lo + span * 1.16;
    auto plot = [&](int which, Color c) {
        size_t n = energyHistory_.size();
        for (size_t i = 1; i < n; i++) {
            auto val = [&](size_t k) {
                const EnergySample& s = energyHistory_[k];
                return which == 0 ? (double)s.kinetic : which == 1 ? (double)s.potential : (double)s.kinetic + s.potential;
            };
            float x0 = r.x + (float)(i - 1) / 599 * r.width, x1 = r.x + (float)i / 599 * r.width;
            float y0 = r.y + r.height - (float)((val(i - 1) - lo) / (hi - lo)) * r.height;
            float y1 = r.y + r.height - (float)((val(i) - lo) / (hi - lo)) * r.height;
            DrawLineV({x0, y0}, {x1, y1}, c);
        }
    };
    plot(0, Color{90, 170, 255, 255});
    plot(1, Color{255, 160, 70, 255});
    plot(2, Color{240, 240, 240, 255});
    float lx = r.x + r.width - 168, ly = r.y + r.height - 16;
    DrawRectangleRec({lx - 4, ly - 2, 170, 16}, Color{22, 25, 31, 220});
    text("kinetic", lx, ly, 12, Color{90, 170, 255, 255});
    text("potential", lx + 50, ly, 12, Color{255, 160, 70, 255});
    text("total", lx + 118, ly, 12, Color{240, 240, 240, 255});
    return y + height;
}

float App::drawSpeedHistogram(float y, float height) {
    const Particles& p = world_.p;
    double T = world_.diagnostics().temperature;
    text("Speeds vs Maxwell-Boltzmann (T* = " + fmtFloat("%.2f", T) + ")", panelX_, y, 14, kText);
    Rectangle r{panelX_, y + 20, panelW_, height - 20};
    DrawRectangleRec(r, Color{22, 25, 31, 255});
    DrawRectangleLinesEx(r, 1, Color{55, 60, 72, 255});
    int n = 0;
    const int bins = 36;
    double kT = std::max(1e-6, T * world_.params.ljEpsilon);  // unit-mass molecules
    float vMax = (float)(3.5 * std::sqrt(kT));
    std::vector<int> hist(bins, 0);
    for (int i = 0; i < p.size(); i++) {
        if (p.invMass[i] == 0) continue;
        float v = std::sqrt(p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i]);
        int b = (int)(v / vMax * bins);
        if (b < bins) hist[b]++;
        n++;
    }
    if (n == 0) return y + height;
    float binW = vMax / bins;
    // 2D Maxwell-Boltzmann speed density for unit mass: f(v) = v/kT exp(-v^2 / 2kT)
    float peak = 0;
    std::vector<float> expected(bins);
    for (int b = 0; b < bins; b++) {
        float v = (b + 0.5f) * binW;
        expected[b] = (float)(n * binW * v / kT * std::exp(-v * v / (2 * kT)));
        peak = std::max({peak, expected[b], (float)hist[b]});
    }
    float bw = r.width / bins;
    for (int b = 0; b < bins; b++) {
        float h = hist[b] / peak * (r.height - 8);
        DrawRectangleRec({r.x + b * bw + 1, r.y + r.height - h, bw - 2, h}, Color{90, 170, 255, 200});
    }
    for (int b = 1; b < bins; b++) {
        Vector2 a{r.x + (b - 0.5f) * bw, r.y + r.height - expected[b - 1] / peak * (r.height - 8)};
        Vector2 c{r.x + (b + 0.5f) * bw, r.y + r.height - expected[b] / peak * (r.height - 8)};
        DrawLineEx(a, c, 2, Color{255, 200, 90, 255});
    }
    text("measured", r.x + r.width - 140, r.y + 4, 12, Color{90, 170, 255, 255});
    text("theory", r.x + r.width - 70, r.y + 4, 12, Color{255, 200, 90, 255});
    return y + height;
}

float App::drawPerformanceTab(float y) {
    const PhysicsModel model = world_.params.model;
    const StepStats& s = world_.stats();
    const int n = world_.p.size();

    text("Broad phase", panelX_, y + 5, 15, model == PhysicsModel::NBody || gpuActive_ ? kDim : kText);
    if (model == PhysicsModel::NBody) {
        y = noteRow(y + 30, "Not used: gravity acts between all pairs. N-body uses Barnes-Hut (see Physics).", kDim);
    } else if (gpuActive_) {
        y = noteRow(y + 30, "The GPU uses its own uniform grid.", kDim);
    } else {
        broadphaseDropdownRect_ = {panelX_ + 128, y, panelW_ - 128, 28};
        broadphaseDropdownVisible_ = true;
        y += 36;
    }

    int threads = world_.threads();
    int before = threads;
    y = intSliderRow(y, "CPU threads", &threads, 1, std::max(2, ThreadPool::hardwareThreads()), !gpuActive_);
    if (threads != before) {
        world_.setThreads(threads);
        stepMsHistory_.clear();
    }

    std::string why;
    bool allowed = gpuAllowed(&why);
    bool gpu = gpuActive_;
    checkRow(y, "GPU compute (OpenGL 4.3)", &gpu, allowed || gpuActive_);
    if (gpu != gpuActive_) setGpuEnabled(gpu);
    y += 26;
    if (!gpuActive_ && !allowed) y = noteRow(y, "Unavailable: " + why, kDim);

    y += 4;
    DrawLineEx({panelX_, y}, {panelX_ + panelW_, y}, 1, Color{55, 60, 72, 255});
    y += 8;
    // ---- live statistics
    if (gpuActive_) {
        text("GPU step: " + fmtFloat("%.2f ms", gpu_.lastStepMs()) + " (incl. read-back)", panelX_, y, 15, kText);
        y += 22;
    } else {
        text("Step: " + fmtFloat("%.2f ms", s.stepMs), panelX_, y, 15, kText, true);
        text("broad " + fmtFloat("%.2f", s.broadMs) + "  solve " + fmtFloat("%.2f", s.solveMs) + "  shapes " +
                 fmtFloat("%.2f", s.rigidMs),
             panelX_ + 120, y + 1, 14, kDim);
        y += 22;
        if (model != PhysicsModel::NBody) {
            text("Tests per pass: " + fmtInt(s.tests), panelX_, y, 15, kText);
            y += 20;
            text("Candidate pairs: " + fmtInt(s.candidates), panelX_, y, 15, kText);
            y += 20;
            long long brute = (long long)n * (n - 1) / 2;
            std::string line = "Brute force: " + fmtInt(brute) + " tests";
            if (s.tests > 0 && brute > 0) line += " (" + fmtFloat("%.1f%%", 100.0 * (1.0 - (double)s.tests / brute)) + " saved)";
            text(line, panelX_, y, 15, kDim);
            y += 20;
        }
        if (model == PhysicsModel::Rigid) {
            text("Touching pairs: " + fmtInt(s.contacts) + "   shape contacts: " + fmtInt(s.bodyContacts), panelX_, y, 15, kText);
            y += 20;
        } else if (model == PhysicsModel::NBody) {
            text("Gravity interactions: " + fmtInt(s.interactions), panelX_, y, 15, kText);
            y += 20;
            text("Direct sum would need: " + fmtInt((long long)n * (n - 1)), panelX_, y, 15, kDim);
            y += 20;
        } else {
            text("Neighbour interactions: " + fmtInt(s.interactions), panelX_, y, 15, kText);
            y += 20;
        }
    }

    // step time sparkline
    y += 4;
    Rectangle r{panelX_, y, panelW_, 56};
    DrawRectangleRec(r, Color{22, 25, 31, 255});
    DrawRectangleLinesEx(r, 1, Color{55, 60, 72, 255});
    float maxMs = 1;
    for (float v : stepMsHistory_) maxMs = std::max(maxMs, v);
    for (size_t i = 1; i < stepMsHistory_.size(); i++) {
        float x0 = r.x + (float)(i - 1) / 299 * r.width, x1 = r.x + (float)i / 299 * r.width;
        DrawLineV({x0, r.y + r.height - stepMsHistory_[i - 1] / maxMs * (r.height - 4)},
                  {x1, r.y + r.height - stepMsHistory_[i] / maxMs * (r.height - 4)}, Color{120, 220, 160, 255});
    }
    text("step time, max " + fmtFloat("%.1f ms", maxMs), r.x + 6, r.y + 3, 12, kDim);
    float budget = 16.7f;
    if (budget < maxMs) {
        float by = r.y + r.height - budget / maxMs * (r.height - 4);
        DrawLineV({r.x, by}, {r.x + r.width, by}, Color{255, 120, 100, 120});
    }
    y += 64;

    // compare all broad phases on the current particles
    bool canCompare = model != PhysicsModel::NBody && n > 0;
    if (!canCompare) GuiDisable();
    if (GuiButton({panelX_, y, panelW_, 28}, "Compare all broad phases on this frame")) compareBroadphases();
    GuiEnable();
    if (dropdownOpen_) GuiLock();
    y += 34;
    if (!comparison_.empty()) y = noteRow(y, "Results are shown over the simulation.", kDim);
    return y;
}

float App::drawViewTab(float y) {
    const PhysicsModel model = world_.params.model;
    text("Particle colour", panelX_, y + 5, 15, kText);
    int mode = (int)colorMode_;
    GuiToggleGroup({panelX_ + 128, y, (panelW_ - 128 - 8) / 3, 26}, "Base;Speed;Density", &mode);
    if (mode == (int)ColorMode::Density && model != PhysicsModel::SPH) {
        toast("Density colouring is only available for SPH scenes");
        mode = (int)colorMode_;
    }
    colorMode_ = (ColorMode)mode;
    y += 32;
    const char* legend = colorMode_ == ColorMode::Speed ? "Dark blue = slow, yellow/orange = fast (relative to the mean)."
                         : colorMode_ == ColorMode::Density ? "Blue = sparse, white = rest density, red = compressed."
                                                            : "Each particle's own colour.";
    y = noteRow(y, legend, kDim);
    y += 6;
    y = checkRow(y, "Show broad-phase structure (G)", &showStructure_);
    y = noteRow(y, model == PhysicsModel::NBody ? "Draws the Barnes-Hut quadtree." : "Draws the grid cells or tree nodes of the broad phase.", kDim);
    y = checkRow(y, "Show links (ropes, cloth, blobs)", &showLinks_);
    y += 6;
    if (GuiButton({panelX_, y, panelW_, 28}, "Keyboard shortcuts and help (H)")) showHelp_ = true;
    return y + 34;
}

void App::compareBroadphases() {
    comparison_.clear();
    int n = world_.p.size();
    std::vector<float> extent(n);
    for (int i = 0; i < n; i++) extent[i] = world_.particleExtent(i);
    BroadphaseInput in{world_.p.x.data(), world_.p.y.data(), extent.data(), n};
    std::vector<std::pair<int, int>> reference;
    std::string referenceName;
    bool haveReference = false;
    const int bruteLimit = 30000;
    // brute force first so the others can be checked against it
    for (int k = (int)BroadphaseKind::Count - 1; k >= 0; k--) {
        auto kind = (BroadphaseKind)k;
        BroadphaseComparison row{broadphaseName(kind), 0, 0, -1, true};
        if (kind == BroadphaseKind::BruteForce && n > bruteLimit) {
            comparison_.push_back(row);
            continue;
        }
        auto bp = makeBroadphase(kind);
        std::vector<Pair> pairs;
        double best = 1e30;
        for (int rep = 0; rep < 3; rep++) {
            pairs.clear();
            auto t0 = std::chrono::steady_clock::now();
            bp->build(in, world_.pool());
            bp->findPairs(pairs, world_.pool());
            best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        }
        std::vector<std::pair<int, int>> sorted;
        sorted.reserve(pairs.size());
        for (const Pair& p : pairs) sorted.push_back({p.a, p.b});
        std::sort(sorted.begin(), sorted.end());
        if (!haveReference) {
            reference = std::move(sorted);
            referenceName = row.name;
            haveReference = true;
        } else {
            row.matches = sorted == reference;
        }
        row.tests = bp->stats().tests;
        row.pairs = bp->stats().pairs;
        row.ms = best;
        comparison_.push_back(row);
    }
    std::reverse(comparison_.begin(), comparison_.end());
    bool allMatch = std::all_of(comparison_.begin(), comparison_.end(), [](const BroadphaseComparison& c) { return c.matches; });
    comparisonNote_ = std::string(allMatch ? "All methods found the same pairs" : "Mismatch (!) against the reference") +
                      " (reference: " + referenceName + ")." +
                      "\nBest of 3 runs, " + std::to_string(world_.threads()) + " thread(s).";
}

void App::drawComparisonCard() {
    comparisonRect_ = {0, 0, 0, 0};
    if (comparison_.empty()) return;
    const float w = 560, rowH = 24;
    float h = 112 + rowH * comparison_.size();
    Rectangle r{worldRect_.x + 14, worldRect_.y + 44, w, h};
    comparisonRect_ = r;
    DrawRectangleRounded(r, 0.05f, 8, Color{28, 32, 42, 245});
    DrawRectangleRoundedLinesEx(r, 0.05f, 8, 1.5f, Color{110, 170, 255, 255});
    float x = r.x + 18, y = r.y + 14;
    text("Broad phases on the current " + fmtInt(world_.p.size()) + " particles", x, y, 18, WHITE, true);
    if (GuiButton({r.x + w - 74, r.y + 10, 60, 26}, "Close")) {
        comparison_.clear();
        return;
    }
    y += 32;
    const float cols[] = {0, 190, 310, 410, 480};
    const char* heads[] = {"method", "AABB tests", "pairs", "ms", "vs best"};
    for (int k = 0; k < 5; k++) text(heads[k], x + cols[k], y, 15, kDim);
    y += 22;
    double best = 1e30;
    for (const auto& c : comparison_)
        if (c.ms >= 0) best = std::min(best, c.ms);
    for (const auto& c : comparison_) {
        bool isBest = c.ms == best;
        Color col = isBest ? Color{120, 220, 160, 255} : kText;
        text(c.name, x + cols[0], y, 16, col, isBest);
        if (c.ms < 0) {
            text("skipped (too many particles for O(n^2))", x + cols[1], y, 15, kDim);
        } else {
            text(fmtInt(c.tests), x + cols[1], y, 16, col);
            text(fmtInt(c.pairs) + (c.matches ? "" : " !"), x + cols[2], y, 16, c.matches ? col : kWarn);
            text(fmtFloat("%.2f", c.ms), x + cols[3], y, 16, col);
            text(isBest ? "fastest" : fmtFloat("%.1fx", c.ms / best), x + cols[4], y, 16, col);
        }
        y += rowH;
    }
    text(comparisonNote_, x, y + 4, 14, kDim);
}
