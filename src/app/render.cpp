// Drawing of the simulation area: particles, constraints, shapes, overlays,
// tool previews and the transient UI that lives over the world.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "app.h"
#include "raymath.h"
#include "rlgl.h"
#include "scenes.h"

namespace {

Color unpack(uint32_t c) {
    Color out;
    std::memcpy(&out, &c, 4);
    return out;
}

Color lerpColor(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return Color{(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t),
                 (unsigned char)(a.b + (b.b - a.b) * t), (unsigned char)(a.a + (b.a - a.a) * t)};
}

// perceptually ordered ramp (dark blue -> teal -> yellow) for speed
Color speedRamp(float t) {
    static const Color stops[] = {{40, 50, 140, 255}, {30, 140, 180, 255}, {60, 200, 120, 255}, {250, 220, 60, 255},
                                  {255, 110, 60, 255}};
    t = std::clamp(t, 0.0f, 1.0f) * 4;
    int i = std::min(3, (int)t);
    return lerpColor(stops[i], stops[i + 1], t - i);
}

// diverging ramp (blue = sparse, white = rest density, red = compressed)
Color densityRamp(float t) {
    if (t < 0.5f) return lerpColor({50, 90, 200, 255}, {235, 235, 240, 255}, t * 2);
    return lerpColor({235, 235, 240, 255}, {220, 60, 50, 255}, (t - 0.5f) * 2);
}

void fillConvex(const std::vector<Vec2>& v, Vec2 center, Color c) {
    rlBegin(RL_TRIANGLES);
    rlColor4ub(c.r, c.g, c.b, c.a);
    for (size_t i = 0; i < v.size(); i++) {
        rlCheckRenderBatchLimit(3);
        const Vec2 &a = v[i], &b = v[(i + 1) % v.size()];
        rlVertex2f(center.x, center.y);
        rlVertex2f(a.x, a.y);
        rlVertex2f(b.x, b.y);
    }
    rlEnd();
}

void outline(const std::vector<Vec2>& v, Color c, float thick) {
    for (size_t i = 0; i < v.size(); i++) DrawLineEx({v[i].x, v[i].y}, {v[(i + 1) % v.size()].x, v[(i + 1) % v.size()].y}, thick, c);
}

void drawArrow(Vector2 from, Vector2 to, Color c) {
    DrawLineEx(from, to, 2, c);
    Vector2 d = Vector2Subtract(to, from);
    float len = Vector2Length(d);
    if (len < 6) return;
    d = Vector2Scale(d, 1 / len);
    Vector2 n{-d.y, d.x};
    Vector2 a = Vector2Add(to, Vector2Add(Vector2Scale(d, -12), Vector2Scale(n, 6)));
    Vector2 b = Vector2Add(to, Vector2Add(Vector2Scale(d, -12), Vector2Scale(n, -6)));
    DrawTriangle(to, b, a, c);
    DrawTriangle(to, a, b, c);
}

}  // namespace

void App::text(const std::string& s, float x, float y, float size, Color c, bool bold) const {
    DrawTextEx(bold ? fontBold_ : font_, s.c_str(), {std::round(x), std::round(y)}, size, 0, c);
}

float App::textWidth(const std::string& s, float size, bool bold) const {
    return MeasureTextEx(bold ? fontBold_ : font_, s.c_str(), size, 0).x;
}

Color App::particleColor(int i, float speedScale) const {
    const Particles& p = world_.p;
    switch (colorMode_) {
        case ColorMode::Speed: {
            float v = std::sqrt(p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i]);
            return speedRamp(v * speedScale);
        }
        case ColorMode::Density: {
            const auto& d = world_.sphDensity();
            if (i < (int)d.size()) return densityRamp(0.5f * d[i] / std::max(0.1f, world_.params.sphRestDensity));
            return unpack(p.color[i]);
        }
        default:
            return unpack(p.color[i]);
    }
}

void App::drawWorld() {
    const Particles& p = world_.p;
    const AABB& b = world_.bounds;
    DrawRectangleRec(worldRect_, Color{24, 27, 34, 255});
    if (world_.params.walls)
        DrawRectangleLinesEx({b.minX - 2, b.minY - 2, b.width() + 4, b.height() + 4}, 2, Color{95, 102, 118, 255});

    rlDisableBackfaceCulling();
    BeginScissorMode((int)worldRect_.x, (int)worldRect_.y, (int)worldRect_.width, (int)worldRect_.height);

    // soft bodies: translucent fill inside the ring
    for (const SoftBody& sb : world_.softBodies) {
        std::vector<Vec2> ring;
        Vec2 c{0, 0};
        for (int i : sb.ring) {
            ring.push_back({p.x[i], p.y[i]});
            c += ring.back();
        }
        if (ring.empty()) continue;
        c = c / (float)ring.size();
        Color col = unpack(p.color[sb.ring[0]]);
        col.a = 90;
        fillConvex(ring, c, col);
    }

    // constraints
    if (showLinks_) {
        for (const Link& l : world_.links) {
            DrawLineEx({p.x[l.a], p.y[l.a]}, {p.x[l.b], p.y[l.b]}, 2.5f, Color{200, 205, 220, 170});
        }
    }

    // polygons
    for (const Body& body : world_.bodies) {
        Color c = unpack(body.color);
        Color fill = c;
        fill.a = body.isStatic() ? 255 : 210;
        fillConvex(body.world, body.pos, fill);
        outline(body.world, body.isStatic() ? Color{150, 155, 170, 255} : lerpColor(c, WHITE, 0.5f), 1.5f);
        if (!body.isStatic()) {
            // orientation tick so rotation is visible
            Vec2 tip = body.pos + Vec2{std::cos(body.angle), std::sin(body.angle)} * (body.boundRadius * 0.5f);
            DrawLineEx({body.pos.x, body.pos.y}, {tip.x, tip.y}, 1.5f, Color{255, 255, 255, 120});
        }
    }

    // particles: one textured quad each (much cheaper than tessellated circles)
    float speedScale = 1.0f;
    if (colorMode_ == ColorMode::Speed) {
        double sum = 0;
        int moving = 0;
        for (int i = 0; i < p.size(); i++)
            if (p.invMass[i] > 0) {
                sum += std::sqrt(p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i]);
                moving++;
            }
        float mean = moving ? (float)(sum / moving) : 1.0f;
        speedScale = 1.0f / std::max(20.0f, 2.5f * mean);
    }
    // SPH particles are drawn wider than their collision radius so overlapping
    // discs read as a continuous liquid
    const float sphDrawRadius = world_.params.model == PhysicsModel::SPH ? 0.3f * world_.params.sphRadius : 0.0f;
    rlSetTexture(circleTex_.id);
    rlBegin(RL_QUADS);
    rlNormal3f(0, 0, 1);
    for (int i = 0; i < p.size(); i++) {
        float r = std::max({p.radius[i], 1.2f, sphDrawRadius});
        float x = p.x[i], y = p.y[i];
        if (x + r < worldRect_.x || x - r > worldRect_.x + worldRect_.width || y + r < worldRect_.y ||
            y - r > worldRect_.y + worldRect_.height)
            continue;
        Color c = particleColor(i, speedScale);
        rlCheckRenderBatchLimit(4);
        rlColor4ub(c.r, c.g, c.b, c.a);
        rlTexCoord2f(0, 0); rlVertex2f(x - r, y - r);
        rlTexCoord2f(0, 1); rlVertex2f(x - r, y + r);
        rlTexCoord2f(1, 1); rlVertex2f(x + r, y + r);
        rlTexCoord2f(1, 0); rlVertex2f(x + r, y - r);
    }
    rlEnd();
    rlSetTexture(0);

    // pinned particles get a small square marker (the grabbed one is shown by the grab line)
    for (int i = 0; i < p.size(); i++)
        if (p.invMass[i] == 0 && i != world_.grabbedParticle())
            DrawRectangleLinesEx({p.x[i] - 4, p.y[i] - 4, 8, 8}, 1.5f, Color{255, 220, 120, 255});

    if (world_.isGrabbing()) {
        Vec2 a = world_.grabAnchorWorld();
        Vector2 m = GetMousePosition();
        DrawLineEx({a.x, a.y}, m, 2, Color{255, 230, 120, 220});
        DrawCircleV({a.x, a.y}, 4, Color{255, 230, 120, 255});
    }
    EndScissorMode();
    rlEnableBackfaceCulling();
}

void App::drawStructureOverlay() {
    if (!showStructure_) return;
    std::vector<DebugRect> rects;
    const int maxRects = 6000;
    std::string label;
    if (world_.params.model == PhysicsModel::NBody) {
        if (!world_.params.barnesHut) {
            label = "Direct summation has no tree (switch to Barnes-Hut in Physics)";
        } else {
            world_.barnesHutRects(rects, maxRects);
            label = "Barnes-Hut quadtree (cells with mass)";
        }
    } else {
        if (state_ != RunState::Running && overlayStale_) {
            world_.rebuildBroadphase();
            overlayStale_ = false;
        }
        if (state_ == RunState::Running) overlayStale_ = true;
        world_.broadphase().debugRects(rects, maxRects);
        label = std::string(broadphaseName(world_.params.broadphase));
        if (world_.params.broadphase == BroadphaseKind::SweepAndPrune) label += ": sorted on x, no cells to draw";
        else if (world_.params.broadphase == BroadphaseKind::BruteForce) label += ": no structure (tests every pair)";
        else if (world_.params.broadphase == BroadphaseKind::UniformGrid || world_.params.broadphase == BroadphaseKind::SpatialHash)
            label += ": occupied cells";
        else label += ": tree nodes, colour = depth";
    }
    BeginScissorMode((int)worldRect_.x, (int)worldRect_.y, (int)worldRect_.width, (int)worldRect_.height);
    for (const DebugRect& r : rects) {
        Color c = r.depth == 0 ? Color{120, 220, 255, 255} : ColorFromHSV(fmodf(r.depth * 37.0f, 360.0f), 0.6f, 1.0f);
        c.a = 170;
        DrawRectangleLinesEx({r.minX, r.minY, r.maxX - r.minX, r.maxY - r.minY}, 1, c);
    }
    EndScissorMode();
    if ((int)rects.size() >= maxRects) label += " (first 6000 shown)";
    float w = textWidth(label, 15) + 16;
    DrawRectangleRec({worldRect_.x + 8, worldRect_.y + 8, w, 24}, Color{0, 0, 0, 170});
    text(label, worldRect_.x + 16, worldRect_.y + 12, 15, Color{150, 225, 255, 255});
}

void App::drawToolPreview() {
    Vector2 m = GetMousePosition();
    if (!CheckCollisionPointRec(m, worldRect_) || showHelp_ || dropdownOpen_) return;
    PhysicsModel model = world_.params.model;
    BeginScissorMode((int)worldRect_.x, (int)worldRect_.y, (int)worldRect_.width, (int)worldRect_.height);
    Color ghost{255, 255, 255, 90};
    switch (tool_) {
        case Tool::Attract:
        case Tool::Repel: {
            bool active = world_.mouse.active;
            Color c = tool_ == Tool::Attract ? Color{120, 200, 255, 255} : Color{255, 140, 120, 255};
            c.a = active ? 200 : 90;
            DrawCircleLinesV(m, forceRadius_, c);
            DrawCircleLinesV(m, forceRadius_ * 0.5f, Color{c.r, c.g, c.b, (unsigned char)(c.a / 2)});
            break;
        }
        case Tool::Erase:
            DrawCircleLinesV(m, eraseRadius_, Color{255, 110, 110, 200});
            break;
        case Tool::Ball: {
            bool spray = ballSpray_ || model == PhysicsModel::SPH || model == PhysicsModel::LennardJones;
            if (spray) {
                DrawCircleLinesV(m, 12, ghost);
            } else if (dragging_) {
                DrawCircleV({dragStart_.x, dragStart_.y}, ballRadius_, ghost);
                drawArrow({dragStart_.x, dragStart_.y}, m, Color{255, 230, 120, 230});
            } else {
                float r = model == PhysicsModel::NBody ? std::max(2.0f, std::sqrt(starMass_) * 0.6f) : ballRadius_;
                DrawCircleV(m, r, ghost);
            }
            break;
        }
        case Tool::Rope:
        case Tool::Wall:
            if (dragging_) {
                DrawLineEx({dragStart_.x, dragStart_.y}, m, tool_ == Tool::Wall ? 12.0f : 3.0f, ghost);
                if (tool_ == Tool::Rope && ropePinned_)
                    DrawRectangleLinesEx({dragStart_.x - 5, dragStart_.y - 5, 10, 10}, 2, Color{255, 220, 120, 255});
            } else {
                DrawCircleLinesV(m, 5, ghost);
            }
            break;
        case Tool::Blob:
            DrawCircleLinesV(m, blobRadius_, ghost);
            break;
        case Tool::Box:
            DrawRectangleLinesEx({m.x - shapeSize_ / 2, m.y - shapeSize_ / 2, shapeSize_, shapeSize_}, 1.5f, ghost);
            break;
        case Tool::Polygon:
            DrawPolyLines(m, polygonSides_, shapeSize_ / 2, 0, ghost);
            break;
        case Tool::Grab: {
            if (world_.isGrabbing() || movingBody_ >= 0 || movingParticle_ >= 0) break;
            Vec2 mp{m.x, m.y};
            int body = world_.pickBody(mp);
            if (body >= 0) {
                outline(world_.bodies[body].world, Color{255, 230, 120, 200}, 2);
            } else {
                int i = world_.pickParticle(mp, 12);
                if (i >= 0) DrawCircleLinesV({world_.p.x[i], world_.p.y[i]}, world_.p.radius[i] + 3, Color{255, 230, 120, 200});
            }
            break;
        }
        default:
            break;
    }
    EndScissorMode();
}

void App::drawHintBar() {
    float y = worldRect_.y + worldRect_.height;
    DrawRectangleRec({0, y, worldRect_.width, kHintHeight}, Color{30, 33, 41, 255});
    DrawLineEx({0, y}, {worldRect_.width, y}, 1, Color{55, 60, 72, 255});
    if (!hoverHint_.empty()) text(hoverHint_, 10, y + 6, 15, Color{255, 200, 120, 255});
    else text(hintText(), 10, y + 6, 15, Color{200, 205, 215, 255});
    if (lagging_ && state_ == RunState::Running) {
        std::string warn = "Simulation slower than real time";
        text(warn, worldRect_.width - textWidth(warn, 15) - 10, y + 6, 15, Color{255, 190, 90, 255});
    }
}

const char* App::hintText() const {
    bool running = state_ == RunState::Running;
    PhysicsModel model = world_.params.model;
    switch (tool_) {
        case Tool::Grab:
            if (running) return "Grab: drag a ball or shape to pull it, release to throw.   Space pause  |  E edit  |  H help";
            return state_ == RunState::Editing ? "Move: drag an object to reposition it.   Space runs the simulation"
                                               : "Paused. Drag to move objects, N steps one frame, Space resumes.";
        case Tool::Attract:
            return running ? "Attract: hold the left button to pull everything inside the circle."
                           : "Attract only works while running: press Space.";
        case Tool::Repel:
            return running ? "Repel: hold the left button to push everything out of the circle."
                           : "Repel only works while running: press Space.";
        case Tool::Ball:
            if (model == PhysicsModel::SPH) return "Fluid: hold the left button to pour fluid.   Space runs the simulation";
            if (model == PhysicsModel::LennardJones) return "Molecules: hold the left button to add molecules.   Space runs";
            if (model == PhysicsModel::NBody) return "Star: click to place, or drag to launch it (arrow = velocity).   Space runs";
            return ballSpray_ ? "Ball: hold the left button to spray balls.   Space runs the simulation"
                              : "Ball: click to drop, or drag to throw (arrow = velocity).   Space runs the simulation";
        case Tool::Rope: return "Rope: drag from the anchor to the free end.   Space runs the simulation";
        case Tool::Blob: return "Soft body: click to drop a pressurised blob.   Space runs the simulation";
        case Tool::Box: return "Box: click to place.   Space runs the simulation";
        case Tool::Polygon: return "Polygon: click to place a convex polygon.   Space runs the simulation";
        case Tool::Wall: return "Wall: drag to draw a static wall.   Space runs the simulation";
        case Tool::Erase: return "Erase: hold the left button over particles and shapes.   Space runs the simulation";
        default: return "";
    }
}

void App::drawToast() {
    double now = GetTime();
    if (toastText_.empty() || now > toastUntil_) return;
    float alpha = (float)std::min(1.0, (toastUntil_ - now) / 0.4);
    float w = textWidth(toastText_, 17) + 28;
    float x = worldRect_.x + (worldRect_.width - w) / 2, y = worldRect_.y + 14;
    DrawRectangleRounded({x, y, w, 32}, 0.4f, 6, Color{40, 46, 60, (unsigned char)(235 * alpha)});
    text(toastText_, x + 14, y + 7, 17, Color{235, 240, 250, (unsigned char)(255 * alpha)});
}

void App::drawCaption() {
    if (caption_.empty()) return;
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= caption_.size()) {
        size_t nl = caption_.find('\n', start);
        lines.push_back(caption_.substr(start, nl == std::string::npos ? std::string::npos : nl - start));
        if (nl == std::string::npos) break;
        start = nl + 1;
    }
    const float size = 24, lineH = 32;
    float w = 0;
    for (size_t i = 0; i < lines.size(); i++) w = std::max(w, textWidth(lines[i], i == 0 ? size : size - 4, i == 0));
    w += 40;
    float h = lines.size() * lineH + 18;
    float x = worldRect_.x + (worldRect_.width - w) / 2, y = worldRect_.y + worldRect_.height - h - 16;
    DrawRectangleRounded({x, y, w, h}, 0.15f, 8, Color{8, 10, 16, 215});
    DrawRectangleRoundedLinesEx({x, y, w, h}, 0.15f, 8, 1.5f, Color{110, 170, 255, 200});
    for (size_t i = 0; i < lines.size(); i++)
        text(lines[i], x + 20, y + 10 + i * lineH, i == 0 ? size : size - 4, i == 0 ? Color{255, 255, 255, 255} : Color{200, 210, 225, 255},
             i == 0);
}

void App::drawHelpOverlay() {
    float w = 660, h = 580;
    float x = worldRect_.x + (worldRect_.width - w) / 2, y = worldRect_.y + (worldRect_.height - h) / 2;
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Color{0, 0, 0, 120});
    DrawRectangleRounded({x, y, w, h}, 0.04f, 8, Color{32, 36, 46, 250});
    DrawRectangleRoundedLinesEx({x, y, w, h}, 0.04f, 8, 1.5f, Color{110, 170, 255, 255});
    text("How it works", x + 24, y + 18, 24, WHITE, true);
    const char* lines[] = {
        "States (top of the panel):",
        "   RUNNING  time advances. Grab / Attract / Repel act on the simulation.",
        "   PAUSED   time is frozen. Step (N) advances one frame; Grab moves objects.",
        "   EDITING  time is frozen. Edit tools add, draw and erase objects.",
        "   Picking an edit tool enters EDITING automatically.",
        "",
        "Keys:",
        "   Space  run / pause          E  edit mode on / off       N  step (paused)",
        "   R      reset the scene      G  broad-phase overlay      C  colour mode",
        "   1-0    tools in toolbar order                           Tab  next panel tab",
        "   H      this help            Esc  close / cancel        Q  quit",
        "",
        "Scenes pick a physics model: rigid bodies, SPH fluid, Lennard-Jones",
        "molecules or N-body gravity. Each model shows its own settings in the",
        "Physics tab; tools that a model can't use are greyed out with a reason.",
        "",
        "Performance tab: switch the broad phase (how candidate pairs are found),",
        "thread count and GPU compute, and compare all broad phases on the",
        "current particles.",
    };
    float ty = y + 60;
    for (const char* l : lines) {
        bool heading = l[0] != ' ' && l[0] != 0 && l[std::strlen(l) - 1] == ':';
        text(l, x + 24, ty, 16, heading ? Color{150, 200, 255, 255} : Color{220, 225, 235, 255}, heading);
        ty += 23;
    }
    text("Click anywhere or press H to close", x + 24, y + h - 34, 15, Color{150, 155, 170, 255});
}

void App::drawCursor() {
    Vector2 m = GetMousePosition();
    Vector2 pts[] = {m, {m.x, m.y + 20}, {m.x + 5, m.y + 15}, {m.x + 9, m.y + 23}, {m.x + 12, m.y + 21}, {m.x + 8, m.y + 14}, {m.x + 14, m.y + 14}};
    // white arrow with a dark outline
    DrawTriangle(pts[0], pts[1], pts[6], WHITE);
    DrawTriangle(pts[2], pts[3], pts[4], WHITE);
    DrawTriangle(pts[2], pts[4], pts[5], WHITE);
    for (int i = 0; i < 7; i++) DrawLineEx(pts[i], pts[(i + 1) % 7], 1.2f, BLACK);
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) DrawCircleLinesV(m, 10, Color{255, 230, 120, 255});
}
