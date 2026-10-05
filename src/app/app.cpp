#include "app.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>

#include "raymath.h"
#include "rlgl.h"
#include "scenes.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "font_dejavu.h"
#include "font_dejavu_bold.h"

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

namespace {

bool isEditTool(Tool t) { return t >= Tool::Ball; }

Texture2D makeCircleTexture() {
    const int size = 64;
    Image img = GenImageColor(size, size, BLANK);
    Color* px = (Color*)img.data;
    for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++) {
            float dx = x + 0.5f - size / 2.0f, dy = y + 0.5f - size / 2.0f;
            float d = std::sqrt(dx * dx + dy * dy);
            float a = std::clamp(size / 2.0f - d, 0.0f, 1.0f);  // 1px anti-aliased edge
            px[y * size + x] = Color{255, 255, 255, (unsigned char)(a * 255)};
        }
    Texture2D tex = LoadTextureFromImage(img);
    UnloadImage(img);
    GenTextureMipmaps(&tex);
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
    return tex;
}

}  // namespace

App::App(const AppOptions& options) : options_(options) {
    unsigned flags = FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT;
    if (options_.recordPath.empty()) flags |= FLAG_WINDOW_RESIZABLE;
    if (options_.gpuSelfTest) flags = FLAG_WINDOW_HIDDEN;
    SetConfigFlags(flags);
    InitWindow(options_.width, options_.height, "Particle Simulator");
    SetExitKey(KEY_Q);
    SetWindowMinSize(1100, 700);  // below this the panel would cover the world
    SetTargetFPS(options_.recordPath.empty() ? 60 : 0);

    font_ = LoadFont_FontDejavu();
    fontBold_ = LoadFont_FontDejavuBold();
    SetTextureFilter(font_.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(fontBold_.texture, TEXTURE_FILTER_BILINEAR);
    circleTex_ = makeCircleTexture();
    applyStyle();

    int threads = options_.threads > 0 ? options_.threads : ThreadPool::hardwareThreads();
    world_.setThreads(threads);
    gpu_.init();

    updateLayout();
    scene_ = std::clamp(options_.scene, 0, (int)sceneList().size() - 1);
    loadCurrentScene();
    if (!options_.recordPath.empty()) beginRecording();
}

App::~App() {
    endRecording();
    gpu_.release();
    UnloadTexture(circleTex_);
    // the embedded fonts' glyph tables are static arrays: only the textures are ours to free
    UnloadTexture(font_.texture);
    UnloadTexture(fontBold_.texture);
    CloseWindow();
}

int App::run() {
    if (options_.gpuSelfTest) {
        // headless check: run the same scene on the CPU and on the GPU and compare
        if (!gpu_.available()) {
            std::printf("GPU self-test: unavailable (%s)\n", gpu_.status().c_str());
            return 2;
        }
        struct Result {
            double energyChange;
            int outside, deepOverlaps;
            bool finite;
        };
        auto measure = [&](bool onGpu) {
            loadScene(world_, 1, 4000, 7);  // elastic gas
            double e0 = world_.diagnostics().kinetic;
            if (onGpu) gpu_.upload(world_);
            for (int i = 0; i < 180; i++) {
                if (onGpu) gpu_.step(world_);
                else world_.step();
            }
            world_.computeDiagnostics();
            Result r{(world_.diagnostics().kinetic - e0) / e0, 0, 0, true};
            for (int i = 0; i < world_.p.size(); i++) {
                r.finite &= std::isfinite(world_.p.x[i]) && std::isfinite(world_.p.vx[i]);
                r.outside += !world_.bounds.contains({world_.p.x[i], world_.p.y[i]});
            }
            // pairs overlapping by more than a quarter of their radii
            auto bp = makeBroadphase(BroadphaseKind::UniformGrid);
            bp->build({world_.p.x.data(), world_.p.y.data(), world_.p.radius.data(), world_.p.size()}, world_.pool());
            std::vector<Pair> pairs;
            bp->findPairs(pairs, world_.pool());
            for (const Pair& pr : pairs) {
                float d = std::hypot(world_.p.x[pr.a] - world_.p.x[pr.b], world_.p.y[pr.a] - world_.p.y[pr.b]);
                if (d < 0.75f * (world_.p.radius[pr.a] + world_.p.radius[pr.b])) r.deepOverlaps++;
            }
            return r;
        };
        Result cpu = measure(false), gpu = measure(true);
        std::printf("GPU self-test: %d particles, 180 steps, last GPU step %.2f ms\n", world_.p.size(), gpu_.lastStepMs());
        std::printf("  CPU: energy %+.2f%%, outside %d, deep overlaps %d\n", cpu.energyChange * 100, cpu.outside, cpu.deepOverlaps);
        std::printf("  GPU: energy %+.2f%%, outside %d, deep overlaps %d, finite %s\n", gpu.energyChange * 100, gpu.outside,
                    gpu.deepOverlaps, gpu.finite ? "yes" : "NO");
        // The GPU solves contacts Jacobi-style (all at once, from the old state), which is
        // less exact than the CPU's sequential solve: it typically loses ~5-10% of the
        // kinetic energy here, so only gross failures count.
        bool ok = gpu.finite && gpu.outside == 0 && std::fabs(gpu.energyChange) < 0.2 &&
                  gpu.deepOverlaps <= 3 * cpu.deepOverlaps + world_.p.size() / 200;
        std::printf("GPU self-test: %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    }
    while (!WindowShouldClose() && !quit_) {
        frame();
        if (options_.recordFrames > 0 && frameIndex_ >= options_.recordFrames) break;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// frame
// ---------------------------------------------------------------------------

void App::updateLayout() {
    float w = (float)GetScreenWidth(), h = (float)GetScreenHeight();
    panelRect_ = {w - kPanelWidth, 0, kPanelWidth, h};
    worldRect_ = {0, kToolbarHeight, std::max(100.0f, w - kPanelWidth), std::max(100.0f, h - kToolbarHeight - kHintHeight)};
    AABB b{worldRect_.x + 2, worldRect_.y + 2, worldRect_.x + worldRect_.width - 2, worldRect_.y + worldRect_.height - 2};
    if (b.minX != world_.bounds.minX || b.minY != world_.bounds.minY || b.maxX != world_.bounds.maxX ||
        b.maxY != world_.bounds.maxY) {
        world_.bounds = b;
        gpuDirty_ = true;
    }
}

void App::frame() {
    updateLayout();
    if (!options_.captionFile.empty()) {
        std::ifstream in(options_.captionFile);
        std::stringstream ss;
        ss << in.rdbuf();
        caption_ = ss.str();
        while (!caption_.empty() && (caption_.back() == '\n' || caption_.back() == ' ')) caption_.pop_back();
    }
    float frameTime = recording() ? 1.0f / 60.0f : GetFrameTime();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 m = GetMousePosition();
        if (showHelp_) {
            showHelp_ = false;  // this click only closes the help
            press_ = Press::Swallow;
        } else if (dropdownOpen_) {
            press_ = Press::Swallow;  // only the open dropdown may react
        } else if (CheckCollisionPointRec(m, worldRect_) && !CheckCollisionPointRec(m, comparisonRect_)) {
            press_ = Press::World;
        } else {
            press_ = Press::Ui;
        }
    }

    handleShortcuts();
    handleWorldMouse();
    advanceSimulation(frameTime);

    BeginDrawing();
    ClearBackground(Color{18, 20, 26, 255});
    drawWorld();
    drawStructureOverlay();
    drawToolPreview();
    drawComparisonCard();
    drawToolbar();
    drawPanel();
    drawHintBar();
    drawToast();
    drawCaption();
    if (showHelp_) drawHelpOverlay();
    if (recording()) {
        drawCursor();
        captureFrame();
    }
    EndDrawing();
    // released this frame: the UI (which reacts on release) has now seen it
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) press_ = Press::None;

    if (!options_.statusFile.empty()) {
        if (FILE* f = std::fopen(options_.statusFile.c_str(), "w")) {
            std::fprintf(f, "%lld\n", frameIndex_);
            std::fclose(f);
        }
    }
    frameIndex_++;
}

void App::handleShortcuts() {
    if (IsKeyPressed(KEY_SPACE)) setState(state_ == RunState::Running ? RunState::Paused : RunState::Running);
    if (IsKeyPressed(KEY_E)) setState(state_ == RunState::Editing ? RunState::Paused : RunState::Editing);
    if ((IsKeyPressed(KEY_N) || IsKeyPressed(KEY_RIGHT)) && state_ == RunState::Paused) stepOnce();
    if (IsKeyPressed(KEY_R)) loadCurrentScene();
    if (IsKeyPressed(KEY_G)) showStructure_ = !showStructure_;
    if (IsKeyPressed(KEY_H) || IsKeyPressed(KEY_F1)) showHelp_ = !showHelp_;
    if (IsKeyPressed(KEY_TAB)) tab_ = (Tab)(((int)tab_ + 1) % 3);
    if (IsKeyPressed(KEY_C)) {
        colorMode_ = (ColorMode)(((int)colorMode_ + 1) % 3);
        if (colorMode_ == ColorMode::Density && world_.params.model != PhysicsModel::SPH) colorMode_ = ColorMode::Base;
    }
    const int keys[] = {KEY_ONE, KEY_TWO, KEY_THREE, KEY_FOUR, KEY_FIVE, KEY_SIX, KEY_SEVEN, KEY_EIGHT, KEY_NINE, KEY_ZERO};
    for (int k = 0; k < (int)Tool::Count; k++)
        if (IsKeyPressed(keys[k])) selectTool((Tool)k);
    if (IsKeyPressed(KEY_ESCAPE)) {
        if (showHelp_) showHelp_ = false;
        else if (dragging_) dragging_ = false;
        else if (state_ == RunState::Editing) setState(RunState::Paused);
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

void App::setState(RunState s) {
    if (s == state_) return;
    world_.releaseGrab();
    movingParticle_ = movingBody_ = -1;
    dragging_ = false;
    accumulator_ = 0;
    RunState old = state_;
    state_ = s;
    if (s == RunState::Editing) {
        if (!isEditTool(tool_)) {
            std::string why;
            tool_ = toolAvailable(Tool::Ball, &why) ? Tool::Ball : Tool::Erase;
        }
        toast("Edit mode: simulation paused. Space runs it again.");
    } else {
        if (isEditTool(tool_)) tool_ = Tool::Grab;
        if (s == RunState::Paused) toast(old == RunState::Editing ? "Left edit mode (paused)" : "Paused: N steps one frame");
        else toast("Running");
    }
}

bool App::toolAvailable(Tool t, std::string* reason) const {
    PhysicsModel m = world_.params.model;
    auto fail = [&](const char* why) {
        if (reason) *reason = why;
        return false;
    };
    switch (t) {
        case Tool::Grab:
            if (gpuActive_) return fail("Grab is not available while the GPU runs the physics");
            return true;
        case Tool::Rope:
        case Tool::Blob:
            if (m != PhysicsModel::Rigid) return fail("Ropes and soft bodies need a Rigid-body scene");
            if (gpuActive_) return fail("Turn off GPU compute to add ropes or soft bodies");
            return true;
        case Tool::Box:
        case Tool::Polygon:
        case Tool::Wall:
            if (m != PhysicsModel::Rigid && m != PhysicsModel::SPH) return fail("Shapes need a Rigid-body or SPH scene");
            if (gpuActive_) return fail("Turn off GPU compute to add shapes");
            return true;
        default:
            return true;
    }
}

void App::selectTool(Tool t) {
    std::string why;
    if (!toolAvailable(t, &why)) {
        toast(why);
        return;
    }
    if (isEditTool(t) && state_ != RunState::Editing) setState(RunState::Editing);
    if (!isEditTool(t) && state_ == RunState::Editing && t != Tool::Grab) setState(RunState::Running);
    tool_ = t;
}

void App::loadCurrentScene() {
    world_.releaseGrab();
    movingParticle_ = movingBody_ = -1;
    dragging_ = false;
    const SceneInfo& info = sceneList()[scene_];
    if (info.defaultCount == 0) sceneCount_ = 0;
    else if (sceneCount_ <= 0) sceneCount_ = info.defaultCount;
    sceneCount_ = info.defaultCount ? std::clamp(sceneCount_, info.minCount, info.maxCount) : 0;
    loadScene(world_, scene_, sceneCount_, seed_);
    energyHistory_.clear();
    stepMsHistory_.clear();
    comparison_.clear();
    comparisonNote_.clear();
    accumulator_ = 0;
    gpuDirty_ = true;
    std::string why;
    if (gpuActive_ && !gpuAllowed(&why)) {
        gpuActive_ = false;
        toast("GPU compute switched off: " + why);
    }
    if (!toolAvailable(tool_, nullptr)) tool_ = state_ == RunState::Editing ? Tool::Ball : Tool::Grab;
    if (colorMode_ == ColorMode::Density && world_.params.model != PhysicsModel::SPH) colorMode_ = ColorMode::Base;
}

void App::markWorldEdited() {
    world_.invalidate();
    world_.computeDiagnostics();
    gpuDirty_ = true;
    overlayStale_ = true;
    comparison_.clear();
}

bool App::gpuAllowed(std::string* reason) const {
    auto fail = [&](const std::string& why) {
        if (reason) *reason = why;
        return false;
    };
    if (!gpu_.available()) return fail(gpu_.status());
    if (world_.params.model != PhysicsModel::Rigid) return fail("only the Rigid-body model runs on the GPU");
    if (!world_.links.empty() || !world_.softBodies.empty() || !world_.bodies.empty())
        return fail("the scene has ropes, soft bodies or shapes (GPU handles plain balls)");
    return true;
}

void App::setGpuEnabled(bool on) {
    std::string why;
    if (on && !gpuAllowed(&why)) {
        toast("GPU compute unavailable: " + why);
        return;
    }
    if (on == gpuActive_) return;
    gpuActive_ = on;
    gpuDirty_ = true;
    world_.releaseGrab();
    movingParticle_ = movingBody_ = -1;
    dragging_ = false;
    // keep the tool consistent with the state: edit tools only while editing
    if (on && !toolAvailable(tool_, nullptr)) tool_ = state_ == RunState::Editing ? Tool::Ball : Tool::Attract;
    toast(on ? "Physics now runs on the GPU" : "Physics back on the CPU");
}

void App::toast(const std::string& text) {
    toastText_ = text;
    toastUntil_ = GetTime() + 2.5;
}

// ---------------------------------------------------------------------------
// simulation
// ---------------------------------------------------------------------------

void App::advanceSimulation(float frameTime) {
    if (state_ != RunState::Running) {
        accumulator_ = 0;
        lagging_ = false;
        return;
    }
    const double dt = world_.params.dt;
    accumulator_ += std::min(frameTime, 0.1f);
    int steps = 0;
    // a little tolerance keeps 60 Hz displays at exactly one step per frame
    while (accumulator_ >= dt * 0.98 && steps < 4) {
        stepOnce();
        accumulator_ -= dt;
        steps++;
    }
    lagging_ = accumulator_ >= dt;
    if (lagging_) accumulator_ = 0;  // can't keep up: drop time instead of spiralling
    if (accumulator_ < 0) accumulator_ = 0;
}

void App::stepOnce() {
    double ms;
    if (gpuActive_) {
        if (gpuDirty_) {
            gpu_.upload(world_);
            gpuDirty_ = false;
        }
        gpu_.step(world_);
        ms = gpu_.lastStepMs();
    } else {
        world_.step();
        ms = world_.stats().stepMs;
    }
    stepMsHistory_.push_back((float)ms);
    if (stepMsHistory_.size() > 300) stepMsHistory_.pop_front();
    const Diagnostics& d = world_.diagnostics();
    energyHistory_.push_back({(float)d.kinetic, (float)d.potential});
    if (energyHistory_.size() > 600) energyHistory_.pop_front();
}

// ---------------------------------------------------------------------------
// mouse
// ---------------------------------------------------------------------------

void App::handleWorldMouse() {
    Vector2 m = GetMousePosition();
    Vec2 mp{m.x, m.y};
    bool inWorld = CheckCollisionPointRec(m, worldRect_) && !CheckCollisionPointRec(m, comparisonRect_);
    world_.mouse.active = false;
    // tools only react to presses that started in the world (see press_)
    const bool owned = press_ == Press::World;
    bool pressed = owned && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    bool down = owned && IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    bool released = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) dragging_ = false;  // cancel a drag
    // keep drags inside the world
    Vec2 clamped{std::clamp(mp.x, world_.bounds.minX, world_.bounds.maxX), std::clamp(mp.y, world_.bounds.minY, world_.bounds.maxY)};

    switch (tool_) {
        case Tool::Grab: {
            if (pressed) {
                int body = world_.pickBody(mp);
                int particle = body < 0 ? world_.pickParticle(mp, 12) : -1;
                if (state_ == RunState::Running) {
                    if (body >= 0) world_.grabBody(body, mp);
                    else if (particle >= 0) world_.grabParticle(particle, mp);
                } else {
                    movingBody_ = body;
                    movingParticle_ = particle;
                    if (body >= 0) moveOffset_ = world_.bodies[body].pos - mp;
                    if (particle >= 0) moveOffset_ = Vec2{world_.p.x[particle], world_.p.y[particle]} - mp;
                }
            }
            if (down) {
                if (world_.isGrabbing()) world_.setGrabTarget(clamped);
                if (movingParticle_ >= 0 && movingParticle_ < world_.p.size()) {
                    Vec2 t = clamped + moveOffset_;
                    world_.p.x[movingParticle_] = world_.p.px[movingParticle_] = t.x;
                    world_.p.y[movingParticle_] = world_.p.py[movingParticle_] = t.y;
                    world_.p.vx[movingParticle_] = world_.p.vy[movingParticle_] = 0;  // placed, like bodies
                    markWorldEdited();
                }
                if (movingBody_ >= 0 && movingBody_ < (int)world_.bodies.size()) {
                    Body& b = world_.bodies[movingBody_];
                    b.pos = clamped + moveOffset_;
                    b.vel = {0, 0};
                    b.angVel = 0;
                    b.updateWorld();
                    markWorldEdited();
                }
            }
            if (released) {
                world_.releaseGrab();
                movingParticle_ = movingBody_ = -1;
            }
            break;
        }
        case Tool::Attract:
        case Tool::Repel:
            if (down && inWorld && state_ == RunState::Running) {
                world_.mouse.active = true;
                world_.mouse.pos = mp;
                world_.mouse.radius = forceRadius_;
                world_.mouse.strength = tool_ == Tool::Attract ? forceStrength_ : -forceStrength_;
            }
            break;
        case Tool::Ball: {
            if (ballSprays()) {
                if (down && inWorld) sprayParticles(mp);
            } else {
                if (pressed) {
                    dragging_ = true;
                    dragStart_ = mp;
                }
                if (released && dragging_) {
                    spawnBall(dragStart_, (clamped - dragStart_) * 2.5f);
                    dragging_ = false;
                }
            }
            break;
        }
        case Tool::Rope:
        case Tool::Wall:
            if (pressed) {
                dragging_ = true;
                dragStart_ = mp;
            }
            if (released && dragging_) {
                dragging_ = false;
                if (length(clamped - dragStart_) > 12) {
                    if (tool_ == Tool::Rope) spawnRope(dragStart_, clamped);
                    else {
                        world_.bodies.push_back(makeWallBody(dragStart_, clamped, 12, packRGBA(110, 110, 125)));
                        markWorldEdited();
                    }
                }
            }
            break;
        case Tool::Blob:
            if (pressed) spawnBlob(mp);
            break;
        case Tool::Box:
        case Tool::Polygon:
            if (pressed) spawnShape(mp, tool_ == Tool::Polygon);
            break;
        case Tool::Erase:
            if (down && inWorld) eraseAt(mp);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// edit tools
// ---------------------------------------------------------------------------

namespace {
std::mt19937& toolRng() {
    static std::mt19937 rng(12345);
    return rng;
}
float urand(float a, float b) { return std::uniform_real_distribution<float>(a, b)(toolRng()); }
int nextGroup() {
    static int group = 100000;
    return ++group;
}
}  // namespace

void App::spawnBall(Vec2 pos, Vec2 vel) {
    PhysicsModel model = world_.params.model;
    float r = ballRandomSize_ ? urand(0.5f * ballRadius_, ballRadius_) : ballRadius_;
    if (model == PhysicsModel::NBody) {
        world_.addParticle(pos, vel, std::max(2.0f, std::sqrt(starMass_) * 0.6f), packRGBA(255, 230, 180), 0, starMass_);
    } else {
        // don't drop a ball inside a shape
        if (world_.pickBody(pos) >= 0) return;
        world_.addParticle(pos, vel, r, paletteColor((int)urand(0, 10)));
    }
    markWorldEdited();
}

void App::sprayParticles(Vec2 pos) {
    PhysicsModel model = world_.params.model;
    // particles per frame and spacing per model
    int perFrame = model == PhysicsModel::SPH ? 5 : model == PhysicsModel::LennardJones ? 2 : 1;
    float radius = model == PhysicsModel::SPH ? 3.5f : model == PhysicsModel::LennardJones ? 4.0f : ballRadius_;
    float minGap = model == PhysicsModel::SPH ? 4.0f : 2.0f * radius;
    for (int k = 0; k < perFrame; k++) {
        Vec2 p = pos + Vec2{urand(-12, 12), urand(-12, 12)};
        if (!world_.bounds.contains(p) || world_.pickBody(p) >= 0) continue;
        if (world_.pickParticle(p, minGap - radius) >= 0) continue;
        if (model == PhysicsModel::SPH) {
            world_.addParticle(p, {0, 0}, radius, packRGBA(60, 140, 230));
        } else if (model == PhysicsModel::LennardJones) {
            float vT = std::sqrt(world_.params.targetTemperature * world_.params.ljEpsilon);
            std::normal_distribution<float> n(0, vT);
            world_.addParticle(p, {n(toolRng()), n(toolRng())}, radius, paletteColor(6), 0, 1.0f);
        } else if (model == PhysicsModel::NBody) {
            world_.addParticle(p, {0, 0}, 1.5f, packRGBA(255, 230, 180), 0, 0.5f);
        } else {
            float r = ballRandomSize_ ? urand(0.5f * ballRadius_, ballRadius_) : ballRadius_;
            world_.addParticle(p, {0, 0}, r, paletteColor((int)urand(0, 10)));
        }
    }
    markWorldEdited();
}

void App::eraseAt(Vec2 pos) {
    bool changed = false;
    for (int i = world_.p.size() - 1; i >= 0; i--) {
        float dx = world_.p.x[i] - pos.x, dy = world_.p.y[i] - pos.y;
        float r = eraseRadius_ + world_.p.radius[i];
        if (dx * dx + dy * dy < r * r) {
            world_.removeParticle(i);  // descending order: indices below i are unaffected
            changed = true;
        }
    }
    int body = world_.pickBody(pos);
    if (body >= 0) {
        world_.removeBody(body);
        changed = true;
    }
    if (changed) markWorldEdited();
}

void App::spawnBlob(Vec2 pos) {
    if (world_.pickBody(pos) >= 0) return;
    buildSoftBody(world_, pos, blobRadius_, paletteColor((int)urand(0, 10)), nextGroup(), {0, 0});
    markWorldEdited();
}

void App::spawnRope(Vec2 a, Vec2 b) {
    buildRope(world_, a, b, 4, nextGroup(), ropePinned_, 12);
    markWorldEdited();
}

void App::spawnShape(Vec2 pos, bool polygon) {
    float density = shapeStatic_ ? 0.0f : kDefaultDensity;
    uint32_t color = shapeStatic_ ? packRGBA(110, 110, 125) : paletteColor((int)urand(0, 10));
    if (polygon)
        world_.bodies.push_back(makeRegularPolygonBody(pos, shapeSize_ * 0.5f, polygonSides_, urand(0, 3.14159f), density, color));
    else
        world_.bodies.push_back(makeBoxBody(pos, shapeSize_, shapeSize_, 0, density, color));
    markWorldEdited();
}

// ---------------------------------------------------------------------------
// recording
// ---------------------------------------------------------------------------

void App::beginRecording() {
#ifndef _WIN32
    signal(SIGPIPE, SIG_IGN);  // if ffmpeg dies, report the failed write instead of being killed
#endif
    // yuv420p needs even dimensions: pad odd window sizes by one pixel
    char cmd[1024];
    std::snprintf(cmd, sizeof cmd,
                  "ffmpeg -y -loglevel error -f rawvideo -pixel_format rgba -video_size %dx%d -framerate 60 -i - "
                  "-vf \"pad=ceil(iw/2)*2:ceil(ih/2)*2\" -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p "
                  "-movflags +faststart \"%s\"",
                  GetScreenWidth(), GetScreenHeight(), options_.recordPath.c_str());
    recordPipe_ = popen(cmd, "w");
    if (!recordPipe_) {
        std::fprintf(stderr, "recording: could not start ffmpeg (is it installed?)\n");
        quit_ = true;
    }
}

void App::captureFrame() {
    if (!recordPipe_) return;
    rlDrawRenderBatchActive();
    int w = GetScreenWidth(), h = GetScreenHeight();
    unsigned char* pixels = rlReadScreenPixels(w, h);  // top-down RGBA
    size_t bytes = (size_t)w * h * 4;
    bool ok = std::fwrite(pixels, 1, bytes, recordPipe_) == bytes;
    RL_FREE(pixels);
    if (!ok) {
        std::fprintf(stderr, "recording: ffmpeg stopped accepting frames (bad output path?)\n");
        endRecording();
        quit_ = true;
    }
}

void App::endRecording() {
    if (recordPipe_) {
        int status = pclose(recordPipe_);
        recordPipe_ = nullptr;
        if (status != 0) std::fprintf(stderr, "recording: ffmpeg exited with status %d\n", status);
        else std::printf("recording: wrote %s (%lld frames)\n", options_.recordPath.c_str(), frameIndex_);
    }
}
