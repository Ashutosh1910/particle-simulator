#include "scenes.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace {

constexpr float kPi = 3.14159265f;

enum SceneId {
    BallPit,
    ElasticGas,
    MixedSizes,
    StressTest,
    RopesCloth,
    SoftBodies,
    BoxesRamps,
    DamBreak,
    LJGas,
    LJCrystal,
    Galaxy,
    GalaxyCollision,
};

struct Rng {
    std::mt19937 gen;
    explicit Rng(unsigned seed) : gen(seed) {}
    float uniform(float a, float b) { return std::uniform_real_distribution<float>(a, b)(gen); }
    float normal(float sd) { return std::normal_distribution<float>(0, sd)(gen); }
    int integer(int a, int b) { return std::uniform_int_distribution<int>(a, b)(gen); }
};

// Places up to `count` particles on a jittered lattice inside `area`.
void fillLattice(World& w, Rng& rng, AABB area, int count, float spacing, float rMin, float rMax, float speed,
                 int colorBase, float mass = -1) {
    int cols = std::max(1, (int)(area.width() / spacing));
    for (int k = 0; k < count; k++) {
        int cx = k % cols, cy = k / cols;
        float x = area.minX + spacing * (cx + 0.5f) + rng.uniform(-0.1f, 0.1f) * spacing;
        float y = area.maxY - spacing * (cy + 0.5f);
        if (y < area.minY) break;
        float r = rng.uniform(rMin, rMax);
        w.addParticle({x, y}, {rng.uniform(-speed, speed), rng.uniform(-speed, speed)}, r,
                      paletteColor(colorBase + rng.integer(0, 4)), 0, mass);
    }
}

void addCloth(World& w, Vec2 topLeft, int cols, int rows, float spacing, int group) {
    int base = w.p.size();
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < cols; x++) {
            uint8_t shade = (uint8_t)(150 + 80 * ((x / 4 + y / 4) % 2));
            int idx = w.addParticle(topLeft + Vec2{x * spacing, y * spacing}, {0, 0}, spacing * 0.3f,
                                    packRGBA(90, shade, 230), group, 0.2f);
            if (y == 0 && (x % 4 == 0 || x == cols - 1)) w.p.invMass[idx] = 0;
        }
    auto id = [&](int x, int y) { return base + y * cols + x; };
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < cols; x++) {
            if (x + 1 < cols) w.links.push_back({id(x, y), id(x + 1, y), spacing, 1.0f});
            if (y + 1 < rows) w.links.push_back({id(x, y), id(x, y + 1), spacing, 1.0f});
        }
}

void addGalaxy(World& w, Rng& rng, Vec2 c, Vec2 drift, int count, float radius, float coreMass, int colorBase, bool clockwise) {
    w.addParticle(c, drift, 9, packRGBA(255, 240, 200), 0, coreMass);
    const float G = w.params.G;
    struct Star { float r, a, m; };
    std::vector<Star> stars(count);
    for (Star& s : stars) {
        s.r = radius * (0.08f + 0.92f * std::sqrt(rng.uniform(0, 1)));
        s.a = rng.uniform(0, 2 * kPi);
        s.m = rng.uniform(0.2f, 0.8f);
    }
    std::sort(stars.begin(), stars.end(), [](const Star& a, const Star& b) { return a.r < b.r; });
    float enclosed = coreMass;
    for (int i = 0; i < count; i++) {
        const Star& s = stars[i];
        // circular speed for the mass inside this radius (core + inner disc) under
        // Plummer-softened gravity: v^2 = G M r^2 / (r^2 + eps^2)^(3/2)
        float soft2 = s.r * s.r + w.params.softening * w.params.softening;
        float v = std::sqrt(G * enclosed * s.r * s.r / (soft2 * std::sqrt(soft2))) * rng.uniform(0.95f, 1.05f);
        Vec2 dir{std::cos(s.a), std::sin(s.a)};
        Vec2 tangent = clockwise ? Vec2{dir.y, -dir.x} : Vec2{-dir.y, dir.x};
        int shade = std::min(4, (int)(5 * s.r / radius));
        w.addParticle(c + dir * s.r, drift + tangent * v, 1.2f + s.m, paletteColor(colorBase + shade), 0, s.m);
        enclosed += s.m;
    }
}

}  // namespace

void buildRope(World& w, Vec2 a, Vec2 b, float radius, int group, bool pinFirst, float endRadius) {
    float len = length(b - a);
    int n = std::max(2, (int)(len / (2.2f * radius)));
    int prev = -1;
    for (int i = 0; i <= n; i++) {
        Vec2 p = a + (b - a) * ((float)i / n);
        bool heavyEnd = i == n && endRadius > 0;
        float r = heavyEnd ? endRadius : radius;
        int idx = w.addParticle(p, {0, 0}, r, heavyEnd ? paletteColor(1) : packRGBA(220, 200, 160), group,
                                massForRadius(r) * (heavyEnd ? 2.0f : 1.0f));
        if (i == 0 && pinFirst) w.p.invMass[idx] = 0;
        if (prev >= 0) w.links.push_back({prev, idx, length(p - Vec2{w.p.x[prev], w.p.y[prev]}), 1.0f});
        prev = idx;
    }
}

void buildSoftBody(World& w, Vec2 center, float radius, uint32_t color, int group, Vec2 velocity) {
    const float particleRadius = 6;
    int n = std::max(12, (int)(2 * kPi * radius / (2 * particleRadius + 1)));
    SoftBody sb;
    for (int i = 0; i < n; i++) {
        float a = 2 * kPi * i / n;
        sb.ring.push_back(w.addParticle(center + Vec2{std::cos(a), std::sin(a)} * radius, velocity, particleRadius, color,
                                        group, 0.3f));
    }
    sb.restArea = 0;
    for (int i = 0; i < n; i++) {
        int a = sb.ring[i], b = sb.ring[(i + 1) % n];
        sb.restArea += 0.5f * (w.p.x[a] * w.p.y[b] - w.p.x[b] * w.p.y[a]);
        w.links.push_back({a, b, length(Vec2{w.p.x[b] - w.p.x[a], w.p.y[b] - w.p.y[a]}), 0.8f});
    }
    sb.stiffness = 0.6f;
    w.softBodies.push_back(sb);
}

uint32_t paletteColor(int i) {
    static const uint32_t colors[] = {
        packRGBA(86, 180, 233), packRGBA(230, 159, 0), packRGBA(0, 158, 115), packRGBA(240, 228, 66),
        packRGBA(213, 94, 0),   packRGBA(204, 121, 167), packRGBA(120, 200, 255), packRGBA(255, 120, 120),
        packRGBA(170, 230, 120), packRGBA(255, 200, 140),
    };
    return colors[((i % 10) + 10) % 10];
}

const std::vector<SceneInfo>& sceneList() {
    static const std::vector<SceneInfo> scenes = {
        {"Ball pit", PhysicsModel::Rigid,
         "Balls of different sizes falling into a pit.\nTry the Grab tool, or Edit mode to add things.", 2000, 100, 30000},
        {"Elastic gas (energy check)", PhysicsModel::Rigid,
         "The original simulator: no gravity, restitution 1.\nTotal energy (graph) should stay flat.", 1500, 100, 30000},
        {"Mixed sizes", PhysicsModel::Rigid,
         "A few boulders among many pebbles. The grid cell must\nfit the biggest ball: compare broad phases.", 4000, 500, 40000},
        {"Broad-phase stress test", PhysicsModel::Rigid,
         "Many tiny particles. Switch broad phase and thread\ncount in Performance and watch the step time.", 30000, 1000, 150000},
        {"Ropes & cloth", PhysicsModel::Rigid,
         "Distance constraints. Grab and drag the cloth; turn on\n'Tear links' in Physics and pull hard.", 0, 0, 0},
        {"Soft bodies", PhysicsModel::Rigid,
         "Pressurised rings (area constraint + edge links)\nbouncing over static ramps.", 0, 0, 0},
        {"Boxes & ramps", PhysicsModel::Rigid,
         "Convex polygons (SAT + clipping, sequential impulses)\nstacked, sliding down ramps, hit by balls.", 0, 0, 0},
        {"SPH dam break", PhysicsModel::SPH,
         "A block of fluid collapses and sloshes over a low wall;\nlight boxes float on it. Colour by density in View.", 3000, 500, 12000},
        {"Lennard-Jones gas / liquid", PhysicsModel::LennardJones,
         "Molecules with a Lennard-Jones potential. Lower the\ntemperature below ~0.45 to condense droplets.", 1200, 100, 6000},
        {"Lennard-Jones crystal", PhysicsModel::LennardJones,
         "Cold molecules under gravity settle into a\nhexagonal crystal. Heat it up to melt it.", 900, 100, 4000},
        {"Galaxy (Barnes-Hut)", PhysicsModel::NBody,
         "A rotating disc of stars around a heavy core.\nToggle Barnes-Hut vs direct O(n^2) in Physics.", 4000, 200, 40000},
        {"Galaxy collision", PhysicsModel::NBody,
         "Two discs on a collision course. Tidal tails form\nas they pass through each other.", 3000, 200, 40000},
    };
    return scenes;
}

void loadScene(World& w, int scene, int count, unsigned seed) {
    const SceneInfo& info = sceneList()[std::clamp(scene, 0, (int)sceneList().size() - 1)];
    BroadphaseKind bp = w.params.broadphase;
    w.clear();
    w.params = Params{};
    w.params.broadphase = bp;
    w.params.model = info.model;
    if (info.defaultCount > 0) count = std::clamp(count > 0 ? count : info.defaultCount, info.minCount, info.maxCount);
    Rng rng(seed);
    const AABB B = w.bounds;
    const float W = B.width(), H = B.height();
    Params& P = w.params;

    switch ((SceneId)scene) {
        case BallPit: {
            P.restitution = 0.3f;
            P.friction = 0.2f;
            float r = std::clamp(std::sqrt(W * H * 0.45f / (count * kPi)), 2.0f, 9.0f);
            fillLattice(w, rng, {B.minX, B.minY, B.maxX, B.maxY}, count, 2.3f * r, 0.6f * r, r, 60, 0);
            break;
        }
        case ElasticGas: {
            P.gravity = {0, 0};
            P.restitution = 1;
            P.friction = 0;
            float r = std::clamp(std::sqrt(W * H * 0.12f / (count * kPi)), 1.5f, 10.0f);
            for (int i = 0; i < count; i++)
                w.addParticle({rng.uniform(B.minX + r, B.maxX - r), rng.uniform(B.minY + r, B.maxY - r)},
                              {rng.uniform(-300, 300), rng.uniform(-300, 300)}, r, paletteColor(rng.integer(0, 9)));
            break;
        }
        case MixedSizes: {
            P.restitution = 0.2f;
            P.friction = 0.2f;
            for (int i = 0; i < 12; i++)
                w.addParticle({B.minX + W * (i + 0.5f) / 12, B.minY + 60}, {0, 0}, rng.uniform(32, 50),
                              packRGBA(200, 200, 210));
            float r = std::clamp(std::sqrt(W * H * 0.3f / (count * kPi)), 1.5f, 6.0f);
            fillLattice(w, rng, {B.minX, B.minY + 140, B.maxX, B.maxY}, count, 2.2f * r, 0.6f * r, r, 30, 0);
            break;
        }
        case StressTest: {
            P.gravity = {0, 0};
            P.restitution = 1;
            P.friction = 0;
            P.substeps = 2;
            float r = std::clamp(std::sqrt(W * H * 0.15f / (count * kPi)), 0.8f, 6.0f);
            for (int i = 0; i < count; i++)
                w.addParticle({rng.uniform(B.minX + r, B.maxX - r), rng.uniform(B.minY + r, B.maxY - r)},
                              {rng.uniform(-150, 150), rng.uniform(-150, 150)}, r, paletteColor(rng.integer(0, 9)));
            break;
        }
        case RopesCloth: {
            P.integrator = Integrator::Verlet;
            P.restitution = 0.2f;
            P.friction = 0.3f;
            P.drag = 0.2f;
            P.constraintIterations = 6;
            float spacing = std::max(9.0f, std::min(W * 0.5f / 40, 14.0f));
            addCloth(w, {B.maxX - 40 * spacing - W * 0.06f, B.minY + 40}, 40, 26, spacing, 1);
            for (int i = 0; i < 4; i++) {
                float x = B.minX + W * (0.04f + 0.08f * i);
                buildRope(w, {x, B.minY + 30}, {x + 90, B.minY + 30 + 50.0f * i}, 4, 2 + i, true, 14);
            }
            // a pile of balls under the cloth to throw around
            fillLattice(w, rng, {B.minX + W * 0.45f, B.maxY - 90, B.maxX - 10, B.maxY}, 200, 19, 6, 9, 0, 0);
            break;
        }
        case SoftBodies: {
            P.integrator = Integrator::Verlet;
            P.restitution = 0.3f;
            P.friction = 0.3f;
            P.constraintIterations = 6;
            w.bodies.push_back(makeWallBody({B.minX + W * 0.05f, B.minY + H * 0.45f}, {B.minX + W * 0.45f, B.minY + H * 0.62f}, 14,
                                            packRGBA(110, 110, 125)));
            w.bodies.push_back(makeWallBody({B.maxX - W * 0.05f, B.minY + H * 0.70f}, {B.minX + W * 0.55f, B.minY + H * 0.82f}, 14,
                                            packRGBA(110, 110, 125)));
            for (int b = 0; b < 7; b++)
                buildSoftBody(w, {B.minX + W * (0.1f + 0.12f * b), B.minY + 90 + 50.0f * (b % 2)}, rng.uniform(40, 65),
                              paletteColor(b), 10 + b, {rng.uniform(-80, 80), 0});
            for (int i = 0; i < 120; i++)
                w.addParticle({rng.uniform(B.minX + 20, B.maxX - 20), rng.uniform(B.minY + 200, B.minY + 300)}, {0, 0},
                              rng.uniform(5, 9), paletteColor(rng.integer(0, 9)));
            break;
        }
        case BoxesRamps: {
            P.restitution = 0.1f;
            P.friction = 0.5f;
            uint32_t wall = packRGBA(110, 110, 125);
            w.bodies.push_back(makeWallBody({B.minX + 20, B.minY + H * 0.30f}, {B.minX + W * 0.38f, B.minY + H * 0.50f}, 14, wall));
            w.bodies.push_back(makeWallBody({B.minX + W * 0.30f, B.minY + H * 0.78f}, {B.minX + W * 0.05f, B.minY + H * 0.68f}, 14, wall));
            // pyramid on the floor
            float s = 36;
            for (int row = 0; row < 6; row++)
                for (int k = 0; k <= 5 - row; k++) {
                    float x = B.minX + W * 0.62f + (k - (5 - row) * 0.5f) * (s + 1);
                    float y = B.maxY - s * 0.5f - row * (s + 0.5f);
                    w.bodies.push_back(makeBoxBody({x, y}, s, s, 0, kDefaultDensity, paletteColor(row + 2)));
                }
            for (int i = 0; i < 8; i++)
                w.bodies.push_back(makeRegularPolygonBody({B.minX + 60 + i * 45.0f, B.minY + 60}, rng.uniform(14, 22), 3 + i % 5,
                                                          rng.uniform(0, kPi), kDefaultDensity, paletteColor(i)));
            for (int i = 0; i < 150; i++)
                w.addParticle({rng.uniform(B.minX + W * 0.4f, B.maxX - 20), rng.uniform(B.minY + 20, B.minY + 160)}, {0, 0},
                              rng.uniform(4, 8), paletteColor(rng.integer(0, 9)));
            break;
        }
        case DamBreak: {
            P.restitution = 0.05f;
            P.friction = 0.0f;
            float spacing = 0.3f * P.sphRadius;
            int cols = std::max(4, (int)(W * 0.35f / spacing));
            for (int i = 0; i < count; i++) {
                float x = B.minX + 6 + (i % cols) * spacing + rng.uniform(-0.5f, 0.5f);
                float y = B.maxY - 6 - (i / cols) * spacing;
                if (y < B.minY + 10) break;
                w.addParticle({x, y}, {0, 0}, 3.5f, packRGBA(60, 140, 230));
            }
            // the dam is a static wall that the user can erase, plus two floating boxes
            w.bodies.push_back(makeBoxBody({B.minX + W * 0.68f, B.maxY - 120}, 80, 50, 0.2f, kDefaultDensity * 0.35f, packRGBA(200, 150, 90)));
            w.bodies.push_back(makeBoxBody({B.minX + W * 0.85f, B.maxY - 140}, 60, 60, -0.3f, kDefaultDensity * 0.35f, packRGBA(220, 120, 90)));
            w.bodies.push_back(makeWallBody({B.minX + W * 0.55f, B.maxY}, {B.minX + W * 0.55f, B.maxY - 70}, 16, packRGBA(110, 110, 125)));
            break;
        }
        case LJGas:
        case LJCrystal: {
            bool crystal = scene == LJCrystal;
            P.gravity = crystal ? Vec2{0, 300} : Vec2{0, 0};
            P.substeps = 32;  // close collisions are stiff: fewer substeps visibly break energy conservation
            P.integrator = Integrator::Verlet;
            P.targetTemperature = crystal ? 0.12f : 1.0f;
            P.restitution = 1;
            P.friction = 0;
            float spacing = crystal ? 10.0f : std::max(10.0f, std::sqrt(W * H / count));
            int cols = std::max(1, (int)((crystal ? W * 0.6f : W) / spacing) - 1);
            float x0 = crystal ? B.minX + W * 0.2f : B.minX + spacing;
            float vT = std::sqrt(P.targetTemperature * P.ljEpsilon);  // per-axis thermal speed for unit mass
            for (int i = 0; i < count; i++) {
                float x = x0 + (i % cols) * spacing + ((i / cols) % 2) * spacing * 0.5f;
                float y = (crystal ? B.maxY - 40 : B.minY + spacing) + (crystal ? -1.0f : 1.0f) * (i / cols) * spacing * 0.87f;
                if (y < B.minY + 5 || y > B.maxY - 5) break;
                w.addParticle({x, y}, {rng.normal(vT), rng.normal(vT)}, 4, paletteColor(6), 0, 1.0f);
            }
            break;
        }
        case Galaxy:
        case GalaxyCollision: {
            P.gravity = {0, 0};
            P.walls = false;
            P.integrator = Integrator::Verlet;
            // close passes between the heavy cores are fast: with too few substeps or
            // too little softening energy is visibly not conserved (collision, max
            // |dE| over 15 s: 2 substeps +1700%, 8 substeps/softening 12 18%,
            // 8 substeps/softening 20 2%)
            P.substeps = scene == Galaxy ? 4 : 8;
            P.G = 1500;
            P.softening = scene == Galaxy ? 8.0f : 20.0f;
            Vec2 c{B.minX + W * 0.5f, B.minY + H * 0.5f};
            float R = std::min(W, H) * 0.42f;
            if (scene == Galaxy) {
                addGalaxy(w, rng, c, {0, 0}, count, R, 9000, 0, false);
            } else {
                addGalaxy(w, rng, c + Vec2{-W * 0.25f, -H * 0.12f}, {25, 8}, count / 2, R * 0.55f, 6000, 0, false);
                addGalaxy(w, rng, c + Vec2{W * 0.25f, H * 0.12f}, {-25, -8}, count - count / 2, R * 0.55f, 6000, 5, true);
            }
            break;
        }
    }
    w.invalidate();
    w.computeDiagnostics();
}
