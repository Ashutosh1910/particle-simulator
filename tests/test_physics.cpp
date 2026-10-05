#include <cmath>
#include <random>

#include "../src/physics/world.h"
#include "test_framework.h"

namespace {

bool allFinite(const World& w) {
    for (int i = 0; i < w.p.size(); i++)
        if (!std::isfinite(w.p.x[i]) || !std::isfinite(w.p.y[i]) || !std::isfinite(w.p.vx[i]) ||
            !std::isfinite(w.p.vy[i]))
            return false;
    for (const Body& b : w.bodies)
        if (!std::isfinite(b.pos.x) || !std::isfinite(b.pos.y) || !std::isfinite(b.angle)) return false;
    return true;
}

void elasticGas(World& w, int n, unsigned seed, bool mixed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(0, 1);
    w.clear();
    w.bounds = {0, 0, 1000, 800};
    w.params = Params{};
    w.params.gravity = {0, 0};
    w.params.restitution = 1;
    w.params.friction = 0;
    for (int i = 0; i < n; i++) {
        float r = mixed ? 3 + 9 * u(rng) : 5;
        w.addParticle({20 + 960 * u(rng), 20 + 760 * u(rng)}, {-300 + 600 * u(rng), -300 + 600 * u(rng)}, r, 0);
    }
}

double totalEnergy(World& w) {
    w.computeDiagnostics();
    return w.diagnostics().kinetic + w.diagnostics().potential;
}

}  // namespace

TEST(elastic_collisions_conserve_kinetic_energy) {
    for (Integrator integ : {Integrator::SymplecticEuler, Integrator::Verlet}) {
        World w;
        elasticGas(w, 800, 1, true);
        w.params.integrator = integ;
        double e0 = totalEnergy(w);
        for (int s = 0; s < 300; s++) w.step();
        double e1 = totalEnergy(w);
        double rel = std::fabs(e1 - e0) / e0;
        std::printf("    integrator %d: energy change %.3f%%\n", (int)integ, (e1 - e0) / e0 * 100);
        CHECK_MSG(rel < 0.02, "integrator %d: energy %.1f -> %.1f (%.3f%%)", (int)integ, e0, e1, rel * 100);
        CHECK(allFinite(w));
    }
}

TEST(momentum_conserved_without_walls) {
    World w;
    elasticGas(w, 600, 2, true);
    w.params.walls = false;
    w.computeDiagnostics();
    Vec2 m0 = w.diagnostics().momentum;
    for (int s = 0; s < 200; s++) w.step();
    Vec2 m1 = w.diagnostics().momentum;
    float scale = 0;
    for (int i = 0; i < w.p.size(); i++) scale += w.p.mass(i) * 300;
    CHECK_MSG(length(m1 - m0) < 1e-3f * scale, "momentum drift %.3f (scale %.1f)", length(m1 - m0), scale);
}

TEST(results_identical_for_any_thread_count) {
    for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
        World a, b;
        elasticGas(a, 1500, 3, true);
        elasticGas(b, 1500, 3, true);
        a.params.gravity = b.params.gravity = {0, 500};
        a.params.restitution = b.params.restitution = 0.6f;
        a.params.broadphase = b.params.broadphase = (BroadphaseKind)k;
        a.setThreads(1);
        b.setThreads(4);
        for (int s = 0; s < 60; s++) {
            a.step();
            b.step();
        }
        bool same = true;
        for (int i = 0; i < a.p.size(); i++) same &= a.p.x[i] == b.p.x[i] && a.p.y[i] == b.p.y[i];
        CHECK_MSG(same, "%s differs between 1 and 4 threads", broadphaseName((BroadphaseKind)k));
    }
}

TEST(barnes_hut_matches_direct_sum) {
    World w;
    w.params.model = PhysicsModel::NBody;
    w.params.walls = false;
    std::mt19937 rng(4);
    std::normal_distribution<float> g(0, 150);
    for (int i = 0; i < 1500; i++) w.addParticle({500 + g(rng), 400 + g(rng)}, {0, 0}, 2, 0, 0, 1 + (i % 5));
    std::vector<float> dax, day, bax, bay;
    double dpe, bpe;
    long long dn, bn;
    w.computeGravity(false, dax, day, dpe, dn);
    w.params.theta = 0;
    w.computeGravity(true, bax, bay, bpe, bn);
    double maxErr = 0;
    for (int i = 0; i < w.p.size(); i++) {
        double ref = std::hypot(dax[i], day[i]) + 1e-6;
        maxErr = std::max(maxErr, std::hypot(dax[i] - bax[i], day[i] - bay[i]) / ref);
    }
    CHECK_MSG(maxErr < 1e-3, "theta=0 max relative error %g", maxErr);
    CHECK_MSG(std::fabs(dpe - bpe) < 1e-3 * std::fabs(dpe), "potential %g vs %g", dpe, bpe);

    w.params.theta = 0.5f;
    w.computeGravity(true, bax, bay, bpe, bn);
    double meanErr = 0;
    for (int i = 0; i < w.p.size(); i++)
        meanErr += std::hypot(dax[i] - bax[i], day[i] - bay[i]) / (std::hypot(dax[i], day[i]) + 1e-6);
    meanErr /= w.p.size();
    CHECK_MSG(meanErr < 0.02, "theta=0.5 mean relative error %g", meanErr);
    CHECK_MSG(bn < dn / 3, "Barnes-Hut did %lld interactions vs direct %lld", bn, dn);
}

TEST(velocity_verlet_conserves_lennard_jones_energy) {
    double drift[2];
    for (int mode = 0; mode < 2; mode++) {
        World w;
        w.params.model = PhysicsModel::LennardJones;
        w.params.integrator = mode == 0 ? Integrator::Verlet : Integrator::SymplecticEuler;
        w.params.thermostat = false;
        w.params.gravity = {0, 0};
        w.params.substeps = 32;
        w.bounds = {0, 0, 600, 600};
        std::mt19937 rng(5);
        std::normal_distribution<float> v(0, 150);
        for (int y = 0; y < 15; y++)
            for (int x = 0; x < 15; x++) w.addParticle({100 + x * 12.0f, 100 + y * 12.0f}, {v(rng), v(rng)}, 4, 0, 0, 1);
        w.step();
        double e0 = totalEnergy(w);
        // largest deviation over the run: the end point alone depends on the seed
        double worst = 0;
        for (int s = 0; s < 300; s++) {
            w.step();
            worst = std::max(worst, std::fabs(totalEnergy(w) - e0) / std::fabs(e0));
        }
        drift[mode] = worst;
        CHECK(allFinite(w));
    }
    CHECK_MSG(drift[0] < 0.005, "velocity Verlet max energy error %.3f%%", drift[0] * 100);
    CHECK(drift[0] * 3 < drift[1]);
    std::printf("    LJ max energy error: velocity Verlet %.3f%%, symplectic Euler %.3f%%\n", drift[0] * 100, drift[1] * 100);
}

TEST(nbody_orbit_energy_verlet_beats_euler) {
    double drift[2];
    for (int mode = 0; mode < 2; mode++) {
        World w;
        w.params.model = PhysicsModel::NBody;
        w.params.walls = false;
        w.params.gravity = {0, 0};
        w.params.integrator = mode == 0 ? Integrator::Verlet : Integrator::SymplecticEuler;
        w.params.substeps = 2;
        w.params.softening = 1;
        w.addParticle({500, 400}, {0, 0}, 10, 0, 0, 1000);
        float v = std::sqrt(w.params.G * 1000 / 150.0f);
        w.addParticle({650, 400}, {0, v * 1.2f}, 2, 0, 0, 0.01f);
        w.step();
        double e0 = totalEnergy(w);
        for (int s = 0; s < 600; s++) w.step();
        drift[mode] = std::fabs(totalEnergy(w) - e0) / std::fabs(e0);
    }
    std::printf("    orbit energy drift: velocity Verlet %.4f%%, symplectic Euler %.4f%%\n", drift[0] * 100, drift[1] * 100);
    CHECK(drift[0] < 0.01);
}

TEST(polygon_collision_geometry) {
    Body floor = makeBoxBody({0, 0}, 200, 20, 0, 0, 0);  // spans y in [-10, 10]
    Body box = makeBoxBody({0, -19}, 20, 20, 0, kDefaultDensity, 0);  // bottom at y = -9: 1px overlap
    Vec2 n;
    ContactPoint cps[2];
    int count = collidePolygons(box, floor, n, cps);
    CHECK(count == 2);
    CHECK(std::fabs(n.x) < 1e-5f && std::fabs(n.y - 1) < 1e-5f);  // from box down into the floor
    for (int i = 0; i < count; i++) CHECK(std::fabs(cps[i].depth - 1) < 1e-3f);
    box = makeBoxBody({0, -30}, 20, 20, 0, kDefaultDensity, 0);
    CHECK(collidePolygons(box, floor, n, cps) == 0);

    ContactPoint cp;
    CHECK(collidePolygonCircle(floor, {0, 14}, 5, n, cp));  // circle below the floor, 1px overlap
    CHECK(std::fabs(n.y - 1) < 1e-5f && std::fabs(cp.depth - 1) < 1e-3f);
    CHECK(collidePolygonCircle(floor, {103, 13}, 5, n, cp));  // near the corner (100, 10)
    CHECK(n.x > 0 && n.y > 0);
    CHECK(!collidePolygonCircle(floor, {110, 20}, 5, n, cp));
    CHECK(pointInBody(floor, {50, 5}) && !pointInBody(floor, {50, 15}));
}

TEST(box_stack_is_stable) {
    World w;
    w.bounds = {0, 0, 800, 600};
    w.params.restitution = 0.1f;
    w.params.friction = 0.6f;
    for (int i = 0; i < 4; i++) w.bodies.push_back(makeBoxBody({400, 600 - 20 - 40.5f * i}, 40, 40, 0, kDefaultDensity, 0));
    for (int s = 0; s < 300; s++) w.step();
    CHECK(allFinite(w));
    for (int i = 0; i < 4; i++) {
        const Body& b = w.bodies[i];
        CHECK_MSG(std::fabs(b.pos.x - 400) < 3 && std::fabs(b.pos.y - (600 - 20 - 40 * i)) < 3 && std::fabs(b.angle) < 0.05f,
                  "box %d at (%.1f, %.1f) angle %.3f", i, b.pos.x, b.pos.y, b.angle);
    }
}

TEST(balls_rest_on_static_obstacle_and_floor) {
    World w;
    w.bounds = {0, 0, 800, 600};
    w.bodies.push_back(makeWallBody({100, 300}, {500, 400}, 12, 0));  // slanted static ramp
    for (int i = 0; i < 200; i++) w.addParticle({120.0f + (i % 20) * 16, 50.0f + (i / 20) * 16}, {0, 0}, 6, 0);
    for (int s = 0; s < 400; s++) w.step();
    CHECK(allFinite(w));
    // nobody may be inside the ramp
    const Body& ramp = w.bodies[0];
    int inside = 0;
    for (int i = 0; i < w.p.size(); i++) inside += pointInBody(ramp, {w.p.x[i], w.p.y[i]});
    CHECK_MSG(inside == 0, "%d balls inside the ramp", inside);
    for (int i = 0; i < w.p.size(); i++) CHECK(w.p.y[i] <= 600 - 6 + 0.01f);
}

TEST(rope_and_cloth_hold_their_shape) {
    World w;
    w.bounds = {0, 0, 1000, 800};
    w.params.constraintIterations = 8;
    w.params.drag = 1.5f;  // let the swinging rope settle
    int prev = -1;
    for (int i = 0; i < 30; i++) {
        int idx = w.addParticle({500 + i * 10.0f, 100}, {0, 0}, 4, 0, 1, i == 0 ? 0 : -1);
        if (i == 0) w.p.invMass[idx] = 0;
        if (prev >= 0) w.links.push_back({prev, idx, 10, 1});
        prev = idx;
    }
    for (int s = 0; s < 300; s++) w.step();
    float worst = 0;
    for (const Link& l : w.links) {
        float d = std::hypot(w.p.x[l.b] - w.p.x[l.a], w.p.y[l.b] - w.p.y[l.a]);
        worst = std::max(worst, std::fabs(d - l.rest) / l.rest);
    }
    CHECK_MSG(worst < 0.15f, "worst stretch %.1f%%", worst * 100);
    CHECK(w.p.x[0] == 500 && w.p.y[0] == 100);  // pinned end stays put
    CHECK(w.p.y[29] > 300);  // the rope fell and hangs down
}

TEST(soft_body_keeps_area) {
    for (Integrator integ : {Integrator::SymplecticEuler, Integrator::Verlet}) {
        World w;
        w.bounds = {0, 0, 800, 600};
        w.params.integrator = integ;
        SoftBody sb;
        int n = 24;
        float R = 60;
        for (int i = 0; i < n; i++) {
            float a = 2 * 3.14159265f * i / n;
            sb.ring.push_back(w.addParticle({400 + R * std::cos(a), 300 + R * std::sin(a)}, {0, 0}, 7, 0, 2));
        }
        for (int i = 0; i < n; i++) {
            int a = sb.ring[i], b = sb.ring[(i + 1) % n];
            w.links.push_back({a, b, std::hypot(w.p.x[b] - w.p.x[a], w.p.y[b] - w.p.y[a]), 1});
        }
        sb.restArea = 0;
        for (int i = 0; i < n; i++) {
            int a = sb.ring[i], b = sb.ring[(i + 1) % n];
            sb.restArea += 0.5f * (w.p.x[a] * w.p.y[b] - w.p.x[b] * w.p.y[a]);
        }
        sb.stiffness = 0.5f;
        w.softBodies.push_back(sb);
        for (int s = 0; s < 300; s++) w.step();
        float area = 0;
        for (int i = 0; i < n; i++) {
            int a = sb.ring[i], b = sb.ring[(i + 1) % n];
            area += 0.5f * (w.p.x[a] * w.p.y[b] - w.p.x[b] * w.p.y[a]);
        }
        CHECK_MSG(std::fabs(area / sb.restArea - 1) < 0.25f, "integrator %d area ratio %.2f", (int)integ, area / sb.restArea);
        CHECK(allFinite(w));
    }
}

TEST(sph_dam_break_stays_stable) {
    World w;
    w.params.model = PhysicsModel::SPH;
    w.params.restitution = 0.1f;
    w.params.friction = 0;
    w.bounds = {0, 0, 800, 600};
    float spacing = 0.3f * w.params.sphRadius;
    for (int y = 0; y < 30; y++)
        for (int x = 0; x < 30; x++) w.addParticle({20 + x * spacing, 600 - 20 - y * spacing}, {0, 0}, 4, 0);
    w.bodies.push_back(makeBoxBody({600, 300}, 60, 40, 0, kDefaultDensity * 0.5f, 0));
    float maxDensity = 0;
    for (int s = 0; s < 400; s++) {
        w.step();
        for (float d : w.sphDensity()) maxDensity = std::max(maxDensity, d);
    }
    CHECK(allFinite(w));
    for (int i = 0; i < w.p.size(); i++) CHECK(w.bounds.contains({w.p.x[i], w.p.y[i]}));
    CHECK_MSG(maxDensity < 6 * w.params.sphRestDensity, "max density %.1f", maxDensity);
    w.computeDiagnostics();
    std::printf("    SPH after 400 steps: max density %.1f (rest %.1f), max speed %.0f px/s\n", maxDensity,
                w.params.sphRestDensity, w.diagnostics().maxSpeed);
    // the fluid must have spread out over the floor
    float right = 0;
    for (int i = 0; i < w.p.size(); i++) right = std::max(right, w.p.x[i]);
    CHECK(right > 500);
}

TEST(thermostat_reaches_target_temperature) {
    World w;
    w.params.model = PhysicsModel::LennardJones;
    w.params.gravity = {0, 0};
    w.params.targetTemperature = 1.5f;
    w.params.substeps = 16;
    w.bounds = {0, 0, 600, 600};
    std::mt19937 rng(6);
    std::normal_distribution<float> v(0, 30);
    for (int y = 0; y < 20; y++)
        for (int x = 0; x < 20; x++) w.addParticle({60 + x * 22.0f, 60 + y * 22.0f}, {v(rng), v(rng)}, 4, 0, 0, 1);
    for (int s = 0; s < 240; s++) w.step();
    double T = w.diagnostics().temperature;
    CHECK_MSG(std::fabs(T - 1.5) < 0.2, "temperature %.2f", T);
}

TEST(removing_particles_keeps_links_valid) {
    World w;
    for (int i = 0; i < 10; i++) w.addParticle({i * 10.0f, 0}, {0, 0}, 3, 0);
    for (int i = 0; i + 1 < 10; i++) w.links.push_back({i, i + 1, 10, 1});
    w.grabParticle(9, {90, 0});
    w.removeParticle(3);  // particle 9 moves into slot 3
    CHECK(w.p.size() == 9);
    CHECK(w.links.size() == 7);
    for (const Link& l : w.links) CHECK(l.a >= 0 && l.a < 9 && l.b >= 0 && l.b < 9 && l.a != l.b);
    CHECK(w.grabbedParticle() == 3);
    CHECK(w.p.x[3] == 90);
    w.releaseGrab();
    CHECK(w.p.invMass[3] > 0);
}

TEST(drag_decays_velocity_at_the_same_rate_for_both_integrators) {
    for (Integrator integ : {Integrator::SymplecticEuler, Integrator::Verlet}) {
        World w;
        w.params.model = PhysicsModel::LennardJones;
        w.params.integrator = integ;
        w.params.thermostat = false;
        w.params.gravity = {0, 0};
        w.params.drag = 1;
        w.params.walls = false;
        w.addParticle({0, 0}, {100, 0}, 4, 0, 0, 1);
        for (int s = 0; s < 60; s++) w.step();  // 1 second
        CHECK_MSG(std::fabs(w.p.vx[0] - 100 * std::exp(-1.0f)) < 1.0f, "integrator %d: v = %.2f, expected %.2f",
                  (int)integ, w.p.vx[0], 100 * std::exp(-1.0f));
    }
}

TEST(grabbed_particle_keeps_its_gravity) {
    World w;
    w.params.model = PhysicsModel::NBody;
    w.params.walls = false;
    w.addParticle({0, 0}, {0, 0}, 9, 0, 0, 9000);
    w.addParticle({100, 0}, {0, 0}, 2, 0, 0, 1);
    std::vector<float> ax, ay;
    double pe;
    long long n;
    w.computeGravity(true, ax, ay, pe, n);
    float before = ax[1];
    w.grabParticle(0, {0, 0});
    w.computeGravity(true, ax, ay, pe, n);
    CHECK_MSG(std::fabs(ax[1] - before) < 1e-3f * std::fabs(before), "acceleration %.1f -> %.1f", before, ax[1]);
    CHECK(w.p.x[0] == 0 && w.p.y[0] == 0);  // grabbing doesn't move the particle
}

// theta = 1 is the largest opening angle the UI allows. Plain monopole Barnes-Hut is
// ~10% off there; letting a particle's own cell be approximated made it much worse.
TEST(barnes_hut_error_bounded_at_max_theta) {
    World w;
    w.params.model = PhysicsModel::NBody;
    w.params.theta = 1.0f;
    std::mt19937 rng(9);
    std::normal_distribution<float> g(0, 150);
    for (int i = 0; i < 1000; i++) w.addParticle({g(rng), g(rng)}, {0, 0}, 2, 0, 0, 1);
    std::vector<float> dax, day, bax, bay;
    double pe;
    long long n;
    w.computeGravity(false, dax, day, pe, n);
    w.computeGravity(true, bax, bay, pe, n);
    double meanErr = 0;
    for (int i = 0; i < w.p.size(); i++)
        meanErr += std::hypot(dax[i] - bax[i], day[i] - bay[i]) / (std::hypot(dax[i], day[i]) + 1e-6);
    meanErr /= w.p.size();
    CHECK_MSG(meanErr < 0.15, "theta=1 mean relative error %g", meanErr);
}

TEST(every_model_is_identical_for_any_thread_count) {
    for (PhysicsModel model : {PhysicsModel::SPH, PhysicsModel::LennardJones, PhysicsModel::NBody}) {
        World a, b;
        for (World* w : {&a, &b}) {
            w->params.model = model;
            w->bounds = {0, 0, 800, 600};
            std::mt19937 rng(21);
            std::uniform_real_distribution<float> u(0, 1);
            for (int i = 0; i < 1500; i++)
                w->addParticle({50 + 700 * u(rng), 50 + 500 * u(rng)}, {u(rng) * 50 - 25, u(rng) * 50 - 25}, 3.5f, 0, 0, 1);
        }
        a.setThreads(1);
        b.setThreads(4);
        for (int s = 0; s < 20; s++) {
            a.step();
            b.step();
        }
        bool same = true;
        for (int i = 0; i < a.p.size(); i++) same &= a.p.x[i] == b.p.x[i] && a.p.y[i] == b.p.y[i];
        CHECK_MSG(same, "%s differs between 1 and 4 threads", modelName(model));
    }
}
