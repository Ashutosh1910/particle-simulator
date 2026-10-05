#include "world.h"

#include <chrono>
#include <cmath>

namespace {
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
}  // namespace

const char* modelName(PhysicsModel m) {
    switch (m) {
        case PhysicsModel::Rigid: return "Rigid bodies";
        case PhysicsModel::SPH: return "SPH fluid";
        case PhysicsModel::LennardJones: return "Lennard-Jones molecules";
        case PhysicsModel::NBody: return "N-body gravity";
        default: return "?";
    }
}

World::World() : pool_(1) { syncBroadphase(); }
World::~World() = default;

void World::clear() {
    p.clear();
    links.clear();
    softBodies.clear();
    bodies.clear();
    grab_ = Grab{};
    mouse = MouseForce{};
    pairs_.clear();
    density_.clear();
    bh_.clear();
    time_ = 0;
    steps_ = 0;
    forcesValid_ = false;
    forcePotential_ = 0;
    stats_ = StepStats{};
    diag_ = Diagnostics{};
}

void World::setThreads(int threads) { pool_.setThreadCount(threads); }

void World::syncBroadphase() {
    if (broadphase_ && broadphaseKind_ == params.broadphase) return;
    broadphaseKind_ = params.broadphase;
    broadphase_ = makeBroadphase(params.broadphase);
}

float World::particleExtent(int i) const {
    float r = p.radius[i];
    switch (params.model) {
        case PhysicsModel::SPH: return std::max(r, 0.5f * params.sphRadius);
        // sigma_i = 2 r / 2^(1/6); pair cutoff = cutoff * (sigma_i + sigma_j) / 2
        case PhysicsModel::LennardJones: return std::max(r, params.ljCutoff * r / 1.122462f);
        default: return r;
    }
}

void World::rebuildBroadphase() {
    syncBroadphase();
    int n = p.size();
    extent_.resize(n);
    for (int i = 0; i < n; i++) extent_[i] = particleExtent(i);
    broadphase_->build({p.x.data(), p.y.data(), extent_.data(), n}, pool_);
}

void World::findCandidatePairs() {
    auto t0 = Clock::now();
    rebuildBroadphase();
    pairs_.clear();
    broadphase_->findPairs(pairs_, pool_);
    stats_.tests = broadphase_->stats().tests;
    stats_.candidates = broadphase_->stats().pairs;
    stats_.broadMs += msSince(t0);
}

// ---------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------

void World::step() {
    auto t0 = Clock::now();
    syncBroadphase();
    float frameDt = params.dt * params.timeScale;
    int subs = std::clamp(params.substeps, 1, 64);
    float h = frameDt / subs;
    stats_.broadMs = stats_.solveMs = stats_.rigidMs = 0;
    stats_.contacts = stats_.bodyContacts = 0;
    stats_.interactions = 0;
    if (h > 0) {
        for (int s = 0; s < subs; s++) {
            switch (params.model) {
                case PhysicsModel::Rigid: substepRigid(h, s, subs); break;
                case PhysicsModel::SPH: substepSPH(h, s, subs); break;
                default: substepForces(h, s, subs); break;
            }
        }
        time_ += frameDt;
    }
    grab_.prevTarget = grab_.target;
    steps_++;
    computeDiagnostics();
    stats_.stepMs = msSince(t0);
}

void World::applyExternal(float h) {
    const Vec2 g = params.gravity;
    const float damp = 1.0f / (1.0f + params.drag * h);  // implicit linear drag
    const MouseForce m = mouse;
    pool_.parallelFor(p.size(), [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            if (p.invMass[i] == 0) continue;
            float ax = g.x, ay = g.y;
            if (m.active) {
                float dx = m.pos.x - p.x[i], dy = m.pos.y - p.y[i];
                float d = std::sqrt(dx * dx + dy * dy);
                if (d < m.radius && d > 1) {
                    float s = m.strength * (1 - d / m.radius) / d;
                    ax += dx * s;
                    ay += dy * s;
                }
            }
            p.vx[i] = (p.vx[i] + ax * h) * damp;
            p.vy[i] = (p.vy[i] + ay * h) * damp;
        }
    }, 2048);
}

void World::moveGrabbedParticle(int sub, int subs, float h) {
    if (grab_.kind != GrabKind::Particle) return;
    int i = grab_.index;
    float t = (float)(sub + 1) / subs;
    Vec2 pos = grab_.prevTarget + (grab_.target - grab_.prevTarget) * t;
    Vec2 vel = (grab_.target - grab_.prevTarget) / (h * subs);
    p.x[i] = pos.x; p.y[i] = pos.y;
    p.vx[i] = vel.x; p.vy[i] = vel.y;
}

void World::substepRigid(float h, int sub, int subs) {
    const bool pbd = params.integrator == Integrator::Verlet;
    applyExternal(h);
    pool_.parallelFor(p.size(), [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            p.px[i] = p.x[i];
            p.py[i] = p.y[i];
            if (p.invMass[i] == 0) continue;
            p.x[i] += p.vx[i] * h;
            p.y[i] += p.vy[i] * h;
        }
    }, 2048);
    moveGrabbedParticle(sub, subs, h);

    findCandidatePairs();
    auto t0 = Clock::now();
    solveParticleContacts(h, pbd);
    for (int it = 0; it < params.constraintIterations; it++) {
        solveLinks(h, !pbd);
        solveSoftBodies(h, !pbd);
    }
    if (params.tearRatio > 1) {
        for (size_t k = 0; k < links.size();) {
            const Link& l = links[k];
            float dx = p.x[l.b] - p.x[l.a], dy = p.y[l.b] - p.y[l.a];
            if (dx * dx + dy * dy > l.rest * l.rest * params.tearRatio * params.tearRatio) {
                links[k] = links.back();
                links.pop_back();
            } else {
                k++;
            }
        }
    }
    if (pbd) {
        // position-based Verlet: velocity is whatever the positions did this substep
        pool_.parallelFor(p.size(), [&](int, int b, int e) {
            for (int i = b; i < e; i++) {
                if (p.invMass[i] == 0) continue;
                p.vx[i] = (p.x[i] - p.px[i]) / h;
                p.vy[i] = (p.y[i] - p.py[i]) / h;
            }
        }, 2048);
        // the velocity pass sets each contact's normal speed exactly; a few
        // Gauss-Seidel sweeps let neighbouring contacts agree (one sweep loses ~20%
        // of the energy per 5 s in a dense elastic gas, four lose about 2%)
        for (int pass = 0; pass < 4; pass++) particleContactVelocityPass(h);
    }
    stats_.solveMs += msSince(t0);
    stepBodies(h);
    solveParticleWalls(h);
}

// ---------------------------------------------------------------------------
// particle-particle contacts
// ---------------------------------------------------------------------------
//
// Contacts are solved Gauss-Seidel style (each correction is visible to the
// next), which is inherently sequential. To parallelise, the world is cut into
// vertical stripes at least one interaction range wide. A pair is assigned to
// the stripe of its left-most particle, so it only touches particles in that
// stripe and the next one. All even stripes are then independent of each other
// and run in parallel, followed by all odd stripes. The stripe layout does not
// depend on the thread count, so results are identical for any number of threads.

void World::solveParticleContacts(float h, bool positionBased) {
    int n = p.size();
    if (n == 0) return;
    float maxExtent = 0, xMin = 1e30f, xMax = -1e30f;
    for (int i = 0; i < n; i++) {
        maxExtent = std::max(maxExtent, extent_[i]);
        xMin = std::min(xMin, p.x[i]);
        xMax = std::max(xMax, p.x[i]);
    }
    float width = std::max(2.0f * maxExtent * 1.001f, std::max(bounds.width(), 1.0f) / 32.0f);
    int stripes = std::clamp((int)std::ceil((xMax - xMin) / width) + 1, 1, 4096);
    stripeOf_.resize(n);
    for (int i = 0; i < n; i++) stripeOf_[i] = std::min(stripes - 1, (int)((p.x[i] - xMin) / width));
    stripePairs_.resize(stripes);
    stripeContacts_.resize(stripes);
    stripeContactCount_.assign(stripes, 0);
    for (int s = 0; s < stripes; s++) {
        stripePairs_[s].clear();
        stripeContacts_[s].clear();
    }
    for (int k = 0; k < (int)pairs_.size(); k++) {
        const Pair& pr = pairs_[k];
        stripePairs_[std::min(stripeOf_[pr.a], stripeOf_[pr.b])].push_back(k);
    }

    const float e = params.restitution, mu = params.friction;
    // below this approach speed a contact is treated as resting (no bounce), which
    // stops stacks from jittering under gravity
    const float restSpeed = 2.0f * length(params.gravity) * h + 1e-3f;

    auto solveStripe = [&](int s) {
        int contacts = 0;
        for (int k : stripePairs_[s]) {
            int i = pairs_[k].a, j = pairs_[k].b;
            if (p.group[i] != 0 && p.group[i] == p.group[j]) continue;
            float wi = p.invMass[i], wj = p.invMass[j], w = wi + wj;
            if (w == 0) continue;
            float dx = p.x[j] - p.x[i], dy = p.y[j] - p.y[i];
            float r = p.radius[i] + p.radius[j];
            float d2 = dx * dx + dy * dy;
            if (d2 >= r * r) continue;
            contacts++;
            float d = std::sqrt(d2);
            Vec2 nrm = d > 1e-6f ? Vec2{dx / d, dy / d} : Vec2{1, 0};
            float overlap = r - d;
            p.x[i] -= nrm.x * overlap * wi / w; p.y[i] -= nrm.y * overlap * wi / w;
            p.x[j] += nrm.x * overlap * wj / w; p.y[j] += nrm.y * overlap * wj / w;
            if (positionBased) {
                // Overlap that already existed at the start of the substep is not
                // caused by this substep's motion: move the start positions too, so
                // removing it does not turn into velocity (pre-stabilisation).
                float pdx = p.px[j] - p.px[i], pdy = p.py[j] - p.py[i];
                float pre = std::clamp(r - std::sqrt(pdx * pdx + pdy * pdy), 0.0f, overlap);
                p.px[i] -= nrm.x * pre * wi / w; p.py[i] -= nrm.y * pre * wi / w;
                p.px[j] += nrm.x * pre * wj / w; p.py[j] += nrm.y * pre * wj / w;
            }

            Vec2 vrel{p.vx[j] - p.vx[i], p.vy[j] - p.vy[i]};
            float vn = dot(vrel, nrm);
            if (positionBased) {
                stripeContacts_[s].push_back({i, j, nrm, vn});
                continue;
            }
            if (vn >= 0) continue;
            float bounce = vn > -restSpeed ? 0.0f : e;
            float jn = -(1 + bounce) * vn / w;
            p.vx[i] -= nrm.x * jn * wi; p.vy[i] -= nrm.y * jn * wi;
            p.vx[j] += nrm.x * jn * wj; p.vy[j] += nrm.y * jn * wj;
            if (mu > 0) {
                Vec2 vt = vrel - nrm * vn;
                float vtLen = length(vt);
                if (vtLen > 1e-6f) {
                    Vec2 t = vt / vtLen;
                    float jt = std::min(vtLen / w, mu * jn);  // Coulomb: |jt| <= mu * jn
                    p.vx[i] += t.x * jt * wi; p.vy[i] += t.y * jt * wi;
                    p.vx[j] -= t.x * jt * wj; p.vy[j] -= t.y * jt * wj;
                }
            }
        }
        stripeContactCount_[s] = contacts;
    };
    for (int color = 0; color < 2; color++) {
        int count = (stripes - color + 1) / 2;
        pool_.parallelFor(count, [&](int, int b, int en) {
            for (int k = b; k < en; k++) solveStripe(color + 2 * k);
        }, 1);
    }
    int total = 0;
    for (int c : stripeContactCount_) total += c;
    stats_.contacts = total;
}

void World::particleContactVelocityPass(float h) {
    const float e = params.restitution, mu = params.friction;
    const float restSpeed = 2.0f * length(params.gravity) * h + 1e-3f;
    int stripes = (int)stripeContacts_.size();
    auto solveStripe = [&](int s) {
        for (const PbdContact& c : stripeContacts_[s]) {
            float wi = p.invMass[c.i], wj = p.invMass[c.j], w = wi + wj;
            if (w == 0) continue;
            Vec2 vrel{p.vx[c.j] - p.vx[c.i], p.vy[c.j] - p.vy[c.i]};
            // contacts that were not approaching at the start of the substep only
            // had their old overlap removed (without adding velocity): leave them
            if (c.vnBefore >= 0) continue;
            float vn = dot(vrel, c.n);
            // Mueller et al. "Detailed rigid body simulation with XPBD": restore the
            // pre-solve approach speed times restitution, remove the rest
            float bounce = c.vnBefore < -restSpeed ? -e * c.vnBefore : 0.0f;
            float dvn = -vn + bounce;
            p.vx[c.i] -= c.n.x * dvn * wi / w; p.vy[c.i] -= c.n.y * dvn * wi / w;
            p.vx[c.j] += c.n.x * dvn * wj / w; p.vy[c.j] += c.n.y * dvn * wj / w;
            if (mu > 0) {
                Vec2 vt = vrel - c.n * vn;
                float vtLen = length(vt);
                if (vtLen > 1e-6f) {
                    float dvt = std::min(vtLen, mu * std::fabs(dvn));
                    Vec2 t = vt / vtLen;
                    p.vx[c.i] += t.x * dvt * wi / w; p.vy[c.i] += t.y * dvt * wi / w;
                    p.vx[c.j] -= t.x * dvt * wj / w; p.vy[c.j] -= t.y * dvt * wj / w;
                }
            }
        }
    };
    for (int color = 0; color < 2; color++) {
        int count = (stripes - color + 1) / 2;
        pool_.parallelFor(count, [&](int, int b, int en) {
            for (int k = b; k < en; k++) solveStripe(color + 2 * k);
        }, 1);
    }
}

void World::solveParticleWalls(float h) {
    if (!params.walls) return;
    const bool elastic = params.model == PhysicsModel::LennardJones;  // a thermodynamic box must not lose energy
    const float e = elastic ? 1.0f : params.restitution;
    const float mu = elastic ? 0.0f : params.friction;
    const float restSpeed = 2.0f * length(params.gravity) * h + 1e-3f;
    const AABB B = bounds;
    pool_.parallelFor(p.size(), [&](int, int b, int en) {
        for (int i = b; i < en; i++) {
            if (p.invMass[i] == 0) continue;
            float r = p.radius[i];
            // returns the new normal velocity and applies friction to the tangential one
            auto bounceAxis = [&](float& vn, float& vt) {
                float before = vn;
                vn = std::fabs(before) < restSpeed ? 0.0f : -before * e;
                float dv = std::fabs(vn - before);
                if (mu > 0) vt -= std::copysign(std::min(std::fabs(vt), mu * dv), vt);
            };
            float lo = B.minX + r, hi = B.maxX - r;
            if (lo > hi) lo = hi = 0.5f * (B.minX + B.maxX);
            if (p.x[i] < lo) { p.x[i] = lo; if (p.vx[i] < 0) bounceAxis(p.vx[i], p.vy[i]); }
            else if (p.x[i] > hi) { p.x[i] = hi; if (p.vx[i] > 0) bounceAxis(p.vx[i], p.vy[i]); }
            lo = B.minY + r; hi = B.maxY - r;
            if (lo > hi) lo = hi = 0.5f * (B.minY + B.maxY);
            if (p.y[i] < lo) { p.y[i] = lo; if (p.vy[i] < 0) bounceAxis(p.vy[i], p.vx[i]); }
            else if (p.y[i] > hi) { p.y[i] = hi; if (p.vy[i] > 0) bounceAxis(p.vy[i], p.vx[i]); }
        }
    }, 2048);
}

// ---------------------------------------------------------------------------
// constraints
// ---------------------------------------------------------------------------

void World::solveLinks(float h, bool adjustVelocity) {
    for (const Link& l : links) {
        float wa = p.invMass[l.a], wb = p.invMass[l.b], w = wa + wb;
        if (w == 0) continue;
        float dx = p.x[l.b] - p.x[l.a], dy = p.y[l.b] - p.y[l.a];
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6f) continue;
        float s = l.stiffness * (len - l.rest) / (w * len);
        float ax = dx * s * wa, ay = dy * s * wa, bx = -dx * s * wb, by = -dy * s * wb;
        p.x[l.a] += ax; p.y[l.a] += ay;
        p.x[l.b] += bx; p.y[l.b] += by;
        if (adjustVelocity) {
            // symplectic Euler: carry the correction into the velocity so the
            // constraint removes the motion that violated it
            p.vx[l.a] += ax / h; p.vy[l.a] += ay / h;
            p.vx[l.b] += bx / h; p.vy[l.b] += by / h;
        }
    }
}

void World::solveSoftBodies(float h, bool adjustVelocity) {
    for (const SoftBody& sb : softBodies) {
        int n = (int)sb.ring.size();
        if (n < 3) continue;
        float area = 0;
        for (int k = 0; k < n; k++) {
            int a = sb.ring[k], b = sb.ring[(k + 1) % n];
            area += p.x[a] * p.y[b] - p.x[b] * p.y[a];
        }
        area *= 0.5f;
        float c = area - sb.restArea;
        // gradient of the area with respect to vertex k
        float denom = 0;
        for (int k = 0; k < n; k++) {
            int prev = sb.ring[(k + n - 1) % n], next = sb.ring[(k + 1) % n];
            float gx = 0.5f * (p.y[next] - p.y[prev]), gy = 0.5f * (p.x[prev] - p.x[next]);
            denom += p.invMass[sb.ring[k]] * (gx * gx + gy * gy);
        }
        if (denom < 1e-9f) continue;
        float lambda = -sb.stiffness * c / denom;
        // gradients must use the positions before this pass, so compute them all first
        ringDx_.resize(n);
        ringDy_.resize(n);
        for (int k = 0; k < n; k++) {
            int prev = sb.ring[(k + n - 1) % n], next = sb.ring[(k + 1) % n];
            float w = p.invMass[sb.ring[k]];
            ringDx_[k] = lambda * w * 0.5f * (p.y[next] - p.y[prev]);
            ringDy_[k] = lambda * w * 0.5f * (p.x[prev] - p.x[next]);
        }
        for (int k = 0; k < n; k++) {
            int i = sb.ring[k];
            p.x[i] += ringDx_[k];
            p.y[i] += ringDy_[k];
            if (adjustVelocity) {
                p.vx[i] += ringDx_[k] / h;
                p.vy[i] += ringDy_[k] / h;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// editing
// ---------------------------------------------------------------------------

int World::addParticle(Vec2 pos, Vec2 vel, float radius, uint32_t color, int group, float mass) {
    if (mass < 0) mass = massForRadius(radius);
    invalidate();
    return p.add(pos.x, pos.y, vel.x, vel.y, radius, mass, color, group);
}

void World::removeParticle(int i) {
    if (i < 0 || i >= p.size()) return;
    int last = p.size() - 1;
    if (grab_.kind == GrabKind::Particle) {
        if (grab_.index == i) releaseGrab();
        else if (grab_.index == last) grab_.index = i;
    }
    for (size_t k = 0; k < links.size();) {
        Link& l = links[k];
        if (l.a == i || l.b == i) {
            links[k] = links.back();
            links.pop_back();
            continue;
        }
        if (l.a == last) l.a = i;
        if (l.b == last) l.b = i;
        k++;
    }
    for (size_t k = 0; k < softBodies.size();) {
        auto& ring = softBodies[k].ring;
        if (std::find(ring.begin(), ring.end(), i) != ring.end()) {
            softBodies[k] = softBodies.back();  // a ring with a hole can't hold pressure
            softBodies.pop_back();
            continue;
        }
        for (int& idx : ring)
            if (idx == last) idx = i;
        k++;
    }
    p.swapRemove(i);
    invalidate();
}

void World::removeBody(int i) {
    if (i < 0 || i >= (int)bodies.size()) return;
    int last = (int)bodies.size() - 1;
    if (grab_.kind == GrabKind::Body) {
        if (grab_.index == i) releaseGrab();
        else if (grab_.index == last) grab_.index = i;
    }
    bodies[i] = std::move(bodies[last]);
    bodies.pop_back();
}

int World::pickParticle(Vec2 pos, float maxDistance) const {
    int best = -1;
    float bestD = 1e30f;
    for (int i = 0; i < p.size(); i++) {
        float dx = p.x[i] - pos.x, dy = p.y[i] - pos.y;
        float d = std::sqrt(dx * dx + dy * dy) - p.radius[i];
        if (d < maxDistance && d < bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

int World::pickBody(Vec2 pos) const {
    int found = -1;
    for (int i = 0; i < (int)bodies.size(); i++) {
        if (!pointInBody(bodies[i], pos)) continue;
        if (!bodies[i].isStatic()) return i;  // prefer dynamic bodies over obstacles
        found = i;
    }
    return found;
}

void World::grabParticle(int i, Vec2 target) {
    releaseGrab();
    if (i < 0 || i >= p.size()) return;
    grab_.kind = GrabKind::Particle;
    grab_.index = i;
    grab_.target = grab_.prevTarget = target;
    grab_.savedInvMass = p.invMass[i];
    p.invMass[i] = 0;  // kinematic while held: pushes everything, nothing pushes it
}

void World::grabBody(int i, Vec2 worldPoint) {
    releaseGrab();
    if (i < 0 || i >= (int)bodies.size() || bodies[i].isStatic()) return;
    const Body& b = bodies[i];
    grab_.kind = GrabKind::Body;
    grab_.index = i;
    float c = std::cos(-b.angle), s = std::sin(-b.angle);
    grab_.local = rotate(worldPoint - b.pos, c, s);
    grab_.target = grab_.prevTarget = worldPoint;
}

void World::releaseGrab() {
    if (grab_.kind == GrabKind::Particle && grab_.index >= 0 && grab_.index < p.size())
        p.invMass[grab_.index] = grab_.savedInvMass;
    grab_ = Grab{};
}

Vec2 World::grabAnchorWorld() const {
    if (grab_.kind == GrabKind::Particle) return {p.x[grab_.index], p.y[grab_.index]};
    if (grab_.kind == GrabKind::Body) {
        const Body& b = bodies[grab_.index];
        return b.pos + rotate(grab_.local, std::cos(b.angle), std::sin(b.angle));
    }
    return grab_.target;
}

// ---------------------------------------------------------------------------
// diagnostics
// ---------------------------------------------------------------------------

void World::computeDiagnostics() {
    Diagnostics d;
    double ke = 0, pe = 0, mx = 0, my = 0;
    float maxSpeed2 = 0;
    int moving = 0;
    const Vec2 g = params.gravity;
    for (int i = 0; i < p.size(); i++) {
        if (p.invMass[i] == 0) continue;
        double m = 1.0 / p.invMass[i];
        double v2 = (double)p.vx[i] * p.vx[i] + (double)p.vy[i] * p.vy[i];
        ke += 0.5 * m * v2;
        pe -= m * ((double)g.x * p.x[i] + (double)g.y * p.y[i]);
        mx += m * p.vx[i];
        my += m * p.vy[i];
        maxSpeed2 = std::max(maxSpeed2, (float)v2);
        moving++;
    }
    for (const Body& b : bodies) {
        if (b.isStatic()) continue;
        double m = b.mass();
        ke += 0.5 * m * lengthSq(b.vel) + (b.invInertia > 0 ? 0.5 * b.angVel * b.angVel / b.invInertia : 0.0);
        pe -= m * ((double)g.x * b.pos.x + (double)g.y * b.pos.y);
        mx += m * b.vel.x;
        my += m * b.vel.y;
    }
    if (params.model == PhysicsModel::LennardJones || params.model == PhysicsModel::NBody) pe += forcePotential_;
    d.kinetic = ke;
    d.potential = pe;
    d.momentum = {(float)mx, (float)my};
    d.maxSpeed = std::sqrt(maxSpeed2);
    if (params.model == PhysicsModel::LennardJones && moving > 0 && params.ljEpsilon > 0)
        d.temperature = ke / moving / params.ljEpsilon;  // 2D: <KE> = kT per particle
    diag_ = d;
}
