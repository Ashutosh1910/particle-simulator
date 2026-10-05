// Force-based models (Lennard-Jones, N-body gravity) and the SPH fluid.
#include <chrono>
#include <cmath>
#include <iterator>
#include <numeric>

#include "world.h"

namespace {
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
}  // namespace

// Converts the broad-phase pair list into per-particle neighbour lists (CSR,
// both directions) so per-particle loops can run in parallel without races.
// range <= 0 keeps every candidate pair.
void World::buildNeighbourLists(float range) {
    int n = p.size();
    nbStart_.assign(n + 1, 0);
    float r2 = range * range;
    auto keep = [&](const Pair& pr) {
        if (range <= 0) return true;
        float dx = p.x[pr.b] - p.x[pr.a], dy = p.y[pr.b] - p.y[pr.a];
        return dx * dx + dy * dy < r2;
    };
    for (const Pair& pr : pairs_)
        if (keep(pr)) { nbStart_[pr.a + 1]++; nbStart_[pr.b + 1]++; }
    for (int i = 0; i < n; i++) nbStart_[i + 1] += nbStart_[i];
    nbList_.resize(nbStart_[n]);
    std::vector<int> cursor(nbStart_.begin(), nbStart_.end() - 1);
    for (const Pair& pr : pairs_)
        if (keep(pr)) { nbList_[cursor[pr.a]++] = pr.b; nbList_[cursor[pr.b]++] = pr.a; }
}

// ---------------------------------------------------------------------------
// SPH: double-density relaxation (Clavet, Beaudoin, Poulin 2005)
// ---------------------------------------------------------------------------
//
// Each substep: apply gravity, apply pairwise viscosity impulses, predict
// positions, then push particles apart/together so that every particle's
// (near-)density approaches the rest density, and finally derive velocities
// from the position change. Both the viscosity and the relaxation are done
// Jacobi-style (each particle accumulates its own symmetric correction) so they
// parallelise; the original paper updates positions sequentially.

void World::substepSPH(float h, int sub, int subs) {
    int n = p.size();
    applyExternal(h);
    findCandidatePairs();
    auto t0 = Clock::now();
    const float H = params.sphRadius;
    buildNeighbourLists(H);
    stats_.interactions = (long long)nbList_.size();

    const float sigma = params.sphViscosityLinear, beta = params.sphViscosityQuadratic;
    scratchX_.assign(n, 0);
    scratchY_.assign(n, 0);
    if (sigma > 0 || beta > 0) {
        pool_.parallelFor(n, [&](int, int b, int e) {
            for (int i = b; i < e; i++) {
                float dvx = 0, dvy = 0;
                for (int k = nbStart_[i]; k < nbStart_[i + 1]; k++) {
                    int j = nbList_[k];
                    float dx = p.x[j] - p.x[i], dy = p.y[j] - p.y[i];
                    float d = std::sqrt(dx * dx + dy * dy);
                    if (d < 1e-6f || d >= H) continue;
                    float rx = dx / d, ry = dy / d;
                    float u = (p.vx[i] - p.vx[j]) * rx + (p.vy[i] - p.vy[j]) * ry;  // approach speed
                    if (u <= 0) continue;
                    float imp = h * (1 - d / H) * (sigma * u + beta * u * u) * 0.5f;
                    dvx -= rx * imp;
                    dvy -= ry * imp;
                }
                scratchX_[i] = dvx;
                scratchY_[i] = dvy;
            }
        }, 512);
    }
    pool_.parallelFor(n, [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            p.px[i] = p.x[i];
            p.py[i] = p.y[i];
            if (p.invMass[i] == 0) continue;
            p.vx[i] += scratchX_[i];
            p.vy[i] += scratchY_[i];
            p.x[i] += p.vx[i] * h;
            p.y[i] += p.vy[i] * h;
        }
    }, 2048);
    moveGrabbedParticle(sub, subs, h);

    density_.assign(n, 0);
    nearDensity_.assign(n, 0);
    pool_.parallelFor(n, [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            float rho = 0, rhoNear = 0;
            for (int k = nbStart_[i]; k < nbStart_[i + 1]; k++) {
                int j = nbList_[k];
                float dx = p.x[j] - p.x[i], dy = p.y[j] - p.y[i];
                float d = std::sqrt(dx * dx + dy * dy);
                if (d >= H) continue;
                float q = 1 - d / H;
                rho += q * q;
                rhoNear += q * q * q;
            }
            density_[i] = rho;
            nearDensity_[i] = rhoNear;
        }
    }, 512);

    const float k = params.sphStiffness, kNear = params.sphNearStiffness, rho0 = params.sphRestDensity;
    pool_.parallelFor(n, [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            float pi = k * (density_[i] - rho0), pni = kNear * nearDensity_[i];
            float dxSum = 0, dySum = 0;
            for (int kk = nbStart_[i]; kk < nbStart_[i + 1]; kk++) {
                int j = nbList_[kk];
                float dx = p.x[j] - p.x[i], dy = p.y[j] - p.y[i];
                float d = std::sqrt(dx * dx + dy * dy);
                if (d < 1e-6f || d >= H) continue;
                float q = 1 - d / H;
                float pj = k * (density_[j] - rho0), pnj = kNear * nearDensity_[j];
                float D = 0.5f * h * h * ((pi + pj) * q + (pni + pnj) * q * q);
                dxSum -= D * dx / d;
                dySum -= D * dy / d;
            }
            scratchX_[i] = dxSum;
            scratchY_[i] = dySum;
        }
    }, 512);
    pool_.parallelFor(n, [&](int, int b, int e) {
        for (int i = b; i < e; i++) {
            if (p.invMass[i] == 0) continue;
            p.x[i] += scratchX_[i];
            p.y[i] += scratchY_[i];
            p.vx[i] = (p.x[i] - p.px[i]) / h;
            p.vy[i] = (p.y[i] - p.py[i]) / h;
        }
    }, 2048);
    stats_.solveMs += msSince(t0);
    stepBodies(h);
    solveParticleWalls(h);
}

// ---------------------------------------------------------------------------
// Lennard-Jones / N-body: force-based integration
// ---------------------------------------------------------------------------

void World::substepForces(float h, int sub, int subs) {
    int n = p.size();
    const bool verlet = params.integrator == Integrator::Verlet;
    const Vec2 g = params.gravity;
    const MouseForce m = mouse;
    // external acceleration (gravity + mouse field) for particle i
    auto ext = [&](int i, float& ax, float& ay) {
        ax = g.x; ay = g.y;
        if (m.active) {
            float dx = m.pos.x - p.x[i], dy = m.pos.y - p.y[i];
            float d = std::sqrt(dx * dx + dy * dy);
            if (d < m.radius && d > 1) {
                float s = m.strength * (1 - d / m.radius) / d;
                ax += dx * s;
                ay += dy * s;
            }
        }
    };
    // v += (a_pair + a_ext) * dt, with implicit drag over the same dt
    auto kick = [&](float dt) {
        const float damp = 1.0f / (1.0f + params.drag * dt);
        pool_.parallelFor(n, [&](int, int b, int e) {
            for (int i = b; i < e; i++) {
                if (p.invMass[i] == 0) continue;
                float ax, ay;
                ext(i, ax, ay);
                p.vx[i] = (p.vx[i] + (p.ax[i] + ax) * dt) * damp;
                p.vy[i] = (p.vy[i] + (p.ay[i] + ay) * dt) * damp;
            }
        }, 2048);
    };
    auto drift = [&]() {
        pool_.parallelFor(n, [&](int, int b, int e) {
            for (int i = b; i < e; i++) {
                if (p.invMass[i] == 0) continue;
                p.x[i] += p.vx[i] * h;
                p.y[i] += p.vy[i] * h;
            }
        }, 2048);
    };

    // anything that changes the force law invalidates the cached accelerations
    const float key[] = {(float)params.model, params.ljEpsilon, params.ljCutoff, params.G, params.softening, params.theta,
                         params.barnesHut ? 1.0f : 0.0f};
    if (!std::equal(std::begin(key), std::end(key), forceKey_)) {
        std::copy(std::begin(key), std::end(key), forceKey_);
        forcesValid_ = false;
    }
    // Accelerations are always evaluated at the end of a substep, after the walls
    // have clamped positions, so they match the positions the next substep starts
    // from (and the reported potential energy matches the reported kinetic energy).
    if (!forcesValid_) computeForces();
    if (verlet) {
        // velocity Verlet (kick-drift-kick): second order and time-reversible
        kick(0.5f * h);
        drift();
        moveGrabbedParticle(sub, subs, h);
        solveParticleWalls(h);
        computeForces();
        kick(0.5f * h);
    } else {
        // symplectic Euler: v += a(x) dt, then x += v dt
        kick(h);
        drift();
        moveGrabbedParticle(sub, subs, h);
        solveParticleWalls(h);
        computeForces();
    }
    if (params.model == PhysicsModel::LennardJones) {
        applyThermostat(h);
        // Safety net for low substep counts: a molecule may not move more than
        // 0.3 sigma per substep, otherwise a hard collision overshoots deep into
        // the r^-12 wall and the system explodes.
        pool_.parallelFor(n, [&](int, int b, int e) {
            for (int i = b; i < e; i++) {
                float vmax = 0.3f * (2.0f * p.radius[i] / 1.122462f) / h;
                float v2 = p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i];
                if (v2 > vmax * vmax) {
                    float s = vmax / std::sqrt(v2);
                    p.vx[i] *= s;
                    p.vy[i] *= s;
                }
            }
        }, 2048);
    }
}

void World::computeForces() {
    auto t0 = Clock::now();
    double potential = 0;
    long long interactions = 0;
    if (params.model == PhysicsModel::LennardJones) {
        findCandidatePairs();
        t0 = Clock::now();
        computeLennardJones(potential, interactions);
    } else {
        computeGravity(params.barnesHut, p.ax, p.ay, potential, interactions);
    }
    forcePotential_ = potential;
    stats_.interactions = interactions;
    forcesValid_ = true;
    stats_.solveMs += msSince(t0);
}

void World::computeLennardJones(double& potential, long long& interactions) {
    int n = p.size();
    buildNeighbourLists(0);
    interactions = (long long)nbList_.size();
    const float eps = params.ljEpsilon, cutoff = params.ljCutoff;
    const float inv2pow16 = 1.0f / 1.122462f;
    // Shifted-force potential: both the potential and the force go to zero at the
    // cutoff, so particles crossing it don't get kicks that break energy conservation.
    //   V_sf(r) = V(r) - V(rc) + (r - rc) F(rc),  F_sf(r) = F(r) - F(rc)
    // In units of sigma, V(rc) and F(rc)*sigma depend only on the cutoff ratio.
    const float sc6 = std::pow(1.0f / cutoff, 6.0f), sc12 = sc6 * sc6;
    const float vCut = 4 * eps * (sc12 - sc6);
    const float fCutSigma = 24 * eps * (2 * sc12 - sc6) / cutoff;  // F(rc) * sigma
    const int workers = pool_.threadCount();
    std::vector<double> pe(workers, 0.0);
    pool_.parallelFor(n, [&](int w, int b, int e) {
        double localPe = 0;
        for (int i = b; i < e; i++) {
            float fx = 0, fy = 0;
            for (int k = nbStart_[i]; k < nbStart_[i + 1]; k++) {
                int j = nbList_[k];
                float sigma = (p.radius[i] + p.radius[j]) * inv2pow16;
                float rc = cutoff * sigma;
                float dx = p.x[i] - p.x[j], dy = p.y[i] - p.y[j];
                float r2 = dx * dx + dy * dy;
                if (r2 >= rc * rc || r2 < 1e-12f) continue;
                float r = std::sqrt(r2);
                // clamp very close approaches so overlapping spawns can't explode
                float rClamped = std::max(r, 0.8f * sigma);
                float s2 = sigma * sigma / (rClamped * rClamped), s6 = s2 * s2 * s2, s12 = s6 * s6;
                float fCut = fCutSigma / sigma;
                float fMag = 24 * eps * (2 * s12 - s6) / rClamped - fCut;  // > 0 repulsive
                fx += fMag * dx / r;
                fy += fMag * dy / r;
                // below the clamp the force is constant, so the potential continues linearly
                double v = 4 * eps * (s12 - s6) - vCut + (rClamped - rc) * fCut + (rClamped - r) * fMag;
                localPe += 0.5 * v;  // each pair is visited twice
            }
            p.ax[i] = fx * p.invMass[i];
            p.ay[i] = fy * p.invMass[i];
        }
        pe[w] += localPe;
    }, 512);
    potential = std::accumulate(pe.begin(), pe.end(), 0.0);
}

void World::applyThermostat(float h) {
    if (!params.thermostat || params.ljEpsilon <= 0) return;
    double ke = 0;
    int moving = 0;
    for (int i = 0; i < p.size(); i++) {
        if (p.invMass[i] == 0) continue;
        ke += 0.5 * (p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i]) / p.invMass[i];
        moving++;
    }
    if (moving == 0 || ke <= 0) return;
    double T = ke / moving, T0 = params.targetTemperature * params.ljEpsilon;
    // Berendsen weak coupling: relax T towards T0 with time constant tau
    double lambda = std::sqrt(std::max(0.0, 1.0 + (h / params.thermostatTau) * (T0 / T - 1.0)));
    float s = (float)std::clamp(lambda, 0.9, 1.1);
    for (int i = 0; i < p.size(); i++) {
        if (p.invMass[i] == 0) continue;
        p.vx[i] *= s;
        p.vy[i] *= s;
    }
}

// ---------------------------------------------------------------------------
// N-body gravity
// ---------------------------------------------------------------------------

void World::buildBarnesHut() {
    int n = p.size();
    bh_.clear();
    bhItems_.resize(n);
    std::iota(bhItems_.begin(), bhItems_.end(), 0);
    if (n == 0) return;
    float minX = 1e30f, minY = 1e30f, maxX = -1e30f, maxY = -1e30f;
    for (int i = 0; i < n; i++) {
        minX = std::min(minX, p.x[i]); maxX = std::max(maxX, p.x[i]);
        minY = std::min(minY, p.y[i]); maxY = std::max(maxY, p.y[i]);
    }
    float half = 0.5f * std::max(maxX - minX, maxY - minY) + 1e-3f;
    bh_.push_back({0.5f * (minX + maxX), 0.5f * (minY + maxY), half, 0, 0, 0, -1, 0, n, 0});
    const int kMaxDepth = 48;
    for (size_t idx = 0; idx < bh_.size(); idx++) {
        BhNode node = bh_[idx];
        if (node.end - node.begin <= 1 || node.depth >= kMaxDepth) continue;
        auto first = bhItems_.begin() + node.begin, last = bhItems_.begin() + node.end;
        auto midY = std::partition(first, last, [&](int i) { return p.y[i] < node.cy; });
        auto midTop = std::partition(first, midY, [&](int i) { return p.x[i] < node.cx; });
        auto midBottom = std::partition(midY, last, [&](int i) { return p.x[i] < node.cx; });
        int b[5] = {node.begin, (int)(midTop - bhItems_.begin()), (int)(midY - bhItems_.begin()),
                    (int)(midBottom - bhItems_.begin()), node.end};
        float q = 0.5f * node.half;
        float ox[4] = {-q, q, -q, q}, oy[4] = {-q, -q, q, q};
        bh_[idx].child = (int)bh_.size();
        for (int c = 0; c < 4; c++)
            bh_.push_back({node.cx + ox[c], node.cy + oy[c], q, 0, 0, 0, -1, b[c], b[c + 1], node.depth + 1});
    }
    // children are always stored after their parent: accumulate bottom-up
    for (int idx = (int)bh_.size() - 1; idx >= 0; idx--) {
        BhNode& node = bh_[idx];
        double m = 0, mx = 0, my = 0;
        if (node.child < 0) {
            for (int k = node.begin; k < node.end; k++) {
                int i = bhItems_[k];
                double mi = gmass_[i];
                m += mi; mx += mi * p.x[i]; my += mi * p.y[i];
            }
        } else {
            for (int c = 0; c < 4; c++) {
                const BhNode& ch = bh_[node.child + c];
                m += ch.mass; mx += (double)ch.mass * ch.comX; my += (double)ch.mass * ch.comY;
            }
        }
        node.mass = (float)m;
        node.comX = m > 0 ? (float)(mx / m) : node.cx;
        node.comY = m > 0 ? (float)(my / m) : node.cy;
    }
}

void World::computeGravity(bool barnesHut, std::vector<float>& ax, std::vector<float>& ay, double& potential,
                           long long& interactions) {
    int n = p.size();
    ax.assign(n, 0);
    ay.assign(n, 0);
    const float G = params.G, eps2 = params.softening * params.softening;
    const float theta2 = params.theta * params.theta;
    const int workers = pool_.threadCount();
    std::vector<double> pe(workers, 0.0);
    std::vector<long long> count(workers, 0);
    // gravitational mass: a grabbed particle is kinematic (invMass 0) but still pulls
    gmass_.resize(n);
    for (int i = 0; i < n; i++) gmass_[i] = p.mass(i);
    if (grab_.kind == GrabKind::Particle && grab_.index < n && grab_.savedInvMass > 0)
        gmass_[grab_.index] = 1.0f / grab_.savedInvMass;
    if (barnesHut) buildBarnesHut();

    pool_.parallelFor(n, [&](int w, int b, int e) {
        double localPe = 0;
        long long localCount = 0;
        int stack[256];
        for (int i = b; i < e; i++) {
            float xi = p.x[i], yi = p.y[i];
            float accX = 0, accY = 0;
            double phi = 0;
            auto pull = [&](float mx, float my, float mass) {
                float dx = mx - xi, dy = my - yi;
                float r2 = dx * dx + dy * dy + eps2;
                float inv = 1.0f / std::sqrt(r2);
                float s = G * mass * inv * inv * inv;
                accX += s * dx;
                accY += s * dy;
                phi -= (double)G * mass * inv;
                localCount++;
            };
            if (!barnesHut) {
                for (int j = 0; j < n; j++)
                    if (j != i) pull(p.x[j], p.y[j], gmass_[j]);
            } else {
                int top = 0;
                stack[top++] = 0;
                while (top > 0) {
                    const BhNode& node = bh_[stack[--top]];
                    if (node.mass <= 0) continue;
                    if (node.child < 0) {
                        for (int k = node.begin; k < node.end; k++) {
                            int j = bhItems_[k];
                            if (j != i) pull(p.x[j], p.y[j], gmass_[j]);
                        }
                        continue;
                    }
                    float dx = node.comX - xi, dy = node.comY - yi;
                    float size = 2 * node.half;
                    // far enough: the whole cell acts like one mass at its centre of mass.
                    // A cell containing the particle itself is always opened (otherwise a
                    // large theta would let the particle attract itself).
                    bool containsSelf = std::fabs(xi - node.cx) <= node.half && std::fabs(yi - node.cy) <= node.half;
                    if (!containsSelf && size * size < theta2 * (dx * dx + dy * dy)) {
                        pull(node.comX, node.comY, node.mass);
                    } else {
                        for (int c = 0; c < 4; c++) stack[top++] = node.child + c;
                    }
                }
            }
            ax[i] = accX;
            ay[i] = accY;
            localPe += 0.5 * gmass_[i] * phi;
        }
        pe[w] += localPe;
        count[w] += localCount;
    }, 64);
    potential = std::accumulate(pe.begin(), pe.end(), 0.0);
    interactions = std::accumulate(count.begin(), count.end(), 0LL);
}

void World::barnesHutRects(std::vector<DebugRect>& out, int maxRects) const {
    for (const BhNode& node : bh_) {
        if ((int)out.size() >= maxRects) break;
        if (node.mass <= 0) continue;
        out.push_back({node.cx - node.half, node.cy - node.half, node.cx + node.half, node.cy + node.half, node.depth});
    }
}
