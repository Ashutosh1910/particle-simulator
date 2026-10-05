#pragma once
#include <memory>
#include <vector>

#include "../broadphase/broadphase.h"
#include "../core/particles.h"
#include "../core/thread_pool.h"
#include "rigid_body.h"

// Which set of physical laws drives the particles. Each scene picks one.
enum class PhysicsModel {
    Rigid,         // hard spheres + constraints + convex polygons
    SPH,           // smoothed-particle hydrodynamics fluid (double-density relaxation)
    LennardJones,  // molecular dynamics with a Lennard-Jones pair potential
    NBody,         // long-range gravity (Barnes-Hut or direct summation)
    Count
};
const char* modelName(PhysicsModel m);

enum class Integrator {
    SymplecticEuler,  // v += a dt; x += v dt (contacts resolved with impulses)
    Verlet,           // Rigid: position-based Verlet (PBD). Force models: velocity Verlet.
};

struct Params {
    PhysicsModel model = PhysicsModel::Rigid;
    Integrator integrator = Integrator::SymplecticEuler;

    // time stepping: every step() advances dt * timeScale seconds in `substeps` equal substeps
    float dt = 1.0f / 60.0f;
    int substeps = 8;
    float timeScale = 1.0f;

    // external forces
    Vec2 gravity{0, 980};
    float drag = 0;  // linear air drag, 1/s

    // contacts (Rigid + walls)
    float restitution = 0.5f;
    float friction = 0.1f;
    bool walls = true;
    int rigidIterations = 8;        // velocity iterations for polygon contacts
    int constraintIterations = 4;   // relaxation passes for links / soft bodies
    float tearRatio = 0;            // links break when stretched beyond rest*tearRatio (0 = unbreakable)

    // SPH (Clavet et al. 2005, "Particle-based viscoelastic fluid simulation")
    float sphRadius = 26;           // interaction radius h, px
    float sphRestDensity = 5;
    float sphStiffness = 4.0e4f;
    float sphNearStiffness = 8.0e4f;
    float sphViscosityLinear = 0.0f;
    float sphViscosityQuadratic = 0.01f;

    // Lennard-Jones
    float ljEpsilon = 20000;        // well depth (px^2/s^2 per unit mass)
    float ljCutoff = 2.5f;          // in units of sigma
    bool thermostat = true;
    float targetTemperature = 1.0f; // in units of epsilon
    float thermostatTau = 0.2f;     // Berendsen relaxation time, s

    // N-body
    float G = 500;
    float softening = 4;
    bool barnesHut = true;
    float theta = 0.6f;

    BroadphaseKind broadphase = BroadphaseKind::UniformGrid;
};

// Distance constraint between two particles (ropes, cloth, soft-body skeletons).
struct Link {
    int a, b;
    float rest;
    float stiffness;  // 0..1 fraction of the error corrected per relaxation pass
};

// Closed ring of particles that tries to keep its area (a pressurised blob).
struct SoftBody {
    std::vector<int> ring;  // particle indices in order around the ring
    float restArea;
    float stiffness;        // 0..1
};

// Mouse attract / repel field applied to everything within `radius`.
struct MouseForce {
    bool active = false;
    Vec2 pos;
    float radius = 160;
    float strength = 0;  // px/s^2 at the centre, > 0 attracts, < 0 repels
};

struct StepStats {
    double stepMs = 0;      // wall time of the last step()
    double broadMs = 0;     // broad phase build + pair search (all substeps)
    double solveMs = 0;     // narrow phase, constraints, forces (all substeps)
    double rigidMs = 0;     // polygon bodies (all substeps)
    long long tests = 0;    // broad-phase tests in the last pass
    int candidates = 0;     // broad-phase pairs in the last pass
    int contacts = 0;       // touching particle pairs in the last substep
    int bodyContacts = 0;   // polygon contact points in the last substep
    long long interactions = 0;  // force evaluations (SPH/LJ neighbours, N-body interactions) in the last substep
};

struct Diagnostics {
    double kinetic = 0;
    double potential = 0;  // gravity field + pair potential (LJ / N-body)
    Vec2 momentum;
    double temperature = 0;  // LJ: mean kinetic energy per particle / epsilon
    float maxSpeed = 0;
};

class World {
public:
    World();
    ~World();

    Params params;
    AABB bounds{0, 0, 1280, 800};  // walls (and the default grid area)
    Particles p;
    std::vector<Link> links;
    std::vector<SoftBody> softBodies;
    std::vector<Body> bodies;
    MouseForce mouse;

    // Advances the simulation by params.dt * params.timeScale.
    void step();
    double time() const { return time_; }
    // For external integrators (GPU path): account for a step computed elsewhere.
    void advanceClock(double dt) { time_ += dt; steps_++; }
    long long stepCount() const { return steps_; }

    void clear();
    void setThreads(int threads);
    int threads() const { return pool_.threadCount(); }
    ThreadPool& pool() { return pool_; }
    // Call after changing params.broadphase.
    void syncBroadphase();
    const Broadphase& broadphase() const { return *broadphase_; }
    // Builds the broad phase for the current positions using the current model's extents.
    void rebuildBroadphase();

    // Half-size of particle i's broad-phase box for the current model (radius or half the interaction range).
    float particleExtent(int i) const;

    // Call after editing particles/bodies or switching integrator/model: invalidates caches.
    void invalidate() { forcesValid_ = false; }

    // Editing helpers that keep links, soft bodies and the grab consistent.
    int addParticle(Vec2 pos, Vec2 vel, float radius, uint32_t color, int group = 0, float mass = -1);
    void removeParticle(int i);
    void removeBody(int i);
    int pickParticle(Vec2 pos, float maxDistance) const;
    int pickBody(Vec2 pos) const;

    // Grabbing: a grabbed particle follows the target exactly (kinematic); a grabbed
    // body is pulled by a damped spring attached at the grab point.
    void grabParticle(int i, Vec2 target);
    void grabBody(int i, Vec2 worldPoint);
    void setGrabTarget(Vec2 target) { grab_.target = target; }
    void releaseGrab();
    bool isGrabbing() const { return grab_.kind != GrabKind::None; }
    int grabbedParticle() const { return grab_.kind == GrabKind::Particle ? grab_.index : -1; }
    int grabbedBody() const { return grab_.kind == GrabKind::Body ? grab_.index : -1; }
    Vec2 grabAnchorWorld() const;

    const StepStats& stats() const { return stats_; }
    const Diagnostics& diagnostics() const { return diag_; }
    void computeDiagnostics();

    // Per-particle scalar from the last SPH step (density) for colouring; empty otherwise.
    const std::vector<float>& sphDensity() const { return density_; }

    // Barnes-Hut tree cells for the overlay (N-body only).
    void barnesHutRects(std::vector<DebugRect>& out, int maxRects) const;

    // Direct O(n^2) and Barnes-Hut accelerations, exposed for tests.
    void computeGravity(bool barnesHut, std::vector<float>& ax, std::vector<float>& ay, double& potential,
                        long long& interactions);

private:
    enum class GrabKind { None, Particle, Body };
    struct Grab {
        GrabKind kind = GrabKind::None;
        int index = -1;
        Vec2 local;   // body-space grab point
        Vec2 target;
        Vec2 prevTarget;
        float savedInvMass = 0;
    };

    // per-model substeps
    void substepRigid(float h, int sub, int subs);
    void substepSPH(float h, int sub, int subs);
    void substepForces(float h, int sub, int subs);  // Lennard-Jones and N-body

    // shared building blocks
    void applyExternal(float h);
    void moveGrabbedParticle(int sub, int subs, float h);
    void findCandidatePairs();
    void buildNeighbourLists(float range);
    void solveParticleContacts(float h, bool positionBased);
    void particleContactVelocityPass(float h);
    void solveLinks(float h, bool adjustVelocity);
    void solveSoftBodies(float h, bool adjustVelocity);
    void stepBodies(float h);
    void solveParticleWalls(float h);
    void computeForces();  // LJ / N-body accelerations into p.ax/p.ay
    void computeLennardJones(double& potential, long long& interactions);
    void applyThermostat(float h);

    ThreadPool pool_;
    std::unique_ptr<Broadphase> broadphase_;
    BroadphaseKind broadphaseKind_ = BroadphaseKind::Count;
    std::vector<Pair> pairs_;
    std::vector<float> extent_;

    // neighbour lists (CSR) for SPH / LJ
    std::vector<int> nbStart_, nbList_;
    std::vector<float> density_, nearDensity_, scratchX_, scratchY_;
    std::vector<float> ringDx_, ringDy_;  // soft-body corrections

    // particle contacts grouped into vertical stripes for parallel solving
    struct PbdContact {
        int i, j;
        Vec2 n;
        float vnBefore;
    };
    std::vector<int> stripeOf_;
    std::vector<std::vector<int>> stripePairs_;          // indices into pairs_
    std::vector<std::vector<PbdContact>> stripeContacts_;
    std::vector<int> stripeContactCount_;

    bool forcesValid_ = false;
    double forcePotential_ = 0;
    Grab grab_;
    double time_ = 0;
    long long steps_ = 0;
    StepStats stats_;
    Diagnostics diag_;

    // Barnes-Hut tree (rebuilt each force evaluation)
    struct BhNode {
        float cx, cy, half;  // square cell
        float mass, comX, comY;
        int child;           // first of 4 children, -1 for leaf
        int begin, end;      // particle range in bhItems_
        int depth;
    };
    std::vector<BhNode> bh_;
    std::vector<int> bhItems_;
    void buildBarnesHut();
};
