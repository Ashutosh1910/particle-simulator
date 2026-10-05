#pragma once
#include <string>

#include "../physics/world.h"

// Optional GPU path for the "Rigid" model with plain balls (no links, soft
// bodies or polygons), implemented with OpenGL 4.3 compute shaders:
//
//   1. integrate: gravity, drag, mouse field, x += v dt, walls; count particles per grid cell
//   2. prefix sum over the cell counts (single work group)
//   3. scatter particle indices into their cells
//   4. collide: every particle scans the 3x3 neighbouring cells and accumulates
//      its own position correction and impulse (Jacobi), writing to a second buffer
//
// The CPU World stays the source of truth: positions/velocities are read back
// after each step and re-uploaded only when the world was edited.
//
// Compiled only when PSIM_GPU is defined (OpenGL 4.3 raylib build). macOS
// supports at most OpenGL 4.1, so there the GPU path is unavailable.
class GpuSim {
public:
    GpuSim();
    ~GpuSim();
    GpuSim(const GpuSim&) = delete;
    GpuSim& operator=(const GpuSim&) = delete;

    // Compiles the shaders. Safe to call more than once. Needs a GL context.
    bool init();
    bool available() const { return available_; }
    // Why the GPU path is or isn't available (shown in the UI).
    const std::string& status() const { return status_; }

    // Copies particles to the GPU (call after any edit).
    void upload(const World& world);
    // Advances one frame (params.dt * timeScale in params.substeps substeps) and
    // copies positions and velocities back into world.p.
    void step(World& world);
    double lastStepMs() const { return lastStepMs_; }
    void release();

private:
    bool available_ = false;
    bool initTried_ = false;
    std::string status_;
    double lastStepMs_ = 0;
    int count_ = 0;
    int capacity_ = 0;
    int cellCapacity_ = 0;
    // GL object ids (0 = none)
    unsigned progClear_ = 0, progIntegrate_ = 0, progScan_ = 0, progScatter_ = 0, progCollide_ = 0;
    unsigned bufState_[2] = {0, 0};  // vec4(x, y, vx, vy) ping-pong
    unsigned bufProps_ = 0;          // vec2(radius, invMass)
    unsigned bufCellCount_ = 0, bufCellStart_ = 0, bufCellItems_ = 0, bufParticleCell_ = 0;
    int current_ = 0;
    void ensureCapacity(int particles, int cells);
};
