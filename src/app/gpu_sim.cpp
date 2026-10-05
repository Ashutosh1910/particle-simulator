#include "gpu_sim.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#ifdef PSIM_GPU
#include "raylib.h"
#include "rlgl.h"

// rlgl has no wrapper for glMemoryBarrier; fetch it through GLFW (linked into raylib).
extern "C" {
typedef void (*PsimGlProc)(void);
PsimGlProc glfwGetProcAddress(const char* procname);
}
typedef void (*PfnMemoryBarrier)(unsigned int barriers);
typedef void (*PfnDeleteShader)(unsigned int shader);
static PfnMemoryBarrier s_memoryBarrier = nullptr;
static PfnDeleteShader s_deleteShader = nullptr;
static const unsigned int kShaderStorageBarrierBit = 0x00002000;  // GL_SHADER_STORAGE_BARRIER_BIT
static const unsigned int kBufferUpdateBarrierBit = 0x00000200;   // GL_BUFFER_UPDATE_BARRIER_BIT

static void barrier() {
    if (s_memoryBarrier) s_memoryBarrier(kShaderStorageBarrierBit | kBufferUpdateBarrierBit);
}

namespace {

// Shared declarations. Each shader body declares its own local_size_x.
const char* kCommon = R"(#version 430
layout(std430, binding = 0) buffer StateIn { vec4 stateIn[]; };
layout(std430, binding = 1) buffer StateOut { vec4 stateOut[]; };
layout(std430, binding = 2) readonly buffer Props { vec2 props[]; };   // radius, inverse mass
layout(std430, binding = 3) buffer CellCount { uint cellCount[]; };
layout(std430, binding = 4) buffer CellStart { uint cellStart[]; };
layout(std430, binding = 5) buffer CellItems { uint cellItems[]; };
layout(std430, binding = 6) buffer ParticleCell { uint particleCell[]; };
uniform int n;
uniform int cols;
uniform int rows;
uniform float cellSize;
uniform vec2 origin;
uniform float dt;
uniform vec2 gravity;
uniform float damp;
uniform vec4 bounds;        // minX, minY, maxX, maxY
uniform float restitution;
uniform float restSpeed;
uniform vec4 mouse;         // x, y, radius, strength (strength 0 = off)
ivec2 cellOf(vec2 p) {
    return clamp(ivec2(floor((p - origin) / cellSize)), ivec2(0), ivec2(cols - 1, rows - 1));
}
)";

const char* kClear = R"(
layout(local_size_x = 256) in;
void main() {
    uint c = gl_GlobalInvocationID.x;
    if (c < uint(cols * rows)) cellCount[c] = 0u;
}
)";

const char* kIntegrate = R"(
layout(local_size_x = 256) in;
float bounce(float v) { return abs(v) < restSpeed ? 0.0 : -v * restitution; }
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uint(n)) return;
    vec4 s = stateIn[i];
    vec2 pr = props[i];
    if (pr.y > 0.0) {
        vec2 a = gravity;
        if (mouse.w != 0.0) {
            vec2 d = mouse.xy - s.xy;
            float dist = length(d);
            if (dist < mouse.z && dist > 1.0) a += d * (mouse.w * (1.0 - dist / mouse.z) / dist);
        }
        vec2 v = (s.zw + a * dt) * damp;
        vec2 x = s.xy + v * dt;
        float r = pr.x;
        if (x.x < bounds.x + r) { x.x = bounds.x + r; if (v.x < 0.0) v.x = bounce(v.x); }
        if (x.x > bounds.z - r) { x.x = bounds.z - r; if (v.x > 0.0) v.x = bounce(v.x); }
        if (x.y < bounds.y + r) { x.y = bounds.y + r; if (v.y < 0.0) v.y = bounce(v.y); }
        if (x.y > bounds.w - r) { x.y = bounds.w - r; if (v.y > 0.0) v.y = bounce(v.y); }
        s = vec4(x, v);
        stateIn[i] = s;
    }
    ivec2 c = cellOf(s.xy);
    uint cell = uint(c.y * cols + c.x);
    particleCell[i] = cell;
    atomicAdd(cellCount[cell], 1u);
}
)";

// Exclusive prefix sum over all cells in one work group of 1024 threads: each
// thread sums a contiguous chunk, the chunk sums are scanned in shared memory,
// then each thread writes the offsets of its chunk. cellCount is reset to 0 so
// the scatter pass can reuse it as an insertion cursor.
const char* kScan = R"(
layout(local_size_x = 1024) in;
shared uint partial[1024];
void main() {
    uint t = gl_LocalInvocationID.x;
    uint cells = uint(cols * rows);
    uint chunk = (cells + 1023u) / 1024u;
    uint begin = min(t * chunk, cells), end = min(begin + chunk, cells);
    uint sum = 0u;
    for (uint c = begin; c < end; c++) sum += cellCount[c];
    partial[t] = sum;
    barrier();
    for (uint offset = 1u; offset < 1024u; offset <<= 1u) {
        uint v = t >= offset ? partial[t - offset] : 0u;
        barrier();
        partial[t] += v;
        barrier();
    }
    uint running = t == 0u ? 0u : partial[t - 1u];
    for (uint c = begin; c < end; c++) {
        uint count = cellCount[c];
        cellStart[c] = running;
        running += count;
        cellCount[c] = 0u;
    }
    if (t == 1023u) cellStart[cells] = partial[1023];
}
)";

const char* kScatter = R"(
layout(local_size_x = 256) in;
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uint(n)) return;
    uint cell = particleCell[i];
    uint slot = cellStart[cell] + atomicAdd(cellCount[cell], 1u);
    cellItems[slot] = i;
}
)";

// Jacobi contact solve: each particle accumulates its own share of every
// overlap and impulse, reading only the previous state, so there are no races.
// Position corrections are averaged over the particle's contacts (with
// over-relaxation 1.5, Macklin et al. 2014) so dense piles don't overshoot.
const char* kCollide = R"(
layout(local_size_x = 256) in;
const float omega = 1.5;
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uint(n)) return;
    vec4 s = stateIn[i];
    vec2 pr = props[i];
    if (pr.y == 0.0) { stateOut[i] = s; return; }
    vec2 dx = vec2(0.0), dv = vec2(0.0);
    int contacts = 0;
    ivec2 c = cellOf(s.xy);
    for (int oy = -1; oy <= 1; oy++) {
        int cy = c.y + oy;
        if (cy < 0 || cy >= rows) continue;
        for (int ox = -1; ox <= 1; ox++) {
            int cx = c.x + ox;
            if (cx < 0 || cx >= cols) continue;
            uint cell = uint(cy * cols + cx);
            for (uint k = cellStart[cell]; k < cellStart[cell + 1u]; k++) {
                uint j = cellItems[k];
                if (j == i) continue;
                vec4 o = stateIn[j];
                vec2 po = props[j];
                vec2 d = o.xy - s.xy;
                float r = pr.x + po.x;
                float d2 = dot(d, d);
                if (d2 >= r * r || d2 < 1e-12) continue;
                float dist = sqrt(d2);
                vec2 nrm = d / dist;
                float w = pr.y + po.y;
                dx -= nrm * ((r - dist) * pr.y / w);
                contacts++;
                float vn = dot(o.zw - s.zw, nrm);
                if (vn < 0.0) {
                    float e = vn > -restSpeed ? 0.0 : restitution;
                    dv -= nrm * (-(1.0 + e) * vn / w * pr.y);
                }
            }
        }
    }
    if (contacts > 0) dx *= min(1.0, omega / float(contacts));
    stateOut[i] = vec4(s.xy + dx, s.zw + dv);
}
)";

unsigned compile(const char* body) {
    std::string src = std::string(kCommon) + body;
    unsigned shader = rlCompileShader(src.c_str(), RL_COMPUTE_SHADER);
    if (shader == 0) return 0;
    unsigned program = rlLoadComputeShaderProgram(shader);
    if (s_deleteShader) s_deleteShader(shader);  // the linked program keeps what it needs
    return program;
}

void setInt(unsigned prog, const char* name, int v) {
    int loc = rlGetLocationUniform(prog, name);
    if (loc >= 0) rlSetUniform(loc, &v, RL_SHADER_UNIFORM_INT, 1);
}
void setFloat(unsigned prog, const char* name, float v) {
    int loc = rlGetLocationUniform(prog, name);
    if (loc >= 0) rlSetUniform(loc, &v, RL_SHADER_UNIFORM_FLOAT, 1);
}
void setVec2(unsigned prog, const char* name, float x, float y) {
    float v[2] = {x, y};
    int loc = rlGetLocationUniform(prog, name);
    if (loc >= 0) rlSetUniform(loc, v, RL_SHADER_UNIFORM_VEC2, 1);
}
void setVec4(unsigned prog, const char* name, float x, float y, float z, float w) {
    float v[4] = {x, y, z, w};
    int loc = rlGetLocationUniform(prog, name);
    if (loc >= 0) rlSetUniform(loc, v, RL_SHADER_UNIFORM_VEC4, 1);
}

struct GridDims {
    float cellSize, ox, oy;
    int cols, rows;
};
GridDims gridFor(const World& w) {
    float maxR = 1;
    for (float r : w.p.radius) maxR = std::max(maxR, r);
    GridDims g;
    g.cellSize = 2 * maxR * 1.0001f;
    g.ox = w.bounds.minX;
    g.oy = w.bounds.minY;
    g.cols = std::max(1, (int)std::ceil(w.bounds.width() / g.cellSize));
    g.rows = std::max(1, (int)std::ceil(w.bounds.height() / g.cellSize));
    return g;
}

}  // namespace

GpuSim::GpuSim() { status_ = "not initialised"; }
GpuSim::~GpuSim() = default;

bool GpuSim::init() {
    if (initTried_) return available_;
    initTried_ = true;
    if (rlGetVersion() != RL_OPENGL_43) {
        status_ = "needs an OpenGL 4.3 context (raylib built with OPENGL_VERSION 4.3)";
        return false;
    }
    s_memoryBarrier = (PfnMemoryBarrier)glfwGetProcAddress("glMemoryBarrier");
    s_deleteShader = (PfnDeleteShader)glfwGetProcAddress("glDeleteShader");
    progClear_ = compile(kClear);
    progIntegrate_ = compile(kIntegrate);
    progScan_ = compile(kScan);
    progScatter_ = compile(kScatter);
    progCollide_ = compile(kCollide);
    if (!progClear_ || !progIntegrate_ || !progScan_ || !progScatter_ || !progCollide_ || !s_memoryBarrier) {
        status_ = "compute shaders failed to compile";
        release();
        return false;
    }
    available_ = true;
    status_ = "OpenGL 4.3 compute shaders ready";
    return true;
}

void GpuSim::ensureCapacity(int particles, int cells) {
    if (particles > capacity_) {
        capacity_ = std::max(particles, capacity_ * 2);
        for (unsigned& b : bufState_) {
            if (b) rlUnloadShaderBuffer(b);
            b = rlLoadShaderBuffer(capacity_ * 16, nullptr, RL_DYNAMIC_COPY);
        }
        if (bufProps_) rlUnloadShaderBuffer(bufProps_);
        if (bufCellItems_) rlUnloadShaderBuffer(bufCellItems_);
        if (bufParticleCell_) rlUnloadShaderBuffer(bufParticleCell_);
        bufProps_ = rlLoadShaderBuffer(capacity_ * 8, nullptr, RL_DYNAMIC_COPY);
        bufCellItems_ = rlLoadShaderBuffer(capacity_ * 4, nullptr, RL_DYNAMIC_COPY);
        bufParticleCell_ = rlLoadShaderBuffer(capacity_ * 4, nullptr, RL_DYNAMIC_COPY);
    }
    if (cells + 1 > cellCapacity_) {
        cellCapacity_ = std::max(cells + 1, cellCapacity_ * 2);
        if (bufCellCount_) rlUnloadShaderBuffer(bufCellCount_);
        if (bufCellStart_) rlUnloadShaderBuffer(bufCellStart_);
        bufCellCount_ = rlLoadShaderBuffer(cellCapacity_ * 4, nullptr, RL_DYNAMIC_COPY);
        bufCellStart_ = rlLoadShaderBuffer(cellCapacity_ * 4, nullptr, RL_DYNAMIC_COPY);
    }
}

void GpuSim::upload(const World& world) {
    if (!available_) return;
    count_ = world.p.size();
    GridDims g = gridFor(world);
    ensureCapacity(std::max(count_, 1), g.cols * g.rows);
    std::vector<float> state(4 * (size_t)count_), props(2 * (size_t)count_);
    for (int i = 0; i < count_; i++) {
        state[4 * i] = world.p.x[i];
        state[4 * i + 1] = world.p.y[i];
        state[4 * i + 2] = world.p.vx[i];
        state[4 * i + 3] = world.p.vy[i];
        props[2 * i] = world.p.radius[i];
        props[2 * i + 1] = world.p.invMass[i];
    }
    current_ = 0;
    if (count_ > 0) {
        rlUpdateShaderBuffer(bufState_[0], state.data(), count_ * 16, 0);
        rlUpdateShaderBuffer(bufProps_, props.data(), count_ * 8, 0);
    }
}

void GpuSim::step(World& world) {
    if (!available_) return;
    auto t0 = std::chrono::steady_clock::now();
    const Params& P = world.params;
    float frameDt = P.dt * P.timeScale;
    int subs = std::clamp(P.substeps, 1, 64);
    float h = frameDt / subs;
    if (count_ == 0 || h <= 0) {
        world.advanceClock(frameDt);
        return;
    }
    GridDims g = gridFor(world);
    ensureCapacity(count_, g.cols * g.rows);
    const MouseForce& m = world.mouse;
    float restSpeed = 2.0f * length(P.gravity) * h + 1e-3f;
    unsigned programs[] = {progClear_, progIntegrate_, progScan_, progScatter_, progCollide_};
    for (unsigned prog : programs) {
        rlEnableShader(prog);
        setInt(prog, "n", count_);
        setInt(prog, "cols", g.cols);
        setInt(prog, "rows", g.rows);
        setFloat(prog, "cellSize", g.cellSize);
        setVec2(prog, "origin", g.ox, g.oy);
        setFloat(prog, "dt", h);
        setVec2(prog, "gravity", P.gravity.x, P.gravity.y);
        setFloat(prog, "damp", 1.0f / (1.0f + P.drag * h));
        setVec4(prog, "bounds", world.bounds.minX, world.bounds.minY, world.bounds.maxX, world.bounds.maxY);
        setFloat(prog, "restitution", P.restitution);
        setFloat(prog, "restSpeed", restSpeed);
        setVec4(prog, "mouse", m.pos.x, m.pos.y, m.radius, m.active ? m.strength : 0.0f);
    }
    unsigned particleGroups = (unsigned)(count_ + 255) / 256;
    unsigned cellGroups = (unsigned)(g.cols * g.rows + 255) / 256;
    rlBindShaderBuffer(bufProps_, 2);
    rlBindShaderBuffer(bufCellCount_, 3);
    rlBindShaderBuffer(bufCellStart_, 4);
    rlBindShaderBuffer(bufCellItems_, 5);
    rlBindShaderBuffer(bufParticleCell_, 6);
    for (int s = 0; s < subs; s++) {
        rlBindShaderBuffer(bufState_[current_], 0);
        rlBindShaderBuffer(bufState_[1 - current_], 1);
        rlEnableShader(progClear_);
        rlComputeShaderDispatch(cellGroups, 1, 1);
        barrier();
        rlEnableShader(progIntegrate_);
        rlComputeShaderDispatch(particleGroups, 1, 1);
        barrier();
        rlEnableShader(progScan_);
        rlComputeShaderDispatch(1, 1, 1);
        barrier();
        rlEnableShader(progScatter_);
        rlComputeShaderDispatch(particleGroups, 1, 1);
        barrier();
        rlEnableShader(progCollide_);
        rlComputeShaderDispatch(particleGroups, 1, 1);
        barrier();
        current_ = 1 - current_;
    }
    rlDisableShader();

    std::vector<float> state(4 * (size_t)count_);
    rlReadShaderBuffer(bufState_[current_], state.data(), count_ * 16, 0);
    int n = std::min(count_, world.p.size());
    for (int i = 0; i < n; i++) {
        world.p.x[i] = state[4 * i];
        world.p.y[i] = state[4 * i + 1];
        world.p.vx[i] = state[4 * i + 2];
        world.p.vy[i] = state[4 * i + 3];
    }
    world.advanceClock(frameDt);
    world.computeDiagnostics();
    lastStepMs_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void GpuSim::release() {
    unsigned* progs[] = {&progClear_, &progIntegrate_, &progScan_, &progScatter_, &progCollide_};
    for (unsigned* p : progs) {
        if (*p) rlUnloadShaderProgram(*p);
        *p = 0;
    }
    unsigned* bufs[] = {&bufState_[0], &bufState_[1], &bufProps_, &bufCellCount_, &bufCellStart_, &bufCellItems_, &bufParticleCell_};
    for (unsigned* b : bufs) {
        if (*b) rlUnloadShaderBuffer(*b);
        *b = 0;
    }
    capacity_ = cellCapacity_ = 0;
    available_ = false;
}

#else  // !PSIM_GPU

GpuSim::GpuSim() {
    status_ = "not built with GPU support (needs OpenGL 4.3; macOS supports up to 4.1). Build with GPU=1 on Linux/Windows.";
}
GpuSim::~GpuSim() = default;
bool GpuSim::init() { return false; }
void GpuSim::ensureCapacity(int, int) {}
void GpuSim::upload(const World&) {}
void GpuSim::step(World&) {}
void GpuSim::release() {}

#endif
