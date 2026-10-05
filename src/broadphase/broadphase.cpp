#include "broadphase.h"

#include <chrono>

#include "brute_force.h"
#include "bvh.h"
#include "quadtree.h"
#include "spatial_hash.h"
#include "sweep_and_prune.h"
#include "uniform_grid.h"

namespace {
double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

void Broadphase::build(const BroadphaseInput& in, ThreadPool& pool) {
    auto t0 = std::chrono::steady_clock::now();
    n_ = in.n;
    cx_.assign(in.x, in.x + in.n);
    cy_.assign(in.y, in.y + in.n);
    minX_.resize(n_); minY_.resize(n_); maxX_.resize(n_); maxY_.resize(n_);
    maxExtent_ = 0;
    bounds_ = AABB{0, 0, 0, 0};
    if (n_ > 0) bounds_ = AABB{1e30f, 1e30f, -1e30f, -1e30f};
    for (int i = 0; i < n_; i++) {
        float e = in.extent[i];
        minX_[i] = in.x[i] - e; maxX_[i] = in.x[i] + e;
        minY_[i] = in.y[i] - e; maxY_[i] = in.y[i] + e;
        maxExtent_ = std::max(maxExtent_, e);
        bounds_.minX = std::min(bounds_.minX, minX_[i]);
        bounds_.minY = std::min(bounds_.minY, minY_[i]);
        bounds_.maxX = std::max(bounds_.maxX, maxX_[i]);
        bounds_.maxY = std::max(bounds_.maxY, maxY_[i]);
    }
    buildImpl(pool);
    stats_.buildMs = msSince(t0);
}

void Broadphase::findPairs(std::vector<Pair>& out, ThreadPool& pool) {
    auto t0 = std::chrono::steady_clock::now();
    int workers = pool.threadCount();
    perWorker_.resize(workers);
    tests_.assign(workers, 0);
    for (auto& v : perWorker_) v.clear();
    if (n_ > 1) findPairsImpl(pool);

    // concatenate in worker order: chunks are contiguous index ranges, so the
    // result is identical for any thread count
    size_t start = out.size(), total = 0;
    for (auto& v : perWorker_) total += v.size();
    out.reserve(start + total);
    stats_.tests = 0;
    for (int w = 0; w < workers; w++) {
        out.insert(out.end(), perWorker_[w].begin(), perWorker_[w].end());
        stats_.tests += tests_[w];
    }
    stats_.pairs = (int)total;
    stats_.queryMs = msSince(t0);
}

std::unique_ptr<Broadphase> makeBroadphase(BroadphaseKind kind) {
    switch (kind) {
        case BroadphaseKind::UniformGrid: return std::make_unique<UniformGrid>();
        case BroadphaseKind::SpatialHash: return std::make_unique<SpatialHash>();
        case BroadphaseKind::Quadtree: return std::make_unique<Quadtree>();
        case BroadphaseKind::SweepAndPrune: return std::make_unique<SweepAndPrune>();
        case BroadphaseKind::BVH: return std::make_unique<BVH>();
        case BroadphaseKind::BruteForce: return std::make_unique<BruteForce>();
        default: return std::make_unique<UniformGrid>();
    }
}

const char* broadphaseName(BroadphaseKind kind) {
    switch (kind) {
        case BroadphaseKind::UniformGrid: return "Uniform grid";
        case BroadphaseKind::SpatialHash: return "Spatial hash";
        case BroadphaseKind::Quadtree: return "Quadtree";
        case BroadphaseKind::SweepAndPrune: return "Sweep and prune";
        case BroadphaseKind::BVH: return "BVH";
        case BroadphaseKind::BruteForce: return "Brute force O(n^2)";
        default: return "?";
    }
}
