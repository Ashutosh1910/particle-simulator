#include "broadphase.h"

#include <chrono>
#include <cmath>

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
    inputCount_ = in.n;
    // NaN/inf would break sorting, cell indexing and tree splits: drop them
    ids_.clear();
    bool allFinite = true;
    for (int i = 0; i < in.n && allFinite; i++)
        allFinite = std::isfinite(in.x[i]) && std::isfinite(in.y[i]) && std::isfinite(in.extent[i]);
    if (!allFinite) {
        for (int i = 0; i < in.n; i++)
            if (std::isfinite(in.x[i]) && std::isfinite(in.y[i]) && std::isfinite(in.extent[i])) ids_.push_back(i);
    }
    n_ = allFinite ? in.n : (int)ids_.size();
    cx_.resize(n_); cy_.resize(n_);
    minX_.resize(n_); minY_.resize(n_); maxX_.resize(n_); maxY_.resize(n_);
    maxExtent_ = 0;
    bounds_ = AABB{0, 0, 0, 0};
    if (n_ > 0) bounds_ = AABB{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (int k = 0; k < n_; k++) {
        int i = allFinite ? k : ids_[k];
        float x = in.x[i], y = in.y[i], e = std::fabs(in.extent[i]);
        cx_[k] = x; cy_[k] = y;
        minX_[k] = x - e; maxX_[k] = x + e;
        minY_[k] = y - e; maxY_[k] = y + e;
        maxExtent_ = std::max(maxExtent_, e);
        bounds_.minX = std::min(bounds_.minX, minX_[k]);
        bounds_.minY = std::min(bounds_.minY, minY_[k]);
        bounds_.maxX = std::max(bounds_.maxX, maxX_[k]);
        bounds_.maxY = std::max(bounds_.maxY, maxY_[k]);
    }
    buildImpl(pool);
    stats_.buildMs = msSince(t0);
}

void Broadphase::findPairs(std::vector<Pair>& out, ThreadPool& pool) {
    auto t0 = std::chrono::steady_clock::now();
    int workers = pool.threadCount();
    out_.resize(workers);
    for (auto& w : out_) {
        w.pairs.clear();
        w.tests = 0;
    }
    if (n_ > 1) findPairsImpl(pool);

    // concatenate in worker order: chunks are contiguous index ranges, so the
    // result is identical for any thread count
    size_t start = out.size(), total = 0;
    for (auto& w : out_) total += w.pairs.size();
    out.reserve(start + total);
    stats_.tests = 0;
    for (auto& w : out_) {
        out.insert(out.end(), w.pairs.begin(), w.pairs.end());
        stats_.tests += w.tests;
    }
    if (!ids_.empty()) {
        // ids_ is increasing, so a < b still holds after mapping
        for (size_t k = start; k < out.size(); k++) out[k] = {ids_[out[k].a], ids_[out[k].b]};
    }
    stats_.pairs = (int)total;
    stats_.queryMs = msSince(t0);
}

void Broadphase::queryAABB(const AABB& box, std::vector<int>& out) const {
    size_t start = out.size();
    if (n_ > 0) queryImpl(box, out);
    if (!ids_.empty())
        for (size_t k = start; k < out.size(); k++) out[k] = ids_[out[k]];
}

float cellSizeFor(float maxExtent, const AABB& bounds) {
    // Overlapping boxes have centres at most 2*maxExtent apart. Each stored box
    // edge is rounded by up to half an ulp, so add a few ulps of the largest
    // coordinate on top of the relative margin.
    float maxAbs = std::max({std::fabs(bounds.minX), std::fabs(bounds.maxX), std::fabs(bounds.minY), std::fabs(bounds.maxY)});
    return std::max(2.0f * maxExtent * 1.0001f + 8.0f * FLT_EPSILON * maxAbs, 1e-3f);
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
