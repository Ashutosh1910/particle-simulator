#pragma once
#include <cfloat>
#include <memory>
#include <vector>

#include "../core/math2.h"
#include "../core/thread_pool.h"

// A candidate pair produced by the broad phase. Always a < b, never repeated.
struct Pair {
    int a, b;
};

struct BroadphaseStats {
    long long tests = 0;  // AABB-vs-AABB tests performed while finding pairs
    int pairs = 0;        // candidate pairs reported (AABBs overlap)
    double buildMs = 0;
    double queryMs = 0;
};

struct DebugRect {
    float minX, minY, maxX, maxY;
    int depth;  // tree depth, or 0 for flat structures
};

// Every particle i is represented by the square AABB centred on (x[i], y[i]) with
// half-size extent[i]. For hard-sphere collisions extent = radius; for SPH or
// Lennard-Jones it is half the interaction range so that overlapping boxes cover
// every pair closer than the range.
struct BroadphaseInput {
    const float* x = nullptr;
    const float* y = nullptr;
    const float* extent = nullptr;
    int n = 0;
};

enum class BroadphaseKind { UniformGrid, SpatialHash, Quadtree, SweepAndPrune, BVH, BruteForce, Count };

class Broadphase {
public:
    virtual ~Broadphase() = default;
    virtual BroadphaseKind kind() const = 0;

    // Captures the particle boxes and builds the acceleration structure.
    // Particles with a non-finite position or extent are left out entirely.
    void build(const BroadphaseInput& in, ThreadPool& pool);
    // Appends every pair whose boxes overlap (a < b). Uses the boxes from build().
    void findPairs(std::vector<Pair>& out, ThreadPool& pool);
    // Appends every particle whose box overlaps `box` (no duplicates, any order).
    // Not thread-safe: call from one thread at a time.
    void queryAABB(const AABB& box, std::vector<int>& out) const;
    // Cells or tree nodes for the structure overlay.
    virtual void debugRects(std::vector<DebugRect>& out, int maxRects) const { (void)out; (void)maxRects; }

    // Number of particles passed to the last build().
    int count() const { return inputCount_; }
    const BroadphaseStats& stats() const { return stats_; }

protected:
    // Implementations work on the compacted particles 0..n_-1; the base class maps
    // their indices back to the caller's.
    virtual void buildImpl(ThreadPool& pool) = 0;
    // Pairs found by worker w go to out_[w].pairs, tests to out_[w].tests.
    virtual void findPairsImpl(ThreadPool& pool) = 0;
    virtual void queryImpl(const AABB& box, std::vector<int>& out) const = 0;

    bool boxesOverlap(int i, int j) const {
        return minX_[i] <= maxX_[j] && minX_[j] <= maxX_[i] && minY_[i] <= maxY_[j] && minY_[j] <= maxY_[i];
    }
    bool boxOverlaps(int i, const AABB& b) const {
        return minX_[i] <= b.maxX && b.minX <= maxX_[i] && minY_[i] <= b.maxY && b.minY <= maxY_[i];
    }

    int n_ = 0;            // particles in the structure (finite ones)
    int inputCount_ = 0;   // particles passed to build()
    std::vector<int> ids_;  // structure index -> caller index; empty when they are the same
    std::vector<float> cx_, cy_;                    // box centres
    std::vector<float> minX_, minY_, maxX_, maxY_;  // boxes
    float maxExtent_ = 0;
    // Widest stored box side (computed in double). Two overlapping boxes have
    // min corners at most this far apart on each axis, so grid-like structures
    // binning boxes by their min corner with cells at least this wide only need
    // to look at adjacent cells. Unlike a margin based on the largest coordinate,
    // a single far-away particle doesn't inflate it.
    double maxBoxWidth_ = 0;
    AABB bounds_;  // bounding box of all particle boxes

    // one cache line per worker so workers don't false-share vector headers
    struct alignas(64) WorkerOutput {
        std::vector<Pair> pairs;
        long long tests = 0;
    };
    std::vector<WorkerOutput> out_;
    BroadphaseStats stats_;
};

std::unique_ptr<Broadphase> makeBroadphase(BroadphaseKind kind);
const char* broadphaseName(BroadphaseKind kind);
