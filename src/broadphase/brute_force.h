#pragma once
#include "broadphase.h"

// Reference O(n^2) broad phase: tests every pair. Used as the ground truth in
// tests and as the baseline in benchmarks.
class BruteForce : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::BruteForce; }
    void queryImpl(const AABB& box, std::vector<int>& out) const override {
        for (int i = 0; i < n_; i++)
            if (boxOverlaps(i, box)) out.push_back(i);
    }

protected:
    void buildImpl(ThreadPool&) override {}
    void findPairsImpl(ThreadPool& pool) override {
        pool.parallelFor(n_, [&](int worker, int begin, int end) {
            auto& out = out_[worker].pairs;
            long long tests = 0;
            for (int i = begin; i < end; i++)
                for (int j = i + 1; j < n_; j++) {
                    tests++;
                    if (boxesOverlap(i, j)) out.push_back({i, j});
                }
            out_[worker].tests += tests;
        }, 64);
    }
};
