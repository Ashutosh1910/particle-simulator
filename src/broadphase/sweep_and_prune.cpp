#include "sweep_and_prune.h"

#include <algorithm>
#include <numeric>

void SweepAndPrune::buildImpl(ThreadPool&) {
    order_.resize(n_);
    std::iota(order_.begin(), order_.end(), 0);
    // ties broken by index so the order (and the pair list) is deterministic
    std::sort(order_.begin(), order_.end(), [&](int a, int b) {
        return minX_[a] < minX_[b] || (minX_[a] == minX_[b] && a < b);
    });
    sortedMinX_.resize(n_);
    for (int k = 0; k < n_; k++) sortedMinX_[k] = minX_[order_[k]];
}

void SweepAndPrune::findPairsImpl(ThreadPool& pool) {
    // each start position sweeps independently, so the sweep parallelises over k
    pool.parallelFor(n_, [&](int worker, int begin, int end) {
        auto& out = perWorker_[worker];
        long long tests = 0;
        for (int k = begin; k < end; k++) {
            int i = order_[k];
            float right = maxX_[i];
            for (int m = k + 1; m < n_ && sortedMinX_[m] <= right; m++) {
                int j = order_[m];
                tests++;
                if (boxesOverlap(i, j)) out.push_back(i < j ? Pair{i, j} : Pair{j, i});
            }
        }
        tests_[worker] += tests;
    }, 256);
}

void SweepAndPrune::queryAABB(const AABB& box, std::vector<int>& out) const {
    // boxes starting right of the query cannot overlap it; boxes starting left of
    // it can only overlap if they start within maxExtent*2 of its left edge
    auto first = std::lower_bound(sortedMinX_.begin(), sortedMinX_.end(), box.minX - 2.0002f * maxExtent_ - 1e-3f);
    for (auto it = first; it != sortedMinX_.end() && *it <= box.maxX; ++it) {
        int i = order_[it - sortedMinX_.begin()];
        if (boxOverlaps(i, box)) out.push_back(i);
    }
}
