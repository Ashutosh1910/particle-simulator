#include "spatial_hash.h"

#include <cmath>
#include <unordered_set>

int SpatialHash::bucketOf(int cx, int cy) const {
    unsigned h = (unsigned)cx * 92837111u ^ (unsigned)cy * 689287499u;
    return (int)(h % (unsigned)tableSize_);
}

void SpatialHash::buildImpl(ThreadPool& pool) {
    cellSize_ = std::max(2.0f * maxExtent_ * 1.0001f, 1e-3f);
    tableSize_ = std::max(1, 2 * n_ + 1);
    bucketStart_.assign(tableSize_ + 1, 0);
    itemBucket_.resize(n_);
    bucketItems_.resize(n_);
    pool.parallelFor(n_, [&](int, int b, int e) {
        for (int i = b; i < e; i++) itemBucket_[i] = bucketOf(cellCoord(cx_[i]), cellCoord(cy_[i]));
    }, 4096);
    for (int i = 0; i < n_; i++) bucketStart_[itemBucket_[i] + 1]++;
    for (int t = 0; t < tableSize_; t++) bucketStart_[t + 1] += bucketStart_[t];
    std::vector<int> cursor(bucketStart_.begin(), bucketStart_.end() - 1);
    for (int i = 0; i < n_; i++) bucketItems_[cursor[itemBucket_[i]]++] = i;
}

void SpatialHash::findPairsImpl(ThreadPool& pool) {
    pool.parallelFor(n_, [&](int worker, int begin, int end) {
        auto& out = perWorker_[worker];
        long long tests = 0;
        for (int i = begin; i < end; i++) {
            int cx = cellCoord(cx_[i]), cy = cellCoord(cy_[i]);
            // the 9 neighbouring cells may hash to the same bucket: visit each bucket once
            int seen[9], nSeen = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int bkt = bucketOf(cx + dx, cy + dy);
                    bool dup = false;
                    for (int s = 0; s < nSeen; s++) dup |= seen[s] == bkt;
                    if (dup) continue;
                    seen[nSeen++] = bkt;
                    for (int k = bucketStart_[bkt]; k < bucketStart_[bkt + 1]; k++) {
                        int j = bucketItems_[k];
                        if (j <= i) continue;  // each pair is reported by its lower index
                        tests++;
                        if (boxesOverlap(i, j)) out.push_back({i, j});
                    }
                }
        }
        tests_[worker] += tests;
    }, 256);
}

void SpatialHash::queryAABB(const AABB& box, std::vector<int>& out) const {
    if (n_ == 0) return;
    int x0 = cellCoord(std::max(box.minX, bounds_.minX) - maxExtent_);
    int x1 = cellCoord(std::min(box.maxX, bounds_.maxX) + maxExtent_);
    int y0 = cellCoord(std::max(box.minY, bounds_.minY) - maxExtent_);
    int y1 = cellCoord(std::min(box.maxY, bounds_.maxY) + maxExtent_);
    if (x1 < x0 || y1 < y0) return;
    // a huge query box touches more cells than there are particles: just scan
    if ((double)(x1 - x0 + 1) * (y1 - y0 + 1) > n_) {
        for (int i = 0; i < n_; i++)
            if (boxOverlaps(i, box)) out.push_back(i);
        return;
    }
    stamp_.resize(tableSize_, 0);
    if (++stampValue_ == 0) {  // wrapped: reset marks
        std::fill(stamp_.begin(), stamp_.end(), 0u);
        stampValue_ = 1;
    }
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            int bkt = bucketOf(x, y);
            if (stamp_[bkt] == stampValue_) continue;
            stamp_[bkt] = stampValue_;
            for (int k = bucketStart_[bkt]; k < bucketStart_[bkt + 1]; k++)
                if (boxOverlaps(bucketItems_[k], box)) out.push_back(bucketItems_[k]);
        }
}

void SpatialHash::debugRects(std::vector<DebugRect>& out, int maxRects) const {
    // draw the cell of every particle once (cells, not buckets, are what is spatial)
    std::unordered_set<long long> drawn;
    for (int i = 0; i < n_ && (int)out.size() < maxRects; i++) {
        int cx = cellCoord(cx_[i]), cy = cellCoord(cy_[i]);
        long long key = ((long long)cx << 32) ^ (unsigned)cy;
        if (!drawn.insert(key).second) continue;
        out.push_back({cx * cellSize_, cy * cellSize_, (cx + 1) * cellSize_, (cy + 1) * cellSize_, 0});
    }
}
