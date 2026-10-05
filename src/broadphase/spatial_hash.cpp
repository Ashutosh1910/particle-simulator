#include "spatial_hash.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

int SpatialHash::bucketOf(long long cx, long long cy) const {
    unsigned long long h = (unsigned long long)cx * 92837111ull ^ (unsigned long long)cy * 689287499ull;
    h ^= h >> 29;
    return (int)(h % (unsigned long long)tableSize_);
}

void SpatialHash::buildImpl(ThreadPool& pool) {
    cellSize_ = std::max(maxBoxWidth_ * (1 + 1e-6), 1e-3);  // see Broadphase::maxBoxWidth_
    tableSize_ = std::max(1, 2 * n_ + 1);
    bucketStart_.assign(tableSize_ + 1, 0);
    itemBucket_.resize(n_);
    bucketItems_.resize(n_);
    pool.parallelFor(n_, [&](int, int b, int e) {
        for (int i = b; i < e; i++) itemBucket_[i] = bucketOf(cellCoord(minX_[i]), cellCoord(minY_[i]));
    }, 4096);
    for (int i = 0; i < n_; i++) bucketStart_[itemBucket_[i] + 1]++;
    for (int t = 0; t < tableSize_; t++) bucketStart_[t + 1] += bucketStart_[t];
    std::vector<int> cursor(bucketStart_.begin(), bucketStart_.end() - 1);
    for (int i = 0; i < n_; i++) bucketItems_[cursor[itemBucket_[i]]++] = i;
}

void SpatialHash::findPairsImpl(ThreadPool& pool) {
    pool.parallelFor(n_, [&](int worker, int begin, int end) {
        auto& out = out_[worker].pairs;
        long long tests = 0;
        for (int i = begin; i < end; i++) {
            long long cx = cellCoord(minX_[i]), cy = cellCoord(minY_[i]);
            // the 9 neighbouring cells may hash to the same bucket: visit each bucket once
            int seen[9], nSeen = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int bkt = bucketOf(cx + dx, cy + dy);
                    bool dup = false;
                    for (int s = 0; s < nSeen; s++) dup |= seen[s] == bkt;
                    if (dup) continue;
                    seen[nSeen++] = bkt;
                    // items in a bucket are in increasing index order (stable counting
                    // sort); each pair is reported by its lower index, so start after i
                    const int* first = bucketItems_.data() + bucketStart_[bkt];
                    const int* last = bucketItems_.data() + bucketStart_[bkt + 1];
                    for (const int* it = std::upper_bound(first, last, i); it != last; ++it) {
                        int j = *it;
                        tests++;
                        if (boxesOverlap(i, j)) out.push_back({i, j});
                    }
                }
        }
        out_[worker].tests += tests;
    }, 256);
}

void SpatialHash::queryImpl(const AABB& box, std::vector<int>& out) const {
    if (n_ == 0) return;
    // a box overlapping the query has its min corner in [query.min - maxWidth, query.max]
    long long x0 = cellCoord(std::max((double)box.minX, (double)bounds_.minX) - maxBoxWidth_);
    long long x1 = cellCoord((double)std::min(box.maxX, bounds_.maxX));
    long long y0 = cellCoord(std::max((double)box.minY, (double)bounds_.minY) - maxBoxWidth_);
    long long y1 = cellCoord((double)std::min(box.maxY, bounds_.maxY));
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
    for (long long y = y0; y <= y1; y++)
        for (long long x = x0; x <= x1; x++) {
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
        long long cx = cellCoord(minX_[i]), cy = cellCoord(minY_[i]);
        long long key = cx * 1000003LL ^ cy;
        if (!drawn.insert(key).second) continue;
        out.push_back({(float)(cx * cellSize_), (float)(cy * cellSize_), (float)((cx + 1) * cellSize_),
                       (float)((cy + 1) * cellSize_), 0});
    }
}
