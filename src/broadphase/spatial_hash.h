#pragma once
#include "broadphase.h"

// Dense spatial hash: unbounded integer cells (cell size >= largest box) are
// hashed into a table of ~2n buckets, then particles are counting-sorted by
// bucket. Unlike the uniform grid it needs no world bounds and its memory is
// O(n) however spread out the particles are; the price is hash collisions,
// which show up as extra (rejected) tests.
class SpatialHash : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::SpatialHash; }
    void queryAABB(const AABB& box, std::vector<int>& out) const override;
    void debugRects(std::vector<DebugRect>& out, int maxRects) const override;

protected:
    void buildImpl(ThreadPool& pool) override;
    void findPairsImpl(ThreadPool& pool) override;

private:
    int cellCoord(float v) const { return (int)std::floor(v / cellSize_); }
    int bucketOf(int cx, int cy) const;

    float cellSize_ = 1;
    int tableSize_ = 1;
    std::vector<int> bucketStart_;  // tableSize + 1
    std::vector<int> bucketItems_;
    std::vector<int> itemBucket_;
    mutable std::vector<unsigned> stamp_;  // per-bucket visit marks for queryAABB
    mutable unsigned stampValue_ = 0;
};
