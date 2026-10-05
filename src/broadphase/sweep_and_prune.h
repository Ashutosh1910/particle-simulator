#pragma once
#include "broadphase.h"

// Sort-and-sweep on the x axis: boxes are sorted by their left edge, then each
// box is compared only with the boxes whose left edge lies inside its x
// interval. Works without any cell size, so it is insensitive to mixed radii,
// but degrades when many boxes share the same x range (e.g. a tall column).
class SweepAndPrune : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::SweepAndPrune; }

protected:
    void queryImpl(const AABB& box, std::vector<int>& out) const override;
    void buildImpl(ThreadPool& pool) override;
    void findPairsImpl(ThreadPool& pool) override;

private:
    std::vector<int> order_;      // indices sorted by minX
    std::vector<float> sortedMinX_;
};
