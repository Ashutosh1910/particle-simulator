#pragma once
#include "broadphase.h"

// Bounding volume hierarchy built top-down every step: each node splits its
// particles at the median of the longest axis (nth_element), down to leaves of
// kLeafSize. Nodes store the exact union of their children's boxes, so mixed
// sizes are handled naturally (a big particle only enlarges its own branch).
class BVH : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::BVH; }
    void queryImpl(const AABB& box, std::vector<int>& out) const override;
    void debugRects(std::vector<DebugRect>& out, int maxRects) const override;

protected:
    void buildImpl(ThreadPool& pool) override;
    void findPairsImpl(ThreadPool& pool) override;

private:
    static constexpr int kLeafSize = 4;
    struct Node {
        AABB box;
        int left = -1, right = -1;  // children, -1 for leaves
        int begin = 0, end = 0;     // leaf range in items_
        int depth = 0;
    };
    int buildNode(int begin, int end, int depth);
    template <class Visit>
    void visit(const AABB& box, Visit&& fn) const;

    std::vector<Node> nodes_;
    std::vector<int> items_;
    int maxDepth_ = 0;
};
