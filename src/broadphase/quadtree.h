#pragma once
#include "broadphase.h"

// Region quadtree over particle centres, rebuilt each step by recursively
// partitioning an index array in place (no per-node allocations). Nodes are
// split until they hold <= kLeafSize particles. Queries treat each node as
// "loose" by maxExtent so a box is found even if its centre is in a neighbour.
// Adapts to uneven density far better than a fixed grid.
class Quadtree : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::Quadtree; }
    void debugRects(std::vector<DebugRect>& out, int maxRects) const override;

protected:
    void queryImpl(const AABB& box, std::vector<int>& out) const override;
    void buildImpl(ThreadPool& pool) override;
    void findPairsImpl(ThreadPool& pool) override;

private:
    static constexpr int kLeafSize = 8;
    // Nodes stop splitting once they are no wider than the widest box (smaller cells
    // can't separate overlapping boxes anyway). The depth cap only guards against
    // degenerate input: 2^-128 of even a 1e30-wide root is below any box size.
    static constexpr int kMaxDepth = 128;
    struct Node {
        AABB box;
        int firstChild;  // -1 for leaves; children are firstChild..firstChild+3
        int begin, end;  // range in items_
        int depth;
    };
    void split(int node);
    template <class Visit>
    void visit(const AABB& box, Visit&& fn) const;

    std::vector<Node> nodes_;
    std::vector<int> items_;
};
