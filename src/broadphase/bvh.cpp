#include "bvh.h"

#include <algorithm>
#include <numeric>

void BVH::buildImpl(ThreadPool&) {
    nodes_.clear();
    items_.resize(n_);
    std::iota(items_.begin(), items_.end(), 0);
    maxDepth_ = 0;
    if (n_ == 0) return;
    nodes_.reserve(2 * (n_ / kLeafSize + 1));
    buildNode(0, n_, 0);
}

int BVH::buildNode(int begin, int end, int depth) {
    int idx = (int)nodes_.size();
    nodes_.emplace_back();
    maxDepth_ = std::max(maxDepth_, depth);
    AABB box{1e30f, 1e30f, -1e30f, -1e30f};
    AABB centres{1e30f, 1e30f, -1e30f, -1e30f};
    for (int k = begin; k < end; k++) {
        int i = items_[k];
        box.minX = std::min(box.minX, minX_[i]); box.maxX = std::max(box.maxX, maxX_[i]);
        box.minY = std::min(box.minY, minY_[i]); box.maxY = std::max(box.maxY, maxY_[i]);
        centres.minX = std::min(centres.minX, cx_[i]); centres.maxX = std::max(centres.maxX, cx_[i]);
        centres.minY = std::min(centres.minY, cy_[i]); centres.maxY = std::max(centres.maxY, cy_[i]);
    }
    nodes_[idx].box = box;
    nodes_[idx].depth = depth;
    nodes_[idx].begin = begin;
    nodes_[idx].end = end;
    if (end - begin <= kLeafSize) return idx;

    bool splitX = centres.width() >= centres.height();
    const std::vector<float>& key = splitX ? cx_ : cy_;
    int mid = begin + (end - begin) / 2;
    std::nth_element(items_.begin() + begin, items_.begin() + mid, items_.begin() + end,
                     [&](int a, int b) { return key[a] < key[b] || (key[a] == key[b] && a < b); });
    int left = buildNode(begin, mid, depth + 1);
    int right = buildNode(mid, end, depth + 1);
    nodes_[idx].left = left;  // nodes_ may have reallocated: index, don't hold references
    nodes_[idx].right = right;
    return idx;
}

template <class Visit>
void BVH::visit(const AABB& box, Visit&& fn) const {
    if (nodes_.empty()) return;
    int stack[128];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (!overlaps(node.box, box)) continue;
        if (node.left < 0) {
            for (int k = node.begin; k < node.end; k++) fn(items_[k]);
        } else {
            stack[top++] = node.right;
            stack[top++] = node.left;
        }
    }
}

void BVH::findPairsImpl(ThreadPool& pool) {
    pool.parallelFor(n_, [&](int worker, int begin, int end) {
        auto& out = perWorker_[worker];
        long long tests = 0;
        for (int i = begin; i < end; i++) {
            AABB box{minX_[i], minY_[i], maxX_[i], maxY_[i]};
            visit(box, [&](int j) {
                if (j > i) {
                    tests++;
                    if (boxesOverlap(i, j)) out.push_back({i, j});
                }
            });
        }
        tests_[worker] += tests;
    }, 256);
}

void BVH::queryAABB(const AABB& box, std::vector<int>& out) const {
    visit(box, [&](int j) {
        if (boxOverlaps(j, box)) out.push_back(j);
    });
}

void BVH::debugRects(std::vector<DebugRect>& out, int maxRects) const {
    // breadth-first so the coarse levels are drawn first when capped
    std::vector<int> queue{0};
    for (size_t q = 0; q < queue.size() && !nodes_.empty() && (int)out.size() < maxRects; q++) {
        const Node& n = nodes_[queue[q]];
        out.push_back({n.box.minX, n.box.minY, n.box.maxX, n.box.maxY, n.depth});
        if (n.left >= 0) {
            queue.push_back(n.left);
            queue.push_back(n.right);
        }
    }
}
