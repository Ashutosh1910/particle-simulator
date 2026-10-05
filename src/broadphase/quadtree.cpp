#include "quadtree.h"

#include <algorithm>
#include <numeric>

void Quadtree::buildImpl(ThreadPool&) {
    nodes_.clear();
    items_.resize(n_);
    std::iota(items_.begin(), items_.end(), 0);
    if (n_ == 0) return;
    // square root cell around all centres
    float size = std::max({bounds_.width(), bounds_.height(), 1e-3f});
    AABB root{bounds_.minX, bounds_.minY, bounds_.minX + size, bounds_.minY + size};
    nodes_.push_back({root, -1, 0, n_, 0});
    // breadth-first so nodes_ can grow while we iterate
    for (size_t i = 0; i < nodes_.size(); i++) split((int)i);
}

void Quadtree::split(int idx) {
    Node node = nodes_[idx];
    if (node.end - node.begin <= kLeafSize || node.depth >= kMaxDepth) return;
    float mx = 0.5f * (node.box.minX + node.box.maxX), my = 0.5f * (node.box.minY + node.box.maxY);
    auto first = items_.begin() + node.begin, last = items_.begin() + node.end;
    auto midY = std::partition(first, last, [&](int i) { return cy_[i] < my; });
    auto midTop = std::partition(first, midY, [&](int i) { return cx_[i] < mx; });
    auto midBottom = std::partition(midY, last, [&](int i) { return cx_[i] < mx; });
    int b0 = node.begin, b1 = (int)(midTop - items_.begin()), b2 = (int)(midY - items_.begin()),
        b3 = (int)(midBottom - items_.begin()), b4 = node.end;
    const AABB& r = node.box;
    int child = (int)nodes_.size();
    nodes_[idx].firstChild = child;
    int d = node.depth + 1;
    nodes_.push_back({{r.minX, r.minY, mx, my}, -1, b0, b1, d});
    nodes_.push_back({{mx, r.minY, r.maxX, my}, -1, b1, b2, d});
    nodes_.push_back({{r.minX, my, mx, r.maxY}, -1, b2, b3, d});
    nodes_.push_back({{mx, my, r.maxX, r.maxY}, -1, b3, b4, d});
}

template <class Visit>
void Quadtree::visit(const AABB& box, Visit&& fn) const {
    if (nodes_.empty()) return;
    // a box overlapping the query has its centre within maxExtent of the query
    AABB q{box.minX - maxExtent_, box.minY - maxExtent_, box.maxX + maxExtent_, box.maxY + maxExtent_};
    int stack[4 * kMaxDepth + 8];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (node.begin == node.end || !overlaps(node.box, q)) continue;
        if (node.firstChild < 0) {
            for (int k = node.begin; k < node.end; k++) fn(items_[k]);
        } else {
            for (int c = 3; c >= 0; c--) stack[top++] = node.firstChild + c;
        }
    }
}

void Quadtree::findPairsImpl(ThreadPool& pool) {
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

void Quadtree::queryAABB(const AABB& box, std::vector<int>& out) const {
    visit(box, [&](int j) {
        if (boxOverlaps(j, box)) out.push_back(j);
    });
}

void Quadtree::debugRects(std::vector<DebugRect>& out, int maxRects) const {
    for (const Node& n : nodes_) {
        if ((int)out.size() >= maxRects) break;
        if (n.begin == n.end) continue;
        out.push_back({n.box.minX, n.box.minY, n.box.maxX, n.box.maxY, n.depth});
    }
}
