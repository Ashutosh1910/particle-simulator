#pragma once
#include "broadphase.h"

// Uniform grid with cell size >= the largest box, stored as a flat
// counting-sort (cellStart / cellItems) and rebuilt in O(n) every step.
// Each occupied cell is checked against itself and four "forward" neighbours
// (right, down-left, down, down-right) so every adjacent cell pair is visited once.
class UniformGrid : public Broadphase {
public:
    BroadphaseKind kind() const override { return BroadphaseKind::UniformGrid; }
    void queryAABB(const AABB& box, std::vector<int>& out) const override;
    void debugRects(std::vector<DebugRect>& out, int maxRects) const override;

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float cellSize() const { return cellSize_; }

protected:
    void buildImpl(ThreadPool& pool) override;
    void findPairsImpl(ThreadPool& pool) override;

private:
    int cellX(float x) const;
    int cellY(float y) const;

    float cellSize_ = 1;
    float originX_ = 0, originY_ = 0;
    int cols_ = 1, rows_ = 1;
    std::vector<int> cellStart_;  // size cols*rows + 1
    std::vector<int> cellItems_;  // particle indices sorted by cell
    std::vector<int> itemCell_;
};
