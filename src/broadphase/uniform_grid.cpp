#include "uniform_grid.h"

#include <cmath>

int UniformGrid::cellX(float x) const {
    return std::clamp((int)std::floor((x - originX_) / cellSize_), 0, cols_ - 1);
}
int UniformGrid::cellY(float y) const {
    return std::clamp((int)std::floor((y - originY_) / cellSize_), 0, rows_ - 1);
}

void UniformGrid::buildImpl(ThreadPool& pool) {
    // Two boxes can only overlap if their centres are at most 2*maxExtent apart,
    // so with this cell size overlapping boxes always sit in the same or adjacent
    // cells. The small factor keeps that true under float rounding.
    cellSize_ = std::max(2.0f * maxExtent_ * 1.0001f, 1e-3f);
    originX_ = bounds_.minX;
    originY_ = bounds_.minY;
    float w = std::max(bounds_.width(), cellSize_);
    float h = std::max(bounds_.height(), cellSize_);
    // Cap the number of cells so a few far-away particles can't explode memory.
    const double maxCells = std::max(4096.0, 4.0 * n_);
    if ((double)(w / cellSize_) * (h / cellSize_) > maxCells) cellSize_ = std::sqrt(w * h / (float)maxCells) * 1.0001f;
    cols_ = std::max(1, (int)std::ceil(w / cellSize_));
    rows_ = std::max(1, (int)std::ceil(h / cellSize_));

    int cells = cols_ * rows_;
    cellStart_.assign(cells + 1, 0);
    itemCell_.resize(n_);
    cellItems_.resize(n_);
    pool.parallelFor(n_, [&](int, int b, int e) {
        for (int i = b; i < e; i++) itemCell_[i] = cellY(cy_[i]) * cols_ + cellX(cx_[i]);
    }, 4096);
    for (int i = 0; i < n_; i++) cellStart_[itemCell_[i] + 1]++;
    for (int c = 0; c < cells; c++) cellStart_[c + 1] += cellStart_[c];
    // scatter: cursor reuses a scratch copy of the start offsets
    std::vector<int> cursor(cellStart_.begin(), cellStart_.end() - 1);
    for (int i = 0; i < n_; i++) cellItems_[cursor[itemCell_[i]]++] = i;
}

void UniformGrid::findPairsImpl(ThreadPool& pool) {
    pool.parallelFor(rows_, [&](int worker, int rowBegin, int rowEnd) {
        auto& out = perWorker_[worker];
        long long tests = 0;
        auto cellVsCell = [&](int a, int b) {
            for (int k = cellStart_[a]; k < cellStart_[a + 1]; k++) {
                int i = cellItems_[k];
                int start = (a == b) ? k + 1 : cellStart_[b];
                for (int m = start; m < cellStart_[b + 1]; m++) {
                    int j = cellItems_[m];
                    tests++;
                    if (boxesOverlap(i, j)) out.push_back(i < j ? Pair{i, j} : Pair{j, i});
                }
            }
        };
        for (int y = rowBegin; y < rowEnd; y++) {
            for (int x = 0; x < cols_; x++) {
                int c = y * cols_ + x;
                if (cellStart_[c] == cellStart_[c + 1]) continue;
                cellVsCell(c, c);
                if (x + 1 < cols_) cellVsCell(c, c + 1);
                if (y + 1 < rows_) {
                    if (x > 0) cellVsCell(c, c + cols_ - 1);
                    cellVsCell(c, c + cols_);
                    if (x + 1 < cols_) cellVsCell(c, c + cols_ + 1);
                }
            }
        }
        tests_[worker] += tests;
    }, 1);
}

void UniformGrid::queryAABB(const AABB& box, std::vector<int>& out) const {
    if (n_ == 0) return;
    int x0 = cellX(box.minX - maxExtent_), x1 = cellX(box.maxX + maxExtent_);
    int y0 = cellY(box.minY - maxExtent_), y1 = cellY(box.maxY + maxExtent_);
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            int c = y * cols_ + x;
            for (int k = cellStart_[c]; k < cellStart_[c + 1]; k++)
                if (boxOverlaps(cellItems_[k], box)) out.push_back(cellItems_[k]);
        }
}

void UniformGrid::debugRects(std::vector<DebugRect>& out, int maxRects) const {
    for (int c = 0; c < cols_ * rows_ && (int)out.size() < maxRects; c++) {
        if (cellStart_[c] == cellStart_[c + 1]) continue;
        float x = originX_ + (c % cols_) * cellSize_, y = originY_ + (c / cols_) * cellSize_;
        out.push_back({x, y, x + cellSize_, y + cellSize_, 0});
    }
}
