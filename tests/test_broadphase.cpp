#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>

#include "../src/broadphase/broadphase.h"
#include "../src/broadphase/uniform_grid.h"
#include "test_framework.h"

namespace {

struct Cloud {
    std::vector<float> x, y, e;
    BroadphaseInput input() const { return {x.data(), y.data(), e.data(), (int)x.size()}; }
};

Cloud makeCloud(int n, unsigned seed, int layout) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(0, 1);
    Cloud c;
    for (int i = 0; i < n; i++) {
        float x, y, r;
        switch (layout) {
            case 0:  // uniform, equal sizes
                x = u(rng) * 1000; y = u(rng) * 800; r = 5; break;
            case 1:  // mixed sizes
                x = u(rng) * 1000; y = u(rng) * 800; r = (u(rng) < 0.05f) ? 20 + 30 * u(rng) : 1 + 3 * u(rng); break;
            case 2:  // tight clusters plus far outliers (stress for grids and trees)
                if (u(rng) < 0.02f) { x = -5000 + 10000 * u(rng); y = -5000 + 10000 * u(rng); }
                else { x = 500 + 40 * u(rng); y = 400 + 40 * u(rng); }
                r = 2 + 2 * u(rng); break;
            default:  // many identical positions (degenerate trees)
                x = (float)(i % 7); y = 3; r = 1; break;
        }
        c.x.push_back(x); c.y.push_back(y); c.e.push_back(r);
    }
    return c;
}

std::vector<std::pair<int, int>> sortedPairs(const std::vector<Pair>& pairs) {
    std::vector<std::pair<int, int>> v;
    for (auto p : pairs) v.push_back({p.a, p.b});
    std::sort(v.begin(), v.end());
    return v;
}

}  // namespace

TEST(broadphases_match_brute_force) {
    for (int layout = 0; layout < 4; layout++) {
        for (int threads : {1, 3}) {
            ThreadPool pool(threads);
            Cloud cloud = makeCloud(layout == 3 ? 300 : 3000, 42 + layout, layout);
            auto reference = makeBroadphase(BroadphaseKind::BruteForce);
            reference->build(cloud.input(), pool);
            std::vector<Pair> refPairs;
            reference->findPairs(refPairs, pool);
            auto expected = sortedPairs(refPairs);
            for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
                auto bp = makeBroadphase((BroadphaseKind)k);
                bp->build(cloud.input(), pool);
                std::vector<Pair> pairs;
                bp->findPairs(pairs, pool);
                for (auto p : pairs) CHECK(p.a < p.b);
                auto got = sortedPairs(pairs);
                CHECK_MSG(got == expected, "%s layout %d threads %d: %zu pairs, expected %zu",
                          broadphaseName((BroadphaseKind)k), layout, threads, got.size(), expected.size());
                CHECK(std::adjacent_find(got.begin(), got.end()) == got.end());
                CHECK(bp->stats().pairs == (int)expected.size());
                if ((BroadphaseKind)k != BroadphaseKind::BruteForce && layout < 2)
                    CHECK_MSG(bp->stats().tests < reference->stats().tests / 4, "%s did %lld tests",
                              broadphaseName((BroadphaseKind)k), bp->stats().tests);
            }
        }
    }
}

TEST(broadphase_pair_order_independent_of_threads) {
    Cloud cloud = makeCloud(4000, 7, 1);
    for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
        std::vector<Pair> a, b;
        ThreadPool p1(1), p4(4);
        auto bp = makeBroadphase((BroadphaseKind)k);
        bp->build(cloud.input(), p1);
        bp->findPairs(a, p1);
        bp->build(cloud.input(), p4);
        bp->findPairs(b, p4);
        bool same = a.size() == b.size();
        for (size_t i = 0; same && i < a.size(); i++) same = a[i].a == b[i].a && a[i].b == b[i].b;
        CHECK_MSG(same, "%s", broadphaseName((BroadphaseKind)k));
    }
}

TEST(broadphase_query_aabb_matches_scan) {
    ThreadPool pool(2);
    for (int layout = 0; layout < 3; layout++) {
        Cloud cloud = makeCloud(2000, 99 + layout, layout);
        std::mt19937 rng(5);
        std::uniform_real_distribution<float> u(-100, 1100);
        for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
            auto bp = makeBroadphase((BroadphaseKind)k);
            bp->build(cloud.input(), pool);
            for (int q = 0; q < 50; q++) {
                float x = u(rng), y = u(rng), w = std::abs(u(rng)) * 0.2f, h = std::abs(u(rng)) * 0.2f;
                AABB box{x, y, x + w, y + h};
                std::set<int> expected;
                for (int i = 0; i < (int)cloud.x.size(); i++) {
                    AABB b{cloud.x[i] - cloud.e[i], cloud.y[i] - cloud.e[i], cloud.x[i] + cloud.e[i], cloud.y[i] + cloud.e[i]};
                    if (overlaps(b, box)) expected.insert(i);
                }
                std::vector<int> got;
                bp->queryAABB(box, got);
                std::set<int> gotSet(got.begin(), got.end());
                CHECK_MSG(gotSet == expected && got.size() == gotSet.size(), "%s layout %d query %d",
                          broadphaseName((BroadphaseKind)k), layout, q);
            }
        }
    }
}

TEST(thread_pool_covers_range_once) {
    for (int threads : {1, 2, 3, 8}) {
        ThreadPool pool(threads);
        for (int n : {0, 1, 5, 127, 128, 1000, 12345}) {
            std::vector<int> hits(n, 0);
            pool.parallelFor(n, [&](int, int b, int e) {
                for (int i = b; i < e; i++) hits[i]++;
            }, 1);
            CHECK(std::all_of(hits.begin(), hits.end(), [](int h) { return h == 1; }));
        }
        pool.setThreadCount(threads + 1);
        std::vector<int> hits(1000, 0);
        pool.parallelFor(1000, [&](int, int b, int e) { for (int i = b; i < e; i++) hits[i]++; }, 1);
        CHECK(std::all_of(hits.begin(), hits.end(), [](int h) { return h == 1; }));
    }
}

namespace {
// runs every broad phase and checks it against a direct O(n^2) scan of the input
void checkAgainstScan(const Cloud& cloud, ThreadPool& pool, const char* label) {
    std::vector<std::pair<int, int>> expected;
    auto ok = [&](int i) { return std::isfinite(cloud.x[i]) && std::isfinite(cloud.y[i]) && std::isfinite(cloud.e[i]); };
    int n = (int)cloud.x.size();
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) {
            if (!ok(i) || !ok(j)) continue;
            AABB a{cloud.x[i] - cloud.e[i], cloud.y[i] - cloud.e[i], cloud.x[i] + cloud.e[i], cloud.y[i] + cloud.e[i]};
            AABB b{cloud.x[j] - cloud.e[j], cloud.y[j] - cloud.e[j], cloud.x[j] + cloud.e[j], cloud.y[j] + cloud.e[j]};
            if (overlaps(a, b)) expected.push_back({i, j});
        }
    for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
        auto bp = makeBroadphase((BroadphaseKind)k);
        bp->build(cloud.input(), pool);
        std::vector<Pair> pairs;
        bp->findPairs(pairs, pool);
        auto got = sortedPairs(pairs);
        CHECK_MSG(got == expected, "%s (%s): %zu pairs, expected %zu", broadphaseName((BroadphaseKind)k), label, got.size(),
                  expected.size());
        CHECK(bp->count() == n);
    }
}
}  // namespace

TEST(broadphase_ignores_non_finite_particles) {
    ThreadPool pool(2);
    Cloud c = makeCloud(400, 11, 0);
    float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    c.x[3] = nan;
    c.y[50] = inf;
    c.x[51] = -inf;
    c.e[77] = nan;
    checkAgainstScan(c, pool, "nan/inf");
    Cloud tiny;
    tiny.x = {0, nan, 1, 2};
    tiny.y = {0, 0, 0, 0};
    tiny.e = {1, 1, 1, 1};
    checkAgainstScan(tiny, pool, "tiny nan");
}

TEST(broadphase_large_coordinates) {
    ThreadPool pool(2);
    for (float offset : {1e5f, 1e6f, 3e7f}) {
        std::mt19937 rng(17);
        std::uniform_real_distribution<float> u(0, 1);
        Cloud c;
        for (int i = 0; i < 600; i++) {
            c.x.push_back(offset + u(rng) * 200);
            c.y.push_back(offset + u(rng) * 200);
            c.e.push_back(0.3f + u(rng));
        }
        checkAgainstScan(c, pool, "offset");
    }
}

TEST(broadphase_huge_query_and_empty_inputs) {
    ThreadPool pool(1);
    Cloud c;
    c.x = {0, 100, 200};
    c.y = {0, 0, 0};
    c.e = {1, 1, 1};
    for (int k = 0; k < (int)BroadphaseKind::Count; k++) {
        auto bp = makeBroadphase((BroadphaseKind)k);
        bp->build(c.input(), pool);
        std::vector<int> got;
        bp->queryAABB({-1e20f, -1e20f, 1e20f, 1e20f}, got);
        CHECK_MSG(got.size() == 3, "%s returned %zu of 3", broadphaseName((BroadphaseKind)k), got.size());
        got.clear();
        bp->queryAABB({-5000, -5000, 5000, 5000}, got);  // more cells than particles (hash falls back to a scan)
        CHECK(got.size() == 3);
        for (int n : {0, 1}) {
            Cloud small;
            for (int i = 0; i < n; i++) { small.x.push_back(5); small.y.push_back(5); small.e.push_back(1); }
            bp->build(small.input(), pool);
            std::vector<Pair> pairs;
            bp->findPairs(pairs, pool);
            CHECK(pairs.empty());
            std::vector<int> q;
            bp->queryAABB({0, 0, 10, 10}, q);
            CHECK((int)q.size() == n);
        }
    }
}

TEST(uniform_grid_memory_bounded_with_far_outlier) {
    ThreadPool pool(1);
    Cloud c;
    c.x = {0, 1e13f, 5, 6};
    c.y = {0, 0, 3, 3};
    c.e = {1, 1, 1, 1};
    UniformGrid grid;
    grid.build(c.input(), pool);
    CHECK_MSG((long long)grid.cols() * grid.rows() <= 3 * 4096 + 1, "%d x %d cells", grid.cols(), grid.rows());
    std::vector<Pair> pairs;
    grid.findPairs(pairs, pool);
    CHECK(pairs.size() == 1);
}

TEST(thread_pool_chunks_are_contiguous_and_ordered) {
    ThreadPool pool(4);
    std::vector<int> owner(1000, -1);
    pool.parallelFor(1000, [&](int w, int b, int e) {
        for (int i = b; i < e; i++) owner[i] = w;
    }, 1);
    // worker w must own the w-th contiguous chunk: owners never decrease
    CHECK(std::is_sorted(owner.begin(), owner.end()));
    CHECK(owner.front() == 0 && owner.back() == 3);
}

TEST(one_far_outlier_does_not_make_trees_or_hash_quadratic) {
    ThreadPool pool(1);
    Cloud c = makeCloud(5000, 23, 0);
    long long brute = 5000LL * 4999 / 2;
    for (float far : {1e9f, 1e30f})
    for (BroadphaseKind k : {BroadphaseKind::SpatialHash, BroadphaseKind::Quadtree, BroadphaseKind::BVH,
                             BroadphaseKind::SweepAndPrune}) {
        c.x[0] = far;  // a single particle flung far away
        auto bp = makeBroadphase(k);
        bp->build(c.input(), pool);
        std::vector<Pair> pairs;
        bp->findPairs(pairs, pool);
        CHECK_MSG(bp->stats().tests * 10 < brute, "%s did %lld tests (outlier at %g)", broadphaseName(k), bp->stats().tests, far);
    }
}

TEST(spatial_hash_handles_points_far_from_the_origin) {
    ThreadPool pool(1);
    Cloud c;
    std::mt19937 rng(31);
    std::uniform_real_distribution<float> u(0, 1500);
    for (int i = 0; i < 4000; i++) {
        c.x.push_back(2e6f + u(rng));
        c.y.push_back(2e6f + u(rng));
        c.e.push_back(i % 2 ? 0.0f : 0.5f);
    }
    auto bp = makeBroadphase(BroadphaseKind::SpatialHash);
    bp->build(c.input(), pool);
    std::vector<Pair> pairs;
    bp->findPairs(pairs, pool);
    CHECK_MSG(bp->stats().tests < 100000, "spatial hash did %lld tests", bp->stats().tests);
    checkAgainstScan(c, pool, "far from origin");
}

TEST(one_huge_particle_does_not_make_trees_quadratic) {
    ThreadPool pool(1);
    Cloud c = makeCloud(5000, 29, 0);
    c.e[0] = 300;  // one particle far bigger than the rest
    for (BroadphaseKind k : {BroadphaseKind::Quadtree, BroadphaseKind::BVH}) {
        auto bp = makeBroadphase(k);
        bp->build(c.input(), pool);
        std::vector<Pair> pairs;
        bp->findPairs(pairs, pool);
        CHECK_MSG(bp->stats().tests < 200000, "%s did %lld tests", broadphaseName(k), bp->stats().tests);
    }
    checkAgainstScan(c, pool, "huge particle");
}
