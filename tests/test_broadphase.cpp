#include <algorithm>
#include <random>
#include <set>

#include "../src/broadphase/broadphase.h"
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
