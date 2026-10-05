// Headless benchmark: compares the broad phases on the same particle clouds.
//
//   psim_bench [--n 1000,10000,50000] [--threads 1,4] [--repeat 5]
//
// For every layout, particle count and thread count it reports, per broad
// phase, the AABB tests performed, the candidate pairs found and the average
// build + pair-search time. Particle density is kept constant as n grows.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "../src/broadphase/broadphase.h"

namespace {

std::vector<int> parseList(const char* s) {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(std::atoi(item.c_str()));
    return out;
}

struct Cloud {
    std::vector<float> x, y, e;
};

// layout 0: uniform equal radii, 1: mixed radii (5% large), 2: clustered
Cloud makeCloud(int n, int layout, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(0, 1);
    // ~ 25% area coverage for radius-4 particles
    float side = std::sqrt((float)n * 3.14159f * 16 / 0.25f);
    Cloud c;
    for (int i = 0; i < n; i++) {
        float x = u(rng) * side, y = u(rng) * side, r = 4;
        if (layout == 1) r = u(rng) < 0.05f ? 20 + 20 * u(rng) : 2 + 2 * u(rng);
        if (layout == 2) {
            int cluster = i % 8;
            float cx = side * (0.1f + 0.8f * ((cluster * 37) % 8) / 8.0f), cy = side * (0.1f + 0.8f * ((cluster * 53) % 8) / 8.0f);
            float a = u(rng) * 6.2832f, d = std::sqrt(u(rng)) * side * 0.06f;
            x = cx + std::cos(a) * d;
            y = cy + std::sin(a) * d;
        }
        c.x.push_back(x); c.y.push_back(y); c.e.push_back(r);
    }
    return c;
}

const char* layoutName(int l) { return l == 0 ? "uniform" : l == 1 ? "mixed sizes" : "clustered"; }

}  // namespace

int main(int argc, char** argv) {
    std::vector<int> sizes{1000, 10000, 50000}, threads{1, ThreadPool::hardwareThreads()};
    int repeat = 5;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--n")) sizes = parseList(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--threads")) threads = parseList(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--repeat")) repeat = std::max(1, std::atoi(argv[i + 1]));
    }
    threads.erase(std::unique(threads.begin(), threads.end()), threads.end());

    std::printf("%-12s %7s %3s  %-20s %14s %10s %10s %9s\n", "layout", "n", "thr", "broad phase", "AABB tests",
                "pairs", "ms/pass", "speedup");
    for (int layout = 0; layout < 3; layout++) {
        for (int n : sizes) {
            Cloud cloud = makeCloud(n, layout, 1234 + n);
            BroadphaseInput in{cloud.x.data(), cloud.y.data(), cloud.e.data(), n};
            for (int t : threads) {
                ThreadPool pool(t);
                double bruteMs = -1;
                for (int k = (int)BroadphaseKind::Count - 1; k >= 0; k--) {  // brute force first (baseline)
                    auto kind = (BroadphaseKind)k;
                    bool skip = kind == BroadphaseKind::BruteForce && n > 20000;
                    auto bp = makeBroadphase(kind);
                    std::vector<Pair> pairs;
                    double best = 1e30;
                    for (int r = 0; r < (skip ? 0 : repeat); r++) {
                        pairs.clear();
                        auto t0 = std::chrono::steady_clock::now();
                        bp->build(in, pool);
                        bp->findPairs(pairs, pool);
                        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
                    }
                    if (skip) {
                        std::printf("%-12s %7d %3d  %-20s %14s %10s %10s %9s\n", layoutName(layout), n, t,
                                    broadphaseName(kind), "(skipped)", "", "", "");
                        continue;
                    }
                    if (kind == BroadphaseKind::BruteForce) bruteMs = best;
                    char speedup[32] = "";
                    if (bruteMs > 0) std::snprintf(speedup, sizeof speedup, "%.0fx", bruteMs / best);
                    std::printf("%-12s %7d %3d  %-20s %14lld %10d %10.3f %9s\n", layoutName(layout), n, t,
                                broadphaseName(kind), bp->stats().tests, bp->stats().pairs, best, speedup);
                }
            }
        }
    }
    return 0;
}
