#include <chrono>
#include <cstring>

#include "test_framework.h"

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    for (auto& t : testRegistry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        int before = testFailures();
        auto t0 = std::chrono::steady_clock::now();
        std::printf("[ RUN  ] %s\n", t.name);
        t.fn();
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("[ %s ] %s (%.0f ms)\n", testFailures() == before ? " OK " : "FAIL", t.name, ms);
        run++;
    }
    std::printf("\n%d tests, %d failed checks\n", run, testFailures());
    return testFailures() == 0 ? 0 : 1;
}
