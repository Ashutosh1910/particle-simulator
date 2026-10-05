#pragma once
// Tiny self-contained test harness (no external dependencies).
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& testRegistry() {
    static std::vector<TestCase> tests;
    return tests;
}
inline int& testFailures() {
    static int failures = 0;
    return failures;
}

struct TestRegistrar {
    TestRegistrar(const char* name, std::function<void()> fn) { testRegistry().push_back({name, std::move(fn)}); }
};

#define TEST_CAT2(a, b) a##b
#define TEST_CAT(a, b) TEST_CAT2(a, b)
#define TEST(name)                                                                  \
    static void name();                                                             \
    static TestRegistrar TEST_CAT(registrar_, name)(#name, name);                   \
    static void name()

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::printf("    FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            testFailures()++;                                                       \
        }                                                                           \
    } while (0)

#define CHECK_MSG(cond, ...)                                                        \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::printf("    FAILED %s:%d: %s -- ", __FILE__, __LINE__, #cond);     \
            std::printf(__VA_ARGS__);                                               \
            std::printf("\n");                                                      \
            testFailures()++;                                                       \
        }                                                                           \
    } while (0)
