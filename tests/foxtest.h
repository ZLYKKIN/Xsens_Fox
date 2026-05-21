// Minimal dependency-free test harness (no GoogleTest / QTest needed).
// Each test executable is a single translation unit that includes this header,
// runs its checks, and returns non-zero if any failed (picked up by CTest).
#pragma once

#include <cmath>
#include <cstdio>

static int fox_checks   = 0;
static int fox_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++fox_checks;                                                          \
        if (!(cond)) {                                                         \
            ++fox_failures;                                                    \
            std::fprintf(stderr, "  FAIL %s:%d: CHECK(%s)\n",                  \
                         __FILE__, __LINE__, #cond);                           \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                  \
    do {                                                                       \
        ++fox_checks;                                                          \
        const double _a = (a), _b = (b), _e = (eps);                          \
        if (!(std::fabs(_a - _b) <= _e)) {                                     \
            ++fox_failures;                                                    \
            std::fprintf(stderr,                                               \
                         "  FAIL %s:%d: |%.9g - %.9g| > %.3g  (%s vs %s)\n",   \
                         __FILE__, __LINE__, _a, _b, _e, #a, #b);              \
        }                                                                      \
    } while (0)

#define RUN(fn)                                                                \
    do {                                                                       \
        std::printf("[ RUN      ] %s\n", #fn);                                 \
        fn();                                                                  \
    } while (0)

static int fox_report(const char* suite)
{
    std::printf("[==========] %s: %d checks, %d failed\n",
                suite, fox_checks, fox_failures);
    return fox_failures == 0 ? 0 : 1;
}
