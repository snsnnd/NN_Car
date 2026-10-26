#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <math.h>
#include <stdio.h>

static int g_failures = 0;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                \
            g_failures += 1;                                                        \
        }                                                                           \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                           \
    do {                                                                            \
        double a_ = (double)(actual);                                               \
        double e_ = (double)(expected);                                             \
        if (fabs(a_ - e_) > (double)(tol)) {                                        \
            printf("  FAIL %s:%d: %s=%.9g expected %.9g (tol %.3g)\n", __FILE__,     \
                   __LINE__, #actual, a_, e_, (double)(tol));                       \
            g_failures += 1;                                                        \
        }                                                                           \
    } while (0)

#define CHECK_TRUE(cond) CHECK(cond)

#define TEST_END()                                                                  \
    do {                                                                            \
        if (g_failures == 0) {                                                      \
            printf("  ok\n");                                                       \
        }                                                                           \
        return (g_failures == 0) ? 0 : 1;                                           \
    } while (0)

#endif
