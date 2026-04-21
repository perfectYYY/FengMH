/*
 * test_util.h — Unity 风格的极简断言宏（M1 阶段内置，避免第三方源码引入）
 *
 * 等 M2 阶段引入完整 Unity 后，本文件改为 #include "unity.h" 即可无侵入替换。
 */
#ifndef APP_TEST_UTIL_H_
#define APP_TEST_UTIL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char* name;
    void (*fn)(void);
} tu_case_t;

extern int   g_tu_fail;
extern int   g_tu_pass;
extern const char* g_tu_case;

#define TEST_FAIL_AT(msg) do {                                             \
    fprintf(stderr, "  [FAIL] %s:%d  %s :: %s\n",                          \
            __FILE__, __LINE__, g_tu_case, (msg));                         \
    g_tu_fail++; return;                                                   \
} while (0)

#define TEST_ASSERT(cond) do {                                             \
    if (!(cond)) TEST_FAIL_AT("TEST_ASSERT(" #cond ")");                   \
} while (0)

#define TEST_ASSERT_TRUE(c)   TEST_ASSERT(c)
#define TEST_ASSERT_FALSE(c)  TEST_ASSERT(!(c))

#define TEST_ASSERT_EQUAL_INT(exp, got) do {                               \
    long _e = (long)(exp), _g = (long)(got);                               \
    if (_e != _g) {                                                        \
        char _b[128];                                                      \
        snprintf(_b, sizeof _b, "EQUAL_INT exp=%ld got=%ld", _e, _g);      \
        TEST_FAIL_AT(_b);                                                  \
    }                                                                      \
} while (0)

#define TEST_ASSERT_EQUAL_UINT(exp, got) do {                              \
    unsigned long _e = (unsigned long)(exp), _g = (unsigned long)(got);    \
    if (_e != _g) {                                                        \
        char _b[128];                                                      \
        snprintf(_b, sizeof _b, "EQUAL_UINT exp=%lu got=%lu", _e, _g);     \
        TEST_FAIL_AT(_b);                                                  \
    }                                                                      \
} while (0)

#define TEST_ASSERT_EQUAL_HEX8(exp, got) do {                              \
    unsigned _e = (unsigned)(exp) & 0xFFu, _g = (unsigned)(got) & 0xFFu;   \
    if (_e != _g) {                                                        \
        char _b[128];                                                      \
        snprintf(_b, sizeof _b, "EQUAL_HEX8 exp=0x%02X got=0x%02X",_e,_g); \
        TEST_FAIL_AT(_b);                                                  \
    }                                                                      \
} while (0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, exp, got) do {                     \
    float _e = (float)(exp), _g = (float)(got), _d = (float)(delta);       \
    float _diff = _g - _e; if (_diff < 0) _diff = -_diff;                  \
    if (_diff > _d) {                                                      \
        char _b[160];                                                      \
        snprintf(_b, sizeof _b,                                            \
          "FLOAT_WITHIN |got-exp|=%g > %g (exp=%g got=%g)",                \
          _diff, _d, _e, _g);                                              \
        TEST_FAIL_AT(_b);                                                  \
    }                                                                      \
} while (0)

#define TEST_ASSERT_NULL(p)     TEST_ASSERT((p) == NULL)
#define TEST_ASSERT_NOT_NULL(p) TEST_ASSERT((p) != NULL)

#define TU_RUN(fn_)  do { g_tu_case = #fn_; fn_(); g_tu_pass++; } while (0)

/* 只需每个测试 bin 在 main() 里定义一次 */
#define TU_MAIN_EPILOGUE()                                                 \
    fprintf(stdout, "\n==== %d cases, pass=%d fail=%d ====\n",             \
            g_tu_pass + g_tu_fail, g_tu_pass - g_tu_fail, g_tu_fail);      \
    return (g_tu_fail == 0) ? 0 : 1;

#ifdef __cplusplus
}
#endif

#endif /* APP_TEST_UTIL_H_ */
