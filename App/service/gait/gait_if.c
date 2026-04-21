/*
 * gait_if.c — 公共工具
 */
#include "gait_if.h"
#include <math.h>

float gait_wrap01(float x) {
    /* 处理任意大正负数 */
    float w = x - floorf(x);
    if (w < 0.0f) w += 1.0f;
    if (w >= 1.0f) w -= 1.0f;
    return w;
}
