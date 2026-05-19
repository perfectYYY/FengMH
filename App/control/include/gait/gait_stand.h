/*
 * gait_stand.h — 站立步态：所有腿恒支撑、关节维持给定高度
 */
#ifndef APP_SERVICE_GAIT_STAND_H_
#define APP_SERVICE_GAIT_STAND_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

gait_if_t* gait_stand_create(void);

#ifdef __cplusplus
}
#endif

#endif
