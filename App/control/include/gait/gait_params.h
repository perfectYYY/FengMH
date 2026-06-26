/*
 * gait_params.h — 出厂默认步态参数
 */
#ifndef APP_SERVICE_GAIT_PARAMS_H_
#define APP_SERVICE_GAIT_PARAMS_H_

#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* trot 默认参数：FL/RR 同相，FR/RL 同相 */
extern const gait_params_t GAIT_PARAMS_TROT_DEFAULT;
/* walk 默认参数：FL -> RR -> FR -> RL 四拍，三支撑一摆动 */
extern const gait_params_t GAIT_PARAMS_WALK_DEFAULT;
/* stand 默认参数：duty=1，所有腿恒支撑 */
extern const gait_params_t GAIT_PARAMS_STAND_DEFAULT;

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_GAIT_PARAMS_H_ */
