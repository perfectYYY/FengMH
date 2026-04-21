/*
 * leg_controller.c
 */
#include "leg_controller.h"
#include "log.h"
#include <string.h>

static const char* TAG = "LEG";

static const motor_logical_id_t HIP[GAIT_LEG_NUM]   = { MOTOR_ID_FL_HIP, MOTOR_ID_FR_HIP, MOTOR_ID_RL_HIP, MOTOR_ID_RR_HIP };
static const motor_logical_id_t KNEE[GAIT_LEG_NUM]  = { MOTOR_ID_FL_KNEE,MOTOR_ID_FR_KNEE,MOTOR_ID_RL_KNEE,MOTOR_ID_RR_KNEE };
static const motor_logical_id_t WHEEL[GAIT_LEG_NUM] = { MOTOR_ID_FL_WHEEL,MOTOR_ID_FR_WHEEL,MOTOR_ID_RL_WHEEL,MOTOR_ID_RR_WHEEL };

void leg_controller_init(leg_controller_t* lc) {
    if (!lc) return;
    memset(lc, 0, sizeof(*lc));
}

app_err_t leg_controller_bind_from_registry(leg_controller_t* lc) {
    if (!lc) return APP_ERR_INVALID_ARG;
    int bound = 0;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        lc->leg[i].hip   = motor_get(HIP[i]);
        lc->leg[i].knee  = motor_get(KNEE[i]);
        lc->leg[i].wheel = motor_get(WHEEL[i]);
        bound += (lc->leg[i].hip ? 1 : 0)
              +  (lc->leg[i].knee ? 1 : 0)
              +  (lc->leg[i].wheel ? 1 : 0);
    }
    LOGI("bind from registry: %d/12 actuators present", bound);
    return APP_OK;
}

static int try_set_pos(motor_dev_t* d, float rad) {
    if (!d || !d->ops || !d->ops->set_position) return APP_ERR_UNSUPPORTED;
    /* M2 暂用低增益，避免误装阶段乱动 */
    return d->ops->set_position(d, rad, 0.0f, 1.5f, 0.1f, 0.0f);
}
static int try_set_vel(motor_dev_t* d, float v) {
    if (!d || !d->ops || !d->ops->set_velocity) return APP_ERR_UNSUPPORTED;
    return d->ops->set_velocity(d, v);
}

app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o) {
    if (!lc || !o) return APP_ERR_INVALID_ARG;
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        const gait_leg_target_t* t = &o->leg[i];
        if (!lc->leg[i].hip || !lc->leg[i].knee || !lc->leg[i].wheel) {
            lc->miss_cnt++;
            continue;
        }
        try_set_pos(lc->leg[i].hip,   t->hip_rad);
        try_set_pos(lc->leg[i].knee,  t->knee_rad);
        try_set_vel(lc->leg[i].wheel, t->wheel_rads);
        lc->send_cnt++;
    }
    return APP_OK;
}
