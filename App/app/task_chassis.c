/*
 * task_chassis.c — 把 task_comm 的 chassis 指令翻译为步态/腿控
 *
 * M2 阶段：
 *   - 维护 gait_machine（默认 stand）
 *   - 收到 |vx|+|vy|+|wz| > eps 时切到 trot；否则切到 stand
 *   - 500Hz 节拍调用 leg_controller_apply
 */
#include "task_chassis.h"
#include "log.h"
#include "config.h"
#include "task_comm.h"
#include "task_safety.h"
#include "gait_if.h"
#include "gait_stand.h"
#include "gait_trot.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "leg_controller.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <math.h>

static const char* TAG = "CHASSIS";

static gait_machine_t  s_gm;
static leg_controller_t s_lc;
static gait_if_t* s_stand;
static gait_if_t* s_trot;

void task_chassis_init(void) {
    s_stand = gait_stand_create();
    s_trot  = gait_trot_create();
    gait_machine_init(&s_gm);
    gait_machine_set(&s_gm, s_stand, &GAIT_PARAMS_STAND_DEFAULT);
    leg_controller_init(&s_lc);
    leg_controller_bind_from_registry(&s_lc);
    LOGI("chassis init done");
}

static void try_switch(float v_norm) {
    if (v_norm > 0.05f) {
        if (s_gm.current != s_trot) {
            gait_machine_request(&s_gm, s_trot, &GAIT_PARAMS_TROT_DEFAULT, 0.3f);
        }
    } else {
        if (s_gm.current != s_stand) {
            gait_machine_request(&s_gm, s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f);
        }
    }
}

void task_chassis_entry(void* arg) {
    (void)arg;
    LOGI("task_chassis started");
#if APP_TARGET_MCU
    const float dt = 0.002f; /* 500Hz */
    gait_output_t out;
    for (;;) {
        if (task_safety_estop_active()) {
            osDelay(20);
            continue;
        }
        task_comm_chassis_cmd_t cmd;
        task_comm_get_chassis(&cmd);
        float n = fabsf(cmd.vx) + fabsf(cmd.vy) + fabsf(cmd.wz);
        try_switch(n);
        gait_machine_update(&s_gm, dt, &out);
        leg_controller_apply(&s_lc, &out);
        osDelay(2);
    }
#endif
}
