/*
 * task_chassis.c
 *
 * 状态机（task 内）：
 *   ACT_STAND  默认空闲（gait_stand）
 *   ACT_TROT   ONLINE 收到 |v|>eps 切入
 *   ACT_SCRIPT 由 task_chassis_play_script() 切入；脚本播完自动回 ACT_STAND
 *
 * 模式：
 *   ONLINE      → 仅根据 task_comm 决策 STAND/TROT，不接受脚本
 *   STANDALONE  → 完全忽略 USB；默认放 SCRIPT(SCRIPT_BUILTIN_STAND_HOLD)
 *   AUTO        → 心跳活 → ONLINE 行为；心跳超时 → 切 STANDALONE 行为
 *
 * 心跳超时 = now_ms - task_comm_last_rx_ms() > online_timeout_ms (默认 500)
 *
 */
#include "task_chassis.h"
#include "log.h"
#include "config.h"
#include "task_comm.h"
#include "task_safety.h"
#include "bsp_time.h"
#include "gait_if.h"
#include "gait_stand.h"
#include "gait_trot.h"
#include "gait_machine.h"
#include "gait_params.h"
#include "../script/gait_script.h"
#include "../script/script_builtin.h"
#include "leg_controller.h"
#include "motor_registry.h"
#include "motor_go.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"
#include "service/attitude/attitude_estimator.h"
#include "service/attitude/steer_controller.h"

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

#include <math.h>

static const char* TAG = "CHASSIS";

typedef enum {
    ACT_STAND = 0,
    ACT_TROT,
    ACT_SCRIPT,
} active_t;

static gait_machine_t   s_gm;
static leg_controller_t s_lc;
static gait_if_t*       s_stand;
static gait_if_t*       s_trot;
static gait_if_t*       s_script;

static chassis_mode_t   s_mode = CHASSIS_MODE_AUTO;
static active_t         s_active = ACT_STAND;
static uint32_t         s_online_timeout_ms = 500;

static const motor_logical_id_t WHEEL_ID[GAIT_LEG_NUM] = {
    MOTOR_ID_FL_WHEEL, MOTOR_ID_FR_WHEEL, MOTOR_ID_RL_WHEEL, MOTOR_ID_RR_WHEEL
};

static const gait_params_t S_BRINGUP_TROT_PARAMS = {
    .body_height_m    = 0.20f,
    .step_length_m    = 0.015f,
    .step_height_m    = 0.010f,
    .period_s         = 1.0f,
    .duty             = 0.75f,
    .phase_offset     = { 0.0f, 0.5f, 0.5f, 0.0f },
    .touchdown_thresh = 0.0f,
};

static int bringup_is_normal(void) {
    return APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_NORMAL;
}

static int bringup_allow_trot(void) {
    return bringup_is_normal() || (APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static int bringup_allow_go_tx(void) {
    return bringup_is_normal() ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_ZERO) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_LEG_HOLD) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_STAND_LOW_GAIN) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static int bringup_allow_m3508_tx(void) {
    return bringup_is_normal() ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_ZERO) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_JOG) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_STAND_LOW_GAIN) ||
           (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_TROT_LOW_GAIN);
}

static void bringup_configure_leg_controller(void) {
    if (bringup_is_normal()) {
        leg_controller_set_output_options(0x0Fu, 1U, 1U, 1.5f, 0.1f);
        return;
    }

    switch (APP_BRINGUP_STAGE) {
        case APP_BRINGUP_STAGE_GO_LEG_HOLD:
            leg_controller_set_output_options((uint8_t)APP_BRINGUP_LEG_MASK, 1U, 0U, 0.08f, 0.02f);
            break;

        case APP_BRINGUP_STAGE_STAND_LOW_GAIN:
            /* 匹配 Wheel-legged stand: KP=0.35 KD=0.08 */
            leg_controller_set_output_options(0x0Fu, 1U, 1U, 0.35f, 0.08f);
            break;

        case APP_BRINGUP_STAGE_TROT_LOW_GAIN:
            leg_controller_set_output_options(0x0Fu, 1U, 1U, 0.20f, 0.04f);
            break;

        default:
            leg_controller_set_output_options(0x00u, 0U, 0U, 0.0f, 0.0f);
            break;
    }
}

static void bringup_apply_wheel_jog(void) {
    for (int i = 0; i < GAIT_LEG_NUM; i++) {
        motor_dev_t* wheel = motor_get(WHEEL_ID[i]);
        if (!wheel || !wheel->ops || !wheel->ops->set_velocity) {
            continue;
        }

        float target = (((uint8_t)(1U << i) & (uint8_t)APP_BRINGUP_WHEEL_MASK) != 0U) ?
                       (float)APP_BRINGUP_WHEEL_JOG_RAD_S : 0.0f;
        (void)wheel->ops->set_velocity(wheel, target);
    }
}

static int request_gait(gait_if_t* g, const gait_params_t* p, float blend) {
    if (s_gm.current == g) return APP_OK;
    /* gait_machine_request 只在 RUN 态接受切换；BLEND 中先用 set 强切 */
    if (s_gm.state != GM_STATE_RUN) {
        return gait_machine_set(&s_gm, g, p);
    }
    return gait_machine_request(&s_gm, g, p, blend);
}

void task_chassis_init(void) {
    s_stand  = gait_stand_create();
    s_trot   = gait_trot_create();
    s_script = gait_script_create();
    gait_machine_init(&s_gm);
    gait_machine_set(&s_gm, s_stand, &GAIT_PARAMS_STAND_DEFAULT);
    leg_controller_init(&s_lc);
    leg_controller_bind_from_registry(&s_lc);
    bringup_configure_leg_controller();
    s_active = ACT_STAND;
    s_mode   = CHASSIS_MODE_AUTO;          /* host 单测可重复 init 时需复位 */
    s_online_timeout_ms = 500;

    /* BMI088 转向初始化 */
    attitude_estimator_init();
    steer_controller_init();

    LOGI("chassis init: mode=AUTO active=stand timeout=%ums bringup_stage=%d",
         (unsigned)s_online_timeout_ms, (int)APP_BRINGUP_STAGE);
}

void task_chassis_set_mode(chassis_mode_t m) {
    if (m == s_mode) return;
    s_mode = m;
    LOGI("mode -> %d", (int)m);
}
chassis_mode_t task_chassis_get_mode(void) { return s_mode; }

void     task_chassis_set_online_timeout_ms(uint32_t ms) { s_online_timeout_ms = ms; }
uint32_t task_chassis_get_online_timeout_ms(void)        { return s_online_timeout_ms; }

int task_chassis_play_script(const script_t* s, float blend_dur_s) {
    if (!s) return APP_ERR_INVALID_ARG;
    int r = gait_script_set_script(s_script, s);
    if (r != APP_OK) return r;
    r = request_gait(s_script, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_SCRIPT;
    return r;
}

int task_chassis_stop_script(float blend_dur_s) {
    int r = request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, blend_dur_s);
    if (r == APP_OK) s_active = ACT_STAND;
    return r;
}

const char* task_chassis_active_gait_name(void) {
    if (!s_gm.current || !s_gm.current->ops || !s_gm.current->ops->name) return "?";
    return s_gm.current->ops->name();
}

/* ─── BMI088 转向接口 ─── */

void task_chassis_reset_yaw(void) {
    attitude_estimator_reset_yaw();
}

float task_chassis_get_yaw(void) {
    return attitude_estimator_get_yaw();
}

/* 是否处在"事实上的离线"状态 */
static int is_offline(uint32_t now_ms) {
    if (s_mode == CHASSIS_MODE_STANDALONE) return 1;
    if (s_mode == CHASSIS_MODE_ONLINE)     return 0;
    /* AUTO：从未收过任何帧（按 dispatch_hit 判定，不依赖时戳基准）→ 离线 */
    if (task_comm_dispatch_hit() == 0) return 1;
    uint32_t last = task_comm_last_rx_ms();
    return (now_ms - last) > s_online_timeout_ms;
}

/* 在线分支：根据 chassis_cmd 二选一 stand/trot */
static void online_decide(void) {
    task_comm_chassis_cmd_t cmd;
    task_comm_get_chassis(&cmd);
    float n = fabsf(cmd.vx) + fabsf(cmd.vy) + fabsf(cmd.wz);
    if (n > 0.05f) {
        if (s_active != ACT_TROT) {
            const gait_params_t* trot_params = bringup_is_normal() ?
                                               &GAIT_PARAMS_TROT_DEFAULT :
                                               &S_BRINGUP_TROT_PARAMS;
            if (request_gait(s_trot, trot_params, 0.3f) == APP_OK) s_active = ACT_TROT;
        }
    } else {
        if (s_active != ACT_STAND) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
        }
    }
}

/* 离线分支：保留当前 SCRIPT；否则确保是 stand */
static void offline_decide(void) {
    if (s_active == ACT_SCRIPT) {
        /* 脚本播完自动回 stand */
        if (gait_script_state(s_script) == SP_STATE_DONE) {
            if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
        }
        return;
    }
    if (s_active != ACT_STAND) {
        if (request_gait(s_stand, &GAIT_PARAMS_STAND_DEFAULT, 0.3f) == APP_OK) s_active = ACT_STAND;
    }
}

void task_chassis_step_for_test(float dt_s, uint32_t now_ms) {
#if !APP_TARGET_HOST
    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BOARD_ONLY) {
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_BUS_ZERO_TX) {
        motor_go_send_all();
        motor_m3508_send_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_ZERO) {
        motor_m3508_send_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_GO_ZERO) {
        motor_go_send_all();
        /* 在锁定态下计算 zero_offset，避免 GO_LEG_HOLD 切入闭环时飞踢 */
        (void)motor_go_calibrate_all();
        return;
    }

    if (APP_BRINGUP_STAGE == APP_BRINGUP_STAGE_M3508_JOG) {
        bringup_apply_wheel_jog();
        motor_m3508_send_all();
        return;
    }

    /*
     * 预标定安全网：当 stage >= GO_LEG_HOLD 但未跑过 GO_ZERO 时，
     * 先在锁定态下收几个周期编码器反馈、算 zero_offset，避免首次闭环飞踢。
     */
    {
        static uint32_t s_pre_calib_cnt = 0;
        static uint8_t  s_pre_calib_done = 0;

        if (!s_pre_calib_done && APP_BRINGUP_STAGE >= APP_BRINGUP_STAGE_GO_LEG_HOLD) {
            motor_go_send_all();          /* 锁定态收发：mode=0，收集编码器反馈 */
            if (s_pre_calib_cnt < 50U) {  /* 100ms @ 500Hz */
                s_pre_calib_cnt++;
                return;
            }
            (void)motor_go_calibrate_all();
            s_pre_calib_done = 1;
            LOGI("pre-calib: motors calibrated, entering stand");
            /* 同周期不继续：下一个周期走正常控制流程 */
            return;
        }
    }
#endif

    if (!bringup_allow_trot()) offline_decide();
    else if (is_offline(now_ms)) offline_decide();
    else                         online_decide();

    /* ─── BMI088 转向: IMU 读取 + 姿态估计 + 转向 PID ─── */
    {
        imu_bmi088_data_t imu_data;
        if (imu_bmi088_read(&imu_data) == APP_OK) {
            attitude_estimator_update(imu_data.gyro, imu_data.accel, dt_s);

            /* 转向控制: 从 task_comm 获取 steer_mode, 若为 YAW 则运行 PID */
            task_comm_chassis_cmd_t cmd;
            task_comm_get_chassis(&cmd);
            if (cmd.steer_mode == 1) {  /* STEER_MODE_YAW */
                steer_controller_set_mode(1);  /* STEER_MODE_YAW */
                steer_controller_set_target_yaw(cmd.target_yaw);
            } else {
                steer_controller_set_mode(0);  /* STEER_MODE_OFF */
            }

            float current_yaw = attitude_estimator_get_yaw();
            (void)steer_controller_update(current_yaw, dt_s);
            /* TODO: 将 steer PID 输出的 wz 修正传入 gait/leg_controller 实现转向 */
        }
    }

    gait_output_t out;
    gait_machine_update(&s_gm, dt_s, &out);
    leg_controller_apply(&s_lc, &out);

#if !APP_TARGET_HOST
    /* MCU 端：将电机指令推送到物理总线 */
    if (bringup_allow_go_tx()) {
        motor_go_send_all();
    }
    if (bringup_allow_m3508_tx()) {
        motor_m3508_send_all();
    }
#endif
}

void task_chassis_entry(void* arg) {
    (void)arg;
    LOGI("task_chassis started");
#if APP_TARGET_MCU
    const float dt = 0.002f; /* 500Hz */
    for (;;) {
        if (task_safety_estop_active()) {
            osDelay(20);
            continue;
        }
        uint32_t now = (uint32_t)bsp_time_now_ms();
        task_chassis_step_for_test(dt, now);
        osDelay(2);
    }
#endif
}
