/*
 * motor_m3508.c — M3508/C620 电机驱动 vtable 实现
 *
 *
 * 1. 4 个 M3508 轮毂电机分属 2 条 FDCAN 总线
 * 2. 每条总线发送一个 0x200 控制帧，包含最多 4 个电机的电流指令
 * 3. 反馈帧每条总线独立：0x201 (DJI_ID=1)、0x202 (DJI_ID=2)，通过 FDCAN RX 回调解码
 * 4. 减速比 268/17 ≈ 15.76 自动在 vtable 中处理：对外角度/速度已除以减速比
 * 5. 温度保护：>80°C 限功率 50%，>85°C 调用 disable
 */
#include "motor_m3508.h"
#include "motor_registry.h"
#include "bsp_fdcan.h"
#include "bsp_time.h"
#include "log.h"

#include <string.h>
#include <math.h>

static const char* TAG = "M3508";

/* ─── CLAMP 宏  ─── */
#define CLAMP(_V, _MIN, _MAX) \
    ((_V) <= (_MIN) ? (_MIN) : ((_V) >= (_MAX) ? (_MAX) : (_V)))

/* ─── 内部辅助 ─── */

static int16_t m3508_limit_current(float raw) {
    if (raw > (float)M3508_CURRENT_RAW_MAX)  return M3508_CURRENT_RAW_MAX;
    if (raw < (float)-M3508_CURRENT_RAW_MAX) return -M3508_CURRENT_RAW_MAX;
    return (int16_t)raw;
}

/* 功率限制：根据转子转速限制最大允许电流 raw 值 */
static int16_t m3508_power_limit(float desire_current, float rpm) {
    float w = fabsf(rpm) * 2.0f * 3.14159265f / 60.0f;
    if (w < 0.5f) w = 0.5f;
    float max_torque   = M3508_POWER_LIMIT_W / w;
    float max_curr_a   = max_torque / M3508_TORQUE_KT;
    float max_curr_val = max_curr_a * ((float)M3508_CURRENT_RAW_MAX / M3508_CURRENT_LIMIT_A);
    if (max_curr_val > 16000.0f) max_curr_val = 16000.0f;
    if (desire_current >  max_curr_val) return (int16_t) max_curr_val;
    if (desire_current < -max_curr_val) return (int16_t)-max_curr_val;
    return (int16_t)desire_current;
}

static float m3508_rpm_to_rads(float rpm) {
    return rpm * 2.0f * 3.14159265f / 60.0f;
}

static float m3508_encoder_to_rad(int32_t counts) {
    return ((float)counts / (float)M3508_ENCODER_COUNTS) * 2.0f * 3.14159265f;
}

static float m3508_raw_to_ampere(int16_t raw) {
    return ((float)raw / M3508_CURRENT_RAW_MAX) * M3508_CURRENT_LIMIT_A;
}

static int16_t m3508_ampere_to_raw(float a) {
    float raw = (a / M3508_CURRENT_LIMIT_A) * M3508_CURRENT_RAW_MAX;
    return m3508_limit_current(raw);
}

static float m3508_ampere_to_raw_f(float a) {
    return (a / M3508_CURRENT_LIMIT_A) * M3508_CURRENT_RAW_MAX;
}

static const motor_cfg_t* m3508_cfg(const motor_dev_t* dev) {
    if (!dev) return NULL;
    return motor_get_cfg((motor_logical_id_t)dev->state.id);
}

static float m3508_cfg_sign(const motor_cfg_t* cfg) {
    return cfg ? (float)cfg->dir : 1.0f;
}

static float m3508_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float m3508_output_torque_to_raw(const motor_dev_t* dev, float tau_nm) {
    float motor_current_a = m3508_cfg_sign(m3508_cfg(dev)) * tau_nm
                          / (M3508_TORQUE_KT * M3508_REDUCTION_RATIO);
    return m3508_ampere_to_raw_f(motor_current_a);
}

/*
 * 编码器多圈解算
 * 在 feed_rx 中调用，更新 total_angle (累计编码器计数)
 */
static void m3508_update_angle(m3508_drv_ctx_t* ctx, uint16_t ecd) {
    if (ctx->msg_cnt == 0) {
        ctx->ecd        = ecd;
        ctx->last_ecd   = ecd;
        ctx->offset_ecd = ecd;
        ctx->round_cnt  = 0;
        ctx->total_angle = 0;
        ctx->msg_cnt    = 1;
        return;
    }

    ctx->last_ecd = ctx->ecd;
    ctx->ecd      = ecd;

    /* 跨零点判断 */
    int32_t delta = (int32_t)ctx->ecd - (int32_t)ctx->last_ecd;
    if (delta > M3508_HALF_ENCODER) {
        ctx->round_cnt--;
    } else if (delta < -M3508_HALF_ENCODER) {
        ctx->round_cnt++;
    }

    ctx->total_angle = (int32_t)ctx->round_cnt * M3508_ENCODER_COUNTS
                     + (int32_t)ctx->ecd
                     - (int32_t)ctx->offset_ecd;
    ctx->msg_cnt++;
}

/* ─── motor_ops_t vtable 实现 ─── */

static int m3508_set_current(motor_dev_t* dev, float iq_a) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->cmd_current_raw = m3508_ampere_to_raw(m3508_cfg_sign(m3508_cfg(dev)) * iq_a);
    ctx->velocity_hold_active = 0U;
    ctx->ctrl_mode = M3508_MODE_CURRENT;
    return APP_OK;
}

static int m3508_set_torque(motor_dev_t* dev, float tau_nm) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->target_torque_nm = tau_nm;
    ctx->velocity_hold_active = 0U;
    ctx->ctrl_mode = M3508_MODE_TORQUE;
    return APP_OK;
}

static int m3508_set_position(motor_dev_t* dev, float pos, float vel,
                               float kp, float kd, float tau_ff) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    const uint8_t mit_request = (fabsf(vel) > 1e-6f ||
                                 fabsf(kp) > 1e-6f ||
                                 fabsf(kd) > 1e-6f ||
                                 fabsf(tau_ff) > 1e-6f) ? 1U : 0U;
    if (mit_request) {
        ctx->mit_pos_des_rad = pos;
        ctx->mit_vel_des_rads = vel;
        ctx->mit_kp = m3508_clampf(kp, 0.0f, M3508_MIT_KP_MAX);
        ctx->mit_kd = m3508_clampf(kd, 0.0f, M3508_MIT_KD_MAX);
        ctx->mit_tau_ff_nm = m3508_clampf(tau_ff,
                                          -M3508_MIT_TAU_MAX_NM,
                                          M3508_MIT_TAU_MAX_NM);
        ctx->mit_tau_limit_nm = M3508_MIT_TAU_MAX_NM;
        ctx->mit_pos_err_limit_rad = M3508_MIT_POS_ERR_MAX_RAD;
        ctx->target_vel_rads = m3508_cfg_sign(m3508_cfg(dev)) * vel;
        ctx->velocity_hold_active = 0U;
        ctx->pos_err_sum = 0.0f;
        ctx->trq_err_sum = 0.0f;
        ctx->ctrl_mode = M3508_MODE_MIT;
        return APP_OK;
    }

    /* 输出轴 rad → 转子侧累计编码器计数，与 ctx->total_angle 单位一致 */
    ctx->target_position = (int32_t)(m3508_cfg_sign(m3508_cfg(dev)) * pos * M3508_REDUCTION_RATIO
                           / (2.0f * 3.14159265f) * (float)M3508_ENCODER_COUNTS);
    ctx->velocity_hold_active = 0U;
    ctx->ctrl_mode = M3508_MODE_POSITION;
    return APP_OK;
}

static int m3508_set_velocity(motor_dev_t* dev, float vel_rads) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->target_vel_rads = m3508_cfg_sign(m3508_cfg(dev)) * vel_rads;
    ctx->ctrl_mode = M3508_MODE_VELOCITY;
    return APP_OK;
}

static int m3508_enable(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->online = 1;
    ctx->velocity_hold_active = 0U;
    ctx->pos_err_sum = 0.0f;
    ctx->trq_err_sum = 0.0f;
    app_pid_reset(&ctx->speed_pid);
    return APP_OK;
}

static int m3508_disable(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->online = 0;
    ctx->cmd_current_raw = 0;
    ctx->target_vel_rads = 0.0f;
    ctx->velocity_hold_active = 0U;
    ctx->pos_err_sum = 0.0f;
    ctx->trq_err_sum = 0.0f;
    app_pid_reset(&ctx->speed_pid);
    return APP_OK;
}

static int m3508_reset_fault(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    dev->state.err_cnt = 0;
    return m3508_enable(dev);
}

static int m3508_feed_rx(motor_dev_t* dev, const uint8_t* data, uint8_t dlc) {
    if (!dev || !dev->drv_ctx || !data || dlc < 8) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;

    /* 解析 C620 反馈帧 (8B) */
    uint16_t ecd       = (uint16_t)((data[0] << 8) | data[1]);
    int16_t  speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    int16_t  current_raw = (int16_t)((data[4] << 8) | data[5]);
    uint8_t  temp      = data[6];

    /* 编码器多圈解算 */
    m3508_update_angle(ctx, ecd);

    /* EMA 速度滤波 + 电流反馈存储 */
    ctx->actual_current_raw = current_raw;
    if (ctx->msg_cnt == 1) {
        ctx->filter_speed = (float)speed_rpm;   /* 首帧：直接赋初值，防止滤波器冷启动跳变 */
    } else {
        ctx->filter_speed = M3508_EMA_ALPHA * (float)speed_rpm
                          + (1.0f - M3508_EMA_ALPHA) * ctx->filter_speed;
    }

    /* 更新 motor_state_t (输出轴物理量) */
    float sign = m3508_cfg_sign(m3508_cfg(dev));
    dev->state.angle_rad     = sign * m3508_encoder_to_rad(ctx->total_angle) / M3508_REDUCTION_RATIO;
    dev->state.velocity_rads = sign * m3508_rpm_to_rads((float)speed_rpm) / M3508_REDUCTION_RATIO;
    dev->state.torque_nm     = sign * m3508_raw_to_ampere(current_raw) * M3508_TORQUE_KT * M3508_REDUCTION_RATIO;
    dev->state.temperature_c = (float)temp;
    dev->state.online        = 1;
    dev->state.last_rx_tick  = bsp_time_now_ms();
    dev->state.rx_cnt++;
    ctx->online              = 1;

    /* 温度保护 */
    if (temp > 85) {
        if (dev->ops->disable) dev->ops->disable(dev);
        LOGE("M3508 DJI%u overtemp %u°C, disabled", (unsigned)ctx->dji_id, (unsigned)temp);
    } else if (temp > 80) {
        ctx->temp_limit_phase = 1;  /* 限功率 */
    } else {
        ctx->temp_limit_phase = 0;
    }

    return APP_OK;
}

/* M3508 vtable 实例 */
static const motor_ops_t s_m3508_ops = {
    .set_current  = m3508_set_current,
    .set_torque   = m3508_set_torque,
    .set_position = m3508_set_position,
    .set_velocity = m3508_set_velocity,
    .enable       = m3508_enable,
    .disable      = m3508_disable,
    .reset_fault  = m3508_reset_fault,
    .feed_rx      = m3508_feed_rx,
};

/* ─── 驱动实例管理 ─── */

static motor_dev_t      s_m3508_devs[M3508_MOTOR_COUNT];
static m3508_drv_ctx_t  s_m3508_ctxs[M3508_MOTOR_COUNT];
volatile m3508_trace_buffer_t g_m3508_trace;
volatile m3508_mit_ramp_debug_t g_m3508_mit_ramp;

/*
 * 总线映射：DJI_ID 全局连续，与 C620 拨码一致。
 *   FDCAN1: FL_WHEEL ID=1 (反馈 0x201), RL_WHEEL ID=2 (反馈 0x202)
 *   FDCAN2: RR_WHEEL ID=3 (反馈 0x203), FR_WHEEL ID=4 (反馈 0x204)
 * ID=3,4 的 C620 读取 0x200 帧的 bytes[4..7]，slot = dji_id-1 自动对齐。
 */
typedef struct {
    motor_logical_id_t logical_id;
    bsp_fdcan_bus_t    fdcan_bus;
    uint8_t            dji_id;
} m3508_bus_map_t;

static const m3508_bus_map_t s_m3508_map[M3508_MOTOR_COUNT] = {
    { MOTOR_ID_FL_WHEEL, BSP_FDCAN_1, 1 },
    { MOTOR_ID_RL_WHEEL, BSP_FDCAN_1, 2 },
    { MOTOR_ID_RR_WHEEL, BSP_FDCAN_2, 3 },
    { MOTOR_ID_FR_WHEEL, BSP_FDCAN_2, 4 },
};

app_err_t motor_m3508_get_wheel_diag(motor_logical_id_t id,
                                     m3508_wheel_diag_t* out) {
    if (!out) return APP_ERR_INVALID_ARG;
    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        if (s_m3508_map[i].logical_id != id) continue;
        const motor_cfg_t* cfg = motor_get_cfg(id);
        float sign = cfg ? (float)cfg->dir : 1.0f;
        const m3508_drv_ctx_t* ctx = &s_m3508_ctxs[i];
        out->target_velocity_rads = sign * ctx->target_vel_rads;
        out->filtered_velocity_rads = sign * m3508_rpm_to_rads(ctx->filter_speed)
                                    / M3508_REDUCTION_RATIO;
        out->cmd_current_raw = (int16_t)(sign * (float)ctx->cmd_current_raw);
        out->actual_current_raw = (int16_t)(sign * (float)ctx->actual_current_raw);
        out->online = s_m3508_devs[i].state.online;
        return APP_OK;
    }
    memset(out, 0, sizeof(*out));
    return APP_ERR_NOT_FOUND;
}

/* 速度 PID 默认参数 (转子侧) */
#define M3508_PID_MOVE_KP   4.0f
#define M3508_PID_MOVE_KI   8.0f
#define M3508_PID_HOLD_KP   3.0f
#define M3508_PID_HOLD_KI   0.3f
#define M3508_PID_KD        0.0f
#define M3508_PID_MAX_OUT   16000.0f
#define M3508_PID_DT_S      0.002f   /* 500Hz = 2ms */
#define M3508_PID_MOVE_I_OUT_LIMIT 6500.0f /* 非零速度：允许 I 项补足持续摩擦负载 */
#define M3508_PID_HOLD_I_OUT_LIMIT 1200.0f /* 0 速锁轮：限制积分，避免静止颤动 */
#define M3508_PID_I_STATE_LIMIT   (M3508_PID_MOVE_I_OUT_LIMIT / M3508_PID_MOVE_KI)
#define M3508_PID_TARGET_ZERO_RPM 8.0f     /* 目标接近 0 时不积分，避免静止颤动 */
#define M3508_PID_ERR_DEADBAND_RPM 3.0f    /* 误差小于该值时泄放积分 */
#define M3508_PID_I_FREEZE_ERR_RPM 35.0f   /* 已运动时，小于该误差不再继续积分 */
#define M3508_PID_I_FREEZE_ERR_RATIO 0.20f /* 已运动时，误差小于目标比例则冻结 I */
#define M3508_PID_I_FREEZE_MEAS_RATIO 0.50f /* 反馈速度达到目标比例后认为已破静摩擦 */
#define M3508_PID_I_FREEZE_MIN_RPM 20.0f
#define M3508_PID_I_DECAY_ZERO    0.0f     /* 零速目标：清积分 */
#define M3508_PID_I_DECAY_NEAR    0.98f    /* 接近目标：慢慢泄放积分 */
#define M3508_PID_I_DECAY_TRACKING 0.9995f /* 已经跟上目标时，极慢泄放 I 防止堆积 */
#define M3508_PID_SLEW_RAW_STEP   60.0f    /* 500Hz 下每拍最大电流 raw 变化量 */
#define M3508_STATIC_FF_RAW       0.0f     /* 不依赖地面摩擦前馈，先让 PI 自己补偿 */
#define M3508_STATIC_FF_ERR_RPM   30.0f
#define M3508_HOLD_KP_RPM_PER_CNT 0.5f     /* 0 速锁轮：编码器误差 -> 转子侧 rpm */
#define M3508_HOLD_MAX_RPM        500.0f
#define M3508_HOLD_DEADBAND_CNT   2.0f

/* EMA 速度滤波系数 */
#define M3508_EMA_ALPHA     0.3f

/* 位置环 PID 参数 */
#define M3508_POS_KP        1.0f
#define M3508_POS_KI        0.03f
#define M3508_POS_KD        0.15f
#define M3508_POS_MAX_OUT   6000.0f   /* 最大输出转速 (转子侧 rpm) */
#define M3508_POS_MAX_SUM   5000.0f
#define M3508_POS_DEADBAND  25.0f     /* 死区 (编码器计数) */

/* 力矩环 PID 参数 */
#define M3508_TRQ_KP        0.5f
#define M3508_TRQ_KI        0.01f
#define M3508_TRQ_MAX_OUT   16000.0f
#define M3508_TRQ_MAX_SUM   8000.0f
#define M3508_TX_WARN_INTERVAL_MS 100U

static uint32_t s_m3508_tx_err_cnt;
static uint32_t s_m3508_last_tx_warn_ms;

static float m3508_mit_limit_tau(float requested_nm) {
    float limit = (requested_nm > 0.0f) ? requested_nm : M3508_MIT_TAU_MAX_NM;
    return m3508_clampf(limit, 0.0f, M3508_MIT_TAU_HARD_MAX_NM);
}

static float m3508_mit_limit_pos_err(float requested_rad) {
    float limit = (requested_rad > 0.0f) ? requested_rad : M3508_MIT_POS_ERR_MAX_RAD;
    return m3508_clampf(limit, 0.0f, M3508_MIT_POS_ERR_MAX_RAD);
}

void motor_m3508_trace_reset(uint32_t decim) {
    memset((void*)&g_m3508_trace, 0, sizeof(g_m3508_trace));
    g_m3508_trace.decim = (decim == 0U) ? 1U : decim;
}

void motor_m3508_trace_enable(uint8_t enable) {
    g_m3508_trace.enabled = enable ? 1U : 0U;
}

app_err_t motor_m3508_set_mit_limits(motor_dev_t* dev,
                                     float tau_limit_nm,
                                     float pos_err_limit_rad) {
    if (!dev || !dev->drv_ctx || dev->state.type != MOTOR_M3508) {
        return APP_ERR_INVALID_ARG;
    }
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->mit_tau_limit_nm = m3508_mit_limit_tau(tau_limit_nm);
    ctx->mit_pos_err_limit_rad = m3508_mit_limit_pos_err(pos_err_limit_rad);
    return APP_OK;
}

static int16_t m3508_trace_float_to_i16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static void m3508_trace_record(void) {
    static uint32_t s_trace_div;

    if (!g_m3508_trace.enabled) return;
    uint32_t decim = g_m3508_trace.decim;
    if (decim == 0U) decim = 1U;
    if (++s_trace_div < decim) return;
    s_trace_div = 0U;

    uint32_t idx = g_m3508_trace.write_idx % M3508_TRACE_CAPACITY;
    volatile m3508_trace_sample_t* sample = &g_m3508_trace.samples[idx];
    sample->tick_ms = (uint32_t)bsp_time_now_ms();
    sample->online_mask = 0U;
    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        const m3508_drv_ctx_t* ctx = &s_m3508_ctxs[i];
        sample->total_angle[i] = ctx->total_angle;
        sample->cmd_current_raw[i] = ctx->cmd_current_raw;
        sample->actual_current_raw[i] = ctx->actual_current_raw;
        sample->filter_speed_rpm[i] = m3508_trace_float_to_i16(ctx->filter_speed);
        sample->target_vel_mrad_s[i] = m3508_trace_float_to_i16(ctx->target_vel_rads * 1000.0f);
        sample->mit_pos_des_mrad[i] = m3508_trace_float_to_i16(ctx->mit_pos_des_rad * 1000.0f);
        sample->mit_pos_err_mrad[i] = m3508_trace_float_to_i16(ctx->mit_pos_err_rad * 1000.0f);
        sample->mit_tau_cmd_mNm[i] = m3508_trace_float_to_i16(ctx->mit_tau_cmd_nm * 1000.0f);
        sample->ctrl_mode[i] = ctx->ctrl_mode;
        if (ctx->online) {
            sample->online_mask |= (uint8_t)(1U << i);
        }
    }
    g_m3508_trace.write_idx++;
    if (g_m3508_trace.sample_count < M3508_TRACE_CAPACITY) {
        g_m3508_trace.sample_count++;
    }
}

static void m3508_speed_pid_set_tunings(m3508_drv_ctx_t* ctx,
                                        float kp,
                                        float ki,
                                        float i_out_limit,
                                        uint8_t reset) {
    if (!ctx) return;
    app_pid_set_tunings(&ctx->speed_pid, kp, ki, M3508_PID_KD);
    if (ki > 1e-6f) {
        float i_state_limit = i_out_limit / ki;
        app_pid_set_integral_limits(&ctx->speed_pid, i_state_limit, -i_state_limit);
    } else {
        app_pid_set_integral_limits(&ctx->speed_pid, 0.0f, 0.0f);
    }
    if (reset) {
        app_pid_reset(&ctx->speed_pid);
    }
}

static void m3508_speed_pid_decay_integral(app_pid_t* pid, float decay) {
    if (!pid) return;
    pid->integral *= m3508_clampf(decay, 0.0f, 1.0f);
    if (fabsf(pid->integral) < 1e-6f) {
        pid->integral = 0.0f;
    }
}

static uint8_t m3508_speed_pid_should_integrate(float target_rpm,
                                                float measurement_rpm,
                                                float err_rpm) {
    const float abs_target = fabsf(target_rpm);
    const float abs_measure = fabsf(measurement_rpm);
    const float abs_err = fabsf(err_rpm);
    const uint8_t same_dir = (target_rpm * measurement_rpm) > 0.0f ? 1U : 0U;
    const float freeze_err = fmaxf(M3508_PID_I_FREEZE_ERR_RPM,
                                   M3508_PID_I_FREEZE_ERR_RATIO * abs_target);
    const float moving_rpm = fmaxf(M3508_PID_I_FREEZE_MIN_RPM,
                                   M3508_PID_I_FREEZE_MEAS_RATIO * abs_target);

    if (same_dir && abs_measure >= moving_rpm && abs_err <= freeze_err) {
        return 0U;
    }
    return 1U;
}

static float m3508_speed_pid_update(m3508_drv_ctx_t* ctx,
                                    float target_rpm,
                                    float measurement_rpm,
                                    uint8_t clear_integral,
                                    float i_out_limit) {
    if (!ctx) return 0.0f;
    app_pid_t* pid = &ctx->speed_pid;
    const float dt_s = M3508_PID_DT_S;
    const float err = target_rpm - measurement_rpm;
    const float prev_target = pid->prev_error + pid->prev_measurement;

    if (!pid->initialized) {
        pid->prev_measurement = measurement_rpm;
        pid->prev_error = err;
        pid->prev_prev_error = err;
        pid->initialized = 1U;
    }

    const uint8_t target_reversed = (!clear_integral &&
                                     fabsf(prev_target) >= M3508_PID_TARGET_ZERO_RPM &&
                                     (target_rpm * prev_target < 0.0f)) ? 1U : 0U;

    if (clear_integral || target_reversed) {
        m3508_speed_pid_decay_integral(pid, M3508_PID_I_DECAY_ZERO);
    } else if (fabsf(err) <= M3508_PID_ERR_DEADBAND_RPM) {
        m3508_speed_pid_decay_integral(pid, M3508_PID_I_DECAY_NEAR);
    } else if (m3508_speed_pid_should_integrate(target_rpm, measurement_rpm, err)) {
        const float ki_abs = fabsf(pid->Ki);
        if (ki_abs > 1e-6f) {
            const float i_limit = i_out_limit / ki_abs;
            pid->integral += err * dt_s;
            pid->integral = m3508_clampf(pid->integral, -i_limit, i_limit);
        }
    } else {
        m3508_speed_pid_decay_integral(pid, M3508_PID_I_DECAY_TRACKING);
    }

    const float prop = pid->Kp * err;
    const float inte = pid->Ki * pid->integral;
    const float deri = pid->Kd * (measurement_rpm - pid->prev_measurement) / dt_s;
    pid->output = prop + inte - deri;
    pid->output = m3508_clampf(pid->output, pid->output_min, pid->output_max);
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = err;
    pid->prev_measurement = measurement_rpm;
    return pid->output;
}

static float m3508_slew_current(float target_raw, float last_raw) {
    float delta = target_raw - last_raw;
    if (delta > M3508_PID_SLEW_RAW_STEP) {
        return last_raw + M3508_PID_SLEW_RAW_STEP;
    }
    if (delta < -M3508_PID_SLEW_RAW_STEP) {
        return last_raw - M3508_PID_SLEW_RAW_STEP;
    }
    return target_raw;
}

static float m3508_velocity_hold_target_rpm(m3508_drv_ctx_t* ctx) {
    if (!ctx) return 0.0f;
    if (!ctx->velocity_hold_active) {
        ctx->velocity_hold_position = ctx->total_angle;
        ctx->velocity_hold_active = 1U;
        app_pid_reset(&ctx->speed_pid);
    }

    float err_cnt = (float)(ctx->velocity_hold_position - ctx->total_angle);
    if (fabsf(err_cnt) < M3508_HOLD_DEADBAND_CNT) {
        err_cnt = 0.0f;
    }
    return m3508_clampf(M3508_HOLD_KP_RPM_PER_CNT * err_cnt,
                        -M3508_HOLD_MAX_RPM,
                        M3508_HOLD_MAX_RPM);
}

static float m3508_apply_static_ff(float output, float target_rpm, float err_rpm) {
    if (M3508_STATIC_FF_RAW <= 0.0f) {
        (void)target_rpm;
        (void)err_rpm;
        return output;
    }
    if (fabsf(target_rpm) < M3508_STATIC_FF_ERR_RPM) {
        return output;
    }
    if (err_rpm > M3508_STATIC_FF_ERR_RPM) {
        output += M3508_STATIC_FF_RAW;
    } else if (err_rpm < -M3508_STATIC_FF_ERR_RPM) {
        output -= M3508_STATIC_FF_RAW;
    }
    return output;
}

static void m3508_stop_ctx(m3508_drv_ctx_t* ctx) {
    if (!ctx) return;
    ctx->cmd_current_raw = 0;
    ctx->target_vel_rads = 0.0f;
    ctx->mit_vel_des_rads = 0.0f;
    ctx->mit_tau_ff_nm = 0.0f;
    ctx->mit_tau_cmd_nm = 0.0f;
    ctx->velocity_hold_active = 0U;
    ctx->ctrl_mode = M3508_MODE_CURRENT;
}

static void m3508_hold_ctx_mit(m3508_drv_ctx_t* ctx) {
    if (!ctx) return;
    ctx->mit_vel_des_rads = 0.0f;
    ctx->target_vel_rads = 0.0f;
    ctx->velocity_hold_active = 0U;
    ctx->ctrl_mode = M3508_MODE_MIT;
}

static void m3508_mit_ramp_update(void) {
    volatile m3508_mit_ramp_debug_t* ramp = &g_m3508_mit_ramp;

    if (!ramp->enable) {
        ramp->active = 0U;
        return;
    }
    if (ramp->wheel_index >= M3508_MOTOR_COUNT || ramp->duration_ms == 0U) {
        ramp->enable = 0U;
        ramp->active = 0U;
        ramp->done = 1U;
        return;
    }

    m3508_drv_ctx_t* ctx = &s_m3508_ctxs[ramp->wheel_index];
    motor_dev_t* dev = &s_m3508_devs[ramp->wheel_index];
    if (!ctx->online || !dev->state.online) {
        ramp->enable = 0U;
        ramp->active = 0U;
        ramp->done = 1U;
        return;
    }

    if (!ramp->active) {
        ramp->active = 1U;
        ramp->done = 0U;
        ramp->elapsed_ms = 0U;
        ramp->start_angle_rad = dev->state.angle_rad;
        ramp->final_angle_rad = dev->state.angle_rad;
        ramp->theta_ref_rad = dev->state.angle_rad;
        ramp->last_tick_ms = (uint32_t)bsp_time_now_ms();
        ctx->velocity_hold_active = 0U;
        ctx->pos_err_sum = 0.0f;
        ctx->trq_err_sum = 0.0f;
        ctx->mit_pos_err_rad = 0.0f;
        ctx->mit_tau_cmd_nm = 0.0f;
    }

    uint32_t now_ms = (uint32_t)bsp_time_now_ms();
    uint32_t dt_ms = now_ms - ramp->last_tick_ms;
    if (dt_ms == 0U) {
        dt_ms = (uint32_t)(M3508_PID_DT_S * 1000.0f + 0.5f);
    }
    if (dt_ms > 20U) {
        dt_ms = 20U;
    }
    if ((ramp->elapsed_ms + dt_ms) > ramp->duration_ms) {
        dt_ms = ramp->duration_ms - ramp->elapsed_ms;
    }
    ramp->last_tick_ms = now_ms;
    const float dt_s = (float)dt_ms * 0.001f;

    ramp->theta_ref_rad += ramp->omega_des_rads * dt_s;
    ramp->elapsed_ms += dt_ms;

    ctx->mit_pos_des_rad = ramp->theta_ref_rad;
    ctx->mit_vel_des_rads = ramp->omega_des_rads;
    ctx->mit_kp = m3508_clampf(ramp->kp, 0.0f, M3508_MIT_KP_MAX);
    ctx->mit_kd = m3508_clampf(ramp->kd, 0.0f, M3508_MIT_KD_MAX);
    ctx->mit_tau_ff_nm = m3508_clampf(ramp->tau_ff_nm,
                                      -M3508_MIT_TAU_HARD_MAX_NM,
                                      M3508_MIT_TAU_HARD_MAX_NM);
    ctx->mit_tau_limit_nm = m3508_mit_limit_tau(ramp->tau_limit_nm);
    ctx->mit_pos_err_limit_rad = m3508_mit_limit_pos_err(ramp->pos_err_limit_rad);
    ctx->target_vel_rads = m3508_cfg_sign(m3508_cfg(dev)) * ramp->omega_des_rads;
    ctx->ctrl_mode = M3508_MODE_MIT;

    ramp->final_angle_rad = dev->state.angle_rad;
    ramp->last_cmd_current_raw = ctx->cmd_current_raw;
    ramp->last_actual_current_raw = ctx->actual_current_raw;
    ramp->last_speed_rpm = m3508_trace_float_to_i16(ctx->filter_speed);

    if (ramp->elapsed_ms >= ramp->duration_ms) {
        ramp->enable = 0U;
        ramp->active = 0U;
        ramp->done = 1U;
        ramp->final_angle_rad = dev->state.angle_rad;
        ramp->last_cmd_current_raw = ctx->cmd_current_raw;
        ramp->last_actual_current_raw = ctx->actual_current_raw;
        ramp->last_speed_rpm = m3508_trace_float_to_i16(ctx->filter_speed);
        if (ramp->hold_after_done) {
            m3508_hold_ctx_mit(ctx);
        } else {
            m3508_stop_ctx(ctx);
        }
    }
}

typedef struct {
    uint32_t rx_cnt;
    uint32_t last_rx_tick;
    int16_t  speed_rpm;
    int16_t  current_raw;
    uint16_t ecd;
    uint8_t  temperature_c;
} m3508_probe_seen_t;

static volatile m3508_probe_seen_t s_m3508_probe_seen[BSP_FDCAN_BUS_MAX][8];

app_err_t motor_m3508_init_all(void) {
    memset(s_m3508_devs, 0, sizeof(s_m3508_devs));
    memset(s_m3508_ctxs, 0, sizeof(s_m3508_ctxs));
    memset((void*)s_m3508_probe_seen, 0, sizeof(s_m3508_probe_seen));
    memset((void*)&g_m3508_mit_ramp, 0, sizeof(g_m3508_mit_ramp));
    motor_m3508_trace_reset(1U);
    s_m3508_tx_err_cnt = 0;
    s_m3508_last_tx_warn_ms = 0;

    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        const m3508_bus_map_t* m = &s_m3508_map[i];

        /* 初始化驱动上下文 */
        s_m3508_ctxs[i].bus_id  = (uint8_t)m->fdcan_bus;
        s_m3508_ctxs[i].dji_id  = m->dji_id;
        s_m3508_ctxs[i].online  = 0;

        /* 初始化速度 PID (转子侧) */
        app_pid_init(&s_m3508_ctxs[i].speed_pid,
                     M3508_PID_MOVE_KP, M3508_PID_MOVE_KI, M3508_PID_KD,
                     M3508_PID_MAX_OUT, -M3508_PID_MAX_OUT,
                     M3508_PID_I_STATE_LIMIT,
                     -M3508_PID_I_STATE_LIMIT);

        /* 初始化 motor_dev_t */
        s_m3508_devs[i].ops     = &s_m3508_ops;
        s_m3508_devs[i].drv_ctx = &s_m3508_ctxs[i];
        s_m3508_devs[i].state.id      = (uint16_t)m->logical_id;
        s_m3508_devs[i].state.type    = MOTOR_M3508;
        s_m3508_devs[i].state.can_bus = (uint8_t)m->fdcan_bus;
        s_m3508_devs[i].state.can_id  = m->dji_id;

        /* 绑定到 registry */
        motor_registry_bind(m->logical_id, &s_m3508_devs[i]);
    }

    /* 注册 FDCAN RX 回调 */
    bsp_fdcan_attach_rx(BSP_FDCAN_1, motor_m3508_fdcan_rx_cb, NULL);
    bsp_fdcan_attach_rx(BSP_FDCAN_2, motor_m3508_fdcan_rx_cb, NULL);

    LOGI("M3508: %u instances initialized, bound to registry", M3508_MOTOR_COUNT);
    return APP_OK;
}

/* ─── FDCAN RX 回调 ─── */

void motor_m3508_fdcan_rx_cb(bsp_fdcan_bus_t bus,
                               const bsp_fdcan_frame_t* f,
                               void* user) {
    (void)user;

    /* 只处理 0x201~0x208 反馈帧 */
    if (f->can_id < M3508_FB_ID_BASE || f->can_id > 0x208) return;

    uint8_t dji_id = (uint8_t)(f->can_id - M3508_FB_ID_BASE + 1);  /* 全局 ID：0x201→1, 0x202→2, 0x203→3, 0x204→4 */
    if (bus < BSP_FDCAN_BUS_MAX && dji_id >= 1U && dji_id <= 8U && f->dlc >= 8U) {
        volatile m3508_probe_seen_t* seen = &s_m3508_probe_seen[bus][dji_id - 1U];
        seen->ecd = (uint16_t)((f->data[0] << 8) | f->data[1]);
        seen->speed_rpm = (int16_t)((f->data[2] << 8) | f->data[3]);
        seen->current_raw = (int16_t)((f->data[4] << 8) | f->data[5]);
        seen->temperature_c = f->data[6];
        seen->last_rx_tick = (uint32_t)bsp_time_now_ms();
        seen->rx_cnt++;
    }

    /* 查找匹配的电机实例 */
    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        if (s_m3508_ctxs[i].bus_id != (uint8_t)bus) continue;
        if (s_m3508_ctxs[i].dji_id != dji_id) continue;

        /* 找到匹配电机，解码反馈 */
        m3508_feed_rx(&s_m3508_devs[i], f->data, f->dlc);
        break;
    }
}

/* ─── 周期性发送 ─── */

static float m3508_apply_temperature_derate(const m3508_drv_ctx_t* ctx,
                                            float output) {
    if (ctx->temp_limit_phase == 1) {
        output *= 0.5f;
    }
    return output;
}

static void m3508_run_position_control(m3508_drv_ctx_t* ctx) {
    float err = (float)(ctx->target_position - ctx->total_angle);
    if (fabsf(err) < M3508_POS_DEADBAND) {
        ctx->cmd_current_raw = 0;
        ctx->pos_err_sum = 0.0f;
        return;
    }

    ctx->pos_err_sum += err;
    if (ctx->pos_err_sum >  M3508_POS_MAX_SUM) ctx->pos_err_sum =  M3508_POS_MAX_SUM;
    if (ctx->pos_err_sum < -M3508_POS_MAX_SUM) ctx->pos_err_sum = -M3508_POS_MAX_SUM;

    /* D 项直接用滤波速度反馈，与新版 Core 驱动行为一致。 */
    float pos_speed = M3508_POS_KP * err
                    + M3508_POS_KI * ctx->pos_err_sum * M3508_PID_DT_S
                    - M3508_POS_KD * ctx->filter_speed;
    if (pos_speed >  M3508_POS_MAX_OUT) pos_speed =  M3508_POS_MAX_OUT;
    if (pos_speed < -M3508_POS_MAX_OUT) pos_speed = -M3508_POS_MAX_OUT;

    float output = app_pid_update_dt(&ctx->speed_pid, pos_speed,
                                     ctx->filter_speed, M3508_PID_DT_S);
    output = m3508_apply_temperature_derate(ctx, output);
    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
}

static void m3508_run_torque_control(motor_dev_t* dev,
                                     m3508_drv_ctx_t* ctx) {
    /* 前馈：目标力矩 -> 目标电流 raw；PID 做闭环补偿。 */
    float tgt_curr = m3508_output_torque_to_raw(dev, ctx->target_torque_nm);
    float err = tgt_curr - (float)ctx->actual_current_raw;
    ctx->trq_err_sum += err;
    if (ctx->trq_err_sum >  M3508_TRQ_MAX_SUM) ctx->trq_err_sum =  M3508_TRQ_MAX_SUM;
    if (ctx->trq_err_sum < -M3508_TRQ_MAX_SUM) ctx->trq_err_sum = -M3508_TRQ_MAX_SUM;

    float output = tgt_curr
                 + M3508_TRQ_KP * err
                 + M3508_TRQ_KI * ctx->trq_err_sum * M3508_PID_DT_S;
    if (output >  M3508_TRQ_MAX_OUT) output =  M3508_TRQ_MAX_OUT;
    if (output < -M3508_TRQ_MAX_OUT) output = -M3508_TRQ_MAX_OUT;

    output = m3508_apply_temperature_derate(ctx, output);
    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
}

static void m3508_run_mit_control(motor_dev_t* dev,
                                  m3508_drv_ctx_t* ctx) {
    float pos_err = ctx->mit_pos_des_rad - dev->state.angle_rad;
    float vel_err = ctx->mit_vel_des_rads - dev->state.velocity_rads;
    const float pos_err_limit = m3508_mit_limit_pos_err(ctx->mit_pos_err_limit_rad);
    const float tau_limit = m3508_mit_limit_tau(ctx->mit_tau_limit_nm);
    pos_err = m3508_clampf(pos_err, -pos_err_limit, pos_err_limit);

    float tau_out = ctx->mit_kp * pos_err
                  + ctx->mit_kd * vel_err
                  + ctx->mit_tau_ff_nm;
    tau_out = m3508_clampf(tau_out, -tau_limit, tau_limit);

    ctx->mit_pos_err_rad = pos_err;
    ctx->mit_tau_cmd_nm = tau_out;

    float output = m3508_output_torque_to_raw(dev, tau_out);
    output = m3508_apply_temperature_derate(ctx, output);
    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
}

static void m3508_run_current_control(m3508_drv_ctx_t* ctx) {
    /* cmd_current_raw 已由 set_current 直接写入，仅做温度限功率。 */
    if (ctx->temp_limit_phase == 1) {
        ctx->cmd_current_raw = (int16_t)((float)ctx->cmd_current_raw * 0.5f);
    }
}

static float m3508_velocity_target_to_rpm(const m3508_drv_ctx_t* ctx) {
    return ctx->target_vel_rads * M3508_REDUCTION_RATIO
         * 60.0f / (2.0f * 3.14159265f);
}

static uint8_t m3508_pid_needs_tuning_update(const app_pid_t* pid,
                                             float kp,
                                             float ki) {
    return (fabsf(pid->Kp - kp) > 1e-6f ||
            fabsf(pid->Ki - ki) > 1e-6f) ? 1U : 0U;
}

static float m3508_prepare_velocity_target_rpm(m3508_drv_ctx_t* ctx,
                                               uint8_t* clear_integral,
                                               float* i_out_limit) {
    float target_rpm = m3508_velocity_target_to_rpm(ctx);
    uint8_t reset_pid = 0U;
    *clear_integral = 0U;
    *i_out_limit = M3508_PID_MOVE_I_OUT_LIMIT;

    if (fabsf(target_rpm) < M3508_PID_TARGET_ZERO_RPM) {
        *i_out_limit = M3508_PID_HOLD_I_OUT_LIMIT;
        if (m3508_pid_needs_tuning_update(&ctx->speed_pid,
                                          M3508_PID_HOLD_KP,
                                          M3508_PID_HOLD_KI)) {
            reset_pid = 1U;
        }
        m3508_speed_pid_set_tunings(ctx,
                                    M3508_PID_HOLD_KP,
                                    M3508_PID_HOLD_KI,
                                    *i_out_limit,
                                    reset_pid);
        target_rpm = m3508_velocity_hold_target_rpm(ctx);
        if (fabsf(target_rpm) < M3508_PID_ERR_DEADBAND_RPM) {
            *clear_integral = 1U;
        }
        return target_rpm;
    }

    if (ctx->velocity_hold_active) {
        ctx->velocity_hold_active = 0U;
        reset_pid = 1U;
    }
    if (m3508_pid_needs_tuning_update(&ctx->speed_pid,
                                      M3508_PID_MOVE_KP,
                                      M3508_PID_MOVE_KI)) {
        reset_pid = 1U;
    }
    m3508_speed_pid_set_tunings(ctx,
                                M3508_PID_MOVE_KP,
                                M3508_PID_MOVE_KI,
                                *i_out_limit,
                                reset_pid);
    return target_rpm;
}

static void m3508_run_velocity_control(m3508_drv_ctx_t* ctx) {
    uint8_t clear_integral = 0U;
    float i_out_limit = M3508_PID_MOVE_I_OUT_LIMIT;
    float target_rpm = m3508_prepare_velocity_target_rpm(ctx,
                                                         &clear_integral,
                                                         &i_out_limit);

    float output = m3508_speed_pid_update(ctx, target_rpm, ctx->filter_speed,
                                          clear_integral, i_out_limit);
    output = m3508_apply_static_ff(output, target_rpm, target_rpm - ctx->filter_speed);
    output = m3508_apply_temperature_derate(ctx, output);
    output = m3508_slew_current(output, (float)ctx->cmd_current_raw);
    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
}

static void m3508_run_control_mode(motor_dev_t* dev,
                                   m3508_drv_ctx_t* ctx) {
    if (!ctx->online || !dev->state.online) {
        ctx->cmd_current_raw = 0;
        return;
    }

    switch (ctx->ctrl_mode) {
    case M3508_MODE_POSITION:
        m3508_run_position_control(ctx);
        break;

    case M3508_MODE_TORQUE:
        m3508_run_torque_control(dev, ctx);
        break;

    case M3508_MODE_MIT:
        m3508_run_mit_control(dev, ctx);
        break;

    case M3508_MODE_CURRENT:
        m3508_run_current_control(ctx);
        break;

    case M3508_MODE_VELOCITY:
    default:
        m3508_run_velocity_control(ctx);
        break;
    }
}

static void m3508_pack_current_slot(uint8_t tx_data[8],
                                    const m3508_drv_ctx_t* ctx) {
    uint8_t slot = ctx->dji_id - 1U;
    if (slot < 4U) {
        int16_t iq = ctx->cmd_current_raw;
        tx_data[slot * 2U] = (uint8_t)(iq >> 8);
        tx_data[slot * 2U + 1U] = (uint8_t)(iq);
    }
}

static void m3508_log_tx_failure(bsp_fdcan_bus_t bus,
                                 app_err_t ret,
                                 const uint8_t tx_data[8]) {
    uint32_t now = (uint32_t)bsp_time_now_ms();
    if ((now - s_m3508_last_tx_warn_ms) < M3508_TX_WARN_INTERVAL_MS) {
        return;
    }

    s_m3508_last_tx_warn_ms = now;
    LOGW("M3508 tx bus%u failed ret=%d cnt=%lu iq=[%d,%d,%d,%d]",
         (unsigned)bus,
         (int)ret,
         (unsigned long)s_m3508_tx_err_cnt,
         (int16_t)((tx_data[0] << 8) | tx_data[1]),
         (int16_t)((tx_data[2] << 8) | tx_data[3]),
         (int16_t)((tx_data[4] << 8) | tx_data[5]),
         (int16_t)((tx_data[6] << 8) | tx_data[7]));
}

static void m3508_log_probe_tx_failure(bsp_fdcan_bus_t bus,
                                       app_err_t ret) {
    uint32_t now = (uint32_t)bsp_time_now_ms();
    if ((now - s_m3508_last_tx_warn_ms) < M3508_TX_WARN_INTERVAL_MS) {
        return;
    }

    s_m3508_last_tx_warn_ms = now;
    LOGW("M3508 probe tx bus%u failed ret=%d cnt=%lu",
         (unsigned)bus,
         (int)ret,
         (unsigned long)s_m3508_tx_err_cnt);
}

static app_err_t m3508_send_current_frame(bsp_fdcan_bus_t bus,
                                          const uint8_t tx_data[8]) {
    bsp_fdcan_frame_t frame;
    frame.can_id = M3508_TX_ID;
    frame.dlc = 8;
    memcpy(frame.data, tx_data, 8);
    frame.rx_tick = 0;

    app_err_t ret = bsp_fdcan_send(bus, &frame);
    if (ret != APP_OK) {
        s_m3508_tx_err_cnt++;
        m3508_log_tx_failure(bus, ret, tx_data);
    }
    return ret;
}

static app_err_t m3508_send_probe_frame(bsp_fdcan_bus_t bus,
                                        const uint8_t probe_data[8]) {
    bsp_fdcan_frame_t frame;
    frame.can_id = M3508_TX_ID_HIGH;
    frame.dlc = 8;
    memcpy(frame.data, probe_data, 8);
    frame.rx_tick = 0;

    app_err_t ret = bsp_fdcan_send(bus, &frame);
    if (ret != APP_OK) {
        s_m3508_tx_err_cnt++;
        m3508_log_probe_tx_failure(bus, ret);
    }
    return ret;
}

static void m3508_run_bus_controls(bsp_fdcan_bus_t bus,
                                   uint8_t tx_data[8]) {
    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        if (s_m3508_ctxs[i].bus_id != (uint8_t)bus) continue;

        motor_dev_t* dev = &s_m3508_devs[i];
        m3508_drv_ctx_t* ctx = &s_m3508_ctxs[i];
        m3508_run_control_mode(dev, ctx);
        m3508_pack_current_slot(tx_data, ctx);
    }
}

/*
 * 每条总线构造一个 0x200 控制帧 (8B)。
 * 每个电机的电流 raw 占 2 字节，按 DJI ID 排列（每条总线 DJI_ID=1→slot0, 2→slot1）。
 * 未使用的槽位填 0。
 */
app_err_t motor_m3508_send_all(void) {
    app_err_t ret_all = APP_OK;

    m3508_mit_ramp_update();

    /* 按总线分组 */
    for (int bus = BSP_FDCAN_1; bus <= BSP_FDCAN_2; bus++) {
        uint8_t tx_data[8];
        uint8_t probe_data[8];
        memset(tx_data, 0, 8);
        memset(probe_data, 0, 8);

        m3508_run_bus_controls((bsp_fdcan_bus_t)bus, tx_data);

        app_err_t ret = m3508_send_current_frame((bsp_fdcan_bus_t)bus, tx_data);
        if (ret != APP_OK) {
            ret_all = ret;
        }

        /* DJI ID 5~8 使用 0x1FF 控制帧。这里周期性发零电流探测帧，
         * 这样即使电调 ID 不在当前映射表内，也能触发/维持反馈并被
         * s_m3508_probe_seen 记录出来。 */
        ret = m3508_send_probe_frame((bsp_fdcan_bus_t)bus, probe_data);
        if (ret != APP_OK) {
            ret_all = ret;
        }
    }
    m3508_trace_record();
    return ret_all;
}
