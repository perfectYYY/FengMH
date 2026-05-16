/*
 * motor_m3508.c — M3508/C620 电机驱动 vtable 实现
 *
 *
 * 1. 4 个 M3508 轮毂电机分属 2 条 FDCAN 总线
 * 2. 每条总线发送一个 0x200 控制帧，包含最多 4 个电机的电流指令
 * 3. 反馈帧每条总线独立：0x201 (DJI_ID=1)、0x202 (DJI_ID=2)，通过 FDCAN RX 回调解码
 * 4. 减速比 187:1 自动在 vtable 中处理：对外角度/速度已除以减速比
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
    ctx->cmd_current_raw = m3508_ampere_to_raw(iq_a);
    ctx->ctrl_mode = M3508_MODE_CURRENT;
    return APP_OK;
}

static int m3508_set_torque(motor_dev_t* dev, float tau_nm) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->target_torque_nm = tau_nm;
    ctx->ctrl_mode = M3508_MODE_TORQUE;
    return APP_OK;
}

static int m3508_set_position(motor_dev_t* dev, float pos, float vel,
                               float kp, float kd, float tau_ff) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    (void)vel; (void)kp; (void)kd; (void)tau_ff;
    /* 输出轴 rad → 转子侧累计编码器计数，与 ctx->total_angle 单位一致 */
    ctx->target_position = (int32_t)(pos * M3508_REDUCTION_RATIO
                           / (2.0f * 3.14159265f) * (float)M3508_ENCODER_COUNTS);
    ctx->ctrl_mode = M3508_MODE_POSITION;
    return APP_OK;
}

static int m3508_set_velocity(motor_dev_t* dev, float vel_rads) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->target_vel_rads = vel_rads;
    ctx->ctrl_mode = M3508_MODE_VELOCITY;
    return APP_OK;
}

static int m3508_enable(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    m3508_drv_ctx_t* ctx = (m3508_drv_ctx_t*)dev->drv_ctx;
    ctx->online = 1;
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
    dev->state.angle_rad     = m3508_encoder_to_rad(ctx->total_angle) / M3508_REDUCTION_RATIO;
    dev->state.velocity_rads = m3508_rpm_to_rads((float)speed_rpm) / M3508_REDUCTION_RATIO;
    dev->state.torque_nm     = m3508_raw_to_ampere(current_raw) * M3508_TORQUE_KT * M3508_REDUCTION_RATIO;
    dev->state.temperature_c = (float)temp;
    dev->state.online        = 1;
    dev->state.last_rx_tick  = bsp_time_now_ms();
    dev->state.rx_cnt++;

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

/*
 * 总线映射沿用老工程 M3508Task:
 *   CAN1: FL_WHEEL 0x201, RL_WHEEL 0x202
 *   CAN2: RR_WHEEL 0x201, FR_WHEEL 0x202
 */
typedef struct {
    motor_logical_id_t logical_id;
    bsp_fdcan_bus_t    fdcan_bus;
    uint8_t            dji_id;
} m3508_bus_map_t;

static const m3508_bus_map_t s_m3508_map[M3508_MOTOR_COUNT] = {
    { MOTOR_ID_FL_WHEEL, BSP_FDCAN_1, 1 },
    { MOTOR_ID_RL_WHEEL, BSP_FDCAN_1, 2 },
    { MOTOR_ID_RR_WHEEL, BSP_FDCAN_2, 1 },
    { MOTOR_ID_FR_WHEEL, BSP_FDCAN_2, 2 },
};

/* 速度 PID 默认参数 (转子侧) */
#define M3508_PID_KP        3.0f
#define M3508_PID_KI        0.3f
#define M3508_PID_KD        0.0f
#define M3508_PID_MAX_OUT   16000.0f
#define M3508_PID_MAX_SUM   10000.0f
#define M3508_PID_DT_S      0.002f   /* 500Hz = 2ms */

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

app_err_t motor_m3508_init_all(void) {
    memset(s_m3508_devs, 0, sizeof(s_m3508_devs));
    memset(s_m3508_ctxs, 0, sizeof(s_m3508_ctxs));

    for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
        const m3508_bus_map_t* m = &s_m3508_map[i];

        /* 初始化驱动上下文 */
        s_m3508_ctxs[i].bus_id  = (uint8_t)m->fdcan_bus;
        s_m3508_ctxs[i].dji_id  = m->dji_id;
        s_m3508_ctxs[i].online  = 0;

        /* 初始化速度 PID (转子侧) */
        app_pid_init(&s_m3508_ctxs[i].speed_pid,
                     M3508_PID_KP, M3508_PID_KI, M3508_PID_KD,
                     M3508_PID_MAX_OUT, -M3508_PID_MAX_OUT,
                     M3508_PID_MAX_SUM * M3508_PID_DT_S,
                     -M3508_PID_MAX_SUM * M3508_PID_DT_S);

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

    uint8_t dji_id = (uint8_t)(f->can_id - M3508_FB_ID_BASE + 1);  /* 每条总线实际只有 1 或 2 */

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

/*
 * 每条总线构造一个 0x200 控制帧 (8B)。
 * 每个电机的电流 raw 占 2 字节，按 DJI ID 排列（每条总线 DJI_ID=1→slot0, 2→slot1）。
 * 未使用的槽位填 0。
 */
app_err_t motor_m3508_send_all(void) {
    /* 按总线分组 */
    for (int bus = BSP_FDCAN_1; bus <= BSP_FDCAN_2; bus++) {
        uint8_t tx_data[8];
        memset(tx_data, 0, 8);

        for (int i = 0; i < M3508_MOTOR_COUNT; i++) {
            if (s_m3508_ctxs[i].bus_id != (uint8_t)bus) continue;

            /* 按控制模式运行 PID */
            m3508_drv_ctx_t* ctx = &s_m3508_ctxs[i];
            if (ctx->online) {
                float output = 0.0f;
                switch (ctx->ctrl_mode) {

                case M3508_MODE_POSITION: {
                    float err = (float)(ctx->target_position - ctx->total_angle);
                    if (fabsf(err) < M3508_POS_DEADBAND) {
                        ctx->cmd_current_raw = 0;
                        ctx->pos_err_sum     = 0.0f;
                        break;
                    }
                    ctx->pos_err_sum += err;
                    if (ctx->pos_err_sum >  M3508_POS_MAX_SUM) ctx->pos_err_sum =  M3508_POS_MAX_SUM;
                    if (ctx->pos_err_sum < -M3508_POS_MAX_SUM) ctx->pos_err_sum = -M3508_POS_MAX_SUM;
                    /* D 项直接用滤波速度反馈，与新版 Core 驱动行为一致 */
                    float pos_speed = M3508_POS_KP * err
                                    + M3508_POS_KI * ctx->pos_err_sum * M3508_PID_DT_S
                                    - M3508_POS_KD * ctx->filter_speed;
                    if (pos_speed >  M3508_POS_MAX_OUT) pos_speed =  M3508_POS_MAX_OUT;
                    if (pos_speed < -M3508_POS_MAX_OUT) pos_speed = -M3508_POS_MAX_OUT;
                    output = app_pid_update_dt(&ctx->speed_pid, pos_speed,
                                               ctx->filter_speed, M3508_PID_DT_S);
                    if (ctx->temp_limit_phase == 1) output *= 0.5f;
                    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
                    break;
                }

                case M3508_MODE_TORQUE: {
                    /* 前馈：目标力矩 → 目标电流 raw；PID 做闭环补偿 */
                    float int_curr   = (float)M3508_CURRENT_RAW_MAX / M3508_CURRENT_LIMIT_A;
                    float tgt_curr   = (ctx->target_torque_nm / M3508_TORQUE_KT) * int_curr;
                    float err        = tgt_curr - (float)ctx->actual_current_raw;
                    ctx->trq_err_sum += err;
                    if (ctx->trq_err_sum >  M3508_TRQ_MAX_SUM) ctx->trq_err_sum =  M3508_TRQ_MAX_SUM;
                    if (ctx->trq_err_sum < -M3508_TRQ_MAX_SUM) ctx->trq_err_sum = -M3508_TRQ_MAX_SUM;
                    output = tgt_curr
                           + M3508_TRQ_KP * err
                           + M3508_TRQ_KI * ctx->trq_err_sum * M3508_PID_DT_S;
                    if (output >  M3508_TRQ_MAX_OUT) output =  M3508_TRQ_MAX_OUT;
                    if (output < -M3508_TRQ_MAX_OUT) output = -M3508_TRQ_MAX_OUT;
                    if (ctx->temp_limit_phase == 1) output *= 0.5f;
                    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
                    break;
                }

                case M3508_MODE_CURRENT:
                    /* cmd_current_raw 已由 set_current 直接写入，仅做温度限功率 */
                    if (ctx->temp_limit_phase == 1) {
                        ctx->cmd_current_raw = (int16_t)((float)ctx->cmd_current_raw * 0.5f);
                    }
                    break;

                case M3508_MODE_VELOCITY:
                default: {
                    float target_rpm = ctx->target_vel_rads * M3508_REDUCTION_RATIO
                                     * 60.0f / (2.0f * 3.14159265f);
                    output = app_pid_update_dt(&ctx->speed_pid, target_rpm,
                                               ctx->filter_speed, M3508_PID_DT_S);
                    if (ctx->temp_limit_phase == 1) output *= 0.5f;
                    ctx->cmd_current_raw = m3508_power_limit(output, ctx->filter_speed);
                    break;
                }
                }
            }

            /* 将电流指令填入 0x200 帧 */
            uint8_t slot = ctx->dji_id - 1;  /* DJI ID 1→slot0, 2→slot1 */
            if (slot < 4) {
                int16_t iq = ctx->cmd_current_raw;
                tx_data[slot * 2]     = (uint8_t)(iq >> 8);
                tx_data[slot * 2 + 1] = (uint8_t)(iq);
            }
        }

        /* 发送 0x200 控制帧 */
        bsp_fdcan_frame_t frame;
        frame.can_id = M3508_TX_ID;
        frame.dlc    = 8;
        memcpy(frame.data, tx_data, 8);
        frame.rx_tick = 0;

        bsp_fdcan_send((bsp_fdcan_bus_t)bus, &frame);
    }
    return APP_OK;
}
