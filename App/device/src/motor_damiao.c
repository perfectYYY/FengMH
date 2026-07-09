/*
 * motor_damiao.c - Damiao DM43xx motor driver.
 */
#include "motor_damiao.h"

#include "bsp_time.h"
#include "config.h"
#include "dm4310_posvel.h"
#include "log.h"

#include <string.h>

static const char* TAG = "DAMIAO";

#define DAMIAO_AUTO_ENABLE_PERIOD_MS   50U
#define DAMIAO_AUTO_ENABLE_RETRY_MS    200U
/*
 * Motion control declares feedback stale after 250 ms.  Do not wait the old
 * 2 seconds before probing an enabled motor again, otherwise command/response
 * motors can remain silent long enough to latch the arm state machine.
 */
#define DAMIAO_FEEDBACK_REENABLE_MS 200U
#define DAMIAO_TIME_UNSET       0xFFFFFFFFUL

typedef struct {
    motor_logical_id_t logical_id;
    bsp_fdcan_bus_t bus;
    uint32_t can_id;
    damiao_model_t model;
} damiao_map_t;

static const damiao_map_t s_damiao_map[DAMIAO_MOTOR_COUNT] = {
    { MOTOR_ID_ARM_J1, BSP_FDCAN_3, DAMIAO_CAN_ID_J1, DAMIAO_MODEL_DM4340 },
    { MOTOR_ID_ARM_J2, BSP_FDCAN_3, DAMIAO_CAN_ID_J2, DAMIAO_MODEL_DM4340 },
    { MOTOR_ID_ARM_J3, BSP_FDCAN_3, DAMIAO_CAN_ID_J3, DAMIAO_MODEL_DM4340 },
    { MOTOR_ID_ARM_J4, BSP_FDCAN_3, DAMIAO_CAN_ID_J4, DAMIAO_MODEL_DM4310 },
};

static motor_dev_t s_damiao_devs[DAMIAO_MOTOR_COUNT];
static damiao_drv_ctx_t s_damiao_ctxs[DAMIAO_MOTOR_COUNT];
static uint32_t s_last_auto_enable_check_ms = DAMIAO_TIME_UNSET;
static uint32_t s_auto_enable_count;

volatile uint32_t debug_damiao_last_rx_can_id;
volatile uint8_t debug_damiao_last_payload_motor_id;
volatile uint32_t debug_damiao_feedback_route_count[DAMIAO_MOTOR_COUNT];
volatile uint32_t debug_damiao_feedback_invalid_motor_id_count;

static float damiao_clampf(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static uint16_t damiao_float_to_uint(float v, float min_v, float max_v, uint8_t bits) {
    const float span = max_v - min_v;
    const float max_int = (float)((1UL << bits) - 1UL);
    v = damiao_clampf(v, min_v, max_v);
    return (uint16_t)((v - min_v) * max_int / span);
}

static float damiao_uint_to_centered_float(uint16_t raw, float max_abs, uint8_t bits) {
    const int32_t center = 1L << (bits - 1U);
    return ((float)((int32_t)raw - center) * max_abs) / (float)center;
}

static void damiao_model_ranges(damiao_model_t model,
                                float* v_min,
                                float* v_max,
                                float* t_min,
                                float* t_max) {
    if (model == DAMIAO_MODEL_DM4310) {
        *v_min = DAMIAO_DM4310_V_MIN;
        *v_max = DAMIAO_DM4310_V_MAX;
        *t_min = DAMIAO_DM4310_T_MIN;
        *t_max = DAMIAO_DM4310_T_MAX;
    } else {
        *v_min = DAMIAO_DM4340_V_MIN;
        *v_max = DAMIAO_DM4340_V_MAX;
        *t_min = DAMIAO_DM4340_T_MIN;
        *t_max = DAMIAO_DM4340_T_MAX;
    }
}

static float damiao_model_v_abs(damiao_model_t model) {
    return (model == DAMIAO_MODEL_DM4310) ? DAMIAO_DM4310_V_MAX : DAMIAO_DM4340_V_MAX;
}

static float damiao_model_t_abs(damiao_model_t model) {
    return (model == DAMIAO_MODEL_DM4310) ? DAMIAO_DM4310_T_MAX : DAMIAO_DM4340_T_MAX;
}

app_err_t motor_damiao_pack_mit(damiao_model_t model,
                                float pos_rad,
                                float vel_rads,
                                float kp,
                                float kd,
                                float tau_ff_nm,
                                uint8_t out[8]) {
    if (!out) return APP_ERR_NULL_PTR;

    float v_min;
    float v_max;
    float t_min;
    float t_max;
    damiao_model_ranges(model, &v_min, &v_max, &t_min, &t_max);

    uint16_t p_uint = damiao_float_to_uint(pos_rad, DAMIAO_P_MIN, DAMIAO_P_MAX, 16U);
    uint16_t v_uint = damiao_float_to_uint(vel_rads, v_min, v_max, 12U);
    uint16_t kp_uint = damiao_float_to_uint(kp, DAMIAO_KP_MIN, DAMIAO_KP_MAX, 12U);
    uint16_t kd_uint = damiao_float_to_uint(kd, DAMIAO_KD_MIN, DAMIAO_KD_MAX, 12U);
    uint16_t t_uint = damiao_float_to_uint(tau_ff_nm, t_min, t_max, 12U);

    out[0] = (uint8_t)(p_uint >> 8);
    out[1] = (uint8_t)p_uint;
    out[2] = (uint8_t)(v_uint >> 4);
    out[3] = (uint8_t)(((v_uint & 0x0FU) << 4) | ((kp_uint >> 8) & 0x0FU));
    out[4] = (uint8_t)kp_uint;
    out[5] = (uint8_t)(kd_uint >> 4);
    out[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | ((t_uint >> 8) & 0x0FU));
    out[7] = (uint8_t)t_uint;
    return APP_OK;
}

app_err_t motor_damiao_parse_feedback(damiao_model_t model,
                                      const uint8_t data[8],
                                      motor_state_t* out_state,
                                      damiao_drv_ctx_t* out_ctx) {
    if (!data || !out_state) return APP_ERR_NULL_PTR;

    const uint8_t motor_id = (uint8_t)(data[0] & 0x0FU);
    const uint8_t state_code = (uint8_t)((data[0] >> 4) & 0x0FU);
    const uint16_t raw_pos = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    const uint16_t raw_vel = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    const uint16_t raw_tau = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);
    const uint32_t now = (uint32_t)bsp_time_now_ms();

    out_state->angle_rad = damiao_uint_to_centered_float(raw_pos, DAMIAO_P_MAX, 16U);
    out_state->velocity_rads = damiao_uint_to_centered_float(raw_vel,
                                                             damiao_model_v_abs(model),
                                                             12U);
    out_state->torque_nm = damiao_uint_to_centered_float(raw_tau,
                                                         damiao_model_t_abs(model),
                                                         12U);
    out_state->temperature_c = (float)data[7];
    out_state->online = (state_code == DAMIAO_STATE_ENABLED) ? 1U : 0U;
    out_state->last_rx_tick = now;
    out_state->rx_cnt++;
    if (state_code >= DAMIAO_ERR_OVERVOLT) {
        out_state->err_cnt++;
    }

    if (out_ctx) {
        out_ctx->state_code = state_code;
        out_ctx->error = (damiao_error_t)state_code;
        out_ctx->enabled = out_state->online;
        out_ctx->temp_mos_c = data[6];
        out_ctx->temp_rotor_c = data[7];
        out_ctx->raw_position = raw_pos;
        out_ctx->raw_velocity = raw_vel;
        out_ctx->raw_torque = raw_tau;
        (void)motor_id;
    }
    return APP_OK;
}

static app_err_t damiao_send(motor_dev_t* dev, const uint8_t data[8]) {
    if (!dev || !dev->drv_ctx || !data) return APP_ERR_INVALID_ARG;
    damiao_drv_ctx_t* ctx = (damiao_drv_ctx_t*)dev->drv_ctx;
    bsp_fdcan_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = ctx->can_id + DAMIAO_MIT_MODE_ID;
    frame.dlc = 8U;
    memcpy(frame.data, data, 8U);
    return bsp_fdcan_send((bsp_fdcan_bus_t)ctx->bus_id, &frame);
}

static int damiao_set_position(motor_dev_t* dev, float pos, float vel,
                               float kp, float kd, float tau_ff) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    damiao_drv_ctx_t* ctx = (damiao_drv_ctx_t*)dev->drv_ctx;
    uint8_t data[8];
    app_err_t err = motor_damiao_pack_mit(ctx->model, pos, vel, kp, kd, tau_ff, data);
    if (err != APP_OK) return err;
    return damiao_send(dev, data);
}

static int damiao_set_torque(motor_dev_t* dev, float tau_nm) {
    if (!dev) return APP_ERR_INVALID_ARG;
    return damiao_set_position(dev, dev->state.angle_rad, 0.0f, 0.0f, 0.0f, tau_nm);
}

static int damiao_set_velocity(motor_dev_t* dev, float vel_rads) {
    if (!dev) return APP_ERR_INVALID_ARG;
    return damiao_set_position(dev, dev->state.angle_rad, vel_rads, 0.0f, 0.0f, 0.0f);
}

static int damiao_enable(motor_dev_t* dev) {
    static const uint8_t enable_data[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU
    };
    return damiao_send(dev, enable_data);
}

static int damiao_disable(motor_dev_t* dev) {
    static const uint8_t disable_data[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU
    };
    if (dev) {
        dev->state.online = 0U;
        if (dev->drv_ctx) {
            damiao_drv_ctx_t* ctx = (damiao_drv_ctx_t*)dev->drv_ctx;
            ctx->enabled = 0U;
            ctx->state_code = DAMIAO_STATE_DISABLED;
        }
    }
    return damiao_send(dev, disable_data);
}

static int damiao_reset_fault(motor_dev_t* dev) {
    if (dev) {
        dev->state.err_cnt = 0U;
    }
    return damiao_enable(dev);
}

static int damiao_feed_rx(motor_dev_t* dev, const uint8_t* data, uint8_t dlc) {
    if (!dev || !dev->drv_ctx || !data || dlc < 8U) return APP_ERR_INVALID_ARG;
    damiao_drv_ctx_t* ctx = (damiao_drv_ctx_t*)dev->drv_ctx;
    return motor_damiao_parse_feedback(ctx->model, data, &dev->state, ctx);
}

static uint8_t damiao_feedback_needs_enable(const motor_dev_t* dev,
                                            const damiao_drv_ctx_t* ctx,
                                            uint32_t now_ms) {
    if (!dev || !ctx) return 0U;

    /* 电机已报告硬件故障时保持失能，禁止自动重使能与Safety互相抢状态。 */
    if (ctx->error >= DAMIAO_ERR_OVERVOLT) {
        return 0U;
    }

    if (dev->state.rx_cnt == 0U) {
        return 1U;
    }

    if (!ctx->enabled) {
        return 1U;
    }

    if ((now_ms - dev->state.last_rx_tick) >= DAMIAO_FEEDBACK_REENABLE_MS) {
        return 1U;
    }

    return 0U;
}

app_err_t motor_damiao_process(uint32_t now_ms) {
    if (s_last_auto_enable_check_ms != DAMIAO_TIME_UNSET &&
        (now_ms - s_last_auto_enable_check_ms) < DAMIAO_AUTO_ENABLE_PERIOD_MS) {
        return APP_OK;
    }
    s_last_auto_enable_check_ms = now_ms;

    app_err_t result = APP_OK;
    for (uint32_t i = 0U; i < DAMIAO_MOTOR_COUNT; i++) {
        motor_dev_t* dev = &s_damiao_devs[i];
        damiao_drv_ctx_t* ctx = &s_damiao_ctxs[i];
        if (!dev->ops || !dev->ops->enable) continue;
        if (!damiao_feedback_needs_enable(dev, ctx, now_ms)) continue;
        if (ctx->enable_attempt_count > 0U &&
            (now_ms - ctx->last_enable_attempt_ms) < DAMIAO_AUTO_ENABLE_RETRY_MS) {
            continue;
        }

        int err = dev->ops->enable(dev);
        if (err == APP_OK) {
            ctx->last_enable_attempt_ms = now_ms;
            ctx->enable_attempt_count++;
            s_auto_enable_count++;
        } else if (result == APP_OK) {
            result = (app_err_t)err;
        }
    }
    return result;
}

uint32_t motor_damiao_auto_enable_count(void) {
    return s_auto_enable_count;
}

static const motor_ops_t s_damiao_ops = {
    .set_current  = NULL,
    .set_torque   = damiao_set_torque,
    .set_position = damiao_set_position,
    .set_velocity = damiao_set_velocity,
    .enable       = damiao_enable,
    .disable      = damiao_disable,
    .reset_fault  = damiao_reset_fault,
    .feed_rx      = damiao_feed_rx,
};

static int damiao_index_for_payload_motor_id(bsp_fdcan_bus_t bus,
                                             const uint8_t data[8]) {
    if (!data) return -1;
    const uint8_t motor_id = (uint8_t)(data[0] & 0x0FU);
    if (motor_id < 1U || motor_id > DAMIAO_MOTOR_COUNT) return -1;

    const uint32_t idx = (uint32_t)(motor_id - 1U);
    return (s_damiao_map[idx].bus == bus) ? (int)idx : -1;
}

void motor_damiao_fdcan_rx_cb(bsp_fdcan_bus_t bus,
                              const bsp_fdcan_frame_t* frame,
                              void* user) {
    (void)user;
    if (!frame || frame->dlc < 8U) return;

    debug_damiao_last_rx_can_id = frame->can_id;
    debug_damiao_last_payload_motor_id = (uint8_t)(frame->data[0] & 0x0FU);

    const int idx = damiao_index_for_payload_motor_id(bus, frame->data);
    if (idx < 0) {
        debug_damiao_feedback_invalid_motor_id_count++;
        return;
    }
    debug_damiao_feedback_route_count[idx]++;

    motor_dev_t* dev = &s_damiao_devs[idx];
    if (dev->ops && dev->ops->feed_rx) {
        (void)dev->ops->feed_rx(dev, frame->data, frame->dlc);

        /* 同步原 damiao_new1 的反馈全局量和 current_arm_pose。 */
        FDCAN_RxHeaderTypeDef legacy_header = {
            .Identifier = frame->can_id,
        };
        DM_Parse_Feedback(&legacy_header, (uint8_t*)frame->data);
    }
}

app_err_t motor_damiao_init_all(void) {
    memset(s_damiao_devs, 0, sizeof(s_damiao_devs));
    memset(s_damiao_ctxs, 0, sizeof(s_damiao_ctxs));
    s_last_auto_enable_check_ms = DAMIAO_TIME_UNSET;
    s_auto_enable_count = 0U;
    debug_damiao_last_rx_can_id = 0U;
    debug_damiao_last_payload_motor_id = 0U;
    memset((void*)debug_damiao_feedback_route_count,
           0,
           sizeof(debug_damiao_feedback_route_count));
    debug_damiao_feedback_invalid_motor_id_count = 0U;

    for (uint32_t i = 0; i < DAMIAO_MOTOR_COUNT; i++) {
        const damiao_map_t* map = &s_damiao_map[i];
        s_damiao_ctxs[i].bus_id = (uint8_t)map->bus;
        s_damiao_ctxs[i].can_id = map->can_id;
        s_damiao_ctxs[i].model = map->model;

        s_damiao_devs[i].ops = &s_damiao_ops;
        s_damiao_devs[i].drv_ctx = &s_damiao_ctxs[i];
        s_damiao_devs[i].state.id = (uint16_t)map->logical_id;
        s_damiao_devs[i].state.type = MOTOR_DAMIAO;
        s_damiao_devs[i].state.can_bus = (uint8_t)map->bus;
        s_damiao_devs[i].state.can_id = map->can_id;
        (void)motor_registry_bind(map->logical_id, &s_damiao_devs[i]);
    }

    (void)bsp_fdcan_attach_rx(BSP_FDCAN_3, motor_damiao_fdcan_rx_cb, NULL);
    LOGI("Damiao: %u instances initialized on FDCAN3", (unsigned)DAMIAO_MOTOR_COUNT);
    return APP_OK;
}
