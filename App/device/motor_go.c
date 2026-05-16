/*
 * motor_go.c — GO-8010  电机驱动 vtable 实现
 *
 * 1. 8 个 GO 电机分属 2 条 RS485 总线 (USART2/USART3)
 * 2. 每条总线上电机按顺序 ID 0~3 寻址
 * 3. TX 采用逐电机发送（1kHz 周期由 motor_go_send_all 驱动）
 * 4. RX 在 UART 回调中解码，只更新 state，不触发重发
 * 5. 零位标定：enable 时记录当前位置为零位
 */
#include "motor_go.h"
#include "motor_registry.h"
#include "bsp_uart.h"
#include "bsp_time.h"
#include "log.h"

#include <string.h>
#include <math.h>

static const char* TAG = "GO";

/* ─── CLAMP 宏─── */
#define CLAMP(_V, _MIN, _MAX) \
    ((_V) <= (_MIN) ? (_MIN) : ((_V) >= (_MAX) ? (_MAX) : (_V)))

#define GO_TX_WAIT_TIMEOUT_US 250U

/* ─── CRC16-CCITT 查表 ─── */
static const uint16_t s_crc_ccitt_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t go_crc16(uint16_t crc, const uint8_t* buf, size_t len) {
    while (len--)
        crc = (crc >> 8) ^ s_crc_ccitt_table[(crc ^ *buf++) & 0xff];
    return crc;
}

static const motor_cfg_t* go_cfg(const motor_dev_t* dev) {
    if (!dev) return NULL;
    return motor_get_cfg((motor_logical_id_t)dev->state.id);
}

static float go_cfg_sign(const motor_cfg_t* cfg) {
    return cfg ? (float)cfg->dir : 1.0f;
}

static float go_cfg_gear(const motor_cfg_t* cfg) {
    if (!cfg || cfg->gear_ratio <= 0.0f) return 1.0f;
    return cfg->gear_ratio;
}

static float go_motor_pos_to_joint(const motor_cfg_t* cfg,
                                    float zero_offset,
                                    float motor_pos) {
    return go_cfg_sign(cfg) * ((motor_pos - zero_offset) / go_cfg_gear(cfg));
}

static float go_joint_pos_to_motor(const motor_cfg_t* cfg,
                                    float zero_offset,
                                    float joint_pos) {
    return (go_cfg_sign(cfg) * joint_pos * go_cfg_gear(cfg)) + zero_offset;
}

static float go_motor_vel_to_joint(const motor_cfg_t* cfg, float motor_vel) {
    return go_cfg_sign(cfg) * (motor_vel / go_cfg_gear(cfg));
}

static float go_joint_vel_to_motor(const motor_cfg_t* cfg, float joint_vel) {
    return go_cfg_sign(cfg) * joint_vel * go_cfg_gear(cfg);
}

static float go_motor_tau_to_joint(const motor_cfg_t* cfg, float motor_tau) {
    return go_cfg_sign(cfg) * motor_tau * go_cfg_gear(cfg);
}

static float go_joint_tau_to_motor(const motor_cfg_t* cfg, float joint_tau) {
    return go_cfg_sign(cfg) * (joint_tau / go_cfg_gear(cfg));
}

/* ─── RIS 协议帧结构  ─── */
#pragma pack(push, 1)

/* 模式控制字段 1B */
typedef struct {
    uint8_t id     : 4;   /* 电机 ID */
    uint8_t status : 3;   /* 工作模式: 1=FOC闭环（含零力矩）2=校准; 0 不建立通信 */
    uint8_t reserve: 1;
} go_ris_mode_t;

/* 发送控制参数 12B */
typedef struct {
    int16_t tor_des;   /* 力矩 q8 */
    int16_t spd_des;   /* 速度 q8 */
    int32_t pos_des;   /* 位置 q15 */
    int16_t k_pos;     /* 刚度 q15 */
    int16_t k_spd;     /* 阻尼 q15 */
} go_ris_comd_t;

/* 发送帧 17B */
typedef struct {
    uint8_t       head[2];  /* 0xFE 0xEE */
    go_ris_mode_t mode;
    go_ris_comd_t comd;
    uint16_t      crc16;
} go_ris_send_t;

/* 接收反馈数据 11B */
typedef struct {
    int16_t  torque;        /* 力矩 q8 */
    int16_t  speed;         /* 速度 q8 */
    int32_t  pos;           /* 位置 q15 */
    int8_t   temp;          /* 温度 °C */
    uint8_t  MError : 3;   /* 错误码 */
    uint16_t force  : 12;  /* 足底力 12bit */
    uint8_t  none   : 1;
} go_ris_fbk_t;

/* 接收帧 16B */
typedef struct {
    uint8_t       head[2];  /* 0xFD 0xEE */
    go_ris_mode_t mode;
    go_ris_fbk_t  fbk;
    uint16_t      crc16;
} go_ris_back_t;

#pragma pack(pop)

/* ─── 内部编码/解码函数 ─── */

static void go_encode_cmd(const go_drv_ctx_t* ctx, go_ris_send_t* frame) {
    float pos = ctx->cmd_pos;
    float vel = ctx->cmd_vel;
    float tau = ctx->cmd_tau;
    float kp  = ctx->cmd_kp;
    float kd  = ctx->cmd_kd;

    uint8_t motor_id = (uint8_t)CLAMP(ctx->motor_id, 0, 15);
    uint8_t mode     = (uint8_t)CLAMP(ctx->mode, 0, 3);
    kp  = CLAMP(kp, 0.0f, 25.599f);
    kd  = CLAMP(kd, 0.0f, 25.599f);
    tau = CLAMP(tau, -127.99f, 127.99f);
    vel = CLAMP(vel, -804.00f, 804.00f);
    pos = CLAMP(pos, -411774.0f, 411774.0f);

    frame->head[0]     = 0xFE;
    frame->head[1]     = 0xEE;
    frame->mode.id     = motor_id;
    frame->mode.status = mode;
    frame->comd.k_pos  = (int16_t)(kp / 25.6f * 32768);
    frame->comd.k_spd  = (int16_t)(kd / 25.6f * 32768);
    frame->comd.pos_des = (int32_t)(pos / 6.28318f * 32768);
    frame->comd.spd_des = (int16_t)(vel / 6.28318f * 256);
    frame->comd.tor_des = (int16_t)(tau * 256.0f);
    frame->crc16       = go_crc16(0, (const uint8_t*)frame, 15);
}

static int go_decode_fbk(const uint8_t* raw, uint16_t len,
                          motor_state_t* state, go_drv_ctx_t* ctx) {
    if (len < GO_RX_FRAME_SIZE) return -1;

    go_ris_back_t fbk;
    memcpy(&fbk, raw, sizeof(fbk));

    /* 帧头校验 */
    if (fbk.head[0] != 0xFD || fbk.head[1] != 0xEE) return -2;

    /* CRC 校验 */
    uint16_t calc = go_crc16(0, raw, 14);
    if (fbk.crc16 != calc) return -3;

    /* 反量化 */
    float pos_rad = 6.28318f * ((float)fbk.fbk.pos) / 32768.0f;
    float vel_rads = ((float)fbk.fbk.speed / 256.0f) * 6.28318f;
    float tau_nm   = (float)fbk.fbk.torque / 256.0f;

    const motor_cfg_t* cfg = motor_get_cfg((motor_logical_id_t)state->id);

    state->angle_rad     = go_motor_pos_to_joint(cfg, ctx->zero_offset, pos_rad);
    state->velocity_rads = go_motor_vel_to_joint(cfg, vel_rads);
    state->torque_nm     = go_motor_tau_to_joint(cfg, tau_nm);
    state->temperature_c = (float)fbk.fbk.temp;
    state->online       = 1;
    state->last_rx_tick = bsp_time_now_ms();
    state->rx_cnt++;

    ctx->foot_force = fbk.fbk.force;
    ctx->mode       = fbk.mode.status;
    ctx->online     = 1;

    return 0;
}

/* ─── motor_ops_t vtable 实现 ─── */

static int go_set_current(motor_dev_t* dev, float iq_a) {
    /* GO-8010 不支持直接电流控制，通过力矩模式间接实现 */
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    ctx->cmd_tau = go_joint_tau_to_motor(go_cfg(dev), iq_a);
    ctx->cmd_kp  = 0.0f;
    ctx->cmd_kd  = 0.0f;
    return APP_OK;
}

static int go_set_torque(motor_dev_t* dev, float tau_nm) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    ctx->cmd_tau = go_joint_tau_to_motor(go_cfg(dev), tau_nm);
    ctx->cmd_kp  = 0.0f;
    ctx->cmd_kd  = 0.0f;
    return APP_OK;
}

/* 前向声明：go_set_position 自动校准需要 */
static int go_calibrate(motor_dev_t* dev);

static int go_set_position(motor_dev_t* dev, float pos, float vel,
                            float kp, float kd, float tau_ff) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    const motor_cfg_t* cfg = go_cfg(dev);

    /* 首次进入闭环位置控制前自动校准（安全网：即使 GO_ZERO 没跑也能正常工作） */
    if (!ctx->calibrated && dev->state.rx_cnt > 0U) {
        (void)go_calibrate(dev);
    }

    /* 未完成标定：保持零力矩悬浮，禁止下发位置指令
     * 防止 zero_offset=0 时 cmd_pos 计算出负几十 rad 顶限位 */
    if (!ctx->calibrated) {
        ctx->cmd_kp  = 0.0f;
        ctx->cmd_kd  = 0.0f;
        ctx->cmd_tau = 0.0f;
        ctx->mode    = 1;  /* FOC 使能但零力矩，维持通信以便收到首包后能标定 */
        return APP_OK;
    }

    ctx->cmd_pos = go_joint_pos_to_motor(cfg, ctx->zero_offset, pos);
    ctx->cmd_vel = go_joint_vel_to_motor(cfg, vel);
    ctx->cmd_kp  = kp;
    ctx->cmd_kd  = kd;
    ctx->cmd_tau = go_joint_tau_to_motor(cfg, tau_ff);
    ctx->mode    = 1;  /* FOC 闭环 */
    return APP_OK;
}

static int go_set_velocity(motor_dev_t* dev, float vel_rads) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    const motor_cfg_t* cfg = go_cfg(dev);
    ctx->cmd_vel = go_joint_vel_to_motor(cfg, vel_rads);
    ctx->cmd_kp  = 0.0f;
    ctx->cmd_kd  = 0.5f;  /* 纯速度阻尼控制 */
    ctx->cmd_pos = go_joint_pos_to_motor(cfg, ctx->zero_offset, dev->state.angle_rad);
    return APP_OK;
}

/*
 * 仅计算零位偏移，不改变电机模式。
 * 在 GO_ZERO 阶段调用 — 电机保持零力矩（mode=1, kp/kd/tau=0），zero_offset 就绪后可切闭环。
 */
static int go_calibrate(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    const motor_cfg_t* cfg = go_cfg(dev);

    if (ctx->calibrated) return APP_OK;
    if (dev->state.rx_cnt == 0U) return APP_ERR_TIMEOUT; /* 还没收到过反馈 */

    float boot_angle = cfg ? cfg->boot_angle : 0.0f;
    float raw_pos = go_joint_pos_to_motor(cfg, ctx->zero_offset, dev->state.angle_rad);

    ctx->zero_offset = raw_pos - (go_cfg_sign(cfg) * boot_angle * go_cfg_gear(cfg));
    ctx->calibrated  = 1;
    dev->state.angle_rad = boot_angle;

    LOGI("GO%u bus%u calib: zero=%.3f rad boot=%.3f",
         (unsigned)ctx->motor_id, (unsigned)ctx->bus_id,
         (double)ctx->zero_offset, (double)boot_angle);
    return APP_OK;
}

static int go_enable(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;

    /* 先校准（如果还未校准） */
    (void)go_calibrate(dev);

    ctx->mode = 1;  /* FOC 闭环 */
    return APP_OK;
}

static int go_disable(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    ctx->mode = 1;  /* 零力矩：FOC 使能，kp/kd/tau 全零，电机可自由转动并正常回包 */
    ctx->cmd_kp  = 0.0f;
    ctx->cmd_kd  = 0.0f;
    ctx->cmd_tau = 0.0f;
    ctx->cmd_vel = 0.0f;
    return APP_OK;
}

static int go_reset_fault(motor_dev_t* dev) {
    if (!dev || !dev->drv_ctx) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    ctx->mode = 1;  /* 零力矩，保持通信 */
    dev->state.err_cnt = 0;
    return APP_OK;
}

static int go_feed_rx(motor_dev_t* dev, const uint8_t* data, uint8_t dlc) {
    if (!dev || !dev->drv_ctx || !data) return APP_ERR_INVALID_ARG;
    go_drv_ctx_t* ctx = (go_drv_ctx_t*)dev->drv_ctx;
    return go_decode_fbk(data, (uint16_t)dlc, &dev->state, ctx);
}

/* GO-8010 vtable 实例 */
static const motor_ops_t s_go_ops = {
    .set_current  = go_set_current,
    .set_torque   = go_set_torque,
    .set_position = go_set_position,
    .set_velocity = go_set_velocity,
    .enable       = go_enable,
    .disable      = go_disable,
    .reset_fault  = go_reset_fault,
    .feed_rx      = go_feed_rx,
};

/* ─── 驱动实例管理 ─── */

/* 8 个 GO 电机驱动实例 */
static motor_dev_t   s_go_devs[GO_MOTOR_COUNT];
static go_drv_ctx_t s_go_ctxs[GO_MOTOR_COUNT];

/*
 * 总线映射沿用老工程 MotorInstance_Init():
 *   0/1 -> USART3, 3/4 -> USART2, 6/7 -> USART3, 9/10 -> USART2。
 * RIS 协议 ID 也沿用这些全局物理槽位。
 */
typedef struct {
    motor_logical_id_t logical_id;
    bsp_uart_bus_t     uart_bus;
    uint8_t            bus_motor_id;  /* 总线上的顺序 ID */
} go_bus_map_t;

static const go_bus_map_t s_go_map[GO_MOTOR_COUNT] = {
    { MOTOR_ID_FL_HIP,  BSP_UART_3, 0 },
    { MOTOR_ID_FL_KNEE, BSP_UART_3, 1 },
    { MOTOR_ID_RL_HIP,  BSP_UART_2, 3 },
    { MOTOR_ID_RL_KNEE, BSP_UART_2, 4 },
    { MOTOR_ID_RR_HIP,  BSP_UART_3, 6 },
    { MOTOR_ID_RR_KNEE, BSP_UART_3, 7 },
    { MOTOR_ID_FR_HIP,  BSP_UART_2, 9 },
    { MOTOR_ID_FR_KNEE, BSP_UART_2, 10 },
};

app_err_t motor_go_init_all(void) {
    memset(s_go_devs, 0, sizeof(s_go_devs));
    memset(s_go_ctxs, 0, sizeof(s_go_ctxs));

    for (int i = 0; i < GO_MOTOR_COUNT; i++) {
        const go_bus_map_t* m = &s_go_map[i];

        /* 初始化驱动上下文 */
        s_go_ctxs[i].bus_id      = (uint8_t)m->uart_bus;
        s_go_ctxs[i].motor_id    = m->bus_motor_id;
        s_go_ctxs[i].mode        = 1;  /* 零力矩：mode=1, kp/kd/tau=0，上电即可通信 */
        s_go_ctxs[i].cmd_kp      = 0.0f;
        s_go_ctxs[i].cmd_kd      = 0.0f;
        {
            const motor_cfg_t* cfg = motor_get_cfg(m->logical_id);
            s_go_ctxs[i].zero_offset = cfg ? cfg->zero_offset : 0.0f;
        }

        /* 初始化 motor_dev_t */
        s_go_devs[i].ops     = &s_go_ops;
        s_go_devs[i].drv_ctx = &s_go_ctxs[i];
        s_go_devs[i].state.id      = (uint16_t)m->logical_id;
        s_go_devs[i].state.type    = MOTOR_GO;
        s_go_devs[i].state.can_bus = (uint8_t)m->uart_bus;
        s_go_devs[i].state.can_id  = m->bus_motor_id;

        /* 绑定到 registry */
        motor_registry_bind(m->logical_id, &s_go_devs[i]);
    }

    /* 注册 UART RX 回调 */
    bsp_uart_attach_rx(BSP_UART_2, motor_go_uart_rx_cb, NULL);
    bsp_uart_attach_rx(BSP_UART_3, motor_go_uart_rx_cb, NULL);

    LOGI("GO motors: %u instances initialized, bound to registry", GO_MOTOR_COUNT);
    return APP_OK;
}

/* ─── UART RX 回调 ─── */

void motor_go_uart_rx_cb(bsp_uart_bus_t bus, const uint8_t* data,
                           uint32_t len, void* user) {
    (void)user;
    if (len < GO_RX_FRAME_SIZE) return;

    /* 从反馈帧中提取电机 ID */
    go_ris_back_t* fbk = (go_ris_back_t*)data;
    if (fbk->head[0] != 0xFD || fbk->head[1] != 0xEE) return;
    uint8_t rx_motor_id = fbk->mode.id;

    /* 在该总线上查找匹配的电机实例 */
    for (int i = 0; i < GO_MOTOR_COUNT; i++) {
        if (s_go_ctxs[i].bus_id != (uint8_t)bus) continue;
        if (s_go_ctxs[i].motor_id != rx_motor_id) continue;

        /* 找到匹配电机，解码反馈 */
        go_decode_fbk(data, (uint16_t)len, &s_go_devs[i].state, &s_go_ctxs[i]);
        break;
    }
}

/* ─── 周期性发送 ─── */

app_err_t motor_go_send_all(void) {
    /* 按总线分组发送:
     * RS485 半双工 + DMA，每发一帧后等 DMA 完成再发同总线下一帧。
     * 17B @ 4Mbps 物理传输约 42.5us；用短超时等待，避免 1ms tick 级
     * delay 把 500Hz 底盘任务拖到几十 Hz。
     */
    for (int bus = BSP_UART_2; bus <= BSP_UART_3; bus++) {
        for (int i = 0; i < GO_MOTOR_COUNT; i++) {
            if (s_go_ctxs[i].bus_id != (uint8_t)bus) continue;

            go_ris_send_t frame;
            go_encode_cmd(&s_go_ctxs[i], &frame);

            bsp_uart_send((bsp_uart_bus_t)bus,
                          (const uint8_t*)&frame,
                          sizeof(frame));

            (void)bsp_uart_wait_tx_done((bsp_uart_bus_t)bus, GO_TX_WAIT_TIMEOUT_US);
        }
    }
    return APP_OK;
}

app_err_t motor_go_calibrate_all(void) {
    int ok = 0;
    for (int i = 0; i < GO_MOTOR_COUNT; i++) {
        if (go_calibrate(&s_go_devs[i]) == APP_OK) {
            ok++;
        }
    }
    return (ok > 0) ? APP_OK : APP_ERR_TIMEOUT;
}
