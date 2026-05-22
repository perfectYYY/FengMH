/*
 * motor_go.h — GO-8010 (宇树) 电机驱动 vtable 接口
 *
 * 实现 motor_ops_t vtable，通过 bsp_uart (RS485) 与 GO-8010 通信。
 * RIS 协议：17B TX 帧 / 16B RX 帧，CRC16-CCITT 校验。
 *
 * 8 个 GO-8010 电机分属 2 条 RS485 总线，沿用老工程物理槽位：
 *   - USART2 (BSP_UART_2): RL_HIP(3), RL_KNEE(4), FR_HIP(9), FR_KNEE(10)
 *   - USART3 (BSP_UART_3): FL_HIP(0), FL_KNEE(1), RR_HIP(6), RR_KNEE(7)
 *
 * RIS 帧里的电机 ID 使用老工程全局 ID，不在每条总线上重新编号。
 */
#ifndef APP_DEVICE_MOTOR_GO_H_
#define APP_DEVICE_MOTOR_GO_H_

#include "motor_if.h"
#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GO-8010 电机数量 (腿关节：4腿 × 2关节) */
#define GO_MOTOR_COUNT  8

/* RIS 协议常量 */
#define GO_TX_FRAME_SIZE  17   /* 发送帧长度 */
#define GO_RX_FRAME_SIZE  16   /* 接收帧长度 */
#define GO_MOTOR_ID_BROADCAST  15  /* 广播 ID */

/* GO-8010 驱动私有上下文 */
typedef struct {
    uint8_t         bus_id;      /* UART 总线索引 (BSP_UART_2 或 BSP_UART_3) */
    uint8_t         motor_id;    /* RIS 帧里的老工程全局电机 ID */
    uint8_t         mode;        /* 当前模式: 0=锁定, 1=FOC闭环, 2=校准 */
    float           zero_offset; /* enable 时锁存的原始电机位置参考 (rad) */
    uint8_t         calibrated;  /* 是否已完成零位标定 */
    uint8_t         online;      /* 是否在线 (收到过有效反馈) */

    /* 发送缓冲 (当前期望指令) */
    float           cmd_pos;     /* 期望位置 (rad) */
    float           cmd_vel;     /* 期望速度 (rad/s) */
    float           cmd_tau;     /* 期望前馈力矩 (Nm) */
    float           cmd_kp;      /* 位置刚度 */
    float           cmd_kd;      /* 速度阻尼 */
    float           raw_pos_rad; /* 最近一次反馈的电机侧原始位置 (rad) */
    float           raw_vel_rads;/* 最近一次反馈的电机侧原始速度 (rad/s) */
    float           raw_tau_nm;  /* 最近一次反馈的电机侧原始力矩 (Nm) */

    /* 足底力传感器 */
    uint16_t        foot_force;  /* 原始力传感器数据 */
} go_drv_ctx_t;

typedef struct {
    uint8_t logical_id;
    uint8_t online;
    uint8_t calibrated;
    uint8_t bus_id;
    uint8_t motor_id;
    uint8_t mode;
    float raw_pos_rad;
    float zero_offset_rad;
    float joint_angle_rad;
    float cmd_pos_rad;
    float cmd_vel_rads;
    float cmd_tau_nm;
    float cmd_kp;
    float cmd_kd;
} go_debug_state_t;

/*
 * 初始化所有 GO-8010 电机驱动实例并绑定到 motor_registry
 * - 创建 motor_dev_t 实例
 * - 注册 UART RX 回调
 * - 调用 motor_registry_bind()
 */
app_err_t motor_go_init_all(void);

/*
 * 周期性发送函数：在 task_chassis 的控制周期中调用
 * 遍历所有 GO 电机，按总线拼接 TX 帧并通过 bsp_uart_send 发送
 */
app_err_t motor_go_send_all(void);

/*
 * 零位标定：对所有已在线但未标定的 GO 电机计算 zero_offset。
 * 在 GO_ZERO 阶段调用 — 电机保持锁定态，仅更新 ctx->zero_offset。
 * 返回值：至少一个电机成功标定返回 APP_OK；全部离线返回 APP_ERR_TIMEOUT。
 */
app_err_t motor_go_calibrate_all(void);

app_err_t motor_go_debug_get_state(uint16_t logical_id, go_debug_state_t* out);

/*
 * 获取指定总线上 GO 电机共享的 RX 回调
 * 供 bsp_uart_attach_rx 使用
 */
void motor_go_uart_rx_cb(bsp_uart_bus_t bus, const uint8_t* data,
                          uint32_t len, void* user);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_GO_H_ */
