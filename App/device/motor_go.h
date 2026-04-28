/*
 * motor_go.h — GO-8010 (宇树) 电机驱动 vtable 接口
 *
 * 实现 motor_ops_t vtable，通过 bsp_uart (RS485) 与 GO-8010 通信。
 * RIS 协议：17B TX 帧 / 16B RX 帧，CRC16-CCITT 校验。
 *
 * 12 个 GO-8010 电机分属 2 条 RS485 总线：
 *   - USART2 (BSP_UART_2): FR_HIP, FR_KNEE, RR_HIP, RR_KNEE (4个)
 *   - USART3 (BSP_UART_3): FL_HIP, FL_KNEE, RL_HIP, RL_KNEE (4个)
 *
 * 每条总线上的电机按顺序 ID 0~3 寻址，驱动内部维护总线映射。
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
    uint8_t         motor_id;    /* 总线上的电机 ID (0~3) */
    uint8_t         mode;        /* 当前模式: 0=锁定, 1=FOC闭环, 2=校准 */
    float           zero_offset; /* 零位偏移 (rad)，enable 时自动记录 */
    uint8_t         calibrated;  /* 是否已完成零位标定 */
    uint8_t         online;      /* 是否在线 (收到过有效反馈) */

    /* 发送缓冲 (当前期望指令) */
    float           cmd_pos;     /* 期望位置 (rad) */
    float           cmd_vel;     /* 期望速度 (rad/s) */
    float           cmd_tau;     /* 期望前馈力矩 (Nm) */
    float           cmd_kp;      /* 位置刚度 */
    float           cmd_kd;      /* 速度阻尼 */

    /* 足底力传感器 */
    uint16_t        foot_force;  /* 原始力传感器数据 */
} go_drv_ctx_t;

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
 * 获取指定总线上 GO 电机共享的 RX 回调
 * 供 bsp_uart_attach_rx 使用
 */
void motor_go_uart_rx_cb(bsp_uart_bus_t bus, const uint8_t* data,
                          uint32_t len, void* user);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_GO_H_ */
