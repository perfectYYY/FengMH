/*
 * motor_m3508.h — M3508/C620 (大疆) 电机驱动 vtable 接口
 *
 * 实现 motor_ops_t vtable，通过 bsp_fdcan 与 C620 电调通信。
 * C620 协议：0x200 控制帧 (8B, 4×int16_t 电流), 0x201~0x208 反馈帧。
 *
 * 4 个 M3508 电机分属 2 条 FDCAN 总线：
 *   - FDCAN1: FL_WHEEL (ID=1), FR_WHEEL (ID=2)
 *   - FDCAN2: RL_WHEEL (ID=3), RR_WHEEL (ID=4)
 *
 * 速度 PID 闭环使用 app_pid_t，默认参数 Kp=3.0, Ki=0.3, Kd=0.0
 * 减速比 187:1 自动在 vtable 中处理。
 */
#ifndef APP_DEVICE_MOTOR_M3508_H_
#define APP_DEVICE_MOTOR_M3508_H_

#include "motor_if.h"
#include "bsp_fdcan.h"
/* 使用带路径的 include，避免与 Core/Inc/pid.h (旧版 PID_Controller) 冲突 */
#include "service/pid/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* M3508 电机数量 (4 个轮毂电机) */
#define M3508_MOTOR_COUNT  4

/* C620 协议常量 */
#define M3508_TX_ID            0x200    /* 控制帧 CAN ID */
#define M3508_FB_ID_BASE       0x201    /* 反馈帧起始 CAN ID */
#define M3508_ENCODER_COUNTS   8192     /* 单圈编码器计数 */
#define M3508_HALF_ENCODER     4096     /* 半圈编码器 */
#define M3508_CURRENT_RAW_MAX  16384    /* 电流指令 raw 上限 */
#define M3508_CURRENT_LIMIT_A  20.0f    /* 满量程电流 (A) */
#define M3508_TORQUE_KT        0.01562f /* 转矩常数 N·m/A */
#define M3508_REDUCTION_RATIO  187.0f   /* 减速比 */

/* M3508 驱动私有上下文 */
typedef struct {
    uint8_t       bus_id;       /* FDCAN 总线索引 (BSP_FDCAN_1 或 BSP_FDCAN_2) */
    uint8_t       dji_id;       /* C620 DJI ID (1~4)，决定 0x200 帧中的字节偏移 */
    uint8_t       online;

    /* 编码器多圈解算 */
    uint16_t      ecd;          /* 当前编码器原始值 */
    uint16_t      last_ecd;     /* 上一次编码器值 */
    uint16_t      offset_ecd;   /* 编码器零偏 */
    int32_t       total_angle;  /* 累计编码器计数 (含圈数) */
    int16_t       round_cnt;    /* 累计圈数 */
    uint32_t      msg_cnt;      /* 接收帧计数 */

    /* 速度 PID (转子侧) */
    app_pid_t     speed_pid;
    float         target_vel_rads;  /* 目标速度 (输出轴 rad/s) */
    int16_t       cmd_current_raw;  /* 当前下发的电流 raw 指令 */

    /* 温度保护 */
    uint8_t       temp_limit_phase; /* 0=正常 1=限功率 2=停机 */
} m3508_drv_ctx_t;

/*
 * 初始化所有 M3508 电机驱动实例并绑定到 motor_registry
 * - 创建 motor_dev_t 实例
 * - 初始化速度 PID
 * - 注册 FDCAN RX 回调
 * - 调用 motor_registry_bind()
 */
app_err_t motor_m3508_init_all(void);

/*
 * 周期性发送函数：在 task_chassis 的控制周期中调用
 * 构造 0x200 控制帧，按总线通过 bsp_fdcan_send 发送
 */
app_err_t motor_m3508_send_all(void);

/*
 * FDCAN RX 回调：供 bsp_fdcan_attach_rx 使用
 */
void motor_m3508_fdcan_rx_cb(bsp_fdcan_bus_t bus,
                              const bsp_fdcan_frame_t* f,
                              void* user);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_M3508_H_ */
