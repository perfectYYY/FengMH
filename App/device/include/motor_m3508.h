/*
 * motor_m3508.h — M3508/C620 (大疆) 电机驱动 vtable 接口
 *
 * 实现 motor_ops_t vtable，通过 bsp_fdcan 与 C620 电调通信。
 * C620 协议：0x200 控制帧 (8B, 4×int16_t 电流), 0x201~0x208 反馈帧。
 *
 * 4 个 M3508 电机分属 2 条 FDCAN 总线（左右各一条）：
 *   - FDCAN1: FL_WHEEL (DJI_ID=1), RL_WHEEL (DJI_ID=2)  → 反馈 0x201, 0x202
 *   - FDCAN2: RR_WHEEL (DJI_ID=3), FR_WHEEL (DJI_ID=4)  → 反馈 0x203, 0x204
 * DJI_ID 全局连续：1-2 在 FDCAN1，3-4 在 FDCAN2；
 * ID=3,4 的 C620 固定读取 0x200 帧的 bytes[4,5] 和 bytes[6,7]。
 *
 * 速度 PI 闭环使用 app_pid_t，默认参数见 motor_m3508.c。
 * 减速比 268/17 ≈ 15.76 自动在 vtable 中处理。
 */
#ifndef APP_DEVICE_MOTOR_M3508_H_
#define APP_DEVICE_MOTOR_M3508_H_

#include "motor_if.h"
#include "bsp_fdcan.h"
/* 使用模块路径 include，避免与 Core/Inc/pid.h (旧版 PID_Controller) 冲突 */
#include "pid/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* M3508 电机数量 (4 个轮毂电机) */
#define M3508_MOTOR_COUNT  4

/* C620 协议常量 */
#define M3508_TX_ID            0x200    /* 控制帧 CAN ID: DJI ID 1~4 */
#define M3508_TX_ID_HIGH       0x1FF    /* 控制帧 CAN ID: DJI ID 5~8 */
#define M3508_FB_ID_BASE       0x201    /* 反馈帧起始 CAN ID */
#define M3508_ENCODER_COUNTS   8192     /* 单圈编码器计数 */
#define M3508_HALF_ENCODER     4096     /* 半圈编码器 */
#define M3508_CURRENT_RAW_MAX  16384    /* 电流指令 raw 上限 */
#define M3508_CURRENT_LIMIT_A  20.0f    /* 满量程电流 (A) */
#define M3508_TORQUE_KT        0.01562f /* 转矩常数 N·m/A */
#define M3508_REDUCTION_RATIO  (268.0f / 17.0f)  /* 减速比 M3508P (268:17) ≈ 15.76 */
#define M3508_POWER_LIMIT_W    150.0f   /* 功率保护上限 (W) */
#define M3508_EMA_ALPHA        0.3f     /* 速度 EMA 滤波系数：0=全平滑 1=无滤波 */
#define M3508_TRACE_CAPACITY   2048u    /* 运行态环形记录容量，供 OpenOCD 停机后 dump */
#define M3508_MIT_TAU_MAX_NM   2.0f     /* MIT 输出轴力矩保护上限，先保守验证 */
#define M3508_MIT_TAU_HARD_MAX_NM 4.0f  /* 调试 ramp 允许提高到的输出轴力矩硬上限 */
#define M3508_MIT_POS_ERR_MAX_RAD 0.5f  /* MIT 位置误差限幅，避免调试指令突跳 */
#define M3508_MIT_KP_MAX       40.0f    /* 输出轴 N·m/rad */
#define M3508_MIT_KD_MAX       8.0f     /* 输出轴 N·m·s/rad */

/* 控制模式 */
#define M3508_MODE_VELOCITY  0u   /* 速度环 */
#define M3508_MODE_POSITION  1u   /* 位置环 (级联速度环) */
#define M3508_MODE_TORQUE    2u   /* 力矩环 (前馈 + 电流 PI) */
#define M3508_MODE_CURRENT   3u   /* 直接电流控制 */
#define M3508_MODE_MIT       4u   /* MIT 阻抗控制: τ = kp·Δpos + kd·Δvel + τ_ff */

/* M3508 驱动私有上下文 */
typedef struct {
    uint8_t       bus_id;       /* FDCAN 总线索引 (BSP_FDCAN_1 或 BSP_FDCAN_2) */
    uint8_t       dji_id;       /* C620 DJI ID（全局 1-4）：1/2 在 FDCAN1，3/4 在 FDCAN2；slot=dji_id-1 决定 0x200 帧字节偏移 */
    uint8_t       online;
    uint8_t       ctrl_mode;    /* 当前控制模式 M3508_MODE_* */

    /* 编码器多圈解算 */
    uint16_t      ecd;          /* 当前编码器原始值 */
    uint16_t      last_ecd;     /* 上一次编码器值 */
    uint16_t      offset_ecd;   /* 编码器零偏 */
    int32_t       total_angle;  /* 累计编码器计数 (含圈数) */
    int16_t       round_cnt;    /* 累计圈数 */
    uint32_t      msg_cnt;      /* 接收帧计数 */

    /* 速度反馈 */
    int16_t       actual_current_raw;   /* C620 反馈电流 raw */
    float         filter_speed;         /* EMA 滤波速度 (转子侧 rpm) */

    /* 速度环 */
    app_pid_t     speed_pid;
    float         target_vel_rads;      /* 目标速度 (输出轴 rad/s) */
    int16_t       cmd_current_raw;      /* 当前下发的电流 raw 指令 */
    int32_t       velocity_hold_position; /* 速度目标为 0 时锁存的转子侧编码器位置 */
    uint8_t       velocity_hold_active;   /* 0 速位置保持是否已锁存 */

    /* 位置环状态 */
    int32_t       target_position;      /* 目标位置 (转子侧累计编码器计数) */
    float         pos_err_sum;          /* 位置环积分 */

    /* 力矩环状态 */
    float         target_torque_nm;     /* 目标力矩 (Nm, 输出轴) */
    float         trq_err_sum;          /* 力矩环积分 */

    /* MIT 阻抗控制参数 (输出轴单位) */
    float         mit_pos_des_rad;   /* 目标位置 (输出轴 rad) */
    float         mit_vel_des_rads;  /* 目标速度 (输出轴 rad/s) */
    float         mit_kp;            /* 刚度增益 (N·m/rad) */
    float         mit_kd;            /* 阻尼增益 (N·m·s/rad) */
    float         mit_tau_ff_nm;     /* 力矩前馈 (输出轴 N·m) */
    float         mit_tau_limit_nm;  /* 力矩限幅 (输出轴 N·m)，<=0 时使用默认上限 */
    float         mit_pos_err_limit_rad; /* 位置误差限幅，<=0 时使用默认上限 */
    float         mit_pos_err_rad;   /* 最近一次 MIT 位置误差，供 trace/GDB 调试 */
    float         mit_tau_cmd_nm;    /* 最近一次 MIT 输出力矩，供 trace/GDB 调试 */

    /* 温度保护 */
    uint8_t       temp_limit_phase; /* 0=正常 1=限功率 2=停机 */
} m3508_drv_ctx_t;

typedef struct {
    uint32_t tick_ms;
    int32_t  total_angle[M3508_MOTOR_COUNT];
    int16_t  cmd_current_raw[M3508_MOTOR_COUNT];
    int16_t  actual_current_raw[M3508_MOTOR_COUNT];
    int16_t  filter_speed_rpm[M3508_MOTOR_COUNT];
    int16_t  target_vel_mrad_s[M3508_MOTOR_COUNT];
    int16_t  mit_pos_des_mrad[M3508_MOTOR_COUNT];
    int16_t  mit_pos_err_mrad[M3508_MOTOR_COUNT];
    int16_t  mit_tau_cmd_mNm[M3508_MOTOR_COUNT];
    uint8_t  ctrl_mode[M3508_MOTOR_COUNT];
    uint8_t  online_mask;
} m3508_trace_sample_t;

typedef struct {
    uint32_t write_idx;
    uint32_t sample_count;
    uint32_t decim;
    uint8_t  enabled;
    uint8_t  reserved[3];
    m3508_trace_sample_t samples[M3508_TRACE_CAPACITY];
} m3508_trace_buffer_t;

extern volatile m3508_trace_buffer_t g_m3508_trace;

typedef struct {
    uint8_t  enable;        /* GDB 写 1 启动；完成/异常时固件自动清 0 */
    uint8_t  wheel_index;   /* 0=FL, 1=RL, 2=RR, 3=FR */
    uint8_t  active;        /* 固件内部状态 */
    uint8_t  done;          /* 完成后置 1，下一次启动前可由 GDB 清 0 */
    uint8_t  hold_after_done; /* 1=结束后保持 MIT 锁轮，0=结束后零电流 */
    uint8_t  reserved[3];
    uint32_t duration_ms;   /* 运行时长；0 表示不启动，防止误触发 */
    uint32_t elapsed_ms;    /* 已运行时间 */
    uint32_t last_tick_ms;  /* 固件内部真实时间戳 */
    float    omega_des_rads; /* 输出轴目标速度 rad/s */
    float    theta_ref_rad; /* ramp 内部积分位置参考 */
    float    kp;            /* 输出轴 N·m/rad */
    float    kd;            /* 输出轴 N·m·s/rad */
    float    tau_ff_nm;     /* 输出轴 N·m */
    float    tau_limit_nm;  /* 输出轴 N·m，<=0 使用默认 2Nm，最大 4Nm */
    float    pos_err_limit_rad; /* <=0 使用默认 0.5rad */
    float    start_angle_rad;
    float    final_angle_rad;
    int16_t  last_cmd_current_raw;
    int16_t  last_actual_current_raw;
    int16_t  last_speed_rpm;
} m3508_mit_ramp_debug_t;

extern volatile m3508_mit_ramp_debug_t g_m3508_mit_ramp;

void motor_m3508_trace_reset(uint32_t decim);
void motor_m3508_trace_enable(uint8_t enable);
app_err_t motor_m3508_set_mit_limits(motor_dev_t* dev,
                                     float tau_limit_nm,
                                     float pos_err_limit_rad);

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
