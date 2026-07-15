/*
 * proto_defs.h — USB CDC 协议常量 / FuncID / Payload 结构体
 *  帧结构: 0x55 0xAA | FuncID(1) | Len(1) | Payload(Len) | Checksum(1)
 *  校验: Byte0..Byte(4+Len-1) 累加取低 8 位
 */
#ifndef APP_SERVICE_PROTO_DEFS_H_
#define APP_SERVICE_PROTO_DEFS_H_

#include "types.h"
#include "config.h"
#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_HEAD1 0x55u
#define PROTO_HEAD2 0xAAu

/* 下行（上位机 → 下位机） */
#define PROTO_FUNC_CHASSIS_CMD  0x10
#define PROTO_FUNC_ARM_TARGET   0x11
#define PROTO_FUNC_ARM_CMD      PROTO_FUNC_ARM_TARGET  /* 兼容旧命名: 实际语义为 ARM_TARGET */
#define PROTO_FUNC_GAIT_CMD     0x12
#define PROTO_FUNC_MIT_CMD      0x13   /* MIT 阻抗控制单个电机 (调试用) */
#define PROTO_FUNC_ARM_PUMP     0x14
#define PROTO_FUNC_MODE_CMD     0x15
#define PROTO_FUNC_WHEEL_TEST   0x16   /* 绕过 gait/planner 的四轮 MIT 诊断 */
#define PROTO_FUNC_ARM_AUX_GPIO 0x17   /* 调试用：直接控制机械臂辅助 GPIO */
#define PROTO_FUNC_TROT_TEST_CONFIG 0x18 /* 模式3原地 TROT 测试参数 */
#define PROTO_FUNC_STATUS_REQ   0x20
#define PROTO_FUNC_USB_CDC_PING 0x21

/* 上行（下位机 → 上位机） */
#define PROTO_FUNC_STATE        0x80
#define PROTO_FUNC_MOTOR_STATE  0x81
#define PROTO_FUNC_LOG_MIRROR   0x82
#define PROTO_FUNC_IMU_STATE    0x83
#define PROTO_FUNC_USB_CDC_STATE 0x84
#define PROTO_FUNC_USB_CDC_PONG 0x85
#define PROTO_FUNC_ARM_FEEDBACK 0x86
#define PROTO_FUNC_ARM_MOTOR_ANGLES 0x87
#define PROTO_FUNC_WHEEL_STATE  0x88
#define PROTO_FUNC_CHASSIS_DIAG 0x89
#define PROTO_FUNC_TROT_TEST_DIAG 0x8A
#define PROTO_FUNC_MODE_FEEDBACK 0x8B
#define PROTO_FUNC_EVENT        0x8F

#define PROTO_STEER_MODE_OFF       0u
#define PROTO_STEER_MODE_ABSOLUTE  1u
#define PROTO_STEER_MODE_AUTO_HOLD 2u

#define PROTO_ROBOT_MODE_IDLE   0u
#define PROTO_ROBOT_MODE_NAV    1u
#define PROTO_ROBOT_MODE_ARM    2u
#define PROTO_ROBOT_MODE_ESTOP  3u
#define PROTO_ROBOT_MODE_ERROR  4u
#define PROTO_ROBOT_MODE_REAR_PLACE 5u
#define PROTO_ROBOT_MODE_MAX    PROTO_ROBOT_MODE_REAR_PLACE

typedef struct __attribute__((packed)) {
    float vx;   /* m/s  前后 */
    float vy;   /* m/s  侧向 */
    float wz;   /* rad/s 偏航 */
} payload_chassis_cmd_t;  /* FuncID 0x10, 旧帧 len=12 */

/* 扩展版: 增加 target_yaw + steer_mode (17 bytes)
 *   向后兼容: 旧协议 12 字节帧也接受, steer_mode 默认 AUTO_HOLD */
#define PROTO_CHASSIS_CMD_LEGACY_LEN   12u
#define PROTO_CHASSIS_CMD_EXT_LEN      17u

#define PROTO_GAIT_ACTION_STAND             0u
#define PROTO_GAIT_ACTION_TROT              1u
#define PROTO_GAIT_ACTION_SET_TROT_PARAMS   2u
#define PROTO_GAIT_ACTION_WALK              3u
#define PROTO_GAIT_ACTION_SET_WALK_PARAMS   4u

typedef struct __attribute__((packed)) {
    uint8_t action;      /* PROTO_GAIT_ACTION_* */
    uint8_t reserved[3];
    float body_height_m;
    float step_length_m;
    float step_height_m;
    float period_s;
    float duty;
    float phase_offset[GAIT_LEG_NUM];
    float touchdown_thresh;
    float blend_dur_s;
} payload_gait_cmd_t;  /* FuncID 0x12, len=48 */

typedef struct __attribute__((packed)) {
    uint8_t motor_id;    /* motor_logical_id_t: FL_WHEEL=2, RL_WHEEL=5, RR_WHEEL=8, FR_WHEEL=11 */
    float   pos_rad;     /* 目标位置 (输出轴 rad) */
    float   vel_rads;    /* 目标速度 (输出轴 rad/s) */
    float   kp;          /* 刚度增益 (N·m/rad)，= 0 时退化为纯速度/力矩模式 */
    float   kd;          /* 阻尼增益 (N·m·s/rad) */
    float   tau_ff_nm;   /* 力矩前馈 (输出轴 N·m) */
} payload_mit_cmd_t;   /* FuncID 0x13, len=21 */

typedef struct __attribute__((packed)) {
    uint8_t target_type;  /* 0=grasp, 1=place */
    float   x_m;          /* arm_base x, m */
    float   y_m;          /* arm_base y, m */
    float   z_m;          /* arm_base z, m */
} payload_arm_target_t; /* FuncID 0x11, len=13 */

#define PROTO_ARM_TARGET_GRASP 0u
#define PROTO_ARM_TARGET_PLACE 1u

typedef struct __attribute__((packed)) {
    uint8_t pump_on;      /* 1=vacuum on, 0=release */
} payload_arm_pump_t;    /* FuncID 0x14, len=1 */

typedef struct __attribute__((packed)) {
    uint8_t mode;         /* PROTO_ROBOT_MODE_* */
} payload_mode_cmd_t;    /* FuncID 0x15, len=1 */

#define PROTO_WHEEL_TEST_DISABLE        0u
#define PROTO_WHEEL_TEST_ENABLE         1u
#define PROTO_WHEEL_TEST_COAST          2u
#define PROTO_WHEEL_TEST_VERTICAL_DRIVE 3u
#define PROTO_DIAG_WHEEL_TEST_MODE_SHIFT 12u
#define PROTO_DIAG_WHEEL_TEST_MODE_MASK  0x3000u

typedef struct __attribute__((packed)) {
    uint8_t enable;       /* 0=退出锁轮，1=纯轮驱，2=自由推动，3=原地踏步轮驱 */
    uint8_t wheel_mask;   /* bit0..3 = FL/FR/RL/RR；未选轮目标速度为 0 */
    uint8_t reserved[2];
    float wheel_rads[GAIT_LEG_NUM]; /* FL/FR/RL/RR 输出轴 rad/s */
} payload_wheel_test_t;  /* FuncID 0x16, len=20；需持续刷新 */

typedef struct __attribute__((packed)) {
    float step_height_m;              /* 0..0.060 m */
    float period_s;                   /* 0.20..1.00 s */
    float duty;                       /* 0.50..0.85 */
    float foot_z_trim_m[GAIT_LEG_NUM]; /* FL/FR/RL/RR, -0.010..0.010 m */
} payload_trot_test_config_t; /* FuncID 0x18, len=28 */

#define PROTO_ARM_AUX_GPIO_PC8 0u
#define PROTO_ARM_AUX_GPIO_PC9 1u
#define PROTO_ARM_AUX_GPIO_PA8 2u
#define PROTO_ARM_AUX_GPIO_PA9 3u

typedef struct __attribute__((packed)) {
    uint8_t channel;  /* PROTO_ARM_AUX_GPIO_* */
    uint8_t on;       /* 1=on, 0=off */
} payload_arm_aux_gpio_t; /* FuncID 0x17, len=2 */

#define PROTO_ARM_STATE_IDLE    0u
#define PROTO_ARM_STATE_MOVING  1u
#define PROTO_ARM_STATE_REACHED 2u
#define PROTO_ARM_STATE_ERROR   3u

typedef struct __attribute__((packed)) {
    uint8_t arm_state;    /* PROTO_ARM_STATE_* */
    float   end_x_m;      /* current end-effector x in arm_base, m */
    float   end_y_m;      /* current end-effector y in arm_base, m */
    float   end_z_m;      /* current end-effector z in arm_base, m */
    float   theta1_rad;   /* base joint angle, rad */
} payload_arm_feedback_t; /* FuncID 0x86, len=17 */

typedef struct __attribute__((packed)) {
    uint8_t mode;                /* applied PROTO_ROBOT_MODE_* */
    uint8_t safety_stop_active;  /* task_safety_estop_active() */
} payload_mode_feedback_t; /* FuncID 0x8B, len=2 */

/* J1..J4 原始电机反馈角度；online_mask bit0..3 对应 J1..J4，角度单位 rad。 */
typedef struct __attribute__((packed)) {
    uint8_t online_mask;
    float   j1_angle_rad;
    float   j2_angle_rad;
    float   j3_angle_rad;
    float   j4_angle_rad;
} payload_arm_motor_angles_t; /* FuncID 0x87, len=17 */

/* 四轮同步实际转速；online_mask bit0..3 对应 FL/FR/RL/RR。 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    fl_velocity_rads;
    float    fr_velocity_rads;
    float    rl_velocity_rads;
    float    rr_velocity_rads;
    uint8_t  online_mask;
} payload_wheel_state_t; /* FuncID 0x88, len=21 */

/* BMI088 v2 接收诊断；主控通过 0x83 周期上行。 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint32_t sequence;
    float    roll_rad;
    float    pitch_rad;
    float    yaw_rad;
    float    gyro_z_rad_s;
    float    velocity_n_m_s;
    float    velocity_w_m_s;
    float    velocity_u_m_s;
    uint32_t accepted;
    uint32_t crc_errors;
    uint32_t skipped_samples;
    uint16_t age_ms;
    uint8_t  valid;
    uint8_t  reserved;
} payload_imu_state_t; /* FuncID 0x83, len=52 */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int16_t target_mrad_s[GAIT_LEG_NUM];
    int16_t filtered_mrad_s[GAIT_LEG_NUM];
    int16_t cmd_current_raw[GAIT_LEG_NUM];
    int16_t actual_current_raw[GAIT_LEG_NUM];
    int16_t yaw_mrad;
    int16_t gyro_z_mrad_s;
    int16_t effective_wz_mrad_s;
    int16_t slip_residual_mrad_s;
    uint16_t speed_scale_permille;
    uint16_t flags;
} payload_chassis_diag_t; /* FuncID 0x89, len=48 */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t phase_permille;                 /* 0..999 */
    uint8_t stance_mask;                     /* bit0..3 = FL/FR/RL/RR */
    uint8_t reserved;
    int16_t target_z_mm[GAIT_LEG_NUM];        /* FL/FR/RL/RR */
    int16_t joint_error_mrad[GAIT_LEG_NUM * 2]; /* 每腿 hip,knee */
} payload_trot_test_diag_t; /* FuncID 0x8A, len=32 */

typedef struct __attribute__((packed)) {
    uint8_t req_kind;  /* 0=state, 1=motor, 2=stats */
} payload_status_req_t;   /* FuncID 0x20, len=1 */

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PROTO_DEFS_H_ */
