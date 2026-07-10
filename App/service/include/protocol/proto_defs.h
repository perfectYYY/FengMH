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
#define PROTO_FUNC_EVENT        0x8F

#define PROTO_ROBOT_MODE_IDLE   0u
#define PROTO_ROBOT_MODE_NAV    1u
#define PROTO_ROBOT_MODE_ARM    2u
#define PROTO_ROBOT_MODE_ESTOP  3u
#define PROTO_ROBOT_MODE_ERROR  4u

typedef struct __attribute__((packed)) {
    float vx;   /* m/s  前后 */
    float vy;   /* m/s  侧向 */
    float wz;   /* rad/s 偏航 */
} payload_chassis_cmd_t;  /* FuncID 0x10, 旧帧 len=12 */

/* 扩展版: 增加 target_yaw + steer_mode (17 bytes)
 *   向后兼容: 旧协议 12 字节帧也接受, steer_mode 默认 0 */
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

#define PROTO_WHEEL_TEST_DISABLE 0u
#define PROTO_WHEEL_TEST_ENABLE  1u

typedef struct __attribute__((packed)) {
    uint8_t enable;       /* 0=退出并锁轮，1=进入直接轮驱测试 */
    uint8_t wheel_mask;   /* bit0..3 = FL/FR/RL/RR；未选轮目标速度为 0 */
    uint8_t reserved[2];
    float wheel_rads[GAIT_LEG_NUM]; /* FL/FR/RL/RR 输出轴 rad/s */
} payload_wheel_test_t;  /* FuncID 0x16, len=20；需持续刷新 */

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
    uint8_t req_kind;  /* 0=state, 1=motor, 2=stats */
} payload_status_req_t;   /* FuncID 0x20, len=1 */

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PROTO_DEFS_H_ */
