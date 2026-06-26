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
#define PROTO_FUNC_ARM_CMD      0x11
#define PROTO_FUNC_GAIT_CMD     0x12
#define PROTO_FUNC_MIT_CMD      0x13   /* MIT 阻抗控制单个电机 (调试用) */
#define PROTO_FUNC_STATUS_REQ   0x20
#define PROTO_FUNC_USB_CDC_PING 0x21

/* 上行（下位机 → 上位机） */
#define PROTO_FUNC_STATE        0x80
#define PROTO_FUNC_MOTOR_STATE  0x81
#define PROTO_FUNC_LOG_MIRROR   0x82
#define PROTO_FUNC_IMU_STATE    0x83
#define PROTO_FUNC_USB_CDC_STATE 0x84
#define PROTO_FUNC_USB_CDC_PONG 0x85
#define PROTO_FUNC_EVENT        0x8F

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
    uint8_t req_kind;  /* 0=state, 1=motor, 2=stats */
} payload_status_req_t;   /* FuncID 0x20, len=1 */

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_PROTO_DEFS_H_ */
