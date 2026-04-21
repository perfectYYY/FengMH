/*
 * motor_if.h — 统一电机抽象（vtable）
 *
 * 与 SYSTEM_DESIGN.md §4 对齐。所有电机实现（M3508/GO/达妙）必须提供
 * 以下 vtable，并通过 motor_registry 暴露给上层使用。
 */
#ifndef APP_DEVICE_MOTOR_IF_H_
#define APP_DEVICE_MOTOR_IF_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_UNKNOWN = 0,
    MOTOR_M3508,
    MOTOR_GO,
    MOTOR_DAMIAO
} motor_type_t;

typedef struct {
    uint16_t id;
    motor_type_t type;
    uint8_t  can_bus;
    uint32_t can_id;

    uint8_t  online;
    float    angle_rad;
    float    velocity_rads;
    float    torque_nm;
    float    temperature_c;

    app_tick_t last_rx_tick;
    uint32_t   rx_cnt;
    uint32_t   err_cnt;
} motor_state_t;

struct motor_dev_s;
typedef struct motor_dev_s motor_dev_t;

typedef struct {
    int (*set_current) (motor_dev_t*, float iq_a);
    int (*set_torque)  (motor_dev_t*, float tau_nm);
    int (*set_position)(motor_dev_t*, float pos, float vel, float kp, float kd, float tau_ff);
    int (*set_velocity)(motor_dev_t*, float vel_rads);
    int (*enable)      (motor_dev_t*);
    int (*disable)     (motor_dev_t*);
    int (*reset_fault) (motor_dev_t*);
    int (*feed_rx)     (motor_dev_t*, const uint8_t* data, uint8_t dlc);
} motor_ops_t;

struct motor_dev_s {
    motor_state_t      state;
    const motor_ops_t* ops;
    void*              drv_ctx;  /* 具体驱动私有上下文 */
};

/* 提供给 vtable 未实现项的默认"不支持"实现（避免 NULL 解引用） */
int motor_op_unsupported(motor_dev_t* dev, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_IF_H_ */
