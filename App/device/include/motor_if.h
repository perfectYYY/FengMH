/*
 * motor_if.h — 统一电机抽象（vtable）
 *
 * 所有电机实现（M3508/GO/达妙）通过同一组 vtable 暴露给控制层，
 * 再由 motor_registry 按 logical id 查找。
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
    float    foot_force_n;     /* GO 电机足底力传感器 (N) */

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
    /*
     * PROJECT-WIDE INVARIANT: disable implementations must be no-op and must
     * never emit a hardware-disable/zero-torque command. Use stand/hold via
     * the normal controller instead. This slot remains only for ABI stability.
     */
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

/* 默认实现：返回 APP_ERR_UNSUPPORTED */
static inline int motor_op_default_unsupported(motor_dev_t* dev) {
    (void)dev;
    return APP_ERR_UNSUPPORTED;
}

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_MOTOR_IF_H_ */
