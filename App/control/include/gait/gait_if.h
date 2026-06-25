/*
 * gait_if.h — 步态统一接口（策略模式）
 *
 * 任意步态都按 gait_if_t 实现，由 gait_machine 调度切换。
 * 上层只负责"喂时间"和"读输出"。所有运算严禁触碰外设，便于 host 测试。
 */
#ifndef APP_SERVICE_GAIT_IF_H_
#define APP_SERVICE_GAIT_IF_H_

#include "types.h"
#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 四足顺序：FL/FR/RL/RR；与 motor_registry 中腿的顺序一致 */
typedef enum {
    GAIT_LEG_FL = 0,
    GAIT_LEG_FR,
    GAIT_LEG_RL,
    GAIT_LEG_RR,
    GAIT_LEG_NUM
} gait_leg_t;

/*
 * 单条腿的步态目标。
 *
 * foot_x_m / foot_z_m 是机体系足端位移，IK 使用这两个字段解算关节角。
 * hip_rad / knee_rad 是兼容旧脚本关键帧的遗留字段；新步态代码应写 foot_*，
 * 再由 leg_controller/IK 填充关节目标。
 */
typedef struct {
    float foot_x_m;
    float foot_z_m;
    float hip_rad;
    float knee_rad;
    float wheel_rads;
    uint8_t in_stance;  /* 1=支撑相 0=摆动相 */
} gait_leg_target_t;

typedef struct {
    gait_leg_target_t leg[GAIT_LEG_NUM];
    float             phase;        /* [0,1) 当前主相位 */
    uint32_t          tick_count;   /* 本次步态从 enter 起累计的 update 次数 */
} gait_output_t;

/* 公共参数（高度/步长/周期/步高/占空比/相位偏移） */
typedef struct {
    float body_height_m;     /* 站立高度 */
    float step_length_m;     /* 单步长，纵向 */
    float turn_step_m;       /* yaw 步态转向步长差：正值右侧腿更向前、左侧腿更向后 */
    float step_height_m;     /* 抬腿离地高度 */
    float period_s;          /* 步态周期 */
    float duty;              /* 支撑相占空比 [0,1] */
    float phase_offset[GAIT_LEG_NUM]; /* 各腿相对主相位偏移 [0,1) */
    float touchdown_thresh;  /* 摆动相切到支撑相的相位阈值，多用于 sanity-check */
} gait_params_t;

struct gait_if_s;
typedef struct gait_if_s gait_if_t;

typedef struct {
    int (*init)      (gait_if_t* self);
    int (*set_param) (gait_if_t* self, const gait_params_t* p);
    int (*update)    (gait_if_t* self, float dt_s, gait_output_t* out);
    int (*exit)      (gait_if_t* self);
    const char* (*name)(void);
} gait_ops_t;

struct gait_if_s {
    const gait_ops_t* ops;
    gait_params_t     params;
    void*             ctx;
};

/* 工具：把 [-inf,+inf] 折回 [0,1) */
float gait_wrap01(float x);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_GAIT_IF_H_ */
