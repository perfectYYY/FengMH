/*
 * leg_controller.h — 腿控制器：把 gait_output 翻译成"对每条腿的关节/轮电机指令"
 *
 * 持有每条腿的 hip / knee / wheel 电机句柄，把 gait_output_t 的足端
 * 目标先做 IK，再转换成 GO 关节位置指令和 M3508 轮毂指令。
 */
#ifndef APP_SERVICE_LEG_CONTROLLER_H_
#define APP_SERVICE_LEG_CONTROLLER_H_

#include "gait_if.h"
#include "motor_if.h"
#include "motor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    motor_dev_t* hip;
    motor_dev_t* knee;
    motor_dev_t* wheel;
} leg_actuators_t;

typedef struct {
    leg_actuators_t leg[GAIT_LEG_NUM];
    uint32_t        send_cnt;
    uint32_t        miss_cnt;  /* 某条腿电机为空时累加 */
} leg_controller_t;

typedef struct {
    uint8_t  enable;       /* 诊断位：轮子 MIT 已配置，初始化后保持 1 */
    uint8_t  reset;        /* GDB 写 1 重新锁存各轮参考位置 */
    uint8_t  drive_mask;   /* bit0..3: 当前使用有界积分 MIT 驱动的轮子 */
    uint8_t  hold_mask;    /* bit0..3: 当前使用 MIT 定点锁轮的轮子 */
    float    kp;           /* 输出轴 N·m/rad；DRIVE 中等效为速度积分增益 */
    float    kd;           /* 输出轴 N·m·s/rad；DRIVE 中等效为速度比例增益 */
    float    tau_limit_nm;
    float    pos_err_limit_rad;
    float    stance_tau_ff_nm; /* 支撑相按目标轮速方向给的滚阻前馈 */
    float    theta_ref_rad[GAIT_LEG_NUM];
    float    velocity_i_rad[GAIT_LEG_NUM]; /* DRIVE 的有界速度误差积分 */
    uint8_t  ref_valid[GAIT_LEG_NUM];
} leg_wheel_mit_debug_t;

extern volatile leg_wheel_mit_debug_t g_leg_wheel_mit;

typedef struct {
    uint8_t enable;              /* 1=向 GO 关节位置指令叠加 tau_ff */
    uint8_t compensate_leg_mass; /* 1=补偿腿/轮自身质量 */
    uint8_t compensate_payload;  /* 1=补偿 payload_mass_kg */
    uint8_t use_payload_com;     /* 1=按 payload_com_x/y 投影分配到支撑腿 */
    uint8_t payload_active_mask; /* bit0..3: 本周期参与 payload 分配的支撑腿 */
    uint8_t compensate_balance;  /* 1=补偿 balance_fz/mx/my 虚拟支撑力 */
    uint8_t balance_active_mask; /* bit0..3: 本周期参与 balance 分配的支撑腿 */
    uint8_t reserved[1];
    float scale;                 /* 总体比例，必要时可用 -1 反向验证符号 */
    float payload_mass_kg;       /* 机身/机械臂额外载荷总质量 */
    float payload_com_x_m;       /* 载荷等效质心在机体系前后投影，+x=前 */
    float payload_com_y_m;       /* 载荷等效质心在机体系横向投影，+y=左 */
    float support_half_length_m; /* 支撑足到机体中心前后距离 */
    float support_half_track_m;  /* 支撑足到机体中心横向距离 */
    float max_leg_payload_kg;    /* 单腿载荷限幅；<=0 表示不限 */
    float payload_applied_mass_kg;
    float payload_leg_mass_kg[GAIT_LEG_NUM];
    float balance_fz_n;          /* 额外垂向支撑力总和，+ 表示增加支撑 */
    float balance_mx_nm;         /* 期望机身 roll 方向力矩，Mx = sum(y_i * Fz_i) */
    float balance_my_nm;         /* 期望机身 pitch 方向力矩，My = sum(-x_i * Fz_i) */
    float max_balance_leg_force_n; /* 单腿 balance 力限幅；<=0 表示不限 */
    float balance_applied_fz_n;
    float balance_applied_mx_nm;
    float balance_applied_my_nm;
    float balance_leg_force_n[GAIT_LEG_NUM];
    float max_tau_nm;            /* 单关节前馈限幅 */
    float hip_tau_ff_nm[GAIT_LEG_NUM];
    float knee_tau_ff_nm[GAIT_LEG_NUM];
} leg_gravity_comp_debug_t;

extern volatile leg_gravity_comp_debug_t g_leg_gravity_comp;

/* 左前腿固定关节前馈，可在调试器中在线微调，单位为逻辑关节 N·m。 */
extern volatile uint8_t g_fl_fixed_tau_ff_enable;
extern volatile float g_fl_hip_fixed_tau_ff_nm;
extern volatile float g_fl_knee_fixed_tau_ff_nm;

void      leg_controller_init(leg_controller_t* lc);
/* 从 motor_registry 中按约定 logical id 装配 */
app_err_t leg_controller_bind_from_registry(leg_controller_t* lc);
/* 设置站立高度 (IK 解算参数) */
void      leg_controller_set_stand_height(float h);
void      leg_controller_set_output_options(uint8_t leg_mask,
                                            uint8_t enable_joints,
                                            uint8_t enable_wheels,
                                            float joint_kp,
                                            float joint_kd);
/* 应用步态输出：先做 IK 解算，再推送给电机 */
app_err_t leg_controller_apply(leg_controller_t* lc, const gait_output_t* o);
app_err_t leg_controller_apply_dt(leg_controller_t* lc, const gait_output_t* o, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
