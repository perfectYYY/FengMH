/*
 * chassis_control.h - Readable chassis control pipeline.
 *
 * The app task owns RTOS timing and communication plumbing. This module owns
 * the robot behavior: mode selection, steering, gait switching, gait update,
 * IK dispatch, and motor command flushing.
 */
#ifndef APP_CONTROL_CHASSIS_CONTROL_H_
#define APP_CONTROL_CHASSIS_CONTROL_H_

#include <stdint.h>

#include "chassis_types.h"
#include "gait_if.h"
#include "script_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    float target_yaw_rad;
    uint8_t steer_mode; /* PROTO_STEER_MODE_* compatible values */
    uint32_t seq;
} chassis_control_command_t;

typedef struct {
    chassis_control_command_t command;
    uint32_t valid_frame_count;
    uint32_t last_rx_ms;
} chassis_control_input_t;

typedef struct {
    chassis_mode_t mode;
    chassis_gait_active_t active_gait;
    uint8_t online;
    uint8_t moving;
    uint8_t heading_hold_active;
    uint8_t wheel_saturated;
    uint16_t diagnostic_flags;
    float effective_wz_rad_s;
    float yaw_rad;
    float gyro_z_rad_s;
    float slip_residual_rad_s;
    float speed_scale;
    float stand_height_m;
    float wheel_rads[GAIT_LEG_NUM];
    gait_params_t gait_params;
} chassis_control_status_t;

enum {
    CHASSIS_DIAG_IMU_READY        = 1U << 0,
    CHASSIS_DIAG_HEADING_HOLD     = 1U << 1,
    CHASSIS_DIAG_WHEEL_SATURATED = 1U << 2,
    CHASSIS_DIAG_SLIP_WARNING     = 1U << 3,
    CHASSIS_DIAG_SLIP_ACTIVE      = 1U << 4,
    CHASSIS_DIAG_CURRENT_LIMITED  = 1U << 5,
};

typedef struct {
    uint8_t enable;             /* 1=把 roll/pitch 姿态误差转换为腿部 tau_ff */
    uint8_t stance_only;        /* 兼容字段；力矩补偿始终只分配到支撑腿 */
    uint8_t reserved[2];
    float scale;                /* 总体比例，必要时可用 -1 反向验证符号 */
    float half_length_m;        /* 腿接触点到机体中心的前后距离 */
    float half_track_m;         /* 腿接触点到机体中心线的横向距离 */
    float max_foot_z_m;         /* 旧 foot_z 模式字段；力矩模式不再使用 */
    float roll_kp_nm_per_rad;
    float pitch_kp_nm_per_rad;
    float roll_kd_nm_per_rads;
    float pitch_kd_nm_per_rads;
    float deadband_rad;
    float max_moment_nm;
    float max_leg_force_n;
    float smooth_tau_s;
    float roll_rad;
    float pitch_rad;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float target_mx_nm;
    float target_my_nm;
    float filtered_mx_nm;
    float filtered_my_nm;
    float foot_z_delta_m[GAIT_LEG_NUM];
} chassis_attitude_comp_debug_t;

extern volatile chassis_attitude_comp_debug_t g_chassis_attitude_comp;

typedef struct {
    uint8_t enable;             /* 1=根据机械臂末端位置更新腿部 payload 补偿 */
    uint8_t enable_leg_tau_ff;  /* 1=同步打开 g_leg_gravity_comp.enable */
    uint8_t prefer_measured;    /* 1=优先使用电机反馈 FK 得到的末端位置 */
    uint8_t include_link_mass;  /* 1=等效载荷包含 L2/L3 连杆质量 */
    uint8_t source_valid;
    uint8_t source_measured;
    uint8_t reserved[2];
    float scale;                /* 载荷质量比例，用于现场调参 */
    float end_to_com_ratio;     /* 末端点映射到等效质心的比例 */
    float arm_mount_x_m;        /* 机械臂基座相对机体中心偏置，+x=前 */
    float arm_mount_y_m;        /* 机械臂基座相对机体中心偏置，+y=左 */
    float support_half_length_m;
    float support_half_track_m;
    float smooth_tau_s;         /* 载荷/质心一阶滤波时间常数 */
    float max_leg_payload_kg;   /* 写入 leg 补偿的单腿载荷限幅 */
    float max_tau_nm;           /* 写入 leg 补偿的关节 tau_ff 限幅 */
    float source_end_x_m;
    float source_end_y_m;
    float source_end_z_m;
    float end_mass_kg;
    float target_total_mass_kg;
    float filtered_total_mass_kg;
    float target_com_x_m;
    float target_com_y_m;
    float filtered_com_x_m;
    float filtered_com_y_m;
} chassis_arm_load_comp_debug_t;

extern volatile chassis_arm_load_comp_debug_t g_chassis_arm_load_comp;

typedef struct {
    uint8_t active;
    uint8_t wheel_mask;
    uint8_t reserved[2];
    float wheel_rads[GAIT_LEG_NUM];
    uint32_t last_cmd_ms;
    uint32_t command_count;
    uint32_t timeout_count;
} chassis_wheel_test_debug_t;

extern volatile chassis_wheel_test_debug_t g_chassis_wheel_test;

void chassis_control_init(void);
void chassis_control_tick(const chassis_control_input_t* input,
                          float dt_s,
                          uint32_t now_ms);

void chassis_control_set_mode(chassis_mode_t mode);
chassis_mode_t chassis_control_get_mode(void);
chassis_gait_active_t chassis_control_get_gait_active(void);

int chassis_control_play_script(const script_t* script, float blend_dur_s);
int chassis_control_stop_script(float blend_dur_s);
int chassis_control_start_stand(float blend_dur_s);
int chassis_control_set_trot_params(const gait_params_t* params);
void chassis_control_get_trot_params(gait_params_t* out);
int chassis_control_start_trot(const gait_params_t* params, float blend_dur_s);
int chassis_control_set_walk_params(const gait_params_t* params);
void chassis_control_get_walk_params(gait_params_t* out);
int chassis_control_start_walk(const gait_params_t* params, float blend_dur_s);

void chassis_control_set_online_timeout_ms(uint32_t timeout_ms);
uint32_t chassis_control_get_online_timeout_ms(void);

int chassis_control_set_wheel_test(uint8_t enable,
                                   uint8_t wheel_mask,
                                   const float wheel_rads[GAIT_LEG_NUM],
                                   uint32_t now_ms);

const char* chassis_control_active_gait_name(void);

void chassis_control_reset_yaw(void);
float chassis_control_get_yaw(void);
float chassis_control_get_effective_wz(void);
void chassis_control_get_status(chassis_control_status_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CHASSIS_CONTROL_H_ */
