/*
 * host_sim.h — PC-side terminal actuator for the real FengMH control stack.
 *
 * This module is compiled only into build_host/libfengmh_sim.dylib. It binds
 * virtual motors into motor_registry, runs the same chassis/leg controller code
 * used by firmware, and exposes the resulting motor/leg state to Python.
 */
#ifndef TOOLS_LEG_VIZ_HOST_SIM_H_
#define TOOLS_LEG_VIZ_HOST_SIM_H_

#include <stdint.h>
#include "gait_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t online;
    float angle_rad;
    float velocity_rads;
    float torque_nm;
    float pos_target_rad;
    float vel_target_rads;
    float kp;
    float kd;
    float tau_ff;
    uint32_t pos_calls;
    uint32_t vel_calls;
} host_motor_snapshot_t;

typedef struct {
    float hip_cmd_rad;
    float knee_cmd_rad;
    float hip_raw_rad;
    float knee_raw_rad;
    float foot_body_x_m;
    float foot_body_y_m;
    float foot_down_z_m;
    float knee_body_x_m;
    float knee_body_y_m;
    float knee_down_z_m;
    float crank_body_x_m;
    float crank_body_y_m;
    float crank_down_z_m;
    float lower_mount_body_x_m;
    float lower_mount_body_y_m;
    float lower_mount_down_z_m;
    uint8_t ik_ok;
} host_leg_pose_t;

int host_sim_init(void);
int host_sim_set_stand_height(float height_m);
int host_sim_apply_foot_targets(const float* dx_m, const float* dz_m, const float* wheel_rads);
int host_sim_set_trot_params(const gait_params_t* params);
int host_sim_play_trot(const gait_params_t* params, float blend_dur_s);
int host_sim_play_script(const char* name);
int host_sim_stop_script(void);
int host_sim_step(float dt_s);
const char* host_sim_active_gait_name(void);
int host_sim_get_motor_snapshot(int logical_id, host_motor_snapshot_t* out);
int host_sim_get_leg_pose(int leg_id, host_leg_pose_t* out);

#ifdef __cplusplus
}
#endif

#endif /* TOOLS_LEG_VIZ_HOST_SIM_H_ */
