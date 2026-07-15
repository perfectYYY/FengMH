/*
 * arm_control.c - Arm-control facade.
 *
 * This stage runs kinematics, safe staged motion, trajectory planning, measured
 * feedback, and a gated Damiao command path.
 */
#include "arm_control.h"

#include "arm_gravity_comp.h"
#include "arm_kinematics.h"
#include "arm_legacy_compat.h"
#include "arm_pump.h"
#include "config.h"
#include "motor_damiao.h"
#include "motor_registry.h"

#include <math.h>
#include <string.h>

#define ARM_MOTOR_COUNT 4U

#define ARM_GRAVITY_KP_4340 0.0f
#define ARM_GRAVITY_KD_4340 0.05f

#define ARM_SETTLE_POSITION_TOLERANCE_RAD APP_ARM_DEG2RAD(1.0f)
#define ARM_SETTLE_STAGE_TOLERANCE_RAD    APP_ARM_DEG2RAD(3.0f)
#define ARM_SETTLE_EXTEND_TOLERANCE_RAD   APP_ARM_DEG2RAD(7.0f)
#define ARM_SETTLE_VELOCITY_TOLERANCE     0.05f
#define ARM_SETTLE_STABLE_TIME_MS         200U
#define ARM_SETTLE_TIMEOUT_MS             4000U
#define ARM_FINE_UPDATE_TOLERANCE_RAD     APP_ARM_DEG2RAD(0.20f)
#define ARM_FINE_SETTLE_TOLERANCE_RAD     APP_ARM_DEG2RAD(2.0f)
#define ARM_FINE_TIMEOUT_ACCEPT_RAD       APP_ARM_DEG2RAD(3.0f)

#define ARM_SAFE_MOVE_J2_MOTOR_RAD        APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J2_DEG)
#define ARM_SAFE_MOVE_J3_LOGICAL_RAD      APP_ARM_DEG2RAD(APP_ARM_SAFE_MOVE_J3_DEG)
#define ARM_FINE_MAX_BASE_DELTA_RAD       APP_ARM_DEG2RAD(8.0f)
#define ARM_FINE_MAX_ARM_DELTA_RAD        APP_ARM_DEG2RAD(12.0f)
#define ARM_REAR_PLACE_FORBIDDEN_J1_RAD   APP_ARM_DEG2RAD(-180.1203f)
#define ARM_J1_AVOIDANCE_EPS_RAD          APP_ARM_DEG2RAD(0.05f)

typedef enum {
    ARM_SAFE_MOVE_IDLE = 0,
    ARM_SAFE_MOVE_RETRACT = 1,
    ARM_SAFE_MOVE_ROTATE_BASE = 2,
    ARM_SAFE_MOVE_EXTEND = 3,
} arm_safe_move_stage_t;

typedef struct {
    arm_control_status_t status;
    payload_arm_feedback_t feedback;
    arm_motion_sample_t motion_sample;
    arm_motion_sample_t measured_sample;
    arm_joint_angles_t target_angles;
    arm_joint_angles_t final_target_angles;
    arm_joint_angles_t active_target_angles;
    arm_joint_angles_t measured_angles;
    motor_dev_t* motor[ARM_MOTOR_COUNT];
    uint8_t plan_pending;
    uint8_t settle_stable_active;
    uint32_t settle_start_ms;
    uint32_t settle_stable_ms;
    uint32_t control_last_ms;
    uint8_t error_latched;
    uint8_t j1_free_mode;
    uint8_t rear_place_avoidance_enabled;
} arm_control_ctx_t;

static arm_control_ctx_t s_arm;

volatile float debug_arm_host_target_j1_motor_deg;
volatile float debug_arm_selected_target_j1_motor_deg;
volatile uint8_t debug_arm_rear_place_avoidance_enabled;
volatile uint32_t debug_arm_grasp_j1_reject_count;

static const motor_logical_id_t S_ARM_MOTOR_IDS[ARM_MOTOR_COUNT] = {
    MOTOR_ID_ARM_J1,
    MOTOR_ID_ARM_J2,
    MOTOR_ID_ARM_J3,
    MOTOR_ID_ARM_J4,
};

static void stop_motion_and_target(void);
static void set_stage_target(arm_safe_move_stage_t stage,
                             const arm_joint_angles_t* target,
                             uint32_t now_ms);
static void start_safe_retract_stage(uint32_t now_ms);
static void start_safe_rotate_stage(uint32_t now_ms);
static void start_safe_extend_stage(uint32_t now_ms);

static void calculate_gravity_scaled(float theta2_geo_rad,
                                     float theta3_geo_rad,
                                     float theta4_geo_rad,
                                     float* tau2_nm,
                                     float* tau3_nm,
                                     float* tau4_nm) {
    arm_gravity_comp_calculate(theta2_geo_rad,
                               theta3_geo_rad,
                               theta4_geo_rad,
                               tau2_nm,
                               tau3_nm,
                               tau4_nm);
    if (tau2_nm) *tau2_nm *= dbg_gravity_scale_all * dbg_gravity_scale_2;
    if (tau3_nm) *tau3_nm *= dbg_gravity_scale_all * dbg_gravity_scale_3;
    if (tau4_nm) *tau4_nm *= dbg_gravity_scale_all * dbg_gravity_scale_4;
    debug_gravity_tau2 = tau2_nm ? *tau2_nm : 0.0f;
    debug_gravity_tau3 = tau3_nm ? *tau3_nm : 0.0f;
    debug_gravity_tau4 = tau4_nm ? *tau4_nm : 0.0f;
}

static void enforce_t4_coupling(arm_motion_sample_t* sample) {
    if (!sample) return;
    sample->position[3] = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET, sample->position[2]);
    sample->velocity[3] = -sample->velocity[2];
    sample->acceleration[3] = -sample->acceleration[2];
}

static void init_planned_home_sample(void) {
    memset(&s_arm.motion_sample, 0, sizeof(s_arm.motion_sample));
    enforce_t4_coupling(&s_arm.motion_sample);
}

static void reset_feedback(void) {
    memset(&s_arm.feedback, 0, sizeof(s_arm.feedback));
    s_arm.feedback.arm_state = PROTO_ARM_STATE_IDLE;
}

static uint8_t control_period_due(uint32_t now_ms) {
    if (s_arm.control_last_ms != 0U &&
        (now_ms - s_arm.control_last_ms) < APP_ARM_CONTROL_PERIOD_MS) {
        return 0U;
    }
    s_arm.control_last_ms = now_ms;
    return 1U;
}

static void sample_to_angles(const arm_motion_sample_t* sample,
                             arm_joint_angles_t* angles) {
    if (!sample || !angles) return;
    memset(angles, 0, sizeof(*angles));
    angles->theta1_motor_rad = sample->position[0];
    angles->theta2_motor_rad = sample->position[1];
    angles->theta3_motor_rad = sample->position[2];
    angles->theta4_motor_rad = sample->position[3];
    angles->theta1_geo_rad =
        angles->theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    angles->theta2_geo_rad =
        angles->theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    angles->theta3_geo_rad =
        angles->theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    angles->theta4_geo_rad =
        angles->theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
}

static void angles_to_positions(const arm_joint_angles_t* angles,
                                float position[ARM_MOTION_JOINT_COUNT]) {
    if (!angles || !position) return;
    position[0] = angles->theta1_motor_rad;
    position[1] = angles->theta2_motor_rad;
    position[2] = angles->theta3_motor_rad;
    position[3] = angles->theta4_motor_rad;
}

static void reset_settle_tracking(uint32_t now_ms) {
    s_arm.settle_start_ms = now_ms;
    s_arm.settle_stable_ms = now_ms;
    s_arm.settle_stable_active = 0U;
    s_arm.status.reached = 0U;
    s_arm.status.settle_error_rad = 0.0f;
    s_arm.status.settle_speed_rads = 0.0f;
}

static void current_reference_angles(arm_joint_angles_t* out) {
    if (!out) return;
    if (s_arm.status.motor_feedback_fresh) {
        *out = s_arm.measured_angles;
    } else {
        sample_to_angles(&s_arm.motion_sample, out);
    }
}

static uint8_t target_delta_inside_fine_window(const arm_joint_angles_t* target,
                                               const arm_joint_angles_t* reference) {
    if (!target || !reference) return 0U;
    if (fabsf(target->theta1_motor_rad - reference->theta1_motor_rad) >
        ARM_FINE_MAX_BASE_DELTA_RAD) return 0U;
    if (fabsf(target->theta2_motor_rad - reference->theta2_motor_rad) >
        ARM_FINE_MAX_ARM_DELTA_RAD) return 0U;
    if (fabsf(target->theta3_motor_rad - reference->theta3_motor_rad) >
        ARM_FINE_MAX_ARM_DELTA_RAD) return 0U;
    return 1U;
}

static uint8_t target_is_inside_fine_window(const arm_joint_angles_t* target) {
    arm_joint_angles_t reference;
    current_reference_angles(&reference);
    return target_delta_inside_fine_window(target, &reference);
}

static uint8_t rear_place_requires_staged_motion(uint8_t target_type) {
    return (s_arm.rear_place_avoidance_enabled &&
            target_type == PROTO_ARM_TARGET_PLACE) ? 1U : 0U;
}

static uint8_t joint_targets_match(const arm_joint_angles_t* a,
                                   const arm_joint_angles_t* b,
                                   float tolerance_rad) {
    if (!a || !b) return 0U;

    const float delta[3] = {
        fabsf(a->theta1_motor_rad - b->theta1_motor_rad),
        fabsf(a->theta2_motor_rad - b->theta2_motor_rad),
        fabsf(a->theta3_motor_rad - b->theta3_motor_rad),
    };

    for (uint32_t i = 0U; i < 3U; i++) {
        if (delta[i] > tolerance_rad) return 0U;
    }
    return 1U;
}

static uint8_t joint_target_is_valid(const arm_joint_angles_t* target) {
    if (!target) return 0U;

    const float position[ARM_MOTION_JOINT_COUNT] = {
        target->theta1_motor_rad,
        target->theta2_motor_rad,
        target->theta3_motor_rad,
        target->theta4_motor_rad,
    };
    for (uint32_t i = 0U; i < ARM_MOTION_JOINT_COUNT; i++) {
        if (!isfinite(position[i]) || fabsf(position[i]) > DAMIAO_P_MAX) {
            return 0U;
        }
    }

    const float motor2_physical =
        ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(target->theta2_motor_rad);
    const float motor3_physical =
        ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(target->theta3_motor_rad);
    if (motor2_physical < ARM_KINEMATICS_MOTOR2_PHYSICAL_MIN_RAD ||
        motor2_physical > ARM_KINEMATICS_MOTOR2_PHYSICAL_MAX_RAD ||
        motor3_physical < ARM_KINEMATICS_MOTOR3_PHYSICAL_MIN_RAD ||
        motor3_physical > ARM_KINEMATICS_MOTOR3_PHYSICAL_MAX_RAD) {
        return 0U;
    }

    return 1U;
}

static void normalize_theta1_to_reference(arm_joint_angles_t* target,
                                          float reference_theta1_rad) {
    if (!target) return;
    while ((target->theta1_motor_rad - reference_theta1_rad) > APP_ARM_PI) {
        target->theta1_motor_rad -= 2.0f * APP_ARM_PI;
    }
    while ((target->theta1_motor_rad - reference_theta1_rad) < -APP_ARM_PI) {
        target->theta1_motor_rad += 2.0f * APP_ARM_PI;
    }
    target->theta1_geo_rad =
        target->theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
}

static uint8_t theta1_segment_crosses_rear_place_forbidden(float start_rad,
                                                           float end_rad) {
    if (!isfinite(start_rad) || !isfinite(end_rad)) return 1U;

    const float low = (start_rad < end_rad) ? start_rad : end_rad;
    const float high = (start_rad < end_rad) ? end_rad : start_rad;
    int32_t first = (int32_t)floorf(
        (low - ARM_REAR_PLACE_FORBIDDEN_J1_RAD) / (2.0f * APP_ARM_PI)) - 1;
    int32_t last = (int32_t)ceilf(
        (high - ARM_REAR_PLACE_FORBIDDEN_J1_RAD) / (2.0f * APP_ARM_PI)) + 1;

    for (int32_t k = first; k <= last; k++) {
        const float forbidden =
            ARM_REAR_PLACE_FORBIDDEN_J1_RAD + (2.0f * APP_ARM_PI * (float)k);
        if (forbidden >= (low - ARM_J1_AVOIDANCE_EPS_RAD) &&
            forbidden <= (high + ARM_J1_AVOIDANCE_EPS_RAD)) {
            return 1U;
        }
    }
    return 0U;
}

static void avoid_rear_place_forbidden_theta1(arm_joint_angles_t* target,
                                              float reference_theta1_rad) {
    if (!target || !s_arm.rear_place_avoidance_enabled) return;

    normalize_theta1_to_reference(target, reference_theta1_rad);
    const float nearest = target->theta1_motor_rad;
    float best = nearest;
    float best_distance = INFINITY;
    uint8_t found_clear = 0U;

    for (int32_t k = -2; k <= 2; k++) {
        const float candidate = nearest + (2.0f * APP_ARM_PI * (float)k);
        const float distance = fabsf(candidate - reference_theta1_rad);
        if (theta1_segment_crosses_rear_place_forbidden(reference_theta1_rad,
                                                        candidate)) {
            continue;
        }
        if (!found_clear || distance < best_distance) {
            best = candidate;
            best_distance = distance;
            found_clear = 1U;
        }
    }

    if (!found_clear) {
        best = (nearest >= reference_theta1_rad) ?
               nearest - (2.0f * APP_ARM_PI) :
               nearest + (2.0f * APP_ARM_PI);
    }

    target->theta1_motor_rad = best;
    target->theta1_geo_rad =
        target->theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    debug_arm_selected_target_j1_motor_deg =
        APP_ARM_RAD2DEG(target->theta1_motor_rad);
}

static void select_theta1_route_to_reference(arm_joint_angles_t* target,
                                             float reference_theta1_rad) {
    if (!target) return;
    normalize_theta1_to_reference(target, reference_theta1_rad);
    avoid_rear_place_forbidden_theta1(target, reference_theta1_rad);
}

static float wrap_degrees_0_to_360(float angle_deg) {
    float wrapped = fmodf(angle_deg, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped;
}

uint8_t arm_control_grasp_j1_angle_forbidden(float theta1_motor_rad) {
    if (!isfinite(theta1_motor_rad)) return 1U;

    const float angle_deg = APP_ARM_RAD2DEG(theta1_motor_rad);
    const float wrapped_deg = wrap_degrees_0_to_360(angle_deg);
    const float a_min_wrapped = wrap_degrees_0_to_360(
        APP_ARM_GRASP_J1_FORBIDDEN_A_MIN_DEG);
    const float boundary_epsilon_deg = 0.001f;

    /* A=[-57,44] 跨越 0 度；旧 B=[-227,-136] 已按现场策略移除。 */
    if (wrapped_deg >= a_min_wrapped - boundary_epsilon_deg ||
        wrapped_deg <= APP_ARM_GRASP_J1_FORBIDDEN_A_MAX_DEG +
                       boundary_epsilon_deg) {
        return 1U;
    }
    return 0U;
}

app_err_t arm_control_validate_host_target(uint8_t target_type,
                                           float x_m,
                                           float y_m,
                                           float z_m) {
    if ((target_type != PROTO_ARM_TARGET_GRASP &&
         target_type != PROTO_ARM_TARGET_PLACE) ||
        !isfinite(x_m) || !isfinite(y_m) || !isfinite(z_m)) {
        return APP_ERR_INVALID_ARG;
    }

    const arm_pose_t target_pose = {
        .x_m = x_m,
        .y_m = y_m,
        .z_m = z_m,
        .pitch_rad = 0.0f,
    };
    arm_joint_angles_t target_angles;
    if (arm_kinematics_inverse(&ARM_KINEMATICS_DEFAULT_PARAMS,
                               &ARM_KINEMATICS_DEFAULT_OFFSET,
                               &target_pose,
                               &target_angles) != 0) {
        return APP_ERR_INVALID_ARG;
    }

    debug_arm_host_target_j1_motor_deg =
        APP_ARM_RAD2DEG(target_angles.theta1_motor_rad);
    if (target_type == PROTO_ARM_TARGET_GRASP &&
        arm_control_grasp_j1_angle_forbidden(target_angles.theta1_motor_rad)) {
        debug_arm_grasp_j1_reject_count++;
        return APP_ERR_INVALID_ARG;
    }
    return APP_OK;
}

static arm_joint_angles_t make_safe_joint_target(float theta1_motor_rad) {
    arm_joint_angles_t target;
    memset(&target, 0, sizeof(target));
    target.theta1_motor_rad = theta1_motor_rad;
    target.theta1_geo_rad =
        theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    target.theta2_motor_rad = ARM_SAFE_MOVE_J2_MOTOR_RAD;
    target.theta3_motor_rad = ARM_SAFE_MOVE_J3_LOGICAL_RAD;
    target.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET, target.theta3_motor_rad);
    target.theta2_geo_rad =
        target.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    target.theta3_geo_rad =
        target.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    target.theta4_geo_rad =
        target.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;
    return target;
}

static void store_target_pose_status(uint8_t target_type,
                                     const arm_joint_angles_t* target) {
    if (!target) return;

    arm_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS, target, &pose);

    s_arm.status.target_valid = 1U;
    s_arm.status.target_type = target_type;
    s_arm.status.target_x_m = pose.x_m;
    s_arm.status.target_y_m = pose.y_m;
    s_arm.status.target_z_m = pose.z_m;
    s_arm.status.target_seq++;
}

static float max_joint_error_to_target(const arm_joint_angles_t* measured,
                                       const arm_joint_angles_t* target) {
    if (!measured || !target) return INFINITY;

    const float err[3] = {
        fabsf(measured->theta1_motor_rad - target->theta1_motor_rad),
        fabsf(measured->theta2_motor_rad - target->theta2_motor_rad),
        fabsf(measured->theta3_motor_rad - target->theta3_motor_rad),
    };
    uint32_t first = s_arm.j1_free_mode ? 1U : 0U;
    float largest = err[first];
    for (uint32_t i = first + 1U; i < 3U; i++) {
        if (err[i] > largest) largest = err[i];
    }
    return largest;
}

static float max_measured_speed(void) {
    const float speed[3] = {
        fabsf(s_arm.measured_sample.velocity[0]),
        fabsf(s_arm.measured_sample.velocity[1]),
        fabsf(s_arm.measured_sample.velocity[2]),
    };
    uint32_t first = s_arm.j1_free_mode ? 1U : 0U;
    float largest = speed[first];
    for (uint32_t i = first + 1U; i < 3U; i++) {
        if (speed[i] > largest) largest = speed[i];
    }
    return largest;
}

static float settle_tolerance_for_stage(void) {
    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_EXTEND) {
        return ARM_SETTLE_EXTEND_TOLERANCE_RAD;
    }
    if (s_arm.status.safe_move_stage != ARM_SAFE_MOVE_IDLE) {
        return ARM_SETTLE_STAGE_TOLERANCE_RAD;
    }
    if (s_arm.status.fine_tracking_active) {
        return ARM_FINE_SETTLE_TOLERANCE_RAD;
    }
    return ARM_SETTLE_POSITION_TOLERANCE_RAD;
}

static void bind_arm_motors_from_registry(void) {
    s_arm.status.motor_bound_mask = 0U;
    for (uint32_t i = 0U; i < ARM_MOTOR_COUNT; i++) {
        s_arm.motor[i] = motor_get(S_ARM_MOTOR_IDS[i]);
        if (s_arm.motor[i] && s_arm.motor[i]->ops &&
            s_arm.motor[i]->ops->set_position) {
            s_arm.status.motor_bound_mask |= (uint8_t)(1U << i);
        }
    }
}

static uint8_t motor_state_is_fresh(const motor_state_t* state, uint32_t now_ms) {
    if (!state || !state->online || state->rx_cnt == 0U) return 0U;
    return ((now_ms - state->last_rx_tick) <= APP_ARM_MOTION_FEEDBACK_ABORT_MS) ?
           1U : 0U;
}

static uint8_t read_measured_angles(uint32_t now_ms,
                                    arm_joint_angles_t* angles,
                                    arm_motion_sample_t* sample) {
    s_arm.status.motor_online_mask = 0U;
    uint8_t fresh_mask = 0U;

    for (uint32_t i = 0U; i < ARM_MOTOR_COUNT; i++) {
        const motor_dev_t* motor = s_arm.motor[i];
        if (!motor) continue;
        if (motor->state.online) {
            s_arm.status.motor_online_mask |= (uint8_t)(1U << i);
        }
        if (motor_state_is_fresh(&motor->state, now_ms)) {
            fresh_mask |= (uint8_t)(1U << i);
        }
    }

    s_arm.status.motor_feedback_fresh =
        (fresh_mask == ((uint8_t)((1U << ARM_MOTOR_COUNT) - 1U))) ? 1U : 0U;
    if (!s_arm.status.motor_feedback_fresh || !angles || !sample) {
        return 0U;
    }

    memset(angles, 0, sizeof(*angles));
    angles->theta1_motor_rad = s_arm.motor[0]->state.angle_rad;
    angles->theta2_motor_rad =
        ARM_KINEMATICS_MOTOR2_TO_LOGICAL(s_arm.motor[1]->state.angle_rad);
    angles->theta3_motor_rad =
        ARM_KINEMATICS_MOTOR3_TO_LOGICAL(s_arm.motor[2]->state.angle_rad);
    angles->theta4_motor_rad = s_arm.motor[3]->state.angle_rad;
    angles->theta1_geo_rad =
        angles->theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    angles->theta2_geo_rad =
        angles->theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    angles->theta3_geo_rad =
        angles->theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    angles->theta4_geo_rad =
        angles->theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;

    sample->position[0] = angles->theta1_motor_rad;
    sample->position[1] = angles->theta2_motor_rad;
    sample->position[2] = angles->theta3_motor_rad;
    sample->position[3] = angles->theta4_motor_rad;
    sample->velocity[0] = s_arm.motor[0]->state.velocity_rads;
    sample->velocity[1] =
        ARM_KINEMATICS_MOTOR2_TO_LOGICAL(s_arm.motor[1]->state.velocity_rads);
    sample->velocity[2] =
        ARM_KINEMATICS_MOTOR3_TO_LOGICAL(s_arm.motor[2]->state.velocity_rads);
    sample->velocity[3] = s_arm.motor[3]->state.velocity_rads;
    memset(sample->acceleration, 0, sizeof(sample->acceleration));
    return 1U;
}

static void refresh_measured_feedback(uint32_t now_ms) {
    arm_pose_t pose;
    if (!read_measured_angles(now_ms, &s_arm.measured_angles, &s_arm.measured_sample)) {
        return;
    }

    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS,
                           &s_arm.measured_angles,
                           &pose);
    s_arm.status.measured_end_x_m = pose.x_m;
    s_arm.status.measured_end_y_m = pose.y_m;
    s_arm.status.measured_end_z_m = pose.z_m;
    s_arm.status.measured_theta1_rad = s_arm.measured_angles.theta1_geo_rad;
}

static void update_pose_feedback_from_sample(uint8_t arm_state,
                                             const arm_motion_sample_t* sample,
                                             uint8_t measured_source) {
    arm_joint_angles_t angles;
    arm_pose_t pose;
    sample_to_angles(sample, &angles);
    arm_kinematics_forward(&ARM_KINEMATICS_DEFAULT_PARAMS, &angles, &pose);

    s_arm.feedback.arm_state = arm_state;
    s_arm.feedback.end_x_m = pose.x_m;
    s_arm.feedback.end_y_m = pose.y_m;
    s_arm.feedback.end_z_m = pose.z_m;
    s_arm.feedback.theta1_rad = angles.theta1_geo_rad;

    if (measured_source) {
        s_arm.status.measured_end_x_m = pose.x_m;
        s_arm.status.measured_end_y_m = pose.y_m;
        s_arm.status.measured_end_z_m = pose.z_m;
        s_arm.status.measured_theta1_rad = angles.theta1_geo_rad;
    } else {
        s_arm.status.planned_end_x_m = pose.x_m;
        s_arm.status.planned_end_y_m = pose.y_m;
        s_arm.status.planned_end_z_m = pose.z_m;
        s_arm.status.planned_theta1_rad = angles.theta1_geo_rad;
    }
    s_arm.status.feedback_source_measured = measured_source;
}

static void update_feedback(uint8_t arm_state) {
    update_pose_feedback_from_sample(arm_state, &s_arm.motion_sample, 0U);
    if (s_arm.status.motor_feedback_fresh) {
        update_pose_feedback_from_sample(arm_state, &s_arm.measured_sample, 1U);
    }
}

static void set_stage_target(arm_safe_move_stage_t stage,
                             const arm_joint_angles_t* target,
                             uint32_t now_ms) {
    if (!target) return;
    arm_joint_angles_t planned = *target;
    if (s_arm.rear_place_avoidance_enabled) {
        arm_joint_angles_t reference;
        current_reference_angles(&reference);
        avoid_rear_place_forbidden_theta1(&planned, reference.theta1_motor_rad);
    }
    s_arm.target_angles = planned;
    s_arm.status.safe_move_stage = (uint8_t)stage;
    s_arm.status.target_pending = 1U;
    reset_settle_tracking(now_ms);
}

static void start_safe_retract_stage(uint32_t now_ms) {
    arm_joint_angles_t reference;
    current_reference_angles(&reference);
    arm_joint_angles_t retract = make_safe_joint_target(reference.theta1_motor_rad);
    s_arm.status.fine_tracking_active = 0U;
    set_stage_target(ARM_SAFE_MOVE_RETRACT, &retract, now_ms);
}

static void start_safe_rotate_stage(uint32_t now_ms) {
    arm_joint_angles_t rotate =
        make_safe_joint_target(s_arm.active_target_angles.theta1_motor_rad);
    set_stage_target(ARM_SAFE_MOVE_ROTATE_BASE, &rotate, now_ms);
}

static void start_safe_extend_stage(uint32_t now_ms) {
    set_stage_target(ARM_SAFE_MOVE_EXTEND, &s_arm.active_target_angles, now_ms);
}

static void plan_final_target(uint32_t now_ms) {
    s_arm.active_target_angles = s_arm.final_target_angles;

    if (rear_place_requires_staged_motion(s_arm.status.target_type)) {
        start_safe_retract_stage(now_ms);
    } else if (target_is_inside_fine_window(&s_arm.final_target_angles)) {
        s_arm.status.fine_tracking_active = 1U;
        set_stage_target(ARM_SAFE_MOVE_IDLE, &s_arm.final_target_angles, now_ms);
    } else {
        start_safe_retract_stage(now_ms);
    }
    s_arm.plan_pending = 0U;
}

static uint8_t advance_safe_stage(uint32_t now_ms) {
    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_RETRACT) {
        start_safe_rotate_stage(now_ms);
        return PROTO_ARM_STATE_MOVING;
    }

    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_ROTATE_BASE) {
        start_safe_extend_stage(now_ms);
        return PROTO_ARM_STATE_MOVING;
    }

    s_arm.status.safe_move_stage = ARM_SAFE_MOVE_IDLE;
    s_arm.status.fine_tracking_active = 1U;
    s_arm.status.reached = 1U;
    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_IDLE) {
        s_arm.status.safe_move_cycle_count++;
    }
    return PROTO_ARM_STATE_REACHED;
}

static uint8_t recover_from_settle_timeout(uint32_t now_ms, float error_rad) {
    if (s_arm.status.target_type == ARM_CONTROL_TARGET_INTERNAL_HOLD) {
        /* 固定等待姿态是持续保持策略，不因普通运动的4秒到位超时退回重补。 */
        reset_settle_tracking(now_ms);
        s_arm.status.last_result = APP_OK;
        return PROTO_ARM_STATE_MOVING;
    }

    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_RETRACT) {
        start_safe_retract_stage(now_ms);
        s_arm.status.last_result = APP_OK;
        return PROTO_ARM_STATE_MOVING;
    }

    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_ROTATE_BASE) {
        start_safe_rotate_stage(now_ms);
        s_arm.status.last_result = APP_OK;
        return PROTO_ARM_STATE_MOVING;
    }

    if (s_arm.status.safe_move_stage == ARM_SAFE_MOVE_EXTEND) {
        s_arm.status.safe_move_stage = ARM_SAFE_MOVE_IDLE;
        s_arm.status.safe_move_cycle_count++;
        if (target_is_inside_fine_window(&s_arm.active_target_angles)) {
            s_arm.status.fine_tracking_active = 1U;
            set_stage_target(ARM_SAFE_MOVE_IDLE,
                             &s_arm.active_target_angles,
                             now_ms);
        } else {
            start_safe_retract_stage(now_ms);
        }
        s_arm.status.last_result = APP_OK;
        return PROTO_ARM_STATE_MOVING;
    }

    if (s_arm.status.fine_tracking_active &&
        error_rad <= ARM_FINE_TIMEOUT_ACCEPT_RAD) {
        s_arm.status.reached = 1U;
        s_arm.status.last_result = APP_OK;
        return PROTO_ARM_STATE_REACHED;
    }

    stop_motion_and_target();
    s_arm.error_latched = 1U;
    s_arm.status.last_result = APP_ERR_TIMEOUT;
    return PROTO_ARM_STATE_ERROR;
}

static uint8_t update_settle_state(uint32_t now_ms,
                                   arm_trajectory_state_t trajectory_state) {
    if (s_arm.status.reached) {
        return PROTO_ARM_STATE_REACHED;
    }

    if (trajectory_state != ARM_TRAJECTORY_FINISHED) {
        return PROTO_ARM_STATE_MOVING;
    }

    if (!s_arm.status.motor_feedback_fresh) {
        return PROTO_ARM_STATE_MOVING;
    }

    const float error = max_joint_error_to_target(&s_arm.measured_angles,
                                                  &s_arm.target_angles);
    const float speed = max_measured_speed();
    const float tolerance = settle_tolerance_for_stage();
    s_arm.status.settle_error_rad = error;
    s_arm.status.settle_speed_rads = speed;

    if (error <= tolerance && speed <= ARM_SETTLE_VELOCITY_TOLERANCE) {
        if (!s_arm.settle_stable_active) {
            s_arm.settle_stable_active = 1U;
            s_arm.settle_stable_ms = now_ms;
        } else if ((now_ms - s_arm.settle_stable_ms) >=
                   ARM_SETTLE_STABLE_TIME_MS) {
            return advance_safe_stage(now_ms);
        }
    } else {
        s_arm.settle_stable_active = 0U;
    }

    if ((now_ms - s_arm.settle_start_ms) >= ARM_SETTLE_TIMEOUT_MS) {
        return recover_from_settle_timeout(now_ms, error);
    }

    return PROTO_ARM_STATE_MOVING;
}

static void stop_motion_and_target(void) {
    s_arm.status.target_valid = 0U;
    s_arm.status.target_pending = 0U;
    s_arm.plan_pending = 0U;
    s_arm.status.safe_move_stage = ARM_SAFE_MOVE_IDLE;
    s_arm.status.fine_tracking_active = 0U;
    s_arm.status.reached = 0U;
    s_arm.settle_stable_active = 0U;
    /* 运动故障不得自动释放真空负载；气泵由显式泵命令控制。 */
    arm_motion_stop(&s_arm.motion_sample);
    s_arm.status.trajectory_state = (uint8_t)ARM_TRAJECTORY_IDLE;
    s_arm.status.motion_duration_s = arm_motion_get_duration();
    s_arm.status.motion_progress = 0.0f;
}

app_err_t arm_control_set_gravity_mode(void) {
    s_arm.status.target_valid = 0U;
    s_arm.status.target_pending = 0U;
    s_arm.plan_pending = 0U;
    s_arm.status.safe_move_stage = ARM_SAFE_MOVE_IDLE;
    s_arm.status.fine_tracking_active = 0U;
    s_arm.status.reached = 0U;
    s_arm.settle_stable_active = 0U;
    arm_motion_stop(&s_arm.motion_sample);
    s_arm.status.trajectory_state = (uint8_t)ARM_TRAJECTORY_IDLE;
    s_arm.status.motion_duration_s = arm_motion_get_duration();
    s_arm.status.motion_progress = 0.0f;
    s_arm.error_latched = 0U;
    s_arm.status.last_result = APP_OK;
    if (s_arm.status.motor_feedback_fresh) {
        update_pose_feedback_from_sample(PROTO_ARM_STATE_IDLE,
                                         &s_arm.measured_sample,
                                         1U);
    } else {
        s_arm.feedback.arm_state = PROTO_ARM_STATE_IDLE;
    }
    return APP_OK;
}

static app_err_t start_pending_target(uint32_t now_ms) {
    float target_position[ARM_MOTION_JOINT_COUNT] = {0};
    angles_to_positions(&s_arm.target_angles, target_position);

    arm_motion_sample_t start = s_arm.motion_sample;
    if (arm_motion_get_state() == ARM_TRAJECTORY_MOVING) {
        (void)arm_motion_update(now_ms, &start);
        enforce_t4_coupling(&start);
    }
    if (s_arm.status.motor_output_enabled) {
        const uint8_t all_bound = (uint8_t)((1U << ARM_MOTOR_COUNT) - 1U);
        if (s_arm.status.motor_bound_mask != all_bound ||
            !s_arm.status.motor_feedback_fresh) {
            stop_motion_and_target();
            s_arm.error_latched = 1U;
            s_arm.status.last_result = APP_ERR_OFFLINE;
            s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
            return APP_ERR_OFFLINE;
        }
    }
    if (s_arm.status.motor_feedback_fresh) {
        start = s_arm.measured_sample;
    }

    const arm_motion_limits_t live_limits = {
        .max_velocity = {
            debug_motion_vmax_1, debug_motion_vmax_2,
            debug_motion_vmax_3, debug_motion_vmax_4,
        },
        .max_acceleration = {
            debug_motion_amax_1, debug_motion_amax_2,
            debug_motion_amax_3, debug_motion_amax_4,
        },
        .min_duration_s = 0.167f,
        .max_duration_s = 15.0f,
    };
    arm_motion_set_limits(&live_limits);

    int result = arm_motion_start(&start, target_position, now_ms);
    s_arm.status.last_result = result;
    s_arm.status.target_pending = 0U;
    s_arm.status.motion_duration_s = arm_motion_get_duration();
    s_arm.status.motion_progress = arm_motion_get_progress(now_ms);
    s_arm.status.trajectory_state = (uint8_t)arm_motion_get_state();

    if (result != 0) {
        stop_motion_and_target();
        s_arm.error_latched = 1U;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }

    reset_settle_tracking(now_ms);
    return APP_OK;
}

static int command_joint(uint32_t index,
                         float pos_rad,
                         float vel_rads,
                         float kp,
                         float kd,
                         float tau_ff_nm) {
    if (index >= ARM_MOTOR_COUNT ||
        !s_arm.motor[index] ||
        !s_arm.motor[index]->ops ||
        !s_arm.motor[index]->ops->set_position) {
        return APP_ERR_UNINIT;
    }

    return s_arm.motor[index]->ops->set_position(s_arm.motor[index],
                                                 pos_rad,
                                                 vel_rads,
                                                 kp,
                                                 kd,
                                                 tau_ff_nm);
}

static app_err_t command_arm_motors(arm_trajectory_state_t trajectory_state) {
    if (!s_arm.status.motor_output_enabled) return APP_OK;
    if (!s_arm.status.enabled || !s_arm.status.target_valid) return APP_OK;
    if (!s_arm.status.motor_feedback_fresh) return APP_ERR_OFFLINE;

    const uint8_t moving = (trajectory_state == ARM_TRAJECTORY_MOVING) ? 1U : 0U;
    const float j1_kp = s_arm.j1_free_mode ? ARM_GRAVITY_KP_4340 :
                        (moving ? debug_move_kp_4340 : debug_hold_kp_4340);
    const float j1_kd = s_arm.j1_free_mode ? ARM_GRAVITY_KD_4340 :
                        (moving ? debug_move_kd_4340 : debug_hold_kd_4340);
    const float j2_kp = moving ? debug_move_kp_4340 : debug_hold_kp_4340;
    const float j2_kd = moving ? debug_move_kd_4340 : debug_hold_kd_4340;
    const float j3_kp = moving ? debug_move_kp_4340 : debug_hold_kp_4340;
    const float j3_kd = moving ? debug_move_kd_4340 : debug_hold_kd_4340;
    const float j4_kp = moving ? debug_move_kp_4310 : debug_hold_kp_4310;
    const float j4_kd = moving ? debug_move_kd_4310 : debug_hold_kd_4310;

    const float pos2 = ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(s_arm.motion_sample.position[1]);
    const float vel2 = ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(s_arm.motion_sample.velocity[1]);
    const float tau2 = ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(s_arm.status.gravity_tau2_nm);
    const float pos3 = ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(s_arm.motion_sample.position[2]);
    const float vel3 = ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(s_arm.motion_sample.velocity[2]);
    const float tau3 = ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(s_arm.status.gravity_tau3_nm);

    s_arm.status.motor_tx_fail_mask = 0U;
    if (command_joint(2U, pos3, vel3, j3_kp, j3_kd, tau3) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x04U;
    }
    if (command_joint(1U, pos2, vel2, j2_kp, j2_kd, tau2) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x02U;
    }
    if (command_joint(0U,
                      s_arm.j1_free_mode ? s_arm.measured_angles.theta1_motor_rad :
                                           s_arm.motion_sample.position[0],
                      s_arm.j1_free_mode ? 0.0f :
                                           s_arm.motion_sample.velocity[0],
                      j1_kp,
                      j1_kd,
                      0.0f) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x01U;
    }
    if (command_joint(3U,
                      s_arm.motion_sample.position[3],
                      s_arm.motion_sample.velocity[3],
                      j4_kp,
                      j4_kd,
                      s_arm.status.gravity_tau4_nm) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x08U;
    }

    if (s_arm.status.motor_tx_fail_mask != 0U) {
        s_arm.status.motor_tx_fail_count++;
        /* 保持原工程语义：记录本周期失败，下周期继续控制。 */
    }
    return APP_OK;
}

static app_err_t command_idle_gravity_hold_motors(void) {
    if (!s_arm.status.motor_output_enabled) return APP_OK;
    if (!s_arm.status.enabled) return APP_OK;
    if (!s_arm.status.motor_feedback_fresh) return APP_OK;

    arm_joint_angles_t target = s_arm.measured_angles;
    target.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET, target.theta3_motor_rad);
    target.theta4_geo_rad =
        target.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;

    calculate_gravity_scaled(s_arm.measured_angles.theta2_geo_rad,
                             s_arm.measured_angles.theta3_geo_rad,
                             s_arm.measured_angles.theta4_geo_rad,
                             &s_arm.status.gravity_tau2_nm,
                             &s_arm.status.gravity_tau3_nm,
                             &s_arm.status.gravity_tau4_nm);

    const float pos2 = ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(target.theta2_motor_rad);
    const float tau2 = ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(s_arm.status.gravity_tau2_nm);
    const float pos3 = ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(target.theta3_motor_rad);
    const float tau3 = ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(s_arm.status.gravity_tau3_nm);

    s_arm.status.motor_tx_fail_mask = 0U;
    if (command_joint(2U, pos3, 0.0f,
                      ARM_GRAVITY_KP_4340, ARM_GRAVITY_KD_4340, tau3) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x04U;
    }
    if (command_joint(1U, pos2, 0.0f,
                      ARM_GRAVITY_KP_4340, ARM_GRAVITY_KD_4340, tau2) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x02U;
    }
    if (command_joint(0U, target.theta1_motor_rad, 0.0f,
                      ARM_GRAVITY_KP_4340, ARM_GRAVITY_KD_4340, 0.0f) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x01U;
    }
    if (command_joint(3U, target.theta4_motor_rad, 0.0f,
                      debug_gravity_kp_4, debug_gravity_kd_4,
                      s_arm.status.gravity_tau4_nm) != APP_OK) {
        s_arm.status.motor_tx_fail_mask |= 0x08U;
    }

    if (s_arm.status.motor_tx_fail_mask != 0U) {
        s_arm.status.motor_tx_fail_count++;
    }
    return APP_OK;
}

static void update_motion_gravity(void) {
    arm_joint_angles_t angles;
    if (s_arm.status.motor_feedback_fresh) {
        angles = s_arm.measured_angles;
    } else {
        sample_to_angles(&s_arm.motion_sample, &angles);
    }
    calculate_gravity_scaled(angles.theta2_geo_rad,
                             angles.theta3_geo_rad,
                             angles.theta4_geo_rad,
                             &s_arm.status.gravity_tau2_nm,
                             &s_arm.status.gravity_tau3_nm,
                             &s_arm.status.gravity_tau4_nm);
}

app_err_t arm_control_init(void) {
    memset(&s_arm, 0, sizeof(s_arm));
    debug_arm_host_target_j1_motor_deg = 0.0f;
    debug_arm_grasp_j1_reject_count = 0U;
    init_planned_home_sample();
    arm_gravity_comp_init();
    (void)arm_pump_init();
    arm_motion_init(NULL);
    arm_motion_stop(&s_arm.motion_sample);
    s_arm.status.trajectory_state = (uint8_t)ARM_TRAJECTORY_IDLE;
    s_arm.status.motor_output_enabled =
        APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE ? 1U : 0U;
    s_arm.status.enabled = 1U;
    bind_arm_motors_from_registry();
    reset_feedback();
    return APP_OK;
}

app_err_t arm_control_set_enabled(uint8_t enabled) {
    s_arm.status.enabled = enabled ? 1U : 0U;
    if (!s_arm.status.enabled) {
        stop_motion_and_target();
        s_arm.error_latched = 0U;
        reset_feedback();
    }
    return APP_OK;
}

app_err_t arm_control_set_motor_output_enabled(uint8_t enabled) {
    s_arm.status.motor_output_enabled = enabled ? 1U : 0U;
    if (!s_arm.status.motor_output_enabled) {
        s_arm.status.motor_tx_fail_mask = 0U;
    }
    return APP_OK;
}

app_err_t arm_control_set_rear_place_avoidance(uint8_t enabled) {
    s_arm.rear_place_avoidance_enabled = enabled ? 1U : 0U;
    debug_arm_rear_place_avoidance_enabled = s_arm.rear_place_avoidance_enabled;
    return APP_OK;
}

app_err_t arm_control_set_j1_free_mode(uint8_t enabled) {
    s_arm.j1_free_mode = enabled ? 1U : 0U;
    return APP_OK;
}

app_err_t arm_control_set_target(uint8_t target_type, float x_m, float y_m, float z_m) {
    if (target_type != PROTO_ARM_TARGET_GRASP &&
        target_type != PROTO_ARM_TARGET_PLACE) {
        stop_motion_and_target();
        s_arm.error_latched = 1U;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }
    if (!isfinite(x_m) || !isfinite(y_m) || !isfinite(z_m)) {
        stop_motion_and_target();
        s_arm.status.last_result = -1;
        s_arm.error_latched = 1U;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }

    arm_pose_t target_pose = {
        .x_m = x_m,
        .y_m = y_m,
        .z_m = z_m,
        .pitch_rad = 0.0f,
    };
    arm_joint_angles_t target_angles;
    int ik_result = arm_kinematics_inverse(&ARM_KINEMATICS_DEFAULT_PARAMS,
                                           &ARM_KINEMATICS_DEFAULT_OFFSET,
                                           &target_pose,
                                           &target_angles);
    s_arm.status.last_result = ik_result;
    if (ik_result != 0) {
        if (!s_arm.status.target_valid &&
            arm_motion_get_state() != ARM_TRAJECTORY_MOVING) {
            stop_motion_and_target();
            s_arm.error_latched = 1U;
            s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        }
        return APP_ERR_INVALID_ARG;
    }

    arm_joint_angles_t reference_angles;
    current_reference_angles(&reference_angles);
    select_theta1_route_to_reference(&target_angles,
                                     reference_angles.theta1_motor_rad);
    debug_arm_selected_target_j1_motor_deg =
        APP_ARM_RAD2DEG(target_angles.theta1_motor_rad);

    const uint8_t had_active_target =
        (s_arm.status.target_valid && !s_arm.error_latched) ? 1U : 0U;
    s_arm.status.target_valid = 1U;
    s_arm.status.target_type = target_type;
    s_arm.status.target_x_m = x_m;
    s_arm.status.target_y_m = y_m;
    s_arm.status.target_z_m = z_m;
    s_arm.status.target_seq++;
    s_arm.final_target_angles = target_angles;
    s_arm.error_latched = 0U;

    if (!had_active_target || s_arm.plan_pending) {
        s_arm.plan_pending = 1U;
        s_arm.status.reached = 0U;
        return APP_OK;
    }

    const uint32_t now_ms = s_arm.status.last_tick_ms;
    const arm_safe_move_stage_t stage =
        (arm_safe_move_stage_t)s_arm.status.safe_move_stage;

    if (stage == ARM_SAFE_MOVE_RETRACT) {
        s_arm.active_target_angles = target_angles;
        return APP_OK;
    }

    if (stage == ARM_SAFE_MOVE_ROTATE_BASE) {
        s_arm.active_target_angles = target_angles;
        arm_joint_angles_t rotate =
            make_safe_joint_target(target_angles.theta1_motor_rad);
        if (joint_targets_match(&rotate,
                                &s_arm.target_angles,
                                ARM_FINE_UPDATE_TOLERANCE_RAD)) {
            return APP_OK;
        }
        set_stage_target(ARM_SAFE_MOVE_ROTATE_BASE, &rotate, now_ms);
        return APP_OK;
    }

    if (stage == ARM_SAFE_MOVE_EXTEND) {
        const arm_joint_angles_t previous_target = s_arm.active_target_angles;
        s_arm.active_target_angles = target_angles;
        if (!target_delta_inside_fine_window(&target_angles,
                                             &previous_target)) {
            start_safe_retract_stage(now_ms);
            return APP_OK;
        }
        if (joint_targets_match(&target_angles,
                                &s_arm.target_angles,
                                ARM_FINE_UPDATE_TOLERANCE_RAD)) {
            return APP_OK;
        }
        set_stage_target(ARM_SAFE_MOVE_EXTEND, &target_angles, now_ms);
        return APP_OK;
    }

    if (s_arm.status.fine_tracking_active) {
        const arm_joint_angles_t previous_target = s_arm.active_target_angles;
        s_arm.active_target_angles = target_angles;
        if (rear_place_requires_staged_motion(target_type)) {
            start_safe_retract_stage(now_ms);
            return APP_OK;
        }
        if (!target_delta_inside_fine_window(&target_angles,
                                             &previous_target)) {
            start_safe_retract_stage(now_ms);
            return APP_OK;
        }
        if (joint_targets_match(&target_angles,
                                &s_arm.target_angles,
                                ARM_FINE_UPDATE_TOLERANCE_RAD)) {
            return APP_OK;
        }
        set_stage_target(ARM_SAFE_MOVE_IDLE, &target_angles, now_ms);
        return APP_OK;
    }

    s_arm.active_target_angles = target_angles;
    if (rear_place_requires_staged_motion(target_type)) {
        start_safe_retract_stage(now_ms);
    } else if (target_is_inside_fine_window(&target_angles)) {
        s_arm.status.fine_tracking_active = 1U;
        if (joint_targets_match(&target_angles,
                                &s_arm.target_angles,
                                ARM_FINE_UPDATE_TOLERANCE_RAD)) {
            return APP_OK;
        }
        set_stage_target(ARM_SAFE_MOVE_IDLE, &target_angles, now_ms);
    } else {
        start_safe_retract_stage(now_ms);
    }
    return APP_OK;
}

app_err_t arm_control_set_joint_target(uint8_t target_type,
                                       const arm_joint_angles_t* target) {
    if (target_type != PROTO_ARM_TARGET_GRASP &&
        target_type != PROTO_ARM_TARGET_PLACE &&
        target_type != ARM_CONTROL_TARGET_INTERNAL_HOLD) {
        stop_motion_and_target();
        s_arm.error_latched = 1U;
        s_arm.status.last_result = APP_ERR_INVALID_ARG;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }
    if (!target) {
        s_arm.status.last_result = APP_ERR_NULL_PTR;
        return APP_ERR_NULL_PTR;
    }

    arm_joint_angles_t requested = *target;
    requested.theta4_motor_rad = arm_kinematics_compute_t4_from_t3(
        &ARM_KINEMATICS_DEFAULT_OFFSET, requested.theta3_motor_rad);
    requested.theta1_geo_rad =
        requested.theta1_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta1_offset_rad;
    requested.theta2_geo_rad =
        requested.theta2_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta2_offset_rad;
    requested.theta3_geo_rad =
        requested.theta3_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta3_offset_rad;
    requested.theta4_geo_rad =
        requested.theta4_motor_rad + ARM_KINEMATICS_DEFAULT_OFFSET.theta4_offset_rad;

    arm_joint_angles_t reference_angles;
    current_reference_angles(&reference_angles);
    select_theta1_route_to_reference(&requested,
                                     reference_angles.theta1_motor_rad);
    debug_arm_selected_target_j1_motor_deg =
        APP_ARM_RAD2DEG(requested.theta1_motor_rad);

    if (!joint_target_is_valid(&requested)) {
        s_arm.status.last_result = APP_ERR_INVALID_ARG;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }

    if (s_arm.status.motor_output_enabled &&
        !s_arm.status.motor_feedback_fresh) {
        s_arm.status.last_result = APP_ERR_OFFLINE;
        return APP_ERR_OFFLINE;
    }

    s_arm.final_target_angles = requested;
    s_arm.active_target_angles = requested;
    store_target_pose_status(target_type, &requested);
    s_arm.error_latched = 0U;
    s_arm.plan_pending = 0U;
    s_arm.status.safe_move_stage = ARM_SAFE_MOVE_IDLE;
    s_arm.status.fine_tracking_active = 0U;
    s_arm.status.reached = 0U;
    set_stage_target(ARM_SAFE_MOVE_IDLE, &requested, s_arm.status.last_tick_ms);
    s_arm.status.last_result = APP_OK;
    return APP_OK;
}

app_err_t arm_control_set_pump(uint8_t pump_on) {
    if (pump_on > 1U) {
        s_arm.error_latched = 1U;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_INVALID_ARG;
    }

    s_arm.status.pump_on = pump_on;
    s_arm.status.pump_seq++;
    app_err_t err = arm_pump_set(pump_on);
    if (err != APP_OK) {
        s_arm.error_latched = 1U;
        s_arm.status.last_result = err;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return err;
    }

    arm_gravity_comp_set_payload_state(pump_on ?
                                       ARM_GRAVITY_PAYLOAD_LOADED :
                                       ARM_GRAVITY_PAYLOAD_EMPTY);
    return APP_OK;
}

app_err_t arm_control_tick(float dt_s, uint32_t now_ms) {
    (void)dt_s;
    s_arm.status.last_tick_ms = now_ms;
    bind_arm_motors_from_registry();
    refresh_measured_feedback(now_ms);

    if (!s_arm.status.enabled) {
        reset_feedback();
        return APP_OK;
    }

    if (s_arm.status.motor_output_enabled) {
        (void)motor_damiao_process(now_ms);
    }

    if (!control_period_due(now_ms)) {
        return APP_OK;
    }

    if (s_arm.plan_pending) {
        plan_final_target(now_ms);
    }

    if (s_arm.status.target_pending) {
        app_err_t start_err = start_pending_target(now_ms);
        if (start_err != APP_OK) {
            return start_err;
        }
    }

    if (!s_arm.status.target_valid) {
        if (s_arm.status.motor_feedback_fresh) {
            update_pose_feedback_from_sample(s_arm.error_latched ?
                                             PROTO_ARM_STATE_ERROR :
                                             PROTO_ARM_STATE_IDLE,
                                             &s_arm.measured_sample,
                                             1U);
            app_err_t hold_err = command_idle_gravity_hold_motors();
            if (hold_err != APP_OK) {
                s_arm.error_latched = 1U;
                s_arm.status.last_result = hold_err;
                s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
                return hold_err;
            }
            if (s_arm.error_latched) {
                s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
            }
            return APP_OK;
        }
        reset_feedback();
        return APP_OK;
    }

    if (s_arm.status.motor_output_enabled && !s_arm.status.motor_feedback_fresh) {
        stop_motion_and_target();
        s_arm.error_latched = 1U;
        s_arm.status.last_result = APP_ERR_OFFLINE;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return APP_ERR_OFFLINE;
    }

    arm_trajectory_state_t trajectory_state =
        arm_motion_update(now_ms, &s_arm.motion_sample);
    enforce_t4_coupling(&s_arm.motion_sample);
    s_arm.status.trajectory_state = (uint8_t)trajectory_state;
    s_arm.status.motion_duration_s = arm_motion_get_duration();
    s_arm.status.motion_progress = arm_motion_get_progress(now_ms);

    update_feedback(PROTO_ARM_STATE_MOVING);
    /* 每个控制周期只更新一次，并优先使用实测角，与原工程一致。 */
    update_motion_gravity();

    app_err_t command_err = command_arm_motors(trajectory_state);
    if (command_err != APP_OK) {
        stop_motion_and_target();
        s_arm.error_latched = 1U;
        s_arm.status.last_result = command_err;
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return command_err;
    }

    uint8_t arm_state = update_settle_state(now_ms, trajectory_state);
    update_feedback(arm_state);
    if (arm_state == PROTO_ARM_STATE_ERROR) {
        s_arm.feedback.arm_state = PROTO_ARM_STATE_ERROR;
        return (app_err_t)s_arm.status.last_result;
    }
    return APP_OK;
}

void arm_control_get_feedback(payload_arm_feedback_t* out) {
    if (!out) return;
    *out = s_arm.feedback;
}

void arm_control_get_status(arm_control_status_t* out) {
    if (!out) return;
    *out = s_arm.status;
}
