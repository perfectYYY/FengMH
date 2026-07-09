/*
 * arm_kinematics.h - Four-joint arm kinematics.
 *
 * Ported from damiao_new1/Core/Src/arm_kinematics.c, with external units
 * normalized to meters and radians for the FengMH control layer.
 */
#ifndef APP_CONTROL_ARM_KINEMATICS_H_
#define APP_CONTROL_ARM_KINEMATICS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ARM_PI 3.14159265358979323846f
#define APP_ARM_RAD2DEG(rad) ((rad) * 180.0f / APP_ARM_PI)
#define APP_ARM_DEG2RAD(deg) ((deg) * APP_ARM_PI / 180.0f)

#define ARM_KINEMATICS_MOTOR2_DIR (1.0f)
#define ARM_KINEMATICS_MOTOR3_DIR (-1.0f)

#define ARM_KINEMATICS_MOTOR2_TO_PHYSICAL(x) (ARM_KINEMATICS_MOTOR2_DIR * (x))
#define ARM_KINEMATICS_MOTOR2_TO_LOGICAL(x)  (ARM_KINEMATICS_MOTOR2_DIR * (x))
#define ARM_KINEMATICS_MOTOR3_TO_PHYSICAL(x) (ARM_KINEMATICS_MOTOR3_DIR * (x))
#define ARM_KINEMATICS_MOTOR3_TO_LOGICAL(x)  (ARM_KINEMATICS_MOTOR3_DIR * (x))

#define ARM_KINEMATICS_MOTOR2_PHYSICAL_MIN_RAD APP_ARM_DEG2RAD(-88.0f)
#define ARM_KINEMATICS_MOTOR2_PHYSICAL_MAX_RAD APP_ARM_DEG2RAD(90.0f)
#define ARM_KINEMATICS_MOTOR3_PHYSICAL_MIN_RAD APP_ARM_DEG2RAD(-40.0f)
#define ARM_KINEMATICS_MOTOR3_PHYSICAL_MAX_RAD APP_ARM_DEG2RAD(110.0f)

typedef struct {
    float l2_m;
    float l3_m;
} arm_kinematics_params_t;

typedef struct {
    float theta1_geo_rad;
    float theta2_geo_rad;
    float theta3_geo_rad;
    float theta4_geo_rad;

    float theta1_motor_rad;
    float theta2_motor_rad;
    float theta3_motor_rad;
    float theta4_motor_rad;
} arm_joint_angles_t;

typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float pitch_rad;
} arm_pose_t;

typedef struct {
    float theta1_offset_rad;
    float theta2_offset_rad;
    float theta3_offset_rad;
    float theta4_offset_rad;
    float theta3_theta4_sum_rad;
} arm_offset_config_t;

extern const arm_kinematics_params_t ARM_KINEMATICS_DEFAULT_PARAMS;
extern const arm_offset_config_t ARM_KINEMATICS_DEFAULT_OFFSET;

void arm_kinematics_forward(const arm_kinematics_params_t* params,
                            const arm_joint_angles_t* angles,
                            arm_pose_t* pose);

/*
 * Returns 0 on success, -1 if outside reach, and -2 if all IK branches violate
 * measured J2/J3 physical limits.
 */
int arm_kinematics_inverse(const arm_kinematics_params_t* params,
                           const arm_offset_config_t* offset,
                           const arm_pose_t* target,
                           arm_joint_angles_t* angles);

float arm_kinematics_compute_t4_from_t3(const arm_offset_config_t* offset,
                                        float theta3_motor_rad);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_KINEMATICS_H_ */
