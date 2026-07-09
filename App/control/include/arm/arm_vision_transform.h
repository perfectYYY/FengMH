/*
 * arm_vision_transform.h - Camera-to-arm coordinate transform.
 *
 * Ported from damiao_new1/Core/Src/vision_transform.c. The legacy entry keeps
 * the old mm/m mixed API, while the new entry uses meters throughout.
 */
#ifndef APP_CONTROL_ARM_VISION_TRANSFORM_H_
#define APP_CONTROL_ARM_VISION_TRANSFORM_H_

#include "arm_kinematics.h"
#include "arm_legacy_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_VISION_CAMERA_OFFSET_X_M  (0.105f)
#define ARM_VISION_CAMERA_OFFSET_Y_M  (0.0f)
#define ARM_VISION_CAMERA_OFFSET_Z_M  (-0.078f)

extern volatile arm_pose_t g_arm_vision_camera_data_m;
extern volatile arm_pose_t g_arm_vision_world_data_m;
extern volatile Arm_Pose_t vision_camera_data;
extern volatile Arm_Pose_t vision_world_data;
extern volatile Arm_Pose_t debug_last_camera_data;
extern volatile Arm_Pose_t debug_last_world_data;
extern volatile uint32_t debug_vision_rx_count;

void arm_vision_transform_update(float theta1_rad,
                                 const arm_pose_t* current_end_pose_m);
void arm_vision_transform_compute(const arm_pose_t* camera_pose_m,
                                  float theta1_rad,
                                  const arm_pose_t* current_end_pose_m,
                                  arm_pose_t* out_world_pose_m);

void Perform_Vision_Coordinate_Transform(const Arm_Joint_Angles_t* angles,
                                         const Arm_Pose_t* current_end_pose);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_VISION_TRANSFORM_H_ */
