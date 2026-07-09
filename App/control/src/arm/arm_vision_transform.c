/*
 * arm_vision_transform.c - Camera-to-arm coordinate transform.
 */
#include "arm_vision_transform.h"

#include <math.h>

volatile arm_pose_t g_arm_vision_camera_data_m = {0};
volatile arm_pose_t g_arm_vision_world_data_m = {0};

volatile Arm_Pose_t vision_camera_data = {0};
volatile Arm_Pose_t vision_world_data = {0};
volatile Arm_Pose_t debug_last_camera_data = {0};
volatile Arm_Pose_t debug_last_world_data = {0};
volatile uint32_t debug_vision_rx_count = 0U;

void arm_vision_transform_compute(const arm_pose_t* camera_pose_m,
                                  float theta1_rad,
                                  const arm_pose_t* current_end_pose_m,
                                  arm_pose_t* out_world_pose_m) {
    if (!camera_pose_m || !current_end_pose_m || !out_world_pose_m) return;

    const float cos_t1 = cosf(theta1_rad);
    const float sin_t1 = sinf(theta1_rad);

    const float x_e = camera_pose_m->y_m + ARM_VISION_CAMERA_OFFSET_X_M;
    const float y_e = camera_pose_m->x_m + ARM_VISION_CAMERA_OFFSET_Y_M;
    const float z_e = -camera_pose_m->z_m + ARM_VISION_CAMERA_OFFSET_Z_M;

    out_world_pose_m->x_m = (x_e * cos_t1) -
                            (y_e * sin_t1) +
                            current_end_pose_m->x_m;
    out_world_pose_m->y_m = (x_e * sin_t1) +
                            (y_e * cos_t1) +
                            current_end_pose_m->y_m;
    out_world_pose_m->z_m = z_e + current_end_pose_m->z_m;
    out_world_pose_m->pitch_rad = 0.0f;
}

void arm_vision_transform_update(float theta1_rad,
                                 const arm_pose_t* current_end_pose_m) {
    arm_pose_t camera_snapshot = {
        .x_m = g_arm_vision_camera_data_m.x_m,
        .y_m = g_arm_vision_camera_data_m.y_m,
        .z_m = g_arm_vision_camera_data_m.z_m,
        .pitch_rad = g_arm_vision_camera_data_m.pitch_rad,
    };
    arm_pose_t world = {0};
    arm_vision_transform_compute(&camera_snapshot,
                                 theta1_rad,
                                 current_end_pose_m,
                                 &world);
    g_arm_vision_world_data_m = world;

    debug_last_camera_data.x = camera_snapshot.x_m;
    debug_last_camera_data.y = camera_snapshot.y_m;
    debug_last_camera_data.z = camera_snapshot.z_m;
    debug_last_camera_data.pitch = camera_snapshot.pitch_rad;
    debug_last_world_data.x = world.x_m * 1000.0f;
    debug_last_world_data.y = world.y_m * 1000.0f;
    debug_last_world_data.z = world.z_m * 1000.0f;
    debug_last_world_data.pitch = 0.0f;
    debug_vision_rx_count++;
}

void Perform_Vision_Coordinate_Transform(const Arm_Joint_Angles_t* angles,
                                         const Arm_Pose_t* current_end_pose) {
    if (!angles || !current_end_pose) return;

    const float cam_x_mm = vision_camera_data.x * 1000.0f;
    const float cam_y_mm = vision_camera_data.y * 1000.0f;
    const float cam_z_mm = vision_camera_data.z * 1000.0f;
    debug_last_camera_data.x = vision_camera_data.x;
    debug_last_camera_data.y = vision_camera_data.y;
    debug_last_camera_data.z = vision_camera_data.z;
    debug_last_camera_data.pitch = vision_camera_data.pitch;

    const float cos_t1 = cosf(angles->theta1_geo);
    const float sin_t1 = sinf(angles->theta1_geo);

    const float x_e = cam_y_mm + ARM_VISION_CAMERA_OFFSET_X_M * 1000.0f;
    const float y_e = cam_x_mm + ARM_VISION_CAMERA_OFFSET_Y_M * 1000.0f;
    const float z_e = -cam_z_mm + ARM_VISION_CAMERA_OFFSET_Z_M * 1000.0f;

    vision_world_data.x = (x_e * cos_t1) -
                          (y_e * sin_t1) +
                          current_end_pose->x;
    vision_world_data.y = (x_e * sin_t1) +
                          (y_e * cos_t1) +
                          current_end_pose->y;
    vision_world_data.z = z_e + current_end_pose->z;
    vision_world_data.pitch = 0.0f;

    debug_last_world_data.x = vision_world_data.x;
    debug_last_world_data.y = vision_world_data.y;
    debug_last_world_data.z = vision_world_data.z;
    debug_last_world_data.pitch = vision_world_data.pitch;
    debug_vision_rx_count++;
}
