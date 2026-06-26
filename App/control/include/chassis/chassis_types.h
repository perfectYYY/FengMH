/*
 * chassis_types.h - Shared chassis control enums.
 *
 * Keep these types in control/ so app tasks, protocol handlers, and the
 * control pipeline all speak the same vocabulary without depending on each
 * other's implementation files.
 */
#ifndef APP_CONTROL_CHASSIS_TYPES_H_
#define APP_CONTROL_CHASSIS_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_MODE_AUTO = 0,
    CHASSIS_MODE_ONLINE,
    CHASSIS_MODE_STANDALONE,
} chassis_mode_t;

typedef enum {
    CHASSIS_GAIT_STAND = 0,
    CHASSIS_GAIT_TROT = 1,
    CHASSIS_GAIT_WALK = 2,
    CHASSIS_GAIT_SCRIPT = 3,
} chassis_gait_active_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CHASSIS_TYPES_H_ */
