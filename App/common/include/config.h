/*
 * config.h - App-wide compile-time switches.
 */
#ifndef APP_COMMON_CONFIG_H_
#define APP_COMMON_CONFIG_H_

/* 运行目标区分：板上固件 vs PC 单测 host 构建 */
#ifndef APP_TARGET_HOST
#define APP_TARGET_HOST 0
#endif

#ifndef APP_TARGET_MCU
#define APP_TARGET_MCU (!APP_TARGET_HOST)
#endif

/*
 * 底盘总开关。当前已恢复整机模式：初始化 GO-8010、M3508、
 * BMI088 及相关总线，并创建底盘控制任务。如需再次只调机械臂，
 * 将该宏改为 0 即可。
 */
#ifndef APP_CHASSIS_ENABLE
#define APP_CHASSIS_ENABLE 1
#endif

/* BMI088 attitude-estimation defaults. Body axes can be remapped per board. */
#ifndef APP_IMU_BODY_X_SOURCE_AXIS
#define APP_IMU_BODY_X_SOURCE_AXIS 0U
#endif
#ifndef APP_IMU_BODY_Y_SOURCE_AXIS
#define APP_IMU_BODY_Y_SOURCE_AXIS 1U
#endif
#ifndef APP_IMU_BODY_Z_SOURCE_AXIS
#define APP_IMU_BODY_Z_SOURCE_AXIS 2U
#endif
#ifndef APP_IMU_BODY_X_SIGN
#define APP_IMU_BODY_X_SIGN 1.0f
#endif
#ifndef APP_IMU_BODY_Y_SIGN
#define APP_IMU_BODY_Y_SIGN 1.0f
#endif
#ifndef APP_IMU_BODY_Z_SIGN
#define APP_IMU_BODY_Z_SIGN 1.0f
#endif
#ifndef APP_IMU_GRAVITY_MPS2
#define APP_IMU_GRAVITY_MPS2 9.80665f
#endif
#ifndef APP_IMU_MAHONY_TWO_KP
#define APP_IMU_MAHONY_TWO_KP 1.0f
#endif
#ifndef APP_IMU_MAHONY_TWO_KI
#define APP_IMU_MAHONY_TWO_KI 0.0f
#endif
#ifndef APP_IMU_BOOT_CAL_DURATION_S
#define APP_IMU_BOOT_CAL_DURATION_S 1.0f
#endif
#ifndef APP_IMU_BOOT_CAL_GYRO_MAX_RAD_S
#define APP_IMU_BOOT_CAL_GYRO_MAX_RAD_S 0.05f
#endif
#ifndef APP_IMU_BOOT_CAL_ACCEL_TOL_MPS2
#define APP_IMU_BOOT_CAL_ACCEL_TOL_MPS2 0.8f
#endif
#ifndef APP_IMU_AHRS_ACCEL_TOL_MPS2
#define APP_IMU_AHRS_ACCEL_TOL_MPS2 0.8f
#endif
#ifndef APP_IMU_STATIC_GYRO_MAX_RAD_S
#define APP_IMU_STATIC_GYRO_MAX_RAD_S 0.02f
#endif
#ifndef APP_IMU_STATIC_ACCEL_TOL_MPS2
#define APP_IMU_STATIC_ACCEL_TOL_MPS2 0.5f
#endif
#ifndef APP_IMU_STATIC_HOLD_S
#define APP_IMU_STATIC_HOLD_S 1.0f
#endif
#ifndef APP_IMU_BIAS_TRACK_TAU_S
#define APP_IMU_BIAS_TRACK_TAU_S 60.0f
#endif

/* Relative odometry gates. Contact is inferred solely from gait state. */
#ifndef APP_ODOM_WHEEL_FEEDBACK_TIMEOUT_MS
#define APP_ODOM_WHEEL_FEEDBACK_TIMEOUT_MS 100U
#endif
#ifndef APP_ODOM_ZUPT_SPEED_M_S
#define APP_ODOM_ZUPT_SPEED_M_S 0.02f
#endif
#ifndef APP_ODOM_ZUPT_GYRO_MAX_RAD_S
#define APP_ODOM_ZUPT_GYRO_MAX_RAD_S 0.03f
#endif
#ifndef APP_ODOM_ZUPT_HOLD_S
#define APP_ODOM_ZUPT_HOLD_S 0.50f
#endif

/* 日志后端开关：SEGGER RTT（板上）/ 标准输出（host） */
#ifndef LOG_BACKEND_RTT
#define LOG_BACKEND_RTT 0   /* 暂不强制依赖 SEGGER 源码；当 RTT 源码接入后切 1 */
#endif

#ifndef LOG_BACKEND_STDIO
#define LOG_BACKEND_STDIO (APP_TARGET_HOST)
#endif

/* 日志默认运行期级别 */
#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL 3  /* INFO */
#endif

/* 协议解析栈缓冲最大 payload 长度 */
#ifndef PROTO_MAX_PAYLOAD
#define PROTO_MAX_PAYLOAD 64
#endif

/*
 * 当前上板调试阶段：只输出 RL 单腿闭环。
 * - 无上位机或 vx/wz=0：RL stand + RL_WHEEL 零速锁定
 * - vx/wz 非零：RL trot 轨迹 + RL_WHEEL 速度闭环
 * 正常四腿联调时把它改为 0，或在编译参数里覆盖。
 */
#ifndef APP_DEBUG_RL_SINGLE_LEG_ONLY
#define APP_DEBUG_RL_SINGLE_LEG_ONLY 0
#endif

/* 兼容旧开关名：历史上只测 RL wheel，现在语义升级为 RL 单腿闭环。 */
#ifndef APP_DEBUG_RL_WHEEL_ONLY
#define APP_DEBUG_RL_WHEEL_ONLY APP_DEBUG_RL_SINGLE_LEG_ONLY
#endif

/*
 * 上电无上位机时默认保持 stand；需要自动 stand/march/stand 演示时再打开。
 */
#ifndef APP_OFFLINE_AUTO_MARCH
#define APP_OFFLINE_AUTO_MARCH 0
#endif

/*
 * 上机 walk 步态首轮验证：板上固件默认强制关闭底盘补偿，只保留步态、
 * IK、关节位置命令和支撑相轮子行进。host 单测仍允许打开补偿路径验证算法。
 */
#ifndef APP_CHASSIS_COMP_FORCE_DISABLE
#define APP_CHASSIS_COMP_FORCE_DISABLE (!APP_TARGET_HOST)
#endif

#ifndef APP_CHASSIS_ATTITUDE_COMP_ENABLE
#define APP_CHASSIS_ATTITUDE_COMP_ENABLE (!APP_CHASSIS_COMP_FORCE_DISABLE)
#endif

#ifndef APP_CHASSIS_ARM_LOAD_COMP_ENABLE
#define APP_CHASSIS_ARM_LOAD_COMP_ENABLE (!APP_CHASSIS_COMP_FORCE_DISABLE)
#endif

#ifndef APP_LEG_TAU_FF_COMP_ENABLE
#define APP_LEG_TAU_FF_COMP_ENABLE (!APP_CHASSIS_COMP_FORCE_DISABLE)
#endif

/*
 * 机械臂达妙输出默认策略：
 * - MCU 固件默认打开：上电后机械臂 task 常驻，自动 enable 达妙并在无目标时
 *   维持当前姿态/重力补偿，等待 USB 目标命令。
 * - host 单测默认关闭：多数算法测试不依赖真实输出，需要测试输出路径时显式打开。
 */
#ifndef APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE
#define APP_ARM_MOTOR_OUTPUT_DEFAULT_ENABLE (!APP_TARGET_HOST)
#endif

/*
 * 机械臂纯重补现场调试模式：
 * - 忽略机器人模式、上位机目标和气泵命令；
 * - 不执行上电固定姿态或抓取/放置循环；
 * - 保持机械臂电机输出和纯重力补偿，急停仍然有效。
 */
#ifndef APP_ARM_FORCE_GRAVITY_ONLY
#define APP_ARM_FORCE_GRAVITY_ONLY 0
#endif

/*
 * 上电后是否直接进入上位机作业准备姿态：
 * - 默认关闭：上电进入 PARK；收到 ARM/REAR_PLACE 后再进入第一箱等待位；
 * - NAV/IDLE 等非机械臂模式会挤走 ARM/REAR_PLACE，并让机械臂回 PARK；
 * - 需要现场纯重补时仍可用 debug_arm_force_gravity_only=1 临时切换。
 */
#ifndef APP_ARM_POWER_ON_HOST_READY_ENABLE
#define APP_ARM_POWER_ON_HOST_READY_ENABLE 0
#endif

/*
 * 测试版执行策略：
 * - 上电后底盘仍按原逻辑站起；
 * - 机械臂不再等待 ARM 模式才进等待位，上电后直接进入抓取固定姿态；
 * - J2/J3 固定到 APP_ARM_FIXED_*，J1 只允许在第一箱/第二箱两个等待角之间切换；
 * - 现场 Live Expressions 修改 debug_arm_box_position_select：
 *     1 = 第一箱等待角，2 = 第二箱等待角。
 */
#ifndef APP_ARM_POWER_ON_FIXED_GRASP_TEST
#define APP_ARM_POWER_ON_FIXED_GRASP_TEST 0
#endif

#ifndef APP_ARM_POWER_ON_FIXED_GRASP_DELAY_MS
#define APP_ARM_POWER_ON_FIXED_GRASP_DELAY_MS 3500U
#endif

/*
 * 正式机械臂 ARM 等待策略：
 * - 进入 ARM 后，先把 J2/J3 收到固定等待角，再把 J1 转到第一箱等待角；
 * - 第一箱 PLACE 完成且气泵关闭后，原地保持 APP_ARM_PLACE_HOLD_AFTER_PUMP_OFF_MS；
 * - 然后先抬高 J2/J3 到安全中间角，再把 J1 转到第二箱等待角，
 *   最后把 J2/J3 回到固定等待角。
 * J1 目标角会按当前位置选择等效圈数（target +/- n*360deg）。
 * ARM模式收到新的GRASP目标后，task_arm立即把控制权交给现有抓放流程。
 */
#define APP_ARM_FIRST_WAIT_J1_MOTOR_DEG 270.0f
#define APP_ARM_SECOND_WAIT_J1_MOTOR_DEG 450.0f
#define APP_ARM_PLACE_HOLD_AFTER_PUMP_OFF_MS 3000U
#define APP_ARM_PARK_J1_MOTOR_DEG (-179.639435f)
#define APP_ARM_PARK_J2_MOTOR_DEG 53.679821f
#define APP_ARM_PARK_J3_MOTOR_DEG (-18.8185368f)
#define APP_ARM_FIXED_J2_MOTOR_DEG 42.4236679f
#define APP_ARM_FIXED_J3_MOTOR_DEG (-11.5840006f)

/* 仅限制上位机 GRASP 坐标反解出的 J1 电机角 A=[-57,44]；PLACE 不使用该禁区。 */
#define APP_ARM_GRASP_J1_FORBIDDEN_A_MIN_DEG (-57.0f)
#define APP_ARM_GRASP_J1_FORBIDDEN_A_MAX_DEG (44.0f)

/* 旧工程三段式移动时的收臂/转底座中间点。 */
#ifndef APP_ARM_SAFE_MOVE_J2_DEG
#define APP_ARM_SAFE_MOVE_J2_DEG 3.256634f
#endif

#ifndef APP_ARM_SAFE_MOVE_J3_DEG
#define APP_ARM_SAFE_MOVE_J3_DEG 21.856606f
#endif

/*
 * 旧 damiao_new1 机械臂串口协议中的反馈 FuncID=0x21，与整机协议里的
 * USB_CDC_PING 冲突。整合固件默认由 task_arm 发送新的 0x86 反馈；只有做
 * 单机械臂旧工具调试时才打开旧 0x21 反馈。
 */
#ifndef APP_ARM_LEGACY_SERIAL_FEEDBACK_ENABLE
#define APP_ARM_LEGACY_SERIAL_FEEDBACK_ENABLE 0
#endif

/* 运动中反馈中断阈值：保持原机械臂工程的 250 ms 安全语义。 */
#ifndef APP_ARM_MOTION_FEEDBACK_ABORT_MS
#define APP_ARM_MOTION_FEEDBACK_ABORT_MS 250U
#endif

#ifndef APP_ARM_CONTROL_PERIOD_MS
#define APP_ARM_CONTROL_PERIOD_MS 10U
#endif

#ifndef APP_SAFETY_MOTOR_TIMEOUT_MS
#define APP_SAFETY_MOTOR_TIMEOUT_MS 100U
#endif

#ifndef APP_SAFETY_DAMIAO_TIMEOUT_MS
#define APP_SAFETY_DAMIAO_TIMEOUT_MS APP_ARM_MOTION_FEEDBACK_ABORT_MS
#endif

#endif /* APP_COMMON_CONFIG_H_ */
