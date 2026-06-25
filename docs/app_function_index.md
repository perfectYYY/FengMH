# App Function Index

This index documents the App-layer files and functions as of branch
`remake_original`. It is intentionally concise: use it as a lookup table before
changing code.

## `App/app`

### `app_init.c`
- `app_init()`: Initializes log, time, FDCAN, UART, USB, SPI, motor registry,
  GO motors, M3508 motors, and BMI088. This is the board bring-up entry.

### `app_tasks.c`
- `app_tasks_create()`: Initializes communication and chassis control, then
  starts FreeRTOS tasks on MCU builds.

### `task_chassis.c`
- `read_chassis_input()`: Copies cached `task_comm` command state into
  `chassis_control_input_t`.
- `task_chassis_init()`: Initializes the chassis behavior module.
- `task_chassis_set_mode()`: Forwards chassis mode changes to `chassis_control`.
- `task_chassis_get_mode()`: Reads current chassis mode.
- `task_chassis_get_gait_active()`: Reads current active gait.
- `task_chassis_set_online_timeout_ms()`: Configures heartbeat timeout.
- `task_chassis_get_online_timeout_ms()`: Reads heartbeat timeout.
- `task_chassis_play_script()`: Requests script gait playback.
- `task_chassis_stop_script()`: Stops script gait and returns to stand.
- `task_chassis_start_stand()`: Manually requests stand gait.
- `task_chassis_set_trot_params()`: Updates default trot parameters.
- `task_chassis_get_trot_params()`: Reads current trot parameters.
- `task_chassis_start_trot()`: Manually requests trot gait.
- `task_chassis_active_gait_name()`: Returns active gait implementation name.
- `task_chassis_reset_yaw()`: Resets yaw estimator state.
- `task_chassis_get_yaw()`: Reads yaw estimate.
- `task_chassis_get_effective_wz()`: Reads last yaw-rate command after steering.
- `task_chassis_step_for_test()`: Runs one chassis control tick.
- `task_chassis_entry()`: 500 Hz MCU task wrapper with estop guard.

### `task_comm.c`
- `handle_chassis()`: Decodes `0x10` chassis speed command payload.
- `gait_params_from_payload()`: Converts protocol gait payload to `gait_params_t`.
- `handle_gait()`: Decodes `0x12` stand/trot/set-params command.
- `handle_mit_cmd()`: Decodes `0x13` single-motor MIT debug command.
- `on_usb_rx()`: Feeds USB bytes into the protocol frame parser.
- `task_comm_init()`: Resets command state and installs protocol dispatch table.
- `task_comm_good_cnt()`: Returns valid frame count.
- `task_comm_bad_cnt()`: Returns invalid frame count.
- `task_comm_dispatch_hit()`: Returns matched handler count.
- `task_comm_dispatch_miss()`: Returns unmatched handler count.
- `task_comm_last_rx_ms()`: Returns latest valid command timestamp.
- `task_comm_get_chassis()`: Copies latest chassis command.
- `send_state_frame()`: Builds and sends `0x80` robot state telemetry.
- `send_motor_frame()`: Builds and sends one round-robin `0x81` motor telemetry frame.
- `task_comm_entry()`: MCU telemetry task loop.

### `task_log.c`
- `task_log_entry()`: MCU log heartbeat task placeholder.

### `task_safety.c`
- `task_safety_estop_active()`: Returns estop state.
- `task_safety_event_count()`: Returns safety event count.
- `task_safety_estop_set()`: Changes estop state and disables motors when entering estop.
- `task_safety_entry()`: MCU safety monitor loop for timeout and temperature checks.

## `App/bsp`

### `bsp_fdcan.c`
- `fdcan_dwt_enable()`: Enables DWT cycle counter on MCU.
- `fdcan_cycles_per_us()`: Converts MCU clock to cycles/us.
- `fdcan_wait_free_level()`: Waits for FDCAN TX FIFO space.
- `fdcan_warn_throttled()`: Rate-limits FDCAN warning logs.
- `bsp_fdcan_init()`: Initializes host mock queues or MCU FDCAN filters/interrupts.
- `bsp_fdcan_attach_rx()`: Registers a bus RX callback.
- `bsp_fdcan_send()`: Sends one CAN frame or pushes it to host TX queue.
- `bsp_fdcan_hal_rxfifo0_cb()`: Bridges HAL RX FIFO callback to BSP callback.
- `bsp_fdcan_hal_error_cb()`: Records FDCAN error interrupt state.
- `bsp_fdcan_test_tx_count()`: Host helper returning queued TX frames.
- `bsp_fdcan_test_pop_tx()`: Host helper popping one queued TX frame.
- `bsp_fdcan_test_inject_rx()`: Host helper injecting an RX frame.
- `bsp_fdcan_test_reset()`: Host helper clearing mock queues.

### `bsp_spi.c`
- `bsp_spi_init()`: Initializes SPI bus or host mock.
- `bsp_spi_set_cs()`: Controls chip-select line or host mock CS state.
- `bsp_spi_transfer_byte()`: Transfers one SPI byte.
- `bsp_spi_test_attach_xfer()`: Host helper installing transfer callback.
- `bsp_spi_test_reset()`: Host helper clearing SPI mock state.

### `bsp_time.c`
- `now_ns_raw()`: Host helper returning monotonic nanoseconds.
- `bsp_time_init()`: Initializes host base time or relies on HAL SysTick.
- `bsp_time_now_ms()`: Returns current time in milliseconds.
- `bsp_time_now_us()`: Returns current time in microseconds.
- `bsp_time_delay_ms()`: Delays milliseconds.
- `bsp_time_delay_us()`: Delays microseconds.
- `bsp_time_test_advance_ms()`: Host helper advancing fake time.

### `bsp_uart.c`
- `uart_dwt_enable()`: Enables DWT timing for MCU UART waits.
- `uart_cycles_per_us()`: Converts MCU clock to cycles/us.
- `bsp_uart_init()`: Initializes UART/RS485 bus or host TX queue.
- `bsp_uart_attach_rx()`: Registers a bus RX callback.
- `bsp_uart_send()`: Sends bytes on UART or pushes host TX queue.
- `bsp_uart_wait_tx_done()`: Waits for RS485 TX window to finish.
- `bsp_uart_on_rx()`: Bridges lower-level RX into registered callback.
- `bsp_uart_on_tx_done()`: Marks MCU DMA TX complete.
- `bsp_uart_test_tx_count()`: Host helper returning queued TX packets.
- `bsp_uart_test_pop_tx()`: Host helper popping one queued TX packet.
- `bsp_uart_test_inject_rx()`: Host helper injecting UART RX bytes.
- `bsp_uart_test_reset()`: Host helper clearing UART mock queues.
- `bsp_uart_hal_rx_event()`: Maps HAL UART RX event to BSP bus callback.
- `bsp_uart_hal_tx_done()`: Maps HAL UART TX complete callback to BSP bus state.

### `bsp_usb_cdc.c`
- `bsp_usb_cdc_init()`: Initializes USB CDC bridge or host TX buffer.
- `bsp_usb_cdc_attach_rx()`: Registers USB RX callback.
- `bsp_usb_cdc_send()`: Sends USB bytes or stores them in host TX buffer.
- `bsp_usb_cdc_on_rx()`: Feeds received USB bytes to registered callback.
- `bsp_usb_cdc_test_inject_rx()`: Host helper injecting USB RX bytes.
- `bsp_usb_cdc_test_tx_size()`: Host helper reading TX buffer length.
- `bsp_usb_cdc_test_read_tx()`: Host helper copying TX buffer contents.
- `bsp_usb_cdc_test_reset()`: Host helper clearing USB mock TX buffer.

## `App/common`

### `log.c`
- `stdio_backend()`: Optional host stdout backend.
- `lookup_tag_level()`: Finds tag-specific log level or global fallback.
- `log_init()`: Resets log backend and level state.
- `log_register_backend()`: Adds a log backend callback.
- `log_set_global_level()`: Sets global log threshold.
- `log_get_global_level()`: Reads global log threshold.
- `log_set_level()`: Sets per-tag log threshold.
- `log_emit()`: Formats and dispatches one log line.
- `log_last_line()`: Returns latest formatted log line.

## `App/control/attitude`

### `attitude_estimator.c`
- `attitude_estimator_init()`: Resets yaw and estimator state.
- `attitude_estimator_update()`: Integrates gyro Z into yaw estimate.
- `attitude_estimator_get_yaw()`: Returns current yaw.
- `attitude_estimator_reset_yaw()`: Resets yaw to zero.
- `attitude_estimator_get_state()`: Returns pointer to estimator state.

### `steer_controller.c`
- `wrap_pi()`: Wraps angle error to `[-pi, pi]`.
- `steer_controller_init()`: Resets steering PID state.
- `steer_controller_set_mode()`: Changes steering mode.
- `steer_controller_set_target_yaw()`: Sets yaw target.
- `steer_controller_update()`: Computes yaw-rate command from yaw error.
- `steer_controller_get_cfg()`: Returns steering gains/config.

## `App/control/chassis`

### `chassis_control.c`
- `controller_height_from_body()`: Converts body height to IK height convention.
- `apply_controller_height()`: Pushes gait body height into leg controller.
- `validate_trot_params()`: Checks gait parameters before use.
- `request_gait()`: Wraps gait-machine set/request behavior.
- `is_offline()`: Resolves current online/offline state.
- `online_decide()`: Selects stand or trot while online.
- `apply_plan_wheel_speed()`: Applies planner wheel speeds to stance legs only.
- `apply_rl_single_leg_debug()`: Optional single-leg debug output path.
- `update_steering()`: Updates yaw estimator and computes effective `wz`.
- `offline_decide()`: Selects conservative offline behavior.
- `go_pre_calibration_ready()`: MCU GO feedback collection window before closed loop.
- `flush_motor_outputs()`: Sends staged motor commands on MCU builds.
- `safe_input_or_zero()`: Copies input or creates a zero command snapshot.
- `make_plan_command()`: Builds planner command from input and effective yaw rate.
- `update_plan_from_input()`: Runs chassis planner and stores the latest plan.
- `update_online_state()`: Updates the cached online/offline status.
- `decide_gait_for_link_state()`: Applies online or offline gait policy.
- `apply_gait_output_to_motors()`: Updates gait output and dispatches leg commands.
- `chassis_control_init()`: Initializes gait, leg, planner, and steering state.
- `chassis_control_tick()`: Runs one full command-to-motor control cycle.
- `chassis_control_set_mode()`: Sets chassis mode.
- `chassis_control_get_mode()`: Reads chassis mode.
- `chassis_control_get_gait_active()`: Reads active gait enum.
- `chassis_control_set_online_timeout_ms()`: Sets heartbeat timeout.
- `chassis_control_get_online_timeout_ms()`: Reads heartbeat timeout.
- `chassis_control_play_script()`: Starts script gait playback.
- `chassis_control_stop_script()`: Stops script gait.
- `chassis_control_start_stand()`: Starts stand gait.
- `chassis_control_set_trot_params()`: Updates trot parameters.
- `chassis_control_get_trot_params()`: Reads trot parameters.
- `chassis_control_start_trot()`: Starts trot gait.
- `chassis_control_active_gait_name()`: Returns current gait name.
- `chassis_control_reset_yaw()`: Resets yaw estimator.
- `chassis_control_get_yaw()`: Reads yaw estimator.
- `chassis_control_get_effective_wz()`: Reads last effective yaw rate.
- `chassis_control_get_status()`: Copies diagnostic status snapshot.

### `chassis_planner.c`
- `clampf_local()`: Clamps a float.
- `signf_local()`: Returns float sign.
- `chassis_planner_init()`: Reserved planner init hook.
- `sanitized_gain()`: Clamps turn gain or uses fallback.
- `planner_limit_wheel()`: Applies configured wheel speed limit.
- `planner_is_low_speed_turn()`: Detects low-speed yaw turn condition.
- `planner_apply_turn_gait()`: Overrides gait params for turn behavior.
- `chassis_planner_update()`: Converts command to gait params and wheel speeds.

## `App/control/gait`

### `gait_if.c`
- `gait_wrap01()`: Wraps phase into `[0, 1)`.

### `gait_machine.c`
- `lerp()`: Linear interpolation helper.
- `blend_outputs()`: Blends two gait outputs.
- `gait_machine_init()`: Clears gait machine state.
- `gait_machine_set()`: Immediately sets current gait.
- `gait_machine_request()`: Requests target gait with optional blend.
- `gait_machine_update()`: Updates current gait or blend transition.
- `gait_machine_state()`: Reads gait machine state.

### `gait_params.c`
- `GAIT_PARAMS_TROT_DEFAULT`: Default trot gait parameters.
- `GAIT_PARAMS_STAND_DEFAULT`: Default stand gait parameters.

### `gait_stand.c`
- `stand_init()`: Stand gait init hook.
- `stand_set_param()`: Stores stand gait parameters.
- `stand_update()`: Outputs stance with zero foot displacement and zero wheel speed.
- `stand_exit()`: Stand gait exit hook.
- `stand_name()`: Returns `"stand"`.
- `gait_stand_create()`: Returns singleton stand gait instance.

### `gait_trot.c`
- `trot_init()`: Resets trot phase.
- `trot_set_param()`: Stores validated trot parameters.
- `gait_trot_foot_traj()`: Computes stance/swing foot trajectory.
- `trot_update()`: Advances trot phase and writes foot targets.
- `trot_exit()`: Trot gait exit hook.
- `trot_name()`: Returns `"trot"`.
- `gait_trot_create()`: Returns singleton trot gait instance.

## `App/control/kinematics`

### `leg_ik.c`
- `leg_ik_clampf()`: Clamps float value.
- `leg_ik_get_joint_limit()`: Gets joint limits for mirror/original leg type.
- `leg_ik_update_best()`: Chooses closest valid joint candidate.
- `leg_ik_constrain_joint_target()`: Enforces joint and thigh-shin relationship limits.
- `leg_ik_solve()`: Solves one leg inverse kinematics.
- `leg_fk_solve()`: Solves one leg forward kinematics.
- `leg_ik_solve_all()`: Converts four foot targets to hip/knee joint targets.

### `leg_params.c`
- `LEG_DIM_DEFAULT`: Default leg geometry and mass parameters.

## `App/control/leg`

### `leg_controller.c`
- `leg_controller_init()`: Clears controller state.
- `leg_controller_bind_from_registry()`: Binds hip/knee/wheel handles by logical ID.
- `leg_controller_set_stand_height()`: Sets IK standing height.
- `leg_controller_set_output_options()`: Configures enabled legs and joint gains.
- `try_set_pos()`: Issues a motor position command when supported.
- `try_set_vel()`: Issues a motor velocity command when supported.
- `try_set_wheel_mit()`: Issues M3508 wheel MIT command and limits.
- `wheel_mit_reset_refs()`: Clears wheel MIT integrated position references.
- `wheel_mit_read_cfg()`: Resolves debug MIT gains and safety limits.
- `wheel_mit_limit_dt()`: Clamps wheel MIT integration timestep.
- `wheel_mit_latch_ref_if_needed()`: Locks a wheel reference angle before MIT use.
- `wheel_mit_command()`: Sends one wheel MIT position/velocity command.
- `wheel_mit_apply_stance()`: Integrates and commands stance-phase wheel MIT.
- `wheel_mit_apply_swing()`: Holds or zeroes swing-phase wheel behavior.
- `try_set_wheel()`: Selects wheel velocity path or debug MIT path.
- `leg_output_enabled()`: Checks the configured leg output mask.
- `leg_has_required_actuators()`: Verifies enabled joint/wheel handles exist.
- `send_joint_targets()`: Sends hip/knee position targets.
- `send_wheel_target()`: Sends one wheel velocity or MIT target.
- `send_leg_targets()`: Dispatches all enabled actuators for one leg.
- `leg_controller_apply_dt()`: Runs IK and sends joint/wheel commands.
- `leg_controller_apply()`: Uses default 2 ms timestep.

## `App/control/script`

### `gait_script.c`
- `gs_init()`: Initializes/restarts script player.
- `gs_set_param()`: Stores gait params for script wrapper.
- `gs_update()`: Updates script player or safe stand-hold output.
- `gs_exit()`: Script gait exit hook.
- `gs_name()`: Returns `"script"`.
- `gait_script_create()`: Returns singleton script gait instance.
- `gait_script_set_script()`: Loads a script into player.
- `gait_script_rewind()`: Rewinds current script.
- `gait_script_state()`: Reads player state.

### `script_builtin.c`
- `script_builtin_find()`: Finds built-in script by name.
- `SCRIPT_BUILTIN_STAND_HOLD`: Static stand-hold script.
- `SCRIPT_BUILTIN_WAVE_UP_DOWN`: Static synchronized lift script.
- `SCRIPT_BUILTIN_TROT_STEP`: Static four-keyframe trot script.

### `script_player.c`
- `lerp()`: Linear interpolation helper.
- `normalize_legacy_foot_fields()`: Converts old script hip/knee foot data to foot fields.
- `blend_kf()`: Interpolates two script keyframes.
- `script_sample()`: Samples script output at arbitrary time.
- `script_player_init()`: Clears player state.
- `script_player_load()`: Loads a script and starts running.
- `script_player_reset()`: Rewinds loaded script.
- `script_player_update()`: Advances time and samples current script.
- `script_player_state()`: Reads player state.

## `App/device`

### `imu_bmi088.c`
- `make_i16()`: Combines LSB/MSB into signed 16-bit value.
- `diag_reset()`: Clears IMU diagnostic state.
- `diag_mark()`: Stores latest diagnostic stage and status.
- `acc_write_reg()`: Writes accelerometer register.
- `acc_read_reg()`: Reads accelerometer register.
- `acc_read_multi()`: Reads multiple accelerometer registers.
- `gyro_write_reg()`: Writes gyroscope register.
- `gyro_read_reg()`: Reads gyroscope register.
- `gyro_read_multi()`: Reads multiple gyroscope registers.
- `accel_init()`: Initializes BMI088 accelerometer.
- `gyro_init()`: Initializes BMI088 gyroscope.
- `read_temperature()`: Reads and converts accelerometer temperature.
- `imu_bmi088_init()`: Initializes BMI088 or host mock.
- `imu_bmi088_read()`: Reads accel/gyro/temp sample.
- `imu_bmi088_set_accel_range()`: Changes accelerometer range.
- `imu_bmi088_is_ready()`: Returns IMU ready state.
- `imu_bmi088_get_diag()`: Copies diagnostic state.
- `imu_bmi088_set_gyro_range()`: Changes gyroscope range.

### `motor_go.c`
- `go_crc16()`: Computes RIS frame CRC.
- `go_cfg()`: Looks up motor config for device.
- `go_cfg_sign()`: Returns configured joint direction.
- `go_cfg_gear()`: Returns configured gear ratio.
- `go_motor_pos_to_joint()`: Converts motor-side position to joint-side.
- `go_joint_pos_to_motor()`: Converts joint-side position to motor-side.
- `go_motor_vel_to_joint()`: Converts motor-side velocity to joint-side.
- `go_joint_vel_to_motor()`: Converts joint-side velocity to motor-side.
- `go_motor_tau_to_joint()`: Converts motor-side torque to joint-side.
- `go_joint_tau_to_motor()`: Converts joint-side torque to motor-side.
- `go_encode_cmd()`: Encodes RIS command frame.
- `go_decode_fbk()`: Decodes RIS feedback frame.
- `go_set_current()`: Maps current request to torque-like command.
- `go_set_torque()`: Sets GO torque command.
- `go_set_position()`: Sets GO position/velocity/kp/kd/tau command.
- `go_set_velocity()`: Sets GO velocity damping command.
- `go_calibrate()`: Latches zero offset from latest feedback.
- `go_enable()`: Enables closed-loop mode.
- `go_disable()`: Commands zero-torque mode.
- `go_reset_fault()`: Clears error count and zero-torque mode.
- `go_feed_rx()`: Feeds decoded RX payload to state.
- `go_find_index_by_logical_id()`: Finds GO instance by logical motor ID.
- `motor_go_init_all()`: Creates GO instances and binds registry.
- `motor_go_uart_rx_cb()`: Routes UART feedback to matching GO instance.
- `motor_go_send_all()`: Sends all GO command frames.
- `motor_go_calibrate_all()`: Calibrates all GO motors with feedback.
- `motor_go_debug_get_state()`: Copies debug state for one GO motor.

### `motor_m3508.c`
- `m3508_limit_current()`: Clamps current raw command.
- `m3508_power_limit()`: Limits current by power estimate.
- `m3508_rpm_to_rads()`: Converts rpm to rad/s.
- `m3508_encoder_to_rad()`: Converts encoder count to radians.
- `m3508_raw_to_ampere()`: Converts raw current to amperes.
- `m3508_ampere_to_raw()`: Converts amperes to raw current.
- `m3508_ampere_to_raw_f()`: Float version of ampere-to-raw conversion.
- `m3508_cfg()`: Looks up motor config.
- `m3508_cfg_sign()`: Returns configured joint direction.
- `m3508_clampf()`: Clamps float value.
- `m3508_output_torque_to_raw()`: Converts output torque to raw motor current.
- `m3508_update_angle()`: Maintains multiturn encoder state.
- `m3508_set_current()`: Sets direct current mode.
- `m3508_set_torque()`: Sets torque mode.
- `m3508_set_position()`: Sets position mode or MIT mode.
- `m3508_set_velocity()`: Sets velocity mode.
- `m3508_enable()`: Marks motor enabled and resets loops.
- `m3508_disable()`: Zeros command and disables motor.
- `m3508_reset_fault()`: Clears error count and enables motor.
- `m3508_feed_rx()`: Decodes C620 feedback frame.
- `m3508_mit_limit_tau()`: Clamps MIT torque limit.
- `m3508_mit_limit_pos_err()`: Clamps MIT position error limit.
- `motor_m3508_trace_reset()`: Resets trace ring buffer.
- `motor_m3508_trace_enable()`: Enables/disables trace capture.
- `motor_m3508_set_mit_limits()`: Applies per-command MIT safety limits.
- `m3508_trace_float_to_i16()`: Converts trace float to compact integer.
- `m3508_trace_record()`: Records one trace sample.
- `m3508_speed_pid_set_tunings()`: Updates speed PID gains.
- `m3508_speed_pid_decay_integral()`: Decays PID integral state.
- `m3508_speed_pid_update()`: Updates speed PID with anti-windup behavior.
- `m3508_slew_current()`: Limits current command slew rate.
- `m3508_velocity_hold_target_rpm()`: Converts zero-speed hold error to rpm target.
- `m3508_apply_static_ff()`: Applies optional static feed-forward.
- `m3508_stop_ctx()`: Stops one context with zero current.
- `m3508_hold_ctx_mit()`: Holds current angle using MIT mode.
- `m3508_mit_ramp_update()`: Debug MIT ramp state machine.
- `motor_m3508_init_all()`: Creates M3508 instances and binds registry.
- `motor_m3508_fdcan_rx_cb()`: Routes CAN feedback to matching M3508 instance.
- `m3508_apply_temperature_derate()`: Applies the M3508 thermal half-current derate.
- `m3508_run_position_control()`: Runs cascaded position-to-speed-to-current control.
- `m3508_run_torque_control()`: Runs output-torque feed-forward plus current correction.
- `m3508_run_mit_control()`: Runs wheel MIT impedance control and safety limits.
- `m3508_run_current_control()`: Applies direct current mode thermal derate.
- `m3508_velocity_target_to_rpm()`: Converts output shaft velocity target to rotor rpm.
- `m3508_pid_needs_tuning_update()`: Detects velocity PID gain changes.
- `m3508_prepare_velocity_target_rpm()`: Selects move or zero-speed hold velocity target.
- `m3508_run_velocity_control()`: Runs velocity PI, feed-forward, slew, and power limits.
- `m3508_run_control_mode()`: Dispatches one motor context to the active control mode.
- `m3508_pack_current_slot()`: Packs one raw current command into the DJI CAN slot.
- `m3508_log_tx_failure()`: Rate-limited logging for 0x200 frame send failure.
- `m3508_log_probe_tx_failure()`: Rate-limited logging for 0x1FF probe frame failure.
- `m3508_send_current_frame()`: Sends the bus 0x200 current frame.
- `m3508_send_probe_frame()`: Sends the zero-current 0x1FF probe frame.
- `m3508_run_bus_controls()`: Runs all motors assigned to one FDCAN bus.
- `motor_m3508_send_all()`: Runs control loops and sends CAN current frames.

### `motor_registry.c`
- `motor_registry_init()`: Clears logical motor registry.
- `motor_get()`: Returns motor handle by logical ID.
- `motor_get_cfg()`: Returns immutable motor config by logical ID.
- `motor_registry_count()`: Returns registry slot count.
- `motor_registry_bind()`: Binds logical motor ID to a runtime device.

## `App/service`

### `pid.c`
- `clamp_f()`: Clamps float value.
- `sort_limits()`: Orders high/low limit pointers.
- `normalize_dt()`: Provides safe timestep fallback.
- `dt_from_ms()`: Computes timestep from millisecond timestamp.
- `app_pid_reset()`: Clears PID dynamic state.
- `app_pid_set_tunings()`: Sets PID gains.
- `app_pid_set_output_limits()`: Sets output clamp.
- `app_pid_set_integral_limits()`: Sets integral clamp.
- `app_pid_update_dt()`: Positional PID update with explicit timestep.
- `app_pid_update_ms()`: Positional PID update from timestamp.
- `app_pid_update_inc_dt()`: Incremental PID update with explicit timestep.
- `app_pid_update_inc_ms()`: Incremental PID update from timestamp.

### `proto_dispatch.c`
- `proto_dispatcher_init()`: Installs function-ID dispatch table.
- `proto_dispatch_on_frame()`: Calls matching protocol handler or records miss.

### `proto_frame.c`
- `proto_frame_init()`: Initializes parser state and callback.
- `proto_frame_reset()`: Clears parser state.
- `fail_and_reset()`: Counts bad frame and resets parser.
- `proto_frame_feed()`: Incrementally parses framed byte stream.
- `proto_frame_build()`: Builds one framed packet with checksum.
