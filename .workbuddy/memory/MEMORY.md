# Long-term Memory

## Project: FengMH (四足机器人)

### Repository structure
- Main workspace: `/Users/code/FengMH` (currently on `codex/firmware-dev`)
- `codex/pc-leg-tools` branch worktree: `/Users/code/FengMH-pc-leg-tools`
- MCU firmware: STM32H723VGTX, App/Core/Drivers/Middlewares
- Host PC tools: `tools/leg_viz/` (Python + C shared library)

### Digital Twin architecture (codex/pc-leg-tools)
- Mode enum: `HOST_SIM_MODE_SIM=0`, `HOST_SIM_MODE_TWIN=1`
- SIM mode: virtual motors respond to control stack commands immediately
- TWIN mode: virtual motors read from serial telemetry via `host_sim_inject_motor_angle()`
- Serial telemetry: USB CDC, proto_frame format, function ID 0x81
- Thread safety: pthread_mutex_t around all motor state reads/writes
- Build: `cmake -S App/test -B build_host && cmake --build build_host --target fengmh_sim`
- Shared library: `build_host/libfengmh_sim.dylib` (macOS) or `.so` (Linux)

### Build conventions
- Host builds use `APP_TARGET_HOST=1` define
- Non-MSVC targets link `m` and `pthread`
- macOS: `B460800`/`B921600` baud rates not available, use `#ifdef` guards
- Python tools access C code via ctypes from `sim_bridge.py`

### Key files
- `tools/leg_viz/host_sim.c/h` — C shared lib core (virtual motors + twin mode)
- `tools/leg_viz/twin_serial.c/h` — POSIX serial reader (pthread, proto_frame)
- `tools/leg_viz/sim_bridge.py` — ctypes wrapper
- `tools/leg_viz/leg_viz.py` — 3D matplotlib GUI
- `App/test/CMakeLists.txt` — Host build config

### UART/RS485 电机通信架构
- 8个GO-8010电机，分2条RS485总线 (USART2/3)，每条4个电机
- HAL UART 回调桥接在 `Core/Src/main.c:424-434`（已确认完好）
- `bsp_uart.c` 中 `bsp_uart_hal_rx_event()` / `bsp_uart_hal_tx_done()` 已实现且在 `bsp_uart.h` 中声明 (guard: `#if !APP_TARGET_HOST`)
- **已知问题**: `motor_go_send_all()` 每条总线连续发4帧，第2帧起因 `s_tx_busy` 被丢弃 → 每条总线仅第1个电机收到指令

### SPI2 / BMI088 IMU 集成
- SPI2: Mode0 (CPOL=0 CPHA=0), prescaler=16 (~10MHz @ PLL), soft NSS — 对齐 Wheel-legged 参考
- **SPI2_SCK: PB13** (2026-05-02 从 PB10 修正, CubeMX IOC 改动)
- CS pins: PC0 (ACC), PC3 (GYRO) — Active Low via bsp_spi_set_cs()
- INT pins: PE10 (ACC_INT), PE12 (GYRO_INT) — GPIO_MODE_INPUT (polling, no EXTI)
- **Pin Speed: LOW** (对齐 Wheel-legged, 删除了 VERY_HIGH 覆盖)
- BMI088 driver: `App/device/imu_bmi088.h/.c` — soft reset, CHIP_ID check (ACC=0x1E, GYRO=0x0F), register write-verify
- **ACC init 顺序 (2026-05-02 修正)**: PWR_CTRL(0x04) 必须在 CONFIG 之前写入, 否则 ACC 不响应配置
- ACC config table: PWR_CTRL → PWR_CONF → ACC_CONF(0xAB, ODR 800Hz) → RANGE(±3G) → INT1_IO_CTRL → INT_MAP_DATA
- GYRO config table: RANGE(±2000dps) → BANDWIDTH(0x82, 1000Hz/116Hz) → LPM1 → GYRO_CTRL(DRDY_ON) → INT3_IO_CONF → INT3_MAP
- **Multi-byte read fix (2026-05-02)**: acc_read_multi/gyro_read_multi 不再用单字节 tx 作为多字节传输的 TX buffer (stack buffer overrun); 改用局部 tx[9] 数组
- **Pre-check**: 复位前先读一次 CHIP_ID 确认 SPI 通信 (对齐 Wheel-legged)
- Attitude estimator: `App/service/attitude/attitude_estimator.h/.c` — gyro Z-axis integration with dead-zone drift suppression (0.01 rad/s threshold)
- Steer controller: `App/service/attitude/steer_controller.h/.c` — yaw PID (kp=2.0, ki=0.1, kd=0.05, wz_max=3.0 rad/s)
- Protocol: chassis_cmd extended with target_yaw + steer_mode (backward-compatible: 12B old, 17B new)
- app_init.c: bsp_spi_init(BSP_SPI_2) + imu_bmi088_init() with graceful degradation
- TODO: wz correction not yet wired into gait/leg_controller (gait layer lacks turning support)

### Flash 命令
- 调试探针: ICWorkshop PowerDebugger Wireless TX (CMSIS-DAP v2, VID=0x303A PID=0x40FF)
- 构建: CLion cmake-build-debug
```bash
# 构建
/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --build /Users/code/FengMH/cmake-build-debug --target FengMH.elf -- -j 8

# 烧录
openocd -s /opt/homebrew/share/openocd/scripts \
    -f interface/cmsis-dap.cfg -c 'transport select swd' \
    -c 'adapter speed 500' -f target/stm32h7x.cfg \
    -c 'init' -c 'reset halt' \
    -c 'program cmake-build-debug/FengMH.elf verify' -c 'reset' -c 'exit'
```
