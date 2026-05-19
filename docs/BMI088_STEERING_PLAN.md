# BMI088 转向系统 -- 开发计划

## 目标

上位机发送 `vx`（前进速度）和 `target_yaw`（目标偏航角），机器人利用 BMI088 IMU 陀螺仪闭环控制偏航角，实现自动转向运动规划。

---

## 1. 现状分析

| 项目 | CPU | IMU | SPI | 构建系统 |
|------|-----|-----|-----|----------|
| CtrBoard-H7_IMU (参考) | STM32H723VGTx | BMI088 | SPI2, Mode3, 15MHz | Keil MDK |
| FengMH (本项目) | STM32H723VGTx | **无** | **未启用** | CMake + arm-none-eabi-gcc |

### FengMH 现有架构

```
App/
├── common/     config.h, err.h, types.h, log.h/.c
├── bsp/        bsp_uart, bsp_fdcan, bsp_usb_cdc, bsp_time
├── device/     motor_if, motor_go, motor_m3508, motor_registry
├── service/
│   ├── protocol/    USB CDC 帧协议 (proto_frame + proto_dispatch)
│   ├── gait/        步态接口 + 状态机 (gait_machine + gait_trot/stand)
│   ├── kinematics/  腿运动学 (leg_ik)
│   ├── leg/         腿控制器 (leg_controller)
│   └── script/      脚本步态
└── app/
    ├── task_comm      USB 通信任务
    ├── task_chassis   500Hz 底盘控制任务
    ├── task_safety    安全监控
    └── task_log       日志输出
```

### 关键代码风格

- 文件/函数: `snake_case` (`bsp_uart_init`, `motor_go_send_all`)
- 类型: `snake_case_t` (`motor_dev_t`, `gait_output_t`)
- 枚举: `UPPER_SNAKE_CASE` (`MOTOR_GO`, `APP_OK`)
- 静态全局: `s_` 前缀 (`s_parser`, `s_gm`)
- 错误码: `app_err_t` 统一枚举
- 每模块 `static const char* TAG = "MOD";` 用于日志
- 使用 `#if APP_TARGET_HOST` 支持 PC 端仿真测试

### BMI088 参考驱动架构

```
BMI088driver.c/h       -- 寄存器访问 + 初始化 + 数据读取 (协议无关)
BMI088reg.h            -- 寄存器映射
BMI088Middleware.c/h   -- 平台相关层 (SPI传输, GPIO片选, 延时)
```

SPI 通信: 模式3 (CPOL=1, CPHA=1), 8bit, MSB, 加速度计和陀螺仪各有独立 CS 引脚。

---

## 2. 新增模块架构

```
App/
├── bsp/
│   └── bsp_spi.h/.c          ← [新增] SPI 抽象层
├── device/
│   └── imu_bmi088.h/.c       ← [新增] BMI088 驱动
├── service/
│   └── attitude/
│       ├── attitude_if.h      ← [新增] 姿态接口定义
│       ├── attitude_estimator.h/.c  ← [新增] 偏航角估计 (陀螺仪积分)
│       └── steer_controller.h/.c    ← [新增] 转向 PID 控制器
└── app/
    └── task_chassis.h/.c      ← [修改] 集成转向控制
```

```
                    ┌───────────────────────────────────┐
                    │          task_chassis (500Hz)       │
                    │                                    │
  s_chassis ──────▶│  steer_controller_update()          │
  {vx,vy,wz,      │    │                                │
   target_yaw,    │    │  wz_corrected = yaw_pid(        │
   steer_mode}    │    │    target_yaw - current_yaw)     │
                    │    │                                │
  attitude ──────▶│    │  current_yaw = attitude_est_    │
  estimator       │    │    get_yaw()                     │
  (gyro积分)      │    ▼                                │
                    │  gait_machine_update(vx, vy, wz_c)  │
                    │    │                                │
                    │    ▼                                │
                    │  leg_controller_apply()             │
                    │    │                                │
                    │    ▼                                │
                    │  motor_go_send_all()                │
                    └───────────────────────────────────┘
```

---

## 3. 实现步骤

### 步骤 1: 硬件 SPI 启用

**文件变更:**

| 动作 | 文件 | 说明 |
|------|------|------|
| 修改 | `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_conf.h` | 解除 `HAL_SPI_MODULE_ENABLED` 注释 |
| 修改 | `FengMH.ioc` (CubeMX) | 配置 SPI2: Mode3, 8bit, MSB, 预分频=32 (~15MHz) |
| 修改 | `Core/Inc/spi.h` | CubeMX 重新生成 |
| 修改 | `Core/Src/spi.c` | CubeMX 重新生成 `MX_SPI2_Init()` |
| 修改 | `Core/Inc/gpio.h` | 增加 CS/INT 引脚定义 |
| 修改 | `cmake/firmware.cmake` | 将 `spi.c` 加入源文件列表 |

**SPI2 引脚规划 (复用参考项目):**

| 信号 | 引脚 | 说明 |
|------|------|------|
| SPI2_SCK | PB13 | AF5 |
| SPI2_MOSI | PC1 | AF5 |
| SPI2_MISO | PC2 | AF5 |
| ACC_CS | PC0 | 推挽输出，初始高 |
| GYRO_CS | PC3 | 推挽输出，初始高 |
| ACC_INT | PE10 | 输入，上升沿中断 (可选) |
| GYRO_INT | PE12 | 输入，上升沿中断 (可选) |

---

### 步骤 2: BSP SPI 抽象层

新增 `App/bsp/include/bsp_spi.h` 和 `App/bsp/src/bsp_spi.c`，遵循现有 BSP 层模式。

**设计要点:**
- 与 `bsp_uart` / `bsp_fdcan` 风格一致
- MCU 侧：封装 `HAL_SPI_TransmitReceive()`
- 函数: `bsp_spi_init()`, `bsp_spi_write_read()`, `bsp_spi_set_cs()`
- 支持 `#if APP_TARGET_HOST` host mock 模式
- 支持多种 SPI 外设 (SPI2, SPI3...)

```c
// bsp_spi.h 接口
typedef enum {
    BSP_SPI_2 = 0,
    BSP_SPI_MAX
} bsp_spi_id_t;

app_err_t bsp_spi_init(bsp_spi_id_t id);
app_err_t bsp_spi_write_read(bsp_spi_id_t id, uint8_t tx, uint8_t* rx);
app_err_t bsp_spi_cs_low(bsp_spi_id_t id, uint8_t cs_pin);
app_err_t bsp_spi_cs_high(bsp_spi_id_t id, uint8_t cs_pin);
```

---

### 步骤 3: BMI088 设备驱动

新增 `App/device/include/imu_bmi088.h` 和 `App/device/src/imu_bmi088.c`，基于参考项目移植，适配 FengMH 代码风格。

**改编对照:**

| 参考文件 | FengMH 新文件 | 变更 |
|----------|--------------|------|
| `BMI088reg.h` | 合并到 `imu_bmi088.h` | 寄存器宏内联到头部 |
| `BMI088driver.h` | `imu_bmi088.h` | 命名适配: `BMI088_init` → `imu_bmi088_init` |
| `BMI088driver.c` | `imu_bmi088.c` | 错误码改用 `app_err_t`，日志用 `LOGE/LOGI` |
| `BMI088Middleware.h` | `imu_bmi088.h` (接口) + `imu_bmi088.c` (实现) | 不再需要独立的 middleware 文件 |
| `BMI088Middleware.c` | `imu_bmi088.c` | CS 控制改用 `bsp_spi_*`，延时用 `bsp_time` |

**驱动接口:**

```c
// imu_bmi088.h
typedef enum {
    IMU_ACCEL_RANGE_3G  = 0,
    IMU_ACCEL_RANGE_6G,
    IMU_ACCEL_RANGE_12G,
    IMU_ACCEL_RANGE_24G,
} imu_accel_range_t;

typedef enum {
    IMU_GYRO_RANGE_2000 = 0,
    IMU_GYRO_RANGE_1000,
    IMU_GYRO_RANGE_500,
    IMU_GYRO_RANGE_250,
    IMU_GYRO_RANGE_125,
} imu_gyro_range_t;

typedef struct {
    float gyro[3];    // rad/s (x, y, z)
    float accel[3];   // m/s^2 (x, y, z)
    float temp;       // 摄氏度
    uint8_t status;   // 传感器状态
} imu_bmi088_data_t;

app_err_t imu_bmi088_init(void);
app_err_t imu_bmi088_read(imu_bmi088_data_t* data);
app_err_t imu_bmi088_set_accel_range(imu_accel_range_t range);
app_err_t imu_bmi088_set_gyro_range(imu_gyro_range_t range);
```

**关键实现细节:**
- 加速度计 ODR: 800Hz (ACC_CONF=0xA0)
- 陀螺仪 ODR: 1000Hz / BW=116Hz (GYRO_BANDWIDTH=0x82)
- 加速度计量程: ±3G (稳定状态下足够)
- 陀螺仪量程: ±2000 dps (转向时角速度大)
- 复位等待: 80ms (参考驱动已验证)
- CHIP_ID 验证: ACC=0x1E, GYRO=0x0F

---

### 步骤 4: 姿态估计器

新增 `App/control/include/attitude/attitude_if.h`、`App/control/include/attitude/attitude_estimator.h`
和 `App/control/src/attitude/attitude_estimator.c`。

**设计思路:**
- 转向控制只需要偏航角 (yaw)，暂不引入完整 AHRS
- 使用陀螺仪 Z 轴积分 + 静态误差补偿
- 后续可扩展为互补滤波 / Mahony / Madgwick

```c
// attitude_if.h
typedef struct {
    float yaw;      // 偏航角 (rad)
    float pitch;    // 俯仰角 (rad, 预留)
    float roll;     // 横滚角 (rad, 预留)
} attitude_state_t;

// attitude_estimator.h
app_err_t attitude_estimator_init(void);
app_err_t attitude_estimator_update(float gyro[3], float accel[3], float dt_s);
float     attitude_estimator_get_yaw(void);
void      attitude_estimator_reset_yaw(void);
const attitude_state_t* attitude_estimator_get_state(void);
```

**偏航角积分:**
```
yaw += gyro_z * dt_s  (先不引入 accel 融合，仅陀螺仪积分)
```

**漂移抑制策略 (方案1 — 先简化):**
- 静止时 (gyro 模长 < 阈值) 锁定偏航角不积分
- 后续可用加速度计修正或磁力计

---

### 步骤 5: 转向控制器

新增 `App/control/include/attitude/steer_controller.h` 和
`App/control/src/attitude/steer_controller.c`。

```c
// steer_controller.h
typedef enum {
    STEER_MODE_OFF = 0,      // 无转向控制 (wz 直接透传)
    STEER_MODE_YAW,           // 偏航角闭环 (target_yaw 模式)
    STEER_MODE_YAW_RATE,     // 角速度闭环 (wz 模式, 现有行为)
} steer_mode_t;

typedef struct {
    steer_mode_t mode;
    float target_yaw;         // 目标偏航角 (rad)
    float yaw_kp;             // 比例增益
    float yaw_ki;             // 积分增益
    float yaw_kd;             // 微分增益
    float yaw_i_max;          // 积分限幅
    float wz_max;             // 角速度限幅 (rad/s)
} steer_controller_cfg_t;

app_err_t steer_controller_init(void);
app_err_t steer_controller_set_target(float yaw_rad);
app_err_t steer_controller_set_mode(steer_mode_t mode);
float     steer_controller_update(float current_yaw, float dt_s);
```

**PID 控制律:**
```
yaw_error = wrap_pi(target_yaw - current_yaw)  // 最短路径
wz_output = kp * yaw_error + ki * i_term + kd * d_term
wz_output = clamp(wz_output, -wz_max, wz_max)
```

**偏航角包裹函数:**
```c
static inline float wrap_pi(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
```

---

### 步骤 6: 协议扩展

**文件变更:** `App/service/include/protocol/proto_defs.h`

**方案:** 在 `payload_chassis_cmd_t` 中新增字段：

```c
// proto_defs.h (修改)
#define PROTO_FUNC_CHASSIS_CMD   0x10   // 不变

typedef struct __attribute__((packed)) {
    float vx;         // 前进速度 (m/s)
    float vy;         // 横向速度 (m/s)
    float wz;         // 角速度 (rad/s) — steer_mode=OFF 时直接使用
    float target_yaw; // 目标偏航角 (rad) — steer_mode=YAW 时使用
    uint8_t steer_mode; // 0=OFF(现有行为), 1=YAW(偏航闭环)
} payload_chassis_cmd_t;
```

**兼容性:** 主机端不发送新字段时 (帧长=12)，`steer_mode` 默认为 0，行为不变。

---

### 步骤 7: 集成到 task_chassis

**文件变更:** `App/app/include/task_chassis.h/.c`

**修改点:**

1. 新增静态变量:
   ```c
   static attitude_state_t s_attitude;
   ```

2. 控制循环中 (500Hz) 新增:
   ```c
   // 1. 读取 IMU 数据
   imu_bmi088_data_t imu_data;
   imu_bmi088_read(&imu_data);

   // 2. 更新姿态估计
   attitude_estimator_update(imu_data.gyro, imu_data.accel, DT_S);

   // 3. 转向控制
   if (s_chassis.steer_mode == STEER_MODE_YAW) {
       float current_yaw = attitude_estimator_get_yaw();
       wz_cmd = steer_controller_update(current_yaw, DT_S);
   } else {
       wz_cmd = s_chassis.wz;  // 现有直通行为
   }

   // 4. 步态更新 (使用修正后的 wz)
   gait_machine_update(&s_gm, DT_S, vx_cmd, vy_cmd, wz_cmd, &out);
   ```

3. 初始化函数 `task_chassis_init()` 中新增:
   ```c
   attitude_estimator_init();
   steer_controller_init();
   // 默认参数: kp=2.0, ki=0.1, kd=0.05, wz_max=3.0 rad/s
   ```

---

### 步骤 8: 初始化流程集成

**文件变更:** `App/app/include/app_init.h/.c`

在 `app_init()` 的 BSP 初始化阶段后，设备初始化阶段前：

```c
// BSP 初始化 (已有)
bsp_uart_init_all();
bsp_fdcan_init_all();
// ...

// SPI 初始化 (新增)
bsp_spi_init(BSP_SPI_2);

// IMU 初始化 (新增)
app_err_t err = imu_bmi088_init();
if (err != APP_OK) {
    LOGE(TAG, "BMI088 init failed: %d", err);
    // 不阻塞 — 机器人仍可在无 IMU 下运行 (降级模式)
}
```

---

## 4. 文件清单

| # | 文件 | 动作 | 说明 |
|---|------|------|------|
| 1 | `Drivers/.../stm32h7xx_hal_conf.h` | 修改 | 启用 HAL_SPI_MODULE_ENABLED |
| 2 | `Core/Inc/spi.h` | 修改 | CubeMX 重新生成 SPI2 配置 |
| 3 | `Core/Src/spi.c` | 修改 | CubeMX 生成 MX_SPI2_Init() |
| 4 | `Core/Inc/gpio.h` | 修改 | 添加 CS/INT 引脚定义 |
| 5 | `Core/Inc/main.h` | 修改 | 添加 SPI2 handle 声明 |
| 6 | `Core/Src/main.c` | 修改 | 调用 MX_SPI2_Init() |
| 7 | `Core/Src/stm32h7xx_it.c` | 修改 | 添加 EXTI 中断处理 (可选) |
| 8 | `cmake/firmware.cmake` | 修改 | 添加新源文件到构建 |
| 9 | `App/bsp/include/bsp_spi.h` | **新增** | SPI BSP 抽象层头文件 |
| 10 | `App/bsp/src/bsp_spi.c` | **新增** | SPI BSP 抽象层实现 |
| 11 | `App/device/include/imu_bmi088.h` | **新增** | BMI088 驱动头文件 |
| 12 | `App/device/src/imu_bmi088.c` | **新增** | BMI088 驱动实现 |
| 13 | `App/control/include/attitude/attitude_if.h` | **新增** | 姿态接口定义 |
| 14 | `App/control/include/attitude/attitude_estimator.h` | **新增** | 姿态估计器头文件 |
| 15 | `App/control/src/attitude/attitude_estimator.c` | **新增** | 姿态估计器实现 (陀螺仪积分) |
| 16 | `App/control/include/attitude/steer_controller.h` | **新增** | 转向控制器头文件 |
| 17 | `App/control/src/attitude/steer_controller.c` | **新增** | 转向 PID 控制器实现 |
| 18 | `App/service/include/protocol/proto_defs.h` | 修改 | 扩展 chassis_cmd payload |
| 19 | `App/service/src/protocol/proto_dispatch.c` | 修改 | 适配新的 payload 长度检查 |
| 20 | `App/app/include/task_chassis.h` | 修改 | 导出 yaw_reset/steer 接口 |
| 21 | `App/app/src/task_chassis.c` | 修改 | 集成 IMU + 转向控制 |
| 22 | `App/app/src/app_init.c` | 修改 | 添加 SPI + IMU 初始化 |
| 23 | `App/common/include/err.h` | 修改 | 添加 IMU 相关错误码 |

**总计: 8 个修改文件 + 9 个新增文件 = 17 个文件变更**

---

## 5. 解耦策略

1. **BSP 抽象**: IMU 驱动不直接调用 HAL，通过 `bsp_spi` 适配层，支持 host mock 测试
2. **接口隔离**: `attitude_estimator` 不依赖 BMI088 — 接收 `float gyro[3]` 即可，可更换 IMU
3. **模式开关**: `steer_mode` 允许在 YAW 闭环和 WZ 直通之间切换，不影响现有步态
4. **降级运行**: IMU 初始化失败不影响底盘基本控制，`steer_mode` 始终为 OFF 时行为不变
5. **独立编译单元**: 每个新增模块都是独立的 `.c` 文件，不侵入现有模块内部实现

---

## 6. 测试策略

### 6.1 Host 端仿真测试

- `imu_bmi088.c` 使用 `#if APP_TARGET_HOST` 提供 mock 数据
- `attitude_estimator` 在 host 端独立运行，测试积分精度
- `steer_controller` PID 在 host 端测试阶跃响应

### 6.2 MCU 端分阶段测试

| 阶段 | 测试内容 | 验证方式 |
|------|---------|---------|
| 阶段 1 | SPI2 引脚电平 + 读写寄存器 | 逻辑分析仪 / CHIP_ID 回读 |
| 阶段 2 | BMI088 初始化 + 数据读取 | USB CDC 输出 gyro/accel 值 |
| 阶段 3 | 陀螺仪偏航积分 (手动旋转) | 串口输出 yaw 角度 |
| 阶段 4 | 转向 PID (悬空状态) | 给定目标角，观察 wz 输出 |
| 阶段 5 | 整机集成测试 | 设 target_yaw，走路验证转向 |

---

## 7. 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `steer_controller.yaw_kp` | 2.0 | P 增益 |
| `steer_controller.yaw_ki` | 0.1 | I 增益 |
| `steer_controller.yaw_kd` | 0.05 | D 增益 |
| `steer_controller.yaw_i_max` | 0.5 | 积分限幅 (rad) |
| `steer_controller.wz_max` | 3.0 | 输出角速度限幅 (rad/s) |
| `attitude_estimator.gyro_thresh` | 0.01 | 静止判定阈值 (rad/s) |

---

## 8. 后续扩展方向

- [ ] 加速度计融合 → 完整 AHRS (Madgwick/Mahony)
- [ ] 磁力计 → 绝对航向参考
- [ ] 自动校准 → 静止时自动归零 gyro bias
- [ ] 路径跟踪 → 结合 vx + yaw 实现轨迹控制
- [ ] 上位机可视化 → 实时显示 yaw 角度和目标
