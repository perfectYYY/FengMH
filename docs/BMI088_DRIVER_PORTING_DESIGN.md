# BMI088 驱动层移植设计与实现说明

本文档描述如何把 `Wheel-legged` 中的 BMI088 驱动移植到 FengMH 项目，并保持当前项目的代码风格、分层解耦和 host/mock 测试能力。

目标不是原样复制 `BMI088driver.c/.h`、`BMI088Middleware.c/.h`，而是保留已验证的寄存器配置、SPI 读写时序和初始化流程，改造成 FengMH 的 `App/device + App/bsp` 架构。

## 1. 参考来源

参考文件：

| 来源文件 | 作用 | FengMH 中的落点 |
|---|---|---|
| `Wheel-legged/Core/Src/BMI088driver.c` | BMI088 初始化、寄存器读写、数据读取 | `App/device/src/imu_bmi088.c` |
| `Wheel-legged/Core/Inc/BMI088driver.h` | 对外接口、错误码、灵敏度系数 | `App/device/include/imu_bmi088.h` |
| `Wheel-legged/Core/Inc/BMI088reg.h` | BMI088 寄存器宏 | `App/device/src/imu_bmi088.c` 内部宏，或独立私有头 |
| `Wheel-legged/Core/Src/BMI088Middleware.c` | SPI、CS、delay 平台适配 | `App/bsp/include/bsp_spi.h` + `App/bsp/src/bsp_spi.c` + `bsp_time.*` |
| `Wheel-legged/Core/Src/ImuTask.c` | 5 ms 周期读取 BMI088 并更新姿态 | `task_chassis` 或后续独立 `task_imu` |

保留 Wheel-legged 的核心点：

- BMI088 使用 SPI2。
- BMI088 加速度计和陀螺仪是两个独立 SPI slave，各自一个 CS。
- SPI 读操作中，加速度计读取需要额外 dummy byte。
- 初始化顺序是 `chip id 检查 -> soft reset -> 再检查 chip id -> 写配置表 -> 回读校验`。
- 默认配置为加速度计 800 Hz、±3G，陀螺仪 1000 Hz / 116 Hz、±2000 dps。

## 2. CubeMX 配置提醒

涉及 `Core/Src/spi.c`、`Core/Src/gpio.c`、`Core/Inc/main.h` 的内容，必须通过 CubeMX 修改后生成，不建议手写覆盖。

你需要在 CubeMX 中确认：

| 项 | 配置 |
|---|---|
| SPI | SPI2 |
| Mode | Master |
| Direction | 2 Lines Full Duplex |
| Data Size | 8 Bits |
| First Bit | MSB First |
| NSS | Software |
| CPOL | Low |
| CPHA | 1 Edge |
| SPI Mode | Mode 0 |
| Prescaler | 16，当前约 7.5 Mbits/s |

引脚需要和实际硬件一致：

| 信号 | 建议引脚 | 说明 |
|---|---|---|
| SPI2_SCK | PB13 | 与 `Wheel-legged` 参考工程一致 |
| SPI2_MOSI | PC1 | AF5 |
| SPI2_MISO | PC2_C | AF5 |
| ACC_CS | PC0 | GPIO Output，默认 High |
| GYRO_CS | PC3_C | GPIO Output，默认 High |
| ACC_INT | PE10 | GPIO EXTI Rising；当前轮询读取，不依赖中断 |
| GYRO_INT | PE12 | GPIO EXTI Rising；当前轮询读取，不依赖中断 |

当前仓库已经按 `Wheel-legged` 统一为 `PB13/PC1/PC2_C + PC0/PC3_C`。后续再次打开 CubeMX 时，需要确认 CS 默认电平仍为 High，SPI2 仍为 Mode0；CubeMX 曾把 CS 默认电平生成成 Low，会导致上电时两个 BMI088 从设备被同时选中。

## 3. 分层架构

BMI088 驱动层只负责“传感器初始化和数据读取”，不做姿态解算、不参与转向控制、不直接碰 HAL。

推荐结构：

```text
App/
├── bsp/
│   ├── bsp_spi.h
│   └── bsp_spi.c
├── device/
│   ├── imu_bmi088.h
│   └── imu_bmi088.c
├── service/
│   └── attitude/
│       ├── attitude_estimator.h/.c
│       └── steer_controller.h/.c
└── app/
    └── task_chassis.c
```

依赖方向：

```text
task_chassis
    -> attitude_estimator
    -> imu_bmi088
    -> bsp_spi / bsp_time
    -> CubeMX HAL layer
```

禁止方向：

- `imu_bmi088.c` 不直接 include `spi.h`、`gpio.h`、`stm32h7xx_hal.h`。
- `imu_bmi088.c` 不调用 `HAL_GPIO_WritePin()`。
- `imu_bmi088.c` 不做 yaw 积分。
- `bsp_spi.c` 不知道 BMI088 寄存器含义。

## 4. 命名映射

从 Wheel-legged 移植时按以下方式改名：

| Wheel-legged 函数 | FengMH 函数 | 可见性 |
|---|---|---|
| `BMI088_init()` | `imu_bmi088_init()` | public |
| `bmi088_accel_init()` | `bmi088_accel_init()` | static |
| `bmi088_gyro_init()` | `bmi088_gyro_init()` | static |
| `BMI088_read()` | `imu_bmi088_read()` | public |
| `BMI088_write_single_reg()` | `bmi088_write_reg()` | static |
| `BMI088_read_single_reg()` | `bmi088_read_reg()` | static |
| `BMI088_read_muli_reg()` | `bmi088_read_multi()` | static |
| `BMI088_ACCEL_NS_L/H()` | `bsp_spi_set_cs(BSP_SPI_CS_BMI088_ACCEL, selected)` | BSP |
| `BMI088_GYRO_NS_L/H()` | `bsp_spi_set_cs(BSP_SPI_CS_BMI088_GYRO, selected)` | BSP |
| `BMI088_delay_ms/us()` | `bsp_time_delay_ms/us()` | BSP |

## 5. 对外接口

目标头文件：`App/device/include/imu_bmi088.h`

```c
#ifndef APP_DEVICE_IMU_BMI088_H_
#define APP_DEVICE_IMU_BMI088_H_

#include "err.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_BMI088_ACCEL_RANGE_3G = 0,
    IMU_BMI088_ACCEL_RANGE_6G,
    IMU_BMI088_ACCEL_RANGE_12G,
    IMU_BMI088_ACCEL_RANGE_24G,
} imu_bmi088_accel_range_t;

typedef enum {
    IMU_BMI088_GYRO_RANGE_2000DPS = 0,
    IMU_BMI088_GYRO_RANGE_1000DPS,
    IMU_BMI088_GYRO_RANGE_500DPS,
    IMU_BMI088_GYRO_RANGE_250DPS,
    IMU_BMI088_GYRO_RANGE_125DPS,
} imu_bmi088_gyro_range_t;

typedef struct {
    float gyro[3];       /* rad/s: x, y, z */
    float accel[3];      /* m/s^2: x, y, z */
    float temperature_c;
    uint8_t status;      /* 0=valid */
} imu_bmi088_data_t;

app_err_t imu_bmi088_init(void);
app_err_t imu_bmi088_read(imu_bmi088_data_t* out);
app_err_t imu_bmi088_set_accel_range(imu_bmi088_accel_range_t range);
app_err_t imu_bmi088_set_gyro_range(imu_bmi088_gyro_range_t range);
uint8_t   imu_bmi088_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif
```

接口语义：

| 接口 | 语义 |
|---|---|
| `imu_bmi088_init()` | 初始化 accel + gyro；失败返回错误，不应死循环阻塞 |
| `imu_bmi088_read()` | 读取一次 accel、gyro、temperature，单位转换为 SI 单位 |
| `imu_bmi088_set_accel_range()` | 设置加速度计量程，同时更新换算系数 |
| `imu_bmi088_set_gyro_range()` | 设置陀螺仪量程，同时更新换算系数 |
| `imu_bmi088_is_ready()` | 上层判断 IMU 是否可用 |

## 6. 私有寄存器与常量

建议先放在 `imu_bmi088.c` 顶部，避免污染全局 include。

```c
#define BMI088_ACC_CHIP_ID              0x00u
#define BMI088_ACC_CHIP_ID_VALUE        0x1Eu
#define BMI088_ACC_XOUT_L               0x12u
#define BMI088_TEMP_M                   0x22u
#define BMI088_TEMP_L                   0x23u
#define BMI088_ACC_CONF                 0x40u
#define BMI088_ACC_RANGE                0x41u
#define BMI088_INT1_IO_CTRL             0x53u
#define BMI088_INT_MAP_DATA             0x58u
#define BMI088_ACC_PWR_CONF             0x7Cu
#define BMI088_ACC_PWR_CTRL             0x7Du
#define BMI088_ACC_SOFTRESET            0x7Eu

#define BMI088_GYRO_CHIP_ID             0x00u
#define BMI088_GYRO_CHIP_ID_VALUE       0x0Fu
#define BMI088_GYRO_X_L                 0x02u
#define BMI088_GYRO_RANGE               0x0Fu
#define BMI088_GYRO_BANDWIDTH           0x10u
#define BMI088_GYRO_LPM1                0x11u
#define BMI088_GYRO_SOFTRESET           0x14u
#define BMI088_GYRO_CTRL                0x15u
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16u
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18u

#define BMI088_SOFTRESET_VALUE          0xB6u
#define BMI088_LONG_DELAY_MS            80u
#define BMI088_COM_WAIT_US              150u
#define BMI088_TEMP_FACTOR              0.125f
#define BMI088_TEMP_OFFSET              23.0f
#define BMI088_GRAVITY_MPS2             9.80665f
#define BMI088_DEG_TO_RAD               0.017453292519943295f

#define BMI088_ACC_READ_REG(_reg)       ((uint8_t)((_reg) | 0x80u))
#define BMI088_ACC_WRITE_REG(_reg)      ((uint8_t)((_reg) & 0x7Fu))
#define BMI088_GYRO_READ_REG(_reg)      ((uint8_t)((_reg) | 0x80u))
#define BMI088_GYRO_WRITE_REG(_reg)     ((uint8_t)((_reg) & 0x7Fu))
```

## 7. 静态状态

```c
typedef struct {
    uint8_t reg;
    uint8_t val;
    app_err_t err;
} bmi088_reg_cfg_t;

static const char* TAG = "BMI088";

static uint8_t s_ready;
static imu_bmi088_accel_range_t s_accel_range = IMU_BMI088_ACCEL_RANGE_3G;
static imu_bmi088_gyro_range_t s_gyro_range = IMU_BMI088_GYRO_RANGE_2000DPS;

static float s_accel_lsb_per_g = 10920.0f;
static float s_gyro_lsb_per_dps = 16.384f;
```

配置表照搬 Wheel-legged 的寄存器组合：

```c
static const bmi088_reg_cfg_t S_ACC_INIT_CFG[] = {
    { BMI088_ACC_PWR_CTRL,  0x04u, APP_ERR_IMU_ACC_PWR_CTRL },
    { BMI088_ACC_PWR_CONF,  0x00u, APP_ERR_IMU_ACC_PWR_CONF },
    { BMI088_ACC_CONF,      0xABu, APP_ERR_IMU_ACC_CONF },
    { BMI088_ACC_RANGE,     0x00u, APP_ERR_IMU_ACC_RANGE },
    { BMI088_INT1_IO_CTRL,  0x08u, APP_ERR_IMU_ACC_INT },
    { BMI088_INT_MAP_DATA,  0x04u, APP_ERR_IMU_ACC_INT },
};

static const bmi088_reg_cfg_t S_GYRO_INIT_CFG[] = {
    { BMI088_GYRO_RANGE,             0x00u, APP_ERR_IMU_GYRO_RANGE },
    { BMI088_GYRO_BANDWIDTH,         0x82u, APP_ERR_IMU_GYRO_BW },
    { BMI088_GYRO_LPM1,              0x00u, APP_ERR_IMU_GYRO_LPM },
    { BMI088_GYRO_CTRL,              0x80u, APP_ERR_IMU_GYRO_CTRL },
    { BMI088_GYRO_INT3_INT4_IO_CONF, 0x00u, APP_ERR_IMU_GYRO_INT },
    { BMI088_GYRO_INT3_INT4_IO_MAP,  0x01u, APP_ERR_IMU_GYRO_INT },
};
```

如果 `err.h` 暂时没有这些细分错误码，可以先统一返回 `APP_ERR_IO`，日志中保留具体寄存器名。后续再补细分错误码。

## 8. BSP SPI 依赖

目标接口已在 FengMH 中存在：

```c
app_err_t bsp_spi_init(bsp_spi_bus_t bus);
app_err_t bsp_spi_set_cs(bsp_spi_cs_t cs, uint8_t selected);
app_err_t bsp_spi_transmit_receive(bsp_spi_bus_t bus,
                                   const uint8_t* tx,
                                   uint8_t* rx,
                                   uint32_t len,
                                   uint32_t timeout_ms);
```

CS 语义：

```c
bsp_spi_set_cs(BSP_SPI_CS_BMI088_ACCEL, 1); /* selected: CS low */
bsp_spi_set_cs(BSP_SPI_CS_BMI088_ACCEL, 0); /* unselected: CS high */
```

`bsp_spi.c` 是唯一允许调用 `HAL_SPI_TransmitReceive()` 和 `HAL_GPIO_WritePin()` 的地方。

## 9. SPI 基础函数实现

### 9.1 单字节收发

```c
static app_err_t bmi088_xfer_byte(uint8_t tx, uint8_t* rx)
{
    uint8_t local_rx = 0U;
    app_err_t err;

    err = bsp_spi_transmit_receive(BSP_SPI_2, &tx, &local_rx, 1U, 10U);
    if (rx) {
        *rx = local_rx;
    }
    return err;
}
```

### 9.2 通用写寄存器

```c
static app_err_t bmi088_write_reg(bsp_spi_cs_t cs, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];
    app_err_t err;

    tx[0] = reg;
    tx[1] = val;

    err = bsp_spi_set_cs(cs, 1U);
    if (err != APP_OK) return err;

    err = bsp_spi_transmit_receive(BSP_SPI_2, tx, rx, 2U, 10U);

    (void)bsp_spi_set_cs(cs, 0U);
    return err;
}
```

### 9.3 通用读寄存器

注意：加速度计读操作比陀螺仪多一个 dummy byte。

```c
static app_err_t bmi088_read_reg(bsp_spi_cs_t cs,
                                 uint8_t reg,
                                 uint8_t has_extra_dummy,
                                 uint8_t* val)
{
    uint8_t tx[3] = { 0 };
    uint8_t rx[3] = { 0 };
    uint32_t len;
    app_err_t err;

    if (!val) return APP_ERR_INVALID_ARG;

    tx[0] = (uint8_t)(reg | 0x80u);
    tx[1] = 0x55u;
    tx[2] = 0x55u;
    len = has_extra_dummy ? 3U : 2U;

    err = bsp_spi_set_cs(cs, 1U);
    if (err != APP_OK) return err;

    err = bsp_spi_transmit_receive(BSP_SPI_2, tx, rx, len, 10U);

    (void)bsp_spi_set_cs(cs, 0U);
    if (err != APP_OK) return err;

    *val = has_extra_dummy ? rx[2] : rx[1];
    return APP_OK;
}
```

### 9.4 通用连续读取

这是移植时最容易出错的地方。不要只传一个字节 tx 指针却要求 HAL 收发 `1 + len` 字节；必须提供足够长度的 tx/rx 缓冲区。

```c
static app_err_t bmi088_read_multi(bsp_spi_cs_t cs,
                                   uint8_t reg,
                                   uint8_t has_extra_dummy,
                                   uint8_t* buf,
                                   uint32_t len)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint32_t total;
    uint32_t offset;
    app_err_t err;

    if (!buf || len == 0U) return APP_ERR_INVALID_ARG;
    if (len > 8U) return APP_ERR_INVALID_ARG;

    offset = has_extra_dummy ? 2U : 1U;
    total = len + offset;

    memset(tx, 0x55, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = (uint8_t)(reg | 0x80u);

    err = bsp_spi_set_cs(cs, 1U);
    if (err != APP_OK) return err;

    err = bsp_spi_transmit_receive(BSP_SPI_2, tx, rx, total, 10U);

    (void)bsp_spi_set_cs(cs, 0U);
    if (err != APP_OK) return err;

    memcpy(buf, &rx[offset], len);
    return APP_OK;
}
```

封装成更清晰的 accel/gyro 函数：

```c
static app_err_t accel_write_reg(uint8_t reg, uint8_t val)
{
    return bmi088_write_reg(BSP_SPI_CS_BMI088_ACCEL,
                            BMI088_ACC_WRITE_REG(reg),
                            val);
}

static app_err_t accel_read_reg(uint8_t reg, uint8_t* val)
{
    return bmi088_read_reg(BSP_SPI_CS_BMI088_ACCEL,
                           BMI088_ACC_READ_REG(reg),
                           1U,
                           val);
}

static app_err_t accel_read_multi(uint8_t reg, uint8_t* buf, uint32_t len)
{
    return bmi088_read_multi(BSP_SPI_CS_BMI088_ACCEL,
                             BMI088_ACC_READ_REG(reg),
                             1U,
                             buf,
                             len);
}

static app_err_t gyro_write_reg(uint8_t reg, uint8_t val)
{
    return bmi088_write_reg(BSP_SPI_CS_BMI088_GYRO,
                            BMI088_GYRO_WRITE_REG(reg),
                            val);
}

static app_err_t gyro_read_reg(uint8_t reg, uint8_t* val)
{
    return bmi088_read_reg(BSP_SPI_CS_BMI088_GYRO,
                           BMI088_GYRO_READ_REG(reg),
                           0U,
                           val);
}

static app_err_t gyro_read_multi(uint8_t reg, uint8_t* buf, uint32_t len)
{
    return bmi088_read_multi(BSP_SPI_CS_BMI088_GYRO,
                             BMI088_GYRO_READ_REG(reg),
                             0U,
                             buf,
                             len);
}
```

说明：如果 `bmi088_read_reg()` 内部已经 `reg | 0x80`，外层就不要再次 `BMI088_ACC_READ_REG(reg)`。最终实现时二选一即可，避免重复 OR 没问题但语义混乱。推荐让 `bmi088_read_reg()` 内部负责 OR，外层传裸寄存器地址。

## 10. 初始化函数实现

### 10.1 写配置并回读校验

```c
static app_err_t bmi088_write_check(app_err_t (*write_fn)(uint8_t, uint8_t),
                                    app_err_t (*read_fn)(uint8_t, uint8_t*),
                                    const bmi088_reg_cfg_t* cfg,
                                    uint32_t cfg_count)
{
    for (uint32_t i = 0; i < cfg_count; i++) {
        uint8_t readback = 0U;
        app_err_t err;

        err = write_fn(cfg[i].reg, cfg[i].val);
        if (err != APP_OK) return err;

        bsp_time_delay_us(BMI088_COM_WAIT_US);

        err = read_fn(cfg[i].reg, &readback);
        if (err != APP_OK) return err;

        if (readback != cfg[i].val) {
            LOGE("reg 0x%02x mismatch: write=0x%02x read=0x%02x",
                 cfg[i].reg, cfg[i].val, readback);
            return cfg[i].err;
        }
    }

    return APP_OK;
}
```

### 10.2 加速度计初始化

```c
static app_err_t bmi088_accel_init(void)
{
    uint8_t id = 0U;
    app_err_t err;

    err = accel_read_reg(BMI088_ACC_CHIP_ID, &id);
    if (err != APP_OK) return err;
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    err = accel_write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFTRESET_VALUE);
    if (err != APP_OK) return err;
    bsp_time_delay_ms(BMI088_LONG_DELAY_MS);

    err = accel_read_reg(BMI088_ACC_CHIP_ID, &id);
    if (err != APP_OK) return err;
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    if (id != BMI088_ACC_CHIP_ID_VALUE) {
        LOGE("acc chip id invalid: 0x%02x", id);
        return APP_ERR_IO;
    }

    err = bmi088_write_check(accel_write_reg,
                             accel_read_reg,
                             S_ACC_INIT_CFG,
                             ARRAY_SIZE(S_ACC_INIT_CFG));
    if (err != APP_OK) return err;

    LOGI("acc init ok");
    return APP_OK;
}
```

### 10.3 陀螺仪初始化

```c
static app_err_t bmi088_gyro_init(void)
{
    uint8_t id = 0U;
    app_err_t err;

    err = gyro_read_reg(BMI088_GYRO_CHIP_ID, &id);
    if (err != APP_OK) return err;
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    err = gyro_write_reg(BMI088_GYRO_SOFTRESET, BMI088_SOFTRESET_VALUE);
    if (err != APP_OK) return err;
    bsp_time_delay_ms(BMI088_LONG_DELAY_MS);

    err = gyro_read_reg(BMI088_GYRO_CHIP_ID, &id);
    if (err != APP_OK) return err;
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    if (id != BMI088_GYRO_CHIP_ID_VALUE) {
        LOGE("gyro chip id invalid: 0x%02x", id);
        return APP_ERR_IO;
    }

    err = bmi088_write_check(gyro_write_reg,
                             gyro_read_reg,
                             S_GYRO_INIT_CFG,
                             ARRAY_SIZE(S_GYRO_INIT_CFG));
    if (err != APP_OK) return err;

    LOGI("gyro init ok");
    return APP_OK;
}
```

### 10.4 对外初始化

```c
app_err_t imu_bmi088_init(void)
{
    app_err_t err;

#if APP_TARGET_HOST
    s_ready = 1U;
    return APP_OK;
#endif

    s_ready = 0U;

    err = bmi088_accel_init();
    if (err != APP_OK) {
        LOGE("acc init failed: %d", (int)err);
        return err;
    }

    err = bmi088_gyro_init();
    if (err != APP_OK) {
        LOGE("gyro init failed: %d", (int)err);
        return err;
    }

    s_ready = 1U;
    LOGI("init ok");
    return APP_OK;
}
```

## 11. 数据读取函数实现

### 11.1 原始值转换

```c
static int16_t make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

static float accel_raw_to_mps2(int16_t raw)
{
    return ((float)raw / s_accel_lsb_per_g) * BMI088_GRAVITY_MPS2;
}

static float gyro_raw_to_rads(int16_t raw)
{
    return ((float)raw / s_gyro_lsb_per_dps) * BMI088_DEG_TO_RAD;
}
```

### 11.2 温度读取

```c
static app_err_t bmi088_read_temperature(float* temperature_c)
{
    uint8_t buf[2];
    int16_t raw;
    app_err_t err;

    if (!temperature_c) return APP_ERR_INVALID_ARG;

    err = accel_read_multi(BMI088_TEMP_M, buf, 2U);
    if (err != APP_OK) return err;

    raw = (int16_t)(((uint16_t)buf[0] << 3) | ((uint16_t)buf[1] >> 5));
    if (raw > 1023) {
        raw -= 2048;
    }

    *temperature_c = ((float)raw * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
    return APP_OK;
}
```

### 11.3 对外读取

```c
app_err_t imu_bmi088_read(imu_bmi088_data_t* out)
{
    uint8_t buf[8];
    app_err_t err;

    if (!out) return APP_ERR_INVALID_ARG;
    if (!s_ready) return APP_ERR_UNINIT;

#if APP_TARGET_HOST
    memset(out, 0, sizeof(*out));
    out->status = 0U;
    return APP_OK;
#endif

    memset(out, 0, sizeof(*out));

    err = accel_read_multi(BMI088_ACC_XOUT_L, buf, 6U);
    if (err != APP_OK) {
        out->status = 1U;
        return err;
    }

    out->accel[0] = accel_raw_to_mps2(make_i16(buf[0], buf[1]));
    out->accel[1] = accel_raw_to_mps2(make_i16(buf[2], buf[3]));
    out->accel[2] = accel_raw_to_mps2(make_i16(buf[4], buf[5]));

    err = gyro_read_multi(BMI088_GYRO_X_L, buf, 6U);
    if (err != APP_OK) {
        out->status = 1U;
        return err;
    }

    out->gyro[0] = gyro_raw_to_rads(make_i16(buf[0], buf[1]));
    out->gyro[1] = gyro_raw_to_rads(make_i16(buf[2], buf[3]));
    out->gyro[2] = gyro_raw_to_rads(make_i16(buf[4], buf[5]));

    err = bmi088_read_temperature(&out->temperature_c);
    if (err != APP_OK) {
        out->status = 2U;
        return err;
    }

    out->status = 0U;
    return APP_OK;
}
```

和 Wheel-legged 的差异：

- Wheel-legged 从 `BMI088_GYRO_CHIP_ID` 开始读 8 字节，然后跳过 chip id 取 gyro 数据。
- FengMH 推荐直接从 `BMI088_GYRO_X_L` 读 6 字节，逻辑更清晰。
- 如果想完全照抄 Wheel-legged 的方式，也可以从 `0x00` 读 8 字节，但要先确认 `buf[0] == 0x0F`。

## 12. 量程设置函数

```c
app_err_t imu_bmi088_set_accel_range(imu_bmi088_accel_range_t range)
{
    uint8_t reg = 0U;
    float lsb_per_g = 10920.0f;

    switch (range) {
        case IMU_BMI088_ACCEL_RANGE_3G:
            reg = 0x00u;
            lsb_per_g = 10920.0f;
            break;
        case IMU_BMI088_ACCEL_RANGE_6G:
            reg = 0x01u;
            lsb_per_g = 5460.0f;
            break;
        case IMU_BMI088_ACCEL_RANGE_12G:
            reg = 0x02u;
            lsb_per_g = 2730.0f;
            break;
        case IMU_BMI088_ACCEL_RANGE_24G:
            reg = 0x03u;
            lsb_per_g = 1365.0f;
            break;
        default:
            return APP_ERR_INVALID_ARG;
    }

    if (s_ready) {
        app_err_t err = accel_write_reg(BMI088_ACC_RANGE, reg);
        if (err != APP_OK) return err;
    }

    s_accel_range = range;
    s_accel_lsb_per_g = lsb_per_g;
    return APP_OK;
}

app_err_t imu_bmi088_set_gyro_range(imu_bmi088_gyro_range_t range)
{
    uint8_t reg = 0U;
    float lsb_per_dps = 16.384f;

    switch (range) {
        case IMU_BMI088_GYRO_RANGE_2000DPS:
            reg = 0x00u;
            lsb_per_dps = 16.384f;
            break;
        case IMU_BMI088_GYRO_RANGE_1000DPS:
            reg = 0x01u;
            lsb_per_dps = 32.768f;
            break;
        case IMU_BMI088_GYRO_RANGE_500DPS:
            reg = 0x02u;
            lsb_per_dps = 65.536f;
            break;
        case IMU_BMI088_GYRO_RANGE_250DPS:
            reg = 0x03u;
            lsb_per_dps = 131.072f;
            break;
        case IMU_BMI088_GYRO_RANGE_125DPS:
            reg = 0x04u;
            lsb_per_dps = 262.144f;
            break;
        default:
            return APP_ERR_INVALID_ARG;
    }

    if (s_ready) {
        app_err_t err = gyro_write_reg(BMI088_GYRO_RANGE, reg);
        if (err != APP_OK) return err;
    }

    s_gyro_range = range;
    s_gyro_lsb_per_dps = lsb_per_dps;
    return APP_OK;
}

uint8_t imu_bmi088_is_ready(void)
{
    return s_ready;
}
```

## 13. 上层调用逻辑

### 13.1 初始化

`app_init()` 中：

```c
bsp_spi_init(BSP_SPI_2);

app_err_t err = imu_bmi088_init();
if (err != APP_OK) {
    LOGW("BMI088 init failed: %d, steering disabled", (int)err);
}
```

初始化失败不阻塞整车，转向功能降级为 `wz` 直通。

### 13.2 周期读取

如果先不新增 `task_imu`，可在 `task_chassis_step_for_test()` 中读取：

```c
imu_bmi088_data_t imu;

if (imu_bmi088_read(&imu) == APP_OK) {
    attitude_estimator_update(imu.gyro, imu.accel, dt_s);
}
```

后续如果 IMU 和 chassis 解耦，可新增 `task_imu`：

```text
task_imu 500Hz:
    imu_bmi088_read()
    attitude_estimator_update()

task_chassis 500Hz:
    yaw = attitude_estimator_get_yaw()
    wz = steer_controller_update(yaw, dt)
```

## 14. 代码逻辑流程

初始化流程：

```text
app_init
  -> bsp_spi_init(SPI2)
  -> imu_bmi088_init
      -> bmi088_accel_init
          -> read ACC_CHIP_ID
          -> write ACC_SOFTRESET
          -> delay 80 ms
          -> read ACC_CHIP_ID
          -> write/check ACC config table
      -> bmi088_gyro_init
          -> read GYRO_CHIP_ID
          -> write GYRO_SOFTRESET
          -> delay 80 ms
          -> read GYRO_CHIP_ID
          -> write/check GYRO config table
      -> s_ready = 1
```

读取流程：

```text
imu_bmi088_read
  -> accel_read_multi(ACC_XOUT_L, 6)
  -> raw accel -> m/s^2
  -> gyro_read_multi(GYRO_X_L, 6)
  -> raw gyro -> rad/s
  -> accel_read_multi(TEMP_M, 2)
  -> raw temp -> degC
```

转向使用流程：

```text
upper computer sends:
    vx + target_yaw + steer_mode

task_chassis:
    imu_bmi088_read
    attitude_estimator_update
    current_yaw = attitude_estimator_get_yaw
    wz = steer_controller_update(current_yaw, dt)
    chassis_planner_update(vx, wz)
    gait/leg/wheel output
```

当前实现中 `PROTO_FUNC_CHASSIS_CMD` 兼容两种 payload：

| 长度 | 内容 | 行为 |
|---|---|---|
| 12 bytes | `vx, vy, wz` | 旧协议，`wz` 直通 |
| 17 bytes | `vx, vy, wz, target_yaw, steer_mode` | `steer_mode=1` 时使用 BMI088 yaw 闭环输出有效 `wz` |

`chassis_planner_update()` 负责把有效 `vx/wz` 转成两类输出：一是参考 Wheel-legged 步长公式动态调整 trot 周期，二是按左右轮差速生成四个 `wheel_rads`。该模块不读取 IMU、不碰电机，是纯 service 层逻辑，便于后续替换成更完整的轨迹规划器。

## 15. Host 测试策略

`APP_TARGET_HOST` 下：

- `imu_bmi088_init()` 直接返回 `APP_OK`。
- `imu_bmi088_read()` 默认返回零值。
- 如果要测寄存器读写，可以利用 `bsp_spi_test_attach_xfer()` 注入 fake SPI 行为。

建议单测：

| 测试 | 期望 |
|---|---|
| host init | 返回 `APP_OK`，`imu_bmi088_is_ready()==1` |
| host read | gyro/accel/temp 全零，status=0 |
| accel raw convert | 10920 LSB -> 9.80665 m/s^2 |
| gyro raw convert | 16.384 LSB -> 1 dps -> 0.01745 rad/s |
| bad arg | `imu_bmi088_read(NULL)` 返回 `APP_ERR_INVALID_ARG` |
| not ready | 未 init 前 read 返回 `APP_ERR_UNINIT` |

## 16. 实装注意点

1. 不要把 Wheel-legged 的 `BMI088_delay_us()` 直接搬过来。它写死了 480 MHz SysTick，FengMH 应走 `bsp_time_delay_us()`。
2. 不要让 `imu_bmi088.c` 直接依赖 `hspi2`。`hspi2` 只能留在 `bsp_spi.c`。
3. 加速度计 SPI 读必须处理 dummy byte，否则 chip id 和 accel 数据会错位。
4. 连续读取必须准备完整 tx/rx 缓冲区，不能传单字节地址却读取多个字节。
5. 初始化失败应返回错误，上层降级；不要在驱动里 `while(1)`。
6. `task_chassis` 不应每周期重复调用 `steer_controller_set_mode()`，否则会一直清 PID 积分。
7. 驱动输出单位统一为 SI：gyro `rad/s`，accel `m/s^2`，temperature `degC`。

## 17. 建议实施顺序

1. 通过 CubeMX 修正 SPI2 和 CS 引脚，重新生成。
2. 整理 `bsp_spi.c`，确认 CS 语义和 `hspi2` 绑定正确。
3. 按本文实现 `imu_bmi088.c/h`。
4. 在 `app_init()` 调 `imu_bmi088_init()`，日志打印 chip id。
5. 在调试输出中观察 accel/gyro/temp。
6. 再接入 `attitude_estimator` 和 `steer_controller`。
7. 最后做 `vx + target_yaw` 的自动转向规划。

本轮实装状态：

- `imu_bmi088.c/h` 已按 Wheel-legged 寄存器配置、dummy-byte 时序和 SI 单位输出完成移植。
- `bsp_time_delay_us()` 已补齐，供 BMI088 初始化寄存器间隔使用。
- `task_comm` 已支持 12/17 字节下行协议兼容。
- `chassis_planner` 已作为独立 service 接入 `task_chassis`，`vx + target_yaw` 会在 `steer_mode=1` 时走 yaw PID，再输出左右轮差速。
- `task_chassis` 只在转向模式变化时重置 PID，避免每周期清积分。
