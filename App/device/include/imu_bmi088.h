/*
 * imu_bmi088.h — BMI088 IMU 设备驱动
 *
 * BMI088 集成 6 轴惯性传感器:
 *   - 加速度计 (ACC): ±3G ~ ±24G, ODR up to 1600Hz
 *   - 陀螺仪 (GYRO):  ±125dps ~ ±2000dps, ODR up to 2000Hz
 *
 * SPI2 总线, 模式0 (CPOL=0 CPHA=1Edge), 软 NSS,
 * 通过 bsp_spi 抽象层访问, 支持 host mock 测试。
 */
#ifndef APP_DEVICE_IMU_BMI088_H_
#define APP_DEVICE_IMU_BMI088_H_

#include "err.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 寄存器地址 ─── */

/* ACC (加速度计) 寄存器 */
#define BMI088_ACC_CHIP_ID         0x00u   /* RO: 芯片ID, 应读回 0x1E */
#define BMI088_ACC_ERR_REG         0x02u   /* RO: 错误寄存器 */
#define BMI088_ACC_STATUS          0x03u   /* RO: 状态 */
#define BMI088_ACC_X_LSB           0x12u   /* RO: X 轴低字节 */
#define BMI088_ACC_X_MSB           0x13u   /* RO: X 轴高字节 */
#define BMI088_ACC_Y_LSB           0x14u   /* RO: Y 轴低字节 */
#define BMI088_ACC_Y_MSB           0x15u   /* RO: Y 轴高字节 */
#define BMI088_ACC_Z_LSB           0x16u   /* RO: Z 轴低字节 */
#define BMI088_ACC_Z_MSB           0x17u   /* RO: Z 轴高字节 */
#define BMI088_ACC_TEMP_MSB        0x22u   /* RO: 温度高字节 */
#define BMI088_ACC_TEMP_LSB        0x23u   /* RO: 温度低字节 */
#define BMI088_ACC_CONF            0x40u   /* RW: 加速度计配置 */
#define BMI088_ACC_RANGE           0x41u   /* RW: 量程 */
#define BMI088_ACC_PWR_CONF        0x7Cu   /* RW: 功耗模式 */
#define BMI088_ACC_PWR_CTRL        0x7Du   /* RW: 电源控制 */
#define BMI088_ACC_SOFTRESET       0x7Eu   /* WO: 软复位 */

/* GYRO (陀螺仪) 寄存器 */
#define BMI088_GYRO_CHIP_ID        0x00u   /* RO: 芯片ID, 应读回 0x0F */
#define BMI088_GYRO_RATE_X_LSB     0x02u   /* RO: X 轴低字节 */
#define BMI088_GYRO_RATE_X_MSB     0x03u   /* RO: X 轴高字节 */
#define BMI088_GYRO_RATE_Y_LSB     0x04u   /* RO: Y 轴低字节 */
#define BMI088_GYRO_RATE_Y_MSB     0x05u   /* RO: Y 轴高字节 */
#define BMI088_GYRO_RATE_Z_LSB     0x06u   /* RO: Z 轴低字节 */
#define BMI088_GYRO_RATE_Z_MSB     0x07u   /* RO: Z 轴高字节 */
#define BMI088_GYRO_RANGE          0x0Fu   /* RW: 量程 */
#define BMI088_GYRO_BANDWIDTH      0x10u   /* RW: 带宽 */
#define BMI088_GYRO_LPM1           0x11u   /* RW: 低功耗模式1 */
#define BMI088_GYRO_SOFTRESET      0x14u   /* WO: 软复位 */
#define BMI088_GYRO_INT_CTRL       0x15u   /* RW: 中断控制 */

/* ─── 芯片 ID ─── */
#define BMI088_ACC_CHIP_ID_VAL     0x1Eu
#define BMI088_GYRO_CHIP_ID_VAL    0x0Fu

/* ─── 敏感度常量 ─── */
#define BMI088_ACC_3G_SENS         10920.0f   /* LSB/g  @ ±3G */
#define BMI088_ACC_6G_SENS          5460.0f   /* LSB/g  @ ±6G */
#define BMI088_ACC_12G_SENS         2730.0f   /* LSB/g  @ ±12G */
#define BMI088_ACC_24G_SENS         1365.0f   /* LSB/g  @ ±24G */
#define BMI088_GYRO_2000_SENS       16.384f   /* LSB/(dps) @ ±2000dps */
#define BMI088_GYRO_1000_SENS       32.768f   /* LSB/(dps) @ ±1000dps */
#define BMI088_GYRO_500_SENS        65.536f   /* LSB/(dps) @ ±500dps */
#define BMI088_GYRO_250_SENS       131.072f   /* LSB/(dps) @ ±250dps */
#define BMI088_GYRO_125_SENS       262.144f   /* LSB/(dps) @ ±125dps */

#define BMI088_GRAVITY              9.80665f  /* m/s^2 */

/* ─── 枚举 ─── */

typedef enum {
    BMI088_ACCEL_RANGE_3G  = 0,
    BMI088_ACCEL_RANGE_6G,
    BMI088_ACCEL_RANGE_12G,
    BMI088_ACCEL_RANGE_24G,
} imu_bmi088_accel_range_t;

typedef enum {
    BMI088_GYRO_RANGE_2000 = 0,
    BMI088_GYRO_RANGE_1000,
    BMI088_GYRO_RANGE_500,
    BMI088_GYRO_RANGE_250,
    BMI088_GYRO_RANGE_125,
} imu_bmi088_gyro_range_t;

/* ─── 数据类型 ─── */

typedef struct {
    float gyro[3];     /* rad/s (x, y, z) */
    float accel[3];    /* m/s^2  (x, y, z) */
    float temp;        /* 摄氏度 */
    uint8_t status;    /* 0=OK */
} imu_bmi088_data_t;

typedef enum {
    IMU_BMI088_DIAG_NONE = 0,
    IMU_BMI088_DIAG_ACC_INITIAL_ID,
    IMU_BMI088_DIAG_ACC_SOFTRESET,
    IMU_BMI088_DIAG_ACC_CHIP_ID,
    IMU_BMI088_DIAG_ACC_CFG,
    IMU_BMI088_DIAG_GYRO_INITIAL_ID,
    IMU_BMI088_DIAG_GYRO_SOFTRESET,
    IMU_BMI088_DIAG_GYRO_CHIP_ID,
    IMU_BMI088_DIAG_GYRO_CFG,
    IMU_BMI088_DIAG_READY,
    IMU_BMI088_DIAG_READ,
} imu_bmi088_diag_stage_t;

typedef struct {
    imu_bmi088_diag_stage_t stage;
    app_err_t err;
    app_err_t io_err;
    uint8_t reg;
    uint8_t val;
    uint8_t acc_initial_id;
    uint8_t acc_chip_id;
    uint8_t gyro_initial_id;
    uint8_t gyro_chip_id;
} imu_bmi088_diag_t;

/* ─── API ─── */

app_err_t imu_bmi088_init(void);
app_err_t imu_bmi088_read(imu_bmi088_data_t* data);
app_err_t imu_bmi088_set_accel_range(imu_bmi088_accel_range_t range);
app_err_t imu_bmi088_set_gyro_range(imu_bmi088_gyro_range_t range);
uint8_t   imu_bmi088_is_ready(void);
void      imu_bmi088_get_diag(imu_bmi088_diag_t* out);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_IMU_BMI088_H_ */
