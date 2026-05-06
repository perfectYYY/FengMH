/*
 * imu_bmi088.c — BMI088 IMU 设备驱动实现
 *
 * SPI 通信: 模式0 (CPOL=0, CPHA=0), 8bit, MSB
 * 总线: SPI2, CS: PC0(ACC), PC3(GYRO)
 *
 * 移植自 Wheel-legged BMI088 驱动, 适配 FengMH 代码风格:
 *   - bsp_spi 抽象层替代 HAL 直调
 *   - app_err_t 错误码
 *   - LOGE/LOGI 日志
 *   - APP_TARGET_HOST mock 支持
 */
#include "imu_bmi088.h"
#include "bsp_spi.h"
#include "bsp_time.h"
#include "config.h"
#include "log.h"

#include <string.h>
#include <math.h>

static const char* TAG = "BMI088";

/* ─── 内部常量 ─── */

#define BMI088_SPI_BUS    BSP_SPI_2
#define BMI088_CS_ACCEL   BSP_SPI_CS_BMI088_ACCEL
#define BMI088_CS_GYRO    BSP_SPI_CS_BMI088_GYRO
#define BMI088_SPI_TIMEOUT 10u  /* ms */

/* 软复位后等待时间 (ms) */
#define BMI088_STARTUP_DELAY_MS 80u
#define BMI088_COM_WAIT_US      150u
#define BMI088_TEMP_FACTOR      0.125f
#define BMI088_TEMP_OFFSET      23.0f
#define BMI088_DUMMY_BYTE       0x55u
#define BMI088_MAX_XFER_LEN     16u

/* ACC 写寄存器: bit7=0 表示写 */
#define BMI088_ACC_WRITE(reg)  ((reg) & 0x7Fu)
/* ACC 读寄存器: bit7=1 表示读 */
#define BMI088_ACC_READ(reg)   ((reg) | 0x80u)

/* GYRO 写寄存器: bit7=0 表示写 */
#define BMI088_GYRO_WRITE(reg) ((reg) & 0x7Fu)
/* GYRO 读寄存器: bit7=1 表示读 */
#define BMI088_GYRO_READ(reg)  ((reg) | 0x80u)

/* ─── 寄存器配置表 ─── */

/* 初始化的寄存器写-验证表。每项: {reg, val, mask, desc} */
typedef struct {
    uint8_t reg;
    uint8_t val;
    uint8_t mask;  /* 回读比对时只比较 mask 位 */
    const char* desc;
} bmi088_reg_cfg_t;

/* ACC 初始化配置，寄存器组合与 Wheel-legged 保持一致。 */
static const bmi088_reg_cfg_t S_ACC_CFG[] = {
    { BMI088_ACC_PWR_CTRL,     0x04u, 0xFFu, "pwr_ctrl"     },  /* enable accel */
    { BMI088_ACC_PWR_CONF,     0x00u, 0xFFu, "pwr_conf"     },  /* active mode */
    { BMI088_ACC_CONF,          0xABu, 0xFFu, "acc_conf"     },  /* ODR 800Hz (0xB), BW normal, bit7=1 */
    { BMI088_ACC_RANGE,         0x00u, 0x03u, "acc_range"    },  /* ±3G */
    { 0x53u,                    0x08u, 0xFFu, "int1_io"      },  /* INT1 push-pull low, enabled */
    { 0x58u,                    0x04u, 0xFFu, "int_map"      },  /* ACC DRDY -> INT1 */
};

/* GYRO 初始化配置，寄存器组合与 Wheel-legged 保持一致。 */
static const bmi088_reg_cfg_t S_GYRO_CFG[] = {
    { BMI088_GYRO_RANGE,        0x00u, 0x07u, "gyro_range"   },  /* ±2000dps */
    { BMI088_GYRO_BANDWIDTH,    0x82u, 0xFFu, "gyro_bw"      },  /* ODR 1000Hz, BW 116Hz */
    { BMI088_GYRO_LPM1,         0x00u, 0xFFu, "gyro_lpm1"    },  /* 正常模式, 非 suspend */
    { BMI088_GYRO_INT_CTRL,     0x80u, 0xFFu, "gyro_ctrl"    },  /* DRDY on */
    { 0x16u,                    0x00u, 0xFFu, "gyro_int_io"  },  /* INT3 push-pull low */
    { 0x18u,                    0x01u, 0xFFu, "gyro_int_map" },  /* GYRO DRDY -> INT3 */
};

/* ─── 静态状态 ─── */

static imu_bmi088_accel_range_t s_accel_range = BMI088_ACCEL_RANGE_3G;
static imu_bmi088_gyro_range_t  s_gyro_range  = BMI088_GYRO_RANGE_2000;
static float s_accel_scale = BMI088_ACC_3G_SENS;    /* LSB/g */
static float s_gyro_scale  = BMI088_GYRO_2000_SENS; /* LSB/(dps) */
static bool s_initialized = false;

static int16_t make_i16(uint8_t lsb, uint8_t msb) {
    return (int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

/* ─── SPI 底层辅助 ─── */

/* ACC 单寄存器写 */
static app_err_t acc_write_reg(uint8_t reg, uint8_t val) {
    app_err_t err;
    uint8_t tx[2] = { BMI088_ACC_WRITE(reg), val };
    uint8_t rx[2] = {0};

    err = bsp_spi_set_cs(BMI088_CS_ACCEL, 1);  /* CS low */
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, 2U, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_ACCEL, 0);        /* CS high */
    return err;
}

/* ACC 单寄存器读 */
static app_err_t acc_read_reg(uint8_t reg, uint8_t* val) {
    app_err_t err;
    uint8_t tx[3] = { BMI088_ACC_READ(reg), BMI088_DUMMY_BYTE, BMI088_DUMMY_BYTE };
    uint8_t rx[3] = {0};

    if (!val) return APP_ERR_INVALID_ARG;

    err = bsp_spi_set_cs(BMI088_CS_ACCEL, 1);
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, 3U, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_ACCEL, 0);
    if (err != APP_OK) return err;
    *val = rx[2];
    return err;
}

/* ACC 多字节读 (连续读, 地址自增) */
static app_err_t acc_read_multi(uint8_t reg, uint8_t* buf, uint32_t len) {
    app_err_t err;
    uint8_t tx[BMI088_MAX_XFER_LEN];
    uint8_t rx[BMI088_MAX_XFER_LEN];
    uint32_t total;

    if (!buf || len == 0U) return APP_ERR_INVALID_ARG;
    total = len + 2U;
    if (total > BMI088_MAX_XFER_LEN) return APP_ERR_INVALID_ARG;
    memset(tx, BMI088_DUMMY_BYTE, total);
    memset(rx, 0, total);
    tx[0] = BMI088_ACC_READ(reg);

    err = bsp_spi_set_cs(BMI088_CS_ACCEL, 1);
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, total, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_ACCEL, 0);

    if (err != APP_OK) return err;
    memcpy(buf, &rx[2], len);
    return APP_OK;
}

/* GYRO 单寄存器写 */
static app_err_t gyro_write_reg(uint8_t reg, uint8_t val) {
    app_err_t err;
    uint8_t tx[2] = { BMI088_GYRO_WRITE(reg), val };
    uint8_t rx[2] = {0};

    err = bsp_spi_set_cs(BMI088_CS_GYRO, 1);
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, 2U, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_GYRO, 0);
    return err;
}

/* GYRO 单寄存器读 */
static app_err_t gyro_read_reg(uint8_t reg, uint8_t* val) {
    app_err_t err;
    uint8_t tx[2] = { BMI088_GYRO_READ(reg), 0xFFu };
    uint8_t rx[2] = {0};

    if (!val) return APP_ERR_INVALID_ARG;

    err = bsp_spi_set_cs(BMI088_CS_GYRO, 1);
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, 2U, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_GYRO, 0);
    *val = rx[1];
    return err;
}

/* GYRO 多字节读 */
static app_err_t gyro_read_multi(uint8_t reg, uint8_t* buf, uint32_t len) {
    app_err_t err;
    uint8_t tx[BMI088_MAX_XFER_LEN];
    uint8_t rx[BMI088_MAX_XFER_LEN];
    uint32_t total;

    if (!buf || len == 0U) return APP_ERR_INVALID_ARG;
    total = len + 1U;
    if (total > BMI088_MAX_XFER_LEN) return APP_ERR_INVALID_ARG;
    memset(tx, BMI088_DUMMY_BYTE, total);
    memset(rx, 0, total);
    tx[0] = BMI088_GYRO_READ(reg);

    err = bsp_spi_set_cs(BMI088_CS_GYRO, 1);
    if (err != APP_OK) return err;
    err = bsp_spi_transmit_receive(BMI088_SPI_BUS, tx, rx, total, BMI088_SPI_TIMEOUT);
    bsp_spi_set_cs(BMI088_CS_GYRO, 0);

    if (err != APP_OK) return err;
    memcpy(buf, &rx[1], len);
    return APP_OK;
}

/* ─── 初始化 ─── */

static app_err_t accel_init(void) {
    uint8_t id;
    app_err_t err;

    /* 1. 软复位 */
    err = acc_read_reg(BMI088_ACC_CHIP_ID, &id);
    if (err != APP_OK) {
        LOGE("acc initial chip_id read failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    err = acc_write_reg(BMI088_ACC_SOFTRESET, 0xB6u);
    if (err != APP_OK) {
        LOGE("acc softreset write failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_ms(BMI088_STARTUP_DELAY_MS);

    /* 2. CHIP_ID 校验 */
    err = acc_read_reg(BMI088_ACC_CHIP_ID, &id);
    if (err != APP_OK) {
        LOGE("acc chip_id read failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_us(BMI088_COM_WAIT_US);
    if (id != BMI088_ACC_CHIP_ID_VAL) {
        LOGE("acc bad chip_id: 0x%02X (exp 0x%02X)", id, BMI088_ACC_CHIP_ID_VAL);
        return APP_ERR_IO;
    }
    LOGI("acc chip_id: 0x%02X OK", id);

    /* 3. 写-验证寄存器配置 */
    for (size_t i = 0; i < ARRAY_SIZE(S_ACC_CFG); i++) {
        err = acc_write_reg(S_ACC_CFG[i].reg, S_ACC_CFG[i].val);
        if (err != APP_OK) {
            LOGE("acc write %s failed: %d", S_ACC_CFG[i].desc, (int)err);
            return err;
        }
        bsp_time_delay_us(BMI088_COM_WAIT_US);

        /* 回读验证 */
        uint8_t r;
        err = acc_read_reg(S_ACC_CFG[i].reg, &r);
        if (err != APP_OK) {
            LOGE("acc readback %s failed: %d", S_ACC_CFG[i].desc, (int)err);
            return err;
        }
        bsp_time_delay_us(BMI088_COM_WAIT_US);
        if ((r & S_ACC_CFG[i].mask) != (S_ACC_CFG[i].val & S_ACC_CFG[i].mask)) {
            LOGE("acc %s mismatch: w=0x%02X r=0x%02X mask=0x%02X",
                 S_ACC_CFG[i].desc, S_ACC_CFG[i].val, r, S_ACC_CFG[i].mask);
            return APP_ERR_IO;
        }
        LOGI("acc %s: 0x%02X OK", S_ACC_CFG[i].desc, r);
    }

    LOGI("acc init done");
    return APP_OK;
}

static app_err_t gyro_init(void) {
    uint8_t id;
    app_err_t err;

    /* 1. 软复位 */
    err = gyro_read_reg(BMI088_GYRO_CHIP_ID, &id);
    if (err != APP_OK) {
        LOGE("gyro initial chip_id read failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_us(BMI088_COM_WAIT_US);

    err = gyro_write_reg(BMI088_GYRO_SOFTRESET, 0xB6u);
    if (err != APP_OK) {
        LOGE("gyro softreset write failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_ms(BMI088_STARTUP_DELAY_MS);

    /* 2. CHIP_ID 校验 */
    err = gyro_read_reg(BMI088_GYRO_CHIP_ID, &id);
    if (err != APP_OK) {
        LOGE("gyro chip_id read failed: %d", (int)err);
        return APP_ERR_IO;
    }
    bsp_time_delay_us(BMI088_COM_WAIT_US);
    if (id != BMI088_GYRO_CHIP_ID_VAL) {
        LOGE("gyro bad chip_id: 0x%02X (exp 0x%02X)", id, BMI088_GYRO_CHIP_ID_VAL);
        return APP_ERR_IO;
    }
    LOGI("gyro chip_id: 0x%02X OK", id);

    /* 3. 写-验证寄存器配置 */
    for (size_t i = 0; i < ARRAY_SIZE(S_GYRO_CFG); i++) {
        err = gyro_write_reg(S_GYRO_CFG[i].reg, S_GYRO_CFG[i].val);
        if (err != APP_OK) {
            LOGE("gyro write %s failed: %d", S_GYRO_CFG[i].desc, (int)err);
            return err;
        }
        bsp_time_delay_us(BMI088_COM_WAIT_US);

        uint8_t r;
        err = gyro_read_reg(S_GYRO_CFG[i].reg, &r);
        if (err != APP_OK) {
            LOGE("gyro readback %s failed: %d", S_GYRO_CFG[i].desc, (int)err);
            return err;
        }
        bsp_time_delay_us(BMI088_COM_WAIT_US);
        if ((r & S_GYRO_CFG[i].mask) != (S_GYRO_CFG[i].val & S_GYRO_CFG[i].mask)) {
            LOGE("gyro %s mismatch: w=0x%02X r=0x%02X mask=0x%02X",
                 S_GYRO_CFG[i].desc, S_GYRO_CFG[i].val, r, S_GYRO_CFG[i].mask);
            return APP_ERR_IO;
        }
        LOGI("gyro %s: 0x%02X OK", S_GYRO_CFG[i].desc, r);
    }

    LOGI("gyro init done");
    return APP_OK;
}

static app_err_t read_temperature(float* temp) {
    uint8_t buf[2];
    int16_t raw;
    app_err_t err;

    if (!temp) return APP_ERR_INVALID_ARG;

    err = acc_read_multi(BMI088_ACC_TEMP_MSB, buf, 2U);
    if (err != APP_OK) return err;

    raw = (int16_t)(((uint16_t)buf[0] << 3) | ((uint16_t)buf[1] >> 5));
    if (raw > 1023) {
        raw -= 2048;
    }

    *temp = ((float)raw * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
    return APP_OK;
}

/* ─── 公共 API ─── */

app_err_t imu_bmi088_init(void) {
    app_err_t err;

#if APP_TARGET_HOST
    /* Host mock: 跳过硬件初始化 */
    s_initialized = true;
    LOGI("imu_bmi088 init (host mock)");
    return APP_OK;
#endif

    err = accel_init();
    if (err != APP_OK) {
        LOGE("accel_init failed: %d", (int)err);
        return err;
    }

    err = gyro_init();
    if (err != APP_OK) {
        LOGE("gyro_init failed: %d", (int)err);
        return err;
    }

    s_initialized = true;
    LOGI("imu_bmi088 init complete");
    return APP_OK;
}

app_err_t imu_bmi088_read(imu_bmi088_data_t* data) {
    if (!data) return APP_ERR_INVALID_ARG;
    if (!s_initialized) return APP_ERR_UNINIT;

#if APP_TARGET_HOST
    /* Host mock: 返回零值 */
    memset(data, 0, sizeof(*data));
    return APP_OK;
#endif

    uint8_t buf[8];
    app_err_t err;
    int16_t raw;

    /* ── 读加速度计 (6 bytes, 从 ACC_X_LSB 起) ── */
    err = acc_read_multi(BMI088_ACC_X_LSB, buf, 6U);
    if (err != APP_OK) {
        LOGE("acc read failed: %d", (int)err);
        data->status = 1;
        return err;
    }

    raw = make_i16(buf[0], buf[1]);
    data->accel[0] = ((float)raw / s_accel_scale) * BMI088_GRAVITY;

    raw = make_i16(buf[2], buf[3]);
    data->accel[1] = ((float)raw / s_accel_scale) * BMI088_GRAVITY;

    raw = make_i16(buf[4], buf[5]);
    data->accel[2] = ((float)raw / s_accel_scale) * BMI088_GRAVITY;

    /* ── 读陀螺仪 (6 bytes angular rate, 从 GYRO_RATE_X_LSB 起) ── */
    err = gyro_read_multi(BMI088_GYRO_RATE_X_LSB, buf, 6U);
    if (err != APP_OK) {
        LOGE("gyro read failed: %d", (int)err);
        data->status = 1;
        return err;
    }

    raw = make_i16(buf[0], buf[1]);
    data->gyro[0] = ((float)raw / s_gyro_scale) * ((float)M_PI / 180.0f);

    raw = make_i16(buf[2], buf[3]);
    data->gyro[1] = ((float)raw / s_gyro_scale) * ((float)M_PI / 180.0f);

    raw = make_i16(buf[4], buf[5]);
    data->gyro[2] = ((float)raw / s_gyro_scale) * ((float)M_PI / 180.0f);

    err = read_temperature(&data->temp);
    if (err != APP_OK) {
        LOGE("temp read failed: %d", (int)err);
        data->status = 2;
        return err;
    }

    data->status = 0;

    return APP_OK;
}

app_err_t imu_bmi088_set_accel_range(imu_bmi088_accel_range_t range) {
    uint8_t reg_val;
    float scale;

    switch (range) {
        case BMI088_ACCEL_RANGE_3G:  reg_val = 0x00u; scale = BMI088_ACC_3G_SENS;  break;
        case BMI088_ACCEL_RANGE_6G:  reg_val = 0x01u; scale = BMI088_ACC_6G_SENS;  break;
        case BMI088_ACCEL_RANGE_12G: reg_val = 0x02u; scale = BMI088_ACC_12G_SENS; break;
        case BMI088_ACCEL_RANGE_24G: reg_val = 0x03u; scale = BMI088_ACC_24G_SENS; break;
        default: return APP_ERR_INVALID_ARG;
    }

    if (!s_initialized) { s_accel_range = range; s_accel_scale = scale; return APP_OK; }

    app_err_t err = acc_write_reg(BMI088_ACC_RANGE, reg_val);
    if (err == APP_OK) {
        s_accel_range = range;
        s_accel_scale = scale;
    }
    return err;
}

uint8_t imu_bmi088_is_ready(void) {
    return s_initialized ? 1U : 0U;
}

app_err_t imu_bmi088_set_gyro_range(imu_bmi088_gyro_range_t range) {
    uint8_t reg_val;
    float scale;

    switch (range) {
        case BMI088_GYRO_RANGE_2000: reg_val = 0x00u; scale = BMI088_GYRO_2000_SENS; break;
        case BMI088_GYRO_RANGE_1000: reg_val = 0x01u; scale = BMI088_GYRO_1000_SENS; break;
        case BMI088_GYRO_RANGE_500:  reg_val = 0x02u; scale = BMI088_GYRO_500_SENS;  break;
        case BMI088_GYRO_RANGE_250:  reg_val = 0x03u; scale = BMI088_GYRO_250_SENS;  break;
        case BMI088_GYRO_RANGE_125:  reg_val = 0x04u; scale = BMI088_GYRO_125_SENS;  break;
        default: return APP_ERR_INVALID_ARG;
    }

    if (!s_initialized) { s_gyro_range = range; s_gyro_scale = scale; return APP_OK; }

    app_err_t err = gyro_write_reg(BMI088_GYRO_RANGE, reg_val);
    if (err == APP_OK) {
        s_gyro_range = range;
        s_gyro_scale = scale;
    }
    return err;
}
