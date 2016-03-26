#ifndef MPU9250_H
#define MPU9250_H

#include "stdint.h"

#define MPU_MAX_DEVICES 1

// MPU9250 Accelerometer and gyroscope registers
#define SELF_TEST_X_GYRO            0x00
#define SELF_TEST_Y_GYRO            0x01
#define SELF_TEST_Z_GYRO            0x02
#define SELF_TEST_X_ACCEL           0x0d
#define SELF_TEST_Y_ACCEL           0x0e
#define SELF_TEST_Z_ACCEL           0x0f
#define XG_OFFSET_H                 0x13
#define XG_OFFSET_L                 0x14
#define YG_OFFSET_H                 0x15
#define YG_OFFSET_L                 0x16
#define ZG_OFFSET_H                 0x17
#define ZG_OFFSET_L                 0x18
#define SMPLRT_DIV                  0x19
#define CONFIG                      0x1a
#define GYRO_CONFIG                 0x1b
#define ACCEL_CONFIG                0x1c
#define ACCEL_CONFIG2               0x1d
#define LP_ACCEL_ODR                0x1e
#define WOM_THR                     0x1f
#define FIFO_EN                     0x23
#define I2C_MST_CTRL                0x24
#define I2C_SLV0_ADDR               0x25
#define I2C_SLV0_REG                0x26
#define I2C_SLV0_CTRL               0x27
#define I2C_SLV1_ADDR               0x28
#define I2C_SLV1_REG                0x29
#define I2C_SLV1_CTRL               0x2a
#define I2C_SLV2_ADDR               0x2b
#define I2C_SLV2_REG                0x2c
#define I2C_SLV2_CTRL               0x2d
#define I2C_SLV3_ADDR               0x2e
#define I2C_SLV3_REG                0x2f
#define I2C_SLV3_CTRL               0x30
#define I2C_SLV4_ADDR               0x31
#define I2C_SLV4_REG                0x32
#define I2C_SLV4_DO                 0x33
#define I2C_SLV4_CTRL               0x34
#define I2C_SLV4_DI                 0x35
#define I2C_MST_STATUS              0x36
#define INT_PIN_CFG                 0x37
#define INT_ENABLE                  0x38
#define INT_STATUS                  0x3a
#define ACCEL_XOUT_H                0x3b
#define ACCEL_XOUT_L                0x3c
#define ACCEL_YOUT_H                0x3d
#define ACCEL_YOUT_L                0x3e
#define ACCEL_ZOUT_H                0x3f
#define ACCEL_ZOUT_L                0x40
#define TEMP_OUT_H                  0x41
#define TEMP_OUT_L                  0x42
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0x45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48
#define EXT_SENS_DATA_00            0x49
#define EXT_SENS_DATA_01            0x4a
#define EXT_SENS_DATA_02            0x4b
#define EXT_SENS_DATA_03            0x4c
#define EXT_SENS_DATA_04            0x4d
#define EXT_SENS_DATA_05            0x4e
#define EXT_SENS_DATA_06            0x4f
#define EXT_SENS_DATA_07            0x50
#define EXT_SENS_DATA_08            0x51
#define EXT_SENS_DATA_09            0x52
#define EXT_SENS_DATA_10            0x53
#define EXT_SENS_DATA_11            0x54
#define EXT_SENS_DATA_12            0x55
#define EXT_SENS_DATA_13            0x56
#define EXT_SENS_DATA_14            0x57
#define EXT_SENS_DATA_15            0x58
#define EXT_SENS_DATA_16            0x59
#define EXT_SENS_DATA_17            0x5a
#define EXT_SENS_DATA_18            0x5b
#define EXT_SENS_DATA_19            0x5c
#define EXT_SENS_DATA_20            0x5d
#define EXT_SENS_DATA_21            0x5e
#define EXT_SENS_DATA_22            0x5f
#define EXT_SENS_DATA_23            0x60
#define I2C_SLV0_DO                 0x63
#define I2C_SLV1_DO                 0x64
#define I2C_SLV2_DO                 0x65
#define I2C_SLV3_DO                 0x66
#define I2C_MST_DELAY_CTRL          0x67
#define SIGNAL_PATH_RESET           0x68
#define MOT_DETECT_CTRL             0x69
#define USER_CTRL                   0x6a
#define PWR_MGMT_1                  0x6b
#define PWR_MGMT_2                  0x6c
#define FIFO_COUNTH                 0x72
#define FIFO_COUNTL                 0x73
#define FIFO_R_W                    0x74
#define WHO_AM_I                    0x75
#define XA_OFFSET_H                 0x77
#define XA_OFFSET_L                 0x78
#define YA_OFFSET_H                 0x7a
#define YA_OFFSET_L                 0x7b
#define ZA_OFFSET_H                 0x7d
#define ZA_OFFSET_L                 0x7e

// MPU9250 characteristics
#define GYRO_SENS                   131.0f          // LSB/dps
#define ACCEL_SENS                  2048.0f         // LSB/g
#define MAG_SENS                    0.15f           // uT/LSB
#define TEMP_SEN                    338.87f         // LSB/degC
#define TEMP_OFFSET                 21.0f           // degC
#define QUAT_SENS                   1073741824.f    // 2^30


// Magnetometer AK8963 registers
#define AK8963_I2C_ADDRESS      0x0c

#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
#define AK8963_CNTL                 0x0a
#define AK8963_ASTC                 0x0c
#define AK8963_I2CDIS               0x0f
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

//  IMU hardware device defines
#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)


#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

#define AK8963_SECONDARY

#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY
#else
/* #warning "No compass = less profit for Invensense. Lame." */
#endif

#if defined AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)
#elif defined AK8963_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#endif

#ifdef AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#endif

/* Hardware registers needed by driver. */
typedef struct {
    uint8_t who_am_i;
    uint8_t rate_div;
    uint8_t lpf;
    uint8_t prod_id;
    uint8_t user_ctrl;
    uint8_t fifo_en;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;
    uint8_t accel_cfg2;
    uint8_t lp_accel_odr;
    uint8_t motion_thr;
    uint8_t motion_dur;
    uint8_t fifo_count_h;
    uint8_t fifo_r_w;
    uint8_t raw_gyro;
    uint8_t raw_accel;
    uint8_t temp;
    uint8_t int_enable;
    uint8_t dmp_int_status;
    uint8_t int_status;
    uint8_t accel_intel;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    uint8_t int_pin_cfg;
    uint8_t mem_r_w;
    uint8_t accel_offs;
    uint8_t i2c_mst;
    uint8_t bank_sel;
    uint8_t mem_start_addr;
    uint8_t prgm_start_h;
#if defined AK89xx_SECONDARY
    uint8_t s0_addr;
    uint8_t s0_reg;
    uint8_t s0_ctrl;
    uint8_t s1_addr;
    uint8_t s1_reg;
    uint8_t s1_ctrl;
    uint8_t s4_ctrl;
    uint8_t s0_do;
    uint8_t s1_do;
    uint8_t i2c_delay_ctrl;
    uint8_t raw_compass;
    /* The I2C_MST_VDDIO bit is in this register. */
    uint8_t yg_offs_tc;
#endif
} MPU_Regs;

/* Information specific to a particular device. */
typedef struct {
    uint8_t addr;
    uint16_t max_fifo;
    uint8_t num_reg;
    uint16_t temp_sens;
    int16_t temp_offset;
    uint16_t bank_size;
#if defined AK89xx_SECONDARY
    uint16_t compass_fsr;
#endif
} HW;

typedef struct {
    uint16_t gyro_fsr;
    uint8_t accel_fsr;
    uint16_t lpf;
    uint16_t sample_rate;
    uint8_t sensors_on;
    uint8_t fifo_sensors;
    uint8_t dmp_on;
} MotionIntCache;

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
typedef struct {
    /* Matches gyro_cfg >> 3 & 0x03 */
    uint8_t gyro_fsr;
    float gyro_sens;
    /* Matches accel_cfg >> 3 & 0x03 */
    uint8_t accel_fsr;
    float accel_sens;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    uint8_t sensors;
    /* Matches config register. */
    uint8_t lpf;
    uint8_t clk_src;
    /* Sample rate, NOT rate divider. */
    uint16_t sample_rate;
    /* Matches fifo_en register. */
    uint8_t fifo_enable;
    /* Matches int enable register. */
    uint8_t int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    uint8_t bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    uint8_t accel_half;
    /* 1 if device in low-power accel-only mode. */
    uint8_t lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    uint8_t int_motion_only;
    MotionIntCache cache;
    /* 1 for active low interrupts. */
    uint8_t active_low_int;
    /* 1 for latched interrupts. */
    uint8_t latched_int;
    /* 1 if DMP is enabled. */
    uint8_t dmp_on;
    /* Ensures that DMP will only be loaded once. */
    uint8_t dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    uint16_t dmp_sample_rate;
    /* Compass sample rate. */
#ifdef AK89xx_SECONDARY
    uint16_t compass_sample_rate;
    uint8_t compass_addr;
    float mag_sens_adj[3];
#endif
} MPU_Config;

typedef struct {
    uint32_t gyro_sens;
    uint32_t accel_sens;
    uint8_t reg_rate_div;
    uint8_t reg_lpf;
    uint8_t reg_gyro_fsr;
    uint8_t reg_accel_fsr;
    uint16_t wait_ms;
    uint8_t packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
} MPU_Test;

typedef struct {
    const MPU_Regs *reg;
    const HW *hw;
    MPU_Config chip_cfg;
    MPU_Test *test;
} MPU_State;

typedef struct {
    void (*cb)(void);
    uint8_t pin;
    uint8_t lp_exit;
    uint8_t active_low;
} MPU_IntParams;


void MPU_EXTI_Init(void);
void Mag_Init(void);


int MPU_MemWrite(uint16_t mem_addr, uint8_t *data, uint16_t length);
int MPU_MemRead(uint16_t mem_addr, uint8_t *data, uint16_t length);

int MPU_GetGyroFsr(uint16_t *fsr);
int MPU_SetGyroFsr(uint16_t fsr);
int MPU_GetAccelFsr(uint8_t *fsr);
int MPU_SetAccelFsr(uint16_t fsr);
int MPU_GetLPF(uint16_t *lpf);
int MPU_SetLPF(uint16_t lpf);


int MPU_SetIntLatched(uint8_t enable);
int MPU_LowPowerAccelMode(uint8_t rate);
int MPU_SetSampleRate(uint16_t rate);
int MPU_ResetFIFO(void); 
int MPU_ConfigureFIFO(uint8_t sensors);
int MPU_SetCompassSampleRate(uint8_t rate);
int MPU_SetSensors(uint8_t sensors);
int MPU_SetBypass(uint8_t bypass_on);
int MPU_SetIntEnable(uint8_t enable);

int MPU_SelectDevice(int device);
void MPU_InitStructures(void);
int MPU_Init(MPU_IntParams *int_param);

int MPU_LoadFirmware(uint16_t length, const uint8_t *firmware,
    uint16_t start_addr, uint16_t sample_rate);
int MPU_SetDMPState(uint8_t enable);

void Compass_WriteByte(uint8_t reg, uint8_t data);

int MPU_RunSelfTest(long *gyro, long *accel);

#endif
