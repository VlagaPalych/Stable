#ifndef MPU9250_REGS_H
#define MPU9250_REGS_H

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

#define AK8963_I2C_ADDR 0x0c

#endif // MPU9250_REGS_H
