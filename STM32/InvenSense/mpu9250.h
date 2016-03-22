#ifndef MPU9250_H
#define MPU9250_H

#include "stdint.h"

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


void SPI2_Init(void);
uint8_t SPI2_Transfer(uint8_t byte);
void IMU_NSS_Low(void);
void IMU_NSS_High(void);
void IMU_NSS_Init(void);
uint8_t SPI2_Read(uint8_t address); 
void IMU_Init(void);
void IMU_EXTI_Init(void);
void Mag_Init(void);
void IMU_DMA_Init(void);
void IMU_DMA_Run(uint8_t *tx, uint8_t *rx, uint8_t size);
void IMU_Write(uint8_t address, uint8_t *data, uint8_t size);
void IMU_Read(uint8_t address, uint8_t *data, uint8_t size);
void MPU_LoadFirmware(uint8_t *firmware, uint16_t length, uint16_t start_addr);

void MPU_MemWrite(uint16_t addr, uint8_t *data, uint16_t size);
void MPU_MemRead(uint16_t addr, uint8_t *data, uint16_t size);

#define DMP_CODE_SIZE           3062
extern uint8_t dmp_memory[DMP_CODE_SIZE];
extern uint16_t startAddress;

#endif
