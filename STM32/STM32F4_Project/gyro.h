#ifndef GYRO_H
#define GYRO_H

#include "stdint.h"

#define GYRO_HZ100  0x09
#define GYRO_HZ250  0x03
#define GYRO_HZ500  0x01
#define GYRO_HZ1000 0x00

extern uint8_t gyroFreshFreq;
extern uint8_t gyroCurFreq;
extern float gyroCurDT;

extern uint8_t gyroCalibrationOn;
extern uint32_t calibrIndex;
extern uint32_t calibrNumber;
extern float xSum, ySum, zSum;

extern float gyro_xOffset;
extern float gyro_yOffset;
extern float gyro_zOffset;

extern float gyroAngle;

extern int16_t gx, gy, gz;

extern uint8_t vals_index;

void I2C1_Init(void);
void Gyro_MultipleBytesRead(void);

uint8_t Gyro_SingleByteRead(uint8_t gyroAddress);
void Gyro_SingleByteWrite(uint8_t gyroAddress, uint8_t data);
void Gyro_ReadWithInterrupt(void);

void Gyro_Init(void);
void Gyro_Calibr(void);
void Gyro_EXTI_Init(void);
void Gyro_DMA_Init(void);

#endif
