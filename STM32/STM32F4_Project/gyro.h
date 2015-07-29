#ifndef GYRO_H
#define GYRO_H

#include "stdint.h"

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
