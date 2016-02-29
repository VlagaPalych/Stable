#ifndef ACCEL_MAGNE_H
#define ACCEL_MAGNE_H

#include "stdint.h"

typedef enum {AM_ACCEL, AM_MAG} device;

void I2C1_Init(void);
uint8_t AM_SingleRead(device dvc, uint8_t address);
void AM_Init(void);
void AM_EXTI_Init(void);
void AM_DMA_Init(void);
void AM_DMA_Run(uint8_t *data, uint8_t size);

#endif
