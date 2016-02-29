#ifndef ACCEL_MAGNE_H
#define ACCEL_MAGNE_H

#include "stdint.h"

void I2C1_Init(void);
uint8_t AM_SingleRead(uint8_t address);
void AM_Init(void);

#endif
