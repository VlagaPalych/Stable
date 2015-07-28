#ifndef GYRO_H
#define GYRO_H

#include "stdint.h"

extern uint8_t vals_index;

void I2C1_Init(void);
void GYRO_Read(uint8_t addr);

#endif
