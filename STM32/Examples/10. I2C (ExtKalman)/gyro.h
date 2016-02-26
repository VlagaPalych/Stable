#ifndef GYRO_H
#define GYRO_H

#include "stdint.h"

void SPI1_Init(void);
void Gyro_NSS_High(void);
void Gyro_NSS_Low(void); 
uint16_t SPI1_Transfer(uint16_t data);
uint8_t Gyro_Read(uint8_t address);
void Gyro_Write(uint8_t address, uint8_t data);
void Gyro_Init(void);
void Gyro_EXTI_Init(void);
void Gyro_DMA_Init(void);

#endif
