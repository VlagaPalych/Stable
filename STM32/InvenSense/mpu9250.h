#ifndef MPU9250_H
#define MPU9250_H

#include "stdint.h"

void SPI2_Init();
uint16_t SPI2_Transfer(uint16_t byte);
void IMU_NSS_Low();
void IMU_NSS_High();
void IMU_NSS_Init();
uint8_t SPI2_Read(uint8_t address); 
void IMU_Init();
void IMU_EXTI_Init();
void Mag_Init();
void IMU_DMA_Init();
void IMU_DMA_Run(uint16_t *tx, uint16_t *rx, uint8_t size);

#endif