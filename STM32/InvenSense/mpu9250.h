#ifndef MPU9250_H
#define MPU9250_H

#include "stdint.h"

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
