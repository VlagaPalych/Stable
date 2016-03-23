#ifndef SPI_H
#define SPI_H

#include "stdint.h"

#define SPI2_SCK    ((uint8_t)13)   // PB
#define SPI2_MISO   ((uint8_t)14)   // PB
#define SPI2_MOSI   ((uint8_t)15)   // PB
#define MPU_NSS     ((uint8_t)12)   // PB
#define IMU_INT     ((uint8_t)1)    // PA

#define READ_COMMAND            0x80
#define WRITE_COMMAND           0x00

void MPU_NSS_Init(void);
void MPU_NSS_Low(void);
void MPU_NSS_High(void);

void MPU_WriteByte(uint8_t address, uint8_t data);
uint8_t MPU_ReadByte(uint8_t address);
void MPU_Write(uint8_t address, uint8_t *data, uint8_t size);
void MPU_Read(uint8_t address, uint8_t *data, uint8_t size);

void SPI2_Init(void);

#endif // SPI_H
