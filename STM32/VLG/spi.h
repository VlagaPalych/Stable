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

extern "C" void mpu_nss_init(void);
extern "C" void mpu_nss_low(void);
extern "C" void mpu_nss_high(void);

extern "C" void mpu_write_byte(uint8_t address, uint8_t data);
extern "C" uint8_t mpu_read_byte(uint8_t address);
extern "C" void mpu_write(uint8_t address, uint8_t *data, uint16_t size);
extern "C" void mpu_read(uint8_t address, uint8_t *data, uint16_t size);

extern "C" void spi2_init(void);

#endif // SPI_H
