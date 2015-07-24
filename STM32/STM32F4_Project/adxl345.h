#ifndef ADXL345_H
#define ADXL345_H

#include "stdint.h"

#define READ_COMMAND                0x80
#define WRITE_COMMAND               0x00

#define DEVID_ADDRESS               0x00
//#define DEVID                       0xe5

#define INT_ENABLE_ADDRESS          0x2e
#define DATA_READY_INT              0x80

#define INT_MAPPING_ADDRESS         0x2f
#define DATA_READY_INT0_MAPPING     0x80

#define POWER_CTL_ADDRESS           0x2d
#define MEASUREMENT_MODE            0x08

#define DATA_FORMAT_ADDRESS         0x31
#define FULL_RES_MODE               0x08

#define BW_RATE_ADDRESS             0x2c
#define HZ100                       0x0a
#define HZ800                       0x0d
#define HZ1600                      0x0e
#define ACCEL_FREQ                  HZ100

#define CALIBR_NUMBER               800

extern int16_t ax, ay, az;

void Delay(void);

void NSS_Low(void);
void NSS_High(void);

uint16_t SPI1_Transfer(uint16_t byte);
uint8_t ADXL345_Read(uint8_t address);
void ADXL345_Write(uint8_t address, uint8_t data);

void ADXL345_Init(void);
void SPI2_Init(void);
void SPI2_GPIO_Init(void);

void ADXL345_Calibr(void);
void ADXL345_GetAccel(int16_t *x, int16_t *y, int16_t *z);

void Accel_EXTI_Init(void);

void ADXL345_DMA_Init(void);

void EXTI1_IRQHandler(void);

#endif 
