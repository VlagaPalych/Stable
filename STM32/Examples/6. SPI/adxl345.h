#ifndef ADXL345_h
#define ADXL345_h

#define READ_COMMAND                0x80
#define WRITE_COMMAND               0x00

#define DEVID_ADDRESS               0x00
#define DEVID                       0xe5

#define INT_ENABLE_ADDRESS          0x2e
#define DATA_READY_INT              0x80

#define INT_MAPPING_ADDRESS         0x2f
#define DATA_READY_INT0_MAPPING     0x7f

#define POWER_CTL_ADDRESS           0x2d
#define MEASUREMENT_MODE            0x08

#define DATA_FORMAT_ADDRESS         0x31
#define FULL_RES_MODE               0x08

#define BW_RATE_ADDRESS             0x2c
#define HZ100                       0x0a
#define HZ800                       0x0d
#define HZ1600                      0x0e

void ADXL345_Init(void);
void ADXL345_Calibr(void);
void EXTI_Init(void);

#endif
