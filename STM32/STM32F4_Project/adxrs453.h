#ifndef ADXRS453
#define ADXRS453

#include "stdint.h"

extern float adxrs_Sum;
extern uint16_t adxrs_CalibrIndex;
extern uint16_t adxrs_CalibrNumber;
extern uint8_t adxrs_CalibrationOn;
extern float adxrs_Offset;

extern int16_t adxrs_data;

extern uint16_t adxrsResponses[2];

void SPI3_Init();

uint16_t SPI3_Transfer(uint16_t data);
uint16_t ADXRS453_Read(uint8_t address);
void ADXRS_DMA_Read(void);
void ADXRS_TIM_Init(void);
void ADXRS_Calibr(void);

#endif