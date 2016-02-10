#ifndef ADXRS290_H
#define ADXRS290_H

#include "stdint.h"
#include "processing.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

#define ADXRS290_NUMBER 2
#define ADXRS290_DATA_SIZE 3

#define SPI2_ACCEL_USING  ADXRS290_NUMBER
#define SPI2_NOONE_USING (ADXRS290_NUMBER + 1)
extern uint8_t SPI2_curUsing;

extern uint8_t ars_spiIndex[ADXRS290_NUMBER];
extern uint8_t ars_rawData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE*2];
extern int16_t ars_data[ADXRS290_NUMBER][ADXRS290_DATA_SIZE]; 
extern float ars_termoData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];
extern float ars_filteredData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];
extern float ars_angleRate[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];

#define ADXRS290_CALIBR_SAMPLES 3000
extern uint8_t adxrs290_calibr_on;
extern uint32_t adxrs290_calibr_index;
extern uint32_t adxrs290_calibr_number;
extern float adxrs290_offset[2];
extern float adxrs290_sum[2];
extern float calibrated_ar[2];

void All_NSS_High(void);
void SPI2_SensorsPoll(void);

void ADXRS290_NSS_Init(uint8_t i);
void ADXRS290_NSS_Low(uint8_t i);
void ADXRS290_NSS_High(uint8_t i);
void ADXRS290_EXTI_Init(uint8_t i);
void ADXRS290_Init(uint8_t i);
void ADXRS290_Calibr();

#endif
