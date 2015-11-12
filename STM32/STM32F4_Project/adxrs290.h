#ifndef ADXRS290_H
#define ADXRS290_H

#include "stdint.h"
#include "processing.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

#define ADXRS290_NUMBER 2
#define ADXRS290_DATA_SIZE 3
#define ADXRS290_FILTER_SIZE 116


#define SPI2_ACCEL_USING  ADXRS290_NUMBER
#define SPI2_NOONE_USING (ADXRS290_NUMBER + 1)
extern uint8_t SPI2_curUsing;

extern float adxrs290_filterCfs[ADXRS290_FILTER_SIZE];

extern uint8_t ars_spiIndex[ADXRS290_NUMBER];
extern uint8_t ars_rawData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE*2];
extern int16_t ars_data[ADXRS290_NUMBER][ADXRS290_DATA_SIZE]; 
extern float ars_termoData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];
extern float ars_filteredData[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];
extern float ars_angleRate[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];

extern uint8_t ars_calibrationOn[ADXRS290_NUMBER];
extern uint32_t ars_calibrIndex[ADXRS290_NUMBER];
extern uint32_t ars_calibrNumber[ADXRS290_NUMBER];
extern float ars_offset[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];
extern float ars_sum[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1];

extern int16_t ars_history[ADXRS290_NUMBER][ADXRS290_DATA_SIZE-1][HISTORY_SIZE];
extern uint16_t ars_historyIndex[ADXRS290_NUMBER];
extern uint16_t ars_curHistoryIndex[ADXRS290_NUMBER];
extern uint8_t ars_processIndex[ADXRS290_NUMBER];
extern uint8_t ars_processNumber[ADXRS290_NUMBER];
extern uint8_t ars_lowpassReady[ADXRS290_NUMBER];
extern uint8_t ars_doProcess[ADXRS290_NUMBER];

void All_NSS_High(void);
void SPI2_SensorsPoll(void);

void ADXRS290_NSS_Init(uint8_t i);
void ADXRS290_NSS_Low(uint8_t i);
void ADXRS290_NSS_High(uint8_t i);
void ADXRS290_EXTI_Init(uint8_t i);
void ADXRS290_Init(uint8_t i);
void ADXRS290_Calibr(uint8_t i);

#endif