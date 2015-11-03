#ifndef ADXRS290_H
#define ADXRS290_H

#include "stdint.h"
#include "processing.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

#define ADXRS290_DATA_SIZE 3
#define ADXRS290_FILTER_SIZE 416

extern uint8_t SPI2_Busy;

typedef enum {NONE, ACCEL_USING, ARS1_USING, ARS2_USING} SPI2_Using;

extern SPI2_Using SPI2_curUsing;

extern float adxrs290_filterCfs[ADXRS290_FILTER_SIZE];

void All_NSS_High(void);
void SPI2_SensorsPoll(void);

//-------------------------------------------------------------------
// ARS1
//-------------------------------------------------------------------

extern uint8_t ARS1_EXTI;

void ARS1_VDD_Init(void); 
void ARS1_NSS_Init(void);
void ARS1_NSS_Low(void);
void ARS1_NSS_High(void);

void ARS1_Init(void);

void ARS1_EXTI_Init(void);
//void ARS1_DMA_Init(void);
void ARS1_GetData(void);
void ARS1_Calibr(void);

extern uint8_t ars1_spiIndex;
extern uint8_t ars1_rawData[ADXRS290_DATA_SIZE*2];
extern int16_t ars1_data[ADXRS290_DATA_SIZE];
extern float ars1_termoData[ADXRS290_DATA_SIZE-1];
extern float ars1_filteredData[ADXRS290_DATA_SIZE-1];
extern float ars1_angleRate[ADXRS290_DATA_SIZE-1];

extern int16_t ars1_history[ADXRS290_DATA_SIZE-1][HISTORY_SIZE];
extern uint16_t ars1_historyIndex;
extern uint16_t ars1_curHistoryIndex;
extern uint8_t ars1_processIndex;
extern uint8_t ars1_processNumber;
extern uint8_t ars1_lowpassReady;
extern uint8_t ars1_doProcess;

extern uint8_t ars1_calibrationOn;
extern uint32_t ars1_calibrIndex;
extern uint32_t ars1_calibrNumber;
extern float ars1_offset[ADXRS290_DATA_SIZE-1];
extern float ars1_sum[ADXRS290_DATA_SIZE-1];

extern float ars1_termoCf[ADXRS290_DATA_SIZE-1];

//-------------------------------------------------------------------
// ARS2
//-------------------------------------------------------------------

extern uint8_t ARS2_EXTI; // PE;

void ARS2_VDD_Init(void); 
void ARS2_NSS_Init(void);
void ARS2_NSS_Low(void);
void ARS2_NSS_High(void);

void ARS2_Init(void);

void ARS2_EXTI_Init(void);
void ARS2_DMA_Init(void);
void ARS2_GetData(void);
void ARS2_Calibr(void);

extern uint8_t ars2_spiIndex;
extern uint8_t ars2_rawData[ADXRS290_DATA_SIZE*2];
extern int16_t ars2_data[ADXRS290_DATA_SIZE];
extern float ars2_termoData[ADXRS290_DATA_SIZE-1];
extern float ars2_filteredData[ADXRS290_DATA_SIZE-1];
extern float ars2_angleRate[ADXRS290_DATA_SIZE-1];

extern int16_t ars2_history[ADXRS290_DATA_SIZE-1][HISTORY_SIZE];
extern uint16_t ars2_historyIndex;
extern uint16_t ars2_curHistoryIndex;
extern uint8_t ars2_processIndex;
extern uint8_t ars2_processNumber;
extern uint8_t ars2_lowpassReady;
extern uint8_t ars2_doProcess;

extern uint8_t ars2_calibrationOn;
extern uint32_t ars2_calibrIndex;
extern uint32_t ars2_calibrNumber;
extern float ars2_offset[ADXRS290_DATA_SIZE-1];
extern float ars2_sum[ADXRS290_DATA_SIZE-1];

extern float ars2_termoCf[ADXRS290_DATA_SIZE-1];

#endif
