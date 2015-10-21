#ifndef ADXRS290_H
#define ADXRS290_H

#include "stdint.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

#define ADXRS290_DATA_SIZE 3

extern uint8_t SPI2_Busy;

typedef enum { NOBODY, ACCEL, ARS1, ARS2 } usingSPI2;
extern usingSPI2 curUsingSPI2;

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

extern int16_t ars1_data[ADXRS290_DATA_SIZE];

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
void ARS1_Calibr(void);

extern int16_t ars2_data[ADXRS290_DATA_SIZE];

#endif
