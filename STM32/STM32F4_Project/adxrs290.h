#ifndef ADXRS290_H
#define ADXRS290_H

#include "stdint.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

typedef enum { NOBODY, ACCEL, ARS1, ARS2 } usingSPI2;
extern usingSPI2 curUsingSPI2;

void SPI2_SensorsPoll(void);

//-------------------------------------------------------------------
// ARS1
//-------------------------------------------------------------------

void ARS1_Init(void);

void ARS1_NSS_Low(void);
void ARS1_NSS_High(void);

//-------------------------------------------------------------------
// ARS2
//-------------------------------------------------------------------

extern uint8_t ARS2_EXTI; // PA;
extern uint8_t ars2_dma_status;

void ARS2_Init(void);
void ARS2_EXTI_Init(void);
void ARS2_DMA_Init(void);
void ARS2_GetData(void);

void ARS2_NSS_Low(void);
void ARS2_NSS_High(void);
#endif
