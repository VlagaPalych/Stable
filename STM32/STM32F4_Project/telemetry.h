#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

extern uint8_t curFreq;
extern uint8_t freshFreq;

void USART_Init(void);
void Telemetry_TIM_Init(void);
void SendTelemetry(void);

#endif
