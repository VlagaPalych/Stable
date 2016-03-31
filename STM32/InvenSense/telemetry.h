#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"
#include "commands.h"

extern Message message;

void USART1_Init(void);
void Telemetry_Send();
void Telemetry_DMA_Init(void);

#endif
