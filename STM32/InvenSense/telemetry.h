#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
    float euler[3];
} Message;

extern Message message;

void USART1_Init(void);
void Telemetry_Send(Message *msg);
void Telemetry_DMA_Init(void);

#endif
