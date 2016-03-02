#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
    float accel[3];
    float magField[3];
} Message;

void USART1_Init(void);
void send_to_uart(uint8_t data);
void Telemetry_Send(Message *msg);
void Telemetry_DMA_Init(void);

#endif
