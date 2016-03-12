#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
//    float w1[3];
//    float w2[3];
//    float accel[3];
//    float magField[3];
//    float angleRate[3];
//    float q[4];
    float angleRate[3];
    float euler[3];
    float eulerRate[3];
} Message;

extern Message message;

void USART1_Init(void);
void Telemetry_Send(Message *msg);
void Telemetry_DMA_Init(void);

#endif
