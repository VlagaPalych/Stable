#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
    float arx;
    float ary;
    float phi_x;
    float phi_y;
} Message;

extern Message message;
extern uint8_t Message_Size;

extern uint8_t curFreq;
extern uint8_t freshFreq;
extern uint8_t recalibrate;

extern uint8_t turnUselessOn;
extern uint8_t gyroRecalibrationOn;
extern uint8_t telemetryOn;

void USART_Init(void);
void Telemetry_TIM_Init(void);
void SendTelemetry(Message *msg);
void Telemetry_DMA_Init(void) ;
void send_to_uart(uint8_t data);

void SendRaw(void);
void SendRawAndProcessed(void);

#endif
