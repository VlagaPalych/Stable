#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
	float angle;
	float angleRate;
	uint16_t pwm1;
	uint16_t pwm2;
	uint16_t freq1;
	uint16_t freq2;
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
