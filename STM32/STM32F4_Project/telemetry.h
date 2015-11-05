#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stdint.h"

typedef struct {
//    float ars1_x;
//    float ars1_y;
//    int16_t ars1_t;
//    float ars2_x;
//    float ars2_y;
//    int16_t ars2_t;
//    float ars3_z;
//    
//    int16_t accel_x;
//    int16_t accel_y;
//    int16_t accel_z;
    float roll;
    float pitch;
    
    float rollRate;
    float pitchRate;
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
