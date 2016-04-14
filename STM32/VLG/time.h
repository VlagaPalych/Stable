#ifndef TIME_H
#define TIME_H

#include "stdint.h"

extern "C" void delay_ms(uint32_t nTime);
extern "C" void delay_us(uint16_t us);

extern "C" void SysTick_Init();
extern "C" uint32_t millis();

#endif // TIME_H
