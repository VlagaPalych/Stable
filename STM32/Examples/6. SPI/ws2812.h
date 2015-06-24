#ifndef WS2812_H
#define WS2812_H
#include "stm32f30x.h"

#define BRIGHT_0 0x00
#define BRIGHT_1 0x01
#define BRIGHT_2 0x02
#define BRIGHT_3 0x04
#define BRIGHT_4 0x08
#define BRIGHT_5 0x10
#define BRIGHT_6 0x20
#define BRIGHT_7 0x40
#define BRIGHT_8 0x80

#define LED_NUMBER      79
#define SKIP_LEDS       0
#define REAL_LED_NUMBER (LED_NUMBER - SKIP_LEDS - 1)

void WS2812_Init(void);

uint32_t WS2812_RGB(uint32_t red, uint32_t green, uint32_t blue);

void WS2812_Label(uint8_t led, uint8_t labelLength, uint32_t labelColor);

void WS2812_LabelWithTail(uint8_t led, uint8_t labelLength, uint32_t labelColor, uint8_t tailLeft, uint8_t tailLength, uint32_t tailColor);

void WS2812_LabelWithGradientTail(uint8_t led, uint8_t labelLength, uint32_t labelColor, uint8_t tailLeft, uint8_t tailLength, uint32_t tailColor);

void WS2812_StableLabel(uint8_t led, uint32_t color1, uint32_t color2);

#endif
