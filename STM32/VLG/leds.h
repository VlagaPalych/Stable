#ifndef LEDS_H
#define LEDS_H

#include "stdint.h"

extern "C" void leds_init();
extern "C" void led_on(uint8_t num);
extern "C" void led_off(uint8_t num);
extern "C" void led_toggle(uint8_t num);

#endif // LEDS_H
