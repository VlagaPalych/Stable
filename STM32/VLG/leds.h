#ifndef LEDS_H
#define LEDS_H

#include "stdint.h"

enum led_t {
    GREEN   = 12,
    ORANGE  = 13,
    RED     = 14, 
    BLUE    = 15
};

extern "C" void leds_init();
extern "C" void led_on(led_t led);
extern "C" void led_off(led_t led);
extern "C" void led_toggle(led_t led);


    

#endif // LEDS_H
