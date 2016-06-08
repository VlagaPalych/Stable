#include "leds.h"
#include "stm32f4xx.h"

#define LEDS_NUM 4

static led_t leds[LEDS_NUM] = {GREEN, ORANGE, RED, BLUE};

extern "C" void leds_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    
    uint32_t moder = 0;
    for (uint8_t i = 0; i < LEDS_NUM; i++) {
        moder |= (3 << leds[i]*2);
    }
    GPIOD->MODER &= ~moder;
    moder = 0;
    for (uint8_t i = 0; i < LEDS_NUM; i++) {
        moder |= (1 << leds[i]*2);
    }
    GPIOD->MODER |= moder;
}

extern "C" void led_on(led_t led) {
    GPIOD->BSRRL |= (1 << led);
}

extern "C" void led_off(led_t led) {
    GPIOD->BSRRH |= (1 << led);
}

extern "C" void led_toggle(led_t led) {
    if (GPIOD->ODR & (1 << led)) {
        led_off(led);
    } else {
        led_on(led);
    }
}
