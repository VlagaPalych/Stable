#include "leds.h"
#include "stm32f4xx.h"

#define LEDS_NUM 4
static uint8_t led_pins[LEDS_NUM] = {12, 13, 14, 15};

extern "C" void leds_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    
    uint32_t moder = 0;
    for (uint8_t i = 0; i < LEDS_NUM; i++) {
        moder |= (3 << led_pins[i]*2);
    }
    GPIOD->MODER &= ~moder;
    moder = 0;
    for (uint8_t i = 0; i < LEDS_NUM; i++) {
        moder |= (1 << led_pins[i]*2);
    }
    GPIOD->MODER |= moder;
}

extern "C" void led_on(uint8_t num) {
    GPIOD->BSRRL |= (1 << led_pins[num]);
}

extern "C" void led_off(uint8_t num) {
    GPIOD->BSRRH |= (1 << led_pins[num]);
}

extern "C" void led_toggle(uint8_t num) {
    if (GPIOD->ODR & (1 << led_pins[num])) {
        led_off(num);
    } else {
        led_on(num);
    }
}
