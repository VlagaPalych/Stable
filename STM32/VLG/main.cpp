#pragma import(__use_realtime_heap)

#include "stm32f4xx.h"
#include "main.h"
#include "leds.h"
#include "time.h"

VLG_InertialSensor inertial_sensor;
VLG_Compass compass;

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;    
}

int main() {
    RCC_Init();
    SysTick_Init();
    leds_init();
    
    if (inertial_sensor.init()) {
        // @TODO: error message
    }
    if (compass.init()) {
        // @TODO: error message
    }
    
    while (1) {
        if (compass.fresh_data()) {
            compass.update_calibr();
        }
    }
}
