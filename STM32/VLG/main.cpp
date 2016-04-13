#include "stm32f4xx.h"
#include "VLG_InertialSensor.h"

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;    
}

int main() {
    RCC_Init();
    GPIOD->MODER &= ~(3 << 12*2);
    GPIOD->MODER |= (1 << 12*2);
    
    VLG_InertialSensor mpu;
    mpu.init();
    
    while (1) {}
}
