#include "stm32f30x.h"
#include "gyro.h"

void RCC_Init() {
    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_DMA1EN;
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;
}

uint16_t answer = 0;

int main() {
    RCC_Init();
    Gyro_Init();
    Gyro_EXTI_Init();
    
    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    while (1) {
    }
}
