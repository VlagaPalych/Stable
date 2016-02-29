#include "stm32f3xx.h"
#include "gyro.h"
#include "accel_magne.h"

void RCC_Init() {
    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_DMA1EN;
    RCC->APB1ENR    |= RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;
}

uint16_t answer = 0;

int main() {
    RCC_Init();
    Gyro_Init();
    Gyro_EXTI_Init();
    
    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    AM_Init();
    
    while (1) {
        answer = AM_SingleRead(0x20);
    }
}
