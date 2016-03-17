#include "stm32f4xx.h" 
#include "mpu9250.h"

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;    
}

uint8_t nack = 0;
int main() {
    RCC_Init();
    IMU_NSS_Init();
    IMU_NSS_High();
    SPI2_Init();
    
    IMU_Init();
    IMU_EXTI_Init();
    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    Mag_Init();
    while (1) {
        //nack = IMU_ReadByte(54);
    }
}
