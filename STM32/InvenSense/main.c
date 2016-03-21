#include "stm32f4xx.h" 
#include "mpu9250.h"

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;    
}


// < 65536 ms
void Delay_ms(uint16_t ms) {
    TIM6->PSC = 15999;
    TIM6->ARR = ms;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN)!=0);
}
// < 65536 us
void Delay_us(uint16_t us) {
    TIM6->PSC = 15;
    TIM6->ARR = us;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN)!=0);
}

uint8_t send[4] = {0x21, 0x10, 0x08, 0xff};
uint8_t recv[4];

int main() {
    RCC_Init();
//    GPIOA->MODER &= ~(3 << 15*2);
//    GPIOA->MODER |= 1 << 15*2;

    IMU_NSS_Init();
    IMU_NSS_High();
    SPI2_Init();

    IMU_Init();
    Mag_Init();

    MPU_LoadFirmware(dmp_memory, DMP_CODE_SIZE, startAddress);

    IMU_DMA_Init();
    IMU_EXTI_Init();
    EXTI->SWIER |= EXTI_SWIER_SWIER1;

    while (1) {
    }
}
