#include "stm32f30x.h"
#include "gyro.h"
#include "accel_magne.h"
#include "extra_math.h"

Quat orientation;
float w1[VECT_SIZE], w2[VECT_SIZE];
float v1[VECT_SIZE], v2[VECT_SIZE];

float euler[3];

void RCC_Init() {
    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_DMA1EN;
    RCC->APB1ENR    |= RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;
}

uint16_t answer = 0;

int main() {
    QUEST_Init();
    
    RCC_Init();
    
    Gyro_Init();
    Gyro_EXTI_Init();
    
    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    
    AM_Init();
    AM_EXTI_Init();
    
    EXTI->SWIER |= EXTI_SWIER_SWIER4;
    
    while (1) {
        if (quest_run) {
            quest_run = 0;
            QUEST();
            Quat_ToEuler(orientation, euler);
        }
    }
}
