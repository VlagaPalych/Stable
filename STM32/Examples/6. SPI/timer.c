#include "stm32f30x.h"
#include "timer.h"

uint8_t DELAY_FLAG = 0;

void Timer_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    TIM2->PSC = 95;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0x0);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void Delay(uint32_t us) {
    TIM2->ARR = us;
    
    TIM2->CR1 |= TIM_CR1_CEN;
    
    DELAY_FLAG = 1;
    while (DELAY_FLAG) { __nop(); }
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        DELAY_FLAG = 0;
    }
}
