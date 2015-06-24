#include "stm32f30x.h"
#include "math.h"

#define PI 3.14159
#define SIGNAL_SIZE 256
uint16_t signal[SIGNAL_SIZE];
uint16_t i = 0;

int main() {
    for (i = 0; i < SIGNAL_SIZE; i++) {
        signal[i] = (uint16_t)(sin(2*PI*i/SIGNAL_SIZE)*2047.5 + 2047.5);
    }
    
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    GPIOA->MODER |= GPIO_MODER_MODER4;
    
    TIM6->PSC = 0;
    TIM6->ARR = 2000;
//    TIM6->DIER |= TIM_DIER_UIE;
//    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR2 |= TIM_CR2_MMS_1;
    
    DAC->CR |= DAC_CR_TEN1;
    
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP;
 
    DMA1_Channel3->CPAR = (uint32_t)(&DAC->DHR12R1);
    DMA1_Channel3->CMAR = (uint32_t)(signal);
    DMA1_Channel3->CNDTR = SIGNAL_SIZE;
    DMA1_Channel3->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
    DMA1_Channel3->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL | DMA_CCR_EN;
    
    DAC->CR |= DAC_CR_EN1;
    DAC->CR |= DAC_CR_DMAEN1;
    TIM6->CR1 |= TIM_CR1_CEN;
    
    while (1) {
    }
}

//void TIM6_DAC_IRQHandler() {
//    TIM6->SR &= ~TIM_SR_UIF;
//    DAC->DHR8R1 = signal[i];
//    i++;
//    if (i == 256) i = 0;
//}
