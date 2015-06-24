/* This example uses TIM3 to get delays.
	 LED is attached to PC8 */

#include "stm32f30x.h"

GPIO_TypeDef *port = GPIOC;
uint32_t RCC_PORT_EN = RCC_AHBENR_GPIOCEN;
uint32_t led = 8;

TIM_TypeDef *timer = TIM3;
uint32_t RCC_TIMER_EN = RCC_APB1ENR_TIM3EN;

int main() {
	// Init GPIO
	RCC->AHBENR |= RCC_PORT_EN; // Turn tacting GPIOC on
	
	port->MODER 	|= 1 << led*2; 	// General purpose output mode
	port->OTYPER 	&= ~(1 << led); // Push-pull
	port->OSPEEDR	|= 3 << led*2;	// High speed
	port->PUPDR 	|= 1 << led*2;	// Pull-up
	
	// Init Timer
	RCC->APB1ENR 	|= RCC_TIMER_EN; // Turn tacting TIM3 
	
	timer->PSC = 7199; 				// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 10kHz
	timer->ARR = 10000;				// Number of ticks to get overflow
	
	timer->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);// Enable interrupts from TIM3
	
	timer->CR1 = TIM_CR1_CEN;	// Enable timer
	
	while (1) {
//		if (timer->SR & TIM_SR_UIF) {
//			timer->SR &= ~TIM_SR_UIF;
//			port->ODR ^= 1 << led;
//		}
	}
}

void TIM3_IRQHandler() {
	timer->SR &= ~TIM_SR_UIF;
	port->ODR ^= 1 << led;
}
