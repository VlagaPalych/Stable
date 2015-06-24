/* This example illustrates how to get pwm on some pin using pwm mode of timer.
   LED is attached to PC8. */

#include "stm32f30x.h"

GPIO_TypeDef *port = GPIOC;
uint32_t RCC_PORT_EN = RCC_AHBENR_GPIOCEN;
uint32_t led = 8;

TIM_TypeDef *timer = TIM3;
uint32_t RCC_TIMER_EN = RCC_APB1ENR_TIM3EN;

void initLed() {
	RCC->AHBENR |= RCC_PORT_EN;
	
	port->MODER 	|= 2 << led*2;  // Alternate function mode
	port->OTYPER 	&= ~(1 << led); // Push-pull
	port->OSPEEDR |= 3 << led*2;	// High speed
	port->PUPDR 	|= 1 << led*2;	// Pull-up
	port->AFR[1] 	|= 2;						// AFR[1] because we use pin 8, 2 because we want to use AF2 = TIM3CH3
																// Alternate functions list for each pin can be found in datasheet
}

void initTimer() {
	RCC->APB1ENR 	|= RCC_TIMER_EN;// Turn tacting TIM3 
	
	timer->PSC 		= 7199;					// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 100kHz
	timer->ARR 		= 100;					// Number of ticks to get overflow
	timer->CCR3 	= 50;						// CCy and ARR ratio set duty cycle of pwm
	timer->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // Set pwm mode 1
	timer->CCER 	|= TIM_CCER_CC3E; // Enable TIM3 capture/compre register 3
	
	TIM3->CR1 = TIM_CR1_CEN;			// Enable timer
}

int main() {
	initLed();
	initTimer();
	
	while (1) {
	}
}
