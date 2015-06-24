/* This example illustrates use of external interrupts like button presses
	 LED is attached to PC8.
	 Button is attached to PA0.
*/

#include "stm32f30x.h"

GPIO_TypeDef *port = GPIOC;
uint32_t RCC_PORT_EN = RCC_AHBENR_GPIOCEN;
uint32_t led = 8;

TIM_TypeDef *timer = TIM3;
uint32_t RCC_TIMER_EN = RCC_APB1ENR_TIM3EN;


void initLed() {
	RCC->AHBENR |= RCC_PORT_EN; 		// Enable port clock 
	
	port->MODER 	|= 1 << led*2; 		// General purpose output mode
	port->OTYPER 	&= ~(1 << led); 	// Push-pull
	port->OSPEEDR 	|= 3 << led*2;	// High speed
	port->PUPDR 	|= 1 << led*2;		// Pull-up
}

void initTimer() {
	RCC->APB1ENR 	|= RCC_TIMER_EN; // Enable timer clock  
	
	timer->PSC = 7199; 				// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 10kHz
	timer->ARR = 500;					// 50 ms to avoid button rattle
	
	timer->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);// Enable interrupts from TIM3
}

void initInterrupt() {
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA; // allow interrupts from PA0
	//EXTI->FTSR 	|= EXTI_FTSR_TR0;	// falling events
	EXTI->RTSR 	|= EXTI_RTSR_TR0;  	// rising events
	EXTI->IMR 	|= EXTI_IMR_MR0; 		// we don't mask events on line 0
	NVIC_EnableIRQ(EXTI0_IRQn); 		// enable EXTI0 interrupt
}

void initButton() {
	RCC->AHBENR   |= ((1UL << 17) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  |=  ((3UL << 2*0)  );         /* PA.0 is High Speed         */
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
	
	initInterrupt();
}

int main() {
	initLed();
	initButton();	
	initTimer();
	
	while (1) {
	}
}

void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF;   	// Clear interrupt flag
		TIM3->CR1 &= ~TIM_CR1_CEN; 	// Turn timer off
		port->ODR ^= 1 << led;			// Toggle led
	}
}

void EXTI0_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR0) {
		EXTI->PR |= EXTI_PR_PR0;		// Clear interrupt flag
		TIM3->CNT = 0;							// Reset timer counter
		TIM3->CR1 |= TIM_CR1_CEN;		// Turn timer on
	}
}
