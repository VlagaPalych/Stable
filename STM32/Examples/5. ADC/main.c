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
	RCC->APB1ENR 	|= RCC_TIMER_EN;// Enable TIM3 clock
	
	timer->PSC 		= 719;					// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 100kHz
	timer->ARR 		= 1000;					// Number of ticks to get overflow
	timer->CCR3 	= 0;						// CCy and ARR ratio set duty cycle of pwm
	timer->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // Set pwm mode 1
	timer->CCER 	|= TIM_CCER_CC3E; // Enable TIM3 capture/compre register 3
	
	TIM3->CR1 = TIM_CR1_CEN;			// Enable timer
}

void initPot() {
	RCC->AHBENR = RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER |= 3 << 1*2; 		// Analog input PA1
}

int calibrationValue 			= 0;
int ADC1ConvertedValue 		= 0;
int ADC1ConvertedVoltage 	= 0;

void initADC() {
	RCC->CFGR2 	|= RCC_CFGR2_ADCPRE12_DIV2;		// Set PLL clock to ADC12 division factor
																						// If we don't set this the frequenct is bigger than maximum ADC frequency
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;				// Enable ADC12 clock
	
	// Enable ADC voltage regulator
	ADC1->CR &= ~ADC_CR_ADVREGEN;							// 0x00 - intermediate state
	ADC1->CR |= ADC_CR_ADVREGEN_0;						// 0x01 - enabled
	
	// Calibration
	ADC1->CR &= ~ADC_CR_ADCALDIF;							// Writing ADCAL will launch a calibration in Single-ended inputs Mode
	ADC1->CR |= ADC_CR_ADCAL;									// Launch calibration
	while (ADC1->CR & ADC_CR_ADCAL);					// Wait until calibration ends
	calibrationValue = ADC1->CALFACT;
	
	ADC1->CFGR |= ADC_CFGR_CONT; 							// ADC_ContinuousConvMode_Enable
  ADC1->CFGR &= ~ADC_CFGR_RES; 							// 12-bit data resolution
  ADC1->CFGR &= ~ADC_CFGR_ALIGN; 						// Right data alignment

	ADC1->SQR1 	|= ADC_SQR1_SQ1_1;						// Set ADC1_IN2 to be the 1st in regular conversion
	ADC1->SQR1 	&= ~ADC_SQR1_L; 							// ADC regular channel sequence length = 0 => 1 conversion/sequence
  ADC1->SMPR1 |= ADC_SMPR1_SMP2_1 | ADC_SMPR1_SMP2_0; 	// = 0x03 => sampling time 7.5 ADC clock cycles
  ADC1->CR 		|= ADC_CR_ADEN; 							// Enable ADC1
  while(!ADC1->ISR & ADC_ISR_ADRD); 				// Wait for ADRDY
	
	ADC1->IER 	|= ADC_IER_EOC;								// End of regular conversion interrupt enable
	NVIC_EnableIRQ(ADC1_2_IRQn);

  ADC1->CR |= ADC_CR_ADSTART; 							// Start ADC1 Software Conversion 
}

int main() {
	initLed();
	initTimer();
	initPot();
	initADC();
	
	while (1) {
	}
}

void ADC1_2_IRQHandler() {
	ADC1ConvertedValue = ADC1->DR; 														// Get ADC1 converted data
	ADC1ConvertedVoltage = (ADC1ConvertedValue *3300) / 4096; 	// Compute the voltage
	TIM3->CCR3 = ADC1ConvertedVoltage * 33 / 10;							// Set duty cycle of led's pwm
}
