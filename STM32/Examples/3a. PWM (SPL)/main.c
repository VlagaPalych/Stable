#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"


int main() {
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef timerPWM;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	
	gpio.GPIO_Mode 	= GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;
	gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
	gpio.GPIO_Pin 	= GPIO_Pin_8;
	
	GPIO_Init(GPIOC, &gpio);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_2);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = 720;
	timer.TIM_Period = 100;
	TIM_TimeBaseInit(TIM3, &timer);
	
	TIM_OCStructInit(&timerPWM);
	timerPWM.TIM_Pulse = 50;
	timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
	timerPWM.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM3, &timerPWM);
	
	TIM_Cmd(TIM3, ENABLE);
	
	while (1) {
	}
}
