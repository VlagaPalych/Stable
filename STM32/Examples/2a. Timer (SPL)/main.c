#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"


int main() {
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef timer;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Pin = GPIO_Pin_10;
	
	GPIO_Init(GPIOE, &gpio);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = 7200;
	timer.TIM_Period = 10000;
	TIM_TimeBaseInit(TIM3, &timer);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);
	
	while (1) {
	}
}

void TIM3_IRQHandler() {
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10) == 1) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	} else {
		GPIO_SetBits(GPIOE, GPIO_Pin_10);
	}
}
