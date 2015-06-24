#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

void softDelay(uint32_t time) {
	while (time > 0) {
		time--;
	}
}

int main() {
	GPIO_InitTypeDef gpio;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	
	gpio.GPIO_Mode 	= GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;
	gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
	gpio.GPIO_Pin 	= GPIO_Pin_10;
	GPIO_Init(GPIOE, &gpio);
	
	while(1) {
		GPIO_SetBits(GPIOE, GPIO_Pin_10);
		softDelay(10000000);
		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
		softDelay(10000000);
	}
}
