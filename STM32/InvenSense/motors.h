#ifndef MOTORS_H
#define MOTORS_H

#include "stdint.h"

extern int minPwm;
extern int maxPwm;
extern int pwm1;
extern int pwm2;
extern int COUNT1;
extern int COUNT2;
extern uint8_t ENGRDY;

extern uint8_t tim3_status;

void Motors_GPIO_Init(void);
//void Motors_EXTI_Init(void);
void Motors_TIM_Init(void);
void Motors_Init(void);
void Motors_Stop(void);
void Motors_InitForStab(void);

// with current pwms
void Motors_Run(void);
void Motors_SetPwm(void);

#endif
