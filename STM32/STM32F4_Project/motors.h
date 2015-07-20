#ifndef MOTORS_H
#define MOTORS_H

extern int pwm1;
extern int pwm2;
//extern int COUNT1;
//extern int COUNT2;

void Motors_GPIO_Init(void);
//void Motors_EXTI_Init(void);
void Motors_TIM_Init(void);
void Motors_Init(void);

#endif
