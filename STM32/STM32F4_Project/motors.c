#include "motors.h"
#include "stm32f4xx.h"

// Motors control and frequency measurement pins
uint8_t MOT_PWM1    = 12;   // PD12
uint8_t MOT_PWM2    = 14;   // PD14
uint8_t MOT_FREQ1   = 2;    // PE2
uint8_t MOT_FREQ2   = 4;    // PE4

extern uint8_t ENGRDY;

void Motors_GPIO_Init() {
    // PD12, PD14 - motor pwm
    GPIOD->MODER    |= (2 << MOT_PWM1*2) | (2 << MOT_PWM2*2);               // alternative function
    GPIOD->OTYPER   &= ~((1 << MOT_PWM1) | (1 << MOT_PWM2));                // push-pull
    GPIOD->OSPEEDR  |= (3 << MOT_PWM1*2) | (3 << MOT_PWM2*2);               // high speed
    GPIOD->PUPDR    |= (1 << MOT_PWM1*2) | (1 << MOT_PWM2*2);               // pull-up
    GPIOD->AFR[1]   |= (2 << (MOT_PWM1 - 8)*4) | (2 << (MOT_PWM2 - 8)*4);   // AF2

    // motor frequency calculation
//    GPIOE->MODER 	&= ~((3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2));          // input
//    GPIOE->OTYPER	&= ~((1 << MOT_FREQ1) | (1 << MOT_FREQ2));              // push-pull
//    GPIOE->OSPEEDR 	|= (3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2);             // high speed 
//    GPIOE->PUPDR    |= (1 << MOT_FREQ1*2) | (1 << MOT_FREQ2*2);             // pull-up
}

//void Motors_EXTI_Init() {
//    // frequency calculation interrupt for motor1
//    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;   
//    EXTI->FTSR 	|= EXTI_FTSR_TR2; 
//    EXTI->IMR 	|= EXTI_IMR_MR2;
//    NVIC_EnableIRQ(EXTI2_IRQn); 
//    
//    // frequency calculation interrupt for motor2
//    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PE; 
//    EXTI->FTSR 	|= EXTI_FTSR_TR4; 
//    EXTI->IMR 	|= EXTI_IMR_MR4;
//    NVIC_EnableIRQ(EXTI4_IRQn); 
//}

void Motors_Init() {
    Motors_GPIO_Init();
    //Motors_EXTI_Init();
    Motors_TIM_Init();
}

void Motors_TIM_Init() {
    // TIM4 - PWM timer for motors
	TIM4->PSC 		= 7;	
	TIM4->ARR 		= 10000;
	TIM4->CCR1 	    = 2000;
	TIM4->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
	TIM4->CCER 	    |= TIM_CCER_CC1E; 
    
    TIM4->CCR3 	    = 2000;		
	TIM4->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; 
	TIM4->CCER 	    |= TIM_CCER_CC3E; 
	TIM4->CR1       = TIM_CR1_CEN;
    
    // TIM3 - turning on timer for motors
    TIM3->PSC = 7;
    TIM3->ARR = 5000;
    TIM3->DIER |= 1;					
	NVIC_EnableIRQ(TIM3_IRQn);   
    TIM3->CR1 = TIM_CR1_CEN;

//    TIM5->DIER |= 1;					// timer for frequency calculation
//	  NVIC_EnableIRQ(TIM5_IRQn);
//    TIM5->CR1 = TIM_CR1_CEN;
    
//    TIM9->DIER |= 1;					// timer for frequency calculation
//    TIM9->CR1 = TIM_CR1_CEN;
}

uint8_t status = 0;
int pwm         = 0;
int timer       = 0;
int timer_1000  = 2000;
int timer_2000  = 2000;
int pwm_step    = 5;

void TIM3_IRQHandler() { //moving pwm from 2 to 1ms in the start
    TIM3->SR &= ~TIM_SR_UIF;

    if (status == 0) {
        timer++;
        if (pwm == 1000 && timer == timer_1000) {
            status = 1;
            timer = 0;
        } else if (pwm == 2000 && timer == timer_2000) {
            status = 2;
            timer = 0;
        }
    } else if (status == 1) {
        pwm += pwm_step;
        TIM4->CCR1 = pwm;
        TIM4->CCR3 = pwm;
        if (pwm == 2000) {
            status = 0;
            timer = 0;
        }
    } else if (status == 2) {
        pwm -= pwm_step;
        TIM4->CCR1 = pwm;
        TIM4->CCR3 = pwm;
        if (pwm == 1000) {
            ENGRDY = 1;
            status = 3;
            timer = 0;
        }
    }    
}

//void EXTI2_IRQHandler() { //this used to calculate the frequency of motor
//    if (EXTI->PR & EXTI_PR_PR2) {
//        EXTI->PR |= EXTI_PR_PR2;
//        if (TIM5->CR1 && 1) {
//            COUNT1 = TIM5->CNT;
//            TIM5->CR1 &= ~1;
//            TIM5->CNT = 0;
//        }
//        else {
//            TIM5->CNT = 0;
//            TIM5->CR1 |= 1;
//        }
//    }
//}

//void EXTI4_IRQHandler() { //this used to calculate the frequency of motor
//    if (EXTI->PR & EXTI_PR_PR4) {
//        EXTI->PR |= EXTI_PR_PR4;
//        if (TIM9->CR1 && 1) {
//            COUNT2 = TIM9->CNT;
//            TIM9->CR1 &= ~1;
//            TIM9->CNT = 0;
//        }
//        else {
//            TIM9->CNT = 0;
//            TIM9->CR1 |= 1;
//        }
//    }
//}
