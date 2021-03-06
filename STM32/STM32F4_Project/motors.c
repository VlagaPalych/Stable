#include "motors.h"
#include "processing.h"
#include "stm32f4xx.h"

// Motors control and frequency measurement pins
uint8_t MOT_PWM1    = 12;   // PD12
uint8_t MOT_PWM2    = 14;   // PD14
uint8_t MOT_FREQ1   = 2;    // PE2
uint8_t MOT_FREQ2   = 4;    // PE4

int minPwm = 1200;
int maxPwm = 2000;

extern uint8_t ENGRDY;
extern uint8_t STABRDY;


void Motors_GPIO_Init() {
    // PD12, PD14 - motor pwm
    GPIOD->MODER    |= (2 << MOT_PWM1*2) | (2 << MOT_PWM2*2);               // alternative function
    GPIOD->OTYPER   &= ~((1 << MOT_PWM1) | (1 << MOT_PWM2));                // push-pull
    GPIOD->OSPEEDR  |= (3 << MOT_PWM1*2) | (3 << MOT_PWM2*2);               // high speed
    GPIOD->PUPDR    |= (1 << MOT_PWM1*2) | (1 << MOT_PWM2*2);               // pull-up
    GPIOD->AFR[1]   |= (2 << (MOT_PWM1 - 8)*4) | (2 << (MOT_PWM2 - 8)*4);   // AF2

    // motor frequency calculation
    GPIOE->MODER 	&= ~((3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2));          // input
    GPIOE->OTYPER	&= ~((1 << MOT_FREQ1) | (1 << MOT_FREQ2));              // push-pull
    GPIOE->OSPEEDR 	|= (3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2);             // high speed 
    GPIOE->PUPDR    |= (1 << MOT_FREQ1*2) | (1 << MOT_FREQ2*2);             // pull-up
}

void Motors_EXTI_Init() {
    // frequency calculation interrupt for motor1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;   
    EXTI->FTSR 	|= EXTI_FTSR_TR2; 
    EXTI->IMR 	|= EXTI_IMR_MR2;
    NVIC_SetPriority(EXTI2_IRQn, 0x01); 
    NVIC_EnableIRQ(EXTI2_IRQn); 
    
    // frequency calculation interrupt for motor2
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PE; 
    EXTI->FTSR 	|= EXTI_FTSR_TR4; 
    EXTI->IMR 	|= EXTI_IMR_MR4;
    NVIC_SetPriority(EXTI4_IRQn, 0x01); 
    NVIC_EnableIRQ(EXTI4_IRQn); 
}

void Motors_Init() {
    Motors_GPIO_Init();
    Motors_EXTI_Init();
    Motors_TIM_Init();
}

// Start procedure:
// Hold 10% duty cycle for 1500 ms
uint8_t tim3_status = 0;
int start_pwm   = 1000;
int timer       = 0;
int timer_1000  = 300; 


void Motors_TIM_Init() {
    // TIM4 - PWM timer for motors
    // Motors are controlled by pwm signal of frequency 100 Hz
    // Possible duty cycles - [10%, 20%]
	TIM4->PSC 		= 63;	
	TIM4->ARR 		= 10000;
	TIM4->CCR1 	    = start_pwm;
	TIM4->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; 
	TIM4->CCER 	    |= TIM_CCER_CC1E; 
    
    TIM4->CCR3 	    = start_pwm;		
	TIM4->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE; 
	TIM4->CCER 	    |= TIM_CCER_CC3E; 
	TIM4->CR1       = TIM_CR1_CEN;
    
    // TIM3 - timer for turning motors on
    TIM3->PSC = 63;
    TIM3->ARR = 5000;
    TIM3->DIER |= 1;			
    NVIC_SetPriority(TIM3_IRQn, 0x01);     
	NVIC_EnableIRQ(TIM3_IRQn);      
    TIM3->CR1 = TIM_CR1_CEN;

    
    TIM5->PSC = 3;
    TIM5->ARR = 50000;
    TIM5->DIER |= 1;					// timer for frequency calculation
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 = TIM_CR1_CEN;
    
    TIM9->PSC = 3;
    TIM9->ARR = 50000;
    TIM9->DIER |= 1;					// timer for frequency calculation
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    TIM9->CR1 = TIM_CR1_CEN;
}


void TIM3_IRQHandler() { 
    TIM3->SR &= ~TIM_SR_UIF;

    if (tim3_status == 0) {
        timer++;
        if (timer == timer_1000) {
            tim3_status = 1;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            ENGRDY = 1;
            timer = 0;
        } 
    }       
}

void Motors_Stop() {
    pwm = 0;
    pwm1 = 1000;
    pwm2 = 1000;
    Motors_Run();
}

uint8_t Motor1Off = 0;
uint8_t Motor2Off = 0;
#define MOTOR_START_DELAY   10
void EXTI2_IRQHandler() { //this used to calculate the frequency of motor
    //GPIOD->ODR |= 1 << 15;
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR = EXTI_PR_PR2;
        if (TIM5->CR1 && 1) {
            COUNT1 = TIM5->CNT; 
//            if (COUNT1 < 40000 && Motor1Off ) {
//                Motor1Off--;
//            //    if(!Motor1Off) TIM4->CCR1 = pwm1;
//            }
//            if ((COUNT1 < 10000) && Motor1Off==MOTOR_START_DELAY-1) {
//                COUNT1 = 50000;
//                Motor1Off=MOTOR_START_DELAY;
//            }
            //TIM5->CR1 &= ~1;
            TIM5->CNT = 0;
            
           
        }
//        else {
//            TIM5->CNT = 0;
//            TIM5->CR1 |= 1;
//        }
    }
     //GPIOD->ODR &= ~(1 << 15);
}

void TIM5_IRQHandler() {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        Motor1Off = MOTOR_START_DELAY;
        COUNT1 = 50000;
    }
}

void TIM1_BRK_TIM9_IRQHandler() {
    if (TIM9->SR & TIM_SR_UIF) {
        TIM9->SR &= ~TIM_SR_UIF;
        Motor2Off = MOTOR_START_DELAY;
        COUNT2 = 50000;
    }
}

void EXTI4_IRQHandler() { //this used to calculate the frequency of motor
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR = EXTI_PR_PR4;
        if (TIM9->CR1 && 1) {
            COUNT2 = TIM9->CNT;
//             if (COUNT2 < 40000 && Motor2Off) {
//                Motor2Off --;
//              // if(!Motor2Off) TIM4->CCR3 = pwm2;
//            }
//            if ((COUNT2 < 10000) && Motor2Off==MOTOR_START_DELAY-1) {
//                COUNT2 = 50000;
//                Motor2Off=MOTOR_START_DELAY;
//            }
            //TIM9->CR1 &= ~1;
            TIM9->CNT = 0;
            
           
        }
//        else {
//            TIM9->CNT = 0;
//            TIM9->CR1 |= 1;
//        }
    }
}

void Motors_Run(void) {
//    if (Motor1Off && pwm1 != 1000) { // motor is off
//        TIM4->CCR1 = 1900;
//    } else {
        TIM4->CCR1 = pwm1;
//    }
//    if (Motor2Off && pwm2 != 1000) {
//        TIM4->CCR3= 1900;
//    } else {
        TIM4->CCR3 = pwm2;
    //}
}

void Motors_SetPwm() {
    if (pwm > 0) {
        pwm1 = pwm + minPwm;
        if (pwm1 > maxPwm) {
            pwm1 = maxPwm;
            pwm = maxPwm - minPwm;
        }
        pwm2 = minPwm;
    } else {
        pwm2 = -pwm + minPwm;
        if (pwm2 > maxPwm) {
            pwm2 = maxPwm;
            pwm = minPwm - maxPwm;
        }
        pwm1 = minPwm;
    }
}
