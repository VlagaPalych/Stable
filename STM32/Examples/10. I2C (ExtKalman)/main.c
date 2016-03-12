#include "stm32f30x.h"
#include "gyro.h"
#include "accel_magne.h"
#include "extra_math.h"
#include "telemetry.h"
#include "string.h"

Quat orientation;
extern float gyro_angleRate[VECT_SIZE];
extern float accel[VECT_SIZE];
extern float magField[VECT_SIZE];
float angleRate[VECT_SIZE];

extern float w1[VECT_SIZE], w2[VECT_SIZE];
extern float v1[VECT_SIZE], v2[VECT_SIZE];

float euler[VECT_SIZE];
float eulerRate[VECT_SIZE];

uint8_t Message_Size = 0;

extern float x_aposteriori_data[7];

void RCC_Init() {
    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_DMA1EN;
    RCC->APB1ENR    |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
}

uint8_t BUTTON_PIN = 0; // PA

void Button_Init() {
    GPIOA->MODER &= ~(3 << BUTTON_PIN*2);
    GPIOA->OSPEEDR |= 3 << BUTTON_PIN*2;
    
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->RTSR 	|= EXTI_RTSR_TR0;  	    // rising events
	EXTI->IMR 	|= EXTI_IMR_MR0; 		// we don't mask events on line 0
	NVIC_EnableIRQ(EXTI0_IRQn); 		// enable EXTI0 interrupt
}

void EXTI0_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR0) {
		EXTI->PR = EXTI_PR_PR0;		    // Clear interrupt flag
		
        Telemetry_Send(&message);
	}
}

void TIM2_Init() {
    TIM2->PSC = 7199;
    TIM2->ARR = 10000;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0x01);   
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        
        Telemetry_Send(&message);
    }
}

int main() { 
    QUEST_Init();
    Message_Size = sizeof(Message);
    RCC_Init();
    
    GPIOA->MODER &= ~(3 << 15*2);
    GPIOA->MODER |= 1 << 15*2;
    
    Gyro_Init();
    Gyro_EXTI_Init();  
    
    EXTI->SWIER |= EXTI_SWIER_SWIER1;   
    
    AM_Init();
    AM_EXTI_Init();
    
    EXTI->SWIER |= EXTI_SWIER_SWIER4;
    
    USART1_Init();
    //Button_Init();
    //TIM2_Init();
    
    while (1) {
        if (process) {
            GPIOA->BSRR |= GPIO_BSRR_BS_15;
            process = 0;
            memcpy(angleRate, gyro_angleRate, VECT_SIZE*sizeof(float));
            
            QUEST();          
            
//            memcpy(message.q+1, orientation.v, VECT_SIZE*sizeof(float));
//            message.q[0] = orientation.w;
            
            memcpy(zk_data, angleRate, VECT_SIZE*sizeof(float));
            zk_data[3] = orientation.w;
            memcpy(zk_data+VECT_SIZE+1, orientation.v, VECT_SIZE*sizeof(float));
            
            Kalman();
            orientation.w = x_aposteriori_data[3];
            memcpy(orientation.v, x_aposteriori_data+VECT_SIZE+1, VECT_SIZE*sizeof(float));
            Quat_ToEuler(orientation, euler);
            
            angleRate_to_eulerRate(angleRate, euler, eulerRate);
            
            radians_to_degrees(euler);
            radians_to_degrees(eulerRate);
            
            // telemetry
//            memcpy(message.w1, w1, VECT_SIZE*sizeof(float));
//            memcpy(message.w2, w2, VECT_SIZE*sizeof(float));
//            memcpy(message.accel, accel, VECT_SIZE*sizeof(float));
//            memcpy(message.magField, magField, VECT_SIZE*sizeof(float));
            memcpy(message.angleRate, angleRate, VECT_SIZE*sizeof(float));
//            memcpy(message.q+1, orientation.v, VECT_SIZE*sizeof(float));
//            message.q[0] = orientation.w;
            memcpy(message.euler, euler, VECT_SIZE*sizeof(float));
            memcpy(message.eulerRate, eulerRate, VECT_SIZE*sizeof(float));
            Telemetry_Send(&message);
            GPIOA->BSRR |= GPIO_BSRR_BR_15;
        }
    }
}
