#include "stm32f4xx.h" 
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "syssolve.h"
#include "adxl345.h"
#include "telemetry.h"

#define G (0xFFFF/4)
#define CTRL_REG4 0x20
#define FREQ100 0x67
#define FREQ800 0x87
#define CTRL_REG3 0x23
#define INT1_EN 0x88
#define PWM_STABLE 2000

// SPI communication pins for accelerometer
uint8_t SPI2_SCK    = 13;    // PB
uint8_t SPI2_MISO   = 14;    // PB
uint8_t SPI2_MOSI   = 15;    // PB
uint8_t SPI2_NSS    = 12;    // PB
uint8_t ACCEL_INT1  = 1;    // PA1
uint8_t ACCEL_VCC   = 13;   // PC14

// Motors control and frequency measurement pins
uint8_t MOT_PWM1    = 12;   // PD12
uint8_t MOT_PWM2    = 14;   // PD14
uint8_t MOT_FREQ1   = 2;    // PE2
uint8_t MOT_FREQ2   = 4;    // PE4

uint8_t BUTTON      = 10;   // PE10

uint8_t UART2_TX    = 2;    // PA2
uint8_t UART2_RX    = 3;    // PA3

uint8_t SERV_PWM    = 14;   // PB14

int16_t ax = 0, ay = 0, az = 0;
int param = 100;
int xoff, yoff, zoff;
int ENGRDY = 0;
int COUNT1 = 0;
int COUNT2 = 0;
uint8_t NEW_PWM_RV = 0;
//int16_t accelRegisters[6] = {0xA800, 0xA900, 0xAA00, 0xAB00, 0xAC00, 0xAD00};
//int16_t accelRegisters[6] = {0xA800, 0, 0, 0, 0, 0};
//int16_t accel[6];

extern uint8_t stabilizationOn;


uint8_t received = 0;
char str[100];
int PWM_RX = 0, pwm = 0, i=-1, st = 1;
uint8_t SEND_TELEMETRY_FLAG = 0;

double y[3];                // 3 last angles
double u[3];                // 3 last thrusts

double w[3];                // unknown coeffs in diff eq
double Afull[10*3];         // matrix of our equation Afull * w = Bfull
double Bfull[10];           // right column

uint8_t anglesAccumulated;  // number of angles we have at the moment
uint8_t row;


int16_t ax, ay, az;
int16_t xOffset, yOffset, zOffset;

 

 
void USART_init(void) {
    GPIOA->MODER |= (2 << UART2_TX*2) | (2 << UART2_RX*2);
    GPIOA->OTYPER &= ~((1 << UART2_TX) | (1 << UART2_RX));
    GPIOA->OSPEEDR |= (3 << UART2_TX*2) | (3 << UART2_RX*2);
    GPIOA->PUPDR |= (1 << UART2_TX*2) | (1 << UART2_RX*2);
    GPIOA->AFR[0]|= (7 << UART2_TX*4) | (7 << UART2_RX*4);

    USART2->BRR = 0x341; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART2_IRQn);
}

void RCC_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_SPI2EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_USART2EN;
}

void EXTI_Init() {
    // acclerometer external interrupt on ACCEL_INT1
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PE;
    EXTI->FTSR 	|= EXTI_FTSR_TR0;  
    EXTI->IMR 	|= EXTI_IMR_MR0;
    //NVIC_SetPriority(EXTI0_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI0_IRQn); 
    
    // frequency calculation interrupt for motor1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;   
    EXTI->FTSR 	|= EXTI_FTSR_TR2; 
    EXTI->IMR 	|= EXTI_IMR_MR2;
    //NVIC_SetPriority(EXTI2_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI2_IRQn); 
    
    // frequency calculation interrupt for motor2
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PE; 
    EXTI->FTSR 	|= EXTI_FTSR_TR4; 
    EXTI->IMR 	|= EXTI_IMR_MR4;
    //NVIC_SetPriority(EXTI4_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI4_IRQn); 
    
    // button
    SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI10_PE; 
	EXTI->RTSR 	|= EXTI_RTSR_TR10;  	
	EXTI->IMR 	|= EXTI_IMR_MR10; 		
	NVIC_EnableIRQ(EXTI15_10_IRQn);  		
}

void GPIO_init() {
    // PD12, PD14 - motor pwm
    GPIOD->MODER    |= (2 << MOT_PWM1*2) | (2 << MOT_PWM2*2);               // alternative function
    GPIOD->OTYPER   &= ~((1 << MOT_PWM1) | (1 << MOT_PWM2));                // push-pull
    GPIOD->OSPEEDR  |= (3 << MOT_PWM1*2) | (3 << MOT_PWM2*2);               // high speed
    GPIOD->PUPDR    |= (1 << MOT_PWM1*2) | (1 << MOT_PWM2*2);               // pull-up
    GPIOD->AFR[1]   |= (2 << (MOT_PWM1 - 8)*4) | (2 << (MOT_PWM2 - 8)*4);   // AF2
    

    
    // motor frequency calculation
//    GPIOE->MODER 	&= ~((3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2));          // input
//    GPIOE->OTYPER	&= ~((1 << MOT_FREQ1) | (1 << MOT_FREQ2));              // push-pull
//	GPIOE->OSPEEDR 	|= (3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2);             // high speed 
//    GPIOE->PUPDR    |= (1 << MOT_FREQ1*2) | (1 << MOT_FREQ2*2);             // pull-up
//    
//    // servo pwm control
//    GPIOB->MODER    |= 2 << SERV_PWM*2;                                     // alternative function
//    GPIOB->OTYPER   &= ~(1 << SERV_PWM);                                    // push-pull
//    GPIOB->OSPEEDR  |= 3 << SERV_PWM*2;                                     // high speed
//    GPIOB->PUPDR    |= 1 << SERV_PWM*2;                                     // pull-up
//    GPIOB->AFR[1]   |= 9 << (SERV_PWM - 8)*4;                               // AF9 - TIM12CH1
//    
//    GPIOB->MODER    |= 1 << 10*2;
//    GPIOB->OTYPER   &= ~(1 << 10);
//    GPIOB->OSPEEDR  |= 3 << 10*2;
//    GPIOB->PUPDR    |= 1 << 10*2;
}

void TIMERS_init() {
	pwm = 2000;
	TIM4->PSC 		= 7;	//PWM timer		// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 100kHz
	TIM4->ARR 		= 10000;			// Number of ticks to get overflow
	TIM4->CCR1 	= pwm;		//first motor		// CCy and ARR ratio set duty cycle of pwm
	TIM4->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // Set pwm mode 1
	TIM4->CCER 	|= TIM_CCER_CC1E; // Enable TIM3 capture/compre register 3
    
    TIM4->CCR3 	= pwm;		//second one		// CCy and ARR ratio set duty cycle of pwm
	TIM4->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // Set pwm mode 1
	TIM4->CCER 	|= TIM_CCER_CC3E; // Enable TIM3 capture/compre register 3
	
	TIM4->CR1 = TIM_CR1_CEN;		// Enable timer
    
    TIM3->PSC = 7;
    TIM3->ARR = 5000;
    TIM3->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);
    
    TIM3->CR1 = TIM_CR1_CEN;
    
    TIM2->PSC = 799;                   //button press timer
    TIM2->ARR = 2000;					// 50 ms to avoid button rattle
	TIM2->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM5->DIER |= 1;					// timer for frequency calculation
	//NVIC_EnableIRQ(TIM5_IRQn);
    
    TIM5->CR1 = TIM_CR1_CEN;
    
    TIM9->DIER |= 1;					// timer for frequency calculation
    
    TIM9->CR1 = TIM_CR1_CEN;
    
    TIM7->PSC = 7;
    TIM7->ARR = 5000;
    TIM7->DIER |= 1;
    NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    TIM12->PSC      = 7;
    TIM12->ARR      = 20000;
    TIM12->CCR1     = 1472;
    TIM12->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM12->CCER     |= TIM_CCER_CC1E;
    
    TIM12->CR1      |= TIM_CR1_CEN;
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


uint8_t status = 0;
int timer = 0;
int timer_1000 = 2000;
int timer_2000 = 2000;
int pwm_step = 5;

void TIM3_IRQHandler() { //moving pwm from 2 to 1ms in the start
    TIM3->SR &= ~TIM_SR_UIF;
    //if ( timer > 0) timer--;
    /*else if (TIM4->CCR1 == 1000 || TIM4->CCR3 == 1000) {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM4->CCR1 = 1000;
        TIM4->CCR3 = 1000;
        ENGRDY = 1;
    }
    else {
        TIM4->CCR1--;
        TIM4->CCR3--;
    } */
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
   



void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    SendTelemetry();
}

uint8_t KOEFF_RX = 0;
uint8_t KOEFF2_RX = 0;
double koeff = 0;
double my_pow = 0;
double d_st = 0;
int k = 0;

uint8_t value;
int main() {
    RCC_Init();
    ADXL345_Init();
    ADXL345_Calibr();
    Accel_EXTI_Init();
    USART_init();
    GPIO_init();
    TIMERS_init(); 
    while(ENGRDY != 1) {};
    
    TIM7->CR1 |= TIM_CR1_CEN;
    while(1) {
        //SendTelemetry();
    }
}
