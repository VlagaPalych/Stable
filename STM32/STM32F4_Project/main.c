#include "stm32f4xx.h" 
#include "adxl345.h"
#include "telemetry.h"
#include "motors.h"
#include "processing.h"
#include "gyro.h"

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

uint8_t ENGRDY = 0;
uint8_t STABRDY = 0;
int pwm1 = 0;
int pwm2 = 0;
int COUNT1 = 0;
int COUNT2 = 0;

float F = 0;
float Kp = 3;
float Kd = 2;
float Ki = 0.001;
float angle = 0;
float angularVelocity = 0;

void RCC_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_USART2EN;
}

int main() {
    RCC_Init();
    
//    GPIOD->MODER |= 1 << (15 * 2);
//    GPIOD->OTYPER &= ~(1 << 15);
//    GPIOD->OSPEEDR |= 3UL << (15*2);
//    
//    ADXL345_Init();
//    ADXL345_Calibr();

//    Motors_Init();
//    USART_Init();
//    
//    while(ENGRDY != 1) {};
//        
////    angleAveragingOn = 1;
////    angVelAveragingOn = 1;
//    allocAveraging();    
//    Accel_EXTI_Init();
//    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    I2C1_Init();
    GYRO_Read(0x00);
    while (vals_index == 0) { __nop();}
    GYRO_Read(0x1d);
    while (vals_index == 0) { __nop();}
    GYRO_Read(0x1e);
    while(1) {

    }
}
