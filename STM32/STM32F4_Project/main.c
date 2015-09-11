#include "stm32f4xx.h" 
#include "adxl345.h"
#include "telemetry.h"
#include "motors.h"
#include "processing.h"
#include "gyro.h"

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

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
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_USART1EN;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN;
}

int main() {
    RCC_Init();   
    
    GPIOB->MODER |= 1 << 8*2;
    GPIOC->MODER |= 1;
    GPIOC->BSRRL |= 1;
    GPIOD->MODER = 1 << 15*2;
    
    ADXL345_Init();
    Accel_EXTI_Init();
    ADXL345_Calibr();
    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    Gyro_Init();
    Gyro_EXTI_Init();
    Gyro_Calibr();

    Motors_Init();
    USART_Init();
    
//    while((USART1->SR & USART_SR_TXE)==0)__NOP(); 
//    USART1->DR = 'A'; 
//    
//    while((USART1->SR & USART_SR_TXE)==0)__NOP(); 
//    USART1->DR = 'T'; 
    
//    while (1) {
//    while((USART1->SR & USART_SR_TXE)==0)__NOP(); 
//    USART1->DR = 'k'; 
////    send_to_uart('k');
////        send_to_uart('b');
////        send_to_uart('c');
//    }
    
//    send_to_uart('+');
//    send_to_uart('P');
//    send_to_uart('E');
    
    while(ENGRDY != 1) {};
        
//    Accel_EXTI_Init();
//    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
    //while(1) send_to_uart('1');
    //send_to_uart('T');    
    Processing_TIM_Init();
    
    while(1) {    
        if (doGyroProcess) {
            
            finalGX = lowpass(gxHistory, gxCurHistoryIndex, gyro_b, GYRO_FILTER_SIZE);
            doGyroProcess = 0;
            
        } 
        // Lowpass filtering of accelerations
        if (doAccelProcess) {
            
            finalAX = lowpass(axHistory, axCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE);
            finalAZ = lowpass(azHistory, azCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE); 
            doAccelProcess = 0;
        } 
        
        
    }
}
