#include "stm32f4xx.h" 
#include "adxl345.h"
#include "telemetry.h"
#include "motors.h"
#include "processing.h"
#include "gyro.h"
#include "adxrs453.h"
#include "adxrs290.h"

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

uint8_t ENGRDY = 0;

int pwm = 0;
int pwm1 = 0;
int pwm2 = 0;
int COUNT1 = 0;
int COUNT2 = 0;

float F = 0;
float Kp = 2;
float Kd = 0.01;
float Ki = 0;
float angle = 0;
float angularVelocity = 0;
float filterScale=0.9;

void RCC_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_USART1EN;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | 
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_SPI3EN;
}

int main() {
    RCC_Init();   
    
    GPIOB->MODER |= 0 << 8*2;
    GPIOC->MODER |= 1;
    GPIOC->BSRRL |= 1;
    
    Accel_VDD_Init();
    Accel_NSS_Init();
    Accel_NSS_High();
    
    ARS1_VDD_Init();
    ARS1_NSS_Init();
    ARS1_NSS_High();
    
    ARS2_VDD_Init();
    ARS2_NSS_Init(); 
    ARS2_NSS_High();
    
    SPI2_Init();
//    ADXL345_Init();
//    Accel_EXTI_Init();
//    ADXL345_Calibr();
//    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
//    SPI3_Init();
//    ADXRS_TIM_Init();
//    ADXRS_Calibr();
//     
    ARS1_Init();
//    ARS1_EXTI_Init();
//    EXTI->SWIER |= EXTI_SWIER_SWIER10;
    
//    ARS2_Init();
//    ARS2_EXTI_Init();
//    EXTI->SWIER |= EXTI_SWIER_SWIER15;


//    Motors_Init();
//    USART_Init();
////    
//    while(ENGRDY != 1) {};
////          
//    Processing_TIM_Init();
    
    while(1) { 
        ARS1_NSS_Low();
        SPI2_Read(0x10);
        ARS1_NSS_High();
//        if (doAdxrsProcess) {
//            doAdxrsProcess = 0;
//            filteredVel = lowpass(adxrsHistory, adxrsCurHistoryIndex, adxrs_b, ADXRS_FILTER_SIZE);
//            
//            
//            if (adxrs_CalibrationOn) {
//                adxrs_Sum += filteredVel;
//            
//                if (adxrs_CalibrIndex == 0) {
//                    adxrs_Sum = 0;
//                }
//                adxrs_CalibrIndex++;
//                
//                if (adxrs_CalibrIndex == adxrs_CalibrNumber) {
//                    adxrs_Offset = adxrs_Sum / adxrs_CalibrNumber;
//                    adxrs_CalibrationOn = 0;
//                    angle = 0;
//                }
//            }
//            filteredVel -= adxrs_Offset;
//            
//        }
//        // Lowpass filtering of accelerations
//        if (doAccelProcess) {    
//            filteredAX = filterScale * lowpass(axHistory, axCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE);
//            filteredAZ = filterScale * lowpass(azHistory, azCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE); 
//            doAccelProcess = 0;
//        } 
        
        
    }
}
