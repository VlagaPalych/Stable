#include "stm32f4xx.h" 
#include "adxl345.h"
#include "telemetry.h"
#include "motors.h"
#include "processing.h"
#include "gyro.h"
#include "adxrs453.h"
#include "adxrs290.h"

/*

Several interrupts and their priorities used in the program.

EXTI1           - DRDY interrupt from Accel                     0x06
EXTI15_10       - DRDY interrupt from ARS1 and ARS2             0x06

DMA1_Stream3    - reading from Accel finished                   0x05

EXTI2           - impulse interrupt from Motor1 (COUNT1)        0x01
EXTI4           - impulse interrupt from Motor2 (COUNT2)        0x01

USART1          - command received                              0x02

TIM3            - timer for turning motors on                   0x01

TIM2            - timer providing equidistance of samples       0x04
DMA1_Stream2    - reading from ARS3 finished                    0x03

*/

float angleRate[3];
float angle[3];

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

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
float filterScale=0.9;

void RCC_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_USART1EN;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | 
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_SPI3EN;
}

int main() {
    uint8_t i = 0;
    
    RCC_Init();   
    
    GPIOB->MODER |= 0 << 8*2;
    GPIOC->MODER |= 1;
    GPIOC->BSRRL |= 1;
    GPIOD->MODER |= 1 << 15*2;
    
    Accel_VDD_Init();
    ARS1_VDD_Init();
    ARS2_VDD_Init();
    
    Accel_NSS_Init();
    Accel_NSS_High();
    
    ARS1_NSS_Init();
    ARS1_NSS_High();
    
    ARS2_NSS_Init(); 
    ARS2_NSS_High();
    
    SPI2_Init();
    
    ADXL345_Init();
    Accel_EXTI_Init();
    ADXL345_Calibr();

    ARS1_Init();
    ARS1_EXTI_Init();
    ARS1_Calibr();
    
    ARS2_Init();
    ARS2_EXTI_Init();
    ARS2_Calibr();
//    
    SPI3_Init();
    ADXRS_TIM_Init();
    ADXRS_Calibr();
//  
    EXTI->SWIER |= EXTI_SWIER_SWIER1;

    Motors_Init();
    USART_Init();
    
    while(ENGRDY != 1) {};
          
    Processing_TIM_Init();    

    while(1) { 

        if (doAdxrsProcess) {
            GPIOD->BSRRL |= 1 << 15;  
            doAdxrsProcess = 0;
            filteredVel = lowpass(adxrsHistory, adxrsCurHistoryIndex, adxrs_b, ADXRS_FILTER_SIZE);
                     
            if (adxrs_CalibrationOn) {
                adxrs_Sum += filteredVel;
            
                if (adxrs_CalibrIndex == 0) {
                    adxrs_Sum = 0;
                }
                adxrs_CalibrIndex++;
                
                if (adxrs_CalibrIndex == adxrs_CalibrNumber) {
                    adxrs_Offset = adxrs_Sum / adxrs_CalibrNumber;
                    adxrs_CalibrationOn = 0;
                }
            }
            filteredVel -= adxrs_Offset;
            GPIOD->BSRRH |= 1 << 15; 
        }
        // Lowpass filtering of accelerations
        if (doAccelProcess) {   
            GPIOD->BSRRL |= 1 << 15;           
            filteredAX = filterScale * lowpass(axHistory, axCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE);
            filteredAZ = filterScale * lowpass(azHistory, azCurHistoryIndex, accel_b, ACCEL_FILTER_SIZE); 
            doAccelProcess = 0;
            GPIOD->BSRRH |= 1 << 15; 
        } 
        
        if (ars1_doProcess) {
            GPIOD->BSRRL |= 1 << 15; 
            ars1_doProcess = 0;

            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars1_filteredData[i] = ars1_data[i]; //lowpass(ars1_history[i], ars1_curHistoryIndex, adxrs290_filterCfs, ADXRS290_FILTER_SIZE);
            }
            
            if (ars1_calibrationOn) {
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars1_sum[i] += ars1_filteredData[i];
                }
                ars1_calibrIndex++;          
                if (ars1_calibrIndex == ars1_calibrNumber) {
                    for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                        ars1_offset[i] = ars1_sum[i] / ars1_calibrNumber;
                    }
                    ars1_calibrationOn = 0;
                }
            }
            
            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars1_filteredData[i] -= ars1_offset[i];
            }
            GPIOD->BSRRH |= 1 << 15; 
        }
      
        if (ars2_doProcess) {
            GPIOD->BSRRL |= 1 << 15; 
            ars2_doProcess = 0;

            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars2_filteredData[i] = ars2_data[i]; //lowpass(ars2_history[i], ars2_curHistoryIndex, adxrs290_filterCfs, ADXRS290_FILTER_SIZE);
            }
            
            if (ars2_calibrationOn) {
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars2_sum[i] += ars2_filteredData[i];
                }
                ars2_calibrIndex++;          
                if (ars2_calibrIndex == ars2_calibrNumber) {
                    for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                        ars2_offset[i] = ars2_sum[i] / ars2_calibrNumber;
                    }
                    ars2_calibrationOn = 0;
                }
            }
            
            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars2_filteredData[i] -= ars2_offset[i];
            }
            GPIOD->BSRRH |= 1 << 15; 
        }
    }
}
