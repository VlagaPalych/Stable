#include "stm32f4xx.h" 
#include "adxl345.h"
#include "telemetry.h"
#include "motors.h"
#include "processing.h"
#include "gyro.h"
#include "adxrs453.h"
#include "adxrs290.h"
#include "quaternion.h"

#include "filters.h"

/*

Main tacting frequency is 64 MHz
AHB         64 MHz
APB1        32 MHz
APB2        64 MHz
APB1 timers 64 MHz


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

TIM7            - processing timer                              0xff

*/

Quat orient = {{1, 0, 0}, 0};

float angleRate[3];
float angle[3] = {0, 0, 0};

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

void checkCalibrationFinish() {
    if ((adxrs_CalibrationOn /*| ars_calibrationOn[0] | ars_calibrationOn[1]*/ | accelCalibrationOn) == 0) {
        Processing_TIM_Init();
    }
}

//void nelder_mead(float (*f)(float *, uint8_t), uint8_t n, float *x_init, float *step, float e, float *x_min);

//float rosenbrock(float *x, uint8_t n) {
//    return (1 - x[0])*(1 - x[0]) + 10*(x[1] - x[0]*x[0])*(x[1] - x[0]*x[0]);
//}

//float x_init[2] = {0, 0};
//float step[2] = {2, 2};
//float x_min[2];



int main() {
    uint8_t i = 0, j = 0;
    
    RCC_Init();   
    
    //nelder_mead(rosenbrock, 2, x_init, step, 1e-6, x_min);
    
    GPIOB->MODER |= 0 << 8*2;
    GPIOC->MODER |= 1;
    GPIOC->BSRRL |= 1;
    GPIOD->MODER |= 1 << 15*2;
    GPIOA->MODER |= 1 << 2*2;
    
    Message_Size = sizeof(Message);
    // Filters initialization
    // Accel
    for (i = 0; i < 3; i++) {
        arm_fir_decimate_init_f32(&accel_lpf[i], ACCEL_FILTER_SIZE, ACCEL_DECIMATION, accel_lpf_coeffs, accel_lpf_state[i], ACCEL_DECIMATION);
    }
    // ADXRS453
    arm_fir_decimate_init_f32(&adxrs453_lpf, ADXRS453_FILTER_SIZE, ADXRS453_DECIMATION, adxrs453_lpf_coeffs, adxrs453_lpf_state, ADXRS453_DECIMATION);
    
    
    Accel_VDD_Init();
    Accel_NSS_Init();
    Accel_NSS_High();
    
    for (i = 0; i < ADXRS290_NUMBER; i++) {
        ADXRS290_NSS_Init(i);
        ADXRS290_NSS_High(i);
    }
    
    SPI2_Init();
    
    ADXL345_Init();
    Accel_EXTI_Init();
    //ADXL345_Calibr();

//    for (i = 0; i < 1; i++) {
//        ADXRS290_Init(i);
//        ADXRS290_EXTI_Init(i);
        //ADXRS290_Calibr(i);
//        arm_fir_decimate_init_f32(&adxrs290_lpf[i], ADXRS290_NUM_TAPS, ADXRS290_DECIMATION, adxrs290_lpf_cf, ars_state[i], ADXRS290_DECIMATION); 
//    }
//    ADXRS290_Init(2);
//    ADXRS290_EXTI_Init(2);
//    ADXRS290_Calibr(2);
//    
    SPI3_Init();
    ADXRS_TIM_Init();
//    ADXRS_Calibr();
     
    EXTI->SWIER |= EXTI_SWIER_SWIER1;

//    Motors_Init();
    USART_Init();
    Processing_TIM_Init();

//    while(ENGRDY != 1) {};   

    while(1) { 
        
        if (doAdxrsProcess) {
            doAdxrsProcess = 0;
            filtered_arz = lowpass(adxrsHistory, adxrsCurHistoryIndex, adxrs_b, ADXRS_FILTER_SIZE);
            arm_fir_decimate_f32(&adxrs453_lpf, adxrs453_history + adxrs453_history_filter_index - ADXRS453_FILTER_SIZE, &filtered_arz, ADXRS453_DECIMATION);
                     
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
//                    checkCalibrationFinish();
//                }
//            }
//            filteredVel -= adxrs_Offset;
        }
//        // Lowpass filtering of accelerations
        if (doAccelProcess) { 
            GPIOD->BSRRL |= 1 << 15;
            for (i = 0; i < 3; i++) {
                //filtered_a[i] = filterScale * lowpass(accel_history[i], accel_curHistoryIndex[i], accel_b, ACCEL_FILTER_SIZE);
                arm_fir_decimate_f32(&accel_lpf[i], accel_history[i] + accel_history_filter_index - ACCEL_FILTER_SIZE, &filtered_a[i], ACCEL_DECIMATION);
            }
            doAccelProcess = 0;
            GPIOD->BSRRH |= 1 << 15;
        } 
      
        
//        for (i = 0; i < 1; i++) {
//            if (ars_doProcess[i]) {
//                ars_doProcess[i] = 0;
//                GPIOA->BSRRL |= 1 << 2;
//                for (j = 0; j < ADXRS290_DATA_SIZE-1; j++) {
//                    //ars_filteredData[i][j] = ars_termoData[i][j]; 
//                    //ars_filteredData[i][j] = lowpass(ars_history[i][j], ars_curHistoryIndex[i], adxrs290_filterCfs, ADXRS290_FILTER_SIZE);
//                    arm_fir_decimate_f32(&adxrs290_lpf[j], &ars_history[i][j][ars_curHistoryIndex[i]-ADXRS290_NUM_TAPS], &ars_filteredData[i][j], ADXRS290_DECIMATION);
//                }
                
//                if (ars_calibrationOn[i]) {
//                    for (j = 0; j < ADXRS290_DATA_SIZE-1; j++) {
//                        ars_sum[i][j] += ars_filteredData[i][j];
//                    }
//                    ars_calibrIndex[i]++;          
//                    if (ars_calibrIndex[i] == ars_calibrNumber[i]) {
//                        for (j = 0; j < ADXRS290_DATA_SIZE-1; j++) {
//                            ars_offset[i][j] = ars_sum[i][j] / ars_calibrNumber[i];
//                        }
//                        ars_calibrationOn[i] = 0;
//                        checkCalibrationFinish();
//                    }
//                }
//                
//                for (j = 0; j < ADXRS290_DATA_SIZE-1; j++) {
//                    ars_filteredData[i][j] -= ars_offset[i][j];
//                }
//                GPIOA->BSRRH |= 1 << 2;
//            }
//        }
        
    }
}
