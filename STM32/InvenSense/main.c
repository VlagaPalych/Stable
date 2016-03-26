#include "stm32f4xx.h" 
#include "mpu9250.h"
#include "dmp.h"
#include "extra_math.h"
#include "string.h"
#include "telemetry.h"
#include "spi.h"
#include "dma.h"

#include "mpl.h"
#include "quaternion_supervisor.h"
#include "fusion_9axis.h"
#include "fast_no_motion.h"
#include "gyro_tc.h"

extern float angleRate[3];
extern float accel[3];
extern float magField[3];
extern Quat orientation;
float euler[3];
float eulerRate[3];

uint8_t Message_Size = 0;

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;    
}


// < 65536 ms
void Delay_ms(uint16_t ms) {
    TIM6->PSC = 15999;
    TIM6->ARR = ms;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN)!=0);
}
// < 65536 us
void Delay_us(uint16_t us) {
    TIM6->PSC = 15;
    TIM6->ARR = us;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN)!=0);
}

#include "dmp.h"
extern uint8_t process;
int res = 0;
long gyro_st_bias[3];
long accel_st_bias[3];
extern MPU_Test *test;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

int main() {
    QUEST_Init();
    Message_Size = sizeof(Message);
    RCC_Init();

    SPI2_Init();
    res = MPU_SelectDevice(0);
    MPU_InitStructures();
    
    res = MPU_Init(NULL);
    
    res = inv_init_mpl();
    res = MPU_RunSelfTest(gyro_st_bias, accel_st_bias);
    
    accel_st_bias[0] = accel_st_bias[0] * (0xffff / 2 / 16) / 65536L; 
    accel_st_bias[1] = accel_st_bias[1] * (0xffff / 2 / 16) / 65536L; 
    accel_st_bias[2] = accel_st_bias[2] * (0xffff / 2 / 16) / 65536L; 
    gyro_st_bias[0] = gyro_st_bias[0] * (0xffff / 2 / 1000) / 65536L; 
    gyro_st_bias[1] = gyro_st_bias[1] * (0xffff / 2 / 1000) / 65536L; 
    gyro_st_bias[2] = gyro_st_bias[2] * (0xffff / 2 / 1000) / 65536L; 
    
    res = MPU_SetAccelBias(accel_st_bias);
    MPU_SetGyroBias(gyro_st_bias);
    
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    
    
    
    MPU_DMA_Init();
    MPU_EXTI_Init();
    
    res = MPU_SetSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);
//    res = MPU_ConfigureFIFO(INV_XYZ_ACCEL | INV_XYZ_GYRO);

    DMP_SelectDevice(0);
    DMP_InitStructures();
    res = DMP_LoadMotionDriverFirmware();
    res = DMP_SetFIFORate(200);
    res = MPU_SetDMPState(1);
    res = DMP_EnableFeature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL);
     
    USART1_Init();
    Telemetry_DMA_Init();

    while (1) {
        if (process) {
            process = 0;
            QUEST();          
            
            degrees_to_radians(angleRate);
            memcpy(zk_data, angleRate, VECT_SIZE*sizeof(float));
            zk_data[3] = orientation.w;
            memcpy(zk_data+VECT_SIZE+1, orientation.v, VECT_SIZE*sizeof(float));
            
            Kalman();
            orientation.w = x_aposteriori_data[3];
            memcpy(orientation.v, x_aposteriori_data+VECT_SIZE+1, VECT_SIZE*sizeof(float));
            //quaternionToEuler(&orientation, &euler[0], &euler[1], &euler[2]);
            Quat_ToEuler(orientation, euler);
            
            memcpy(angleRate, x_aposteriori_data, VECT_SIZE*sizeof(float));
            angleRate_to_eulerRate(angleRate, euler, eulerRate);
            
            radians_to_degrees(euler);
            radians_to_degrees(eulerRate);
            
            memcpy(message.euler, euler, 3*sizeof(float));
            Telemetry_Send(&message);

        }
    }
}
