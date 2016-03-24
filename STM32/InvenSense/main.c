#include "stm32f4xx.h" 
#include "mpu9250.h"
#include "dmp.h"
#include "extra_math.h"
#include "string.h"
#include "telemetry.h"
#include "spi.h"
#include "dma.h"

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

extern uint8_t process;
int res = 0;
int main() {
//    QUEST_Init();
//    Message_Size = sizeof(Message);
    RCC_Init();
//    GPIOA->MODER &= ~(3 << 15*2);
//    GPIOA->MODER |= 1 << 15*2;

    SPI2_Init();
    res = MPU_SelectDevice(0);
    MPU_InitStructures();
    res = MPU_Init(NULL);

    
    MPU_DMA_Init();
    MPU_EXTI_Init();
    
    res = MPU_SetSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
    res = MPU_ConfigureFIFO(INV_XYZ_ACCEL | INV_XYZ_GYRO);
 
//    MPU_Init();
//    Mag_Init();

////    MPU_LoadFirmware(dmp_memory, DMP_CODE_SIZE, startAddress);
////    DMP_EnableFeature(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO);
////    DMP_SetFIFORate(200);
////    MPU_SetDMPState(1);

//    MPU_DMA_Init();
//    Delay_ms(100);
//    MPU_EXTI_Init();
//    EXTI->SWIER |= EXTI_SWIER_SWIER1;
    
//    USART1_Init();
//    Telemetry_DMA_Init();

    while (1) {
//        if (process) {
//            process = 0;
//            QUEST();          
//            
//            degrees_to_radians(angleRate);
//            memcpy(zk_data, angleRate, VECT_SIZE*sizeof(float));
//            zk_data[3] = orientation.w;
//            memcpy(zk_data+VECT_SIZE+1, orientation.v, VECT_SIZE*sizeof(float));
//            
//            Kalman();
//            orientation.w = x_aposteriori_data[3];
//            memcpy(orientation.v, x_aposteriori_data+VECT_SIZE+1, VECT_SIZE*sizeof(float));
//            Quat_ToEuler(orientation, euler);
//            
//            memcpy(angleRate, x_aposteriori_data, VECT_SIZE*sizeof(float));
//            angleRate_to_eulerRate(angleRate, euler, eulerRate);
//            
//            radians_to_degrees(euler);
//            radians_to_degrees(eulerRate);
//            
//            memcpy(message.euler, euler, 3*sizeof(float));
//            Telemetry_Send(&message);

//        }
    }
}
