#include "stm32f4xx.h"
#include "main.h"
#include "leds.h"
#include "time.h"
#include "dmp.h"
#include "mpu9250_regs.h"

VLG_InertialSensor inertial_sensor;
VLG_Compass compass;

void RCC_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;    
}

uint8_t res;
int main() {
    RCC_Init();
    SysTick_Init();
    leds_init();
    
    if (inertial_sensor.init()) {
        // @TODO: error message
    }
//    if (compass.init()) {
//        // @TODO: error message
//    }
    
    //inertial_sensor.set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);
    res = inertial_sensor.dmp_load_firmware();
    res = inertial_sensor.dmp_set_fifo_rate(200);
    res = inertial_sensor.dmp_set_state(1);
    res = inertial_sensor.dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL);
    
    while (1) {
    }
}
