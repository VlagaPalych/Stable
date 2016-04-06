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
#include "compass_vec_cal.h"
#include "mag_disturb.h"
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "ml_math_func.h"

#include "main.h"

//****************************************
uint8_t algorithm = BIT_MPL | BIT_DMP | BIT_MINE;
uint8_t new_data = 0;
uint8_t meas1 = BIT_MPL | BIT_DMP | BIT_MINE;

float mpl_euler[3];
float dmp_euler[3];
float mine_euler[3];

long mpl_euler_fixed[3];
long mpl_accel_fixed[3];
float mpl_accel[3];
long mpl_gyro_fixed[3];
float mpl_gyro[3];
long mpl_compass_fixed[3];
float mpl_compass[3];
long mpl_quat_fixed[4];
Quat mpl_orient;
long mpl_heading_fixed;
float mpl_heading;

long dmp_quat_data[4];
Quat dmp_orient;

float mine_accel[3];
float mine_gyro[3];
float mine_compass[3];
float compass_bias[3] = {4.6125, 4.3375, 6.4125};
float compass_scale[3] = {0.9814, 1.0393, 0.9814};
Quat mine_orient;
float mine_quest_euler[3];
extern float w1[3];
extern float w2[3];
extern float v1[3];
extern float v2[3];

#define CALIBR_COUNT 400
uint16_t meas1_count = CALIBR_COUNT;
float mine_accel_sum[3] = {0, 0, 0};
float mine_compass_sum[3] = {0, 0, 0};

//****************************************

float eulerRate[3];

uint16_t pwm1;
uint16_t pwm2;
uint16_t freq1;
uint64_t freq2;
float F;

uint8_t Message_Size = 0;

#define TICK_FREQ (1000u)

volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;
unsigned long idle_time=0;
extern uint32_t SystemCoreClock; //16000000=16Mhz (original value)

void SysTick_Init() {
    SystemCoreClockUpdate();                               // Update the system clock variable (might not have been set before)
                                                           // With this call, the core clock gets set to 56MHz
    
	if (SysTick_Config (SystemCoreClock / TICK_FREQ)) {     // Setup SysTick Timer for 1 msec interrupts
		while (1);                                          // Handle Error
	}
}

void mdelay(unsigned long nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

int get_tick_count(unsigned long *count)
{
    count[0] = g_ul_ms_ticks;
	return 0;
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}

void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}

void SysTick_Handler(void)
{
  	TimingDelay_Decrement();
    TimeStamp_Increment();
}

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


int res = 0;
long gyro_st_bias[3];
long accel_st_bias[3];
extern MPU_Test *test;
extern MPU_State *st;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

//#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
//#define COMPASS_ENABLED 1
//#elif defined AK8975_SECONDARY
//static struct platform_data_s compass_pdata = {
//    .orientation = {-1, 0, 0,
//                     0, 1, 0,
//                     0, 0,-1}
//};
//#define COMPASS_ENABLED 1
//#elif defined AK8963_SECONDARY
//static struct platform_data_s compass_pdata = {
//    .orientation = {-1, 0, 0,
//                     0,-1, 0,
//                     0, 0, 1}
//};
//#define COMPASS_ENABLED 1
//#endif

int8_t accuracy;
inv_time_t read_timestamp;
uint8_t j = 0;
long tmp;
uint16_t    gyro_fsr;
uint8_t     accel_fsr;
uint16_t    compass_fsr;
uint8_t     self_test_res = 0;

int main() {
    QUEST_Init();
    RCC_Init();
    SysTick_Init();
    
    GPIOA->MODER &= ~(3 << 15*2);
    GPIOA->MODER |= 1 << 15*2;

    SPI2_Init();
    res = MPU_SelectDevice(0);
    MPU_InitStructures();
    
    res = MPU_Init(NULL);
    
    res = inv_init_mpl();
    self_test_res = MPU_RunSelfTest(gyro_st_bias, accel_st_bias);
    
//    if (self_test_res & 0x03) {
//        for (j = 0; j < 3; j++) {
//            accel_st_bias[j] = accel_st_bias[j] * (0xffff / 2 / 16) / 65536L;
//            gyro_st_bias[j] = gyro_st_bias[j] * (0xffff / 2 / 1000) / 65536L;
//        }
//        res = MPU_SetAccelBias(accel_st_bias);
//        MPU_SetGyroBias(gyro_st_bias);
//        res = MPU_SetAccelBias(accel_st_bias);
//        MPU_SetGyroBias(gyro_st_bias);
//    } 
    // TODO: handle error 
    
    MPU_SetAccelFsr(2);
    MPU_SetGyroFsr(2000);
//    if (self_test_res & 0x03) {
//        for (j = 0; j < 3; j++) {
//            accel_st_bias[j] = accel_st_bias[j] / (0xffff / 2 / 16) * (0xffff / 2 / 2) * 65536L;
//            gyro_st_bias[j] = gyro_st_bias[j] / (0xffff / 2 / 1000) * (0xffff / 2 / 2000) * 65536L;
//        }
//        inv_set_accel_bias(accel_st_bias, 3);
//        inv_set_gyro_bias(gyro_st_bias, 3);
//    }
    
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_9x_fusion_use_timestamps(1);
    inv_enable_fast_nomot();
    //inv_enable_gyro_tc();
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
    
    inv_enable_eMPL_outputs();
    res = inv_start_mpl();
    
    
    MPU_DMA_Init();
    MPU_EXTI_Init();
    
    res = MPU_SetSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);
//    res = MPU_ConfigureFIFO(INV_XYZ_ACCEL | INV_XYZ_GYRO);

    MPU_GetGyroFsr(&gyro_fsr);
    MPU_GetAccelFsr(&accel_fsr);

    inv_set_gyro_sample_rate(1000000L / st->chip_cfg.sample_rate);
    inv_set_accel_sample_rate(1000000L / st->chip_cfg.sample_rate);
    inv_set_compass_sample_rate(1000000L / st->chip_cfg.sample_rate);
    
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)st->hw->compass_fsr<<15);

    DMP_SelectDevice(0);
    DMP_InitStructures();
    res = DMP_LoadMotionDriverFirmware();
    res = DMP_SetFIFORate(200);
    res = MPU_SetDMPState(1);
    res = DMP_EnableFeature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL);
     
    USART1_Init();
    Telemetry_DMA_Init();

    while (1) {
        if (new_data & BIT_MPL) {
            new_data &= ~BIT_MPL;

//            inv_execute_on_data();
//            
//            if (inv_get_sensor_type_accel(mpl_accel_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                for (j = 0; j < 3; j++) {
//                    mpl_accel[j] = (float)mpl_accel_fixed[j] / 65536.0f;
//                }
//            }
//            if (inv_get_sensor_type_gyro(mpl_gyro_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                for (j = 0; j < 3; j++) {
//                    mpl_gyro[j] = (float)mpl_gyro_fixed[j] / 65536.0f;
//                }
//            }
//            if (inv_get_sensor_type_compass(mpl_compass_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                for (j = 0; j < 3; j++) {
//                    mpl_compass[j] = (float)mpl_compass_fixed[j] / 65536.0f;
//                }
//            }
//            if (inv_get_sensor_type_quat(mpl_quat_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                mpl_orient.w = mpl_quat_fixed[0] / QUAT_SENS;
//                for (j = 0; j < 3; j++) {
//                    mpl_orient.v[j] = (float)mpl_quat_fixed[j+1] / QUAT_SENS;
//                }
//            }
//            if (inv_get_sensor_type_euler(mpl_euler_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                for (j = 0; j < 3; j++) {
//                    mpl_euler[j] = (float)mpl_euler_fixed[j] / 65536.0f;
//                }
//            }
//            if (inv_get_sensor_type_heading(&mpl_heading_fixed, &accuracy, (inv_time_t*)&read_timestamp)) {
//                mpl_heading = (float)mpl_heading_fixed / 65536.0f;
//            }                
        }
        if (new_data & BIT_DMP) {
            new_data &= ~BIT_DMP;
            
//            dmp_orient.w = dmp_quat_data[0] / QUAT_SENS;
//            for (j = 0; j < 3; j++) {
//                dmp_orient.v[j] = dmp_quat_data[j+1] / QUAT_SENS;
//            }
//            Quat_ToEuler(dmp_orient, dmp_euler);
//            radians_to_degrees(dmp_euler);
        }
        if (new_data & BIT_MINE) {
            new_data &= ~BIT_MINE;
            
            if (meas1 & BIT_MINE) {
                if (meas1_count == 0) {
                    meas1 &= ~BIT_MINE;
                    
                    for (j = 0; j < VECT_SIZE; j++) {
                        w1[j] = mine_accel_sum[j] / CALIBR_COUNT;
                        w2[j] = mine_compass_sum[j] / CALIBR_COUNT;
                    }

                    Vect_Norm(w1);
                    Vect_Norm(w2);
                } else {
                    for (j = 0; j < VECT_SIZE; j++) {
                        mine_accel_sum[j] += mine_accel[j];
                        mine_compass_sum[j] += mine_compass[j];
                    }
                    meas1_count--;
                }
            } else {
                memcpy(v1, mine_accel, VECT_SIZE*sizeof(float));
                memcpy(v2, mine_compass, VECT_SIZE*sizeof(float));
                
                Vect_Norm(v1);
                Vect_Norm(v2);
                
                QUEST();
                Quat_ToEuler(mine_orient, mine_quest_euler);
                radians_to_degrees(mine_quest_euler);
                memcpy(mine_euler, mine_quest_euler, VECT_SIZE*sizeof(float));
                
                degrees_to_radians(mine_gyro);
                memcpy(zk_data, mine_gyro, VECT_SIZE*sizeof(float));
                zk_data[3] = mine_orient.w;
                memcpy(zk_data+VECT_SIZE+1, mine_orient.v, VECT_SIZE*sizeof(float));
                
                Kalman();
                mine_orient.w = x_aposteriori_data[3];
                memcpy(mine_orient.v, x_aposteriori_data+VECT_SIZE+1, VECT_SIZE*sizeof(float));
                Quat_ToEuler(mine_orient, mine_euler);
                
                memcpy(mine_gyro, x_aposteriori_data, VECT_SIZE*sizeof(float));
                angleRate_to_eulerRate(mine_gyro, mine_euler, eulerRate);
                
                radians_to_degrees(mine_euler);
                radians_to_degrees(eulerRate);
                
                memcpy(dmp_euler, mine_quest_euler, VECT_SIZE*sizeof(float));
//                F = mine_orient.w;
//                
//                memcpy(dmp_euler, mine_gyro, VECT_SIZE*sizeof(float));
                Telemetry_Send();
            }
        }
    }
}
