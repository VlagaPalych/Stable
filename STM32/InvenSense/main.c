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
#include "arm_math.h"

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


#define BUTTON_PIN ((uint8_t)0) // PA
uint8_t mag_calibr_on = 0;
uint8_t mag_calibrated = 0;
#define RED_LED ((uint8_t)14) // PD

void turn_red_led(uint8_t on) {
    if (on) {
        GPIOD->BSRRL |= 1 << RED_LED;
    } else {
        GPIOD->BSRRH |= 1 << RED_LED;
    }
}

void compass_calibr_timer_init(uint8_t enable) {
    if (enable) {
        GPIOD->MODER &= ~(3 << 12*2);
        GPIOD->MODER |= 1 << 12*2;
        TIM2->PSC = 15999;
        TIM2->ARR = 10;
        TIM2->DIER |= TIM_DIER_UIE;
        NVIC_SetPriority(TIM2_IRQn, 0x02);
        NVIC_EnableIRQ(TIM2_IRQn);
        TIM2->CR1 |= TIM_CR1_CEN;
    } else {
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
        NVIC_DisableIRQ(TIM2_IRQn);
        GPIOD->BSRRH |= 1 << 12;
    }
}

extern float mag_calibr_mat_data[VECT_SIZE*VECT_SIZE];
extern float mag_bias[VECT_SIZE];

uint8_t mag_calibr_raw_data[7];
#define MAG_CALIBR_MATRIX_ROW_NUMBER 1300
#define A_COLS 9
float mag_calibr_data[MAG_CALIBR_MATRIX_ROW_NUMBER*3];
float mag_calibr_tmp[3];
uint16_t mag_calibr_row = 0;
#define MAG_CALIBR_DIFF_THRESH      0.15f

float pA[MAG_CALIBR_MATRIX_ROW_NUMBER*A_COLS];
float pAt[MAG_CALIBR_MATRIX_ROW_NUMBER*A_COLS];
float pB[MAG_CALIBR_MATRIX_ROW_NUMBER];
float pAsys[A_COLS*A_COLS];
float pBsys[A_COLS];
float pAsysInv[A_COLS*A_COLS];
float pSol[A_COLS];
float pC[3*3];
float pD[3];
float pCInv[3*3];

arm_matrix_instance_f32 A, At, B;
arm_matrix_instance_f32 Asys = {A_COLS, A_COLS, pAsys};
arm_matrix_instance_f32 Bsys = {A_COLS, 1, pBsys};
arm_matrix_instance_f32 AsysInv = {A_COLS, A_COLS, pAsysInv};
arm_matrix_instance_f32 Sol = {A_COLS, 1, pSol};
arm_matrix_instance_f32 C = {3, 3, pC};
arm_matrix_instance_f32 D = {3, 1, pD};
arm_matrix_instance_f32 CInv = {3, 3, pCInv};
arm_matrix_instance_f32 mag_bias_mat = {3, 1, mag_bias};



float dist(float a[3], float b[3]) {
    float d[3], d2[3];
    arm_sub_f32(a, b, d, 3);
    arm_mult_f32(d, d, d2, 3);
    return Vect_Mod(d2);
}

void mag_process_fresh_calibr_data() {
    float dist_prev;
    MPU_Read(EXT_SENS_DATA_00, mag_calibr_raw_data, 7);
    compass_float_data(mag_calibr_raw_data, mag_calibr_tmp);

    if (mag_calibr_row == 0) {
        memcpy(&mag_calibr_data[mag_calibr_row*3], mag_calibr_tmp, VECT_SIZE*sizeof(float));
        mag_calibr_row = 1;
        memcpy(mpl_euler, mag_calibr_tmp, 3*sizeof(float));
        Telemetry_Send();
    } else {
        dist_prev = dist(mag_calibr_tmp, &mag_calibr_data[(mag_calibr_row-1)*3]);
        if (dist_prev > MAG_CALIBR_DIFF_THRESH * Vect_Mod(&mag_calibr_data[(mag_calibr_row-1)*3])) {
            memcpy(&mag_calibr_data[mag_calibr_row*3], mag_calibr_tmp, VECT_SIZE*sizeof(float));
            mag_calibr_row++;
            memcpy(mpl_euler, mag_calibr_tmp, 3*sizeof(float));
            Telemetry_Send();
        }
    }
}

extern uint8_t telemetry_on;
#define MAG_CROSS_SENS_THRESH 0.05
int mag_calibrate() {
    uint16_t i = 0;
    float x, y, z;
    float a2;
    float a, b, c, d, e, f;
    float cos_xy, cos_xz, cos_yz;
    float sin_xy, sin_xz, sin_yz;
    float tmp;
    
    arm_mat_init_f32(&A, mag_calibr_row, A_COLS, pA);
    arm_mat_init_f32(&At, A_COLS, mag_calibr_row, pAt);
    arm_mat_init_f32(&B, mag_calibr_row, 1, pB);
    
    for (i = 0; i < mag_calibr_row; i++) {
        x = mag_calibr_data[i*3 + 0];
        y = mag_calibr_data[i*3 + 1];
        z = mag_calibr_data[i*3 + 2];
        
        pA[i*A_COLS + 0] = x;
        pA[i*A_COLS + 1] = y;
        pA[i*A_COLS + 2] = z;
        pA[i*A_COLS + 3] = -y*y;
        pA[i*A_COLS + 4] = -z*z;
        pA[i*A_COLS + 5] = -x*y;
        pA[i*A_COLS + 6] = -x*z;
        pA[i*A_COLS + 7] = -y*z;
        pA[i*A_COLS + 8] = 1;
        
        pB[i] = x*x;
    }
    
    arm_mat_trans_f32(&A, &At);
    arm_mat_mult_f32(&At, &A, &Asys);
    arm_mat_mult_f32(&At, &B, &Bsys);
    
    arm_mat_inverse_f32(&Asys, &AsysInv);
    arm_mat_mult_f32(&AsysInv, &Bsys, &Sol);
    
    for (i = 3; i < 8; i++) {
        if (pSol[i] < 0) {
            pSol[i] = 1e-12;
        }
    }
    
    if ((pSol[5] > MAG_CROSS_SENS_THRESH) || 
        (pSol[6] > MAG_CROSS_SENS_THRESH) || 
        (pSol[7] > MAG_CROSS_SENS_THRESH)) {
            memcpy(mpl_euler, &pSol[0], 3*sizeof(float));
            memcpy(dmp_euler, &pSol[3], 3*sizeof(float));
            memcpy(mine_euler, &pSol[6], 3*sizeof(float));
            Telemetry_Send();
            return -1;
    }
    
    pC[0*3+0] = 2.0f;       pC[0*3+1] = pSol[5];      pC[0*3+2] = pSol[6];
    pC[1*3+0] = pSol[5];    pC[1*3+1] = 2*pSol[3];    pC[1*3+2] = pSol[7];
    pC[2*3+0] = pSol[6];    pC[2*3+2] = pSol[7];      pC[2*3+2] = 2*pSol[4];
    
    pD[0] = pSol[0];
    pD[1] = pSol[1];
    pD[2] = pSol[2];
    
    arm_mat_inverse_f32(&C, &CInv);
    arm_mat_mult_f32(&CInv, &D, &mag_bias_mat);
    
    a2 = pSol[8] +  mag_bias[0]*mag_bias[0] + 
                    pSol[3]*mag_bias[1]*mag_bias[1] +
                    pSol[4]*mag_bias[2]*mag_bias[2] +
                    pSol[5]*mag_bias[0]*mag_bias[1] +
                    pSol[6]*mag_bias[0]*mag_bias[2] +
                    pSol[7]*mag_bias[1]*mag_bias[2];
    
    arm_sqrt_f32(a2, &a);
    arm_sqrt_f32(a2 / pSol[3], &b);
    arm_sqrt_f32(a2 / pSol[4], &c);
    arm_sqrt_f32(a2 / pSol[5], &d);
    arm_sqrt_f32(a2 / pSol[6], &e);
    arm_sqrt_f32(a2 / pSol[7], &f);
    
    cos_xy = 2*a*b/d/d;
    cos_xz = 2*a*c/e/e;
    cos_yz = 2*b*c/f/f;
 
    arm_sqrt_f32(1 - cos_xy*cos_xy, &sin_xy);
    arm_sqrt_f32(1 - cos_xz*cos_xz, &sin_xz);
    arm_sqrt_f32(1 - cos_yz*cos_yz, &sin_yz);
    
    arm_sqrt_f32(1 - cos_xz*cos_xz - cos_yz*cos_yz*sin_xy*sin_xy, &tmp);
    
    mag_calibr_mat_data[0*3+0] = 1.0/a;     mag_calibr_mat_data[0*3+1] = cos_xy/b;      mag_calibr_mat_data[0*3+2] = cos_xz/c;
    mag_calibr_mat_data[1*3+0] = 0;         mag_calibr_mat_data[1*3+1] = sin_xy/b;      mag_calibr_mat_data[1*3+2] = cos_yz*sin_xy/c;
    mag_calibr_mat_data[2*3+0] = 0;         mag_calibr_mat_data[2*3+1] = 0;             mag_calibr_mat_data[2*3+2] = tmp/c;
    
    mag_calibr_save_to_flash();
    
    mag_calibrated = 1;
    mag_calibr_on = 0;
    
    memcpy(mpl_euler, &pSol[0], 3*sizeof(float));
    memcpy(dmp_euler, &pSol[3], 3*sizeof(float));
    memcpy(mine_euler, &pSol[6], 3*sizeof(float));
    Telemetry_Send();
    telemetry_on = 0;
    meas1 |= BIT_MINE;
    return 0;
}

#define MAG_CALIBR_FLASH_ADDR 0x080e0004

void mag_calibr_save_to_flash() {
    uint8_t i = 0;
    uint32_t *pInt, data;
    flash_unlock();
    flash_erase_sector(MAG_CALIBR_FLASH_ADDR);
    for (i = 0; i < 9; i++) {
        pInt = (uint32_t *)&mag_calibr_mat_data[i];
        data = pInt[0];
        flash_write(MAG_CALIBR_FLASH_ADDR + sizeof(float)*i, data); 
    }
    for (i = 0; i < 3; i++) {
        pInt = (uint32_t *)&mag_bias[i];
        data = pInt[0];
        flash_write(MAG_CALIBR_FLASH_ADDR + sizeof(float)*(i+9), data); 
    }
    flash_lock();
}

void mag_calibr_restore_from_flash() {
    uint8_t i = 0;
    uint32_t *pInt, data;
    flash_unlock();
    for (i = 0; i < 9; i++) {
        data = flash_read(MAG_CALIBR_FLASH_ADDR + sizeof(float)*i);
        if (data == 0xffffffff) {
            turn_red_led(1);
            return;
        }
        pInt = (uint32_t *)&mag_calibr_mat_data[i];
        pInt[0] = data;
    }
    for (i = 0; i < 3; i++) {
        data = flash_read(MAG_CALIBR_FLASH_ADDR + sizeof(float)*(i+9));
        pInt = (uint32_t *)&mag_bias[i];
        pInt[0] = data;
    }
    flash_lock();
    turn_red_led(0);
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        
        if (mag_calibr_row < MAG_CALIBR_MATRIX_ROW_NUMBER) {
            mag_process_fresh_calibr_data();
        } else {
            if (mag_calibrate()) {
                turn_red_led(1);
            };
        }
        
        GPIOD->ODR ^= 1 << 12;
    }
}

void Button_Init() {
    GPIOA->MODER &= ~(3 << BUTTON_PIN*2);
    GPIOA->OSPEEDR |= 3 << BUTTON_PIN*2;
    
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->RTSR 	|= EXTI_RTSR_TR0;  	    // rising events
    //EXTI->FTSR 	|= EXTI_FTSR_TR0;  	    
	EXTI->IMR 	|= EXTI_IMR_MR0; 		// we don't mask events on line 0
	NVIC_EnableIRQ(EXTI0_IRQn); 		// enable EXTI0 interrupt
}

#include "commands.h"

void EXTI0_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR0) {
		EXTI->PR = EXTI_PR_PR0;		    // Clear interrupt flag
	
        // to avoid rattle
        Delay_ms(25);
        if ((GPIOA->IDR & 1) == 0) {
            return;
        }
        
        if (!mag_calibr_on) {
            mag_calibr_on = 1;
            mag_calibr_row = 0;
            telemetry_on = 1;
            //MPU_SetCompassSampleRate(8);
            compass_calibr_timer_init(1);
        } else {
            compass_calibr_timer_init(0);
            //MPU_SetCompassSampleRate(100);
            if (mag_calibrate()) {
                turn_red_led(1);
            };
        }
    }
}


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
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;    
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

//*********************************
// Flash block

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
void flash_unlock(void) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}

// returns 1 when erase and write enabled
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);
}
 
// erases the sector address belongs to
int flash_erase_sector(uint32_t address) {
    uint8_t sector;
    if ((address < 0x08000000) || (address > 0x080fffff)) {
        return -1;
    }
    if (address < 0x08010000) {
        sector = (uint8_t)((address & 0xffff) / 0x4000);
    } else if (address < 0x08020000){
        sector = 4;
    } else {
        sector = (uint8_t)((address & 0xfffff) / 0x20000) + 4;
    }
    while(!flash_ready()); 
    
    FLASH->CR |= FLASH_CR_SER | (sector << 3);
    FLASH->CR |= FLASH_CR_STRT; 
   
    while(!flash_ready()); 
    FLASH->CR &= ~FLASH_CR_SER;
}

// erases ALL pages
void flash_erase_all_pages(void) {
    while(!flash_ready());
    FLASH->CR |= FLASH_CR_MER;  
    FLASH->CR |= FLASH_CR_STRT; 
    while(!flash_ready());
    FLASH->CR &= ~FLASH_CR_MER;
}

void flash_write(uint32_t address, uint32_t data) {
    while(!flash_ready());
    FLASH->CR |= FLASH_CR_PSIZE_1;
    FLASH->CR |= FLASH_CR_PG;
    
    *(__IO uint32_t*)address = data;
    while(!flash_ready());
    FLASH->CR &= ~FLASH_CR_PG;
}

uint32_t flash_read(uint32_t address) {
  return (*(__IO uint32_t*) address);
}

//*********************************

int8_t accuracy;
inv_time_t read_timestamp;
uint8_t j = 0;
long tmp;
uint16_t    gyro_fsr;
uint8_t     accel_fsr;
uint16_t    compass_fsr;
uint8_t     self_test_res = 0;
uint32_t flash_test;

#define ACCEL_DATASHEET_ERROR 0.00424f
float roll, pitch;
float roll_error, pitch_error;
float roll_atan_arg, pitch_atan_arg;
float accel_norm, gravity_norm;

float sqrf(float x) {
    return x*x;
}

int main() {
    QUEST_Init();
    RCC_Init();
    SysTick_Init();
    Button_Init();
    
    GPIOD->MODER &= ~(3 << RED_LED*2);
    GPIOD->MODER |= (1 << RED_LED*2);
    mag_calibr_restore_from_flash();
//    flash_unlock();
//    flash_erase_sector(0x080e0000);
//    flash_write(0x080e0000, 0x2101ff83);
//    flash_test = flash_read(0x080e0000);
//    flash_lock();
    
    GPIOA->MODER &= ~(3 << 15*2);
    GPIOA->MODER |= 1 << 15*2;

    SPI2_Init();
    res = MPU_SelectDevice(0);
    MPU_InitStructures();
    
    res = MPU_Init(NULL);
    
    res = inv_init_mpl();
    self_test_res = MPU_RunSelfTest(gyro_st_bias, accel_st_bias);
    
    if (self_test_res & 0x03) {
        for (j = 0; j < 3; j++) {
            accel_st_bias[j] = accel_st_bias[j] * (0xffff / 2 / 16) / 65536L;
            gyro_st_bias[j] = gyro_st_bias[j] * (0xffff / 2 / 1000) / 65536L;
        }
        res = MPU_SetAccelBias(accel_st_bias);
        MPU_SetGyroBias(gyro_st_bias);
        res = MPU_SetAccelBias(accel_st_bias);
        MPU_SetGyroBias(gyro_st_bias);
    } 
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
            
//            if (meas1 & BIT_MINE) {
//                if (meas1_count == 0) {
//                    meas1 &= ~BIT_MINE;
//                    
//                    for (j = 0; j < VECT_SIZE; j++) {
//                        w1[j] = mine_accel_sum[j] / CALIBR_COUNT;
//                        w2[j] = mine_compass_sum[j] / CALIBR_COUNT;
//                    }

//                    Vect_Norm(w1);
//                    Vect_Norm(w2);
//                } else {
//                    for (j = 0; j < VECT_SIZE; j++) {
//                        mine_accel_sum[j] += mine_accel[j];
//                        mine_compass_sum[j] += mine_compass[j];
//                    }
//                    meas1_count--;
//                }
//            } else {

                accel_norm = Vect_Mod(mine_accel);

                roll_atan_arg = mine_accel[1] / mine_accel[2];
                pitch_atan_arg = -mine_accel[0] / sqrtf(sqrf(mine_accel[1]) + sqrf(mine_accel[2]));
                
                roll = atan(roll_atan_arg);
                pitch = atan(pitch_atan_arg);
                
                if (accel_norm > 1.05) {
                
                roll_error = acosf(sqrtf( (1 - sqrf(mine_accel[0])) /
                            (sqrf(mine_accel[1]) + sqrf(mine_accel[2]))));
                             
                pitch_error = acosf(1 / accel_norm);
                } else {
                
                // static case errors
                roll_error = (1.0 / (1.0 + roll_atan_arg*roll_atan_arg)) *
                            (fabsf(1.0/mine_accel[2]) + fabsf(-mine_accel[1]*mine_accel[2]/mine_accel[1]))*ACCEL_DATASHEET_ERROR;
                            
                pitch_error = (1.0 / (1.0 + pitch_atan_arg*pitch_atan_arg)) * 
                              (fabsf(-1.0 / sqrtf(mine_accel[1]*mine_accel[1] + mine_accel[2]*mine_accel[2])) +
                               fabsf(pitch_atan_arg * 2 * mine_accel[1] * -0.5 / (mine_accel[1]*mine_accel[1] + mine_accel[2]*mine_accel[2])) + 
                               fabsf(pitch_atan_arg * 2 * mine_accel[2] * -0.5 / (mine_accel[1]*mine_accel[1] + mine_accel[2]*mine_accel[2]))) * ACCEL_DATASHEET_ERROR;
                }
                mine_euler[0] = roll;
                mine_euler[1] = pitch;
                
                mpl_euler[0] = roll_error;
                mpl_euler[1] = pitch_error;
                
//                memcpy(v1, mine_accel, VECT_SIZE*sizeof(float));
//                memcpy(v2, mine_compass, VECT_SIZE*sizeof(float));
//                
//                Vect_Norm(v1);
//                Vect_Norm(v2);
//                
//                QUEST();
//                Quat_ToEuler(mine_orient, mine_quest_euler);
//                radians_to_degrees(mine_quest_euler);
//                memcpy(mine_euler, mine_quest_euler, VECT_SIZE*sizeof(float));
//                
//                degrees_to_radians(mine_gyro);
//                memcpy(zk_data, mine_gyro, VECT_SIZE*sizeof(float));
//                zk_data[3] = mine_orient.w;
//                memcpy(zk_data+VECT_SIZE+1, mine_orient.v, VECT_SIZE*sizeof(float));
//                
//                Kalman();
//                mine_orient.w = x_aposteriori_data[3];
//                memcpy(mine_orient.v, x_aposteriori_data+VECT_SIZE+1, VECT_SIZE*sizeof(float));
//                Quat_ToEuler(mine_orient, mine_euler);
//                
//                memcpy(mine_gyro, x_aposteriori_data, VECT_SIZE*sizeof(float));
//                angleRate_to_eulerRate(mine_gyro, mine_euler, eulerRate);
                
//                radians_to_degrees(mine_euler);
//                radians_to_degrees(eulerRate);
                
//                memcpy(mpl_euler, mine_compass, VECT_SIZE*sizeof(float));
//                F = mine_orient.w;
//                
//                memcpy(dmp_euler, mine_gyro, VECT_SIZE*sizeof(float));
                Telemetry_Send();
           // }
        }
    }
}
