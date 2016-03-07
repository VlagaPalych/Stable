#include "stm32f30x.h"
#include "accel_magne.h"
#include "extra_math.h"
#include "string.h"
#include "telemetry.h"
#include "arm_math.h"

uint8_t I2C1_SCL    = 6; // PB
uint8_t I2C1_SDA    = 7; // PB
uint8_t AM_INT1     = 4; // PE
uint8_t AM_INT2     = 5; // PE
uint8_t AM_DRDY     = 2; // PE;

#define ACCEL_ADDRESS 25

#define AM_CTRL_REG1_A          0x20
#define AM_CTRL_REG1_A_VALUE    0x57
#define AM_CTRL_REG3_A          0x22
#define AM_CTRL_REG3_A_VALUE    0x10
#define AM_CTRL_REG4_A          0x23
#define AM_CTRL_REG4_A_VALUE    0x38

#define ACCEL_SENSITIVITY   0.012f      // 12 mg/LSB

uint8_t accel_data[7];
float non_calibr_accel_data[3];
float accel[3];

#define MAG_ADDRESS 0x1e

#define AM_CRA_REG_M            0x00
#define AM_CRA_REG_M_VALUE      0x18
#define AM_MR_REG_M             0x02
#define AM_MR_REG_M_VALUE       0x00

#define MAG_SENSITIVITY     1100.0f       // 1100 LSB/Gauss

uint8_t mag_data[6];
float non_calibr_mag_data[3];
float magField[3];

extern float w1[VECT_SIZE], w2[VECT_SIZE];
extern float v1[VECT_SIZE], v2[VECT_SIZE];

uint8_t meas1 = 1;
uint8_t quest_run = 0;

float accel_trans_matrix_data[VECT_SIZE*VECT_SIZE] = {
    1.0160,     -0.0243,    -0.0229,
    -0.0004,    1.0046,     -0.0353,
    0.0119,     0.0115,     0.9749
};

float accel_bias[VECT_SIZE] = {
    -0.0038,
    0.0120,
    0.0340
};

arm_matrix_instance_f32 accel_trans_matrix = {VECT_SIZE, VECT_SIZE, accel_trans_matrix_data};
arm_matrix_instance_f32 non_calibr_accel = {VECT_SIZE, 1, non_calibr_accel_data};
arm_matrix_instance_f32 accel_mat = {VECT_SIZE, 1, accel};

float mag_trans_matrix_data[VECT_SIZE*VECT_SIZE] = {
    3.1804,     0.4338,     -0.1249,
    0.4379,     4.0072,     0.1038,
    0.2601,     -0.1797,    3.1062
};

float mag_bias[VECT_SIZE] = {
    0.0198, 
    -0.3726, 
    0.0481
};

arm_matrix_instance_f32 mag_trans_matrix = {VECT_SIZE, VECT_SIZE, mag_trans_matrix_data};
arm_matrix_instance_f32 non_calibr_mag = {VECT_SIZE, 1, non_calibr_mag_data};
arm_matrix_instance_f32 magField_mat = {VECT_SIZE, 1, magField};

void I2C1_SetDevice(device dvc) {
    I2C1->CR2 &= ~I2C_CR2_SADD;
    switch (dvc) {
        case AM_ACCEL:
            I2C1->CR2 |= ACCEL_ADDRESS << 1;
            break;
        case AM_MAG:
            I2C1->CR2 |= MAG_ADDRESS << 1;
            break;
    }
}

void I2C1_GPIO_Init() {
    GPIOB->MODER    |= (2 << I2C1_SCL*2) | (2 << I2C1_SDA*2);
    GPIOB->OTYPER   |= (1 << I2C1_SCL) | (1 << I2C1_SDA);
    GPIOB->OSPEEDR  |= (3 << I2C1_SCL*2) | (3 << I2C1_SDA*2);
    GPIOB->PUPDR    |= (1 << I2C1_SCL*2) | (1 << I2C1_SDA*2);
    GPIOB->AFR[0]   |= (4 << I2C1_SCL*4) | (4 << I2C1_SDA*4);
}

void I2C1_Init() {
    I2C1_GPIO_Init();
    
    I2C1->CR1 = 0;
    // PRESC = 0
    // SDADEl = 1
    // SCLDEL = 0
    // SCLL = 7
    // SCLH = 7
    I2C1->TIMINGR |= (1 << 16) | (7 << 8) | 7; 
    I2C1->CR1 |= I2C_CR1_PE;
}


uint8_t AM_SingleRead(device dvc, uint8_t address) {
    I2C1_SetDevice(dvc);
        
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;   // write
    I2C1->CR2 &= ~I2C_CR2_NBYTES;
    I2C1->CR2 |= 1 << 16;           // 1 byte
    I2C1->CR2 &= ~I2C_CR2_AUTOEND;  // for restart
    
    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = address;
    
    while ((I2C1->ISR & I2C_ISR_TC) == 0);
    
    I2C1->CR2 |= I2C_CR2_RD_WRN;    // read
    I2C1->CR2 |= 1 << 16;           // 1 byte
    
    I2C1->CR2 |= I2C_CR2_START;
    
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop
    
    while ((I2C1->ISR & I2C_ISR_RXNE) == 0);
    return (I2C1->RXDR);
}

void AM_MultiRead(device dvc, uint8_t address, uint8_t *data, uint8_t size) {
    I2C1_SetDevice(dvc);
    
    AM_DMA_Run(data, size);
    
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;   // write
    I2C1->CR2 &= ~I2C_CR2_NBYTES;
    I2C1->CR2 |= 1 << 16;           // 1 byte
    I2C1->CR2 &= ~I2C_CR2_AUTOEND;  // for restart
    
    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = (1 << 7) | address;
    
    while ((I2C1->ISR & I2C_ISR_TC) == 0);
    
    I2C1->CR2 |= I2C_CR2_RD_WRN;    // read
    I2C1->CR2 &= ~I2C_CR2_NBYTES;
    I2C1->CR2 |= size << 16;        // size bytes  
    I2C1->CR2 |= I2C_CR2_START;
    
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop
}

void AM_Write(device dvc, uint8_t address, uint8_t data) {
    I2C1_SetDevice(dvc);
    
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;   // write
    I2C1->CR2 &= ~I2C_CR2_NBYTES;
    I2C1->CR2 |= 2 << 16;           // 2 bytes
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop  

    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = address;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = data;
}

uint8_t am_test = 0;
void AM_Init() {
    I2C1_Init();

    // Accelerometer
    
    // normal mode, all axis on
    AM_Write(AM_ACCEL, AM_CTRL_REG1_A, AM_CTRL_REG1_A_VALUE); 
    am_test = AM_SingleRead(AM_ACCEL, AM_CTRL_REG1_A);
    
    // data ready interrupt on INT1
    AM_Write(AM_ACCEL, AM_CTRL_REG3_A, AM_CTRL_REG3_A_VALUE);
    am_test = AM_SingleRead(AM_ACCEL, AM_CTRL_REG3_A);
    
    // +- 16G, high resolution
    AM_Write(AM_ACCEL, AM_CTRL_REG4_A, AM_CTRL_REG4_A_VALUE);
    am_test = AM_SingleRead(AM_ACCEL, AM_CTRL_REG4_A);
    
    
    // Magnetometer
    
    // 75 Hz
    AM_Write(AM_MAG, AM_CRA_REG_M, AM_CRA_REG_M_VALUE);
    am_test = AM_SingleRead(AM_MAG, AM_CRA_REG_M);
    
    // Continuous-conversion mode
    AM_Write(AM_MAG, AM_MR_REG_M, AM_MR_REG_M_VALUE);
    am_test = AM_SingleRead(AM_MAG, AM_MR_REG_M);
    
    // Adding DMA
    I2C1->CR1 |= I2C_CR1_RXDMAEN;
    AM_DMA_Init();
}

void AM_EXTI_Init() {
    GPIOE->OSPEEDR |= 3 << AM_INT1*2;
    
    SYSCFG->EXTICR[1] |= SYSCFG_EXTIRCR_EXTI4_PE;
    
    EXTI->RTSR  |= EXTI_RTSR_TR4;       // rising
    EXTI->IMR   |= EXTI_IMR_MR4;        // non-masking
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void AM_DMA_Init() {
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
      
    DMA1_Channel7->CPAR = (uint32_t)(&I2C1->RXDR);
    DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE; 
}

void AM_DMA_Run(uint8_t *data, uint8_t size) {
    DMA1_Channel7->CMAR = (uint32_t)(data);
    DMA1_Channel7->CNDTR = size;
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void EXTI4_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR = EXTI_PR_PR4;
        
        AM_MultiRead(AM_ACCEL, 0x27, accel_data, 7);
    }
}

void DMA1_Channel7_IRQHandler() {
    uint8_t i = 0;
    int16_t tmp = 0;
    float swap = 0;
    if ((DMA1->ISR & DMA_ISR_TCIF7) != 0) {
        DMA1->IFCR |= DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7;
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
        
        if (DMA1_Channel7->CMAR == (uint32_t)accel_data) {
            for (i = 0; i < 3; i++) {
                tmp = (accel_data[2*i+2] << 8) | accel_data[2*i+1];
                tmp >>= 4;                  // only upper 12 bits are meaningful
                non_calibr_accel_data[i] = tmp * ACCEL_SENSITIVITY;
            }
            
            AM_MultiRead(AM_MAG, 0x03, mag_data, 6);
        } else if (DMA1_Channel7->CMAR == (uint32_t)mag_data) {
            for (i = 0; i < 3; i++) {
                tmp = (mag_data[2*i] << 8) | mag_data[2*i+1];
                non_calibr_mag_data[i] = tmp / MAG_SENSITIVITY; 
            }
            swap = non_calibr_mag_data[2];
            non_calibr_mag_data[2] = non_calibr_mag_data[1];
            non_calibr_mag_data[1] = swap;
            
            //memcpy(accel, non_calibr_accel_data, VECT_SIZE*sizeof(float));
            memcpy(magField, non_calibr_mag_data, VECT_SIZE*sizeof(float));
            
            // taking in account calibration
            arm_sub_f32(non_calibr_accel_data, accel_bias, non_calibr_accel_data, VECT_SIZE);
            arm_mat_mult_f32(&accel_trans_matrix, &non_calibr_accel, &accel_mat);
//            
//            arm_mat_mult_f32(&mag_trans_matrix, &non_calibr_mag, &non_calibr_mag);
//            arm_sub_f32(non_calibr_mag_data, mag_bias, magField, VECT_SIZE);
            
            // telemetry
            memcpy(message.accel, accel, VECT_SIZE*sizeof(float));
            memcpy(message.magField, magField, VECT_SIZE*sizeof(float));
            
            //Telemetry_Send(&message);
                   
            if (meas1) {
                meas1 = 0;
                memcpy(w1, accel, VECT_SIZE*sizeof(float));
                memcpy(w2, magField, VECT_SIZE*sizeof(float));
                
                Vect_Norm(w1);
                Vect_Norm(w2);
            } else {
                quest_run = 1;
                memcpy(v1, accel, VECT_SIZE*sizeof(float));
                memcpy(v2, magField, VECT_SIZE*sizeof(float));
                
                Vect_Norm(v1);
                Vect_Norm(v2);
            }
        }
    }
}
