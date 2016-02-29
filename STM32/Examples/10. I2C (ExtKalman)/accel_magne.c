#include "stm32f30x.h"
#include "accel_magne.h"

uint8_t I2C1_SCL    = 6; // PB
uint8_t I2C1_SDA    = 7; // PB
uint8_t AM_INT1     = 4; // PE
uint8_t AM_INT2     = 5; // PE
uint8_t AM_DRDY     = 2; // PE;

#define AM_ADDRESS 25

#define AM_CTRL_REG1_A          0x20
#define AM_CTRL_REG1_A_VALUE    0x57
#define AM_CTRL_REG3_A          0x22
#define AM_CTRL_REG3_A_VALUE    0x10
#define AM_CTRL_REG4_A          0x23
#define AM_CTRL_REG4_A_VALUE    0x38

#define ACCEL_SENSITIVITY   0.012f

uint8_t accel_data[7];
float accel[3];


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
    
    I2C1->CR2 |= AM_ADDRESS << 1;
}


uint8_t AM_SingleRead(uint8_t address) {
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

void AM_MultiRead(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;
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
    I2C1->CR2 |= size << 16;        // size bytes  
    I2C1->CR2 |= I2C_CR2_START;
    
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop
    
//    for (i = 0; i < size; i++) {
//        while ((I2C1->ISR & I2C_ISR_RXNE) == 0);
//        data[i] = I2C1->RXDR;
//    }
}

void AM_Write(uint8_t address, uint8_t data) {
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

    // normal mode, all axis on
    AM_Write(AM_CTRL_REG1_A, AM_CTRL_REG1_A_VALUE); 
    am_test = AM_SingleRead(AM_CTRL_REG1_A);
    
    // data ready interrupt on INT1
    AM_Write(AM_CTRL_REG3_A, AM_CTRL_REG3_A_VALUE);
    am_test = AM_SingleRead(AM_CTRL_REG3_A);
    
    // +- 16G, high resolution
    AM_Write(AM_CTRL_REG4_A, AM_CTRL_REG4_A_VALUE);
    am_test = AM_SingleRead(AM_CTRL_REG4_A);
    
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
        
        AM_MultiRead(0x27, accel_data, 7);
    }
}

void DMA1_Channel7_IRQHandler() {
    uint8_t i = 0;
    int16_t tmp = 0;
    if ((DMA1->ISR & DMA_ISR_TCIF7) != 0) {
        DMA1->IFCR |= DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7;
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
        
        for (i = 0; i < 3; i++) {
            tmp = (accel_data[2*i+2] << 8) | accel_data[2*i+1];
            tmp >>= 4;                  // only upper 12 bits are meaningful
            accel[i] = tmp * ACCEL_SENSITIVITY;
        }
    }
}
