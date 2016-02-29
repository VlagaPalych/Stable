#include "stm32f3xx.h"

uint8_t I2C1_SCL    = 6; // PB
uint8_t I2C1_SDA    = 7; // PB
uint8_t AM_INT1     = 4; // PE
uint8_t AM_INT2     = 5; // PE
uint8_t AM_DRDY     = 2; // PE;

#define AM_ADDRESS 25

#define AM_CTRL_REG1_A          0x20
#define AM_CTRL_REG1_A_VALUE    0x57

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
    I2C1->CR1 |= I2C_CR1_NACKIE | /*I2C_CR1_RXIE | I2C_CR1_TXIE |*/ I2C_CR1_PE;
    
    I2C1->CR2 |= AM_ADDRESS << 1;
    NVIC_EnableIRQ(I2C1_EV_IRQn);
}


uint8_t AM_SingleRead(uint8_t address) {
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;   // write
    I2C1->CR2 |= 1 << 16;           // 1 byte
    I2C1->CR2 &= ~I2C_CR2_AUTOEND;  // for restart
    
    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = address;
    
    //while ((I2C1->ISR & I2C_ISR_TC) == 0);
    
    I2C1->CR2 |= I2C_CR2_RD_WRN;    // read
    I2C1->CR2 |= 1 << 16;           // 1 byte
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop
    
    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_RXNE) == 0);
    return (I2C1->RXDR);
    
    //while ((I2C1->ISR & I2C_ISR_TC) == 0); 
}

void AM_Write(uint8_t address, uint8_t data) {
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;   // write
    I2C1->CR2 |= 2 << 16;           // 2 bytes
    I2C1->CR2 |= I2C_CR2_AUTOEND;   // for stop  

    I2C1->CR2 |= I2C_CR2_START;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = address;
    
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0);
    I2C1->TXDR = data;
    
    //while ((I2C1->ISR & I2C_ISR_TC) == 0);
}

void I2C1_IRQHandler() {
    if (I2C1->ISR & I2C_ISR_NACKF) {
        
    } else if (I2C1->ISR & I2C_ISR_TXIS) {
    } else if (I2C1->ISR & I2C_ISR_RXNE) {
    }
}

uint8_t am_test = 0;
void AM_Init() {
    I2C1_Init();
    
    // normal mode, all axis on
    AM_Write(AM_CTRL_REG1_A, AM_CTRL_REG1_A_VALUE); 
    am_test = AM_SingleRead(AM_CTRL_REG1_A);
}