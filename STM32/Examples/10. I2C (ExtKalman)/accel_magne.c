#include "stdint.h"
#include "stm32f30x.h"

uint8_t I2C1_SCL    = 6; // PB
uint8_t I2C1_SDA    = 7; // PB
uint8_t AM_INT1     = 4; // PE
uint8_t AM_INT2     = 5; // PE
uint8_t AM_DRDY     = 2; // PE;

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
    I2C1->CR2 = I2C_CR2_FREQ_3;
    I2C1->CCR = 0x28;
    I2C1->TRISE = 9; 
    I2C1->CR1 |= I2C_CR1_PE;
}