#include "gyro.h"
#include "stm32f4xx.h"

uint8_t I2C1_SCL = 6; // PB
uint8_t I2C1_SDA = 7; // PB

uint8_t gyro[6];
uint8_t gyroRegisters[6] = {0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22};

void I2C1_GPIO_Init() {
    GPIOB->AFR[0]   |= (4 << I2C1_SCL*4) | (4 << I2C1_SDA*4); 
    GPIOB->MODER    |= (2 << I2C1_SCL*2) | (2 << I2C1_SDA*2) | (1 << 5*2);
    GPIOB->OTYPER   |= (1 << I2C1_SCL) | (1 << I2C1_SDA);
    GPIOB->OSPEEDR  |= (3 << I2C1_SCL*2) | (3 << I2C1_SDA*2);
    GPIOB->PUPDR    |= (1 << I2C1_SCL*2) | (1 << I2C1_SDA*2);
    
    GPIOB->BSRRL |= 1 << 5;
}
void dl() {
    volatile uint32_t value;
    value = 0xfff;
    while (value--);
}

void I2C1_Init() {
    I2C1_GPIO_Init();
    dl();
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    
    //I2C1->CR1 = I2C_CR1_SWRST; 
    I2C1->CR1  = 0;
    I2C1->TRISE = 9; //300/125 + 1
    I2C1->CR2 = I2C_CR2_LAST | I2C_CR2_DMAEN |/*I2C_CR2_ITEVTEN |*/ I2C_CR2_FREQ_3; //8 MHZ HSE
    I2C1->CCR = 0x28; //8 MHZ/100 KHZ
    I2C1->OAR1 = 0x21;
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
    
   
}

uint8_t GYRO_ADDR = 0x68;
uint8_t read = 0;
uint8_t addr = 0;
uint8_t gyro_val = 0;

uint8_t vals_index = 0;
uint8_t vals[3] = {0, 0, 0};

void Gyro_DMA_Init(void);

void GYRO_Read(uint8_t addr) {
   
    volatile uint8_t value;
    while ((I2C1->SR2 & I2C_SR2_BUSY)!=0) {}
    I2C1->CR1 |= I2C_CR1_START;
     Gyro_DMA_Init();
//    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
//   
//    (void) I2C1->SR1;
//    I2C1->DR = 0xd0;
//    
//    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
//    (void) I2C1->SR1;
//    (void) I2C1->SR2;
//    I2C1->DR = addr;
//    
//    while ((I2C1->SR1 & I2C_SR1_BTF) == 0) {}
//    I2C1->CR1 |= I2C_CR1_START;
//    
//    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
//	(void) I2C1->SR1;
//    I2C1->DR = 0xd1;
//    
//    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
//	(void) I2C1->SR1;
//	(void) I2C1->SR2;
//        
//    while ((I2C1->SR1 & I2C_SR1_RXNE) == 0)	{}
//    value = I2C1->DR;
//    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_EV_IRQHandler() {
    uint8_t tmp;
    if (I2C1->SR1 & I2C_SR1_SB) {
        (void) I2C1->SR1;     
        tmp = (GYRO_ADDR << 1) | read;
        I2C1->DR = tmp;
    } 
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void) I2C1->SR1;
        (void) I2C1->SR2;
        if (!read) {
            I2C1->DR = addr;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
    }
    if (I2C1->SR1 & I2C_SR1_BTF) {
        I2C1->CR1 |= I2C_CR1_START;
        read = 1;
    }
    if (I2C1->SR1 & I2C_SR1_RXNE) {
        vals[vals_index] = I2C1->DR;
        
        
        vals_index++;
    }
}

void Gyro_DMA_Init() {
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    
    DMA1_Stream7->CR    = 0;
    DMA1_Stream0->CR    = 0;
    DMA1_Stream0->PAR   = (uint32_t)&(I2C1->DR);
    DMA1_Stream0->M0AR  = (uint32_t)gyro;
    DMA1_Stream0->NDTR  = 6;
    DMA1_Stream0->CR    |= DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
    
    DMA1_Stream7->PAR   = (uint32_t)&(I2C1->DR);
    DMA1_Stream7->M0AR  = (uint32_t)gyroRegisters;     
    DMA1_Stream7->NDTR  = 6;
    DMA1_Stream7->CR    |= DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL | DMA_SxCR_EN;  
}

void DMA1_Stream0_IRQHandler() {
    
}

void DMA1_Stream7_IRQHandler() {
}

