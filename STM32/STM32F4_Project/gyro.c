#include "gyro.h"
#include "stm32f4xx.h"
#include "adxl345.h"

uint8_t I2C1_SCL = 6; // PB
uint8_t I2C1_SDA = 7; // PB

uint8_t gyro[6];
uint8_t gyroRegisters[6] = {0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22};

uint8_t GYRO_ADDR = 0x68;
uint8_t read = 0;
uint8_t addr = 0;
uint8_t gyro_val = 0;

uint8_t gyroIndex = 0;

int16_t gyro_xOffset;
int16_t gyro_yOffset;
int16_t gyro_zOffset;

void I2C1_GPIO_Init() {
    GPIOB->AFR[0]   |= (4 << I2C1_SCL*4) | (4 << I2C1_SDA*4); 
    GPIOB->MODER    |= (2 << I2C1_SCL*2) | (2 << I2C1_SDA*2) | (1 << 5*2);
    GPIOB->OTYPER   |= (1 << I2C1_SCL) | (1 << I2C1_SDA);
    GPIOB->OSPEEDR  |= (3 << I2C1_SCL*2) | (3 << I2C1_SDA*2);
    GPIOB->PUPDR    |= (1 << I2C1_SCL*2) | (1 << I2C1_SDA*2);
    
    GPIOB->BSRRL |= 1 << 5;
}
void delay() {
    volatile uint32_t value;
    value = 0xfff;
    while (value--);
}

void I2C1_Init() {
    I2C1_GPIO_Init();
    delay();
    
    I2C1->CR1 = 0;
    I2C1->CR2 = I2C_CR2_FREQ_3;
    I2C1->CCR = 0x28;
    I2C1->TRISE = 9; 
    I2C1->CR1 |= I2C_CR1_PE;
}

void Gyro_DMA_Init(void);

void Gyro_Init() {
    uint8_t gyro_test = 0;
    
    I2C1_Init();
    
    Gyro_SingleByteWrite(0x15, 0x09);
    gyro_test = Gyro_SingleByteRead(0x15);
    
    Gyro_SingleByteWrite(0x16, 0x1A);
    gyro_test = Gyro_SingleByteRead(0x16);
    
    Gyro_SingleByteWrite(0x17, 0x01);
    gyro_test = Gyro_SingleByteRead(0x17);
    
    Gyro_SingleByteWrite(0x3E, 0x19);
    gyro_test = Gyro_SingleByteRead(0x3E);
}

uint8_t Gyro_SingleByteRead(uint8_t gyroAddress) {
    uint8_t value = 0;
    
    while ((I2C1->SR2 & I2C_SR2_BUSY)!=0) {}
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
        
    // EV5
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
    (void) I2C1->SR1;
    I2C1->DR = 0xd0;
    
    // EV6
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
    (void) I2C1->SR1;
    (void) I2C1->SR2;
        
    // EV8_1    
    while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {}    
    I2C1->DR = gyroAddress;
        
    // EV8_2
    while (((I2C1->SR1 & I2C_SR1_TXE) == 0) && ((I2C1->SR1 & I2C_SR1_BTF) == 0)) {}
    I2C1->CR1 |= I2C_CR1_START;
    
    // EV5
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
	(void) I2C1->SR1;
    I2C1->DR = 0xd1;
    
    // EV6
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
    I2C1->CR1 &= ~I2C_CR1_ACK;
	(void) I2C1->SR1;
	(void) I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;    
      
    // EV7
    while ((I2C1->SR1 & I2C_SR1_RXNE) == 0)	{}
    value = I2C1->DR;
        
    return value;
}


void Gyro_SingleByteWrite(uint8_t gyroAddress, uint8_t data) {
    while ((I2C1->SR2 & I2C_SR2_BUSY)!=0) {}
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
        
    // EV5
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
    (void) I2C1->SR1;
    I2C1->DR = 0xd0;
    
    // EV6
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
    (void) I2C1->SR1;
    (void) I2C1->SR2;
        
    // EV8_1    
    while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {}    
    I2C1->DR = gyroAddress;
        
    // EV8    
    while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {}    
    I2C1->DR = data;
        
    // EV8_2
    while (((I2C1->SR1 & I2C_SR1_TXE) == 0) && ((I2C1->SR1 & I2C_SR1_BTF) == 0)) {}
    I2C1->CR1 |= I2C_CR1_STOP;
}



void Gyro_EXTI_Init() {    
    GPIOB->OSPEEDR |= 3 << 3*2;
    
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    I2C1->CR2 |= I2C_CR2_ITEVTEN;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR3; 
    EXTI->IMR 	|= EXTI_IMR_MR3;
    NVIC_SetPriority(EXTI3_IRQn, 0x04);
    NVIC_EnableIRQ(EXTI3_IRQn); 
}

void EXTI3_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR3) {
        EXTI->PR = EXTI_PR_PR3;
        Gyro_ReadWithInterrupt();
    }
}

void Gyro_MultipleBytesRead() {
    while ((I2C1->SR2 & I2C_SR2_BUSY)!=0) {}
    addr = 0x1d;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    
    // EV5
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
    (void) I2C1->SR1;
    I2C1->DR = 0xd0;
    
    // EV6
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
    (void) I2C1->SR1;
    (void) I2C1->SR2;
        
    // EV8_1    
    while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {}    
    I2C1->DR = addr;
        
    // EV8_2
    while (((I2C1->SR1 & I2C_SR1_TXE) == 0) && ((I2C1->SR1 & I2C_SR1_BTF) == 0)) {}
    I2C1->CR1 |= I2C_CR1_START;
    
    // EV5
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {}
	(void) I2C1->SR1;
    I2C1->DR = 0xd1;
    
    // EV6
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {}
	(void) I2C1->SR1;
	(void) I2C1->SR2;
    
    for (gyroIndex = 0; gyroIndex < 6; gyroIndex++) {
        // EV7
        while ((I2C1->SR1 & I2C_SR1_RXNE) == 0)	{}
        if (gyroIndex == 5) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        gyro[gyroIndex] = I2C1->DR;     
    }
    
    ((uint8_t *)(&gx))[0] = gyro[0];
    ((uint8_t *)(&gx))[1] = gyro[1];
    ((uint8_t *)(&gy))[0] = gyro[2];
    ((uint8_t *)(&gy))[1] = gyro[3];
    ((uint8_t *)(&gz))[0] = gyro[4];
    ((uint8_t *)(&gz))[1] = gyro[5];
}

void Gyro_ReadWithInterrupt() {
    while ((I2C1->SR2 & I2C_SR2_BUSY) !=0 ) {}
    addr = 0x1d;
    read = 0;
    gyroIndex = 0;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    
    Gyro_DMA_Init();
}

void I2C1_EV_IRQHandler() {
    uint8_t tmp;
    
    // EV5
    if (I2C1->SR1 & I2C_SR1_SB) {
        (void) I2C1->SR1;     
        tmp = (GYRO_ADDR << 1) | read;
        I2C1->DR = tmp;
    } 
    
    // EV6
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        (void) I2C1->SR1;
        (void) I2C1->SR2;
    }
    
    // EV8_2
    if ((I2C1->SR1 & I2C_SR1_TXE) && (I2C1->SR1 & I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_START;
        read = 1;
    }
    
    // EV8_1
    if (I2C1->SR1 & I2C_SR1_TXE) {
        if (!read) {
            I2C1->DR = addr;
        }
    }
    
    // EV7
//    if ((I2C1->SR1 & I2C_SR1_RXNE) /*&& (DMA1_Stream0->NDTR == 0)*/) {
//        gyro[gyroIndex] = I2C1->DR;     
//        gyroIndex++;  
//        if (gyroIndex == 5) {
//            I2C1->CR1 &= ~I2C_CR1_ACK;
//            I2C1->CR1 |= I2C_CR1_STOP;
//        
//            ((uint8_t *)(&gx))[0] = gyro[0];
//            ((uint8_t *)(&gx))[1] = gyro[1];
//            ((uint8_t *)(&gy))[0] = gyro[2];
//            ((uint8_t *)(&gy))[1] = gyro[3];
//            ((uint8_t *)(&gz))[0] = gyro[4];
//            ((uint8_t *)(&gz))[1] = gyro[5];
//            
//            gx -= gyro_xOffset;
//            gy -= gyro_yOffset;
//            gz -= gyro_zOffset;
//        }  
 //   }
}

void Gyro_Calibr(void) {
    int i = 0;
    float xSum = 0, ySum = 0, zSum = 0;
    
    while (!(GPIOB->IDR & (1 << 3))) {}
    Gyro_MultipleBytesRead();
    xSum += gx;
    ySum += gy;
    zSum += gz;
        
    for (i = 0; i < CALIBR_NUMBER; i++) {
        while (!(GPIOB->IDR & (1 << 3))) {}
        Gyro_MultipleBytesRead();
        xSum += gx;
        ySum += gy;
        zSum += gz; 
    }         
    gyro_xOffset = (int16_t)(xSum / CALIBR_NUMBER);
    gyro_yOffset = (int16_t)(ySum / CALIBR_NUMBER);
    gyro_zOffset = (int16_t)(zSum / CALIBR_NUMBER);
    
    I2C1->CR2 |= I2C_CR2_LAST | I2C_CR2_DMAEN;
}

void Gyro_DMA_Init() {
    NVIC_SetPriority(DMA1_Stream0_IRQn, 0x03);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    
    DMA1_Stream0->CR    = 0;
    DMA1_Stream0->PAR   = (uint32_t)&(I2C1->DR);
    DMA1_Stream0->M0AR  = (uint32_t)gyro;
    DMA1_Stream0->NDTR  = 6;
    DMA1_Stream0->CR    |= DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
}

void DMA1_Stream0_IRQHandler() {
    float tmp;
    if (DMA1->LISR & DMA_LISR_TCIF0) {
        I2C1->CR1 |= I2C_CR1_STOP;
        
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
        
        
        
        ((uint8_t *)(&gx))[0] = gyro[0];
        ((uint8_t *)(&gx))[1] = gyro[1];
        ((uint8_t *)(&gy))[0] = gyro[2];
        ((uint8_t *)(&gy))[1] = gyro[3];
        ((uint8_t *)(&gz))[0] = gyro[4];
        ((uint8_t *)(&gz))[1] = gyro[5];
        
        gx -= gyro_xOffset;
        gy -= gyro_yOffset;
        gz -= gyro_zOffset;

        gx = (int16_t)(((float)gx/32767) * 2000);
        gy = (int16_t)(((float)gy/32767) * 2000);
        gz = (int16_t)(((float)gz/32767) * 2000);
    }
}
