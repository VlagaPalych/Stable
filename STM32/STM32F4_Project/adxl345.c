#include "adxl345.h"
#include "stm32f4xx.h" 
#include "processing.h"
#include "telemetry.h"

#define ACCEL_DMA

// SPI communication pins for accelerometer
uint8_t SPI2_SCK    = 13;   // PB
uint8_t SPI2_MISO   = 14;   // PB
uint8_t SPI2_MOSI   = 15;   // PB
uint8_t SPI2_NSS    = 12;   // PB
uint8_t ACCEL_INT1  = 1;    // PA
uint8_t ACCEL_VCC   = 13;   // PC

int16_t accelRegisters[6] = {0xB200, 0xB300, 0xB400, 0xB500, 0xB600, 0xB700};
int16_t accel[6];

int16_t xOffset, yOffset, zOffset;


void Delay() {
  volatile uint32_t i;
  for (i=0; i < 0xF0000; i++){ __nop();__nop();}
}

void ADXL345_AccelVCCInit() {
    GPIOC->MODER    |= 1 << ACCEL_VCC*2;
    GPIOC->OTYPER   &= ~(1 << ACCEL_VCC);
    GPIOC->OSPEEDR  |= 3 << ACCEL_VCC*2;
    
    GPIOC->BSRRL |= 1 << ACCEL_VCC;
    Delay();
}

void NSS_Low() {
    GPIOB->BSRRH |= (1 << SPI2_NSS);
}

void NSS_High() {
    GPIOB->BSRRL |= (1 << SPI2_NSS);
}

void SPI2_GPIO_Init() {
	GPIOB->MODER 	|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2) | (1 << SPI2_NSS*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2) | (3 << SPI2_NSS*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~((1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI) | (1 << SPI2_NSS)); 
	
	NSS_High();
}

void SPI2_Init() {
    SPI2_GPIO_Init();
	
	SPI2->CR1 = 0;
	SPI2->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	//SPI2->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}
    uint8_t test = 0;
void ADXL345_Init() {

    
    ADXL345_AccelVCCInit();
    SPI2_Init();  
    NSS_Low();
    //while(test!=0xE5) {
    //    NSS_Low();
        test = ADXL345_Read(DEVID_ADDRESS);
    //    NSS_High();
    //}

    ADXL345_Write(INT_MAPPING_ADDRESS, DATA_READY_INT0_MAPPING);
    test = ADXL345_Read(INT_MAPPING_ADDRESS);

    ADXL345_Write(POWER_CTL_ADDRESS, MEASUREMENT_MODE);
    test = ADXL345_Read(POWER_CTL_ADDRESS);


    ADXL345_Write(DATA_FORMAT_ADDRESS, FULL_RES_MODE);
    test = ADXL345_Read(DATA_FORMAT_ADDRESS);

    ADXL345_Write(BW_RATE_ADDRESS, ACCEL_FREQ);
    test = ADXL345_Read(BW_RATE_ADDRESS);


    ADXL345_Write(INT_ENABLE_ADDRESS, DATA_READY_INT);
    test = ADXL345_Read(INT_ENABLE_ADDRESS);

    NSS_High();
}

uint16_t SPI1_Transfer(uint16_t byte) { 
    
	while ((SPI2->SR & SPI_SR_TXE)==0);
    //NSS_Low();
	SPI2->DR = byte;
	
	while ((SPI2->SR & SPI_SR_RXNE)==0);
    //NSS_High();
	return (SPI2->DR);
}

uint8_t ADXL345_Read(uint8_t address) {
    uint16_t tmp = 0;
    
    address |= READ_COMMAND;
    tmp = SPI1_Transfer(address << 8);
    
    return tmp & 0xFF;
}

void ADXL345_Write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |= data;
    SPI1_Transfer(tmp);
}

void ADXL345_GetAccel(int16_t *x, int16_t *y, int16_t *z) {
    NSS_Low();
    ((uint8_t *)x)[0] = ADXL345_Read(0x32);
    ((uint8_t *)x)[1] = ADXL345_Read(0x33);
    ((uint8_t *)y)[0] = ADXL345_Read(0x34);
    ((uint8_t *)y)[1] = ADXL345_Read(0x35);
    ((uint8_t *)z)[0] = ADXL345_Read(0x36);
    ((uint8_t *)z)[1] = ADXL345_Read(0x37);
    NSS_High();
}

void ADXL345_Calibr() {
    int i = 0;
    double xSum = 0, ySum = 0, zSum = 0;
    
    while (!(GPIOA->IDR & (1 << ACCEL_INT1))) {}
    ADXL345_GetAccel(&ax, &ay, &az); 
    xSum += ax;
    ySum += ay;
    zSum += az;
        
    for (i = 0; i < CALIBR_NUMBER; i++) {
        while (!(GPIOA->IDR & (1 << ACCEL_INT1))) {}
        ADXL345_GetAccel(&ax, &ay, &az); 
        xSum += ax;
        ySum += ay;
        zSum += az;  
    }         
    xOffset = (int16_t)(xSum / CALIBR_NUMBER);
    yOffset = (int16_t)(ySum / CALIBR_NUMBER);
    zOffset = (int16_t)(zSum / CALIBR_NUMBER) - 0xFF;
}

void Accel_EXTI_Init() {    
    GPIOA->OSPEEDR |= 3 << 1*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR1; 
    EXTI->IMR 	|= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI1_IRQn, 0x04);
    NVIC_EnableIRQ(EXTI1_IRQn); 
}

#ifndef ACCEL_DMA

void EXTI1_IRQHandler() {  //what to do when accelerometer is ready
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        ADXL345_GetAccel(&ax, &ay, &az);
        ax -= xOffset;
        ay -= yOffset;
        az -= zOffset;
        
        control();
    }
}

#else

void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        
        //Delay(5000);
        GPIOD->BSRRL |= 1 << 15;
        NSS_Low();
        ADXL345_DMA_Init();
    }
}

void ADXL345_DMA_Init() {  
    DMA1->HIFCR = DMA_HIFCR_CFEIF4;
    DMA1_Stream4->CR    = 0;
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x03);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    DMA1_Stream3->CR    = 0;
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR  = (uint32_t)accel;
    DMA1_Stream3->NDTR  = 6;
    DMA1_Stream3->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN;
    
    
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
     
    DMA1_Stream4->CR    = 0;
    DMA1_Stream4->M0AR  = (uint32_t)accelRegisters;     
    DMA1_Stream4->NDTR  = 6;
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 /*| DMA_SxCR_PFCTRL*/ | DMA_SxCR_PL | DMA_SxCR_EN;     
}


void updateFreq() {
    if (freshFreq) {
        NSS_Low();
        ADXL345_Write(BW_RATE_ADDRESS, curFreq);
        NSS_High();
        freshFreq = 0;
    }
}

void DMA1_Stream3_IRQHandler() {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;
        DMA1->LIFCR = DMA_LIFCR_CHTIF3;

        if(GPIOA->IDR & GPIO_IDR_IDR_1) {  
            ADXL345_DMA_Init();
            GPIOD->ODR ^= 1 << 15;
        } else NSS_High();
        
        updateFreq();
        
        ((uint8_t *)(&ax))[0] = (uint8_t)(accel[0] & 0xFF);
        ((uint8_t *)(&ax))[1] = (uint8_t)(accel[1] & 0xFF);
        ((uint8_t *)(&ay))[0] = (uint8_t)(accel[2] & 0xFF);
        ((uint8_t *)(&ay))[1] = (uint8_t)(accel[3] & 0xFF);
        ((uint8_t *)(&az))[0] = (uint8_t)(accel[4] & 0xFF);
        ((uint8_t *)(&az))[1] = (uint8_t)(accel[5] & 0xFF);
        
        ax -= xOffset;
        ay -= yOffset;
        az -= zOffset;
        
        process();
        GPIOD->BSRRH |= 1 << 15;
    }
}

void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;
    } 
    if (DMA1->HISR & DMA_HISR_HTIF4) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF4;
    }
    
}


#endif
