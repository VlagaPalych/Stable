#include "adxl345.h"
#include "stm32f4xx.h" 
#include "processing.h"
#include "telemetry.h"
#include "adxrs290.h"

#define ACCEL_DMA

// SPI communication pins for accelerometer
uint8_t SPI2_SCK    = 13;   // PB
uint8_t SPI2_MISO   = 14;   // PB
uint8_t SPI2_MOSI   = 15;   // PB
uint8_t ACCEL_NSS    = 12;   // PB
uint8_t ACCEL_INT1  = 1;    // PA
uint8_t ACCEL_VDD   = 11;   // PB

int16_t accelRegisters[6] = {0xB200, 0xB300, 0xB400, 0xB500, 0xB600, 0xB700};
int16_t accel[6];

uint8_t accelCalibrationOn = 0;
uint32_t accelCalibrIndex = 0;
uint32_t accelCalibrNumber = 0;
float accel_xSum = 0, accel_ySum = 0, accel_zSum = 0;

int16_t xOffset, yOffset, zOffset;

uint8_t freshFreq   = 0;
uint8_t curFreq     = HZ1600;
float curDT         = 0.000625;

void Delay() {
  volatile uint32_t i;
  for (i=0; i < 0xF0000; i++){ __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
}

void Accel_VDD_Init() {
    GPIOB->MODER    |= 1 << ACCEL_VDD*2;
    GPIOB->OTYPER   &= ~(1 << ACCEL_VDD);
    GPIOB->OSPEEDR  |= 3 << ACCEL_VDD*2;
    
    GPIOB->BSRRL |= 1 << ACCEL_VDD;
}

void Accel_NSS_Init() {
    GPIOB->MODER    |= 1 << ACCEL_NSS*2;
    GPIOB->OSPEEDR 	|= 3 << ACCEL_NSS*2;
    GPIOB->OTYPER	&= ~(1 << ACCEL_NSS);
}

void Accel_NSS_Low() {
    GPIOB->BSRRH |= (1 << ACCEL_NSS);
}

void Accel_NSS_High() {
    GPIOB->BSRRL |= (1 << ACCEL_NSS);
}

void SPI2_GPIO_Init() {
	GPIOB->MODER 	|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~((1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI)); 
    GPIOB->PUPDR    |= (1 << SPI2_MISO*2); // Pull-up MISO
	
	Accel_NSS_High();
}

void SPI2_Init() {
    SPI2_GPIO_Init();
    
	SPI2->CR1 = 0;
	SPI2->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	//SPI2->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}

void ADXL345_Init() {  
    uint8_t accel_test = 0;  
    
    Accel_VDD_Init();
    Accel_NSS_Init();
    Delay();    
    
    Accel_NSS_Low();
    
    accel_test = SPI2_Read(DEVID_ADDRESS);

    SPI2_Write(INT_MAPPING_ADDRESS, DATA_READY_INT0_MAPPING);
    accel_test = SPI2_Read(INT_MAPPING_ADDRESS);

    SPI2_Write(POWER_CTL_ADDRESS, MEASUREMENT_MODE);
    accel_test = SPI2_Read(POWER_CTL_ADDRESS);


    SPI2_Write(DATA_FORMAT_ADDRESS, FULL_RES_MODE);
    accel_test = SPI2_Read(DATA_FORMAT_ADDRESS);

    SPI2_Write(BW_RATE_ADDRESS, curFreq);
    accel_test = SPI2_Read(BW_RATE_ADDRESS);


    SPI2_Write(INT_ENABLE_ADDRESS, DATA_READY_INT);
    accel_test = SPI2_Read(INT_ENABLE_ADDRESS);

    Accel_NSS_High();
}

uint16_t SPI2_Transfer(uint16_t byte) { 
	while ((SPI2->SR & SPI_SR_TXE)==0);
	SPI2->DR = byte;
	
	while ((SPI2->SR & SPI_SR_RXNE)==0);
	return (SPI2->DR);
}

uint8_t SPI2_Read(uint8_t address) {
    uint16_t tmp = 0;
    
    address |= READ_COMMAND;
    tmp = SPI2_Transfer(address << 8);
    
    return tmp & 0xFF;
}

void SPI2_Write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |= data;
    SPI2_Transfer(tmp);
}

void ADXL345_GetAccel(int16_t *x, int16_t *y, int16_t *z) {
    Accel_NSS_Low();
    ((uint8_t *)x)[0] = SPI2_Read(0x32);
    ((uint8_t *)x)[1] = SPI2_Read(0x33);
    ((uint8_t *)y)[0] = SPI2_Read(0x34);
    ((uint8_t *)y)[1] = SPI2_Read(0x35);
    ((uint8_t *)z)[0] = SPI2_Read(0x36);
    ((uint8_t *)z)[1] = SPI2_Read(0x37);
    Accel_NSS_High();
}

void ADXL345_Calibr() {
    accel_xSum = 0; accel_ySum = 0; accel_zSum = 0;
    accelCalibrIndex = 0;
    accelCalibrNumber = (int)(8.0 / curDT);
    accelCalibrationOn = 1;
}

void Accel_EXTI_Init() {   
    GPIOA->MODER |= 1 << 2*2;;    
    GPIOA->OSPEEDR |= 3 << 1*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR1; 
    EXTI->IMR 	|= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI1_IRQn, 0x06);
    NVIC_EnableIRQ(EXTI1_IRQn); 
}

#ifndef ACCEL_DMA

void EXTI1_IRQHandler() {  //what to do when accelerometer is ready
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        ADXL345_GetAccel(&ax, &ay, &az);
        ax -= xOffset;
        ay -= yOffset;
        az -= zOffset;
        
        control();
    }
}

#else

void Accel_GetData() {
    if (SPI2_Busy == 0/*(SPI2->SR & SPI_SR_BSY) == 0*/) {
        GPIOA->BSRRL |= 1 << 2;
        SPI2_Busy = 1;
        ADXL345_DMA_Init();
    } 
}

void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        
        Accel_GetData();
    }
}

void ADXL345_DMA_Init() {
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x05);    
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1->HIFCR = DMA_HIFCR_CFEIF4;
    
    DMA1_Stream4->CR    = 0;
    DMA1_Stream3->CR    = 0;
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR  = (uint32_t)accel;
    DMA1_Stream3->NDTR  = 6;
    DMA1_Stream3->CR    = DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
    Accel_NSS_Low();
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->CR    = 0;
    DMA1_Stream4->M0AR  = (uint32_t)accelRegisters;     
    DMA1_Stream4->NDTR  = 6;
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 /*| DMA_SxCR_PFCTRL*/ | DMA_SxCR_PL | DMA_SxCR_EN;     
}


void Accel_UpdateFreq() {
    if (freshFreq) {
        Accel_NSS_Low();
        SPI2_Write(BW_RATE_ADDRESS, curFreq);
        Accel_NSS_High();
        freshFreq = 0;
    }
}

void DMA1_Stream3_IRQHandler() {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
        All_NSS_High();
        switch (curUsingSPI2) {
            case ACCEL:         
                Accel_UpdateFreq();
                
                ((uint8_t *)(&ax))[0] = (uint8_t)(accel[0] & 0xFF);
                ((uint8_t *)(&ax))[1] = (uint8_t)(accel[1] & 0xFF);
                ((uint8_t *)(&ay))[0] = (uint8_t)(accel[2] & 0xFF);
                ((uint8_t *)(&ay))[1] = (uint8_t)(accel[3] & 0xFF);
                ((uint8_t *)(&az))[0] = (uint8_t)(accel[4] & 0xFF);
                ((uint8_t *)(&az))[1] = (uint8_t)(accel[5] & 0xFF);
            
                
                if (accelCalibrationOn) {
                    accel_xSum += ax;
                    accel_ySum += ay;
                    accel_zSum += az;  
                    
                    accelCalibrIndex++;
                    
                    if (accelCalibrIndex == accelCalibrNumber) {
                        xOffset = (int16_t)(accel_xSum / accelCalibrNumber);
                        yOffset = (int16_t)(accel_ySum / accelCalibrNumber);
                        zOffset = (int16_t)(accel_zSum / accelCalibrNumber) - 0xFF;
                        accelCalibrationOn = 0;
                    }
                }
                
                ax -= xOffset;
                ay -= yOffset;
                az -= zOffset;
                
                if ((ax > 100) || (ax < -100)) {
                    ax++;
                    ax--;
                }
                
                axHistory[axHistoryIndex] = ax;
                axHistoryIndex++;
                
                azHistory[azHistoryIndex] = az;
                azHistoryIndex++;
                
                if (azHistoryIndex >= ACCEL_FILTER_SIZE) {
                    accelLowpassReady = 1;
                }
                
                processCounter++;
                if (processCounter == 16) {
                    processCounter = 0;
                    
                    axCurHistoryIndex = axHistoryIndex - 1;
                    azCurHistoryIndex = azHistoryIndex - 1;
                    if (lowpassOn && accelLowpassReady) {
                        doAccelProcess = 1;
                    }
                } 
                
                
                break;
            case ARS1:
                break;
            case ARS2:
//                if(GPIOA->IDR & GPIO_IDR_IDR_5) {  
//                    ARS2_DMA_Init();
//                } else ARS2_NSS_High();
            
              //  ARS2_NSS_High();
//                ars2_dma_status++;
//                if (ars2_dma_status == 2) {
//                    // data from ars2 received
//                } else {
//                    ARS2_NSS_Low();
//                    ARS2_DMA_Init();
//                }
                break;
            default:
                break;
        }
        GPIOA->BSRRH |= 1 << 2;
        SPI2_SensorsPoll();
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
