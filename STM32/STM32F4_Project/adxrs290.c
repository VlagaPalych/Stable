#include "adxrs290.h"
#include "adxl345.h"
#include "stm32f4xx.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

uint8_t SPI2_Busy = 0;

uint8_t adxrs290_regs[ADXRS290_DATA_SIZE*2] = {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d};

usingSPI2 curUsingSPI2 = ACCEL;

void All_NSS_High() {
    Accel_NSS_High();
    ARS1_NSS_High();
    ARS2_NSS_High();
}

void SPI2_SensorsPoll(void) { 
    SPI2_Busy = 0;
    if (GPIOD->IDR & (1 << ARS1_EXTI)) {
        // ARS1 DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER10;
    } else if (GPIOE->IDR & (1 << ARS2_EXTI)) {
        // ARS2 DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
    } else if (GPIOA->IDR & (1 << ACCEL_INT1)) {
        // Accel DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER1;
    }
}

void EXTI15_10_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR = EXTI_PR_PR10;
        
        ARS1_GetData();
    } else if (EXTI->PR & EXTI_PR_PR15) {
        EXTI->PR = EXTI_PR_PR15;
        
        ARS2_GetData();
    }
}

//-------------------------------------------------------------------
// ARS1
//-------------------------------------------------------------------

uint8_t ARS1_NSS    = 9;    // PD
uint8_t ARS1_VDD    = 8;    // PD
uint8_t ARS1_EXTI   = 10;   // PD

uint8_t ars1_rawData[ADXRS290_DATA_SIZE*2];
int16_t ars1_data[ADXRS290_DATA_SIZE];     

uint8_t ars1_calibrationOn = 0;
int16_t ars1_offset[ADXRS290_DATA_SIZE];
float ars1_sum[ADXRS290_DATA_SIZE];
uint32_t ars1_calibrIndex = 0;
uint32_t ars1_calibrNumber = 0;

void ARS1_VDD_Init() {
    GPIOD->MODER |= 1 << ARS1_VDD*2;
    GPIOD->BSRRL |= 1 << ARS1_VDD;
}

void ARS1_NSS_Init() {
    GPIOD->MODER |= 1 << ARS1_NSS*2;
    GPIOD->OSPEEDR |= 3 << ARS1_NSS*2;
}

void ARS1_NSS_Low() {
    GPIOD->BSRRH |= 1 << ARS1_NSS;
}

void ARS1_NSS_High() {
    GPIOD->BSRRL |= 1 << ARS1_NSS;
}

void ARS1_Init() {
    uint8_t ars1_test = 0;
    
    ARS1_NSS_Init();
    ARS1_NSS_High();
    
    // Turn measurement and termometer on
    ARS1_NSS_Low();
    SPI2_Write(0x10, 0x03);
    ARS1_NSS_High();
    
    ARS1_NSS_Low();
    ars1_test = SPI2_Read(0x10);
    ARS1_NSS_High();
    
    // Turn DATA_READY interrupt on
    ARS1_NSS_Low();
    SPI2_Write(0x12, 0x01);
    ARS1_NSS_High();
    
    ARS1_NSS_Low();
    ars1_test = SPI2_Read(0x12);
    ARS1_NSS_High();
}

void ARS1_EXTI_Init() {
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PD;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR10; 
    EXTI->IMR 	|= EXTI_IMR_MR10;
    NVIC_SetPriority(EXTI15_10_IRQn, 0x06);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ARS1_Calibr() {
    uint8_t i = 0;
    for (i = 0; i < ADXRS290_DATA_SIZE; i++) {
        ars1_sum[i] = 0;
    }
    ars1_calibrIndex = 0;
    ars1_calibrNumber = 4200; // TODO: make number of calibration samples clear
    ars1_calibrationOn = 1;
}

void ARS1_GetData() {
    uint8_t i = 0;
    
    if (SPI2_Busy == 0/*(SPI2->SR & SPI_SR_BSY) == 0*/) {
        SPI2_Busy = 1;
        
        for (i = 0; i < ADXRS290_DATA_SIZE*2; i++) {
            ARS1_NSS_Low();
            ars1_rawData[i] = SPI2_Read(adxrs290_regs[i]);
            ARS1_NSS_High();
        }
        All_NSS_High();
        
        ars1_data[0] = (ars1_rawData[1] << 8) | ars1_rawData[0];
        ars1_data[1] = (ars1_rawData[3] << 8) | ars1_rawData[2];
        ars1_data[2] = ((ars1_rawData[5] & 0x0f) << 8) | ars1_rawData[4];
        
        if (ars1_calibrationOn) {
            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars1_sum[i] += ars1_data[i];
            }
            ars1_calibrIndex++;          
            if (ars1_calibrIndex == ars1_calibrNumber) {
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars1_offset[i] = (int16_t)(ars1_sum[i] / ars1_calibrNumber);
                }
                ars1_calibrationOn = 0;
            }
        }
        for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
            ars1_data[i] -= ars1_offset[i];
        }
        
        SPI2_SensorsPoll();
    }
}

//-------------------------------------------------------------------
// ARS2
//-------------------------------------------------------------------

uint8_t ARS2_NSS    = 10; // PB
uint8_t ARS2_VDD    = 14; // PE
uint8_t ARS2_EXTI   = 15; // PE

uint8_t ars2_rawData[ADXRS290_DATA_SIZE*2];
int16_t ars2_data[ADXRS290_DATA_SIZE];       // 2 angle rates

uint8_t ars2_calibrationOn = 0;
int16_t ars2_offset[ADXRS290_DATA_SIZE];
float ars2_sum[ADXRS290_DATA_SIZE];
uint32_t ars2_calibrIndex = 0;
uint32_t ars2_calibrNumber = 0;

void ARS2_VDD_Init() {
    GPIOE->MODER |= 1 << ARS2_VDD*2;
    GPIOE->BSRRL |= 1 << ARS2_VDD;
}

void ARS2_NSS_Init() {
    GPIOB->MODER    |= 1 << ARS2_NSS*2;
    GPIOB->OSPEEDR  |= 3 << ARS2_NSS*2;
}

void ARS2_NSS_Low() {
    GPIOB->BSRRH |= 1 << ARS2_NSS;
}

void ARS2_NSS_High() {
    GPIOB->BSRRL |= 1 << ARS2_NSS;
}

void ARS2_Init() {
    uint8_t ars2_test = 0;
    
    ARS2_VDD_Init();
    ARS2_NSS_Init();
    ARS2_NSS_High();
    
    // Turn measurement and termometer on
    ARS2_NSS_Low();
    SPI2_Write(0x10, 0x03);
    ARS2_NSS_High();
    
    ARS2_NSS_Low();
    ars2_test = SPI2_Read(0x10);
    ARS2_NSS_High();
    
    // Turn DATA_READY interrupt on
    ARS2_NSS_Low();
    SPI2_Write(0x12, 0x01);
    ARS2_NSS_High();
    
    ARS2_NSS_Low();
    ars2_test = SPI2_Read(0x12);
    ARS2_NSS_High();
}

void ARS2_EXTI_Init() { // PE15
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PE;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR15; 
    EXTI->IMR 	|= EXTI_IMR_MR15;
    NVIC_SetPriority(EXTI15_10_IRQn, 0x06);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ARS2_Calibr() {
    uint8_t i = 0;
    for (i = 0; i < ADXRS290_DATA_SIZE; i++) {
        ars2_sum[i] = 0;
    }
    ars2_calibrIndex = 0;
    ars2_calibrNumber = 4200; // TODO: make number of calibration samples clear
    ars2_calibrationOn = 1;
}

void ARS2_GetData() {
    uint8_t i = 0;
    
    if (SPI2_Busy == 0/*(SPI2->SR & SPI_SR_BSY) == 0*/) {
        SPI2_Busy = 1;
        
        for (i = 0; i < ADXRS290_DATA_SIZE*2; i++) {
            ARS2_NSS_Low();
            ars2_rawData[i] = SPI2_Read(adxrs290_regs[i]);
            ARS2_NSS_High();
        }
        All_NSS_High();
        
        ars2_data[0] = (ars2_rawData[1] << 8) | ars2_rawData[0];
        ars2_data[1] = (ars2_rawData[3] << 8) | ars2_rawData[2];
        ars2_data[2] = ((ars2_rawData[5] & 0x0f) << 8) | ars2_rawData[4];
        
        if (ars2_calibrationOn) {
            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                ars2_sum[i] += ars2_data[i];
            }
            ars2_calibrIndex++;          
            if (ars2_calibrIndex == ars2_calibrNumber) {
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars2_offset[i] = (int16_t)(ars2_sum[i] / ars2_calibrNumber);
                }
                ars2_calibrationOn = 0;
            }
        }
        for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
            ars2_data[i] -= ars2_offset[i];
        }
        
        SPI2_SensorsPoll();
    }
}

//void ARS2_DMA_Init() {
//    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x05);    
//    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
//    
//    DMA1->HIFCR = DMA_HIFCR_CFEIF4;
//    
//    DMA1_Stream4->CR    = 0;
//    DMA1_Stream3->CR    = 0;
//    
//    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
//    DMA1_Stream3->M0AR  = (uint32_t)(ars2_rawData /*+ ars2_dma_status*2*/);
//    DMA1_Stream3->NDTR  = 2;
//    DMA1_Stream3->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
//                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
//    
//    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
//    DMA1_Stream4->M0AR  = (uint32_t)(adxrs290_regs /*+ ars2_dma_status*2*/);     
//    DMA1_Stream4->NDTR  = 2;
//    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
//                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL | DMA_SxCR_EN;     
//}

