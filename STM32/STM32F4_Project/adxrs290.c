#include "adxrs290.h"
#include "adxl345.h"
#include "stm32f4xx.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------


usingSPI2 curUsingSPI2 = ACCEL;

void All_NSS_High() {
    Accel_NSS_High();
    ARS1_NSS_High();
    ARS2_NSS_High();
}

void SPI2_SensorsPoll(void) {
    
    if (GPIOA->IDR & (1 << ACCEL_INT1)) {
        // Accel DATA_READY
        Accel_GetData();
    } else 
    if (GPIOE->IDR & (1 << ARS2_EXTI)) {
        // ARS2 DATA_READY
        ARS2_GetData();
    }
}

//-------------------------------------------------------------------
// ARS1
//-------------------------------------------------------------------

uint8_t ARS1_NSS = 9; // PD
uint8_t ARS1_VDD = 8; // PD

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
    
    ARS1_VDD_Init();
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

//-------------------------------------------------------------------
// ARS2
//-------------------------------------------------------------------

uint8_t ARS2_NSS    = 10; // PB
uint8_t ARS2_VDD    = 14; // PE
uint8_t ARS2_EXTI   = 15; // PE;

uint8_t ars2_regs[4] = {0x08, 0x09, 0x0a, 0x0b};
uint8_t ars2_data[4];

// 0 - first 2 bytes
// 1 - second 2 bytes
// 2 - finished
uint8_t ars2_dma_status = 0;

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

void ARS2_EXTI_Init() {
    GPIOE->OSPEEDR |= 3 << ARS2_EXTI*2;
    
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PE;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR15; 
    EXTI->IMR 	|= EXTI_IMR_MR15;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ARS2_GetData() {
    ARS2_NSS_Low();
    ars2_data[0] = SPI2_Read(0x08);
    ars2_data[1] = SPI2_Read(0x09);
    ARS2_NSS_High();
    
    ARS2_NSS_Low();
    ars2_data[2] = SPI2_Read(0x0a);
    ars2_data[3] = SPI2_Read(0x0b);
    All_NSS_High();
    
    SPI2_SensorsPoll();
}

void EXTI15_10_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR15) {
        EXTI->PR = EXTI_PR_PR15;

        if ((SPI2->SR & SPI_SR_BSY) == 0) {
            ARS2_GetData();
        } 
//        ars2_dma_status = 0;
//        ARS2_NSS_Low();
//        ARS2_DMA_Init();
    }
}

void ARS2_DMA_Init() {
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x05);    
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1->HIFCR = DMA_HIFCR_CFEIF4;
    
    DMA1_Stream4->CR    = 0;
    DMA1_Stream3->CR    = 0;
    
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR  = (uint32_t)(ars2_data /*+ ars2_dma_status*2*/);
    DMA1_Stream3->NDTR  = 2;
    DMA1_Stream3->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
    
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->M0AR  = (uint32_t)(ars2_regs /*+ ars2_dma_status*2*/);     
    DMA1_Stream4->NDTR  = 2;
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL | DMA_SxCR_EN;     
}

