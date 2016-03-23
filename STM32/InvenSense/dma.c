#include "dma.h"
#include "spi.h"
#include "stm32f4xx.h"

void MPU_DMA_Init() {
    SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x02);    
    NVIC_SetPriority(DMA1_Stream4_IRQn, 0x02);    
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->CR    = DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL; 
    
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL; 
}

void MPU_DMA_Run(uint8_t *tx, uint8_t *rx, uint8_t size) {
    MPU_NSS_Low();
    
    DMA1_Stream3->M0AR  = (uint32_t)rx;
    DMA1_Stream3->NDTR  = size;
    DMA1_Stream3->CR    |= DMA_SxCR_EN; 

    DMA1_Stream4->M0AR  = (uint32_t)tx;     
    DMA1_Stream4->NDTR  = size;
    DMA1_Stream4->CR    |= DMA_SxCR_EN;     
}

void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    } 
}
