#include "adxrs453.h"
#include "stm32f4xx.h" 
#include "filters.h"
#include "telemetry.h"

uint8_t SPI3_SCK = 10; // PC
uint8_t SPI3_MISO = 11; // PC
uint8_t SPI3_MOSI = 12; // PC
uint8_t SPI3_NSS = 15; // PA

float adxrs_Sum = 0;
uint16_t adxrs_CalibrIndex = 0;
uint16_t adxrs_CalibrNumber = 0;
uint8_t adxrs_CalibrationOn = 0;
float adxrs_Offset = 0;

void SPI3_GPIO_Init() {
    GPIOC->MODER 	|= (2 << SPI3_SCK*2) | (2 << SPI3_MISO*2) | (2 << SPI3_MOSI*2);
	GPIOC->OSPEEDR 	|= (3 << SPI3_SCK*2) | (3 << SPI3_MISO*2) | (3 << SPI3_MOSI*2);
	GPIOC->AFR[1] 	|= (6 << (SPI3_SCK-8)*4) | (6 << (SPI3_MISO-8)*4) | (6 << (SPI3_MOSI-8)*4);                         // AF6
	GPIOC->OTYPER	&= ~((1 << SPI3_SCK) | (1 << SPI3_MISO) | (1 << SPI3_MOSI)); 
    GPIOC->PUPDR    |= (1 << SPI3_MISO*2);
    
    GPIOA->MODER &= ~(3 << SPI3_NSS*2);
    GPIOA->MODER |= 1 << SPI3_NSS*2;
    GPIOA->OSPEEDR |= 3 << SPI3_NSS*2;
    GPIOA->OTYPER |= 1 << SPI3_NSS;
}

void SPI3_Init() {
    SPI3_GPIO_Init();
	
	SPI3->CR1 = 0;
	SPI3->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	SPI3->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	SPI3->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8 = 32 / 8 = 4 MHz
	//SPI3->CR1 |= SPI_CR1_CPOL;													// polarity
	//SPI3->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI3->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI3->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI3->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}

void SPI3_NSS_Low() {
    GPIOA->BSRRH |= 1 << SPI3_NSS;
}

void SPI3_NSS_High() {
    GPIOA->BSRRL |= 1 << SPI3_NSS;
}

uint16_t SPI3_Transfer(uint16_t data) { 
    
	while ((SPI3->SR & SPI_SR_TXE)==0);
	SPI3->DR = data;
	
	while ((SPI3->SR & SPI_SR_RXNE)==0);
	return (SPI3->DR);
}

uint16_t ADXRS453_Read(uint8_t address) {
    uint16_t send = 0;
    uint16_t recv = 0;
    uint16_t d15_d11 = 0, d10_d0 = 0;
    uint16_t p = 0;
    uint16_t p0 = 0, p1 = 0;
    uint16_t i = 0;
    
    for (i = 0; i < 8; i++) {
        p ^= address & (1 << i);
    }
    
    send = (1 << 15) | (address << 1);
    
    SPI3_NSS_Low();
    recv = SPI3_Transfer(send);
    d15_d11 = recv & 0x001f;  
    recv = SPI3_Transfer(p);
    d10_d0 = recv & 0xffe0;
    SPI3_NSS_High();

    return d10_d0 | (d15_d11 << 11);
}


uint16_t adxrsCommands[2] =  {0x8000, 0x0000}; //{0xf001, 0x0000};
uint16_t adxrsResponses[2];

uint8_t transferFinished = 1;

void ADXRS_DMA_Read() {
    if (transferFinished) {
        transferFinished = 0;
        NVIC_SetPriority(DMA1_Stream2_IRQn, 0x03);
        NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        
        SPI3_NSS_Low();
        
        DMA1_Stream2->CR    = 0;
        DMA1_Stream5->CR    = 0;

        
        DMA1_Stream2->PAR   = (uint32_t)&(SPI3->DR);
        DMA1_Stream2->M0AR  = (uint32_t)adxrsResponses;     
        DMA1_Stream2->NDTR  = 2;
        DMA1_Stream2->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
                                    DMA_SxCR_TCIE /*| DMA_SxCR_PFCTRL*/ | DMA_SxCR_PL | DMA_SxCR_EN;

        DMA1_Stream5->PAR   = (uint32_t)&(SPI3->DR);
        DMA1_Stream5->M0AR  = (uint32_t)adxrsCommands;
        DMA1_Stream5->NDTR  = 2;
        DMA1_Stream5->CR    |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                    DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN;  
    }        
}

int16_t adxrs_data = 0;
uint8_t adxrsProcessCounter = 0;
uint8_t adxrsLowpassReady = 0;

void DMA1_Stream2_IRQHandler()  {
    uint16_t d15_d11 = 0;
    uint16_t d10_d0 = 0;
    if (DMA1->LISR & DMA_LISR_TCIF2) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2;
        DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;
        
        SPI3_NSS_High();
        
        d15_d11 = adxrsResponses[0] & 0x001f;
        d10_d0 = adxrsResponses[1] & 0xffe0;
        adxrs_data = (int16_t)((d15_d11 << 11) | (d10_d0 >> 5));
        
        transferFinished = 1;
        
        adxrs453_history[adxrs453_history_index] = adxrs_data;
        adxrs453_history_index++;
        
        if (adxrs453_history_index >= ADXRS453_FILTER_SIZE) {
            // enough data for filtering
            adxrsLowpassReady = 1;
            if (accel_history_index >= 2*ADXRS453_FILTER_SIZE) {
                // transition to beginning of array required
                memcpy(adxrs453_history, &adxrs453_history[ADXRS453_FILTER_SIZE], ADXRS453_FILTER_SIZE*sizeof(float)); 
                adxrs453_history_index = ADXRS453_FILTER_SIZE;
            }
        }
        
        adxrsProcessCounter++;
        if (adxrsProcessCounter == ADXRS453_DECIMATION) {
            adxrsProcessCounter = 0;
            
            adxrs453_history_filter_index = adxrs453_history_index;
            if (lowpassOn && adxrsLowpassReady) {
                doAdxrsProcess = 1;
            }
        } 
    }
}

void DMA1_Stream5_IRQHandler()  {
    if (DMA1->HISR & DMA_HISR_TCIF5) {
      
    }
}

void ADXRS_TIM_Init() {
    TIM2->PSC = 63;
    TIM2->ARR = 2500;
    TIM2->DIER |= 1;
    NVIC_SetPriority(TIM2_IRQn, 0x04);
    NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        ADXRS_DMA_Read();
    }
}

void ADXRS_Calibr(void) {
    adxrs_Sum = 0;
    adxrs_CalibrIndex = 0;
    adxrs_CalibrNumber = 400;
    adxrs_CalibrationOn = 1;
}