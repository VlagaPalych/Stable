#include "adxrs453.h"
#include "stm32f4xx.h" 

void SPI3_GPIO_Init() {
    GPIOC->MODER 	|= (2 << SPI3_SCK*2) | (2 << SPI3_MISO*2) | (2 << SPI3_MOSI*2);
	GPIOC->OSPEEDR 	|= (3 << SPI3_SCK*2) | (3 << SPI3_MISO*2) | (3 << SPI3_MOSI*2);
	GPIOC->AFR[1] 	|= (6 << (SPI3_SCK-8)*4) | (6 << (SPI3_MISO-8)*4) | (6 << (SPI3_MOSI-8)*4);                         // AF6
	GPIOC->OTYPER	&= ~((1 << SPI3_SCK) | (1 << SPI3_MISO) | (1 << SPI3_MOSI)); 
    
    GPIOA->MODER |= 1 << SPI3_NSS*2;
    GPIOA->OSPEEDR |= 3 << SPI3_NSS*2;
    GPIOA->OTYPER |= 1 << SPI3_NSS;
}

void SPI3_Init() {
    SPI3_GPIO_Init();
	
	SPI3->CR1 = 0;
	SPI3->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	SPI3->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	//SPI2->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI3->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI3->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI3->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI3->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI3->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}