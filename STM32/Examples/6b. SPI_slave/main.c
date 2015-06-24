#include "stm32f30x.h"

uint32_t SPI2_SCK = 13;
uint32_t SPI2_MISO = 14;
uint32_t SPI2_MOSI = 15;
uint32_t SPI2_NSS = 12;

uint8_t led;

int main() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    
    GPIOE->MODER |= (1 << 8*2) | (1 << 9*2) | (1 << 10*2) | (1 << 11*2) |
                    (1 << 12*2) | (1 << 13*2) | (1 << 14*2) | (1 << 15*2);
    GPIOE->OTYPER &= ~((1 << 8*2) | (1 << 9*2) | (1 << 10*2) | (1 << 11*2) |
                    (1 << 12*2) | (1 << 13*2) | (1 << 14*2) | (1 << 15*2));
    GPIOE->OSPEEDR |= (3 << 8*2) | (3 << 9*2) | (3 << 10*2) | (3 << 11*2) |
                    (3 << 12*2) | (3 << 13*2) | (3 << 14*2) | (3 << 15*2);
    GPIOE->PUPDR |= (1 << 8*2) | (1 << 9*2) | (1 << 10*2) | (1 << 11*2) |
                    (1 << 12*2) | (1 << 13*2) | (1 << 14*2) | (1 << 15*2);
    
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	GPIOB->MODER 		|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2) | (2 << SPI2_NSS*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2) | (3 << SPI2_NSS*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4) | (5 << (SPI2_NSS-8)*4);
	GPIOB->OTYPER 	&= ~( (1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI) | (1 << SPI2_NSS));
    
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	SPI2->CR1 = 0;
	SPI2->CR2 = 0;
	
	SPI2->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0; 																	// baudrate = Fpclk / 256
	SPI2->CR1 |= SPI_CR1_CPOL;															 	// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;															 	// phase	
	SPI2->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;	// 8 bits
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);												 	// MSBFIRST	
	SPI2->CR2 |= SPI_CR2_RXNEIE;
	NVIC_EnableIRQ(SPI2_IRQn);
	SPI2->CR1 &= ~SPI_CR1_MSTR;
	SPI2->CR1 |= SPI_CR1_SPE;
    
    while (1) {
    }
}

void SPI2_IRQHandler() {
	if (SPI2->SR & SPI_SR_RXNE) {
        led = SPI2->DR;
        GPIOE->ODR ^= 1 << led;
	}
}