#include "spi.h"
#include "stm32f4xx.h"

void MPU_NSS_Init() {
    GPIOB->MODER    |= 1 << MPU_NSS*2;
    GPIOB->OSPEEDR 	|= 3 << MPU_NSS*2;
    GPIOB->OTYPER	&= ~(1 << MPU_NSS);
}

void MPU_NSS_Low() {
    GPIOB->BSRRH |= (1 << MPU_NSS);
}

void MPU_NSS_High() {
    GPIOB->BSRRL |= (1 << MPU_NSS);
}

void SPI2_GPIO_Init() {
	GPIOB->MODER 	|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~((1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI)); 
    GPIOB->PUPDR    |= (1 << SPI2_MISO*2); // Pull-up MISO
	
    MPU_NSS_Init();
	MPU_NSS_High();
}

void SPI2_Init() {
    SPI2_GPIO_Init();
    
	SPI2->CR1 = 0;
	//SPI2->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	
	SPI2->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}

uint8_t SPI2_Transfer(uint8_t byte) { 
	while ((SPI2->SR & SPI_SR_TXE)==0);
	SPI2->DR = byte;
	
	while ((SPI2->SR & SPI_SR_RXNE)==0);
	return (SPI2->DR);
}

uint8_t SPI2_Read(uint8_t address) {
    uint8_t tmp = 0;
    address |= READ_COMMAND;
    SPI2_Transfer(address);
    tmp = SPI2_Transfer(0x00); 
    return tmp;
}

void SPI2_Write(uint8_t address, uint8_t data) {
    address |= WRITE_COMMAND;
    SPI2_Transfer(address);
    SPI2_Transfer(data);
}

uint8_t MPU_ReadByte(uint8_t address) {
    uint8_t retval = 0;
    MPU_NSS_Low();
    retval = SPI2_Read(address);
    MPU_NSS_High();
    return retval;
}

void MPU_WriteByte(uint8_t address, uint8_t data) { 
    MPU_NSS_Low();
    SPI2_Write(address, data);
    MPU_NSS_High();
}

void MPU_Read(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;
    MPU_NSS_Low();
    data[0] = SPI2_Read(address);
    for (i = 1; i < size; i++) {
        data[i] = SPI2_Transfer(0x00);
    }
    MPU_NSS_High();
}

void MPU_Write(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;    
    MPU_NSS_Low();
    SPI2_Write(address, data[0]);
    for (i = 1; i < size; i++) {
        SPI2_Transfer(data[i]);
    }
    MPU_NSS_High();
}
