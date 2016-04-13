#include "spi.h"
#include "stm32f4xx.h"

extern "C" void mpu_nss_init() {
    GPIOB->MODER    |= 1 << MPU_NSS*2;
    GPIOB->OSPEEDR 	|= 3 << MPU_NSS*2;
    GPIOB->OTYPER	&= ~(1 << MPU_NSS);
}

extern "C" void mpu_nss_low() {
    GPIOB->BSRRH |= (1 << MPU_NSS);
}

extern "C" void mpu_nss_high() {
    GPIOB->BSRRL |= (1 << MPU_NSS);
}

extern "C" void spi2_gpio_init() {
	GPIOB->MODER 	|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~((1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI)); 
    GPIOB->PUPDR    |= (1 << SPI2_MISO*2); // Pull-up MISO
	
    mpu_nss_init();
	mpu_nss_high();
}

extern "C" void spi2_init() {
    spi2_gpio_init();
    
	SPI2->CR1 = 0;
	
    //@TODO: max frequency
	SPI2->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}

extern "C" uint8_t spi2_transfer(uint8_t byte) { 
	while ((SPI2->SR & SPI_SR_TXE)==0);
	SPI2->DR = byte;
	
	while ((SPI2->SR & SPI_SR_RXNE)==0);
	return (SPI2->DR);
}

extern "C" uint8_t spi2_read(uint8_t address) {
    uint8_t tmp = 0;
    address |= READ_COMMAND;
    spi2_transfer(address);
    tmp = spi2_transfer(0x00); 
    return tmp;
}

extern "C" void spi2_write(uint8_t address, uint8_t data) {
    address |= WRITE_COMMAND;
    spi2_transfer(address);
    spi2_transfer(data);
}

extern "C" uint8_t mpu_read_byte(uint8_t address) {
    uint8_t retval = 0;
    mpu_nss_low();
    retval = spi2_read(address);
    mpu_nss_high();
    return retval;
}

extern "C" void mpu_write_byte(uint8_t address, uint8_t data) { 
    mpu_nss_low();
    spi2_write(address, data);
    mpu_nss_high();
}

extern "C" void mpu_read(uint8_t address, uint8_t *data, uint16_t size) {
    mpu_nss_low();
    data[0] = spi2_read(address);
    for (uint16_t i = 1; i < size; i++) {
        data[i] = spi2_transfer(0x00);
    }
    mpu_nss_high();
}

extern "C" void mpu_write(uint8_t address, uint8_t *data, uint16_t size) {
    mpu_nss_low();
    spi2_write(address, data[0]);
    for (uint16_t i = 1; i < size; i++) {
        spi2_transfer(data[i]);
    }
    mpu_nss_high();
}
