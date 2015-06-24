#include "stm32f30x.h"
#include "adxl345.h"

uint8_t SPI1_SCK = 5;
uint8_t SPI1_MISO = 6;
uint8_t SPI1_MOSI = 7;
uint8_t SPI1_NSS = 4;

uint8_t TEST_LEDS[] = {8, 9, 10, 11, 12, 13};

volatile void delay(){
    volatile uint16_t i;
    for(i=0;i<255;i++){
    __nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();}
}

void TestLeds_GPIO_Init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    
    GPIOE->MODER    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
    GPIOE->OTYPER   &= ~(1 << 8 | 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
	GPIOE->OSPEEDR  |= 3 << 8*2 | 3 << 9*2 | 3 << 10*2 | 3 << 11*2 | 3 << 12*2 | 3 << 13*2;
    GPIOE->PUPDR    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
}

void SPI_GPIO_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER 	|= (2 << SPI1_SCK*2) | (2 << SPI1_MISO*2) | (2 << SPI1_MOSI*2) | (1 << SPI1_NSS*2);
	GPIOA->OSPEEDR 	|= (3 << SPI1_SCK*2) | (3 << SPI1_MISO*2) | (3 << SPI1_MOSI*2) | (3 << SPI1_NSS*2);
	GPIOA->AFR[0] 	|= (5 << SPI1_SCK*4) | (5 << SPI1_MISO*4) | (5 << SPI1_MOSI*4);                         // AF5
	GPIOA->OTYPER	&= ~((1 << SPI1_SCK) | (1 << SPI1_MISO) | (1 << SPI1_MOSI) | (1 << SPI1_NSS)); 
	
	GPIOA->BSRR |= (1 << SPI1_NSS);   // NSS - HIGH
}

void NNS_Low() {
    GPIOA->BSRR |= (1 << SPI1_NSS) << 16;
}

void NSS_High() {
    GPIOA->BSRR |= (1 << SPI1_NSS);
}

void SPI_Init() {
    SPI_GPIO_Init();
    
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	SPI1->CR1 = 0;
	SPI1->CR2 = SPI_CR2_DS_3 |SPI_CR2_DS_2 |SPI_CR2_DS_1 | SPI_CR2_DS_0;        // 16 bits
	
	SPI1->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		
	//SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI1->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI
}

uint16_t SPI1_transfer(uint16_t byte) { 
	while ((SPI1->SR & SPI_SR_TXE)==0);
	SPI1->DR = byte;
	
	while ((SPI1->SR & SPI_SR_RXNE)==0);
	return (SPI1->DR);
}

uint8_t ADXL345_read(uint8_t address) {
    uint16_t tmp = 0;
    
    delay();
    NSS_Low();
    delay();
    
    address |= READ_COMMAND;
    tmp = SPI1_transfer(address << 8);
    
    delay();
    NSS_High();
    delay();
    
    return tmp & 0xFF;
}

void ADXL345_write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    delay();
    NSS_Low();
    delay();
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |=data;
    SPI1_transfer(tmp);
    
    delay();
    NSS_High();
    delay();
}

void ADXL345_Init() {
  uint8_t test = 0;
    
  SPI_Init();  
  TestLeds_GPIO_Init();
    
  test = ADXL345_read(DEVID_ADDRESS);
  if (test != DEVID) {
      GPIOE->BSRR |= (1 << TEST_LEDS[0]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[0]) << 16;
  }

  ADXL345_write(INT_MAPPING_ADDRESS, DATA_READY_INT0_MAPPING);
  test = ADXL345_read(INT_MAPPING_ADDRESS);
  if (test != DATA_READY_INT0_MAPPING) {
      GPIOE->BSRR |= (1 << TEST_LEDS[1]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[1]) << 16;
  }

  ADXL345_write(POWER_CTL_ADDRESS, MEASUREMENT_MODE);
  test = ADXL345_read(POWER_CTL_ADDRESS);
  if (test != MEASUREMENT_MODE) {
      GPIOE->BSRR |= (1 << TEST_LEDS[2]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[2]) << 16;
  }

  ADXL345_write(DATA_FORMAT_ADDRESS, FULL_RES_MODE);
  test = ADXL345_read(DATA_FORMAT_ADDRESS);
  if (test != FULL_RES_MODE) {
      GPIOE->BSRR |= (1 << TEST_LEDS[3]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[3]) << 16;
  }
  
  ADXL345_write(BW_RATE_ADDRESS, HZ100);
  test = ADXL345_read(BW_RATE_ADDRESS);
  if (test != HZ100) {
      GPIOE->BSRR |= (1 << TEST_LEDS[4]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[4]) << 16;
  }
  
  ADXL345_write(INT_ENABLE_ADDRESS, DATA_READY_INT);
  test = ADXL345_read(INT_ENABLE_ADDRESS);
  if (test != DATA_READY_INT) {
      GPIOE->BSRR |= (1 << TEST_LEDS[5]);
  } else {
      GPIOE->BSRR |= (1 << TEST_LEDS[5]) << 16;
  }
}


int main() {
	ADXL345_Init();
	
	while(1) {
	}
}
