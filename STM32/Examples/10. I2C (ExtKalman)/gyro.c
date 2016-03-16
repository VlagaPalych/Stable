#include "stm32f30x.h"
#include "gyro.h"
#include "extra_math.h"

uint8_t SPI1_SCK    = 5; // PA
uint8_t SPI1_MOSI   = 7; // PA
uint8_t SPI1_MISO   = 6; // PA
uint8_t GYRO_NSS    = 3; // PE
uint8_t GYRO_INT1   = 0; // PE;
uint8_t GYRO_INT2   = 1; // PE

#define GYRO_READ_COMMAND   0x80
#define GYRO_WRITE_COMMAND  0x00
#define GYRO_MULTI_COMMAND  0x40

#define GYRO_CTRL_REG1          0x20
#define GYRO_CTRL_REG1_VALUE    0x1f
#define GYRO_CTRL_REG3          0x22
#define GYRO_CTRL_REG3_VALUE    0x08
#define GYRO_CTRL_REG4          0x23
#define GYRO_CTRL_REG4_VALUE    0x30

#define GYRO_SENSITIVITY 8.75e-3//70e-3 // mdps per lsb

uint8_t gyro_data[9];
float gyro_angleRate[3];

// Bias depends on temperature
// Bias = gyro_k*temp + gyro_b;
int8_t gyro_temp;
float gyro_k[3] = {0.007173716852707, -0.002663790962610, 0.024035829009952};
float gyro_b[3] = {0.156616579931222, 0.882064466464646, 0.946608336010653};
float gyro_axis_sensitivity[3] = {0.971826171252673, 1.133234173275460, 0.940997344975712};

uint16_t gyro_dma_tx[5] = {0xe600, 0x0000, 0x0000, 0x0000, 0x0000};
uint16_t gyro_dma_rx[5];


void SPI1_GPIO_Init() {
    GPIOA->MODER 	|= (2 << SPI1_SCK*2) | (2 << SPI1_MISO*2) | (2 << SPI1_MOSI*2);
	GPIOA->OSPEEDR 	|= (3 << SPI1_SCK*2) | (3 << SPI1_MISO*2) | (3 << SPI1_MOSI*2);
	GPIOA->AFR[0] 	|= (5 << SPI1_SCK*4) | (5 << SPI1_MISO*4) | (5 << SPI1_MOSI*4);                         // AF5
	GPIOA->OTYPER	&= ~((1 << SPI1_SCK) | (1 << SPI1_MISO) | (1 << SPI1_MOSI)); 
    GPIOA->PUPDR    |= 2 << SPI1_MOSI;
    
    GPIOE->MODER    |= 1 << GYRO_NSS*2;
    GPIOE->OSPEEDR 	|= 3 << GYRO_NSS*2;
    GPIOE->OTYPER	&= ~(1 << GYRO_NSS);
}

void Gyro_NSS_Low() {
    GPIOE->BSRR |= (1 << GYRO_NSS) << 16;
}

void Gyro_NSS_High() {
    GPIOE->BSRR |= (1 << GYRO_NSS);
}

void SPI1_Init() {
    SPI1_GPIO_Init();
    
    SPI1->CR1 = 0;
	SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 |SPI_CR2_DS_1 | SPI_CR2_DS_0;       // 16 bits
    SPI1->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8 = 72 / 8 = 9 MHz
    SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;										        // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR2 |= SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI1->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI
}

uint16_t SPI1_Transfer(uint16_t data) {
    while ((SPI1->SR & SPI_SR_TXE) == 0);
    SPI1->DR = data;
    
    while ((SPI1->SR & SPI_SR_RXNE) == 0);
    return SPI1->DR;
}

uint8_t Gyro_Read(uint8_t address) {
    uint16_t tmp = 0;
    tmp = address | GYRO_READ_COMMAND;
    tmp = tmp << 8;
    tmp = SPI1_Transfer(tmp);
    return tmp & 0xff;
}

void Gyro_MultiRead(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;
    uint16_t tmp = 0;
    tmp = address | GYRO_READ_COMMAND | GYRO_MULTI_COMMAND;
    tmp = tmp << 8;
    tmp = SPI1_Transfer(tmp);
    
    data[0] = tmp & 0xff;
    for (i = 0; i < size / 2; i++) {
        tmp = SPI1_Transfer(0x0000);
        data[2*i+1] = tmp >> 8;
        data[2*i+2] = tmp & 0xff;
    }
}

void Gyro_Write(uint8_t address, uint8_t data) {
    uint16_t tmp = 0;
    tmp = address | GYRO_WRITE_COMMAND;
    tmp = (tmp << 8) | data;
    SPI1_Transfer(tmp);
}

uint8_t gyro_test = 0;
void Gyro_Init() {
    SPI1_Init();
    
    // WHOAMI test
    Gyro_NSS_Low();
    gyro_test = Gyro_Read(0x0f);
    Gyro_NSS_High();
    
    // Normal mode
    // Output data rate - 95 Hz
    // Cutoff frequency - 25 Hz
    Gyro_NSS_Low();
    Gyro_Write(GYRO_CTRL_REG1, GYRO_CTRL_REG1_VALUE);
    Gyro_NSS_High();
    Gyro_NSS_Low();
    gyro_test = Gyro_Read(GYRO_CTRL_REG1);
    Gyro_NSS_High();
    
    // Data ready interrupt on INT2
    Gyro_NSS_Low();
    Gyro_Write(GYRO_CTRL_REG3, GYRO_CTRL_REG3_VALUE);
    Gyro_NSS_High();
    Gyro_NSS_Low();
    gyro_test = Gyro_Read(GYRO_CTRL_REG3);
    Gyro_NSS_High();
    
    // Sensitivity 2000 dps
//    Gyro_NSS_Low();
//    Gyro_Write(GYRO_CTRL_REG4, GYRO_CTRL_REG4_VALUE);
//    Gyro_NSS_High();
//    Gyro_NSS_Low();
//    gyro_test = Gyro_Read(GYRO_CTRL_REG4);
//    Gyro_NSS_High();
    
    Gyro_DMA_Init();
}

void Gyro_EXTI_Init() {
    GPIOE->OSPEEDR |= 3 << GYRO_INT2*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PE;
    
    EXTI->RTSR  |= EXTI_RTSR_TR1;       // rising
    EXTI->IMR   |= EXTI_IMR_MR1;        // non-masking
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 0x02);
}

void Gyro_GetData() {
    uint8_t i = 0;
    int16_t tmp = 0;
    
    Gyro_NSS_Low();
    Gyro_MultiRead(0x27, gyro_data, 7);
    Gyro_NSS_High();
    
    for (i = 0; i < 3; i++) {
        tmp = (gyro_data[2*i+2] << 8) | gyro_data[2*i+1];
        gyro_angleRate[i] = tmp * GYRO_SENSITIVITY;
    }
}

void Gyro_DMA_Init() {
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 0x02); 
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 0x02); 
      
    DMA1_Channel2->CPAR = (uint32_t)(&SPI1->DR);
    DMA1_Channel2->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE; // | DMA_CCR_EN;
    
    DMA1_Channel3->CPAR = (uint32_t)(&SPI1->DR);
    DMA1_Channel3->CCR =  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE; // | DMA_CCR_EN;
}

void Gyro_DMA_Run() {
    Gyro_NSS_Low();
    
    DMA1_Channel2->CMAR = (uint32_t)(gyro_dma_rx);
    DMA1_Channel2->CNDTR = 5;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    
    DMA1_Channel3->CMAR = (uint32_t)(gyro_dma_tx);
    DMA1_Channel3->CNDTR = 5;
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void DMA1_Channel2_IRQHandler() {
    uint8_t i = 0;
    int16_t tmp = 0;
    float swap = 0;
    
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR |= DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        Gyro_NSS_High();       
        
        gyro_temp = gyro_dma_rx[0] & 0xff;
        
        gyro_data[0] = gyro_dma_rx[1] >> 8;
        gyro_data[1] = gyro_dma_rx[1] & 0xff;
        gyro_data[2] = gyro_dma_rx[2] >> 8;
        gyro_data[3] = gyro_dma_rx[2] & 0xff;
        gyro_data[4] = gyro_dma_rx[3] >> 8;
        gyro_data[5] = gyro_dma_rx[3] & 0xff;
        gyro_data[6] = gyro_dma_rx[4] >> 8;
        
        for (i = 0; i < 3; i++) {
            tmp = (gyro_data[2*i+2] << 8) | gyro_data[2*i+1];
            gyro_angleRate[i] = tmp * GYRO_SENSITIVITY; //degrees
            //gyro_angleRate[i] -= (gyro_k[i] * (-gyro_temp) + gyro_b[i]); 
        }
        swap = gyro_angleRate[1];
        gyro_angleRate[1] = -gyro_angleRate[0];
        gyro_angleRate[0] = swap;
        
        for (i = 0; i < VECT_SIZE; i++) {
            gyro_angleRate[i] -= (gyro_k[i] * (-gyro_temp) + gyro_b[i]); 
            //gyro_angleRate[i] *= gyro_axis_sensitivity[i];
        }
        
        degrees_to_radians(gyro_angleRate);
    }
}

void DMA1_Channel3_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF3) {
        DMA1->IFCR |= DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3;
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    }
}

void EXTI1_IRQHandler() {  
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        //Gyro_GetData();
        Gyro_DMA_Run();
    }
}


