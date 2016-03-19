#include "stm32f4xx.h" 
#include "mpu9250.h"

uint8_t SPI2_SCK    = 13;   // PB
uint8_t SPI2_MISO   = 14;   // PB
uint8_t SPI2_MOSI   = 15;   // PB
uint8_t IMU_NSS     = 12;   // PB
uint8_t IMU_INT     = 1;    // PA

#define READ_COMMAND            0x80
#define WRITE_COMMAND           0x00

#define WHO_AM_I                0x75

#define PWR_MGMT_1              0x68
#define PWR_MGMT_1_VALUE        0x01

#define CONFIG                  0x1a
#define CONFIG_VALUE            0x03

#define SMPLRT_DIV              0x19
#define SMPLRT_DIV_VALUE        0x04

#define GYRO_CONFIG             0x1b
#define GYRO_CONFIG_VALUE       0x00

#define ACCEL_CONFIG            0x1c
#define ACCEL_CONFIG_VALUE      0x18

#define ACCEL_CONFIG_2          0x1d
#define ACCEL_CONFIG_2_VALUE    0x03

#define INT_PIN_CFG             0x37
#define INT_PIN_CFG_VALUE       0x30

#define INT_ENABLE              0x38
#define INT_ENABLE_VALUE        0x01

#define I2C_SLV0_DO             0x63
#define I2C_SLV0_DO_VALUE       0x16

#define USER_CTRL               0x6a
#define USER_CTRL_VALUE         0x20

#define BANK_SEL                0x6d
#define MEM_R_W                 0x6f
#define PRGM_START_H            0x70

#define I2C_MST_CTRL            0x24
#define I2C_MST_CTRL_VALUE      0x40

#define I2C_SLV0_ADDR           0x25      
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27
#define I2C_SLV0_CTRL_1_BYTE    0x81
#define I2C_SLV0_CTRL_7_BYTES   0x87

#define EXT_SENS_DATA_00        0x49

#define AK8963_I2C_ADDRESS      0x0c

#define AK8963_CNTL1            0x0a
#define AK8963_CNTL1_VALUE      0x16
#define AK8963_HXL              0x03
#define AK8963_ASAX             0x10

#define GYRO_SENSITIVITY        131.0f      // LSB/dps
#define ACCEL_SENSITIVITY       2048.0f     // LSB/g
#define MAG_SENSITIVITY         0.15f       // uT/LSB
#define TEMP_SENSITIBITY        338.87f     // LSB/degC
#define TEMP_OFFSET             21.0f       // degC

void Delay_ms(uint16_t ms);
void Delay_us(uint16_t us);

uint16_t imu_dma_tx[12] = {0xba00, 0, 0, 0, 0, 0, 0, 0, 0xc900, 0, 0, 0};
uint16_t imu_dma_rx[12];

float accel[3];
float temp;
float angleRate[3];
float magField[3];

float mag_sens_adj[3];


void IMU_NSS_Init() {
    GPIOB->MODER    |= 1 << IMU_NSS*2;
    GPIOB->OSPEEDR 	|= 3 << IMU_NSS*2;
    GPIOB->OTYPER	&= ~(1 << IMU_NSS);
}

void IMU_NSS_Low() {
    GPIOB->BSRRH |= (1 << IMU_NSS);
}

void IMU_NSS_High() {
    GPIOB->BSRRL |= (1 << IMU_NSS);
}

void SPI2_GPIO_Init() {
	GPIOB->MODER 	|= (2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) | (2 << SPI2_MOSI*2);
	GPIOB->OSPEEDR 	|= (3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) | (3 << SPI2_MOSI*2);
	GPIOB->AFR[1] 	|= (5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) | (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~((1 << SPI2_SCK) | (1 << SPI2_MISO) | (1 << SPI2_MOSI)); 
    GPIOB->PUPDR    |= (1 << SPI2_MISO*2); // Pull-up MISO
	
	IMU_NSS_High();
}

void SPI2_Init() {
    SPI2_GPIO_Init();
    
	SPI2->CR1 = 0;
	SPI2->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	
	SPI2->CR1 |= SPI_CR1_BR_1; 							                        // baudrate = Fpclk / 8
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI                  
}

uint16_t SPI2_Transfer(uint16_t byte) { 
	while ((SPI2->SR & SPI_SR_TXE)==0);
	SPI2->DR = byte;
	
	while ((SPI2->SR & SPI_SR_RXNE)==0);
	return (SPI2->DR);
}

uint8_t SPI2_Read(uint8_t address) {
    uint16_t tmp = 0;
    
    address |= READ_COMMAND;
    tmp = SPI2_Transfer(address << 8);
    
    return tmp & 0xFF;
}

void SPI2_Write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |= data;
    SPI2_Transfer(tmp);
}

uint8_t IMU_ReadByte(uint8_t address) {
    uint8_t retval = 0;
    IMU_NSS_Low();
    retval = SPI2_Read(address);
    IMU_NSS_High();
    return retval;
}

void IMU_WriteByte(uint8_t address, uint8_t data) { 
    IMU_NSS_Low();
    SPI2_Write(address, data);
    IMU_NSS_High();
}

uint8_t imu_test = 0;
void IMU_Init() {
    // WHO_AM_I
    //imu_test = IMU_ReadByte(0x85);
    
    // PLL as clock source
    IMU_WriteByte(PWR_MGMT_1, PWR_MGMT_1_VALUE);
    //imu_test = IMU_ReadByte(PWR_MGMT_1);

    // 1000 Hz sample rate, 41 Hz gyro bandwidth
    IMU_WriteByte(CONFIG, CONFIG_VALUE);
    //imu_test = IMU_ReadByte(CONFIG);   
    
    // divisor = 5, sample rate -> 200 Hz
    IMU_WriteByte(SMPLRT_DIV, SMPLRT_DIV_VALUE);
    //imu_test = IMU_ReadByte(SMPLRT_DIV);
    
    // gyro sensitivity - 250 dps
    IMU_WriteByte(GYRO_CONFIG, GYRO_CONFIG_VALUE);
    //imu_test = IMU_ReadByte(GYRO_CONFIG);
    
    // accel sensitivity - +-16g
    IMU_WriteByte(ACCEL_CONFIG, ACCEL_CONFIG_VALUE);
    //imu_test = IMU_ReadByte(ACCEL_CONFIG);
    
    // 41 Hz accel bandwidth, 1000 Hz sample rate
    IMU_WriteByte(ACCEL_CONFIG_2, ACCEL_CONFIG_2_VALUE);
    //imu_test = IMU_ReadByte(ACCEL_CONFIG_2);
    
    // Interrupt implemented by constant level, not pulses
    IMU_WriteByte(INT_PIN_CFG, INT_PIN_CFG_VALUE);
    //imu_test = IMU_ReadByte(INT_PIN_CFG);
    
    // Raw data ready interrupt enable
    IMU_WriteByte(INT_ENABLE, INT_ENABLE_VALUE);
    //imu_test = IMU_ReadByte(INT_ENABLE);
}

void IMU_EXTI_Init() {
    GPIOA->OSPEEDR |= 3 << IMU_INT*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR  |= EXTI_RTSR_TR1;       // rising
    EXTI->IMR   |= EXTI_IMR_MR1;        // non-masking
    NVIC_SetPriority(EXTI1_IRQn, 0x02);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void IMU_MultiRead(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;
    uint16_t tmp = 0;
    IMU_NSS_Low();
    data[0] = SPI2_Read(address);
    for (i = 0; i < size/2; i++) {
        tmp = SPI2_Transfer(0x0000);
        data[2*i+1] = tmp >> 8;
        data[2*i+2] = tmp & 0xff;
    }
    IMU_NSS_High();
}

void IMU_MultiWrite(uint8_t address, uint8_t *data, uint8_t size) {
    uint8_t i = 0;
    uint16_t tmp = 0;
    
    IMU_NSS_Low();
    SPI2_Write(address, data[0]);
    if (size % 2 == 0) {
        IMU_NSS_High();
        IMU_NSS_Low();
        SPI2_Write(address+1, data[1]);
        for (i = 1; i < size/2; i++) {
            tmp = (data[2*i] << 8) | data[2*i+1];
            SPI2_Transfer(tmp);
        }
    } else for (i = 0; i < size/2; i++) {
        tmp = (data[2*i+1] << 8) | data[2*i+2]; 
        SPI2_Transfer(tmp);
    }
    IMU_NSS_High();
}

void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        
        GPIOA->BSRRL |= 1 << 15;
        IMU_DMA_Run(imu_dma_tx, imu_dma_rx, 12);
    }
}


void IMU_DMA_Init() {
    SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x02);    
    NVIC_SetPriority(DMA1_Stream4_IRQn, 0x02);    
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->CR    = DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_TCIE | DMA_SxCR_PL; 
    
    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL; 
}

void IMU_DMA_Run(uint16_t *tx, uint16_t *rx, uint8_t size) {
    IMU_NSS_Low();
    
    DMA1_Stream3->M0AR  = (uint32_t)rx;
    DMA1_Stream3->NDTR  = size;
    DMA1_Stream3->CR    |= DMA_SxCR_EN; 

    DMA1_Stream4->M0AR  = (uint32_t)tx;     
    DMA1_Stream4->NDTR  = size;
    DMA1_Stream4->CR    |= DMA_SxCR_EN;     
}

int16_t swapHighLow(int16_t data) {
    return (data << 8) | (data >> 8);
}

void DMA1_Stream3_IRQHandler() {
    uint8_t i = 0;
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        IMU_NSS_High();
        
        for (i = 0; i < 3; i++) {
            accel[i] = (int16_t)imu_dma_rx[1+i] / ACCEL_SENSITIVITY;
        }
        
        temp = (int16_t)imu_dma_rx[4] / TEMP_SENSITIBITY + TEMP_OFFSET;
        
        for (i = 0; i < 3; i++) {
            angleRate[i] = (int16_t)imu_dma_rx[5+i] / GYRO_SENSITIVITY;
        }
        
        for (i = 0; i < 3; i++) {
            magField[i] = (int16_t)swapHighLow(imu_dma_rx[8+i]) * MAG_SENSITIVITY * mag_sens_adj[i];
        }
    }
}

void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    } 
}

void Mag_WriteByte(uint8_t address, uint8_t data) {
    // Mag I2C address
    IMU_WriteByte(I2C_SLV0_ADDR, WRITE_COMMAND | AK8963_I2C_ADDRESS);
    IMU_WriteByte(I2C_SLV0_REG, address);
    IMU_WriteByte(I2C_SLV0_DO, data);
    // I2C on, 1 byte
    IMU_WriteByte(I2C_SLV0_CTRL, I2C_SLV0_CTRL_1_BYTE);
}

void Mag_Init() {
    uint8_t tmp[3], i = 0;
    
    // Enable I2C master
    IMU_WriteByte(USER_CTRL, USER_CTRL_VALUE);
    
    Mag_WriteByte(AK8963_CNTL1, 0x00); // Power down magnetometer  
    Delay_ms(10);   
    Mag_WriteByte(AK8963_CNTL1, 0x0f); // Enter Fuse ROM access mode
    Delay_ms(10);
    
    // I2C address for reading
    IMU_WriteByte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDRESS);
    // reading from ASAX register
    IMU_WriteByte(I2C_SLV0_REG, AK8963_ASAX);
    // Read 3 bytes
    IMU_WriteByte(I2C_SLV0_CTRL, 0x83);
    Delay_ms(10);
    IMU_MultiRead(EXT_SENS_DATA_00, tmp, 3);
    
    for (i = 0; i < 3; i++) {
        mag_sens_adj[i] = (tmp[i] - 128)*0.5 / 128.0f + 1.0f;
    }
    
    Mag_WriteByte(AK8963_CNTL1, 0x00); // Power down magnetometer  
    Delay_ms(10);
    

    Mag_WriteByte(AK8963_CNTL1, AK8963_CNTL1_VALUE); // Continuous measurement mode 2 (100 Hz), 16 bit output
    Delay_ms(10);
    
    // Data ready interrupt waits for external sensor data
    IMU_WriteByte(I2C_MST_CTRL, I2C_MST_CTRL_VALUE);
    // I2C address for reading
    IMU_WriteByte(I2C_SLV0_ADDR, READ_COMMAND | AK8963_I2C_ADDRESS);
    // reading from HXL register
    IMU_WriteByte(I2C_SLV0_REG, AK8963_HXL);
    // Read 7 bytes
    IMU_WriteByte(I2C_SLV0_CTRL, I2C_SLV0_CTRL_7_BYTES);
}
