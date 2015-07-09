#include "adxl345.h"
#include "stm32f4xx.h" 

//void ADXL345_DMA_Init() {
//    GPIOE->BSRRH |= (1 << SPI1_NSS);
//    DMA2_Stream0->CR      = 0;
//    DMA2_Stream0->PAR     = (uint32_t)&(SPI1->DR);
//    DMA2_Stream0->M0AR    = (uint32_t)accel;///???
//    DMA2_Stream0->NDTR    = 4;
//    DMA2_Stream0->CR      = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
//                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN;
//    //   DMA2_Stream5->CR      =  0;
//    //DMA2->HIFCR=0xC40;
//    SPI1->CR2|=3;
//    DMA2_Stream5->M0AR    = (uint32_t)accelRegisters;//???
//     
//    DMA2_Stream5->NDTR    = 4;
//  DMA2_Stream5->CR      |= DMA_SxCR_EN;
//    
//   
//}

//double Aappr = 5.13e-7;
//double Bappr = 0.001;
//double Cappr = -1.81;
//double D = 0;
//double x1 = 0;
//double x2 = 0;

//double chooseRoot() {
//    if (D < 0) return 2000;
//    
//    x1 = (-Bappr - sqrt(D)) / (2 * Aappr);
//    x2 = (-Bappr + sqrt(D)) / (2 * Aappr);
//    
//    if (x1 > 1000 && x1 < 2000) return x1;
//    if (x2 > 1000 && x2 < 2000) return x2;
//    return 2000;
//}

//uint8_t angleCount = 0;
//double angleSum = 0;

//void ADXL345_ProcessData(){
////    ((uint8_t *)(&ax))[0] = (uint8_t)(accel[0] & 0xFF);
////    ((uint8_t *)(&ax))[1] = (uint8_t)(accel[1] & 0xFF);
////    ((uint8_t *)(&ay))[0] = (uint8_t)(accel[2] & 0xFF);
////    ((uint8_t *)(&ay))[1] = (uint8_t)(accel[3] & 0xFF);
////    ((uint8_t *)(&az))[0] = (uint8_t)(accel[4] & 0xFF);
////    ((uint8_t *)(&az))[1] = (uint8_t)(accel[5] & 0xFF);
//    ax = accel[1];
//    ay = accel[2];
//    az = accel[3];
//    ax -= xoff;
//    ay -= yoff;
//    az -= zoff;
//    angle = atan((double)ax / (double)az);
//    
//    angleSum += angle;
//    angleCount++;
//    if (angleCount == 8) {
//        angle = angleSum / angleCount; // true angle we work with
//        angleSum = 0;
//        angleCount = 0;
//        
//        if (firstAngleMeasurement) {
//            firstAngleMeasurement = 0;
//        } else {
//            angularVelocity = (angle - prevAngle) / MEASUREMENT_TIME;
//        }
//        prevAngle = angle;
//        
//        F = k1*angle + k2*angularVelocity;
//    
//        if (update) {
//            if (F > 0) {
//                TIM4->CCR1 = 1000;
//                pwm1 = 1000;
//                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
//                pwm2 = (int)chooseRoot();
//                TIM4->CCR3 = pwm2;
//            } else if (F < 0) {
//                TIM4->CCR3 = 1000;
//                pwm2 = 1000;
//                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
//                pwm1 = (int)chooseRoot();
//                TIM4->CCR1 = pwm1;
//            }
//            SEND_TELEMETRY_FLAG = 1;
//        }
//        
//        // Identification block
//        if (anglesAccumulated < 2) {
//            y[anglesAccumulated] = angle;
//            u[anglesAccumulated] = F;
//        } else {
//            y[2] = angle;
//            u[2] = F;
//            
//            row = anglesAccumulated - 2;
//            Afull[row*3 + 0] = (y[1] - y[0]) / MEASUREMENT_TIME;
//            Afull[row*3 + 1] = y[0];
//            Afull[row*3 + 2] = -u[0];
//            
//            Bfull[row] = (2*y[1] - y[2] - y[0]) / MEASUREMENT_TIME / MEASUREMENT_TIME;
//            
//            y[0] = y[1];
//            y[1] = y[2];
//            u[0] = u[1];
//            u[1] = u[2];
//            
//            if (anglesAccumulated == 11) {
//                system_solve(Afull, Bfull, w, 10, 3);
//                anglesAccumulated = 1;
//            }      
//        } 
//        anglesAccumulated++;
//    }
//}

//void EXTI0_IRQHandler() {
//    if (EXTI->PR & EXTI_PR_PR0) {
//        EXTI->PR = EXTI_PR_PR0;
//       
//        ADXL345_DMA_Init();
//    }
//}

//void DMA2_Stream5_IRQHandler() {
//    if (DMA2->HISR & DMA_HISR_TCIF5) {
//        DMA2->HIFCR = DMA_HIFCR_CTCIF5;
//    } 
//    if (DMA2->HISR & DMA_HISR_HTIF5) {
//        DMA2->HIFCR = DMA_HIFCR_CHTIF5;
//    }
//    
//}

//void DMA2_Stream0_IRQHandler() {
//    GPIOE->BSRRL |= (1 << SPI1_NSS);
//    if (DMA2->LISR & DMA_LISR_TCIF0) {
//        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
//        if((GPIOA->IDR & (1 << 1)) == 1) {  
//            ADXL345_DMA_Init();
//        }
//        
//        ADXL345_ProcessData();
//    }
//}
void Delay() {
  volatile uint32_t i;
  for (i=0; i != 0x70000; i++);
}

void ADXL345_AccelVCCInit() {
    GPIOC->MODER    |= 1 << ACCEL_VCC*2;
    GPIOC->OTYPER   &= ~(1 << ACCEL_VCC);
    GPIOC->OSPEEDR  |= 3 << ACCEL_VCC*2;
    GPIOC->PUPDR    |= 1 << ACCEL_VCC*2;
    
    GPIOC->BSRRL |= 1 << ACCEL_VCC;
    Delay();
}

void NSS_Low() {
    GPIOA->BSRRH |= (1 << SPI1_NSS);
}

void NSS_High() {
    GPIOA->BSRRL |= (1 << SPI1_NSS);
}

void SPI1_GPIO_Init() {
	GPIOA->MODER 	|= (2 << SPI1_SCK*2) | (2 << SPI1_MISO*2) | (2 << SPI1_MOSI*2) | (1 << SPI1_NSS*2);
	GPIOA->OSPEEDR 	|= (3 << SPI1_SCK*2) | (3 << SPI1_MISO*2) | (3 << SPI1_MOSI*2) | (3 << SPI1_NSS*2);
	GPIOA->AFR[0] 	|= (5 << SPI1_SCK*4) | (5 << SPI1_MISO*4) | (5 << SPI1_MOSI*4);                         // AF5
	GPIOA->OTYPER	&= ~((1 << SPI1_SCK) | (1 << SPI1_MISO) | (1 << SPI1_MOSI) | (1 << SPI1_NSS)); 
	
	NSS_High();
}

void SPI1_Init() {
    SPI1_GPIO_Init();
	
	SPI1->CR1 = 0;
	SPI1->CR1 |= SPI_CR1_DFF;                                                   // 16 bits
	
	SPI1->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

//    SPI1->CR2 |= SPI_CR2_TXDMAEN;
//    SPI1->CR2 |= SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI1->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI
}

void ADXL345_Init() {
    uint8_t test = 0;
    
    ADXL345_AccelVCCInit();
    SPI1_Init();  
    NSS_Low();
    //while(1) {
    //    NSS_Low();
        test = ADXL345_Read(DEVID_ADDRESS);
    //    NSS_High();
    //}

    ADXL345_Write(INT_MAPPING_ADDRESS, DATA_READY_INT0_MAPPING);
    test = ADXL345_Read(INT_MAPPING_ADDRESS);

    ADXL345_Write(POWER_CTL_ADDRESS, MEASUREMENT_MODE);
    test = ADXL345_Read(POWER_CTL_ADDRESS);


    ADXL345_Write(DATA_FORMAT_ADDRESS, FULL_RES_MODE);
    test = ADXL345_Read(DATA_FORMAT_ADDRESS);

    ADXL345_Write(BW_RATE_ADDRESS, ACCEL_FREQ);
    test = ADXL345_Read(BW_RATE_ADDRESS);


    ADXL345_Write(INT_ENABLE_ADDRESS, DATA_READY_INT);
    test = ADXL345_Read(INT_ENABLE_ADDRESS);

    NSS_High();
}

uint16_t SPI1_Transfer(uint16_t byte) { 
	while ((SPI1->SR & SPI_SR_TXE)==0);
	SPI1->DR = byte;
	
	while ((SPI1->SR & SPI_SR_RXNE)==0);
	return (SPI1->DR);
}

uint8_t ADXL345_Read(uint8_t address) {
    uint16_t tmp = 0;
    
    address |= READ_COMMAND;
    tmp = SPI1_Transfer(address << 8);
    
    return tmp & 0xFF;
}

void ADXL345_Write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |= data;
    SPI1_Transfer(tmp);
}

void ADXL345_GetAccel(int16_t *x, int16_t *y, int16_t *z) {
    NSS_Low();
    ((uint8_t *)x)[0] = ADXL345_Read(0x32);
    ((uint8_t *)x)[1] = ADXL345_Read(0x33);
    ((uint8_t *)y)[0] = ADXL345_Read(0x34);
    ((uint8_t *)y)[1] = ADXL345_Read(0x35);
    ((uint8_t *)z)[0] = ADXL345_Read(0x36);
    ((uint8_t *)z)[1] = ADXL345_Read(0x37);
    NSS_High();
}

void ADXL345_Calibr() {
    int i = 0;
    double xSum = 0, ySum = 0, zSum = 0;
    
    while (!(GPIOA->IDR & (1 << ACCEL_INT1))) {}
    ADXL345_GetAccel(&ax, &ay, &az); 
    xSum += ax;
    ySum += ay;
    zSum += az;
        
    for (i = 0; i < CALIBR_NUMBER; i++) {
        while (!(GPIOA->IDR & (1 << ACCEL_INT1))) {}
        ADXL345_GetAccel(&ax, &ay, &az); 
        xSum += ax;
        ySum += ay;
        zSum += az;  
    }         
    xOffset = (int16_t)(xSum / CALIBR_NUMBER);
    yOffset = (int16_t)(ySum / CALIBR_NUMBER);
    zOffset = (int16_t)(zSum / CALIBR_NUMBER) - 0xFF;
}

void Accel_EXTI_Init() {    
    GPIOA->OSPEEDR |= 3 << 1*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR1; 
    EXTI->IMR 	|= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI1_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI1_IRQn); 
}

void EXTI1_IRQHandler() {  //what to do when accelerometer is ready
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        ADXL345_GetAccel(&ax, &ay, &az);
        ax -= xOffset;
        ay -= yOffset;
        az -= zOffset;
        
        //ADXL345_ProcessData();
    }
}

