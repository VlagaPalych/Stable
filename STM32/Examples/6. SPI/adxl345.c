#include "stm32f30x.h"
#include "adxl345.h"
#include "timer.h"
#include "ws2812.h"
#include "math.h"
#include "string.h"

//#define DELAY_TIME 3 // Time of turning CS on
//---------------------------------------------------------------------------------
uint8_t ACCEL_VCC = 14;

uint8_t SPI1_SCK    = 5;
uint8_t SPI1_MISO   = 6;
uint8_t SPI1_MOSI   = 7;
uint8_t SPI1_NSS    = 4;

uint8_t TEST_LEDS[] = {8, 9, 10, 11, 12, 13};

int16_t x, y, z;
int16_t xOffset, yOffset, zOffset;

int16_t accelRegisters[6] = {0xB200, 0xB300, 0xB400, 0xB500, 0xB600, 0xB700};
int16_t accel[6];

//uint16_t cbufptr;
//double ang[0x0fff];
//---------------------------------------------------------------------------------

#define MEASUREMENT_TIME    1e-2
#define AVERAGE_COUNT       8
#define ACCEL_FREQ          HZ100
#define WINDOW_SIZE         3
#define PI                  3.14159
#define ANGLE_THRESHOLD     (PI / 90)
#define RPS_PER_LED         (PI / 2)
#define LABEL_COLOR         0x0000FF00
#define TAIL_COLOR          0x00FF00FF
#define TAIL_LEFT           1
#define ACCEL_EPS           65000
#define STD_G               0xFF

#define FULL_ANGLE          (PI / 2)
#define ANGLE_PER_LED       (FULL_ANGLE / LED_NUMBER) //0.0785 //0.0393
#define AVERAGE_COUNT_HZ100 1

double angleRaw[AVERAGE_COUNT];
double tmp[AVERAGE_COUNT];
uint8_t rawCount    = 0;
uint8_t realCount   = 0;
uint8_t hz100Count  = 0;

uint8_t left = 0;
uint8_t dataFlag = 0;

//double prevAngles[AVERAGE_COUNT_HZ100];
double angle = 0;
double averageAngle = 0;
double angleSigma = 0;

//double prevAngle = 0;
//double angleVelocity = 0;
//double prevAngleVelocity = 0;
//double angleAcceleration = 0;
//uint8_t firstAngleMeasurement = 1;
//uint8_t firstVelocityMeasurement = 1;

//--------------------------------------------------------------------------------

int ADXL345_LedRound(double x) {
    int intX = (int)x;
    if (x > 0) {
        return (x - intX >= 0.5) ? intX + 1 : intX;
    } else {
        return (intX - x >= 0.5) ? intX - 1 : intX;
    }
}

void ADXL345_Sort(double *a) {
    int16_t i, j;
    double b;
    for (i = 0; i < WINDOW_SIZE; i++) {
        for (j = i + 1; j < WINDOW_SIZE; j++) {
            if (a[i] > a[j]) {
                b = a[i];
                a[i] = a[j];
                a[j] = b;
            }
        }
    }
}

void ADXL345_MedianFilterAngle() {
    double window[WINDOW_SIZE];
    int i, j;
    for (i = 0; i < rawCount; i++) {
        if (i == 0) {
            window[0] = angleRaw[0];
            for (j = 1; j < WINDOW_SIZE; j++) {
                window[j] = angleRaw[j-1];
            }
        } else if (i == rawCount - 1) {
            window[WINDOW_SIZE-1] = angleRaw[rawCount-1];
            for (j = 0; j < WINDOW_SIZE - 1; j++) {
                window[j] = angleRaw[rawCount-2 + j];
            }
        } else {
            for (j = 0; j < WINDOW_SIZE; j++) {
                window[j] = angleRaw[i-1 + j];
            }
        }
        ADXL345_Sort(window);
        tmp[i] = window[WINDOW_SIZE / 2];
    }
    memcpy(angleRaw, tmp, rawCount * sizeof(int16_t));
}

void ADXL345_Average() {
    double angleSum = 0;
    int i = 0;
    for (i = 0; i < rawCount; i++) {
        angleSum += angleRaw[i];
    }
    angle = angleSum / rawCount;
}

void ADXL345_ProcessRawData() {
    ADXL345_MedianFilterAngle();
    ADXL345_Average();
}

uint32_t ADXL345_Abs(int32_t x) {
    return x < 0 ? -x : x;
}

uint8_t defineBrightness(double extraAngle) {
    uint8_t bright = 0xFF * extraAngle / ANGLE_PER_LED;
    if (bright < 60) return 0;
    return bright;
}

uint8_t led = 0;
//double extraAngle = 0;
//uint32_t bright1 = 0;
//uint32_t bright2 = 0;
double currentLedAngle = -PI/2;

void ADXL345_ProcessData() {
//    double angleDiff = 0;
//    uint8_t tailLength = 0;
//    double angleSum = 0;
//    uint16_t i = 0;
    
    ((uint8_t *)(&x))[0] = (uint8_t)(accel[0] & 0xFF);
    ((uint8_t *)(&x))[1] = (uint8_t)(accel[1] & 0xFF);
    ((uint8_t *)(&y))[0] = (uint8_t)(accel[2] & 0xFF);
    ((uint8_t *)(&y))[1] = (uint8_t)(accel[3] & 0xFF);
    ((uint8_t *)(&z))[0] = (uint8_t)(accel[4] & 0xFF);
    ((uint8_t *)(&z))[1] = (uint8_t)(accel[5] & 0xFF);
    
    x -= xOffset;
    y -= yOffset;
    z -= zOffset;
    
    angle = atan((double)y / (double)z);
    if (fabs(angle) < FULL_ANGLE / 2) {
        if (fabs(angle - currentLedAngle) > 0.75 * ANGLE_PER_LED) {
            led = ADXL345_LedRound(angle * (LED_NUMBER - 1) / FULL_ANGLE + ((double)(LED_NUMBER - 1)) / 2); 
            currentLedAngle = led * FULL_ANGLE / ((double)(LED_NUMBER - 1)) - FULL_ANGLE / 2;
            WS2812_Label(led, 3, 0xff00ff);
        }
    }
    
    //realCount++;
    
    //if (ADXL345_Abs(((int32_t)x)*x + ((int32_t)y)*y + ((int32_t)z)*z - STD_G*STD_G) < ACCEL_EPS) {
//        if (z == 0) {
//            angleRaw[rawCount] = y > 0 ? PI / 2 : -PI / 2;
//        } else {
//            angleRaw[rawCount] = atan((double)y / (double)z);
//        }
//        rawCount++;
        
//        if (realCount == AVERAGE_COUNT) {
//            ADXL345_ProcessRawData();
//            if (fabs(angle) < FULL_ANGLE / 2) {
//            if (cbufptr == 0x0fff) {
//                dataFlag = 1;
//                angleSum = 0;
//                for (i = 0; i < cbufptr; i++) {
//                    angleSum += ang[i];
//                }
//                averageAngle = angleSum / cbufptr;
//                angleSum = 0;
//                for (i = 0; i < cbufptr; i++) {
//                    angleSum += (ang[i] - averageAngle) * (ang[i] - averageAngle);
//                }
//                sigma = sqrt(angleSum / cbufptr);
//            }
//            ang[cbufptr++] = angle;

            //angle = atan((double)y / z);
            
//            if (firstAngleMeasurement) {
//                firstAngleMeasurement = 0;
//            } else {
//                angleDiff = angle - prevAngle;
//                if (angleDiff > 0 && angleDiff > ANGLE_THRESHOLD) {
//                    left = TAIL_LEFT ? 1 : 0;
//                } else if (angleDiff < 0 && angleDiff < -ANGLE_THRESHOLD) {
//                    left = TAIL_LEFT ? 0 : 1;
//                }
//                
//                angleVelocity = (angle - prevAngle) / MEASUREMENT_TIME;
//                if (firstVelocityMeasurement) {
//                    firstVelocityMeasurement = 0;
//                } else {
//                    angleAcceleration = (angleVelocity - prevAngleVelocity) / MEASUREMENT_TIME;
//                } 
//                prevAngleVelocity = angleVelocity;
//            }
//            prevAngle = angle;
//            tailLength = fabs(angleVelocity) / RPS_PER_LED;
            
//            extraAngle = angle / ANGLE_PER_LED - (int)(angle / ANGLE_PER_LED);
//            prevAngles[hz100Count] = angle;
//            hz100Count++;
//            
//            if (hz100Count == AVERAGE_COUNT_HZ100) {
//                angleSum = prevAngles[0];
//                for (i = 1; i < AVERAGE_COUNT_HZ100; i++) {
//                    angleSum += prevAngles[i];
//                    prevAngles[i-1] = prevAngles[i];
//                }
//                angle = angleSum / AVERAGE_COUNT_HZ100;
//                hz100Count--;
//            }
//            if (fabs(angle - currentLedAngle) > 0.75 * ANGLE_PER_LED) {
//                led = ADXL345_LedRound(angle * (LED_NUMBER - 1) / FULL_ANGLE + ((double)(LED_NUMBER - 1)) / 2); 
//                currentLedAngle = led * FULL_ANGLE / ((double)(LED_NUMBER - 1)) - FULL_ANGLE / 2;
//            }
//            
//            angle = atan((double)y / (double)z);
//            led = ADXL345_LedRound(angle * (LED_NUMBER - 1) / FULL_ANGLE + ((double)(LED_NUMBER - 1)) / 2); 
//            WS2812_Label(led, 3, 0xff00ff);
            //WS2812_LabelWithTail(led, 1, 0x00ff00, left, tailLength, 0xff00ff);
            //WS2812_StableLabel(led, bright1, bright2);
            
//            rawCount    = 0;
//            realCount   = 0;
//        }
//        }
     /*else {
        GPIOC->BSRR |= (1 << ACCEL_VCC) << 16;
        for (i = 0; i < 65534; i++) {}
        ADXL345_Init();
    }*/

}

void TestLeds_GPIO_Init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    
    GPIOE->MODER    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
    GPIOE->OTYPER   &= ~(1 << 8 | 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
	GPIOE->OSPEEDR  |= 3 << 8*2 | 3 << 9*2 | 3 << 10*2 | 3 << 11*2 | 3 << 12*2 | 3 << 13*2;
    GPIOE->PUPDR    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
}

void SPI1_GPIO_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER 	|= (2 << SPI1_SCK*2) | (2 << SPI1_MISO*2) | (2 << SPI1_MOSI*2) | (1 << SPI1_NSS*2);
	GPIOA->OSPEEDR 	|= (3 << SPI1_SCK*2) | (3 << SPI1_MISO*2) | (3 << SPI1_MOSI*2) | (3 << SPI1_NSS*2);
	GPIOA->AFR[0] 	|= (5 << SPI1_SCK*4) | (5 << SPI1_MISO*4) | (5 << SPI1_MOSI*4);                         // AF5
	GPIOA->OTYPER	&= ~((1 << SPI1_SCK) | (1 << SPI1_MISO) | (1 << SPI1_MOSI) | (1 << SPI1_NSS)); 
	
	GPIOA->BSRR |= (1 << SPI1_NSS);   // NSS - HIGH
}

void NSS_Low() {
    GPIOA->BSRR |= (1 << SPI1_NSS) << 16;
}

void NSS_High() {
    GPIOA->BSRR |= (1 << SPI1_NSS);
}

void EXTI_Init() {
    //RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    
    //GPIOA->MODER &= ~(3 << 1*2);
    GPIOA->OSPEEDR |= 3 << 1*2;
    //GPIOA->PUPDR &= ~(3 << 1*2);
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR1; 
    EXTI->IMR 	|= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI1_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI1_IRQn); 
}

void SPI1_Init() {
    SPI1_GPIO_Init();
    
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	SPI1->CR1 = 0;
	SPI1->CR2 = SPI_CR2_DS_3 |SPI_CR2_DS_2 |SPI_CR2_DS_1 | SPI_CR2_DS_0;        // 16 bits
	
	SPI1->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		
	//SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR2 |= SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI1->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI
}

uint16_t SPI1_Transfer(uint16_t byte) { 
	while ((SPI1->SR & SPI_SR_TXE)==0);
	SPI1->DR = byte;
	
	while ((SPI1->SR & SPI_SR_RXNE)==0);
	return (SPI1->DR);
}

uint8_t ADXL345_read(uint8_t address) {
    uint16_t tmp = 0;
    
    address |= READ_COMMAND;
    tmp = SPI1_Transfer(address << 8);
    
    return tmp & 0xFF;
}

void ADXL345_write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= WRITE_COMMAND;
    tmp = address << 8;
    tmp |= data;
    SPI1_Transfer(tmp);
}

void ADXL345_AccelVCCInit()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    GPIOC->MODER    |= 1 << ACCEL_VCC*2;
    GPIOC->OTYPER   &= ~(1 << ACCEL_VCC);
    GPIOC->OSPEEDR  |= 3 << ACCEL_VCC*2;
    GPIOC->PUPDR    |= 1 << ACCEL_VCC*2;
    
    GPIOC->BSRR |= 1 << ACCEL_VCC;
    Delay(200000);
}

void ADXL345_Init() {
    uint8_t test = 0;
    Timer_Init();
    
    ADXL345_AccelVCCInit();
    SPI1_Init();  
    TestLeds_GPIO_Init();
    //EXTI_Init();
    NSS_Low();

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

    ADXL345_write(BW_RATE_ADDRESS, ACCEL_FREQ);
    test = ADXL345_read(BW_RATE_ADDRESS);
    if (test != ACCEL_FREQ) {
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
    NSS_High();
}

void ADXL345_GetAccel(int16_t *x, int16_t *y, int16_t *z) {
    NSS_Low();
    ((uint8_t *)x)[0] = ADXL345_read(0x32);
    ((uint8_t *)x)[1] = ADXL345_read(0x33);
    ((uint8_t *)y)[0] = ADXL345_read(0x34);
    ((uint8_t *)y)[1] = ADXL345_read(0x35);
    ((uint8_t *)z)[0] = ADXL345_read(0x36);
    ((uint8_t *)z)[1] = ADXL345_read(0x37);
    NSS_High();
}

#define CALIBR_NUMBER 100
double angles[CALIBR_NUMBER];

void ADXL345_Calibr() {
    int i = 0;
    double xSum = 0, ySum = 0, zSum = 0, angleSum = 0;
    
    while (!(GPIOA->IDR & (1 << 1))) {}
    ADXL345_GetAccel(&x, &y, &z); 
    xSum += x;
    ySum += y;
    zSum += z;
        
    for (i = 0; i < CALIBR_NUMBER; i++) {
        while (!(GPIOA->IDR & (1 << 1))) {}
        ADXL345_GetAccel(&x, &y, &z); 
        xSum += x;
        ySum += y;
        zSum += z;
        angles[i] = atan((double)y / (double)z);
        angleSum += angles[i];     
    }         
    xOffset = (int16_t)(xSum / CALIBR_NUMBER);
    yOffset = (int16_t)(ySum / CALIBR_NUMBER);
    zOffset = (int16_t)(zSum / CALIBR_NUMBER) - 0xFF;
    averageAngle = angleSum / CALIBR_NUMBER;
    
    angleSum = 0;
    for (i = 0; i < CALIBR_NUMBER; i++) {
        angleSum += (angles[i] - averageAngle) * (angles[i] - averageAngle);
    }
    angleSigma = sqrt(angleSum / CALIBR_NUMBER);
}

void ADXL345_DMA_Init() {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    
    DMA1_Channel2->CCR      = 0;
    DMA1_Channel2->CPAR     = (uint32_t)&(SPI1->DR);
    DMA1_Channel2->CMAR     = (uint32_t)accel;
    DMA1_Channel2->CNDTR    = 6;
    DMA1_Channel2->CCR      = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                               DMA_CCR_TCIE | DMA_CCR_PL | DMA_CCR_EN;
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    
    DMA1_Channel3->CCR      = 0;
    DMA1_Channel3->CPAR     = (uint32_t)&(SPI1->DR);
    DMA1_Channel3->CMAR     = (uint32_t)accelRegisters;
    DMA1_Channel3->CNDTR    = 6;
    DMA1_Channel3->CCR      = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                                DMA_CCR_PL | DMA_CCR_EN;
}

void EXTI1_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        NSS_Low();
        ADXL345_DMA_Init();
    }
}

void DMA1_Channel2_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        if((GPIOA->IDR & (1 << 1)) == 1) {  
            ADXL345_DMA_Init();
        } else NSS_High();
        
        ADXL345_ProcessData();
    }
}
