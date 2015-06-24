#include "stm32f30x.h"
#include "math.h"
#include "stdio.h"
#include "string.h"


#define READ_COMMAND                0x80
#define WRITE_COMMAND               0x00

#define DEVID_ADDRESS               0x00
#define DEVID                       0xe5

#define INT_ENABLE_ADDRESS          0x2e
#define DATA_READY_INT              0x80

#define INT_MAPPING_ADDRESS         0x2f
#define DATA_READY_INT0_MAPPING     0x7f

#define POWER_CTL_ADDRESS           0x2d
#define MEASUREMENT_MODE            0x08

#define DATA_FORMAT_ADDRESS         0x31
#define FULL_RES_MODE               0x08

#define BW_RATE_ADDRESS             0x2c
#define HZ100                       0x0a
#define HZ800                       0x0d
#define HZ1600                      0x0e
#define ACCEL_FREQ                  HZ800

#define CALIBR_NUMBER               800

#define MEASUREMENT_TIME            0.01


uint8_t ACCEL_VCC   = 14;

uint8_t SPI1_SCK    = 5;
uint8_t SPI1_MISO   = 6;
uint8_t SPI1_MOSI   = 7;
uint8_t SPI1_NSS    = 4;

uint8_t TEST_LEDS[] = {8, 9, 10, 11, 12, 13};

// Motors control and frequency measurement pins
uint8_t MOT_PWM1    = 12;   // PD12
uint8_t MOT_PWM2    = 14;   // PD14
uint8_t MOT_FREQ1   = 2;    // PE2
uint8_t MOT_FREQ2   = 4;    // PE4

uint8_t UART2_TX    = 2;    // PA2
uint8_t UART2_RX    = 3;    // PA3

uint8_t SERVO_PWM   = 3;   // PB14
uint8_t VENTI       = 10;   // PB10

int16_t x, y, z;
int16_t xOffset, yOffset, zOffset;

int16_t accelRegisters[6] = {0xB200, 0xB300, 0xB400, 0xB500, 0xB600, 0xB700};
int16_t accel[6];

uint8_t firstAngleMeasurement = 0;
double angle            = 0;
double prevAngle        = 0;
double angularVelocity  = 0;
uint8_t angleCount      = 0;
double angleSum         = 0;

uint8_t ENGRDY      = 0;
uint8_t DELAY_FLAG  = 0;
int timer           = 200;

uint8_t received = 0;
int pwm1 = 0;
int pwm2 = 0;
int COUNT1 = 0;
int COUNT2 = 0;
uint8_t update = 0;

double F = 0;
double k1 = 2.15;
double k2 = -1.5e-4;

double Aappr = -4.596e-7;
double Bappr = 0.002354;
double Cappr = -2.154;
double D = 0;
double x1 = 0;
double x2 = 0;

double chooseRoot() {
    if (D < 0) return 2000;
    
    x1 = (-Bappr - sqrt(D)) / (2 * Aappr);
    x2 = (-Bappr + sqrt(D)) / (2 * Aappr);
    
    if (x1 > 1000 && x1 < 2000) return x1;
    if (x2 > 1000 && x2 < 2000) return x2;
    return 2000;
}

void ADXL345_ProcessData() {
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
    
    angleSum += angle;
    angleCount++;
    if (angleCount == 8) {
        angle = angleSum / angleCount;
        angleSum = 0;
        angleCount = 0;
        
        if (firstAngleMeasurement) {
            firstAngleMeasurement = 0;
        } else {
            angularVelocity = (angle - prevAngle) / MEASUREMENT_TIME;
        }
        prevAngle = angle;
        
        F = k1*angle + k2*angularVelocity;
    
        if (update) {
            if (F > 0) {
                TIM4->CCR1 = 1000;
                pwm1 = 1000;
                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
                pwm2 = (int)chooseRoot();
                TIM4->CCR3 = pwm2;
            } else if (F < 0) {
                TIM4->CCR3 = 1000;
                pwm2 = 1000;
                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
                pwm1 = (int)chooseRoot();
                TIM4->CCR1 = pwm1;
            }
        }
    }
}

void Venti_On() {
    GPIOB->BSRR |= (1 << VENTI);
}

void Venti_Off() {
    GPIOB->BSRR |= (1 << VENTI) << 16;
}

void Servo_Left10() {
    int curPwm = TIM2->CCR2;
    TIM2->CCR2 = curPwm - 100;
}

void Servo_Right10() {
    int curPwm = TIM2->CCR2;
    TIM2->CCR2 = curPwm + 100;
}

void RCC_Init() {
    RCC->CR |= RCC_CR_HSEON; //HSE
    while (!(RCC->CR & RCC_CR_HSERDY)) {};
    RCC->CFGR = RCC_CFGR_SW_HSE;
    while(!(RCC->CFGR & RCC_CFGR_SWS_HSE)) {};
    
    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
                        RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_DMA1EN;
    RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR    |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_SPI1EN |
                        RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN;
}

void Timer_Init() {
    TIM1->PSC = 7;
    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0x0);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

void Delay(uint32_t us) {
    TIM1->ARR = us;
    
    TIM1->CR1 |= TIM_CR1_CEN;
    
    DELAY_FLAG = 1;
    while (DELAY_FLAG) {}
}

void TIM1_UP_TIM16_IRQHandler() {
    //if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR &= ~TIM_SR_UIF;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        DELAY_FLAG = 0;
    //}
}

void TestLeds_GPIO_Init() {    
    GPIOE->MODER    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
    GPIOE->OTYPER   &= ~(1 << 8 | 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
	GPIOE->OSPEEDR  |= 3 << 8*2 | 3 << 9*2 | 3 << 10*2 | 3 << 11*2 | 3 << 12*2 | 3 << 13*2;
    GPIOE->PUPDR    |= 1 << 8*2 | 1 << 9*2 | 1 << 10*2 | 1 << 11*2 | 1 << 12*2 | 1 << 13*2;
}

void SPI1_GPIO_Init() {
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

void Accel_EXTI_Init() {    
    GPIOA->OSPEEDR |= 3 << 1*2;
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR1; 
    EXTI->IMR 	|= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI1_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI1_IRQn); 
}

void SPI1_Init() {
    SPI1_GPIO_Init();
	
	SPI1->CR1 = 0;
	SPI1->CR2 = SPI_CR2_DS_3 |SPI_CR2_DS_2 |SPI_CR2_DS_1 | SPI_CR2_DS_0;        // 16 bits
	
	SPI1->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		

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
    int delay = 0;
    
    GPIOC->MODER    |= 1 << ACCEL_VCC*2;
    GPIOC->OTYPER   &= ~(1 << ACCEL_VCC);
    GPIOC->OSPEEDR  |= 3 << ACCEL_VCC*2;
    GPIOC->PUPDR    |= 1 << ACCEL_VCC*2;
    
    GPIOC->BSRR |= 1 << ACCEL_VCC;
    Delay(5000000);
    //for (delay = 0; delay <= 2000000; delay++) {}
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

void ADXL345_Calibr() {
    int i = 0;
    double xSum = 0, ySum = 0, zSum = 0;
    
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
    }         
    xOffset = (int16_t)(xSum / CALIBR_NUMBER);
    yOffset = (int16_t)(ySum / CALIBR_NUMBER);
    zOffset = (int16_t)(zSum / CALIBR_NUMBER) - 0xFF;
}

void ADXL345_DMA_Init() {
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
        Delay(5000);
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

void Motors_GPIO_Init() {
    // PD12, PD14 - motor pwm
    GPIOD->MODER    |= (2 << MOT_PWM1*2) | (2 << MOT_PWM2*2);               // alternative function
    GPIOD->OTYPER   &= ~((1 << MOT_PWM1) | (1 << MOT_PWM2));                // push-pull
    GPIOD->OSPEEDR  |= (3 << MOT_PWM1*2) | (3 << MOT_PWM2*2);               // high speed
    GPIOD->PUPDR    |= (1 << MOT_PWM1*2) | (1 << MOT_PWM2*2);               // pull-up
    GPIOD->AFR[1]   |= (2 << (MOT_PWM1 - 8)*4) | (2 << (MOT_PWM2 - 8)*4);   // AF2
}

void Motors_Timer_Init() {
    TIM4->PSC 		= 7;	
	TIM4->ARR 		= 10000;			
	TIM4->CCR1 	    = 2000;		
	TIM4->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
	TIM4->CCER 	    |= TIM_CCER_CC1E; 
    
    TIM4->CCR3 	    = 2000;		
	TIM4->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; 
	TIM4->CCER 	    |= TIM_CCER_CC3E; 
	
	TIM4->CR1 = TIM_CR1_CEN;		
}

void Motors_FreqCount_Init() {
    // motor frequency calculation
    GPIOE->MODER 	&= ~((3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2));          // input
    GPIOE->OTYPER	&= ~((1 << MOT_FREQ1) | (1 << MOT_FREQ2));              // push-pull
	GPIOE->OSPEEDR 	|= (3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2);             // high speed 
    GPIOE->PUPDR    |= (1 << MOT_FREQ1*2) | (1 << MOT_FREQ2*2);             // pull-up
    
    // frequency calculation interrupt for motor1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;   
    EXTI->FTSR 	|= EXTI_FTSR_TR2; 
    EXTI->IMR 	|= EXTI_IMR_MR2;
    NVIC_SetPriority(EXTI2_TS_IRQn, 0xFF);
    NVIC_EnableIRQ(EXTI2_TS_IRQn); 
    
    // frequency calculation interrupt for motor2
    SYSCFG->EXTICR[1] |= SYSCFG_EXTIRCR_EXTI4_PE; 
    EXTI->FTSR 	|= EXTI_FTSR_TR4; 
    EXTI->IMR 	|= EXTI_IMR_MR4;
    NVIC_SetPriority(EXTI4_IRQn, 0xFF);
    NVIC_EnableIRQ(EXTI4_IRQn); 
    
    TIM15->DIER |= 1;					// timer for frequency calculation
    TIM15->CR1 = TIM_CR1_CEN;
    
    TIM16->DIER |= 1;					// timer for frequency calculation
    TIM16->CR1 = TIM_CR1_CEN;
}

void EXTI2_IRQHandler() { //this used to calculate the frequency of motor
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR |= EXTI_PR_PR2;
        if (TIM15->CR1 && 1) {
            COUNT1 = TIM15->CNT;
            TIM15->CR1 &= ~1;
            TIM15->CNT = 0;
        }
        else {
            TIM15->CNT = 0;
            TIM15->CR1 |= 1;
        }
    }
}

void EXTI4_IRQHandler() { //this used to calculate the frequency of motor
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR |= EXTI_PR_PR4;
        if (TIM16->CR1 && 1) {
            COUNT2 = TIM16->CNT;
            TIM16->CR1 &= ~1;
            TIM16->CNT = 0;
        }
        else {
            TIM16->CNT = 0;
            TIM16->CR1 |= 1;
        }
    }
}

void Motors_Init() {
    Motors_GPIO_Init();
    Motors_Timer_Init();
    Motors_FreqCount_Init();
    
    TIM8->PSC = 7;
    TIM8->ARR = 5000;
    TIM8->DIER |= 1;
    NVIC_SetPriority(TIM8_UP_IRQn, 0x0);    
	NVIC_EnableIRQ(TIM8_UP_IRQn);
    
    TIM8->CR1 = TIM_CR1_CEN;
}

void TIM8_UP_IRQHandler() {
    if (TIM8->SR & TIM_SR_UIF) {
        TIM8->SR &= ~TIM_SR_UIF;
        if (timer > 0) {
            timer--;
        }if (TIM4->CCR1 == 1000 || TIM4->CCR3 == 1000) {
            TIM3->CR1 &= ~TIM_CR1_CEN;
            TIM4->CCR1 = 1000;
            TIM4->CCR3 = 1000;
            ENGRDY = 1;
        }
        else {
            TIM4->CCR1--;
            TIM4->CCR3--;
        } 
    }
}

void USART_Send(uint8_t data) {
    while(!(USART2->ISR & USART_ISR_TC)); 
    USART2->TDR = data; 
}
 
void USART_Init(void) {
    GPIOA->MODER    |= (2 << UART2_TX*2) | (2 << UART2_RX*2);
    GPIOA->OTYPER   &= ~((1 << UART2_TX) | (1 << UART2_RX));
    GPIOA->OSPEEDR  |= (3 << UART2_TX*2) | (3 << UART2_RX*2);
    GPIOA->PUPDR    |= (1 << UART2_TX*2) | (1 << UART2_RX*2);
    GPIOA->AFR[0]   |= (7 << UART2_TX*4) | (7 << UART2_RX*4);

    USART2->BRR = 0x341; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler() {
    if (USART2->ISR & USART_ISR_RXNE) {
        received = USART2->RDR;
        
        switch (received) {
            case 'n':
                Venti_On();
                break;
            case 'f':
                Venti_Off();
                break;
            case 'b': // begin stabilization
                update = 1;
                break;
            case 's': // stop motors and stabilization
                TIM4->CCR1 = 1000;
                TIM4->CCR3 = 1000;
                pwm1 = 1000;
                pwm2 = 1000;
                update = 0;
                break;
            case 'c': // recalibrate and begin stabilization
                ADXL345_Calibr();
                update = 1;
                break;
            case 'l': // servo to the left
                Servo_Left10();
                break;
            case 'r': // servo to the right
                Servo_Right10();
                break;
            default:
                break;
        } 
        
        // if (received == 'p') {
        // PWM_RX = 1;
        // i = -1;
        // st = 1;
        // pwm = 0;
        // }
        // else if (received == 'e') {
        // PWM_RX = 0;
        // while( i >= 0) {
        // pwm += (str[i--]-'0') * st;
        // st*=10;
        // }
        // pwm1 = pwm2 = pwm;
        // TIM4->CCR1 = pwm1;
        // TIM4->CCR3 = pwm2;
        // NEW_PWM_RV = 1;
        // }
        // else if (PWM_RX == 1) str[++i] = received;
    }
}

void Telemetry_Send() {
    char tele[100];
    uint8_t len = 0, i;
    
    if (pwm1 == 0 && pwm2 == 0) {
        return;
    }
    //sprintf(tele, "%8.2f\t%8.4f\t%8.4f\t%8d\t%8d\t%8d\t%8d\n", F, angle, angularVelocity, pwm1, COUNT1, pwm2, COUNT2);
    sprintf(tele, "%d\t%d\t%d\t%d\n", pwm1, COUNT1, pwm2, COUNT2);
    len = strlen(tele);
    
    for (i = 0; i < len; i++) {
        USART_Send(tele[i]);
    }
}

void Telemetry_Init() {
    USART_Init();
    
    TIM7->PSC = 7;
    TIM7->ARR = 5000;
    TIM7->DIER |= 1;
    NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    Telemetry_Send();
}

void Venti_Init() {
    GPIOB->MODER    |= 1 << VENTI*2;
    GPIOB->OTYPER   &= ~(1 << VENTI);
    GPIOB->OSPEEDR  |= 3 << VENTI*2;
    GPIOB->PUPDR    |= 1 << VENTI*2;
}

void Servo_Init() {
    GPIOB->MODER    |= 2 << SERVO_PWM*2;
    GPIOB->OTYPER   &= ~(1 << SERVO_PWM);
    GPIOB->OSPEEDR  |= 3 << SERVO_PWM*2;
    GPIOB->PUPDR    |= 1 << SERVO_PWM*2;
    GPIOB->AFR[0]   |= 1 << SERVO_PWM*4;
    
    TIM2->PSC       = 7;
    TIM2->ARR       = 20000;
    TIM2->CCR2      = 10000;
    TIM2->CCMR1     |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM2->CCER      |= TIM_CCER_CC2E;
    
    TIM2->CR1       |= TIM_CR1_CEN;
}

int main() {
    RCC_Init();
    Venti_Init();
//    Servo_Init();
    Motors_Init();
    
    while(ENGRDY != 1) {};
    ADXL345_Init();
	ADXL345_Calibr();
    Accel_EXTI_Init();
        
    Telemetry_Init();
    update = 1;
    while (1) {
    }
}
